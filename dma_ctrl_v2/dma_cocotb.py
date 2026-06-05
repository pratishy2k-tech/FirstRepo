# tb_dma_top.py
# CocoTB testbench for dma_top
# Tests: single-channel transfer, multi-channel concurrent,
#        error injection, descriptor chaining, interrupt verification

import cocotb
from cocotb.clock       import Clock
from cocotb.triggers    import RisingEdge, FallingEdge, Timer, with_timeout
from cocotb.result      import TestFailure
from cocotb.utils       import get_sim_time
import random
import logging

log = logging.getLogger("dma_tb")
log.setLevel(logging.DEBUG)

# ================================================================
# Constants — mirror dma_pkg.vh
# ================================================================
CLK_PERIOD_NS  = 10

NUM_CHANNELS   = 8
DESC_DEPTH     = 16
ADDR_WIDTH     = 64
DATA_WIDTH     = 512
DATA_BYTES     = DATA_WIDTH // 8

# CSR word addresses (paddr[9:2])
CSR_DMA_CTRL   = 0x00
CSR_CH_EN      = 0x01
CSR_IRQ_STATUS = 0x02
CSR_IRQ_MASK   = 0x03

def ch_base(n):
    return 0x10 + n * 8

CH_CTRL_OFF    = 0x00
CH_STATUS_OFF  = 0x01
CH_DESC_BASE   = 0x02
CH_DESC_HEAD   = 0x03
CH_DESC_TAIL   = 0x04
CH_XFER_CNT    = 0x05
CH_ERR_CODE    = 0x06

# Descriptor ctrl field bits
DESC_CTRL_VALID   = (1 << 0)
DESC_CTRL_IRQ_EN  = (1 << 1)
DESC_CTRL_CHAIN   = (1 << 2)
DESC_CTRL_BURST_4 = (2 << 4)   # AxSIZE=4 => 16 bytes/beat

# IRQ bits
IRQ_DONE_BITS  = 0x00FF
IRQ_ERR_BITS   = 0xFF00


# ================================================================
# AXI4 Memory Model (behavioral slave)
# ================================================================
class AXI4MemoryModel:
    """
    Behavioral AXI4 slave — backed by a Python dict for sparse addressing.
    Responds to AR/R and AW/W/B channels on the given DUT port prefix.
    """

    def __init__(self, dut, prefix: str, data_width: int = 512):
        self.dut        = dut
        self.prefix     = prefix
        self.data_width = data_width
        self.data_bytes = data_width // 8
        self.mem        = {}          # addr (aligned) -> bytes
        self._log       = logging.getLogger(f"axi_mem_{prefix}")

        # Pre-fill with random data for reads
        self._fill_region(0x0000_0000_0000_0000, 4096)

    def _fill_region(self, base_addr: int, size: int):
        for off in range(0, size, self.data_bytes):
            aligned = base_addr + off
            self.mem[aligned] = bytearray(
                [random.randint(0, 255) for _ in range(self.data_bytes)]
            )

    def _read_mem(self, addr: int) -> int:
        aligned = (addr // self.data_bytes) * self.data_bytes
        if aligned not in self.mem:
            self.mem[aligned] = bytearray(self.data_bytes)
        data_bytes = self.mem[aligned]
        return int.from_bytes(data_bytes, 'little')

    def _write_mem(self, addr: int, data: int, strb: int):
        aligned = (addr // self.data_bytes) * self.data_bytes
        if aligned not in self.mem:
            self.mem[aligned] = bytearray(self.data_bytes)
        data_bytes = self.mem[aligned]
        for byte_idx in range(self.data_bytes):
            if (strb >> byte_idx) & 1:
                data_bytes[byte_idx] = (data >> (byte_idx * 8)) & 0xFF

    def _sig(self, name):
        return getattr(self.dut, f"{self.prefix}_{name}")

    async def run_read_slave(self):
        """Respond to AXI AR/R channels."""
        dut = self.dut
        arvalid = self._sig("arvalid")
        arready = self._sig("arready")
        araddr  = self._sig("araddr")
        arlen   = self._sig("arlen")
        arsize  = self._sig("arsize")
        arid    = self._sig("arid")
        rvalid  = self._sig("rvalid")
        rready  = self._sig("rready")
        rdata   = self._sig("rdata")
        rresp   = self._sig("rresp")
        rlast   = self._sig("rlast")
        rid     = self._sig("rid")

        arready.value = 1
        rvalid.value  = 0

        while True:
            await RisingEdge(dut.clk)
            if arvalid.value == 1 and arready.value == 1:
                addr     = int(araddr.value)
                burst_n  = int(arlen.value) + 1
                burst_sz = 1 << int(arsize.value)
                xid      = int(arid.value)
                arready.value = 0

                for beat in range(burst_n):
                    beat_data = self._read_mem(addr + beat * burst_sz)
                    await RisingEdge(dut.clk)
                    rvalid.value = 1
                    rdata.value  = beat_data
                    rresp.value  = 0   # OKAY
                    rlast.value  = 1 if (beat == burst_n - 1) else 0
                    rid.value    = xid

                    # Wait for rready
                    while True:
                        await RisingEdge(dut.clk)
                        if rready.value == 1:
                            break

                rvalid.value  = 0
                rlast.value   = 0
                arready.value = 1

    async def run_write_slave(self):
        """Respond to AXI AW/W/B channels."""
        dut = self.dut
        awvalid = self._sig("awvalid")
        awready = self._sig("awready")
        awaddr  = self._sig("awaddr")
        awlen   = self._sig("awlen")
        awsize  = self._sig("awsize")
        awid    = self._sig("awid")
        wvalid  = self._sig("wvalid")
        wready  = self._sig("wready")
        wdata   = self._sig("wdata")
        wstrb   = self._sig("wstrb")
        wlast   = self._sig("wlast")
        bvalid  = self._sig("bvalid")
        bready  = self._sig("bready")
        bresp   = self._sig("bresp")
        bid     = self._sig("bid")

        awready.value = 1
        wready.value  = 1
        bvalid.value  = 0

        while True:
            await RisingEdge(dut.clk)
            if awvalid.value == 1 and awready.value == 1:
                addr     = int(awaddr.value)
                burst_n  = int(awlen.value) + 1
                burst_sz = 1 << int(awsize.value)
                xid      = int(awid.value)
                awready.value = 0
                wready.value  = 1

                for beat in range(burst_n):
                    # Wait for W beat
                    while True:
                        await RisingEdge(dut.clk)
                        if wvalid.value == 1 and wready.value == 1:
                            beat_data = int(wdata.value)
                            beat_strb = int(wstrb.value)
                            self._write_mem(addr + beat * burst_sz,
                                            beat_data, beat_strb)
                            break

                wready.value = 0
                # Send write response
                await RisingEdge(dut.clk)
                bvalid.value = 1
                bresp.value  = 0   # OKAY
                bid.value    = xid

                while True:
                    await RisingEdge(dut.clk)
                    if bready.value == 1:
                        break

                bvalid.value  = 0
                awready.value = 1
                wready.value  = 1


# ================================================================
# APB3 Driver
# ================================================================
class APBDriver:
    def __init__(self, dut):
        self.dut = dut

    async def write(self, waddr: int, data: int):
        """Perform one APB write transaction."""
        dut = self.dut
        await RisingEdge(dut.clk)
        dut.psel.value    = 1
        dut.pwrite.value  = 1
        dut.paddr.value   = waddr << 2   # word offset to byte offset
        dut.pwdata.value  = data & 0xFFFF_FFFF
        dut.penable.value = 0
        await RisingEdge(dut.clk)
        dut.penable.value = 1
        await RisingEdge(dut.clk)
        # Wait for pready
        while not dut.pready.value:
            await RisingEdge(dut.clk)
        dut.psel.value    = 0
        dut.penable.value = 0
        dut.pwrite.value  = 0

    async def read(self, waddr: int) -> int:
        """Perform one APB read transaction."""
        dut = self.dut
        await RisingEdge(dut.clk)
        dut.psel.value    = 1
        dut.pwrite.value  = 0
        dut.paddr.value   = waddr << 2
        dut.penable.value = 0
        await RisingEdge(dut.clk)
        dut.penable.value = 1
        await RisingEdge(dut.clk)
        while not dut.pready.value:
            await RisingEdge(dut.clk)
        val = int(dut.prdata.value)
        dut.psel.value    = 0
        dut.penable.value = 0
        return val


# ================================================================
# Descriptor Builder
# ================================================================
def build_descriptor(src_addr: int, dst_addr: int,
                     byte_cnt: int, ctrl: int,
                     next_desc: int = 0) -> int:
    """Pack a 256-bit DMA descriptor as a Python integer."""
    desc  = (src_addr  & 0xFFFF_FFFF_FFFF_FFFF)
    desc |= (dst_addr  & 0xFFFF_FFFF_FFFF_FFFF) << 64
    desc |= (byte_cnt  & 0xFFFF_FFFF)            << 128
    desc |= (next_desc & 0xFFFF_FFFF)            << 160
    desc |= (ctrl      & 0xFFFF_FFFF)            << 192
    desc |= 0                                    << 224   # status=0
    return desc


async def load_descriptor(apb: APBDriver, ch: int,
                          desc_idx: int, desc: int):
    """
    Load a 256-bit descriptor into channel ch, slot desc_idx.
    The CSR provides a 256-bit descriptor write path via 8 x 32-bit writes.
    This function uses the sw_desc_wr mechanism exposed via the CSR.
    """
    # Write 8 x 32-bit words to a dedicated descriptor load window
    # Offset layout: 0x80 = desc load base
    # [ch*8 + word] -> 0x80 + ch*8 + word
    base = 0x80 + ch * 8
    for w in range(8):
        word_val = (desc >> (w * 32)) & 0xFFFF_FFFF
        await apb.write(base + w, word_val)
    # Commit: write desc_idx to trigger sw_desc_wr
    await apb.write(ch_base(ch) + CH_DESC_BASE, desc_idx)


# ================================================================
# Reset Utility
# ================================================================
async def do_reset(dut, cycles: int = 20):
    dut.rst_n.value   = 0
    dut.psel.value    = 0
    dut.penable.value = 0
    dut.pwrite.value  = 0
    dut.paddr.value   = 0
    dut.pwdata.value  = 0
    await Timer(cycles * CLK_PERIOD_NS, units='ns')
    dut.rst_n.value   = 1
    await RisingEdge(dut.clk)


# ================================================================
# Interrupt Monitor
# ================================================================
class IRQMonitor:
    def __init__(self, dut, apb: APBDriver):
        self.dut   = dut
        self.apb   = apb
        self.done  = [False] * NUM_CHANNELS
        self.err   = [False] * NUM_CHANNELS

    async def poll(self):
        """Poll IRQ status register and update local state."""
        irq = await self.apb.read(CSR_IRQ_STATUS)
        for ch in range(NUM_CHANNELS):
            if (irq >> ch) & 1:
                self.done[ch] = True
            if (irq >> (ch + 8)) & 1:
                self.err[ch] = True
        return irq

    async def wait_done(self, ch: int, timeout_ns: int = 100_000):
        """Wait until channel ch raises done interrupt."""
        deadline = get_sim_time('ns') + timeout_ns
        while True:
            irq = await self.apb.read(CSR_IRQ_STATUS)
            if (irq >> ch) & 1:
                # Clear the
