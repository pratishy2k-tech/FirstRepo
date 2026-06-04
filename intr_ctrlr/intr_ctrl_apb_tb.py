# =============================================================
# test_interrupt_controller_apb.py
# Cocotb testbench for interrupt_controller_apb
#
# Tests covered:
#   T1  - Reset state verification
#   T2  - Line interrupt mode (msg_mode=0), IRQ0 fires INT pin
#   T3  - Switch to messaged mode (msg_mode=1), verify APB write
#   T4  - APB PWDATA == ISR value check
#   T5  - APB target address from MADDR registers
#   T6  - PSLVERR sets apb_err sticky bit in MICR
#   T7  - apb_err cleared by W1C on MICR[1]
#   T8  - APB busy signal during transaction
#   T8  - Multiple interrupts sequenced through APB
#   T9  - Software interrupt in messaged mode
#   T10 - Switching mode mid-operation
# =============================================================

import cocotb
from cocotb.clock      import Clock
from cocotb.triggers   import RisingEdge, FallingEdge, Timer, First
from cocotb.result     import TestFailure, TestSuccess
import logging

# ─────────────────────────────────────────────────────────────
# Register Address Map (must match RTL)
# ─────────────────────────────────────────────────────────────
ADDR_IMR      = 0x0
ADDR_IPR      = 0x1
ADDR_ISR      = 0x2
ADDR_TRIG     = 0x3
ADDR_SWIR     = 0x4
ADDR_GCR      = 0x5
ADDR_VECT     = 0x6
ADDR_STAT     = 0x7
ADDR_MICR     = 0x8
ADDR_MADDR_LO = 0x9
ADDR_MADDR_HI = 0xA

# APB FSM states (for logging)
APB_IDLE   = 0b00
APB_SETUP  = 0b01
APB_ACCESS = 0b10

# ─────────────────────────────────────────────────────────────
# APB Slave Model
# Runs as a background coroutine, responds to APB transactions
# ─────────────────────────────────────────────────────────────
class APBSlaveModel:
    """
    Lightweight APB slave model that responds to write transactions.
    Configurable: ready_latency (cycles before asserting PREADY)
                  inject_error  (assert PSLVERR on next transaction)
    """
    def __init__(self, dut, ready_latency=1, inject_error=False):
        self.dut           = dut
        self.ready_latency = ready_latency
        self.inject_error  = inject_error
        self.transactions  = []       # Log of all received transactions
        self.log           = logging.getLogger("APBSlave")
        self._running      = False

    async def start(self):
        """Start the APB slave response loop."""
        self._running = True
        dut = self.dut

        # Default outputs
        dut.PREADY.value  = 0
        dut.PSLVERR.value = 0

        while self._running:
            # Wait for PSEL to go high (SETUP phase)
            await RisingEdge(dut.clk)

            if dut.PSEL.value == 1 and dut.PENABLE.value == 0:
                # SETUP phase detected — wait for ACCESS phase
                await RisingEdge(dut.clk)

                # Now in ACCESS phase — add configured latency
                for _ in range(self.ready_latency - 1):
                    await RisingEdge(dut.clk)

                # Assert PREADY (and optionally PSLVERR)
                dut.PREADY.value  = 1
                dut.PSLVERR.value = 1 if self.inject_error else 0

                # Log the transaction
                txn = {
                    "addr"  : int(dut.PADDR.value),
                    "data"  : int(dut.PWDATA.value),
                    "write" : int(dut.PWRITE.value),
                    "error" : self.inject_error
                }
                self.transactions.append(txn)
                self.log.info(
                    f"APB TXN | ADDR=0x{txn['addr']:04X} "
                    f"DATA=0x{txn['data']:02X} "
                    f"PSLVERR={txn['error']}"
                )

                await RisingEdge(dut.clk)
                dut.PREADY.value  = 0
                dut.PSLVERR.value = 0

                # Reset inject_error after one use
                self.inject_error = False

    def stop(self):
        self._running = False

    def last_transaction(self):
        return self.transactions[-1] if self.transactions else None

    def transaction_count(self):
        return len(self.transactions)


# ─────────────────────────────────────────────────────────────
# Helper: CPU register read / write tasks
# ─────────────────────────────────────────────────────────────
async def cpu_write(dut, addr, data):
    """Perform one CPU register write."""
    await RisingEdge(dut.clk)
    dut.cpu_wr.value    = 1
    dut.cpu_rd.value    = 0
    dut.cpu_addr.value  = addr
    dut.cpu_wdata.value = data
    await RisingEdge(dut.clk)
    dut.cpu_wr.value    = 0
    dut.cpu_wdata.value = 0


async def cpu_read(dut, addr):
    """Perform one CPU register read, return the read value."""
    await RisingEdge(dut.clk)
    dut.cpu_rd.value   = 1
    dut.cpu_wr.value   = 0
    dut.cpu_addr.value = addr
    await Timer(1, units="ns")          # Let combinational settle
    val = int(dut.cpu_rdata.value)
    await RisingEdge(dut.clk)
    dut.cpu_rd.value   = 0
    return val


async def cpu_ack(dut):
    """Assert cpu_ack for one cycle."""
    await RisingEdge(dut.clk)
    dut.cpu_ack.value = 1
    await RisingEdge(dut.clk)
    dut.cpu_ack.value = 0


async def reset_dut(dut, cycles=4):
    """Apply active-low reset for N cycles."""
    dut.rst_n.value     = 0
    dut.irq.value       = 0
    dut.cpu_ack.value   = 0
    dut.cpu_wr.value    = 0
    dut.cpu_rd.value    = 0
    dut.cpu_addr.value  = 0
    dut.cpu_wdata.value = 0
    dut.PREADY.value    = 0
    dut.PSLVERR.value   = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)


async def wait_apb_idle(dut, timeout=50):
    """Wait until APB is no longer busy (apb_busy == 0)."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.apb_busy.value) == 0:
            return
    raise TestFailure("Timeout waiting for APB to become idle")


async def fire_irq_edge(dut, irq_mask, cycles=2):
    """Assert IRQ lines for N cycles then deassert (edge mode)."""
    await RisingEdge(dut.clk)
    dut.irq.value = irq_mask
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    dut.irq.value = 0


# ─────────────────────────────────────────────────────────────
# TEST 1: Reset state verification
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_01_reset_state(dut):
    """T1: All registers at correct reset values."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    dut._log.info("T1: Checking reset state")

    # IMR should be 0xFF (all masked)
    val = await cpu_read(dut, ADDR_IMR)
    assert val == 0xFF, f"T1 FAIL: IMR expected 0xFF, got 0x{val:02X}"

    # GCR should be 0x00 (GIE disabled)
    val = await cpu_read(dut, ADDR_GCR)
    assert val == 0x00, f"T1 FAIL: GCR expected 0x00, got 0x{val:02X}"

    # IPR should be 0x00
    val = await cpu_read(dut, ADDR_IPR)
    assert val == 0x00, f"T1 FAIL: IPR expected 0x00, got 0x{val:02X}"

    # MICR should be 0x00 (line mode, no error)
    val = await cpu_read(dut, ADDR_MICR)
    assert val == 0x00, f"T1 FAIL: MICR expected 0x00, got 0x{val:02X}"

    # APB outputs should be deasserted
    assert int(dut.PSEL.value)    == 0, "T1 FAIL: PSEL should be 0 at reset"
    assert int(dut.PENABLE.value) == 0, "T1 FAIL: PENABLE should be 0 at reset"
    assert int(dut.INT.value)     == 0, "T1 FAIL: INT should be 0 at reset"

    dut._log.info("T1 PASS: Reset state verified")


# ─────────────────────────────────────────────────────────────
# TEST 2: Line interrupt mode (msg_mode=0) — IRQ0 fires INT pin
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_02_line_interrupt_mode(dut):
    """T2: In line mode, INT pin is asserted; APB stays idle."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    dut._log.info("T2: Configuring line interrupt mode")

    # MICR[0] = 0 (line mode — this is already reset default)
    await cpu_write(dut, ADDR_MICR,     0x00)
    await cpu_write(dut, ADDR_TRIG,     0xFF)  # All edge
    await cpu_write(dut, ADDR_IMR,      0xFE)  # Unmask IRQ0
    await cpu_write(dut, ADDR_GCR,      0x01)  # GIE = 1

    # Fire IRQ0
    await fire_irq_edge(dut, 0x01)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    # INT pin must be asserted
    assert int(dut.INT.value) == 1, \
        f"T2 FAIL: INT expected 1 in line mode, got {int(dut.INT.value)}"

    # APB must NOT have triggered any transaction
    assert slave.transaction_count() == 0, \
        f"T2 FAIL: APB should have 0 txns in line mode, got {slave.transaction_count()}"

    # INT_VECT must be 0 (IRQ0)
    assert int(dut.INT_VECT.value) == 0, \
        f"T2 FAIL: INT_VECT expected 0, got {int(dut.INT_VECT.value)}"

    await cpu_ack(dut)
    await RisingEdge(dut.clk)

    assert int(dut.INT.value) == 0, "T2 FAIL: INT should clear after cpu_ack"

    slave.stop()
    dut._log.info("T2 PASS: Line interrupt mode works correctly")


# ─────────────────────────────────────────────────────────────
# TEST 3: Messaged interrupt mode — APB write fires on interrupt
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_03_messaged_interrupt_apb_fires(dut):
    """T3: In messaged mode, APB write is initiated instead of INT pin."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    # Configure for messaged mode
    await cpu_write(dut, ADDR_TRIG,     0xFF)   # All edge triggered
    await cpu_write(dut, ADDR_IMR,      0xFE)   # Unmask IRQ0
    await cpu_write(dut, ADDR_GCR,      0x01)   # GIE = 1
    await cpu_write(dut, ADDR_MADDR_LO, 0x80)   # Target addr = 0x1080
    await cpu_write(dut, ADDR_MADDR_HI, 0x10)
    await cpu_write(dut, ADDR_MICR,     0x01)   # msg_mode = 1

    dut._log.info("T3: Firing IRQ0 in messaged mode")

    await fire_irq_edge(dut, 0x01)

    # Wait for APB transaction to complete
    await wait_apb_idle(dut, timeout=30)

    # INT pin should NOT be asserted in messaged mode
    assert int(dut.INT.value) == 0, \
        f"T3 FAIL: INT pin should be 0 in messaged mode, got {int(dut.INT.value)}"

    # Exactly one APB transaction should have occurred
    assert slave.transaction_count() == 1, \
        f"T3 FAIL: Expected 1 APB txn, got {slave.transaction_count()}"

    dut._log.info(f"T3 PASS: APB transaction fired correctly in messaged mode")


# ─────────────────────────────────────────────────────────────
# TEST 4: PWDATA equals ISR value
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_04_pwdata_equals_isr(dut):
    """T4: APB PWDATA (message payload) must equal ISR register value."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFB)   # Unmask IRQ0, IRQ1 (mask=1111_1011 -> unmask bit2)
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MADDR_LO, 0x00)
    await cpu_write(dut, ADDR_MADDR_HI, 0x00)
    await cpu_write(dut, ADDR_MICR,     0x01)   # Messaged mode

    # Fire IRQ2 only
    await fire_irq_edge(dut, 0x04)              # IRQ2 = bit 2
    await wait_apb_idle(dut, timeout=30)

    # Read ISR from DUT
    isr_val = await cpu_read(dut, ADDR_ISR)

    # APB data should match ISR
    txn = slave.last_transaction()
    assert txn is not None, "T4 FAIL: No APB transaction recorded"
    assert txn["data"] == isr_val, \
        f"T4 FAIL: PWDATA=0x{txn['data']:02X} != ISR=0x{isr_val:02X}"

    # ISR for IRQ2 should be 0x04
    assert isr_val == 0x04, \
        f"T4 FAIL: ISR expected 0x04 for IRQ2, got 0x{isr_val:02X}"

    slave.stop()
    dut._log.info(f"T4 PASS: PWDATA=0x{txn['data']:02X} == ISR=0x{isr_val:02X}")


# ─────────────────────────────────────────────────────────────
# TEST 5: APB PADDR matches MADDR registers
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_05_apb_target_address(dut):
    """T5: PADDR on APB bus must match MADDR_HI:MADDR_LO configuration."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    target_lo = 0xBC
    target_hi = 0xDE
    expected_addr = (target_hi << 8) | target_lo   # 0xDEBC

    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFE)   # Unmask IRQ0
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MADDR_LO, target_lo)
    await cpu_write(dut, ADDR_MADDR_HI, target_hi)
    await cpu_write(dut, ADDR_MICR,     0x01)

    await fire_irq_edge(dut, 0x01)
    await wait_apb_idle(dut, timeout=30)

    txn = slave.last_transaction()
    assert txn is not None, "T5 FAIL: No APB transaction recorded"
    assert txn["addr"] == expected_addr, \
        f"T5 FAIL: PADDR expected 0x{expected_addr:04X}, got 0x{txn['addr']:04X}"

    slave.stop()
    dut._log.info(f"T5 PASS: PADDR=0x{txn['addr']:04X} matches configured address")


# ─────────────────────────────────────────────────────────────
# TEST 6: PSLVERR sets apb_err sticky bit in MICR[1]
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_06_pslverr_sticky_error(dut):
    """T6: When APB slave returns PSLVERR, MICR[1] apb_err must be set."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Slave will inject an error on next transaction
    slave = APBSlaveModel(dut, ready_latency=1, inject_error=True)
    cocotb.start_soon(slave.start())

    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFE)
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MADDR_LO, 0x00)
    await cpu_write(dut, ADDR_MADDR_HI, 0x00)
    await cpu_write(dut, ADDR_MICR,     0x01)

    await fire_irq_edge(dut, 0x01)
    await wait_apb_idle(dut, timeout=30)
    await RisingEdge(dut.clk)   # Let apb_err_r latch

    micr_val = await cpu_read(dut, ADDR_MICR)
    assert (micr_val >> 1) & 1, \
        f"T6 FAIL: MICR[1] (apb_err) should be 1 after PSLVERR, MICR=0x{micr_val:02X}"

    slave.stop()
    dut._log.info(f"T6 PASS: apb_err sticky bit set correctly, MICR=0x{micr_val:02X}")


# ─────────────────────────────────────────────────────────────
# TEST 7: apb_err W1C clear via MICR[1]
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_07_apb_err_w1c_clear(dut):
    """T7: apb_err sticky bit clears when CPU writes 1 to MICR[1]."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1, inject_error=True)
    cocotb.start_soon(slave.start())

    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFE)
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MICR,     0x01)

    await fire_irq_edge(dut, 0x01)
    await wait_apb_idle(dut, timeout=30)
    await RisingEdge(dut.clk)

    # Verify error is set
    micr_before = await cpu_read(dut, ADDR_MICR)
    assert (micr_before >> 1) & 1, \
        f"T7 FAIL: apb_err not set before clear attempt, MICR=0x{micr_before:02X}"

    # W1C: write 0b10 to clear bit[1]
    await cpu_write(dut, ADDR_MICR, 0x02)
    await RisingEdge(dut.clk)

    micr_after = await cpu_read(dut, ADDR_MICR)
    assert not ((micr_after >> 1) & 1), \
        f"T7 FAIL: apb_err should be 0 after W1C, MICR=0x{micr_after:02X}"

    slave.stop()
    dut._log.info(f"T7 PASS: apb_err W1C cleared, MICR=0x{micr_after:02X}")


# ─────────────────────────────────────────────────────────────
# TEST 8: apb_busy signal during APB transaction
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_08_apb_busy_signal(dut):
    """T8: apb_busy is 1 during APB transaction, 0 when idle."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    # Use a slow slave (3-cycle latency) to give us time to sample busy
    slave = APBSlaveModel(dut, ready_latency=3)
    cocotb.start_soon(slave.start())

    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFE)
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MICR,     0x01)

    await fire_irq_edge(dut, 0x01)

    # Wait one cycle for FSM to enter SETUP
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    # apb_busy should be high
    busy_during = int(dut.apb_busy.value)

    # Now wait for it to finish
    await wait_apb_idle(dut, timeout=30)
    busy_after = int(dut.apb_busy.value)

    assert busy_during == 1, \
        f"T8 FAIL: apb_busy should be 1 during transaction, got {busy_during}"
    assert busy_after == 0, \
        f"T8 FAIL: apb_busy should be 0 after transaction, got {busy_after}"

    slave.stop()
    dut._log.info("T8 PASS: apb_busy behaves correctly")


# ─────────────────────────────────────────────────────────────
# TEST 9: Software interrupt in messaged mode
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_09_sw_interrupt_messaged(dut):
    """T9: Software-injected interrupt in messaged mode triggers APB write."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xF7)   # Unmask IRQ3
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MADDR_LO, 0x40)
    await cpu_write(dut, ADDR_MADDR_HI, 0x00)
    await cpu_write(dut, ADDR_MICR,     0x01)   # Messaged mode

    # Inject SW interrupt on IRQ3
    await cpu_write(dut, ADDR_SWIR, 0x08)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await cpu_write(dut, ADDR_SWIR, 0x00)       # Clear SW inject

    await wait_apb_idle(dut, timeout=30)

    txn = slave.last_transaction()
    assert txn is not None, "T9 FAIL: No APB transaction for SW interrupt"
    assert txn["data"] == 0x08, \
        f"T9 FAIL: Expected PWDATA=0x08 for IRQ3, got 0x{txn['data']:02X}"

    slave.stop()
    dut._log.info(f"T9 PASS: SW interrupt in messaged mode, PWDATA=0x{txn['data']:02X}")


# ─────────────────────────────────────────────────────────────
# TEST 10: Mode switch mid-operation
# ─────────────────────────────────────────────────────────────
@cocotb.test()
async def test_10_mode_switch(dut):
    """T10: Switch from line to messaged mid-test. Verify correct behavior."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    await reset_dut(dut)

    slave = APBSlaveModel(dut, ready_latency=1)
    cocotb.start_soon(slave.start())

    await cpu_write(dut, ADDR_TRIG,     0xFF)
    await cpu_write(dut, ADDR_IMR,      0xFE)   # Unmask IRQ0
    await cpu_write(dut, ADDR_GCR,      0x01)
    await cpu_write(dut, ADDR_MADDR_LO, 0x10)
    await cpu_write(dut, ADDR_MADDR_HI, 0x00)

    # ---- Phase A: Line mode ----
    await cpu_write(dut, ADDR_MICR, 0x00)       # Line mode

    await fire_irq_edge(dut, 0x01)
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    assert int(dut.INT.value) == 1,  "T10 FAIL: INT should be high in line mode"
    assert slave.transaction_count() == 0, "T10 FAIL: No APB txn expected in line mode"

    await cpu_ack(dut)
    await RisingEdge(dut.clk)

    # ---- Phase B: Switch to messaged mode ----
    await cpu_write(dut, ADDR_MICR, 0x01)       # Messaged mode

    await fire_irq_edge(dut, 0x01)
    await wait_apb_idle(dut, timeout=30)

    assert int(dut.INT.value) == 0, "T10 FAIL: INT should be 0 in messaged mode"
    assert slave.transaction_count() == 1, \
        f"T10 FAIL: Expected 1 APB txn after mode switch, got {slave.transaction_count()}"

    slave.stop()
    dut._log.info("T10 PASS: Mode switch behaves correctly")
