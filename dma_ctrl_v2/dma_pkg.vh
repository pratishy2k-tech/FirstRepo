// dma_pkg.vh
// DMA Controller Package — Types, Parameters, Descriptor Layout

`ifndef DMA_PKG_VH
`define DMA_PKG_VH

// -------------------------------------------------------
// Global Parameters
// -------------------------------------------------------
`define DMA_NUM_CHANNELS     8
`define DMA_ADDR_WIDTH       64
`define DMA_DATA_WIDTH       512
`define DMA_STRB_WIDTH       (`DMA_DATA_WIDTH / 8)
`define DMA_LEN_WIDTH        32
`define DMA_ID_WIDTH         8
`define DMA_BURST_LEN        256    // Max AXI burst beats
`define DMA_DESC_DEPTH       16     // Descriptors per channel ring
`define DMA_DESC_ADDR_BITS   4      // log2(DMA_DESC_DEPTH)
`define DMA_IRQ_WIDTH        16     // Interrupt lines

// -------------------------------------------------------
// AXI4 Burst Types
// -------------------------------------------------------
`define AXI_BURST_FIXED      2'b00
`define AXI_BURST_INCR       2'b01
`define AXI_BURST_WRAP       2'b10

// -------------------------------------------------------
// AXI4 Response Codes
// -------------------------------------------------------
`define AXI_RESP_OKAY        2'b00
`define AXI_RESP_EXOKAY      2'b01
`define AXI_RESP_SLVERR      2'b10
`define AXI_RESP_DECERR      2'b11

// -------------------------------------------------------
// Descriptor Structure (256-bit / 32 bytes):
//  [63:0]   src_addr    - Source address
//  [127:64] dst_addr    - Destination address
//  [159:128] byte_cnt  - Bytes to transfer
//  [191:160] next_desc - Next descriptor index (ring)
//  [223:192] ctrl      - Control flags (see below)
//  [255:224] status    - Status (written back by HW)
//
// ctrl[0]   : valid     - Descriptor is valid
// ctrl[1]   : irq_en    - Raise interrupt on completion
// ctrl[2]   : chain     - Chain to next_desc when done
// ctrl[3]   : dir       - 0=mem-to-mem, 1=dev-to-mem
// ctrl[7:4] : burst_sz  - AXI burst size (AxSIZE)
// ctrl[15:8]: axi_cache - AXI cache attribute
// ctrl[23:16]: axi_prot - AXI protection attribute
//
// status[0] : done      - Transfer completed
// status[1] : src_err   - AXI read error
// status[2] : dst_err   - AXI write error
// status[3] : desc_err  - Descriptor fetch error
// status[4] : len_err   - Zero-length / misaligned error
// -------------------------------------------------------

// Descriptor field offsets (bit positions within 256-bit word)
`define DESC_SRC_ADDR_LO     0
`define DESC_SRC_ADDR_HI     63
`define DESC_DST_ADDR_LO     64
`define DESC_DST_ADDR_HI     127
`define DESC_BYTECNT_LO      128
`define DESC_BYTECNT_HI      159
`define DESC_NEXT_LO         160
`define DESC_NEXT_HI         191
`define DESC_CTRL_LO         192
`define DESC_CTRL_HI         223
`define DESC_STATUS_LO       224
`define DESC_STATUS_HI       255

// ctrl bit aliases
`define DESC_CTRL_VALID      0
`define DESC_CTRL_IRQ_EN     1
`define DESC_CTRL_CHAIN      2
`define DESC_CTRL_DIR        3
`define DESC_CTRL_BURST_SZ   7:4
`define DESC_CTRL_CACHE      15:8
`define DESC_CTRL_PROT       23:16

// status bit aliases
`define DESC_STA_DONE        0
`define DESC_STA_SRC_ERR     1
`define DESC_STA_DST_ERR     2
`define DESC_STA_DESC_ERR    3
`define DESC_STA_LEN_ERR     4

// -------------------------------------------------------
// CSR Register Map (APB word-offset)
// -------------------------------------------------------
`define CSR_DMA_CTRL         8'h00   // Global enable/reset
`define CSR_CH_EN            8'h01   // Channel enable bitmask
`define CSR_IRQ_STATUS       8'h02   // Interrupt status (W1C)
`define CSR_IRQ_MASK         8'h03   // Interrupt mask
`define CSR_CH_BASE(n)       (8'h10 + (n)*8'h08)  // Ch n base
// Per-channel offsets from CH_BASE(n):
`define CH_CTRL_OFF          8'h00   // Start/stop/pause
`define CH_STATUS_OFF        8'h01   // Running/done/error
`define CH_DESC_BASE_OFF     8'h02   // Descriptor ring base
`define CH_DESC_HEAD_OFF     8'h03   // Head pointer
`define CH_DESC_TAIL_OFF     8'h04   // Tail pointer (SW writes)
`define CH_XFER_CNT_OFF      8'h05   // Bytes transferred
`define CH_ERR_CODE_OFF      8'h06   // Last error code
`define CH_CFG_OFF           8'h07   // Channel config

// IRQ bit assignments (per DMA_IRQ_WIDTH=16)
// [7:0]  = transfer done, one per channel
// [15:8] = error, one per channel
`define IRQ_DONE_BIT(n)      (n)
`define IRQ_ERR_BIT(n)       ((n) + 8)

`endif // DMA_PKG_VH
