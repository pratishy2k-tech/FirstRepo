// dma_top.v
// Top-level DMA controller with 8 channels
// APB CSR + 8x AXI4 read/write masters

`include "dma_pkg.vh"

module dma_top #(
    parameter NUM_CH      = `DMA_NUM_CHANNELS,
    parameter ADDR_WIDTH  = `DMA_ADDR_WIDTH,
    parameter DATA_WIDTH  = `DMA_DATA_WIDTH,
    parameter ID_WIDTH    = `DMA_ID_WIDTH
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ======================================================
    // APB3 CSR Interface
    // ======================================================
    input  wire                    psel,
    input  wire                    penable,
    input  wire                    pwrite,
    input  wire [9:0]              paddr,
    input  wire [31:0]             pwdata,
    output wire [31:0]             prdata,
    output wire                    pready,
    output wire                    pslverr,

    // ======================================================
    // Interrupt Output
    // ======================================================
    output wire [`DMA_IRQ_WIDTH-1:0] irq_out,

    // ======================================================
    // AXI4 Master — Channel 0
    // ======================================================
    // Read
    output wire [ID_WIDTH-1:0]     m0_axi_arid,
    output wire [ADDR_WIDTH-1:0]   m0_axi_araddr,
    output wire [7:0]              m0_axi_arlen,
    output wire [2:0]              m0_axi_arsize,
    output wire [1:0]              m0_axi_arburst,
    output wire                    m0_axi_arlock,
    output wire [3:0]              m0_axi_arcache,
    output wire [2:0]              m0_axi_arprot,
    output wire [3:0]              m0_axi_arqos,
    output wire                    m0_axi_arvalid,
    input  wire                    m0_axi_arready,
    input  wire [ID_WIDTH-1:0]     m0_axi_rid,
    input  wire [DATA_WIDTH-1:0]   m0_axi_rdata,
    input  wire [1:0]              m0_axi_rresp,
    input  wire                    m0_axi_rlast,
    input  wire                    m0_axi_rvalid,
    output wire                    m0_axi_rready,
    // Write
    output wire [ID_WIDTH-1:0]     m0_axi_awid,
    output wire [ADDR_WIDTH-1:0]   m0_axi_awaddr,
    output wire [7:0]              m0_axi_awlen,
    output wire [2:0]              m0_axi_awsize,
    output wire [1:0]              m0_axi_awburst,
    output wire                    m0_axi_awlock,
    output wire [3:0]              m0_axi_awcache,
    output wire [2:0]              m0_axi_awprot,
    output wire [3:0]              m0_axi_awqos,
    output wire                    m0_axi_awvalid,
    input  wire                    m0_axi_awready,
    output wire [DATA_WIDTH-1:0]   m0_axi_wdata,
    output wire [DATA_WIDTH/8-1:0] m0_axi_wstrb,
    output wire                    m0_axi_wlast,
    output wire                    m0_axi_wvalid,
    input  wire                    m0_axi_wready,
    input  wire [ID_WIDTH-1:0]     m0_axi_bid,
    input  wire [1:0]              m0_axi_bresp,
    input  wire                    m0_axi_bvalid,
    output wire                    m0_axi_bready

    // NOTE: Channels 1..7 have identical port declarations.
    // For brevity they are wired internally via generate blocks.
    // In a real implementation, replicate the port list for each channel.
);

    // -------------------------------------------------------
    // Internal wiring arrays
    // -------------------------------------------------------
    // CSR -> channels
    wire [NUM_CH-1:0]       ch_enable_w;
    wire [NUM_CH-1:0]       ch_pause_w;
    wire [NUM_CH-1:0]       sw_tail_wr_w;
    wire [4*NUM_CH-1:0]     sw_tail_w;
    wire [NUM_CH-1:0]       sw_head_wr_w;
    wire [4*NUM_CH-1:0]     sw_head_w;
    wire [NUM_CH-1:0]       sw_desc_wr_w;
    wire [4*NUM_CH-1:0]     sw_desc_addr_w;
    wire [256*NUM_CH-1:0]   sw_desc_data_w;

    // Channels -> CSR
    wire [NUM_CH-1:0]       ch_active_w;
    wire [NUM_CH-1:0]       ch_done_w;
    wire [NUM_CH-1:0]       ch_err_w;
    wire [5*NUM_CH-1:0]     err_code_w;
    wire [4*NUM_CH-1:0]     head_ptr_w;
    wire [4*NUM_CH-1:0]     tail_ptr_w;
    wire [32*NUM_CH-1:0]    xfer_count_w;

    // AXI buses (flattened arrays for generate)
    wire [ID_WIDTH-1:0]     axi_arid    [NUM_CH-1:0];
    wire [ADDR_WIDTH-1:0]   axi_araddr  [NUM_CH-1:0];
    wire [7:0]              axi_arlen   [NUM_CH-1:0];
    wire [2:0]              axi_arsize  [NUM_CH-1:0];
    wire [1:0]              axi_arburst [NUM_CH-1:0];
    wire                    axi_arlock  [NUM_CH-1:0];
    wire [3:0]              axi_arcache [NUM_CH-1:0];
    wire [2:0]              axi_arprot  [NUM_CH-1:0];
    wire [3:0]              axi_arqos   [NUM_CH-1:0];
    wire                    axi_arvalid [NUM_CH-1:0];
    wire                    axi_arready [NUM_CH-1:0];
    wire [ID_WIDTH-1:0]     axi_rid     [NUM_CH-1:0];
    wire [DATA_WIDTH-1:0]   axi_rdata   [NUM_CH-1:0];
    wire [1:0]              axi_rresp   [NUM_CH-1:0];
    wire                    axi_rlast   [NUM_CH-1:0];
    wire                    axi_rvalid  [NUM_CH-1:0];
    wire                    axi_rready  [NUM_CH-1:0];

    wire [ID_WIDTH-1:0]     axi_awid    [NUM_CH-1:0];
    wire [ADDR_WIDTH-1:0]   axi_awaddr  [NUM_CH-1:0];
    wire [7:0]              axi_awlen   [NUM_CH-1:0];
    wire [2:0]              axi_awsize  [NUM_CH-1:0];
    wire [1:0]              axi_awburst [NUM_CH-1:0];
    wire                    axi_awlock  [NUM_CH-1:0];
    wire [3:0]              axi_awcache [NUM_CH-1:0];
    wire [2:0]              axi_awprot  [NUM_CH-1:0];
    wire [3:0]              axi_awqos   [NUM_CH-1:0];
    wire                    axi_awvalid [NUM_CH-1:0];
    wire                    axi_awready [NUM_CH-1:0];
    wire [DATA_WIDTH-1:0]   axi_wdata   [NUM_CH-1:0];
    wire [DATA_WIDTH/8-1:0] axi_wstrb   [NUM_CH-1:0];
    wire                    axi_wlast   [NUM_CH-1:0];
    wire                    axi_wvalid  [NUM_CH-1:0];
    wire                    axi_wready  [NUM_CH-1:0];
    wire [ID_WIDTH-1:0]     axi_bid     [NUM_CH-1:0];
    wire [1:0]              axi_bresp   [NUM_CH-1:0];
    wire                    axi_bvalid  [NUM_CH-1:0];
    wire                    axi_bready  [NUM_CH-1:0];

    // -------------------------------------------------------
    // Tie channel 0 AXI buses to top-level ports
    // -------------------------------------------------------
    assign m0_axi_arid     = axi_arid   [0];
    assign m0_axi_araddr   = axi_araddr [0];
    assign m0_axi_arlen    = axi_arlen  [0];
    assign m0_axi_arsize   = axi_arsize [0];
    assign m0_axi_arburst  = axi_arburst[0];
    assign m0_axi_arlock   = axi_arlock [0];
    assign m0_axi_arcache  = axi_arcache[0];
    assign m0_axi_arprot   = axi_arprot [0];
    assign m0_axi_arqos    = axi_arqos  [0];
    assign m0_axi_arvalid  = axi_arvalid[0];
    assign axi_arready[0]  = m0_axi_arready;
    assign axi_rid    [0]  = m0_axi_rid;
    assign axi_rdata  [0]  = m0_axi_rdata;
    assign axi_rresp  [0]  = m0_axi_rresp;
    assign axi_rlast  [0]  = m0_axi_rlast;
    assign axi_rvalid [0]  = m0_axi_rvalid;
    assign m0_axi_rready   = axi_rready [0];

    assign m0_axi_awid     = axi_awid   [0];
    assign m0_axi_awaddr   = axi_awaddr [0];
    assign m0_axi_awlen    = axi_awlen  [0];
    assign m0_axi_awsize   = axi_awsize [0];
    assign m0_axi_awburst  = axi_awburst[0];
    assign m0_axi_awlock   = axi_awlock [0];
    assign m0_axi_awcache  = axi_awcache[0];
    assign m0_axi_awprot   = axi_awprot [0];
    assign m0_axi_awqos    = axi_awqos  [0];
    assign m0_axi_awvalid  = axi_awvalid[0];
    assign axi_awready[0]  = m0_axi_awready;
    assign m0_axi_wdata    = axi_wdata  [0];
    assign m0_axi_wstrb    = axi_wstrb  [0];
    assign m0_axi_wlast    = axi_wlast  [0];
    assign m0_axi_wvalid   = axi_wvalid [0];
    assign axi_wready [0]  = m0_axi_wready;
    assign axi_bid    [0]  = m0_axi_bid;
    assign axi_bresp  [0]  = m0_axi_bresp;
    assign axi_bvalid [0]  = m0_axi_bvalid;
    assign m0_axi_bready   = axi_bready [0];

    // Channels 1..7: Tie to zero (stub for this example)
    // In a real design, add ports for each channel
    genvar gi;
    generate
        for (gi = 1; gi < NUM_CH; gi = gi + 1) begin : ch_axi_stub
            assign axi_arready[gi] = 1'b1;
            assign axi_rid    [gi] = {ID_WIDTH{1'b0}};
            assign axi_rdata  [gi] = {DATA_WIDTH{1'b0}};
            assign axi_rresp  [gi] = 2'b00;
            assign axi_rlast  [gi] = 1'b0;
            assign axi_rvalid [gi] = 1'b0;
            assign axi_awready[gi] = 1'b1;
            assign axi_wready [gi] = 1'b1;
            assign axi_bid    [gi] = {ID_WIDTH{1'b0}};
            assign axi_bresp  [gi] = 2'b00;
            assign axi_bvalid [gi] = 1'b0;
        end
    endgenerate

    // -------------------------------------------------------
    // CSR instantiation
    // -------------------------------------------------------
    dma_csr #(
        .NUM_CH   (NUM_CH),
        .NUM_REGS (256)
    ) u_csr (
        .pclk          (clk),
        .presetn       (rst_n),
        .psel          (psel),
        .penable       (penable),
        .pwrite        (pwrite),
        .paddr         (paddr),
        .pwdata        (pwdata),
        .prdata        (prdata),
        .pready        (pready),
        .pslverr       (pslverr),
        .ch_enable     (ch_enable_w),
        .ch_pause      (ch_pause_w),
        .sw_tail_wr    (sw_tail_wr_w),
        .sw_tail       (sw_tail_w),
        .sw_head_wr    (sw_head_wr_w),
        .sw_head       (sw_head_w),
        .sw_desc_wr    (sw_desc_wr_w),
        .sw_desc_addr  (sw_desc_addr_w),
        .sw_desc_data  (sw_desc_data_w),
        .ch_active     (ch_active_w),
        .ch_done       (ch_done_w),
        .ch_err        (ch_err_w),
        .err_code      (err_code_w),
        .head_ptr      (head_ptr_w),
        .tail_ptr      (tail_ptr_w),
        .xfer_count    (xfer_count_w),
        .irq_out       (irq_out),
        .dma_enable    (),
        .dma_reset     ()
    );

    // -------------------------------------------------------
    // Generate 8 DMA channels
    // -------------------------------------------------------
    generate
        for (gi = 0; gi < NUM_CH; gi = gi + 1) begin : dma_ch_gen
            dma_channel #(
                .CH_ID      (gi),
                .ADDR_WIDTH (ADDR_WIDTH),
                .DATA_WIDTH (DATA_WIDTH),
                .ID_WIDTH   (ID_WIDTH)
            ) u_ch (
                .clk          (clk),
                .rst_n        (rst_n),
                .ch_enable    (ch_enable_w[gi]),
                .ch_pause     (ch_pause_w[gi]),
                .sw_tail_wr   (sw_tail_wr_w[gi]),
                .sw_tail      (sw_tail_w[gi*4+3:gi*4]),
                .sw_head_wr   (sw_head_wr_w[gi]),
                .sw_head      (sw_head_w[gi*4+3:gi*4]),
                .sw_desc_wr   (sw_desc_wr_w[gi]),
                .sw_desc_addr (sw_desc_addr_w[gi*4+3:gi*4]),
                .sw_desc_data (sw_desc_data_w[gi*256+255:gi*256]),
                .head_ptr     (head_ptr_w[gi*4+3:gi*4]),
                .tail_ptr     (tail_ptr_w[gi*4+3:gi*4]),
                .ch_active    (ch_active_w[gi]),
                .ch_done      (ch_done_w[gi]),
                .ch_err       (ch_err_w[gi]),
                .err_code     (err_code_w[gi*5+4:gi*5]),
                .xfer_count   (xfer_count_w[gi*32+31:gi*32]),
                .m_axi_arid   (axi_arid   [gi]),
                .m_axi_araddr (axi_araddr [gi]),
                .m_axi_arlen  (axi_arlen  [gi]),
                .m_axi_arsize (axi_arsize [gi]),
                .m_axi_arburst(axi_arburst[gi]),
                .m_axi_arlock (axi_arlock [gi]),
                .m_axi_arcache(axi_arcache[gi]),
                .m_axi_arprot (axi_arprot [gi]),
                .m_axi_arqos  (axi_arqos  [gi]),
                .m_axi_arvalid(axi_arvalid[gi]),
                .m_axi_arready(axi_arready[gi]),
                .m_axi_rid    (axi_rid    [gi]),
                .m_axi_rdata  (axi_rdata  [gi]),
                .m_axi_rresp  (axi_rresp  [gi]),
                .m_axi_rlast  (axi_rlast  [gi]),
                .m_axi_rvalid (axi_rvalid [gi]),
                .m_axi_rready (axi_rready [gi]),
                .m_axi_awid   (axi_awid   [gi]),
                .m_axi_awaddr (axi_awaddr [gi]),
                .m_axi_awlen  (axi_awlen  [gi]),
                .m_axi_awsize (axi_awsize [gi]),
                .m_axi_awburst(axi_awburst[gi]),
                .m_axi_awlock (axi_awlock [gi]),
                .m_axi_awcache(axi_awcache[gi]),
                .m_axi_awprot (axi_awprot [gi]),
                .m_axi_awqos  (axi_awqos  [gi]),
                .m_axi_awvalid(axi_awvalid[gi]),
                .m_axi_awready(axi_awready[gi]),
                .m_axi_wdata  (axi_wdata  [gi]),
                .m_axi_wstrb  (axi_wstrb  [gi]),
                .m_axi_wlast  (axi_wlast  [gi]),
                .m_axi_wvalid (axi_wvalid [gi]),
                .m_axi_wready (axi_wready [gi]),
                .m_axi_bid    (axi_bid    [gi]),
                .m_axi_bresp  (axi_bresp  [gi]),
                .m_axi_bvalid (axi_bvalid [gi]),
                .m_axi_bready (axi_bready [gi])
            );
        end
    endgenerate

endmodule
