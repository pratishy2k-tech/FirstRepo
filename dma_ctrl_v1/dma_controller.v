// =============================================================================
//  Generic 8-Channel Descriptor-Based AXI DMA Controller
//  Top-Level Module
// =============================================================================
//  Register Map (AXI4-Lite, base + ch*0x100)
//    0x000 : GLOBAL_CTRL      [0]=enable [1]=sw_reset
//    0x004 : GLOBAL_IRQ_STATUS [7:0]=ch_done [15:8]=ch_err
//    0x008 : GLOBAL_IRQ_MASK
//
//  Per-Channel (offset = 0x100 * ch_id)
//    0x100 : CH_CTRL          [0]=enable [1]=pause [2]=abort
//    0x104 : CH_STATUS        [0]=idle [1]=busy [2]=done [3]=err [6:4]=err_code
//    0x108 : CH_DESC_ADDR_LO  descriptor ring base (lo 32b)
//    0x10C : CH_DESC_ADDR_HI  descriptor ring base (hi 32b)
//    0x110 : CH_CURR_DESC_LO  current descriptor pointer (lo 32b)
//    0x114 : CH_CURR_DESC_HI  current descriptor pointer (hi 32b)
//    0x118 : CH_BYTES_DONE    bytes transferred for current descriptor
//    0x11C : CH_IRQ_CNT       interrupt count (saturates at 0xFF)
//
//  Descriptor (fetched from memory, 48 bytes):
//    [63:0]   src_addr
//    [127:64] dst_addr
//    [159:128] length       (bytes)
//    [191:160] ctrl         [0]=intr_on_cmp [1]=is_last [3:2]=burst_type
//                           [5:4]=src_prot  [7:6]=dst_prot
//    [255:192] next_desc    (address of next descriptor, ignored if is_last)
// =============================================================================

`timescale 1ns/1ps

module dma_controller #(
    parameter N_CHANNELS     = 8,
    parameter ADDR_WIDTH     = 64,
    parameter DATA_WIDTH     = 64,
    parameter ID_WIDTH       = 4,
    parameter AXI_LITE_DW    = 32,
    parameter AXI_LITE_AW    = 16,
    parameter DESC_FIFO_DEPTH= 4,
    parameter MAX_BURST_LEN  = 16   // AXI beats per burst
)(
    input  wire                         aclk,
    input  wire                         aresetn,

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave (CSR interface)
    // -------------------------------------------------------------------------
    input  wire [AXI_LITE_AW-1:0]       s_axil_awaddr,
    input  wire                         s_axil_awvalid,
    output reg                          s_axil_awready,
    input  wire [AXI_LITE_DW-1:0]       s_axil_wdata,
    input  wire [AXI_LITE_DW/8-1:0]    s_axil_wstrb,
    input  wire                         s_axil_wvalid,
    output reg                          s_axil_wready,
    output reg  [1:0]                   s_axil_bresp,
    output reg                          s_axil_bvalid,
    input  wire                         s_axil_bready,
    input  wire [AXI_LITE_AW-1:0]       s_axil_araddr,
    input  wire                         s_axil_arvalid,
    output reg                          s_axil_arready,
    output reg  [AXI_LITE_DW-1:0]       s_axil_rdata,
    output reg  [1:0]                   s_axil_rresp,
    output reg                          s_axil_rvalid,
    input  wire                         s_axil_rready,

    // -------------------------------------------------------------------------
    // AXI4 Master (shared bus, N_CHANNELS ports muxed internally)
    // -------------------------------------------------------------------------
    // Write Address Channel
    output wire [ID_WIDTH-1:0]          m_axi_awid,
    output wire [ADDR_WIDTH-1:0]        m_axi_awaddr,
    output wire [7:0]                   m_axi_awlen,
    output wire [2:0]                   m_axi_awsize,
    output wire [1:0]                   m_axi_awburst,
    output wire [1:0]                   m_axi_awprot,
    output wire                         m_axi_awvalid,
    input  wire                         m_axi_awready,
    // Write Data Channel
    output wire [DATA_WIDTH-1:0]        m_axi_wdata,
    output wire [DATA_WIDTH/8-1:0]      m_axi_wstrb,
    output wire                         m_axi_wlast,
    output wire                         m_axi_wvalid,
    input  wire                         m_axi_wready,
    // Write Response Channel
    input  wire [ID_WIDTH-1:0]          m_axi_bid,
    input  wire [1:0]                   m_axi_bresp,
    input  wire                         m_axi_bvalid,
    output wire                         m_axi_bready,
    // Read Address Channel
    output wire [ID_WIDTH-1:0]          m_axi_arid,
    output wire [ADDR_WIDTH-1:0]        m_axi_araddr,
    output wire [7:0]                   m_axi_arlen,
    output wire [2:0]                   m_axi_arsize,
    output wire [1:0]                   m_axi_arburst,
    output wire [1:0]                   m_axi_arprot,
    output wire                         m_axi_arvalid,
    input  wire                         m_axi_arready,
    // Read Data Channel
    input  wire [ID_WIDTH-1:0]          m_axi_rid,
    input  wire [DATA_WIDTH-1:0]        m_axi_rdata,
    input  wire [1:0]                   m_axi_rresp,
    input  wire                         m_axi_rlast,
    input  wire                         m_axi_rvalid,
    output wire                         m_axi_rready,

    // -------------------------------------------------------------------------
    // Interrupts
    // -------------------------------------------------------------------------
    output wire [N_CHANNELS-1:0]        irq_done,   // transfer complete
    output wire [N_CHANNELS-1:0]        irq_err,    // error interrupt
    output wire                         irq_global  // OR of all
);

    // =========================================================================
    // Parameters & Local constants
    // =========================================================================
    localparam CH_REG_BASE   = 16'h0100;
    localparam CH_REG_STRIDE = 16'h0100;

    // Error codes stored in CH_STATUS[6:4]
    localparam ERR_NONE      = 3'h0;
    localparam ERR_SLVERR    = 3'h1;  // AXI SLVERR on read or write
    localparam ERR_DECERR    = 3'h2;  // AXI DECERR
    localparam ERR_TIMEOUT   = 3'h3;  // bus timeout
    localparam ERR_DESC      = 3'h4;  // descriptor fetch error
    localparam ERR_ALIGN     = 3'h5;  // address alignment error

    // =========================================================================
    // Global Registers
    // =========================================================================
    reg        global_enable;
    reg        global_sw_reset;
    reg [15:0] global_irq_mask;   // [7:0]=done mask, [15:8]=err mask
    reg [15:0] global_irq_status; // sticky, W1C

    // =========================================================================
    // Per-Channel Registers  (arrays indexed by channel)
    // =========================================================================
    reg                  ch_enable      [0:N_CHANNELS-1];
    reg                  ch_pause       [0:N_CHANNELS-1];
    reg                  ch_abort       [0:N_CHANNELS-1];
    reg [ADDR_WIDTH-1:0] ch_desc_base   [0:N_CHANNELS-1];
    reg [ADDR_WIDTH-1:0] ch_curr_desc   [0:N_CHANNELS-1];
    reg [31:0]           ch_bytes_done  [0:N_CHANNELS-1];
    reg [7:0]            ch_irq_cnt     [0:N_CHANNELS-1];

    // Status from channel engines
    wire [N_CHANNELS-1:0] ch_idle_w;
    wire [N_CHANNELS-1:0] ch_busy_w;
    wire [N_CHANNELS-1:0] ch_done_w;
    wire [N_CHANNELS-1:0] ch_err_w;
    wire [2:0]            ch_err_code_w [0:N_CHANNELS-1];
    wire [ADDR_WIDTH-1:0] ch_curr_desc_w[0:N_CHANNELS-1];
    wire [31:0]           ch_bytes_done_w[0:N_CHANNELS-1];

    // IRQ wires per channel
    wire [N_CHANNELS-1:0] ch_irq_done_w;
    wire [N_CHANNELS-1:0] ch_irq_err_w;

    // =========================================================================
    // AXI-Lite Write/Read arbitration state
    // =========================================================================
    reg [AXI_LITE_AW-1:0] aw_addr_lat;
    reg                    aw_done;
    reg [AXI_LITE_DW-1:0] w_data_lat;
    reg [AXI_LITE_DW/8-1:0] w_strb_lat;
    reg                    w_done;

    // Write address latch
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axil_awready <= 1'b0;
            aw_done        <= 1'b0;
        end else begin
            if (s_axil_awvalid && s_axil_awready) begin
                aw_addr_lat    <= s_axil_awaddr;
                s_axil_awready <= 1'b0;
                aw_done        <= 1'b1;
            end else if (!aw_done) begin
                s_axil_awready <= 1'b1;
            end else if (s_axil_bvalid && s_axil_bready) begin
                aw_done <= 1'b0;
            end
        end
    end

    // Write data latch
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axil_wready <= 1'b0;
            w_done        <= 1'b0;
        end else begin
            if (s_axil_wvalid && s_axil_wready) begin
                w_data_lat    <= s_axil_wdata;
                w_strb_lat    <= s_axil_wstrb;
                s_axil_wready <= 1'b0;
                w_done        <= 1'b1;
            end else if (!w_done) begin
                s_axil_wready <= 1'b1;
            end else if (s_axil_bvalid && s_axil_bready) begin
                w_done <= 1'b0;
            end
        end
    end

    // Write response
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axil_bvalid <= 1'b0;
            s_axil_bresp  <= 2'b00;
        end else begin
            if (aw_done && w_done && !s_axil_bvalid) begin
                s_axil_bvalid <= 1'b1;
                s_axil_bresp  <= 2'b00; // OKAY
                // --- Perform register write ---
                do_reg_write(aw_addr_lat, w_data_lat, w_strb_lat);
            end else if (s_axil_bvalid && s_axil_bready) begin
                s_axil_bvalid <= 1'b0;
            end
        end
    end

    // Read channel
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axil_arready <= 1'b0;
            s_axil_rvalid  <= 1'b0;
            s_axil_rresp   <= 2'b00;
            s_axil_rdata   <= '0;
        end else begin
            if (s_axil_arvalid && !s_axil_arready) begin
                s_axil_arready <= 1'b1;
            end else begin
                s_axil_arready <= 1'b0;
            end

            if (s_axil_arvalid && s_axil_arready) begin
                s_axil_rvalid <= 1'b1;
                s_axil_rresp  <= 2'b00;
                s_axil_rdata  <= do_reg_read(s_axil_araddr);
            end else if (s_axil_rvalid && s_axil_rready) begin
                s_axil_rvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Register Write Task
    // =========================================================================
    integer ci;
    task do_reg_write;
        input [AXI_LITE_AW-1:0] addr;
        input [AXI_LITE_DW-1:0] data;
        input [AXI_LITE_DW/8-1:0] strb;
        reg [AXI_LITE_DW-1:0] masked;
        reg [15:0] ch_off;
        integer    ch_id;
        begin
            masked = data & {{8{strb[3]}},{8{strb[2]}},{8{strb[1]}},{8{strb[0]}}};
            if (addr[15:8] == 8'h00) begin
                // Global registers
                case (addr[7:0])
                    8'h00: begin
                        global_enable   <= masked[0];
                        global_sw_reset <= masked[1];
                    end
                    8'h08: global_irq_mask   <= masked[15:0];
                    8'h04: global_irq_status <= global_irq_status & ~masked[15:0]; // W1C
                    default: ;
                endcase
            end else begin
                ch_off = addr - CH_REG_BASE;
                ch_id  = ch_off / CH_REG_STRIDE;
                if (ch_id < N_CHANNELS) begin
                    case (ch_off % CH_REG_STRIDE)
                        8'h00: begin
                            ch_enable[ch_id] <= masked[0];
                            ch_pause[ch_id]  <= masked[1];
                            ch_abort[ch_id]  <= masked[2];
                        end
                        8'h08: ch_desc_base[ch_id][31:0]  <= masked;
                        8'h0C: ch_desc_base[ch_id][63:32] <= masked;
                        default: ;
                    endcase
                end
            end
        end
    endtask

    // =========================================================================
    // Register Read Function
    // =========================================================================
    function [AXI_LITE_DW-1:0] do_reg_read;
        input [AXI_LITE_AW-1:0] addr;
        reg [15:0] ch_off;
        integer    ch_id;
        begin
            do_reg_read = 32'hDEAD_BEEF;
            if (addr[15:8] == 8'h00) begin
                case (addr[7:0])
                    8'h00: do_reg_read = {30'b0, global_sw_reset, global_enable};
                    8'h04: do_reg_read = {16'b0, global_irq_status};
                    8'h08: do_reg_read = {16'b0, global_irq_mask};
                    default: do_reg_read = 32'h0;
                endcase
            end else begin
                ch_off = addr - CH_REG_BASE;
                ch_id  = ch_off / CH_REG_STRIDE;
                if (ch_id < N_CHANNELS) begin
                    case (ch_off % CH_REG_STRIDE)
                        8'h00: do_reg_read = {29'b0, ch_abort[ch_id], ch_pause[ch_id], ch_enable[ch_id]};
                        8'h04: do_reg_read = {25'b0, ch_err_code_w[ch_id],
                                              ch_err_w[ch_id], ch_done_w[ch_id],
                                              ch_busy_w[ch_id], ch_idle_w[ch_id]};
                        8'h08: do_reg_read = ch_curr_desc_w[ch_id][31:0];
                        8'h0C: do_reg_read = ch_curr_desc_w[ch_id][63:32];
                        8'h10: do_reg_read = ch_desc_base[ch_id][31:0];
                        8'h14: do_reg_read = ch_desc_base[ch_id][63:32];
                        8'h18: do_reg_read = ch_bytes_done_w[ch_id];
                        8'h1C: do_reg_read = {24'b0, ch_irq_cnt[ch_id]};
                        default: do_reg_read = 32'h0;
                    endcase
                end
            end
        end
    endfunction

    // =========================================================================
    // Global IRQ status update
    // =========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            global_irq_status <= 16'h0;
        end else begin
            // Set bits on rising edge of done/err
            global_irq_status[7:0]  <= global_irq_status[7:0]  | ch_irq_done_w;
            global_irq_status[15:8] <= global_irq_status[15:8] | ch_irq_err_w;
        end
    end

    // IRQ counter per channel
    genvar gi;
    generate
        for (gi = 0; gi < N_CHANNELS; gi = gi + 1) begin : irq_cnt_blk
            always @(posedge aclk or negedge aresetn) begin
                if (!aresetn) begin
                    ch_irq_cnt[gi] <= 8'h0;
                end else if (ch_irq_done_w[gi] || ch_irq_err_w[gi]) begin
                    ch_irq_cnt[gi] <= (ch_irq_cnt[gi] == 8'hFF) ? 8'hFF : ch_irq_cnt[gi] + 1;
                end
            end
        end
    endgenerate

    // =========================================================================
    // AXI4 Master Arbiter / MUX (Round-Robin across 8 channels)
    // =========================================================================
    // Internal per-channel AXI master signals
    wire [ADDR_WIDTH-1:0] ch_axi_araddr  [0:N_CHANNELS-1];
    wire [7:0]            ch_axi_arlen   [0:N_CHANNELS-1];
    wire [2:0]            ch_axi_arsize  [0:N_CHANNELS-1];
    wire [1:0]            ch_axi_arburst [0:N_CHANNELS-1];
    wire [1:0]            ch_axi_arprot  [0:N_CHANNELS-1];
    wire                  ch_axi_arvalid [0:N_CHANNELS-1];
    reg                   ch_axi_arready [0:N_CHANNELS-1];

    wire [DATA_WIDTH-1:0] ch_axi_rdata   [0:N_CHANNELS-1];
    wire [1:0]            ch_axi_rresp   [0:N_CHANNELS-1];
    wire                  ch_axi_rlast   [0:N_CHANNELS-1];
    wire                  ch_axi_rvalid  [0:N_CHANNELS-1];
    reg                   ch_axi_rready  [0:N_CHANNELS-1];

    wire [ADDR_WIDTH-1:0] ch_axi_awaddr  [0:N_CHANNELS-1];
    wire [7:0]            ch_axi_awlen   [0:N_CHANNELS-1];
    wire [2:0]            ch_axi_awsize  [0:N_CHANNELS-1];
    wire [1:0]            ch_axi_awburst [0:N_CHANNELS-1];
    wire [1:0]            ch_axi_awprot  [0:N_CHANNELS-1];
    wire                  ch_axi_awvalid [0:N_CHANNELS-1];
    reg                   ch_axi_awready [0:N_CHANNELS-1];

    wire [DATA_WIDTH-1:0] ch_axi_wdata   [0:N_CHANNELS-1];
    wire [DATA_WIDTH/8-1:0] ch_axi_wstrb [0:N_CHANNELS-1];
    wire                  ch_axi_wlast   [0:N_CHANNELS-1];
    wire                  ch_axi_wvalid  [0:N_CHANNELS-1];
    reg                   ch_axi_wready  [0:N_CHANNELS-1];

    wire [1:0]            ch_axi_bresp   [0:N_CHANNELS-1];
    wire                  ch_axi_bvalid  [0:N_CHANNELS-1];
    reg                   ch_axi_bready  [0:N_CHANNELS-1];

    // Arbiter grants
    reg [2:0] ar_grant, aw_grant;
    reg       ar_busy, aw_busy;

    // Round-robin arbiter for AR channel
    axi_rr_arbiter #(.N(N_CHANNELS), .AW(ADDR_WIDTH), .DW(DATA_WIDTH))
    u_ar_arb (
        .aclk         (aclk),
        .aresetn      (aresetn),
        // requests
        .ch_arvalid   ({ch_axi_arvalid[7],ch_axi_arvalid[6],ch_axi_arvalid[5],
                        ch_axi_arvalid[4],ch_axi_arvalid[3],ch_axi_arvalid[2],
                        ch_axi_arvalid[1],ch_axi_arvalid[0]}),
        .ch_araddr    ({ch_axi_araddr[7], ch_axi_araddr[6], ch_axi_araddr[5],
                        ch_axi_araddr[4], ch_axi_araddr[3], ch_axi_araddr[2],
                        ch_axi_araddr[1], ch_axi_araddr[0]}),
        .ch_arlen     ({ch_axi_arlen[7],  ch_axi_arlen[6],  ch_axi_arlen[5],
                        ch_axi_arlen[4],  ch_axi_arlen[3],  ch_axi_arlen[2],
                        ch_axi_arlen[1],  ch_axi_arlen[0]}),
        .ch_arsize    ({ch_axi_arsize[7], ch_axi_arsize[6], ch_axi_arsize[5],
                        ch_axi_arsize[4], ch_axi_arsize[3], ch_axi_arsize[2],
                        ch_axi_arsize[1], ch_axi_arsize[0]}),
        .ch_arburst   ({ch_axi_arburst[7],ch_axi_arburst[6],ch_axi_arburst[5],
                        ch_axi_arburst[4],ch_axi_arburst[3],ch_axi_arburst[2],
                        ch_axi_arburst[1],ch_axi_arburst[0]}),
        .ch_arprot    ({ch_axi_arprot[7], ch_axi_arprot[6], ch_axi_arprot[5],
                        ch_axi_arprot[4], ch_axi_arprot[3], ch_axi_arprot[2],
                        ch_axi_arprot[1], ch_axi_arprot[0]}),
        .ch_arready   ({ch_axi_arready[7],ch_axi_arready[6],ch_axi_arready[5],
                        ch_axi_arready[4],ch_axi_arready[3],ch_axi_arready[2],
                        ch_axi_arready[1],ch_axi_arready[0]}),
        // AXI master output
        .m_axi_araddr (m_axi_araddr),
        .m_axi_arlen  (m_axi_arlen),
        .m_axi_arsize (m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arprot (m_axi_arprot),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_arid   (m_axi_arid),
        // read data back to channels
        .m_axi_rdata  (m_axi_rdata),
        .m_axi_rresp  (m_axi_rresp),
        .m_axi_rlast  (m_axi_rlast),
        .m_axi_rvalid (m_axi_rvalid),
        .m_axi_rready (m_axi_rready),
        .m_axi_rid    (m_axi_rid),
        .ch_rdata     ({ch_axi_rdata[7], ch_axi_rdata[6], ch_axi_rdata[5],
                        ch_axi_rdata[4], ch_axi_rdata[3], ch_axi_rdata[2],
                        ch_axi_rdata[1], ch_axi_rdata[0]}),
        .ch_rresp     ({ch_axi_rresp[7], ch_axi_rresp[6], ch_axi_rresp[5],
                        ch_axi_rresp[4], ch_axi_rresp[3], ch_axi_rresp[2],
                        ch_axi_rresp[1], ch_axi_rresp[0]}),
        .ch_rlast     ({ch_axi_rlast[7], ch_axi_rlast[6], ch_axi_rlast[5],
                        ch_axi_rlast[4], ch_axi_rlast[3], ch_axi_rlast[2],
                        ch_axi_rlast[1], ch_axi_rlast[0]}),
        .ch_rvalid    ({ch_axi_rvalid[7],ch_axi_rvalid[6],ch_axi_rvalid[5],
                        ch_axi_rvalid[4],ch_axi_rvalid[3],ch_axi_rvalid[2],
                        ch_axi_rvalid[1],ch_axi_rvalid[0]}),
        .ch_rready    ({ch_axi_rready[7],ch_axi_rready[6],ch_axi_rready[5],
                        ch_axi_rready[4],ch_axi_rready[3],ch_axi_rready[2],
                        ch_axi_rready[1],ch_axi_rready[0]})
    );

    // AW/W/B arbiter
    axi_wr_arbiter #(.N(N_CHANNELS), .AW(ADDR_WIDTH), .DW(DATA_WIDTH))
    u_aw_arb (
        .aclk         (aclk),
        .aresetn      (aresetn),
        .ch_awvalid   ({ch_axi_awvalid[7],ch_axi_awvalid[6],ch_axi_awvalid[5],
                        ch_axi_awvalid[4],ch_axi_awvalid[3],ch_axi_awvalid[2],
                        ch_axi_awvalid[1],ch_axi_awvalid[0]}),
        .ch_awaddr    ({ch_axi_awaddr[7], ch_axi_awaddr[6], ch_axi_awaddr[5],
                        ch_axi_awaddr[4], ch_axi_awaddr[3], ch_axi_awaddr[2],
                        ch_axi_awaddr[1], ch_axi_awaddr[0]}),
        .ch_awlen     ({ch_axi_awlen[7],  ch_axi_awlen[6],  ch_axi_awlen[5],
                        ch_axi_awlen[4],  ch_axi_awlen[3],  ch_axi_awlen[2],
                        ch_axi_awlen[1],  ch_axi_awlen[0]}),
        .ch_awsize    ({ch_axi_awsize[7], ch_axi_awsize[6], ch_axi_awsize[5],
                        ch_axi_awsize[4], ch_axi_awsize[3], ch_axi_awsize[2],
                        ch_axi_awsize[1], ch_axi_awsize[0]}),
        .ch_awburst   ({ch_axi_awburst[7],ch_axi_awburst[6],ch_axi_awburst[5],
                        ch_axi_awburst[4],ch_axi_awburst[3],ch_axi_awburst[2],
                        ch_axi_awburst[1],ch_axi_awburst[0]}),
        .ch_awprot    ({ch_axi_awprot[7], ch_axi_awprot[6], ch_axi_awprot[5],
                        ch_axi_awprot[4], ch_axi_awprot[3], ch_axi_awprot[2],
                        ch_axi_awprot[1], ch_axi_awprot[0]}),
        .ch_awready   ({ch_axi_awready[7],ch_axi_awready[6],ch_axi_awready[5],
                        ch_axi_awready[4],ch_axi_awready[3],ch_axi_awready[2],
                        ch_axi_awready[1],ch_axi_awready[0]}),
        .ch_wdata     ({ch_axi_wdata[7],  ch_axi_wdata[6],  ch_axi_wdata[5],
                        ch_axi_wdata[4],  ch_axi_wdata[3],  ch_axi_wdata[2],
                        ch_axi_wdata[1],  ch_axi_wdata[0]}),
        .ch_wstrb     ({ch_axi_wstrb[7],  ch_axi_wstrb[6],  ch_axi_wstrb[5],
                        ch_axi_wstrb[4],  ch_axi_wstrb[3],  ch_axi_wstrb[2],
                        ch_axi_wstrb[1],  ch_axi_wstrb[0]}),
        .ch_wlast     ({ch_axi_wlast[7],  ch_axi_wlast[6],  ch_axi_wlast[5],
                        ch_axi_wlast[4],  ch_axi_wlast[3],  ch_axi_wlast[2],
                        ch_axi_wlast[1],  ch_axi_wlast[0]}),
        .ch_wvalid    ({ch_axi_wvalid[7], ch_axi_wvalid[6], ch_axi_wvalid[5],
                        ch_axi_wvalid[4], ch_axi_wvalid[3], ch_axi_wvalid[2],
                        ch_axi_wvalid[1], ch_axi_wvalid[0]}),
        .ch_wready    ({ch_axi_wready[7], ch_axi_wready[6], ch_axi_wready[5],
                        ch_axi_wready[4], ch_axi_wready[3], ch_axi_wready[2],
                        ch_axi_wready[1], ch_axi_wready[0]}),
        .ch_bresp     ({ch_axi_bresp[7],  ch_axi_bresp[6],  ch_axi_bresp[5],
                        ch_axi_bresp[4],  ch_axi_bresp[3],  ch_axi_bresp[2],
                        ch_axi_bresp[1],  ch_axi_bresp[0]}),
        .ch_bvalid    ({ch_axi_bvalid[7], ch_axi_bvalid[6], ch_axi_bvalid[5],
                        ch_axi_bvalid[4], ch_axi_bvalid[3], ch_axi_bvalid[2],
                        ch_axi_bvalid[1], ch_axi_bvalid[0]}),
        .ch_bready    ({ch_axi_bready[7], ch_axi_bready[6], ch_axi_bready[5],
                        ch_axi_bready[4], ch_axi_bready[3], ch_axi_bready[2],
                        ch_axi_bready[1], ch_axi_bready[0]}),
        .m_axi_awid   (m_axi_awid),
        .m_axi_awaddr (m_axi_awaddr),
        .m_axi_awlen  (m_axi_awlen),
        .m_axi_awsize (m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awprot (m_axi_awprot),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata  (m_axi_wdata),
        .m_axi_wstrb  (m_axi_wstrb),
        .m_axi_wlast  (m_axi_wlast),
        .m_axi_wvalid (m_axi_wvalid),
        .m_axi_wready (m_axi_wready),
        .m_axi_bid    (m_axi_bid),
        .m_axi_bresp  (m_axi_bresp),
        .m_axi_bvalid (m_axi_bvalid),
        .m_axi_bready (m_axi_bready)
    );

    // =========================================================================
    // Channel Instances
    // =========================================================================
    generate
        for (gi = 0; gi < N_CHANNELS; gi = gi + 1) begin : ch_inst
            dma_channel #(
                .CHANNEL_ID   (gi),
                .ADDR_WIDTH   (ADDR_WIDTH),
                .DATA_WIDTH   (DATA_WIDTH),
                .MAX_BURST_LEN(MAX_BURST_LEN)
            ) u_ch (
                .aclk          (aclk),
                .aresetn       (aresetn & ~global_sw_reset),
                .ch_enable     (ch_enable[gi] & global_enable),
                .ch_pause      (ch_pause[gi]),
                .ch_abort      (ch_abort[gi]),
                .ch_desc_base  (ch_desc_base[gi]),
                // Status outputs
                .ch_idle       (ch_idle_w[gi]),
                .ch_busy       (ch_busy_w[gi]),
                .ch_done       (ch_done_w[gi]),
                .ch_err        (ch_err_w[gi]),
                .ch_err_code   (ch_err_code_w[gi]),
                .ch_curr_desc  (ch_curr_desc_w[gi]),
                .ch_bytes_done (ch_bytes_done_w[gi]),
                // IRQ pulses
                .irq_done      (ch_irq_done_w[gi]),
                .irq_err       (ch_irq_err_w[gi]),
                // AXI Read
                .m_axi_araddr  (ch_axi_araddr[gi]),
                .m_axi_arlen   (ch_axi_arlen[gi]),
                .m_axi_arsize  (ch_axi_arsize[gi]),
                .m_axi_arburst (ch_axi_arburst[gi]),
                .m_axi_arprot  (ch_axi_arprot[gi]),
                .m_axi_arvalid (ch_axi_arvalid[gi]),
                .m_axi_arready (ch_axi_arready[gi]),
                .m_axi_rdata   (ch_axi_rdata[gi]),
                .m_axi_rresp   (ch_axi_rresp[gi]),
                .m_axi_rlast   (ch_axi_rlast[gi]),
                .m_axi_rvalid  (ch_axi_rvalid[gi]),
                .m_axi_rready  (ch_axi_rready[gi]),
                // AXI Write
                .m_axi_awaddr  (ch_axi_awaddr[gi]),
                .m_axi_awlen   (ch_axi_awlen[gi]),
                .m_axi_awsize  (ch_axi_awsize[gi]),
                .m_axi_awburst (ch_axi_awburst[gi]),
                .m_axi_awprot  (ch_axi_awprot[gi]),
                .m_axi_awvalid (ch_axi_awvalid[gi]),
                .m_axi_awready (ch_axi_awready[gi]),
                .m_axi_wdata   (ch_axi_wdata[gi]),
                .m_axi_wstrb   (ch_axi_wstrb[gi]),
                .m_axi_wlast   (ch_axi_wlast[gi]),
                .m_axi_wvalid  (ch_axi_wvalid[gi]),
                .m_axi_wready  (ch_axi_wready[gi]),
                .m_axi_bresp   (ch_axi_bresp[gi]),
                .m_axi_bvalid  (ch_axi_bvalid[gi]),
                .m_axi_bready  (ch_axi_bready[gi])
            );
        end
    endgenerate

    // =========================================================================
    // Global IRQ output
    // =========================================================================
    assign irq_done   = ch_irq_done_w & global_irq_mask[7:0];
    assign irq_err    = ch_irq_err_w  & global_irq_mask[15:8];
    assign irq_global = |irq_done | |irq_err;

endmodule
