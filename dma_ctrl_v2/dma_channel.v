// dma_channel.v
// One DMA channel: ties together descriptor ring + AXI masters

`include "dma_pkg.vh"

module dma_channel #(
    parameter CH_ID       = 0,
    parameter ADDR_WIDTH  = `DMA_ADDR_WIDTH,
    parameter DATA_WIDTH  = `DMA_DATA_WIDTH,
    parameter ID_WIDTH    = `DMA_ID_WIDTH
)(
    input  wire clk,
    input  wire rst_n,

    // --- CSR control ---
    input  wire        ch_enable,
    input  wire        ch_pause,
    input  wire        sw_tail_wr,
    input  wire [3:0]  sw_tail,
    input  wire        sw_head_wr,
    input  wire [3:0]  sw_head,

    // SW descriptor load port
    input  wire        sw_desc_wr,
    input  wire [3:0]  sw_desc_addr,
    input  wire [255:0] sw_desc_data,

    // --- Status outputs ---
    output wire [3:0]  head_ptr,
    output wire [3:0]  tail_ptr,
    output reg         ch_active,
    output reg         ch_done,       // Pulse on descriptor completion
    output reg         ch_err,        // Pulse on any error
    output reg [4:0]   err_code,      // Error bits

    // Byte count of last completed transfer
    output reg [31:0]  xfer_count,

    // === AXI4 Read Master ports ===
    output wire [ID_WIDTH-1:0]     m_axi_arid,
    output wire [ADDR_WIDTH-1:0]   m_axi_araddr,
    output wire [7:0]              m_axi_arlen,
    output wire [2:0]              m_axi_arsize,
    output wire [1:0]              m_axi_arburst,
    output wire                    m_axi_arlock,
    output wire [3:0]              m_axi_arcache,
    output wire [2:0]              m_axi_arprot,
    output wire [3:0]              m_axi_arqos,
    output wire                    m_axi_arvalid,
    input  wire                    m_axi_arready,
    input  wire [ID_WIDTH-1:0]     m_axi_rid,
    input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output wire                    m_axi_rready,

    // === AXI4 Write Master ports ===
    output wire [ID_WIDTH-1:0]     m_axi_awid,
    output wire [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output wire [7:0]              m_axi_awlen,
    output wire [2:0]              m_axi_awsize,
    output wire [1:0]              m_axi_awburst,
    output wire                    m_axi_awlock,
    output wire [3:0]              m_axi_awcache,
    output wire [2:0]              m_axi_awprot,
    output wire [3:0]              m_axi_awqos,
    output wire                    m_axi_awvalid,
    input  wire                    m_axi_awready,
    output wire [DATA_WIDTH-1:0]   m_axi_wdata,
    output wire [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                    m_axi_wlast,
    output wire                    m_axi_wvalid,
    input  wire                    m_axi_wready,
    input  wire [ID_WIDTH-1:0]     m_axi_bid,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output wire                    m_axi_bready
);

    // -------------------------------------------------------
    // Internal wires
    // -------------------------------------------------------

    // Descriptor ring <-> descriptor SRAM
    wire        desc_rd_en;
    wire [3:0]  desc_rd_addr;
    wire [255:0] desc_rd_data;
    wire        desc_rd_valid;
    wire        desc_wr_en;
    wire [3:0]  desc_wr_addr;
    wire [31:0] desc_wr_status;

    // Ring manager <-> channel engine
    wire        desc_avail;
    wire [63:0] desc_src_addr;
    wire [63:0] desc_dst_addr;
    wire [31:0] desc_byte_cnt;
    wire [31:0] desc_ctrl;
    wire [3:0]  desc_idx;

    // Read master -> write master data path (internal FIFO bypass)
    wire        rd_data_valid;
    wire        rd_data_ready;
    wire [DATA_WIDTH-1:0] rd_data;
    wire [DATA_WIDTH/8-1:0] rd_data_keep;
    wire        rd_data_last;
    wire [ID_WIDTH-1:0] rd_data_id;

    // Write master status
    wire        wr_done;
    wire [ID_WIDTH-1:0] wr_done_id;
    wire        wr_err;
    wire [ID_WIDTH-1:0] wr_err_id;
    wire        rd_err;
    wire [ID_WIDTH-1:0] rd_err_id;

    // Read master command
    reg         rd_cmd_valid;
    wire        rd_cmd_ready;

    // Write master command
    reg         wr_cmd_valid;
    wire        wr_cmd_ready;

    // -------------------------------------------------------
    // State machine
    // -------------------------------------------------------
    localparam ST_IDLE    = 3'd0;
    localparam ST_LAUNCH  = 3'd1;
    localparam ST_RUNNING = 3'd2;
    localparam ST_WAIT_WR = 3'd3;
    localparam ST_DONE    = 3'd4;
    localparam ST_ERROR   = 3'd5;

    reg [2:0]  state;
    reg        desc_consume;
    reg        desc_done;
    reg [31:0] desc_done_status;
    reg        desc_error;
    reg [63:0] latch_src;
    reg [63:0] latch_dst;
    reg [31:0] latch_len;
    reg [31:0] latch_ctrl;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            ch_active    <= 1'b0;
            ch_done      <= 1'b0;
            ch_err       <= 1'b0;
            err_code     <= 5'b0;
            rd_cmd_valid <= 1'b0;
            wr_cmd_valid <= 1'b0;
            desc_consume <= 1'b0;
            desc_done    <= 1'b0;
            desc_error   <= 1'b0;
            xfer_count   <= 32'd0;
        end else begin
            // Defaults
            ch_done      <= 1'b0;
            ch_err       <= 1'b0;
            desc_consume <= 1'b0;
            desc_done    <= 1'b0;
            desc_error   <= 1'b0;

            case (state)
                // -------------------------------------------
                ST_IDLE: begin
                    ch_active <= 1'b0;
                    if (ch_enable && !ch_pause && desc_avail) begin
                        latch_src  <= desc_src_addr;
                        latch_dst  <= desc_dst_addr;
                        latch_len  <= desc_byte_cnt;
                        latch_ctrl <= desc_ctrl;
                        desc_consume <= 1'b1;
                        state        <= ST_LAUNCH;
                    end
                end

                // -------------------------------------------
                ST_LAUNCH: begin
                    ch_active <= 1'b1;
                    // Issue read and write commands simultaneously
                    rd_cmd_valid <= 1'b1;
                    wr_cmd_valid <= 1'b1;

                    if (rd_cmd_valid && rd_cmd_ready)
                        rd_cmd_valid <= 1'b0;
                    if (wr_cmd_valid && wr_cmd_ready)
                        wr_cmd_valid <= 1'b0;

                    if (!rd_cmd_valid && !wr_cmd_valid)
                        state <= ST_RUNNING;
                end

                // -------------------------------------------
                ST_RUNNING: begin
                    // Check for errors
                    if (rd_err) begin
                        err_code    <= 5'h02; // SRC_ERR
                        desc_error  <= 1'b1;
                        desc_done_status <= 32'h02;
                        state       <= ST_ERROR;
                    end else if (wr_err) begin
                        err_code    <= 5'h04; // DST_ERR
                        desc_error  <= 1'b1;
                        desc_done_status <= 32'h04;
                        state       <= ST_ERROR;
                    end else if (wr_done) begin
                        xfer_count  <= latch_len;
                        desc_done   <= 1'b1;
                        desc_done_status <= 32'h01; // DONE
                        state       <= ST_DONE;
                    end
                end

                // -------------------------------------------
                ST_DONE: begin
                    ch_done  <= 1'b1;
                    ch_active<= 1'b0;
                    state    <= ST_IDLE;
                end

                // -------------------------------------------
                ST_ERROR: begin
                    ch_err   <= 1'b1;
                    ch_active<= 1'b0;
                    state    <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------
    // Descriptor SRAM
    // -------------------------------------------------------
    desc_sram #(
        .DEPTH     (`DMA_DESC_DEPTH),
        .ADDR_BITS (`DMA_DESC_ADDR_BITS)
    ) u_desc_sram (
        .clk         (clk),
        .wr_en       (desc_wr_en),
        .wr_addr     (desc_wr_addr),
        .wr_data     ({224'd0, desc_wr_status}),
        .wr_mask     (32'hFFFFFFFF),
        .rd_en       (desc_rd_en),
        .rd_addr     (desc_rd_addr),
        .rd_data     (desc_rd_data),
        .rd_valid    (desc_rd_valid),
        .sw_wr_en    (sw_desc_wr),
        .sw_wr_addr  (sw_desc_addr),
        .sw_wr_data  (sw_desc_data)
    );

    // -------------------------------------------------------
    // Descriptor Ring Manager
    // -------------------------------------------------------
    desc_ring_manager #(
        .DEPTH     (`DMA_DESC_DEPTH),
        .ADDR_BITS (`DMA_DESC_ADDR_BITS),
        .CH_ID     (CH_ID)
    ) u_ring (
        .clk              (clk),
        .rst_n            (rst_n),
        .sw_tail_wr       (sw_tail_wr),
        .sw_tail          (sw_tail),
        .sw_head_wr       (sw_head_wr),
        .sw_head          (sw_head),
        .desc_wr_en       (desc_wr_en),
        .desc_wr_addr     (desc_wr_addr),
        .desc_wr_status   (desc_wr_status),
        .desc_rd_en       (desc_rd_en),
        .desc_rd_addr     (desc_rd_addr),
        .desc_rd_data     (desc_rd_data),
        .desc_rd_valid    (desc_rd_valid),
        .desc_avail       (desc_avail),
        .desc_src_addr    (desc_src_addr),
        .desc_dst_addr    (desc_dst_addr),
        .desc_byte_cnt    (desc_byte_cnt),
        .desc_ctrl        (desc_ctrl),
        .desc_idx         (desc_idx),
        .desc_consume     (desc_consume),
        .desc_done        (desc_done),
        .desc_done_status (desc_done_status),
        .desc_error       (desc_error),
        .head_ptr         (head_ptr),
        .tail_ptr         (tail_ptr),
        .ring_full        (),
        .ring_empty       ()
    );

    // -------------------------------------------------------
    // AXI Read Master
    // -------------------------------------------------------
    axi_read_master #(
        .ADDR_WIDTH  (ADDR_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .ID_WIDTH    (ID_WIDTH),
        .MAX_BURST   (`DMA_BURST_LEN)
    ) u_rd_master (
        .clk              (clk),
        .rst_n            (rst_n),
        .cmd_valid        (rd_cmd_valid),
        .cmd_ready        (rd_cmd_ready),
        .cmd_addr         (latch_src),
        .cmd_byte_len     (latch_len),
        .cmd_id           (CH_ID[ID_WIDTH-1:0]),
        .cmd_burst_size   (latch_ctrl[`DESC_CTRL_BURST_SZ]),
        .cmd_cache        (latch_ctrl[`DESC_CTRL_CACHE]),
        .cmd_prot         (latch_ctrl[`DESC_CTRL_PROT[2:0]]),
        .data_valid       (rd_data_valid),
        .data_ready       (rd_data_ready),
        .data             (rd_data),
        .data_keep        (rd_data_keep),
        .data_last        (rd_data_last),
        .data_id          (rd_data_id),
        .err_resp         (rd_err),
        .err_id           (rd_err_id),
        .m_axi_arid       (m_axi_arid),
        .m_axi_araddr     (m_axi_araddr),
        .m_axi_arlen      (m_axi_arlen),
        .m_axi_arsize     (m_axi_arsize),
        .m_axi_arburst    (m_axi_arburst),
        .m_axi_arlock     (m_axi_arlock),
        .m_axi_arcache    (m_axi_arcache),
        .m_axi_arprot     (m_axi_arprot),
        .m_axi_arqos      (m_axi_arqos),
        .m_axi_arvalid    (m_axi_arvalid),
        .m_axi_arready    (m_axi_arready),
        .m_axi_rid        (m_axi_rid),
        .m_axi_rdata      (m_axi_rdata),
        .m_axi_rresp      (m_axi_rresp),
        .m_axi_rlast      (m_axi_rlast),
        .m_axi_rvalid     (m_axi_rvalid),
        .m_axi_rready     (m_axi_rready)
    );

    // -------------------------------------------------------
    // AXI Write Master
    // -------------------------------------------------------
    axi_write_master #(
        .ADDR_WIDTH  (ADDR_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .ID_WIDTH    (ID_WIDTH),
        .MAX_BURST   (`DMA_BURST_LEN)
    ) u_wr_master (
        .clk              (clk),
        .rst_n            (rst_n),
        .cmd_valid        (wr_cmd_valid),
        .cmd_ready        (wr_cmd_ready),
        .cmd_addr         (latch_dst),
        .cmd_byte_len     (latch_len),
        .cmd_id           (CH_ID[ID_WIDTH-1:0]),
        .cmd_burst_size   (latch_ctrl[`DESC_CTRL_BURST_SZ]),
        .cmd_cache        (latch_ctrl[`DESC_CTRL_CACHE]),
        .cmd_prot         (latch_ctrl[`DESC_CTRL_PROT[2:0]]),
        .wdata_valid      (rd_data_valid),
        .wdata_ready      (rd_data_ready),
        .wdata            (rd_data),
        .wdata_keep       (rd_data_keep),
        .wdata_last       (rd_data_last),
        .wdata_id         (rd_data_id),
        .done             (wr_done),
        .done_id          (wr_done_id),
        .err_resp         (wr_err),
        .err_id           (wr_err_id),
        .m_axi_awid       (m_axi_awid),
        .m_axi_awaddr     (m_axi_awaddr),
        .m_axi_awlen      (m_axi_awlen),
        .m_axi_awsize     (m_axi_awsize),
        .m_axi_awburst    (m_axi_awburst),
        .m_axi_awlock     (m_axi_awlock),
        .m_axi_awcache    (m_axi_awcache),
        .m_axi_awprot     (m_axi_awprot),
        .m_axi_awqos      (m_axi_awqos),
        .m_axi_awvalid    (m_axi_awvalid),
        .m_axi_awready    (m_axi_awready),
        .m_axi_wdata      (m_axi_wdata),
        .m_axi_wstrb      (m_axi_wstrb),
        .m_axi_wlast      (m_axi_wlast),
        .m_axi_wvalid     (m_axi_wvalid),
        .m_axi_wready     (m_axi_wready),
        .m_axi_bid        (m_axi_bid),
        .m_axi_bresp      (m_axi_bresp),
        .m_axi_bvalid     (m_axi_bvalid),
        .m_axi_bready     (m_axi_bready)
    );

endmodule
