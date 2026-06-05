// axi_write_master.v
// Pipelined AXI4 Write Master — issues burst write transactions

`include "dma_pkg.vh"

module axi_write_master #(
    parameter ADDR_WIDTH  = `DMA_ADDR_WIDTH,
    parameter DATA_WIDTH  = `DMA_DATA_WIDTH,
    parameter ID_WIDTH    = `DMA_ID_WIDTH,
    parameter MAX_BURST   = `DMA_BURST_LEN
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // --- Command Interface ---
    input  wire                    cmd_valid,
    output wire                    cmd_ready,
    input  wire [ADDR_WIDTH-1:0]   cmd_addr,
    input  wire [31:0]             cmd_byte_len,
    input  wire [ID_WIDTH-1:0]     cmd_id,
    input  wire [3:0]              cmd_burst_size,
    input  wire [7:0]              cmd_cache,
    input  wire [2:0]              cmd_prot,

    // --- Write Data Input ---
    input  wire                    wdata_valid,
    output reg                     wdata_ready,
    input  wire [DATA_WIDTH-1:0]   wdata,
    input  wire [DATA_WIDTH/8-1:0] wdata_keep,
    input  wire                    wdata_last,
    input  wire [ID_WIDTH-1:0]     wdata_id,

    // --- Status ---
    output reg                     done,
    output reg  [ID_WIDTH-1:0]     done_id,
    output reg                     err_resp,
    output reg  [ID_WIDTH-1:0]     err_id,

    // === AXI4 AW Channel ===
    output reg  [ID_WIDTH-1:0]     m_axi_awid,
    output reg  [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output reg  [7:0]              m_axi_awlen,
    output reg  [2:0]              m_axi_awsize,
    output reg  [1:0]              m_axi_awburst,
    output reg                     m_axi_awlock,
    output reg  [3:0]              m_axi_awcache,
    output reg  [2:0]              m_axi_awprot,
    output reg  [3:0]              m_axi_awqos,
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,

    // === AXI4 W Channel ===
    output reg  [DATA_WIDTH-1:0]   m_axi_wdata,
    output reg  [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg                     m_axi_wlast,
    output reg                     m_axi_wvalid,
    input  wire                    m_axi_wready,

    // === AXI4 B Channel ===
    input  wire [ID_WIDTH-1:0]     m_axi_bid,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output reg                     m_axi_bready
);

    // -------------------------------------------------------
    localparam ST_IDLE     = 3'd0;
    localparam ST_ADDR     = 3'd1;
    localparam ST_DATA     = 3'd2;
    localparam ST_RESP     = 3'd3;

    reg [2:0]              state;
    reg [ADDR_WIDTH-1:0]   cur_addr;
    reg [31:0]             bytes_remain;
    reg [ID_WIDTH-1:0]     cur_id;
    reg [3:0]              cur_burst_size;
    reg [7:0]              cur_cache;
    reg [2:0]              cur_prot;
    reg [7:0]              burst_remain;    // beats left in burst
    reg [31:0]             bytes_per_beat;

    assign cmd_ready = (state == ST_IDLE);

    function [31:0] size_to_bytes;
        input [3:0] sz;
        begin size_to_bytes = 32'd1 << sz; end
    endfunction

    function [8:0] calc_burst_len;
        input [31:0] remain;
        input [31:0] bpb;
        input integer max_b;
        reg [31:0] beats;
        begin
            beats = remain / bpb;
            if (beats > max_b) beats = max_b;
            if (beats > 256)   beats = 256;
            if (beats == 0)    beats = 1;
            calc_burst_len = beats[8:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            m_axi_awvalid  <= 1'b0;
            m_axi_wvalid   <= 1'b0;
            m_axi_bready   <= 1'b0;
            wdata_ready    <= 1'b0;
            done           <= 1'b0;
            err_resp       <= 1'b0;
            bytes_remain   <= 32'd0;
        end else begin
            done     <= 1'b0;
            err_resp <= 1'b0;

            case (state)
                // -----------------------------------------
                ST_IDLE: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    if (cmd_valid) begin
                        cur_addr       <= cmd_addr;
                        bytes_remain   <= cmd_byte_len;
                        cur_id         <= cmd_id;
                        cur_burst_size <= cmd_burst_size;
                        cur_cache      <= cmd_cache;
                        cur_prot       <= cmd_prot;
                        bytes_per_beat <= size_to_bytes(cmd_burst_size);
                        state          <= ST_ADDR;
                    end
                end

                // -----------------------------------------
                ST_ADDR: begin
                    if (bytes_remain == 0) begin
                        state <= ST_IDLE;
                    end else begin
                        burst_remain  <= calc_burst_len(bytes_remain,
                                                        bytes_per_beat,
                                                        MAX_BURST) - 1;
                        m_axi_awid    <= cur_id;
                        m_axi_awaddr  <= cur_addr;
                        m_axi_awlen   <= calc_burst_len(bytes_remain,
                                                        bytes_per_beat,
                                                        MAX_BURST) - 1;
                        m_axi_awsize  <= cur_burst_size[2:0];
                        m_axi_awburst <= `AXI_BURST_INCR;
                        m_axi_awlock  <= 1'b0;
                        m_axi_awcache <= cur_cache[3:0];
                        m_axi_awprot  <= cur_prot;
                        m_axi_awqos   <= 4'd0;
                        m_axi_awvalid <= 1'b1;
                        wdata_ready   <= 1'b1;

                        if (m_axi_awvalid && m_axi_awready) begin
                            m_axi_awvalid <= 1'b0;
                            state         <= ST_DATA;
                        end
                    end
                end

                // -----------------------------------------
                ST_DATA: begin
                    if (wdata_valid && wdata_ready) begin
                        m_axi_wdata  <= wdata;
                        m_axi_wstrb  <= wdata_keep;
                        m_axi_wlast  <= (burst_remain == 0);
                        m_axi_wvalid <= 1'b1;

                        if (m_axi_wvalid && m_axi_wready) begin
                            if (bytes_remain >= bytes_per_beat)
                                bytes_remain <= bytes_remain - bytes_per_beat;
                            else
                                bytes_remain <= 32'd0;

                            if (burst_remain == 0) begin
                                // Last beat in burst
                                m_axi_wvalid  <= 1'b0;
                                wdata_ready   <= 1'b0;
                                cur_addr      <= cur_addr + (bytes_per_beat *
                                                 (burst_remain + 1));
                                m_axi_bready  <= 1'b1;
                                state         <= ST_RESP;
                            end else begin
                                burst_remain <= burst_remain - 1;
                            end
                        end
                    end else begin
                        m_axi_wvalid <= 1'b0;
                    end
                end

                // -----------------------------------------
                ST_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        done_id      <= m_axi_bid;

                        if (m_axi_bresp != `AXI_RESP_OKAY &&
                            m_axi_bresp != `AXI_RESP_EXOKAY) begin
                            err_resp <= 1'b1;
                            err_id   <= m_axi_bid;
                        end else if (bytes_remain == 0) begin
                            done  <= 1'b1;
                        end
                        state <= ST_ADDR;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
