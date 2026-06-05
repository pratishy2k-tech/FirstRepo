// axi_read_master.v
// Pipelined AXI4 Read Master — issues burst transactions
// Supports up to DMA_BURST_LEN beats per transaction

`include "dma_pkg.vh"

module axi_read_master #(
    parameter ADDR_WIDTH  = `DMA_ADDR_WIDTH,
    parameter DATA_WIDTH  = `DMA_DATA_WIDTH,
    parameter ID_WIDTH    = `DMA_ID_WIDTH,
    parameter MAX_BURST   = `DMA_BURST_LEN
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // --- Command Interface (from DMA engine) ---
    input  wire                    cmd_valid,
    output wire                    cmd_ready,
    input  wire [ADDR_WIDTH-1:0]   cmd_addr,
    input  wire [31:0]             cmd_byte_len,
    input  wire [ID_WIDTH-1:0]     cmd_id,
    input  wire [3:0]              cmd_burst_size,  // AxSIZE
    input  wire [7:0]              cmd_cache,
    input  wire [2:0]              cmd_prot,

    // --- Data Output Interface ---
    output reg                     data_valid,
    input  wire                    data_ready,
    output reg  [DATA_WIDTH-1:0]   data,
    output reg  [DATA_WIDTH/8-1:0] data_keep,
    output reg                     data_last,
    output reg  [ID_WIDTH-1:0]     data_id,

    // --- Status ---
    output reg                     err_resp,        // AXI SLVERR/DECERR
    output reg  [ID_WIDTH-1:0]     err_id,

    // === AXI4 AR Channel ===
    output reg  [ID_WIDTH-1:0]     m_axi_arid,
    output reg  [ADDR_WIDTH-1:0]   m_axi_araddr,
    output reg  [7:0]              m_axi_arlen,
    output reg  [2:0]              m_axi_arsize,
    output reg  [1:0]              m_axi_arburst,
    output reg                     m_axi_arlock,
    output reg  [3:0]              m_axi_arcache,
    output reg  [2:0]              m_axi_arprot,
    output reg  [3:0]              m_axi_arqos,
    output reg                     m_axi_arvalid,
    input  wire                    m_axi_arready,

    // === AXI4 R Channel ===
    input  wire [ID_WIDTH-1:0]     m_axi_rid,
    input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output reg                     m_axi_rready
);

    // -------------------------------------------------------
    // Internal state
    // -------------------------------------------------------
    localparam ST_IDLE    = 3'd0;
    localparam ST_ADDR    = 3'd1;
    localparam ST_DATA    = 3'd2;
    localparam ST_FINISH  = 3'd3;

    reg [2:0]              state;
    reg [ADDR_WIDTH-1:0]   cur_addr;
    reg [31:0]             bytes_remain;
    reg [ID_WIDTH-1:0]     cur_id;
    reg [3:0]              cur_burst_size;
    reg [7:0]              cur_cache;
    reg [2:0]              cur_prot;
    reg [7:0]              burst_beats;     // beats in current burst
    reg [31:0]             bytes_per_beat;  // 1 << burst_size

    // cmd_ready: accept new command only when idle
    assign cmd_ready = (state == ST_IDLE);

    // -------------------------------------------------------
    // Byte count per beat from AxSIZE
    // -------------------------------------------------------
    function [31:0] size_to_bytes;
        input [3:0] sz;
        begin
            size_to_bytes = 32'd1 << sz;
        end
    endfunction

    // -------------------------------------------------------
    // Burst length computation
    //   beats = min(bytes_remain / bytes_per_beat, MAX_BURST)
    //   Capped to 256 (AXI4 limit: arlen = beats-1, max 255)
    // -------------------------------------------------------
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

    // -------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            data_valid    <= 1'b0;
            err_resp      <= 1'b0;
            bytes_remain  <= 32'd0;
        end else begin
            // Default de-asserts
            err_resp <= 1'b0;

            case (state)
                // -----------------------------------------
                ST_IDLE: begin
                    m_axi_arvalid <= 1'b0;
                    data_valid    <= 1'b0;
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
                        burst_beats   <= calc_burst_len(bytes_remain,
                                                        bytes_per_beat,
                                                        MAX_BURST) - 1;
                        m_axi_arid    <= cur_id;
                        m_axi_araddr  <= cur_addr;
                        m_axi_arlen   <= calc_burst_len(bytes_remain,
                                                        bytes_per_beat,
                                                        MAX_BURST) - 1;
                        m_axi_arsize  <= cur_burst_size[2:0];
                        m_axi_arburst <= `AXI_BURST_INCR;
                        m_axi_arlock  <= 1'b0;
                        m_axi_arcache <= cur_cache[3:0];
                        m_axi_arprot  <= cur_prot;
                        m_axi_arqos   <= 4'd0;
                        m_axi_arvalid <= 1'b1;
                        m_axi_rready  <= 1'b1;

                        if (m_axi_arvalid && m_axi_arready) begin
                            m_axi_arvalid <= 1'b0;
                            state         <= ST_DATA;
                        end
                    end
                end

                // -----------------------------------------
                ST_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        data_valid <= 1'b1;
                        data       <= m_axi_rdata;
                        data_keep  <= {(DATA_WIDTH/8){1'b1}};
                        data_last  <= m_axi_rlast;
                        data_id    <= m_axi_rid;

                        if (m_axi_rresp != `AXI_RESP_OKAY &&
                            m_axi_rresp != `AXI_RESP_EXOKAY) begin
                            err_resp <= 1'b1;
                            err_id   <= m_axi_rid;
                        end

                        // Advance byte count
                        if (bytes_remain >= bytes_per_beat)
                            bytes_remain <= bytes_remain - bytes_per_beat;
                        else
                            bytes_remain <= 32'd0;

                        if (m_axi_rlast) begin
                            // Burst done
                            cur_addr  <= cur_addr + (bytes_per_beat *
                                         (burst_beats + 1));
                            m_axi_rready <= 1'b0;
                            state        <= ST_ADDR;
                        end
                    end else begin
                        data_valid <= 1'b0;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
