// =============================================================================
//  DMA Channel Engine
//  Implements descriptor fetch, AXI read, internal FIFO, AXI write
// =============================================================================

`timescale 1ns/1ps

module dma_channel #(
    parameter CHANNEL_ID    = 0,
    parameter ADDR_WIDTH    = 64,
    parameter DATA_WIDTH    = 64,
    parameter MAX_BURST_LEN = 16,
    parameter FIFO_DEPTH    = 64   // data FIFO depth in DATA_WIDTH beats
)(
    input  wire                    aclk,
    input  wire                    aresetn,

    // Control inputs from CSR
    input  wire                    ch_enable,
    input  wire                    ch_pause,
    input  wire                    ch_abort,
    input  wire [ADDR_WIDTH-1:0]   ch_desc_base,

    // Status outputs
    output reg                     ch_idle,
    output reg                     ch_busy,
    output reg                     ch_done,
    output reg                     ch_err,
    output reg  [2:0]              ch_err_code,
    output reg  [ADDR_WIDTH-1:0]   ch_curr_desc,
    output reg  [31:0]             ch_bytes_done,

    // Interrupt pulse outputs (single-cycle high)
    output reg                     irq_done,
    output reg                     irq_err,

    // AXI4 Master Read interface
    output reg  [ADDR_WIDTH-1:0]   m_axi_araddr,
    output reg  [7:0]              m_axi_arlen,
    output reg  [2:0]              m_axi_arsize,
    output reg  [1:0]              m_axi_arburst,
    output reg  [1:0]              m_axi_arprot,
    output reg                     m_axi_arvalid,
    input  wire                    m_axi_arready,
    input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output reg                     m_axi_rready,

    // AXI4 Master Write interface
    output reg  [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output reg  [7:0]              m_axi_awlen,
    output reg  [2:0]              m_axi_awsize,
    output reg  [1:0]              m_axi_awburst,
    output reg  [1:0]              m_axi_awprot,
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,
    output reg  [DATA_WIDTH-1:0]   m_axi_wdata,
    output reg  [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output reg                     m_axi_wlast,
    output reg                     m_axi_wvalid,
    input  wire                    m_axi_wready,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output reg                     m_axi_bready
);

    // =========================================================================
    // Descriptor fields (decoded after fetch)
    // =========================================================================
    reg [ADDR_WIDTH-1:0]  desc_src_addr;
    reg [ADDR_WIDTH-1:0]  desc_dst_addr;
    reg [31:0]            desc_length;
    reg [31:0]            desc_ctrl;
    reg [ADDR_WIDTH-1:0]  desc_next;

    wire desc_intr_on_cmp = desc_ctrl[0];
    wire desc_is_last     = desc_ctrl[1];
    wire [1:0] desc_src_prot = desc_ctrl[5:4];
    wire [1:0] desc_dst_prot = desc_ctrl[7:6];

    // =========================================================================
    // Internal FIFO (simple synchronous FIFO between read and write engines)
    // =========================================================================
    localparam FIFO_AW = $clog2(FIFO_DEPTH);

    reg [DATA_WIDTH-1:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [FIFO_AW:0]      fifo_wr_ptr, fifo_rd_ptr;
    wire                 fifo_empty = (fifo_wr_ptr == fifo_rd_ptr);
    wire                 fifo_full  = (fifo_wr_ptr[FIFO_AW] != fifo_rd_ptr[FIFO_AW]) &&
                                      (fifo_wr_ptr[FIFO_AW-1:0] == fifo_rd_ptr[FIFO_AW-1:0]);
    wire [FIFO_AW:0]     fifo_count = fifo_wr_ptr - fifo_rd_ptr;

    reg                  fifo_wr_en;
    reg [DATA_WIDTH-1:0] fifo_wr_data;
    reg                  fifo_rd_en;
    wire [DATA_WIDTH-1:0] fifo_rd_data = fifo_mem[fifo_rd_ptr[FIFO_AW-1:0]];

    always @(posedge aclk) begin
        if (fifo_wr_en && !fifo_full)
            fifo_mem[fifo_wr_ptr[FIFO_AW-1:0]] <= fifo_wr_data;
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            fifo_wr_ptr <= 0;
            fifo_rd_ptr <= 0;
        end else begin
            if (fifo_wr_en && !fifo_full)
                fifo_wr_ptr <= fifo_wr_ptr + 1;
            if (fifo_rd_en && !fifo_empty)
                fifo_rd_ptr <= fifo_rd_ptr + 1;
        end
    end

    // =========================================================================
    // Descriptor Fetch State Machine
    // Descriptors are fetched from memory using the AXI read channel.
    // A descriptor is 256 bits = 4 x 64-bit beats.
    // =========================================================================
    localparam  // Descriptor fetch states
        DF_IDLE        = 4'd0,
        DF_FETCH_ADDR  = 4'd1,
        DF_FETCH_WAIT  = 4'd2,
        DF_FETCH_DATA  = 4'd3,
        DF_DECODE      = 4'd4,
        DF_DONE        = 4'd5,
        DF_ERROR       = 4'd6;

    reg [3:0]  df_state;
    reg [1:0]  df_beat_cnt;  // 0..3 (4 beats per descriptor)
    reg [255:0] desc_raw;    // raw descriptor data

    // =========================================================================
    // Read Engine State Machine
    // =========================================================================
    localparam
        RD_IDLE   = 3'd0,
        RD_ADDR   = 3'd1,
        RD_DATA   = 3'd2,
        RD_BURST  = 3'd3,
        RD_DONE   = 3'd4;

    reg [2:0]  rd_state;
    reg [ADDR_WIDTH-1:0] rd_addr_cur;
    reg [31:0] rd_bytes_rem;
    reg [7:0]  rd_burst_beats_rem;
    reg        rd_active;  // descriptor loaded, reading in progress

    // =========================================================================
    // Write Engine State Machine
    // =========================================================================
    localparam
        WR_IDLE   = 3'd0,
        WR_ADDR   = 3'd1,
        WR_DATA   = 3'd2,
        WR_RESP   = 3'd3,
        WR_DONE   = 3'd4;

    reg [2:0]  wr_state;
    reg [ADDR_WIDTH-1:0] wr_addr_cur;
    reg [31:0] wr_bytes_rem;
    reg [7:0]  wr_burst_beats_rem;
    reg [7:0]  wr_burst_len_lat;

    // Transfer bytes
    localparam BEAT_BYTES = DATA_WIDTH / 8;  // bytes per AXI beat

    // =========================================================================
    // Top-level channel FSM
    // =========================================================================
    localparam
        CH_IDLE    = 3'd0,
        CH_DESC    = 3'd1,  // fetch descriptor
        CH_RUN     = 3'd2,  // run RD + WR engines
        CH_NEXT    = 3'd3,  // advance to next descriptor
        CH_DONE    = 3'd4,  // all descriptors processed
        CH_ERR     = 3'd5,  // error halt
        CH_ABORT   = 3'd6;

    reg [2:0]  ch_state;
    reg [2:0]  ch_err_code_r;
    reg        desc_phase_done;  // descriptor engine finished
    reg        xfer_done;        // read+write engines finished one descriptor

    // Timeout counter (watchdog)
    localparam TIMEOUT_VAL = 32'd100_000;
    reg [31:0] timeout_cnt;
    reg        timeout_armed;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            ch_state       <= CH_IDLE;
            ch_idle        <= 1'b1;
            ch_busy        <= 1'b0;
            ch_done        <= 1'b0;
            ch_err         <= 1'b0;
            ch_err_code    <= 3'h0;
            ch_curr_desc   <= '0;
            ch_bytes_done  <= 32'h0;
            irq_done       <= 1'b0;
            irq_err        <= 1'b0;
            df_state       <= DF_IDLE;
            rd_state       <= RD_IDLE;
            wr_state       <= WR_IDLE;
            rd_active      <= 1'b0;
            desc_phase_done<= 1'b0;
            xfer_done      <= 1'b0;
            timeout_cnt    <= 32'h0;
            timeout_armed  <= 1'b0;
            fifo_wr_ptr    <= 0;
            fifo_rd_ptr    <= 0;
        end else begin
            // Default pulse signals
            irq_done <= 1'b0;
            irq_err  <= 1'b0;

            case (ch_state)
                CH_IDLE: begin
                    ch_idle <= 1'b1;
                    ch_busy <= 1'b0;
                    ch_done <= 1'b0;
                    ch_err  <= 1'b0;
                    if (ch_enable && !ch_abort) begin
                        ch_curr_desc    <= ch_desc_base;
                        ch_bytes_done   <= 32'h0;
                        fifo_wr_ptr     <= 0;
                        fifo_rd_ptr     <= 0;
                        ch_state        <= CH_DESC;
                        ch_idle         <= 1'b0;
                        ch_busy         <= 1'b1;
                        df_state        <= DF_FETCH_ADDR;
                        desc_phase_done <= 1'b0;
                    end
                end

                CH_DESC: begin
                    // Run descriptor fetch FSM
                    desc_fetch_fsm();
                    if (desc_phase_done) begin
                        ch_state <= CH_RUN;
                        rd_state <= RD_ADDR;
                        wr_state <= WR_ADDR;
                        rd_addr_cur    <= desc_src_addr;
                        wr_addr_cur    <= desc_dst_addr;
                        rd_bytes_rem   <= desc_length;
                        wr_bytes_rem   <= desc_length;
                        timeout_cnt    <= 32'h0;
                        timeout_armed  <= 1'b1;
                        xfer_done      <= 1'b0;
                    end
                end

                CH_RUN: begin
                    if (ch_abort) begin
                        ch_state <= CH_ABORT;
                    end else if (ch_pause) begin
                        // Hold in RUN state, stall engines
                    end else begin
                        // Watchdog timeout
                        timeout_cnt <= timeout_cnt + 1;
                        if (timeout_cnt >= TIMEOUT_VAL) begin
                            ch_err_code_r <= 3'h3;  // timeout
                            ch_state      <= CH_ERR;
                        end else begin
                            read_engine_fsm();
                            write_engine_fsm();
                            if (xfer_done) begin
                                ch_bytes_done  <= desc_length;
                                timeout_armed  <= 1'b0;
                                ch_state       <= CH_NEXT;
                            end
                        end
                    end
                end

                CH_NEXT: begin
                    if (desc_intr_on_cmp) irq_done <= 1'b1;
                    if (desc_is_last) begin
                        ch_state <= CH_DONE;
                    end else begin
                        ch_curr_desc    <= desc_next;
                        fifo_wr_ptr     <= 0;
                        fifo_rd_ptr     <= 0;
                        desc_phase_done <= 1'b0;
                        df_state        <= DF_FETCH_ADDR;
                        ch_state        <= CH_DESC;
                    end
                end

                CH_DONE: begin
                    ch_busy  <= 1'b0;
                    ch_idle  <= 1'b1;
                    ch_done  <= 1'b1;
                    irq_done <= 1'b1;
                    ch_state <= CH_IDLE;
                end

                CH_ERR: begin
                    ch_busy     <= 1'b0;
                    ch_err      <= 1'b1;
                    ch_err_code <= ch_err_code_r;
                    irq_err     <= 1'b1;
                    ch_state    <= CH_IDLE;
                end

                CH_ABORT: begin
                    // Drain and halt
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;
                    ch_busy       <= 1'b0;
                    ch_idle       <= 1'b1;
                    ch_state      <= CH_IDLE;
                end

                default: ch_state <= CH_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Descriptor Fetch FSM (Task)
    // Reads 4 consecutive 64-bit words from ch_curr_desc
    // =========================================================================
    task desc_fetch_fsm;
        begin
            case (df_state)
                DF_FETCH_ADDR: begin
                    m_axi_araddr  <= ch_curr_desc;
                    m_axi_arlen   <= 8'd3;  // 4 beats (256 bits)
                    m_axi_arsize  <= 3'b011; // 8 bytes
                    m_axi_arburst <= 2'b01;  // INCR
                    m_axi_arprot  <= 2'b00;
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready  <= 1'b0;
                    df_beat_cnt   <= 2'b00;
                    df_state      <= DF_FETCH_WAIT;
                end

                DF_FETCH_WAIT: begin
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        df_state      <= DF_FETCH_DATA;
                    end
                end

                DF_FETCH_DATA: begin
                    if (m_axi_rvalid) begin
                        // Check for AXI error on descriptor fetch
                        if (m_axi_rresp == 2'b10) begin
                            ch_err_code_r <= 3'h1; // SLVERR
                            m_axi_rready  <= 1'b0;
                            df_state      <= DF_ERROR;
                        end else if (m_axi_rresp == 2'b11) begin
                            ch_err_code_r <= 3'h2; // DECERR
                            m_axi_rready  <= 1'b0;
                            df_state      <= DF_ERROR;
                        end else begin
                            // Pack descriptor word
                            case (df_beat_cnt)
                                2'h0: desc_raw[63:0]    <= m_axi_rdata;
                                2'h1: desc_raw[127:64]  <= m_axi_rdata;
                                2'h2: desc_raw[191:128] <= m_axi_rdata;
                                2'h3: desc_raw[255:192] <= m_axi_rdata;
                            endcase
                            df_beat_cnt <= df_beat_cnt + 1;
                            if (m_axi_rlast) begin
                                m_axi_rready <= 1'b0;
                                df_state     <= DF_DECODE;
                            end
                        end
                    end
                end

                DF_DECODE: begin
                    desc_src_addr <= desc_raw[63:0];
                    desc_dst_addr <= desc_raw[127:64];
                    desc_length   <= desc_raw[159:128];
                    desc_ctrl     <= desc_raw[191:160];
                    desc_next     <= desc_raw[255:192];
                    desc_phase_done <= 1'b1;
                    df_state      <= DF_DONE;
                end

                DF_DONE: begin
                    // wait for parent to transition
                    desc_phase_done <= 1'b0;
                end

                DF_ERROR: begin
                    ch_state <= CH_ERR;
                end

                default: df_state <= DF_IDLE;
            endcase
        end
    endtask

    // =========================================================================
    // Read Engine FSM (Task)
    // Reads data from src_addr into internal FIFO
    // =========================================================================
    task read_engine_fsm;
        integer burst_bytes;
        integer burst_len;
        begin
            case (rd_state)
                RD_IDLE: ; // waiting

                RD_ADDR: begin
                    if (!fifo_full && rd_bytes_rem > 0) begin
                        // Calculate burst length
                        burst_bytes = (rd_bytes_rem >= MAX_BURST_LEN * BEAT_BYTES) ?
                                       MAX_BURST_LEN * BEAT_BYTES : rd_bytes_rem;
                        burst_len   = (burst_bytes + BEAT_BYTES - 1) / BEAT_BYTES;

                        m_axi_araddr  <= rd_addr_cur;
                        m_axi_arlen   <= burst_len - 1;
                        m_axi_arsize  <= $clog2(BEAT_BYTES);
                        m_axi_arburst <= 2'b01; // INCR
                        m_axi_arprot  <= desc_src_prot;
                        m_axi_arvalid <= 1'b1;
                        m_axi_rready  <= 1'b0;
                        rd_burst_beats_rem <= burst_len;
                        rd_state <= RD_BURST;
                    end
                end

                RD_BURST: begin
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        rd_state      <= RD_DATA;
                    end
                end

                RD_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        if (m_axi_rresp[1]) begin // SLVERR or DECERR
                            ch_err_code_r <= (m_axi_rresp == 2'b10) ? 3'h1 : 3'h2;
                            m_axi_rready  <= 1'b0;
                            rd_state      <= RD_IDLE;
                            ch_state      <= CH_ERR;
                        end else begin
                            // Write to FIFO
                            fifo_wr_en   <= !fifo_full;
                            fifo_wr_data <= m_axi_rdata;

                            rd_addr_cur        <= rd_addr_cur + BEAT_BYTES;
                            rd_bytes_rem       <= (rd_bytes_rem >= BEAT_BYTES) ?
                                                   rd_bytes_rem - BEAT_BYTES : 32'h0;
                            rd_burst_beats_rem <= rd_burst_beats_rem - 1;
                            timeout_cnt        <= 32'h0; // reset watchdog on activity

                            if (m_axi_rlast) begin
                                m_axi_rready <= 1'b0;
                                if (rd_bytes_rem <= BEAT_BYTES) begin
                                    rd_state <= RD_DONE;
                                end else begin
                                    rd_state <= RD_ADDR; // next burst
                                end
                            end
                        end
                    end else begin
                        fifo_wr_en <= 1'b0;
                    end
                end

                RD_DONE: begin
                    fifo_wr_en <= 1'b0;
                    // Signal write engine that all data is in FIFO
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    endtask

    // =========================================================================
    // Write Engine FSM (Task)
    // Writes data from internal FIFO to dst_addr
    // =========================================================================
    task write_engine_fsm;
        integer wr_burst_bytes;
        integer wr_burst_len;
        begin
            case (wr_state)
                WR_IDLE: ;

                WR_ADDR: begin
                    if (!fifo_empty && wr_bytes_rem > 0) begin
                        wr_burst_bytes = (wr_bytes_rem >= MAX_BURST_LEN * BEAT_BYTES) ?
                                          MAX_BURST_LEN * BEAT_BYTES : wr_bytes_rem;
                        wr_burst_len   = (wr_burst_bytes + BEAT_BYTES - 1) / BEAT_BYTES;
                        wr_burst_len_lat <= wr_burst_len;

                        m_axi_awaddr  <= wr_addr_cur;
                        m_axi_awlen   <= wr_burst_len - 1;
                        m_axi_awsize  <= $clog2(BEAT_BYTES);
                        m_axi_awburst <= 2'b01;
                        m_axi_awprot  <= desc_dst_prot;
                        m_axi_awvalid <= 1'b1;
                        wr_burst_beats_rem <= wr_burst_len;
                        wr_state <= WR_DATA;
                    end
                end

                WR_DATA: begin
                    if (m_axi_awready) m_axi_awvalid <= 1'b0;

                    if (!fifo_empty) begin
                        m_axi_wdata  <= fifo_rd_data;
                        m_axi_wstrb  <= {(DATA_WIDTH/8){1'b1}};
                        m_axi_wvalid <= 1'b1;
                        m_axi_wlast  <= (wr_burst_beats_rem == 1);

                        if (m_axi_wready && m_axi_wvalid) begin
                            fifo_rd_en         <= 1'b1;
                            wr_addr_cur        <= wr_addr_cur + BEAT_BYTES;
                            wr_bytes_rem       <= (wr_bytes_rem >= BEAT_BYTES) ?
                                                   wr_bytes_rem - BEAT_BYTES : 32'h0;
                            wr_burst_beats_rem <= wr_burst_beats_rem - 1;
                            timeout_cnt        <= 32'h0;

                            if (m_axi_wlast) begin
                                m_axi_wvalid <= 1'b0;
                                m_axi_wlast  <= 1'b0;
                                wr_state     <= WR_RESP;
                                m_axi_bready <= 1'b1;
                            end
                        end else begin
                            fifo_rd_en <= 1'b0;
                        end
                    end else begin
                        fifo_rd_en   <= 1'b0;
                        m_axi_wvalid <= 1'b0;
                    end
                end

                WR_RESP: begin
                    fifo_rd_en <= 1'b0;
                    if (m_axi_bvalid) begin
                        m_axi_bready <= 1'b0;
                        if (m_axi_bresp[1]) begin
                            ch_err_code_r <= (m_axi_bresp == 2'b10) ? 3'h1 : 3'h2;
                            wr_state  <= WR_IDLE;
                            ch_state  <= CH_ERR;
                        end else begin
                            if (wr_bytes_rem == 0) begin
                                wr_state <= WR_DONE;
                            end else begin
                                wr_state <= WR_ADDR;  // next burst
                            end
                        end
                    end
                end

                WR_DONE: begin
                    fifo_rd_en <= 1'b0;
                    if (rd_state == RD_DONE) begin
                        xfer_done <= 1'b1;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    endtask

endmodule
