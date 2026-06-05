// desc_ring_manager.v
// Per-channel descriptor ring buffer manager
// Manages head/tail pointers and descriptor state machine

`include "dma_pkg.vh"

module desc_ring_manager #(
    parameter DEPTH     = `DMA_DESC_DEPTH,
    parameter ADDR_BITS = `DMA_DESC_ADDR_BITS,
    parameter CH_ID     = 0
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- SW Interface (from CSR) ---
    input  wire        sw_tail_wr,           // SW advances tail
    input  wire [ADDR_BITS-1:0] sw_tail,    // New tail value
    input  wire        sw_head_wr,           // SW sets head (init)
    input  wire [ADDR_BITS-1:0] sw_head,

    // --- Descriptor Memory Interface ---
    // Write port: HW updates status field of completed desc
    output reg         desc_wr_en,
    output reg  [ADDR_BITS-1:0] desc_wr_addr,
    output reg  [31:0] desc_wr_status,

    // Read port: fetch descriptor fields
    output reg         desc_rd_en,
    output reg  [ADDR_BITS-1:0] desc_rd_addr,
    input  wire [255:0] desc_rd_data,        // Full 256-bit descriptor
    input  wire        desc_rd_valid,        // Data available next cycle

    // --- Engine Interface ---
    output reg         desc_avail,           // A descriptor is ready
    output reg  [63:0] desc_src_addr,
    output reg  [63:0] desc_dst_addr,
    output reg  [31:0] desc_byte_cnt,
    output reg  [31:0] desc_ctrl,
    output reg  [ADDR_BITS-1:0] desc_idx,   // Current descriptor index

    input  wire        desc_consume,         // Engine accepted descriptor
    input  wire        desc_done,            // Transfer complete
    input  wire [31:0] desc_done_status,     // Status word to write back
    input  wire        desc_error,           // Error occurred

    // --- Status outputs ---
    output wire [ADDR_BITS-1:0] head_ptr,
    output wire [ADDR_BITS-1:0] tail_ptr,
    output reg                  ring_full,
    output reg                  ring_empty
);

    // -------------------------------------------------------
    localparam ST_IDLE    = 3'd0;
    localparam ST_FETCH   = 3'd1;
    localparam ST_WAIT    = 3'd2;
    localparam ST_DECODE  = 3'd3;
    localparam ST_ACTIVE  = 3'd4;
    localparam ST_WB      = 3'd5;   // Write-back status

    reg [2:0]           state;
    reg [ADDR_BITS-1:0] head;
    reg [ADDR_BITS-1:0] tail;
    reg [255:0]         cur_desc;
    reg [ADDR_BITS-1:0] cur_idx;

    assign head_ptr = head;
    assign tail_ptr = tail;

    // Ring occupancy
    wire [ADDR_BITS:0] occupancy = (tail >= head) ?
                                   (tail - head) :
                                   (DEPTH - head + tail);

    always @(*) begin
        ring_full  = (occupancy == (DEPTH - 1));
        ring_empty = (head == tail);
    end

    // -------------------------------------------------------
    // Tail pointer update from SW
    // -------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head  <= {ADDR_BITS{1'b0}};
            tail  <= {ADDR_BITS{1'b0}};
            state <= ST_IDLE;
            desc_avail  <= 1'b0;
            desc_rd_en  <= 1'b0;
            desc_wr_en  <= 1'b0;
        end else begin
            // SW updates
            if (sw_tail_wr) tail <= sw_tail;
            if (sw_head_wr) head <= sw_head;

            // Default de-asserts
            desc_rd_en  <= 1'b0;
            desc_wr_en  <= 1'b0;
            desc_avail  <= 1'b0;

            case (state)
                // -------------------------------------------
                ST_IDLE: begin
                    if (!ring_empty) begin
                        // Fetch next descriptor
                        cur_idx     <= head;
                        desc_rd_en  <= 1'b1;
                        desc_rd_addr<= head;
                        state       <= ST_FETCH;
                    end
                end

                // -------------------------------------------
                ST_FETCH: begin
                    // Wait one cycle for SRAM read latency
                    state <= ST_WAIT;
                end

                ST_WAIT: begin
                    if (desc_rd_valid) begin
                        cur_desc <= desc_rd_data;
                        state    <= ST_DECODE;
                    end
                end

                // -------------------------------------------
                ST_DECODE: begin
                    // Check valid bit
                    if (!cur_desc[`DESC_CTRL_LO + `DESC_CTRL_VALID]) begin
                        // Descriptor not valid yet — re-fetch later
                        state <= ST_IDLE;
                    end else begin
                        // Check for zero-length error
                        if (cur_desc[`DESC_BYTECNT_HI:`DESC_BYTECNT_LO] == 0) begin
                            // Write back LEN_ERR status
                            desc_wr_en     <= 1'b1;
                            desc_wr_addr   <= cur_idx;
                            desc_wr_status <= 32'h10; // LEN_ERR
                            // Advance head
                            head  <= (head + 1) % DEPTH;
                            state <= ST_IDLE;
                        end else begin
                            // Expose descriptor to engine
                            desc_src_addr <= cur_desc[`DESC_SRC_ADDR_HI:
                                                       `DESC_SRC_ADDR_LO];
                            desc_dst_addr <= cur_desc[`DESC_DST_ADDR_HI:
                                                       `DESC_DST_ADDR_LO];
                            desc_byte_cnt <= cur_desc[`DESC_BYTECNT_HI:
                                                       `DESC_BYTECNT_LO];
                            desc_ctrl     <= cur_desc[`DESC_CTRL_HI:
                                                       `DESC_CTRL_LO];
                            desc_idx      <= cur_idx;
                            desc_avail    <= 1'b1;
                            state         <= ST_ACTIVE;
                        end
                    end
                end

                // -------------------------------------------
                ST_ACTIVE: begin
                    desc_avail <= 1'b1;  // hold valid
                    if (desc_consume) begin
                        desc_avail <= 1'b0;
                        // Wait for completion
                        state <= ST_WB;
                    end
                end

                // -------------------------------------------
                ST_WB: begin
                    if (desc_done || desc_error) begin
                        // Write back status to descriptor
                        desc_wr_en     <= 1'b1;
                        desc_wr_addr   <= cur_idx;
                        desc_wr_status <= desc_done_status;
                        // Advance head
                        head  <= (head + 1) % DEPTH;
                        // Check chain bit
                        if (cur_desc[`DESC_CTRL_LO + `DESC_CTRL_CHAIN]) begin
                            // Jump to next descriptor index
                            head  <= cur_desc[`DESC_NEXT_HI:`DESC_NEXT_LO];
                        end
                        state <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
