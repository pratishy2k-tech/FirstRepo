// desc_sram.v
// Dual-port synchronous SRAM for descriptor storage
// 16 entries x 256 bits

`include "dma_pkg.vh"

module desc_sram #(
    parameter DEPTH     = `DMA_DESC_DEPTH,
    parameter ADDR_BITS = `DMA_DESC_ADDR_BITS
)(
    input  wire                  clk,

    // Write Port (status write-back from HW, or SW init)
    input  wire                  wr_en,
    input  wire [ADDR_BITS-1:0]  wr_addr,
    input  wire [255:0]          wr_data,
    input  wire [31:0]           wr_mask,      // Which status bits to update

    // Read Port (descriptor fetch)
    input  wire                  rd_en,
    input  wire [ADDR_BITS-1:0]  rd_addr,
    output reg  [255:0]          rd_data,
    output reg                   rd_valid,

    // SW init port (load full descriptors)
    input  wire                  sw_wr_en,
    input  wire [ADDR_BITS-1:0]  sw_wr_addr,
    input  wire [255:0]          sw_wr_data
);

    reg [255:0] mem [0:DEPTH-1];
    integer i;

    // Initialize to zero
    initial begin
        for (i = 0; i < DEPTH; i = i + 1)
            mem[i] = 256'd0;
    end

    // --- Write: HW updates only status field [255:224] ---
    always @(posedge clk) begin
        if (wr_en) begin
            // Merge status bits using mask
            mem[wr_addr][`DESC_STATUS_HI:`DESC_STATUS_LO] <=
                (mem[wr_addr][`DESC_STATUS_HI:`DESC_STATUS_LO] & ~wr_mask) |
                (wr_data[`DESC_STATUS_HI:`DESC_STATUS_LO]      &  wr_mask);
        end
        // SW writes full descriptor
        if (sw_wr_en) begin
            mem[sw_wr_addr] <= sw_wr_data;
        end
    end

    // --- Read: synchronous, 1-cycle latency ---
    always @(posedge clk) begin
        rd_valid <= 1'b0;
        if (rd_en) begin
            rd_data  <= mem[rd_addr];
            rd_valid <= 1'b1;
        end
    end

endmodule
