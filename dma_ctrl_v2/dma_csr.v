// dma_csr.v
// APB3 slave providing register access to all 8 DMA channels

`include "dma_pkg.vh"

module dma_csr #(
    parameter NUM_CH   = `DMA_NUM_CHANNELS,
    parameter NUM_REGS = 256
)(
    input  wire        pclk,
    input  wire        presetn,

    // --- APB3 Slave Interface ---
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [9:0]  paddr,          // 10-bit = 1K byte range
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output reg         pready,
    output reg         pslverr,

    // --- Channel control outputs (to channels) ---
    output reg  [NUM_CH-1:0] ch_enable,
    output reg  [NUM_CH-1:0] ch_pause,

    // Per-channel descriptor ring control
    output reg  [NUM_CH-1:0] sw_tail_wr,
    output reg  [4*NUM_CH-1:0] sw_tail,
    output reg  [NUM_CH-1:0] sw_head_wr,
    output reg  [4*NUM_CH-1:0] sw_head,

    // Per-channel descriptor load
    output reg  [NUM_CH-1:0] sw_desc_wr,
    output reg  [4*NUM_CH-1:0] sw_desc_addr,
    output reg  [256*NUM_CH-1:0] sw_desc_data,

    // --- Channel status inputs ---
    input  wire [NUM_CH-1:0] ch_active,
    input  wire [NUM_CH-1:0] ch_done,
    input  wire [NUM_CH-1:0] ch_err,
    input  wire [5*NUM_CH-1:0] err_code,
    input  wire [4*NUM_CH-1:0] head_ptr,
    input  wire [4*NUM_CH-1:0] tail_ptr,
    input  wire [32*NUM_CH-1:0] xfer_count,

    // --- Interrupt output ---
    output reg  [`DMA_IRQ_WIDTH-1:0] irq_out,

    // --- Global ---
    output reg         dma_enable,
    output reg         dma_reset
);

    // -------------------------------------------------------
    // Register file (flat 256 x 32-bit)
    // -------------------------------------------------------
    reg [31:0] regs [0:NUM_REGS-1];
    integer i;

    initial begin
        for (i = 0; i < NUM_REGS; i = i + 1)
            regs[i] = 32'd0;
    end

    // APB decodes paddr[9:2] = word offset
    wire [7:0] waddr = paddr[9:2];

    // -------------------------------------------------------
    // APB write / read logic
    // -------------------------------------------------------
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            pready   <= 1'b1;
            pslverr  <= 1'b0;
            prdata   <= 32'd0;
            dma_enable <= 1'b0;
            dma_reset  <= 1'b0;
            ch_enable  <= 8'hFF;   // all channels enabled by default
            ch_pause   <= 8'h00;
            irq_out    <= {`DMA_IRQ_WIDTH{1'b0}};
            sw_tail_wr <= {NUM_CH{1'b0}};
            sw_head_wr <= {NUM_CH{1'b0}};
            sw_desc_wr <= {NUM_CH{1'b0}};
        end else begin
            pready   <= 1'b1;
            pslverr  <= 1'b0;
            sw_tail_wr <= {NUM_CH{1'b0}};
            sw_head_wr <= {NUM_CH{1'b0}};
            sw_desc_wr <= {NUM_CH{1'b0}};

            // ----- Update IRQ status from hardware events -----
            // Done interrupts
            irq_out[7:0]  <= irq_out[7:0] | ch_done;
            // Error interrupts
            irq_out[15:8] <= irq_out[15:8] | ch_err;

            // ----- APB Transaction -----
            if (psel && penable) begin
                if (pwrite) begin
                    // ------ Write ------
                    case (waddr)
                        8'h00: begin   // DMA_CTRL
                            dma_enable <= pwdata[0];
                            dma_reset  <= pwdata[1];
                            regs[8'h00] <= pwdata;
                        end
                        8'h01: begin   // CH_EN
                            ch_enable <= pwdata[7:0];
                            regs[8'h01] <= pwdata;
                        end
                        8'h02: begin   // IRQ_STATUS (W1C)
                            irq_out <= irq_out & ~pwdata[`DMA_IRQ_WIDTH-1:0];
                            regs[8'h02] <= regs[8'h02] & ~pwdata;
                        end
                        8'h03: begin   // IRQ_MASK
                            regs[8'h03] <= pwdata;
                        end
                        default: begin
                            // Per-channel registers
                            // CH_BASE(n) + offset
                            begin : ch_wr_blk
                                reg [3:0] ch_n;
                                reg [7:0] off;
                                reg found;
                                found = 1'b0;
                                for (ch_n = 0; ch_n < NUM_CH; ch_n = ch_n + 1) begin
                                    if (!found &&
                                        (waddr >= (8'h10 + ch_n * 8)) &&
                                        (waddr <  (8'h10 + ch_n * 8 + 8))) begin
                                        found = 1'b1;
                                        off = waddr - (8'h10 + ch_n * 8);
                                        case (off)
                                            8'h00: begin  // CH_CTRL
                                                ch_pause[ch_n] <= pwdata[1];
                                            end
                                            8'h02: begin  // CH_DESC_BASE
                                                regs[waddr] <= pwdata;
                                            end
                                            8'h03: begin  // CH_DESC_HEAD
                                                sw_head[ch_n*4+3:ch_n*4] <= pwdata[3:0];
                                                sw_head_wr[ch_n] <= 1'b1;
                                            end
                                            8'h04: begin  // CH_DESC_TAIL
                                                sw_tail[ch_n*4+3:ch_n*4] <= pwdata[3:0];
                                                sw_tail_wr[ch_n] <= 1'b1;
                                            end
                                            default: regs[waddr] <= pwdata;
                                        endcase
                                    end
                                end
                            end
                        end
                    endcase
                end else begin
                    // ------ Read ------
                    case (waddr)
                        8'h00: prdata <= {30'd0, dma_reset, dma_enable};
                        8'h01: prdata <= {24'd0, ch_enable};
                        8'h02: prdata <= {{(32-`DMA_IRQ_WIDTH){1'b0}}, irq_out};
                        8'h03: prdata <= regs[8'h03]; // IRQ_MASK
                        default: begin
                            begin : ch_rd_blk
                                reg [3:0] ch_n;
                                reg [7:0] off;
                                reg found;
                                found = 1'b0;
                                for (ch_n = 0; ch_n < NUM_CH; ch_n = ch_n + 1) begin
                                    if (!found &&
                                        (waddr >= (8'h10 + ch_n * 8)) &&
                                        (waddr <  (8'h10 + ch_n * 8 + 8))) begin
                                        found = 1'b1;
                                        off = waddr - (8'h10 + ch_n * 8);
                                        case (off)
                                            8'h00: prdata <= {29'd0,
                                                              ch_pause[ch_n],
                                                              ch_err[ch_n],
                                                              ch_active[ch_n]};
                                            8'h01: prdata <= {26'd0,
                                                              err_code[ch_n*5+4:ch_n*5],
                                                              ch_done[ch_n]};
                                            8'h03: prdata <= {28'd0,
                                                              head_ptr[ch_n*4+3:ch_n*4]};
                                            8'h04: prdata <= {28'd0,
                                                              tail_ptr[ch_n*4+3:ch_n*4]};
                                            8'h05: prdata <= xfer_count[ch_n*32+31:ch_n*32];
                                            8'h06: prdata <= {27'd0,
                                                              err_code[ch_n*5+4:ch_n*5]};
                                            default: prdata <= regs[waddr];
                                        endcase
                                    end
                                end
                                if (!found) prdata <= 32'hDEAD_C0DE;
                            end
                        end
                    endcase
                end
            end
        end
    end

endmodule
