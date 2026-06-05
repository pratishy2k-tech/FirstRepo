// =============================================================================
//  AXI4 Round-Robin Arbiter — Write Channels (AW/W/B)
// =============================================================================

`timescale 1ns/1ps

module axi_wr_arbiter #(
    parameter N  = 8,
    parameter AW = 64,
    parameter DW = 64
)(
    input  wire                 aclk,
    input  wire                 aresetn,

    // From channel engines
    input  wire [N-1:0]         ch_awvalid,
    input  wire [N*AW-1:0]      ch_awaddr,
    input  wire [N*8-1:0]       ch_awlen,
    input  wire [N*3-1:0]       ch_awsize,
    input  wire [N*2-1:0]       ch_awburst,
    input  wire [N*2-1:0]       ch_awprot,
    output reg  [N-1:0]         ch_awready,

    input  wire [N*DW-1:0]      ch_wdata,
    input  wire [N*(DW/8)-1:0]  ch_wstrb,
    input  wire [N-1:0]         ch_wlast,
    input  wire [N-1:0]         ch_wvalid,
    output reg  [N-1:0]         ch_wready,

    output reg  [N*2-1:0]       ch_bresp,
    output reg  [N-1:0]         ch_bvalid,
    input  wire [N-1:0]         ch_bready,

    // AXI4 Master Write
    output reg  [3:0]           m_axi_awid,
    output reg  [AW-1:0]        m_axi_awaddr,
    output reg  [7:0]           m_axi_awlen,
    output reg  [2:0]           m_axi_awsize,
    output reg  [1:0]           m_axi_awburst,
    output reg  [1:0]           m_axi_awprot,
    output reg                  m_axi_awvalid,
    input  wire                 m_axi_awready,

    output reg  [DW-1:0]        m_axi_wdata,
    output reg  [DW/8-1:0]      m_axi_wstrb,
    output reg                  m_axi_wlast,
    output reg                  m_axi_wvalid,
    input  wire                 m_axi_wready,

    input  wire [3:0]           m_axi_bid,
    input  wire [1:0]           m_axi_bresp,
    input  wire                 m_axi_bvalid,
    output reg                  m_axi_bready
);

    reg [$clog2(N)-1:0] rr_ptr;
    reg [$clog2(N)-1:0] granted;
    reg                 busy;

    localparam
        ARB_IDLE = 2'd0,
        ARB_AW   = 2'd1,
        ARB_W    = 2'd2,
        ARB_B    = 2'd3;

    reg [1:0] arb_state;
    integer k;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rr_ptr        <= 0;
            granted       <= 0;
            busy          <= 1'b0;
            arb_state     <= ARB_IDLE;
            ch_awready    <= '0;
            ch_wready     <= '0;
            ch_bvalid     <= '0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_bready  <= 1'b0;
        end else begin
            ch_awready <= '0;
            ch_wready  <= '0;

            case (arb_state)
                ARB_IDLE: begin
                    for (k = 0; k < N; k = k + 1) begin
                        if (ch_awvalid[(rr_ptr+k)%N] && !busy) begin
                            granted <= (rr_ptr + k) % N;
                            busy    <= 1'b1;
                            m_axi_awaddr  <= ch_awaddr [((rr_ptr+k)%N)*AW    +: AW   ];
                            m_axi_awlen   <= ch_awlen  [((rr_ptr+k)%N)*8     +: 8    ];
                            m_axi_awsize  <= ch_awsize [((rr_ptr+k)%N)*3     +: 3    ];
                            m_axi_awburst <= ch_awburst[((rr_ptr+k)%N)*2     +: 2    ];
                            m_axi_awprot  <= ch_awprot [((rr_ptr+k)%N)*2     +: 2    ];
                            m_axi_awid    <= (rr_ptr + k) % N;
                            m_axi_awvalid <= 1'b1;
                            ch_awready[(rr_ptr+k)%N] <= 1'b1;
                            arb_state     <= ARB_AW;
                        end
                    end
                end

                ARB_AW: begin
                    if (m_axi_awready && m_axi_awvalid) begin
                        m_axi_awvalid <= 1'b0;
                        arb_state     <= ARB_W;
                    end
                end

                ARB_W: begin
                    // Forward write data from granted channel
                    if (ch_wvalid[granted]) begin
                        m_axi_wdata  <= ch_wdata[granted*DW     +: DW    ];
                        m_axi_wstrb  <= ch_wstrb[granted*(DW/8) +: (DW/8)];
                        m_axi_wlast  <= ch_wlast[granted];
                        m_axi_wvalid <= 1'b1;

                        if (m_axi_wready) begin
                            ch_wready[granted] <= 1'b1;
                            if (ch_wlast[granted]) begin
                                m_axi_wvalid <= 1'b0;
                                m_axi_bready <= 1'b1;
                                arb_state    <= ARB_B;
                            end
                        end else begin
                            ch_wready[granted] <= 1'b0;
                        end
                    end
                end

                ARB_B: begin
                    ch_wready <= '0;
                    if (m_axi_bvalid) begin
                        m_axi_bready      <= 1'b0;
                        ch_bresp[granted*2 +: 2] <= m_axi_bresp;
                        ch_bvalid[granted]       <= 1'b1;
                    end
                    if (ch_bvalid[granted] && ch_bready[granted]) begin
                        ch_bvalid[granted] <= 1'b0;
                        busy               <= 1'b0;
                        rr_ptr             <= (granted + 1) % N;
                        arb_state          <= ARB_IDLE;
                    end
                end
            endcase
        end
    end

endmodule
