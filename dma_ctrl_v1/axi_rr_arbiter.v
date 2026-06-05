// =============================================================================
//  AXI4 Round-Robin Arbiter — Read Channels
//  Arbitrates N channels' AR requests onto a single master AR port.
//  Uses ID field to route R responses back to requesting channel.
// =============================================================================

`timescale 1ns/1ps

module axi_rr_arbiter #(
    parameter N  = 8,
    parameter AW = 64,
    parameter DW = 64
)(
    input  wire                 aclk,
    input  wire                 aresetn,

    // Slave-side (from channel engines)
    input  wire [N-1:0]         ch_arvalid,
    input  wire [N*AW-1:0]      ch_araddr,
    input  wire [N*8-1:0]       ch_arlen,
    input  wire [N*3-1:0]       ch_arsize,
    input  wire [N*2-1:0]       ch_arburst,
    input  wire [N*2-1:0]       ch_arprot,
    output reg  [N-1:0]         ch_arready,

    // Read data back to channels
    output reg  [N*DW-1:0]      ch_rdata,
    output reg  [N*2-1:0]       ch_rresp,
    output reg  [N-1:0]         ch_rlast,
    output reg  [N-1:0]         ch_rvalid,
    input  wire [N-1:0]         ch_rready,

    // Master-side (to memory/interconnect)
    output reg  [3:0]           m_axi_arid,
    output reg  [AW-1:0]        m_axi_araddr,
    output reg  [7:0]           m_axi_arlen,
    output reg  [2:0]           m_axi_arsize,
    output reg  [1:0]           m_axi_arburst,
    output reg  [1:0]           m_axi_arprot,
    output reg                  m_axi_arvalid,
    input  wire                 m_axi_arready,

    input  wire [3:0]           m_axi_rid,
    input  wire [DW-1:0]        m_axi_rdata,
    input  wire [1:0]           m_axi_rresp,
    input  wire                 m_axi_rlast,
    input  wire                 m_axi_rvalid,
    output reg                  m_axi_rready
);

    reg [$clog2(N)-1:0] rr_ptr;   // round-robin pointer
    reg                 busy;     // arbitration in progress
    reg [$clog2(N)-1:0] granted;  // currently granted channel

    integer k;

    // Round-robin grant logic
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rr_ptr        <= 0;
            busy          <= 1'b0;
            granted       <= 0;
            m_axi_arvalid <= 1'b0;
            ch_arready    <= '0;
            m_axi_rready  <= 1'b0;
        end else begin
            ch_arready    <= '0;
            m_axi_arvalid <= 1'b0;

            if (!busy) begin
                // Find next requesting channel starting from rr_ptr
                for (k = 0; k < N; k = k + 1) begin
                    if (ch_arvalid[(rr_ptr + k) % N] && !busy) begin
                        granted       <= (rr_ptr + k) % N;
                        busy          <= 1'b1;
                        m_axi_araddr  <= ch_araddr[((rr_ptr+k)%N)*AW +: AW];
                        m_axi_arlen   <= ch_arlen [((rr_ptr+k)%N)*8  +: 8 ];
                        m_axi_arsize  <= ch_arsize[((rr_ptr+k)%N)*3  +: 3 ];
                        m_axi_arburst <= ch_arburst[((rr_ptr+k)%N)*2 +: 2 ];
                        m_axi_arprot  <= ch_arprot [((rr_ptr+k)%N)*2 +: 2 ];
                        m_axi_arid    <= (rr_ptr + k) % N;
                        m_axi_arvalid <= 1'b1;
                        ch_arready[(rr_ptr+k)%N] <= 1'b1;
                    end
                end
            end else begin
                // Wait for AR handshake
                if (m_axi_arready && m_axi_arvalid) begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                end

                // Route R data back to granted channel
                if (m_axi_rvalid && m_axi_rready) begin
                    ch_rdata [granted*DW +: DW] <= m_axi_rdata;
                    ch_rresp [granted*2  +: 2 ] <= m_axi_rresp;
                    ch_rlast [granted]           <= m_axi_rlast;
                    ch_rvalid[granted]           <= 1'b1;

                    if (m_axi_rlast) begin
                        m_axi_rready <= 1'b0;
                        busy         <= 1'b0;
                        rr_ptr       <= (granted + 1) % N;
                    end
                end else begin
                    // Clear channel rvalid after handshake
                    for (k = 0; k < N; k = k + 1) begin
                        if (ch_rvalid[k] && ch_rready[k])
                            ch_rvalid[k] <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
