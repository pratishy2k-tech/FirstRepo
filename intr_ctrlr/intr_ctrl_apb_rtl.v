// ============================================================
// Interrupt Controller with APB Messaged Interrupt Support
//
// New Features over base design:
//   - APB Master interface for messaged interrupt delivery
//   - MICR register: selects line vs messaged interrupt mode
//   - MADDR_LO / MADDR_HI: configurable APB target address
//   - Messaged interrupt write data = ISR (interrupt status)
//   - APB FSM: IDLE -> SETUP -> ACCESS (standard APB)
//   - PSLVERR detection with error sticky bit in MICR
//   - line INT still driven in line mode (backward compatible)
// ============================================================
//Base Design
// ============================================================
// Interrupt Controller - RTL Verilog
// Features:
//   - 8 external interrupt sources (IRQ[7:0])
//   - Fixed-priority encoder (IRQ0 = highest priority)
//   - Interrupt Mask Register (IMR)
//   - Interrupt Pending Register (IPR)
//   - Interrupt Service Register (ISR)
//   - Edge / Level trigger mode per channel
//   - Software interrupt injection
//   - Global interrupt enable (GIE)
//   - CPU interrupt acknowledge
//   - 3-bit interrupt vector output
//   - Register-mapped read/write interface
// ============================================================

// ============================================================
// Interrupt Controller with APB Messaged Interrupt Support
//
// New Features over base design:
//   - APB Master interface for messaged interrupt delivery
//   - MICR register: selects line vs messaged interrupt mode
//   - MADDR_LO / MADDR_HI: configurable APB target address
//   - Messaged interrupt write data = ISR (interrupt status)
//   - APB FSM: IDLE -> SETUP -> ACCESS (standard APB)
//   - PSLVERR detection with error sticky bit in MICR
//   - line INT still driven in line mode (backward compatible)
// ============================================================

module interrupt_controller_apb (
    // --------------------------------------------------------
    // System Signals
    // --------------------------------------------------------
    input  wire        clk,
    input  wire        rst_n,

    // --------------------------------------------------------
    // External Interrupt Lines
    // --------------------------------------------------------
    input  wire [7:0]  irq,

    // --------------------------------------------------------
    // CPU Register Interface (existing)
    // --------------------------------------------------------
    input  wire        cpu_ack,
    input  wire        cpu_wr,
    input  wire        cpu_rd,
    input  wire [3:0]  cpu_addr,      // Extended to 4-bit for new regs
    input  wire [7:0]  cpu_wdata,
    output reg  [7:0]  cpu_rdata,

    // --------------------------------------------------------
    // Line Interrupt Output (legacy / line mode)
    // --------------------------------------------------------
    output wire        INT,
    output wire [2:0]  INT_VECT,
    output wire        INT_VALID,

    // --------------------------------------------------------
    // APB Master Interface (messaged interrupt)
    // --------------------------------------------------------
    output reg         PSEL,          // APB select
    output reg         PENABLE,       // APB enable
    output reg         PWRITE,        // Always 1 (write only)
    output reg [15:0]  PADDR,         // Configurable target address
    output reg [7:0]   PWDATA,        // ISR value written as message
    input  wire        PREADY,        // Slave ready
    input  wire        PSLVERR,       // Slave error

    // APB transaction status outputs
    output wire        apb_busy,      // High while APB txn in progress
    output wire        apb_err        // Sticky error flag (cleared by MICR write)
);

// ============================================================
// Register Map (4-bit address space)
// ============================================================
// 0x0  IMR      Interrupt Mask Register          (R/W)
// 0x1  IPR      Interrupt Pending Register        (R/C)
// 0x2  ISR      Interrupt Service Register        (R/C)
// 0x3  TRIG     Trigger Mode Register             (R/W)
// 0x4  SWIR     Software Interrupt Register       (R/W)
// 0x5  GCR      Global Control Register           (R/W) [0]=GIE
// 0x6  VECT     Current Interrupt Vector          (R)
// 0x7  STAT     Status Register                   (R)
// 0x8  MICR     Message Interrupt Control Reg     (R/W)
//                 [0]   = mode: 0=line, 1=messaged
//                 [1]   = apb_err sticky (W1C)
//                 [7:2] = reserved
// 0x9  MADDR_LO Message APB target address [7:0]  (R/W)
// 0xA  MADDR_HI Message APB target address [15:8] (R/W)

    localparam ADDR_IMR      = 4'h0;
    localparam ADDR_IPR      = 4'h1;
    localparam ADDR_ISR      = 4'h2;
    localparam ADDR_TRIG     = 4'h3;
    localparam ADDR_SWIR     = 4'h4;
    localparam ADDR_GCR      = 4'h5;
    localparam ADDR_VECT     = 4'h6;
    localparam ADDR_STAT     = 4'h7;
    localparam ADDR_MICR     = 4'h8;
    localparam ADDR_MADDR_LO = 4'h9;
    localparam ADDR_MADDR_HI = 4'hA;

// ============================================================
// APB FSM State Encoding
// ============================================================
    localparam APB_IDLE   = 2'b00;
    localparam APB_SETUP  = 2'b01;
    localparam APB_ACCESS = 2'b10;

// ============================================================
// Internal Registers
// ============================================================
    reg [7:0]  imr;           // Interrupt Mask Register
    reg [7:0]  ipr;           // Interrupt Pending Register
    reg [7:0]  isr;           // Interrupt Service Register
    reg [7:0]  trig_mode;     // Trigger: 1=edge, 0=level
    reg [7:0]  swir;          // Software Interrupt Register
    reg        gie;           // Global Interrupt Enable

    // New registers
    reg        msg_mode;      // 0=line interrupt, 1=messaged interrupt
    reg        apb_err_r;     // APB error sticky bit
    reg [7:0]  maddr_lo;      // APB message address [7:0]
    reg [7:0]  maddr_hi;      // APB message address [15:8]

    // APB FSM
    reg [1:0]  apb_state;
    reg [7:0]  apb_data_latch; // Latched ISR value at time of trigger

// ============================================================
// Edge Detection
// ============================================================
    reg  [7:0] irq_prev;
    wire [7:0] irq_posedge;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) irq_prev <= 8'h00;
        else        irq_prev <= irq;
    end

    assign irq_posedge = irq & ~irq_prev;

// ============================================================
// IRQ Source Combining & Masking
// ============================================================
    wire [7:0] irq_raw;
    wire [7:0] irq_active;

    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : irq_src
            assign irq_raw[gi] = (trig_mode[gi]) ? irq_posedge[gi] : irq[gi];
        end
    endgenerate

    assign irq_active = (irq_raw | swir) & ~imr;

// ============================================================
// Interrupt Pending Register (IPR)
// ============================================================
    wire [7:0] ipr_set   = irq_active;
    wire [7:0] ipr_clear;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ipr <= 8'h00;
        end else if (cpu_wr && cpu_addr == ADDR_IPR) begin
            ipr <= (ipr | ipr_set) & ~cpu_wdata;  // W1C
        end else begin
            ipr <= (ipr | ipr_set) & ~ipr_clear;
        end
    end

// ============================================================
// Priority Encoder (IPR -> highest pending IRQ)
// ============================================================
    reg [2:0] priority_vect;
    reg       priority_valid;

    always @(*) begin
        casez (ipr)
            8'b????_???1 : begin priority_vect = 3'd0; priority_valid = 1'b1; end
            8'b????_??10 : begin priority_vect = 3'd1; priority_valid = 1'b1; end
            8'b????_?100 : begin priority_vect = 3'd2; priority_valid = 1'b1; end
            8'b????_1000 : begin priority_vect = 3'd3; priority_valid = 1'b1; end
            8'b???1_0000 : begin priority_vect = 3'd4; priority_valid = 1'b1; end
            8'b??10_0000 : begin priority_vect = 3'd5; priority_valid = 1'b1; end
            8'b?100_0000 : begin priority_vect = 3'd6; priority_valid = 1'b1; end
            8'b1000_0000 : begin priority_vect = 3'd7; priority_valid = 1'b1; end
            default      : begin priority_vect = 3'd0; priority_valid = 1'b0; end
        endcase
    end

// ============================================================
// Interrupt Service Register (ISR)
// ============================================================
    wire isr_empty       = (isr == 8'h00);
    wire issue_new       = gie && priority_valid && isr_empty;
    wire [7:0] prio_hot  = 8'h01 << priority_vect;

    assign ipr_clear = issue_new ? prio_hot : 8'h00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            isr <= 8'h00;
        end else if (cpu_wr && cpu_addr == ADDR_ISR) begin
            isr <= isr & ~cpu_wdata;               // W1C by CPU
        end else if (cpu_ack) begin
            isr <= 8'h00;                          // CPU ACK clears ISR
        end else if (issue_new) begin
            isr <= prio_hot;                       // Load new interrupt
        end
    end

// ============================================================
// ISR -> Vector Encoder
// ============================================================
    reg [2:0] isr_vect;
    reg       isr_valid;

    always @(*) begin
        casez (isr)
            8'b????_???1 : begin isr_vect = 3'd0; isr_valid = 1'b1; end
            8'b????_??10 : begin isr_vect = 3'd1; isr_valid = 1'b1; end
            8'b????_?100 : begin isr_vect = 3'd2; isr_valid = 1'b1; end
            8'b????_1000 : begin isr_vect = 3'd3; isr_valid = 1'b1; end
            8'b???1_0000 : begin isr_vect = 3'd4; isr_valid = 1'b1; end
            8'b??10_0000 : begin isr_vect = 3'd5; isr_valid = 1'b1; end
            8'b?100_0000 : begin isr_vect = 3'd6; isr_valid = 1'b1; end
            8'b1000_0000 : begin isr_vect = 3'd7; isr_valid = 1'b1; end
            default      : begin isr_vect = 3'd0; isr_valid = 1'b0; end
        endcase
    end

// ============================================================
// Line Interrupt Outputs (active in line mode OR always valid)
// ============================================================
    // INT is only driven in line mode (msg_mode == 0)
    assign INT       = gie && isr_valid && !msg_mode;
    assign INT_VECT  = isr_vect;
    assign INT_VALID = isr_valid;

// ============================================================
// APB Messaged Interrupt FSM
// ============================================================
//
//  Trigger condition:
//    - msg_mode == 1
//    - isr_valid goes high (new interrupt loaded into ISR)
//    - APB FSM is currently IDLE
//
//  APB Write Sequence (standard 3-phase):
//    IDLE  : wait for trigger
//    SETUP : Assert PSEL, drive PADDR/PWDATA/PWRITE, PENABLE=0
//    ACCESS: Assert PENABLE, wait for PREADY
//            On PREADY -> go back to IDLE (or log error on PSLVERR)
//
//  PWDATA = ISR value (messaged interrupt carries status payload)

    // Detect rising edge of isr_valid as the APB trigger
    reg isr_valid_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) isr_valid_prev <= 1'b0;
        else        isr_valid_prev <= isr_valid;
    end

    wire apb_trigger = msg_mode && isr_valid && !isr_valid_prev
                       && (apb_state == APB_IDLE);

    // APB FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apb_state      <= APB_IDLE;
            PSEL           <= 1'b0;
            PENABLE        <= 1'b0;
            PWRITE         <= 1'b0;
            PADDR          <= 16'h0000;
            PWDATA         <= 8'h00;
            apb_data_latch <= 8'h00;
            apb_err_r      <= 1'b0;
        end else begin

            // ---- Sticky error clear by CPU W1C on MICR[1] ----
            if (cpu_wr && cpu_addr == ADDR_MICR && cpu_wdata[1])
                apb_err_r <= 1'b0;

            case (apb_state)

                // ------------------------------------------------
                // IDLE: wait for messaged interrupt trigger
                // ------------------------------------------------
                APB_IDLE: begin
                    PSEL    <= 1'b0;
                    PENABLE <= 1'b0;
                    PWRITE  <= 1'b0;

                    if (apb_trigger) begin
                        // Latch ISR value and target address
                        apb_data_latch <= isr;
                        apb_state      <= APB_SETUP;
                    end
                end

                // ------------------------------------------------
                // SETUP: Assert PSEL, present address/data
                //        PENABLE stays low for one cycle
                // ------------------------------------------------
                APB_SETUP: begin
                    PSEL    <= 1'b1;
                    PENABLE <= 1'b0;
                    PWRITE  <= 1'b1;
                    PADDR   <= {maddr_hi, maddr_lo};
                    PWDATA  <= apb_data_latch;  // ISR is the message payload
                    apb_state <= APB_ACCESS;
                end

                // ------------------------------------------------
                // ACCESS: Assert PENABLE, wait for PREADY
                // ------------------------------------------------
                APB_ACCESS: begin
                    PENABLE <= 1'b1;

                    if (PREADY) begin
                        // Transfer complete
                        PSEL      <= 1'b0;
                        PENABLE   <= 1'b0;
                        PWRITE    <= 1'b0;

                        if (PSLVERR) begin
                            apb_err_r <= 1'b1;   // Latch slave error
                        end

                        apb_state <= APB_IDLE;
                    end
                    // If not PREADY: hold all signals (APB wait state)
                end

                default: apb_state <= APB_IDLE;

            endcase
        end
    end

    assign apb_busy = (apb_state != APB_IDLE);
    assign apb_err  = apb_err_r;

// ============================================================
// CPU Register Write Interface
// ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imr       <= 8'hFF;   // All masked
            trig_mode <= 8'hFF;   // All edge
            swir      <= 8'h00;
            gie       <= 1'b0;
            msg_mode  <= 1'b0;    // Default: line interrupt
            maddr_lo  <= 8'h00;
            maddr_hi  <= 8'h00;
        end else if (cpu_wr) begin
            case (cpu_addr)
                ADDR_IMR      : imr      <= cpu_wdata;
                ADDR_TRIG     : trig_mode<= cpu_wdata;
                ADDR_SWIR     : swir     <= cpu_wdata;
                ADDR_GCR      : gie      <= cpu_wdata[0];
                ADDR_MICR     : begin
                                    msg_mode <= cpu_wdata[0];
                                    // apb_err_r W1C handled in FSM block
                                end
                ADDR_MADDR_LO : maddr_lo <= cpu_wdata;
                ADDR_MADDR_HI : maddr_hi <= cpu_wdata;
                default       : ;
            endcase
        end
    end

// ============================================================
// CPU Register Read Interface
// ============================================================
    always @(*) begin
        cpu_rdata = 8'h00;
        if (cpu_rd) begin
            case (cpu_addr)
                ADDR_IMR      : cpu_rdata = imr;
                ADDR_IPR      : cpu_rdata = ipr;
                ADDR_ISR      : cpu_rdata = isr;
                ADDR_TRIG     : cpu_rdata = trig_mode;
                ADDR_SWIR     : cpu_rdata = swir;
                ADDR_GCR      : cpu_rdata = {7'b0, gie};
                ADDR_VECT     : cpu_rdata = {5'b0, isr_vect};
                ADDR_STAT     : cpu_rdata = {6'b0, apb_busy, isr_valid};
                ADDR_MICR     : cpu_rdata = {6'b0, apb_err_r, msg_mode};
                ADDR_MADDR_LO : cpu_rdata = maddr_lo;
                ADDR_MADDR_HI : cpu_rdata = maddr_hi;
                default       : cpu_rdata = 8'h00;
            endcase
        end
    end

endmodule

