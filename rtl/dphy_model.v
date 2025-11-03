`timescale 1ns/1ps

module dphy_model #(
    parameter integer SETTLE_CYC = 6,  // HS-SETTLE in byteclk cycles
    parameter integer TRAIL_CYC  = 2,  // HS-TRAIL in byteclk cycles
    parameter integer EXIT_CYC   = 4   // HS-EXIT in byteclk cycles
)(
    input  wire        byteclk,
    input  wire        resetn,

    // From CSI-2 controller
    input  wire        txrequest_hs,
    input  wire        txwrite_hs,
    input  wire [7:0]  lane0_byte,
    input  wire        lane0_valid,
    input  wire [7:0]  lane1_byte,
    input  wire        lane1_valid,

    // Back to controller
    output reg         txready_hs,

    // PHY observable pins (abstracted)
    output reg         TxDDRClkHS,     // HS clock toggles in SETTLE/HS/TRAIL
    output reg  [1:0]  D0,             // packed DDR proxy for lane0
    output reg  [1:0]  D1,             // packed DDR proxy for lane1
    output reg         hs_active,      // in HS data state
    output reg         hs_clk_active   // HS clock running (gate for TxDDRClkHS)
);

    // State machine
    typedef enum reg [1:0] {LP=2'b00, SETTLE=2'b01, HS=2'b10, TRAIL=2'b11} hs_state_t;
    hs_state_t state, next;

    integer settle_cnt;
    integer trail_cnt;
    integer exit_cnt;

    // HS clock generation: toggle on byteclk when active
    always @(posedge byteclk or negedge resetn) begin
        if (!resetn) begin
                      TxDDRClkHS    <= 1'b0;
            hs_clk_active <= 1'b0;
        end else begin
            // Gate HS clock activity by state
            if (state==SETTLE || state==HS || state==TRAIL)
                hs_clk_active <= 1'b1;
            else
                hs_clk_active <= 1'b0;

            if (hs_clk_active)
                TxDDRClkHS <= ~TxDDRClkHS;
            else
                TxDDRClkHS <= 1'b0;
        end
    end

    // Sequential state/control
    always @(posedge byteclk or negedge resetn) begin
        if (!resetn) begin
            state       <= LP;
            settle_cnt  <= 0;
            trail_cnt   <= 0;
            exit_cnt    <= 0;
            txready_hs  <= 1'b0;
            hs_active   <= 1'b0;
            D0          <= 2'b00;
            D1          <= 2'b00;
        end else begin
            state <= next;

            case (state)
              LP: begin
                    txready_hs <= 1'b0;
                    hs_active  <= 1'b0;
                    D0         <= 2'b00;
                    D1         <= 2'b00;
                    if (txrequest_hs)
                        settle_cnt <= SETTLE_CYC;
                  end
                            SETTLE: begin
                    hs_active  <= 1'b0;
                    D0         <= 2'b00;
                    D1         <= 2'b00;
                    if (settle_cnt > 0) settle_cnt <= settle_cnt - 1;
                    // Assert ready close to end of settle
                    if (settle_cnt == 1) txready_hs <= 1'b1;
                  end

              HS: begin
                    hs_active  <= 1'b1;
                    txready_hs <= 1'b1; // hold ready during HS
                    if (txwrite_hs) begin
                        D0 <= { (lane0_valid ? lane0_byte[1] : 1'b0),
                                (lane0_valid ? lane0_byte[0] : 1'b0) };
                        D1 <= { (lane1_valid ? lane1_byte[1] : 1'b0),
                                (lane1_valid ? lane1_byte[0] : 1'b0) };
                    end else begin
                        D0 <= 2'b00;
                        D1 <= 2'b00;
                    end
                  end

              TRAIL: begin
                    hs_active  <= 1'b0;
                    txready_hs <= 1'b0; // deassert during trail/exit
                    D0         <= 2'b00;
                    D1         <= 2'b00;
                    if (trail_cnt > 0) trail_cnt <= trail_cnt - 1;
                    if (exit_cnt  > 0) exit_cnt  <= exit_cnt  - 1;
                  end
            endcase
        end
    end

    // Next-state logic
    always @* begin
        next = state;
        case (state)
          LP:     if (txrequest_hs)     next = SETTLE;
                    SETTLE: if (settle_cnt == 0)  next = HS;
          HS:     if (!txrequest_hs)    next = TRAIL;
          TRAIL:  if (exit_cnt == 0)    next = LP;
          default:                      next = LP;
        endcase
    end

    // Arm TRAIL / EXIT counters when leaving HS (request goes low)
    always @(posedge byteclk or negedge resetn) begin
        if (!resetn) begin
            trail_cnt <= 0;
            exit_cnt  <= 0;
        end else if (state == HS && !txrequest_hs) begin
            trail_cnt <= TRAIL_CYC;
            exit_cnt  <= EXIT_CYC;
        end
    end

endmodule
