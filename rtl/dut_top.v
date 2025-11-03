module dut_top (
    input  wire        clk,          // byte clock
    input  wire        resetn,

    // From cocotb driver
    input  wire [7:0]  lane0_byte,
    input  wire        lane0_valid,
    input  wire [7:0]  lane1_byte,
    input  wire        lane1_valid,
    input  wire        txrequest_hs,
    input  wire        txwrite_hs,
    input  wire        sop,
    input  wire        eop,

    // To cocotb monitor and checks
    output wire        txready_hs,
    output wire        TxDDRClkHS,
    output wire [1:0]  D0,
    output wire [1:0]  D1
);

    // Direct pass-through (dummy_csi_tx can be omitted if cocotb drives signals)
    // dphy modeling
    dphy_model #(
        .SETTLE_CYC(6),
        .TRAIL_CYC(2),
        .EXIT_CYC(4)
    ) u_dphy (
        .byteclk      (clk),
        .resetn       (resetn),
        .txrequest_hs (txrequest_hs),
        .txwrite_hs   (txwrite_hs),
        .lane0_byte   (lane0_byte),
        .lane0_valid  (lane0_valid),
        .lane1_byte   (lane1_byte),
        .lane1_valid  (lane1_valid),
        .txready_hs   (txready_hs),
        .TxDDRClkHS   (TxDDRClkHS),
        .D0           (D0),
        .D1           (D1),
        .hs_active    (),   // unused external
        .hs_clk_active()    // unused external
    );

endmodule
