module fourcore_bcj_datapath (
    input  logic        i_clk,
    input  logic        i_rst,

    // ============================================================
    // shared j position, per-lane j mass
    // ============================================================
    input  logic [15:0] i_j_x,
    input  logic [15:0] i_j_y,
    input  logic [15:0] i_j0_m,
    input  logic [15:0] i_j1_m,
    input  logic [15:0] i_j2_m,
    input  logic [15:0] i_j3_m,

    // ============================================================
    // lane 0
    // ============================================================
    input  logic [15:0] i_i0_x,
    input  logic [15:0] i_i0_y,
    input  logic [26:0] i_prev0_x,
    input  logic [26:0] i_prev0_y,
    output logic [26:0] o_out0_x,
    output logic [26:0] o_out0_y,

    // ============================================================
    // lane 1
    // ============================================================
    input  logic [15:0] i_i1_x,
    input  logic [15:0] i_i1_y,
    input  logic [26:0] i_prev1_x,
    input  logic [26:0] i_prev1_y,
    output logic [26:0] o_out1_x,
    output logic [26:0] o_out1_y,

    // ============================================================
    // lane 2
    // ============================================================
    input  logic [15:0] i_i2_x,
    input  logic [15:0] i_i2_y,
    input  logic [26:0] i_prev2_x,
    input  logic [26:0] i_prev2_y,
    output logic [26:0] o_out2_x,
    output logic [26:0] o_out2_y,

    // ============================================================
    // lane 3
    // ============================================================
    input  logic [15:0] i_i3_x,
    input  logic [15:0] i_i3_y,
    input  logic [26:0] i_prev3_x,
    input  logic [26:0] i_prev3_y,
    output logic [26:0] o_out3_x,
    output logic [26:0] o_out3_y
);

    two_body_core u_core0 (
        .i_clk   (i_clk),
        .i_rst   (i_rst),
        .i_b1_x  (i_i0_x),
        .i_b1_y  (i_i0_y),
        .i_b2_x  (i_j_x),
        .i_b2_y  (i_j_y),
        .i_m_b2  (i_j0_m),
        .i_a_b1_x(i_prev0_x),
        .i_a_b1_y(i_prev0_y),
        .o_a_b1_x(o_out0_x),
        .o_a_b1_y(o_out0_y)
    );

    two_body_core u_core1 (
        .i_clk   (i_clk),
        .i_rst   (i_rst),
        .i_b1_x  (i_i1_x),
        .i_b1_y  (i_i1_y),
        .i_b2_x  (i_j_x),
        .i_b2_y  (i_j_y),
        .i_m_b2  (i_j1_m),
        .i_a_b1_x(i_prev1_x),
        .i_a_b1_y(i_prev1_y),
        .o_a_b1_x(o_out1_x),
        .o_a_b1_y(o_out1_y)
    );

    two_body_core u_core2 (
        .i_clk   (i_clk),
        .i_rst   (i_rst),
        .i_b1_x  (i_i2_x),
        .i_b1_y  (i_i2_y),
        .i_b2_x  (i_j_x),
        .i_b2_y  (i_j_y),
        .i_m_b2  (i_j2_m),
        .i_a_b1_x(i_prev2_x),
        .i_a_b1_y(i_prev2_y),
        .o_a_b1_x(o_out2_x),
        .o_a_b1_y(o_out2_y)
    );

    two_body_core u_core3 (
        .i_clk   (i_clk),
        .i_rst   (i_rst),
        .i_b1_x  (i_i3_x),
        .i_b1_y  (i_i3_y),
        .i_b2_x  (i_j_x),
        .i_b2_y  (i_j_y),
        .i_m_b2  (i_j3_m),
        .i_a_b1_x(i_prev3_x),
        .i_a_b1_y(i_prev3_y),
        .o_a_b1_x(o_out3_x),
        .o_a_b1_y(o_out3_y)
    );

endmodule
