//==============================================================================
// two_body_core
//
// Timing summary:
//   c0      : capture b1/b2/m2 inputs
//   c0->c2  : dx, dy
//   c2->c3  : dx2, dy2
//   c3->c5  : r2
//   c5->c7  : r2 + eps^2
//   c7->c11 : invsqrt raw
//   c11->c12: register s
//   c12->c13: s2, m2*s
//   c13->c14: k = m2*s^3
//   c14->c15: term = dx*k, dy*k (mul comb + REG)
//   c15->c17: out = prev + term_reg (FpAdd = 2)
//
// prev must be driven from outside for the c15 sampling edge.
//==============================================================================

module two_body_core #(
    parameter int DATA_W = 27
) (
    input  logic        i_clk,
    input  logic        i_rst,   // active-low synchronous reset

    // 27-bit S1E8M18 inputs
    input  logic [DATA_W-1:0] i_b1_x,
    input  logic [DATA_W-1:0] i_b1_y,
    input  logic [DATA_W-1:0] i_b2_x,
    input  logic [DATA_W-1:0] i_b2_y,
    input  logic [DATA_W-1:0] i_m_b2,

    // 27-bit previous accel
    input  logic [DATA_W-1:0] i_a_b1_x,
    input  logic [DATA_W-1:0] i_a_b1_y,

    // 27-bit output accel
    output logic [DATA_W-1:0] o_a_b1_x,
    output logic [DATA_W-1:0] o_a_b1_y
);

    // eps^2 = 0.25
    localparam logic [26:0] EPSILON_SQUARE = {1'b0, 8'd125, 18'd0};

    // Body inputs
    logic [26:0] b1_x_27;
    logic [26:0] b1_y_27;
    logic [26:0] b2_x_27;
    logic [26:0] b2_y_27;
    logic [26:0] m2_27;

    assign b1_x_27 = i_b1_x;
    assign b1_y_27 = i_b1_y;
    assign b2_x_27 = i_b2_x;
    assign b2_y_27 = i_b2_y;
    assign m2_27   = i_m_b2;

    // c0: register core-format inputs
    logic [26:0] r_b1_x_c0;
    logic [26:0] r_b1_y_c0;
    logic [26:0] r_b2_x_c0;
    logic [26:0] r_b2_y_c0;
    logic [26:0] r_m2_c0;

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            r_b1_x_c0 <= 27'd0;
            r_b1_y_c0 <= 27'd0;
            r_b2_x_c0 <= 27'd0;
            r_b2_y_c0 <= 27'd0;
            r_m2_c0   <= 27'd0;
        end else begin
            r_b1_x_c0 <= b1_x_27;
            r_b1_y_c0 <= b1_y_27;
            r_b2_x_c0 <= b2_x_27;
            r_b2_y_c0 <= b2_y_27;
            r_m2_c0   <= m2_27;
        end
    end

    // Reset-mux for fplib blocks (they have no reset)
    logic en;
    logic [26:0] b2x_in;
    logic [26:0] b2y_in;

    assign en     = i_rst;
    assign b2x_in = en ? r_b2_x_c0 : 27'd0;
    assign b2y_in = en ? r_b2_y_c0 : 27'd0;

    // c0 comb: negate b1 so dx = x2 + (-x1), dy = y2 + (-y1)
    logic [26:0] w_b1_x_neg;
    logic [26:0] w_b1_y_neg;

    FpNegate u_neg_b1x (
        .iA       (en ? r_b1_x_c0 : 27'd0),
        .oNegative(w_b1_x_neg)
    );

    FpNegate u_neg_b1y (
        .iA       (en ? r_b1_y_c0 : 27'd0),
        .oNegative(w_b1_y_neg)
    );

    // c0 -> c2: dx, dy (FpAdd = 2 cycles)
    logic [26:0] w_dx_c2;
    logic [26:0] w_dy_c2;

    FpAdd u_add_dx (
        .iCLK(i_clk),
        .iA  (b2x_in),
        .iB  (w_b1_x_neg),
        .oSum(w_dx_c2)
    );

    FpAdd u_add_dy (
        .iCLK(i_clk),
        .iA  (b2y_in),
        .iB  (w_b1_y_neg),
        .oSum(w_dy_c2)
    );

    // c2 -> c3: dx2, dy2 (mul comb + reg = 1 cycle)
    logic [26:0] w_dx2_comb;
    logic [26:0] w_dy2_comb;
    logic [26:0] r_dx2_c3;
    logic [26:0] r_dy2_c3;

    FpMul u_mul_dx2 (
        .iA   (w_dx_c2),
        .iB   (w_dx_c2),
        .oProd(w_dx2_comb)
    );

    FpMul u_mul_dy2 (
        .iA   (w_dy_c2),
        .iB   (w_dy_c2),
        .oProd(w_dy2_comb)
    );

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            r_dx2_c3 <= 27'd0;
            r_dy2_c3 <= 27'd0;
        end else begin
            r_dx2_c3 <= w_dx2_comb;
            r_dy2_c3 <= w_dy2_comb;
        end
    end

    // c3 -> c5: r2 = dx2 + dy2 (FpAdd = 2 cycles)
    logic [26:0] w_r2_c5;

    FpAdd u_add_r2 (
        .iCLK(i_clk),
        .iA  (en ? r_dx2_c3 : 27'd0),
        .iB  (en ? r_dy2_c3 : 27'd0),
        .oSum(w_r2_c5)
    );

    // c5 -> c7: r2e = r2 + eps2 (FpAdd = 2 cycles)
    logic [26:0] w_r2e_c7;

    FpAdd u_add_r2e (
        .iCLK(i_clk),
        .iA  (en ? w_r2_c5 : 27'd0),
        .iB  (en ? EPSILON_SQUARE : 27'd0),
        .oSum(w_r2e_c7)
    );

    // c7 -> c11: s_raw = inv_sqrt(r2e)
    logic [26:0] w_s_c11;

    FpInvSqrt u_invsqrt (
        .iCLK    (i_clk),
        .iA      (en ? w_r2e_c7 : 27'd0),
        .oInvSqrt(w_s_c11)
    );

    // c11 -> c12: register s before mul usage
    logic [26:0] r_s_c12;

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            r_s_c12 <= 27'd0;
        end else begin
            r_s_c12 <= w_s_c11;
        end
    end

    // Align m2 to c12
    logic [26:0] r_m2_pipe [12];

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            for (int i = 0; i < 12; i++) begin
                r_m2_pipe[i] <= 27'd0;
            end
        end else begin
            r_m2_pipe[0] <= r_m2_c0;
            for (int i = 0; i < 11; i++) begin
                r_m2_pipe[i+1] <= r_m2_pipe[i];
            end
        end
    end

    logic [26:0] m2_c12;
    assign m2_c12 = r_m2_pipe[11];

    // c12 -> c13: s2 = s*s, t = m2*s
    logic [26:0] w_s2_comb;
    logic [26:0] w_t_comb;
    logic [26:0] r_s2_c13;
    logic [26:0] r_t_c13;

    FpMul u_mul_s2 (
        .iA   (r_s_c12),
        .iB   (r_s_c12),
        .oProd(w_s2_comb)
    );

    FpMul u_mul_t (
        .iA   (m2_c12),
        .iB   (r_s_c12),
        .oProd(w_t_comb)
    );

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            r_s2_c13 <= 27'd0;
            r_t_c13  <= 27'd0;
        end else begin
            r_s2_c13 <= w_s2_comb;
            r_t_c13  <= w_t_comb;
        end
    end

    // c13 -> c14: k = m2*s^3
    logic [26:0] w_k_comb;
    logic [26:0] r_k_c14;

    FpMul u_mul_k (
        .iA   (r_t_c13),
        .iB   (r_s2_c13),
        .oProd(w_k_comb)
    );

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            r_k_c14 <= 27'd0;
        end else begin
            r_k_c14 <= w_k_comb;
        end
    end

    // Align dx/dy to c14
    logic [26:0] r_dx_pipe [12];
    logic [26:0] r_dy_pipe [12];

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            for (int i = 0; i < 12; i++) begin
                r_dx_pipe[i] <= 27'd0;
                r_dy_pipe[i] <= 27'd0;
            end
        end else begin
            r_dx_pipe[0] <= w_dx_c2;
            r_dy_pipe[0] <= w_dy_c2;
            for (int i = 0; i < 11; i++) begin
                r_dx_pipe[i+1] <= r_dx_pipe[i];
                r_dy_pipe[i+1] <= r_dy_pipe[i];
            end
        end
    end

    logic [26:0] dx_c14;
    logic [26:0] dy_c14;

    assign dx_c14 = r_dx_pipe[11];
    assign dy_c14 = r_dy_pipe[11];

    // c14 -> c15: pair term = dx*k, dy*k (mul comb + reg)
    logic [26:0] w_ax_term_c15;
    logic [26:0] w_ay_term_c15;
    logic [26:0] r_ax_term_c15;
    logic [26:0] r_ay_term_c15;

    FpMul u_mul_ax (
        .iA   (dx_c14),
        .iB   (r_k_c14),
        .oProd(w_ax_term_c15)
    );

    FpMul u_mul_ay (
        .iA   (dy_c14),
        .iB   (r_k_c14),
        .oProd(w_ay_term_c15)
    );

    always_ff @(posedge i_clk) begin
        if (!i_rst) begin
            r_ax_term_c15 <= 27'd0;
            r_ay_term_c15 <= 27'd0;
        end else begin
            r_ax_term_c15 <= w_ax_term_c15;
            r_ay_term_c15 <= w_ay_term_c15;
        end
    end

    // c15 -> c17: out = prev + term_reg (FpAdd = 2 cycles)
    logic [26:0] w_out_x_c17;
    logic [26:0] w_out_y_c17;

    FpAdd u_add_outx (
        .iCLK(i_clk),
        .iA  (en ? i_a_b1_x : 27'd0),
        .iB  (en ? r_ax_term_c15 : 27'd0),
        .oSum(w_out_x_c17)
    );

    FpAdd u_add_outy (
        .iCLK(i_clk),
        .iA  (en ? i_a_b1_y : 27'd0),
        .iB  (en ? r_ay_term_c15 : 27'd0),
        .oSum(w_out_y_c17)
    );

    assign o_a_b1_x = w_out_x_c17;
    assign o_a_b1_y = w_out_y_c17;

endmodule
