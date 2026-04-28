//==============================================================================
// two_body_core (late-prev version, FIXED)
//
// Goal of this version:
//   - b1/b2/m2 launch at c0
//   - datapath computes the pair term internally
//   - previous acceleration is NOT delayed inside the core
//   - external logic/TB supplies i_a_b1_x/y only when the pair term reaches
//     the final add input point
//   - because FpAdd is 2-stage, output appears exactly 2 cycles after prev
//     is sampled
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
// So: prev must be driven from outside for the c15 sampling edge.
//==============================================================================

module two_body_core
(
    input         i_clk,
    input         i_rst,   // active-low reset

    // 16-bit S1E8M7 inputs
    input  [15:0] i_b1_x,
    input  [15:0] i_b1_y,
    input  [15:0] i_b2_x,
    input  [15:0] i_b2_y,
    input  [15:0] i_m_b2,

    // 27-bit previous accel in core format (S1E8M18)
    // LATE INPUT: sample this only when term reaches final add point.
    input  [26:0] i_a_b1_x,
    input  [26:0] i_a_b1_y,

    // 27-bit output accel in core format (S1E8M18)
    output [26:0] o_a_b1_x,
    output [26:0] o_a_b1_y
);

  integer k;

  //--------------------------------------------------------------------------
  // eps^2 in core format (S1E8M18)
  //--------------------------------------------------------------------------
  localparam [26:0] epsilon_square = {1'd0, 8'd125, 18'd0};

  //--------------------------------------------------------------------------
  // Pack 16-bit S1E8M7 -> 27-bit (S1E8M18 style): {s,e,m7,11'b0}
  // Preserve zero exactly.
  //--------------------------------------------------------------------------
  wire [26:0] b1_x_27 = (i_b1_x[14:0] == 15'd0) ? 27'd0 : {i_b1_x[15], i_b1_x[14:7], i_b1_x[6:0], 11'd0};
  wire [26:0] b1_y_27 = (i_b1_y[14:0] == 15'd0) ? 27'd0 : {i_b1_y[15], i_b1_y[14:7], i_b1_y[6:0], 11'd0};
  wire [26:0] b2_x_27 = (i_b2_x[14:0] == 15'd0) ? 27'd0 : {i_b2_x[15], i_b2_x[14:7], i_b2_x[6:0], 11'd0};
  wire [26:0] b2_y_27 = (i_b2_y[14:0] == 15'd0) ? 27'd0 : {i_b2_y[15], i_b2_y[14:7], i_b2_y[6:0], 11'd0};
  wire [26:0] m2_27   = (i_m_b2[14:0] == 15'd0) ? 27'd0 : {i_m_b2[15], i_m_b2[14:7], i_m_b2[6:0], 11'd0};

  //--------------------------------------------------------------------------
  // c0: register packed inputs
  //--------------------------------------------------------------------------
  reg [26:0] r_b1_x_c0, r_b1_y_c0, r_b2_x_c0, r_b2_y_c0;
  reg [26:0] r_m2_c0;

  always @(posedge i_clk or negedge i_rst) begin
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

  //--------------------------------------------------------------------------
  // Reset-mux for fplib blocks (they have no reset)
  //--------------------------------------------------------------------------
  wire en = i_rst;
  wire [26:0] b2x_in = en ? r_b2_x_c0 : 27'd0;
  wire [26:0] b2y_in = en ? r_b2_y_c0 : 27'd0;

  //--------------------------------------------------------------------------
  // c0 comb: negate b1 so dx = x2 + (-x1), dy = y2 + (-y1)
  //--------------------------------------------------------------------------
  wire [26:0] w_b1_x_neg, w_b1_y_neg;
  FpNegate u_neg_b1x (.iA(en ? r_b1_x_c0 : 27'd0), .oNegative(w_b1_x_neg));
  FpNegate u_neg_b1y (.iA(en ? r_b1_y_c0 : 27'd0), .oNegative(w_b1_y_neg));

  //--------------------------------------------------------------------------
  // c0 -> c2: dx, dy (FpAdd = 2)
  //--------------------------------------------------------------------------
  wire [26:0] w_dx_c2, w_dy_c2;
  FpAdd u_add_dx (.iCLK(i_clk), .iA(b2x_in), .iB(w_b1_x_neg), .oSum(w_dx_c2));
  FpAdd u_add_dy (.iCLK(i_clk), .iA(b2y_in), .iB(w_b1_y_neg), .oSum(w_dy_c2));

  //--------------------------------------------------------------------------
  // c2 -> c3: dx2, dy2 (mul comb + reg = 1)
  //--------------------------------------------------------------------------
  wire [26:0] w_dx2_comb, w_dy2_comb;
  reg  [26:0] r_dx2_c3,   r_dy2_c3;

  FpMul u_mul_dx2 (.iA(w_dx_c2), .iB(w_dx_c2), .oProd(w_dx2_comb));
  FpMul u_mul_dy2 (.iA(w_dy_c2), .iB(w_dy_c2), .oProd(w_dy2_comb));

  always @(posedge i_clk or negedge i_rst) begin
    if (!i_rst) begin
      r_dx2_c3 <= 27'd0;
      r_dy2_c3 <= 27'd0;
    end else begin
      r_dx2_c3 <= w_dx2_comb;
      r_dy2_c3 <= w_dy2_comb;
    end
  end

  //--------------------------------------------------------------------------
  // c3 -> c5: r2 = dx2 + dy2 (FpAdd = 2)
  //--------------------------------------------------------------------------
  wire [26:0] w_r2_c5;
  FpAdd u_add_r2 (.iCLK(i_clk), .iA(en ? r_dx2_c3 : 27'd0), .iB(en ? r_dy2_c3 : 27'd0), .oSum(w_r2_c5));

  //--------------------------------------------------------------------------
  // c5 -> c7: r2e = r2 + eps2 (FpAdd = 2)
  //--------------------------------------------------------------------------
  wire [26:0] w_r2e_c7;
  FpAdd u_add_r2e (.iCLK(i_clk), .iA(en ? w_r2_c5 : 27'd0), .iB(en ? epsilon_square : 27'd0), .oSum(w_r2e_c7));

  //--------------------------------------------------------------------------
  // c7 -> c11: s_raw = inv_sqrt(r2e)
  //--------------------------------------------------------------------------
  wire [26:0] w_s_c11;
  FpInvSqrt u_invsqrt (.iCLK(i_clk), .iA(en ? w_r2e_c7 : 27'd0), .oInvSqrt(w_s_c11));

  //--------------------------------------------------------------------------
  // c11 -> c12: register s before mul usage
  //--------------------------------------------------------------------------
  reg [26:0] r_s_c12;
  always @(posedge i_clk or negedge i_rst) begin
    if (!i_rst) r_s_c12 <= 27'd0;
    else        r_s_c12 <= w_s_c11;
  end

  //--------------------------------------------------------------------------
  // Align m2 to c12
  //--------------------------------------------------------------------------
  reg [26:0] r_m2_pipe[0:11];
  always @(posedge i_clk or negedge i_rst) begin
    if (!i_rst) begin
      for (k = 0; k < 12; k = k + 1) r_m2_pipe[k] <= 27'd0;
    end else begin
      r_m2_pipe[0] <= r_m2_c0;
      for (k = 0; k < 11; k = k + 1) r_m2_pipe[k+1] <= r_m2_pipe[k];
    end
  end
  wire [26:0] m2_c12 = r_m2_pipe[11];

  //--------------------------------------------------------------------------
  // c12 -> c13: s2 = s*s, t = m2*s
  //--------------------------------------------------------------------------
  wire [26:0] w_s2_comb, w_t_comb;
  reg  [26:0] r_s2_c13,  r_t_c13;

  FpMul u_mul_s2 (.iA(r_s_c12), .iB(r_s_c12), .oProd(w_s2_comb));
  FpMul u_mul_t  (.iA(m2_c12),  .iB(r_s_c12), .oProd(w_t_comb));

  always @(posedge i_clk or negedge i_rst) begin
    if (!i_rst) begin
      r_s2_c13 <= 27'd0;
      r_t_c13  <= 27'd0;
    end else begin
      r_s2_c13 <= w_s2_comb;
      r_t_c13  <= w_t_comb;
    end
  end

  //--------------------------------------------------------------------------
  // c13 -> c14: k = m2*s^3
  //--------------------------------------------------------------------------
  wire [26:0] w_k_comb;
  reg  [26:0] r_k_c14;

  FpMul u_mul_k (.iA(r_t_c13), .iB(r_s2_c13), .oProd(w_k_comb));

  always @(posedge i_clk or negedge i_rst) begin
    if (!i_rst) r_k_c14 <= 27'd0;
    else        r_k_c14 <= w_k_comb;
  end

  //--------------------------------------------------------------------------
  // Align dx/dy to c14
  //--------------------------------------------------------------------------
  reg [26:0] r_dx_pipe[0:11];
  reg [26:0] r_dy_pipe[0:11];

  always @(posedge i_clk or negedge i_rst) begin
    if (!i_rst) begin
      for (k = 0; k < 12; k = k + 1) begin
        r_dx_pipe[k] <= 27'd0;
        r_dy_pipe[k] <= 27'd0;
      end
    end else begin
      r_dx_pipe[0] <= w_dx_c2;
      r_dy_pipe[0] <= w_dy_c2;
      for (k = 0; k < 11; k = k + 1) begin
        r_dx_pipe[k+1] <= r_dx_pipe[k];
        r_dy_pipe[k+1] <= r_dy_pipe[k];
      end
    end
  end

  wire [26:0] dx_c14 = r_dx_pipe[11];
  wire [26:0] dy_c14 = r_dy_pipe[11];

  //--------------------------------------------------------------------------
  // c14 -> c15: pair term = dx*k, dy*k (mul comb + reg)
  // FIX: insert the register BETWEEN term and final FpAdd.
  //--------------------------------------------------------------------------
  wire [26:0] w_ax_term_c15;
  wire [26:0] w_ay_term_c15;
  reg  [26:0] r_ax_term_c15;
  reg  [26:0] r_ay_term_c15;

  FpMul u_mul_ax (.iA(dx_c14), .iB(r_k_c14), .oProd(w_ax_term_c15));
  FpMul u_mul_ay (.iA(dy_c14), .iB(r_k_c14), .oProd(w_ay_term_c15));

  always @(posedge i_clk or negedge i_rst) begin
    if (!i_rst) begin
      r_ax_term_c15 <= 27'd0;
      r_ay_term_c15 <= 27'd0;
    end else begin
      r_ax_term_c15 <= w_ax_term_c15;
      r_ay_term_c15 <= w_ay_term_c15;
    end
  end

  //--------------------------------------------------------------------------
  // c15 -> c17: out = prev + term_reg (FpAdd = 2)
  // prev is taken DIRECTLY from the external interface here.
  //--------------------------------------------------------------------------
  wire [26:0] w_out_x_c17, w_out_y_c17;
  FpAdd u_add_outx (.iCLK(i_clk), .iA(en ? i_a_b1_x         : 27'd0), .iB(en ? r_ax_term_c15 : 27'd0), .oSum(w_out_x_c17));
  FpAdd u_add_outy (.iCLK(i_clk), .iA(en ? i_a_b1_y         : 27'd0), .iB(en ? r_ay_term_c15 : 27'd0), .oSum(w_out_y_c17));

  assign o_a_b1_x = w_out_x_c17;
  assign o_a_b1_y = w_out_y_c17;

endmodule
