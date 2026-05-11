`timescale 1ns/1ps

module tb_nbody_integrator;

  localparam int DATA_W = 27;
  localparam int SIGN_SHIFT = 26;

  logic clk;
  logic reset;
  logic i_start;
  logic o_done;

  logic [DATA_W-1:0] i_x;
  logic [DATA_W-1:0] i_y;
  logic [DATA_W-1:0] i_vx;
  logic [DATA_W-1:0] i_vy;
  logic [DATA_W-1:0] i_ax;
  logic [DATA_W-1:0] i_ay;

  logic [DATA_W-1:0] o_x;
  logic [DATA_W-1:0] o_y;
  logic [DATA_W-1:0] o_vx;
  logic [DATA_W-1:0] o_vy;

  int err_count;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  nbody_integrator #(
      .DATA_W(DATA_W)
  ) dut (
      .clk    (clk),
      .reset  (reset),
      .i_start(i_start),
      .o_done (o_done),
      .i_x    (i_x),
      .i_y    (i_y),
      .i_vx   (i_vx),
      .i_vy   (i_vy),
      .i_ax   (i_ax),
      .i_ay   (i_ay),
      .o_x    (o_x),
      .o_y    (o_y),
      .o_vx   (o_vx),
      .o_vy   (o_vy)
  );

  function automatic logic [17:0] rtl_frac18_from_u27(input logic [DATA_W-1:0] u);
    begin
      rtl_frac18_from_u27 = {1'b1, u[17:1]};
    end
  endfunction

  function automatic int lod37_shift_amt(input logic [36:0] x);
    begin
      lod37_shift_amt = 37;
      for (int bit_idx = 36; bit_idx >= 0; bit_idx--) begin
        if (x[bit_idx] && lod37_shift_amt == 37) begin
          lod37_shift_amt = 36 - bit_idx;
        end
      end
    end
  endfunction

  function automatic logic [DATA_W-1:0] fp27_add_model(
    input logic [DATA_W-1:0] a,
    input logic [DATA_W-1:0] b
  );
    logic sa, sb;
    int ea, eb;
    logic [17:0] af, bf;
    logic a_larger;
    int exp_diff_a, exp_diff_b;
    int larger_exp;
    logic [36:0] a_ext, b_ext;
    logic [36:0] a_shifted, b_shifted;
    logic [36:0] pre_sum;
    int shft_amt;
    logic [53:0] pre_frac_54;
    logic [53:0] pre_frac_shft;
    logic [53:0] uflow_shift;
    logic [17:0] osum_f;
    int osum_e;
    logic underflow;
    logic out_sign;
    begin
      sa = a[SIGN_SHIFT];
      sb = b[SIGN_SHIFT];
      ea = a[25:18];
      eb = b[25:18];
      af = rtl_frac18_from_u27(a);
      bf = rtl_frac18_from_u27(b);

      a_larger = ((ea > eb) || ((ea == eb) && (af > bf)));
      exp_diff_a = (eb - ea) & 8'hff;
      exp_diff_b = (ea - eb) & 8'hff;
      larger_exp = (eb > ea) ? eb : ea;

      a_ext = {1'b0, af, 18'b0};
      b_ext = {1'b0, bf, 18'b0};
      a_shifted = a_larger ? a_ext : ((exp_diff_a > 35) ? 37'd0 : (a_ext >> exp_diff_a));
      b_shifted = !a_larger ? b_ext : ((exp_diff_b > 35) ? 37'd0 : (b_ext >> exp_diff_b));

      if ((sa ^ sb) && a_larger) begin
        pre_sum = a_shifted - b_shifted;
      end else if ((sa ^ sb) && !a_larger) begin
        pre_sum = b_shifted - a_shifted;
      end else begin
        pre_sum = a_shifted + b_shifted;
      end

      shft_amt = lod37_shift_amt(pre_sum);
      pre_frac_54 = {pre_sum, 17'b0};
      pre_frac_shft = pre_frac_54 << (shft_amt + 1);
      uflow_shift = pre_frac_54 << shft_amt;
      osum_f = pre_frac_shft[53:36];
      osum_e = (larger_exp - shft_amt + 1) & 8'hff;
      underflow = ~uflow_shift[53];
      out_sign = a_larger ? sa : sb;

      if ((ea == 0) && (eb == 0)) begin
        fp27_add_model = '0;
      end else if (ea == 0) begin
        fp27_add_model = b;
      end else if (eb == 0) begin
        fp27_add_model = a;
      end else if (underflow) begin
        fp27_add_model = '0;
      end else if (pre_sum == 0) begin
        fp27_add_model = '0;
      end else begin
        fp27_add_model = {out_sign, osum_e[7:0], osum_f};
      end
    end
  endfunction

  task automatic clear_inputs;
    begin
      i_start = 1'b0;
      i_x = '0;
      i_y = '0;
      i_vx = '0;
      i_vy = '0;
      i_ax = '0;
      i_ay = '0;
    end
  endtask

  task automatic run_case(
    input string name,
    input logic [DATA_W-1:0] x,
    input logic [DATA_W-1:0] y,
    input logic [DATA_W-1:0] vx,
    input logic [DATA_W-1:0] vy,
    input logic [DATA_W-1:0] ax,
    input logic [DATA_W-1:0] ay
  );
    logic [DATA_W-1:0] exp_vx;
    logic [DATA_W-1:0] exp_vy;
    logic [DATA_W-1:0] exp_x;
    logic [DATA_W-1:0] exp_y;
    int wait_cycles;
    begin
      exp_vx = fp27_add_model(vx, ax);
      exp_vy = fp27_add_model(vy, ay);
      exp_x = fp27_add_model(x, exp_vx);
      exp_y = fp27_add_model(y, exp_vy);

      @(negedge clk);
      i_x = x;
      i_y = y;
      i_vx = vx;
      i_vy = vy;
      i_ax = ax;
      i_ay = ay;
      i_start = 1'b1;
      @(posedge clk);
      @(negedge clk);
      i_start = 1'b0;

      wait_cycles = 0;
      do begin
        @(posedge clk);
        #1;
        wait_cycles++;
      end while (o_done !== 1'b1 && wait_cycles < 20);

      if (o_done !== 1'b1) begin
        $display("ERROR %s timed out waiting for o_done", name);
        err_count++;
      end else if (wait_cycles != 9) begin
        $display("ERROR %s done latency got %0d expected 9", name, wait_cycles);
        err_count++;
      end

      if (o_vx !== exp_vx || o_vy !== exp_vy || o_x !== exp_x || o_y !== exp_y) begin
        $display("ERROR %s mismatch", name);
        $display("  got vx=%07h vy=%07h x=%07h y=%07h", o_vx, o_vy, o_x, o_y);
        $display("  exp vx=%07h vy=%07h x=%07h y=%07h", exp_vx, exp_vy, exp_x, exp_y);
        err_count++;
      end else begin
        $display("PASS %s vx=%07h vy=%07h x=%07h y=%07h", name, o_vx, o_vy, o_x, o_y);
      end

      @(posedge clk);
      #1;
      if (o_done !== 1'b0) begin
        $display("ERROR %s o_done did not deassert after one cycle", name);
        err_count++;
      end
    end
  endtask

  initial begin
    err_count = 0;
    reset = 1'b1;
    clear_inputs();

    repeat (3) @(posedge clk);
    #1;
    if (o_done !== 1'b0 || o_x !== '0 || o_y !== '0 || o_vx !== '0 || o_vy !== '0) begin
      $display("ERROR reset outputs not zero");
      err_count++;
    end

    @(negedge clk);
    reset = 1'b0;

    run_case("zero", 27'h0000000, 27'h0000000, 27'h0000000, 27'h0000000,
             27'h0000000, 27'h0000000);

    run_case("positive", 27'h3fc0000, 27'h4040000, 27'h3f80000, 27'h3f00000,
             27'h3e80000, 27'h3e00000);

    run_case("signed", 27'h4040000, 27'h6040000, 27'h3fc0000, 27'h5fc0000,
             27'h5fc0000, 27'h3fc0000);

    run_case("mantissa", 27'h3fc1234, 27'h4045678, 27'h3f8abcd, 27'h5f81234,
             27'h3e85555, 27'h3e8aaaa);

    if (err_count == 0) begin
      $display("PASS: tb_nbody_integrator completed without errors");
    end else begin
      $display("FAIL: tb_nbody_integrator saw %0d error(s)", err_count);
    end

    $finish;
  end

endmodule
