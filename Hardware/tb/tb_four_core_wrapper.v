`timescale 1ns/1ps

module tb_four_core_wrapper;

  localparam integer PIPE_LAT = 18;

  reg         clk;
  reg         rst_n;
  reg         i_clear_prev;
  reg         i_load_en;
  reg         i_compute_en;
  reg  [3:0]  i_load_idx;
  reg  [15:0] i_load_x;
  reg  [15:0] i_load_y;
  reg  [1:0]  i_grp_sel;
  reg  [15:0] i_j_x;
  reg  [15:0] i_j_y;
  reg  [15:0] i_j_m;
  reg  [3:0]  i_lane_mask;

  wire [26:0] o_res0_x;
  wire [26:0] o_res0_y;
  wire [26:0] o_res1_x;
  wire [26:0] o_res1_y;
  wire [26:0] o_res2_x;
  wire [26:0] o_res2_y;
  wire [26:0] o_res3_x;
  wire [26:0] o_res3_y;
  wire        o_res_vld;

  integer err_count;
  integer grp;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  four_core_wrapper #(
      .PIPE_LAT(PIPE_LAT)
  ) dut (
      .i_clk       (clk),
      .i_rst       (rst_n),
      .i_clear_prev(i_clear_prev),
      .i_load_en   (i_load_en),
      .i_compute_en(i_compute_en),
      .i_load_idx  (i_load_idx),
      .i_load_x    (i_load_x),
      .i_load_y    (i_load_y),
      .i_grp_sel   (i_grp_sel),
      .i_j_x       (i_j_x),
      .i_j_y       (i_j_y),
      .i_j_m       (i_j_m),
      .i_lane_mask (i_lane_mask),
      .o_res0_x    (o_res0_x),
      .o_res0_y    (o_res0_y),
      .o_res1_x    (o_res1_x),
      .o_res1_y    (o_res1_y),
      .o_res2_x    (o_res2_x),
      .o_res2_y    (o_res2_y),
      .o_res3_x    (o_res3_x),
      .o_res3_y    (o_res3_y),
      .o_res_vld   (o_res_vld)
  );

  task automatic expect_zero_outputs;
    begin
      if ((o_res0_x !== 27'd0) || (o_res0_y !== 27'd0) ||
          (o_res1_x !== 27'd0) || (o_res1_y !== 27'd0) ||
          (o_res2_x !== 27'd0) || (o_res2_y !== 27'd0) ||
          (o_res3_x !== 27'd0) || (o_res3_y !== 27'd0) ||
          (o_res_vld !== 1'b0)) begin
        $display("ERROR zero-check failed at t=%0t", $time);
        err_count = err_count + 1;
      end
    end
  endtask

  task automatic load_tile;
    integer idx;
    begin
      for (idx = 0; idx < 16; idx = idx + 1) begin
        @(negedge clk);
        i_load_en   = 1'b1;
        i_load_idx  = idx[3:0];
        i_load_x    = 16'h3c00 + idx[15:0];
        i_load_y    = 16'h4000 + idx[15:0];
        i_grp_sel   = 2'd0;
        i_clear_prev = 1'b0;
        i_compute_en = 1'b0;
        i_j_x       = 16'd0;
        i_j_y       = 16'd0;
        i_j_m       = 16'd0;
        i_lane_mask = 4'h0;
        @(posedge clk);
      end

      @(negedge clk);
      i_load_en  = 1'b0;
      i_load_idx = 4'd0;
      i_load_x   = 16'd0;
      i_load_y   = 16'd0;
    end
  endtask

  task automatic issue_one_compute;
    input [1:0] grp_sel;
    input [3:0] lane_mask;
    input [15:0] jx;
    input [15:0] jy;
    input [15:0] jm;
    begin
      @(negedge clk);
      i_grp_sel    = grp_sel;
      i_lane_mask  = lane_mask;
      i_j_x        = jx;
      i_j_y        = jy;
      i_j_m        = jm;
      i_compute_en = 1'b1;
      @(posedge clk);
      @(negedge clk);
      i_compute_en = 1'b0;
      i_lane_mask  = 4'h0;
      i_j_x        = 16'd0;
      i_j_y        = 16'd0;
      i_j_m        = 16'd0;
    end
  endtask

  task automatic wait_pipe_and_check_group;
    input [1:0] grp_sel;
    integer cyc;
    begin
      for (cyc = 0; cyc < PIPE_LAT - 1; cyc = cyc + 1) begin
        @(posedge clk);
        if (o_res_vld !== 1'b0) begin
          $display("ERROR res_vld asserted too early for group %0d at t=%0t", grp_sel, $time);
          err_count = err_count + 1;
        end
      end

      @(posedge clk);
      i_grp_sel = grp_sel;
      #1;
      if (o_res_vld !== 1'b1) begin
        $display("ERROR res_vld missing for group %0d at t=%0t", grp_sel, $time);
        err_count = err_count + 1;
      end

      if ((o_res0_x === 27'd0) && (o_res0_y === 27'd0) &&
          (o_res1_x === 27'd0) && (o_res1_y === 27'd0) &&
          (o_res2_x === 27'd0) && (o_res2_y === 27'd0) &&
          (o_res3_x === 27'd0) && (o_res3_y === 27'd0)) begin
        $display("ERROR group %0d outputs all zero at t=%0t", grp_sel, $time);
        err_count = err_count + 1;
      end
    end
  endtask

  initial begin
    rst_n        = 1'b0;
    i_clear_prev = 1'b0;
    i_load_en    = 1'b0;
    i_compute_en = 1'b0;
    i_load_idx   = 4'd0;
    i_load_x     = 16'd0;
    i_load_y     = 16'd0;
    i_grp_sel    = 2'd0;
    i_j_x        = 16'd0;
    i_j_y        = 16'd0;
    i_j_m        = 16'd0;
    i_lane_mask  = 4'd0;
    err_count    = 0;

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);
    expect_zero_outputs();

    load_tile();
    @(posedge clk);
    expect_zero_outputs();

    for (grp = 0; grp < 4; grp = grp + 1) begin
      issue_one_compute(grp[1:0], (grp == 0) ? 4'b0001 : 4'b0000,
                        16'h4200 + grp[15:0],
                        16'h4180 + grp[15:0],
                        16'h3c00);
      wait_pipe_and_check_group(grp[1:0]);
    end

    @(negedge clk);
    i_grp_sel     = 2'd0;
    i_clear_prev  = 1'b1;
    @(posedge clk);
    @(negedge clk);
    i_clear_prev  = 1'b0;
    repeat (3) @(posedge clk);
    #1;
    expect_zero_outputs();

    if (err_count == 0) begin
      $display("PASS: tb_four_core_wrapper completed without errors");
    end else begin
      $display("FAIL: tb_four_core_wrapper saw %0d error(s)", err_count);
    end

    $finish;
  end

endmodule
