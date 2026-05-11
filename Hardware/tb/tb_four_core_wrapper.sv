`timescale 1ns/1ps

module tb_four_core_wrapper;

  localparam int DATA_W = 27;
  localparam int N_BODIES = 256;
  localparam integer PIPE_LAT = 18;
  localparam string INPUT_FILE = "tb/frame_input/frame0_256binit200_27bits.txt";
  localparam string OUT_FILE   = "tb/output/four_core_wrapper_accel_256binit200_27bits.txt";

  logic         clk;
  logic         rst_n;
  logic         i_clear_prev;
  logic         i_load_en;
  logic         i_compute_en;
  logic  [3:0]  i_load_idx;
  logic  [DATA_W-1:0] i_load_x;
  logic  [DATA_W-1:0] i_load_y;
  logic  [1:0]  i_grp_sel;
  logic  [DATA_W-1:0] i_j_x;
  logic  [DATA_W-1:0] i_j_y;
  logic  [DATA_W-1:0] i_j_m;
  logic  [3:0]  i_lane_mask;

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
  integer fo;
  logic [DATA_W-1:0] px [0:N_BODIES-1];
  logic [DATA_W-1:0] py [0:N_BODIES-1];
  logic [DATA_W-1:0] m  [0:N_BODIES-1];

  initial clk = 1'b0;
  always #5 clk = ~clk;

  four_core_wrapper #(
      .DATA_W(DATA_W)
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
        i_load_x    = px[idx];
        i_load_y    = py[idx];
        i_grp_sel   = 2'd0;
        i_clear_prev = 1'b0;
        i_compute_en = 1'b0;
        i_j_x       = '0;
        i_j_y       = '0;
        i_j_m       = '0;
        i_lane_mask = 4'h0;
        @(posedge clk);
      end

      @(negedge clk);
      i_load_en  = 1'b0;
      i_load_idx = 4'd0;
      i_load_x   = '0;
      i_load_y   = '0;
    end
  endtask

  task automatic issue_one_compute;
    input [1:0] grp_sel;
    input [3:0] lane_mask;
    input [DATA_W-1:0] jx;
    input [DATA_W-1:0] jy;
    input [DATA_W-1:0] jm;
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
      i_j_x        = '0;
      i_j_y        = '0;
      i_j_m        = '0;
    end
  endtask

  task automatic read_frame_file(input string fname);
    int fd;
    string line;
    int idx;
    logic [DATA_W-1:0] lpx, lpy, lvx, lvy, lm;
    int got;
    begin
      for (int t = 0; t < N_BODIES; t++) begin
        px[t] = '0;
        py[t] = '0;
        m[t]  = '0;
      end

      fd = $fopen(fname, "r");
      if (fd == 0) $fatal(1, "ERROR: cannot open INPUT_FILE=%s", fname);

      while (!$feof(fd)) begin
        line = "";
        void'($fgets(line, fd));

        if (line.len() == 0) continue;
        if (line.substr(0,0) == "#") continue;

        got = $sscanf(line, "%d %h %h %h %h %h",
                      idx, lpx, lpy, lvx, lvy, lm);

        if (got == 6 && idx >= 0 && idx < N_BODIES) begin
          px[idx] = lpx;
          py[idx] = lpy;
          m[idx]  = lm;
        end
      end
      $fclose(fd);
    end
  endtask

  task automatic wait_pipe_and_check_group;
    input [1:0] grp_sel;
    integer cyc;
    integer base;
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

      base = grp_sel * 4;
      $fwrite(fo, "%4d  %07h  %07h\n", base + 0, o_res0_x, o_res0_y);
      $fwrite(fo, "%4d  %07h  %07h\n", base + 1, o_res1_x, o_res1_y);
      $fwrite(fo, "%4d  %07h  %07h\n", base + 2, o_res2_x, o_res2_y);
      $fwrite(fo, "%4d  %07h  %07h\n", base + 3, o_res3_x, o_res3_y);
    end
  endtask

  initial begin
    rst_n        = 1'b0;
    i_clear_prev = 1'b0;
    i_load_en    = 1'b0;
    i_compute_en = 1'b0;
    i_load_idx   = 4'd0;
    i_load_x     = '0;
    i_load_y     = '0;
    i_grp_sel    = 2'd0;
    i_j_x        = '0;
    i_j_y        = '0;
    i_j_m        = '0;
    i_lane_mask  = 4'd0;
    err_count    = 0;

    read_frame_file(INPUT_FILE);
    fo = $fopen(OUT_FILE, "w");
    if (fo == 0) $fatal(1, "ERROR: cannot open OUT_FILE=%s", OUT_FILE);
    $fwrite(fo, "# i ax ay (S1E8M18 hex)\n");

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);
    expect_zero_outputs();

    load_tile();
    @(posedge clk);
    expect_zero_outputs();

    for (grp = 0; grp < 4; grp = grp + 1) begin
      issue_one_compute(grp[1:0], (grp == 0) ? 4'b0001 : 4'b0000,
                        px[16 + grp],
                        py[16 + grp],
                        m[16 + grp]);
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
    $fclose(fo);
    $display("DONE. Wrote %s", OUT_FILE);

    $finish;
  end

endmodule
