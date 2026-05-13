`timescale 1ns/1ps

// Full-frame testbench for four_core_wrapper.
//
// This drives the wrapper the same way nbody_control intends to use it:
//   - load one 16-body tile into wrapper local memory
//   - clear wrapper accumulation state once for the tile
//   - for each j body, hold j stable while groups 0,1,2,3 are issued
//   - after the pipeline drains, sample all 16 accumulated lane outputs
//
// Output format:
//   # i ax ay (S1E8M18 hex)

module tb_four_core_wrapper;

  localparam int DATA_W = 27;
  localparam int N_BODIES = 1024;
  localparam int TILE_SIZE = 16;
  localparam int GROUP_SIZE = 4;
  localparam int N_GROUPS = TILE_SIZE / GROUP_SIZE;
  localparam int PIPE_LAT = 18;
  localparam int RUN_TIMEOUT_CYCLES = 2000000;

  localparam string INPUT_FILE = "tb/frame_input/frame0_1024binit200_27bits.txt";
  localparam string OUT_FILE   = "tb/output/four_core_wrapper_accel_1024binit200_27bits.txt";

  logic clk;
  logic rst_n;
  logic i_clear_prev;
  logic i_load_en;
  logic i_compute_en;
  logic [3:0] i_load_idx;
  logic [DATA_W-1:0] i_load_x;
  logic [DATA_W-1:0] i_load_y;
  logic [1:0] i_grp_sel;
  logic [DATA_W-1:0] i_j_x;
  logic [DATA_W-1:0] i_j_y;
  logic [DATA_W-1:0] i_j_m;
  logic [3:0] i_lane_mask;

  wire [DATA_W-1:0] o_res0_x;
  wire [DATA_W-1:0] o_res0_y;
  wire [DATA_W-1:0] o_res1_x;
  wire [DATA_W-1:0] o_res1_y;
  wire [DATA_W-1:0] o_res2_x;
  wire [DATA_W-1:0] o_res2_y;
  wire [DATA_W-1:0] o_res3_x;
  wire [DATA_W-1:0] o_res3_y;
  wire              o_res_vld;

  logic [DATA_W-1:0] px [0:N_BODIES-1];
  logic [DATA_W-1:0] py [0:N_BODIES-1];
  logic [DATA_W-1:0] m  [0:N_BODIES-1];
  logic [DATA_W-1:0] ax [0:N_BODIES-1];
  logic [DATA_W-1:0] ay [0:N_BODIES-1];

  int err_count;
  int fo;
  int cycles;

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

  task automatic drive_idle;
    begin
      i_clear_prev = 1'b0;
      i_load_en = 1'b0;
      i_compute_en = 1'b0;
      i_load_idx = 4'd0;
      i_load_x = '0;
      i_load_y = '0;
      i_j_x = '0;
      i_j_y = '0;
      i_j_m = '0;
      i_lane_mask = 4'h0;
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
        m[t] = '0;
        ax[t] = '0;
        ay[t] = '0;
      end

      fd = $fopen(fname, "r");
      if (fd == 0) $fatal(1, "ERROR: cannot open INPUT_FILE=%s", fname);

      while (!$feof(fd)) begin
        line = "";
        void'($fgets(line, fd));

        if (line.len() == 0) continue;
        if (line.substr(0, 0) == "#") continue;

        got = $sscanf(line, "%d %h %h %h %h %h",
                      idx, lpx, lpy, lvx, lvy, lm);

        if (got == 6 && idx >= 0 && idx < N_BODIES) begin
          px[idx] = lpx;
          py[idx] = lpy;
          m[idx] = lm;
        end
      end

      $fclose(fd);
    end
  endtask

  task automatic load_tile(input int tile_base);
    int lane;
    int body_idx;
    begin
      for (lane = 0; lane < TILE_SIZE; lane++) begin
        body_idx = tile_base + lane;
        @(negedge clk);
        drive_idle();
        i_load_en = 1'b1;
        i_load_idx = lane[3:0];
        if (body_idx < N_BODIES) begin
          i_load_x = px[body_idx];
          i_load_y = py[body_idx];
        end else begin
          i_load_x = '0;
          i_load_y = '0;
        end
        @(posedge clk);
      end

      @(negedge clk);
      drive_idle();
    end
  endtask

  task automatic pulse_clear_prev;
    begin
      @(negedge clk);
      drive_idle();
      i_grp_sel = 2'd0;
      i_clear_prev = 1'b1;
      @(posedge clk);
      @(negedge clk);
      i_clear_prev = 1'b0;
    end
  endtask

  function automatic logic [3:0] make_lane_mask(input int tile_base, input int grp, input int j_idx);
    int lane_body;
    begin
      for (int lane = 0; lane < GROUP_SIZE; lane++) begin
        lane_body = tile_base + grp * GROUP_SIZE + lane;
        make_lane_mask[lane] = (lane_body >= N_BODIES) || (lane_body == j_idx);
      end
    end
  endfunction

  task automatic issue_one_compute(
    input [1:0] grp_sel,
    input [3:0] lane_mask,
    input [DATA_W-1:0] jx,
    input [DATA_W-1:0] jy,
    input [DATA_W-1:0] jm
  );
    begin
      @(negedge clk);
      drive_idle();
      i_grp_sel = grp_sel;
      i_lane_mask = lane_mask;
      i_j_x = jx;
      i_j_y = jy;
      i_j_m = jm;
      i_compute_en = 1'b1;
      @(posedge clk);
    end
  endtask

  task automatic drain_tile;
    begin
      @(negedge clk);
      drive_idle();
      i_grp_sel = 2'd0;

      for (int cyc = 0; cyc < PIPE_LAT + 4; cyc++) begin
        @(posedge clk);
      end

      for (int grp = 0; grp < N_GROUPS; grp++) begin
        @(negedge clk);
        drive_idle();
        i_grp_sel = grp[1:0];
        #1;
        if (o_res_vld !== 1'b1) begin
          $display("ERROR: o_res_vld not set for tile group %0d at t=%0t", grp, $time);
          err_count++;
        end
      end
    end
  endtask

  task automatic save_group(input int tile_base, input int grp);
    int base;
    begin
      base = tile_base + grp * GROUP_SIZE;
      if (base + 0 < N_BODIES) begin ax[base + 0] = o_res0_x; ay[base + 0] = o_res0_y; end
      if (base + 1 < N_BODIES) begin ax[base + 1] = o_res1_x; ay[base + 1] = o_res1_y; end
      if (base + 2 < N_BODIES) begin ax[base + 2] = o_res2_x; ay[base + 2] = o_res2_y; end
      if (base + 3 < N_BODIES) begin ax[base + 3] = o_res3_x; ay[base + 3] = o_res3_y; end
    end
  endtask

  task automatic run_tile(input int tile_base);
    begin
      load_tile(tile_base);
      repeat (2) @(posedge clk);
      pulse_clear_prev();

      for (int j = 0; j < N_BODIES; j++) begin
        for (int grp = 0; grp < N_GROUPS; grp++) begin
          issue_one_compute(grp[1:0],
                            make_lane_mask(tile_base, grp, j),
                            px[j], py[j], m[j]);
          cycles++;
          if (cycles > RUN_TIMEOUT_CYCLES) begin
            $fatal(1, "ERROR: timeout while streaming wrapper computes");
          end
        end
      end

      drain_tile();
      for (int grp = 0; grp < N_GROUPS; grp++) begin
        @(negedge clk);
        drive_idle();
        i_grp_sel = grp[1:0];
        #1;
        save_group(tile_base, grp);
      end
    end
  endtask

  task automatic write_output(input string fname);
    begin
      fo = $fopen(fname, "w");
      if (fo == 0) $fatal(1, "ERROR: cannot open OUT_FILE=%s", fname);

      $fwrite(fo, "# i ax ay (S1E8M18 hex)\n");
      for (int i = 0; i < N_BODIES; i++) begin
        $fwrite(fo, "%4d  %07h  %07h\n", i, ax[i], ay[i]);
      end

      $fclose(fo);
    end
  endtask

  initial begin
    rst_n = 1'b0;
    i_grp_sel = 2'd0;
    drive_idle();
    err_count = 0;
    cycles = 0;

    read_frame_file(INPUT_FILE);

    repeat (5) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    repeat (5) @(posedge clk);

    for (int tile_base = 0; tile_base < N_BODIES; tile_base += TILE_SIZE) begin
      run_tile(tile_base);
    end

    write_output(OUT_FILE);

    if (err_count == 0) begin
      $display("PASS: tb_four_core_wrapper completed without errors");
    end else begin
      $display("FAIL: tb_four_core_wrapper saw %0d error(s)", err_count);
    end
    $display("DONE. cycles=%0d wrote %s", cycles, OUT_FILE);
    $finish;
  end

endmodule
