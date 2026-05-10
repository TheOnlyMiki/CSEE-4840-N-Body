`timescale 1ns/1ps

//------------------------------------------------------------------------------
// TB: multi-body streaming scheduler driving a 2-body core
// - Reads frame0 file:  idx px py vx vy m   (all hex, S1E8M7 for px/py/vx/vy/m)
// - Streams all pairs (b1 accumulates contributions from every b2!=b1)
// - Core interface (NEW):
//     inputs: 16-bit S1E8M7 (pos/mass), prev accel: 27-bit S1E8M18
//     outputs: 27-bit S1E8M18
// - TB truncates final 27-bit accum to 16-bit S1E8M7 for "chip output" file
// - Core has fixed epsilon_square internally 

module tb_core_accel;

  // -----------------------------
  // User knobs
  // -----------------------------
  localparam int N_BODIES  = 256;   // <-- set any N you want
  localparam int PIPE_LAT  = 18;  // as per your datapath

  localparam string INPUT_FILE = "../../input/TB/frame_input/frame0_256binit200.txt";
  localparam string OUT_FILE   = "../../input/TB/accel_output/eps025_accel_256binit200.txt";
  localparam string OUT_FILE_27bits   = "../../input/TB/eps025new_temp_27bits_256binit200.txt";

  // -----------------------------
  // DUT IO (NEW CORE INTERFACE)
  //   pos/mass: 16-bit S1E8M7
  //   prev accel in: 27-bit
  //   accel out: 27-bit
  // -----------------------------
  logic        i_clk;
  logic        i_rst;  // active-low reset in your RTL: if(!i_rst) reset, else run

  logic [15:0] i_b1_x, i_b1_y;
  logic [15:0] i_b2_x, i_b2_y;
  logic [15:0] i_m_b2;

  logic [26:0] i_a_b1_x, i_a_b1_y;       // 27-bit prev accel
  wire  [26:0] o_a_b1_x_27, o_a_b1_y_27; // 27-bit DUT accel out

  // Truncated "chip output" (16-bit) - TB performs truncation
  wire [15:0] o_a_b1_x, o_a_b1_y;

  // Truncation: keep sign/exp, take top 7 mant bits (of 18)
  // Optional zero-detect (treat exact 0 as 0)
  wire outx_zero = (o_a_b1_x_27[25:0] == 26'd0);
  wire outy_zero = (o_a_b1_y_27[25:0] == 26'd0);
  assign o_a_b1_x = outx_zero ? 16'd0 : {o_a_b1_x_27[26], o_a_b1_x_27[25:18], o_a_b1_x_27[17:11]};
  assign o_a_b1_y = outy_zero ? 16'd0 : {o_a_b1_y_27[26], o_a_b1_y_27[25:18], o_a_b1_y_27[17:11]};

  // -----------------------------
  // DUT (eps fixed inside core; no i_eps2 port)
  // -----------------------------
  two_body_core dut (
    .i_clk    (i_clk),
    .i_rst    (i_rst),

    .i_b1_x   (i_b1_x),
    .i_b1_y   (i_b1_y),
    .i_b2_x   (i_b2_x),
    .i_b2_y   (i_b2_y),
    .i_m_b2   (i_m_b2),

    .i_a_b1_x (i_a_b1_x),
    .i_a_b1_y (i_a_b1_y),

    .o_a_b1_x (o_a_b1_x_27),
    .o_a_b1_y (o_a_b1_y_27)
  );

  // -----------------------------
  // Clock
  // -----------------------------
  initial i_clk = 1'b0;
  always #5 i_clk = ~i_clk; // 100MHz

  // -----------------------------
  // Frame storage
  // -----------------------------
  logic [15:0] px [0:N_BODIES-1];
  logic [15:0] py [0:N_BODIES-1];
  logic [15:0] m  [0:N_BODIES-1];

  // Output accel (chip view, 16-bit)
  logic [15:0] ax [0:N_BODIES-1];
  logic [15:0] ay [0:N_BODIES-1];

  // -----------------------------
  // Streaming control (TB-only)
  // -----------------------------
  logic in_valid;
  logic [PIPE_LAT-1:0] vsh;
  logic out_valid;
  assign out_valid = vsh[PIPE_LAT-1];

  always_ff @(posedge i_clk) begin
    if (!i_rst) vsh <= '0;
    else        vsh <= {vsh[PIPE_LAT-2:0], in_valid};
  end

  // -----------------------------
  // Helpers
  // -----------------------------
  task automatic drive_idle();
    begin
      i_b1_x   <= 16'h0000;
      i_b1_y   <= 16'h0000;
      i_b2_x   <= 16'h0000;
      i_b2_y   <= 16'h0000;
      i_m_b2   <= 16'h0000;

      i_a_b1_x <= 27'd0;
      i_a_b1_y <= 27'd0;

      in_valid <= 1'b0;
    end
  endtask

  task automatic drive_txn(
    input int b1,
    input int b2,
    input logic [26:0] a_prev_x_27,
    input logic [26:0] a_prev_y_27
  );
    begin
      i_b1_x   <= px[b1];
      i_b1_y   <= py[b1];
      i_b2_x   <= px[b2];
      i_b2_y   <= py[b2];
      i_m_b2   <= m[b2];

      i_a_b1_x <= a_prev_x_27;
      i_a_b1_y <= a_prev_y_27;

      in_valid <= 1'b1;
    end
  endtask


  // -----------------------------
  // Read input frame file
  // line format: idx px py vx vy m (hex)
  // -----------------------------
  task automatic read_frame_file(input string fname);
    int fd;
    string line;
    int idx;
    logic [15:0] lpx, lpy, lvx, lvy, lm;
    int got;
    begin
      for (int t = 0; t < N_BODIES; t++) begin
        px[t] = 16'h0000;
        py[t] = 16'h0000;
        m[t]  = 16'h0000;
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

  // -----------------------------
  // Auto-alignment queue (tracks which txn produced each output)
  // -----------------------------
  typedef struct packed {
    int b1;
    int b2;
  } txn_t;

  txn_t q[$];

  // -----------------------------
  // Scheduling state
  // We keep per-b1 running accel in 27-bit (as the core output is "prev+term")
  // busy[b1] prevents issuing next (b1, b2) until previous output for b1 returns.
  // -----------------------------
  logic [26:0] a_acc_x_27 [0:N_BODIES-1];
  logic [26:0] a_acc_y_27 [0:N_BODIES-1];
  logic        busy       [0:N_BODIES-1];
  int          next_j     [0:N_BODIES-1];

  function automatic int pick_b1();
    int b;
    begin
      pick_b1 = -1;
      for (b = 0; b < N_BODIES; b++) begin
        if (!busy[b]) begin
          // skip self-pair
          while (next_j[b] < N_BODIES && next_j[b] == b) next_j[b]++;
          if (next_j[b] < N_BODIES) begin
            pick_b1 = b;
            return pick_b1;
          end
        end
      end
    end
  endfunction

  function automatic bit all_done();
    int b;
    begin
      all_done = 1'b1;
      for (b = 0; b < N_BODIES; b++) begin
        int tmp;
        tmp = next_j[b];
        while (tmp < N_BODIES && tmp == b) tmp++;
        if (tmp < N_BODIES) all_done = 1'b0;
        if (busy[b])        all_done = 1'b0;
      end
      if (q.size() != 0) all_done = 1'b0;
      if (vsh != '0)     all_done = 1'b0;
    end
  endfunction

  integer fo;
  integer b;
  integer b1;
  integer b2;

  // -----------------------------
  // Main
  // -----------------------------
  initial begin
    drive_idle();

    // reset (active-low)
    i_rst = 1'b0;
    repeat (5) @(posedge i_clk);
    i_rst = 1'b1;
    repeat (5) @(posedge i_clk);

    read_frame_file(INPUT_FILE);

    // init per-body accumulators and sched
    for (b = 0; b < N_BODIES; b++) begin
      a_acc_x_27[b] = 27'd0;
      a_acc_y_27[b] = 27'd0;
      busy[b]       = 1'b0;
      next_j[b]     = 0;
    end

    // streaming loop
    while (!all_done()) begin
      @(posedge i_clk); #1;

      // 1) receive aligned output (27-bit), update per-b1 accumulator
      if (out_valid) begin
        txn_t t;
        if (q.size() == 0) $fatal(1, "out_valid but txn queue empty (alignment bug)");
        t = q.pop_front();

        // core returns prev+term already, so we overwrite accumulator
        a_acc_x_27[t.b1] = o_a_b1_x_27;
        a_acc_y_27[t.b1] = o_a_b1_y_27;

        busy[t.b1] = 1'b0;
      end

      // 2) issue one txn if possible
      b1 = pick_b1();
      if (b1 >= 0) begin
        b2 = next_j[b1];
        while (b2 < N_BODIES && b2 == b1) b2++;

        if (b2 < N_BODIES) begin
          drive_txn(b1, b2, a_acc_x_27[b1], a_acc_y_27[b1]);

          q.push_back('{b1:b1, b2:b2});

          busy[b1]   = 1'b1;
          next_j[b1] = b2 + 1;
        end else begin
          drive_idle();
        end
      end else begin
        drive_idle();
      end
    end

    // Final "chip output" is 16-bit => truncate here for file output
    for (b = 0; b < N_BODIES; b++) begin
      ax[b] = (a_acc_x_27[b][25:0] == 26'd0) ? 16'd0
             : {a_acc_x_27[b][26], a_acc_x_27[b][25:18], a_acc_x_27[b][17:11]};
      ay[b] = (a_acc_y_27[b][25:0] == 26'd0) ? 16'd0
             : {a_acc_y_27[b][26], a_acc_y_27[b][25:18], a_acc_y_27[b][17:11]};
    end

    // write 16-bit output
    fo = $fopen(OUT_FILE, "w");
    if (fo == 0) $fatal(1, "ERROR: cannot open OUT_FILE=%s", OUT_FILE);

    $fwrite(fo, "# i ax ay (S1E8M7 hex)\n");
    for (b = 0; b < N_BODIES; b++) begin
      $fwrite(fo, "%4d  %04h  %04h\n", b, ax[b], ay[b]);
    end
    $fclose(fo);

    // write 27-bit accumulated output
    fo = $fopen(OUT_FILE_27bits, "w");
    if (fo == 0) $fatal(1, "ERROR: cannot open OUT_FILE_27bits=%s", OUT_FILE_27bits);
    $fwrite(fo, "# i ax27 ay27 (S1E8M18 hex)\n");
    for (b = 0; b < N_BODIES; b++) begin
      $fwrite(fo, "%4d  %07h  %07h\n", b, a_acc_x_27[b], a_acc_y_27[b]);
    end
    $fclose(fo);

    $display("DONE. Wrote %s", OUT_FILE);
    $display("DONE. Wrote %s", OUT_FILE_27bits);
    $finish;
  end

endmodule
