`timescale 1ns/1ps

module tb_nbody_control;

  localparam int MAX_BODIES = 256;
  localparam int DATA_W = 27;
  localparam int PTR_W = 8;

  localparam string INPUT_FILE = "tb/frame_input/frame0_256binit200_27bits.txt";
  localparam string OUT_FILE   = "tb/output/control_accel_256binit200_27bits.txt";

  logic clk;
  logic reset;

  logic        go;
  logic        read_enable;
  logic        first_step;
  logic [31:0] n_bodies;
  logic [31:0] gap;
  logic        done;

  logic [PTR_W-1:0]  body_raddr;
  logic [DATA_W-1:0] body_x;
  logic [DATA_W-1:0] body_y;
  logic [DATA_W-1:0] body_m;
  logic [DATA_W-1:0] body_vx;
  logic [DATA_W-1:0] body_vy;
  logic [DATA_W-1:0] body_ax;
  logic [DATA_W-1:0] body_ay;

  logic              body_update_we;
  logic [PTR_W-1:0]  body_update_addr;
  logic [DATA_W-1:0] body_update_x;
  logic [DATA_W-1:0] body_update_y;
  logic [DATA_W-1:0] body_update_vx;
  logic [DATA_W-1:0] body_update_vy;

  logic              accel_we;
  logic [PTR_W-1:0]  accel_waddr;
  logic [DATA_W-1:0] accel_ax;
  logic [DATA_W-1:0] accel_ay;

  logic              cpu_body_we;
  logic [PTR_W-1:0]  cpu_body_waddr;
  logic [DATA_W-1:0] cpu_x;
  logic [DATA_W-1:0] cpu_y;
  logic [DATA_W-1:0] cpu_m;
  logic [DATA_W-1:0] cpu_vx;
  logic [DATA_W-1:0] cpu_vy;

  logic [DATA_W-1:0] init_x  [0:MAX_BODIES-1];
  logic [DATA_W-1:0] init_y  [0:MAX_BODIES-1];
  logic [DATA_W-1:0] init_m  [0:MAX_BODIES-1];
  logic [DATA_W-1:0] init_vx [0:MAX_BODIES-1];
  logic [DATA_W-1:0] init_vy [0:MAX_BODIES-1];
  logic              accel_seen [0:MAX_BODIES-1];

  int err_count;
  int accel_count;
  int update_count;
  int fo;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  nbody_control #(
      .MAX_BODIES(MAX_BODIES),
      .DATA_W(DATA_W)
  ) dut_control (
      .clk             (clk),
      .reset           (reset),
      .go              (go),
      .read_enable     (read_enable),
      .first_step      (first_step),
      .n_bodies        (n_bodies),
      .gap             (gap),
      .done            (done),
      .body_raddr      (body_raddr),
      .body_x          (body_x),
      .body_y          (body_y),
      .body_m          (body_m),
      .body_vx         (body_vx),
      .body_vy         (body_vy),
      .body_ax         (body_ax),
      .body_ay         (body_ay),
      .body_update_we  (body_update_we),
      .body_update_addr(body_update_addr),
      .body_update_x   (body_update_x),
      .body_update_y   (body_update_y),
      .body_update_vx  (body_update_vx),
      .body_update_vy  (body_update_vy),
      .accel_we        (accel_we),
      .accel_waddr     (accel_waddr),
      .accel_ax        (accel_ax),
      .accel_ay        (accel_ay)
  );

  nbody_mem #(
      .MAX_BODIES(MAX_BODIES),
      .DATA_W(DATA_W),
      .PTR_W(PTR_W)
  ) dut_mem (
      .clk             (clk),
      .cpu_body_we     (cpu_body_we),
      .cpu_body_waddr  (cpu_body_waddr),
      .cpu_x           (cpu_x),
      .cpu_y           (cpu_y),
      .cpu_m           (cpu_m),
      .cpu_vx          (cpu_vx),
      .cpu_vy          (cpu_vy),
      .body_raddr      (body_raddr),
      .body_x          (body_x),
      .body_y          (body_y),
      .body_m          (body_m),
      .body_vx         (body_vx),
      .body_vy         (body_vy),
      .body_ax         (body_ax),
      .body_ay         (body_ay),
      .body_update_we  (body_update_we),
      .body_update_addr(body_update_addr),
      .body_update_x   (body_update_x),
      .body_update_y   (body_update_y),
      .body_update_vx  (body_update_vx),
      .body_update_vy  (body_update_vy),
      .accel_we        (accel_we),
      .accel_waddr     (accel_waddr),
      .accel_ax        (accel_ax),
      .accel_ay        (accel_ay)
  );

  task automatic clear_inputs;
    begin
      go = 1'b0;
      read_enable = 1'b1;
      first_step = 1'b0;
      n_bodies = MAX_BODIES;
      gap = 32'd1;
      cpu_body_we = 1'b0;
      cpu_body_waddr = '0;
      cpu_x = '0;
      cpu_y = '0;
      cpu_m = '0;
      cpu_vx = '0;
      cpu_vy = '0;
    end
  endtask

  task automatic read_frame_file(input string fname);
    int fd;
    string line;
    int idx;
    logic [DATA_W-1:0] lpx, lpy, lvx, lvy, lm;
    int got;
    begin
      for (int i = 0; i < MAX_BODIES; i++) begin
        init_x[i] = '0;
        init_y[i] = '0;
        init_m[i] = '0;
        init_vx[i] = '0;
        init_vy[i] = '0;
        accel_seen[i] = 1'b0;
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

        if (got == 6 && idx >= 0 && idx < MAX_BODIES) begin
          init_x[idx] = lpx;
          init_y[idx] = lpy;
          init_vx[idx] = lvx;
          init_vy[idx] = lvy;
          init_m[idx] = lm;
        end
      end

      $fclose(fd);
    end
  endtask

  task automatic cpu_write_body(
    input [PTR_W-1:0] addr,
    input [DATA_W-1:0] x,
    input [DATA_W-1:0] y,
    input [DATA_W-1:0] mass,
    input [DATA_W-1:0] vx,
    input [DATA_W-1:0] vy
  );
    begin
      @(negedge clk);
      cpu_body_we = 1'b1;
      cpu_body_waddr = addr;
      cpu_x = x;
      cpu_y = y;
      cpu_m = mass;
      cpu_vx = vx;
      cpu_vy = vy;
      @(posedge clk);
      @(negedge clk);
      cpu_body_we = 1'b0;
    end
  endtask

  task automatic preload_memory;
    begin
      for (int i = 0; i < MAX_BODIES; i++) begin
        cpu_write_body(PTR_W'(i), init_x[i], init_y[i], init_m[i], init_vx[i], init_vy[i]);
      end
    end
  endtask

  task automatic pulse_go;
    begin
      @(negedge clk);
      go = 1'b1;
      @(posedge clk);
      @(negedge clk);
      go = 1'b0;
    end
  endtask

  task automatic run_and_capture;
    int cycles;
    begin
      cycles = 0;
      accel_count = 0;
      update_count = 0;

      fo = $fopen(OUT_FILE, "w");
      if (fo == 0) $fatal(1, "ERROR: cannot open OUT_FILE=%s", OUT_FILE);
      $fwrite(fo, "# i ax ay (S1E8M18 hex)\n");

      pulse_go();

      while (done !== 1'b1 && cycles < 200000) begin
        @(posedge clk);
        #1;
        cycles++;

        if (accel_we) begin
          if (accel_waddr >= MAX_BODIES) begin
            $display("ERROR accel_waddr out of range: %0d", accel_waddr);
            err_count++;
          end else begin
            if (accel_seen[accel_waddr]) begin
              $display("ERROR duplicate accel write addr=%0d", accel_waddr);
              err_count++;
            end
            accel_seen[accel_waddr] = 1'b1;
            accel_count++;
            $fwrite(fo, "%4d  %07h  %07h\n", accel_waddr, accel_ax, accel_ay);
          end
        end

        if (body_update_we) begin
          update_count++;
        end
      end

      $fclose(fo);

      if (done !== 1'b1) begin
        $display("ERROR timed out waiting for done after %0d cycles", cycles);
        err_count++;
      end

      if (accel_count != MAX_BODIES) begin
        $display("ERROR accel_count=%0d expected %0d", accel_count, MAX_BODIES);
        err_count++;
      end

      if (update_count != MAX_BODIES) begin
        $display("ERROR update_count=%0d expected %0d", update_count, MAX_BODIES);
        err_count++;
      end

      for (int i = 0; i < MAX_BODIES; i++) begin
        if (!accel_seen[i]) begin
          $display("ERROR missing accel write addr=%0d", i);
          err_count++;
        end
      end

      $display("DONE. cycles=%0d accel_count=%0d update_count=%0d wrote %s",
               cycles, accel_count, update_count, OUT_FILE);
    end
  endtask

  initial begin
    err_count = 0;
    reset = 1'b1;
    clear_inputs();
    read_frame_file(INPUT_FILE);

    repeat (4) @(posedge clk);
    preload_memory();

    @(negedge clk);
    reset = 1'b0;

    run_and_capture();

    if (err_count == 0) begin
      $display("PASS: tb_nbody_control completed without errors");
    end else begin
      $display("FAIL: tb_nbody_control saw %0d error(s)", err_count);
    end

    $finish;
  end

endmodule
