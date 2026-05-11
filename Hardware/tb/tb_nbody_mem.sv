`timescale 1ns/1ps

module tb_nbody_mem;

  localparam int MAX_BODIES = 16;
  localparam int DATA_W = 27;
  localparam int PTR_W = 4;

  logic clk;

  logic             cpu_body_we;
  logic [PTR_W-1:0] cpu_body_waddr;
  logic [DATA_W-1:0] cpu_x;
  logic [DATA_W-1:0] cpu_y;
  logic [DATA_W-1:0] cpu_m;
  logic [DATA_W-1:0] cpu_vx;
  logic [DATA_W-1:0] cpu_vy;

  logic [PTR_W-1:0] body_raddr;
  logic [DATA_W-1:0] body_x;
  logic [DATA_W-1:0] body_y;
  logic [DATA_W-1:0] body_m;
  logic [DATA_W-1:0] body_vx;
  logic [DATA_W-1:0] body_vy;
  logic [DATA_W-1:0] body_ax;
  logic [DATA_W-1:0] body_ay;

  logic             body_update_we;
  logic [PTR_W-1:0] body_update_addr;
  logic [DATA_W-1:0] body_update_x;
  logic [DATA_W-1:0] body_update_y;
  logic [DATA_W-1:0] body_update_vx;
  logic [DATA_W-1:0] body_update_vy;

  logic              accel_we;
  logic [PTR_W-1:0]  accel_waddr;
  logic [DATA_W-1:0] accel_ax;
  logic [DATA_W-1:0] accel_ay;

  int err_count;

  initial clk = 1'b0;
  always #5 clk = ~clk;

  nbody_mem #(
      .MAX_BODIES(MAX_BODIES),
      .DATA_W(DATA_W),
      .PTR_W(PTR_W)
  ) dut (
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
      cpu_body_we      = 1'b0;
      cpu_body_waddr   = '0;
      cpu_x            = '0;
      cpu_y            = '0;
      cpu_m            = '0;
      cpu_vx           = '0;
      cpu_vy           = '0;
      body_raddr       = '0;
      body_update_we   = 1'b0;
      body_update_addr = '0;
      body_update_x    = '0;
      body_update_y    = '0;
      body_update_vx   = '0;
      body_update_vy   = '0;
      accel_we         = 1'b0;
      accel_waddr      = '0;
      accel_ax         = '0;
      accel_ay         = '0;
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
      cpu_body_we    = 1'b1;
      cpu_body_waddr = addr;
      cpu_x          = x;
      cpu_y          = y;
      cpu_m          = mass;
      cpu_vx         = vx;
      cpu_vy         = vy;
      @(posedge clk);
      @(negedge clk);
      cpu_body_we = 1'b0;
    end
  endtask

  task automatic update_body(
    input [PTR_W-1:0] addr,
    input [DATA_W-1:0] x,
    input [DATA_W-1:0] y,
    input [DATA_W-1:0] vx,
    input [DATA_W-1:0] vy
  );
    begin
      @(negedge clk);
      body_update_we   = 1'b1;
      body_update_addr = addr;
      body_update_x    = x;
      body_update_y    = y;
      body_update_vx   = vx;
      body_update_vy   = vy;
      @(posedge clk);
      @(negedge clk);
      body_update_we = 1'b0;
    end
  endtask

  task automatic write_accel(
    input [PTR_W-1:0] addr,
    input [DATA_W-1:0] ax,
    input [DATA_W-1:0] ay
  );
    begin
      @(negedge clk);
      accel_we    = 1'b1;
      accel_waddr = addr;
      accel_ax    = ax;
      accel_ay    = ay;
      @(posedge clk);
      @(negedge clk);
      accel_we = 1'b0;
    end
  endtask

  task automatic check_body(
    input [PTR_W-1:0] addr,
    input [DATA_W-1:0] exp_x,
    input [DATA_W-1:0] exp_y,
    input [DATA_W-1:0] exp_m,
    input [DATA_W-1:0] exp_vx,
    input [DATA_W-1:0] exp_vy,
    input [DATA_W-1:0] exp_ax,
    input [DATA_W-1:0] exp_ay
  );
    begin
      @(negedge clk);
      body_raddr = addr;
      @(posedge clk);
      #1;
      if (body_x !== exp_x || body_y !== exp_y || body_m !== exp_m ||
          body_vx !== exp_vx || body_vy !== exp_vy ||
          body_ax !== exp_ax || body_ay !== exp_ay) begin
        $display("ERROR addr=%0d got x=%07h y=%07h m=%07h vx=%07h vy=%07h ax=%07h ay=%07h",
                 addr, body_x, body_y, body_m, body_vx, body_vy, body_ax, body_ay);
        $display("              exp x=%07h y=%07h m=%07h vx=%07h vy=%07h ax=%07h ay=%07h",
                 exp_x, exp_y, exp_m, exp_vx, exp_vy, exp_ax, exp_ay);
        err_count++;
      end
    end
  endtask

  task automatic simultaneous_cpu_update_accel;
    begin
      @(negedge clk);
      cpu_body_we      = 1'b1;
      cpu_body_waddr   = 4'd7;
      cpu_x            = 27'h1111111;
      cpu_y            = 27'h1222222;
      cpu_m            = 27'h1333333;
      cpu_vx           = 27'h1444444;
      cpu_vy           = 27'h1555555;

      body_update_we   = 1'b1;
      body_update_addr = 4'd7;
      body_update_x    = 27'h2111111;
      body_update_y    = 27'h2222222;
      body_update_vx   = 27'h2444444;
      body_update_vy   = 27'h2555555;

      accel_we         = 1'b1;
      accel_waddr      = 4'd7;
      accel_ax         = 27'h2666666;
      accel_ay         = 27'h2777777;

      @(posedge clk);
      @(negedge clk);
      cpu_body_we    = 1'b0;
      body_update_we = 1'b0;
      accel_we       = 1'b0;
    end
  endtask

  initial begin
    err_count = 0;
    clear_inputs();
    repeat (2) @(posedge clk);

    cpu_write_body(4'd3, 27'h1010001, 27'h1020002, 27'h1030003, 27'h1040004, 27'h1050005);
    check_body(4'd3, 27'h1010001, 27'h1020002, 27'h1030003, 27'h1040004, 27'h1050005,
               27'h0000000, 27'h0000000);

    write_accel(4'd3, 27'h1a60006, 27'h1a70007);
    check_body(4'd3, 27'h1010001, 27'h1020002, 27'h1030003, 27'h1040004, 27'h1050005,
               27'h1a60006, 27'h1a70007);

    update_body(4'd3, 27'h2010001, 27'h2020002, 27'h2040004, 27'h2050005);
    check_body(4'd3, 27'h2010001, 27'h2020002, 27'h1030003, 27'h2040004, 27'h2050005,
               27'h1a60006, 27'h1a70007);

    cpu_write_body(4'd5, 27'h3010001, 27'h3020002, 27'h3030003, 27'h3040004, 27'h3050005);
    check_body(4'd5, 27'h3010001, 27'h3020002, 27'h3030003, 27'h3040004, 27'h3050005,
               27'h0000000, 27'h0000000);
    check_body(4'd3, 27'h2010001, 27'h2020002, 27'h1030003, 27'h2040004, 27'h2050005,
               27'h1a60006, 27'h1a70007);

    simultaneous_cpu_update_accel();
    check_body(4'd7, 27'h2111111, 27'h2222222, 27'h1333333, 27'h2444444, 27'h2555555,
               27'h2666666, 27'h2777777);

    if (err_count == 0) begin
      $display("PASS: tb_nbody_mem completed without errors");
    end else begin
      $display("FAIL: tb_nbody_mem saw %0d error(s)", err_count);
    end

    $finish;
  end

endmodule
