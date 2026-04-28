module nbody_mem #(
    parameter integer MAX_BODIES = 256,
    parameter integer PTR_W = 8
) (
    input  logic clk,

    // CPU writes one complete body when VY_IN is committed.
    input  logic             cpu_body_we,
    input  logic [PTR_W-1:0] cpu_body_waddr,
    input  logic [15:0]      cpu_x,
    input  logic [15:0]      cpu_y,
    input  logic [15:0]      cpu_m,
    input  logic [15:0]      cpu_vx,
    input  logic [15:0]      cpu_vy,

    // CPU output read port.
    input  logic [PTR_W-1:0] out_raddr,
    output logic [15:0]      out_x,
    output logic [15:0]      out_y,

    // Tile load read port for the force core.
    input  logic [PTR_W-1:0] tile_raddr,
    output logic [15:0]      tile_x,
    output logic [15:0]      tile_y,

    // J-body read port for the force core.
    input  logic [PTR_W-1:0] j_raddr,
    output logic [15:0]      j_x,
    output logic [15:0]      j_y,
    output logic [15:0]      j_m,

    // Integrator read port.
    input  logic [PTR_W-1:0] integ_raddr,
    output logic [15:0]      integ_x,
    output logic [15:0]      integ_y,
    output logic [15:0]      integ_vx,
    output logic [15:0]      integ_vy,
    output logic [26:0]      integ_ax,
    output logic [26:0]      integ_ay,

    // Integrator writeback port.
    input  logic             body_update_we,
    input  logic [PTR_W-1:0] body_update_addr,
    input  logic [15:0]      body_update_x,
    input  logic [15:0]      body_update_y,
    input  logic [15:0]      body_update_vx,
    input  logic [15:0]      body_update_vy,

    // Acceleration writeback ports, one per lane.
    input  logic [3:0]       accel_we,
    input  logic [PTR_W-1:0] accel_waddr [4],
    input  logic [26:0]      accel_ax    [4],
    input  logic [26:0]      accel_ay    [4]
);

    logic [15:0] x_mem  [MAX_BODIES];
    logic [15:0] y_mem  [MAX_BODIES];
    logic [15:0] m_mem  [MAX_BODIES];
    logic [15:0] vx_mem [MAX_BODIES];
    logic [15:0] vy_mem [MAX_BODIES];
    logic [26:0] ax_mem [MAX_BODIES];
    logic [26:0] ay_mem [MAX_BODIES];

    integer lane;

    always_ff @(posedge clk) begin
        if (cpu_body_we) begin
            x_mem[cpu_body_waddr]  <= cpu_x;
            y_mem[cpu_body_waddr]  <= cpu_y;
            m_mem[cpu_body_waddr]  <= cpu_m;
            vx_mem[cpu_body_waddr] <= cpu_vx;
            vy_mem[cpu_body_waddr] <= cpu_vy;
            ax_mem[cpu_body_waddr] <= 27'd0;
            ay_mem[cpu_body_waddr] <= 27'd0;
        end

        if (body_update_we) begin
            x_mem[body_update_addr]  <= body_update_x;
            y_mem[body_update_addr]  <= body_update_y;
            vx_mem[body_update_addr] <= body_update_vx;
            vy_mem[body_update_addr] <= body_update_vy;
        end

        for (lane = 0; lane < 4; lane = lane + 1) begin
            if (accel_we[lane]) begin
                ax_mem[accel_waddr[lane]] <= accel_ax[lane];
                ay_mem[accel_waddr[lane]] <= accel_ay[lane];
            end
        end
    end

    always_comb begin
        out_x = x_mem[out_raddr];
        out_y = y_mem[out_raddr];

        tile_x = x_mem[tile_raddr];
        tile_y = y_mem[tile_raddr];

        j_x = x_mem[j_raddr];
        j_y = y_mem[j_raddr];
        j_m = m_mem[j_raddr];

        integ_x  = x_mem[integ_raddr];
        integ_y  = y_mem[integ_raddr];
        integ_vx = vx_mem[integ_raddr];
        integ_vy = vy_mem[integ_raddr];
        integ_ax = ax_mem[integ_raddr];
        integ_ay = ay_mem[integ_raddr];
    end

endmodule
