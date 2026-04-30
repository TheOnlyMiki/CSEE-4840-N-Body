module nbody_mem #(
    parameter int MAX_BODIES = 256,
    parameter int DATA_W = 27,
    parameter int PTR_W = 8
) (
    input  logic clk,

    // CPU writes one complete body when VY_IN is committed.
    input  logic             cpu_body_we,
    input  logic [PTR_W-1:0] cpu_body_waddr,
    input  logic [DATA_W-1:0] cpu_x,
    input  logic [DATA_W-1:0] cpu_y,
    input  logic [DATA_W-1:0] cpu_m,
    input  logic [DATA_W-1:0] cpu_vx,
    input  logic [DATA_W-1:0] cpu_vy,

    // Shared synchronous body read port. Data is valid one clock after body_raddr
    // is sampled.
    input  logic [PTR_W-1:0] body_raddr,
    output logic [DATA_W-1:0] body_x,
    output logic [DATA_W-1:0] body_y,
    output logic [DATA_W-1:0] body_m,
    output logic [DATA_W-1:0] body_vx,
    output logic [DATA_W-1:0] body_vy,
    output logic [DATA_W-1:0] body_ax,
    output logic [DATA_W-1:0] body_ay,

    // Integrator writeback port.
    input  logic             body_update_we,
    input  logic [PTR_W-1:0] body_update_addr,
    input  logic [DATA_W-1:0] body_update_x,
    input  logic [DATA_W-1:0] body_update_y,
    input  logic [DATA_W-1:0] body_update_vx,
    input  logic [DATA_W-1:0] body_update_vy,

    // Serialized acceleration writeback port.
    input  logic              accel_we,
    input  logic [PTR_W-1:0]  accel_waddr,
    input  logic [DATA_W-1:0] accel_ax,
    input  logic [DATA_W-1:0] accel_ay
);

    (* ramstyle = "M10K" *) logic [DATA_W-1:0] x_mem  [0:MAX_BODIES-1];
    (* ramstyle = "M10K" *) logic [DATA_W-1:0] y_mem  [0:MAX_BODIES-1];
    (* ramstyle = "M10K" *) logic [DATA_W-1:0] m_mem  [0:MAX_BODIES-1];
    (* ramstyle = "M10K" *) logic [DATA_W-1:0] vx_mem [0:MAX_BODIES-1];
    (* ramstyle = "M10K" *) logic [DATA_W-1:0] vy_mem [0:MAX_BODIES-1];
    (* ramstyle = "M10K" *) logic [DATA_W-1:0] ax_mem [0:MAX_BODIES-1];
    (* ramstyle = "M10K" *) logic [DATA_W-1:0] ay_mem [0:MAX_BODIES-1];

    logic             x_we;
    logic             y_we;
    logic             m_we;
    logic             vx_we;
    logic             vy_we;
    logic             ax_we;
    logic             ay_we;
    logic [PTR_W-1:0] x_waddr;
    logic [PTR_W-1:0] y_waddr;
    logic [PTR_W-1:0] m_waddr;
    logic [PTR_W-1:0] vx_waddr;
    logic [PTR_W-1:0] vy_waddr;
    logic [PTR_W-1:0] ax_waddr;
    logic [PTR_W-1:0] ay_waddr;
    logic [DATA_W-1:0] x_wdata;
    logic [DATA_W-1:0] y_wdata;
    logic [DATA_W-1:0] m_wdata;
    logic [DATA_W-1:0] vx_wdata;
    logic [DATA_W-1:0] vy_wdata;
    logic [DATA_W-1:0] ax_wdata;
    logic [DATA_W-1:0] ay_wdata;

    always_comb begin
        x_we    = cpu_body_we;
        y_we    = cpu_body_we;
        m_we    = cpu_body_we;
        vx_we   = cpu_body_we;
        vy_we   = cpu_body_we;
        ax_we   = cpu_body_we;
        ay_we   = cpu_body_we;
        x_waddr = cpu_body_waddr;
        y_waddr = cpu_body_waddr;
        m_waddr = cpu_body_waddr;
        vx_waddr = cpu_body_waddr;
        vy_waddr = cpu_body_waddr;
        ax_waddr = cpu_body_waddr;
        ay_waddr = cpu_body_waddr;
        x_wdata = cpu_x;
        y_wdata = cpu_y;
        m_wdata = cpu_m;
        vx_wdata = cpu_vx;
        vy_wdata = cpu_vy;
        ax_wdata = '0;
        ay_wdata = '0;

        if (body_update_we) begin
            x_we    = 1'b1;
            y_we    = 1'b1;
            vx_we   = 1'b1;
            vy_we   = 1'b1;
            x_waddr = body_update_addr;
            y_waddr = body_update_addr;
            vx_waddr = body_update_addr;
            vy_waddr = body_update_addr;
            x_wdata = body_update_x;
            y_wdata = body_update_y;
            vx_wdata = body_update_vx;
            vy_wdata = body_update_vy;
        end

        if (accel_we) begin
            ax_we    = 1'b1;
            ay_we    = 1'b1;
            ax_waddr = accel_waddr;
            ay_waddr = accel_waddr;
            ax_wdata = accel_ax;
            ay_wdata = accel_ay;
        end
    end

    always_ff @(posedge clk) begin
        body_x  <= x_mem[body_raddr];
        body_y  <= y_mem[body_raddr];
        body_m  <= m_mem[body_raddr];
        body_vx <= vx_mem[body_raddr];
        body_vy <= vy_mem[body_raddr];
        body_ax <= ax_mem[body_raddr];
        body_ay <= ay_mem[body_raddr];

        if (x_we)  x_mem[x_waddr]   <= x_wdata;
        if (y_we)  y_mem[y_waddr]   <= y_wdata;
        if (m_we)  m_mem[m_waddr]   <= m_wdata;
        if (vx_we) vx_mem[vx_waddr] <= vx_wdata;
        if (vy_we) vy_mem[vy_waddr] <= vy_wdata;
        if (ax_we) ax_mem[ax_waddr] <= ax_wdata;
        if (ay_we) ay_mem[ay_waddr] <= ay_wdata;
    end

endmodule
