// Integration unit
//
// One-body update:
//   vx' = vx + ax
//   vy' = vy + ay
//   x'  = x  + vx'
//   y'  = y  + vy'
//
// The control unit supplies ax/ay as either a half-step or full-step
// acceleration increment for leapfrog integration.
//
// All numeric inputs and outputs use the 27-bit S1E8M18 core format.

module nbody_integrator #(
    parameter int DATA_W = 27
) (
    input  logic        clk,
    input  logic        reset,   // active-high reset

    input  logic        i_start,
    output logic        o_done,

    input  logic [DATA_W-1:0] i_x,
    input  logic [DATA_W-1:0] i_y,
    input  logic [DATA_W-1:0] i_vx,
    input  logic [DATA_W-1:0] i_vy,
    input  logic [DATA_W-1:0] i_ax,
    input  logic [DATA_W-1:0] i_ay,

    output logic [DATA_W-1:0] o_x,
    output logic [DATA_W-1:0] o_y,
    output logic [DATA_W-1:0] o_vx,
    output logic [DATA_W-1:0] o_vy
);

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_WAIT_V,
        ST_WAIT_XY,
        ST_DONE
    } state_t;

    state_t state;
    logic [2:0] wait_count;

    logic [DATA_W-1:0] x_27;
    logic [DATA_W-1:0] y_27;
    logic [DATA_W-1:0] vx_27;
    logic [DATA_W-1:0] vy_27;
    logic [DATA_W-1:0] ax_27;
    logic [DATA_W-1:0] ay_27;

    logic [DATA_W-1:0] vx_new_27;
    logic [DATA_W-1:0] vy_new_27;
    logic [DATA_W-1:0] x_new_27;
    logic [DATA_W-1:0] y_new_27;

    FpAdd u_add_vx (
        .iCLK(clk),
        .iA  (vx_27),
        .iB  (ax_27),
        .oSum(vx_new_27)
    );

    FpAdd u_add_vy (
        .iCLK(clk),
        .iA  (vy_27),
        .iB  (ay_27),
        .oSum(vy_new_27)
    );

    FpAdd u_add_x (
        .iCLK(clk),
        .iA  (x_27),
        .iB  (vx_new_27),
        .oSum(x_new_27)
    );

    FpAdd u_add_y (
        .iCLK(clk),
        .iA  (y_27),
        .iB  (vy_new_27),
        .oSum(y_new_27)
    );

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state      <= ST_IDLE;
            wait_count <= 3'd0;
            o_done     <= 1'b0;
            x_27       <= '0;
            y_27       <= '0;
            vx_27      <= '0;
            vy_27      <= '0;
            ax_27      <= '0;
            ay_27      <= '0;
            o_x        <= '0;
            o_y        <= '0;
            o_vx       <= '0;
            o_vy       <= '0;
        end else begin
            o_done <= 1'b0;

            unique case (state)
                ST_IDLE: begin
                    if (i_start) begin
                        x_27       <= i_x;
                        y_27       <= i_y;
                        vx_27      <= i_vx;
                        vy_27      <= i_vy;
                        ax_27      <= i_ax;
                        ay_27      <= i_ay;
                        wait_count <= 3'd0;
                        state      <= ST_WAIT_V;
                    end
                end

                ST_WAIT_V: begin
                    if (wait_count == 3'd3) begin
                        wait_count <= 3'd0;
                        state      <= ST_WAIT_XY;
                    end else begin
                        wait_count <= wait_count + 1'b1;
                    end
                end

                ST_WAIT_XY: begin
                    if (wait_count == 3'd3) begin
                        o_vx  <= vx_new_27;
                        o_vy  <= vy_new_27;
                        o_x   <= x_new_27;
                        o_y   <= y_new_27;
                        state <= ST_DONE;
                    end else begin
                        wait_count <= wait_count + 1'b1;
                    end
                end

                ST_DONE: begin
                    o_done <= 1'b1;
                    state  <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
