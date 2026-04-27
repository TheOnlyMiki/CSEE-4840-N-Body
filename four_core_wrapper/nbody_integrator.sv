// Integration Unit
//
// One-body update:
//   vx' = vx + ax
//   vy' = vy + ay
//   x'  = x  + vx'
//   y'  = y  + vy'
//
// Inputs x/y/vx/vy are 16-bit S1E8M7. Accelerations ax/ay are 27-bit S1E8M18.
// Outputs are converted back to 16-bit S1E8M7.

module nbody_integrator (
    input  logic        clk,
    input  logic        reset,   // active-high reset

    input  logic        i_start,
    output logic        o_done,

    input  logic [15:0] i_x,
    input  logic [15:0] i_y,
    input  logic [15:0] i_vx,
    input  logic [15:0] i_vy,
    input  logic [26:0] i_ax,
    input  logic [26:0] i_ay,

    output logic [15:0] o_x,
    output logic [15:0] o_y,
    output logic [15:0] o_vx,
    output logic [15:0] o_vy
);

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_WAIT_V,
        ST_WAIT_XY,
        ST_DONE
    } state_t;

    state_t state;
    logic [2:0] wait_count;

    logic [26:0] x_27;
    logic [26:0] y_27;
    logic [26:0] vx_27;
    logic [26:0] vy_27;
    logic [26:0] ax_27;
    logic [26:0] ay_27;

    logic [26:0] vx_new_27;
    logic [26:0] vy_new_27;
    logic [26:0] x_new_27;
    logic [26:0] y_new_27;

    function automatic logic [26:0] fp16_to_fp27(input logic [15:0] value);
        fp16_to_fp27 = (value[14:0] == 15'd0) ? 27'd0 :
                       {value[15], value[14:7], value[6:0], 11'd0};
    endfunction

    function automatic logic [15:0] fp27_to_fp16(input logic [26:0] value);
        fp27_to_fp16 = (value[25:18] == 8'd0) ? 16'd0 :
                       {value[26], value[25:18], value[17:11]};
    endfunction

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
            x_27       <= 27'd0;
            y_27       <= 27'd0;
            vx_27      <= 27'd0;
            vy_27      <= 27'd0;
            ax_27      <= 27'd0;
            ay_27      <= 27'd0;
            o_x        <= 16'd0;
            o_y        <= 16'd0;
            o_vx       <= 16'd0;
            o_vy       <= 16'd0;
        end else begin
            o_done <= 1'b0;

            unique case (state)
                ST_IDLE: begin
                    if (i_start) begin
                        x_27       <= fp16_to_fp27(i_x);
                        y_27       <= fp16_to_fp27(i_y);
                        vx_27      <= fp16_to_fp27(i_vx);
                        vy_27      <= fp16_to_fp27(i_vy);
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
                        o_vx  <= fp27_to_fp16(vx_new_27);
                        o_vy  <= fp27_to_fp16(vy_new_27);
                        o_x   <= fp27_to_fp16(x_new_27);
                        o_y   <= fp27_to_fp16(y_new_27);
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
