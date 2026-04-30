// Avalon-MM top-level shell for the N-body accelerator.
//
// Register map, 32-bit words:
//   0x00 GO        W  Pulse high to start computation. Also resets input/output
//                  body pointers to 0.
//   0x01 N_BODIES  W  Number of active bodies in the simulation.
//   0x02 GAP       W  Number of timesteps executed internally between DONE pulses.
//   0x03 X_IN      W  Input X position for the current body.
//   0x04 Y_IN      W  Input Y position for the current body.
//   0x05 M_IN      W  Input mass for the current body.
//   0x06 VX_IN     W  Input X velocity for the current body.
//   0x07 VY_IN     W  Input Y velocity for the current body. Writing this register
//                  commits the current body and increments the input pointer.
//   0x08 DONE      R  High when GAP timesteps have completed.
//   0x09 READ      W  Write 1 after DONE is observed. Write 0 after reading all
//                  outputs; this clears DONE and arms the next GO.
//   0x0A OUT_X     R  Output X position for the current output body.
//   0x0B OUT_Y     R  Output Y position for the current output body. Reading this
//                  register increments the output pointer.
// Data payloads use the low 27 bits of each 32-bit Avalon word.

module nbody_accel_avmm #(
    parameter int MAX_BODIES = 256
) (
    input  logic        clk,
    input  logic        reset,       // active-high Platform Designer reset

    input  logic        chipselect,
    input  logic        read,
    input  logic        write,
    input  logic [7:0]  address,
    input  logic [31:0] writedata,
    output logic [31:0] readdata
);

    localparam logic [7:0] REG_GO       = 8'h00;
    localparam logic [7:0] REG_N_BODIES = 8'h01;
    localparam logic [7:0] REG_GAP      = 8'h02;
    localparam logic [7:0] REG_X_IN     = 8'h03;
    localparam logic [7:0] REG_Y_IN     = 8'h04;
    localparam logic [7:0] REG_M_IN     = 8'h05;
    localparam logic [7:0] REG_VX_IN    = 8'h06;
    localparam logic [7:0] REG_VY_IN    = 8'h07;
    localparam logic [7:0] REG_DONE     = 8'h08;
    localparam logic [7:0] REG_READ     = 8'h09;
    localparam logic [7:0] REG_OUT_X    = 8'h0A;
    localparam logic [7:0] REG_OUT_Y    = 8'h0B;

    localparam int DATA_W = 27;
    localparam int PAD_W = 32 - DATA_W;
    localparam int PTR_W = $clog2(MAX_BODIES);
    localparam logic [PTR_W-1:0] MAX_BODY_PTR = PTR_W'(MAX_BODIES - 1);

    logic        go_pulse;
    logic        read_reg;
    logic        done;

    logic [31:0] n_bodies_reg;
    logic [31:0] gap_reg;

    logic [PTR_W-1:0] input_ptr;
    logic [PTR_W-1:0] output_ptr;

    logic [DATA_W-1:0] x_in_shadow;
    logic [DATA_W-1:0] y_in_shadow;
    logic [DATA_W-1:0] m_in_shadow;
    logic [DATA_W-1:0] vx_in_shadow;
    logic [DATA_W-1:0] vy_in_shadow;

    logic             cpu_body_we;
    logic [PTR_W-1:0] cpu_body_waddr;
    logic [PTR_W-1:0] control_body_raddr;
    logic [PTR_W-1:0] mem_body_raddr;
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

    assign mem_body_raddr = done ? output_ptr : control_body_raddr;

    nbody_mem #(
        .MAX_BODIES(MAX_BODIES),
        .DATA_W(DATA_W),
        .PTR_W(PTR_W)
    ) u_mem (
        .clk             (clk),

        .cpu_body_we     (cpu_body_we),
        .cpu_body_waddr  (cpu_body_waddr),
        .cpu_x           (x_in_shadow),
        .cpu_y           (y_in_shadow),
        .cpu_m           (m_in_shadow),
        .cpu_vx          (vx_in_shadow),
        .cpu_vy          (vy_in_shadow),

        .body_raddr      (mem_body_raddr),
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

    nbody_control #(
        .MAX_BODIES(MAX_BODIES),
        .DATA_W(DATA_W)
    ) u_control (
        .clk             (clk),
        .reset           (reset),

        .go              (go_pulse),
        .read_enable     (read_reg),
        .n_bodies        (n_bodies_reg),
        .gap             (gap_reg),
        .done            (done),

        .body_raddr      (control_body_raddr),
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

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            go_pulse      <= 1'b0;
            read_reg      <= 1'b0;
            n_bodies_reg  <= 32'd0;
            gap_reg       <= 32'd0;
            input_ptr     <= '0;
            output_ptr    <= '0;
            x_in_shadow   <= '0;
            y_in_shadow   <= '0;
            m_in_shadow   <= '0;
            vx_in_shadow  <= '0;
            vy_in_shadow  <= '0;
            cpu_body_we    <= 1'b0;
            cpu_body_waddr <= '0;
        end else begin
            go_pulse    <= 1'b0;
            cpu_body_we <= 1'b0;

            if (chipselect && write) begin
                unique case (address)
                    REG_GO: begin
                        if (writedata[0]) begin
                            go_pulse   <= 1'b1;
                            read_reg   <= 1'b1;
                            input_ptr  <= '0;
                            output_ptr <= '0;
                        end
                    end

                    REG_N_BODIES: n_bodies_reg <= writedata;
                    REG_GAP:      gap_reg      <= writedata;
                    REG_X_IN:     x_in_shadow  <= writedata[DATA_W-1:0];
                    REG_Y_IN:     y_in_shadow  <= writedata[DATA_W-1:0];
                    REG_M_IN:     m_in_shadow  <= writedata[DATA_W-1:0];
                    REG_VX_IN:    vx_in_shadow <= writedata[DATA_W-1:0];

                    REG_VY_IN: begin
                        vy_in_shadow  <= writedata[DATA_W-1:0];
                        cpu_body_we    <= 1'b1;
                        cpu_body_waddr <= input_ptr;

                        if (input_ptr != MAX_BODY_PTR) begin
                            input_ptr <= input_ptr + 1'b1;
                        end
                    end

                    REG_READ: begin
                        read_reg <= writedata[0];
                        if (!writedata[0]) begin
                            output_ptr <= '0;
                        end
                    end

                    default: begin
                    end
                endcase
            end

            if (chipselect && read && (address == REG_OUT_Y)) begin
                if (output_ptr != MAX_BODY_PTR) begin
                    output_ptr <= output_ptr + 1'b1;
                end
            end
        end
    end

    always_comb begin
        unique case (address)
            REG_DONE:  readdata = {31'd0, done};
            REG_OUT_X: readdata = {{PAD_W{1'b0}}, body_x};
            REG_OUT_Y: readdata = {{PAD_W{1'b0}}, body_y};
            default:   readdata = 32'd0;
        endcase
    end

endmodule
