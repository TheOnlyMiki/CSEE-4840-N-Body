module nbody_control #(
    parameter int MAX_BODIES = 256,
    parameter int DATA_W = 27
) (
    input  logic clk,
    input  logic reset,

    input  logic        go,
    input  logic        read_enable,
    input  logic [31:0] n_bodies,
    input  logic [31:0] gap,
    output logic        done,

    output logic [$clog2(MAX_BODIES)-1:0] tile_raddr,
    input  logic [DATA_W-1:0]             tile_x,
    input  logic [DATA_W-1:0]             tile_y,

    output logic [$clog2(MAX_BODIES)-1:0] j_raddr,
    input  logic [DATA_W-1:0]             j_x,
    input  logic [DATA_W-1:0]             j_y,
    input  logic [DATA_W-1:0]             j_m,

    output logic [$clog2(MAX_BODIES)-1:0] integ_raddr,
    input  logic [DATA_W-1:0]             integ_x,
    input  logic [DATA_W-1:0]             integ_y,
    input  logic [DATA_W-1:0]             integ_vx,
    input  logic [DATA_W-1:0]             integ_vy,
    input  logic [DATA_W-1:0]             integ_ax,
    input  logic [DATA_W-1:0]             integ_ay,

    output logic                          body_update_we,
    output logic [$clog2(MAX_BODIES)-1:0] body_update_addr,
    output logic [DATA_W-1:0]             body_update_x,
    output logic [DATA_W-1:0]             body_update_y,
    output logic [DATA_W-1:0]             body_update_vx,
    output logic [DATA_W-1:0]             body_update_vy,

    output logic [3:0]                    accel_we,
    output logic [$clog2(MAX_BODIES)-1:0] accel_waddr [4],
    output logic [DATA_W-1:0]             accel_ax    [4],
    output logic [DATA_W-1:0]             accel_ay    [4]
);

    typedef enum logic [3:0] {
        ST_IDLE,
        ST_LOAD_TILE,
        ST_CLEAR_GROUP,
        ST_COMPUTE_GROUP,
        ST_DRAIN_GROUP,
        ST_STORE_GROUP,
        ST_NEXT_GROUP,
        ST_NEXT_TILE,
        ST_INTEGRATE_START,
        ST_INTEGRATE_WAIT,
        ST_DONE
    } state_t;

    localparam int PTR_W = $clog2(MAX_BODIES);
    localparam logic [PTR_W-1:0] MAX_BODY_PTR = PTR_W'(MAX_BODIES - 1);
    localparam logic [PTR_W-1:0] TILE_STRIDE  = PTR_W'(16);
    localparam int PIPE_LAT = 18;
    localparam int WAIT_W = $clog2(PIPE_LAT + 4);

    state_t state;

    logic [31:0] timestep_count;
    logic [PTR_W-1:0] tile_base;
    logic [PTR_W-1:0] j_body_idx;
    logic [PTR_W-1:0] integrate_idx;
    logic [WAIT_W-1:0] wait_count;
    logic [1:0] compute_grp;

    logic        core_clear_prev;
    logic        core_load_en;
    logic        core_compute_en;
    logic [3:0]  core_load_idx;
    logic [DATA_W-1:0] core_load_x;
    logic [DATA_W-1:0] core_load_y;
    logic [1:0]  core_grp_sel;
    logic [DATA_W-1:0] core_j_x;
    logic [DATA_W-1:0] core_j_y;
    logic [DATA_W-1:0] core_j_m;
    logic [3:0]  core_lane_mask;

    logic [DATA_W-1:0] core_res_x [4];
    logic [DATA_W-1:0] core_res_y [4];
    logic        core_res_vld;

    logic        integrator_start;
    logic        integrator_done;
    logic [DATA_W-1:0] integrator_x_out;
    logic [DATA_W-1:0] integrator_y_out;
    logic [DATA_W-1:0] integrator_vx_out;
    logic [DATA_W-1:0] integrator_vy_out;

    function automatic logic [31:0] ptr_to_u32(input logic [PTR_W-1:0] value);
        ptr_to_u32 = {{(32-PTR_W){1'b0}}, value};
    endfunction

    function automatic logic [PTR_W-1:0] lane_body_idx(
        input logic [PTR_W-1:0] base,
        input logic [1:0]       grp,
        input logic [1:0]       lane
    );
        lane_body_idx = base + PTR_W'({grp, 2'b00}) + PTR_W'(lane);
    endfunction

    function automatic logic lane_is_active(
        input logic [PTR_W-1:0] base,
        input logic [1:0]       grp,
        input logic [1:0]       lane,
        input logic [31:0]      active_bodies
    );
        logic [31:0] idx32;
        begin
            idx32 = ptr_to_u32(base) + {28'd0, grp, 2'b00} + {30'd0, lane};
            lane_is_active = (idx32 < active_bodies);
        end
    endfunction

    function automatic logic [3:0] make_lane_mask(
        input logic [PTR_W-1:0] base,
        input logic [1:0]       grp,
        input logic [PTR_W-1:0] j_idx,
        input logic [31:0]      active_bodies
    );
        logic [31:0] lane0;
        logic [31:0] lane1;
        logic [31:0] lane2;
        logic [31:0] lane3;
        begin
            lane0 = ptr_to_u32(base) + {28'd0, grp, 2'b00} + 32'd0;
            lane1 = ptr_to_u32(base) + {28'd0, grp, 2'b00} + 32'd1;
            lane2 = ptr_to_u32(base) + {28'd0, grp, 2'b00} + 32'd2;
            lane3 = ptr_to_u32(base) + {28'd0, grp, 2'b00} + 32'd3;

            make_lane_mask[0] = (lane0 >= active_bodies) || (lane0 == ptr_to_u32(j_idx));
            make_lane_mask[1] = (lane1 >= active_bodies) || (lane1 == ptr_to_u32(j_idx));
            make_lane_mask[2] = (lane2 >= active_bodies) || (lane2 == ptr_to_u32(j_idx));
            make_lane_mask[3] = (lane3 >= active_bodies) || (lane3 == ptr_to_u32(j_idx));
        end
    endfunction

    assign tile_raddr  = tile_base + PTR_W'(core_load_idx);
    assign j_raddr     = j_body_idx;
    assign integ_raddr = integrate_idx;

    four_core_wrapper #(
        .DATA_W(DATA_W)
    ) u_core (
        .i_clk       (clk),
        .i_rst       (~reset),
        .i_clear_prev(core_clear_prev),
        .i_load_en   (core_load_en),
        .i_compute_en(core_compute_en),
        .i_load_idx  (core_load_idx),
        .i_load_x    (core_load_x),
        .i_load_y    (core_load_y),
        .i_grp_sel   (core_grp_sel),
        .i_j_x       (core_j_x),
        .i_j_y       (core_j_y),
        .i_j_m       (core_j_m),
        .i_lane_mask (core_lane_mask),
        .o_res0_x    (core_res_x[0]),
        .o_res0_y    (core_res_y[0]),
        .o_res1_x    (core_res_x[1]),
        .o_res1_y    (core_res_y[1]),
        .o_res2_x    (core_res_x[2]),
        .o_res2_y    (core_res_y[2]),
        .o_res3_x    (core_res_x[3]),
        .o_res3_y    (core_res_y[3]),
        .o_res_vld   (core_res_vld)
    );

    nbody_integrator #(
        .DATA_W(DATA_W)
    ) u_integrator (
        .clk    (clk),
        .reset  (reset),
        .i_start(integrator_start),
        .o_done (integrator_done),
        .i_x    (integ_x),
        .i_y    (integ_y),
        .i_vx   (integ_vx),
        .i_vy   (integ_vy),
        .i_ax   (integ_ax),
        .i_ay   (integ_ay),
        .o_x    (integrator_x_out),
        .o_y    (integrator_y_out),
        .o_vx   (integrator_vx_out),
        .o_vy   (integrator_vy_out)
    );

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state            <= ST_IDLE;
            done             <= 1'b0;
            timestep_count   <= 32'd0;
            tile_base        <= '0;
            j_body_idx       <= '0;
            integrate_idx    <= '0;
            wait_count       <= '0;
            compute_grp      <= 2'd0;
            core_clear_prev  <= 1'b0;
            core_load_en     <= 1'b0;
            core_compute_en  <= 1'b0;
            core_load_idx    <= 4'd0;
            core_load_x      <= '0;
            core_load_y      <= '0;
            core_grp_sel     <= 2'd0;
            core_j_x         <= '0;
            core_j_y         <= '0;
            core_j_m         <= '0;
            core_lane_mask   <= 4'hF;
            integrator_start <= 1'b0;
            body_update_we   <= 1'b0;
            body_update_addr <= '0;
            body_update_x    <= '0;
            body_update_y    <= '0;
            body_update_vx   <= '0;
            body_update_vy   <= '0;
            accel_we         <= 4'd0;

            for (int lane = 0; lane < 4; lane++) begin
                accel_waddr[lane] <= '0;
                accel_ax[lane]    <= '0;
                accel_ay[lane]    <= '0;
            end
        end else begin
            core_clear_prev  <= 1'b0;
            core_load_en     <= 1'b0;
            core_compute_en  <= 1'b0;
            integrator_start <= 1'b0;
            body_update_we   <= 1'b0;
            accel_we         <= 4'd0;

            unique case (state)
                ST_IDLE: begin
                    done <= 1'b0;
                    if (go) begin
                        tile_base      <= '0;
                        core_load_idx  <= 4'd0;
                        compute_grp    <= 2'd0;
                        j_body_idx     <= '0;
                        integrate_idx  <= '0;
                        timestep_count <= 32'd0;
                        state          <= ST_LOAD_TILE;
                    end
                end

                ST_LOAD_TILE: begin
                    core_load_en  <= 1'b1;
                    core_load_idx <= core_load_idx;

                    if (ptr_to_u32(tile_base) + {28'd0, core_load_idx} < n_bodies) begin
                        core_load_x <= tile_x;
                        core_load_y <= tile_y;
                    end else begin
                        core_load_x <= '0;
                        core_load_y <= '0;
                    end

                    if (core_load_idx == 4'd15) begin
                        core_load_idx   <= 4'd0;
                        compute_grp     <= 2'd0;
                        j_body_idx      <= '0;
                        core_clear_prev <= 1'b1;
                        state           <= ST_CLEAR_GROUP;
                    end else begin
                        core_load_idx <= core_load_idx + 1'b1;
                    end
                end

                ST_CLEAR_GROUP: begin
                    core_clear_prev <= 1'b1;
                    core_grp_sel    <= compute_grp;
                    j_body_idx      <= '0;
                    wait_count      <= '0;
                    state           <= ST_COMPUTE_GROUP;
                end

                ST_COMPUTE_GROUP: begin
                    if (n_bodies == 32'd0) begin
                        integrate_idx <= '0;
                        state         <= ST_INTEGRATE_START;
                    end else begin
                        core_compute_en <= 1'b1;
                        core_grp_sel    <= compute_grp;
                        core_j_x        <= j_x;
                        core_j_y        <= j_y;
                        core_j_m        <= j_m;
                        core_lane_mask  <= make_lane_mask(tile_base, compute_grp, j_body_idx, n_bodies);

                        if ((j_body_idx == MAX_BODY_PTR) || (ptr_to_u32(j_body_idx) + 32'd1 >= n_bodies)) begin
                            wait_count <= '0;
                            state      <= ST_DRAIN_GROUP;
                        end else begin
                            j_body_idx <= j_body_idx + 1'b1;
                        end
                    end
                end

                ST_DRAIN_GROUP: begin
                    core_grp_sel <= compute_grp;

                    if (wait_count == WAIT_W'(PIPE_LAT + 1)) begin
                        state <= ST_STORE_GROUP;
                    end else begin
                        wait_count <= wait_count + 1'b1;
                    end
                end

                ST_STORE_GROUP: begin
                    core_grp_sel <= compute_grp;

                    for (int lane = 0; lane < 4; lane++) begin
                        logic [1:0] lane_sel;

                        lane_sel = lane[1:0];
                        if (lane_is_active(tile_base, compute_grp, lane_sel, n_bodies)) begin
                            accel_we[lane]    <= 1'b1;
                            accel_waddr[lane] <= lane_body_idx(tile_base, compute_grp, lane_sel);
                            accel_ax[lane]    <= core_res_x[lane];
                            accel_ay[lane]    <= core_res_y[lane];
                        end
                    end

                    state <= ST_NEXT_GROUP;
                end

                ST_NEXT_GROUP: begin
                    if (compute_grp == 2'd3) begin
                        state <= ST_NEXT_TILE;
                    end else begin
                        compute_grp <= compute_grp + 1'b1;
                        state       <= ST_CLEAR_GROUP;
                    end
                end

                ST_NEXT_TILE: begin
                    if (ptr_to_u32(tile_base) + 32'd16 >= n_bodies) begin
                        integrate_idx <= '0;
                        state         <= ST_INTEGRATE_START;
                    end else begin
                        tile_base     <= tile_base + TILE_STRIDE;
                        core_load_idx <= 4'd0;
                        compute_grp   <= 2'd0;
                        state         <= ST_LOAD_TILE;
                    end
                end

                ST_INTEGRATE_START: begin
                    if (n_bodies == 32'd0) begin
                        state <= ST_DONE;
                    end else begin
                        integrator_start <= 1'b1;
                        state            <= ST_INTEGRATE_WAIT;
                    end
                end

                ST_INTEGRATE_WAIT: begin
                    if (integrator_done) begin
                        body_update_we   <= 1'b1;
                        body_update_addr <= integrate_idx;
                        body_update_x    <= integrator_x_out;
                        body_update_y    <= integrator_y_out;
                        body_update_vx   <= integrator_vx_out;
                        body_update_vy   <= integrator_vy_out;

                        if ((integrate_idx == MAX_BODY_PTR) || (ptr_to_u32(integrate_idx) + 32'd1 >= n_bodies)) begin
                            if (timestep_count + 32'd1 >= gap) begin
                                state <= ST_DONE;
                            end else begin
                                timestep_count <= timestep_count + 32'd1;
                                tile_base      <= '0;
                                core_load_idx  <= 4'd0;
                                compute_grp    <= 2'd0;
                                state          <= ST_LOAD_TILE;
                            end
                        end else begin
                            integrate_idx <= integrate_idx + 1'b1;
                            state         <= ST_INTEGRATE_START;
                        end
                    end
                end

                ST_DONE: begin
                    done <= 1'b1;
                    if (!read_enable) begin
                        done  <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
