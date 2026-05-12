module nbody_control #(
    parameter int MAX_BODIES = 1024,
    parameter int DATA_W = 27
) (
    input  logic clk,
    input  logic reset,

    input  logic        go,
    input  logic        read_enable,
    input  logic        first_step,
    input  logic [31:0] n_bodies,
    input  logic [31:0] gap,
    output logic        done,

    output logic [$clog2(MAX_BODIES)-1:0] body_raddr,
    input  logic [DATA_W-1:0]             body_x,
    input  logic [DATA_W-1:0]             body_y,
    input  logic [DATA_W-1:0]             body_m,
    input  logic [DATA_W-1:0]             body_vx,
    input  logic [DATA_W-1:0]             body_vy,
    input  logic [DATA_W-1:0]             body_ax,
    input  logic [DATA_W-1:0]             body_ay,

    output logic                          body_update_we,
    output logic [$clog2(MAX_BODIES)-1:0] body_update_addr,
    output logic [DATA_W-1:0]             body_update_x,
    output logic [DATA_W-1:0]             body_update_y,
    output logic [DATA_W-1:0]             body_update_vx,
    output logic [DATA_W-1:0]             body_update_vy,

    output logic                          accel_we,
    output logic [$clog2(MAX_BODIES)-1:0] accel_waddr,
    output logic [DATA_W-1:0]             accel_ax,
    output logic [DATA_W-1:0]             accel_ay
);

    typedef enum logic [3:0] {
        ST_IDLE,
        ST_LOAD_TILE_PRIME,
        ST_LOAD_TILE,
        ST_CLEAR_GROUP,
        ST_COMPUTE_PRIME,
        ST_COMPUTE_GROUP,
        ST_DRAIN_GROUP,
        ST_STORE_GROUP,
        ST_NEXT_GROUP,
        ST_NEXT_TILE,
        ST_INTEGRATE_PRIME,
        ST_INTEGRATE_START,
        ST_INTEGRATE_WAIT,
        ST_UPDATE_COMMIT,
        ST_DONE
    } state_t;

    typedef enum logic [1:0] {
        UPD_NEXT_BODY,
        UPD_NEXT_STEP,
        UPD_DONE
    } update_next_t;

    localparam int PTR_W = $clog2(MAX_BODIES);
    localparam logic [PTR_W-1:0] MAX_BODY_PTR = PTR_W'(MAX_BODIES - 1);
    localparam logic [PTR_W-1:0] TILE_STRIDE  = PTR_W'(16);
    localparam int ACTIVE_W = PTR_W + 1;
    localparam logic [31:0] MAX_BODIES_U32 = 32'(MAX_BODIES);
    localparam int PIPE_LAT = 18;
    localparam int WAIT_W = $clog2(PIPE_LAT + 4);

    state_t state;
    update_next_t update_next;

    logic [31:0] timestep_count;
    logic run_initial_half_step;
    logic [PTR_W-1:0] tile_base;
    logic [PTR_W-1:0] j_body_idx;
    logic [PTR_W-1:0] current_j_idx;
    logic [PTR_W-1:0] integrate_idx;
    logic [WAIT_W-1:0] wait_count;
    logic [1:0] compute_grp;
    logic [1:0] store_lane;
    logic [ACTIVE_W-1:0] active_count;
    logic [ACTIVE_W-1:0] active_count_next;
    logic core_rst_n;

    logic        core_clear_prev;
    logic        core_load_en;
    logic        core_compute_en;
    logic [3:0]  load_idx_count;
    logic [3:0]  core_load_idx;
    logic [DATA_W-1:0] core_load_x;
    logic [DATA_W-1:0] core_load_y;
    logic [1:0]  core_grp_sel;
    logic [DATA_W-1:0] core_j_x;
    logic [DATA_W-1:0] core_j_y;
    logic [DATA_W-1:0] core_j_m;
    logic [3:0]  core_lane_mask;

    logic [DATA_W-1:0] current_j_x;
    logic [DATA_W-1:0] current_j_y;
    logic [DATA_W-1:0] current_j_m;

    logic [DATA_W-1:0] core_res_x [4];
    logic [DATA_W-1:0] core_res_y [4];
    logic        unused_core_res_vld;

    logic        integrator_start;
    logic        integrator_done;
    logic        integrator_half_step;
    logic [7:0]  integrator_ax_half_exp;
    logic [7:0]  integrator_ay_half_exp;
    logic [DATA_W-1:0] integrator_ax_in;
    logic [DATA_W-1:0] integrator_ay_in;
    logic [DATA_W-1:0] integrator_x_out;
    logic [DATA_W-1:0] integrator_y_out;
    logic [DATA_W-1:0] integrator_vx_out;
    logic [DATA_W-1:0] integrator_vy_out;

    function automatic logic [ACTIVE_W-1:0] ptr_to_active(input logic [PTR_W-1:0] value);
        ptr_to_active = {{(ACTIVE_W-PTR_W){1'b0}}, value};
    endfunction

    function automatic logic [ACTIVE_W-1:0] load_idx_to_active(
        input logic [3:0] value
    );
        load_idx_to_active = ACTIVE_W'(value);
    endfunction

    function automatic logic [ACTIVE_W-1:0] lane_idx_to_active(
        input logic [PTR_W-1:0] base,
        input logic [1:0]       grp,
        input logic [1:0]       lane
    );
        lane_idx_to_active = ptr_to_active(base) + ACTIVE_W'({grp, 2'b00}) + ACTIVE_W'(lane);
    endfunction

    function automatic logic is_last_active_body(
        input logic [PTR_W-1:0] idx,
        input logic [ACTIVE_W-1:0] active_bodies
    );
        is_last_active_body = (idx == MAX_BODY_PTR) ||
                              (ptr_to_active(idx) + ACTIVE_W'(1) >= active_bodies);
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
        input logic [ACTIVE_W-1:0] active_bodies
    );
        logic [ACTIVE_W-1:0] lane_idx;
        begin
            lane_idx = lane_idx_to_active(base, grp, lane);
            lane_is_active = (lane_idx < active_bodies);
        end
    endfunction

    function automatic logic [3:0] make_lane_mask(
        input logic [PTR_W-1:0] base,
        input logic [1:0]       grp,
        input logic [PTR_W-1:0] j_idx,
        input logic [ACTIVE_W-1:0] active_bodies
    );
        logic [ACTIVE_W-1:0] lane0;
        logic [ACTIVE_W-1:0] lane1;
        logic [ACTIVE_W-1:0] lane2;
        logic [ACTIVE_W-1:0] lane3;
        logic [ACTIVE_W-1:0] j_idx_active;
        begin
            lane0 = lane_idx_to_active(base, grp, 2'd0);
            lane1 = lane_idx_to_active(base, grp, 2'd1);
            lane2 = lane_idx_to_active(base, grp, 2'd2);
            lane3 = lane_idx_to_active(base, grp, 2'd3);
            j_idx_active = ptr_to_active(j_idx);

            make_lane_mask[0] = (lane0 >= active_bodies) || (lane0 == j_idx_active);
            make_lane_mask[1] = (lane1 >= active_bodies) || (lane1 == j_idx_active);
            make_lane_mask[2] = (lane2 >= active_bodies) || (lane2 == j_idx_active);
            make_lane_mask[3] = (lane3 >= active_bodies) || (lane3 == j_idx_active);
        end
    endfunction

    always_comb begin
        if (n_bodies > MAX_BODIES_U32) begin
            active_count_next = ACTIVE_W'(MAX_BODIES);
        end else begin
            active_count_next = n_bodies[ACTIVE_W-1:0];
        end
    end

    always_comb begin
        body_raddr = '0;

        unique case (state)
            ST_LOAD_TILE_PRIME: begin
                body_raddr = tile_base + PTR_W'(load_idx_count);
            end

            ST_LOAD_TILE: begin
                if (load_idx_count == 4'd15) begin
                    body_raddr = tile_base + PTR_W'(load_idx_count);
                end else begin
                    body_raddr = tile_base + PTR_W'(load_idx_count) + PTR_W'(1);
                end
            end

            ST_COMPUTE_PRIME: begin
                body_raddr = j_body_idx;
            end

            ST_COMPUTE_GROUP: begin
                if ((compute_grp == 2'd2) &&
                    !is_last_active_body(j_body_idx, active_count)) begin
                    body_raddr = j_body_idx + 1'b1;
                end else begin
                    body_raddr = j_body_idx;
                end
            end

            ST_INTEGRATE_PRIME,
            ST_INTEGRATE_START,
            ST_INTEGRATE_WAIT: begin
                body_raddr = integrate_idx;
            end

            default: begin
                body_raddr = '0;
            end
        endcase
    end

    // Leapfrog starts with one half-step velocity kick. Later kicks are full-step.
    assign integrator_half_step    = run_initial_half_step && (timestep_count == 32'd0);
    assign integrator_ax_half_exp  = (body_ax[25:18] > 8'd1) ? (body_ax[25:18] - 8'd1) : 8'd0;
    assign integrator_ay_half_exp  = (body_ay[25:18] > 8'd1) ? (body_ay[25:18] - 8'd1) : 8'd0;
    assign integrator_ax_in        = integrator_half_step ? {body_ax[26], integrator_ax_half_exp, body_ax[17:0]} : body_ax;
    assign integrator_ay_in        = integrator_half_step ? {body_ay[26], integrator_ay_half_exp, body_ay[17:0]} : body_ay;

    four_core_wrapper #(
        .DATA_W(DATA_W)
    ) u_core (
        .i_clk       (clk),
        .i_rst       (core_rst_n),
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
        .o_res_vld   (unused_core_res_vld)
    );

    nbody_integrator #(
        .DATA_W(DATA_W)
    ) u_integrator (
        .clk        (clk),
        .reset      (reset),
        .i_start    (integrator_start),
        .o_done     (integrator_done),
        .i_x        (body_x),
        .i_y        (body_y),
        .i_vx       (body_vx),
        .i_vy       (body_vy),
        .i_ax       (integrator_ax_in),
        .i_ay       (integrator_ay_in),
        .o_x        (integrator_x_out),
        .o_y        (integrator_y_out),
        .o_vx       (integrator_vx_out),
        .o_vy       (integrator_vy_out)
    );

    always_ff @(posedge clk) begin
        if (reset) begin
            core_rst_n            <= 1'b0;
            state                 <= ST_IDLE;
            update_next           <= UPD_DONE;
            done                  <= 1'b0;
            timestep_count        <= 32'd0;
            run_initial_half_step <= 1'b0;
            tile_base             <= '0;
            j_body_idx            <= '0;
            current_j_idx         <= '0;
            integrate_idx         <= '0;
            wait_count            <= '0;
            compute_grp           <= 2'd0;
            store_lane            <= 2'd0;
            active_count          <= '0;
            core_clear_prev       <= 1'b0;
            core_load_en          <= 1'b0;
            core_compute_en       <= 1'b0;
            load_idx_count        <= 4'd0;
            core_load_idx         <= 4'd0;
            core_load_x           <= '0;
            core_load_y           <= '0;
            core_grp_sel          <= 2'd0;
            core_j_x              <= '0;
            core_j_y              <= '0;
            core_j_m              <= '0;
            core_lane_mask        <= 4'hF;
            current_j_x           <= '0;
            current_j_y           <= '0;
            current_j_m           <= '0;
            integrator_start      <= 1'b0;
            body_update_we        <= 1'b0;
            body_update_addr      <= '0;
            body_update_x         <= '0;
            body_update_y         <= '0;
            body_update_vx        <= '0;
            body_update_vy        <= '0;
            accel_we              <= 1'b0;
            accel_waddr           <= '0;
            accel_ax              <= '0;
            accel_ay              <= '0;
        end else begin
            core_clear_prev  <= 1'b0;
            core_load_en     <= 1'b0;
            core_compute_en  <= 1'b0;
            integrator_start <= 1'b0;
            body_update_we   <= 1'b0;
            accel_we         <= 1'b0;
            core_rst_n       <= 1'b1;

            unique case (state)
                ST_IDLE: begin
                    done <= 1'b0;
                    if (go) begin
                        tile_base             <= '0;
                        load_idx_count        <= 4'd0;
                        core_load_idx         <= 4'd0;
                        compute_grp           <= 2'd0;
                        j_body_idx            <= '0;
                        current_j_idx         <= '0;
                        integrate_idx         <= '0;
                        timestep_count        <= 32'd0;
                        run_initial_half_step <= first_step;
                        active_count          <= active_count_next;
                        if (active_count_next == '0) begin
                            state <= ST_DONE;
                        end else begin
                            state <= ST_LOAD_TILE_PRIME;
                        end
                    end
                end

                ST_LOAD_TILE_PRIME: begin
                    state <= ST_LOAD_TILE;
                end

                ST_LOAD_TILE: begin
                    core_load_en  <= 1'b1;
                    core_load_idx <= load_idx_count;

                    if (ptr_to_active(tile_base) + load_idx_to_active(load_idx_count) < active_count) begin
                        core_load_x <= body_x;
                        core_load_y <= body_y;
                    end else begin
                        core_load_x <= '0;
                        core_load_y <= '0;
                    end

                    if (load_idx_count == 4'd15) begin
                        load_idx_count  <= 4'd0;
                        compute_grp     <= 2'd0;
                        j_body_idx      <= '0;
                        state           <= ST_CLEAR_GROUP;
                    end else begin
                        load_idx_count <= load_idx_count + 1'b1;
                    end
                end

                ST_CLEAR_GROUP: begin
                    core_clear_prev <= 1'b1;
                    core_grp_sel    <= 2'd0;
                    j_body_idx      <= '0;
                    current_j_idx   <= '0;
                    wait_count      <= '0;
                    state           <= ST_COMPUTE_PRIME;
                end

                ST_COMPUTE_PRIME: begin
                    current_j_x   <= body_x;
                    current_j_y   <= body_y;
                    current_j_m   <= body_m;
                    current_j_idx <= j_body_idx;
                    compute_grp   <= 2'd0;
                    state <= ST_COMPUTE_GROUP;
                end

                ST_COMPUTE_GROUP: begin
                    core_compute_en <= 1'b1;
                    core_grp_sel    <= compute_grp;
                    core_j_x        <= current_j_x;
                    core_j_y        <= current_j_y;
                    core_j_m        <= current_j_m;
                    core_lane_mask  <= make_lane_mask(tile_base, compute_grp, current_j_idx, active_count);

                    if (compute_grp == 2'd3) begin
                        if (is_last_active_body(j_body_idx, active_count)) begin
                            wait_count <= '0;
                            state      <= ST_DRAIN_GROUP;
                        end else begin
                            j_body_idx    <= j_body_idx + 1'b1;
                            current_j_idx <= j_body_idx + 1'b1;
                            current_j_x   <= body_x;
                            current_j_y   <= body_y;
                            current_j_m   <= body_m;
                            compute_grp   <= 2'd0;
                        end
                    end else begin
                        compute_grp <= compute_grp + 1'b1;
                    end
                end

                ST_DRAIN_GROUP: begin
                    core_grp_sel <= 2'd0;

                    if (wait_count == WAIT_W'(PIPE_LAT + 1)) begin
                        compute_grp <= 2'd0;
                        store_lane <= 2'd0;
                        state      <= ST_STORE_GROUP;
                    end else begin
                        wait_count <= wait_count + 1'b1;
                    end
                end

                ST_STORE_GROUP: begin
                    core_grp_sel <= compute_grp;

                    if (lane_is_active(tile_base, compute_grp, store_lane, active_count)) begin
                        accel_we    <= 1'b1;
                        accel_waddr <= lane_body_idx(tile_base, compute_grp, store_lane);
                        accel_ax    <= core_res_x[store_lane];
                        accel_ay    <= core_res_y[store_lane];
                    end

                    if (store_lane == 2'd3) begin
                        state <= ST_NEXT_GROUP;
                    end else begin
                        store_lane <= store_lane + 1'b1;
                    end
                end

                ST_NEXT_GROUP: begin
                    if (compute_grp == 2'd3) begin
                        state <= ST_NEXT_TILE;
                    end else begin
                        compute_grp <= compute_grp + 1'b1;
                        core_grp_sel <= compute_grp + 1'b1;
                        store_lane  <= 2'd0;
                        state       <= ST_STORE_GROUP;
                    end
                end

                ST_NEXT_TILE: begin
                    if (ptr_to_active(tile_base) + ACTIVE_W'(16) >= active_count) begin
                        integrate_idx <= '0;
                        state         <= ST_INTEGRATE_PRIME;
                    end else begin
                        tile_base     <= tile_base + TILE_STRIDE;
                        load_idx_count <= 4'd0;
                        core_load_idx <= 4'd0;
                        compute_grp   <= 2'd0;
                        j_body_idx    <= '0;
                        current_j_idx <= '0;
                        state         <= ST_LOAD_TILE_PRIME;
                    end
                end

                ST_INTEGRATE_PRIME: begin
                    state <= ST_INTEGRATE_START;
                end

                ST_INTEGRATE_START: begin
                    integrator_start <= 1'b1;
                    state            <= ST_INTEGRATE_WAIT;
                end

                ST_INTEGRATE_WAIT: begin
                    if (integrator_done) begin
                        body_update_we   <= 1'b1;
                        body_update_addr <= integrate_idx;
                        body_update_x    <= integrator_x_out;
                        body_update_y    <= integrator_y_out;
                        body_update_vx   <= integrator_vx_out;
                        body_update_vy   <= integrator_vy_out;

                        if (is_last_active_body(integrate_idx, active_count)) begin
                            if (timestep_count + 32'd1 >= gap) begin
                                update_next <= UPD_DONE;
                            end else begin
                                update_next    <= UPD_NEXT_STEP;
                                timestep_count <= timestep_count + 1'b1;
                            end
                        end else begin
                            update_next   <= UPD_NEXT_BODY;
                            integrate_idx <= integrate_idx + 1'b1;
                        end

                        state <= ST_UPDATE_COMMIT;
                    end
                end

                ST_UPDATE_COMMIT: begin
                    unique case (update_next)
                        UPD_NEXT_BODY: begin
                            state <= ST_INTEGRATE_PRIME;
                        end

                        UPD_NEXT_STEP: begin
                            tile_base     <= '0;
                            load_idx_count <= 4'd0;
                            core_load_idx <= 4'd0;
                            compute_grp   <= 2'd0;
                            j_body_idx    <= '0;
                            current_j_idx <= '0;
                            integrate_idx <= '0;
                            state         <= ST_LOAD_TILE_PRIME;
                        end

                        default: begin
                            state <= ST_DONE;
                        end
                    endcase
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
