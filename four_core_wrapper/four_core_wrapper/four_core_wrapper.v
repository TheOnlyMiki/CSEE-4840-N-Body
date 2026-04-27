module four_core_wrapper (
    input  wire        i_clk,
    input  wire        i_rst,

    // ============================================================
    // external phase control (no internal state machine here)
    // load / compute / readout are assumed to be externally scheduled
    // and mutually well-behaved.
    // ============================================================
    input  wire        i_clear_prev,
    input  wire        i_load_en,
    input  wire        i_compute_en,

    // ============================================================
    // load 16 i-bodies into local wrapper memory
    // note: while loading a new tile, wrapper cached outputs / feedback
    // are cleared so stale results do not leak across tiles.
    // ============================================================
    input  wire [3:0]  i_load_idx,
    input  wire [15:0] i_load_x,
    input  wire [15:0] i_load_y,

    // ============================================================
    // single external group select
    // - combinationally selects which 4 i-bodies feed the datapath
    // - combinationally selects which cached 4-lane result group is read out
    // - when i_compute_en=1, it is sampled at this clock edge as the c0 group,
    //   then delayed internally to c17 for out_mem writeback address.
    // ============================================================
    input  wire [1:0]  i_grp_sel,

    // ============================================================
    // shared j input for current compute issue
    // ============================================================
    input  wire [15:0] i_j_x,
    input  wire [15:0] i_j_y,
    input  wire [15:0] i_j_m,
    input  wire [3:0]  i_lane_mask,

    // ============================================================
    // accumulated outputs
    // always come from internal out_mem, never directly from datapath
    // ============================================================
    output wire [26:0] o_res0_x,
    output wire [26:0] o_res0_y,
    output wire [26:0] o_res1_x,
    output wire [26:0] o_res1_y,
    output wire [26:0] o_res2_x,
    output wire [26:0] o_res2_y,
    output wire [26:0] o_res3_x,
    output wire [26:0] o_res3_y,

    // selected group's cache has been written at least once since last clear/load.
    // this is NOT a "final all-j done" flag; final completion is still scheduled externally.
    output wire        o_res_vld
);

    localparam integer PIPE_LAT = 18;

    // ============================================================
    // local i-body memory (16 entries)
    // ============================================================
    reg [15:0] i_x_mem [0:15];
    reg [15:0] i_y_mem [0:15];

    // ============================================================
    // internal cached accumulated outputs (real wrapper outputs)
    // ============================================================
    reg [26:0] acc_x_bank [0:15];
    reg [26:0] acc_y_bank [0:15];
    reg [3:0]  grp_written;

    // ============================================================
    // fixed feedback chain (no per-group selection here)
    // ============================================================
    reg [26:0] prev0_x_d0, prev0_x_d1, prev0_y_d0, prev0_y_d1;
    reg [26:0] prev1_x_d0, prev1_x_d1, prev1_y_d0, prev1_y_d1;
    reg [26:0] prev2_x_d0, prev2_x_d1, prev2_y_d0, prev2_y_d1;
    reg [26:0] prev3_x_d0, prev3_x_d1, prev3_y_d0, prev3_y_d1;

    // ============================================================
    // c0->c17 metadata pipe for writeback address alignment
    // the sampling point is the same posedge that launches datapath c0.
    // ============================================================
    reg       vld_pipe [0:PIPE_LAT-1];
    reg [1:0] grp_pipe [0:PIPE_LAT-1];

    integer k;
    integer ii;

    // ============================================================
    // current group selection for datapath/readout
    // ============================================================
    wire [3:0] grp_base = {i_grp_sel, 2'b00};

    wire [15:0] cur_i0_x = i_x_mem[grp_base + 4'd0];
    wire [15:0] cur_i0_y = i_y_mem[grp_base + 4'd0];
    wire [15:0] cur_i1_x = i_x_mem[grp_base + 4'd1];
    wire [15:0] cur_i1_y = i_y_mem[grp_base + 4'd1];
    wire [15:0] cur_i2_x = i_x_mem[grp_base + 4'd2];
    wire [15:0] cur_i2_y = i_y_mem[grp_base + 4'd2];
    wire [15:0] cur_i3_x = i_x_mem[grp_base + 4'd3];
    wire [15:0] cur_i3_y = i_y_mem[grp_base + 4'd3];

    // active-high mask: 1 => self/invalid/hold, so new term is nulled by mj=0
    wire [15:0] j0_m_eff = i_lane_mask[0] ? 16'd0 : i_j_m;
    wire [15:0] j1_m_eff = i_lane_mask[1] ? 16'd0 : i_j_m;
    wire [15:0] j2_m_eff = i_lane_mask[2] ? 16'd0 : i_j_m;
    wire [15:0] j3_m_eff = i_lane_mask[3] ? 16'd0 : i_j_m;

    // ============================================================
    // datapath instances
    // ============================================================
    wire [26:0] dp_out0_x, dp_out0_y;
    wire [26:0] dp_out1_x, dp_out1_y;
    wire [26:0] dp_out2_x, dp_out2_y;
    wire [26:0] dp_out3_x, dp_out3_y;

    fourcore_bcj_datapath u_dp (
        .i_clk    (i_clk),
        .i_rst    (i_rst),
        .i_j_x    (i_compute_en ? i_j_x    : 16'd0),
        .i_j_y    (i_compute_en ? i_j_y    : 16'd0),
        .i_j0_m   (i_compute_en ? j0_m_eff : 16'd0),
        .i_j1_m   (i_compute_en ? j1_m_eff : 16'd0),
        .i_j2_m   (i_compute_en ? j2_m_eff : 16'd0),
        .i_j3_m   (i_compute_en ? j3_m_eff : 16'd0),
        .i_i0_x   (i_compute_en ? cur_i0_x : 16'd0),
        .i_i0_y   (i_compute_en ? cur_i0_y : 16'd0),
        .i_prev0_x(prev0_x_d1),
        .i_prev0_y(prev0_y_d1),
        .o_out0_x (dp_out0_x),
        .o_out0_y (dp_out0_y),
        .i_i1_x   (i_compute_en ? cur_i1_x : 16'd0),
        .i_i1_y   (i_compute_en ? cur_i1_y : 16'd0),
        .i_prev1_x(prev1_x_d1),
        .i_prev1_y(prev1_y_d1),
        .o_out1_x (dp_out1_x),
        .o_out1_y (dp_out1_y),
        .i_i2_x   (i_compute_en ? cur_i2_x : 16'd0),
        .i_i2_y   (i_compute_en ? cur_i2_y : 16'd0),
        .i_prev2_x(prev2_x_d1),
        .i_prev2_y(prev2_y_d1),
        .o_out2_x (dp_out2_x),
        .o_out2_y (dp_out2_y),
        .i_i3_x   (i_compute_en ? cur_i3_x : 16'd0),
        .i_i3_y   (i_compute_en ? cur_i3_y : 16'd0),
        .i_prev3_x(prev3_x_d1),
        .i_prev3_y(prev3_y_d1),
        .o_out3_x (dp_out3_x),
        .o_out3_y (dp_out3_y)
    );

    // ============================================================
    // sequential state updates
    // ============================================================
    always @(posedge i_clk or negedge i_rst) begin
        if (!i_rst) begin
            for (k = 0; k < 16; k = k + 1) begin
                i_x_mem[k]    <= 16'd0;
                i_y_mem[k]    <= 16'd0;
                acc_x_bank[k] <= 27'd0;
                acc_y_bank[k] <= 27'd0;
            end
            grp_written <= 4'b0000;

            prev0_x_d0 <= 27'd0; prev0_x_d1 <= 27'd0; prev0_y_d0 <= 27'd0; prev0_y_d1 <= 27'd0;
            prev1_x_d0 <= 27'd0; prev1_x_d1 <= 27'd0; prev1_y_d0 <= 27'd0; prev1_y_d1 <= 27'd0;
            prev2_x_d0 <= 27'd0; prev2_x_d1 <= 27'd0; prev2_y_d0 <= 27'd0; prev2_y_d1 <= 27'd0;
            prev3_x_d0 <= 27'd0; prev3_x_d1 <= 27'd0; prev3_y_d0 <= 27'd0; prev3_y_d1 <= 27'd0;

            for (ii = 0; ii < PIPE_LAT; ii = ii + 1) begin
                vld_pipe[ii] <= 1'b0;
                grp_pipe[ii] <= 2'd0;
            end
        end else if (i_load_en) begin
            // load current i entry
            i_x_mem[i_load_idx] <= i_load_x;
            i_y_mem[i_load_idx] <= i_load_y;

            // clear wrapper-side cached outputs / feedback / metadata
            // during tile load so no stale values pollute the next compute phase.
            for (k = 0; k < 16; k = k + 1) begin
                acc_x_bank[k] <= 27'd0;
                acc_y_bank[k] <= 27'd0;
            end
            grp_written <= 4'b0000;

            prev0_x_d0 <= 27'd0; prev0_x_d1 <= 27'd0; prev0_y_d0 <= 27'd0; prev0_y_d1 <= 27'd0;
            prev1_x_d0 <= 27'd0; prev1_x_d1 <= 27'd0; prev1_y_d0 <= 27'd0; prev1_y_d1 <= 27'd0;
            prev2_x_d0 <= 27'd0; prev2_x_d1 <= 27'd0; prev2_y_d0 <= 27'd0; prev2_y_d1 <= 27'd0;
            prev3_x_d0 <= 27'd0; prev3_x_d1 <= 27'd0; prev3_y_d0 <= 27'd0; prev3_y_d1 <= 27'd0;

            for (ii = 0; ii < PIPE_LAT; ii = ii + 1) begin
                vld_pipe[ii] <= 1'b0;
                grp_pipe[ii] <= 2'd0;
            end
        end else if (i_clear_prev) begin
            // explicit clear between phases / after sufficient drain
            for (k = 0; k < 16; k = k + 1) begin
                acc_x_bank[k] <= 27'd0;
                acc_y_bank[k] <= 27'd0;
            end
            grp_written <= 4'b0000;

            prev0_x_d0 <= 27'd0; prev0_x_d1 <= 27'd0; prev0_y_d0 <= 27'd0; prev0_y_d1 <= 27'd0;
            prev1_x_d0 <= 27'd0; prev1_x_d1 <= 27'd0; prev1_y_d0 <= 27'd0; prev1_y_d1 <= 27'd0;
            prev2_x_d0 <= 27'd0; prev2_x_d1 <= 27'd0; prev2_y_d0 <= 27'd0; prev2_y_d1 <= 27'd0;
            prev3_x_d0 <= 27'd0; prev3_x_d1 <= 27'd0; prev3_y_d0 <= 27'd0; prev3_y_d1 <= 27'd0;

            for (ii = 0; ii < PIPE_LAT; ii = ii + 1) begin
                vld_pipe[ii] <= 1'b0;
                grp_pipe[ii] <= 2'd0;
            end
        end else begin
            // fixed feedback transport
            prev0_x_d1 <= prev0_x_d0; prev0_y_d1 <= prev0_y_d0; prev0_x_d0 <= dp_out0_x; prev0_y_d0 <= dp_out0_y;
            prev1_x_d1 <= prev1_x_d0; prev1_y_d1 <= prev1_y_d0; prev1_x_d0 <= dp_out1_x; prev1_y_d0 <= dp_out1_y;
            prev2_x_d1 <= prev2_x_d0; prev2_y_d1 <= prev2_y_d0; prev2_x_d0 <= dp_out2_x; prev2_y_d0 <= dp_out2_y;
            prev3_x_d1 <= prev3_x_d0; prev3_y_d1 <= prev3_y_d0; prev3_x_d0 <= dp_out3_x; prev3_y_d0 <= dp_out3_y;

            // sample current external group as the true c0 group when this issue enters core
            vld_pipe[0] <= i_compute_en;
            grp_pipe[0] <= i_grp_sel;
            for (ii = 0; ii < PIPE_LAT-1; ii = ii + 1) begin
                vld_pipe[ii+1] <= vld_pipe[ii];
                grp_pipe[ii+1] <= grp_pipe[ii];
            end

            // c17-aligned out_mem writeback; this is the real wrapper result storage
            if (vld_pipe[PIPE_LAT-1]) begin
                acc_x_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd0] <= dp_out0_x;
                acc_y_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd0] <= dp_out0_y;
                acc_x_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd1] <= dp_out1_x;
                acc_y_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd1] <= dp_out1_y;
                acc_x_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd2] <= dp_out2_x;
                acc_y_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd2] <= dp_out2_y;
                acc_x_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd3] <= dp_out3_x;
                acc_y_bank[{grp_pipe[PIPE_LAT-1], 2'b00} + 4'd3] <= dp_out3_y;
                grp_written[grp_pipe[PIPE_LAT-1]] <= 1'b1;
            end
        end
    end

    // ============================================================
    // real output is always selected from out_mem, using the same single grp_sel
    // ============================================================
    assign o_res0_x = acc_x_bank[grp_base + 4'd0];
    assign o_res0_y = acc_y_bank[grp_base + 4'd0];
    assign o_res1_x = acc_x_bank[grp_base + 4'd1];
    assign o_res1_y = acc_y_bank[grp_base + 4'd1];
    assign o_res2_x = acc_x_bank[grp_base + 4'd2];
    assign o_res2_y = acc_y_bank[grp_base + 4'd2];
    assign o_res3_x = acc_x_bank[grp_base + 4'd3];
    assign o_res3_y = acc_y_bank[grp_base + 4'd3];
    assign o_res_vld = grp_written[i_grp_sel];

endmodule
