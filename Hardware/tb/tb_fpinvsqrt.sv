`timescale 1ns/1ps

module tb_fpinvsqrt;

    reg         iCLK;
    reg  [26:0] iA;
    wire [26:0] oInvSqrt;
    reg  [26:0] exp_out;

    integer fd_in;
    integer ret;
    integer case_idx;
    integer pass_cnt;
    integer fail_cnt;
    integer k;

    // conservative wait; if your RTL is deeper, bump this up
    localparam integer SETTLE_CYCLES = 8;

    // --------------------------------------------------------
    // DUT
    // rename ports here if your RTL uses different names
    // --------------------------------------------------------
    FpInvSqrt dut (
        .iCLK    (iCLK),
        .iA      (iA),
        .oInvSqrt(oInvSqrt)
    );

    // --------------------------------------------------------
    // clock
    // --------------------------------------------------------
    initial begin
        iCLK = 1'b0;
        forever #5 iCLK = ~iCLK;
    end

    // --------------------------------------------------------
    // main
    // --------------------------------------------------------
    initial begin
        iA = 27'd0;
        exp_out = 27'd0;
        case_idx = 0;
        pass_cnt = 0;
        fail_cnt = 0;

        fd_in = $fopen("tb/frame_input/fpinvsqrt_cases.txt", "r");
        if (fd_in == 0) begin
            $display("ERROR: cannot open fpinvsqrt_cases.txt");
            $finish;
        end

        // let pipeline settle
        repeat (3) @(posedge iCLK);

        while (!$feof(fd_in)) begin
            ret = $fscanf(fd_in, "%h %h\n", iA, exp_out);

            if (ret == 2) begin
                // hold this input constant long enough
                for (k = 0; k < SETTLE_CYCLES; k = k + 1)
                    @(posedge iCLK);
                #1;

                if (oInvSqrt === exp_out) begin
                    pass_cnt = pass_cnt + 1;
                    $display("[%0d] x=%07h rtl=%07h py=%07h PASS",
                             case_idx, iA, oInvSqrt, exp_out);
                end
                else begin
                    fail_cnt = fail_cnt + 1;
                    $display("[%0d] x=%07h rtl=%07h py=%07h FAIL",
                             case_idx, iA, oInvSqrt, exp_out);

                    // first-fail stop is very useful here
                    $display("---- first fail detail ----");
                    $display("x      = %07h", iA);
                    $display("rtl    = %07h", oInvSqrt);
                    $display("python = %07h", exp_out);
                    $stop;
                end

                case_idx = case_idx + 1;
            end
            else begin
                ret = $fgetc(fd_in);
            end
        end

        $display("========================================");
        $display("tb_fpinvsqrt done");
        $display("total = %0d, pass = %0d, fail = %0d",
                 case_idx, pass_cnt, fail_cnt);
        $display("========================================");

        $fclose(fd_in);
        $finish;
    end

endmodule
