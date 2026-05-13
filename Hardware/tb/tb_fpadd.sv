`timescale 1ns/1ps

module tb_fpadd;

    reg         iCLK;
    reg  [26:0] iA;
    reg  [26:0] iB;
    wire [26:0] oSum;
    reg  [26:0] exp_out;

    integer fd_in;
    integer ret;
    integer case_idx;
    integer pass_cnt;
    integer fail_cnt;

    FpAdd dut (
        .iCLK(iCLK),
        .iA  (iA),
        .iB  (iB),
        .oSum(oSum)
    );

    initial begin
        iCLK = 0;
        forever #5 iCLK = ~iCLK;
    end

    initial begin
        iA = 0;
        iB = 0;
        exp_out = 0;
        case_idx = 0;
        pass_cnt = 0;
        fail_cnt = 0;

        fd_in = $fopen("tb/frame_input/fpadd_cases.txt", "r");
        if (fd_in == 0) begin
            $display("ERROR: cannot open fpadd_cases.txt");
            $finish;
        end

        @(negedge iCLK);

        while (!$feof(fd_in)) begin
            ret = $fscanf(fd_in, "%h %h %h\n", iA, iB, exp_out);

            if (ret == 3) begin
                // FpAdd is two-stage pipelined, wait 2 cycles for output to settle
                @(posedge iCLK);
                @(posedge iCLK);
                #1;

                if (oSum === exp_out) begin
                    pass_cnt = pass_cnt + 1;
                    $display("[%0d] a=%07h b=%07h rtl=%07h py=%07h PASS",
                             case_idx, iA, iB, oSum, exp_out);
                end
                else begin
                    fail_cnt = fail_cnt + 1;
                    $display("[%0d] a=%07h b=%07h rtl=%07h py=%07h FAIL",
                             case_idx, iA, iB, oSum, exp_out);
                    // Uncomment below to stop on the first failure
                    // $stop;
                end

                case_idx = case_idx + 1;

                // Load next input group with a clean negedge clock edge
                @(negedge iCLK);
            end
            else begin
                ret = $fgetc(fd_in);
            end
        end

        $display("========================================");
        $display("tb_fpadd done");
        $display("total = %0d, pass = %0d, fail = %0d",
                 case_idx, pass_cnt, fail_cnt);
        $display("========================================");

        $fclose(fd_in);
        $finish;
    end

endmodule
