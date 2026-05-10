`timescale 1ns/1ps

module tb_fpmul;

    reg  [26:0] iA;
    reg  [26:0] iB;
    wire [26:0] oProd;
    reg  [26:0] exp_out;

    integer fd_in;
    integer ret;
    integer case_idx;

    FpMul dut (
        .iA(iA),
        .iB(iB),
        .oProd(oProd)
    );

    initial begin
        iA = 0;
        iB = 0;
        exp_out = 0;

        fd_in = $fopen("tb/frame_input/fpmul_cases.txt", "r");
        if (fd_in == 0) begin
            $display("ERROR: cannot open fpmul_cases.txt");
            $finish;
        end

        case_idx = 0;

        while (!$feof(fd_in)) begin
            ret = $fscanf(fd_in, "%h %h %h\n", iA, iB, exp_out);
            $display("ret=%0d iA=%h iB=%h exp=%h", ret, iA, iB, exp_out);

            if (ret == 3) begin
                #1;
                $display("[%0d] rtl=%h py=%h %s",
                         case_idx, oProd, exp_out,
                         (oProd === exp_out) ? "PASS" : "FAIL");
                case_idx = case_idx + 1;
            end
            else begin
                // consume one bad line if needed
                ret = $fgetc(fd_in);
            end
        end

        $fclose(fd_in);
        $finish;
    end

endmodule
