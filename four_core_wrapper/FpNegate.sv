/**************************************************************************
 * Floating Point sign negation                                           *
 * Combinational                                                          *
 *************************************************************************/
module FpNegate (
    input  logic [26:0] iA,
    output logic [26:0] oNegative
);

    // Extract fields of A.
    logic        A_s;
    logic [7:0]  A_e;
    logic [17:0] A_f;

    assign A_s = iA[26];
    assign A_e = iA[25:18];
    assign A_f = iA[17:0];

    // Flip bit 26.
    assign oNegative = {~A_s, A_e, A_f};

endmodule
