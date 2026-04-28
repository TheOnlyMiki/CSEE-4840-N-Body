module FpMul (
    input  logic [26:0] iA,    // First input
    input  logic [26:0] iB,    // Second input
    output logic [26:0] oProd  // Product
);

    // Extract fields of A and B.
    logic        A_s;
    logic [7:0]  A_e;
    logic [17:0] A_f;
    logic        B_s;
    logic [7:0]  B_e;
    logic [17:0] B_f;

    assign A_s = iA[26];
    assign A_e = iA[25:18];
    assign A_f = {1'b1, iA[17:1]};
    assign B_s = iB[26];
    assign B_e = iB[25:18];
    assign B_f = {1'b1, iB[17:1]};

    // XOR sign bits to determine product sign.
    logic oProd_s;
    assign oProd_s = A_s ^ B_s;

    // Multiply the fractions of A and B.
    logic [35:0] pre_prod_frac;
    assign pre_prod_frac = A_f * B_f;

    // Add exponents of A and B.
    logic [8:0] pre_prod_exp;
    assign pre_prod_exp = A_e + B_e;

    // If top bit of product frac is 0, shift left one.
    logic [7:0]  oProd_e;
    logic [17:0] oProd_f;

    assign oProd_e = pre_prod_frac[35] ? (pre_prod_exp - 9'd126) :
                                         (pre_prod_exp - 9'd127);
    assign oProd_f = pre_prod_frac[35] ? pre_prod_frac[34:17] :
                                         pre_prod_frac[33:16];

    // Detect underflow.
    logic underflow;
    assign underflow = pre_prod_exp < 9'h80;

    // Detect zero conditions (either product frac doesn't start with 1, or underflow).
    assign oProd = underflow     ? 27'b0 :
                   (B_e == 8'd0) ? 27'b0 :
                   (A_e == 8'd0) ? 27'b0 :
                   {oProd_s, oProd_e, oProd_f};

endmodule
