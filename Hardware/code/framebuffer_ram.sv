// Simple dual-port 1-bit-per-pixel framebuffer storage.
//
// Port A is used by the Avalon-MM slave for CPU writes.
// Port B is used by the VGA scanout logic for continuous reads.

module framebuffer_ram #(
    parameter int FB_WORDS = 9600,
    parameter int ADDR_W   = 14
) (
    input  logic              clk,

    input  logic              port_a_we,
    input  logic [ADDR_W-1:0] port_a_addr,
    input  logic [31:0]       port_a_writedata,

    input  logic [ADDR_W-1:0] port_b_addr,
    output logic [31:0]       port_b_readdata
);

    (* ramstyle = "M10K" *) logic [31:0] mem [0:FB_WORDS-1];

    always_ff @(posedge clk) begin
        if (port_a_we) begin
            mem[port_a_addr] <= port_a_writedata;
        end

        port_b_readdata <= mem[port_b_addr];
    end

endmodule
