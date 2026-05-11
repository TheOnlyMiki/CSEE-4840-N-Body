/*
 * Avalon memory-mapped VGA bitmap peripheral.
 *
 * This is a separate Platform Designer/Qsys component from the n-body
 * accelerator. Software writes a packed 640x480 1-bpp framebuffer through
 * Avalon-MM; hardware only scans the bitmap out to VGA.
 *
 * Register map, 32-bit word addresses:
 *   0..9599  framebuffer words, read/write
 *
 * Bitmap packing:
 *   WIDTH = 640, HEIGHT = 480, WORDS_PER_ROW = 20, FB_WORDS = 9600
 *   word_index = y * 20 + x / 32
 *   bit_index  = x % 32
 *   frame[y * 20 + x / 32][x % 32] = pixel(x, y)
 *
 * This bit order matches the existing display driver draft in Software/,
 * which packs pixels with (1u << (x % 32)). It is LSB-first within each
 * 32-bit word, unlike the earlier MSB-first proposal.
 */

module vga_bitmap_avmm (
    input  logic        clk,
    input  logic        reset,       // active-high Platform Designer reset

    input  logic        chipselect,
    input  logic        read,
    input  logic        write,
    input  logic [13:0] address,     // word address, valid framebuffer range 0..9599
    input  logic [31:0] writedata,
    output logic [31:0] readdata,

    output logic [7:0]  VGA_R,
    output logic [7:0]  VGA_G,
    output logic [7:0]  VGA_B,
    output logic        VGA_CLK,
    output logic        VGA_HS,
    output logic        VGA_VS,
    output logic        VGA_BLANK_n,
    output logic        VGA_SYNC_n
);

    localparam int WIDTH         = 640;
    localparam int HEIGHT        = 480;
    localparam int WORDS_PER_ROW = 20;
    localparam int FB_WORDS      = HEIGHT * WORDS_PER_ROW;
    localparam int ADDR_W        = 14;

    localparam logic [ADDR_W-1:0] FB_LAST_WORD = ADDR_W'(FB_WORDS - 1);

    logic [10:0] hcount;
    logic [9:0]  vcount;

    logic        av_addr_valid;
    logic        av_read_valid_d;
    logic [31:0] av_readdata;
    logic [13:0] av_ram_addr;

    logic        vga_blank_raw;
    logic        active_video;
    logic        active_video_d;
    logic [9:0]  pixel_x;
    logic [8:0]  pixel_y;
    logic [13:0] pixel_y_ext;
    logic [13:0] scan_addr;
    logic [4:0]  bit_index;
    logic [4:0]  bit_index_d;
    logic [31:0] scan_readdata;
    logic        pixel_bit;

    assign av_addr_valid = (address <= FB_LAST_WORD);
    assign av_ram_addr   = av_addr_valid ? address : 14'd0;

    /*
     * Reuse the Lab 3 VGA timing generator from vga_ball.sv. hcount[10:1]
     * is the 640-pixel column and vcount is the 480-pixel row.
     */
    vga_counters counters (
        .clk50      (clk),
        .reset      (reset),
        .hcount     (hcount),
        .vcount     (vcount),
        .VGA_CLK    (VGA_CLK),
        .VGA_HS     (VGA_HS),
        .VGA_VS     (VGA_VS),
        .VGA_BLANK_n(vga_blank_raw),
        .VGA_SYNC_n (VGA_SYNC_n)
    );

    assign active_video = vga_blank_raw;
    assign pixel_x      = hcount[10:1];
    assign pixel_y      = vcount[8:0];
    assign pixel_y_ext  = {5'd0, pixel_y};

    // word_index = y * 20 + x / 32 = (y << 4) + (y << 2) + x[9:5]
    assign scan_addr = active_video ? ((pixel_y_ext << 4) + (pixel_y_ext << 2) +
                                       {9'd0, pixel_x[9:5]}) :
                                      14'd0;
    assign bit_index = pixel_x[4:0];

    framebuffer_ram #(
        .FB_WORDS(FB_WORDS),
        .ADDR_W  (ADDR_W)
    ) u_framebuffer (
        .clk             (clk),

        .port_a_we       (chipselect && write && av_addr_valid),
        .port_a_addr     (av_ram_addr),
        .port_a_writedata(writedata),
        .port_a_readdata (av_readdata),

        .port_b_addr     (scan_addr),
        .port_b_readdata (scan_readdata)
    );

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            av_read_valid_d <= 1'b0;
            active_video_d  <= 1'b0;
            bit_index_d     <= 5'd0;
        end else begin
            av_read_valid_d <= chipselect && read && av_addr_valid;
            active_video_d  <= active_video;
            bit_index_d     <= bit_index;
        end
    end

    assign readdata    = av_read_valid_d ? av_readdata : 32'd0;
    assign VGA_BLANK_n = active_video_d;
    assign pixel_bit   = scan_readdata[bit_index_d];

    always_comb begin
        if (active_video_d && pixel_bit) begin
            VGA_R = 8'hff;
            VGA_G = 8'hff;
            VGA_B = 8'hff;
        end else begin
            VGA_R = 8'h00;
            VGA_G = 8'h00;
            VGA_B = 8'h00;
        end
    end

endmodule

module vga_counters(
 input logic         clk50, reset,
 output logic [10:0] hcount,  // hcount[10:1] is pixel column
 output logic [9:0]  vcount,  // vcount[9:0] is pixel row
 output logic        VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_n, VGA_SYNC_n);

/*
 * 640 X 480 VGA timing for a 50 MHz clock: one pixel every other cycle
 *
 * HCOUNT 1599 0             1279       1599 0
 *             _______________              ________
 * ___________|    Video      |____________|  Video
 *
 *
 * |SYNC| BP |<-- HACTIVE -->|FP|SYNC| BP |<-- HACTIVE
 *       _______________________      _____________
 * |____|       VGA_HS          |____|
 */
   // Parameters for hcount
   parameter HACTIVE      = 11'd 1280,
             HFRONT_PORCH = 11'd 32,
             HSYNC        = 11'd 192,
             HBACK_PORCH  = 11'd 96,
             HTOTAL       = HACTIVE + HFRONT_PORCH + HSYNC +
                            HBACK_PORCH; // 1600

   // Parameters for vcount
   parameter VACTIVE      = 10'd 480,
             VFRONT_PORCH = 10'd 10,
             VSYNC        = 10'd 2,
             VBACK_PORCH  = 10'd 33,
             VTOTAL       = VACTIVE + VFRONT_PORCH + VSYNC +
                            VBACK_PORCH; // 525

   logic endOfLine;

   always_ff @(posedge clk50 or posedge reset)
     if (reset)          hcount <= 0;
     else if (endOfLine) hcount <= 0;
     else                hcount <= hcount + 11'd 1;

   assign endOfLine = hcount == HTOTAL - 1;

   logic endOfField;

   always_ff @(posedge clk50 or posedge reset)
     if (reset)          vcount <= 0;
     else if (endOfLine)
       if (endOfField)   vcount <= 0;
       else              vcount <= vcount + 10'd 1;

   assign endOfField = vcount == VTOTAL - 1;

   // Horizontal sync: from 0x520 to 0x5DF (0x57F)
   // 101 0010 0000 to 101 1101 1111
   assign VGA_HS = !( (hcount[10:8] == 3'b101) &
                      !(hcount[7:5] == 3'b111));
   assign VGA_VS = !( vcount[9:1] == (VACTIVE + VFRONT_PORCH) / 2);

   assign VGA_SYNC_n = 1'b0; // For putting sync on the green signal; unused

   // Horizontal active: 0 to 1279     Vertical active: 0 to 479
   // 101 0000 0000  1280              01 1110 0000  480
   // 110 0011 1111  1599              10 0000 1100  524
   assign VGA_BLANK_n = !( hcount[10] & (hcount[9] | hcount[8]) ) &
                        !( vcount[9] | (vcount[8:5] == 4'b1111) );

   /* VGA_CLK is 25 MHz
    *             __    __    __
    * clk50    __|  |__|  |__|
    *
    *             _____       __
    * hcount[0]__|     |_____|
    */
   assign VGA_CLK = hcount[0]; // 25 MHz clock: rising edge sensitive

endmodule
