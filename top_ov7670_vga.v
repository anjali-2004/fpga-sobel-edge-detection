// =============================================================================
// Top level: OV7670 live video -> frame buffer -> VGA      (Terasic DE2-115)
//
//   CLOCK_50 ──► ov7670_controller (SCCB config + 25 MHz XCLK) ──► camera
//   camera PCLK/VSYNC/HREF/D[7:0] ──► ov7670_capture ──► frame_buffer (320x240x12)
//   frame_buffer ──► vga_timing (2x upscale) ──► [RGB2GREY] ──► RGB ──► ADV7123 VGA DAC
//
// Port names follow the Terasic DE2-115 default .qsf so its pin assignments
// can be imported unchanged.
//
// Camera wiring on GPIO header (JP5):
//   GPIO[0]  SIOC (SCL)      out        GPIO[6]  D0   in
//   GPIO[1]  SIOD (SDA)      inout      GPIO[7]  D1   in
//   GPIO[2]  VSYNC           in         GPIO[8]  D2   in
//   GPIO[3]  HREF            in         GPIO[9]  D3   in
//   GPIO[4]  PCLK            in         GPIO[10] D4   in
//   GPIO[5]  XCLK            out        GPIO[11] D5   in
//                                       GPIO[12] D6   in
//   GPIO[14] RESET           out        GPIO[13] D7   in
//   GPIO[15] PWDN            out
//
// Controls:  KEY[0] = re-send camera config      SW[0] = greyscale
//            SW[1]  = freeze frame (keep DOWN for live video)
//            SW[2]  = VGA test pattern (colour bars + white border)
// LEDs:      LEDG[0] config done   LEDG[1] capture busy
//            LEDG[2] PCLK counter  LEDG[3] frame counter
//            LEDG[4] PCLK toggling LEDG[5] HREF toggling
//            LEDG[6] VSYNC toggling LEDG[7] data bus toggling
// =============================================================================

module top_ov7670_vga (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [17:0] SW,
    output wire [8:0]  LEDG,
    inout  wire [35:0] GPIO,

    output wire [7:0]  VGA_R,
    output wire [7:0]  VGA_G,
    output wire [7:0]  VGA_B,
    output wire        VGA_CLK,
    output wire        VGA_BLANK_N,
    output wire        VGA_SYNC_N,
    output wire        VGA_HS,
    output wire        VGA_VS
);

    // -------------------------------------------------------------------------
    // GPIO / camera pin map
    // -------------------------------------------------------------------------
    wire cam_sioc, cam_xclk, cam_reset, cam_pwdn, cfg_done;

    assign GPIO[0]     = cam_sioc;
    // GPIO[1] (SIOD) is driven by the i2c_sender inside ov7670_controller
    assign GPIO[2]     = 1'bz;      // VSYNC in
    assign GPIO[3]     = 1'bz;      // HREF in
    assign GPIO[4]     = 1'bz;      // PCLK in
    assign GPIO[5]     = cam_xclk;
    assign GPIO[13:6]  = 8'bz;      // D0..D7 in
    assign GPIO[14]    = cam_reset;
    assign GPIO[15]    = cam_pwdn;
    assign GPIO[35:16] = 20'bz;

    wire       cam_vsync = GPIO[2];
    wire       cam_href  = GPIO[3];
    wire       cam_pclk  = GPIO[4];
    wire [7:0] cam_d     = GPIO[13:6];

    // -------------------------------------------------------------------------
    // Clocks: 25 MHz VGA pixel clock from 50 MHz
    // -------------------------------------------------------------------------
    reg vga_clk_r = 1'b0;
    always @(posedge CLOCK_50) vga_clk_r <= ~vga_clk_r;
    wire vga_clk = vga_clk_r;

    // -------------------------------------------------------------------------
    // Camera configuration (50 MHz domain)
    // -------------------------------------------------------------------------
    reg [1:0] resend_sync = 2'b00;
    always @(posedge CLOCK_50) resend_sync <= {resend_sync[0], ~KEY[0]};

    ov7670_controller u_cam_ctrl (
        .clk             (CLOCK_50),
        .resend          (resend_sync[1]),
        .config_finished (cfg_done),
        .sioc            (cam_sioc),
        .siod            (GPIO[1]),
        .reset           (cam_reset),
        .pwdn            (cam_pwdn),
        .xclk            (cam_xclk)
    );

    // -------------------------------------------------------------------------
    // Capture (PCLK domain)
    // -------------------------------------------------------------------------
    reg [1:0] freeze_sync = 2'b00;
    always @(posedge cam_pclk) freeze_sync <= {freeze_sync[0], SW[1]};
    wire capture_en = ~freeze_sync[1];   // 1 = live, 0 = hold last frame

    wire [16:0] wr_addr;
    wire [11:0] wr_data;
    wire        wr_en;
    wire        cap_busy;

    ov7670_capture u_capture (
        .pclk    (cam_pclk),
        .capture (capture_en),
        .vsync   (cam_vsync),
        .href    (cam_href),
        .d       (cam_d),
        .addr    (wr_addr),
        .dout    (wr_data),
        .we      (wr_en),
        .busy    (cap_busy)
    );

    // -------------------------------------------------------------------------
    // Frame buffer + VGA read side (25 MHz domain)
    // -------------------------------------------------------------------------
    wire [16:0] rd_addr;
    wire [11:0] rd_data;
    wire        vga_hs, vga_vs, vga_active;
    wire [11:0] test_pix;

    frame_buffer u_fb (
        .wclk  (cam_pclk),
        .we    (wr_en),
        .waddr (wr_addr),
        .wdata (wr_data),
        .rclk  (vga_clk),
        .raddr (rd_addr),
        .rdata (rd_data)
    );

    vga_timing u_vga (
        .clk    (vga_clk),
        .raddr  (rd_addr),
        .hs     (vga_hs),
        .vs     (vga_vs),
        .active (vga_active),
        .test_pix (test_pix)
    );

    // Colour / greyscale / test-pattern select
    reg [1:0] grey_sync = 2'b00;
    reg [1:0] test_sync = 2'b00;
    always @(posedge vga_clk) begin
        grey_sync <= {grey_sync[0], SW[0]};
        test_sync <= {test_sync[0], SW[2]};
    end

    wire [7:0]  grey;
    RGB2GREY u_grey (.Din(rd_data), .Dout(grey));

    wire [11:0] cam_pix = grey_sync[1] ? {grey[7:4], grey[7:4], grey[7:4]} : rd_data;
    wire [11:0] pix     = test_sync[1] ? test_pix : cam_pix;

    RGB u_rgb (
        .Din    (pix),
        .Nblank (vga_active),
        .R      (VGA_R),
        .G      (VGA_G),
        .B      (VGA_B)
    );

    assign VGA_CLK     = vga_clk;
    assign VGA_HS      = vga_hs;
    assign VGA_VS      = vga_vs;
    assign VGA_BLANK_N = vga_active;
    assign VGA_SYNC_N  = 1'b0;

    // -------------------------------------------------------------------------
    // Debug LEDs
    // -------------------------------------------------------------------------
    reg [23:0] pclk_cnt  = 24'd0;
    reg [4:0]  frame_cnt = 5'd0;
    reg        vs_d      = 1'b0;

    always @(posedge cam_pclk) begin
        pclk_cnt <= pclk_cnt + 1'b1;
        vs_d     <= cam_vsync;
        if (cam_vsync && !vs_d) frame_cnt <= frame_cnt + 1'b1;
    end

    // Camera signal activity, measured on the 50 MHz clock (works without PCLK)
    wire pclk_alive, href_alive, vsync_alive, data_alive;
    sig_activity u_act_pclk  (.clk(CLOCK_50), .sig(cam_pclk),  .alive(pclk_alive));
    sig_activity u_act_href  (.clk(CLOCK_50), .sig(cam_href),  .alive(href_alive));
    sig_activity u_act_vsync (.clk(CLOCK_50), .sig(cam_vsync), .alive(vsync_alive));
    sig_activity u_act_data  (.clk(CLOCK_50), .sig(^cam_d),    .alive(data_alive));

    assign LEDG = {1'b0, data_alive, vsync_alive, href_alive, pclk_alive,
                   frame_cnt[4], pclk_cnt[23], cap_busy, cfg_done};

endmodule