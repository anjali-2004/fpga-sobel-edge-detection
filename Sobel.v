module Sobel_RGB_Color (
    input         iCLK,
    input         iRST_N,
    input         iValid,
    input  [9:0]  iRed,
    input  [9:0]  iGreen,
    input  [9:0]  iBlue,
    output [9:0]  oRed,
    output [9:0]  oGreen,
    output [9:0]  oBlue
);

    // ----------------------------------------------------
    // Fixes a pipeline-alignment bug in Sobel_RGB_Color.v:
    // that version delayed x/y position and the original
    // color pixel through TWO register stages, but the
    // magnitude/threshold decision only used ONE stage of
    // delay on Gx/Gy -- so the edge decision, the border
    // check, and the displayed color pixel were all looking
    // at different pixels, 1 cycle apart. This version uses
    // exactly ONE pipeline cut: Gx/Gy, x/y position, and the
    // original color pixel are all registered together at
    // the SAME clock edge, so everything downstream of that
    // point stays correctly aligned.
    // ----------------------------------------------------
    reg [9:0] x_count, y_count;

    reg [9:0] lineR1 [0:639];
    reg [9:0] lineR2 [0:639];
    reg [9:0] topR1, topR2, midR1, midR2, botR1, botR2;
    wire [9:0] topR_pixel = lineR1[x_count];
    wire [9:0] midR_pixel = lineR2[x_count];

    reg [9:0] lineG1 [0:639];
    reg [9:0] lineG2 [0:639];
    reg [9:0] topG1, topG2, midG1, midG2, botG1, botG2;
    wire [9:0] topG_pixel = lineG1[x_count];
    wire [9:0] midG_pixel = lineG2[x_count];

    reg [9:0] lineB1 [0:639];
    reg [9:0] lineB2 [0:639];
    reg [9:0] topB1, topB2, midB1, midB2, botB1, botB2;
    wire [9:0] topB_pixel = lineB1[x_count];
    wire [9:0] midB_pixel = lineB2[x_count];

    // ---- Stage A (combinational): Gx/Gy per channel from window taps ----
    reg signed [13:0] GxR_c, GyR_c, GxG_c, GyG_c, GxB_c, GyB_c;
    always @(*) begin
        GxR_c = -$signed({1'b0,topR2}) + $signed({1'b0,topR_pixel})
              - (2*$signed({1'b0,midR2})) + (2*$signed({1'b0,midR_pixel}))
              - $signed({1'b0,botR2}) + $signed({1'b0,iRed});
        GyR_c = -$signed({1'b0,topR2}) - (2*$signed({1'b0,topR1})) - $signed({1'b0,topR_pixel})
              + $signed({1'b0,botR2}) + (2*$signed({1'b0,botR1})) + $signed({1'b0,iRed});

        GxG_c = -$signed({1'b0,topG2}) + $signed({1'b0,topG_pixel})
              - (2*$signed({1'b0,midG2})) + (2*$signed({1'b0,midG_pixel}))
              - $signed({1'b0,botG2}) + $signed({1'b0,iGreen});
        GyG_c = -$signed({1'b0,topG2}) - (2*$signed({1'b0,topG1})) - $signed({1'b0,topG_pixel})
              + $signed({1'b0,botG2}) + (2*$signed({1'b0,botG1})) + $signed({1'b0,iGreen});

        GxB_c = -$signed({1'b0,topB2}) + $signed({1'b0,topB_pixel})
              - (2*$signed({1'b0,midB2})) + (2*$signed({1'b0,midB_pixel}))
              - $signed({1'b0,botB2}) + $signed({1'b0,iBlue});
        GyB_c = -$signed({1'b0,topB2}) - (2*$signed({1'b0,topB1})) - $signed({1'b0,topB_pixel})
              + $signed({1'b0,botB2}) + (2*$signed({1'b0,botB1})) + $signed({1'b0,iBlue});
    end

    // ---- ONE pipeline cut: Gx/Gy, position, and original pixel ----
    // All of these are registered together at the same clock edge,
    // so they stay in lock-step from here on -- no further relative
    // delay is introduced between them.
    reg signed [13:0] GxR_r, GyR_r, GxG_r, GyG_r, GxB_r, GyB_r;
    reg [9:0] x_d1, y_d1;
    reg [9:0] red_d1, green_d1, blue_d1;

    // ---- Stage B (combinational): magnitude + max, from the REGISTERED Gx/Gy ----
    // Because x_d1 / red_d1 / GxR_r etc. were all latched at the same
    // edge, this block and the final register below are reading
    // values that all describe the SAME pixel.
    reg [14:0] magR_c, magG_c, magB_c, magMax_c;
    always @(*) begin
        magR_c = (GxR_r<0 ? -GxR_r : GxR_r) + (GyR_r<0 ? -GyR_r : GyR_r);
        magG_c = (GxG_r<0 ? -GxG_r : GxG_r) + (GyG_r<0 ? -GyG_r : GyG_r);
        magB_c = (GxB_r<0 ? -GxB_r : GxB_r) + (GyB_r<0 ? -GyB_r : GyB_r);
        magMax_c = magR_c;
        if (magG_c > magMax_c) magMax_c = magG_c;
        if (magB_c > magMax_c) magMax_c = magB_c;
    end

    reg [9:0] outR, outG, outB;

    assign oRed   = outR;
    assign oGreen = outG;
    assign oBlue  = outB;

    integer i;

    always @(posedge iCLK or negedge iRST_N) begin
        if (!iRST_N) begin
            x_count <= 0; y_count <= 0;
            topR1<=0;topR2<=0;midR1<=0;midR2<=0;botR1<=0;botR2<=0;
            topG1<=0;topG2<=0;midG1<=0;midG2<=0;botG1<=0;botG2<=0;
            topB1<=0;topB2<=0;midB1<=0;midB2<=0;botB1<=0;botB2<=0;
            GxR_r<=0; GyR_r<=0; GxG_r<=0; GyG_r<=0; GxB_r<=0; GyB_r<=0;
            x_d1<=0; y_d1<=0; red_d1<=0; green_d1<=0; blue_d1<=0;
            outR<=0; outG<=0; outB<=0;
            for (i=0;i<640;i=i+1) begin
                lineR1[i]<=0; lineR2[i]<=0;
                lineG1[i]<=0; lineG2[i]<=0;
                lineB1[i]<=0; lineB2[i]<=0;
            end
        end
        else begin
            // Single pipeline cut: Gx/Gy, position, and original color
            // pixel all move forward TOGETHER, one cycle, from this edge.
            GxR_r <= GxR_c; GyR_r <= GyR_c;
            GxG_r <= GxG_c; GyG_r <= GyG_c;
            GxB_r <= GxB_c; GyB_r <= GyB_c;
            x_d1  <= x_count; y_d1 <= y_count;
            red_d1 <= iRed; green_d1 <= iGreen; blue_d1 <= iBlue;

            // Final output register: reads x_d1 / magMax_c / red_d1 etc.
            // as they stand THIS cycle -- all describing the same pixel,
            // since they were latched together one cycle ago.
            if ((x_d1 < 2) || (x_d1 > 637) || (y_d1 < 2) || (y_d1 > 477)) begin
                outR <= red_d1; outG <= green_d1; outB <= blue_d1;
            end
            else if (magMax_c > 15'd500) begin
                outR <= 10'd0; outG <= 10'd0; outB <= 10'd0;   // edge -> black
            end
            else begin
                outR <= red_d1; outG <= green_d1; outB <= blue_d1;
            end

            if (iValid) begin
                // Red
                topR2<=topR1; topR1<=topR_pixel;
                midR2<=midR1; midR1<=midR_pixel;
                botR2<=botR1; botR1<=iRed;
                lineR1[x_count]<=lineR2[x_count];
                lineR2[x_count]<=iRed;

                // Green
                topG2<=topG1; topG1<=topG_pixel;
                midG2<=midG1; midG1<=midG_pixel;
                botG2<=botG1; botG1<=iGreen;
                lineG1[x_count]<=lineG2[x_count];
                lineG2[x_count]<=iGreen;

                // Blue
                topB2<=topB1; topB1<=topB_pixel;
                midB2<=midB1; midB1<=midB_pixel;
                botB2<=botB1; botB1<=iBlue;
                lineB1[x_count]<=lineB2[x_count];
                lineB2[x_count]<=iBlue;

                if (x_count == 639) begin
                    x_count <= 0;
                    if (y_count == 479) y_count <= 0;
                    else y_count <= y_count + 1;
                    topR1<=0;topR2<=0;midR1<=0;midR2<=0;botR1<=0;botR2<=0;
                    topG1<=0;topG2<=0;midG1<=0;midG2<=0;botG1<=0;botG2<=0;
                    topB1<=0;topB2<=0;midB1<=0;midB2<=0;botB1<=0;botB2<=0;
                end
                else
                    x_count <= x_count + 1;
            end
        end
    end

endmodule

