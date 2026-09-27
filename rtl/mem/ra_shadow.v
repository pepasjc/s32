/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RetroAchievements shadow RAM: 16-bit write port (word address, byte
 * enables, wr_be[1] = bits 15:8) and a 64-bit read port (qword address, data
 * one clock later), on independent clocks. Mirror byte 2w+1:2w of qword n
 * lane (w & 3) is 16-bit word w as written. Eight inferred byte-lane simple
 * dual-port RAMs (one write port, one read port each), the layout of the
 * shadow in jotego's JTFRAME jtframe_ra_mirror.v (github.com/jotego/jtcores,
 * GPL-3.0-or-later), with the copier split out (ra_mirror_gather.v reads it).
 */
module ra_shadow #(parameter
    AW = 15                     // shadow size as a byte address width
)(
    input               clk_w,
    input               wr,
    input      [AW-2:0] wr_word,
    input        [15:0] wr_din,
    input        [ 1:0] wr_be,

    input               clk_r,
    input      [AW-4:0] rd_q,
    output       [63:0] q
);

localparam QWORDS = 1 << (AW-3);

wire [AW-4:0] wq   = wr_word[AW-2:2];
wire   [ 1:0] lane = wr_word[1:0];

genvar b;
generate
    for( b=0; b<8; b=b+1 ) begin : lanes
        wire we = wr && lane == b/2 && wr_be[b%2];
        (* ramstyle = "M10K, no_rw_check" *) reg [7:0] mem[0:QWORDS-1];
        reg [7:0] q_r;
        always @(posedge clk_w) if( we ) mem[wq] <= b%2 ? wr_din[15:8] : wr_din[7:0];
        always @(posedge clk_r) q_r <= mem[rd_q];
        assign q[b*8+:8] = q_r;
    end
endgenerate

endmodule
