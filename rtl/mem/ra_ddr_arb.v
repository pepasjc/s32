/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Two-client Avalon-MM arbiter for a MiSTer DDRAM port: the core's own DDR
 * client (c0, reads and writes) and the RetroAchievements RAM mirror (c1,
 * write bursts only). From the Cave MiSTer core's RA port (cave_ra_ddr_arb),
 * changed so that the grant never depends combinationally on the core's
 * request: cores whose RD/WE are themselves gated by the busy they see would
 * otherwise form a loop.
 *
 * Ownership only changes at a transaction boundary: no write burst half-sent
 * and no read beats still due. At a boundary the mirror wins whenever it is
 * requesting; it asks for one 32-beat burst and then releases the port while
 * it gathers the next 256 bytes from the core's RAM, so the core gets the
 * port between every two mirror bursts. The client not selected sees busy and
 * holds its command, as Avalon requires.
 */
module ra_ddr_arb (
    input               clk,
    input               rst,
    input               ddr_busy,
    input               ddr_dout_ready,
    // client 0: the core
    input               c0_rd,
    input               c0_we,
    input        [28:0] c0_addr,
    input        [ 7:0] c0_be,
    input        [63:0] c0_din,
    input        [ 7:0] c0_burstcnt,
    output              c0_busy,
    // client 1: RA mirror (writes only)
    input               c1_we,
    input        [28:0] c1_addr,
    input        [ 7:0] c1_be,
    input        [63:0] c1_din,
    input        [ 7:0] c1_burstcnt,
    output              c1_busy,
    // DDR
    output              ddr_rd,
    output              ddr_we,
    output       [28:0] ddr_addr,
    output       [ 7:0] ddr_be,
    output       [63:0] ddr_din,
    output       [ 7:0] ddr_burstcnt
);

reg  [8:0] c0_rd_left;   // read beats still to be returned to the core
reg  [7:0] c0_wr_left;   // beats left in the core's open write burst
reg  [7:0] c1_wr_left;   // beats left in the mirror's open write burst

wire c0_open = c0_rd_left != 0 || c0_wr_left != 0;
wire c1_open = c1_wr_left != 0;

wire sel1 = c1_open | (~c0_open & c1_we);

wire c0_acc_rd = !sel1 && c0_rd && !ddr_busy;
wire c0_acc_we = !sel1 && c0_we && !ddr_busy;
wire c1_acc    =  sel1 && c1_we && !ddr_busy;

wire [7:0] c0_len = c0_burstcnt == 0 ? 8'd1 : c0_burstcnt;
wire [7:0] c1_len = c1_burstcnt == 0 ? 8'd1 : c1_burstcnt;

assign c0_busy      = ddr_busy | sel1;
assign c1_busy      = ddr_busy | ~sel1;
assign ddr_rd       = sel1 ? 1'b0         : c0_rd;
assign ddr_we       = sel1 ? c1_we        : c0_we;
assign ddr_addr     = sel1 ? c1_addr      : c0_addr;
assign ddr_be       = sel1 ? c1_be        : c0_be;
assign ddr_din      = sel1 ? c1_din       : c0_din;
assign ddr_burstcnt = sel1 ? c1_burstcnt  : c0_burstcnt;

always @(posedge clk) begin
    if( rst ) begin
        c0_rd_left <= 0;
        c0_wr_left <= 0;
        c1_wr_left <= 0;
    end else begin
        c0_rd_left <= c0_rd_left + (c0_acc_rd ? {1'b0, c0_len} : 9'd0)
                                 - (ddr_dout_ready && c0_rd_left != 0 ? 9'd1 : 9'd0);
        if( c0_acc_we )
            c0_wr_left <= c0_wr_left != 0 ? c0_wr_left - 8'd1 : c0_len - 8'd1;
        if( c1_acc )
            c1_wr_left <= c1_wr_left != 0 ? c1_wr_left - 8'd1 : c1_len - 8'd1;
    end
end

endmodule
