/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RetroAchievements RAM mirror: gather-and-burst copier.
 *
 * Adapted from jtframe_ra_mirror.v of the RetroAchievements fork of jotego's
 * jtcores / JTFRAME (modules/jtframe/target/mister/hdl/jtframe_ra_mirror.v,
 * https://github.com/jotego/jtcores, JTFRAME by Jose Tejada Gomez,
 * GPL-3.0-or-later), by way of the IGS PGM port's pgm_ra_mirror.sv. There is
 * no shadow RAM: once per VBlank the copier reads the core's own RAM(s)
 * through a spare read port, 256 bytes at a time, into a small burst buffer
 * with the DDR port released, then writes that buffer as one 32-beat burst.
 *
 * Read port: rd_addr is a word index into the 64 kB mirror (DW-bit words).
 * The instantiating core maps it to its RAM(s) (page map) and returns
 * rd_data exactly RD_LAT clocks after rd_en, already in RA byte order:
 * bits 8b+7:8b of mirror word i are RA byte i*(DW/8)+b.
 *
 * DDR layout at byte address 0x3D000000 (the "RACH" Full Mirror header used
 * by the RA fork of Main_MiSTer, see ra_ramread.h):
 *   0x00  magic "RACH" (0x52414348 LE), region count 0, flags (bit0 busy),
 *         core version
 *   0x08  frame counter (u32)
 *   0x10  zero (keeps stale mailbox/protocol bytes of other cores clear)
 *   0x100 the 64 kB mirror
 *
 * Copy order: header with busy=1, data, frame counter, header with busy=0.
 * The ARM only accepts a snapshot when busy is clear and the frame counter
 * did not move during its copy. A copy interrupted by hold finishes the
 * burst it started and stops with busy still set.
 *
 * DDR side: a plain Avalon write master. ddr_we is only raised when a whole
 * transaction is ready, so a transaction-boundary arbiter (ra_ddr_arb.v) can
 * interleave the core's own DDR traffic between the mirror's bursts.
 */

module ra_mirror_gather #(parameter
    DW        = 16,             // read port width: 8, 16, 32 or 64
    RD_LAT    = 1,              // rd_en -> rd_data latency, clocks (>= 1)
    BLOCKS    = 256,            // 256-byte blocks copied: 256 = 64 kB
    DDR_BASE  = 29'h07A0_0000,  // 0x3D000000 / 8
    VERSION   = 16'h0100
)(
    input               rst,
    input               clk,
    input               vblank,     // active high; a copy starts on its rising edge
    input               hold,       // ROM download etc.: no copy, no RAM reads

    output              rd_en,
    output [15-$clog2(DW/8):0] rd_addr,
    input      [DW-1:0] rd_data,

    output reg          ddr_we,
    input               ddr_busy,
    output reg   [ 7:0] ddr_burstcnt,
    output reg   [28:0] ddr_addr,
    output       [ 7:0] ddr_be,
    output reg   [63:0] ddr_din,
    output reg          active      // a copy is in progress (not only a burst)
);

localparam [31:0] MAGIC = 32'h5241_4348;
localparam        BW    = $clog2(DW/8);       // log2 bytes per read word
localparam        WPB   = 256 >> BW;          // read words per 256-byte block
localparam        WB    = 8 - BW;             // bits of a word index in a block
localparam        L     = 64 / DW;            // read words per qword
localparam        LB    = (L > 1) ? $clog2(L) : 1;
localparam        SH    = (L > 1) ? LB : 0;      // word index -> qword index shift

localparam [2:0] IDLE=0, HDR_BUSY=1, FILL=2, WRITE=3, FRAME=4, ZERO=5, HDR_DONE=6;

reg  [ 2:0] st;
reg  [ 7:0] blk;        // 256-byte block of the mirror
reg  [WB:0] fcnt;       // read words issued in this block
reg  [ 4:0] ptr;        // burst beat
reg  [31:0] frame;
reg         zeroed, spoiled, vblank_l;

wire accept = ddr_we && !ddr_busy;

// ---------------------------------------------------------------------------
// Reads: issue WPB words, then wait for the last one to land
wire issuing = (st == FILL) && !fcnt[WB] && !hold && !spoiled;
assign rd_en   = issuing;
assign rd_addr = { blk, fcnt[WB-1:0] };

reg  [RD_LAT-1:0] vpipe;
reg  [WB-1:0]     ipipe [0:RD_LAT-1];
integer k;
always @(posedge clk) begin
    vpipe[0] <= issuing;
    ipipe[0] <= fcnt[WB-1:0];
    for( k=1; k<RD_LAT; k=k+1 ) begin
        vpipe[k] <= vpipe[k-1];
        ipipe[k] <= ipipe[k-1];
    end
end
wire          got     = vpipe[RD_LAT-1];
wire [WB-1:0] got_idx = ipipe[RD_LAT-1];
wire          landed  = !(|vpipe) && fcnt[WB];

// ---------------------------------------------------------------------------
// Burst buffer: 32 qwords as L lanes of DW bits (lane j = word L*n+j, in DDR
// bits DW*j+DW-1:DW*j). Next beat's address presented on accept.
wire [ 4:0] buf_rd = (st == WRITE && accept) ? ptr + 5'd1 : ptr;
wire [63:0] buf_q;

genvar j;
generate
    for( j=0; j<L; j=j+1 ) begin : lanes
        reg [DW-1:0] mem[0:31];
        reg [DW-1:0] q;
        wire hit = (L == 1) ? 1'b1 : (got_idx[LB-1:0] == j);
        always @(posedge clk) begin
            if( got && hit ) mem[got_idx >> SH] <= rd_data;
            q <= mem[buf_rd];
        end
        assign buf_q[DW*j +: DW] = q;
    end
endgenerate

assign ddr_be = 8'hff;

always @(*) begin
    case( st )
        HDR_BUSY: ddr_din = { VERSION, 8'h01, 8'd0, MAGIC };
        HDR_DONE: ddr_din = { VERSION, 8'h00, 8'd0, MAGIC };
        FRAME:    ddr_din = { 32'd0, frame };
        ZERO:     ddr_din = 64'd0;
        default:  ddr_din = buf_q;
    endcase
end

always @(posedge clk) begin
    if( rst ) begin
        st           <= IDLE;
        active       <= 0;
        ddr_we       <= 0;
        ddr_burstcnt <= 8'd1;
        ddr_addr     <= DDR_BASE;
        blk          <= 0;
        fcnt         <= 0;
        ptr          <= 0;
        frame        <= 0;
        zeroed       <= 0;
        spoiled      <= 0;
        vblank_l     <= 0;
    end else begin
        vblank_l <= vblank;
        if( hold && st != IDLE ) spoiled <= 1;
        case( st )
            IDLE: begin
                ddr_we  <= 0;
                active  <= 0;
                spoiled <= 0;
                if( vblank && !vblank_l && !hold ) begin
                    active       <= 1;
                    ddr_we       <= 1;
                    ddr_addr     <= DDR_BASE;
                    ddr_burstcnt <= 8'd1;
                    blk          <= 0;
                    st           <= HDR_BUSY;
                end
            end
            HDR_BUSY: if( accept ) begin
                ddr_we <= 0;
                fcnt   <= 0;
                ptr    <= 0;
                st     <= FILL;
            end
            FILL: begin
                if( spoiled || hold ) begin
                    st <= IDLE;
                end else if( landed ) begin
                    ptr          <= 0;
                    ddr_we       <= 1;
                    ddr_addr     <= DDR_BASE + 29'h20 + { 16'd0, blk, 5'd0 };
                    ddr_burstcnt <= 8'd32;
                    st           <= WRITE;
                end else if( !fcnt[WB] ) begin
                    fcnt <= fcnt + 1'd1;
                end
            end
            WRITE: if( accept ) begin
                ptr <= ptr + 5'd1;
                if( &ptr ) begin
                    fcnt <= 0;
                    blk  <= blk + 8'd1;
                    if( spoiled || hold ) begin
                        ddr_we <= 0;
                        st     <= IDLE;
                    end else if( blk == BLOCKS-1 ) begin
                        ddr_addr     <= DDR_BASE + 29'h1;
                        ddr_burstcnt <= 8'd1;
                        frame        <= frame + 32'd1;
                        st           <= FRAME;
                    end else begin
                        ddr_we <= 0;
                        st     <= FILL;
                    end
                end
            end
            FRAME: if( accept ) begin
                ddr_addr <= zeroed ? DDR_BASE : DDR_BASE + 29'h2;
                st       <= zeroed ? HDR_DONE : ZERO;
            end
            ZERO: if( accept ) begin
                zeroed   <= 1;
                ddr_addr <= DDR_BASE;
                st       <= HDR_DONE;
            end
            HDR_DONE: if( accept ) begin
                ddr_we <= 0;
                active <= 0;
                st     <= IDLE;
            end
            default: st <= IDLE;
        endcase
    end
end

endmodule
