// V9938 emulation. Phase 1 (base-9918-compatible modes), phase 2 (Screen 5-8 rendering, sprite
// mode 2, 212-line height), phase 3 (all 4 I/O ports, real palette) and phase 4 (the command
// engine, minus LMCM/LMMC/HMMC) are all here now -- see RPROM/plan-v9938-base9918.md for all four.
// Builds instead of tms9918a.c when -DV9938=ON, implementing the exact same tms9918a.h interface so
// vdp_bus.c/render.c/main.c need no changes at all -- this is a drop-in alternative chip backend,
// not a new subsystem.
//
// Ported from RPROM/AppleII-VGA's v9938 branch (pico/v9938.c, itself a speed-tuned port of MAME's
// v9938.cpp for a 252 MHz Pico target) with four corrections found while porting (one of them
// after an actual hardware test -- see the 4th), all confirmed by comparison against this project's
// own already-hardware-proven tms9918a.c:
//
//  * Sprite collision detection and the 5th-sprite-overflow status bits were being written directly
//    into the status byte from the render core (the AppleII-VGA source even has the collision
//    report commented out entirely: "// we have a collision! //if (p2 < 4) m_stat_reg[0] |= 0x20;").
//    That is a cross-core data race here: the bus core reads the status byte with a tight deadline
//    (a few hundred ns), so it must only ever be touched through the same event-counter/publish
//    pattern tms9918a.c already uses (m_FrameEvents/m_CollisionEvents/m_FifthEvents/m_EventSeq/
//    publish_event(), consumed by status_refresh()/check_interrupt() on the bus side). Both ported
//    here, wired through that existing mechanism instead.
//  * The ported MULTI (multicolor) mode case drew 4 words of foreground colour then 4 of background
//    (16 output pixels) per 8-pixel-wide character cell -- double width. tms9918a.c's own MODE 3
//    (the same hardware feature, reached via a different mode-number encoding) draws 2+2 words (8
//    pixels), which is what an 8x8 character cell actually is; fixed to match.
//  * The sprite pixel loop indexes a collision buffer at `x+1` for a magnified sprite's second pixel
//    without re-checking bounds beyond the first pixel's `x < 256` guard, so `x == 255` writes
//    `col[256]` -- one past a 256-byte buffer. Sized with margin here instead (matching the spirit of
//    tms9918a.c's own 32+256+32 margin for the same reason).
//  * GRAPHIC1/MULTI/GRAPHIC2 all read a colour-table byte that packs two 4-bit palette indices
//    (foreground nibble, background nibble) per cell -- but a nibble of 0 means "use the current
//    backdrop colour (R#7)", not literal palette index 0. The ported source indexed the palette
//    directly with no such substitution; tms9918a.c's equivalent code already has it
//    (`(colour & 15) ? (colour & 15) : BackColour`), ported here to match. Without it, any area
//    using colour-index 0 for its background renders solid black instead of the game's actual
//    backdrop colour -- found on real hardware (a Konami logo intro: white logo on blue in the
//    original, black instead of blue here), not by comparison alone.
//
// Phase 2 adds Screen 5-8 (Graphic4/5/6/7) rendering and sprite mode 2 (the sprite protocol those
// four modes use, genuinely different from sprite mode 1's), ported the same way: AppleII-VGA's
// v9938.c already has Graphic4/5/6 and sprite mode 2 (with the same direct-status-byte-write race
// condition as phase 1's sprite mode 1, fixed the same way -- see the 1st fix above, now with a
// second producer); Graphic7 comes from the raw MAME v9938.cpp instead (AppleII-VGA's own README
// admits it "is not implemented at all" there). Three things deliberately NOT done, not silently
// dropped:
//  * (No longer true, msx-cart-v9938 C22: the "second field" +0x8000/+0x10000 offsets are the R#2 bit5
//    page select and lie inside the 128K; see second_field() above GRAPHIC4.)
//  * Graphic5/Screen6 sprites: the source being ported has its own "// TODO. graphic5 sprites are
//    actually rendered at 512 pix. That means rewriting all the rendering engine stuff" -- carried
//    over as-is, not fixed (fixing it means reworking the shared pixel-output path for 512-wide
//    content).
//  * The command engine and Screen 5-8's palette-dependent colour customization are both in scope
//    as of phases 3-4 (below) -- the V9938 line interrupt is the one thing from this list still
//    out of scope, and still is: nothing in phase 3/4 touches it, MSX1 software never used it.
//
// Phase 3 adds all 4 I/O ports (R#17-indirect register access at $9B, real palette writes at $9A --
// see v9938-msx1-changes.md for the A1 hardware wiring this needed) and rewires every
// palette-indexed mode's colour lookups from the shared default tms9918a_palette[] to this file's
// own mutable v9938_palette[16] (seeded from the same defaults at init, so nothing changes until
// software actually writes a palette entry).
//
// Phase 4 adds the command engine (register R#46 trigger, see command_unit_w and the big comment
// block above it) -- 7 of its 10 real commands (SRCH, LINE, LMMV, LMMM, HMMV, HMMM, YMMM) plus the
// 3 instant ones (ABRT, POINT, PSET); LMCM, LMMC and HMMC are deliberately not ported (all three are
// CPU-interlocked, needing real status-register handshaking this phase doesn't build -- see that
// same comment block for detail). Ported from the raw MAME v9938.cpp, not AppleII-VGA's own
// version, which never finishes this (its own update_command() runs in one 256-iteration burst at
// the very end of each frame -- here it's stepped once per rendered line instead, so blitter-drawn
// content shows up progressively within the frame it's drawn, not a full frame late).

#ifdef FLOPPY_OVERLAY
#include "fdc_overlay.h"
#endif
#include "tms9918a.h"

#include <string.h>
#include <pico/stdlib.h>
#include "colors.h"
#include "vga.h"


#define VRAM_SIZE 0x20000
static volatile uint8_t vram[VRAM_SIZE];

// GRAPHIC7 (Screen8) pixels are direct colour bytes, not palette indices -- this table converts
// one of the 256 possible byte values straight to this project's packed RGB333 uint16_t (same
// convention tms9918a_palette[] uses, colors.h), built once in tms9918a_init(). Formula ported from
// AppleII-VGA's v9938_reset_palette(): bits 7-5 = green, bits 4-2 = red, bits 1-0 = blue, with blue's
// 2 bits expanded to 3 as {0,2,4,7} rather than linearly -- a real V9938 hardware quirk (closer to
// full brightness at blue=3 than a plain <<1 would give), not a transcription choice.
static uint16_t graphic7_palette[256];

// The 16-colour palette every other mode (GRAPHIC1/MULTI/TEXT1/GRAPHIC2/GRAPHIC4/5/6, sprite modes
// 1 and 2) indexes into -- this project's own mutable copy, seeded from colors.c's
// tms9918a_palette[] defaults at the end of tms9918a_init() so the picture is unchanged until
// software actually writes a palette entry via port $9A. Deliberately not colors.c's own
// tms9918a_palette[] itself: that stays the shared, untouched default for -DV9938=OFF builds.
static uint16_t v9938_palette[16];

// Full-width register/status storage (see the file comment): only R#0/1/2/3/4/5/6/7/8/10/11/14/15/23
// have active consumers this phase. Everything else is inert, ready for later phases.
static uint8_t m_cont_reg[48];
static uint8_t m_stat_reg[10];

static uint16_t m_address_latch;   // 14-bit VRAM offset within the page R#14 selects
static uint8_t m_read_ahead;
static uint8_t m_cmd_write;        // first byte of the two-byte $99 sequence
static bool m_cmd_write_first;     // which byte of that sequence comes next
static uint8_t m_pal_write;        // first byte of the separate two-byte $9A palette sequence
static bool m_pal_write_first;
static uint8_t m_mode;

// Who touches what: identical split to tms9918a.c (see its own comment) -- the bus core (core 0)
// owns the registers, VRAM, the address latch and the two-byte write state; the render core (core 1)
// only reads them, and reports frame/sprite events through the counters below rather than touching
// the status byte directly.
static volatile uint32_t m_FrameEvents;
static volatile uint32_t m_CollisionEvents;
static volatile uint32_t m_FifthEvents;
static volatile uint8_t m_FifthAtEvent;
static volatile uint8_t m_FifthSprite;
static volatile uint32_t m_EventSeq;

static uint32_t m_FrameSeen;
static uint32_t m_CollisionSeen;
static uint32_t m_FifthSeen;
static uint32_t m_SnapFrame;
static uint32_t m_SnapCollision;
static uint32_t m_SnapFifth;
static uint32_t m_EventSeqSeen;
static uint8_t m_StatusCache;   // what reading S#0 returns; see status_refresh()
static bool m_INT;


static void publish_event() {
    __atomic_thread_fence(__ATOMIC_RELEASE);
    m_EventSeq++;
}

void tms9918a_set_backdrop(uint8_t colour) {
    m_cont_reg[7] = colour & 0x0f;
}

extern void tms9918a_set_int(bool asserted);

// /INT is asserted while the frame flag is set and the frame interrupt is enabled (R#1 bit5, IE0) --
// same bit position and meaning as the TMS9918A. The V9938 line interrupt (IE1/R#19/S#1 bit0) is a
// later-phase addition; nothing sets it here.
static void check_interrupt() {
    bool b = (m_FrameEvents != m_FrameSeen) && (m_cont_reg[1] & 0x20);

    if(b != m_INT) {
        m_INT = b;
        tms9918a_set_int(b);
    }
}

static void set_mode() {
    m_mode = ((m_cont_reg[0] & 0x0e) << 1) | ((m_cont_reg[1] & 0x18) >> 3);
}

// Defined later in this file, after the command-engine state it resets is declared.
static void command_engine_reset(void);

void tms9918a_init() {
    memset(m_cont_reg, 0, sizeof(m_cont_reg));
    memset(m_stat_reg, 0, sizeof(m_stat_reg));
    command_engine_reset();

    m_FifthSprite = 31;
    m_FifthAtEvent = 31;
    m_FrameEvents = m_CollisionEvents = m_FifthEvents = 0;
    m_FrameSeen = m_CollisionSeen = m_FifthSeen = 0;
    m_SnapFrame = m_SnapCollision = m_SnapFifth = 0;
    m_EventSeq = m_EventSeqSeen = 0;
    m_StatusCache = 31;
    m_INT = false;

    m_address_latch = 0;
    m_read_ahead = 0;
    m_cmd_write = 0;
    m_cmd_write_first = false;
    m_pal_write = 0;
    m_pal_write_first = false;
    m_mode = 0;

    for(uint32_t x = 0; x < VRAM_SIZE; x++)
        vram[x] = 0;

    for(uint32_t i = 0; i < 256; i++) {
        uint32_t r = (i >> 2) & 7;
        uint32_t g = (i >> 5) & 7;
        uint32_t b = (i & 3) << 1;
        if(b == 6)
            b++;
        graphic7_palette[i] = (uint16_t)((r << 6) | (g << 3) | b);
    }

    // Seed the mutable 16-colour palette from the shared default table -- the picture looks
    // exactly like phase 1/2 did until software actually writes a palette entry via $9A.
    for(uint32_t i = 0; i < 16; i++)
        v9938_palette[i] = tms9918a_palette[i];
}

// Works out what reading S#0 returns. Only S#0 is dynamic this phase (frame/collision/5th-sprite);
// S#1-9 are inert stored bytes (always 0 for now -- nothing sets them), matching tms9918a.c's own
// "recompute fresh from counters, never cache a stale flag" approach for the one status register
// that actually changes on its own.
static void status_refresh() {
    m_SnapFrame = m_FrameEvents;
    m_SnapCollision = m_CollisionEvents;
    m_SnapFifth = m_FifthEvents;

    uint8_t value = 0;

    if(m_SnapFrame != m_FrameSeen)
        value |= 0x80;
    if(m_SnapCollision != m_CollisionSeen)
        value |= 0x20;
    if(m_SnapFifth != m_FifthSeen)
        value |= 0x40 | (m_FifthAtEvent & 0x1f);
    else
        value |= m_FifthSprite & 0x1f;

    m_StatusCache = value;
}

uint8_t tms9918a_read_value(uint_fast16_t address) {
    switch(address & 3) {
    case 0:
        return m_read_ahead;
    case 1: {
        const uint_fast8_t reg = m_cont_reg[15] & 0x0f;
        if(reg == 0)
            return m_StatusCache;
        if(reg <= 9)
            return m_stat_reg[reg];
        return 0xff;
    }
    default:
        // Palette ($9A) and indirect-register ($9B) are write-only on real hardware, same as
        // tms9918a_read_value's own "not meaningfully readable" convention for status 10+ above.
        return 0xff;
    }
}

// The CPU's VRAM address to a vram[] index. In GRAPHIC6/7 the V9938 interleaves its two banks, and vram[]
// holds them as the renderer and the command engine address them (bank = bit 0); the CPU's address is
// linear, so it is folded the same way, as AppleII-VGA's v9938_vram_write()/v9938_vram_read() (MAME) do.
static inline uint32_t cpu_vram_index(void) {
    const uint32_t a = ((uint32_t)m_cont_reg[14] << 14) | m_address_latch;
    if(m_mode == 0x14 || m_mode == 0x1c)
        return (((a & 1) << 16) | (a >> 1)) & (VRAM_SIZE - 1);
    return a & (VRAM_SIZE - 1);
}

// Shared by a plain VRAM read and by setting the address latch (which also pre-fetches the byte at
// the just-set address into the read-ahead buffer -- the standard VDP "read ahead" trick, ported
// faithfully from v9938_vram_r: a read returns what was pre-fetched *last* time, not what is at the
// address *now*).
static void vram_read_advance() {
    m_read_ahead = vram[cpu_vram_index()];
    m_address_latch = (m_address_latch + 1) & 0x3fff;
    if(!m_address_latch && (m_cont_reg[0] & 0x0c))
        m_cont_reg[14] = (m_cont_reg[14] + 1) & 7;
}

void tms9918a_read_done(uint_fast16_t address) {
    m_cmd_write_first = false;

    switch(address & 3) {
    case 0:
        vram_read_advance();
        break;
    case 1:
        if((m_cont_reg[15] & 0x0f) == 0) {
            m_FrameSeen = m_SnapFrame;
            m_CollisionSeen = m_SnapCollision;
            m_FifthSeen = m_SnapFifth;
            status_refresh();
            check_interrupt();
        }
        break;
    default:
        break;  // palette/indirect-register reads have no side effects to apply
    }
}

// ---------------------------------------------------------------------------------------------
// Command engine (the blitter, R#32-46). Triggered by a register write to R#46 (bus core, see
// command_unit_w, called from register_write's own case 46 below) and stepped forward
// continuously from the render core (update_command(), called once per rendered line -- see
// render_tms9918a()), not once per frame as AppleII-VGA's own v9938.c does: calling it once per
// frame in one 256-iteration burst at the very end means anything the blitter draws only becomes
// visible a full frame late. Ported from the raw MAME v9938.cpp (not AppleII-VGA's incomplete
// version), minus LMCM (CM 0xA): the CPU reads its result through the machine's own V9938, which a
// listen-only mirror cannot feed. LMMC and HMMC (CM 0xB/0xF) take their data from R#44 writes, which
// the game paces by S#2 bit7 (TR) on the machine's own V9938; core 0 queues those bytes in
// m_xfer_ring and core 1's engine draws them as they come, instead of the reference's TR handshake.
//
// Cross-core note: every earlier phase kept the render core read-only of registers/VRAM
// specifically to avoid races (phase 1/2's whole "report events, never write status directly"
// fix). The command engine breaks that on purpose -- triggering happens on the bus core, stepping
// on the render core -- so m_vdp_engine (the one field both cores touch) is declared volatile, and
// command_unit_w writes every other m_mmc field *first*, release-fences, *then* assigns
// m_vdp_engine last. update_command() checks the volatile m_vdp_engine first; only once it's
// non-null does it acquire-fence and read the m_mmc fields -- so it only ever sees a fully-old or
// fully-new argument set, never a torn mix. Same idiom this file's own publish_event()/m_EventSeq
// already uses, just with the producer/consumer roles swapped. Re-triggering a command before the
// previous one's CE (S#0... S#2 bit0) clears is still possible -- the reference itself only logs a
// warning, never prevents it -- but that matches real hardware's own documented expectation that
// software polls CE first; this fix only guarantees no *torn* state, not that software behaving
// badly produces a sensible picture.
//
// VRAM addressing: ported directly from VDP_VRMP5/6/7/8 (point-based, not the renderer's own
// sequential-byte-scan reformulation of the same hardware interleaving -- confirmed by hand to
// produce identical addresses for GRAPHIC6, so either is correct; this one is the natural fit for
// commands that address arbitrary (X,Y) points). Second-field/expanded-VRAM addressing (MXS/MXD,
// EXPMEM_OFFSET in the reference) is not supported -- no second 128K VRAM bank, same limitation
// the renderer already has -- so every computed address is masked to this file's one 128K vram[]
// regardless, folding an attempted second-field access back into the valid range rather than
// reading/writing out of bounds.
static struct {
    int SX, SY, DX, DY;
    int TX, TY;
    int NX, NY;
    int ASX, ADX, ANX;
    int MX;
    uint8_t CL;
    uint8_t LO;
    uint8_t CM;
    bool MXS, MXD;
    uint32_t XS;   // HMMC/LMMC: m_xfer_ring index of this command's first byte
    bool XNEW;     // HMMC/LMMC: not stepped yet, the engine still has to move m_xfer_tail to XS
} m_mmc;
static void (*volatile m_vdp_engine)(void);
static int m_vdp_ops_count;

// HMMC/LMMC data: the game writes it to R#44 one byte at a time, paced by the machine's own V9938's
// TR bit, which we never see. Core 0 puts every R#44 write here (head) and core 1's engine takes them
// (tail), so no byte is lost when core 1 is busy rendering. Same publish idiom as m_vdp_engine: the
// byte is written, release fence, then the head.
#define XFER_RING_SIZE 128  // 256 until C30, halved for RAM: core 1 drains it whenever idle
static uint8_t m_xfer_ring[XFER_RING_SIZE];
static volatile uint32_t m_xfer_head;  // written by core 0 only
static volatile uint32_t m_xfer_tail;  // written by core 1 only

static inline void xfer_push(uint8_t data) {
    const uint32_t head = m_xfer_head;
    m_xfer_ring[head & (XFER_RING_SIZE - 1)] = data;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    m_xfer_head = head + 1;
}

// Called from tms9918a_init() (defined earlier in this file, before m_mmc is in scope -- hence the
// forward declaration up there and this separate reset function instead of resetting inline).
static void command_engine_reset(void) {
    m_vdp_engine = 0;
    m_vdp_ops_count = 0;
    memset(&m_mmc, 0, sizeof(m_mmc));
}

static const uint8_t cmd_mask[4] = { 0x0f, 0x03, 0x0f, 0xff };  // colour bits per mode (GRAPHIC4-7)
static const int cmd_ppb[4] = { 2, 4, 2, 1 };                   // pixels per byte
static const int cmd_ppl[4] = { 256, 512, 512, 256 };           // pixels per line (end-of-row mask)

// Timing tables (ops per step, one of 8 per command type depending on display-blanked/sprites-off/
// PAL-vs-NTSC), and srch/line/lmmv/ymmm/hmmm/lmmm/hmmv_timing, all ported verbatim from v9938.cpp.
static const int srch_timing[8] = { 818, 1025, 818, 830, 696, 854, 696, 684 };
static const int line_timing[8] = { 1063, 1259, 1063, 1161, 904, 1026, 904, 953 };
static const int hmmv_timing[8] = { 439, 549, 439, 531, 366, 439, 366, 427 };
static const int lmmv_timing[8] = { 873, 1135, 873, 1056, 732, 909, 732, 854 };
static const int ymmm_timing[8] = { 586, 952, 586, 610, 488, 720, 488, 500 };
static const int hmmm_timing[8] = { 818, 1111, 818, 854, 684, 879, 684, 708 };
static const int lmmm_timing[8] = { 1160, 1599, 1160, 1172, 964, 1257, 964, 977 };

static int get_vdp_timing_value(const int timing[8]) {
    return timing[((m_cont_reg[1] >> 6) & 1) | (m_cont_reg[8] & 2) | ((m_cont_reg[9] << 1) & 4)];
}

// Which of the 4 bitmap modes (GRAPHIC4=0/5=1/6=2/7=3) m_mode currently selects, or -1 if it's not
// one of them (the command engine only operates in Screen 5-8). The reference's own formula is
// "m_mode - 5" in MAME's mode numbering, which is linear for GRAPHIC4-6 in this file's own
// GRAPHIC4=0x0c/5=0x10/6=0x14 numbering too, but not GRAPHIC7=0x1c (0x18 isn't a real mode in
// between) -- hence the explicit special case, ported as-is.
static int command_screen_index() {
    if(m_mode == 0x1c)
        return 3;
    if(m_mode == 0x0c || m_mode == 0x10 || m_mode == 0x14)
        return (m_mode >> 2) - 3;
    return -1;
}

static uint32_t vram_addr(int sm, uint32_t x, uint32_t y) {
    uint32_t addr;
    switch(sm) {
    case 0:  addr = ((y & 1023) << 7) + ((x & 255) >> 1); break;
    case 1:  addr = ((y & 1023) << 7) + ((x & 511) >> 2); break;
    case 2:  addr = ((x & 2) << 15) + ((y & 511) << 7) + ((x & 511) >> 2); break;
    default: addr = ((x & 1) << 16) + ((y & 511) << 7) + ((x >> 1) & 127); break;
    }
    return addr & (VRAM_SIZE - 1);
}

static uint8_t vdp_point(int sm, uint32_t x, uint32_t y) {
    switch(sm) {
    case 0:  return (vram[vram_addr(0, x, y)] >> (((~x) & 1) << 2)) & 15;
    case 1:  return (vram[vram_addr(1, x, y)] >> (((~x) & 3) << 1)) & 3;
    case 2:  return (vram[vram_addr(2, x, y)] >> (((~x) & 1) << 2)) & 15;
    default: return vram[vram_addr(3, x, y)];
    }
}

// SET/AND/OR/XOR/NOT and their "T"-prefixed (transparent: skip entirely when CL==0) variants --
// ported verbatim from VDPpsetlowlevel.
static void vdp_pset_lowlevel(uint32_t addr, uint8_t cl, uint8_t mask, uint8_t op) {
    uint8_t val = vram[addr];
    switch(op) {
    case 0:  val = (val & mask) | cl; break;
    case 1:  val = val & (cl | mask); break;
    case 2:  val |= cl; break;
    case 3:  val ^= cl; break;
    case 4:  val = (val & mask) | (uint8_t)~(cl | mask); break;
    case 8:  if(cl) val = (val & mask) | cl; break;
    case 9:  if(cl) val = val & (cl | mask); break;
    case 10: if(cl) val |= cl; break;
    case 11: if(cl) val ^= cl; break;
    case 12: if(cl) val = (val & mask) | (uint8_t)~(cl | mask); break;
    default: break;  // invalid LO value -- the reference just logs a warning and does nothing
    }
    vram[addr] = val;
}

static void vdp_pset(int sm, uint32_t x, uint32_t y, uint8_t cl, uint8_t op) {
    uint8_t sh;
    switch(sm) {
    case 0:  sh = ((~x) & 1) << 2; vdp_pset_lowlevel(vram_addr(0, x, y), (uint8_t)(cl << sh), (uint8_t)~(15 << sh), op); break;
    case 1:  sh = ((~x) & 3) << 1; vdp_pset_lowlevel(vram_addr(1, x, y), (uint8_t)(cl << sh), (uint8_t)~(3 << sh), op); break;
    case 2:  sh = ((~x) & 1) << 2; vdp_pset_lowlevel(vram_addr(2, x, y), (uint8_t)(cl << sh), (uint8_t)~(15 << sh), op); break;
    default: vdp_pset_lowlevel(vram_addr(3, x, y), cl, 0, op); break;
    }
}

// ---- the 7 in-scope engine functions, each stepping forward by up to m_vdp_ops_count/delta
// iterations and saving its locals back into m_mmc if the budget ran out before finishing ----

static void srch_engine() {
    int SX = m_mmc.SX, SY = m_mmc.SY, TX = m_mmc.TX, ANX = m_mmc.ANX;
    uint8_t CL = m_mmc.CL;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(srch_timing);
    int cnt = m_vdp_ops_count;

    while((cnt -= delta) > 0) {
        if((vdp_point(sm, (uint32_t)SX, (uint32_t)SY) == CL) ^ ANX) {
            m_stat_reg[2] |= 0x10;  // border detected
            break;
        }
        SX += TX;
        if(SX & mx) {
            m_stat_reg[2] &= 0xef;  // border not detected
            break;
        }
    }

    if((m_vdp_ops_count = cnt) > 0) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        m_stat_reg[8] = SX & 0xff;
        m_stat_reg[9] = (SX >> 8) | 0xfe;
    } else {
        m_mmc.SX = SX;
    }
}

static void line_engine() {
    int DX = m_mmc.DX, DY = m_mmc.DY, TX = m_mmc.TX, TY = m_mmc.TY;
    int NX = m_mmc.NX, NY = m_mmc.NY, ASX = m_mmc.ASX, ADX = m_mmc.ADX;
    uint8_t CL = m_mmc.CL, LO = m_mmc.LO;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(line_timing);
    int cnt = m_vdp_ops_count;

    if(!(m_cont_reg[45] & 0x01)) {
        while((cnt -= delta) > 0) {
            vdp_pset(sm, (uint32_t)DX, (uint32_t)DY, CL, LO);
            DX += TX;
            if((ASX -= NY) < 0) { ASX += NX; DY += TY; }
            ASX &= 1023;
            if(ADX++ == NX || (DX & mx))
                break;
        }
    } else {
        while((cnt -= delta) > 0) {
            vdp_pset(sm, (uint32_t)DX, (uint32_t)DY, CL, LO);
            DY += TY;
            if((ASX -= NY) < 0) { ASX += NX; DX += TX; }
            ASX &= 1023;
            if(ADX++ == NX || (DX & mx))
                break;
        }
    }

    if((m_vdp_ops_count = cnt) > 0) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        m_cont_reg[38] = DY & 0xff;
        m_cont_reg[39] = (DY >> 8) & 0x03;
    } else {
        m_mmc.DX = DX; m_mmc.DY = DY; m_mmc.ASX = ASX; m_mmc.ADX = ADX;
    }
}

static void lmmv_engine() {
    int DX = m_mmc.DX, DY = m_mmc.DY, TX = m_mmc.TX, TY = m_mmc.TY;
    int NX = m_mmc.NX, NY = m_mmc.NY, ADX = m_mmc.ADX, ANX = m_mmc.ANX;
    uint8_t CL = m_mmc.CL, LO = m_mmc.LO;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(lmmv_timing);
    int cnt = m_vdp_ops_count;

    while((cnt -= delta) > 0) {
        vdp_pset(sm, (uint32_t)ADX, (uint32_t)DY, CL, LO);
        if(!--ANX || ((ADX += TX) & mx)) {
            if(!(--NY & 1023) || (DY += TY) == -1)
                break;
            ADX = DX; ANX = NX;
        }
    }

    if((m_vdp_ops_count = cnt) > 0) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        if(!NY)
            DY += TY;
        m_cont_reg[38] = DY & 0xff; m_cont_reg[39] = (DY >> 8) & 0x03;
        m_cont_reg[42] = NY & 0xff; m_cont_reg[43] = (NY >> 8) & 0x03;
    } else {
        m_mmc.DY = DY; m_mmc.NY = NY; m_mmc.ANX = ANX; m_mmc.ADX = ADX;
    }
}

// HMMC and LMMC: HMMV and LMMV with each byte (HMMC) or pixel (LMMC) taken from m_xfer_ring instead of a
// fixed colour; a step ends early once the ring is empty, and the command waits for the next R#44 write.
static bool xfer_next(uint32_t *tail, uint32_t head, int *cnt, int delta) {
    if(*tail == head)
        return false;
    return (*cnt -= delta) > 0;
}

static void xfer_start(void) {
    if(m_mmc.XNEW) {
        m_xfer_tail = m_mmc.XS;
        m_mmc.XNEW = false;
    }
}

static void hmmc_engine() {
    xfer_start();
    int DX = m_mmc.DX, DY = m_mmc.DY, TX = m_mmc.TX, TY = m_mmc.TY;
    int NX = m_mmc.NX, NY = m_mmc.NY, ADX = m_mmc.ADX, ANX = m_mmc.ANX;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(hmmv_timing);
    int cnt = m_vdp_ops_count;
    uint32_t tail = m_xfer_tail;
    const uint32_t head = m_xfer_head;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    bool done = false;

    while(xfer_next(&tail, head, &cnt, delta)) {
        vram[vram_addr(sm, (uint32_t)ADX, (uint32_t)DY)] = m_xfer_ring[tail++ & (XFER_RING_SIZE - 1)];
        if(!--ANX || ((ADX += TX) & mx)) {
            if(!(--NY & 1023) || (DY += TY) == -1) {
                done = true;
                break;
            }
            ADX = DX; ANX = NX;
        }
    }

    m_xfer_tail = tail;
    m_vdp_ops_count = cnt;
    if(done) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        if(!NY)
            DY += TY;
        m_cont_reg[42] = NY & 0xff; m_cont_reg[43] = (NY >> 8) & 0x03;
        m_cont_reg[38] = DY & 0xff; m_cont_reg[39] = (DY >> 8) & 0x03;
    } else {
        m_mmc.DY = DY; m_mmc.NY = NY; m_mmc.ANX = ANX; m_mmc.ADX = ADX;
    }
}

static void lmmc_engine() {
    xfer_start();
    int DX = m_mmc.DX, DY = m_mmc.DY, TX = m_mmc.TX, TY = m_mmc.TY;
    int NX = m_mmc.NX, NY = m_mmc.NY, ADX = m_mmc.ADX, ANX = m_mmc.ANX;
    uint8_t LO = m_mmc.LO;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(lmmv_timing);
    int cnt = m_vdp_ops_count;
    uint32_t tail = m_xfer_tail;
    const uint32_t head = m_xfer_head;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    bool done = false;

    while(xfer_next(&tail, head, &cnt, delta)) {
        const uint8_t cl = m_xfer_ring[tail++ & (XFER_RING_SIZE - 1)] & cmd_mask[sm];
        vdp_pset(sm, (uint32_t)ADX, (uint32_t)DY, cl, LO);
        if(!--ANX || ((ADX += TX) & mx)) {
            if(!(--NY & 1023) || (DY += TY) == -1) {
                done = true;
                break;
            }
            ADX = DX; ANX = NX;
        }
    }

    m_xfer_tail = tail;
    m_vdp_ops_count = cnt;
    if(done) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        if(!NY)
            DY += TY;
        m_cont_reg[42] = NY & 0xff; m_cont_reg[43] = (NY >> 8) & 0x03;
        m_cont_reg[38] = DY & 0xff; m_cont_reg[39] = (DY >> 8) & 0x03;
    } else {
        m_mmc.DY = DY; m_mmc.NY = NY; m_mmc.ANX = ANX; m_mmc.ADX = ADX;
    }
}

static void lmmm_engine() {
    int SX = m_mmc.SX, SY = m_mmc.SY, DX = m_mmc.DX, DY = m_mmc.DY;
    int TX = m_mmc.TX, TY = m_mmc.TY, NX = m_mmc.NX, NY = m_mmc.NY;
    int ASX = m_mmc.ASX, ADX = m_mmc.ADX, ANX = m_mmc.ANX;
    uint8_t LO = m_mmc.LO;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(lmmm_timing);
    int cnt = m_vdp_ops_count;

    while((cnt -= delta) > 0) {
        vdp_pset(sm, (uint32_t)ADX, (uint32_t)DY, vdp_point(sm, (uint32_t)ASX, (uint32_t)SY), LO);
        if(!--ANX || ((ASX += TX) & mx) || ((ADX += TX) & mx)) {
            if(!(--NY & 1023) || (SY += TY) == -1 || (DY += TY) == -1)
                break;
            ASX = SX; ADX = DX; ANX = NX;
        }
    }

    if((m_vdp_ops_count = cnt) > 0) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        if(!NY) { SY += TY; DY += TY; }
        else if(SY == -1)
            DY += TY;
        m_cont_reg[42] = NY & 0xff; m_cont_reg[43] = (NY >> 8) & 0x03;
        m_cont_reg[34] = SY & 0xff; m_cont_reg[35] = (SY >> 8) & 0x03;
        m_cont_reg[38] = DY & 0xff; m_cont_reg[39] = (DY >> 8) & 0x03;
    } else {
        m_mmc.SY = SY; m_mmc.DY = DY; m_mmc.NY = NY;
        m_mmc.ANX = ANX; m_mmc.ASX = ASX; m_mmc.ADX = ADX;
    }
}

static void hmmv_engine() {
    int DX = m_mmc.DX, DY = m_mmc.DY, TX = m_mmc.TX, TY = m_mmc.TY;
    int NX = m_mmc.NX, NY = m_mmc.NY, ADX = m_mmc.ADX, ANX = m_mmc.ANX;
    uint8_t CL = m_mmc.CL;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(hmmv_timing);
    int cnt = m_vdp_ops_count;

    while((cnt -= delta) > 0) {
        vram[vram_addr(sm, (uint32_t)ADX, (uint32_t)DY)] = CL;
        if(!--ANX || ((ADX += TX) & mx)) {
            if(!(--NY & 1023) || (DY += TY) == -1)
                break;
            ADX = DX; ANX = NX;
        }
    }

    if((m_vdp_ops_count = cnt) > 0) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        if(!NY)
            DY += TY;
        m_cont_reg[42] = NY & 0xff; m_cont_reg[43] = (NY >> 8) & 0x03;
        m_cont_reg[38] = DY & 0xff; m_cont_reg[39] = (DY >> 8) & 0x03;
    } else {
        m_mmc.DY = DY; m_mmc.NY = NY; m_mmc.ANX = ANX; m_mmc.ADX = ADX;
    }
}

static void hmmm_engine() {
    int SX = m_mmc.SX, SY = m_mmc.SY, DX = m_mmc.DX, DY = m_mmc.DY;
    int TX = m_mmc.TX, TY = m_mmc.TY, NX = m_mmc.NX, NY = m_mmc.NY;
    int ASX = m_mmc.ASX, ADX = m_mmc.ADX, ANX = m_mmc.ANX;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(hmmm_timing);
    int cnt = m_vdp_ops_count;

    while((cnt -= delta) > 0) {
        vram[vram_addr(sm, (uint32_t)ADX, (uint32_t)DY)] = vram[vram_addr(sm, (uint32_t)ASX, (uint32_t)SY)];
        if(!--ANX || ((ASX += TX) & mx) || ((ADX += TX) & mx)) {
            if(!(--NY & 1023) || (SY += TY) == -1 || (DY += TY) == -1)
                break;
            ASX = SX; ADX = DX; ANX = NX;
        }
    }

    if((m_vdp_ops_count = cnt) > 0) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        if(!NY) { SY += TY; DY += TY; }
        else if(SY == -1)
            DY += TY;
        m_cont_reg[42] = NY & 0xff; m_cont_reg[43] = (NY >> 8) & 0x03;
        m_cont_reg[34] = SY & 0xff; m_cont_reg[35] = (SY >> 8) & 0x03;
        m_cont_reg[38] = DY & 0xff; m_cont_reg[39] = (DY >> 8) & 0x03;
    } else {
        m_mmc.SY = SY; m_mmc.DY = DY; m_mmc.NY = NY;
        m_mmc.ANX = ANX; m_mmc.ASX = ASX; m_mmc.ADX = ADX;
    }
}

static void ymmm_engine() {
    int SY = m_mmc.SY, DX = m_mmc.DX, DY = m_mmc.DY, TX = m_mmc.TX, TY = m_mmc.TY;
    int NY = m_mmc.NY, ADX = m_mmc.ADX;
    const int sm = command_screen_index();
    if(sm < 0) { m_vdp_engine = 0; return; }
    const int mx = cmd_ppl[sm];
    int delta = get_vdp_timing_value(ymmm_timing);
    int cnt = m_vdp_ops_count;

    while((cnt -= delta) > 0) {
        vram[vram_addr(sm, (uint32_t)ADX, (uint32_t)DY)] = vram[vram_addr(sm, (uint32_t)ADX, (uint32_t)SY)];
        if((ADX += TX) & mx) {
            if(!(--NY & 1023) || (SY += TY) == -1 || (DY += TY) == -1)
                break;
            ADX = DX;
        }
    }

    if((m_vdp_ops_count = cnt) > 0) {
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        if(!NY) { SY += TY; DY += TY; }
        else if(SY == -1)
            DY += TY;
        m_cont_reg[42] = NY & 0xff; m_cont_reg[43] = (NY >> 8) & 0x03;
        m_cont_reg[34] = SY & 0xff; m_cont_reg[35] = (SY >> 8) & 0x03;
        m_cont_reg[38] = DY & 0xff; m_cont_reg[39] = (DY >> 8) & 0x03;
    } else {
        m_mmc.SY = SY; m_mmc.DY = DY; m_mmc.NY = NY; m_mmc.ADX = ADX;
    }
}

// Triggers a command (register write to R#46, called from register_write's own case 46 below).
// Ported from v9938.cpp's command_unit_w -- LMCM/LMMC/HMMC excluded, see the file-level comment.
static void command_unit_w(uint8_t op) {
    const int sm = command_screen_index();
    if(sm < 0)
        return;  // V9938 commands only work in Screen 5-8

    const uint8_t cm = op >> 4;
    if((cm & 0x0c) != 0x0c && cm != 0) {
        // Dot operation: use only the colour bits this mode actually has
        m_cont_reg[44] &= cmd_mask[sm];
        m_stat_reg[7] = m_cont_reg[44];
    }

    switch(cm) {
    case 0x0:  // ABRT
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        return;
    case 0x4:  // POINT
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        m_stat_reg[7] = m_cont_reg[44] = vdp_point(sm,
            (uint32_t)(m_cont_reg[32] + ((int)m_cont_reg[33] << 8)),
            (uint32_t)(m_cont_reg[34] + ((int)m_cont_reg[35] << 8)));
        return;
    case 0x5:  // PSET
        m_stat_reg[2] &= 0xfe;
        m_vdp_engine = 0;
        vdp_pset(sm,
            (uint32_t)(m_cont_reg[36] + ((int)m_cont_reg[37] << 8)),
            (uint32_t)(m_cont_reg[38] + ((int)m_cont_reg[39] << 8)),
            m_cont_reg[44], op & 0x0f);
        return;
    case 0x6: case 0x7: case 0x8: case 0x9: case 0xb: case 0xc: case 0xd: case 0xe: case 0xf:
        break;  // SRCH/LINE/LMMV/LMMM/LMMC/HMMV/HMMM/YMMM/HMMC -- set up below
    default:
        return;  // LMCM(0xa): the game reads its result from the machine's own V9938; nothing to mirror
    }

    m_mmc.SX = (m_cont_reg[32] + ((int)m_cont_reg[33] << 8)) & 511;
    m_mmc.SY = (m_cont_reg[34] + ((int)m_cont_reg[35] << 8)) & 1023;
    m_mmc.DX = (m_cont_reg[36] + ((int)m_cont_reg[37] << 8)) & 511;
    m_mmc.DY = (m_cont_reg[38] + ((int)m_cont_reg[39] << 8)) & 1023;
    m_mmc.NY = (m_cont_reg[42] + ((int)m_cont_reg[43] << 8)) & 1023;
    m_mmc.TY = (m_cont_reg[45] & 0x08) ? -1 : 1;
    m_mmc.MX = cmd_ppl[sm];
    m_mmc.CL = m_cont_reg[44];
    m_mmc.LO = op & 0x0f;
    m_mmc.CM = cm;
    m_mmc.MXS = (m_cont_reg[45] & 0x10) != 0;
    m_mmc.MXD = (m_cont_reg[45] & 0x20) != 0;

    if((cm & 0x0c) == 0x0c) {
        m_mmc.TX = (m_cont_reg[45] & 0x04) ? -cmd_ppb[sm] : cmd_ppb[sm];
        m_mmc.NX = ((m_cont_reg[40] + ((int)m_cont_reg[41] << 8)) & 1023) / cmd_ppb[sm];
    } else {
        m_mmc.TX = (m_cont_reg[45] & 0x04) ? -1 : 1;
        m_mmc.NX = (m_cont_reg[40] + ((int)m_cont_reg[41] << 8)) & 1023;
    }

    if(cm == 0x7) {  // LINE
        m_mmc.ASX = (m_mmc.NX - 1) >> 1;
        m_mmc.ADX = 0;
    } else {
        m_mmc.ASX = m_mmc.SX;
        m_mmc.ADX = m_mmc.DX;
    }

    if(cm == 0x6)  // SRCH
        m_mmc.ANX = (m_cont_reg[45] & 0x02) != 0;
    else
        m_mmc.ANX = m_mmc.NX;

    m_stat_reg[2] |= 0x01;  // CE: command executing

    void (*engine)(void);
    switch(cm) {
    case 0x6: engine = srch_engine; break;
    case 0x7: engine = line_engine; break;
    case 0x8: engine = lmmv_engine; break;
    case 0x9: engine = lmmm_engine; break;
    case 0xc: engine = hmmv_engine; break;
    case 0xd: engine = hmmm_engine; break;
    case 0xb: engine = lmmc_engine; break;
    case 0xf: engine = hmmc_engine; break;
    default:  engine = ymmm_engine; break;  // 0xe
    }

    if(cm == 0xb || cm == 0xf) {
        // The first byte is the one already in R#44 (the game sets it before R#46); bytes still in the
        // ring from before this command are skipped
        m_mmc.XS = m_xfer_head;
        m_mmc.XNEW = true;
        xfer_push(m_cont_reg[44]);
    }

    // See the file-level cross-core note: every m_mmc field above is written before this fence,
    // and m_vdp_engine (volatile) is assigned only after it.
    __atomic_thread_fence(__ATOMIC_RELEASE);
    m_vdp_engine = engine;

    // The reference starts the engine here too ("if(m_vdp_ops_count > 0) engine()"), but this runs on
    // core 0, in the bus loop: that stalled the bus for a burst of copying, and ran the engine at the
    // same time as core 1's update_command(), both cores stepping the same m_mmc. Only core 1 steps it.
}

// Steps the active command forward. Called once per rendered scanline (render_tms9918a()), not
// once per frame -- see the file-level comment on why that matters. Ports update_command()
// verbatim: grant another 13662-ops budget if the last one ran dry, otherwise reset to exactly
// 13662, then step once if a command is active.
static void update_command() {
    void (*engine)(void) = m_vdp_engine;  // one volatile read -- see the file-level cross-core note
    if(m_vdp_ops_count <= 0) {
        m_vdp_ops_count += 13662;
        if(engine && m_vdp_ops_count > 0) {
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            engine();
        }
    } else {
        m_vdp_ops_count = 13662;
        if(engine) {
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
            engine();
        }
    }
}

// Called by vga.c while core 1 waits for a free scanline buffer, which is most of its time, the gap
// between frames included (render_tms9918a()'s 4 steps per line stop for the ~100 lines outside the
// picture). The game starts its next command once the machine's own V9938 says the last one is done;
// a command still waiting here then was overwritten, and its tile never drawn (black squares in Usas).
void tms9918a_idle(void) {
    update_command();
}

// Register write semantics for R#0-7 are unchanged from the TMS9918A (same bit positions). R#44
// (colour argument / POINT result) needs no case of its own -- command_unit_w reads/writes it
// directly, same plain storage as always. R#46 triggers a command (case below).
static void register_write(unsigned int reg, uint8_t data) {
    static const uint8_t reg_mask[28] = {
        0x7e, 0x7b, 0x7f, 0xff, 0x3f, 0xff, 0x3f, 0xff,
        0xfb, 0xbf, 0x07, 0x03, 0xff, 0xff, 0x07, 0x0f,
        0x0f, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x00, 0x7f, 0x3f, 0x07,
    };

    if(reg > 46)
        return;

    if(reg <= 27) {
        data &= reg_mask[reg];
        if(m_cont_reg[reg] == data)
            return;
    }

    if(reg == 0 || reg == 1) {
        m_cont_reg[reg] = data;
        set_mode();
        check_interrupt();
        return;
    }

    if(reg == 15) {
        // Selecting a different status register also cancels any half-completed $9A palette
        // write (matches the reference's own v9938_register_write exactly).
        m_pal_write_first = false;
    }

    if(reg == 44)
        xfer_push(data);  // HMMC/LMMC data, if a command is waiting for it (see m_xfer_ring)

    if(reg == 46) {
        command_unit_w(data);
        m_cont_reg[reg] = data;
        return;
    }

    m_cont_reg[reg] = data;
}

// Port $9B: writes to whichever register R#17 (masked to 6 bits) currently points at, then
// auto-increments R#17 unless its own bit7 (hold) is set. A convenience over the two-byte $99
// sequence, not a separate register space -- reuses register_write() directly.
static void indirect_register_write(uint8_t data) {
    uint_fast8_t reg = m_cont_reg[17] & 0x3f;
    if(reg != 17)
        register_write(reg, data);
    if(!(m_cont_reg[17] & 0x80))
        m_cont_reg[17] = (m_cont_reg[17] + 1) & 0x3f;
}

// Port $9A: the two-byte palette write sequence. Byte 1 packs red (bits6-4) and blue (bits2-0);
// byte 2 supplies green (bits2-0). Written to whichever index R#16 (masked to 4 bits) currently
// holds, which then auto-increments -- a separate two-byte sequence from the $99 one above, with
// its own first-byte latch (m_pal_write/m_pal_write_first), not shared state.
static void palette_write(uint8_t data) {
    if(m_pal_write_first) {
        const uint_fast8_t index = m_cont_reg[16] & 0x0f;
        const uint32_t r = (m_pal_write >> 4) & 7;
        const uint32_t b = m_pal_write & 7;
        const uint32_t g = data & 7;
        v9938_palette[index] = (uint16_t)((r << 6) | (g << 3) | b);
        m_cont_reg[16] = (m_cont_reg[16] + 1) & 0x0f;
        m_pal_write_first = false;
    } else {
        m_pal_write = data;
        m_pal_write_first = true;
    }
}

void tms9918a_write(uint_fast16_t address, uint8_t data) {
    switch(address & 3) {
    case 0:
        // VRAM data port. R#45 bit6 is a command-engine CPU-interlock flag (LMMC); it can never be
        // set this phase (nothing ports the command engine, so nothing ever sets it), so this gate
        // is always open in practice -- kept for faithfulness and so a later phase needs no change
        // here.
        if(!(m_cont_reg[45] & 0x40)) {
            vram[cpu_vram_index()] = data;
        }
        m_address_latch = (m_address_latch + 1) & 0x3fff;
        if(!m_address_latch && (m_cont_reg[0] & 0x0c))
            m_cont_reg[14] = (m_cont_reg[14] + 1) & 7;
        break;

    case 1:
        // Status/command port: the standard two-byte $99 sequence. First byte is always latched;
        // the second byte's top bits say what the pair meant -- bit7 set (and bit6 clear) is a
        // register write (register number is the second byte's low 6 bits), otherwise it sets the
        // VRAM address latch (bit6 clear also triggers the read-ahead pre-fetch above).
        if(m_cmd_write_first) {
            if(data & 0x80) {
                if(!(data & 0x40))
                    register_write(data & 0x3f, m_cmd_write);
            } else {
                m_address_latch = (((uint16_t)data << 8) | m_cmd_write) & 0x3fff;
                if(!(data & 0x40))
                    vram_read_advance();
            }
            m_cmd_write_first = false;
        } else {
            m_cmd_write = data;
            m_cmd_write_first = true;
        }
        break;

    case 2:
        palette_write(data);
        break;

    case 3:
        indirect_register_write(data);
        break;
    }
}

void tms9918a_frame_event() {
    m_FrameEvents++;
    publish_event();
}

void tms9918a_service() {
    uint32_t seq = m_EventSeq;

    if(seq != m_EventSeqSeen) {
        m_EventSeqSeen = seq;
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        status_refresh();
        check_interrupt();
    }
}


// ---------------------------------------------------------------------------------------------
// Render core (core 1): draws one frame. Modes 0 (GRAPHIC1), 1 (MULTI), 2 (TEXT1) and 4
// (GRAPHIC2) plus sprite mode 1 -- the TMS9918A-compatible subset -- ported from AppleII-VGA's
// v9938.c, fixes as described in the file header. Screen height stays fixed at 192 lines this
// phase (R#9 bit7's 212-line mode is a later-phase addition), matching tms9918a.c exactly.

static uint8_t col[256 + 4];  // sprite collision buffer; +4 margin, see the file header's 3rd fix

// AppleII-VGA's v9938_second_field() (MAME's v9938.cpp), less m_blink, which AppleII-VGA never sets.
// True whenever interlace (R#9 bit2) is off, so the R#2 bit5 page offsets below always apply then:
// that is the page select of Screen 5-8 (pages 1 and 3), not only an interlace feature. S#2 bit1
// (the field being shown) is never set here, so with interlace on the offset is never added.
static inline bool second_field(void) {
    return !((m_cont_reg[9] & 0x04) && !(m_stat_reg[2] & 0x02));
}

// GRAPHIC4/6 pens for one line: the palette, but with R#8 bit5 (TP) clear colour 0 is transparent and
// shows the backdrop colour (R#7), as AppleII-VGA's render does by swapping palette entry 0 for the
// line. A local copy rather than a swap: core 0 writes v9938_palette, and a restore could undo that.
static inline void bitmap_pens(uint32_t pens[16], uint32_t back_colour) {
    for(int i = 0; i < 16; i++)
        pens[i] = v9938_palette[i];
    if(!(m_cont_reg[8] & 0x20))
        pens[0] = v9938_palette[back_colour];
}

static void render_v9938_line(unsigned int line) {
    uint32_t sl_pos, sl_pos_line_start;
    struct vga_scanline *sl = vga_prepare_scanline();
    uint32_t bg, fg;
    uint32_t nametbl_addr, colourtbl_addr, patterntbl_addr, colourmask, patternmask, line2, name;
    uint32_t charcode, colour, pattern;

    sl_pos = 0;

    // Pad 40 pixels on the left to center horizontally (same prologue as tms9918a.c)
    sl->data[sl_pos++] = (0 | THEN_EXTEND_7) | ((0 | THEN_EXTEND_7) << 16);
    sl->data[sl_pos++] = (0 | THEN_EXTEND_7) | ((0 | THEN_EXTEND_7) << 16);
    sl->data[sl_pos++] = (0 | THEN_EXTEND_7) | ((0 | THEN_EXTEND_7) << 16);
    sl->data[sl_pos++] = (0 | THEN_EXTEND_3) | ((0 | THEN_EXTEND_3) << 16);
    sl_pos_line_start = sl_pos;

    const uint32_t BackColour = m_cont_reg[7] & 15;

    if(!(m_cont_reg[1] & 0x40)) {
        // Display blanked (R#1 bit6, BL): backdrop colour only. The AppleII-VGA source this was
        // ported from has no equivalent check at all; added to match tms9918a.c's existing,
        // hardware-proven behaviour for this case (otherwise whatever mode happens to be
        // configured still renders, which is visibly wrong during the windows software leaves the
        // display blanked).
        bg = v9938_palette[BackColour];
        for(int x = 0; x < 256; x += 2)
            sl->data[sl_pos++] = (bg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1;
    } else {
        memset(col, 0, sizeof(col));

        switch(m_mode) {
        case 0: /* GRAPHIC1 */
            nametbl_addr = m_cont_reg[2] << 10;
            colourtbl_addr = (m_cont_reg[3] << 6) + ((uint32_t)m_cont_reg[10] << 14);
            patterntbl_addr = m_cont_reg[4] << 11;

            line2 = (line - m_cont_reg[23]) & 255;
            name = (line2 >> 3) << 5;

            for(uint32_t x = 0; x < 32; x++) {
                charcode = vram[(nametbl_addr + name) & (VRAM_SIZE - 1)];
                colour = vram[(colourtbl_addr + (charcode >> 3)) & (VRAM_SIZE - 1)];
                // A colour-table nibble of 0 means "use the current backdrop colour", not literal
                // palette index 0 -- the ternary matters, not cosmetic (see the file header's 4th
                // fix: dropping it renders backdrop-coloured areas as black instead).
                fg = v9938_palette[(colour >> 4) ? (colour >> 4) : BackColour];
                bg = v9938_palette[(colour & 15) ? (colour & 15) : BackColour];
                pattern = vram[(patterntbl_addr + (charcode << 3) + (line2 & 7)) & (VRAM_SIZE - 1)];

                uint32_t bits_to_pixelpair[4] = {
                    (bg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1,
                    (fg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1,
                    (bg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1,
                    (fg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1,
                };
                for(int i = 0; i < 4; i++) {
                    sl->data[sl_pos++] = bits_to_pixelpair[(pattern >> 6) & 0x03];
                    pattern <<= 2;
                }
                name++;
            }
            break;

        case 1: /* MULTI (multicolor) */
            patterntbl_addr = m_cont_reg[4] << 11;
            nametbl_addr = m_cont_reg[2] << 10;

            line2 = (line - m_cont_reg[23]) & 255;
            name = (line2 >> 3) << 5;

            for(uint32_t x = 0; x < 32; x++) {
                colour = vram[(patterntbl_addr + (vram[(nametbl_addr + name) & (VRAM_SIZE - 1)] << 3)
                    + ((line2 >> 2) & 7)) & (VRAM_SIZE - 1)];
                // Same backdrop-substitution rule as GRAPHIC1 above -- see this file's 4th fix.
                fg = v9938_palette[(colour >> 4) ? (colour >> 4) : BackColour];
                bg = v9938_palette[(colour & 15) ? (colour & 15) : BackColour];
                // One 8x8 cell is 8 pixels wide: 4px fg (2 words) then 4px bg (2 words). The source
                // this was ported from drew 4+4 words (16px, double width) here -- see the file
                // header's 2nd fix.
                sl->data[sl_pos++] = (fg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1;
                sl->data[sl_pos++] = (fg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1;
                sl->data[sl_pos++] = (bg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1;
                sl->data[sl_pos++] = (bg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1;
                name++;
            }
            break;

        case 2: /* TEXT1 */
            patterntbl_addr = m_cont_reg[4] << 11;
            nametbl_addr = m_cont_reg[2] << 10;

            fg = v9938_palette[(m_cont_reg[7] >> 4) ? (m_cont_reg[7] >> 4) : BackColour];
            bg = v9938_palette[BackColour];
            uint32_t bits_to_pixelpair_text[4] = {
                (bg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1,
                (fg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1,
                (bg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1,
                (fg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1,
            };

            name = (line >> 3) * 40;
            for(uint32_t x = 0; x < 40; x++) {
                pattern = vram[(patterntbl_addr + (vram[(nametbl_addr + name) & (VRAM_SIZE - 1)] << 3)
                    + ((line + m_cont_reg[23]) & 7)) & (VRAM_SIZE - 1)];
                for(int i = 0; i < 3; i++) {
                    sl->data[sl_pos++] = bits_to_pixelpair_text[(pattern >> 6) & 0x03];
                    pattern <<= 2;
                }
                name = (name + 1) & 0x3ff;
            }
            break;

        case 4: /* GRAPHIC2 */
        case 0x08: /* GRAPHIC3 (Screen4): GRAPHIC2's tiles with sprite mode 2 (drawn below), as AppleII-VGA */
            colourmask = ((m_cont_reg[3] & 0x7f) << 3) | 7;
            patternmask = ((m_cont_reg[4] & 0x03) << 8) | 0xff;

            nametbl_addr = m_cont_reg[2] << 10;
            colourtbl_addr = ((uint32_t)(m_cont_reg[3] & 0x80) << 6) + ((uint32_t)m_cont_reg[10] << 14);
            patterntbl_addr = ((uint32_t)(m_cont_reg[4] & 0x3c) << 11);

            line2 = (line + m_cont_reg[23]) & 255;
            name = (line2 >> 3) << 5;

            for(uint32_t x = 0; x < 32; x++) {
                charcode = vram[(nametbl_addr + name) & (VRAM_SIZE - 1)] + ((line2 & 0xc0) << 2);
                colour = vram[(colourtbl_addr + (((charcode & colourmask) << 3) + (line2 & 7))) & (VRAM_SIZE - 1)];
                pattern = vram[(patterntbl_addr + (((charcode & patternmask) << 3) + (line2 & 7))) & (VRAM_SIZE - 1)];
                // Same backdrop-substitution rule as GRAPHIC1 above -- see this file's 4th fix.
                fg = v9938_palette[(colour >> 4) ? (colour >> 4) : BackColour];
                bg = v9938_palette[(colour & 15) ? (colour & 15) : BackColour];

                uint32_t bits_to_pixelpair[4] = {
                    (bg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1,
                    (fg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1,
                    (bg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1,
                    (fg << 16) | (THEN_EXTEND_1 << 16) | fg | THEN_EXTEND_1,
                };
                for(int i = 0; i < 4; i++) {
                    sl->data[sl_pos++] = bits_to_pixelpair[(pattern >> 6) & 0x03];
                    pattern <<= 2;
                }
                name++;
            }
            break;

        case 0x0c: { /* GRAPHIC4 (Screen5): 256 wide, 16 colours, 2 pixels/byte */
            uint32_t linemask = ((m_cont_reg[2] & 0x1f) << 3) | 7;
            line2 = ((line + m_cont_reg[23]) & linemask) & 255;
            nametbl_addr = ((uint32_t)(m_cont_reg[2] & 0x40) << 10) + line2 * 128;
            if((m_cont_reg[2] & 0x20) && second_field())
                nametbl_addr += 0x8000;
            uint32_t pens[16];
            bitmap_pens(pens, BackColour);
            for(uint32_t x = 0; x < 128; x++) {
                colour = vram[(nametbl_addr + x) & (VRAM_SIZE - 1)];
                uint32_t left = pens[colour >> 4];    // even x
                uint32_t right = pens[colour & 15];   // odd x
                sl->data[sl_pos++] = (right << 16) | (THEN_EXTEND_1 << 16) | left | THEN_EXTEND_1;
            }
            break;
        }

        case 0x10: { /* GRAPHIC5 (Screen6): 512 wide, 4 colours (2 bits/pixel), 4 pixels/byte */
            uint32_t linemask = ((m_cont_reg[2] & 0x1f) << 3) | 7;
            line2 = ((line + m_cont_reg[23]) & linemask) & 255;
            nametbl_addr = ((uint32_t)(m_cont_reg[2] & 0x40) << 10) + (line2 << 7);
            if((m_cont_reg[2] & 0x20) && second_field())
                nametbl_addr += 0x8000;

            // Colour index 0 means backdrop, same idea as GRAPHIC1/MULTI/GRAPHIC2's nibble-zero
            // substitution (file header's 4th fix) -- but R#7 packs two separate 2-bit backdrop
            // values (even/odd dot position), and R#8 bit5 clear is what activates the
            // substitution at all (ported faithfully from the source's own R#8-bit5-gated loop).
            uint32_t pen_bg0[4], pen_bg1[4];
            pen_bg1[0] = v9938_palette[m_cont_reg[7] & 0x03];
            pen_bg0[0] = v9938_palette[(m_cont_reg[7] >> 2) & 0x03];
            uint32_t start = (m_cont_reg[8] & 0x20) ? 0 : 1;
            for(uint32_t i = start; i < 4; i++) {
                pen_bg0[i] = v9938_palette[i];
                pen_bg1[i] = v9938_palette[i];
            }

            for(uint32_t x = 0; x < 128; x++) {
                colour = vram[(nametbl_addr + x) & (VRAM_SIZE - 1)];
                // No THEN_EXTEND_1 here: this mode is genuinely 512 distinct dots wide, two
                // per word, not a repeated pixel pair like the 256-wide modes above.
                sl->data[sl_pos++] = (pen_bg1[(colour >> 4) & 3] << 16) | pen_bg0[colour >> 6];
                sl->data[sl_pos++] = (pen_bg1[colour & 3] << 16) | pen_bg0[(colour >> 2) & 3];
            }
            break;
        }

        case 0x14: { /* GRAPHIC6 (Screen7): 512 wide, 16 colours, 2 pixels/byte, interleaved VRAM */
            uint32_t linemask = ((m_cont_reg[2] & 0x1f) << 3) | 7;
            line2 = ((line + m_cont_reg[23]) & linemask) & 255;
            uint32_t addr = line2 << 8;
            if((m_cont_reg[2] & 0x20) && second_field())
                addr += 0x10000;

            // R#2 bit6 gates a genuinely lower-fidelity path that reads one byte per 8 output
            // words instead of 1 -- confirmed against the raw MAME source (v9938.cpp's
            // mode_graphic6) to be real hardware-accurate behaviour, not a shortcut introduced by
            // whichever port; kept faithful rather than "fixed".
            uint32_t pens[16];
            bitmap_pens(pens, BackColour);
            if(m_cont_reg[2] & 0x40) {
                for(uint32_t x = 0; x < 32; x++) {
                    addr++;
                    colour = vram[(((addr & 1) << 16) | (addr >> 1)) & (VRAM_SIZE - 1)];
                    fg = pens[colour >> 4];
                    bg = pens[colour & 15];
                    for(int i = 0; i < 8; i++)
                        sl->data[sl_pos++] = (bg << 16) | fg;
                    addr += 7;
                }
            } else {
                for(uint32_t x = 0; x < 256; x++) {
                    colour = vram[(((addr & 1) << 16) | (addr >> 1)) & (VRAM_SIZE - 1)];
                    fg = pens[colour >> 4];
                    bg = pens[colour & 15];
                    sl->data[sl_pos++] = (bg << 16) | fg;
                    addr++;
                }
            }
            break;
        }

        case 0x1c: { /* GRAPHIC7 (Screen8): 256 distinct dots stretched over the same 512-dot
                        high-res timing as GRAPHIC5/6 (each byte duplicated across both halves of
                        a word, not split into two colours) -- 256 direct-colour bytes, no palette
                        table involved at all. */
            uint32_t linemask = ((m_cont_reg[2] & 0x1f) << 3) | 7;
            line2 = ((line + m_cont_reg[23]) & linemask) & 255;
            uint32_t addr = line2 << 8;
            if((m_cont_reg[2] & 0x20) && second_field())
                addr += 0x10000;

            // Same R#2 bit6 fast/slow split as GRAPHIC6 above, same faithfulness note.
            if(m_cont_reg[2] & 0x40) {
                for(uint32_t x = 0; x < 32; x++) {
                    addr++;
                    uint32_t pen = graphic7_palette[vram[(((addr & 1) << 16) | (addr >> 1)) & (VRAM_SIZE - 1)]];
                    pen |= pen << 16;
                    for(int i = 0; i < 8; i++)
                        sl->data[sl_pos++] = pen;
                    addr += 7;
                }
            } else {
                for(uint32_t x = 0; x < 256; x++) {
                    uint32_t pen = graphic7_palette[vram[(((addr & 1) << 16) | (addr >> 1)) & (VRAM_SIZE - 1)]];
                    sl->data[sl_pos++] = pen | (pen << 16);
                    addr++;
                }
            }
            break;
        }

        default:
            // Any other mode (a bogus bit combination) isn't wired up this phase --
            // draw the backdrop rather than garbage.
            bg = v9938_palette[BackColour];
            for(int x = 0; x < 256; x += 2)
                sl->data[sl_pos++] = (bg << 16) | (THEN_EXTEND_1 << 16) | bg | THEN_EXTEND_1;
            break;
        }

        // Sprite mode 1 (modes 0, 1 and 4 only -- TEXT1 never has sprites, matching real hardware).
        // Ported from AppleII-VGA's v9938.c with collision/5th-sprite reporting rewired onto the
        // same safe cross-core event mechanism tms9918a.c uses (see the file header's 1st fix).
        if((m_mode == 0 || m_mode == 1 || m_mode == 4) && !(m_cont_reg[8] & 0x02)) {
            const uint32_t attrtbl_addr_base = (m_cont_reg[5] << 7) + ((uint32_t)m_cont_reg[11] << 15);
            const uint32_t patterntbl_addr = m_cont_reg[6] << 11;
            uint32_t height = (m_cont_reg[1] & 2) ? 16 : 8;
            if(m_cont_reg[1] & 1)
                height *= 2;

            uint32_t attrtbl_addr = attrtbl_addr_base;
            bool collided = false;
            bool fifth_encountered = false;
            uint32_t drawn = 0;  // sprites actually drawn this line (p2 in the reference)
            uint32_t examined = 0;  // sprites examined this line (p in the reference)

            for(; examined < 32; examined++, attrtbl_addr += 4) {
                int y = vram[attrtbl_addr & (VRAM_SIZE - 1)];
                if(y == 208)
                    break;
                y = (y - m_cont_reg[23]) & 255;
                y = (y > 208) ? -(~y & 255) : y + 1;

                if((int)line < y || (int)line >= y + (int)height)
                    continue;

                if(drawn == 4) {
                    fifth_encountered = true;
                    break;
                }

                int x = vram[(attrtbl_addr + 1) & (VRAM_SIZE - 1)];
                if(vram[(attrtbl_addr + 3) & (VRAM_SIZE - 1)] & 0x80)
                    x -= 32;

                int code = vram[(attrtbl_addr + 2) & (VRAM_SIZE - 1)];
                if(m_cont_reg[1] & 2)
                    code &= 0xfc;
                const int n = (int)line - y;
                const uint32_t pattern_addr = patterntbl_addr + ((uint32_t)code << 3)
                    + ((m_cont_reg[1] & 1) ? (uint32_t)(n >> 1) : (uint32_t)n);
                const uint8_t c = vram[(attrtbl_addr + 3) & (VRAM_SIZE - 1)] & 0x0f;

                for(int half = 0; half < 2; half++) {
                    if(half == 1 && !(m_cont_reg[1] & 2))
                        break;
                    pattern = vram[(pattern_addr + (uint32_t)(half * 16)) & (VRAM_SIZE - 1)];

                    for(int i = 0; i < 8; i++, pattern <<= 1) {
                        if(pattern & 0x80) {
                            if(x >= 0 && x < 256) {
                                if(col[x] & 0x40)
                                    collided = true;
                                if(!(col[x] & 0x80)) {
                                    if(c || (m_cont_reg[8] & 0x20)) {
                                        col[x] |= 0xc0 | c;
                                        uint32_t *word = &sl->data[sl_pos_line_start + (x >> 1)];
                                        if(x & 1)
                                            *word = (*word & 0x0000ffffu) | ((v9938_palette[c] | THEN_EXTEND_1) << 16);
                                        else
                                            *word = (*word & 0xffff0000u) | (v9938_palette[c] | THEN_EXTEND_1);
                                    } else {
                                        col[x] |= 0x40;
                                    }
                                }
                                if((m_cont_reg[1] & 1) && (x + 1) < 256) {
                                    if(col[x + 1] & 0x40)
                                        collided = true;
                                    if(!(col[x + 1] & 0x80)) {
                                        if(c || (m_cont_reg[8] & 0x20)) {
                                            col[x + 1] |= 0xc0 | c;
                                            uint32_t *word = &sl->data[sl_pos_line_start + ((x + 1) >> 1)];
                                            if((x + 1) & 1)
                                                *word = (*word & 0x0000ffffu) | ((v9938_palette[c] | THEN_EXTEND_1) << 16);
                                            else
                                                *word = (*word & 0xffff0000u) | (v9938_palette[c] | THEN_EXTEND_1);
                                        } else {
                                            col[x + 1] |= 0x40;
                                        }
                                    }
                                }
                            }
                        }
                        x += (m_cont_reg[1] & 1) ? 2 : 1;
                    }
                }

                drawn++;
            }

            m_FifthSprite = (uint8_t)examined;

            if(collided) {
                m_CollisionEvents++;
                publish_event();
            }
            if(fifth_encountered && m_FrameEvents == m_FrameSeen) {
                if(m_FifthEvents == m_FifthSeen)
                    m_FifthAtEvent = (uint8_t)examined;
                m_FifthEvents++;
                publish_event();
            }
        } else if((m_mode == 0x08 || m_mode == 0x0c || m_mode == 0x10 || m_mode == 0x14 || m_mode == 0x1c)
                && !(m_cont_reg[8] & 0x02)) {
            // Sprite mode 2 (Screen 5-8's sprites): a genuinely different protocol from sprite
            // mode 1 above, not just a renumbering -- up to 8 sprites per line not 4, Y=216 is the
            // attribute table's terminator not 208, a per-sprite colour byte with its own EC/CC/IC
            // bits instead of a plain colour nibble, and a per-pixel colour table (not a single
            // colour per sprite). CC (colour-collision-suppress, bit6 of the colour byte) sprites
            // are skipped until a sprite without CC has been seen on this line -- a real hardware
            // rule, not optional. Ported from AppleII-VGA's v9938.c with the identical direct
            // status-byte-write race condition fixed the same way as sprite mode 1 (file header's
            // 1st fix) -- same event-publish mechanism, a second producer into it.
            const uint32_t attrtbl_addr_base = (((uint32_t)m_cont_reg[5] & 0xfc) << 7) + ((uint32_t)m_cont_reg[11] << 15);
            const uint32_t colourtbl_addr = attrtbl_addr_base - 512;
            const uint32_t patterntbl_addr = m_cont_reg[6] << 11;
            // ((R#5 & 3) << 3) | 7 -- ported as-is; the source this came from flags it with its own
            // "// check this!" comment, carried over here rather than silently trusted.
            const uint32_t colourmask = ((m_cont_reg[5] & 3) << 3) | 0x7;
            uint32_t sprite_height = (m_cont_reg[1] & 2) ? 16 : 8;
            if(m_cont_reg[1] & 1)
                sprite_height *= 2;

            uint32_t attrtbl_addr = attrtbl_addr_base;
            bool collided = false;
            bool fifth_encountered = false;
            bool first_cc_seen = false;
            uint32_t drawn = 0;
            uint32_t examined = 0;

            for(; examined < 32; examined++, attrtbl_addr += 4) {
                int y = vram[attrtbl_addr & (VRAM_SIZE - 1)];
                if(y == 216)
                    break;
                y = (y - m_cont_reg[23]) & 255;
                y = (y > 216) ? -(~y & 255) : y + 1;

                if((int)line < y || (int)line >= y + (int)sprite_height)
                    continue;

                if(drawn == 8) {
                    fifth_encountered = true;
                    break;
                }

                int n = (int)line - y;
                if(m_cont_reg[1] & 1)
                    n >>= 1;
                const uint8_t c = vram[(colourtbl_addr + ((examined & colourmask) << 4) + (uint32_t)n) & (VRAM_SIZE - 1)];

                if(c & 0x40) {
                    if(!first_cc_seen) {
                        drawn++;
                        continue;
                    }
                } else {
                    first_cc_seen = true;
                }

                int code = vram[(attrtbl_addr + 2) & (VRAM_SIZE - 1)];
                if(m_cont_reg[1] & 2)
                    code &= 0xfc;
                const uint32_t pattern_addr = patterntbl_addr + ((uint32_t)code << 3) + (uint32_t)n;
                pattern = ((uint32_t)vram[pattern_addr & (VRAM_SIZE - 1)] << 8) | vram[(pattern_addr + 16) & (VRAM_SIZE - 1)];

                int x = vram[(attrtbl_addr + 1) & (VRAM_SIZE - 1)];
                if(c & 0x80)
                    x -= 32;

                const int full_height = (m_cont_reg[1] & 2) ? 16 : 8;
                for(int row = 0; row < full_height; row++, pattern <<= 1) {
                    for(int zoom = 0; zoom <= (m_cont_reg[1] & 1); zoom++) {
                        if(x >= 0 && x < 256) {
                            if((pattern & 0x8000) && !(col[x] & 0x10)) {
                                if((c & 15) || (m_cont_reg[8] & 0x20)) {
                                    // GRAPHIC5 (Screen6) is genuinely 512 dots wide; this sprite
                                    // code still only places sprites in the 256-wide coordinate
                                    // space the other modes share -- the same acknowledged
                                    // limitation the ported source has (see the file header).
                                    uint32_t pixel_index;
                                    if(!(c & 0x40)) {
                                        if(col[x] & 0x20) {
                                            col[x] |= 0x10;
                                        } else {
                                            col[x] |= 0x20 | (c & 15);
                                            pixel_index = (m_mode == 0x10) ? (col[x] >> 2) & 3 : col[x] & 0x0f;
                                            uint32_t *word = &sl->data[sl_pos_line_start + (x >> 1)];
                                            if(x & 1)
                                                *word = (*word & 0x0000ffffu) | ((v9938_palette[pixel_index] | THEN_EXTEND_1) << 16);
                                            else
                                                *word = (*word & 0xffff0000u) | (v9938_palette[pixel_index] | THEN_EXTEND_1);
                                        }
                                    } else {
                                        col[x] |= c & 15;
                                        pixel_index = (m_mode == 0x10) ? (col[x] >> 2) & 3 : col[x] & 0x0f;
                                        uint32_t *word = &sl->data[sl_pos_line_start + (x >> 1)];
                                        if(x & 1)
                                            *word = (*word & 0x0000ffffu) | ((v9938_palette[pixel_index] | THEN_EXTEND_1) << 16);
                                        else
                                            *word = (*word & 0xffff0000u) | (v9938_palette[pixel_index] | THEN_EXTEND_1);
                                    }
                                    col[x] |= 0x80;
                                }
                            } else if(!(c & 0x40) && (col[x] & 0x20)) {
                                col[x] |= 0x10;
                            }

                            if(!(c & 0x60) && (pattern & 0x8000)) {
                                if(col[x] & 0x40)
                                    collided = true;
                                else
                                    col[x] |= 0x40;
                            }
                        }
                        x++;
                    }
                }

                drawn++;
            }

            m_FifthSprite = (uint8_t)examined;

            if(collided) {
                m_CollisionEvents++;
                publish_event();
            }
            if(fifth_encountered && m_FrameEvents == m_FrameSeen) {
                if(m_FifthEvents == m_FifthSeen)
                    m_FifthAtEvent = (uint8_t)examined;
                m_FifthEvents++;
                publish_event();
            }
        } else {
            m_FifthSprite = 31;
        }
    }

#ifdef FLOPPY_OVERLAY
    if(line < FDC_OVERLAY_LINES)
        fdc_overlay_line(line, &sl->data[sl_pos_line_start]);
#endif

    sl->repeat_count = 1;
    sl->length = sl_pos;
    vga_submit_scanline(sl);
}

// Only called from the render core
void render_tms9918a() {
#ifdef FLOPPY_OVERLAY
    fdc_overlay_update();
#endif
    vga_prepare_frame();
    // The top margin: tms9918a.c's 84 lines for 192-line pictures, AppleII-VGA's render_v9938() 64 for
    // 212. Each line is shown twice, so 33 (back porch) + 84 + 2*212 = 541 lines would overrun the
    // 525-line frame: the next frame's vsync wait comes too late and every other frame is black.
    const int visible_lines = (m_cont_reg[9] & 0x80) ? 212 : 192;
    vga_skip_lines(visible_lines == 212 ? 64 : 84);

    for(int line = 0; line < visible_lines; line++) {
        render_v9938_line(line);
        // Steps the command engine forward 4 times per rendered line, not once per frame -- see the
        // command-engine block's own file-level comment for why that matters. The reference expects
        // 256 steps per frame; once per line (192/212) ran slower than the machine's own V9938, so a
        // game that waits for that chip's CE started its next command before ours had finished the
        // last one, cutting it short (black bands, flicker in Konami's MSX2 logos). 4x is about 3x
        // the real chip.
        for(int i = 0; i < 4; i++)
            update_command();
    }

    tms9918a_frame_event();
}
