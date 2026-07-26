// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// c2_emu.h - Carnivore2 RAM-mode emulation for SROM /D15 support.
//
// The Carnivore2 multifunction cartridge lets Nextor's SROM tool upload a
// ROM image into cartridge RAM and launch it directly, avoiding the flash
// write cycle used by /F15. This module emulates the minimum subset of
// the Carnivore2 register file, port 0xF0 detection protocol and bank-
// window RAM read/write path required for SROM /D15 to succeed.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#ifndef C2_EMU_H
#define C2_EMU_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------
// Carnivore2 register offsets (from the CardMDR base).
// The base is one of 0x0F80 / 0x4F80 / 0x8F80 / 0xCF80, selected by
// CardMDR bits [6:5] (default 0x4F80 => bits=01). The 64 byte window
// 0x4F80-0x4FBF decodes via pSltAdr[5:0].
// ---------------------------------------------------------------------
#define C2_REG_CARDMDR    0x00u
#define C2_REG_ADDRM0     0x01u
#define C2_REG_ADDRM1     0x02u
#define C2_REG_ADDRM2     0x03u
#define C2_REG_DATM0      0x04u
#define C2_REG_ADDRFR     0x05u
// Four bank configuration blocks (6 bytes each).
//   Bank1 -> 0x06..0x0B
//   Bank2 -> 0x0C..0x11
//   Bank3 -> 0x12..0x17
//   Bank4 -> 0x18..0x1D
#define C2_REG_BANK_BASE  0x06u
#define C2_REG_BANK_SIZE  0x06u
#define C2_REG_MCONF      0x1Eu
#define C2_REG_CARDMDR2   0x1Fu
#define C2_REG_CONFFL     0x20u
#define C2_REG_NSREG      0x21u
#define C2_REG_LVL        0x22u
#define C2_REG_EEBITS     0x23u
#define C2_REG_LVL1       0x24u
#define C2_REG_PFXN       0x35u

// CardMDR bits
#define C2_CARDMDR_REGS_DISABLE 0x80u   // [7]=1 disables DecMDR (master off)
#define C2_CARDMDR_BASE_MASK    0x60u   // [6:5] base select
#define C2_CARDMDR_BASE_SHIFT   5
#define C2_CARDMDR_SCC_ENABLE   0x10u   // [4]=1 SCC sound enabled
#define C2_CARDMDR_DELAY_RECONF 0x08u   // [3]=1 delayed reconfiguration
#define C2_CARDMDR_BANK_FROM_M  0x04u   // [2]=0 activate banks on start/jmp0/rst0, 1 on read 400X
#define C2_CARDMDR_SHADOW_BIOS  0x02u   // [1]=1 Shadow BIOS to RAM
#define C2_CARDMDR_REG_RD_OFF   0x01u   // [0]=1 disable register readback (writes still decoded)

// RxMult bits (per Carnivore2 mcscc.vhd)
#define C2_MULT_PAGE_REG_EN 0x80u // [7]=1 enable Konami-style page-register writes
#define C2_MULT_A15_MATCH   0x40u // [6]=1 include A15 in 8KB BxAdrD matching
#define C2_MULT_RAM         0x20u // [5]=1 RAM select (else flash/ROM)
#define C2_MULT_WRITE_EN    0x10u // [4]=1 writes to matched bank window are enabled
#define C2_MULT_DISABLE     0x08u // [3]=1 disable bank read+write window
#define C2_MULT_SIZE_MASK   0x07u // [2:0] bank size code

// Size codes -> bank size in bytes
static inline uint32_t c2_mult_size(uint8_t mult)
{
    switch (mult & C2_MULT_SIZE_MASK)
    {
        case 0x07u: return 0x10000u;  // 64 KB
        case 0x06u: return 0x08000u;  // 32 KB
        case 0x05u: return 0x04000u;  // 16 KB
        case 0x04u: return 0x02000u;  //  8 KB
        case 0x03u: return 0x01000u;  //  4 KB
        default:    return 0u;
    }
}

// Port 0xF0 detection state machine (set by the most recent 'C'/'S' write).
typedef enum {
    C2_PF0_IDLE = 0,
    C2_PF0_VERSION = 1,   // next IN returns '2' (0x32)
    C2_PF0_SLOT    = 2    // next IN returns 0x30 | CrSlt
} c2_pf0_state_t;

// Full Carnivore2 register file + runtime state.
typedef struct {
    // MDR control
    uint8_t cardmdr;      // 0x00
    uint8_t addrm0;       // 0x01
    uint8_t addrm1;       // 0x02
    uint8_t addrm2;       // 0x03 (7 bits)
    uint8_t datm0;        // 0x04 (transit; emulated as plain storage)
    uint8_t addrfr;       // 0x05 (7 bits)
    // Four bank configs in order [Mask, Addr, Reg, Mult, MaskR, AdrD]
    uint8_t bank_regs[4][6];
    uint8_t mconf;        // 0x1E
    uint8_t conffl;       // 0x20 (3 bits)
    uint8_t nsreg;        // 0x21
    uint8_t lvl;          // 0x22
    uint8_t eebits;       // 0x23
    uint8_t lvl1;         // 0x24
    uint8_t pfxn;         // 0x35 (2 bits)

    // Detection port state
    c2_pf0_state_t pf0_state;
    uint8_t crslt;        // Reported primary slot (low 2 bits)

    // Uploaded ROM buffer (memory-mapped PSRAM region)
    uint8_t *ram_ptr;     // Base of uploaded ROM storage
    uint32_t ram_size;    // Size of the storage region in bytes

    // Snapshot of highest linear byte touched via bank-window writes,
    // used by the loader to know how much of the ROM has been uploaded.
    uint32_t max_written;

    // AMD flash autoselect state machine (c2ramldr chip-ID probe):
    //   0 = idle, 1 = got AA, 2 = got AA/55, 3 = autoselect mode,
    //   4 = byte-program target pending
    uint8_t flash_state;

    // ----- Decoder cache (rebuilt on register writes) -----
    // Precomputed form of the four bank-config blocks so the hot-path
    // MSX bus loop can decide enable / data-window match / RAM offset
    // with a few compares and no multiplies.
    //
    //   active       : 1 when the bank has a valid data window
    //                  (Mult.DISABLE=0 AND size code is one of 3..7).
    //   match_mask   : high-byte address bits that must match adrd.
    //   match_value  : (adrd & match_mask).
    //   size_mask    : (size-1) — used to mask the low address bits.
    //   base_linear  : (reg & maskR)*size + addrfr*0x10000, precomputed.
    //   is_ram       : 1 when Mult.RAM=1 (RAM window), 0 for flash/ROM.
    //   is_we        : 1 when Mult.WRITE_EN=1.
    //   page_latch_*: Konami-style bank-switch trigger fields (BxMask/
    //                  BxAddr + PAGE_REG_EN gate), pre-evaluated.
    struct {
        uint8_t  active;
        uint8_t  match_mask;
        uint8_t  match_value;
        uint8_t  is_ram;
        uint8_t  is_we;
        uint8_t  page_latch_active;
        uint8_t  page_latch_mask;
        uint8_t  page_latch_value;
        uint32_t size_mask;
        uint32_t base_linear;
    } cache[4];
} c2_state_t;

#define C2_BANK_MASK  0u
#define C2_BANK_ADDR  1u
#define C2_BANK_REG   2u
#define C2_BANK_MULT  3u
#define C2_BANK_MASKR 4u
#define C2_BANK_ADRD  5u

// Initialise the register file to the hardware power-on defaults.
void c2_init(c2_state_t *c2, uint8_t *ram_ptr, uint32_t ram_size, uint8_t crslt);

// CardMDR base address (0x0F80 / 0x4F80 / 0x8F80 / 0xCF80).
static inline uint16_t c2_reg_base(const c2_state_t *c2)
{
    uint16_t sel = (uint16_t)((c2->cardmdr & C2_CARDMDR_BASE_MASK) >> C2_CARDMDR_BASE_SHIFT);
    return (uint16_t)((sel << 14) | 0x0F80u);
}

// True when MSX addr falls within the 64-byte Carnivore2 register window
// AND the master enable (CardMDR[7]) is clear.
static inline bool c2_addr_is_regwin(const c2_state_t *c2, uint16_t addr)
{
    if ((c2->cardmdr & C2_CARDMDR_REGS_DISABLE) != 0u) return false;
    uint16_t base = c2_reg_base(c2);
    return (addr >= base) && (addr < (uint16_t)(base + 0x40u));
}

// Look up which bank (0..3) covers the given MSX address and compute the
// linear offset inside the Carnivore2 RAM buffer. Returns true if the
// address is decoded by an enabled bank.
//
// Hot-path variant: reads only the precomputed `cache[]` entries, no
// multiplies, no switch/case. Rebuilt lazily by c2_reg_write /
// c2_bank_switch_write whenever the underlying registers change.
static inline bool c2_decode_addr(const c2_state_t *c2, uint16_t addr,
                                  uint8_t *bank_idx_out, uint32_t *linear_off_out)
{
    uint8_t addr_high = (uint8_t)(addr >> 8);
    for (uint8_t b = 0; b < 4u; ++b)
    {
        if (!c2->cache[b].active) continue;
        if (((addr_high ^ c2->cache[b].match_value) & c2->cache[b].match_mask) != 0u) continue;
        if (bank_idx_out)   *bank_idx_out = b;
        if (linear_off_out) *linear_off_out = c2->cache[b].base_linear + (addr & c2->cache[b].size_mask);
        return true;
    }
    return false;
}

// Query cached Mult flags for a decoded bank (avoids re-reading bank_regs).
static inline bool c2_bank_is_ram(const c2_state_t *c2, uint8_t bank_idx) { return c2->cache[bank_idx].is_ram != 0u; }
static inline bool c2_bank_is_we (const c2_state_t *c2, uint8_t bank_idx) { return c2->cache[bank_idx].is_we  != 0u; }

// Handle a port 0xF0-0xF3 write (command byte). Updates PF0 state and may
// update CardMDR / Mconf as a side effect. Port must match PFXN selection.
void c2_port_write(c2_state_t *c2, uint8_t data);

// Handle a port 0xF0-0xF3 read. Returns the response byte according to
// the current PF0 state; state is reset to idle after any read.
uint8_t c2_port_read(c2_state_t *c2);

// Handle a memory write targeted at the register window (caller must
// have checked c2_addr_is_regwin()).
void c2_reg_write(c2_state_t *c2, uint16_t addr, uint8_t data);

// Handle a memory read of the register window. Returns 0xFF when
// CardMDR[0]=1 (reg read disabled) even though writes still decode.
uint8_t c2_reg_read(const c2_state_t *c2, uint16_t addr);

// Konami-style bank-switch trigger: updates RxReg when the CPU write
// high byte matches (BxMask/BxAddr) on any enabled bank. Returns true
// if at least one bank latched the write.
bool c2_bank_switch_write(c2_state_t *c2, uint16_t addr, uint8_t data);

// AMD/Spansion flash autoselect state machine (c2ramldr chip-ID probe).
// Feed writes captured by a flash-mode bank window (Mult.RAM=0,
// Mult.WRITE_EN=1). Returns true if the write was absorbed by the
// state machine (caller must not forward it to RAM).
bool c2_flash_cmd_write(c2_state_t *c2, uint16_t addr, uint8_t data);

// Returns the byte to drive on the bus for a flash-mode bank-window
// read. When autoselect is armed, returns the M29W640G IDs; otherwise
// returns the byte stored at the decoded linear RAM offset.
uint8_t c2_flash_read(const c2_state_t *c2, uint16_t addr, uint32_t linear);

#endif // C2_EMU_H
