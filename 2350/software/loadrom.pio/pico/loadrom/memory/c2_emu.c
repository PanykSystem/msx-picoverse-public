// MSX PICOVERSE PROJECT
// (c) 2026 Cristiano Goncalves
// The Retro Hacker
//
// c2_emu.c - Carnivore2 RAM-mode emulation for SROM /D15 support.
//
// Emulates the minimum Carnivore2 surface that SROM /D15 touches:
//   - Port 0xF0 detection protocol (OUT 'C'/'S'/'R'/'H'/'0'..'3'/'A'/'M')
//   - CardMDR register and its base/enable/reg-read-disable bits
//   - Four bank configuration blocks (Mask, Addr, Reg, Mult, MaskR, AdrD)
//   - AddrFR page multiplier
//   - Bank-window RAM read/write path backed by an external PSRAM buffer
//
// Flash-mode (/F15) behaviour is intentionally out of scope: the Pico
// treats AddrM0..AddrM2 + DatM0 as plain storage, ignores DatM0
// auto-increment and does not emulate the AMD flash command set.
//
// This work is licensed under a "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
// License". https://creativecommons.org/licenses/by-nc-sa/4.0/

#include <string.h>
#include "c2_emu.h"
#include "pico.h"

// Forward decl: rebuild the decoder cache from current register state.
static void c2_rebuild_cache(c2_state_t *c2);

// -----------------------------------------------------------------------
// Initialisation — matches the hardware power-on defaults documented in
// mcscc.vhd (CardMDR=0x30, Bank1 enabled as 16KB at 0x4000..0x7FFF, rest
// disabled, AddrFR=0). The RAM buffer is cleared to 0xFF so unpainted
// bytes read back as open-bus.
// -----------------------------------------------------------------------
void c2_init(c2_state_t *c2, uint8_t *ram_ptr, uint32_t ram_size, uint8_t crslt)
{
    memset(c2, 0, sizeof(*c2));
    c2->ram_ptr = ram_ptr;
    c2->ram_size = ram_size;
    c2->crslt = crslt & 0x03u;

    // CardMDR power-on: base=0x4F80 (bits[6:5]=01), bit4=1 SCC enable.
    c2->cardmdr = 0x30u;

    // All banks disabled at power-on — SROM will program them before use.
    // (The real C2 powers up with Bank1 enabled at 0x5000-0x57FF for the
    // Konami-style bank-latch trigger, but we have no firmware running on
    // the cart so leaving banks inert avoids any interference with the
    // MSX while Nextor is booting / DOS is running.)
    for (unsigned b = 0; b < 4u; ++b)
    {
        c2->bank_regs[b][C2_BANK_MULT] = 0x00u;
    }

    c2->mconf = 0xFFu;
    c2->pf0_state = C2_PF0_IDLE;

    if (ram_ptr && ram_size)
    {
        memset(ram_ptr, 0xFFu, ram_size);
    }

    c2_rebuild_cache(c2);
}

// -----------------------------------------------------------------------
// Port 0xF0 command decoder — mirrors the VHDL `case pSltDat(7 downto 0)`
// block. Any unrecognised byte resets PF0_RV to IDLE so a stray read
// returns nothing useful (matches the hardware behaviour).
// -----------------------------------------------------------------------
void __not_in_flash_func(c2_port_write)(c2_state_t *c2, uint8_t data)
{
    switch (data)
    {
        case 'C': // 0x43 — get version (detect)
            c2->pf0_state = C2_PF0_VERSION;
            break;
        case 'S': // 0x53 — get slot
            c2->pf0_state = C2_PF0_SLOT;
            break;
        case 'R': // 0x52 — enable control registers
            c2->cardmdr &= (uint8_t)~C2_CARDMDR_REGS_DISABLE;
            break;
        case 'H': // 0x48 — disable control registers
            c2->cardmdr |= C2_CARDMDR_REGS_DISABLE;
            break;
        case '0': // 0x30 — base = 0x0F80
        case '1': // 0x31 — base = 0x4F80
        case '2': // 0x32 — base = 0x8F80
        case '3': // 0x33 — base = 0xCF80
            c2->cardmdr = (uint8_t)((c2->cardmdr & ~C2_CARDMDR_BASE_MASK)
                                    | (uint8_t)(((data - '0') & 0x03u) << C2_CARDMDR_BASE_SHIFT));
            break;
        case 'A': // 0x41 — main slot only
            c2->mconf = (uint8_t)((c2->mconf & 0x70u) | 0x01u); // clear [7], [3:0]=0001
            break;
        case 'M': // 0x4D — default subslot config
            c2->mconf = (uint8_t)((c2->mconf & 0x70u) | 0x80u | 0x0Fu);
            break;
        default:
            c2->pf0_state = C2_PF0_IDLE;
            break;
    }
}

// -----------------------------------------------------------------------
// Port 0xF0 read. Returns the value the FPGA drives in the same VHDL
// process. Reading resets the PF0 latch so the next IN returns 0xFF
// unless a new command was written first.
// -----------------------------------------------------------------------
uint8_t __not_in_flash_func(c2_port_read)(c2_state_t *c2)
{
    uint8_t value;
    switch (c2->pf0_state)
    {
        case C2_PF0_VERSION: value = (uint8_t)'2'; break;                 // 0x32
        case C2_PF0_SLOT:    value = (uint8_t)(0x30u | (c2->crslt & 0x03u)); break;
        default:             value = 0xFFu; break;
    }
    c2->pf0_state = C2_PF0_IDLE;
    return value;
}

// -----------------------------------------------------------------------
// DatM0 linear address helper — combines AddrM2:AddrM1:AddrM0 into a
// 23-bit byte offset inside the Carnivore2 RAM buffer. SROM's /D15
// upload path programs this once and then auto-increments via DatM0.
// -----------------------------------------------------------------------
static inline uint32_t c2_datm0_addr(const c2_state_t *c2)
{
    return (((uint32_t)(c2->addrm2 & 0x7Fu)) << 16)
         | (((uint32_t)c2->addrm1) << 8)
         | ((uint32_t)c2->addrm0);
}

static inline void c2_datm0_inc(c2_state_t *c2)
{
    uint32_t a = (c2_datm0_addr(c2) + 1u) & 0x7FFFFFu;
    c2->addrm0 = (uint8_t)(a & 0xFFu);
    c2->addrm1 = (uint8_t)((a >> 8) & 0xFFu);
    c2->addrm2 = (uint8_t)((a >> 16) & 0x7Fu);
}

// -----------------------------------------------------------------------
// Register window write — offset is taken from the low 6 bits of the MSX
// address. Only the registers the real firmware latches on the write
// clock are updated; everything else is stored as-is in case SROM reads
// it back for sanity checks.
// -----------------------------------------------------------------------
void __not_in_flash_func(c2_reg_write)(c2_state_t *c2, uint16_t addr, uint8_t data)
{
    uint8_t off = (uint8_t)(addr & 0x3Fu);

    // Bank configuration blocks (6 bytes each, starting at offset 0x06).
    if (off >= C2_REG_BANK_BASE && off <= 0x1Du)
    {
        uint8_t rel = (uint8_t)(off - C2_REG_BANK_BASE);
        uint8_t bank_idx = (uint8_t)(rel / C2_REG_BANK_SIZE);
        uint8_t field = (uint8_t)(rel % C2_REG_BANK_SIZE);
        c2->bank_regs[bank_idx][field] = data;
        c2_rebuild_cache(c2);
        return;
    }

    switch (off)
    {
        case C2_REG_CARDMDR:
        case C2_REG_CARDMDR2: c2->cardmdr = data; break;
        case C2_REG_ADDRM0:   c2->addrm0 = data; break;
        case C2_REG_ADDRM1:   c2->addrm1 = data; break;
        case C2_REG_ADDRM2:   c2->addrm2 = (uint8_t)(data & 0x7Fu); break;
        case C2_REG_DATM0:
        {
            // DatM0 write: stream byte into RAM at AddrM2:AddrM1:AddrM0,
            // then auto-increment the address. This is SROM /D15's bulk
            // upload path (LDI into DE=0x4F84). Writes outside the RAM
            // buffer are discarded silently.
            c2->datm0 = data;
            uint32_t a = c2_datm0_addr(c2);
            if (c2->ram_ptr && a < c2->ram_size)
            {
                c2->ram_ptr[a] = data;
                if (a + 1u > c2->max_written) c2->max_written = a + 1u;
            }
            c2_datm0_inc(c2);
            break;
        }
        case C2_REG_ADDRFR:
            c2->addrfr = (uint8_t)(data & 0x7Fu);
            c2_rebuild_cache(c2);
            break;
        case C2_REG_MCONF:    c2->mconf = data; break;
        case C2_REG_CONFFL:   c2->conffl = (uint8_t)(data & 0x07u); break;
        case C2_REG_NSREG:    c2->nsreg = data; break;
        case C2_REG_LVL:      c2->lvl = data; break;
        case C2_REG_EEBITS:   c2->eebits = data; break;
        case C2_REG_LVL1:     c2->lvl1 = data; break;
        case C2_REG_PFXN:     c2->pfxn = (uint8_t)(data & 0x03u); break;
        default:              break;
    }
}

// -----------------------------------------------------------------------
// Register window read — returns 0xFF when CardMDR[0]=1 because the FPGA
// disables the readback mux in that mode, even though writes keep decoding.
// -----------------------------------------------------------------------
uint8_t __not_in_flash_func(c2_reg_read)(const c2_state_t *c2, uint16_t addr)
{
    if ((c2->cardmdr & C2_CARDMDR_REG_RD_OFF) != 0u) return 0xFFu;

    uint8_t off = (uint8_t)(addr & 0x3Fu);

    if (off >= C2_REG_BANK_BASE && off <= 0x1Du)
    {
        uint8_t rel = (uint8_t)(off - C2_REG_BANK_BASE);
        return c2->bank_regs[rel / C2_REG_BANK_SIZE][rel % C2_REG_BANK_SIZE];
    }

    switch (off)
    {
        case C2_REG_CARDMDR:
        case C2_REG_CARDMDR2: return c2->cardmdr;
        case C2_REG_ADDRM0:   return c2->addrm0;
        case C2_REG_ADDRM1:   return c2->addrm1;
        case C2_REG_ADDRM2:   return (uint8_t)(c2->addrm2 & 0x7Fu);
        case C2_REG_DATM0:
        {
            // DatM0 read: return RAM[AddrM2:AddrM1:AddrM0] and auto-
            // increment the address latch (mirrors write-side behaviour).
            // SROM's /D15 verify path reads DatM0 after each streamed
            // byte via a CHECK helper. `c2` is declared const for the
            // public API but the address latches are stateful, so we
            // cast away const locally — callers must not assume purity.
            c2_state_t *mc2 = (c2_state_t *)c2;
            uint32_t a = c2_datm0_addr(c2);
            uint8_t val = 0xFFu;
            if (c2->ram_ptr && a < c2->ram_size)
                val = c2->ram_ptr[a];
            c2_datm0_inc(mc2);
            mc2->datm0 = val;
            return val;
        }
        case C2_REG_ADDRFR:   return (uint8_t)(c2->addrfr & 0x7Fu);
        case C2_REG_MCONF:    return c2->mconf;
        case C2_REG_CONFFL:   return (uint8_t)(c2->conffl & 0x07u);
        case C2_REG_NSREG:    return c2->nsreg;
        case C2_REG_LVL:      return c2->lvl;
        case C2_REG_EEBITS:   return c2->eebits;
        case C2_REG_LVL1:     return c2->lvl1;
        case C2_REG_PFXN:     return (uint8_t)(c2->pfxn & 0x03u);
        default:              return 0xFFu;
    }
}

// -----------------------------------------------------------------------
// Bank-window decoder (data-read / data-write window).
//
// The per-read fast path now lives inline in c2_emu.h (`c2_decode_addr`),
// reading only the precomputed `cache[]` table. This function rebuilds
// that cache whenever a register write could have changed the decode
// result (bank cfg block, AddrFR).
//
// Per mcscc.vhd "Adress Flash/ROM mapping" process:
//   - BxAdrD defines the base address of the bank's data window.
//   - The number of high-order BxAdrD bits that must match pSltAdr[15:8]
//     depends on the size code (RxMult[2:0]):
//         64K: no bits match (full address space)
//         32K: AdrD[7]     == addr[15]         — match mask 0x80
//         16K: AdrD[7:6]   == addr[15:14]      — match mask 0xC0
//          8K: AdrD[7:5]   == addr[15:13]      — match mask 0xE0
//                         (AdrD[7] only if RxMult[6]=1)
//          4K: AdrD[7:4]   == addr[15:12]      — match mask 0xF0
//   - Inside the window the linear RAM/flash offset is:
//         (RxReg & BxMaskR) * bank_size + (addr & (bank_size-1))
//                                       + AddrFR * 0x10000
//
// BxMask / BxAddr are NOT the data-window selector — they define the
// Konami-style write trigger that latches RxReg (mapper bank-switch),
// handled by c2_bank_switch_write() below. RxMult bit 7 only controls
// that page-register latch; it does NOT gate the bank window itself.
// -----------------------------------------------------------------------
static void c2_rebuild_cache(c2_state_t *c2)
{
    uint32_t addrfr_base = (uint32_t)c2->addrfr * 0x10000u;

    for (uint8_t b = 0; b < 4u; ++b)
    {
        const uint8_t *r = c2->bank_regs[b];
        uint8_t mult = r[C2_BANK_MULT];

        // --- Data window cache ---
        uint8_t  active = 0u;
        uint8_t  match_mask = 0u;
        uint8_t  match_value = 0u;
        uint32_t size_mask = 0u;
        uint32_t base_linear = 0u;
        uint8_t  is_ram = (mult & C2_MULT_RAM)      ? 1u : 0u;
        uint8_t  is_we  = (mult & C2_MULT_WRITE_EN) ? 1u : 0u;

        if ((mult & C2_MULT_DISABLE) == 0u)
        {
            uint32_t size = c2_mult_size(mult);
            if (size != 0u)
            {
                uint8_t adrd = r[C2_BANK_ADRD];
                switch (mult & C2_MULT_SIZE_MASK)
                {
                    case 0x07u: match_mask = 0x00u; break; // 64K
                    case 0x06u: match_mask = 0x80u; break; // 32K
                    case 0x05u: match_mask = 0xC0u; break; // 16K
                    case 0x04u:
                        // 8K: AdrD[6:5] always, AdrD[7] only when Mult[6]=1.
                        match_mask = (uint8_t)(0x60u | ((mult & C2_MULT_A15_MATCH) ? 0x80u : 0x00u));
                        break;
                    case 0x03u: match_mask = 0xF0u; break; // 4K
                    default:    match_mask = 0x00u; size = 0u; break;
                }
                if (size != 0u)
                {
                    active = 1u;
                    match_value = (uint8_t)(adrd & match_mask);
                    size_mask = size - 1u;
                    uint32_t reg_part = (uint32_t)(r[C2_BANK_REG] & r[C2_BANK_MASKR]);
                    base_linear = reg_part * size + addrfr_base;
                }
            }
        }

        c2->cache[b].active      = active;
        c2->cache[b].match_mask  = match_mask;
        c2->cache[b].match_value = match_value;
        c2->cache[b].size_mask   = size_mask;
        c2->cache[b].base_linear = base_linear;
        c2->cache[b].is_ram      = is_ram;
        c2->cache[b].is_we       = is_we;

        // --- Page-latch (Konami-style bank-switch trigger) cache ---
        uint8_t mask   = r[C2_BANK_MASK];
        uint8_t target = r[C2_BANK_ADDR];
        uint8_t latch_active = 0u;
        if ((mult & C2_MULT_PAGE_REG_EN) != 0u &&
            (mult & C2_MULT_DISABLE)     == 0u &&
            mask != 0u)
        {
            latch_active = 1u;
        }
        c2->cache[b].page_latch_active = latch_active;
        c2->cache[b].page_latch_mask   = mask;
        c2->cache[b].page_latch_value  = (uint8_t)(target & mask);
    }
}

// -----------------------------------------------------------------------
// Konami-style bank-switch trigger: VHDL latches RxReg when a CPU write
// hits an address whose high byte matches (BxMask/BxAddr). The mapper
// trigger is INDEPENDENT of the data-window decode and can overlap it
// (e.g. Konami SCC: data window 0x4000-0x7FFF, bank latch at 0x5000).
//
// Returns true if the write updated a bank register (the caller may
// still want to forward it to the RAM window if Mult has RAM|WRITE_EN).
// -----------------------------------------------------------------------
bool __not_in_flash_func(c2_bank_switch_write)(c2_state_t *c2, uint16_t addr, uint8_t data)
{
    uint8_t addr_high = (uint8_t)(addr >> 8);
    bool latched = false;

    for (uint8_t b = 0; b < 4u; ++b)
    {
        if (!c2->cache[b].page_latch_active) continue;
        if (((addr_high ^ c2->cache[b].page_latch_value) & c2->cache[b].page_latch_mask) != 0u)
            continue;
        c2->bank_regs[b][C2_BANK_REG] = data;
        latched = true;
    }
    if (latched) c2_rebuild_cache(c2);
    return latched;
}

// -----------------------------------------------------------------------
// AMD / Spansion / Numonyx flash autoselect emulation.
//
// c2ramldr probes the "flash" chip type by arming R1 as flash (Mult=0x95,
// RAM bit clear), writing the unlock sequence (AA/55/90) and reading
// manufacturer / device IDs. Without this, c2ramldr aborts with:
//   "FlashROM chip's type is not detected!"
//
// We emulate just enough of an M29W640GB response to pass detection:
//   Manufacturer (0x00) = 0x20   (ST / Numonyx)
//   Device C1    (0x02) = 0x7E   (M29W640 family)
//   Extended     (0x06) = 0x80   (extended block available)
//   Device C2    (0x1C) = 0x10   (M29W640G)
//   Device C3    (0x1E) = 0x00   (bottom boot sector -> "M29W640GB")
//
// Autoselect is armed when a flash-mode bank (Mult.RAM=0, Mult.WRITE_EN=1)
// sees the AA/55/90 unlock on low-12-bit offsets 0xAAA/0x555/0xAAA.
// A byte-program command (0xA0) is accepted but silently dropped: this
// firmware treats the flash window as read-only. Any other bank-window
// write (including 0xF0 reset) returns autoselect to idle.
// -----------------------------------------------------------------------
#define C2_AUTOSEL_OFF_CHECK(a_low, want)   (((a_low) & 0xFFFu) == (want))

static uint8_t c2_autosel_id_read(uint16_t addr)
{
    uint16_t off = addr & 0x001Eu;
    switch (off)
    {
        case 0x0000u: return 0x20u;
        case 0x0002u: return 0x7Eu;
        case 0x0006u: return 0x80u;
        case 0x001Cu: return 0x10u;
        case 0x001Eu: return 0x00u;
        default:      return 0xFFu;
    }
}

// Update AMD flash state machine on a write captured by a flash-mode
// bank window. Returns true if the write was absorbed by the state
// machine (caller should not route it to RAM).
bool __not_in_flash_func(c2_flash_cmd_write)(c2_state_t *c2, uint16_t addr, uint8_t data)
{
    uint16_t low = addr & 0x0FFFu;
    switch (c2->flash_state)
    {
        case 0u:
            if (low == 0xAAAu && data == 0xAAu) { c2->flash_state = 1u; return true; }
            if (data == 0xF0u)                  { c2->flash_state = 0u; return true; }
            return false;
        case 1u:
            if (low == 0x555u && data == 0x55u) { c2->flash_state = 2u; return true; }
            c2->flash_state = 0u;
            return true;
        case 2u:
            if (low == 0xAAAu && data == 0x90u) { c2->flash_state = 3u; return true; } // autoselect ON
            if (low == 0xAAAu && data == 0xA0u) { c2->flash_state = 4u; return true; } // program-byte (ignored)
            c2->flash_state = 0u;
            return true;
        case 3u:
            // Any write terminates autoselect (commonly F0 at 0x4000).
            c2->flash_state = 0u;
            return true;
        case 4u:
            // Program-byte target write: swallow, stay idle.
            c2->flash_state = 0u;
            return true;
        default:
            c2->flash_state = 0u;
            return false;
    }
}

// Return value to drive onto the bus while a flash-mode bank window is
// active. `linear` is the decoded PSRAM offset (used only when the bus
// is in normal-read mode, i.e. flash_state != 3).
uint8_t __not_in_flash_func(c2_flash_read)(const c2_state_t *c2, uint16_t addr, uint32_t linear)
{
    if (c2->flash_state == 3u)
    {
        return c2_autosel_id_read(addr);
    }
    if (c2->ram_ptr && linear < c2->ram_size) return c2->ram_ptr[linear];
    return 0xFFu;
}
