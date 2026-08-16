/*
DingusPPC - The Experimental PowerPC Macintosh emulator
Copyright (C) 2018-26 The DingusPPC Development Team
          (See CREDITS.MD for more details)

(You may also contact divingkxt or powermax2286 on Discord)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// The AltiVec (VMX) opcodes for the processor - ppcaltivecopcodes.cpp

#include "ppcemu.h"
#include "ppcmacros.h"
#include "ppcmmu.h"
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cfenv>
#include <limits>

using namespace dppc_interpreter;

namespace dppc_interpreter {

// Vector register access. vpr[r] is a big-endian byte array (byte 0 = MSB).

static inline uint16_t v_u16(const uint8_t* p)
{
    return (uint16_t(p[0]) << 8) | p[1];
}

static inline uint32_t v_u32(const uint8_t* p)
{
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}

static inline void v_w_u16(uint8_t* p, uint16_t v)
{
    p[0] = uint8_t(v >> 8);
    p[1] = uint8_t(v);
}

static inline void v_w_u32(uint8_t* p, uint32_t v)
{
    p[0] = uint8_t(v >> 24);
    p[1] = uint8_t(v >> 16);
    p[2] = uint8_t(v >> 8);
    p[3] = uint8_t(v);
}

static inline float v_f32(const uint8_t* p)
{
    uint32_t bits = v_u32(p);
    float f;
    std::memcpy(&f, &bits, sizeof f);
    return f;
}

static inline void v_w_f32(uint8_t* p, float f)
{
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof bits);
    v_w_u32(p, bits);
}

// Saturating conversions (mirror QEMU int_helper.c).

static inline int8_t  cvtshsb(int16_t x) { return x < INT8_MIN ? INT8_MIN : x > INT8_MAX ? INT8_MAX : (int8_t)x; }
static inline int16_t cvtswsh(int32_t x) { return x < INT16_MIN ? INT16_MIN : x > INT16_MAX ? INT16_MAX : (int16_t)x; }
static inline int32_t cvtsdsw(int64_t x) { return x < INT32_MIN ? INT32_MIN : x > INT32_MAX ? INT32_MAX : (int32_t)x; }
static inline uint8_t  cvtshub(int16_t x) { return x < 0 ? 0 : x > UINT8_MAX ? UINT8_MAX : (uint8_t)x; }
static inline uint16_t cvtswuh(int32_t x) { return x < 0 ? 0 : x > UINT16_MAX ? UINT16_MAX : (uint16_t)x; }
static inline uint32_t cvtsduw(int64_t x) { return x < 0 ? 0 : x > UINT32_MAX ? UINT32_MAX : (uint32_t)x; }
static inline uint8_t  cvtuhub(uint16_t x) { return x > UINT8_MAX ? UINT8_MAX : (uint8_t)x; }
static inline uint16_t cvtuwuh(uint32_t x) { return x > UINT16_MAX ? UINT16_MAX : (uint16_t)x; }
static inline uint32_t cvtuduw(uint64_t x) { return x > UINT32_MAX ? UINT32_MAX : (uint32_t)x; }

static inline void vscr_set_sat()
{
    ppc_state.vscr |= VSCR_bit::VSCR_SAT;
}

static inline void vcmp_set_cr6(bool all_true, bool all_false)
{
    ppc_state.cr = (ppc_state.cr & ~0xF0) | (all_true ? 0x80 : 0) | (all_false ? 0x20 : 0);
}

static inline float f32_round(double x)
{
    int old = std::fegetround();
    std::fesetround(FE_TONEAREST);
    float r = (float)x;
    std::fesetround(old);
    return r;
}

static inline float fp_max(float a, float b)
{
    if (std::isnan(a)) {
        return std::isnan(b) ? std::numeric_limits<float>::quiet_NaN() : b;
    }
    if (std::isnan(b)) {
        return a;
    }
    return a > b ? a : b;
}

static inline float fp_min(float a, float b)
{
    if (std::isnan(a)) {
        return std::isnan(b) ? std::numeric_limits<float>::quiet_NaN() : b;
    }
    if (std::isnan(b)) {
        return a;
    }
    return a < b ? a : b;
}

// Element-wise helpers. VRT = bits 21-25, VRA = bits 16-20, VRB = bits 11-15.

template <typename F>
static void vec_map8(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 16; i++) {
        r[i] = f(a[i], b[i]);
    }
}

template <typename F>
static void vec_map16(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        v_w_u16(r + 2 * i, f(v_u16(a + 2 * i), v_u16(b + 2 * i)));
    }
}

template <typename F>
static void vec_map32(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        v_w_u32(r + 4 * i, f(v_u32(a + 4 * i), v_u32(b + 4 * i)));
    }
}

template <typename F>
static void vec_map8_f32(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        v_w_f32(r + 4 * i, f(v_f32(a + 4 * i), v_f32(b + 4 * i)));
    }
}

template <typename F>
static void vec_un8(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 16; i++) {
        r[i] = f(b[i]);
    }
}

template <typename F>
static void vec_un16(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        v_w_u16(r + 2 * i, f(v_u16(b + 2 * i)));
    }
}

template <typename F>
static void vec_un32(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        v_w_u32(r + 4 * i, f(v_u32(b + 4 * i)));
    }
}

template <typename F>
static void vec_un32_f32(uint32_t opcode, F f)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        v_w_f32(r + 4 * i, f(v_f32(b + 4 * i)));
    }
}

static inline void v_addsub_sat8(uint32_t opcode, int16_t (*op)(int16_t, int16_t),
                                 int8_t (*cvt)(int16_t))
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 16; i++) {
        int16_t t = op((int16_t)a[i], (int16_t)b[i]);
        int8_t res = cvt(t);
        if (res != (int8_t)t) {
            sat = true;
        }
        r[i] = (uint8_t)res;
    }
    if (sat) {
        vscr_set_sat();
    }
}

template <typename T, typename Wide>
static void v_addsub_sat_t(uint32_t opcode, Wide (*op)(Wide, Wide), T (*cvt)(Wide), int n)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < n; i++) {
        Wide av, bv, t;
        T res;
        if (sizeof(T) == 2) {
            av = (Wide)v_u16(a + 2 * i);
            bv = (Wide)v_u16(b + 2 * i);
        } else {
            av = (Wide)v_u32(a + 4 * i);
            bv = (Wide)v_u32(b + 4 * i);
        }
        t = op(av, bv);
        res = cvt(t);
        if ((Wide)res != t) {
            sat = true;
        }
        if (sizeof(T) == 2) {
            v_w_u16(r + 2 * i, (uint16_t)res);
        } else {
            v_w_u32(r + 4 * i, (uint32_t)res);
        }
    }
    if (sat) {
        vscr_set_sat();
    }
}

// ---------------------------------------------------------------------------
// Integer arithmetic and logical instructions.
// ---------------------------------------------------------------------------

static void v_addubm(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a + b); }); }
static void v_adduhm(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return uint16_t(a + b); }); }
static void v_adduwm(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return a + b; }); }
static void v_sububm(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a - b); }); }
static void v_subuhm(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return uint16_t(a - b); }); }
static void v_subuwm(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return a - b; }); }

static void v_addcuw(uint32_t opcode)
{
    vec_map32(opcode, [](uint32_t a, uint32_t b) { return (~a < b) ? 0xFFFFFFFFu : 0; });
}

static void v_subcuw(uint32_t opcode)
{
    vec_map32(opcode, [](uint32_t a, uint32_t b) { return (a >= b) ? 0xFFFFFFFFu : 0; });
}

static void v_addsbs(uint32_t opcode) { v_addsub_sat8(opcode, [](int16_t a, int16_t b) { return int16_t(a + b); }, cvtshsb); }
static void v_subsbs(uint32_t opcode) { v_addsub_sat8(opcode, [](int16_t a, int16_t b) { return int16_t(a - b); }, cvtshsb); }

static void v_addubs(uint32_t opcode)
{
    vec_map8(opcode, [](uint8_t a, uint8_t b) { return cvtuhub(uint16_t(a) + b); });
}

static void v_sububs(uint32_t opcode)
{
    vec_map8(opcode, [](uint8_t a, uint8_t b) { return cvtuhub(uint16_t(a - b)); });
}

static void v_addshs(uint32_t opcode)
{
    v_addsub_sat_t<int16_t, int32_t>(opcode, [](int32_t a, int32_t b) { return a + b; }, cvtswsh, 8);
}

static void v_subshs(uint32_t opcode)
{
    v_addsub_sat_t<int16_t, int32_t>(opcode, [](int32_t a, int32_t b) { return a - b; }, cvtswsh, 8);
}

static void v_adduhs(uint32_t opcode)
{
    v_addsub_sat_t<uint16_t, uint32_t>(opcode, [](uint32_t a, uint32_t b) { return a + b; }, cvtuwuh, 8);
}

static void v_subuhs(uint32_t opcode)
{
    v_addsub_sat_t<uint16_t, uint32_t>(opcode, [](uint32_t a, uint32_t b) { return a - b; }, cvtuwuh, 8);
}

static void v_addsws(uint32_t opcode)
{
    v_addsub_sat_t<int32_t, int64_t>(opcode, [](int64_t a, int64_t b) { return a + b; }, cvtsdsw, 4);
}

static void v_subsws(uint32_t opcode)
{
    v_addsub_sat_t<int32_t, int64_t>(opcode, [](int64_t a, int64_t b) { return a - b; }, cvtsdsw, 4);
}

static void v_adduws(uint32_t opcode)
{
    v_addsub_sat_t<uint32_t, uint64_t>(opcode, [](uint64_t a, uint64_t b) { return a + b; }, cvtuduw, 4);
}

static void v_subuws(uint32_t opcode)
{
    v_addsub_sat_t<uint32_t, uint64_t>(opcode, [](uint64_t a, uint64_t b) { return a - b; }, cvtuduw, 4);
}

static void v_maxub(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return a > b ? a : b; }); }
static void v_maxuh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return a > b ? a : b; }); }
static void v_maxuw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return a > b ? a : b; }); }
static void v_maxsb(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return (int8_t)a > (int8_t)b ? a : b; }); }
static void v_maxsh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return (int16_t)a > (int16_t)b ? a : b; }); }
static void v_maxsw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return (int32_t)a > (int32_t)b ? a : b; }); }
static void v_minub(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return a < b ? a : b; }); }
static void v_minuh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return a < b ? a : b; }); }
static void v_minuw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return a < b ? a : b; }); }
static void v_minsb(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return (int8_t)a < (int8_t)b ? a : b; }); }
static void v_minsh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return (int16_t)a < (int16_t)b ? a : b; }); }
static void v_minsw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return (int32_t)a < (int32_t)b ? a : b; }); }

static void v_avgub(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t((uint16_t(a) + b + 1) >> 1); }); }
static void v_avguh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return uint16_t((uint32_t(a) + b + 1) >> 1); }); }
static void v_avguw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return uint32_t((uint64_t(a) + b + 1) >> 1); }); }
static void v_avgsb(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t((int16_t((int8_t)a) + (int8_t)b + 1) >> 1); }); }
static void v_avgsh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return uint16_t((int32_t((int16_t)a) + (int16_t)b + 1) >> 1); }); }
static void v_avgsw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return uint32_t((int64_t((int32_t)a) + (int32_t)b + 1) >> 1); }); }

static void v_and(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a & b); }); }
static void v_andc(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a & ~b); }); }
static void v_or(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a | b); }); }
static void v_xor(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a ^ b); }); }
static void v_nor(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(~(a | b)); }); }

// ---------------------------------------------------------------------------
// Shift instructions.
// ---------------------------------------------------------------------------

static void v_slb(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a << (b & 7)); }); }
static void v_slh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return uint16_t(a << (b & 15)); }); }
static void v_slw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return a << (b & 31); }); }
static void v_srb(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t(a >> (b & 7)); }); }
static void v_srh(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return uint16_t(a >> (b & 15)); }); }
static void v_srw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return a >> (b & 31); }); }
static void v_srab(uint32_t opcode) { vec_map8(opcode, [](uint8_t a, uint8_t b) { return uint8_t((int8_t)a >> (b & 7)); }); }
static void v_srah(uint32_t opcode) { vec_map16(opcode, [](uint16_t a, uint16_t b) { return uint16_t((int16_t)a >> (b & 15)); }); }
static void v_sraw(uint32_t opcode) { vec_map32(opcode, [](uint32_t a, uint32_t b) { return uint32_t((int32_t)a >> (b & 31)); }); }

static void v_sl(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int n = b[15] & 0x07;
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) {
        acc = (acc << 8) | a[i];
        r[i] = uint8_t((acc << n) >> 24);
    }
}

static void v_sr(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int n = b[15] & 0x07;
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) {
        acc = (acc << 8) | a[i];
        r[15 - i] = uint8_t((acc >> n) & 0xFF);
    }
}

static void v_slo(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int sh = (b[15] >> 3) & 0x0F;
    std::memmove(r, a + sh, 16 - sh);
    std::memset(r + 16 - sh, 0, sh);
}

static void v_sro(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int sh = (b[15] >> 3) & 0x0F;
    std::memmove(r + sh, a, 16 - sh);
    std::memset(r, 0, sh);
}

// ---------------------------------------------------------------------------
// Splat, merge, permute and select instructions.
// ---------------------------------------------------------------------------

static void v_spltb(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    uint8_t v = b[(opcode >> 16) & 0x1F];
    std::memset(r, v, 16);
}

static void v_splth(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    uint16_t v = v_u16(b + 2 * ((opcode >> 16) & 0x07));
    for (int i = 0; i < 8; i++) {
        v_w_u16(r + 2 * i, v);
    }
}

static void v_spltw(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    uint32_t v = v_u32(b + 4 * ((opcode >> 16) & 0x03));
    for (int i = 0; i < 4; i++) {
        v_w_u32(r + 4 * i, v);
    }
}

static void v_spltisb(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    int32_t s = (opcode >> 16) & 0x1F;
    if (s & 0x10) {
        s -= 32;
    }
    std::memset(r, (uint8_t)s, 16);
}

static void v_spltish(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    int32_t s = (opcode >> 16) & 0x1F;
    if (s & 0x10) {
        s -= 32;
    }
    v_w_u16(r + 0, (uint16_t)s);
    v_w_u16(r + 2, (uint16_t)s);
    v_w_u16(r + 4, (uint16_t)s);
    v_w_u16(r + 6, (uint16_t)s);
    v_w_u16(r + 8, (uint16_t)s);
    v_w_u16(r + 10, (uint16_t)s);
    v_w_u16(r + 12, (uint16_t)s);
    v_w_u16(r + 14, (uint16_t)s);
}

static void v_spltisw(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    int32_t s = (opcode >> 16) & 0x1F;
    if (s & 0x10) {
        s -= 32;
    }
    for (int i = 0; i < 4; i++) {
        v_w_u32(r + 4 * i, (uint32_t)s);
    }
}

template <int elem_size>
static void v_mrgh(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int n = 16 / elem_size;
    int half = n / 2;
    for (int i = 0; i < half; i++) {
        std::memcpy(r + 2 * i * elem_size, a + i * elem_size, elem_size);
        std::memcpy(r + (2 * i + 1) * elem_size, b + i * elem_size, elem_size);
    }
}

template <int elem_size>
static void v_mrgl(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int n = 16 / elem_size;
    int half = n / 2;
    for (int i = 0; i < half; i++) {
        std::memcpy(r + 2 * i * elem_size, a + (i + half) * elem_size, elem_size);
        std::memcpy(r + (2 * i + 1) * elem_size, b + (i + half) * elem_size, elem_size);
    }
}

static void v_sel(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 16; i++) {
        r[i] = (c[i] & 0x80) ? b[i] : a[i];
    }
}

static void v_perm(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 16; i++) {
        int s = c[i] & 0x1F;
        r[i] = (s & 0x10) ? b[s & 0x0F] : a[s & 0x0F];
    }
}

// ---------------------------------------------------------------------------
// Pack and unpack instructions.
// ---------------------------------------------------------------------------

static void v_pkuhum(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        r[i] = a[2 * i + 1];
        r[8 + i] = b[2 * i + 1];
    }
}

static void v_pkuwum(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        r[2 * i] = a[4 * i + 2];
        r[2 * i + 1] = a[4 * i + 3];
        r[8 + 2 * i] = b[4 * i + 2];
        r[8 + 2 * i + 1] = b[4 * i + 3];
    }
}

static void v_pkuhus(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        uint16_t av = v_u16(a + 2 * i), bv = v_u16(b + 2 * i);
        uint8_t a8 = cvtuhub(av), b8 = cvtuhub(bv);
        if (a8 != av) {
            sat = true;
        }
        if (b8 != bv) {
            sat = true;
        }
        r[i] = a8;
        r[8 + i] = b8;
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_pkuwus(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        uint32_t av = v_u32(a + 4 * i), bv = v_u32(b + 4 * i);
        uint16_t a16 = cvtuwuh(av), b16 = cvtuwuh(bv);
        if (a16 != av) {
            sat = true;
        }
        if (b16 != bv) {
            sat = true;
        }
        v_w_u16(r + 2 * i, a16);
        v_w_u16(r + 8 + 2 * i, b16);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_pkshus(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        int16_t av = (int16_t)v_u16(a + 2 * i), bv = (int16_t)v_u16(b + 2 * i);
        uint8_t a8 = cvtshub(av), b8 = cvtshub(bv);
        if ((int16_t)a8 != av) {
            sat = true;
        }
        if ((int16_t)b8 != bv) {
            sat = true;
        }
        r[i] = a8;
        r[8 + i] = b8;
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_pkswus(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        int32_t av = (int32_t)v_u32(a + 4 * i), bv = (int32_t)v_u32(b + 4 * i);
        uint16_t a16 = cvtswuh(av), b16 = cvtswuh(bv);
        if ((int32_t)a16 != av) {
            sat = true;
        }
        if ((int32_t)b16 != bv) {
            sat = true;
        }
        v_w_u16(r + 2 * i, a16);
        v_w_u16(r + 8 + 2 * i, b16);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_pkshss(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        int16_t av = (int16_t)v_u16(a + 2 * i), bv = (int16_t)v_u16(b + 2 * i);
        int8_t a8 = cvtshsb(av), b8 = cvtshsb(bv);
        if (a8 != (int8_t)av) {
            sat = true;
        }
        if (b8 != (int8_t)bv) {
            sat = true;
        }
        r[i] = (uint8_t)a8;
        r[8 + i] = (uint8_t)b8;
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_pkswss(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        int32_t av = (int32_t)v_u32(a + 4 * i), bv = (int32_t)v_u32(b + 4 * i);
        int16_t a16 = cvtswsh(av), b16 = cvtswsh(bv);
        if (a16 != (int16_t)av) {
            sat = true;
        }
        if (b16 != (int16_t)bv) {
            sat = true;
        }
        v_w_u16(r + 2 * i, (uint16_t)a16);
        v_w_u16(r + 8 + 2 * i, (uint16_t)b16);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_pkpx(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 2; i++) {
        uint32_t e = v_u32(a + 4 * i);
        v_w_u16(r + 2 * i, (uint16_t)(((e >> 9) & 0xFC00) | ((e >> 6) & 0x3E0) | ((e >> 3) & 0x1F)));
        e = v_u32(b + 4 * i);
        v_w_u16(r + 8 + 2 * i, (uint16_t)(((e >> 9) & 0xFC00) | ((e >> 6) & 0x3E0) | ((e >> 3) & 0x1F)));
    }
}

static void v_upkhsb(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        v_w_u16(r + 2 * i, (uint16_t)(int16_t)(int8_t)b[i]);
    }
}

static void v_upkhsh(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        v_w_u32(r + 4 * i, (uint32_t)(int32_t)(int16_t)v_u16(b + 2 * i));
    }
}

static void v_upklsb(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 8; i++) {
        v_w_u16(r + 2 * i, (uint16_t)(int16_t)(int8_t)b[8 + i]);
    }
}

static void v_upklsh(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        v_w_u32(r + 4 * i, (uint32_t)(int32_t)(int16_t)v_u16(b + 8 + 2 * i));
    }
}

static void v_upkhpx(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        uint16_t e = v_u16(b + 2 * i);
        uint32_t a8 = (e >> 15) ? 0xFF : 0;
        uint32_t r5 = (e >> 10) & 0x1F;
        uint32_t g5 = (e >> 5) & 0x1F;
        uint32_t b5 = e & 0x1F;
        v_w_u32(r + 4 * i, (a8 << 24) | (r5 << 16) | (g5 << 8) | b5);
    }
}

static void v_upklpx(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        uint16_t e = v_u16(b + 8 + 2 * i);
        uint32_t a8 = (e >> 15) ? 0xFF : 0;
        uint32_t r5 = (e >> 10) & 0x1F;
        uint32_t g5 = (e >> 5) & 0x1F;
        uint32_t b5 = e & 0x1F;
        v_w_u32(r + 4 * i, (a8 << 24) | (r5 << 16) | (g5 << 8) | b5);
    }
}

// ---------------------------------------------------------------------------
// Vector sum instructions.
// ---------------------------------------------------------------------------

static void v_sumsws(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int64_t t = (int32_t)v_u32(b + 12);
    for (int i = 0; i < 4; i++) {
        t += (int32_t)v_u32(a + 4 * i);
    }
    int32_t res = cvtsdsw(t);
    if (res != (int32_t)t) {
        sat = true;
    }
    v_w_u32(r, 0);
    v_w_u32(r + 4, 0);
    v_w_u32(r + 8, 0);
    v_w_u32(r + 12, (uint32_t)res);
    if (sat) {
        vscr_set_sat();
    }
}

static void v_sum2sws(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 2; i++) {
        int64_t t = (int32_t)v_u32(b + 4 + 8 * i);
        for (int j = 0; j < 2; j++) {
            t += (int32_t)v_u32(a + (2 * i + j) * 4);
        }
        int32_t res = cvtsdsw(t);
        if (res != (int32_t)t) {
            sat = true;
        }
        v_w_u32(r + 8 * i, 0);
        v_w_u32(r + 8 * i + 4, (uint32_t)res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_sum4sbs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        int64_t t = (int32_t)v_u32(b + 4 * i);
        for (int j = 0; j < 4; j++) {
            t += (int8_t)a[4 * i + j];
        }
        int32_t res = cvtsdsw(t);
        if (res != (int32_t)t) {
            sat = true;
        }
        v_w_u32(r + 4 * i, (uint32_t)res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_sum4shs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        int64_t t = (int32_t)v_u32(b + 4 * i);
        t += (int16_t)v_u16(a + 4 * i) + (int16_t)v_u16(a + 4 * i + 2);
        int32_t res = cvtsdsw(t);
        if (res != (int32_t)t) {
            sat = true;
        }
        v_w_u32(r + 4 * i, (uint32_t)res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_sum4ubs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        uint64_t t = v_u32(b + 4 * i);
        for (int j = 0; j < 4; j++) {
            t += a[4 * i + j];
        }
        uint32_t res = cvtuduw(t);
        if (res != (uint32_t)t) {
            sat = true;
        }
        v_w_u32(r + 4 * i, res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

// ---------------------------------------------------------------------------
// Multiply-sum instructions.
// ---------------------------------------------------------------------------

static void v_mhaddshs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 8; i++) {
        int32_t prod = (int16_t)v_u16(a + 2 * i) * (int16_t)v_u16(b + 2 * i);
        int32_t t = (int16_t)v_u16(c + 2 * i) + (prod >> 15);
        int16_t res = cvtswsh(t);
        if (res != (int16_t)t) {
            sat = true;
        }
        v_w_u16(r + 2 * i, (uint16_t)res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_mhraddshs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 8; i++) {
        int32_t prod = (int16_t)v_u16(a + 2 * i) * (int16_t)v_u16(b + 2 * i) + 0x4000;
        int32_t t = (int16_t)v_u16(c + 2 * i) + (prod >> 15);
        int16_t res = cvtswsh(t);
        if (res != (int16_t)t) {
            sat = true;
        }
        v_w_u16(r + 2 * i, (uint16_t)res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_mladduhm(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 8; i++) {
        int32_t prod = (int16_t)v_u16(a + 2 * i) * (int16_t)v_u16(b + 2 * i);
        v_w_u16(r + 2 * i, (uint16_t)(int16_t)(prod + (int16_t)v_u16(c + 2 * i)));
    }
}

static void v_msumbm(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 4; i++) {
        int32_t t = (int32_t)v_u32(c + 4 * i);
        for (int j = 0; j < 4; j++) {
            t += (int32_t)((int8_t)a[4 * i + j]) * b[4 * i + j];
        }
        v_w_u32(r + 4 * i, (uint32_t)t);
    }
}

static void v_msumshm(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 4; i++) {
        int32_t t = (int32_t)v_u32(c + 4 * i);
        t += (int16_t)v_u16(a + 4 * i) * (int16_t)v_u16(b + 4 * i);
        t += (int16_t)v_u16(a + 4 * i + 2) * (int16_t)v_u16(b + 4 * i + 2);
        v_w_u32(r + 4 * i, (uint32_t)t);
    }
}

static void v_msumshs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 4; i++) {
        int64_t t = (int32_t)v_u32(c + 4 * i);
        t += (int64_t)(int16_t)v_u16(a + 4 * i) * (int16_t)v_u16(b + 4 * i);
        t += (int64_t)(int16_t)v_u16(a + 4 * i + 2) * (int16_t)v_u16(b + 4 * i + 2);
        int32_t res = cvtsdsw(t);
        if (res != (int32_t)t) {
            sat = true;
        }
        v_w_u32(r + 4 * i, (uint32_t)res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_msumubm(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 4; i++) {
        uint32_t t = v_u32(c + 4 * i);
        for (int j = 0; j < 4; j++) {
            t += (uint16_t)a[4 * i + j] * b[4 * i + j];
        }
        v_w_u32(r + 4 * i, t);
    }
}

static void v_msumuhm(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 4; i++) {
        uint32_t t = v_u32(c + 4 * i);
        t += v_u16(a + 4 * i) * v_u16(b + 4 * i);
        t += v_u16(a + 4 * i + 2) * v_u16(b + 4 * i + 2);
        v_w_u32(r + 4 * i, t);
    }
}

static void v_msumuhs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 4; i++) {
        uint64_t t = v_u32(c + 4 * i);
        t += (uint64_t)v_u16(a + 4 * i) * v_u16(b + 4 * i);
        t += (uint64_t)v_u16(a + 4 * i + 2) * v_u16(b + 4 * i + 2);
        uint32_t res = cvtuduw(t);
        if (res != (uint32_t)t) {
            sat = true;
        }
        v_w_u32(r + 4 * i, res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

// ---------------------------------------------------------------------------
// Compare instructions. Vector compares are VX form: opc2 (bits 1-5) = 3, the
// compare type is in bits 6-10, and bit 10 + 16 selects the "dot" form (which
// records all-true/all-false in CR6). VRT = bits 21-25.
// ---------------------------------------------------------------------------

template <typename F>
static void v_cmp_elems(uint32_t opcode, int elem_size, bool set_cr6, F f)
{
    bool all_true = true, all_false = true;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    int n = 16 / elem_size;
    for (int i = 0; i < n; i++) {
        bool t = f(i);
        if (t) {
            all_false = false;
        } else {
            all_true = false;
        }
        std::memset(r + i * elem_size, t ? 0xFF : 0x00, elem_size);
    }
    if (set_cr6) {
        vcmp_set_cr6(all_true, all_false);
    }
}

static void v_cmpequb(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 1, rc, [a, b](int i) { return a[i] == b[i]; });
}

static void v_cmpequh(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 2, rc, [a, b](int i) { return v_u16(a + 2 * i) == v_u16(b + 2 * i); });
}

static void v_cmpequw(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 4, rc, [a, b](int i) { return v_u32(a + 4 * i) == v_u32(b + 4 * i); });
}

static void v_cmpgtsb(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 1, rc, [a, b](int i) { return (int8_t)a[i] > (int8_t)b[i]; });
}

static void v_cmpgtsh(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 2, rc, [a, b](int i) { return (int16_t)v_u16(a + 2 * i) > (int16_t)v_u16(b + 2 * i); });
}

static void v_cmpgtsw(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 4, rc, [a, b](int i) { return (int32_t)v_u32(a + 4 * i) > (int32_t)v_u32(b + 4 * i); });
}

static void v_cmpgtub(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 1, rc, [a, b](int i) { return a[i] > b[i]; });
}

static void v_cmpgtuh(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 2, rc, [a, b](int i) { return v_u16(a + 2 * i) > v_u16(b + 2 * i); });
}

static void v_cmpgtuw(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 4, rc, [a, b](int i) { return v_u32(a + 4 * i) > v_u32(b + 4 * i); });
}

static void v_cmpeqfp(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 4, rc, [a, b](int i) {
        float af = v_f32(a + 4 * i), bf = v_f32(b + 4 * i);
        return !std::isnan(af) && !std::isnan(bf) && af == bf;
    });
}

static void v_cmpgefp(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 4, rc, [a, b](int i) {
        float af = v_f32(a + 4 * i), bf = v_f32(b + 4 * i);
        return !std::isnan(af) && !std::isnan(bf) && af >= bf;
    });
}

static void v_cmpgtfp(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    v_cmp_elems(opcode, 4, rc, [a, b](int i) {
        float af = v_f32(a + 4 * i), bf = v_f32(b + 4 * i);
        return !std::isnan(af) && !std::isnan(bf) && af > bf;
    });
}

static void v_cmpbfp(uint32_t opcode)
{
    bool rc = (opcode >> 10) & 1;
    bool all_in = true;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    for (int i = 0; i < 4; i++) {
        float af = v_f32(a + 4 * i), bf = v_f32(b + 4 * i);
        bool in;
        if (std::isnan(af) || std::isnan(bf)) {
            in = false;
        } else {
            in = (af <= bf) && (af >= -bf);
        }
        if (!in) {
            all_in = false;
        }
        v_w_u32(r + 4 * i, in ? 0 : 0xC0000000u);
    }
    if (rc) {
        vcmp_set_cr6(false, all_in);
    }
}

// ---------------------------------------------------------------------------
// Floating-point instructions.
// ---------------------------------------------------------------------------

static void v_addfp(uint32_t opcode) { vec_map8_f32(opcode, [](float a, float b) { return f32_round(double(a) + double(b)); }); }
static void v_subfp(uint32_t opcode) { vec_map8_f32(opcode, [](float a, float b) { return f32_round(double(a) - double(b)); }); }
static void v_maxfp(uint32_t opcode) { vec_map8_f32(opcode, fp_max); }
static void v_minfp(uint32_t opcode) { vec_map8_f32(opcode, fp_min); }

static void v_refp(uint32_t opcode) { vec_un32_f32(opcode, [](float b) { return f32_round(1.0 / double(b)); }); }
static void v_rsqrtefp(uint32_t opcode) { vec_un32_f32(opcode, [](float b) { return f32_round(1.0 / std::sqrt(double(b))); }); }
static void v_logefp(uint32_t opcode) { vec_un32_f32(opcode, [](float b) { return f32_round(std::log2(double(b))); }); }
static void v_exptefp(uint32_t opcode) { vec_un32_f32(opcode, [](float b) { return f32_round(std::exp2(double(b))); }); }

static void v_rfip(uint32_t opcode) { vec_un32_f32(opcode, [](float b) { return std::ceil(b); }); }
static void v_rfim(uint32_t opcode) { vec_un32_f32(opcode, [](float b) { return std::floor(b); }); }
static void v_rfin(uint32_t opcode)
{
    vec_un32_f32(opcode, [](float b) {
        float fl = std::floor(b);
        float frac = b - fl;
        if (frac > 0.5f) {
            return fl + 1.0f;
        }
        if (frac == 0.5f && std::fmod(fl, 2.0f) != 0.0f) {
            return fl + 1.0f;
        }
        return fl;
    });
}
static void v_rfiz(uint32_t opcode) { vec_un32_f32(opcode, [](float b) { return std::trunc(b); }); }

static void v_cfux(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int uim = (opcode >> 16) & 0x1F;
    for (int i = 0; i < 4; i++) {
        v_w_f32(r + 4 * i, f32_round(std::ldexp(double(v_u32(b + 4 * i)), -uim)));
    }
}

static void v_cfsx(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int uim = (opcode >> 16) & 0x1F;
    for (int i = 0; i < 4; i++) {
        v_w_f32(r + 4 * i, f32_round(std::ldexp(double((int32_t)v_u32(b + 4 * i)), -uim)));
    }
}

static void v_ctuxs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int uim = (opcode >> 16) & 0x1F;
    for (int i = 0; i < 4; i++) {
        float f = v_f32(b + 4 * i);
        uint32_t res;
        if (std::isnan(f)) {
            res = 0;
        } else {
            double t = std::ldexp(double(f), uim);
            if (t >= 4294967296.0) {
                res = 0xFFFFFFFFu;
                sat = true;
            } else if (t < 0.0) {
                res = 0;
                sat = true;
            } else {
                res = (uint32_t)t;
            }
        }
        v_w_u32(r + 4 * i, res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

static void v_ctsxs(uint32_t opcode)
{
    bool sat = false;
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int uim = (opcode >> 16) & 0x1F;
    for (int i = 0; i < 4; i++) {
        float f = v_f32(b + 4 * i);
        uint32_t res;
        if (std::isnan(f)) {
            res = 0;
        } else {
            double t = std::ldexp(double(f), uim);
            if (t >= 2147483648.0) {
                res = 0x7FFFFFFFu;
                sat = true;
            } else if (t < -2147483648.0) {
                res = 0x80000000u;
                sat = true;
            } else {
                res = (uint32_t)(int32_t)t;
            }
        }
        v_w_u32(r + 4 * i, res);
    }
    if (sat) {
        vscr_set_sat();
    }
}

// ---------------------------------------------------------------------------
// VSCR instructions.
// ---------------------------------------------------------------------------

static void v_mfvscr(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    v_w_u32(r, 0);
    v_w_u32(r + 4, 0);
    v_w_u32(r + 8, 0);
    v_w_u32(r + 12, ppc_state.vscr);
}

static void v_mtvscr(uint32_t opcode)
{
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    ppc_state.vscr = v_u32(b + 12);
}

static void v_sldoi(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    int sh = ((opcode >> 1) & 0x3FF) >> 5;
    uint8_t tmp[16];
    for (int i = 0; i < 16; i++) {
        int index = sh + i;
        tmp[i] = (index > 0x0F) ? b[index - 0x10] : a[index];
    }
    std::memcpy(r, tmp, 16);
}

static void v_maddfp(uint32_t opcode)
{
    uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
    const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
    const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
    const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
    for (int i = 0; i < 4; i++) {
        float af = v_f32(a + 4 * i), bf = v_f32(b + 4 * i), cf = v_f32(c + 4 * i);
        v_w_f32(r + 4 * i, f32_round(double(af) * cf + double(bf)));
    }
}

// ---------------------------------------------------------------------------
// Dispatcher.
// ---------------------------------------------------------------------------

void ppc_altivec_opcode(uint32_t opcode)
{
    if (!is_altivec) {
        ppc_illegalop(opcode);
        return;
    }
    if (!(ppc_state.msr & MSR::VEC)) {
        ppc_exception_handler(Except_Type::EXC_NO_ALTIVEC, Exc_Cause::FPU_OFF);
        return;
    }

    if (opcode & 1) {
        switch (opcode & 0x3F) {
        case 32: v_mhaddshs(opcode); return;
        case 33: v_mhraddshs(opcode); return;
        case 34: v_mladduhm(opcode); return;
        case 36: v_msumubm(opcode); return;
        case 37: v_msumbm(opcode); return;
        case 38: v_msumuhm(opcode); return;
        case 39: v_msumuhs(opcode); return;
        case 40: v_msumshm(opcode); return;
        case 41: v_msumshs(opcode); return;
        case 43: v_perm(opcode); return;
        case 47:
        {
            uint8_t* r = ppc_state.vpr[(opcode >> 21) & 0x1F];
            const uint8_t* a = ppc_state.vpr[(opcode >> 16) & 0x1F];
            const uint8_t* b = ppc_state.vpr[(opcode >> 11) & 0x1F];
            const uint8_t* c = ppc_state.vpr[(opcode >> 6) & 0x1F];
            for (int i = 0; i < 4; i++) {
                float af = v_f32(a + 4 * i), bf = v_f32(b + 4 * i), cf = v_f32(c + 4 * i);
                v_w_f32(r + 4 * i, f32_round(bf - double(af) * cf));
            }
            return;
        }
        default: ppc_illegalop(opcode); return;
        }
    }

    switch ((opcode >> 1) & 0x3FF) {
    // opc2 = 0
    case 0: v_addubm(opcode); return;
    case 32: v_adduhm(opcode); return;
    case 64: v_adduwm(opcode); return;
    case 192: v_addcuw(opcode); return;
    case 256: v_addubs(opcode); return;
    case 288: v_adduhs(opcode); return;
    case 320: v_adduws(opcode); return;
    case 384: v_addsbs(opcode); return;
    case 416: v_addshs(opcode); return;
    case 448: v_addsws(opcode); return;
    case 512: v_sububm(opcode); return;
    case 544: v_subuhm(opcode); return;
    case 576: v_subuwm(opcode); return;
    case 704: v_subcuw(opcode); return;
    case 768: v_sububs(opcode); return;
    case 800: v_subuhs(opcode); return;
    case 832: v_subuws(opcode); return;
    case 896: v_subsbs(opcode); return;
    case 928: v_subshs(opcode); return;
    case 960: v_subsws(opcode); return;
    // opc2 = 1
    case 1: v_maxub(opcode); return;
    case 33: v_maxuh(opcode); return;
    case 65: v_maxuw(opcode); return;
    case 129: v_maxsb(opcode); return;
    case 161: v_maxsh(opcode); return;
    case 193: v_maxsw(opcode); return;
    case 257: v_minub(opcode); return;
    case 289: v_minuh(opcode); return;
    case 321: v_minuw(opcode); return;
    case 385: v_minsb(opcode); return;
    case 417: v_minsh(opcode); return;
    case 449: v_minsw(opcode); return;
    case 513: v_avgub(opcode); return;
    case 545: v_avguh(opcode); return;
    case 577: v_avguw(opcode); return;
    case 641: v_avgsb(opcode); return;
    case 673: v_avgsh(opcode); return;
    case 705: v_avgsw(opcode); return;
    // opc2 = 2
    case 130: v_slb(opcode); return;
    case 162: v_slh(opcode); return;
    case 194: v_slw(opcode); return;
    case 226: v_sl(opcode); return;
    case 258: v_srb(opcode); return;
    case 290: v_srh(opcode); return;
    case 322: v_srw(opcode); return;
    case 354: v_sr(opcode); return;
    case 386: v_srab(opcode); return;
    case 418: v_srah(opcode); return;
    case 450: v_sraw(opcode); return;
    // opc2 = 3 (compares, bit 21 = Rc)
    case 3: v_cmpequb(opcode); return;
    case 35: v_cmpequh(opcode); return;
    case 67: v_cmpequw(opcode); return;
    case 99: v_cmpeqfp(opcode); return;
    case 227: v_cmpgefp(opcode); return;
    case 259: v_cmpgtub(opcode); return;
    case 291: v_cmpgtuh(opcode); return;
    case 323: v_cmpgtuw(opcode); return;
    case 355: v_cmpgtfp(opcode); return;
    case 387: v_cmpgtsb(opcode); return;
    case 419: v_cmpgtsh(opcode); return;
    case 451: v_cmpgtsw(opcode); return;
    case 483: v_cmpbfp(opcode); return;
    case 515: v_cmpequb(opcode); return;
    case 547: v_cmpequh(opcode); return;
    case 579: v_cmpequw(opcode); return;
    case 611: v_cmpeqfp(opcode); return;
    case 739: v_cmpgefp(opcode); return;
    case 771: v_cmpgtub(opcode); return;
    case 803: v_cmpgtuh(opcode); return;
    case 835: v_cmpgtuw(opcode); return;
    case 867: v_cmpgtfp(opcode); return;
    case 899: v_cmpgtsb(opcode); return;
    case 931: v_cmpgtsh(opcode); return;
    case 963: v_cmpgtsw(opcode); return;
    case 995: v_cmpbfp(opcode); return;
    // opc2 = 5
    case 0 + 5: v_addfp(opcode); return;
    case 32 + 5: v_subfp(opcode); return;
    case 128 + 5: v_refp(opcode); return;
    case 160 + 5: v_rsqrtefp(opcode); return;
    case 192 + 5: v_exptefp(opcode); return;
    case 224 + 5: v_logefp(opcode); return;
    case 256 + 5: v_rfin(opcode); return;
    case 288 + 5: v_rfiz(opcode); return;
    case 320 + 5: v_rfip(opcode); return;
    case 352 + 5: v_rfim(opcode); return;
    case 384 + 5: v_cfux(opcode); return;
    case 416 + 5: v_cfsx(opcode); return;
    case 448 + 5: v_ctuxs(opcode); return;
    case 480 + 5: v_ctsxs(opcode); return;
    case 512 + 5: v_maxfp(opcode); return;
    case 544 + 5: v_minfp(opcode); return;
    // opc2 = 6
    case 0 + 6: v_mrgh<1>(opcode); return;
    case 32 + 6: v_mrgh<2>(opcode); return;
    case 64 + 6: v_mrgh<4>(opcode); return;
    case 128 + 6: v_mrgl<1>(opcode); return;
    case 160 + 6: v_mrgl<2>(opcode); return;
    case 192 + 6: v_mrgl<4>(opcode); return;
    case 256 + 6: v_spltb(opcode); return;
    case 288 + 6: v_splth(opcode); return;
    case 320 + 6: v_spltw(opcode); return;
    case 384 + 6: v_spltisb(opcode); return;
    case 416 + 6: v_spltish(opcode); return;
    case 448 + 6: v_spltisw(opcode); return;
    case 512 + 6: v_slo(opcode); return;
    case 544 + 6: v_sro(opcode); return;
    // opc2 = 7 (pack/unpack)
    case 0 + 7: v_pkuhum(opcode); return;
    case 32 + 7: v_pkuwum(opcode); return;
    case 64 + 7: v_pkuhus(opcode); return;
    case 96 + 7: v_pkuwus(opcode); return;
    case 128 + 7: v_pkshus(opcode); return;
    case 160 + 7: v_pkswus(opcode); return;
    case 192 + 7: v_pkshss(opcode); return;
    case 224 + 7: v_pkswss(opcode); return;
    case 384 + 7: v_pkpx(opcode); return;
    case 256 + 7: v_upkhsb(opcode); return;
    case 288 + 7: v_upkhsh(opcode); return;
    case 320 + 7: v_upklsb(opcode); return;
    case 352 + 7: v_upklsh(opcode); return;
    case 416 + 7: v_upkhpx(opcode); return;
    case 480 + 7: v_upklpx(opcode); return;
    // opc2 = 4 (vector sums)
    case 768 + 4: v_sum4ubs(opcode); return;
    case 800 + 4: v_sum4shs(opcode); return;
    case 832 + 4: v_sum2sws(opcode); return;
    case 896 + 4: v_sum4sbs(opcode); return;
    case 960 + 4: v_sumsws(opcode); return;
    // opc2 = 22 (vsldoi, sh = XO bits 6-10)
    case 22: v_sldoi(opcode); return;
    case 54: v_sldoi(opcode); return;
    case 86: v_sldoi(opcode); return;
    case 118: v_sldoi(opcode); return;
    case 150: v_sldoi(opcode); return;
    case 182: v_sldoi(opcode); return;
    case 214: v_sldoi(opcode); return;
    case 246: v_sldoi(opcode); return;
    case 278: v_sldoi(opcode); return;
    case 310: v_sldoi(opcode); return;
    case 342: v_sldoi(opcode); return;
    case 374: v_sldoi(opcode); return;
    case 406: v_sldoi(opcode); return;
    case 438: v_sldoi(opcode); return;
    case 470: v_sldoi(opcode); return;
    case 502: v_sldoi(opcode); return;
    // opc2 = 23 (vmaddfp)
    case 119: v_maddfp(opcode); return;
    // opc2 = 21 (vsel)
    case 149: v_sel(opcode); return;
    // mfvscr / mtvscr (VX-style, no operands beyond VRT/VRB)
    case 770: v_mfvscr(opcode); return;
    case 802: v_mtvscr(opcode); return;
    default: ppc_illegalop(opcode); return;
    }
}

// ---------------------------------------------------------------------------
// Vector loads and stores (opcode 31).
// ---------------------------------------------------------------------------

static void vec_avail_check()
{
    if (!is_altivec) {
        ppc_illegalop(0);
        return;
    }
    if (!(ppc_state.msr & MSR::VEC)) {
        ppc_exception_handler(Except_Type::EXC_NO_ALTIVEC, Exc_Cause::FPU_OFF);
    }
}

void ppc_lvx(uint32_t opcode)
{
    vec_avail_check();
    if (ppc_next_instruction_address == 0x0F20) {
        return;
    }
    uint32_t reg_d = (opcode >> 21) & 0x1F;
    uint32_t reg_a = (opcode >> 16) & 0x1F;
    uint32_t reg_b = (opcode >> 11) & 0x1F;
    uint32_t ea = (reg_a ? ppc_state.gpr[reg_a] : 0) + ppc_state.gpr[reg_b];
    ea &= ~0x0F;
    uint8_t* r = ppc_state.vpr[reg_d];
    for (int i = 0; i < 4; i++) {
        uint32_t w = mmu_read_vmem<uint32_t>(opcode, ea + 4 * i);
        v_w_u32(r + 4 * i, w);
    }
}

void ppc_lvxl(uint32_t opcode)
{
    ppc_lvx(opcode);
}

void ppc_stvx(uint32_t opcode)
{
    vec_avail_check();
    if (ppc_next_instruction_address == 0x0F20) {
        return;
    }
    uint32_t reg_s = (opcode >> 21) & 0x1F;
    uint32_t reg_a = (opcode >> 16) & 0x1F;
    uint32_t reg_b = (opcode >> 11) & 0x1F;
    uint32_t ea = (reg_a ? ppc_state.gpr[reg_a] : 0) + ppc_state.gpr[reg_b];
    ea &= ~0x0F;
    const uint8_t* s = ppc_state.vpr[reg_s];
    for (int i = 0; i < 4; i++) {
        mmu_write_vmem<uint32_t>(opcode, ea + 4 * i, v_u32(s + 4 * i));
    }
}

void ppc_stvxl(uint32_t opcode)
{
    ppc_stvx(opcode);
}

template <int elem_size>
static void ppc_lve_common(uint32_t opcode)
{
    vec_avail_check();
    if (ppc_next_instruction_address == 0x0F20) {
        return;
    }
    uint32_t reg_d = (opcode >> 21) & 0x1F;
    uint32_t reg_a = (opcode >> 16) & 0x1F;
    uint32_t reg_b = (opcode >> 11) & 0x1F;
    uint32_t ea = (reg_a ? ppc_state.gpr[reg_a] : 0) + ppc_state.gpr[reg_b];
    int index = (ea & 0x0F) / elem_size;
    uint8_t* r = ppc_state.vpr[reg_d];
    if (elem_size == 1) {
        r[index] = mmu_read_vmem<uint8_t>(opcode, ea);
    } else if (elem_size == 2) {
        v_w_u16(r + 2 * index, mmu_read_vmem<uint16_t>(opcode, ea));
    } else {
        v_w_u32(r + 4 * index, mmu_read_vmem<uint32_t>(opcode, ea));
    }
}

void ppc_lvebx(uint32_t opcode) { ppc_lve_common<1>(opcode); }
void ppc_lvehx(uint32_t opcode) { ppc_lve_common<2>(opcode); }
void ppc_lvewx(uint32_t opcode) { ppc_lve_common<4>(opcode); }

template <int elem_size>
static void ppc_stve_common(uint32_t opcode)
{
    vec_avail_check();
    if (ppc_next_instruction_address == 0x0F20) {
        return;
    }
    uint32_t reg_s = (opcode >> 21) & 0x1F;
    uint32_t reg_a = (opcode >> 16) & 0x1F;
    uint32_t reg_b = (opcode >> 11) & 0x1F;
    uint32_t ea = (reg_a ? ppc_state.gpr[reg_a] : 0) + ppc_state.gpr[reg_b];
    int index = (ea & 0x0F) / elem_size;
    const uint8_t* s = ppc_state.vpr[reg_s];
    if (elem_size == 1) {
        mmu_write_vmem<uint8_t>(opcode, ea, s[index]);
    } else if (elem_size == 2) {
        mmu_write_vmem<uint16_t>(opcode, ea, v_u16(s + 2 * index));
    } else {
        mmu_write_vmem<uint32_t>(opcode, ea, v_u32(s + 4 * index));
    }
}

void ppc_stvebx(uint32_t opcode) { ppc_stve_common<1>(opcode); }
void ppc_stvehx(uint32_t opcode) { ppc_stve_common<2>(opcode); }
void ppc_stvewx(uint32_t opcode) { ppc_stve_common<4>(opcode); }

}    // namespace dppc_interpreter
