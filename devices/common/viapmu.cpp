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

/** High-level VIA-PMU combo device emulation.
 *
 * The PMU handshake runs over a VIA6522 "in-slice" style interface:
 * B bit 3 = TACK, B bit 4 = TREQ, shift register SR carries data.
 * Unlike the CUDA the PMU never touches the port B direction register, so
 * the raw B value written by the CPU is used directly. This follows the
 * protocol of OpenBIOS/QEMU (macio/pmu.c).
 */

#include <core/timermanager.h>
#include <cpu/ppc/ppcemu.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/viapmu.h>
#include <devices/deviceregistry.h>
#include <loguru.hpp>
#include <machines/machinebase.h>

#include <chrono>
#include <cinttypes>
#include <ctime>
#include <string>
#include <vector>

using namespace std;

/*
 * Number of data bytes to be sent with the command (or -1 if a length byte
 * should be sent) and the number of response bytes the PMU will return (or
 * -1 if it will send a length byte). Copied from OpenBIOS pmu.c (matches
 * the Apple boot ROM framing; notably 0xdf/0x9a use length bytes).
 */
static const int8_t pmu_data_len[PMU_DATA_LEN_C][2] = {
/*        0       1       2       3       4       5       6       7   */
/*00*/  {-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*08*/  {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*10*/  { 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*18*/  { 0, 1},{ 0, 1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{ 0, 0},
/*20*/  {-1, 0},{ 0, 0},{ 2, 0},{ 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},
/*28*/  { 0,-1},{ 0,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{ 0,-1},
/*30*/  { 4, 0},{20, 0},{-1, 0},{ 3, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*38*/  { 0, 4},{ 0,20},{ 2,-1},{ 2, 1},{ 3,-1},{-1,-1},{-1,-1},{ 4, 0},
/*40*/  { 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*48*/  { 0, 1},{ 0, 1},{-1,-1},{ 1, 0},{ 1, 0},{-1,-1},{-1,-1},{-1,-1},
/*50*/  { 1, 0},{ 0, 0},{ 2, 0},{ 2, 0},{-1, 0},{ 1, 0},{ 3, 0},{ 1, 0},
/*58*/  { 0, 1},{ 1, 0},{ 0, 2},{ 0, 2},{ 0,-1},{-1,-1},{-1,-1},{-1,-1},
/*60*/  { 2, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*68*/  { 0, 3},{ 0, 3},{ 0, 2},{ 0, 8},{ 0,-1},{ 0,-1},{-1,-1},{-1,-1},
/*70*/  { 1, 0},{ 1, 0},{ 1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*78*/  { 0,-1},{ 0,-1},{-1,-1},{-1,-1},{-1,-1},{ 5, 1},{ 4, 1},{ 4, 1},
/*80*/  { 4, 0},{-1, 0},{ 0, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*88*/  { 0, 5},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*90*/  { 1, 0},{ 2, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*98*/  { 0, 1},{ 0, 1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*a0*/  { 2, 0},{ 2, 0},{ 2, 0},{ 4, 0},{-1, 0},{ 0, 0},{-1, 0},{-1, 0},
/*a8*/  { 1, 1},{ 1, 0},{ 3, 0},{ 2, 0},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*b0*/  {-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*b8*/  {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*c0*/  {-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*c8*/  {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
/*d0*/  { 0, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*d8*/  { 1, 1},{ 1, 1},{-1,-1},{-1,-1},{ 0, 1},{ 0,-1},{-1,-1},{-1,-1},
/*e0*/  {-1, 0},{ 4, 0},{ 0, 1},{-1, 0},{-1, 0},{ 4, 0},{-1, 0},{-1, 0},
/*e8*/  { 3,-1},{-1,-1},{ 0, 1},{-1,-1},{ 0,-1},{-1,-1},{-1,-1},{ 0, 0},
/*f0*/  {-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},{-1, 0},
/*f8*/  {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
};

ViaPmu::ViaPmu() : HWComponent() {
    this->name = "ViaPmu";

    supports_types(HWCompType::I2C_HOST);

    // VIA reset clears all internal registers to logic 0
    this->via_portb = 0;
    this->via_porta = 0;
    this->via_ddrb  = 0;
    this->via_ddra  = 0;
    this->via_acr   = 0;
    this->via_pcr   = 0;
    this->via_sr    = 0;
    this->via_ifr   = 0;
    this->via_ier   = 0;

    // PMU starts with both handshake lines released (idle)
    this->via_portb = 0x18;

    this->cmd_state = 0;
    this->cmd       = 0;
    this->cmdlen    = 0;
    this->rsplen    = 0;
    this->cmd_buf_pos = 0;
    this->cmd_rsp_pos = 0;
    this->cmd_rsp_sz  = 0;

    this->intbits = 0;
    this->intmask = 0;
}

int ViaPmu::device_postinit()
{
    return 0;
}

uint8_t ViaPmu::read(int reg)
{
    uint8_t value;

    switch (reg & 0xF) {
    case VIA_B:
        return this->via_portb;
    case VIA_A:
    case VIA_ANH:
        return this->via_porta;
    case VIA_DIRB:
        return this->via_ddrb;
    case VIA_DIRA:
        return this->via_ddra;
    case VIA_T1CL:
        this->via_ifr &= ~VIA_IF_T1;
        return 0xFF;
    case VIA_T1CH:
        return 0xFF;
    case VIA_T1LL:
        return 0xFF;
    case VIA_T1LH:
        return 0xFF;
    case VIA_T2CL:
        this->via_ifr &= ~VIA_IF_T2;
        return 0xFF;
    case VIA_T2CH:
        return 0xFF;
    case VIA_SR:
        value = this->via_sr;
        this->via_ifr &= ~VIA_IF_SR;
        return value;
    case VIA_ACR:
        return this->via_acr;
    case VIA_PCR:
        return this->via_pcr;
    case VIA_IFR:
        return this->via_ifr;
    case VIA_IER:
        return (this->via_ier | 0x80); // bit 7 always reads as "1"
    }

    return 0; // should never happen!
}

void ViaPmu::write(int reg, uint8_t value)
{
    switch (reg & 0xF) {
    case VIA_B:
        if (this->via_portb != value) {
            this->via_portb = value;
            this->pmu_update();
        }
        break;
    case VIA_A:
    case VIA_ANH:
        this->via_porta = value;
        break;
    case VIA_DIRB:
        this->via_ddrb = value;
        break;
    case VIA_DIRA:
        this->via_ddra = value;
        break;
    case VIA_T1CL:
    case VIA_T1CH:
    case VIA_T1LL:
    case VIA_T1LH:
    case VIA_T2CL:
    case VIA_T2CH:
        break;
    case VIA_SR:
        this->via_sr = value;
        break;
    case VIA_ACR:
        this->via_acr = value;
        break;
    case VIA_PCR:
        this->via_pcr = value;
        break;
    case VIA_IFR:
        // writing a 1 to a flag clears it
        this->via_ifr &= ~value;
        break;
    case VIA_IER:
        // bit 7 = 1 enables interrupts, bit 7 = 0 disables them
        if (value & 0x80)
            this->via_ier |= (value & 0x7F);
        else
            this->via_ier &= ~(value & 0x7F);
        break;
    }
}

void ViaPmu::set_sr_int()
{
    this->via_ifr |= VIA_IF_SR;
}

void ViaPmu::pmu_update()
{
    // Handshake: B bit 4 = TREQ (driven by the CPU), bit 3 = TACK (PMU).
    // State transitions match QEMU's macio/pmu.c (and OpenBIOS).
    uint8_t b = this->via_portb;

    switch (b & 0x18) {
    case 0x10: // TREQ released: "ack release", PMU re-asserts TACK and waits
        b |= 0x08; // TACK
        this->via_portb = b;
        return;
    case 0x18: // idle: both handshake lines released
        return;
    case 0x00: // TREQ + TACK both asserted: invalid, ignore
        LOG_F(WARNING, "PMU: invalid handshake state 0x%X", b);
        return;
    case 0x08: // TREQ asserted: valid request, transfer one byte below
    default:
        break;
    }

    // Valid request: the PMU clears TACK to acknowledge the byte transfer.
    b &= ~0x08; // TACK
    this->via_portb = b;

    // transfer one byte
    switch (this->cmd_state) {
    case 0: // idle
        if (!(this->via_acr & 0x10)) { // SR_OUT
            LOG_F(WARNING, "PMU: protocol error, state idle but ACR not in output mode");
            return;
        }
        this->cmd = this->via_sr;
        this->cmdlen = pmu_data_len[this->cmd][0];
        this->rsplen = pmu_data_len[this->cmd][1];
        this->cmd_buf_pos = 0;
        this->cmd_rsp_pos = 0;
        this->cmd_rsp_sz  = 0;
        this->cmd_state = 1;
        LOG_F(9, "PMU: cmd 0x%02X cmdlen %d rsplen %d", this->cmd, this->cmdlen, this->rsplen);
        break;
    case 1: // cmd
        if (!(this->via_acr & 0x10)) {
            LOG_F(WARNING, "PMU: protocol error, state cmd but ACR not in output mode");
            return;
        }
        if (this->cmdlen == -1) {
            this->cmdlen = this->via_sr;
            LOG_F(9, "PMU: cmd 0x%02X length byte %d", this->cmd, this->cmdlen);
        } else if (this->cmd_buf_pos < PMU_CMD_BUF_SIZE) {
            this->cmd_buf[this->cmd_buf_pos++] = this->via_sr;
        }
        break;
    case 2: // rsp
        if (this->via_acr & 0x10) {
            LOG_F(WARNING, "PMU: protocol error, state rsp but ACR in output mode");
            return;
        }
        if (this->rsplen == -1) {
            this->via_sr = this->cmd_rsp_sz;
            this->rsplen = this->cmd_rsp_sz;
            LOG_F(9, "PMU: cmd 0x%02X response length byte %d", this->cmd, this->cmd_rsp_sz);
        } else if (this->cmd_rsp_pos < this->cmd_rsp_sz) {
            this->via_sr = this->cmd_rsp[this->cmd_rsp_pos++];
        }
        break;
    }

    this->set_sr_int();

    if (this->cmd_state == 1 && this->cmdlen == this->cmd_buf_pos) {
        this->dispatch_cmd();
        this->cmd_state = 2;
    }

    if (this->cmd_state == 2 && this->rsplen == this->cmd_rsp_pos) {
        this->cmd_state = 0;
    }
}

void ViaPmu::dispatch_cmd()
{
    this->cmd_rsp_pos = 0;
    this->cmd_rsp_sz  = 0;

    switch (this->cmd) {
    case 0x10: // PMU_POWER_CTRL0
        LOG_F(INFO, "PMU_POWER_CTRL0, arg = 0x%02X", this->cmd_buf[0]);
        break;
    case 0x11: // PMU_POWER_CTRL
        LOG_F(INFO, "PMU_POWER_CTRL, arg = 0x%02X", this->cmd_buf[0]);
        break;
    case 0x20: // PMU_ADB_CMD
        LOG_F(9, "PMU_ADB_CMD, no ADB devices present");
        break;
    case 0x30: // PMU_SET_RTC
        LOG_F(INFO, "PMU_SET_RTC");
        break;
    case 0x38: { // PMU_READ_RTC
        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            end - std::chrono::system_clock::from_time_t(0));
        uint32_t mac_time = uint32_t(elapsed.count()) + 2082844800ULL; // seconds since 1904-01-01
        this->cmd_rsp[0] = (mac_time >> 24) & 0xFF;
        this->cmd_rsp[1] = (mac_time >> 16) & 0xFF;
        this->cmd_rsp[2] = (mac_time >>  8) & 0xFF;
        this->cmd_rsp[3] =  mac_time        & 0xFF;
        this->cmd_rsp_sz = 4;
        break;
    }
    case 0x70: // PMU_SET_INTR_MASK
        this->intmask = this->cmd_buf[0];
        LOG_F(INFO, "PMU_SET_INTR_MASK = 0x%02X", this->intmask);
        break;
    case 0x78: // PMU_INT_ACK
        this->cmd_rsp[0] = this->intbits;
        this->intbits = 0;
        this->cmd_rsp_sz = 1;
        LOG_F(INFO, "PMU_INT_ACK -> 0x%02X", this->cmd_rsp[0]);
        break;
    case 0x7e: // PMU_SHUTDOWN
        LOG_F(INFO, "PMU_SHUTDOWN: powering off");
        power_off(po_shut_down);
        break;
    case 0x8f: // PMU_POWER_EVENTS
        LOG_F(INFO, "PMU_POWER_EVENTS, arg = 0x%02X", this->cmd_buf[0]);
        break;
    case 0x9a: { // PMU_I2C_CMD
        // args: bus, mode, bus2, address, sub_addr, comb_addr, count, data...
        uint8_t bus   = this->cmd_buf[0];
        uint8_t mode  = this->cmd_buf[1];
        uint8_t addr  = this->cmd_buf[3];
        uint8_t count = this->cmd_buf[6];
        LOG_F(INFO, "PMU_I2C_CMD bus=%d mode=%d addr=0x%02X count=%d len=%d",
              bus, mode, addr, count, this->cmdlen);
        // The boot ROM polls this command and expects the response
        // [length=1, status=0] (e.g. FFF117BC waits for "0x01 0x00").
        // I2C device emulation is not implemented, so always report success
        // with a single status byte and no data.
        this->cmd_rsp[0] = 0; // status: success
        this->cmd_rsp_sz = 1;
        break;
    }
    case 0xd0: // PMU_RESET
        LOG_F(INFO, "PMU_RESET");
        break;
    case 0xdc: // PMU_GET_COVER
        this->cmd_rsp[0] = 0; // cover open
        this->cmd_rsp_sz = 1;
        break;
    case 0xdf: // PMU_SYSTEM_READY
        LOG_F(INFO, "PMU_SYSTEM_READY");
        break;
    case 0xe2: // PMU_DOWNLOAD_STATUS
        this->cmd_rsp[0] = 0x62; // OpenPMU status: ready
        this->cmd_rsp_sz = 1;
        break;
    case 0xe8: // PMU_READ_PMU_RAM
        LOG_F(INFO, "PMU_READ_PMU_RAM");
        break;
    case 0xea: // PMU_GET_VERSION
        // PMU99: version 0x1b, revision 0x0e
        this->cmd_rsp[0] = 0x1b;
        this->cmd_rsp[1] = 0x0e;
        this->cmd_rsp_sz = 2;
        LOG_F(INFO, "PMU_GET_VERSION");
        break;
    default:
        LOG_F(WARNING, "Unsupported PMU command 0x%02X", this->cmd);
        break;
    }
}

static const DeviceDescription ViaPmu_Descriptor = {
    ViaPmu::create, {}, {}, HWCompType::I2C_HOST
};

REGISTER_DEVICE(ViaPmu, ViaPmu_Descriptor);
