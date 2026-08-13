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

/** High-level VIA-PMU combo device emulation (used on New World machines).
 */

#ifndef VIA_PMU_H
#define VIA_PMU_H

#include <core/timermanager.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/hwinterrupt.h>

#include <cinttypes>
#include <string>

#define VIA_CLOCK_HZ 7833600LL

#define VIA_B     0 // port B
#define VIA_A     1 // port A
#define VIA_DIRB  2
#define VIA_DIRA  3
#define VIA_T1CL  4 // T1 low-order latch/counter
#define VIA_T1CH  5 // T1 high-order counter
#define VIA_T1LL  6 // T1 low-order latch
#define VIA_T1LH  7 // T1 high-order latch
#define VIA_T2CL  8 // T2 low-order latch/counter
#define VIA_T2CH  9 // T2 high-order counter
#define VIA_SR    10 // shift register
#define VIA_ACR   11 // auxiliary control register
#define VIA_PCR   12 // peripheral control register
#define VIA_IFR   13 // interrupt flag register
#define VIA_IER   14 // interrupt enable register
#define VIA_ANH   15 // port A, no handshake

#define VIA_IF_T1 0x40 // bit 6
#define VIA_IF_T2 0x20 // bit 5
#define VIA_IF_SR 0x02 // bit 1
#define VIA_IF_IRQ 0x80 // bit 7

#define PMU_DATA_LEN_C 256
#define PMU_CMD_BUF_SIZE 128
#define PMU_RSP_BUF_SIZE 128

/* PMU interrupt bits (macio/pmu.h) */
#define PMU_INT_ADB        0x10 // ADB autopoll or reply data
#define PMU_INT_TICK       0x80 // 1-second tick interrupt

class ViaPmu : public HWComponent {
public:
    ViaPmu();
    ~ViaPmu() override;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<ViaPmu>(new ViaPmu());
    }

    virtual int device_postinit() override;

    uint8_t read(int reg);
    void write(int reg, uint8_t value);

private:
    void set_sr_int();
    void pmu_update();
    void dispatch_cmd();
    void pmu_update_extirq();
    void one_sec_tick();

    // VIA registers (raw, no port-direction gating of the PMU handshake)
    uint8_t via_portb = 0;
    uint8_t via_porta = 0;
    uint8_t via_ddrb  = 0;
    uint8_t via_ddra  = 0;
    uint8_t via_sr    = 0;
    uint8_t via_acr   = 0;
    uint8_t via_pcr   = 0;
    uint8_t via_ifr   = 0;
    uint8_t via_ier   = 0;


    // PMU command state machine
    uint8_t cmd_state = 0; // 0 = idle, 1 = cmd, 2 = rsp
    uint8_t cmd       = 0;
    int16_t cmdlen    = 0;
    int16_t rsplen    = 0;
    uint8_t cmd_buf_pos = 0;
    uint8_t cmd_rsp_pos = 0;
    uint8_t cmd_rsp_sz  = 0;
    uint8_t cmd_buf[PMU_CMD_BUF_SIZE] = {};
    uint8_t cmd_rsp[PMU_RSP_BUF_SIZE] = {};

    // PMU state
    uint8_t intbits = 0;
    uint8_t intmask = 0;

    // PMU interrupt line (OpenPIC via-pmu source) and 1-second tick timer
    InterruptCtrl* int_ctrl = nullptr;
    uint64_t       irq_id = 0;
    uint32_t       one_sec_timer_id = 0;
};

#endif // VIA_PMU_H
