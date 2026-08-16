/*
DingusPPC - The Experimental PowerPC Macintosh emulator
Copyright (C) 2018-26 The DingusPPC Development Team
          (See CREDITS.MD for more details)

(You may also contact divingkxt or powermax2286 on Discord)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

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

/** @file Texas Instruments TSB43AB22 FireWire OHCI controller definitions. */

#ifndef TSB43AB22_H
#define TSB43AB22_H

#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>

#include <cstdint>
#include <memory>

constexpr auto TSB43AB22_DEV_ID = 0x0031; // UniNorth 2 FireWire
constexpr uint32_t TSB43AB22_BAR_SIZE = 0x1000; // 4 KiB

/** TSB43AB22 IEEE 1394 OHCI controller (internal PCI bus device 0x0E).

    A minimal register-level stub of the TI TSB43AB22 integrated
    1394a-2000 OHCI PHY/Link controller found on the Mac mini G4
    (PowerMac10,2). The 2 KB register space is mapped at BAR0
    (0xF5000000, 4 KiB).

    This stub is enough for the boot ROM's PCI enumeration and for
    the Linux firewire_ohci driver to probe without hanging. No DMA
    engine, no link-layer, and no PHY emulation are implemented. */
class TSB43AB22 : public PCIDevice {
public:
    TSB43AB22();
    ~TSB43AB22() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<TSB43AB22>(new TSB43AB22());
    }

    // MMIODevice methods
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

private:
    void notify_bar_change(int bar_num);
    void eval_irq();
    uint32_t read_w1s_w1c(uint32_t current, uint32_t set_addr_val,
                          uint32_t clear_addr_val);

    uint32_t base_addr = 0;

    // OHCI register state (32-bit, big-endian host byte order)
    uint32_t hc_control    = 0;    // 0x050/0x054 (set/clear pair)
    uint32_t self_id_buf   = 0;    // 0x064
    uint32_t config_rom_hdr = 0;   // 0x018
    uint32_t bus_options   = 0x02008060; // 0x020 (S400, gap 60, max_rec 9)
    uint32_t config_rom_map = 0;   // 0x034

    // interrupt registers (set/clear pairs)
    uint32_t int_event     = 0;    // 0x080/0x084
    uint32_t int_mask      = 0;    // 0x088/0x08C
    uint32_t isoch_xmit_int_event = 0; // 0x090/0x094
    uint32_t isoch_xmit_int_mask  = 0; // 0x098/0x09C
    uint32_t isoch_recv_int_event  = 0; // 0x0A0/0x0A4
    uint32_t isoch_recv_int_mask   = 0; // 0x0A8/0x0AC

    // link control (set/clear pair)
    uint32_t link_control  = 0;    // 0x0E0/0x0E4

    // address filter registers
    uint32_t as_req_filter_hi = 0; // 0x100/0x104
    uint32_t as_req_filter_lo = 0; // 0x108/0x10C
    uint32_t phy_req_filter_hi = 0; // 0x110/0x114
    uint32_t phy_req_filter_lo = 0; // 0x118/0x11C
    uint32_t phy_upper_bound  = 0;  // 0x120

    // async DMA context registers
    uint32_t as_req_tr_ctx[3] = {};  // 0x180/0x184/0x18C
    uint32_t as_rsp_tr_ctx[3] = {};  // 0x1A0/0x1A4/0x1AC
    uint32_t as_req_rcv_ctx[3] = {}; // 0x1C0/0x1C4/0x1CC
    uint32_t as_rsp_rcv_ctx[3] = {}; // 0x1E0/0x1E4/0x1EC

    // isoch TX contexts (up to 4, 16 bytes each)
    uint32_t it_ctx[4][3] = {};  // 0x200+n*16: Set/Clear/CmdPtr

    // isoch RX contexts (up to 4, 32 bytes each)
    uint32_t ir_ctx[4][4] = {};  // 0x400+n*32: Set/Clear/CmdPtr/Match

    // misc registers
    uint32_t at_retries     = 0;   // 0x008
    uint32_t csr_data       = 0;   // 0x00C
    uint32_t csr_compare    = 0;   // 0x010
    uint32_t csr_control    = 0;   // 0x014
    uint32_t posted_write_lo = 0;  // 0x038
    uint32_t posted_write_hi = 0;  // 0x03C
    uint32_t ir_multichan_hi = 0;  // 0x070/0x074
    uint32_t ir_multichan_lo = 0;  // 0x078/0x07C
    uint32_t fairness_ctrl  = 0;   // 0x0DC
    uint32_t phy_control    = 0;   // 0x0EC
};

#endif // TSB43AB22_H
