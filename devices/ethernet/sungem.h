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

/** @file UniNorth 2 GMAC (Sun GEM) ethernet controller definitions. */

#ifndef SUN_GEM_H
#define SUN_GEM_H

#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>

#include <cstdint>
#include <memory>

/** UniNorth 2 GMAC (Sun GEM), internal PCI bus device 0x0F (OpenPIC 41).

    A register-level emulation of the Sun GEM (as found in the UniNorth 2
    "Intrepid" memory controller), modeled after QEMU's hw/net/sungem.c.
    There is no host network backend: transmitted frames are read from the
    descriptor ring and dropped; received frames never arrive. The DMA
    engines and MAC/MIF/PCS register files behave enough like the real
    chip that the Linux sungem driver (and Open Firmware) can probe,
    configure and bring the interface up without hanging.

    All GEM registers are little-endian 32-bit words; the dingusppc MMIO
    framework uses big-endian byte-lane semantics, so every access is
    byteswapped at the BAR boundary.

    BAR0 is a 2 MiB window (0xf5200000 on the Mac mini G4) containing:
      0x0000  global registers (GREG)
      0x2000  TX DMA block
      0x3000  WOL block (reads return 0xFFFFFFFF)
      0x4000  RX DMA block
      0x6000  MAC block
      0x6200  MIF (MDIO) block
      0x9000  PCS block */
class SunGEM : public PCIDevice {
public:
    SunGEM();
    ~SunGEM() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<SunGEM>(new SunGEM());
    }

    int device_postinit() override;

    // MMIODevice methods
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

private:
    void notify_bar_change(int bar_num);
    void reset_all(bool pci_reset);
    void reset_tx();
    void reset_rx();
    void update_masks();
    void eval_irq();
    void update_status(uint32_t bits, bool val);
    void eval_cascade_irq();
    void tx_kick();
    uint16_t mii_read(uint8_t phy_addr, uint8_t reg_addr);
    void mii_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t val);
    uint32_t mii_op(uint32_t val);
    uint64_t dma_read_qword(uint32_t addr);

    uint32_t    base_addr = 0;

    // register files (values stored in little-endian byte order)
    uint32_t    gregs[0x2000 >> 2] = {};   // global registers
    uint32_t    txdmaregs[0x1000 >> 2] = {}; // TX DMA block
    uint32_t    rxdmaregs[0x2000 >> 2] = {}; // RX DMA block
    uint32_t    macregs[0x0200 >> 2] = {}; // MAC block
    uint32_t    mifregs[0x0020 >> 2] = {}; // MIF block
    uint32_t    pcsregs[0x0060 >> 2] = {}; // PCS block

    // cached ring sizes
    uint32_t    rx_mask = 0;
    uint32_t    tx_mask = 0;

    // current TX frame being assembled (never sent)
    uint32_t    tx_size = 0;
    uint64_t    tx_first_ctl = 0;

    uint8_t     mac_addr[6] = { 0x00, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E };
};

#endif // SUN_GEM_H
