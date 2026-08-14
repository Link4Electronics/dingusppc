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

/** @file Intrepid (UniNorth 2) ATA/IDE controller definitions. */

#ifndef INTREPID_ATA_H
#define INTREPID_ATA_H

#include <devices/common/ata/idechannel.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/hwinterrupt.h>
#include <devices/common/pci/pcidevice.h>

#include <memory>

class DMAChannel;
class IdeChannel;

constexpr auto INTREPID_ATA6_DEV_ID = 0x003B; // UniNorth 2 ATA-100 ("ata-6")

// BAR0 register layout (relative to the 16 KiB BAR):
constexpr auto ATA_FCR_BASE      = 0x0000; // feature control register (32-bit)
constexpr auto ATA_DBDMA_BASE    = 0x1000; // DBDMA channel 0 registers
constexpr auto ATA_TASKFILE_BASE = 0x2000; // taskfile block
constexpr auto ATA_ALTSTATUS_OFF = 0x0160; // taskfile-relative alt status/ctl
constexpr auto ATA_PIO_CFG_OFF   = 0x0200; // taskfile-relative PIO timing
constexpr auto ATA_ULTRA_CFG_OFF = 0x0210; // taskfile-relative UDMA timing
constexpr auto ATA_POLL_CFG_OFF  = 0x0220; // taskfile-relative polling config
constexpr auto ATA_INT_REG_OFF   = 0x0300; // taskfile-relative interrupt image

/** UniNorth 2 "Intrepid" ATA-6 controller (internal PCI bus, device 0x0D).

    The ATA-6 cell maps into a single 16 KiB MMIO BAR:
      - 0x0000  feature control (Kauai "FCR": UATA_MAGIC|RESET_N|ENABLE)
      - 0x1000  DBDMA channel registers (cmdptr/control/status/...)
      - 0x2000  taskfile block (regs every 16 bytes, timing regs, INT_REG)
    Both the IDE channel interrupt and the DBDMA channel interrupt share the
    PCI INT# pin (OpenPIC source 39, level). */
class IntrepidAta : public PCIDevice, public InterruptCtrl {
public:
    IntrepidAta();
    ~IntrepidAta() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<IntrepidAta>(new IntrepidAta());
    }

    int device_postinit() override;

    // MMIODevice methods
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

    // InterruptCtrl methods (only used for the DBDMA channel IRQ)
    uint64_t register_dev_int(IntSrc src_id) override;
    uint64_t register_dma_int(IntSrc src_id) override;
    void ack_int(uint64_t irq_id, uint8_t irq_line_state) override;
    void ack_dma_int(uint64_t irq_id, uint8_t irq_line_state) override;

private:
    void notify_bar_change(int bar_num);
    void update_pci_irq();

    uint32_t    base_addr = 0;

    // register state
    uint32_t    fcr = 0;            // feature control
    uint32_t    timing[3] = {};     // PIO/ULTRA/POLL config (reg 0x21..0x22)
    uint32_t    int_reg = 0;        // bit30 = IDE IRQ image, bit31 = DMA IRQ

    // interrupt state
    bool        ide_irq = false;
    bool        dma_irq = false;

    IdeChannel* ide_ch = nullptr;
    std::unique_ptr<DMAChannel> ide_dma;
};

#endif // INTREPID_ATA_H
