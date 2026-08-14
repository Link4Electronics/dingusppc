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

/** @file Intrepid (UniNorth 2) ATA/IDE controller emulation.

    The UniNorth 2 "Intrepid" ATA-6 controller (internal PCI bus device
    0x0D) is exposed as a single 16 KiB MMIO BAR (0xf5004000 on the Mac
    mini G4) containing the feature control register, the DBDMA channel
    and the ATA taskfile block. The taskfile block is delegated to the
    shared IdeChannel/MacioIdeChannel class; the DBDMA channel uses the
    shared DMAChannel; both interrupt sources share the PCI INT# pin
    (OpenPIC source 39, level-triggered). */

#include <devices/common/ata/idechannel.h>
#include <devices/common/ata/intrepidata.h>
#include <devices/common/dbdma.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/hwinterrupt.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/common/pci/pcihost.h>
#include <devices/deviceregistry.h>
#include <loguru.hpp>
#include <machines/machinebase.h>

using namespace ata_interface;

IntrepidAta::IntrepidAta() : PCIDevice("intrepid-ata")
{
    this->supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV);

    // set up PCI configuration space header (ata-6@d on the internal bus)
    this->vendor_id = this->subsys_vndr = PCI_VENDOR_APPLE;
    this->device_id = this->subsys_id   = INTREPID_ATA6_DEV_ID;
    this->class_rev = 0xFF000000; // "Unassigned class" (matches real hardware)
    this->irq_pin   = 1;

    // BAR0: 16 KiB MMIO window (0xf5004000 on the Mac mini G4)
    this->bars_cfg[0] = 0xFFFFC000;
    this->finish_config_bars();

    this->pci_notify_bar_change = [this](int bar_num) {
        this->notify_bar_change(bar_num);
    };
}

int IntrepidAta::device_postinit()
{
    // The IDE channel (MacioIdeChannel "Ide0") already registered its
    // interrupt with the machine INT_CTRL; override the callback so the
    // IDE IRQ is reflected in the INT_REG image and onto our PCI INT#
    // pin instead (both IDE and DBDMA interrupts share OpenPIC source 39).
    this->ide_ch = dynamic_cast<IdeChannel*>(gMachineObj->get_comp_by_name("Ide0"));
    if (!this->ide_ch) {
        LOG_F(ERROR, "%s: IDE channel device not found!", this->name.c_str());
        return -1;
    }

    this->ide_ch->set_irq_callback([this](const uint8_t intrq_state) {
        this->ide_irq = intrq_state != 0;
        if (this->ide_irq)
            this->int_reg |= 0x40000000;
        else
            this->int_reg &= ~0x40000000;
        this->update_pci_irq();
    });

    // set up the DBDMA channel at BAR0+0x1000 and connect it to the IDE bus
    this->ide_dma = std::unique_ptr<DMAChannel>(new DMAChannel("IntrepidAta-Dma"));
    this->ide_dma->register_dma_int(this, this->register_dma_int(IntSrc::DMA_IDE0));
    this->ide_dma->connect(this->ide_ch);
    this->ide_ch->connect(this->ide_dma.get());

    return 0;
}

void IntrepidAta::notify_bar_change(int bar_num)
{
    if (bar_num) // only BAR0 is supported
        return;

    uint32_t new_base = this->bars[bar_num] & 0xFFFFC000UL;
    if (new_base != this->base_addr) {
        if (this->base_addr)
            this->host_instance->pci_unregister_mmio_region(this->base_addr,
                0x4000, this);
        this->base_addr = new_base;
        if (new_base)
            this->host_instance->pci_register_mmio_region(new_base, 0x4000, this);
        LOG_F(INFO, "%s: BAR0 set to 0x%X", this->name.c_str(), new_base);
    }
}

uint32_t IntrepidAta::read(uint32_t rgn_start, uint32_t offset, int size)
{
    // feature control register
    if (offset < 4)
        return this->fcr;

    // DBDMA channel registers (single channel, 256-byte stride)
    if (offset >= ATA_DBDMA_BASE && offset < ATA_DBDMA_BASE + 0x100)
        return this->ide_dma->reg_read(offset - ATA_DBDMA_BASE, size);

    // ATA taskfile block
    if (offset >= ATA_TASKFILE_BASE && offset < ATA_TASKFILE_BASE + 0x400) {
        uint8_t reg = (offset - ATA_TASKFILE_BASE) >> 4;

        if (reg <= ATA_Reg::COMMAND)
            return this->ide_ch->read(reg, size);
        if (reg == 0x08) // QEMU quirk: alternate status image
            return this->ide_ch->read(ATA_Reg::ALT_STATUS, size);
        if (reg == ATA_Reg::ALT_STATUS)
            return this->ide_ch->read(ATA_Reg::ALT_STATUS, size);
        if (reg == ATA_Reg::TIME_CONFIG) // PIO timing config
            return this->ide_ch->read(ATA_Reg::TIME_CONFIG, size);
        if (reg == 0x21 || reg == 0x22) // ULTRA/POLL timing config
            return this->timing[reg - 0x21];
        if (reg == ATA_Reg::INT_REG) // interrupt image register
            return this->int_reg;
    }

    return 0;
}

void IntrepidAta::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    // feature control register
    if (offset < 4) {
        this->fcr = value;
        return;
    }

    // DBDMA channel registers
    if (offset >= ATA_DBDMA_BASE && offset < ATA_DBDMA_BASE + 0x100) {
        this->ide_dma->reg_write(offset - ATA_DBDMA_BASE, value, size);
        return;
    }

    // ATA taskfile block
    if (offset >= ATA_TASKFILE_BASE && offset < ATA_TASKFILE_BASE + 0x400) {
        uint8_t reg = (offset - ATA_TASKFILE_BASE) >> 4;

        if (reg <= ATA_Reg::COMMAND) {
            this->ide_ch->write(reg, value, size);
            return;
        }
        if (reg == 0x08) // QEMU quirk: device control image
            this->ide_ch->write(ATA_Reg::DEV_CTRL, value, size);
        else if (reg == ATA_Reg::ALT_STATUS)
            this->ide_ch->write(ATA_Reg::DEV_CTRL, value, size);
        else if (reg == ATA_Reg::TIME_CONFIG)
            this->ide_ch->write(ATA_Reg::TIME_CONFIG, value, size);
        else if (reg == 0x21 || reg == 0x22) // ULTRA/POLL timing config
            this->timing[reg - 0x21] = value;
        else if (reg == ATA_Reg::INT_REG) {
            // writing bit 31 clears the latched DMA interrupt image
            if (value & 0x80000000) {
                this->int_reg &= ~0x80000000;
                this->dma_irq = false;
                this->update_pci_irq();
            }
        }
        return;
    }
}

uint64_t IntrepidAta::register_dev_int(IntSrc src_id)
{
    // device interrupts are routed via the IDE channel callback instead
    return src_id;
}

uint64_t IntrepidAta::register_dma_int(IntSrc src_id)
{
    return src_id;
}

void IntrepidAta::ack_int(uint64_t irq_id, uint8_t irq_line_state)
{
    // unused: the IDE channel IRQ is handled by the set_irq_callback
}

void IntrepidAta::ack_dma_int(uint64_t irq_id, uint8_t irq_line_state)
{
    this->dma_irq = irq_line_state != 0;
    if (this->dma_irq)
        this->int_reg |= 0x80000000;
    else
        this->int_reg &= ~0x80000000;
    this->update_pci_irq();
}

void IntrepidAta::update_pci_irq()
{
    this->pci_interrupt((this->ide_irq || this->dma_irq) ? 1 : 0);
}

static const DeviceDescription IntrepidAta_Descriptor = {
    IntrepidAta::create, {}, {},
    HWCompType::MMIO_DEV | HWCompType::PCI_DEV
};

REGISTER_DEVICE(IntrepidAta, IntrepidAta_Descriptor);
