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

/** KeyLargo I/O controller emulation (Mac mini G4 / PowerMac10,1/10,2). */

#include <core/memaccess.h>
#include <devices/deviceregistry.h>
#include <devices/ioctrl/keylargo.h>
#include <loguru.hpp>
#include <machines/machinebase.h>

#include <cinttypes>
#include <string>
#include <vector>

using namespace std;

KeyLargo::KeyLargo() : PCIDevice("KeyLargo"), InterruptCtrl() {
    supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV | HWCompType::INT_CTRL);

    // populate my PCI config header (mac-io@17 on the main UniNorth bus)
    this->vendor_id   = PCI_VENDOR_APPLE;
    this->device_id   = 0x003E;
    this->class_rev   = 0xFF000000;
    this->cache_ln_sz = 8;

    this->setup_bars({{0, uint32_t(-this->iomem_size)}});

    this->pci_notify_bar_change = [this](int bar_num) {
        this->notify_bar_change(bar_num);
    };

    // connect the VIA-PMU and the OpenPIC (both are created before KeyLargo
    // as KeyLargo subdevices)
    this->viapmu = dynamic_cast<ViaPmu*>(gMachineObj->get_comp_by_name("ViaPmu"));
    this->openpic = dynamic_cast<OpenPic*>(gMachineObj->get_comp_by_name("OpenPic"));
}

int KeyLargo::device_postinit()
{
    // The boot ROM and Open Firmware access the mac-io MMIO window at the
    // fixed device-tree address 0x80000000 without assigning BAR0 first
    // (e.g. reading via-pmu at 0x80016000 during early boot). Map it now,
    // in two chunks around the UniNorth config window at 0x80008000.
    this->base_addr = KL_BAR0_BASE;
    this->host_instance->pci_register_mmio_region(KL_BAR0_BASE,
        KL_BAR0_CHUNK1_SIZE, this);
    this->host_instance->pci_register_mmio_region(KL_BAR0_BASE + KL_BAR0_CHUNK2_OFFSET,
        KL_BAR0_CHUNK2_SIZE, this);
    LOG_F(INFO, "%s: mapped MMIO window at 0x%X (size 0x%X)", this->name.c_str(),
        KL_BAR0_BASE, this->iomem_size);

    // The boot ROM waits for the PMU's "message" interrupt line
    // (KL_GPIO_PMU_MESSAGE_IRQ, GPIO_EXTINT_0+9, bit 1) during its timebase
    // calibration; the emulated PMU has nothing to report, so hold the line
    // asserted like an idle PMU99.
    this->gpio_extint[KL_GPIO_PMU_MSG_IRQ - KL_GPIO_EXTINT_0] |= 0x02;
    return 0;
}

void KeyLargo::notify_bar_change(int bar_num) {
    if (bar_num) // only BAR0 is supported
        return;

    uint32_t new_base = this->bars[bar_num] & 0xFFFFFFF0UL;
    if (new_base != this->base_addr) {
        // The window is registered at the fixed device-tree address; the
        // boot ROM assigns BAR0 = 0x80000000, so this should never move.
        LOG_F(WARNING, "%s: BAR0 set to 0x%X, window stays at 0x%X",
            this->name.c_str(), new_base, this->base_addr);
    }
}

uint32_t KeyLargo::read(uint32_t rgn_start, uint32_t offset, int size) {
    uint32_t abs_offset = rgn_start + offset;
    unsigned sub_addr = (abs_offset >> 12) & 0x7F;

    switch (sub_addr) {
    case KL_SUB_CTRL:
        return this->ctrl_read(abs_offset & 0xFFF, size);
    case KL_SUB_VIA_PMU:
    case KL_SUB_VIA_PMU + 1: // VIA registers span 0x16000..0x17FFF
        return this->viapmu->read((abs_offset >> 9) & 0xF);
    case KL_SUB_I2C:
        return this->i2c_read(abs_offset & 0xFFF, size);
    default:
        if (sub_addr >= KL_SUB_OPENPIC && this->openpic)
            return this->openpic->read(0, abs_offset - KL_OPENPIC_BASE, size);
        if (!(this->unsupported_read_mask & (1 << sub_addr))) {
            this->unsupported_read_mask |= (1 << sub_addr);
            LOG_F(WARNING, "%s: read @%x.%c", this->get_name().c_str(),
                abs_offset, SIZE_ARG(size));
        }
        return 0;
    }
}

void KeyLargo::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) {
    uint32_t abs_offset = rgn_start + offset;
    unsigned sub_addr = (abs_offset >> 12) & 0x7F;

    switch (sub_addr) {
    case KL_SUB_CTRL:
        this->ctrl_write(abs_offset & 0xFFF, value, size);
        break;
    case KL_SUB_VIA_PMU:
    case KL_SUB_VIA_PMU + 1: // VIA registers span 0x16000..0x17FFF
        this->viapmu->write((abs_offset >> 9) & 0xF, value);
        break;
    case KL_SUB_I2C:
        this->i2c_write(abs_offset & 0xFFF, value, size);
        break;
    default:
        if (sub_addr >= KL_SUB_OPENPIC && this->openpic) {
            this->openpic->write(0, abs_offset - KL_OPENPIC_BASE, value, size);
            break;
        }
        if (!(this->unsupported_write_mask & (1 << sub_addr))) {
            this->unsupported_write_mask |= (1 << sub_addr);
            LOG_F(WARNING, "%s: write @%x.%c = %0*x", this->get_name().c_str(),
                abs_offset, SIZE_ARG(size), size * 2, value);
        }
    }
}

uint8_t KeyLargo::ctrl_read_byte(uint32_t offset)
{
    switch (offset) {
    case KL_MBCR + 0:
    case KL_MBCR + 1:
    case KL_MBCR + 2:
    case KL_MBCR + 3:
        return (this->mbcr >> (8 * (3 - (offset - KL_MBCR)))) & 0xFF;
    case KL_FCR0 + 0: case KL_FCR0 + 1: case KL_FCR0 + 2: case KL_FCR0 + 3:
    case KL_FCR1 + 0: case KL_FCR1 + 1: case KL_FCR1 + 2: case KL_FCR1 + 3:
    case KL_FCR2 + 0: case KL_FCR2 + 1: case KL_FCR2 + 2: case KL_FCR2 + 3:
    case KL_FCR3 + 0: case KL_FCR3 + 1: case KL_FCR3 + 2: case KL_FCR3 + 3:
    case KL_FCR4 + 0: case KL_FCR4 + 1: case KL_FCR4 + 2: case KL_FCR4 + 3:
    case KL_FCR5 + 0: case KL_FCR5 + 1: case KL_FCR5 + 2: case KL_FCR5 + 3: {
        int idx = (offset - KL_FCR0) >> 2;
        return (this->fcr[idx] >> (8 * (3 - (offset & 3)))) & 0xFF;
    }
    case KL_GPIO_LEVELS0:
        return this->gpio_levels0;
    case KL_GPIO_LEVELS1:
        return this->gpio_levels1;
    default:
        if (offset >= KL_GPIO_EXTINT_0 && offset < KL_GPIO_EXTINT_0 + 18)
            return this->gpio_extint[offset - KL_GPIO_EXTINT_0];
        if (offset >= KL_GPIO_0 && offset < KL_GPIO_0 + 17)
            return this->gpio[offset - KL_GPIO_0];
        return 0;
    }
}

void KeyLargo::ctrl_write_byte(uint32_t offset, uint8_t value)
{
    switch (offset) {
    case KL_MBCR + 0: case KL_MBCR + 1: case KL_MBCR + 2: case KL_MBCR + 3: {
        int shift = 8 * (3 - (offset - KL_MBCR));
        this->mbcr = (this->mbcr & ~(0xFFU << shift)) | (uint32_t(value) << shift);
        return;
    }
    case KL_FCR0 + 0: case KL_FCR0 + 1: case KL_FCR0 + 2: case KL_FCR0 + 3:
    case KL_FCR1 + 0: case KL_FCR1 + 1: case KL_FCR1 + 2: case KL_FCR1 + 3:
    case KL_FCR2 + 0: case KL_FCR2 + 1: case KL_FCR2 + 2: case KL_FCR2 + 3:
    case KL_FCR3 + 0: case KL_FCR3 + 1: case KL_FCR3 + 2: case KL_FCR3 + 3:
    case KL_FCR4 + 0: case KL_FCR4 + 1: case KL_FCR4 + 2: case KL_FCR4 + 3:
    case KL_FCR5 + 0: case KL_FCR5 + 1: case KL_FCR5 + 2: case KL_FCR5 + 3: {
        int idx = (offset - KL_FCR0) >> 2;
        int shift = 8 * (3 - (offset & 3));
        this->fcr[idx] = (this->fcr[idx] & ~(0xFFU << shift)) | (uint32_t(value) << shift);
        return;
    }
    case KL_GPIO_LEVELS0:
        this->gpio_levels0 = value;
        return;
    case KL_GPIO_LEVELS1:
        this->gpio_levels1 = value;
        return;
    default:
        if (offset >= KL_GPIO_EXTINT_0 && offset < KL_GPIO_EXTINT_0 + 18) {
            this->gpio_extint[offset - KL_GPIO_EXTINT_0] = value;
            return;
        }
        if (offset >= KL_GPIO_0 && offset < KL_GPIO_0 + 17) {
            this->gpio[offset - KL_GPIO_0] = value;
            return;
        }
        LOG_F(9, "%s: ctrl write @%x = %02x", this->get_name().c_str(), offset, value);
    }
}

uint32_t KeyLargo::ctrl_read(uint32_t offset, int size)
{
    uint32_t value = 0;
    for (int i = 0; i < size; i++)
        value = (value << 8) | this->ctrl_read_byte(offset + i);
    return value;
}

void KeyLargo::ctrl_write(uint32_t offset, uint32_t value, int size)
{
    for (int i = 0; i < size; i++)
        this->ctrl_write_byte(offset + i, (value >> (8 * (size - 1 - i))) & 0xFF);
}

uint32_t KeyLargo::i2c_read(uint32_t offset, int size)
{
    uint32_t value = 0;

    if (offset == KL_I2C_STATUS) {
        // No devices on the emulated I2C bus; report the transfer as
        // complete (status bits 0, 2 and 3 set) so the boot ROM's probe
        // routine doesn't spin, and reads return 0xFF (bus pulled high).
        value = 0xFF;
    } else if (offset == KL_I2C_MODE) {
        value = this->i2c_mode;
    } else if (offset == KL_I2C_ADDR) {
        value = this->i2c_addr;
    }

    if (size == 1)
        return value & 0xFF;
    return value;
}

void KeyLargo::i2c_write(uint32_t offset, uint32_t value, int size)
{
    uint8_t byte = value & 0xFF;

    switch (offset) {
    case KL_I2C_CTRL:
        this->i2c_ctrl = byte;
        break;
    case KL_I2C_MODE:
        this->i2c_mode = byte;
        break;
    case KL_I2C_STATUS:
        this->i2c_data = byte;
        break;
    case KL_I2C_ADDR:
        this->i2c_addr = byte;
        break;
    default:
        LOG_F(9, "%s: i2c write @%x = %02x", this->get_name().c_str(), offset, byte);
    }
}

uint64_t KeyLargo::register_dev_int(IntSrc src_id)
{
    // KeyLargo's interrupt controller is the OpenPIC at mac-io+0x40000.
    return this->openpic->register_dev_int(src_id);
}

uint64_t KeyLargo::register_dma_int(IntSrc src_id)
{
    return this->openpic->register_dma_int(src_id);
}

void KeyLargo::ack_int(uint64_t irq_id, uint8_t irq_line_state)
{
    this->openpic->ack_int(irq_id, irq_line_state);
}

void KeyLargo::ack_dma_int(uint64_t irq_id, uint8_t irq_line_state)
{
    this->openpic->ack_dma_int(irq_id, irq_line_state);
}

static const vector<string> KeyLargo_Subdevices = {
    "ViaPmu",
    "OpenPic"
};

static const DeviceDescription KeyLargo_Descriptor = {
    KeyLargo::create, KeyLargo_Subdevices, {},
    HWCompType::MMIO_DEV | HWCompType::PCI_DEV | HWCompType::INT_CTRL
};

REGISTER_DEVICE(KeyLargo, KeyLargo_Descriptor);
