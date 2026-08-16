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
#include <cpu/ppc/ppcemu.h>
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

    // Create audio DMA channels and wire them to the SoundServer.
    // Channel 8 (audio out) pulls PCM samples from guest RAM and feeds
    // them to the host audio device via the I2S TX path.
    this->snd_out_dma = std::unique_ptr<DMAChannel>(new DMAChannel("snd_out"));
    this->snd_out_dma->register_dma_int(this,
        this->register_dma_int(IntSrc::DMA_DAVBUS_Tx));
    this->snd_out_dma->set_callbacks(
        std::bind(&KeyLargo::i2s_dma_out_start, this),
        std::bind(&KeyLargo::i2s_dma_out_stop, this)
    );

    this->snd_in_dma = std::unique_ptr<DMAChannel>(new DMAChannel("snd_in"));
    this->snd_in_dma->register_dma_int(this,
        this->register_dma_int(IntSrc::DMA_DAVBUS_Rx));

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
    case KL_SUB_TIMER:
        return this->timer_read(abs_offset & 0xFFF, size);
    case KL_SUB_ESCC:
    case KL_SUB_ESCC_LEG:
        if (size == 1)
            return this->escc_read_byte(abs_offset & 0xFFF);
        return 0;
    case KL_SUB_I2S:
        return this->i2s_read(abs_offset & 0xFFF, size);
    case KL_SUB_DBDMA:
        return this->dbdma_read(abs_offset & 0xFFF, size);
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
    case KL_SUB_TIMER:
        this->timer_write(abs_offset & 0xFFF, value, size);
        break;
    case KL_SUB_ESCC:
    case KL_SUB_ESCC_LEG:
        for (int i = 0; i < size; i++)
            this->escc_write_byte((abs_offset & 0xFFF) + i,
                (value >> (8 * (size - 1 - i))) & 0xFF);
        break;
    case KL_SUB_I2S:
        this->i2s_write(abs_offset & 0xFFF, value, size);
        break;
    case KL_SUB_DBDMA:
        this->dbdma_write(abs_offset & 0xFFF, value, size);
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

uint32_t KeyLargo::timer_read(uint32_t offset, int size)
{
    switch (offset) {
    case 0x38: // counter low word (little-endian in memory; read via lwbrx)
        return BYTESWAP_32((uint32_t)(this->timer_count() & 0xFFFFFFFF));
    case 0x3C: // counter high word
        return BYTESWAP_32((uint32_t)(this->timer_count() >> 32));
    default:
        if (!(this->unsupported_read_mask & (1 << KL_SUB_TIMER))) {
            this->unsupported_read_mask |= (1 << KL_SUB_TIMER);
            LOG_F(WARNING, "%s: timer read @%x.%c", this->get_name().c_str(),
                KL_BAR0_BASE + 0x15000 + offset, SIZE_ARG(size));
        }
        return 0;
    }
}

void KeyLargo::timer_write(uint32_t offset, uint32_t value, int size)
{
    if (offset == 0x38 || offset == 0x3C) {
        // The boot ROM writes 0 to both words to reset the counter before a
        // measurement window.
        this->timer_reset_ns = get_virt_time_ns();
        return;
    }
    if (!(this->unsupported_write_mask & (1 << KL_SUB_TIMER))) {
        this->unsupported_write_mask |= (1 << KL_SUB_TIMER);
        LOG_F(WARNING, "%s: timer write @%x.%c = %0*x", this->get_name().c_str(),
            KL_BAR0_BASE + 0x15000 + offset, SIZE_ARG(size), size * 2, value);
    }
}

uint64_t KeyLargo::timer_count() const
{
    // Count elapsed emulator virtual time at the DT clock frequency
    // (18.432 MHz = 0.018432 ticks/ns), since the last reset write.
    return (get_virt_time_ns() - this->timer_reset_ns) * 18432ULL / 1000000ULL;
}

/* ---- I2S (sound) sub-block at mac-io + 0x10000 ----
 *
 *  The boot ROM configures I2S clocks and format, then starts DBDMA
 *  channel 8 (audio out) to stream PCM samples.  The I2S TX data path
 *  is wired to the host SoundServer so that the boot chime plays through
 *  the host audio device.
 *
 *  Register map (offsets relative to sub-block base 0x80010000):
 *    0x000  I2S control    (bit 0 = TX enable)
 *    0x004  frame counter
 *    0x008  status          (read-only, 0 = idle)
 *    0x00C  serial clock config
 *    0x010  data FIFO       (write-only, ignored – DMA feeds data directly)
 */

uint32_t KeyLargo::i2s_read(uint32_t offset, int size)
{
    switch (offset & ~3) {
    case KL_I2S_CTRL:
        return this->i2s_ctrl;
    case KL_I2S_FRAME_CNT:
        return this->i2s_frame_cnt;
    case KL_I2S_STATUS:
        return this->i2s_status;
    case KL_I2S_CLK_CFG:
        return this->i2s_clk_cfg;
    default:
        if (!(this->unsupported_read_mask & (1 << KL_SUB_I2S))) {
            this->unsupported_read_mask |= (1 << KL_SUB_I2S);
            LOG_F(WARNING, "%s: I2S read @%x.%c", this->get_name().c_str(),
                KL_BAR0_BASE + 0x10000 + offset, SIZE_ARG(size));
        }
        return 0;
    }
}

void KeyLargo::i2s_write(uint32_t offset, uint32_t value, int size)
{
    switch (offset & ~3) {
    case KL_I2S_CTRL: {
        uint32_t old = this->i2s_ctrl;
        this->i2s_ctrl = value;
        // bit 0 = I2S TX enable
        if ((value & 1) && !(old & 1)) {
            this->i2s_dma_out_start();
        } else if (!(value & 1) && (old & 1)) {
            this->i2s_dma_out_stop();
        }
        break;
    }
    case KL_I2S_FRAME_CNT:
        this->i2s_frame_cnt = value;
        break;
    case KL_I2S_CLK_CFG:
        this->i2s_clk_cfg = value;
        break;
    case KL_I2S_FIFO:
        // write to TX FIFO ignored – data arrives via DMA
        break;
    default:
        if (!(this->unsupported_write_mask & (1 << KL_SUB_I2S))) {
            this->unsupported_write_mask |= (1 << KL_SUB_I2S);
            LOG_F(WARNING, "%s: I2S write @%x.%c = %0*x", this->get_name().c_str(),
                KL_BAR0_BASE + 0x10000 + offset, SIZE_ARG(size), size * 2, value);
        }
    }
}

void KeyLargo::i2s_dma_out_start()
{
    if (this->out_stream_ready)
        return;

    if (!this->snd_server) {
        this->snd_server = dynamic_cast<SoundServer *>(
            gMachineObj->get_comp_by_name("SoundServer"));
        if (!this->snd_server) {
            LOG_F(ERROR, "%s: SoundServer not found", this->get_name().c_str());
            return;
        }
    }

    // Boot chime is 44100 Hz stereo 16-bit PCM.  The I2S serial clock
    // divider is not decoded – just use the standard rate.
    int err = this->snd_server->open_out_stream(44100, this->snd_out_dma.get());
    if (err) {
        LOG_F(ERROR, "%s: unable to open sound output stream: %d",
              this->get_name().c_str(), err);
        return;
    }

    err = this->snd_server->start_out_stream();
    if (err) {
        LOG_F(ERROR, "%s: could not start sound output stream: %d",
              this->get_name().c_str(), err);
        return;
    }

    this->out_stream_ready = true;
    LOG_F(INFO, "%s: I2S TX started, 44100 Hz output stream open", this->get_name().c_str());
}

void KeyLargo::i2s_dma_out_stop()
{
    if (this->out_stream_ready && this->snd_server) {
        this->snd_server->close_out_stream();
        this->out_stream_ready = false;
        LOG_F(INFO, "%s: I2S TX stopped", this->get_name().c_str());
    }
}

/* ---- DBDMA sub-block at mac-io + 0x08000 ----
 *
 *  The DBDMA controller provides 16 DMA channels, each 0x100 bytes apart.
 *  Channel 8 = audio out (TX), channel 9 = audio in (RX).
 *  Only the audio channels are wired; the rest return 0 / are ignored.
 */

uint32_t KeyLargo::dbdma_read(uint32_t offset, int size)
{
    int ch = offset >> 8;

    switch (ch) {
    case KL_DMA_AUDIO_OUT:
        return this->snd_out_dma->reg_read(offset & 0xFF, size);
    case KL_DMA_AUDIO_IN:
        return this->snd_in_dma->reg_read(offset & 0xFF, size);
    default:
        if (!(this->unsupported_read_mask & (1 << KL_SUB_DBDMA))) {
            this->unsupported_read_mask |= (1 << KL_SUB_DBDMA);
            LOG_F(WARNING, "%s: DBDMA ch %d read @%x.%c", this->get_name().c_str(),
                ch, KL_BAR0_BASE + 0x08000 + offset, SIZE_ARG(size));
        }
        return 0;
    }
}

void KeyLargo::dbdma_write(uint32_t offset, uint32_t value, int size)
{
    int ch = offset >> 8;

    switch (ch) {
    case KL_DMA_AUDIO_OUT:
        this->snd_out_dma->reg_write(offset & 0xFF, value, size);
        break;
    case KL_DMA_AUDIO_IN:
        this->snd_in_dma->reg_write(offset & 0xFF, value, size);
        break;
    default:
        if (!(this->unsupported_write_mask & (1 << KL_SUB_DBDMA))) {
            this->unsupported_write_mask |= (1 << KL_SUB_DBDMA);
            LOG_F(WARNING, "%s: DBDMA ch %d write @%x.%c = %0*x", this->get_name().c_str(),
                ch, KL_BAR0_BASE + 0x08000 + offset, SIZE_ARG(size), size * 2, value);
        }
    }
}

/* ESCC registers (MacRISC addressing within the 0x13 sub-block):
 *   +0x00 ch-a status/cmd, +0x10 ch-a data, +0x20 ch-b status/cmd,
 *   +0x30 ch-b data. Status: bit 0 = RX data ready, bit 2 = TX buffer empty. */
uint8_t KeyLargo::escc_read_byte(uint32_t offset)
{
    uint32_t rel = offset & 0xFFF;

    if (rel & 0x10)
        return 0xFF; // data register: no serial input connected

    // status/command register: bit 2 = TX buffer empty (no RX data ever ready)
    return 0x04;
}

void KeyLargo::escc_write_byte(uint32_t offset, uint8_t value)
{
    uint32_t rel = offset & 0xFFF;

    if (rel & 0x10) {
        this->escc_tx_byte(value); // data register = TX
        return;
    }
    // status/command register: command bytes accepted and ignored.
}

void KeyLargo::escc_tx_byte(uint8_t value)
{
    // Accumulate console output and emit complete lines so the ROM's
    // diagnostic output shows up in the emulator log.
    if (value == '\r' || value == '\n') {
        if (!this->escc_tx_buf.empty()) {
            LOG_F(INFO, "%s: ESCC TX: %s", this->get_name().c_str(),
                this->escc_tx_buf.c_str());
            this->escc_tx_buf.clear();
        }
        return;
    }
    if (value >= 0x20 && value < 0x7F)
        this->escc_tx_buf.push_back((char)value);
    else
        this->escc_tx_buf.push_back('.');
    if (this->escc_tx_buf.size() >= 256) {
        LOG_F(INFO, "%s: ESCC TX: %s", this->get_name().c_str(),
            this->escc_tx_buf.c_str());
        this->escc_tx_buf.clear();
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
