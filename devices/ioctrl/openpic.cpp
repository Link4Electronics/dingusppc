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

/** OpenPIC (MPIC) interrupt controller emulation (Mac mini G4 / UniNorth 2).
 *
 * See openpic.h for the register map. Delivery model:
 *  - a device asserts/deasserts a source via ack_int()/ack_dma_int(); the
 *    source latches a rising edge (edge-triggered) or tracks the level
 *    (level-triggered) depending on the SENSE bit of its vector/priority
 *    register.
 *  - a source is deliverable when it is unmasked, its priority is higher
 *    than the CPU's task priority, and its destination includes CPU 0.
 *  - while any source is deliverable the CPU external interrupt line is
 *    asserted (ppc_assert_int); reading INTACK returns the highest-priority
 *    deliverable vector and marks the source in-service (edge sources also
 *    clear their pending latch); writing EOI ends the highest-priority
 *    in-service source.
 */

#include <core/memaccess.h>
#include <cpu/ppc/ppcemu.h>
#include <devices/deviceregistry.h>
#include <devices/ioctrl/openpic.h>
#include <loguru.hpp>

#include <cinttypes>
#include <string>

using namespace std;

// Merge a size-byte write into a big-endian dword register.
static uint32_t merge_size(uint32_t reg, uint32_t value, int size)
{
    if (size >= 4)
        return value;
    uint32_t mask = (1U << (8 * size)) - 1;
    int shift = 8 * (4 - size);
    return (reg & ~(mask << shift)) | ((value & mask) << shift);
}

/** Map an IntSrc ID to its OpenPIC source number (Mac mini G4 device tree). */
int OpenPic::source_number_for_intsrc(IntSrc src_id)
{
    switch (src_id) {
    case IntSrc::VIA_CUDA       : return 25; // via-pmu
    case IntSrc::SCCA           : return 22; // escc ch-a
    case IntSrc::SCCB           : return 23; // escc ch-b
    case IntSrc::DAVBUS         : return 30; // i2s
    case IntSrc::NMI            : return 32; // timer
    case IntSrc::ATA            : return 24; // mac-io ata-3
    case IntSrc::IDE0           : return 39; // pci ata-6
    case IntSrc::USB            : return 26; // mac-io i2c@18000
    case IntSrc::USB_KL0        : return 27; // KeyLargo USB dev 0x18
    case IntSrc::USB_KL1        : return 28; // KeyLargo USB dev 0x19
    case IntSrc::USB_KL2        : return 29; // KeyLargo USB dev 0x1A
    case IntSrc::NEC_USB        : return 63; // NEC USB dev 0x1B
    case IntSrc::PCI_WIRELESS   : return 52; // BCM4318 dev 0x12
    case IntSrc::PCI_GPU        : return 48; // Radeon 9200 (AGP bus)
    case IntSrc::FIREWIRE       : return 40; // TSB43AB22 dev 0x0E
    case IntSrc::ETHERNET       : return 41; // GMAC dev 0x0F
    case IntSrc::DMA_IDE0       : return 12; // ata-3 dma
    case IntSrc::DMA_SCCA_Tx    : return 5;
    case IntSrc::DMA_SCCA_Rx    : return 6;
    case IntSrc::DMA_SCCB_Tx    : return 7;
    case IntSrc::DMA_SCCB_Rx    : return 8;
    case IntSrc::DMA_DAVBUS_Tx  : return 1;
    case IntSrc::DMA_DAVBUS_Rx  : return 2;
    default:
        return -1;
    }
}

OpenPic::OpenPic() : MMIODevice(), InterruptCtrl() {
    supports_types(HWCompType::MMIO_DEV | HWCompType::INT_CTRL);
}

int OpenPic::device_postinit()
{
    return 0;
}

uint64_t OpenPic::register_dev_int(IntSrc src_id)
{
    int source = OpenPic::source_number_for_intsrc(src_id);
    if (source < 0) {
        ABORT_F("%s: unknown interrupt source %d", this->get_name().c_str(), src_id);
        return 0;
    }
    return source;
}

uint64_t OpenPic::register_dma_int(IntSrc src_id)
{
    return OpenPic::register_dev_int(src_id);
}

void OpenPic::ack_int(uint64_t irq_id, uint8_t irq_line_state)
{
    if (irq_id >= OPENPIC_NUM_SOURCES) {
        LOG_F(WARNING, "%s: interrupt on invalid source %" PRIu64, this->get_name().c_str(), irq_id);
        return;
    }

    OpenPicSource& s = this->src[irq_id];
    bool was_input = s.input;
    s.input = irq_line_state != 0;

    if (s.vecpri & OPENPIC_VECPRI_SENSE_LEVEL) {
        s.pending = s.input; // level-triggered: track the line level
    } else if (s.input && !was_input) {
        s.pending = true; // edge-triggered: latch the rising edge
    }

    this->update_cpu_irq();
}

void OpenPic::ack_dma_int(uint64_t irq_id, uint8_t irq_line_state)
{
    this->ack_int(irq_id, irq_line_state);
}

uint32_t OpenPic::read(uint32_t rgn_start, uint32_t offset, int size)
{
    uint32_t reg = 0;

    if (offset >= OPENPIC_GREG_BASE && offset < 0x2000) {
        reg = this->global_read(offset & 0xFFF, size);
    } else if (offset >= OPENPIC_SOURCE_BASE &&
               offset < OPENPIC_SOURCE_BASE + OPENPIC_NUM_SOURCES * OPENPIC_SOURCE_STRIDE) {
        reg = this->source_read(offset - OPENPIC_SOURCE_BASE, size);
    } else if (offset >= OPENPIC_CPU_BASE &&
               offset < OPENPIC_CPU_BASE + OPENPIC_CPU_STRIDE) {
        reg = this->cpu_read(offset - OPENPIC_CPU_BASE, size);
    }

    return reg >> (8 * (4 - size));
}

void OpenPic::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    if (offset >= OPENPIC_GREG_BASE && offset < 0x2000) {
        this->global_write(offset & 0xFFF, value, size);
    } else if (offset >= OPENPIC_SOURCE_BASE &&
               offset < OPENPIC_SOURCE_BASE + OPENPIC_NUM_SOURCES * OPENPIC_SOURCE_STRIDE) {
        this->source_write(offset - OPENPIC_SOURCE_BASE, value, size);
    } else if (offset >= OPENPIC_CPU_BASE &&
               offset < OPENPIC_CPU_BASE + OPENPIC_CPU_STRIDE) {
        this->cpu_write(offset - OPENPIC_CPU_BASE, value, size);
    } else if (offset >= OPENPIC_EOI_WINDOW) {
        this->eoi_best(); // EOI window: any write ends the current interrupt
    }
}

uint32_t OpenPic::global_read(uint32_t offset, int size)
{
    if (offset >= OPENPIC_TIMER_BASE &&
        offset < OPENPIC_TIMER_BASE + OPENPIC_NUM_TIMERS * 0x40)
        return this->timer_read(offset - OPENPIC_TIMER_BASE, size);

    switch (offset) {
    case OPENPIC_GREG_FEATURE_0:
        // MPIC 1.2 (version 2), 1 CPU (last = 0), 64 sources (last = 63)
        return 0x00000002
            | (uint32_t(OPENPIC_NUM_CPUS - 1) << 8)
            | (uint32_t(OPENPIC_NUM_SOURCES - 1) << 16);
    case OPENPIC_GREG_GLOBAL_CONF_0:
        return this->gcr_global_conf;
    case OPENPIC_GREG_GLOBAL_CONF_1:
        return this->global_conf1;
    case OPENPIC_GREG_VENDOR_0:
    case OPENPIC_GREG_VENDOR_1:
    case OPENPIC_GREG_VENDOR_2:
    case OPENPIC_GREG_VENDOR_3:
        return 0;
    case OPENPIC_GREG_VENDOR_ID:
        return this->vendor_id;
    case OPENPIC_GREG_PROCESSOR_INIT:
        return this->processor_init;
    case OPENPIC_GREG_SPURIOUS:
        return this->spurious;
    case OPENPIC_GREG_TIMER_FREQ:
        return this->timer_freq;
    default:
        if (offset >= OPENPIC_GREG_IPI_VECTOR_PRI_0 &&
            offset < OPENPIC_GREG_IPI_VECTOR_PRI_0 + 4 * OPENPIC_GREG_IPI_STRIDE)
            return this->ipi_vecpri[(offset - OPENPIC_GREG_IPI_VECTOR_PRI_0) >> 4];
        return 0;
    }
}

void OpenPic::global_write(uint32_t offset, uint32_t value, int size)
{
    if (offset >= OPENPIC_TIMER_BASE &&
        offset < OPENPIC_TIMER_BASE + OPENPIC_NUM_TIMERS * 0x40) {
        this->timer_write(offset - OPENPIC_TIMER_BASE, value, size);
        return;
    }

    switch (offset) {
    case OPENPIC_GREG_GLOBAL_CONF_0:
        value = merge_size(this->gcr_global_conf, value, size);
        if (value & 0x80000000) {
            // soft reset: return everything to the reset state
            this->gcr_global_conf = 0;
            this->processor_init  = 0;
            this->spurious        = 0x0000FFFF;
            this->timer_freq      = 0;
            for (int i = 0; i < 4; i++) {
                this->ipi_vecpri[i] = 0;
                this->ipi_pending[i] = false;
            }
            for (int i = 0; i < OPENPIC_NUM_TIMERS; i++) {
                this->timer_base_cnt[i] = 0;
                this->timer_cur_cnt[i]  = 0;
                this->timer_vecpri[i]   = 0;
                this->timer_dest[i]     = 0;
            }
            for (auto& s : this->src)
                s = OpenPicSource();
        } else {
            this->gcr_global_conf = value;
        }
        this->update_cpu_irq();
        break;
    case OPENPIC_GREG_GLOBAL_CONF_1:
        this->global_conf1 = merge_size(this->global_conf1, value, size);
        break;
    case OPENPIC_GREG_VENDOR_0:
    case OPENPIC_GREG_VENDOR_1:
    case OPENPIC_GREG_VENDOR_2:
    case OPENPIC_GREG_VENDOR_3:
        break; // read-only
    case OPENPIC_GREG_PROCESSOR_INIT:
        this->processor_init = value;
        break;
    case OPENPIC_GREG_SPURIOUS:
        this->spurious = merge_size(this->spurious, value, size);
        break;
    case OPENPIC_GREG_TIMER_FREQ:
        this->timer_freq = merge_size(this->timer_freq, value, size);
        break;
    default:
        if (offset >= OPENPIC_GREG_IPI_VECTOR_PRI_0 &&
            offset < OPENPIC_GREG_IPI_VECTOR_PRI_0 + 4 * OPENPIC_GREG_IPI_STRIDE) {
            int idx = (offset - OPENPIC_GREG_IPI_VECTOR_PRI_0) >> 4;
            this->ipi_vecpri[idx] = merge_size(this->ipi_vecpri[idx], value, size);
            this->update_cpu_irq();
        }
    }
}

uint32_t OpenPic::source_read(uint32_t offset, int size)
{
    int n = offset / OPENPIC_SOURCE_STRIDE;
    if (n >= OPENPIC_NUM_SOURCES)
        return 0;

    switch (offset & (OPENPIC_SOURCE_STRIDE - 1)) {
    case OPENPIC_IRQ_VECTOR_PRI:
        return this->src[n].vecpri
            | (this->src[n].in_service ? OPENPIC_VECPRI_ACTIVITY : 0);
    case OPENPIC_IRQ_DESTINATION:
        return this->src[n].dest;
    default:
        return 0;
    }
}

void OpenPic::source_write(uint32_t offset, uint32_t value, int size)
{
    int n = offset / OPENPIC_SOURCE_STRIDE;
    if (n >= OPENPIC_NUM_SOURCES)
        return;

    switch (offset & (OPENPIC_SOURCE_STRIDE - 1)) {
    case OPENPIC_IRQ_VECTOR_PRI:
        value = merge_size(this->src[n].vecpri, value, size);
        this->src[n].vecpri = value & ~OPENPIC_VECPRI_ACTIVITY; // activity is read-only
        break;
    case OPENPIC_IRQ_DESTINATION:
        this->src[n].dest = merge_size(this->src[n].dest, value, size);
        break;
    default:
        return;
    }
    this->update_cpu_irq();
}

uint32_t OpenPic::cpu_read(uint32_t offset, int size)
{
    switch (offset) {
    case OPENPIC_CPU_CURRENT_TASK_PRI:
        return this->ctp_reg;
    case OPENPIC_CPU_WHOAMI:
        return 0;
    case OPENPIC_CPU_INTACK: {
        int best = this->best_source();
        if (best >= 0 && best < OPENPIC_NUM_SOURCES) {
            uint32_t vector = this->src[best].vecpri & OPENPIC_VECPRI_VECTOR;
            this->ack_best();
            return vector;
        }
        if (best >= OPENPIC_NUM_SOURCES) {
            uint32_t vector = this->ipi_vecpri[best - OPENPIC_NUM_SOURCES] & OPENPIC_VECPRI_VECTOR;
            this->ack_best();
            return vector;
        }
        return this->spurious & OPENPIC_VECPRI_VECTOR;
    }
    case OPENPIC_CPU_MCACK:
        return 0;
    default:
        return 0; // IPI dispatch registers are write-only
    }
}

void OpenPic::cpu_write(uint32_t offset, uint32_t value, int size)
{
    switch (offset) {
    case OPENPIC_CPU_CURRENT_TASK_PRI:
        this->ctp_reg = merge_size(this->ctp_reg, value, size) & 0xF0000000;
        this->update_cpu_irq();
        break;
    case OPENPIC_CPU_EOI:
        this->eoi_best();
        break;
    case OPENPIC_CPU_MCACK:
        break;
    default:
        if (offset >= OPENPIC_CPU_IPI_DISPATCH_0 &&
            offset < OPENPIC_CPU_IPI_DISPATCH_0 + 4 * OPENPIC_CPU_IPI_DISPATCH_STRIDE) {
            int idx = (offset - OPENPIC_CPU_IPI_DISPATCH_0) >> 4;
            this->ipi_pending[idx] = true; // edge-triggered: raise the IPI
            this->update_cpu_irq();
        }
    }
}

uint32_t OpenPic::timer_read(uint32_t offset, int size)
{
    int n = offset >> 6;
    if (n >= OPENPIC_NUM_TIMERS)
        return 0;

    switch (offset & 0x3F) {
    case OPENPIC_TIMER_CURRENT_CNT:
        return this->timer_cur_cnt[n];
    case OPENPIC_TIMER_BASE_CNT:
        return this->timer_base_cnt[n];
    case OPENPIC_TIMER_VECTOR_PRI:
        return this->timer_vecpri[n];
    case OPENPIC_TIMER_DESTINATION:
        return this->timer_dest[n];
    default:
        return 0;
    }
}

void OpenPic::timer_write(uint32_t offset, uint32_t value, int size)
{
    int n = offset >> 6;
    if (n >= OPENPIC_NUM_TIMERS)
        return;

    switch (offset & 0x3F) {
    case OPENPIC_TIMER_CURRENT_CNT:
        this->timer_cur_cnt[n] = merge_size(this->timer_cur_cnt[n], value, size);
        break;
    case OPENPIC_TIMER_BASE_CNT:
        this->timer_base_cnt[n] = merge_size(this->timer_base_cnt[n], value, size);
        break;
    case OPENPIC_TIMER_VECTOR_PRI:
        this->timer_vecpri[n] = merge_size(this->timer_vecpri[n], value, size);
        break;
    case OPENPIC_TIMER_DESTINATION:
        this->timer_dest[n] = merge_size(this->timer_dest[n], value, size);
        break;
    default:
        break;
    }
}

/** Highest-priority deliverable interrupt: a source index (0..63) or
 *  OPENPIC_NUM_SOURCES + k for IPI k, or -1 when nothing can be delivered. */
int OpenPic::best_source()
{
    uint32_t ctp = (this->ctp_reg >> 24) & 0xF;
    int best = -1;
    int best_pri = -1;

    for (int i = 0; i < OPENPIC_NUM_SOURCES; i++) {
        const OpenPicSource& s = this->src[i];
        if (s.vecpri & OPENPIC_VECPRI_MASK)
            continue;
        if (!s.pending || s.in_service)
            continue;
        if (!(s.dest & 1)) // destination CPU 0
            continue;
        int pri = (s.vecpri >> OPENPIC_VECPRI_PRIORITY_SHIFT) & 0xF;
        if (pri <= ctp)
            continue;
        if (pri > best_pri) {
            best_pri = pri;
            best = i;
        }
    }

    for (int k = 0; k < 4; k++) {
        if (this->ipi_vecpri[k] & OPENPIC_VECPRI_MASK)
            continue;
        if (!this->ipi_pending[k])
            continue;
        int pri = (this->ipi_vecpri[k] >> OPENPIC_VECPRI_PRIORITY_SHIFT) & 0xF;
        if (pri <= ctp)
            continue;
        if (pri > best_pri) {
            best_pri = pri;
            best = OPENPIC_NUM_SOURCES + k;
        }
    }

    return best;
}

/** Mark the highest-priority deliverable interrupt as served (INTACK). */
void OpenPic::ack_best()
{
    int best = this->best_source();
    if (best >= 0 && best < OPENPIC_NUM_SOURCES) {
        OpenPicSource& s = this->src[best];
        s.in_service = true;
        if (!(s.vecpri & OPENPIC_VECPRI_SENSE_LEVEL))
            s.pending = false; // edge-triggered sources clear on acknowledge
    } else if (best >= OPENPIC_NUM_SOURCES) {
        this->ipi_pending[best - OPENPIC_NUM_SOURCES] = false;
    }
    this->update_cpu_irq();
}

/** End the highest-priority in-service interrupt (EOI). */
void OpenPic::eoi_best()
{
    int best = -1;
    int best_pri = -1;
    for (int i = 0; i < OPENPIC_NUM_SOURCES; i++) {
        if (!this->src[i].in_service)
            continue;
        int pri = (this->src[i].vecpri >> OPENPIC_VECPRI_PRIORITY_SHIFT) & 0xF;
        if (pri > best_pri) {
            best_pri = pri;
            best = i;
        }
    }
    if (best >= 0)
        this->src[best].in_service = false;
    this->update_cpu_irq();
}

void OpenPic::update_cpu_irq()
{
    if (this->best_source() >= 0)
        ppc_assert_int();
    else
        ppc_release_int();
}

static const vector<string> OpenPic_Subdevices = {};

static const DeviceDescription OpenPic_Descriptor = {
    OpenPic::create, OpenPic_Subdevices, {},
    HWCompType::MMIO_DEV | HWCompType::INT_CTRL
};

REGISTER_DEVICE(OpenPic, OpenPic_Descriptor);
