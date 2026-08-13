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

/** OpenPIC (MPIC) interrupt controller emulation.
 *
 * The OpenPIC is the interrupt controller used on UniNorth-based New World
 * Macs. On the Mac mini G4 it lives inside the KeyLargo mac-io at
 * mac-io + 0x40000 (256 KiB window, "interrupt-controller@40000", compatible
 * "chrp,open-pic") and is the interrupt-parent of the three UniNorth PCI
 * buses and all mac-io internal devices. It presents a single external
 * interrupt line to the CPU (ppc_assert_int/ppc_release_int).
 *
 * Register map (standard OpenPIC, as described by Linux
 * arch/powerpc/include/asm/mpic.h):
 *   0x01000 global registers (feature reporting, config, IPI, spurious, ...)
 *   0x01100 timer group (8 timers, stride 0x40, group stride 0x1000)
 *   0x10000 per-source registers (stride 0x20, one per interrupt source)
 *   0x20000 per-CPU registers (stride 0x1000; INTACK read @+0xA0, EOI @+0xB0)
 *   0x30000 EOI window (writes only, acts on CPU 0)
 */

#ifndef OPENPIC_H
#define OPENPIC_H

#include <devices/common/hwinterrupt.h>
#include <devices/common/mmiodevice.h>

#include <cinttypes>
#include <memory>

/** Register-block bases within the OpenPIC window (per Linux mpic.h). */
enum OpenPicRegBase : uint32_t {
    OPENPIC_GREG_BASE    = 0x01000,
    OPENPIC_TIMER_BASE   = 0x01100,
    OPENPIC_SOURCE_BASE  = 0x10000,
    OPENPIC_SOURCE_STRIDE= 0x00020,
    OPENPIC_CPU_BASE     = 0x20000,
    OPENPIC_CPU_STRIDE   = 0x01000,
    OPENPIC_EOI_WINDOW   = 0x30000,
};

/** Per-source register offsets (within a 0x20 source block). */
enum OpenPicIrqReg : uint32_t {
    OPENPIC_IRQ_VECTOR_PRI  = 0x00000,
    OPENPIC_IRQ_DESTINATION = 0x00010,
};

/** Per-source vector/priority register bits. */
enum OpenPicVecPri : uint32_t {
    OPENPIC_VECPRI_MASK        = 0x80000000, // 1 = interrupt masked
    OPENPIC_VECPRI_ACTIVITY    = 0x40000000, // read only
    OPENPIC_VECPRI_POLARITY    = 0x00800000, // 1 = positive, 0 = negative
    OPENPIC_VECPRI_SENSE_LEVEL = 0x00400000, // 1 = level, 0 = edge
    OPENPIC_VECPRI_PRIORITY    = 0x000F0000,
    OPENPIC_VECPRI_PRIORITY_SHIFT = 16,
    OPENPIC_VECPRI_VECTOR      = 0x000007FF,
};

/** OpenPIC global registers (offsets within the 0x1000 block). */
enum OpenPicGreg : uint32_t {
    OPENPIC_GREG_FEATURE_0       = 0x00000,
    OPENPIC_GREG_FEATURE_1       = 0x00010,
    OPENPIC_GREG_GLOBAL_CONF_0   = 0x00020,
    OPENPIC_GREG_GLOBAL_CONF_1   = 0x00030,
    OPENPIC_GREG_VENDOR_0        = 0x00040,
    OPENPIC_GREG_VENDOR_1        = 0x00050,
    OPENPIC_GREG_VENDOR_2        = 0x00060,
    OPENPIC_GREG_VENDOR_3        = 0x00070,
    OPENPIC_GREG_VENDOR_ID       = 0x00080,
    OPENPIC_GREG_PROCESSOR_INIT  = 0x00090,
    OPENPIC_GREG_IPI_VECTOR_PRI_0= 0x000A0,
    OPENPIC_GREG_IPI_STRIDE      = 0x00010,
    OPENPIC_GREG_SPURIOUS        = 0x000E0,
    OPENPIC_GREG_TIMER_FREQ      = 0x000F0,
};

/** OpenPIC per-CPU registers (offsets within a CPU block). */
enum OpenPicCpuReg : uint32_t {
    OPENPIC_CPU_IPI_DISPATCH_0    = 0x00040,
    OPENPIC_CPU_IPI_DISPATCH_STRIDE = 0x00010,
    OPENPIC_CPU_CURRENT_TASK_PRI  = 0x00080,
    OPENPIC_CPU_WHOAMI            = 0x00090,
    OPENPIC_CPU_INTACK            = 0x000A0,
    OPENPIC_CPU_EOI               = 0x000B0,
    OPENPIC_CPU_MCACK             = 0x000C0,
};

/** OpenPIC timer registers (offsets within a timer block). */
enum OpenPicTimerReg : uint32_t {
    OPENPIC_TIMER_CURRENT_CNT  = 0x00000,
    OPENPIC_TIMER_BASE_CNT     = 0x00010,
    OPENPIC_TIMER_VECTOR_PRI   = 0x00020,
    OPENPIC_TIMER_DESTINATION  = 0x00030,
};

class OpenPic : public MMIODevice, public InterruptCtrl {
public:
    OpenPic();
    ~OpenPic() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<OpenPic>(new OpenPic());
    }

    int device_postinit() override;

    // MMIODevice methods (dispatched by the KeyLargo MMIO window)
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

    // InterruptCtrl methods
    uint64_t register_dev_int(IntSrc src_id) override;
    uint64_t register_dma_int(IntSrc src_id) override;
    void ack_int(uint64_t irq_id, uint8_t irq_line_state) override;
    void ack_dma_int(uint64_t irq_id, uint8_t irq_line_state) override;

private:
    struct OpenPicSource {
        uint32_t vecpri   = OPENPIC_VECPRI_MASK; // masked until programmed
        uint32_t dest     = 0x00000001;           // delivered to CPU 0
        bool     input    = false;                // current line level
        bool     pending  = false;                // latched (edge) / level
        bool     in_service = false;              // between INTACK and EOI
    };

    static int source_number_for_intsrc(IntSrc src_id);

    uint32_t global_read(uint32_t offset, int size);
    void global_write(uint32_t offset, uint32_t value, int size);
    uint32_t source_read(uint32_t offset, int size);
    void source_write(uint32_t offset, uint32_t value, int size);
    uint32_t cpu_read(uint32_t offset, int size);
    void cpu_write(uint32_t offset, uint32_t value, int size);
    uint32_t timer_read(uint32_t offset, int size);
    void timer_write(uint32_t offset, uint32_t value, int size);

    int  best_source(); // highest-priority deliverable source, -1 if none
    void ack_best();    // mark it in-service and clear edge pending
    void eoi_best();    // clear the highest-priority in-service source
    void update_cpu_irq();

    static const int OPENPIC_NUM_SOURCES = 64; // sources 0..63
    static const int OPENPIC_NUM_TIMERS  = 8;
    static const int OPENPIC_NUM_CPUS    = 1;

    OpenPicSource src[OPENPIC_NUM_SOURCES] = {};

    uint32_t gcr_global_conf  = 0;
    uint32_t global_conf1     = 0;
    uint32_t vendor_id        = 0x0000006B; // Apple (vendor only)
    uint32_t processor_init   = 0;
    uint32_t ipi_vecpri[4]    = {};
    uint32_t spurious         = 0x0000FFFF;
    uint32_t timer_freq       = 0;

    uint32_t timer_base_cnt[OPENPIC_NUM_TIMERS]   = {};
    uint32_t timer_cur_cnt[OPENPIC_NUM_TIMERS]    = {};
    uint32_t timer_vecpri[OPENPIC_NUM_TIMERS]     = {};
    uint32_t timer_dest[OPENPIC_NUM_TIMERS]       = {};

    // CPU 0 state (single CPU)
    uint32_t ctp_reg          = 0xF0000000; // task priority 0xF = all masked
    uint32_t ipi_dispatch[4]  = {};
    bool     ipi_pending[4]   = {};
};

#endif // OPENPIC_H
