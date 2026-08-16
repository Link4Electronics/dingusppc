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

/** @file TSB43AB22 FireWire OHCI controller emulation.

    Minimal register-level stub of the TI TSB43AB22 integrated
    1394a-2000 OHCI PHY/Link controller (internal PCI bus device 0x0E,
    BAR0 4 KiB at 0xF5000000, OpenPIC source 40). The 2 KB OHCI
    register space (0x000-0x7FC) is stubbed so that the boot ROM's PCI
    enumeration, Open Firmware probe, and the Linux firewire_ohci driver
    can all proceed without hanging.

    No DMA engine, link-layer, or PHY emulation is implemented. All
    write-1-to-set / write-1-to-clear register pairs maintain proper
    semantics. */

#include <core/endianswap.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/common/pci/pcihost.h>
#include <devices/common/firewire/tsb43ab22.h>
#include <devices/deviceregistry.h>
#include <loguru.hpp>

// --- OHCI register offsets (BAR0-relative) ---

// Core / CSR
constexpr uint32_t OHCI_VERSION          = 0x000;
constexpr uint32_t OHCI_GUID_ROM         = 0x004;
constexpr uint32_t OHCI_AT_RETRIES        = 0x008;
constexpr uint32_t OHCI_CSR_DATA          = 0x00C;
constexpr uint32_t OHCI_CSR_COMPARE       = 0x010;
constexpr uint32_t OHCI_CSR_CONTROL       = 0x014;
constexpr uint32_t OHCI_CONFIG_ROM_HDR    = 0x018;
constexpr uint32_t OHCI_BUS_ID            = 0x01C;
constexpr uint32_t OHCI_BUS_OPTIONS       = 0x020;
constexpr uint32_t OHCI_GUID_HI           = 0x024;
constexpr uint32_t OHCI_GUID_LO           = 0x028;
constexpr uint32_t OHCI_CONFIG_ROM_MAP    = 0x034;
constexpr uint32_t OHCI_POSTED_WRITE_LO   = 0x038;
constexpr uint32_t OHCI_POSTED_WRITE_HI   = 0x03C;
constexpr uint32_t OHCI_VENDOR_ID         = 0x040;

// Host Controller Control (set/clear pair)
constexpr uint32_t OHCI_HC_CONTROL_SET    = 0x050;
constexpr uint32_t OHCI_HC_CONTROL_CLR    = 0x054;

// Self-ID
constexpr uint32_t OHCI_SELF_ID_BUF       = 0x064;
constexpr uint32_t OHCI_SELF_ID_COUNT     = 0x068;

// Isoch receive multi-channel mask (set/clear pairs)
constexpr uint32_t OHCI_IR_MCMASK_HI_SET  = 0x070;
constexpr uint32_t OHCI_IR_MCMASK_HI_CLR  = 0x074;
constexpr uint32_t OHCI_IR_MCMASK_LO_SET  = 0x078;
constexpr uint32_t OHCI_IR_MCMASK_LO_CLR  = 0x07C;

// Interrupt registers (set/clear pairs)
constexpr uint32_t OHCI_INT_EVENT_SET     = 0x080;
constexpr uint32_t OHCI_INT_EVENT_CLR     = 0x084;
constexpr uint32_t OHCI_INT_MASK_SET      = 0x088;
constexpr uint32_t OHCI_INT_MASK_CLR      = 0x08C;
constexpr uint32_t OHCI_ISO_XMIT_INT_SET  = 0x090;
constexpr uint32_t OHCI_ISO_XMIT_INT_CLR  = 0x094;
constexpr uint32_t OHCI_ISO_XMIT_MSK_SET  = 0x098;
constexpr uint32_t OHCI_ISO_XMIT_MSK_CLR  = 0x09C;
constexpr uint32_t OHCI_ISO_RECV_INT_SET   = 0x0A0;
constexpr uint32_t OHCI_ISO_RECV_INT_CLR   = 0x0A4;
constexpr uint32_t OHCI_ISO_RECV_MSK_SET   = 0x0A8;
constexpr uint32_t OHCI_ISO_RECV_MSK_CLR   = 0x0AC;

// Bandwidth / isoch resource management
constexpr uint32_t OHCI_INIT_BW_AVAIL     = 0x0B0;
constexpr uint32_t OHCI_INIT_CH_AVAIL_HI  = 0x0B4;
constexpr uint32_t OHCI_INIT_CH_AVAIL_LO  = 0x0B8;
constexpr uint32_t OHCI_FAIRNESS_CTRL     = 0x0DC;

// Link Control (set/clear pair) and Node ID
constexpr uint32_t OHCI_LINK_CONTROL_SET  = 0x0E0;
constexpr uint32_t OHCI_LINK_CONTROL_CLR  = 0x0E4;
constexpr uint32_t OHCI_NODE_ID           = 0x0E8;
constexpr uint32_t OHCI_PHY_CONTROL       = 0x0EC;
constexpr uint32_t OHCI_ISO_CYC_TIMER     = 0x0F0;

// Address filter (set/clear pairs)
constexpr uint32_t OHCI_AS_REQ_FILT_HI_SET  = 0x100;
constexpr uint32_t OHCI_AS_REQ_FILT_HI_CLR  = 0x104;
constexpr uint32_t OHCI_AS_REQ_FILT_LO_SET  = 0x108;
constexpr uint32_t OHCI_AS_REQ_FILT_LO_CLR  = 0x10C;
constexpr uint32_t OHCI_PHY_REQ_FILT_HI_SET = 0x110;
constexpr uint32_t OHCI_PHY_REQ_FILT_HI_CLR = 0x114;
constexpr uint32_t OHCI_PHY_REQ_FILT_LO_SET = 0x118;
constexpr uint32_t OHCI_PHY_REQ_FILT_LO_CLR = 0x11C;
constexpr uint32_t OHCI_PHY_UPPER_BOUND      = 0x120;

// Async DMA context registers
constexpr uint32_t OHCI_AS_REQ_TR_CTX_SET  = 0x180;
constexpr uint32_t OHCI_AS_REQ_TR_CTX_CLR  = 0x184;
constexpr uint32_t OHCI_AS_REQ_TR_CMDPTR   = 0x18C;
constexpr uint32_t OHCI_AS_RSP_TR_CTX_SET  = 0x1A0;
constexpr uint32_t OHCI_AS_RSP_TR_CTX_CLR  = 0x1A4;
constexpr uint32_t OHCI_AS_RSP_TR_CMDPTR   = 0x1AC;
constexpr uint32_t OHCI_AS_REQ_RCV_CTX_SET = 0x1C0;
constexpr uint32_t OHCI_AS_REQ_RCV_CTX_CLR = 0x1C4;
constexpr uint32_t OHCI_AS_REQ_RCV_CMDPTR  = 0x1CC;
constexpr uint32_t OHCI_AS_RSP_RCV_CTX_SET = 0x1E0;
constexpr uint32_t OHCI_AS_RSP_RCV_CTX_CLR = 0x1E4;
constexpr uint32_t OHCI_AS_RSP_RCV_CMDPTR  = 0x1EC;

// Isoch TX contexts: base 0x200, stride 16 bytes
constexpr uint32_t OHCI_IT_CTX_BASE        = 0x200;
constexpr uint32_t OHCI_IT_CTX_STRIDE      = 0x10;
constexpr uint32_t OHCI_IT_CTX_COUNT       = 4;

// Isoch RX contexts: base 0x400, stride 32 bytes
constexpr uint32_t OHCI_IR_CTX_BASE        = 0x400;
constexpr uint32_t OHCI_IR_CTX_STRIDE      = 0x20;
constexpr uint32_t OHCI_IR_CTX_COUNT       = 4;

// HCControl bit fields
constexpr uint32_t HCCTL_NO_BYTE_SWAP_DATA = 1u << 30;
constexpr uint32_t HCCTL_BIB_IMAGE_VALID   = 1u << 31;
constexpr uint32_t HCCTL_LPS               = 1u << 19;
constexpr uint32_t HCCTL_POSTED_WRITE_ENA  = 1u << 18;
constexpr uint32_t HCCTL_LINK_ENABLE       = 1u << 17;
constexpr uint32_t HCCTL_SOFT_RESET        = 1u << 16;

// IntEvent / IntMask bit fields
constexpr uint32_t INT_MASTER_ENABLE       = 1u << 31;
constexpr uint32_t INT_SELF_ID_COMPLETE    = 1u << 16;
constexpr uint32_t INT_BUS_RESET           = 1u << 17;

// GUID from the Mac mini G4 device tree (local-guid property)
constexpr uint32_t GUID_HI = 0x001124FF;
constexpr uint32_t GUID_LO = 0xFEE4C4E0;

// TI OUI (upper 24 bits of VendorID register)
constexpr uint32_t TI_OUI = 0x00080024;

TSB43AB22::TSB43AB22() : PCIDevice("TSB43AB22")
{
    this->supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV);

    // PCI configuration space header (firewire@e on the internal bus)
    this->vendor_id   = this->subsys_vndr = PCI_VENDOR_APPLE;
    this->device_id   = this->subsys_id   = TSB43AB22_DEV_ID;
    this->class_rev   = 0x0C001081; // FireWire OHCI, rev 0x81 (129)
    this->irq_pin     = 1;          // INT# A
    this->min_gnt     = 0x0C;
    this->max_lat     = 0x18;

    // BAR0: 4 KiB MMIO window (0xF5000000 on the Mac mini G4)
    this->bars_cfg[0] = 0xFFFFF000;
    this->finish_config_bars();

    this->pci_notify_bar_change = [this](int bar_num) {
        this->notify_bar_change(bar_num);
    };
}

void TSB43AB22::notify_bar_change(int bar_num)
{
    if (bar_num)
        return;

    uint32_t new_base = this->bars[bar_num] & 0xFFFFF000UL;
    if (new_base != this->base_addr) {
        if (this->base_addr)
            this->host_instance->pci_unregister_mmio_region(this->base_addr,
                TSB43AB22_BAR_SIZE, this);
        this->base_addr = new_base;
        if (new_base)
            this->host_instance->pci_register_mmio_region(new_base,
                TSB43AB22_BAR_SIZE, this);
        LOG_F(INFO, "%s: BAR0 set to 0x%X", this->name.c_str(), new_base);
    }
}

// --- Interrupt evaluation ---

void TSB43AB22::eval_irq()
{
    if (!(this->int_mask & INT_MASTER_ENABLE)) {
        this->pci_interrupt(0);
        return;
    }

    uint32_t pending = this->int_event & this->int_mask;
    this->pci_interrupt(pending ? 1 : 0);
}

// --- MMIO read ---

uint32_t TSB43AB22::read(uint32_t rgn_start, uint32_t offset, int size)
{
    uint32_t reg = offset & 0x7FC;

    switch (reg) {
    case OHCI_VERSION:
        return 0x00010001; // OHCI 1.1

    case OHCI_GUID_ROM:
        return 0; // no serial ROM present

    case OHCI_AT_RETRIES:
        return this->at_retries;

    case OHCI_CSR_DATA:
        return this->csr_data;

    case OHCI_CSR_COMPARE:
        return this->csr_compare;

    case OHCI_CSR_CONTROL:
        return this->csr_control;

    case OHCI_CONFIG_ROM_HDR:
        return this->config_rom_hdr;

    case OHCI_BUS_ID:
        return 0x31333934; // "1394"

    case OHCI_BUS_OPTIONS:
        return this->bus_options;

    case OHCI_GUID_HI:
        return GUID_HI;

    case OHCI_GUID_LO:
        return GUID_LO;

    case OHCI_CONFIG_ROM_MAP:
        return this->config_rom_map;

    case OHCI_POSTED_WRITE_LO:
        return this->posted_write_lo;

    case OHCI_POSTED_WRITE_HI:
        return this->posted_write_hi;

    case OHCI_VENDOR_ID:
        return TI_OUI;

    // HCControl: read returns current value (same as Clear address)
    case OHCI_HC_CONTROL_SET:
    case OHCI_HC_CONTROL_CLR:
        return this->hc_control;

    case OHCI_SELF_ID_BUF:
        return this->self_id_buf;

    case OHCI_SELF_ID_COUNT:
        // bit 31 = selfIDError (no bus present), generation=0, size=0
        return 0x80000000;

    case OHCI_IR_MCMASK_HI_SET:
    case OHCI_IR_MCMASK_HI_CLR:
        return this->ir_multichan_hi;

    case OHCI_IR_MCMASK_LO_SET:
    case OHCI_IR_MCMASK_LO_CLR:
        return this->ir_multichan_lo;

    // Interrupt event registers: read returns current events
    case OHCI_INT_EVENT_SET:
    case OHCI_INT_EVENT_CLR:
        return this->int_event;

    case OHCI_INT_MASK_SET:
    case OHCI_INT_MASK_CLR:
        return this->int_mask;

    case OHCI_ISO_XMIT_INT_SET:
    case OHCI_ISO_XMIT_INT_CLR:
        return this->isoch_xmit_int_event;

    case OHCI_ISO_XMIT_MSK_SET:
    case OHCI_ISO_XMIT_MSK_CLR:
        return this->isoch_xmit_int_mask;

    case OHCI_ISO_RECV_INT_SET:
    case OHCI_ISO_RECV_INT_CLR:
        return this->isoch_recv_int_event;

    case OHCI_ISO_RECV_MSK_SET:
    case OHCI_ISO_RECV_MSK_CLR:
        return this->isoch_recv_int_mask;

    case OHCI_INIT_BW_AVAIL:
        return 0x00000F00; // 4915 budget units

    case OHCI_INIT_CH_AVAIL_HI:
        return 0xFFFFFFFE; // channels 1-31 available (ch 0 reserved)

    case OHCI_INIT_CH_AVAIL_LO:
        return 0xFFFFFFFF; // channels 32-63 all available

    case OHCI_FAIRNESS_CTRL:
        return this->fairness_ctrl;

    // Link control: read returns current value
    case OHCI_LINK_CONTROL_SET:
    case OHCI_LINK_CONTROL_CLR:
        return this->link_control;

    case OHCI_NODE_ID:
        // bit 31 = idValid, bit 30 = root, bus=0, node=0
        if (this->hc_control & HCCTL_LINK_ENABLE)
            return 0xC0000000;
        return 0; // not valid when link disabled

    case OHCI_PHY_CONTROL:
        // bit 31 = ReadDone (set), bits[23:16] = read data (0),
        // bit 14 = WritePending (clear)
        return 0x80000000;

    case OHCI_ISO_CYC_TIMER:
        // cycleSeconds[31:25] | cycleCount[24:12] | cycleOffset[11:0]
        return 0;

    case OHCI_AS_REQ_FILT_HI_SET:
    case OHCI_AS_REQ_FILT_HI_CLR:
        return this->as_req_filter_hi;

    case OHCI_AS_REQ_FILT_LO_SET:
    case OHCI_AS_REQ_FILT_LO_CLR:
        return this->as_req_filter_lo;

    case OHCI_PHY_REQ_FILT_HI_SET:
    case OHCI_PHY_REQ_FILT_HI_CLR:
        return this->phy_req_filter_hi;

    case OHCI_PHY_REQ_FILT_LO_SET:
    case OHCI_PHY_REQ_FILT_LO_CLR:
        return this->phy_req_filter_lo;

    case OHCI_PHY_UPPER_BOUND:
        return this->phy_upper_bound;

    // Async request transmit context
    case OHCI_AS_REQ_TR_CTX_SET:
    case OHCI_AS_REQ_TR_CTX_CLR:
        return this->as_req_tr_ctx[0];
    case OHCI_AS_REQ_TR_CMDPTR:
        return this->as_req_tr_ctx[2];

    // Async response transmit context
    case OHCI_AS_RSP_TR_CTX_SET:
    case OHCI_AS_RSP_TR_CTX_CLR:
        return this->as_rsp_tr_ctx[0];
    case OHCI_AS_RSP_TR_CMDPTR:
        return this->as_rsp_tr_ctx[2];

    // Async request receive context
    case OHCI_AS_REQ_RCV_CTX_SET:
    case OHCI_AS_REQ_RCV_CTX_CLR:
        return this->as_req_rcv_ctx[0];
    case OHCI_AS_REQ_RCV_CMDPTR:
        return this->as_req_rcv_ctx[2];

    // Async response receive context
    case OHCI_AS_RSP_RCV_CTX_SET:
    case OHCI_AS_RSP_RCV_CTX_CLR:
        return this->as_rsp_rcv_ctx[0];
    case OHCI_AS_RSP_RCV_CMDPTR:
        return this->as_rsp_rcv_ctx[2];

    default:
        break;
    }

    // Isoch TX contexts
    if (reg >= OHCI_IT_CTX_BASE &&
        reg < OHCI_IT_CTX_BASE + OHCI_IT_CTX_COUNT * OHCI_IT_CTX_STRIDE) {
        unsigned idx = (reg - OHCI_IT_CTX_BASE) / OHCI_IT_CTX_STRIDE;
        unsigned sub = (reg - OHCI_IT_CTX_BASE) % OHCI_IT_CTX_STRIDE;
        if (sub <= 0x0C && (sub & 3) == 0) {
            unsigned elem = sub / 4;
            if (elem == 0) // Set and Clear share the same storage
                return this->it_ctx[idx][0];
            return this->it_ctx[idx][elem];
        }
        return 0;
    }

    // Isoch RX contexts
    if (reg >= OHCI_IR_CTX_BASE &&
        reg < OHCI_IR_CTX_BASE + OHCI_IR_CTX_COUNT * OHCI_IR_CTX_STRIDE) {
        unsigned idx = (reg - OHCI_IR_CTX_BASE) / OHCI_IR_CTX_STRIDE;
        unsigned sub = (reg - OHCI_IR_CTX_BASE) % OHCI_IR_CTX_STRIDE;
        if (sub <= 0x0C && (sub & 3) == 0) {
            unsigned elem = sub / 4;
            if (elem == 0)
                return this->ir_ctx[idx][0];
            return this->ir_ctx[idx][elem];
        }
        if (sub == 0x10) // ContextMatch
            return this->ir_ctx[idx][3];
        return 0;
    }

    return 0;
}

// --- MMIO write ---

void TSB43AB22::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    uint32_t reg = offset & 0x7FC;

    switch (reg) {
    case OHCI_AT_RETRIES:
        this->at_retries = value;
        return;

    case OHCI_CSR_DATA:
        this->csr_data = value;
        return;

    case OHCI_CSR_COMPARE:
        this->csr_compare = value;
        return;

    case OHCI_CSR_CONTROL:
        this->csr_control = value;
        return;

    case OHCI_CONFIG_ROM_HDR:
        this->config_rom_hdr = value;
        return;

    case OHCI_BUS_OPTIONS:
        this->bus_options = value;
        return;

    case OHCI_CONFIG_ROM_MAP:
        this->config_rom_map = value;
        return;

    case OHCI_POSTED_WRITE_LO:
        this->posted_write_lo = value;
        return;

    case OHCI_POSTED_WRITE_HI:
        this->posted_write_hi = value;
        return;

    // HCControl Set/Clear
    case OHCI_HC_CONTROL_SET:
        this->hc_control |= value;
        return;

    case OHCI_HC_CONTROL_CLR:
        this->hc_control &= ~value;
        // softReset self-clears
        this->hc_control &= ~HCCTL_SOFT_RESET;
        return;

    case OHCI_SELF_ID_BUF:
        this->self_id_buf = value;
        return;

    case OHCI_IR_MCMASK_HI_SET:
        this->ir_multichan_hi |= value;
        return;
    case OHCI_IR_MCMASK_HI_CLR:
        this->ir_multichan_hi &= ~value;
        return;

    case OHCI_IR_MCMASK_LO_SET:
        this->ir_multichan_lo |= value;
        return;
    case OHCI_IR_MCMASK_LO_CLR:
        this->ir_multichan_lo &= ~value;
        return;

    // Interrupt event Set/Clear
    case OHCI_INT_EVENT_SET:
        this->int_event |= value;
        this->eval_irq();
        return;

    case OHCI_INT_EVENT_CLR:
        this->int_event &= ~value;
        this->eval_irq();
        return;

    case OHCI_INT_MASK_SET:
        this->int_mask |= value;
        this->eval_irq();
        return;

    case OHCI_INT_MASK_CLR:
        this->int_mask &= ~value;
        this->eval_irq();
        return;

    case OHCI_ISO_XMIT_INT_SET:
        this->isoch_xmit_int_event |= value;
        return;
    case OHCI_ISO_XMIT_INT_CLR:
        this->isoch_xmit_int_event &= ~value;
        return;

    case OHCI_ISO_XMIT_MSK_SET:
        this->isoch_xmit_int_mask |= value;
        return;
    case OHCI_ISO_XMIT_MSK_CLR:
        this->isoch_xmit_int_mask &= ~value;
        return;

    case OHCI_ISO_RECV_INT_SET:
        this->isoch_recv_int_event |= value;
        return;
    case OHCI_ISO_RECV_INT_CLR:
        this->isoch_recv_int_event &= ~value;
        return;

    case OHCI_ISO_RECV_MSK_SET:
        this->isoch_recv_int_mask |= value;
        return;
    case OHCI_ISO_RECV_MSK_CLR:
        this->isoch_recv_int_mask &= ~value;
        return;

    case OHCI_FAIRNESS_CTRL:
        this->fairness_ctrl = value;
        return;

    // Link control Set/Clear
    case OHCI_LINK_CONTROL_SET:
        this->link_control |= value;
        return;

    case OHCI_LINK_CONTROL_CLR:
        this->link_control &= ~value;
        return;

    case OHCI_PHY_CONTROL:
        this->phy_control = value;
        return;

    // Address filter Set/Clear
    case OHCI_AS_REQ_FILT_HI_SET:
        this->as_req_filter_hi |= value;
        return;
    case OHCI_AS_REQ_FILT_HI_CLR:
        this->as_req_filter_hi &= ~value;
        return;

    case OHCI_AS_REQ_FILT_LO_SET:
        this->as_req_filter_lo |= value;
        return;
    case OHCI_AS_REQ_FILT_LO_CLR:
        this->as_req_filter_lo &= ~value;
        return;

    case OHCI_PHY_REQ_FILT_HI_SET:
        this->phy_req_filter_hi |= value;
        return;
    case OHCI_PHY_REQ_FILT_HI_CLR:
        this->phy_req_filter_hi &= ~value;
        return;

    case OHCI_PHY_REQ_FILT_LO_SET:
        this->phy_req_filter_lo |= value;
        return;
    case OHCI_PHY_REQ_FILT_LO_CLR:
        this->phy_req_filter_lo &= ~value;
        return;

    case OHCI_PHY_UPPER_BOUND:
        this->phy_upper_bound = value;
        return;

    // Async DMA contexts
    case OHCI_AS_REQ_TR_CTX_SET:
    case OHCI_AS_REQ_TR_CTX_CLR:
        this->as_req_tr_ctx[0] = value;
        return;
    case OHCI_AS_REQ_TR_CMDPTR:
        this->as_req_tr_ctx[2] = value;
        return;

    case OHCI_AS_RSP_TR_CTX_SET:
    case OHCI_AS_RSP_TR_CTX_CLR:
        this->as_rsp_tr_ctx[0] = value;
        return;
    case OHCI_AS_RSP_TR_CMDPTR:
        this->as_rsp_tr_ctx[2] = value;
        return;

    case OHCI_AS_REQ_RCV_CTX_SET:
    case OHCI_AS_REQ_RCV_CTX_CLR:
        this->as_req_rcv_ctx[0] = value;
        return;
    case OHCI_AS_REQ_RCV_CMDPTR:
        this->as_req_rcv_ctx[2] = value;
        return;

    case OHCI_AS_RSP_RCV_CTX_SET:
    case OHCI_AS_RSP_RCV_CTX_CLR:
        this->as_rsp_rcv_ctx[0] = value;
        return;
    case OHCI_AS_RSP_RCV_CMDPTR:
        this->as_rsp_rcv_ctx[2] = value;
        return;

    default:
        break;
    }

    // Isoch TX contexts
    if (reg >= OHCI_IT_CTX_BASE &&
        reg < OHCI_IT_CTX_BASE + OHCI_IT_CTX_COUNT * OHCI_IT_CTX_STRIDE) {
        unsigned idx = (reg - OHCI_IT_CTX_BASE) / OHCI_IT_CTX_STRIDE;
        unsigned sub = (reg - OHCI_IT_CTX_BASE) % OHCI_IT_CTX_STRIDE;
        if (sub <= 0x0C && (sub & 3) == 0) {
            unsigned elem = sub / 4;
            this->it_ctx[idx][elem] = value;
        }
        return;
    }

    // Isoch RX contexts
    if (reg >= OHCI_IR_CTX_BASE &&
        reg < OHCI_IR_CTX_BASE + OHCI_IR_CTX_COUNT * OHCI_IR_CTX_STRIDE) {
        unsigned idx = (reg - OHCI_IR_CTX_BASE) / OHCI_IR_CTX_STRIDE;
        unsigned sub = (reg - OHCI_IR_CTX_BASE) % OHCI_IR_CTX_STRIDE;
        if (sub <= 0x0C && (sub & 3) == 0) {
            unsigned elem = sub / 4;
            this->ir_ctx[idx][elem] = value;
        } else if (sub == 0x10) {
            this->ir_ctx[idx][3] = value;
        }
        return;
    }

    LOG_F(9, "%s: write to unhandled register 0x%03X = 0x%08X",
        this->name.c_str(), reg, value);
}

static const DeviceDescription TSB43AB22_Descriptor = {
    TSB43AB22::create, {}, {},
    HWCompType::MMIO_DEV | HWCompType::PCI_DEV
};

REGISTER_DEVICE(TSB43AB22, TSB43AB22_Descriptor);
