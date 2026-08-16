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

/** @file KeyLargo (Intrepid) USB OHCI controller emulation.

    Minimal OHCI host controller for the Mac mini G4 (PowerMac10,1/10,2).
    The real machine's keyboard is attached to the KeyLargo OHCI cell
    (main PCI bus device 0x1a, "usb@1a", BAR0 0x80083000). Only what Open
    Firmware needs to enumerate a boot keyboard is implemented: the
    register file, a two-port root hub, process-on-access ED/TD list
    execution, and a control/interrupt responder for a low-speed HID boot
    keyboard on port 1. */

#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/common/pci/pcihost.h>
#include <devices/common/usb/keylargousb.h>
#include <devices/deviceregistry.h>
#include <loguru.hpp>
#include <machines/machinebase.h>

#include <core/endianswap.h>
#include <core/hostevents.h>
#include <core/memaccess.h>
#include <cpu/ppc/ppcmmu.h>

#include <algorithm>
#include <cstring>

// ---------------------------------------------------------------------------
// OHCI register offsets
// ---------------------------------------------------------------------------

constexpr auto HC_REVISION          = 0x00;
constexpr auto HC_CONTROL           = 0x04;
constexpr auto HC_COMMAND_STATUS    = 0x08;
constexpr auto HC_INTERRUPT_STATUS  = 0x0C;
constexpr auto HC_INTERRUPT_ENABLE  = 0x10;
constexpr auto HC_INTERRUPT_DISABLE = 0x14;
constexpr auto HC_HCCA              = 0x18;
constexpr auto HC_PERIOD_CURRENT_ED = 0x1C;
constexpr auto HC_CONTROL_HEAD_ED   = 0x20;
constexpr auto HC_CONTROL_CURRENT_ED = 0x24;
constexpr auto HC_BULK_HEAD_ED      = 0x28;
constexpr auto HC_BULK_CURRENT_ED   = 0x2C;
constexpr auto HC_DONE_HEAD         = 0x30;
constexpr auto HC_FM_INTERVAL       = 0x34;
constexpr auto HC_FM_REMAINING      = 0x38;
constexpr auto HC_FM_NUMBER         = 0x3C;
constexpr auto HC_PERIODIC_START    = 0x40;
constexpr auto HC_LS_THRESHOLD      = 0x44;
constexpr auto HC_RH_DESCRIPTOR_A   = 0x48;
constexpr auto HC_RH_DESCRIPTOR_B   = 0x4C;
constexpr auto HC_RH_STATUS         = 0x50;
constexpr auto HC_RH_PORT_STATUS_1  = 0x54;
constexpr auto HC_RH_PORT_STATUS_2  = 0x58;

// HC_CONTROL bits
constexpr auto HC_CTRL_PLE   = (1 << 2); // PeriodicListEnable
constexpr auto HC_CTRL_IE    = (1 << 3); // IsochronousEnable
constexpr auto HC_CTRL_CCE   = (1 << 4); // ControlListEnable
constexpr auto HC_CTRL_BLE   = (1 << 5); // BulkListEnable
constexpr auto HC_CTRL_HCFS  = (3 << 6); // HostControllerFunctionalState
constexpr auto HC_CTRL_RWC   = (1 << 9); // RemoteWakeupConnected
constexpr auto HC_CTRL_RWE   = (1 << 10); // RemoteWakeupEnable

// HC_COMMAND_STATUS bits
constexpr auto HC_CMD_HCR    = (1 << 0); // HostControllerReset
constexpr auto HC_CMD_CLF    = (1 << 16); // ControlListFilled
constexpr auto HC_CMD_BLF    = (1 << 17); // BulkListFilled

// HC_INTERRUPT_* bits
constexpr auto INT_SO   = (1 << 0); // SchedulingOverrun
constexpr auto INT_WDH  = (1 << 1); // WritebackDoneHead
constexpr auto INT_SF   = (1 << 2); // StartofFrame
constexpr auto INT_RD   = (1 << 3); // ResumeDetected
constexpr auto INT_UE   = (1 << 4); // UnrecoverableError
constexpr auto INT_FNO  = (1 << 5); // FrameNumberOverflow
constexpr auto INT_RHSC = (1 << 6); // RootHubStatusChange
constexpr auto INT_OC   = (1 << 7); // OwnershipChange
constexpr auto INT_MIE  = (1U << 31); // MasterInterruptEnable

// HC_RH_STATUS bits
constexpr auto RH_LPS   = (1 << 0); // LocalPowerStatus (0 = powered)
constexpr auto RH_LPSC  = (1 << 1); // LocalPowerStatusChange
constexpr auto RH_DRWE  = (1 << 16); // DeviceRemoteWakeupEnable
constexpr auto RH_OCI   = (1 << 17); // OverCurrentIndicator

// HC_RH_PORT_STATUS bits
constexpr auto PORT_CCS  = (1 << 0); // CurrentConnectStatus
constexpr auto PORT_PES  = (1 << 1); // PortEnableStatus
constexpr auto PORT_PSS  = (1 << 2); // PortSuspendStatus
constexpr auto PORT_POCI = (1 << 3); // PortOverCurrentIndicator
constexpr auto PORT_PRS  = (1 << 4); // PortResetStatus
constexpr auto PORT_PPS  = (1 << 8); // PortPowerStatus
constexpr auto PORT_LSDA = (1 << 9); // LowSpeedDeviceAttached
constexpr auto PORT_CSC  = (1 << 16); // ConnectStatusChange
constexpr auto PORT_PESC = (1 << 17); // PortEnableStatusChange
constexpr auto PORT_PSSC = (1 << 18); // PortSuspendStatusChange
constexpr auto PORT_POCIC = (1 << 19); // PortOverCurrentIndicatorChange
constexpr auto PORT_PRSC = (1 << 20); // PortResetStatusChange

// ED hwINFO bits
constexpr auto ED_FA    = 0x7F;
constexpr auto ED_EN    = (0xF << 8);
constexpr auto ED_LT    = (1 << 13); // low-speed device
constexpr auto ED_SKIP  = (1 << 14);
constexpr auto ED_FMT   = (0x7 << 24); // format: 7 = control, else general
constexpr auto ED_CTRL  = (0x7 << 24);

// TD hwINFO bits
constexpr auto TD_CC    = (0xF << 0); // condition code
constexpr auto TD_DPID  = (0x3 << 5); // data PID: 0 = setup, 1 = out, 2 = in

// ---------------------------------------------------------------------------
// Descriptors for a low-speed HID boot keyboard
// ---------------------------------------------------------------------------

static const uint8_t kbd_device_desc[18] = {
    0x12, 0x01,              // bLength, bDescriptorType (device)
    0x10, 0x01,              // bcdUSB 1.10
    0x00, 0x00, 0x00,        // bDeviceClass/SubClass/Protocol
    0x08,                    // bMaxPacketSize0
    0xAC, 0x05,              // idVendor = Apple
    0x1C, 0x02,              // idProduct = Apple Pro Keyboard
    0x10, 0x01,              // bcdDevice 1.10
    0x00, 0x00,              // iManufacturer, iProduct
    0x01, 0x00,              // bNumConfigurations
};

static const uint8_t kbd_config_desc[34] = {
    0x09, 0x02,              // bLength, bDescriptorType (config)
    0x22, 0x00,              // wTotalLength = 34
    0x01,                    // bNumInterfaces
    0x01,                    // bConfigurationValue
    0x00,                    // iConfiguration
    0x80,                    // bmAttributes (bus powered)
    0x32,                    // bMaxPower = 100 mA
    // interface descriptor
    0x09, 0x04,              // bLength, bDescriptorType (interface)
    0x00,                    // bInterfaceNumber
    0x00,                    // bAlternateSetting
    0x01,                    // bNumEndpoints
    0x03,                    // bInterfaceClass = HID
    0x01,                    // bInterfaceSubClass = boot
    0x01,                    // bInterfaceProtocol = keyboard
    0x00,                    // iInterface
    // HID descriptor
    0x09, 0x21,              // bLength, bDescriptorType (HID)
    0x10, 0x01,              // bcdHID 1.10
    0x00,                    // bCountryCode
    0x01,                    // bNumDescriptors
    0x22,                    // bDescriptorType (report)
    0x3D, 0x00,              // wDescriptorLength = 61
    // endpoint descriptor
    0x07, 0x05,              // bLength, bDescriptorType (endpoint)
    0x81,                    // bEndpointAddress (EP1 IN)
    0x03,                    // bmAttributes (interrupt)
    0x08, 0x00,              // wMaxPacketSize = 8
    0x0A,                    // bInterval = 10 ms
};

static const uint8_t kbd_report_desc[61] = {
    0x05, 0x01,              // Usage Page (Generic Desktop)
    0x09, 0x06,              // Usage (Keyboard)
    0xA1, 0x01,              // Collection (Application)
    0x05, 0x07,              //   Usage Page (Key Codes)
    0x19, 0xE0,              //   Usage Minimum (224)
    0x29, 0xE7,              //   Usage Maximum (231)
    0x15, 0x00,              //   Logical Minimum (0)
    0x25, 0x01,              //   Logical Maximum (1)
    0x75, 0x01,              //   Report Size (1)
    0x95, 0x08,              //   Report Count (8)
    0x81, 0x02,              //   Input (Data, Variable, Absolute)
    0x95, 0x01,              //   Report Count (1)
    0x75, 0x08,              //   Report Size (8)
    0x81, 0x01,              //   Input (Constant)
    0x19, 0x00,              //   Usage Minimum (0)
    0x29, 0x65,              //   Usage Maximum (101)
    0x15, 0x00,              //   Logical Minimum (0)
    0x25, 0x65,              //   Logical Maximum (101)
    0x75, 0x08,              //   Report Size (8)
    0x95, 0x06,              //   Report Count (6)
    0x81, 0x00,              //   Input (Data, Array)
    0x05, 0x08,              //   Usage Page (LEDs)
    0x19, 0x01,              //   Usage Minimum (1)
    0x29, 0x05,              //   Usage Maximum (5)
    0x95, 0x05,              //   Report Count (5)
    0x75, 0x01,              //   Report Size (1)
    0x91, 0x02,              //   Output (Data, Variable, Absolute)
    0x95, 0x01,              //   Report Count (1)
    0x75, 0x03,              //   Report Size (3)
    0x91, 0x01,              //   Output (Constant)
    0xC0,                    // End Collection
};

static const uint8_t kbd_string0_desc[4] = {
    0x04, 0x03,              // bLength, bDescriptorType (string)
    0x09, 0x04,              // wLANGID (English - US)
};

// ---------------------------------------------------------------------------
// ADB key code -> HID keyboard usage table
// ---------------------------------------------------------------------------

static const uint8_t adbkey_to_hid[128] = {
    0x04, 0x16, 0x07, 0x09, 0x0b, 0x0a, 0x1d, 0x1b, // 0x00 A S D F H G Z X
    0x06, 0x19, 0x00, 0x05, 0x14, 0x1a, 0x08, 0x15, // 0x08 C V ISO B Q W E R
    0x1c, 0x17, 0x1e, 0x1f, 0x20, 0x21, 0x23, 0x22, // 0x10 Y T 1 2 3 4 6 5
    0x2e, 0x26, 0x24, 0x2d, 0x25, 0x27, 0x30, 0x12, // 0x18 = 9 7 - 8 0 ] O
    0x18, 0x2f, 0x0c, 0x13, 0x28, 0x0f, 0x0d, 0x34, // 0x20 U [ I P RET L J '
    0x0e, 0x33, 0x31, 0x36, 0x38, 0x11, 0x10, 0x37, // 0x28 K ; \ , / N M .
    0x2b, 0x2c, 0x35, 0x2a, 0x00, 0x29, 0xe0, 0xe3, // 0x30 TAB SP ` DEL - ESC CTL CMD
    0xe1, 0x39, 0xe2, 0x50, 0x4f, 0x51, 0x52, 0x00, // 0x38 SHFT CAP OPT LEF RGT DWN UP -
    0x00, 0x63, 0x00, 0x55, 0x00, 0x57, 0x00, 0x00, // 0x40 - DEC - MUL - ADD - -
    0x00, 0x54, 0x58, 0x00, 0x56, 0x00, 0x00, 0x00, // 0x48 - DIV ENT - SUB - - -
    0x00, 0x62, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, // 0x50 - KP0 KP1 KP2 KP3 KP4 KP5 KP6
    0x5f, 0x60, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x58 KP7 KP8 KP9 - - - - -
    0x3e, 0x3f, 0x40, 0x3c, 0x41, 0x42, 0x00, 0x44, // 0x60 F5 F6 F7 F3 F8 F9 - F11
    0x00, 0x00, 0x00, 0x43, 0x00, 0x45, 0x00, 0x00, // 0x68 - - - F10 - F12 - -
    0x00, 0x00, 0x4a, 0x4b, 0x4c, 0x3d, 0x4d, 0x3b, // 0x70 - - HOM PgU DEL F4 END F2
    0x4e, 0x3a, 0xe5, 0xe6, 0xe4, 0x00, 0x00, 0x00, // 0x78 PgD F1 RSHA ROpt RCtl - - -
};

// ---------------------------------------------------------------------------

KeyLargoUSB::KeyLargoUSB() : PCIDevice("keylargo-usb")
{
    this->supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV);

    // PCI config space (usb@1a on the main bus)
    this->vendor_id = this->subsys_vndr = PCI_VENDOR_APPLE;
    this->device_id = this->subsys_id   = KEYLARGO_USB_DEV_ID;
    this->class_rev = 0x0C031010; // OHCI, revision 0x10
    this->irq_pin   = 1;

    // BAR0: 4 KiB MMIO window (0x80083000 on the Mac mini G4)
    this->bars_cfg[0] = 0xFFFFF000;
    this->finish_config_bars();

    this->pci_notify_bar_change = [this](int bar_num) {
        this->notify_bar_change(bar_num);
    };

    // initial OHCI register values
    memset(this->regs, 0, sizeof(this->regs));
    this->le32_set(HC_REVISION, 0x10);
    this->le32_set(HC_RH_DESCRIPTOR_A, 0x00000002); // NDP = 2
    this->le32_set(HC_FM_INTERVAL, 0x2EDF);
    this->le32_set(HC_PERIODIC_START, 0x2A2F);
    this->le32_set(HC_LS_THRESHOLD, 0x628);
    this->port_change[0] = PORT_CSC; // keyboard attached
}

int KeyLargoUSB::device_postinit()
{
    EventManager::get_instance()->add_keyboard_handler(this, &KeyLargoUSB::event_handler);
    return 0;
}

void KeyLargoUSB::notify_bar_change(int bar_num)
{
    if (bar_num) // only BAR0 is supported
        return;

    uint32_t new_base = this->bars[bar_num] & 0xFFFFF000UL;
    if (new_base != this->base_addr) {
        if (this->base_addr)
            this->host_instance->pci_unregister_mmio_region(this->base_addr,
                0x1000, this);
        this->base_addr = new_base;
        if (new_base)
            this->host_instance->pci_register_mmio_region(new_base, 0x1000, this);
        LOG_F(INFO, "%s: BAR0 set to 0x%X", this->name.c_str(), new_base);
    }
}

// ---------------------------------------------------------------------------
// guest memory helpers
// ---------------------------------------------------------------------------

uint8_t* KeyLargoUSB::map_guest(uint32_t addr, uint32_t size)
{
    MapDmaResult res = mmu_map_dma_mem(addr, size, false, true);
    if (!(res.type & (RT_RAM | RT_ROM)) || !res.host_va)
        return nullptr;
    return res.host_va;
}

uint32_t KeyLargoUSB::le32_get(uint32_t off)
{
    return READ_DWORD_LE_A((uint32_t*)(this->regs + off));
}

void KeyLargoUSB::le32_set(uint32_t off, uint32_t val)
{
    WRITE_DWORD_LE_A((uint32_t*)(this->regs + off), val);
}

// ---------------------------------------------------------------------------
// MMIO access
// ---------------------------------------------------------------------------

uint32_t KeyLargoUSB::read(uint32_t rgn_start, uint32_t offset, int size)
{
    this->process_hc();
    return this->reg_read(offset, size);
}

uint32_t KeyLargoUSB::reg_read(uint32_t offset, int size)
{
    if (offset >= 0x100)
        return 0;

    if (offset == HC_FM_NUMBER)
        this->le32_set(HC_FM_NUMBER, this->frame);

    switch (size) {
    case 1:
        return this->regs[offset];
    case 2:
        return (this->regs[offset] | (this->regs[offset + 1] << 8));
    default:
        return this->le32_get(offset);
    }
}

void KeyLargoUSB::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    this->process_hc();
    this->reg_write(offset, value, size);
}

void KeyLargoUSB::reg_write(uint32_t offset, uint32_t value, int size)
{
    if (offset >= 0x100)
        return;

    if (offset == HC_COMMAND_STATUS) {
        if (value & HC_CMD_HCR) {
            this->hc_reset();
            return;
        }
        if (value & (HC_CMD_CLF | HC_CMD_BLF)) {
            this->le32_set(HC_COMMAND_STATUS,
                this->le32_get(HC_COMMAND_STATUS) | (value & (HC_CMD_CLF | HC_CMD_BLF)));
            this->process_hc();
            return;
        }
        return;
    }

    if (offset == HC_INTERRUPT_STATUS) {
        // write-1-to-clear
        this->le32_set(HC_INTERRUPT_STATUS,
            this->le32_get(HC_INTERRUPT_STATUS) & ~value);
        this->update_pci_irq();
        return;
    }

    if (offset == HC_INTERRUPT_DISABLE) {
        // write-1-to-clear enables
        this->le32_set(HC_INTERRUPT_ENABLE,
            this->le32_get(HC_INTERRUPT_ENABLE) & ~(value & 0xFFFF00FF));
        this->update_pci_irq();
        return;
    }

    if (offset == HC_INTERRUPT_ENABLE) {
        this->le32_set(HC_INTERRUPT_ENABLE,
            this->le32_get(HC_INTERRUPT_ENABLE) | value);
        this->update_pci_irq();
        return;
    }

    if (offset == HC_HCCA) {
        this->hcca_addr = value & ~0xFFUL;
        this->le32_set(HC_HCCA, this->hcca_addr);
        return;
    }

    if (offset == HC_DONE_HEAD) {
        if (value == 0) {
            this->le32_set(HC_DONE_HEAD, 0);
            this->le32_set(HC_INTERRUPT_STATUS,
                this->le32_get(HC_INTERRUPT_STATUS) & ~INT_WDH);
            this->update_pci_irq();
        }
        return;
    }

    if (offset == HC_RH_STATUS) {
        uint32_t v = this->le32_get(HC_RH_STATUS);
        if (value & RH_LPSC)
            v &= ~RH_LPS; // power on
        if (value & (1U << 31))
            v &= ~RH_DRWE;
        if (value & RH_DRWE)
            v |= RH_DRWE;
        this->le32_set(HC_RH_STATUS, v);
        return;
    }

    if (offset == HC_RH_PORT_STATUS_1 || offset == HC_RH_PORT_STATUS_2) {
        int port = (offset == HC_RH_PORT_STATUS_1) ? 0 : 1;
        // write-1-to-clear change bits
        this->port_change[port] &= ~(value & 0xFFFF0000);
        if (value & PORT_PES)
            this->port_enabled[port] = true;
        if (value & PORT_PSS)
            this->port_suspended[port] = true;
        if (value & PORT_PRS) {
            // reset: completes immediately for our virtual keyboard
            this->port_suspended[port] = false;
            this->port_enabled[port] = true;
            this->port_change[port] |= PORT_PESC | PORT_PRSC;
        }
        if ((value & PORT_PPS) == 0) // power off (we always keep it on)
            this->port_enabled[port] = false;
        if (this->port_change[port]) {
            this->le32_set(HC_INTERRUPT_STATUS,
                this->le32_get(HC_INTERRUPT_STATUS) | INT_RHSC);
            this->update_pci_irq();
        }
        return;
    }

    // default: store the (possibly masked) value
    switch (size) {
    case 1:
        this->regs[offset] = value & 0xFF;
        break;
    case 2:
        this->regs[offset] = value & 0xFF;
        this->regs[offset + 1] = (value >> 8) & 0xFF;
        break;
    default:
        WRITE_DWORD_LE_A((uint32_t*)(this->regs + offset), value);
        break;
    }
}

void KeyLargoUSB::hc_reset()
{
    memset(this->regs, 0, sizeof(this->regs));
    this->le32_set(HC_REVISION, 0x10);
    this->le32_set(HC_RH_DESCRIPTOR_A, 0x00000002);
    this->le32_set(HC_FM_INTERVAL, 0x2EDF);
    this->le32_set(HC_PERIODIC_START, 0x2A2F);
    this->le32_set(HC_LS_THRESHOLD, 0x628);
    this->hcca_addr = 0;
    this->frame = 0;
    this->irq_asserted = false;
    this->port_enabled[0] = this->port_enabled[1] = false;
    this->port_suspended[0] = this->port_suspended[1] = false;
    this->port_change[0] = PORT_CSC;
    this->port_change[1] = 0;
    this->have_setup = false;
    this->resp_ptr = nullptr;
    this->resp_remaining = 0;
    this->device_address = 0;
}

void KeyLargoUSB::update_pci_irq()
{
    uint32_t status = this->le32_get(HC_INTERRUPT_STATUS);
    uint32_t enable = this->le32_get(HC_INTERRUPT_ENABLE);
    bool want = (enable & INT_MIE) && (status & enable & 0xFF);
    if (want != this->irq_asserted) {
        this->irq_asserted = want;
        this->pci_interrupt(want ? 1 : 0);
    }
}

// ---------------------------------------------------------------------------
// ED/TD list execution (process-on-access)
// ---------------------------------------------------------------------------

void KeyLargoUSB::process_hc()
{
    uint32_t ctrl = this->le32_get(HC_CONTROL);
    if (((ctrl & HC_CTRL_HCFS) >> 6) != 1) // not USBOperational
        return;

    // control list
    if ((ctrl & HC_CTRL_CCE) && this->le32_get(HC_CONTROL_HEAD_ED)) {
        this->process_ed_list(this->le32_get(HC_CONTROL_HEAD_ED));
        // the list has been fully serviced
        this->le32_set(HC_COMMAND_STATUS,
            this->le32_get(HC_COMMAND_STATUS) & ~(HC_CMD_CLF | HC_CMD_BLF));
    }

    // periodic list (interrupt EDs via the HCCA interrupt table)
    if ((ctrl & HC_CTRL_PLE) && this->hcca_addr) {
        this->frame++;
        uint8_t* hcca = this->map_guest(this->hcca_addr, 0x100);
        if (hcca) {
            for (int i = 0; i < 32; i++) {
                uint32_t ed = READ_DWORD_LE_A((uint32_t*)(hcca + 4 * i));
                if (ed)
                    this->process_ed_list(ed);
            }
            WRITE_DWORD_LE_A((uint32_t*)(hcca + 0x80), this->frame);
        }
    }
}

void KeyLargoUSB::process_ed_list(uint32_t ed_addr)
{
    uint32_t ed = ed_addr;
    for (int n = 0; ed && n < 64; n++) {
        uint8_t* p = this->map_guest(ed, 32);
        if (!p)
            break;

        uint32_t info   = READ_DWORD_LE_A((uint32_t*)(p + 0));
        uint32_t tailp  = READ_DWORD_LE_A((uint32_t*)(p + 4)) & ~1UL;
        uint32_t headp  = READ_DWORD_LE_A((uint32_t*)(p + 8));
        uint32_t nexted = READ_DWORD_LE_A((uint32_t*)(p + 12));
        uint32_t head   = headp & ~3UL;

        bool halted = (headp & 1) != 0;

        if (!(info & ED_SKIP) && !halted && head != tailp && tailp) {
            uint32_t toggle = (headp & 2) ? 1 : 0;
            uint32_t cur = head;
            for (int m = 0; cur && cur != tailp && m < 64; m++) {
                uint8_t* td = this->map_guest(cur, 16);
                if (!td)
                    break;
                this->process_td(ed, info, cur);
                toggle ^= 1;
                uint32_t td_next = READ_DWORD_LE_A((uint32_t*)(td + 8));
                if (td_next & 1)
                    cur = tailp;
                else
                    cur = td_next & ~1UL;
                if (!cur)
                    cur = tailp;
            }
            // queue empty -> head = tail with HALTED set
            WRITE_DWORD_LE_A((uint32_t*)(p + 8), tailp | 1 | (toggle << 1));
        }

        if (nexted & 1)
            break;
        ed = nexted & ~1UL;
    }
}

void KeyLargoUSB::process_td(uint32_t ed_addr, uint32_t ed_info, uint32_t td_addr)
{
    uint8_t* td = this->map_guest(td_addr, 16);
    if (!td)
        return;

    uint32_t tinfo  = READ_DWORD_LE_A((uint32_t*)(td + 0));
    uint32_t cbp    = READ_DWORD_LE_A((uint32_t*)(td + 4));
    uint32_t be     = READ_DWORD_LE_A((uint32_t*)(td + 12));
    uint32_t format = (ed_info & ED_FMT) >> 24;
    uint32_t dpid   = (tinfo & TD_DPID) >> 5;

    if (format == 7) {
        // control ED
        if (dpid == 0) {
            // SETUP: capture the 8-byte request
            if (cbp) {
                uint8_t* s = this->map_guest(cbp, 8);
                if (s) {
                    memcpy(this->setup_pkt, s, 8);
                    this->have_setup = true;
                    this->resp_ptr = nullptr;
                    this->resp_remaining = 0;
                    // SET_ADDRESS takes effect at the end of the setup stage
                    if (this->setup_pkt[1] == 0x05) {
                        this->device_address = this->setup_pkt[2];
                        this->have_setup = false;
                    }
                }
            }
        }
        else if (dpid == 2) {
            // IN: provide the response for the pending request
            if (this->have_setup && !this->resp_ptr) {
                uint32_t reqtype = this->setup_pkt[0];
                uint32_t wlen = this->setup_pkt[6] | (this->setup_pkt[7] << 8);
                if (reqtype & 0x80) { // device-to-host
                    this->resp_remaining = wlen;
                    this->resp_ptr = this->setup_response(this->resp_remaining);
                }
            }
            if (cbp && this->resp_remaining) {
                uint32_t cap = (be - cbp + 1);
                uint32_t n = std::min(this->resp_remaining, cap);
                uint8_t* dst = this->map_guest(cbp, n);
                if (dst) {
                    memcpy(dst, this->resp_ptr, n);
                    this->resp_ptr += n;
                    this->resp_remaining -= n;
                }
            }
        }
        else if (dpid == 1) {
            // OUT: status/data stage - nothing to send
            if (this->have_setup &&
                (this->setup_pkt[1] == 0x09 || this->setup_pkt[1] == 0x0A)) {
                // SET_CONFIGURATION / SET_INTERFACE acknowledged
                this->have_setup = false;
            }
        }
    }
    else {
        // interrupt/general ED: deliver an 8-byte keyboard report
        this->update_report();
        if (cbp) {
            uint8_t* dst = this->map_guest(cbp, 8);
            if (dst)
                memcpy(dst, this->kbd_report, 8);
        }
    }

    // complete the TD with condition code 0 and link into the done queue
    WRITE_DWORD_LE_A((uint32_t*)(td + 0), (tinfo & ~TD_CC) | 0);
    this->done_queue(td_addr);
}

void KeyLargoUSB::done_queue(uint32_t td_addr)
{
    uint8_t* td = this->map_guest(td_addr, 16);
    if (!td)
        return;

    uint32_t old_head = 0;
    if (this->hcca_addr) {
        uint8_t* h = this->map_guest(this->hcca_addr + 0x84, 4);
        if (h)
            old_head = READ_DWORD_LE_A((uint32_t*)h);
    }
    WRITE_DWORD_LE_A((uint32_t*)(td + 8), old_head);
    if (this->hcca_addr) {
        uint8_t* h = this->map_guest(this->hcca_addr + 0x84, 4);
        if (h)
            WRITE_DWORD_LE_A((uint32_t*)h, td_addr);
    }
    this->le32_set(HC_DONE_HEAD, td_addr);
    this->le32_set(HC_INTERRUPT_STATUS,
        this->le32_get(HC_INTERRUPT_STATUS) | INT_WDH);
    this->update_pci_irq();
}

const uint8_t* KeyLargoUSB::setup_response(uint32_t& resp_len)
{
    uint32_t reqtype = this->setup_pkt[0];
    uint32_t req     = this->setup_pkt[1];
    uint32_t wval    = this->setup_pkt[2] | (this->setup_pkt[3] << 8);

    uint32_t req_class = (reqtype >> 5) & 3;

    if (req_class == 0) { // standard
        if (req == 0x06) { // GET_DESCRIPTOR
            uint32_t dtype = (wval >> 8) & 0xFF;
            uint32_t dindex = wval & 0xFF;
            switch (dtype) {
            case 1: // device
                resp_len = std::min(resp_len, 18U);
                return kbd_device_desc;
            case 2: // configuration
                resp_len = std::min(resp_len, 34U);
                return kbd_config_desc;
            case 3: // string
                if (dindex == 0) {
                    resp_len = std::min(resp_len, 4U);
                    return kbd_string0_desc;
                }
                resp_len = 0;
                return nullptr;
            case 0x22: // HID report
                resp_len = std::min(resp_len, 61U);
                return kbd_report_desc;
            default:
                resp_len = 0;
                return nullptr;
            }
        }
        if (req == 0x08) { // GET_STATUS
            static const uint8_t status[2] = {0x00, 0x00};
            resp_len = std::min(resp_len, 2U);
            return status;
        }
    }
    else if (req_class == 1) { // class (HID)
        if (req == 0x01) { // GET_REPORT
            this->update_report();
            resp_len = std::min(resp_len, 8U);
            return this->kbd_report;
        }
        if (req == 0x03) { // GET_PROTOCOL
            static const uint8_t proto[1] = {0x01}; // boot protocol
            resp_len = std::min(resp_len, 1U);
            return proto;
        }
    }

    resp_len = 0;
    return nullptr;
}

// ---------------------------------------------------------------------------
// keyboard input
// ---------------------------------------------------------------------------

void KeyLargoUSB::event_handler(const KeyboardEvent& event)
{
    uint8_t usage = adbkey_to_hid[event.key & 0x7F];
    if (!usage)
        return;

    bool down = (event.flags & KEYBOARD_EVENT_DOWN) != 0;

    if (usage >= 0xE0 && usage <= 0xE7) {
        uint8_t bit = (uint8_t)(1 << (usage - 0xE0));
        if (down)
            this->kbd_modifiers |= bit;
        else
            this->kbd_modifiers &= ~bit;
    }
    else {
        int i;
        for (i = 0; i < 6; i++) {
            if (this->kbd_keys[i] == usage)
                break;
        }
        if (down) {
            if (i == 6) {
                for (i = 0; i < 6; i++) {
                    if (this->kbd_keys[i] == 0) {
                        this->kbd_keys[i] = usage;
                        break;
                    }
                }
            }
        }
        else if (i < 6) {
            this->kbd_keys[i] = 0;
        }
    }

    this->update_report();
}

void KeyLargoUSB::update_report()
{
    this->kbd_report[0] = this->kbd_modifiers;
    this->kbd_report[1] = 0;
    for (int i = 0; i < 6; i++)
        this->kbd_report[2 + i] = this->kbd_keys[i];
}

static const DeviceDescription KeyLargoUSB_Descriptor = {
    KeyLargoUSB::create, {}, {},
};

REGISTER_DEVICE(KeyLargoUSB, KeyLargoUSB_Descriptor);
