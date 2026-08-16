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

/** @file KeyLargo (Intrepid) USB OHCI controller definitions. */

#ifndef KEYLARGO_USB_H
#define KEYLARGO_USB_H

#include <core/hostevents.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>

#include <memory>

constexpr auto KEYLARGO_USB_DEV_ID = 0x003F; // KeyLargo USB (OHCI)

/** KeyLargo/Intrepid OHCI USB host controller.

    Emulates just enough of a USB 1.1 OHCI controller to enumerate a
    single low-speed HID boot keyboard attached to root hub port 1:

      - Full OHCI register file (LE, accessed via lwbrx/stwbrx).
      - Root hub with two ports; port 1 has a keyboard permanently
        attached (CCS set, low-speed, 8-byte max packet).
      - Process-on-access ED/TD list execution from guest RAM (control
        list and periodic interrupt list) with done-queue / WDH handling.
      - Control-transfer responder for a boot keyboard (device/config/HID
        descriptors, SET_ADDRESS, SET_CONFIGURATION, GET_STATUS, etc.).
      - 8-byte boot-keyboard interrupt-IN reports fed from real keypresses
        via the EventManager keyboard handler. */
class KeyLargoUSB : public PCIDevice {
public:
    KeyLargoUSB();
    ~KeyLargoUSB() = default;

    static std::unique_ptr<HWComponent> create() {
        return std::unique_ptr<KeyLargoUSB>(new KeyLargoUSB());
    }

    int device_postinit() override;

    // MMIODevice methods
    uint32_t read(uint32_t rgn_start, uint32_t offset, int size) override;
    void write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size) override;

private:
    void notify_bar_change(int bar_num);
    void event_handler(const KeyboardEvent& event);

    uint32_t reg_read(uint32_t offset, int size);
    void reg_write(uint32_t offset, uint32_t value, int size);

    uint8_t* map_guest(uint32_t addr, uint32_t size);
    uint32_t le32_get(uint32_t off);
    void le32_set(uint32_t off, uint32_t val);

    void hc_reset();
    void update_pci_irq();
    void process_hc();
    void process_ed_list(uint32_t ed_addr);
    void process_td(uint32_t ed_addr, uint32_t ed_info, uint32_t td_addr);
    void done_queue(uint32_t td_addr);
    const uint8_t* setup_response(uint32_t& resp_len);
    void update_report();

    // OHCI registers (stored as little-endian bytes)
    uint8_t regs[0x100];
    uint32_t base_addr = 0;
    uint32_t hcca_addr = 0;
    uint32_t frame = 0;
    bool irq_asserted = false;

    // root hub port state (port 0 = keyboard, port 1 = empty)
    bool port_present[2] = {true, false};
    bool port_enabled[2] = {false, false};
    bool port_suspended[2] = {false, false};
    uint32_t port_change[2] = {0, 0};

    // current control transfer state
    uint8_t setup_pkt[8];
    bool have_setup = false;
    uint32_t device_address = 0;
    const uint8_t* resp_ptr = nullptr;
    uint32_t resp_remaining = 0;

    // HID keyboard state
    uint8_t kbd_modifiers = 0;
    uint8_t kbd_keys[6] = {0, 0, 0, 0, 0, 0};
    uint8_t kbd_report[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

#endif // KEYLARGO_USB_H
