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

/** @file Construct the Q88 machine (Mac mini G4, PowerMac10,1/10,2). */

#include <cpu/ppc/ppcemu.h>
#include <devices/deviceregistry.h>
#include <devices/memctrl/intrepid.h>
#include <machines/machine.h>
#include <machines/machinebase.h>
#include <machines/machinefactory.h>
#include <machines/machineproperties.h>

// UniNorth AGP bus IRQ mapping (IntSrc IDs are placeholders until the
// UniNorth/KeyLargo interrupt controller is implemented).
static const std::vector<PciIrqMap> intrepid_agp_irq_map = {
    {nullptr        , DEV_FUN(0x0B,0)                  }, // UniNorth AGP
    {"pci_AGP_GPU"  , DEV_FUN(0x10,0), IntSrc::INT_UNKNOWN }, // Radeon 9200
};

// UniNorth main PCI bus IRQ mapping.
static const std::vector<PciIrqMap> intrepid_main_irq_map = {
    {nullptr        , DEV_FUN(0x0B,0)                  }, // UniNorth PCI
    {"pci_KeyLargo" , DEV_FUN(0x0D,0), IntSrc::INT_UNKNOWN }, // KeyLargo
    {"pci_USB"      , DEV_FUN(0x0E,0), IntSrc::INT_UNKNOWN }, // USB
    {"pci_USB2"     , DEV_FUN(0x0F,0), IntSrc::INT_UNKNOWN }, // USB
};

// UniNorth internal PCI bus IRQ mapping.
static const std::vector<PciIrqMap> intrepid_internal_irq_map = {
    {nullptr        , DEV_FUN(0x0B,0)                  }, // UniNorth Internal PCI
    {"pci_FireWire" , DEV_FUN(0x0E,0), IntSrc::INT_UNKNOWN }, // FireWire
    {"pci_GMAC"     , DEV_FUN(0x0F,0), IntSrc::INT_UNKNOWN }, // GMAC
};

class MachineQ88 : public Machine {
public:
    int initialize(const std::string &id);
};

int MachineQ88::initialize(const std::string &id) {
    LOG_F(INFO, "Building machine Q88 (Mac mini G4)...");

    // get pointer to the memory controller/primary PCI bridge object
    Intrepid* intrepid_obj = dynamic_cast<Intrepid*>(gMachineObj->get_comp_by_name("Intrepid"));
    intrepid_obj->set_irq_map(intrepid_main_irq_map);

    // allocate ROM region (New World: 1 MB at 0xFFF00000)
    if (!intrepid_obj->add_rom_region(0xFFF00000, 0x100000)) {
        LOG_F(ERROR, "Could not allocate ROM region!");
        return -1;
    }

    // allocate RAM
    intrepid_obj->setup_ram(GET_INT_PROP("rambank0_size"));

    // configure CPU clocks (100 MHz bus)
    uint64_t bus_freq      = 100000000ULL;
    uint64_t timebase_freq = bus_freq / 4;

    // initialize virtual CPU and request a G4 (MPC7447A) CPU
    ppc_cpu_init(intrepid_obj, PPC_VER::MPC7447A, false, timebase_freq);

    return 0;
}

static const PropMap q88_settings = {
    {"rambank0_size",
        new IntProperty(512, std::vector<uint32_t>({128, 256, 512, 1024}))},
    {"pci_AGP_GPU",
        new StrProperty("")},
    {"pci_KeyLargo",
        new StrProperty("")},
    {"pci_USB",
        new StrProperty("")},
    {"pci_USB2",
        new StrProperty("")},
    {"pci_FireWire",
        new StrProperty("")},
    {"pci_GMAC",
        new StrProperty("")},
};

static std::vector<std::string> q88_devices = {
    "Intrepid",
};

static const DeviceDescription MachineQ88_descriptor = {
    Machine::create<MachineQ88>, q88_devices, q88_settings
};

REGISTER_DEVICE(MachineQ88, MachineQ88_descriptor);

static const MachineDescription q88_descriptor = {
    .name = "Q88",
    .description = "Mac mini G4",
    .machine_root = "MachineQ88",
};

REGISTER_MACHINE(q88, q88_descriptor);
