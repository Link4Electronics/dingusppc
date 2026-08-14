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

/** @file Construct the Cypher machine (Power Mac G5 Quad/Dual Core,
 *         PowerMac11,2). DRAFT: the G5 chipset (U4 memory controller, K2
 *         mac-io) is not emulated yet, so this machine cannot boot.
 */

#include <cpu/ppc/ppcemu.h>
#include <devices/deviceregistry.h>
#include <machines/machine.h>
#include <machines/machinebase.h>
#include <machines/machinefactory.h>
#include <machines/machineproperties.h>

class MachineCypher : public Machine {
public:
    int initialize(const std::string &id);
};

int MachineCypher::initialize(const std::string &id) {
    LOG_F(ERROR, "Building machine Cypher (Power Mac G5 Quad/Dual Core)...");

    // DRAFT: this machine is registered so the genuine PowerMac11,2 (Cypher)
    // boot ROM identifies correctly, but the logic board is not emulated. It
    // uses the U4 memory controller and the K2 mac-io chip; neither exists
    // yet, so there is nothing to wire up here. Once U4/K2 land, this should:
    //   - get the U4 MemCtrl and register its PCI IRQ maps
    //   - register the K2 mac-io at its slot on the U4's PCI bus
    //   - add_rom_region(0xFFF00000, 0x100000)  (1 MB New World ROM)
    //   - setup_ram() with the G5's DDR2 DIMMs
    //   - ppc_cpu_init() with PPC_VER::MPC970MP and real G5 clocks
    //     (2x2.5 GHz quad, 1.25 GHz bus, timebase derived from core clock)
    return -1;
}

static const PropMap cypher_settings = {
    {"rambank0_size",
        new IntProperty(512, std::vector<uint32_t>({256, 512, 1024, 2048, 4096}))},
};

// NOTE: the "U4"/"K2" devices do not exist yet, so they must NOT be listed
// here (DeviceRegistry::create would invoke an empty create func and crash).
static std::vector<std::string> cypher_devices = {
};

static const DeviceDescription MachineCypher_descriptor = {
    Machine::create<MachineCypher>, cypher_devices, cypher_settings
};

REGISTER_DEVICE(MachineCypher, MachineCypher_descriptor);

static const MachineDescription cypher_descriptor = {
    .name = "Cypher",
    .description = "Power Mac G5 Quad Core",
    .machine_root = "MachineCypher",
};

REGISTER_MACHINE(cypher, cypher_descriptor);
