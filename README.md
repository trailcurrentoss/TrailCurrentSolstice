# TrailCurrent Solstice

Solar gateway module that reads data from a Victron MPPT (Maximum Power Point Tracker) solar charge controller via serial and relays the readings over a CAN bus interface. Part of the [TrailCurrent](https://trailcurrent.com) open-source vehicle platform.

## Hardware Overview

- **Board:** [Waveshare ESP32-S3-RS485-CAN](https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN)
- **Microcontroller:** ESP32-S3
- **Function:** Serial-to-CAN bus bridge for Victron MPPT solar charge controller data
- **Key Features:**
  - Victron MPPT VE.Direct serial protocol parsing
  - CAN bus output at 500 kbps
  - Real-time solar panel monitoring
  - Load control via VE.Direct HEX protocol over CAN
  - Hierarchical PCB schematic design

## Hardware Requirements

### Components

- **Board:** Waveshare ESP32-S3-RS485-CAN (off-the-shelf)
- **CAN Bus:** 500 kbps (TX: GPIO 15, RX: GPIO 16)
- **Serial Input:** Victron MPPT VE.Direct connection (TX: GPIO 2, RX: GPIO 1 at 19200 baud)

### KiCAD Library Dependencies

This project uses the consolidated [TrailCurrentKiCADLibraries](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries).

**Setup:**

```bash
# Clone the library
git clone git@github.com:trailcurrentoss/TrailCurrentKiCADLibraries.git

# Set environment variables (add to ~/.bashrc or ~/.zshrc)
export TRAILCURRENT_SYMBOL_DIR="/path/to/TrailCurrentKiCADLibraries/symbols"
export TRAILCURRENT_FOOTPRINT_DIR="/path/to/TrailCurrentKiCADLibraries/footprints"
export TRAILCURRENT_3DMODEL_DIR="/path/to/TrailCurrentKiCADLibraries/3d_models"
```

See [KICAD_ENVIRONMENT_SETUP.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/KICAD_ENVIRONMENT_SETUP.md) in the library repository for detailed setup instructions.

## Opening the Project

1. **Set up environment variables** (see Library Dependencies above)
2. **Open KiCAD:**
   ```bash
   kicad EDA/solstice.kicad_pro
   ```
3. **Verify libraries load** - All symbol and footprint libraries should resolve without errors
4. **View 3D models** - Open PCB and press `Alt+3` to view the 3D visualization

### Schematic Sheets

The design uses a hierarchical schematic with dedicated sheets:
- **Root** - Top-level connections
- **Power** - Power distribution and regulation
- **CAN** - CAN bus transceiver interface
- **MCU** - ESP32 microcontroller and support circuits
- **Connectivity** - Serial interface to Victron MPPT

## Firmware

ESP-IDF 5.5.2 project in the `main/` directory.

**Setup:**
```bash
# Source ESP-IDF environment
source ~/esp/v5.5.2/esp-idf/export.sh

# Set target
idf.py set-target esp32s3

# Build firmware
idf.py build

# Flash to board
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor
```

### Victron MPPT Parameters

The firmware parses the following VE.Direct protocol fields:

| Parameter | Description |
|-----------|-------------|
| V | Battery voltage |
| VPV | Panel voltage |
| PPV | Panel power (watts) |
| I | Panel current |
| CS | Charge state |
| ERR | Error code |
| H19-H23 | Historical yield and power data |

### CAN Bus Protocol

The gateway transmits two messages at 500 kbps with a 33ms update cycle:

**Message 0x2C** (7 bytes) - Solar panel basics:

| Byte | Description |
|------|-------------|
| 0-1 | Panel voltage |
| 2-3 | Solar watts |
| 4-5 | Battery voltage |
| 6 | Solar status |

**Message 0x2D** (3 bytes) - Solar current:

| Byte | Description |
|------|-------------|
| 0 | Current sign |
| 1-2 | Current magnitude |

**Message 0x2E** (1 byte) - Load control (received):

| Byte | Description |
|------|-------------|
| 0 | Load state (0x00=OFF, 0x01=ON, 0x04=Default) |

When a 0x2E message is received over CAN, the gateway sends a VE.Direct HEX SET command to register 0xEDAB on the MPPT to control the load output.

## Manufacturing

- **PCB Files:** Ready for fabrication via standard PCB services (JLCPCB, OSH Park, etc.)
- **BOM Generation:** Export BOM from KiCAD schematic (Tools > Generate BOM)
- **JLCPCB Assembly:** See [BOM_ASSEMBLY_WORKFLOW.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/BOM_ASSEMBLY_WORKFLOW.md) for detailed assembly workflow

## Project Structure

```
├── main/                         # ESP-IDF firmware source
│   ├── main.c                    # Victron MPPT parser, CAN TX/RX, load control
│   ├── can_helper.h              # CAN bus driver header
│   ├── can_helper.c              # CAN bus driver implementation
│   └── CMakeLists.txt            # Component build config
├── EDA/                          # KiCAD hardware design files
│   ├── solstice.kicad_pro
│   ├── solstice.kicad_sch        # Root schematic
│   ├── can.kicad_sch             # CAN subsystem
│   ├── connectivity.kicad_sch    # Serial interface
│   ├── mcu.kicad_sch             # MCU subsystem
│   ├── power.kicad_sch           # Power subsystem
│   └── solstice.kicad_pcb        # PCB layout
├── CAD/                          # Enclosure design
│   └── trailcurrent-solstice-housing.FCStd
└── CMakeLists.txt                # ESP-IDF project config
```

## License

MIT License - See LICENSE file for details.

This is open source hardware. You are free to use, modify, and distribute these designs under the terms of the MIT license.

## Contributing

Improvements and contributions are welcome! Please submit issues or pull requests.

## Support

For questions about:
- **KiCAD setup:** See [KICAD_ENVIRONMENT_SETUP.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/KICAD_ENVIRONMENT_SETUP.md)
- **Assembly workflow:** See [BOM_ASSEMBLY_WORKFLOW.md](https://github.com/trailcurrentoss/TrailCurrentKiCADLibraries/blob/main/BOM_ASSEMBLY_WORKFLOW.md)
