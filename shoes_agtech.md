# PRODUCT FIRMWARE - ARDUPILOT DERIVATIVE SYSTEM (v2.0)

This repository contains the modified ArduPilot source code utilized in our commercial autonomous vehicle platforms. This software is distributed under the **GNU General Public License v3.0 (GPLv3)**. In strict compliance with Section 6 and Section 7 of the GPLv3, both the modified source code and the technical installation pathways are openly provided to downstream users.

**Current Production Version:** `v2.0`  
**Target Vehicle Codebase:** `ArduRover`

---

## SECTION A: SOURCE CODE DISCLOSURE

### 1. License & Derivative Works

This system integrates derivative works of the ArduPilot project. Under the strong copyleft requirements of GPLv3, all modifications, custom libraries, and hardware-specific drivers developed within the ArduPilot ecosystem are open-source.

- **Upstream Repository**: [https://github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)

- **Our Core Modifications (Version 2.0)**:
    - **Modified Native Flight Modes (`mode.cpp`, `mode_acro.cpp`)**: Overrode and customized the standard behavior of `ModeAuto` and `ModeAcro` within the ArduRover codebase to implement specialized steering tracking and non-linear hydrodynamic response logic.
    - **Custom Advanced Parameters**: Embedded new user-configurable parameters (`AP_Param`) into the system architecture, enabling precise runtime tuning of the newly injected control variables.
    - **Custom Proprietary Library Integration**: Introduced a dedicated custom library subsystem integrated natively into the ArduPilot compilation architecture (`libraries/`) to encapsulate high-level algorithmic processes.
    - **Sensor Data Acquisition & Persistence**: Expanded internal global structures and data allocation layers with dedicated variables to interface, process raw values from specialized external sensors, and permanently log telemetry payloads via the onboard `DataFlash` logging architecture.

### 2. How to Access the Source Code

Downstream users and developers can retrieve the exact state of the production firmware through the following channels:

- **Git Repository**: `git clone https://github.com/your-organization/ardupilot-production.git`
- **Release Branch / Tag**: Access the `v2.0-release` tag for verified commercial binaries and corresponding source code.
- **Physical Distribution**: In compliance with GPLv3, a physical copy of this source code can be requested via our technical support department for a period of three (3) years from the date of product purchase.

---

## SECTION B: INSTALLATION & AUTHENTICATION INFORMATION

In compliance with the Anti-Tivoization provisions of the GPLv3, the hardware architecture remains unlocked. Users retain the legal and technical right to execute modified versions of the software on the consumer-facing hardware platform.

### 1. Hardware Specifications & Bootloader Target

- **Target Flight Controller Hardware**: Pixhawk Series Architecture (e.g., STM32H7 / STM32F7 MCU).
- **Bootloader**: Standard ArduPilot Bootloader via DFU/USB interface.
- **Cryptographic Status**: The hardware microcontroller does not enforce proprietary cryptographic signature verification that blocks user-compiled binaries. No hardware encryption keys are required to execute custom code.

### 2. Compilation Environment Setup

To rebuild the production binary from source, execute the following toolchain setup (validated on Ubuntu 22.04 LTS / 24.04 LTS):

```bash
# Clone the repository with submodules
git clone --recursive [https://github.com/your-organization/ardupilot-production.git](https://github.com/your-organization/ardupilot-production.git)
cd ardupilot-production

# Checkout the specific commercial version 2.0 tag
git checkout tags/v2.0-release

# Install the required ARM GCC compiler toolchain and dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile

# Configure the build target for the specific hardware board
./waf configure --board=your_target_board_name

# Compile the firmware binary specifically for Rover architecture
./waf rover
```

### 3. Firmware Flashing Procedure

Once the binary (.apj or .bin file) is generated, users can flash the modified firmware onto the vehicle via two approved methods:

Method I: Via Ground Control Station (QGroundControl / Mission Planner)
Connect the flight controller board to the host PC using a standard USB-C cable.

Open Mission Planner or QGroundControl.

Navigate to Setup -> Install Firmware -> Select Load custom firmware.

Path to the compiled binary (.apj) and initiate the upload process. The onboard bootloader will execute the flashing sequence automatically.

Method II: Via Command Line Interface (CLI Toolchain)
Alternatively, use the integrated waf deployment mechanism directly over the USB interface:

```bash
# Flash the compiled rover binary via direct USB connection
./waf rover --upload
```

LEGAL DISCLAIMER & COPYRIGHT NOTICE
Notice: This software is provided by the copyright holders and contributors "as is" and any express or implied warranties, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose are disclaimed. In no event shall the authors or copyright holders be liable for any direct, indirect, incidental, special, exemplary, or consequential damages arising in any way out of the use of this software.

All original copyright banners within the source headers (C) ArduPilot Dev Team remain untouched and fully preserved.
