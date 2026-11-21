# R1_robot_1 — STM32 Steering Chassis & R1 Firmware Collection

[简体中文](README.md) | **English**

> **Consolidation Note**: This repository consolidates three generations of university STM32 steering-wheel chassis firmware—extracting, cleaning, and organizing the legacy zip archives from `DJI_A_Steering` and `DM446RCT6_Steering` alongside the primary STM32H723 R1 firmware, while scrubbing all IDE clutter, build outputs, and accidental config files from Git history.

## Hardware & Firmware Evolution

| Directory | Legacy Repo | MCU / Board | Peripherals | Role & Architecture |
| --- | --- | --- | --- | --- |
| [`v1_dji_a_f427/`](v1_dji_a_f427/) | `DJI_A_Steering` | **STM32F427IIHx** (DJI Dev Board A) | CAN, USART, DMA | Early exploratory prototype (`BSP` + `Employ` + partial `Actuator` port) |
| [`v2_dm_f446/`](v2_dm_f446/) | `DM446RCT6_Steering` | **STM32F446RCT6** | CAN, TIM, USART, ARM DSP | First working **5-layer** steering chassis: DR16 remote + DM steer motors + ODrive wheel motors + BRT encoders |
| [`v3_r1_h723/`](v3_r1_h723/) | `R1_robot_1` (Primary) | **STM32H723** | **FDCAN**, USB CDC, TIM, USART, DMA, ARM DSP | **Primary production firmware**: full R1 lower-level controller with steering chassis, Gantry, Navigation, MiniPC USB CDC link, inter-board COMM, OPS-9 odometry, DR16 / Logitech F710 remote, and buzzer |

## Directory Structure

```text
R1_robot_1/
├── README.md                  # Chinese primary README
├── README_EN.md               # English README (this file)
├── .gitignore
├── v1_dji_a_f427/             # [v1] Early STM32F427 prototype
├── v2_dm_f446/                # [v2] STM32F446RCT6 5-layer steering chassis
│   └── User_file/             # 1_Middleware -> 2_Devier -> 3_Chariot -> 4_Interaction -> 5_Task
└── v3_r1_h723/                # [v3] STM32H723 full R1 firmware (recommended)
    ├── R1_robot_1.ioc
    ├── openocd.cfg
    ├── Core/ Drivers/ Middlewares/ USB_DEVICE/
    └── User_file/
        ├── 1_Middleware/      # drv_fdcan, drv_minipc, drv_uart, drv_tim, alg_pid, crc
        ├── 2_Device/          # Motor (DM/ODrive/Encoder), OPS, Remote (DR16/LogF710), USB, COMM, Buzzer
        ├── 3_Chariot/         # Chassis, Gantry, Navigation
        ├── 4_Interaction/     # robot state machine
        └── 5_Task/            # periodic scheduler
```

## Toolchain Requirements

All three subprojects use the **STM32CubeMX + CMake + Ninja + ARM GCC** workflow:

- `arm-none-eabi-gcc` / `arm-none-eabi-g++`
- `cmake` (>= 3.22) and `ninja`
- `openocd` (using `v3_r1_h723/openocd.cfg`) or ST-Link / DAP-Link

## Build & Flash (Primary `v3_r1_h723`)

```bash
cd v3_r1_h723

# Build Debug or Release via CMakePresets
cmake --preset Debug
cmake --build --preset Debug

# Flash with OpenOCD
openocd -f openocd.cfg -c "program build/Debug/R1_robot_1.elf verify reset exit"
```

## Next-Generation Upper-Computer Stack

Following this STM32 firmware series, the robots migrated to a **Linux ROS 2 + SOEM EtherCAT + LinkX-4C** architecture:
- [R1-Ethercat_4c](https://github.com/1PengYuyan1/R1-Ethercat_4c)
- [R2-Ethercat_4c](https://github.com/1PengYuyan1/R2-Ethercat_4c)
