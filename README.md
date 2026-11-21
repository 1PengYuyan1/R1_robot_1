# R1_robot_1 — STM32 舵轮底盘与 R1 下位机固件合集

**简体中文** | [English](README_EN.md)

> **整合说明**：本仓库已整合你大学阶段基于 STM32 开发的 **三代舵轮底盘固件演进代码**，原分散在 `DJI_A_Steering` 与 `DM446RCT6_Steering` 中的压缩包工程已解压、去除编译垃圾与 IDE 配置并按硬件代际归档到本仓库中；同时彻底清理了历史遗留的敏感配置文件。

## 三代硬件与固件演进一览

| 目录 | 对应原仓库 | 主控芯片 / 开发板 | 通信与外设 | 软件架构与定位 |
| --- | --- | --- | --- | --- |
| [`v1_dji_a_f427/`](v1_dji_a_f427/) | `DJI_A_Steering` | **STM32F427IIHx**（DJI A 型开发板） | CAN、USART、DMA | 早期探索原型（`BSP` + `Employ` + `Actuator` 移植尝试，未完全闭环） |
| [`v2_dm_f446/`](v2_dm_f446/) | `DM446RCT6_Steering` | **STM32F446RCT6** | CAN、TIM、USART、ARM DSP | 初代可运行**五层架构**舵轮底盘：DR16 遥控 + 达妙转向 + ODrive 轮向 + BRT 编码器 |
| [`v3_r1_h723/`](v3_r1_h723/) | `R1_robot_1`（主版本） | **STM32H723** | **FDCAN**、USB CDC、TIM、USART、DMA、ARM DSP | **最终主力版本**：完整 R1 下位机，支持舵轮底盘、龙门架（Gantry）、导航（Navigation）、MiniPC USB 通信、板间通信、OPS-9 定位、DR16 / Logitech F710 遥控与蜂鸣器 |

## 目录结构

```text
R1_robot_1/
├── README.md                  # 中文主文档（本文件）
├── README_EN.md               # English README
├── .gitignore
├── v1_dji_a_f427/             # [v1] DJI A 板 STM32F427 早期原型
│   ├── Steering_wheel_Try_2.ioc
│   ├── Core/                  # CubeMX 生成代码
│   ├── BSP/                   # CAN / USART 板级支持
│   ├── Employ/                # 舵轮底盘控制算法尝试
│   └── Actuator/              # 执行器驱动移植草稿
├── v2_dm_f446/                # [v2] STM32F446RCT6 五层架构初代舵轮底盘
│   ├── 111.ioc
│   ├── Core/ Drivers/ Middlewares/
│   └── User_file/
│       ├── 1_Middleware/      # drv_can / drv_uart / drv_tim / drv_math / alg_pid
│       ├── 2_Devier/          # dvc_dr16 / dvc_motor_dm / dvc_odrive / dvc_encoder
│       ├── 3_Chariot/         # crt_chassis（四舵轮解算与控制）
│       ├── 4_Interaction/     # robot（整车状态机）
│       └── 5_Task/            # 周期调度任务
└── v3_r1_h723/                # [v3] STM32H723 R1 完整下位机（主力推荐）
    ├── R1_robot_1.ioc
    ├── openocd.cfg
    ├── Core/ Drivers/ Middlewares/ USB_DEVICE/
    └── User_file/
        ├── 1_Middleware/      # drv_fdcan / drv_minipc / drv_uart / drv_tim / alg_pid / crc
        ├── 2_Device/          # Motor(DM/ODrive/Encoder), OPS, Remote(DR16/LogF710), USB, COMM, Buzzer
        ├── 3_Chariot/         # Chassis / Gantry / Navigation
        ├── 4_Interaction/     # robot
        └── 5_Task/            # task
```

## 工具链要求

三代工程均采用 **STM32CubeMX + CMake + Ninja + ARM GCC** 工作流：

- `arm-none-eabi-gcc` / `arm-none-eabi-g++`
- `cmake`（>= 3.22）与 `ninja`
- 烧录调试工具：`openocd`（`v3_r1_h723/openocd.cfg`）或 ST-Link / DAP-Link

## 编译方法（以主力版本 `v3_r1_h723` 为例）

```bash
cd v3_r1_h723

# 使用 CMakePresets 配置并编译 Debug / Release
cmake --preset Debug
cmake --build --preset Debug

# 或编译 Release
cmake --preset Release
cmake --build --preset Release
```

`v2_dm_f446` 与 `v1_dji_a_f427` 同样支持通过各自目录下的 `CMakePresets.json` 进行 CMake 编译。

### 烧录（OpenOCD）

```bash
cd v3_r1_h723
openocd -f openocd.cfg -c "program build/Debug/R1_robot_1.elf verify reset exit"
```

## 后续演进

在完成 STM32 下位机舵轮方案后，R1 与 R2 机器人进一步升级为 **Linux 上位机 ROS 2 + SOEM EtherCAT + LinkX-4C** 架构，详见：
- [R1-Ethercat_4c](https://github.com/1PengYuyan1/R1-Ethercat_4c)
- [R2-Ethercat_4c](https://github.com/1PengYuyan1/R2-Ethercat_4c)
