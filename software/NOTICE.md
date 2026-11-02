# Third-party sources

This firmware bundles a minimal CMSIS subset copied from the user-provided
reference project:

`D:\CODE\FOC_WHEEL\foc-wheel-legged-robot-master\stm32-foc\software`

- `Drivers/CMSIS/Include/`: ARM CMSIS Cortex-M3 and compiler support headers,
  copyright ARM Limited. Each header retains its original Apache-2.0 notice.
- `Drivers/CMSIS/Device/ST/STM32F1xx/Include/`,
  `Core/Src/system_stm32f1xx.c`, and `Core/Startup/`: STMicroelectronics STM32F1
  device headers, system initialization, and startup vectors. Each file retains
  its original BSD-3-Clause notice.

No HAL, DSP, neural-network, RTOS, or CMSIS test libraries are included.

Local adaptations to the ST templates: initial `SystemCoreClock` matches the
8 MHz HSI; the GCC reset handler supplies the newlib initialization hook and
traps if main returns; the ARMCC startup
reserves 2 KiB stack and zero heap. Board initialization selects the operating
clock. The linker targets the rated STM32F103C8 memory: 64 KiB flash and 20 KiB
SRAM. The last four 1 KiB flash pages are reserved: two for electrical calibration and two for CAN node ID journals. Ordinary PI parameters remain RAM-only.

The layout uses the familiar STM32 `Core/`, `Drivers/`, and `MDK-ARM/`
directories, but this is a CMSIS register-level project, not a CubeMX-generated
HAL project. There is no `.ioc` or RTOS. Board/motor/CAN constants live in
`Config/`; register drivers, application state and communications are maintained
in `BSP/`, `App/`, and `Communication/` respectively. `Core/Linker/` holds the
GCC/ARMCC memory layouts. `project.psd1` is the shared GCC, Keil and host-test
layout manifest; relocating files does not change their copyright or license.

Build from the repository root with `build.ps1`; `software/build.ps1` remains
the compatible firmware entry point. Output directories remain under
`software/build/` and `software/MDK-ARM/`. See `docs/PROJECT_STRUCTURE.md` at the
repository root for the full module map.

The complete license texts are bundled at `Drivers/CMSIS/LICENSE-APACHE-2.0.txt`
and `Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE-BSD-3-Clause.txt`.
