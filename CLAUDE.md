# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

不要随意修改任何文件，如果要修改代码或注释必须经过我同意，并且每次对话结束后告诉我有没有动代码或注释.
## Project

Control software for a wheel-legged balancing robot (RoboMaster "hero"), running as a Linux userspace program on an Orange Pi. It talks over USB to a DM-MC02 lower controller board, which owns the IMU, remote receiver, and motors. This program computes joint and wheel torques and sends them back. Comments and docs are mostly in Chinese.

## Build

CMake (≥3.22), C++20, single executable `wheel_leg_sys`. The default build type is Release (`-O3 -march=native`), so binaries are not portable across CPUs.

```bash
cmake -S . -B build -G Ninja        # build/ is already configured with Ninja
cmake --build build
sudo ./build/wheel_leg_sys          # needs USB device access (libusb) and /dev/input
```

Required libraries: Eigen3, qpOASES, CppAD (pkg-config `cppad`), libusb-1.0, CasADi (falls back to `/usr/local/lib/libcasadi.so.3.7` if no CMake config is found), pthread.

Sources are collected with `GLOB_RECURSE` over `src/`, so a new `.cpp`/`.c` file is compiled automatically. Re-run the cmake configure step if it isn't picked up. There is no test suite and no lint configuration.

## Architecture

**Entry and scheduling** ([src/Task/task_and_callback.cpp](src/Task/task_and_callback.cpp)): `main()` calls `Task_Init()` and then sleeps forever. All work happens on `class_Timer` threads ([src/Bsp/Timer.h](src/Bsp/Timer.h)) and the libusb receive callback, all sharing the global `Robot robot`:

- `mc02_callback_usb`: verifies the CRC16 and header `0x5A`, then copies IMU, RC, motor, and attitude data into `robot.mc02_SerialPort.Rx_Data` under `rx_mutex`. The IMU axes are sign-flipped here, and RC channels go through a median filter.
- `TIM_1ms_PeriodElapsedCallback` (1 ms): runs `INS.INS_Update` and `Chassis.Self_Observe()` (the chassis velocity Kalman filter).
- `TIM_2ms_PeriodElapsedCallback`: despite its name, it is started at **0.001 s**. It runs the FSMs, jump/stairs logic, `Chassis.feedback_update()`, `Robot::Chassis_Control()`, ground detection, and the Vofa telemetry, then calls `mc02_send_data()` (header `0xA5` + CRC16).
- `TIM_4ms_PeriodElapsedCallback` (4 ms): `Chassis.MPC_Robot_Calc()`, the 10-state MPC. Its period must match the `dt` passed to `Robot_MPC::Init(0.004)`. It takes its own lock internally, only around the data copy.

Changing a timer period means checking every `dt` that assumes it. [MPC_LQR_对话总结.md](MPC_LQR_对话总结.md) §4 documents existing mismatches, such as `Chassis::dt = 0.002` while the loop runs at 1 ms.

**Layers:**

- `src/Bsp/`: hardware access. `USBManager` (libusb, VID/PID `0x0483:0x5740`), `SerialPort` (the legacy serial path is commented out), and the high-precision timer.
- `src/Devices/`: `mc_02_serial` and the packed Rx/Tx structs that make up the wire protocol with the MC02 ([src/Devices/dm_mc02.h](src/Devices/dm_mc02.h)). Also `class_INS`, `class_GamePad` (evdev), `Class_Vofa_TCP` (debug plotting over TCP), and `user_lib`.
- `src/Interaction/`: `Robot` ([robot.h](src/Interaction/robot.h)) maps remote-control input to targets and chassis modes (zero force / follow gimbal yaw / check-in) and holds the chassis, jump, and stairs FSMs. `class_Chassis` ([Chassis.h](src/Interaction/Chassis.h)) holds both `class_vmc_leg` legs and all the controllers. The remote type is chosen at compile time with the `FUS_I6X` / `GAMEPAD` / `VT03` macros in `robot.h`.
- `src/Algorithm/`: controllers and estimators. Files such as `lqr_k_calc.c`, `leg_*.c`, `leso_*_calc.c`, and `system_dynamic/*.c` are MATLAB Coder output. Regenerate them from MATLAB instead of editing them by hand.

**Control structure in `class_Chassis`:**

- **LQR** (`LQR_Calc`): uses a 10-dim error state `[s, ds, phi(yaw), dphi, θ_ll, dθ_ll, θ_lr, dθ_lr, θ_b, dθ_b]` and outputs `T[0..3]` = left/right wheel torque and left/right hip torque. Gains come from a leg-length polynomial fit (`lqr_k_calc` / the `P[40][6]` coefficient table, which holds fitted gain coefficients and is not a Riccati solution).
- **10-state linear MPC** (`Robot_MPC`, [src/Algorithm/MPC/Robot_MPC.h](src/Algorithm/MPC/Robot_MPC.h)): uses the same error state and the same 4 outputs as the LQR, so it substitutes directly for the LQR torques. It is a condensed QP solved with qpOASES. The model is rebuilt (A/B, Riccati terminal cost, condensing) when leg length changes by more than 2 mm or every 100 ms. All constraint limits live in the `MPC_CFG` namespace at the top of that header.
- **MPC/LQR arbitration**: MPC output is used only when `mpc_enable && MPC_Output_Fresh()` (solved within `mpc_timeout` = 8 ms) and the robot is not spinning and not airborne (`ABOVE_GROUND`). Otherwise the LQR result is used. Setting `mpc_enable = false` reverts fully to the LQR. Status flags (`Get_mpc_ok/used/rebuild_ok/riccati_ok`) are streamed over Vofa.
- **Body MPC** (`MPC_Body_Calc`, `MPC_Controller MPC_body(4, 2, 0, N)`): a 4-state roll/height model that outputs virtual leg forces F_left/F_right. It is a separate channel from the wheel/hip torques. `VMC_Calc_2()` combines leg force and hip torque into joint torques.
- `NMPC*.cpp` (CppAD SQP and CasADi variants) are compiled in but are alternatives to the linear MPC; see §8 of the summary doc. Per the summary doc, `QuaternionEKF` / `Tactical_Fusion` are no longer in the active attitude path, which uses the attitude the MC02 sends up.

**Gotchas:**

- `Chassis.h` has `#define N 15`. `Robot_MPC.h` must be included before that define and deliberately never uses the identifier `N`. Keep it that way in any code included after `Chassis.h`.
- Sign conventions differ between the left and right sides. Right-side joint and wheel torques are negated in `mc02_send_data()`, and joint angles are offset or negated when read. Check these mappings before changing any motor-facing value.
- Some files are GBK-encoded (for example `src/Bsp/Timer.h` and `src/Algorithm/CRC/README.md`), while others are UTF-8. Don't re-encode files you aren't otherwise changing.

## Reference

[MPC_LQR_对话总结.md](MPC_LQR_对话总结.md) is a design discussion of the MPC, LQR, Kalman filter, slip strategy, dt issues, and solver choices. Parts of it describe the older 4-state-only MPC setup that existed before `Robot_MPC` was added. Verify claims in it against the code.
