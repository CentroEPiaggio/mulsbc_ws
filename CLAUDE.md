# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble workspace for the **Mulinex** omnidirectional robot's on-board SBC (Raspberry Pi 4 + MJBots Pi3Hat). Contains the `ros2_control` hardware interface, controllers, and custom messages that communicate with Moteus motor drivers via CAN-FD.

This workspace is separate from the parent `mulinex_ws` Docker workspace (which targets the Intel NUC). This code runs natively on the Raspberry Pi.

## Docker (ARM64 testing on x86 host)

Use Docker to build, run, and test the workspace on an x86 machine. All build/test commands must run inside it. Workspace is bind-mounted at `/ws/` inside the container.

```bash
bash docker/build.bash                              # build image (first time / Dockerfile changes)
docker compose -f docker/docker-compose.yaml up -d  # start container (could be already running)
docker exec mulsbc-arm64-dev bash                   # enter container
docker compose -f docker/docker-compose.yaml down   # stop container
```

## Build Commands

```bash
colcon build --symlink-install
source install/local_setup.bash
```

**Build order matters:** `pi3hat_moteus_int_msgs` must build first (all other packages depend on it).

## Launch

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=omnicar.xacro \
  conf_file:=omnicar.yaml
```

Default args are `omnicar.xacro` / `omnicar.yaml`. URDF files in `pi3hat_hw_interface/urdf/`, config YAMLs in `pi3hat_hw_interface/config/`.

## Package Dependency Graph

```
pi3hat_moteus_int_msgs          <- Custom .msg definitions (base dependency)
  ^
moteus_pi3hat                   <- Low-level C++ library wrapping Pi3Hat CAN-FD (header-only, needs bcm_host)
  ^
pi3hat_hw_interface             <- ros2_control SystemInterface plugin
  ^
omni_controller                 <- Unified omnidirectional controller (wheels IK + leg commands + state broadcasting)
pi3hat_base_controller          <- Standalone joint controller + state/distributor broadcasters
```

## Key Source Files

- **Hardware interface:** `src/pi3hat/pi3hat_hw_interface/src/moteus_pi3hat_interface.cpp` — `on_init/on_configure/on_activate/read/write` cycle
- **Actuator manager:** `src/pi3hat/pi3hat_hw_interface/src/actuator_manager.cpp` — per-motor CAN-FD wrapper, PID config, transmission ratio conversion
- **OmniController:** `src/omni_controller/src/omni_controller.cpp` — unified controller (wheel IK + legs + state broadcasting)
- **Wheel IK:** `src/omni_controller/include/omni_controller/wheel_ik.hpp` — omnidirectional inverse kinematics
- **Messages:** `src/pi3hat/pi3hat_moteus_int_msgs/msg/` — `JointsCommand`, `JointsStates`, `OmniMulinexCommand`, `PacketPass`, `DistributorsState`
- **URDF configs:** `src/pi3hat/pi3hat_hw_interface/urdf/` — `.xacro` files defining motor/distributor configuration
- **Controller YAMLs:** `src/pi3hat/pi3hat_hw_interface/config/` — controller manager configuration

## Hardware Configuration (URDF)

Each joint in `.xacro` configures a Moteus motor:
- `id`/`bus` — CAN ID and Pi3Hat bus (1-5)
- `KP`/`KD`/`KI` — Low-level PID gains (**must not exceed 16-bit representability**)
- `actuator_trasmission` — Motor-to-joint gear ratio
- `second_encoder_trasmission` — Joint-to-secondary-encoder ratio (0 disables)
- `position_offset`, `max_pos_limit`/`min_pos_limit` — Joint-space offset and limits (0 disables)
- `type` — `motor` or `power_dist`

## Hardware State/Command Interfaces

Beyond standard `position`/`velocity`/`effort`:
- **Commands:** `kp_scale_value`, `kd_scale_value` (runtime gain scaling)
- **States:** `temperature`, `q_current`, `d_current`, `motor_temperature`, `abs_position`, `power`, `validity_loss`, `package_loss`, `cycle_duration`

## Robot Configurations

- **Omnicar** (`omnicar.xacro` + `omnicar.yaml`) — 4 mecanum wheels + 2 power distributors. Motors on buses 1-4, distributors on bus 5.
- **Omniquad** (`omniquad12.xacro` + `omniquad12.yaml`) — 4 mecanum wheels + 8 leg joints (LF/LH/RF/RH HFE+KFE) + 2 power distributors.
