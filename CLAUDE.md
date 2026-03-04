# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble workspace for the **Mulinex** omnidirectional robot's on-board SBC (Raspberry Pi 4 + MJBots Pi3Hat). This repo contains the ros2_control hardware interface, controllers, and custom messages that run directly on the robot, communicating with Moteus motor drivers via CAN-FD.

This workspace is separate from the parent `mulinex_ws` Docker workspace (which targets the Intel NUC for development). This code runs natively on the Raspberry Pi.

## Build Commands

```bash
# Source ROS 2
source /opt/ros/$ROS_DISTRO$/setup.bash

# Build entire workspace
colcon build --symlink-install

# Source the workspace overlay after building
source install/local_setup.bash
```

**Build order matters:** `pi3hat_moteus_int_msgs` must be built first (all other packages depend on it).

## Launch

```bash
ros2 launch pi3hat_hw_interface moteu_pi3hat_interface.launch.py \
  urdf_file:=Omnicar.urdf.xacro \
  conf_file:=Omnicar.yaml
```

Default launch args are `JumpingLeg2d.urdf.xacro` / `jump_leg.yaml`. URDF files live in `pi3hat_hw_interface/urdf/`, config YAMLs in `pi3hat_hw_interface/config/`.

The launch file starts `ros2_control_node` (controller_manager) which loads the hardware plugin and spawns controllers defined in the YAML.

## Architecture

### ros2_control Pipeline

```
Commands (topics)  →  Controllers  →  Hardware Interface  →  CAN-FD  →  Moteus Motors
                                      ↕ (read/write cycle)
Feedback (topics)  ←  Broadcasters ←  Hardware Interface  ←  CAN-FD  ←  Moteus Motors
```

### Package Dependency Graph

```
pi3hat_moteus_int_msgs          ← Custom .msg definitions (base dependency)
  ↑
moteus_pi3hat                   ← Low-level C++ library wrapping Pi3Hat CAN-FD (header-only, needs bcm_host)
  ↑
pi3hat_hw_interface             ← ros2_control SystemInterface plugin
  ↑
pi3hat_base_controller          ← Joint controller + state/distributor broadcasters
omni_vel_controller             ← Omnidirectional wheel velocity controller
```

### Packages

**`pi3hat_moteus_int_msgs`** — Custom messages: `JointsCommand`, `JointsStates`, `OmniMulinexCommand`, `PacketPass`, `DistributorsState`.

**`pi3hat_hw_interface`** — The `MoteusPi3Hat_Interface` SystemInterface plugin. Key classes:
- `moteus_pi3hat_interface` — Implements `on_init/on_configure/on_activate/read/write`. Parses URDF for motor and distributor configuration. Manages IMU if `request_attitude=1`.
- `actuator_manager` — Per-motor wrapper around mjbots `Controller`. Handles PID configuration via CAN diagnostic commands, transmission ratio conversion, secondary encoder support, and CAN-FD frame encoding/decoding.
- `elem_info_parsers` — URDF parameter parsing for Pi3Hat config, actuators, and power distributors.

**`pi3hat_base_controller`** — Four ros2_control plugins:
- `Pi3Hat_Joint_Group_Controller` — Subscribes to `~/command` (JointsCommand), maps to hardware command interfaces. Supports `kp_scale`/`kd_scale` gain scaling.
- `Pi3Hat_State_Broadcaster` — Publishes `state_broadcaster/joints_state` (JointsStates) and `state_broadcaster/performance_indexes` (PacketPass).
- `Distributor_State_Broadcaster` — Publishes `distributor_state_broadcaster/distributors_state` (DistributorsState).
- `Debug_Broadcaster` — Debug echoing of joint info.

**`omni_vel_controller`** — `Omni_Vel_Controller` plugin. Subscribes to `~/input_odom` (OmniMulinexCommand), computes 4-wheel omnidirectional inverse kinematics, publishes JointsCommand. Key params: `mecanum_angle`, `wheel_rad`, `feet_type`, `homing_duration`.

### Hardware Configuration (URDF)

Robot configuration is defined in `.urdf.xacro` files. Each joint element configures a Moteus motor:
- `id`/`bus` — CAN ID and Pi3Hat bus number (1-5)
- `KP`/`KD`/`KI` — Low-level PID gains (configured at startup via diagnostic commands; **must not exceed 16-bit representability**)
- `actuator_trasmission` — Motor-to-joint gear ratio (all motor quantities are converted through this)
- `second_encoder_trasmission` — Joint-to-secondary-encoder ratio (0 disables secondary encoder)
- `position_offset`, `max_pos_limit`/`min_pos_limit` — Joint-space offset and saturation (0 disables limits)
- `type` — `motor` or `power_dist`

Controller manager YAML (e.g. `Omnicar.yaml`) sets `update_rate` (typically 500 Hz) and lists which controllers to load with their parameters.

### Robot Configurations

- **Omnicar** (`Omnicar.urdf.xacro` + `Omnicar.yaml`) — 4 omniwheels + 2 power distributors. Motors on buses 1-4, distributors on bus 5.
- **JumpingLeg2d** — 2-DOF leg (HIP + KNEE).
- **SingleJoint/SingleJointSE** — Single motor test setups.

### Hardware State/Command Interfaces

Beyond standard `position`/`velocity`/`effort`, the hardware interface exposes custom interfaces:
- **Commands:** `kp_scale_value`, `kd_scale_value` (runtime gain scaling)
- **States:** `temperature`, `q_current`, `d_current`, `motor_temperature`, `abs_position`, `power`, `validity_loss`, `package_loss`, `cycle_duration`

## Raspberry Pi Prerequisites

- Ubuntu 22.04 with RT kernel on Raspberry Pi 4
- `bcm_host` library: `sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0`
- `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-xacro`
- Python moteus library: `pip3 install moteus==0.3.67 moteus_pi3hat`

## Key Details

- **ROS_DOMAIN_ID**: Must match between robot and operator PC (parent workspace uses 10)
- **RMW_IMPLEMENTATION**: Use `rmw_cyclonedds_cpp`.
- **CPU affinity**: Hardware interface pins to CPU 1 for real-time determinism
- **CAN-FD**: 64-byte frames with configurable bit-resolution per field
- Pi3Hat r4.4 or newer required
