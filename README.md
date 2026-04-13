# Mulinex SBC ROS 2 Workspace

ROS 2 Humble workspace for the **Mulinex** omnidirectional robot's on-board SBC (Raspberry Pi 4 + MJBots Pi3Hat). Contains the `ros2_control` hardware interface, controllers, and custom messages that communicate with Moteus motor drivers via CAN-FD.

## Architecture

### ros2_control Pipeline

```
Commands (topics)  -->  Controllers  -->  Hardware Interface  -->  CAN-FD  -->  Moteus Motors
                                          | (read/write cycle)
Feedback (topics)  <--  Broadcasters <--  Hardware Interface  <--  CAN-FD  <--  Moteus Motors
```

### Package Dependency Graph

```
pi3hat_moteus_int_msgs          <-- Custom .msg definitions (base dependency)
  ^
moteus_pi3hat                   <-- Low-level C++ library wrapping Pi3Hat CAN-FD
  ^
pi3hat_hw_interface             <-- ros2_control SystemInterface plugin
  ^
omni_controller                 <-- Unified omnidirectional controller (wheels IK + legs + state broadcasting)
pi3hat_base_controller          <-- Standalone joint controller + state broadcasters
```

### Robot Configurations

| Config       | XACRO              | YAML              | Description                                         |
| ------------ | ------------------ | ----------------- | --------------------------------------------------- |
| **Omnicar**  | `omnicar.xacro`    | `omnicar.yaml`    | 4 mecanum wheels + 2 power distributors             |
| **Omniquad** | `omniquad12.xacro` | `omniquad12.yaml` | 4 mecanum wheels + 8 leg joints (HFE + KFE per leg) |

## Quick Start

### Build

```bash
colcon build --symlink-install
source install/local_setup.bash
```

> `pi3hat_moteus_int_msgs` must build first (all other packages depend on it).

### Launch

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=omnicar.xacro \
  conf_file:=omnicar.yaml
```
Additional launch arguments:
- `record_bag` (bool, default: `false`): Whether to record a rosbag of all topics except camera and lidar.

Defaults are `omnicar.xacro` / `omnicar.yaml`. URDF files live in `pi3hat_hw_interface/urdf/`, config YAMLs in `pi3hat_hw_interface/config/`.

## Adding and Using Your Robot

**See the [Usage Guide](docs/usage_guide.md)** for the complete walkthrough: creating the XACRO and YAML for a new robot, configuring safety and wheel modes, launching, controlling (topics, services, message types), and reading state feedback.

## Raspberry Pi Prerequisites

- Ubuntu 22.04 with RT kernel on Raspberry Pi 4 ([tested image](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.3_v5.15.98-rt62-raspi_ros2_humble))
- Pi3Hat r4.4 or newer
- `bcm_host` library: `sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0`
- `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-xacro`
- Python moteus library: `pip3 install moteus==0.3.67 moteus_pi3hat`

See [Pi3Hat Robotic Systems](src/pi3hat/README.md) for detailed Raspberry Pi setup instructions.

## Development

### Pre-commit

Install pre-commit and the hooks defined in `.pre-commit-config.yaml`:

```shell
sudo apt install npm
pip install pre-commit
pre-commit install
```

The hooks will run automatically on every `git commit`. To run them manually on all files:

```shell
pre-commit run --all-files
```

### Docker (ARM64 development on x86 host)

For cross-compiling / testing on an x86 machine via QEMU emulation:

```bash
make build    # Build Docker image (first time / Dockerfile changes)
make start    # Start container
make attach   # Enter the container shell
make stop     # Stop container
```

Or directly:

```bash
bash docker/build.bash
docker compose -f docker/docker-compose.yaml up -d
docker exec -it mulsbc-arm64-dev bash
docker compose -f docker/docker-compose.yaml down
```

The workspace is bind-mounted at `/ws/` inside the container.
