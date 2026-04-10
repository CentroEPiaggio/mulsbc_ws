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

Defaults are `omnicar.xacro` / `omnicar.yaml`. URDF files live in `pi3hat_hw_interface/urdf/`, config YAMLs in `pi3hat_hw_interface/config/`.

## Controlling the Robot

The **OmniController** is the primary user-facing controller. All topics are under the `/omni_controller/` namespace.

### Step 1: Activate the controller

Call the activation service before sending commands:

```bash
ros2 service call /omni_controller/activate_srv std_srvs/srv/SetBool "{data: true}"
```

### Step 2: Send commands

**Drive the base** (wheels) by publishing a `Twist` message:

```bash
ros2 topic pub /omni_controller/twist_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Command leg joints** (Omniquad only) by publishing a `JointsCommand`:

```bash
ros2 topic pub /omni_controller/legs_cmd pi3hat_moteus_int_msgs/msg/JointsCommand \
  "{name: ['LF_HFE','LF_KFE'], position: [0.5, -1.0], velocity: [0.0, 0.0], effort: [0.0, 0.0], kp_scale: [1.0, 1.0], kd_scale: [1.0, 1.0]}"
```

### Step 3: Read feedback

```bash
ros2 topic echo /omni_controller/joints_state    # Joint positions, velocities, efforts, currents, temperatures
ros2 topic echo /omni_controller/performance      # CAN-FD communication performance metrics
ros2 topic echo /omni_controller/distributors_state  # Power distributor voltage, current, temperature
```

### Emergency stop

```bash
ros2 service call /omni_controller/emergency_srv std_srvs/srv/SetBool "{data: true}"
```

## Topics Reference (OmniController)

### Input Topics (user publishes)

| Topic                        | Message Type                               | Description                                                                                                                                     |
| ---------------------------- | ------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `/omni_controller/twist_cmd` | `geometry_msgs/msg/Twist`                  | Base velocity: `linear.x` (forward), `linear.y` (left), `angular.z` (CCW). QoS: BestEffort with deadline at `input_frequency` (default 100 Hz). |
| `/omni_controller/legs_cmd`  | `pi3hat_moteus_int_msgs/msg/JointsCommand` | Joint commands (position, velocity, effort, kp/kd scaling). QoS: Reliable. Only active when `joints` are configured.                    |

### Output Topics (user subscribes)

| Topic                                 | Message Type                                   | Description                                                                                                     |
| ------------------------------------- | ---------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| `/omni_controller/joints_state`       | `pi3hat_moteus_int_msgs/msg/JointsStates`      | All motor feedback: position, velocity, effort, current, temperature, secondary encoder data. Always published. |
| `/omni_controller/performance`        | `pi3hat_moteus_int_msgs/msg/PacketPass`        | CAN-FD packet loss and cycle timing. Published when `pub_performance: true` (default).                          |
| `/omni_controller/distributors_state` | `pi3hat_moteus_int_msgs/msg/DistributorsState` | Power distributor states. Published when distributors are configured.                                           |
| `/omni_controller/odom`               | `geometry_msgs/msg/TwistStamped`               | Wheel odometry (base frame velocities). Published when `pub_odom: true`.                                        |

### Services

| Service                          | Type                   | Description                                                               |
| -------------------------------- | ---------------------- | ------------------------------------------------------------------------- |
| `/omni_controller/activate_srv`  | `std_srvs/srv/SetBool` | `data: true` to activate, `data: false` to deactivate command processing. |
| `/omni_controller/emergency_srv` | `std_srvs/srv/SetBool` | `data: true` to enter emergency stop.                                     |

## Customizing for Your Robot

To add a new robot configuration you need two files, both inside `pi3hat_hw_interface/`:

1. A **XACRO** file (`urdf/myrobot.xacro`) — defines the hardware: motors, CAN bus layout, PID gains, gear ratios, and safety thresholds.
2. A **YAML** file (`config/myrobot.yaml`) — defines the controllers: which joints are wheels vs legs, IK type, wheel geometry, and update rate.

### Step 1: Create a XACRO (`pi3hat_hw_interface/urdf/myrobot.xacro`)

The XACRO is a `<ros2_control>` description with a `<hardware>` block (Pi3Hat communication and IMU settings) followed by `<joint>` entries for each motor and power distributor.

Key parameters per joint:

| Parameter                         | Description                                                       |
| --------------------------------- | ----------------------------------------------------------------- |
| `id` / `bus`                      | CAN ID and Pi3Hat bus number (1–5)                                |
| `type`                            | `motor` or `power_dist`                                           |
| `KP` / `KD` / `KI`                | Low-level PID gains (**must not exceed 16-bit representability**) |
| `actuator_trasmission`            | Motor-to-joint gear ratio                                         |
| `position_offset`                 | Joint-space zero offset [rad]                                     |
| `max_pos_limit` / `min_pos_limit` | Joint position limits [rad] (0 disables)                          |
| `second_encoder_trasmission`      | Joint-to-secondary-encoder ratio (0 disables)                     |

Use xacro macros to avoid repetition — see [`omnicar.xacro`](src/pi3hat/pi3hat_hw_interface/urdf/omnicar.xacro) (minimal: wheels only) and [`omniquad12.xacro`](src/pi3hat/pi3hat_hw_interface/urdf/omniquad12.xacro) (wheels + legs) as references.

For the full list of hardware and joint parameters, see [`src/pi3hat/README.md` § "Interface configuration file"](src/pi3hat/README.md#interface-configuration-file).

### Step 2: Create a YAML (`pi3hat_hw_interface/config/myrobot.yaml`)

The YAML configures the controller manager and each controller's parameters.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500 # Hz — must match your real-time loop capability

    omni_controller:
      type: omni_controller/OmniController

omni_controller:
  ros__parameters:
    # Wheel joints — keys depend on IK type (mecanum: LF/LH/RF/RH, differential: LEFT/RIGHT)
    wheel_joints:
      LF: LF_WHEEL
      LH: LH_WHEEL
      RF: RF_WHEEL
      RH: RH_WHEEL

    # Optional: leg joints (omit or leave commented out if not used)
    # joints:
    #     - LF_HFE
    #     - LF_KFE

    # Optional: power distributor names
    # distributor_names:
    #     - Distributor_L

    feet_type: mecanum # "mecanum", "differential", or "none"
    wheel_rad: 0.05 # Wheel radius [m]
    driveshaft_x: 0.235 # Longitudinal offset from center to wheel [m]
    driveshaft_y: 0.188 # Lateral offset from center to wheel [m]
    mecanum_angle: 135.0 # Roller angle [deg] (mecanum only)
    # track_width: 0.4        # Track width [m] (differential only)
```

For the complete parameter table, see [`src/omni_controller/README.md`](src/omni_controller/README.md).

### Step 3: Launch

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=myrobot.xacro \
  conf_file:=myrobot.yaml
```

## Custom Message Definitions

### JointsCommand

Used to command joints (legs via OmniController, or any joint via Pi3Hat_Joint_Group_Controller).

```
std_msgs/Header header
string[]  name        # Joint names
float64[] position    # Position commands [rad] (NaN disables P control)
float64[] velocity    # Velocity commands [rad/s] (NaN disables D control)
float64[] effort      # Effort/torque commands [Nm] (NaN causes motor fault)
float64[] kp_scale    # Proportional gain scale [0.0-1.0]
float64[] kd_scale    # Derivative gain scale [0.0-1.0]
```

### JointsStates

Feedback from all motor joints.

```
std_msgs/Header header
string[]  name         # Joint names
float64[] position     # Joint positions [rad]
float64[] velocity     # Joint velocities [rad/s]
float64[] effort       # Joint torques [Nm]
float64[] current      # Q-axis motor currents [A]
float64[] temperature  # Driver temperatures [C]
float64[] sec_enc_pos  # Secondary encoder positions (if configured)
float64[] sec_enc_vel  # Secondary encoder velocities (if configured)
```

### PacketPass

CAN-FD communication performance metrics.

```
std_msgs/Header header
float64   valid          # Percentage of valid packets received
float64   cycle_dur      # Cycle operation duration [s]
float64   write2read_dur # Time left for Pi3Hat communication [s]
string[]  name           # Motor IDs
float64[] pack_loss      # Packet loss percentage per motor
```

### DistributorsState

Power distributor feedback.

```
std_msgs/Header header
string[]  name         # Distributor names
float64[] current      # Currents [A]
float64[] voltage      # Voltages [V]
float64[] temperature  # Temperatures [C]
```

### OmniMulinexCommand

Legacy demo message for omnidirectional base commands.

```
std_msgs/Header header
float64 v_x         # Velocity X in base frame [m/s]
float64 v_y         # Velocity Y in base frame [m/s]
float64 omega       # Angular velocity around Z [rad/s]
float64 height_rate  # Vertical velocity [m/s]
```

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
