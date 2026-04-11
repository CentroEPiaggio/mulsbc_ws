# Usage Guide: Adding and Using a Custom Robot

This guide walks through the steps to integrate a new robot with the Mulinex SBC workspace, from hardware definition to runtime control.

## Step 1: Create the XACRO File

Create `src/pi3hat/pi3hat_hw_interface/urdf/<robot_name>.xacro`. This file defines every joint on your robot and how it maps to a Moteus motor on the Pi3Hat CAN buses.

### Structure Overview

```xml
<?xml version="1.0" ?>
<robot name="<robot_name>" xmlns:xacro="http://ros.org/wiki/xacro">

    <!-- 1. Define macros for each joint type -->
    <!-- 2. Hardware plugin block -->
    <!-- 3. Instantiate joints -->

</robot>
```

### Joint Types

There are two types of joints, distinguished by the `type` parameter:

- **`motor`** — Any actuated joint (wheels, legs, arms). These receive position/velocity/effort commands.
- **`power_dist`** — Power distributor boards. These only report voltage, current, and temperature.

Among motor joints, the critical distinction is how they are **commanded**:

| Joint role | Command mode | Typical PID | Position limits |
|---|---|---|---|
| **Wheel** | Velocity (IK or direct) | KP=0 or low, KD=1 | Very large (±5000) — effectively unlimited |
| **Non-wheel** (leg, arm) | Position (from external planner) | KP=100, KD=1 | Physical range (e.g. ±3.14 rad) |

### Defining a Joint Macro

Each joint must specify its Moteus configuration. Here is a generic motor joint macro:

```xml
<xacro:macro name="joint" params="name id bus kp kd second_encoder_source
                                   max_pos_limit min_pos_limit actuator_trasmission">
    <joint name="${name}">
        <param name="id">${id}</param>              <!-- Moteus CAN ID -->
        <param name="bus">${bus}</param>             <!-- Pi3Hat bus (1-5) -->
        <param name="second_encoder_source">${second_encoder_source}</param>
        <param name="type">motor</param>

        <!-- PID gains (sent to Moteus on-board controller) -->
        <param name="KP">${kp}</param>
        <param name="KD">${kd}</param>
        <param name="KI">0.0</param>
        <param name="ilimit">0.0</param>
        <param name="iratelimit">0.0</param>
        <param name="max_position_slip">0.0</param>
        <param name="max_velocity_slip">0.0</param>
        <param name="enable_motor_temperature">0</param>

        <!-- Joint limits -->
        <param name="max_pos_limit">${max_pos_limit}</param>
        <param name="min_pos_limit">${min_pos_limit}</param>
        <param name="max_velocity">10.0</param>
        <param name="max_effort">15.0</param>

        <!-- Mechanical -->
        <param name="actuator_trasmission">${actuator_trasmission}</param>
        <param name="position_offset">0</param>
        <param name="second_encoder_trasmission">3.23076923</param>

        <!-- Telemetry resolution (bits). Set to 0 to disable. -->
        <param name="position_res">64</param>
        <param name="velocity_res">64</param>
        <param name="torque_res">64</param>
        <param name="q_current_res">8</param>
        <param name="d_current_res">0</param>
        <param name="abs_position_res">0</param>
        <param name="power_res">8</param>
        <param name="motor_temperature_res">0</param>
        <param name="voltage_res">16</param>
        <param name="temperature_res">8</param>
        <param name="position_error_res">64</param>
        <param name="velocity_error_res">64</param>
        <param name="torque_error_res">64</param>
        <param name="second_encoder_position_res">64</param>
        <param name="second_encoder_velocity_res">64</param>
    </joint>
</xacro:macro>
```

You can then create convenience macros that wrap this generic one:

```xml
<xacro:macro name="leg_joint" params="name id bus">
    <xacro:joint name="${name}" id="${id}" bus="${bus}"
                 kp="100.0" kd="1.0"
                 second_encoder_source="-1"
                 max_pos_limit="3.14" min_pos_limit="-3.14"
                 actuator_trasmission="9"/>
</xacro:macro>

<xacro:macro name="wheel_joint" params="name id bus">
    <xacro:joint name="${name}" id="${id}" bus="${bus}"
                 kp="0.0" kd="1.0"
                 second_encoder_source="-1"
                 max_pos_limit="5000.0" min_pos_limit="-5000.0"
                 actuator_trasmission="15"/>
</xacro:macro>
```

### Key Parameters to Verify with Moteus

Before writing the xacro, you must know:

- **`id`** — The CAN ID configured on each Moteus controller (check with `moteus_tool --info`).
- **`bus`** — Which Pi3Hat CAN-FD bus the motor is physically wired to (buses 1–5).
- **`actuator_trasmission`** — The gear ratio between motor and joint. The hardware interface converts between motor-space and joint-space using this value.
- **`position_offset`** — Joint-space offset to align the zero position with your mechanical design.
- **`second_encoder_source`** — Set to `1` if using a secondary encoder, `-1` to disable.
- **KP/KD** — These are the Moteus on-board PID gains. For wheels in velocity mode, KP is typically 0. For position-controlled joints, KP is typically 100.

### Distributor Macro

If your robot has MJBots power distribution boards:

```xml
<xacro:macro name="distributor_joint" params="name id">
    <joint name="${name}">
        <param name="id">${id}</param>
        <param name="bus">5</param>
        <param name="type">power_dist</param>
        <param name="voltage">64</param>
        <param name="current">64</param>
        <param name="temperature">64</param>
        <param name="energy">0</param>
        <param name="state">0</param>
        <param name="switch_state">0</param>
        <param name="lock_time">0</param>
        <param name="boot_time">0</param>
    </joint>
</xacro:macro>
```

### Hardware Block and Joint Instantiation

```xml
<ros2_control name="MoteusPi3Hat_Interface" type="system">
    <hardware>
        <plugin>pi3hat_hw_interface/MoteusPi3Hat_Interface</plugin>
        <param name="timeout_ns">0</param>
        <param name="min_tx_wait_ns">20000</param>
        <param name="rx_baseline_wait_ns">200000</param>
        <param name="rx_extra_wait_ns">20000</param>
        <param name="CPU_affinity">1</param>

        <!-- IMU (Pi3Hat on-board) -->
        <param name="request_attitude">1</param>
        <param name="attitude_hz">400</param>
        <param name="mounting_deg_roll">0</param>
        <param name="mounting_deg_pitch">0</param>
        <param name="mounting_deg_yaw">0</param>
    </hardware>

    <!-- Instantiate all your joints here -->
    <xacro:leg_joint name="LF_HFE" id="3" bus="2"/>
    <xacro:leg_joint name="LF_KFE" id="4" bus="2"/>
    <xacro:wheel_joint name="LF_WHEEL" id="11" bus="2"/>
    <!-- ... etc ... -->

    <xacro:distributor_joint name="Distributor_L" id="32"/>
    <xacro:distributor_joint name="Distributor_R" id="33"/>
</ros2_control>
```

### Joint Naming Convention

Follow the established pattern:

- **Position prefix:** `LF` (Left Front), `RF` (Right Front), `LH` (Left Hind), `RH` (Right Hind)
- **Joint suffix:** `_WHEEL`, `_HFE` (Hip Flexion/Extension), `_KFE` (Knee Flexion/Extension)
- **Distributors:** `Distributor_L`, `Distributor_R`

The joint names you define here must match exactly in the YAML config.

---

## Step 2: Create the YAML Configuration

Create `src/pi3hat/pi3hat_hw_interface/config/<robot_name>.yaml`. This configures the controller manager, the OmniController, and any other controllers/broadcasters.

### Minimal Configuration (Wheels Only)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500

    state_broadcaster:
      type: pi3hat_base_controller/Pi3HatStateBroadcaster
    omni_controller:
      type: omni_controller/OmniController

state_broadcaster:
  ros__parameters:
    joints:
      - LF_WHEEL
      - RF_WHEEL
      - LH_WHEEL
      - RH_WHEEL

omni_controller:
  ros__parameters:
    # --- Wheel joints (required) ---
    # For mecanum (4 named wheels):
    wheel_joints:
      LF: LF_WHEEL
      LH: LH_WHEEL
      RF: RF_WHEEL
      RH: RH_WHEEL

    # For differential (2 sides, each can have multiple wheels):
    # wheel_joints:
    #   LEFT: [LEFT_WHEEL_1, LEFT_WHEEL_2]
    #   RIGHT: [RIGHT_WHEEL_1, RIGHT_WHEEL_2]

    # --- Wheel inverse kinematics ---
    feet_type: mecanum      # "mecanum", "differential", or "none"
    wheel_rad: 0.05         # Wheel radius [m]
    driveshaft_x: 0.235     # Half wheelbase (center to front axle) [m]
    driveshaft_y: 0.188     # Half track width (center to left wheel) [m]
    mecanum_angle: 135.0    # Mecanum roller angle [deg]
    # track_width: 0.4      # Only for differential mode [m]

    # --- Communication ---
    input_frequency: 100    # Expected command rate [Hz]
    BestEffort_QOS: true    # Use BestEffort QoS for twist commands

    # --- Features ---
    sim: false
    pub_odom: false
    pub_performance: true

    # --- Distributors (optional) ---
    distributor_names:
      - Distributor_R
      - Distributor_L

    # --- Secondary encoders (optional) ---
    second_encoder_joints:
      - LF_WHEEL
      - RF_WHEEL
      - LH_WHEEL
      - RH_WHEEL
```

### Adding Leg/Arm Joints

If your robot has non-wheel joints, add them under `joints` and configure the rest/stand transition:

```yaml
omni_controller:
  ros__parameters:
    # ... wheel config as above ...

    # --- Non-wheel joints ---
    joints:
      - LF_HFE
      - LF_KFE
      - RF_HFE
      - RF_KFE
      - LH_HFE
      - LH_KFE
      - RH_HFE
      - RH_KFE

    # --- Rest/Stand transition ---
    rest_duration: 5.0    # Time to interpolate to rest pose [s]
    stand_duration: 5.0   # Time to interpolate to stand pose [s]

    joint_targets:
      LF_HFE: {q_rest: 0.0, q_stand: -0.56}
      LF_KFE: {q_rest: 0.0, q_stand: 1.8}
      LH_HFE: {q_rest: 0.0, q_stand: 0.56}
      LH_KFE: {q_rest: 0.0, q_stand: -1.8}
      RF_HFE: {q_rest: 0.0, q_stand: 0.56}
      RF_KFE: {q_rest: 0.0, q_stand: -1.8}
      RH_HFE: {q_rest: 0.0, q_stand: -0.56}
      RH_KFE: {q_rest: 0.0, q_stand: 1.8}
```

### Safety Configuration

```yaml
omni_controller:
  ros__parameters:
    safety:
      enabled: true
      temp_warning_threshold: 50.0    # [°C] — triggers WARNING state
      temp_critical_threshold: 57.0   # [°C] — triggers CRITICAL state
      battery_min_voltage: 24.0       # [V]  — triggers CRITICAL if below
      temp_recovery_hysteresis: 1.0   # [°C]
      volt_recovery_hysteresis: 0.5   # [V]
      ema_window_samples: 5000
      critical_strategy: "damping"    # "damping", "stop", or "default_config"
      damping_duration: 3.0           # [s]  — only for damping/default_config
      legs_cmd_timeout: 0.5           # [s]  — legs stop if no command received
      heartbeat_timeout: 1.0          # [s]  — CRITICAL if NUC heartbeat lost
```

**Critical strategies:**

| Strategy | Behavior |
|---|---|
| `damping` | Locks wheels (velocity=0, kd=1). Legs: holds position then cosine-ramps kp from 1→0 over `damping_duration`, then stops. |
| `stop` | Immediately stops all motors (velocity=0, position=NaN, kp=0, kd=1). |
| `default_config` | Locks wheels. Legs: cosine-interpolates from current position to `q_rest` over `damping_duration`, then stops. |

The safety state machine transitions: `NORMAL → WARNING → CRITICAL → DAMPING → STOPPED`. Recovery from STOPPED requires calling the `~/activate_srv` service after fixing the root cause.

### Wheels Mode

The controller supports two wheel command modes, switchable at runtime:

| Mode | Description | Command source |
|---|---|---|
| `WHEEL_IK` (default) | Twist (vx, vy, omega) is converted to wheel velocities via inverse kinematics | `~/twist_cmd` |
| `WHEEL_DIRECT` | Per-wheel velocity commands bypass IK | `~/direct_wheels_cmd` |

Switch mode via:
```bash
# Switch to direct mode
ros2 service call /omni_controller/set_wheel_mode std_srvs/srv/SetBool "{data: true}"

# Switch back to IK mode
ros2 service call /omni_controller/set_wheel_mode std_srvs/srv/SetBool "{data: false}"
```

---

## Step 3: Launch

```bash
# Build
colcon build --symlink-install
source install/local_setup.bash

# Launch with your config
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=<robot_name>.xacro \
  conf_file:=<robot_name>.yaml
```

---

## Step 4: Control the Robot

### Activation

The controller starts in **INACTIVE** state. Activate it:

```bash
ros2 service call /omni_controller/activate_srv std_srvs/srv/SetBool "{data: true}"
```

For robots with legs, use the rest/stand services to transition the pose:

```bash
# Move legs to stand position
ros2 service call /omni_controller/stand_srv std_srvs/srv/SetBool "{data: true}"

# Move legs back to rest position
ros2 service call /omni_controller/rest_srv std_srvs/srv/SetBool "{data: true}"
```

### Emergency Stop

```bash
ros2 service call /omni_controller/emergency_srv std_srvs/srv/SetBool "{data: true}"
```

### Commanding Wheels (IK Mode)

Publish a twist to drive the base:

```bash
# Topic: /omni_controller/twist_cmd
# Type:  geometry_msgs/msg/Twist
ros2 topic pub /omni_controller/twist_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

- `linear.x` — Forward velocity [m/s]
- `linear.y` — Lateral velocity [m/s] (mecanum only)
- `angular.z` — Yaw rate [rad/s]

The twist must be published at or above `input_frequency` Hz, otherwise the deadline QoS triggers a timeout.

### Commanding Wheels (Direct Mode)

After switching to direct mode, publish per-wheel velocities:

```bash
# Topic: /omni_controller/direct_wheels_cmd
# Type:  pi3hat_moteus_int_msgs/msg/JointsCommand
ros2 topic pub /omni_controller/direct_wheels_cmd pi3hat_moteus_int_msgs/msg/JointsCommand \
  "{name: [LF_WHEEL, RF_WHEEL, LH_WHEEL, RH_WHEEL], velocity: [1.0, 1.0, 1.0, 1.0]}"
```

### Commanding Leg/Arm Joints

Publish position commands for non-wheel joints:

```bash
# Topic: /omni_controller/legs_cmd
# Type:  pi3hat_moteus_int_msgs/msg/JointsCommand
ros2 topic pub /omni_controller/legs_cmd pi3hat_moteus_int_msgs/msg/JointsCommand \
  "{name: [LF_HFE, LF_KFE], position: [0.5, 1.2], kp_scale: [1.0, 1.0], kd_scale: [1.0, 1.0]}"
```

Fields in `JointsCommand`:
- `name[]` — Joint names (must match xacro/yaml)
- `position[]` — Target position [rad]. Set to NaN to disable position control.
- `velocity[]` — Target velocity [rad/s]. Set to NaN to disable velocity control.
- `effort[]` — Feedforward torque [Nm]. Set to NaN to disable (causes motor fault if left at 0 unintentionally).
- `kp_scale[]` — Scale the Moteus KP gain [0.0–1.0]
- `kd_scale[]` — Scale the Moteus KD gain [0.0–1.0]

If no leg command is received within `legs_cmd_timeout`, the legs hold their last commanded position.

### NUC Heartbeat

If your robot has an external computer (NUC), it should publish a heartbeat to avoid safety shutdown:

```bash
# Topic: /nuc_heartbeat
# Type:  std_msgs/msg/Empty
ros2 topic pub /nuc_heartbeat std_msgs/msg/Empty "{}" --rate 10
```

If this topic is not received within `heartbeat_timeout`, the safety system triggers CRITICAL state.

---

## Step 5: Read Robot State

### Joint States

```bash
# Topic: /omni_controller/joints_state
# Type:  pi3hat_moteus_int_msgs/msg/JointsStates
ros2 topic echo /omni_controller/joints_state
```

Fields: `name[]`, `position[]`, `velocity[]`, `effort[]`, `current[]`, `temperature[]`, `sec_enc_pos[]`, `sec_enc_vel[]`, `voltage[]`, `power[]`.

### Command Echo

```bash
# Topic: /omni_controller/joints_command
# Type:  pi3hat_moteus_int_msgs/msg/JointsCommand
ros2 topic echo /omni_controller/joints_command
```

Shows the commands actually being sent to the hardware each cycle.

### Safety State

```bash
# Topic: /omni_controller/safety_state
# Type:  std_msgs/msg/UInt8
ros2 topic echo /omni_controller/safety_state
```

Values: `0`=NORMAL, `1`=WARNING, `2`=CRITICAL, `3`=DAMPING, `4`=STOPPED.

### Distributor State

```bash
# Topic: /omni_controller/distributors_state
# Type:  pi3hat_moteus_int_msgs/msg/DistributorsState
ros2 topic echo /omni_controller/distributors_state
```

Fields: `name[]`, `current[]` [A], `voltage[]` [V], `temperature[]` [°C].

### Wheel Odometry

If `pub_odom: true` in the config:

```bash
# Topic: /omni_controller/odom
# Type:  geometry_msgs/msg/TwistStamped
ros2 topic echo /omni_controller/odom
```

### Performance Diagnostics

If `pub_performance: true`:

```bash
# Topic: /omni_controller/performance
# Type:  pi3hat_moteus_int_msgs/msg/PacketPass
ros2 topic echo /omni_controller/performance
```

Fields: `valid` (% valid packets), `cycle_dur` [s], `write2read_dur` [s], `name[]`, `pack_loss[]` (per-motor packet loss %).

---

## Quick Reference: All Topics and Services

### Subscriptions (command inputs)

| Topic | Type | Description |
|---|---|---|
| `~/twist_cmd` | `geometry_msgs/msg/Twist` | Base velocity command (vx, vy, omega) |
| `~/legs_cmd` | `pi3hat_moteus_int_msgs/msg/JointsCommand` | Position commands for non-wheel joints |
| `~/direct_wheels_cmd` | `pi3hat_moteus_int_msgs/msg/JointsCommand` | Per-wheel velocity (direct mode only) |
| `/nuc_heartbeat` | `std_msgs/msg/Empty` | External computer heartbeat |

### Publications (state outputs)

| Topic | Type | Description |
|---|---|---|
| `~/joints_state` | `pi3hat_moteus_int_msgs/msg/JointsStates` | Full joint telemetry |
| `~/joints_command` | `pi3hat_moteus_int_msgs/msg/JointsCommand` | Command echo |
| `~/safety_state` | `std_msgs/msg/UInt8` | Safety state (0–4) |
| `~/distributors_state` | `pi3hat_moteus_int_msgs/msg/DistributorsState` | Power distributor data |
| `~/odom` | `geometry_msgs/msg/TwistStamped` | Wheel odometry (if enabled) |
| `~/performance` | `pi3hat_moteus_int_msgs/msg/PacketPass` | Communication diagnostics (if enabled) |

### Services

| Service | Type | Description |
|---|---|---|
| `~/activate_srv` | `std_srvs/srv/SetBool` | Activate controller (data=true) |
| `~/emergency_srv` | `std_srvs/srv/SetBool` | Emergency stop (data=true) |
| `~/rest_srv` | `std_srvs/srv/SetBool` | Transition legs to rest pose |
| `~/stand_srv` | `std_srvs/srv/SetBool` | Transition legs to stand pose |
| `~/set_wheel_mode` | `std_srvs/srv/SetBool` | Switch wheel mode (true=direct, false=IK) |

All topics prefixed with `~/` are under the controller namespace (e.g., `/omni_controller/twist_cmd`).
