# Pi3Hat Moteus Interface Messages (pi3hat_moteus_int_msgs)

Custom ROS 2 message definitions for Pi3Hat Moteus motor interface.

## Overview

This package defines the custom message types used across the Pi3Hat workspace for motor commanding, state feedback, and performance monitoring.

## Message Definitions

### JointsCommand

Used to command motor joints (legs via OmniController, or any joint via Pi3Hat_Joint_Group_Controller).

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `name` | `string[]` | Joint names |
| `position` | `float64[]` | Position commands [rad] (NaN disables P control) |
| `velocity` | `float64[]` | Velocity commands [rad/s] (NaN disables D control) |
| `effort` | `float64[]` | Effort/torque commands [Nm] (NaN causes motor fault) |
| `kp_scale` | `float64[]` | Proportional gain scale [0.0–1.0] |
| `kd_scale` | `float64[]` | Derivative gain scale [0.0–1.0] |

### JointsStates

Feedback from all motor joints.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `name` | `string[]` | Joint names |
| `position` | `float64[]` | Joint positions [rad] |
| `velocity` | `float64[]` | Joint velocities [rad/s] |
| `effort` | `float64[]` | Joint torques [Nm] |
| `current` | `float64[]` | Q-axis motor currents [A] |
| `temperature` | `float64[]` | Driver temperatures [°C] |
| `sec_enc_pos` | `float64[]` | Secondary encoder positions (if configured) |
| `sec_enc_vel` | `float64[]` | Secondary encoder velocities (if configured) |

### PacketPass

CAN-FD communication performance metrics.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `valid` | `float64` | Percentage of valid packets received |
| `cycle_dur` | `float64` | Cycle operation duration [s] |
| `write2read_dur` | `float64` | Time left for Pi3Hat communication [s] |
| `name` | `string[]` | Motor IDs |
| `pack_loss` | `float64[]` | Packet loss percentage per motor |

### DistributorsState

Power distributor feedback.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `name` | `string[]` | Distributor names |
| `current` | `float64[]` | Currents [A] |
| `voltage` | `float64[]` | Voltages [V] |
| `temperature` | `float64[]` | Temperatures [°C] |

### OmniMulinexCommand

Legacy demo message for omnidirectional base commands.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `v_x` | `float64` | Velocity X in base frame [m/s] |
| `v_y` | `float64` | Velocity Y in base frame [m/s] |
| `omega` | `float64` | Angular velocity around Z [rad/s] |
| `height_rate` | `float64` | Vertical velocity [m/s] |

## See Also

For usage examples, see the [root README](../../../README.md) or the [OmniController README](../../omni_controller/README.md).
