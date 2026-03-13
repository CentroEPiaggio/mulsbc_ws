# Pi3Hat Hardware Interface (pi3hat_hw_interface)

ros2_control SystemInterface plugin for Pi3Hat + Moteus hardware.

## Overview

This package implements the `ros2_control` `SystemInterface` plugin that communicates with Moteus motor drivers through the MJBots Pi3Hat via CAN-FD. It handles the real-time read/write cycle, motor configuration (PID, limits, gear ratios), IMU data, and safety monitoring.

## Quick Start

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=omnicar.xacro \
  conf_file:=omnicar.yaml
```

URDF files are in `urdf/`, config YAMLs in `config/`.

## Hardware Configuration (XACRO)

The hardware is configured via a XACRO file that defines the `<ros2_control>` description. See [`omnicar.xacro`](urdf/omnicar.xacro) and [`omniquad12.xacro`](urdf/omniquad12.xacro) for examples.

### Hardware Parameters (`<hardware>` block)

| Parameter | Description |
|-----------|-------------|
| `timeout_ns` | Overall Pi3Hat cycle timeout [ns] (0 = no timeout) |
| `min_tx_wait_ns` | Minimum wait after transmitting [ns] |
| `rx_baseline_wait_ns` | Base wait time for receiving CAN replies [ns] |
| `rx_extra_wait_ns` | Extra wait per received CAN frame [ns] |
| `CPU_affinity` | CPU core affinity for the real-time thread |
| `request_attitude` | Enable IMU data collection (0 = disabled) |
| `attitude_hz` | IMU sampling rate [Hz] |
| `mounting_deg_roll/pitch/yaw` | IMU mounting orientation offset [deg] |
| `temp_warning_threshold` | Temperature warning threshold [°C] (default: 80) |
| `temp_critical_threshold` | Temperature critical threshold [°C] (default: 100) |
| `battery_min_voltage` | Minimum battery voltage [V] (default: 18.0) |
| `shutdown_delay` | Time in CRITICAL before SHUTDOWN [s] (default: 3.0) |

### Joint Parameters (`<joint>` entries)

| Parameter | Description |
|-----------|-------------|
| `id` / `bus` | CAN ID and Pi3Hat bus number (1–5) |
| `type` | `motor` or `power_dist` |
| `KP` / `KD` / `KI` | Low-level PID gains (**must not exceed 16-bit representability**) |
| `ilimit` / `iratelimit` | Integral limit and integral rate limit |
| `actuator_trasmission` | Motor-to-joint gear ratio |
| `position_offset` | Joint-space zero offset [rad] |
| `max_pos_limit` / `min_pos_limit` | Joint position limits [rad] |
| `max_velocity` | Joint velocity saturation [rad/s] |
| `max_effort` | Joint torque saturation [Nm] |
| `second_encoder_trasmission` | Joint-to-secondary-encoder ratio (0 disables) |
| `second_encoder_source` | Second encoder data source selector |
| `max_position_slip` / `max_velocity_slip` | Slip tolerances |
| `enable_motor_temperature` | Enable motor temperature monitoring (0 = disabled) |
| `*_res` parameters | CAN-FD reply resolution per quantity (e.g., `position_res`, `velocity_res`) |

## State / Command Interfaces

Beyond standard `position`/`velocity`/`effort`:

- **Commands:** `kp_scale_value`, `kd_scale_value` (runtime gain scaling)
- **States:** `temperature`, `q_current`, `d_current`, `motor_temperature`, `abs_position`, `power`, `validity_loss`, `package_loss`, `cycle_duration`

## See Also

- [Pi3Hat Robotic Systems](../README.md) — Detailed Raspberry Pi setup instructions
- [Root README](../../../README.md) — Workspace overview and usage guide
