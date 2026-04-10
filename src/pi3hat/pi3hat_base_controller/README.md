# Pi3Hat Base Controller (pi3hat_base_controller)

ros2_control controllers and state broadcasters for Pi3Hat joints.

## Overview

This package provides standalone ros2_control controller plugins for directly commanding joints and broadcasting state feedback. These are an alternative to the unified `OmniController` — useful for simpler setups or debugging.

## Plugins

### Pi3HatJointGroupController

`pi3hat_joint_group_controller/Pi3HatJointGroupController`

Commands individual joints using the low-level Moteus PID controller.

#### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `joints` | `string[]` | `[]` | List of joint names |
| `init_pos` | `double[]` | `[]` | Initial joint positions [rad] (zeros if not provided) |

#### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `~/command` | `JointsCommand` | sub | Per-joint position, velocity, effort, and kp/kd scale commands |

### Pi3HatStateBroadcaster

`pi3hat_state_broadcaster/Pi3HatStateBroadcaster`

Broadcasts motor joint states and CAN-FD performance metrics.

#### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `joints` | `string[]` | `[]` | List of joint names |
| `second_encoders` | `string[]` | `[]` | Joints equipped with secondary encoders |
| `performance_index` | `bool` | `false` | Enable CAN-FD performance publishing |

#### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `~/joints_state` | `JointsStates` | pub | Joint positions, velocities, efforts, currents, temperatures |
| `~/performance_indexes` | `PacketPass` | pub | CAN cycle time, validation, per-motor packet loss (when `performance_index: true`) |

### DistributorStateBroadcaster

`distributor_state_broadcaster/DistributorStateBroadcaster`

Broadcasts power distributor states.

#### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `~/distributors_state` | `DistributorsState` | pub | Distributor current, voltage, and temperature |

### DebugBroadcaster

`debug_broadcaster/DebugBroadcaster`

Broadcasts additional debug information from Moteus drivers.

## See Also

- [OmniController](../../omni_controller/README.md) — Unified controller with wheel IK and leg commands
- [Root README](../../../README.md) — Workspace overview and usage guide
