# Omni Controller

Unified ros2_control controller for the Mulinex omnidirectional robot. Replaces four separate plugins with a single configurable controller:

| Old plugin | Unified feature |
|---|---|
| `Pi3Hat_Joint_Group_Controller` | Leg command passthrough (`leg_joints`) |
| `Omni_Vel_Controller` | Wheel IK (`wheel_joints` + `feet_type`) |
| `Pi3Hat_State_Broadcaster` | Joint state & performance publishing |
| `Distributor_State_Broadcaster` | Distributor state publishing (`distributor_names`) |

## Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `wheel_joints.<POS>` | `string` or `string[]` | `""` / `[]` | Wheel joint name(s) per position. Mecanum: `LF`, `LH`, `RF`, `RH` (single string each). Differential: `LEFT`, `RIGHT` (string array each — supports multiple wheels per side for skid-steer). |
| `leg_joints` | `string[]` | `[]` | Leg joint names (must match URDF) |
| `distributor_names` | `string[]` | `[]` | Power distributor names |
| `second_encoder_joints` | `string[]` | `[]` | Joints with secondary encoders |
| `feet_type` | `string` | `"none"` | Wheel IK type: `"mecanum"`, `"differential"`, or `"none"` |
| `wheel_rad` | `double` | `0.05` | Wheel radius [m] |
| `driveshaft_x` | `double` | `0.235` | Longitudinal offset from base center to wheel [m] |
| `driveshaft_y` | `double` | `0.188` | Lateral offset from base center to wheel [m] |
| `mecanum_angle` | `double` | `135.0` | Mecanum roller angle [deg] (mecanum IK only) |
| `track_width` | `double` | `0.4` | Track width [m] (differential IK only) |
| `input_frequency` | `int` | `100` | Expected twist command rate [Hz] (sets deadline QoS) |
| `BestEffort_QOS` | `bool` | `true` | Use BestEffort (true) or Reliable (false) QoS for twist subscriber |
| `sim` | `bool` | `false` | Simulation mode (velocity-only command interfaces) |
| `pub_odom` | `bool` | `false` | Publish wheel odometry |
| `pub_performance` | `bool` | `true` | Publish performance indexes |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `~/twist_cmd` | `Twist` | sub | Base velocity command (when `wheel_joints` + IK configured) |
| `~/legs_cmd` | `JointsCommand` | sub | Per-joint leg commands (when `leg_joints` configured) |
| `~/joints_state` | `JointsStates` | pub | All motor joint states |
| `~/performance` | `PacketPass` | pub | CAN packet loss & cycle duration (when `pub_performance: true`) |
| `~/distributors_state` | `DistributorsState` | pub | Power distributor states (when `distributor_names` configured) |
| `~/odom` | `TwistStamped` | pub | Wheel odometry (when `pub_odom: true`) |

## Services

| Service | Type | Description |
|---|---|---|
| `~/activate_srv` | `SetBool` | Transition from INACTIVE to ACTIVE (`data: true`) |
| `~/emergency_srv` | `SetBool` | Transition from ACTIVE to INACTIVE (`data: true`) |

## State Machine

```
on_activate()
     │
     ▼
 INACTIVE ──── ~/activate_srv (data: true) ────► ACTIVE
     ▲                                              │
     │                                              │
     ├──── ~/emergency_srv (data: true) ◄───────────┤
     └──── 10+ deadline misses (auto) ◄─────────────┘
```

- **INACTIVE**: All motor commands zeroed (velocity=0, kp_scale=0, kd_scale=1). State publishing continues.
- **ACTIVE**: Wheel IK and leg commands written to hardware.

## Wheel IK Types

- **`mecanum`**: 4-wheel mecanum inverse kinematics (LF, LH, RF, RH). Requires `wheel_joints.{LF,LH,RF,RH}`, `driveshaft_x`, `driveshaft_y`, `mecanum_angle`, `wheel_rad`.
- **`differential`**: Differential drive with 1 or more wheels per side. Requires `wheel_joints.{LEFT,RIGHT}` (string arrays), `track_width`, `wheel_rad`. All wheels on the same side receive the same velocity. Odometry averages wheel velocities per side.
- **`none`**: No IK. Wheel joints still get zero commands when INACTIVE but receive no IK output when ACTIVE (useful for direct joint control via `leg_joints`).

## Example Config

See [`config/omnicar.yaml`](config/omnicar.yaml) for a complete Omnicar configuration.

### Differential (skid-steer) example

```yaml
omni_controller:
  ros__parameters:
    feet_type: differential
    wheel_joints:
      LEFT:  [left_front_jnt, left_rear_jnt]
      RIGHT: [right_front_jnt, right_rear_jnt]
    track_width: 0.4
    wheel_rad: 0.05
```
