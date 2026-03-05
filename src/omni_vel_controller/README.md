# Omni Vel Controller

ROS 2 controller for omnidirectional (mecanum) wheeled bases. I.e., omnicar and omniquad (traquad is TODO).
Converts base velocity commands (`v_x`, `v_y`, `omega`) into individual wheel velocity references using the mecanum inverse kinematics.

## Supported robots

The controller is parameterized so the same package works for both robots:

| Feature | omnicar | omniquad |
|---|---|---|
| Wheel velocity control | yes | yes |
| `forward_height_rate` | no (disabled) | yes — publishes `~/height_rate` for the leg IK controller |
| `wheel_names` | default joint names | override to match omniquad URDF |

## Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `wheel_names` | `string[]` | `[RF_WHEEL_JNT, LF_WHEEL_JNT, LH_WHEEL_JNT, RH_WHEEL_JNT]` | Wheel joint names (must match URDF) |
| `driveshaft_y` | `double` | `0.188` | Lateral offset from base center to wheel [m] |
| `driveshaft_x` | `double` | `0.235` | Longitudinal offset from base center to wheel [m] |
| `mecanum_angle` | `double` | `45.0` | Mecanum roller angle [deg] |
| `wheel_rad` | `double` | `0.05` | Wheel radius [m] |
| `input_frequency` | `int` | `100` | Expected command rate [Hz] |
| `BestEffort_QOS` | `bool` | `true` | Use BestEffort (true) or Reliable (false) QoS |
| `pub_odom` | `bool` | `false` | Publish wheel odometry on `~/wheel_odom` |
| `forward_height_rate` | `bool` | `false` | Forward `height_rate` from command msg to `~/height_rate` topic |
| `sim` | `bool` | `false` | Simulation mode (reduces command interfaces to velocity only) |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `~/command` | `OmniMulinexCommand` | sub | Base velocity + height rate command |
| `~/joints_reference` | `JointsCommand` | pub | Per-wheel command sent to hardware |
| `~/wheel_odom` | `TwistStamped` | pub | Wheel odometry (when `pub_odom: true`) |
| `~/height_rate` | `Float64` | pub | Height rate forwarded for leg control (when `forward_height_rate: true`) |

## Launch

Omnicar (default)
```shell
ros2 launch omni_vel_controller omni_vel_controller.launch.py
```

OmniQuad
```shell
ros2 launch omni_vel_controller omni_vel_controller.launch.py robot:=omniquad
```

Or load parameters directly via the controller manager YAML in your robot bringup.
