# Pi3hat Robotic Systems

ROS2_control hardware interface to hook up the pi3hat/moteus base robotic system. it allow the user to control the MJBots Moteus driver collocated in a general porpouse robotic actuator, eventually adding a second encoder.
The ros2 packages are developed to be run on a raspberry connected to the MJBots Pi3Hat system, it performs as a fd-can master and IMU sensor manager.

When the pi3hat system is configuring, i.e when the Kp,Kd,pos_limit, etc. are set, the user must not use parameter that exceed the representativity on 16 bit, otherwise this operation will write in memory different memory zone causing unexpected, dangerous and even mortal behavior.

## Prerequisite

Raspberry Pi 4 running ubuntu 22.04 and ROS Humble [(tested on this image)](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.3_v5.15.98-rt62-raspi_ros2_humble)

pi3hat r 4.4 or newer

## Raspberry Setup

1. install the cpp compiler package
   ```shell
   sudo apt install build-essential
   ```

2. install the colcon builder packagers
   ```shell
   sudo apt install python3-colcon-common-extensions
   ```

3. allow the code to access the bmc_host library, needed by
   ```shell
   sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0
   sudo apt install libraspberrypi-dev
   ```

4. install ros2 control framework and xacro
   ```shell
   sudo apt install ros-$ROS_DISTRO$-ros2-control
   sudo apt install ros-$ROS_DISTRO$-ros2-controllers
   sudo apt install ros-$ROS_DISTRO$-xacro
   ```

5. install pip3 and the moteus libraries:
   ```shell
   sudo apt install python3-pip
   sudo pip3 install moteus==0.3.67
   sudo pip3 install moteus_pi3hat
   ```

6. Optional: install the Network-Manager to use the Raspberry PI4 as Access Point
   ```shell
   sudo apt install network-manager
   ```

## ROS2 Packages

1. **moteus_pi3hat**: is the ros2 package version of the original code to operate on the pi3hat using moteus driver

2. **pi3hat_hw_interface**: contains the hardware interface implementation

3. **pi3hat_moteus_int_msgs**: contains the interfaces to use all the controllers available in this repository.
   1. **JointCommand**: is a message used by the joint_controller, it is a sort of sensor_msgs/JointStates message contains also the scales associate with the proportional and the derivative gains.
   2. **JointStates**: is a message used by the state_broadcaster, it is a sort of sensor_msgs/JointStates message contains also the temperature, the current and eventually the second encoder position and velocity.
   3. **PacketPass**: is a message used by the state_broadcaster. it contains the information about the performance of the low level fd can comunication. it provide the state of the whole comunication composed by the communication cycle period in second and the validity loss, i.e. the not completed comminication cycle amount per hundred iteration. Then it provide also the each motor packet loss amount per hundred iteration when the communication cycle is completed.
   4. **OmniMulinexCommand**: message used by the omni_wheels_controller. it provide the floating base 2D twist with the heigth rate. NB used only for demos.

4. **pi3hat_base_controllers**: contains two base controllers. The joints_controller can be use to control each joint indipendently enforsing the drivers low level PID control using the JointCommand messages. The state_broadcaster controller can be use to monitor the joints states and the low level communication state using the JointStates and the PacketPass Messages

5. **pi3hat_omni_controller**: contains a controller to perform the omni_quad Demos.

## Interface configuration file

The [ROS2 Control](https://control.ros.org/master/index.html) framework must be known to understand this part.

The Interfaces parameters can be divided into two groups:
- pi3hat parameters, used to regulate the low level communication and to use the integrated IMU sensor.
- joints parameters, used to set up the low level PID and the communication three.

These parameter must be set into the xacro contain into pi3hat_ha_interface's config folder.
An example of the URDF file is:

```xml
<?xml version="1.0" ?>
<robot name="mulinex"  xmlns:xacro="http://ros.org/wiki/xacro">
    <ros2_control name="MoteusPi3Hat_Interface" type="system">
        <hardware>
            <plugin>pi3hat_hw_interface/MoteusPi3Hat_Interface</plugin>
            <param name="main_timeout">400000</param>
            <param name="can_timeout">20000</param>
            <param name="rcv_timeout">20000</param>

            <param name="attitude">1</param>

            <param name="b2imu_pos_x">0</param>
            <param name="b2imu_pos_y">0</param>
            <param name="b2imu_pos_z">0</param>

            <param name="b2imu_roll">0</param>
            <param name="b2imu_pitch">0</param>
            <param name="b2imu_yaw">PI/2</param>
            <param name="acc_correction">0</param>
        </hardware>

        <joint name="RF_HFE">
            <param name="id">3</param>
            <param name="bus">4</param>
            <param name="motor_transmission">9.0</param>
            <param name="sec_enc_transmission">0.0</param>
            <param name="KP">0.0</param>
            <param name="KD">0.0</param>
            <param name="KI">0.0</param>
            <param name="i_limit">0.0</param>
            <param name="p_lim_max">0.0</param>
            <param name="p_lim_min">0.0</param>
            <param name="p_offset">0.0</param>
            <param name="max_vel">10.0</param>
            <param name="max_torque">5.0</param>
        </joint>

    </ros2_control>
</robot>
```

1. Communication parameter:
   - **main_timeout**: is the base time waited by the pi3hat recive the joint response
   - **can_timeout**: is an extra timeout, it is add to the base one if a can message has been recieved.
   - **rcv_timeout**: is an extra timeout, it is add to the base one if a verified(CRC) message has beed received.

2. IMU parameter:
   - **attitude**: if set to zero the pi3hat do not collect the IMU data.
   - **acc_correction**: if set to zero it collect the raw data from IMU, else it collect the data corrected by a VHS, removing the gravity acceleration and providing the pi3hat orientation.
   - **b2imu_[roll/pitch/yaw]**: orientation offset of the pi3hat frame respect the a desired floating base frame
   - **b2imu_pos_[x/y/z]**: position offset of the pi3hat frame respect the a desired floating base frame

3. Joints parameter:
   - **KP, KD, KI, i_limit**: parameter for the low level PID controller
   - **bus, id**: parameter to identify the pi3hat can bus port and the driver id number
   - **motor_transmission**: value of the transmission rateo from the motor to the actuator joint, all the motor quantities provided by the interfaces take into account this value
   - **sec_enc_transmission**: value of the transmission between actiator joint and the second encoder, the second encoder quantities take into account this value. if set to zero the the interface do not provide the second econder measures.
   - **p_lim_max, p_lim_min**: joint level saturarion position of the actuator in radians, if set to zero the limit is disabled.
   - **p_offset**: joint level offset position of the actuator in radians, if set to zero the limit is disabled.
   - **max_vel**: joint level velocity saturarion.
   - **max_torque**: joint level torque saturation.

## Joints_Controller

### Parameters

- **joints**: the list of all joint
- **init_pos**: the list of all joints start position, if not provided it will be zero at all joints

### Topic

`joint_controller/command`: Message type `pi3hat_moteus_int_msgs/JointCommand`.

- Joints Name, list of joints name
- Positions reference, if NaN the PI control is disabled
- Velocities reference, if NaN the D control is disabled
- Torques reference, if NaN the motor will fault
- kp_gain_scale, base propotional gain scale factor
- kd_gain_scale, base derivative gain scale factor

## State_Broadcaster

### Parameters

- **joints**: the list of all joint
- **second_encoders**: list of joint equipped with second encoder
- **performance_index**: If true will be open also the low level communication performance indexes

### Topic

`state_broadcaster/joints_state`: Message type `pi3hat_moteus_int_msgs/JointStates`.

- Joints Name, list of joints name
- Positions measured
- Velocities measured
- Torques measured
- Current measured
- Driver Temperature measured
- Second Encoder Position, if needed
- Second Encoder Velocity, if needed

`state_broadcaster/performance_indexes`: Message type `pi3hat_moteus_int_msgs/PacketPass`.

- Pi3Hat Cycle time
- Cycle Validation, if one then the system has lost a cycle
- Joints name list
- Joints Packet loss, if one then the low level communication has lost a motor package

## Launch Interface

Set up the robot terminal:

- go into the mulinex_ws folder and make the global and local ros2 source
  ```shell
  cd mulinex_ws
  source /opt/ros/$ROS_DISTRO/setup.bash
  source install/setup.bash
  ```

If a communication with opc are needed also the ROS_DOMAIN_ID must be set, the id value must be the same for both robot and OPC.
```shell
export ROS_DOMAIN_ID=<ID>
```

To launch the interface use the command:
```shell
ros2 launch pi3hat_hw_interface start_MJBots_Pi3Hat_hw_Interface.launch.py urdf_file:=<filename> conf_file:=<filename>
```
the configuration and urdf file must be contained respectively in pi3hat_hw_interface/config and pi3hat_hw_interface/urdf.

## Safety Layer

The hardware interface implements a safety layer that monitors motor driver temperatures and battery voltage during the `read()` cycle. When thresholds are exceeded the system transitions through a state machine and stops the actuators before returning an error to the controller manager.

### Safety State Machine

```
NORMAL ──(temp > warning)──► WARNING ──(temp > critical OR volt < min)──► CRITICAL ──(delay elapsed)──► SHUTDOWN
  ▲                              │                                              │
  └──(temp < warning)────────────┘                                              └── write() sends MakeStop() to all actuators
```

| State | Value | Behaviour |
|-------|-------|-----------|
| `NORMAL` | 0 | All checks pass, normal operation |
| `WARNING` | 1 | Temperature elevated; throttled log at ~1 Hz; commands still sent |
| `CRITICAL` | 2 | Threshold violated; `MakeStop()` sent every cycle; shutdown timer starts |
| `SHUTDOWN` | 3 | `shutdown_delay` elapsed; `write()` keeps sending `MakeStop()`; `read()` returns `ERROR` |

The system can recover from `WARNING` back to `NORMAL` if temperature drops below the warning threshold. `CRITICAL` and `SHUTDOWN` require a system restart.

### Monitored Quantities

- **Driver temperature** — FET temperature reported by each Moteus driver, smoothed with an exponential moving average (EMA, α ≈ 0.004, ~1 s window at 500 Hz). The **maximum** across all actuators is compared against the thresholds.
- **Battery voltage** — output voltage reported by each power distributor, smoothed with the same EMA. The **minimum** across all distributors is compared against the threshold.

### Configuration Parameters (URDF `<hardware>` block)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `temp_warning_threshold` | 50 | °C | Temperature that triggers `WARNING` |
| `temp_critical_threshold` | 57 | °C | Temperature that triggers `CRITICAL` |
| `battery_min_voltage` | 24.0 | V | Voltage below which `CRITICAL` is triggered |
| `shutdown_delay` | 3.0 | s | Time spent in `CRITICAL` before transitioning to `SHUTDOWN` |

Example (Omnicar defaults):

```xml
<param name="temp_warning_threshold">50</param>
<param name="temp_critical_threshold">57</param>
<param name="battery_min_voltage">24.0</param>
<param name="shutdown_delay">3.0</param>
```

### Safety State Interface

The current safety state is exposed as a hardware state interface (`Pi3Hat/safety_state`, type `double`) with the numeric value of the enum (0–3). Controllers and broadcasters can read this interface to monitor the safety status at runtime.

## References

- [Moteus Dirver](https://github.com/mjbots/moteus/blob/main/docs/reference.md)
- [Pi3hat Board](https://github.com/mjbots/pi3hat)
