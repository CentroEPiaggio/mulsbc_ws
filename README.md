# Mulinex SBC ROS 2 Workspace

Workspace for ROS 2 packages running on the Mulinex robot's on-board SBC (Raspberry Pi 4 + MJBots Pi3Hat).

## Pi3hat Robotic Systems

[Pi3hat Robotic Systems](src/pi3hat/README.md)

## Launch

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=<urdf_xacro> \
  conf_file:=<config_yaml>
```

For example, to launch the Omnicar configuration:

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=Omnicar.xacro \
  conf_file:=Omnicar.yaml
```

Defaults are `JumpingLeg2d.xacro` and `jump_leg.yaml`. URDF files are in `pi3hat_hw_interface/urdf/`, config YAMLs in `pi3hat_hw_interface/config/`.