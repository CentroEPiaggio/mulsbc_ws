# Mulinex SBC ROS 2 Workspace

Workspace for ROS 2 packages running on the Mulinex robot's on-board SBC (Raspberry Pi 4 + MJBots Pi3Hat).

## Docker

The docker image is for ARM64 development on x86 hosts via QEMU emulation.

Build, start, enter, and stop the container with `make build`, `make start`, `make enter`, and `make stop`, respectively.

## Pi3hat Robotic Systems

[Pi3hat Robotic Systems](src/pi3hat/README.md)

## Launch

```bash
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py \
  urdf_file:=<urdf_xacro> \
  conf_file:=<config_yaml>
```

Defaults are `omnicar.xacro` and `omnicar.yaml`. URDF files are in `pi3hat_hw_interface/urdf/`, config YAMLs in `pi3hat_hw_interface/config/`.
