# Moteus Pi3Hat Library (moteus_pi3hat)

Low-level C++ library wrapping Pi3Hat CAN-FD communication with Moteus controllers.

## Overview

Header-only C++ library that provides the low-level interface between ROS 2 and the MJBots Pi3Hat board. It handles CAN-FD communication with Moteus motor drivers and IMU data acquisition.

This is a library package — it does not expose ROS 2 nodes, topics, or services. It is used internally by `pi3hat_hw_interface`.

## Dependencies

- `bcm_host` — Broadcom VideoCore library (Raspberry Pi only)
- `rclcpp` — ROS 2 C++ client library

## Raspberry Pi Setup

The `bcm_host` library must be accessible:

```bash
sudo ln /usr/lib/aarch64-linux-gnu/libbcm_host.so /usr/lib/libbcm_host.so.0
sudo apt install libraspberrypi-dev
```

## See Also

- [Pi3Hat Robotic Systems](../README.md) — Detailed Raspberry Pi setup and hardware configuration
- [Pi3Hat Hardware Interface](../pi3hat_hw_interface/README.md) — The ros2_control plugin that uses this library
