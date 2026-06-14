# RosBridge_V2

An ESP32-based omniwheel robot firmware and host-side serial bridge for ROS-style command and odometry exchange.

## Overview

This repository contains two main components:

- `src/main.cpp`: Arduino/ESP32 firmware for an omni-directional robot base. It reads serialized `cmd_vel` commands from USB serial, controls three wheel motors with PID loops, and publishes odometry velocities back to the host.
- `receiver.py`: Python serial bridge and monitor for the robot. It can run in interactive teleop mode, headless monitor mode, or ROS bridge mode using `rospy`.

## Repository Structure

- `platformio.ini` - PlatformIO configuration for the `esp32doit-devkit-v1` board.
- `src/` - Arduino firmware source.
- `lib/` - Project-specific libraries for kinematics and PID control.
- `include/` - Header files used by the firmware.
- `receiver.py` - Python host bridge and command interface.

## Firmware Behavior

The ESP32 firmware:

- accepts serial commands in CSV format: `linear_x,linear_y,angular_z\n`
- computes wheel speeds using omniwheel kinematics
- runs PID control loops for each wheel
- sends back odometry as CSV: `vx,vy,wz\n`
- prints debug messages prefixed with `[DBG]` when debug mode is enabled

## Python Bridge

`receiver.py` provides:

- serial port management and background TX/RX threads
- keyboard teleop controls for manual driving
- optional ROS bridge mode to publish/subscribe ROS messages
- serial port discovery with `--list`

## Usage

### Build and upload firmware

From the repository root:

```bash
platformio run --target upload
```

### Run the Python bridge

List ports:

```bash
python3 receiver.py --list
```

Default monitor mode:

```bash
python3 receiver.py --port /dev/ttyUSB0
```

Keyboard teleop mode:

```bash
python3 receiver.py --port /dev/ttyUSB0 --teleop
```

ROS bridge mode:

```bash
python3 receiver.py --port /dev/ttyUSB0 --ros
```

## Requirements

- PlatformIO with `espressif32` platform for building the firmware
- Python 3
- `pyserial` for serial communication
- `rospy` if using `--ros`

## Notes

- The firmware and bridge communicate at `115200` baud by default.
- The command rate is synchronized to `COMMAND_RATE` in `src/main.cpp`.
- The bridge discards Arduino debug lines beginning with `[DBG]` and only parses odometry CSV lines.
