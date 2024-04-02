# KWR75 Force Sensor ROS2

This repository contains a ROS2 driver for reading data from the Six Axis Force/Torque Sensor KWR75 Series integrated with ros2_control.

Currently tested on Ubuntu 22.04 and ROS2 Humble only.

## Installation

1. Source ROS2 Humble:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Clone the repository into your ROS2 workspace:
   ```bash
   cd /path/to/your_ros2_ws/src
   git clone -b humble https://github.com/ian-chuang/kwr75_force_sensor_ros2.git
   ```

3. Install dependencies using rosdep:
   ```bash
   rosdep install -i --from-path . --rosdistro humble -y
   ```

4. Build the package:
   ```bash
   colcon build
   ```

5. Source the setup files:
   ```bash
   source /path/to/your_ros2_ws/install/local_setup.bash
   ```

## Configuration

1. Find the port that the sensor is currently binding to, e.g., `ttyUSB0`.
2. Run the following command to obtain the serial number:
   ```bash
   udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial
   ```
   Use the first serial number that shows up; the format should look similar to `FT6S4DSP`.

3. Edit the udev rules file:
   ```bash
   sudo nano /etc/udev/rules.d/99-kwr75-force-sensor.rules
   ```
   Add the following line, replacing `<serial number here>` with your sensor's serial number:
   ```bash
   SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="robot/kwr75_force_sensor"
   ```

4. Apply the changes:
   ```bash
   sudo udevadm control --reload && sudo udevadm trigger
   ```

## Usage

Launch the driver with the following command:
```bash
ros2 launch kwr75_force_sensor_ros2 kwr75_control.launch.py com_port:=/dev/robot/kwr75_force_sensor
```

To view the URDF, use:
```bash
ros2 launch kwr75_force_sensor_ros2 view_kwr75.launch.py
```

## How it Works

This driver utilizes the serial_driver package from [ros-drivers](https://github.com/ros-drivers/transport_drivers) to interface with the sensor via serial communication. The code follows the datasheet provided [here](https://github.com/ian-chuang/kwr75_force_sensor_ros2/blob/humble/KWR75-RS422.pdf). It starts by sending the start command `0x48 AA 0D 0A`, which commands the sensor to send data at 1 kHz. Then, it starts a thread to continuously read data from the sensor and convert it into force-torque values as specified in the datasheet.