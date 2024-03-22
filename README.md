# KWR75 Force Torque Sensor ROS Driver



sudo nano /etc/udev/rules.d/99-kwr75-force-sensor.rules
KERNEL=="ttyUSB[0-9]*", ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout", SYMLINK+="robot/kwr75_force_sensor" ATTR{latency_timer}="1"

If you want to add additional devices and don’t know their vendor or product IDs, you can use the command
sudo udevadm info --name=<your_device_name> --attribute-walk



This repository contains a simple ROS driver for reading data from the KWR75 Force Torque Sensor. The driver is implemented in Python and utilizes the `pyserial` library to establish communication with the KWR75 Force Torque Sensor.

## Installation

Before using the driver, make sure you have the `pyserial` library installed. You can install it using the following command:

```bash
pip install pyserial
```

To use the KWR75 Force Torque Sensor ROS driver, follow these steps:

1. Clone the repository into your ROS workspace's source directory:

```bash
cd /path/to/your/workspace/src
git clone https://github.com/ian-chuang/KWR75-Force-Sensor-ROS.git
```

2. Navigate to your ROS workspace and build the packages:

```bash
cd /path/to/your/workspace
catkin_make
```

## Usage

To use the KWR75 Force Torque Sensor ROS driver, follow these steps:

1. Launch the driver using the provided launch file:

```bash
roslaunch kwr75_force_sensor kwr75_bringup.launch
```

2. The wrench message from the KWR75 Force Torque Sensor should now be published on the topic `kwr75/wrench`.

3. You can access the data being published using the `rostopic echo` command:

```bash
rostopic echo kwr75/wrench
```

This will display the force and torque data from the sensor in the terminal.

Feel free to incorporate this driver into your ROS projects that require force torque sensing with the KWR75 sensor.

Please note that this driver is a basic implementation and may require modifications to suit specific use cases or additional functionalities.

## Contributing

If you find any issues or have improvements to suggest, feel free to open an issue or submit a pull request on the [GitHub repository](https://github.com/ian-chuang/KWR75-Force-Sensor-ROS).

## License

This project is licensed under the [MIT License](LICENSE). Feel free to use and modify the code according to the terms of the license.
