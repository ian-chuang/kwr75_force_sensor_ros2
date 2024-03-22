// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Subhas Das, Denis Stogl
//

#ifndef KWR75_FORCE_SENSOR_ROS2__HARDWARE_INTERFACE_HPP_
#define KWR75_FORCE_SENSOR_ROS2__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "serial_driver/serial_driver.hpp"

namespace kwr75_force_sensor
{

static const int DATA_SIZE = 28;

class KWR75ForceSensorHardwareInterface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KWR75ForceSensorHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  /// \breif Callback for when serial data are received
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);

  std::shared_ptr<IoContext> ctx;
  std::shared_ptr<drivers::serial_driver::SerialPortConfig> config;
  std::shared_ptr<drivers::serial_driver::SerialDriver> driver;
  std::string com_port;

  // Store the sensor states for the simulated robot
  std::vector<double> hw_sensor_states;
  std::vector<uint8_t> data;
  const std::vector<uint8_t> COMMAND = { 0x48, 0xAA, 0x0D, 0x0A };
  std::vector<uint8_t> received_data_buffer;
  std::chrono::steady_clock::time_point last_good_call_time_;

};

}  // namespace kwr75_force_sensor

#endif  // KWR75_FORCE_SENSOR_ROS2__HARDWARE_INTERFACE_HPP_