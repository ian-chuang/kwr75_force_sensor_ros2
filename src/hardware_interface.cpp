#include "kwr75_force_sensor_ros2/hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kwr75_force_sensor
{
hardware_interface::CallbackReturn KWR75ForceSensorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  com_port = info_.hardware_parameters["com_port"];
  uint32_t baud_rate = stod(info_.hardware_parameters["baud_rate"]);

  drivers::serial_driver::FlowControl fc = drivers::serial_driver::FlowControl::NONE;
  drivers::serial_driver::Parity pt = drivers::serial_driver::Parity::NONE;
  drivers::serial_driver::StopBits sb = drivers::serial_driver::StopBits::ONE;
  ctx = std::make_shared<IoContext>(2);
  config = std::make_shared<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  driver = std::make_shared<drivers::serial_driver::SerialDriver>(*ctx);

  hw_sensor_states.resize(
    info_.sensors[0].state_interfaces.size(), 0.0);
  data.resize(DATA_SIZE, 0);
  received_data_buffer.resize(0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KWR75ForceSensorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states[i]));
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn KWR75ForceSensorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    driver->init_port(com_port.c_str(), *config);
    if (!driver->port()->is_open()) {
      driver->port()->open();
    }

    driver->port()->async_receive(
        std::bind(
          &KWR75ForceSensorHardwareInterface::receive_callback, this, std::placeholders::_1,
          std::placeholders::_2));

    driver->port()->send(COMMAND);

    last_good_call_time_ = std::chrono::steady_clock::now();

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KWR75ForceSensorHardwareInterface"),
      "Error creating serial port: %s - %s",
      com_port.c_str(), ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("KWR75ForceSensorHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KWR75ForceSensorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver->port()->close();

  RCLCPP_INFO(
    rclcpp::get_logger("KWR75ForceSensorHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KWR75ForceSensorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

void KWR75ForceSensorHardwareInterface::receive_callback(
  const std::vector<uint8_t> & buffer,
  const size_t & bytes_transferred)
{

  // Append received data to the buffer
  received_data_buffer.insert(received_data_buffer.end(), buffer.begin(), buffer.begin() + bytes_transferred);

  // Define the start and end markers
  const uint8_t start_marker[] = {0x48, 0xAA};
  const uint8_t end_marker[] = {0x0D, 0x0A};

  // Search for the start and end markers in the buffer
  auto start_pos = std::search(received_data_buffer.begin(), received_data_buffer.end(), start_marker, start_marker + 2);
  auto end_pos = std::search(received_data_buffer.begin(), received_data_buffer.end(), end_marker, end_marker + 2);

  // Check if both start and end markers are found
  if (start_pos != received_data_buffer.end() && end_pos != received_data_buffer.end()) {
    // Calculate the length of the message
    size_t message_length = std::distance(start_pos, end_pos) + 2;

    // Check if we have received a complete message
    if (message_length <= DATA_SIZE) {
      std::vector<uint8_t> good_data(DATA_SIZE, 0);

      // Copy the message to the data vector
      std::copy(start_pos, start_pos + message_length, good_data.begin());

      // print the received data
      for (size_t i = 0; i < DATA_SIZE; i++) {
        std::cout << std::hex << static_cast<int>(good_data[i]) << " ";
      }
      std::cout << std::endl;

      // get duration
      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_good_call_time_).count();
      last_good_call_time_ = now;
      // print out the duration
      std::cout << "Duration: " << std::dec << duration << " ns" << std::endl;

      // Remove the processed message from the buffer
      received_data_buffer.erase(received_data_buffer.begin(), start_pos + message_length);
    } else {
      // If the message length is greater than the data vector size, discard the buffer
      received_data_buffer.clear();
    }
  }

}

}  // namespace kwr75_force_sensor

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kwr75_force_sensor::KWR75ForceSensorHardwareInterface,
  hardware_interface::SensorInterface)