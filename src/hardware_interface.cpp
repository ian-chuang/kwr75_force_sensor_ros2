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

KWR75ForceSensorHardwareInterface::~KWR75ForceSensorHardwareInterface()
{
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_cleanup(rclcpp_lifecycle::State());
}


hardware_interface::CallbackReturn KWR75ForceSensorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize the sensor states
  hw_sensor_states.resize(
    info_.sensors[0].state_interfaces.size(), 0.0);

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

hardware_interface::CallbackReturn
KWR75ForceSensorHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // setup serial driver
  com_port = info_.hardware_parameters["com_port"];
  uint32_t baud_rate = stod(info_.hardware_parameters["baud_rate"]);
  drivers::serial_driver::FlowControl fc = drivers::serial_driver::FlowControl::NONE;
  drivers::serial_driver::Parity pt = drivers::serial_driver::Parity::NONE;
  drivers::serial_driver::StopBits sb = drivers::serial_driver::StopBits::ONE;
  ctx = std::make_shared<IoContext>(2);
  config = std::make_shared<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  driver = std::make_shared<drivers::serial_driver::SerialDriver>(*ctx);

  // Initialize the data vector and the received data buffer
  data.resize(DATA_SIZE, 0);
  received_data_buffer.resize(0);

  try {
    // open port and start async receive
    driver->init_port(com_port.c_str(), *config);
    if (!driver->port()->is_open()) {
      driver->port()->open();
      driver->port()->async_receive(
        std::bind(
          &KWR75ForceSensorHardwareInterface::receive_callback, this, std::placeholders::_1,
          std::placeholders::_2));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KWR75ForceSensorHardwareInterface"),
      "Error creating serial port: %s - %s",
      com_port.c_str(), ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("KWR75ForceSensorHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KWR75ForceSensorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::unique_lock<std::mutex> time_lock(time_mutex);
  auto current_receive_time = last_receive_time;
  time_lock.unlock();
  std::chrono::steady_clock::time_point new_receive_time = current_receive_time;
  auto start_time = std::chrono::steady_clock::now();

  while (current_receive_time == new_receive_time){
    // Send the command to start data conversion
    driver->port()->send(START_COMMAND);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::unique_lock<std::mutex> time_lock(time_mutex);
    new_receive_time = last_receive_time;
    time_lock.unlock();

    if (std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time).count() > 1)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("KWR75ForceSensorHardwareInterface"),
        "No data received from the sensor for more than 1 second!");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } 

  RCLCPP_INFO(
    rclcpp::get_logger("KWR75ForceSensorHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KWR75ForceSensorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver->port()->send(STOP_COMMAND);

  RCLCPP_INFO(
    rclcpp::get_logger("KWR75ForceSensorHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
KWR75ForceSensorHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  driver->port()->send(STOP_COMMAND);
  driver->port()->close();

  RCLCPP_INFO(rclcpp::get_logger("KWR75ForceSensorHardwareInterface"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KWR75ForceSensorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::unique_lock<std::mutex> data_lock(data_mutex);
  float fx = *reinterpret_cast<float*>(&data[2]);
  float fy = *reinterpret_cast<float*>(&data[6]);
  float fz = *reinterpret_cast<float*>(&data[10]);
  float mx = *reinterpret_cast<float*>(&data[14]);
  float my = *reinterpret_cast<float*>(&data[18]);
  float mz = *reinterpret_cast<float*>(&data[22]);
  data_lock.unlock();

  hw_sensor_states[0] = fx;
  hw_sensor_states[1] = fy;
  hw_sensor_states[2] = fz;
  hw_sensor_states[3] = mx;
  hw_sensor_states[4] = my;
  hw_sensor_states[5] = mz;
  
  std::unique_lock<std::mutex> time_lock(time_mutex);
  std::chrono::steady_clock::time_point last_receive = last_receive_time;
  time_lock.unlock();

  if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - last_receive).count() > 1)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("KWR75ForceSensorHardwareInterface"),
      "No data received from the sensor for more than 1 second!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void KWR75ForceSensorHardwareInterface::receive_callback(
  const std::vector<uint8_t> & buffer,
  const size_t & bytes_transferred)
{
  std::unique_lock<std::mutex> time_lock(time_mutex);
  last_receive_time = std::chrono::steady_clock::now();
  time_lock.unlock();

  // Append received data to the buffer
  received_data_buffer.insert(received_data_buffer.end(), buffer.begin(), buffer.begin() + bytes_transferred);

  // Define the start and end markers
  const uint8_t start_marker[] = {0x48, 0xAA};
  const uint8_t end_marker[] = {0x0D, 0x0A};

  // Find the last occurrence of the end marker
  auto end_pos = std::find_end(received_data_buffer.begin(), received_data_buffer.end(), end_marker, end_marker + 2);

  // Check if the end marker is found
  if (end_pos != received_data_buffer.end()) {
    
    // Search for the start marker within the range before the end marker
    auto start_pos = std::find_end(received_data_buffer.begin(), end_pos, start_marker, start_marker + 2);

    // Check if the start marker is found
    if (start_pos != end_pos) {
      // Calculate the length of the message
      size_t message_length = std::distance(start_pos, end_pos) + 2;

      // Check if we have received a complete message
      if (message_length == DATA_SIZE) {
        // Copy the message to the data vector
        std::unique_lock<std::mutex> data_lock(data_mutex);
        std::copy(start_pos, start_pos + message_length, data.begin());
        data_lock.unlock();
      } 
    }

    // clear anything before the end marker
    received_data_buffer.erase(received_data_buffer.begin(), end_pos + 2);

  } else if (received_data_buffer.size() > 2 * DATA_SIZE) {
    // If the buffer is too long and not containing the end marker, clear the buffer
    received_data_buffer.clear();
  }
}

}  // namespace kwr75_force_sensor

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kwr75_force_sensor::KWR75ForceSensorHardwareInterface,
  hardware_interface::SensorInterface)