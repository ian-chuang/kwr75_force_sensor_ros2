// Force Torque Sensor Interface Node

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <cstring>

class ForceTorqueSensorNode : public rclcpp::Node
{
public:
  ForceTorqueSensorNode() : Node("force_torque_sensor_node")
  {
    // Open serial port
    serial_port.setPort("/dev/ttyUSB0"); // Replace with appropriate port name
    serial_port.setBaudrate(460800);
    serial_port.open();

    // Send command to start data transmission
    char command[] = {0x49, 0xAA, 0x0D, 0x0A};
    serial_port.write(command, sizeof(command));

    // Create timer to read data
    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&ForceTorqueSensorNode::readData, this));
  }

private:
  void readData()
  {
    // Read data from serial port
    uint8_t buffer[28];
    size_t bytes_read = serial_port.read(buffer, sizeof(buffer));

    if (bytes_read == 28)
    {
      // Parse data according to instructions
      if (buffer[0] == 0x49 && buffer[1] == 0xAA)
      {
        float fx, fy, fz, mx, my, mz;
        uint8_t* ptr = buffer + 2;

        memcpy(&fx, ptr, sizeof(float));
        ptr += sizeof(float);
        memcpy(&fy, ptr, sizeof(float));
        ptr += sizeof(float);
        memcpy(&fz, ptr, sizeof(float));
        ptr += sizeof(float);
        memcpy(&mx, ptr, sizeof(float));
        ptr += sizeof(float);
        memcpy(&my, ptr, sizeof(float));
        ptr += sizeof(float);
        memcpy(&mz, ptr, sizeof(float));

        // Do something with the parsed data
        RCLCPP_INFO(get_logger(), "Fx: %f, Fy: %f, Fz: %f, Mx: %f, My: %f, Mz: %f", fx, fy, fz, mx, my, mz);
      }
    }
  }

  serial::Serial serial_port;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForceTorqueSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}