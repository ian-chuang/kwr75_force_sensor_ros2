#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "geometry_msgs/msg/wrench.hpp"

class WrenchPublisher : public rclcpp::Node {
public:
    WrenchPublisher() : Node("wrench_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("wrench_topic", 10);
    }

    void publishWrench(float fx, float fy, float fz, float mx, float my, float mz) {
        auto wrench_msg = std::make_unique<geometry_msgs::msg::Wrench>();
        wrench_msg->force.x = fx;
        wrench_msg->force.y = fy;
        wrench_msg->force.z = fz;
        wrench_msg->torque.x = mx;
        wrench_msg->torque.y = my;
        wrench_msg->torque.z = mz;
        publisher_->publish(std::move(wrench_msg));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<WrenchPublisher>();

    const char * dev_name = "/dev/robot/kwr75_force_sensor";
    uint32_t baud = 460800;
    drivers::serial_driver::FlowControl fc = drivers::serial_driver::FlowControl::NONE;
    drivers::serial_driver::Parity pt = drivers::serial_driver::Parity::NONE;
    drivers::serial_driver::StopBits sb = drivers::serial_driver::StopBits::ONE;
    IoContext ctx(2);
    drivers::serial_driver::SerialPortConfig config(baud, fc, pt, sb);
    drivers::serial_driver::SerialDriver driver(ctx);

    driver.init_port(dev_name, config);
    if (!driver.port()->is_open()) {
      driver.port()->open();
    }

    // Send the command to start data conversion
    std::vector<uint8_t> command = { 0x49, 0xAA, 0x0D, 0x0A };

    // Read and process data from the sensor
    const int DATA_SIZE = 28;
    //create uint_8 vector of zeros
    std::vector<uint8_t> data(DATA_SIZE, 0);
    while (rclcpp::ok()) {
        // Read a frame of data (28 bytes)
        driver.port()->send(command);
        driver.port()->receive(data);

        if (data.size() != DATA_SIZE) {
            RCLCPP_ERROR(node->get_logger(), "Invalid data size: %d", static_cast<int>(data.size()));
            continue;
        }

        // Check if the command code and fixed identification are correct
        if (data[0] == 0x49 && data[1] == 0xAA) {
            // Extract and convert force and moment components
            float fx, fy, fz, mx, my, mz;

            // Convert and process the data according to the provided format
            fx = *reinterpret_cast<float*>(&data[2]);
            fy = *reinterpret_cast<float*>(&data[6]);
            fz = *reinterpret_cast<float*>(&data[10]);
            mx = *reinterpret_cast<float*>(&data[14]);
            my = *reinterpret_cast<float*>(&data[18]);
            mz = *reinterpret_cast<float*>(&data[22]);

            // Print the extracted values
            std::cout << "Fx: " << fx << " Kg\n"
                        << "Fy: " << fy << " Kg\n"
                        << "Fz: " << fz << " Kg\n"
                        << "Mx: " << mx << " Kgm\n"
                        << "My: " << my << " Kgm\n"
                        << "Mz: " << mz << " Kgm\n";
            
            node->publishWrench(fx, fy, fz, mx, my, mz);
        }
        else {
            RCLCPP_ERROR(node->get_logger(), "Invalid command code or fixed identification: %02X %02X", data[0], data[1]);
        }
    }

    // // Close the serial port
    // serial.Close();
    driver.port()->close();
    rclcpp::shutdown();
    return 0;
}