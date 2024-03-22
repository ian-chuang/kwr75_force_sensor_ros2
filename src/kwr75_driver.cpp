#include <iostream>
#include "SerialPort.h"

// Define constants
const int BAUD_RATE = 460800;
const char COMMAND_START = 0x48; // Command code for starting data conversion

// Function to convert bytes to float according to the specified format
float bytesToFloat(const unsigned char* bytes) {
    float value;
    memcpy(&value, bytes, sizeof(float));
    return value;
}

int main() {
    // Initialize the serial port
    SerialPort serial("/dev/ttyUSB0"); // Change this to your actual serial port
    serial.Open(BAUD_RATE);

    if (!serial.IsOpen()) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return 1;
    }

    // Send the command to start data conversion
    unsigned char command[] = { COMMAND_START, 0xAA, 0x0D, 0x0A };
    serial.Write(command, sizeof(command));

    // Read and process data from the sensor
    const int DATA_SIZE = 28;
    unsigned char data[DATA_SIZE];
    while (true) {
        // Read a frame of data (28 bytes)
        serial.Read(data, DATA_SIZE);

        // Check if the command code and fixed identification are correct
        if (data[0] == COMMAND_START && data[1] == 0xAA) {
            // Extract and convert force and moment components
            float fx = bytesToFloat(data + 2);
            float fy = bytesToFloat(data + 6);
            float fz = bytesToFloat(data + 10);
            float mx = bytesToFloat(data + 14);
            float my = bytesToFloat(data + 18);
            float mz = bytesToFloat(data + 22);

            // Output the converted data
            std::cout << "Fx: " << fx << " Kg" << std::endl;
            std::cout << "Fy: " << fy << " Kg" << std::endl;
            std::cout << "Fz: " << fz << " Kg" << std::endl;
            std::cout << "Mx: " << mx << " Kgm" << std::endl;
            std::cout << "My: " << my << " Kgm" << std::endl;
            std::cout << "Mz: " << mz << " Kgm" << std::endl;
        }
    }

    // Close the serial port
    serial.Close();

    return 0;
}
