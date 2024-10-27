// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#ifndef BOBOT_HARDWARE_INTERFACE__BOBOT_ALTIMETER_INTERFACE_HPP
#define BOBOT_HARDWARE_INTERFACE__BOBOT_ALTIMETER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <vector>

/* CODE FOUND AT https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/ */
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

namespace bobot_hardware
{

class BobotAltimeterInterface
{

public:
    BobotAltimeterInterface();
    ~BobotAltimeterInterface();

    // Open the serial connection and talk tuah (its over) the servos
    bool open_serial_connection();

    // Close the serial connection
    bool close_serial_connection();

    void read_serial();

    // Request position data from the servo (only one servo at a time)
    void request_altitude();

    double altitude_feet = 0.0; // initialize with 0 incase the arduino fails during flight
    double altitude_centimeters = 0.0; // initialize with 0 incase the arduino fails during flight

private:
    // Integer to hold the serial port data
    int serial_port;

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // logger string
    std::string ros_logger_string = "BobotAltimeterInterface";

};



};

#endif /* BOBOT_HARDWARE_INTERFACE__BOBOT_ALTIMETER_INTERFACE_HPP */
