// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#ifndef BOBOT_HARDWARE_INTERFACE__BOBOT_SERVO_INTERFACE_HPP
#define BOBOT_HARDWARE_INTERFACE__BOBOT_SERVO_INTERFACE_HPP

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

class BobotServoInterface
{

public:
    BobotServoInterface();
    ~BobotServoInterface();

    // Open the serial connection and talk tuah (its over) the servos
    bool open_serial_connection();

    // Close the serial connection
    bool close_serial_connection();

    // Read
    void read_serial();

    // Send a position command to the servo (only one servo at a time)
    void command_position(uint8_t servoID, uint8_t command_position);

    // Request position data from the servo (only one servo at a time)
    void request_position(int servoID);

    // command multiple position all at once
    void command_positions();

    // request multiple positions all at once
    void request_positions(std::vector<double> joint_positions);

    std::vector<double> servo_positions;

private:

    

    // Integer to hold the serial port data
    int serial_port;

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Allocate memory for read buffer, set size according to your needs
    char read_buf [8]; // we really only need 8 

    // logger string
    std::string ros_logger_string = "BobotServoInterface";

};



};

#endif /* BOBOT_HARDWARE_INTERFACE__BOBOT_SERVO_INTERFACE_HPP */
