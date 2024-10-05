// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include "bobot_hardware_interface/bobot_servo_interface.hpp"

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

    BobotServoInterface::BobotServoInterface()
    {
        
    }

    // Open the serial connection and talk tuah (its over) the servos
    bool BobotServoInterface::open_serial_connection()
    {
        // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
        int serial_port = open("/dev/ttyACM0", O_RDWR);

        // Read in existing settings, and handle any error
        if(tcgetattr(serial_port, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return 1;
        }

        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
        tty.c_cflag |= CS8; // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        
    }

    // Close the serial connection
    bool BobotServoInterface::close_serial_connection()
    {

    }

    // Send a position command to the servo (only one servo at a time)
    void BobotServoInterface::command_position(int servoID, double command_position)
    {

    }

    // Request position data from the servo (only one servo at a time)
    double BobotServoInterface::request_position(int servoID)
    {

    }

    // command multiple position all at once
    void BobotServoInterface::command_positions()
    {
        // TODO
    }

    // request multiple positions all at once
    void BobotServoInterface::request_positions(std::vector<double> joint_positions)
    {
        // TODO
    }

};
