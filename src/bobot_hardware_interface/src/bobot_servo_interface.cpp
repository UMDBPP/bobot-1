// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include "bobot_hardware_interface/bobot_servo_interface.hpp"

/* CODE FOUND AT https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/ */

namespace bobot_hardware
{

    BobotServoInterface::BobotServoInterface()
    {
        // set some flags
        this->tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        this->tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        this->tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
        this->tty.c_cflag |= CS8; // 8 bits per byte (most common)
        this->tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        this->tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        this->tty.c_lflag &= ~ICANON;
        this->tty.c_lflag &= ~ECHO; // Disable echo
        this->tty.c_lflag &= ~ECHOE; // Disable erasure
        this->tty.c_lflag &= ~ECHONL; // Disable new-line echo
        this->tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        this->tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        this->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        this->tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        this->tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        // set timeout and data maximum
        this->tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        this->tty.c_cc[VMIN] = 0;   

        // Set in/out baud rate to be 9600
        cfsetispeed(&this->tty, B9600);
        cfsetospeed(&this->tty, B9600);

        // Save tty settings, also checking for error
        if (tcsetattr(this->serial_port, TCSANOW, &this->tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return;
        }

        // Normally you wouldn't do this memset() call, but since we will just receive
        // ASCII data for this example, we'll set everything to 0 so we can
        // call printf() easily.
        memset(&this->read_buf, '\0', sizeof(this->read_buf));

        // Hardcode this for now, make it better later?
        // initialize the servo vectors
        for(int i=0;i<2;i+=1)
        {
            this->servo_positions.push_back(0.0);
        }

    }

    // Open the serial connection and talk tuah (its over) the servos
    bool BobotServoInterface::open_serial_connection()
    {
        // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
        int serial_port = open("/dev/ttyACM0", O_RDWR);

        // Read in existing settings, and handle any error
        if(tcgetattr(serial_port, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }
        return true;
    }

    // Close the serial connection
    bool BobotServoInterface::close_serial_connection()
    {
        // Close the serial port
        close(this->serial_port);
        return true;
    }

    void BobotServoInterface::read_serial()
    {
        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if (num_bytes < 0) 
        {
            RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "Error reading: %s", strerror(errno));
            return;
        }

        // Do all the serial parsing right now
        // pass the read buff by value, not by reference
        this->parse_serial_data(read_buf);
    }

    void parse_serial_data(char read_buff[8])
    {
        // TODO
    }

    // Send a position command to the servo (only one servo at a time)
    void BobotServoInterface::command_position(int servoID, double command_position)
    {
        // Write to serial port
        std::string servoID_string = std::to_string(servoID);
        std::string command_position_string = std::to_string(command_position);
        std::string final_command = "SET0" + servoID_string + " " + command_position_string + "\n";
        write(serial_port, final_command.c_str(), sizeof(final_command.c_str()));
    }

    // Request position data from the servo (only one servo at a time)
    double BobotServoInterface::request_position(int servoID)
    {
        // Write to serial port
        std::string servoID_string = std::to_string(servoID);
        std::string final_command = "SET0" + servoID_string  + "\n";
        write(serial_port, final_command.c_str(), sizeof(final_command.c_str()));
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