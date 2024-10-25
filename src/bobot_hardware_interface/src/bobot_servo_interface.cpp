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
        // Normally you wouldn't do this memset() call, but since we will just receive
        // ASCII data for this example, we'll set everything to 0 so we can
        // call printf() easily.
        // memset(&this->read_buf, '\0', sizeof(this->read_buf));

        // Hardcode this for now, make it better later?
        // initialize the servo vectors
        for(int i=0;i<3;i+=1)
        {
            this->servo_positions.push_back(0.0);
        }
    }

    BobotServoInterface::~BobotServoInterface() = default;

    // Open the serial connection and talk tuah (its over) the servos
    bool BobotServoInterface::open_serial_connection()
    {
        // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
        serial_port = open("/dev/ttyACM0", O_RDWR);

        // Read in existing settings, and handle any error
        if(tcgetattr(serial_port, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }

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

        // Set in/out baud rate to be 115200
        cfsetispeed(&this->tty, B115200);
        cfsetospeed(&this->tty, B115200);

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return false;
        }
        return true;
    }

    // Close the serial connection
    bool BobotServoInterface::close_serial_connection()
    {
        // Close the serial port
        close(serial_port);
        return true;
    }

    void BobotServoInterface::read_serial()
    {
        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME

        // Allocate memory for read buffer, set size according to your needs
        uint8_t* read_buf = new uint8_t[2]; // 1 byte for identity, 1 byte for data
        int num_bytes = read(serial_port, read_buf, 2); // always expecting bytes of size 2 for servos

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if(num_bytes < 0) 
        {
            RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "Error reading: %s", strerror(errno));
            return;
        }
        if(num_bytes == 2) // expected number of bytes for servo data
     	{
            if(read_buf[0] == 1)
            {
                this->servo_positions[0] = read_buf[1];
            }
            else if(read_buf[0] == 2)
            {
                this->servo_positions[1] = read_buf[1];
            }
        }
        delete[] read_buf;
    }

    // Send a position command to the servo (only one servo at a time)
    void BobotServoInterface::command_position(uint8_t servoID, uint8_t command_position)
    {
        // Write to serial port
        uint8_t* command = new uint8_t[3];
        command[0] = 1; // 1 specifies "SET" command
        command[1] = servoID;
        command[2] = command_position;
        write(serial_port, command, 3);
        delete[] command;
    }

    // Request position data from the servo (only one servo at a time)
    void BobotServoInterface::request_position(uint8_t servoID)
    {
        // Write to serial port
        uint8_t* req_command = new uint8_t[3]; // needs to be same size
        req_command[0] = 2; // 2 specifices "GET" command
        req_command[1] = servoID;
        req_command[2] = 0; // unused
        write(serial_port, req_command, 3);
        delete[] req_command;
    }

    // // command multiple position all at once
    // void BobotServoInterface::command_positions()
    // {
    //     // TODO
    // }

    // // request multiple positions all at once
    // void BobotServoInterface::request_positions(std::vector<double> joint_positions)
    // {
    //     // TODO
    // }

};
