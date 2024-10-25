// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include "bobot_hardware_interface/bobot_altimeter_interface.hpp"

namespace bobot_hardware
{

    BobotAltimeterInterface::BobotAltimeterInterface()
    {
        // Normally you wouldn't do this memset() call, but since we will just receive
        // ASCII data for this example, we'll set everything to 0 so we can
        // call printf() easily.
        // memset(&this->read_buf, '\0', sizeof(this->read_buf));
    }

    BobotAltimeterInterface::~BobotAltimeterInterface() = default;

    // Open the serial connection and talk tuah (its over) the servos
    bool BobotAltimeterInterface::open_serial_connection()
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

        // Set in/out baud rate to be 9600
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
    bool BobotAltimeterInterface::close_serial_connection()
    {
        // Close the serial port
        close(serial_port);
        return true;
    }

    void BobotAltimeterInterface::read_serial()
    {
        // Read bytes. The behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME

        // Allocate memory for read buffer, set size according to your needs
        uint8_t* read_buf = new uint8_t[5]; // we should only need 5
        int num_bytes = read(serial_port, read_buf, 5);

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
        if(num_bytes <= 0) 
        {
            RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "Error reading: %s", strerror(errno));
            return;
        }

        if(num_bytes == 5) // note - need to change to whatever we decided on
        {
	   // RCLCPP_ERROR(rclcpp::get_logger(ros_logger_string), "BALLS %i, %i, %i, %i, %i", read_buf[0], read_buf[1],read_buf[2],read_buf[3],read_buf[4]);
            if(read_buf[0] == 5)
            {
                int32_t bit1_high = read_buf[4] << 24;
                int32_t bit2 = read_buf[3] << 16;
                int32_t bit3 = read_buf[2] << 8;
                int32_t bit4_low = read_buf[1];
                this->altitude_centimeters = bit1_high | bit2 | bit3 | bit4_low; // Get the altitude

                // int32_t bit = 0
                // for int(i=0;i<4;i+=1)
                // {
                //     bit = read_buf[i] << (8 * i);
                //     altitude_centimeters = altitude_centimeters | bit;
                //     bit = 0;
                // }
               this->altitude = altitude_centimeters/30.48; // convert centimeters to feet
            }
        }

        // free up the pointer
        delete[] read_buf;
    }

    // Request position data from the servo (only one servo at a time)
    void BobotAltimeterInterface::request_altitude()
    {
        // Write to serial port
        uint8_t* req_command = new uint8_t[3];
        req_command[0] = 2; // 2 specifies "GET"
        req_command[1] = 5; // 5 is the ID for the altimeter
        req_command[2] = 0; // unused
        write(serial_port, req_command, 3);
        delete[] req_command;
    }

};

