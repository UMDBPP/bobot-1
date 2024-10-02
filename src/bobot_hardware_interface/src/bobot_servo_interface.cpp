// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#include "bobot_hardware_interface/bobot_servo_interface.hpp"

namespace bobot_hardware
{

    BobotServoInterface::BobotServoInterface()
    {
        
    }

    // Open the serial connection and talk tuah (its over) the servos
    bool BobotServoInterface::open_serial_connection()
    {

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
