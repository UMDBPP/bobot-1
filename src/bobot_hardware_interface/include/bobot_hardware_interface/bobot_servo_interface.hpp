// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#ifndef BOBOT_HARDWARE_INTERFACE__BOBOT_SERVO_INTERFACE_HPP
#define BOBOT_HARDWARE_INTERFACE__BOBOT_SERVO_INTERFACE_HPP

#include <map>
#include <vector>

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

    // Send a position command to the servo (only one servo at a time)
    void command_position(int servoID, double command_position);

    // Request position data from the servo (only one servo at a time)
    double request_position(int servoID);

    // command multiple position all at once
    void command_positions();

    // request multiple positions all at once
    void request_positions(std::vector<double> joint_positions);

};



};

#endif /* BOBOT_HARDWARE_INTERFACE__BOBOT_SERVO_INTERFACE_HPP */
