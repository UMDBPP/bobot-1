// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

#ifndef BOBOT_HARDWARE_INTERFACE__BOBOT_ALTIMETER_INTERFACE_HPP
#define BOBOT_HARDWARE_INTERFACE__BOBOT_ALTIMETER_INTERFACE_HPP


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

    // Request position data from the servo (only one servo at a time)
    double request_altitude();

};



};

#endif /* BOBOT_HARDWARE_INTERFACE__BOBOT_ALTIMETER_INTERFACE_HPP */
