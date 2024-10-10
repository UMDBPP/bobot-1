#include "bobot_hardware_interface/bobot_servo_interface.hpp"
#include "bobot_hardware_interface/bobot_altimeter_interface.hpp"
#include <thread>
#include <chrono>
#include <iostream>


int main()
{
    bobot_hardware::BobotServoInterface bobot_servo;
    bobot_hardware::BobotAltimeterInterface bobot_altimeter;
    bobot_servo.open_serial_connection();
    bobot_altimeter.open_serial_connection();
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bobot_altimeter.request_altitude();
        bobot_altimeter.read_serial();
        std::cout << bobot_altimeter.altitude << std::endl;

    }
}