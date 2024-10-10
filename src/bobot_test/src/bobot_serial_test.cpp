#include "bobot_hardware_interface/bobot_servo_interface.hpp"
#include "bobot_hardware_interface/bobot_altimeter_interface.hpp"


int main()
{
    bobot_hardware::BobotServoInterface bobot_servo;
    bobot_servo.open_serial_connection();
}