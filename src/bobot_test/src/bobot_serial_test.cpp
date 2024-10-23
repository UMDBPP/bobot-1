#include "bobot_hardware_interface/bobot_servo_interface.hpp"
#include "bobot_hardware_interface/bobot_altimeter_interface.hpp"
#include <thread>
#include <chrono>
#include <iostream>


int main()
{
    bobot_hardware::BobotServoInterface bobot_servo;
    // bobot_hardware::BobotAltimeterInterface bobot_altimeter;
    bobot_servo.open_serial_connection();
    // bobot_altimeter.open_serial_connection();
    int flipper = 1;
    while(true)
    {
        std::cout << "commanding!" << std::endl;
        if(flipper == 1)
        {
            bobot_servo.command_position(1, 5);
            flipper += 1;
        }
        else if(flipper == 2)
        {
            bobot_servo.command_position(1, 30);
            flipper += 1;
        }
        else if(flipper == 3)
        {
            bobot_servo.command_position(1, 60);
            flipper += 1;
        }
        else if(flipper == 4)
        {
            bobot_servo.command_position(1, 90);
            flipper += 1;
        }
        else if(flipper == 5)
        {
            bobot_servo.command_position(1, 120);
            flipper += 1;
        }
        else if(flipper == 6)
        {
            bobot_servo.command_position(1, 150);
            flipper += 1;
        }
        
        if(flipper == 6)
        {
            flipper = 1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));


    }
}