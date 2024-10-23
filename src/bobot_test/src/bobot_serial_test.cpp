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
    uint8_t count = 5;
    int direction = 1;
    std::cout << "Preparing to command!" << std::endl;
    bobot_servo.command_position(1,5);
    bobot_servo.command_position(2,5);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    while(true)
    {
        std::cout << count*1.0 << std::endl;
	bobot_servo.command_position(1,count);
      	bobot_servo.command_position(2,count);
	count += (1*direction);
	if(count == 175 || count == 5)
	{
	    direction = direction*-1;
	}

        std::this_thread::sleep_for(std::chrono::milliseconds(10));


    }
}
