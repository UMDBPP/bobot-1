
/*
This communication scheme relies on Arduino's String object functions to parse commands and Serial functions (primarily readStringUntil()). 

With this scheme, the format for commands from the computer would be: 
"SET servo_number desired_postion" or "GET servo_number"
*/
#include <Servo.h>
#include <Wire.h>
#include "IntersemaBaro.h"
#include <math.h>

Intersema::BaroPressure_MS5607B baro(true);

// Pin assignment (PWM for control, analog for reading position)
#define S1_PIN 9
#define S2_PIN 11
#define S1_FEEDBACK A4
#define S2_FEEDBACK A4

// Servo objects
Servo servo1;
Servo servo2;

// Buffers
uint8_t* read_buffer = new uint8_t[3];
uint8_t* write_buffer = new uint8_t[2];
uint8_t* long_write_buffer = new uint8_t[5];


void setup() {
  servo1.attach(S1_PIN);
  servo2.attach(S2_PIN);

  pinMode(S1_FEEDBACK, INPUT);
  pinMode(S2_FEEDBACK, INPUT);

  Serial.begin(115200);
  baro.init();
}

void loop() 
{
  // if(Serial.available() > 0)
  // {
    
    Serial.readBytes(read_buffer, 3);  // ; char denotes end of each command

    if (read_buffer[0] == 1) // 1 is set command
    {  // Computer wants to set servo position
      if (read_buffer[1] == 1) 
      {
        servo1.write(1*read_buffer[2]);
      } 
      else if (read_buffer[1] == 2) 
      {
        servo2.write(1*read_buffer[2]);
      }
    }
    else if(read_buffer[0] = 2) // Two stands for "GET"
    {
      if (read_buffer[1] == 1) 
      {  // RED TAPE
        uint8_t position_final = round((-0.2606 * analogRead(S1_FEEDBACK)) + 176.459); // y = mx+b formula for (RED TAPE)
        write_buffer[0] = 1; // the servo we're reading from
        write_buffer[1] = position_final; // The position we've read
        Serial.write(write_buffer, 2);
      }
      else if (read_buffer[1] == 2) 
      {  // BLUE TAPE
        uint8_t position_final = round((-0.28006 * analogRead(S2_FEEDBACK)) + 177.1706); // y = mx+b formula for (BLUE TAPE)
        write_buffer[0] = 2; // the servo we're reading from
        write_buffer[1] = position_final; // The position we've read
        Serial.write(write_buffer, 2);
      } 
      else if (read_buffer[1] == 5) // Alitmeter case
      {
        long_write_buffer[0] = 5; // altimeter data identifier

        uint8_t mask = 0xFF;
        int32_t altitude = baro.getHeightCentiMeters(); // dont need decimal accuracy lmao
        // Stolen from rahul
        // for(int x = 0; x < 4; x += 1)
        // { 
        //     long_write_buffer[1 + x] = (altitude >> (8 * x)) & mask; // this should be in HSB form
        // }
        // Serial.write(long_write_buffer, 5);
      }
    }
    //delete[] read_buffer;
    //delete[] write_buffer;
    //delete[] long_write_buffer;
    Serial.flush();
}
