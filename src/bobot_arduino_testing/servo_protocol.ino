
/*
This communication scheme relies on Arduino's String object functions to parse commands and Serial functions (primarily readStringUntil()). 

With this scheme, the format for commands from the computer would be: 
"SET servo_number desired_postion" or "GET servo_number"
*/
#include <Servo.h>
#include <Wire.h>
#include "IntersemaBaro.h"

Intersema::BaroPressure_MS5607B baro(true);

// Pin assignment (PWM for control, analog for reading position)
#define S1_PIN 9
#define S2_PIN 10
#define S1_FEEDBACK A0
#define S2_FEEDBACK A1

// Servo objects
Servo servo1;
Servo servo2;

void setup() {
  servo1.attach(S1_PIN);
  servo2.attach(S2_PIN);

  pinMode(S1_FEEDBACK, INPUT);
  pinMode(S2_FEEDBACK, INPUT);

  Serial.begin(9600);
  baro.init();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil("\n");  // Newline char denotes end of each command

    if (command.startsWith("SET")) {  // Computer wants to set servo position
      int servo_number = command.charAt(4) - '0';
      int position = command.substring(6).toInt();

      if (servo_number == 1) {
        if ((position >= 0) && (position <= 180)) {  // Ensure out-of-bounds commands are not given
          servo1.write(position);
        }
      } else if (servo_number == 2) {
        if ((position >= 0) && (position <= 180)) {  // Ensure out-of-bounds commands are not given
          servo2.write(position);
        }
      }
    } else if (command.startsWith("GET")) {  // Computer wants to read servo positon
      int servo_number = command.charAt(4) - '0';

      if (servo_number == 1) {  // RED TAPE
        char buff[6];
        int analog_value = analogRead(S1_FEEDBACK);
        double position = (-0.2606 * analog_value) + 176.459;  // y = mx+b formula for (RED TAPE)
        dtostrf(position, 3, 2, buff);
        char final_buff[8];
        final_buff[0] = '!';
        final_buff[1] = buff[0];
        final_buff[2] = buff[1];
        final_buff[3] = buff[2];
        final_buff[4] = buff[3];
        final_buff[5] = buff[4];
        final_buff[6] = buff[5];
        final_buff[7] = buff[6];
        Serial.write(final_buff, 8);
        // Serial.println(final_buff);

      } 
        else if (servo_number == 2) {  // BLUE TAPE
        char buff[6];
        int analog_value = analogRead(S2_FEEDBACK);
        double position = (-0.28006 * analog_value) + 177.1706;  // y = mx+b formula for (BLUE TAPE)
        dtostrf(position, 3, 2, buff);
        char final_buff[8];
        final_buff[0] = '@';
        final_buff[1] = buff[0];
        final_buff[2] = buff[1];
        final_buff[3] = buff[2];
        final_buff[4] = buff[3];
        final_buff[5] = buff[4];
        final_buff[6] = buff[5];
        final_buff[7] = buff[6];
        Serial.write(final_buff, 8);
        // Serial.println(final_buff);
      } 
      else if (servo_number == 5) 
      {  // Altimeter case
        char buff[7];
        // char buff_feet[10];
        int data = baro.getHeightCentiMeters();
        double alt = double(data);
        // float alt_feet = float(alt/30.48);
        dtostrf(alt, 7,0, buff);
        char final_buff[8];
        final_buff[0] = '%';
        final_buff[1] = buff[0];
        final_buff[2] = buff[1];
        final_buff[3] = buff[2];
        final_buff[4] = buff[3];
        final_buff[5] = buff[4];
        final_buff[6] = buff[5];
        final_buff[7] = buff[6];
        // dtostrf(alt_feet, 7, 2, buff);
        // Serial.println(final_buff);
        Serial.write(final_buff, 8);
        // Serial.print(", Feet: ");
        // Serial.println(buff_feet);
      }
    }
  }
}
