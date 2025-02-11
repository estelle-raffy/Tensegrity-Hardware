#include "Stepper.h"
Stepper_c module1;
Stepper_c module2;
Stepper_c module3;
Stepper_c module4;
Stepper_c module5;
Stepper_c module6;

int delay_motor = 50;
int number_of_steps = 1000;
bool elongate_motor = false;
bool shorten_motor = false;

//int dir_pin[6] = { 42, 24, 9, 41, 22, 23};
//int step_pin[6] = { 45, 6, 7, 44,  4, 5};

void setup() {

  Serial.begin( 9600 );
  //----------------------------->> module1 setup
  module1.setPins(42, 45);

  //----------------------------->> module2 setup
  module2.setPins(24, 6);

  //----------------------------->> module3 setup
  module3.setPins(9, 7);

  //----------------------------->> module4 setup
  module4.setPins(41, 44);

  //----------------------------->> module5 setup
  module5.setPins(23, 5);

  //----------------------------->> module6 setup
  module6.setPins(22, 4);

  Serial.println("Set-up finished");
}

void loop() {
  if (Serial.available() > 0) {
    Serial.println("serial available");
    char cmd = Serial.read();
    Serial.print("Received: ");
    Serial.print(cmd);
    //------------------------------- Test motor1-------------------------
    if (cmd == '1') {
      if (elongate_motor) {
        module1.moveStepsDown(number_of_steps); // elongates
        Serial.println("Motor 1 elongates");
        delay(delay_motor);
      }
      else if (shorten_motor) {
        module1.moveStepsUp(number_of_steps);//shorten
        Serial.println("Motor 1 shortens");
        delay(delay_motor);
      }
    }
    //------------------------------- Test motor2-------------------------
    else if (cmd == '2') {
      if (elongate_motor) {
        module2.moveStepsDown(number_of_steps); // elongates
        Serial.println("Motor 2 elongates");
        delay(delay_motor);
      }
      else if (shorten_motor) {
        module2.moveStepsUp(number_of_steps);//shorten
        Serial.println("Motor 2 shortens");
        delay(delay_motor);
      }
    }
    //------------------------------- Test motor3-------------------------
    else if (cmd == '3') {
      if (elongate_motor) {
        module3.moveStepsDown(number_of_steps); // elongates
        Serial.println("Motor 3 elongates");
        delay(delay_motor);
      }
      else if (shorten_motor) {
        module3.moveStepsUp(number_of_steps);//shorten
        Serial.println("Motor 3 shortens");
        delay(delay_motor);
      }
    }
    //------------------------------- Test motor4-------------------------
    else if (cmd == '4') {
      if (elongate_motor) {
        module4.moveStepsDown(number_of_steps); // elongates
        Serial.println("Motor 4 elongates");
        delay(delay_motor);
      }
      else if (shorten_motor) {
        module4.moveStepsUp(number_of_steps);//shorten
        Serial.println("Motor 4 shortens");
        delay(delay_motor);
      }
    }
    //------------------------------- Test motor5-------------------------
    else if (cmd == '5') {
      if (elongate_motor) {
        module5.moveStepsDown(number_of_steps); // elongates
        Serial.println("Motor 5 elongates");
        delay(delay_motor);
      }
      else if (shorten_motor) {
        module5.moveStepsUp(number_of_steps);//shorten
        Serial.println("Motor 5 shortens");
        delay(delay_motor);
      }
    }
    //------------------------------- Test motor6-------------------------
    else if (cmd == '6') {
      if (elongate_motor) {
        module6.moveStepsDown(number_of_steps); // elongates
        Serial.println("Motor 6 elongates");
        delay(delay_motor);
      }
      else if (shorten_motor) {
        module6.moveStepsUp(number_of_steps);//shorten
        Serial.println("Motor 6 shortens");
        delay(delay_motor);
      }
    }
    //------------------------------- Control direction-------------------
    else if ( cmd == 'e'  )  {
      elongate_motor = true;
      shorten_motor = false;
    }
    else if ( cmd == 's'  )  {
      elongate_motor = false;
      shorten_motor = true;
    }

  }
}
