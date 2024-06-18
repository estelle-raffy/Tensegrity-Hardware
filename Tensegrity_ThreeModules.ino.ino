#include "Stepper.h"
Stepper_c module1;
Stepper_c module2;
Stepper_c module3;

void setup() {

  Serial.begin( 9600 );
  //  Serial.println(" *** RESET *** ");

  //----------------------------->> module1 setup
  module1.setPins( 23, 5, 26 );
  module1.setEndStopPin( A9 );
  module1.resistancePin (A0);

  module1.homeToSwitch();
  //module1.moveStepsUp(50);//shorten
  //module1.moveStepsDown(50); // elongates
  delay(50);
  module1.analog_read_resistance();
  Serial.println( module1.analog_read_resistance());

  // Set low pass filter to 0 initially.
  module1.lpf = 0.0;

  int value = module1.readDirPin();
  //Serial.println( module1.readDirPin() );

  module1.motor_update_ts = millis();
  pinMode (A5, INPUT);

  //----------------------------->> module2 setup
  module2.setPins( 24, 6, 27 );
  module2.setEndStopPin ( A7);
  module2.resistancePin (A1);

  module2.homeToSwitch();
  //module2.moveStepsUp(50);//shorten
  //module2.moveStepsDown(100); // elongates
  delay(50);
  module2.analog_read_resistance();
  Serial.println(module2.analog_read_resistance());

  // Set low pass filter to 0 initially.
  module2.lpf = 0.0;

  //int value = module2.readDirPin();
  //Serial.println( module2.readDirPin() );

  module2.motor_update_ts = millis();

//----------------------------->> module3 setup
  module3.setPins(25 ,7, 28 );
  module3.setEndStopPin ( A8);
  module3.resistancePin (A2);

  module3.homeToSwitch();
  //module3.moveStepsUp(50);//shorten
  //module3.moveStepsDown(100); // elongates
  delay(50);
  module3.analog_read_resistance();
  Serial.println(module3.analog_read_resistance());

  // Set low pass filter to 0 initially.
  module3.lpf = 0.0;

  //int value = module3.readDirPin();
  //Serial.println( module3.readDirPin() );

  module3.motor_update_ts = millis();

}

void loop() {
//  module1.analog_read_resistance();
//  Serial.println( "module1 resistance value: ");
//  Serial.println( module1.analog_read_resistance());
//
//  module2.analog_read_resistance();
//  Serial.println( "module2 resistance value: ");
//  Serial.println( module2.analog_read_resistance());

//  module3.analog_read_resistance();
//  Serial.println( "module3 resistance value: ");
//  Serial.println( module3.analog_read_resistance());

  //error-reducing
  module1.errorReducing(150); //------------------------> function in class
  Serial.println("=========================================================");
  module2.errorReducing(875); //------------------------> function in class
  Serial.println("=========================================================");
  module3.errorReducing(870); //------------------------> function in class

}
