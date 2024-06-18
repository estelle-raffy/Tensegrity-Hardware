#include "Stepper.h"
Stepper_c module1;
Stepper_c module2;

void setup() {

  Serial.begin( 9600 );
  //  Serial.println(" *** RESET *** ");

  // put your setup code here, to run once:
  module1.setPins( 23, 5, 26 );
  module1.setEndStopPin( A9 );
  module1.resistancePin (A0);

  //module2.setPins( 27, 6, 24 );
  //module2.setEndStopPin ( A7)
  //module2.resistancePin (A1);

  //module3.setPins(28 ,7, 25 );
  //module3.setEndStopPin ( A8)
  //module3.resistancePin (A2);


  //module1.homeToSwitch();  
  module1.moveStepsUp(5);//shorten 
  //module1.moveStepsDown(100); // elongates 
  module1.analog_read_resistance();
  Serial.println( module1.analog_read_resistance());

  // Set low pass filter to 0 initially.
  module1.lpf = 0.0;

  int value = module1.readDirPin();
  //Serial.println( module1.readDirPin() );

  module1.motor_update_ts = millis();
  pinMode (A5, INPUT);

}

void loop() {
//
//  module1.analog_read_resistance();
//  Serial.println( module1.analog_read_resistance());

//error-reducing 
  module1.errorReducing(150); //------------------------> function in class
  Serial.println(analogRead(A5));
//  

}
