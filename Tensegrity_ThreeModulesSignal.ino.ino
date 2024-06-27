#include "Stepper.h"
Stepper_c module1;
Stepper_c module2;
Stepper_c module3;

float d_mod = 0.0;

void setup() {

  Serial.begin( 9600 );
  //  Serial.println(" *** RESET *** ");

  //----------------------------->> module1 setup
  module1.setPins( 23, 5, 26 );
  module1.setEndStopPin( A9 );
  module1.resistancePin (A0);

  module1.calibrate();//---------------------------------------------------------------new function------------------------
  Serial.println("Demand1 is: ");
  Serial.println(module1.getDemand());
  //module1.moveStepsUp(50);//shorten
  //module1.moveStepsDown(50); // elongates
  delay(50);
  

  int value = module1.readDirPin();
  //Serial.println( module1.readDirPin() );

  module1.motor_update_ts = millis();
  pinMode (A5, INPUT);

  //----------------------------->> module2 setup
  module2.setPins( 24, 6, 27 );
  module2.setEndStopPin ( A7);
  module2.resistancePin (A1);

  module2.calibrate();//---------------------------------------------------------------new function------------------------
  Serial.println("Demand2 is: ");
  Serial.println(module2.getDemand());
  //module2.moveStepsUp(50);//shorten
  //module2.moveStepsDown(100); // elongates
  delay(50);
  

  //int value = module2.readDirPin();
  //Serial.println( module2.readDirPin() );

  module2.motor_update_ts = millis();

  //----------------------------->> module3 setup
  module3.setPins(25 , 7, 28 );
  module3.setEndStopPin ( A8);
  module3.resistancePin (A2);

  module3.calibrate();//---------------------------------------------------------------new function------------------------
  Serial.println("Demand3 is: ");
  Serial.println(module3.getDemand()); 
  //module3.moveStepsUp(50);//shorten
  //module3.moveStepsDown(100); // elongates
  delay(50);
  

  // give the low pass filter a sensible initial
  // reading by taking a set of readings from the
  // stretch cables.
  // Set low pass filter to 0 initially.
  module3.lpf = module3.analog_read_resistance();
  // Set low pass filter to 0 initially.
  module2.lpf = module2.analog_read_resistance();
  // Set low pass filter to 0 initially.
  module1.lpf = module1.analog_read_resistance();
  for ( int i = 0; i < 50; i++ ) {
    module1.updateLPF();
    module2.updateLPF();
    module3.updateLPF();
    delay(5);
  }

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

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1') {
      d_mod = + 10;
    }
    else if (cmd == '2') {
      d_mod = -10;
    }
    else if (cmd == '3') {
      d_mod = 0;
    }
    else if (cmd == 's'){
      module1.motorStarted = true;
      module2.motorStarted = true;
      module3.motorStarted = true;
    }
    else if (cmd == 'q'){
      module1.motorStarted = false;
      module2.motorStarted = false;
      module3.motorStarted = false;
    }
  }
  
  //error-reducing
  Serial.println("MODULE 1");
  module1.errorReducing(module1.getDemand() + d_mod ); //------------------------> function in class
  Serial.println("\n");
  Serial.println("MODULE 2");
  module2.errorReducing(module2.getDemand() + d_mod ); //------------------------> function in class
  Serial.println("\n");
  Serial.println("MODULE 3");
  module3.errorReducing(module3.getDemand() + d_mod ); //------------------------> function in class

}
