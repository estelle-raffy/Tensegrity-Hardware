
#ifndef _STEPPER_H
#define _STEPPER_H

class Stepper_c {

  public:

#define DEBUG false

    // Each module will need these pins
    int dir_pin; // sets direction of travel (high or low)
    int step_pin;// toggled to move motor (high then low)
    int en_pin;  // switches motor on or off
    int end_stop_pin; // end stop switch connected to this
    int resistance_pin; //
    int sensor_value;

    // ----------------------------------------------------> for the error reducing function
    long motor_update_ts;
    float lpf;
    float demand;
    bool motorStarted = false;
    bool pinState = false;
    unsigned long pinState_ts; // time elasped since last change
    unsigned long pinState_ms = 10; // time between pin state changes/steppings

    // Constructor
    Stepper_c() {
      demand = 0.0; // initializes the demand to default value
    }

    float getDemand() {
      return demand; // returns the value of demand variable to allow other parts of the program
      // to access demand within the class instance
    }

    int readDirPin() {
      return digitalRead( dir_pin );
      //return value;

    }

    void setPins(int d, int s, int e) {

      dir_pin = d;
      step_pin = s;
      en_pin = e;

      // Set pins as outputs (to create signals)
      pinMode(dir_pin, OUTPUT);
      pinMode(en_pin, OUTPUT);
      pinMode(step_pin, OUTPUT);

      // Set initial state of these pins
      digitalWrite(en_pin, LOW); // low means "on"
      digitalWrite(dir_pin, LOW);

    }
    void setEndStopPin( int e ) {
      end_stop_pin = e;

      // Set as input to read into this pin.
      // INPUT_PULLUP creates a special circuit to
      // also power the switch
      pinMode(end_stop_pin, INPUT_PULLUP);

    }

    void resistancePin(int r) {
      resistance_pin = r;

      pinMode(resistance_pin, INPUT);

    }


    int analog_read_resistance ( ) { // don't need parameters?, need arguments?
      sensor_value = analogRead(resistance_pin);
      return sensor_value; // any if statement where would only return if interesting (e.g., > threshold)?
      // Serial.println (sensor_value); why not use print instead?

    }


    void updateLPF() {

      float measurement = analog_read_resistance();
      // update the low pass filter.
      float alpha = 0.9;
      lpf = ( lpf * alpha ) + ( measurement * (1.0 - alpha ) );

    }

    void errorReducing( float demand ) { //************************************************************ error reducing function

      /******************** 50ms *********************/

      if (motorStarted) {

        updateLPF();

        long elapsed_time = millis() - motor_update_ts;
        if ( elapsed_time > 10 ) {      // has 50ms passed?
          motor_update_ts = millis();


          float error = (lpf - demand );

          // When should the robot decide it is "close enough"
          // and stop?
          float movement_threshold = 5;
          if (  abs(error) < movement_threshold ) {
            // stop moving
          } else if ( error > 0 ) {
            moveUp();
          } else {
            moveDown();
          }

          if ( DEBUG ) Serial.print( (error  ) );
          Serial.print( lpf );
          Serial.print(",");
          Serial.print(demand);
          Serial.print(",");
          Serial.print(error);
          if ( DEBUG ) Serial.print("\n");
        }

      }
    }
    //*********************************************************************************************** end of error reducing function

    bool moveDown() {

      if ( DEBUG ) Serial.println("Doing step");

      digitalWrite(dir_pin, HIGH);
      if (millis () - pinState_ts > pinState_ms) {
        pinState_ts = millis();
        pinState = ! pinState;
        digitalWrite(step_pin, pinState);
        return true;
      }
      return false;
    }

    bool moveUp() {
      digitalWrite(dir_pin, LOW);
      if (millis () - pinState_ts > pinState_ms) {
        pinState_ts = millis();
        pinState = ! pinState;
        digitalWrite(step_pin, pinState);
        return true;
      }
      return false;
    }

    void moveStepsDown( int steps_to_move ) {
      int step_count = 0;

      // What is an illogical use of this function?
      // if 0 or less, return (do no more of this function).
      if ( steps_to_move <= 0 ) return;

      // Otherwise, move by the requested amount
      while ( step_count < steps_to_move ) {
        if (moveDown()) {
          step_count++;
        }
      }
    }


    void moveStepsUp( int steps_to_move ) {
      int step_count = 0;

      // What is an illogical use of this function?
      // if 0 or less, return (do no more of this function).
      if ( steps_to_move <= 0 ) return;

      // Otherwise, move by the requested amount
      while ( step_count < steps_to_move ) {
        if (moveUp()) {
          step_count++;
        }
      }
    }

    void homeToSwitch() {// homing

      int endStopState;

      // Get a first reading
      endStopState = digitalRead(end_stop_pin);

      if ( DEBUG ) Serial.print( endStopState );


      // Whilst the sensor is reading as 1, this means
      // that the switch has not been pressed yet.
      // THerefore we move the motor down
      while (endStopState == 1) {
        endStopState = digitalRead(end_stop_pin);
        //Serial.println(endStopState);

        if ( DEBUG ) Serial.println("Moving down");
        moveUp();

      }

    }
    //---------------------------------------------------------------new function------------------------
    float calibrate () {

      homeToSwitch();
      delay(100);
      lpf = analog_read_resistance();
      for (int i = 0; i < 50; i++) {
        updateLPF();
        delay(5);
      }
      int measurement1 = lpf; //analog_read_resistance ( );
      Serial.println("Measurement1: ");
      Serial.println(measurement1);
      moveStepsDown(600);
      delay(100);
      lpf = analog_read_resistance();
      for (int i = 0; i < 50; i++) {
        updateLPF();
        delay(5);
      }
      int measurement2 = lpf; //analog_read_resistance ( );
      Serial.println("Measurement2: ");
      Serial.println(measurement2);
      demand = (measurement1 + measurement2) / 2;
      homeToSwitch();

      return demand;

    }
    //---------------------------------------------------------------new function------------------------

  private:

};

#endif
