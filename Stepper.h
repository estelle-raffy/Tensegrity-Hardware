
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

    // Constructor
    Stepper_c() {
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


    void errorReducing( float demand ) {


      // tell module 1 to hold at 855
      //  module1.update( 855.0 );

      // tell module 2 to hold at 860
      //  module2.update( 860.0 );


      float measurement = analog_read_resistance();
      //float demand = 850.0;


      // update the low pass filter.
      float alpha = 0.9;
      lpf = ( lpf * alpha ) + ( measurement * (1.0 - alpha ) );

      // has the lpf removed the noise seen in measurement?
      //  Serial.print( measurement );
      //  Serial.print(",");
      //  Serial.println( lpf );


      /******************** 50ms *********************/
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
        Serial.print( measurement );
        Serial.print(",");
        Serial.print(demand);
        Serial.print(",");
        Serial.print(error);
        if ( DEBUG ) Serial.print("\n");
      }

    }

    void moveDown() {

      if ( DEBUG ) Serial.println("Doing step");

      digitalWrite(dir_pin, HIGH);

      // TO make the motor move one step
      // we just toggle the step pin high
      // then low.
      digitalWrite(step_pin, HIGH);
      delay(50);
      digitalWrite(step_pin, LOW);
      delay(50);
    }

    void moveUp() {
      digitalWrite(dir_pin, LOW);

      // TO make the motor move one step
      // we just toggle the step pin high
      // then low.
      digitalWrite(step_pin, HIGH);
      delay(50);
      digitalWrite(step_pin, LOW);
      delay(50);
    }

    void moveStepsDown( int steps_to_move ) {
      int step_count = 0;

      // What is an illogical use of this function?
      // if 0 or less, return (do no more of this function).
      if ( steps_to_move <= 0 ) return;

      // Otherwise, move by the requested amount
      while ( step_count < steps_to_move ) {
        moveDown();
        delay(25);
        step_count++;
      }

    }


    void moveStepsUp( int steps_to_move ) {
      int step_count = 0;

      // What is an illogical use of this function?
      // if 0 or less, return (do no more of this function).
      if ( steps_to_move <= 0 ) return;

      // Otherwise, move by the requested amount
      while ( step_count < steps_to_move ) {
        moveUp();
        delay(25);
        step_count++;
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
        Serial.println(endStopState);

        if ( DEBUG ) Serial.println("Moving down");
        moveUp();

      }

    }


  private:

};

#endif
