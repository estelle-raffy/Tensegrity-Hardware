#ifndef _STEPPER_H
#define _STEPPER_H

class Stepper_c {

  public:

#define DEBUG false

    // Each module will need these pins
    int dir_pin; // sets direction of travel (high or low)
    int step_pin;// toggled to move motor (high then low)

    void setPins(int d, int s) {

      dir_pin = d;
      step_pin = s;

      // Set pins as outputs (to create signals)
      pinMode(dir_pin, OUTPUT);
      pinMode(step_pin, OUTPUT);

      // Set initial state of these pins
      digitalWrite(dir_pin, HIGH);

    }

    void moveDown() {

      digitalWrite(dir_pin, HIGH);

      // TO make the motor move one step
      // we just toggle the step pin high
      // then low.
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(800);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(800);
    }

    void moveUp() {
      
      digitalWrite(dir_pin, LOW);

      // TO make the motor move one step
      // we just toggle the step pin high
      // then low.
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(800);
      digitalWrite(step_pin, LOW);
      delayMicroseconds(800);
    }

    void moveStepsDown( int steps_to_move ) {
      int step_count = 0;

      // What is an illogical use of this function?
      // if 0 or less, return (do no more of this function).
      if ( steps_to_move <= 0 ) return;

      // Otherwise, move by the requested amount
      while ( step_count < steps_to_move ) {
        moveDown(); 
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
        step_count++;
      }
    }

  private:

};

#endif
