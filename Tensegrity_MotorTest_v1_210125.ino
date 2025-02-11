


#define TEST_EACH_MOTOR true
#define TEST_MOTOR 5



#define TEST_UPDATE_MS 6000
unsigned long test_update_ms_ts;




#define DEFAULT_STEP_DELAY_US 1800 // 150 was the fastest we can step a motor


// How fast should we step the motors?
// We control this by making the delay between
// steps bigger or smaller.
unsigned long step_delay[6] = {};

// to allow us to simply toggle the step pin
// state, we create 6 binary variables
boolean step_pin_state[6];
boolean dir_pin_state[6];

// We need to keep track of when we last updated
// the step for each motor in microseconds (us)
// so we create a timestamp (_ts) for each.
unsigned long step_us_ts[6];

// Pins to operate motors
int step_pin[6] = { 45, 6, 7, 44,  4, 5};
int dir_pin[6] = { 42, 24, 9, 41, 22, 23};

void setup() {
  // put your setup code here, to run once:

  // Initialise step delay to default value
  // and set pins as outputs
  for ( int i = 0; i < 6; i++ ) {
    step_delay[ i ] = DEFAULT_STEP_DELAY_US;
    step_us_ts[i] = micros();
    pinMode( step_pin[i], OUTPUT );
    pinMode( dir_pin[i], OUTPUT );
    digitalWrite( dir_pin[i], LOW );
    digitalWrite( step_pin[i], LOW );
    step_pin_state[i] = HIGH;
    dir_pin_state[i] = HIGH;
  }

  test_update_ms_ts = millis();
}


//
void loop() {

  // For each motor, check if it is time to
  // send a step.
  for ( int motor = 0; motor < 6; motor++ ) {



    if ( micros() - step_us_ts[motor] > step_delay[motor] ) {
      step_us_ts[motor] = micros();

      // Do we want to test just 1 motor?
      if ( TEST_EACH_MOTOR == true ) {
        if ( TEST_MOTOR == motor ) {
          digitalWrite( step_pin[motor], step_pin_state[motor] );

          // toggle the recorded pin state for next time.
          step_pin_state[motor] = !step_pin_state[motor];
        }


      } else { // Not testing each motor, apply to each (all) motors
        digitalWrite( step_pin[motor], step_pin_state[motor] );

        // toggle the recorded pin state for next time.
        step_pin_state[motor] = !step_pin_state[motor];
      }
    }

  }

  // Time to switch direction?
  if ( millis() - test_update_ms_ts > TEST_UPDATE_MS ) {
    test_update_ms_ts = millis();

    // switch direction for each motor
    for ( int motor = 0; motor < 6; motor++ ) {
      digitalWrite( dir_pin[motor], dir_pin_state[motor] );

      // toggle the recorded pin state for next time.
      dir_pin_state[motor] = !dir_pin_state[motor];

    }
  }

}
