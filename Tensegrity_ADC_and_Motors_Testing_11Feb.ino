
/*
   integrating motor functionality



*/

#include "Protocentral_ADS1220.h"
#include <SPI.h>


/***************************************************************

    ADC Chip variables and configuration below



 **************************************************************/
#define PGA          1                 // Programmable Gain = 1
#define VREF         2.048            // Internal reference of 2.048V
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1)

const int cs_pin[6]    = {0, 1, 17, 16, 15, 14};
const int drdy_pin[6]  = {2, 3, 18, 19, 20, 21};

Protocentral_ADS1220 adc_chip[6];

// 6 chips, each with 4 channels
float adc_value[6][4];
float lpf_value[6][4];

boolean CALIBRATING = true;

volatile bool drdyIntrFlag0 = false;
volatile bool drdyIntrFlag1 = false;
volatile bool drdyIntrFlag2 = false;
volatile bool drdyIntrFlag3 = false;
volatile bool drdyIntrFlag4 = false;
volatile bool drdyIntrFlag5 = false;
void drdyInterruptHndlr0() {
  drdyIntrFlag0 = true;
}
void drdyInterruptHndlr1() {
  drdyIntrFlag1 = true;
}
void drdyInterruptHndlr2() {
  drdyIntrFlag2 = true;
}
void drdyInterruptHndlr3() {
  drdyIntrFlag3 = true;
}
void drdyInterruptHndlr4() {
  drdyIntrFlag4 = true;
}
void drdyInterruptHndlr5() {
  drdyIntrFlag5 = true;
}

unsigned long adc_update_ts;

void enableInterruptPin() {
  attachInterrupt( digitalPinToInterrupt( drdy_pin[0]), drdyInterruptHndlr0, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[1]), drdyInterruptHndlr1, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[2]), drdyInterruptHndlr2, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[3]), drdyInterruptHndlr3, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[4]), drdyInterruptHndlr4, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[5]), drdyInterruptHndlr5, FALLING);
}

/***************************************************************

    stepper motor drivers and variables below



 **************************************************************/


#define TEST_ONE_MOTOR true
#define TEST_MOTOR 0



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

  // enable and configure each adc chip
  for ( int i = 0; i < 6; i++ ) {
    adc_chip[i].begin(cs_pin[i], drdy_pin[i]);
    adc_chip[i].set_data_rate(DR_330SPS);
    adc_chip[i].set_pga_gain(PGA_GAIN_1);
    adc_chip[i].set_VREF (1 << 6); //3

    // Set initial adc values read to 0
    for ( int j = 0; j < 4; j++ ) {
      adc_value[i][j] = 0;
      lpf_value[i][j] = 0;
    }

  }

  // Do some initial readings to setup the low
  // pass filter
  for ( int i = 0; i < 100; i++ ) {
    for ( int chip = 2; chip < 6; chip++ ) {
      read_adc_chip( chip );
    }
    delay(1);
  }
  CALIBRATING = false; // indicate that calibration has been done

  // Initialise motors
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


  delay(100);
  Serial.begin(9600);


  adc_update_ts = millis();

}


void loop() {

  // this is non-blocking and will
  // handle it's own timing
  updateMotors();

  // Read and report adc periodically
  if ( millis() - adc_update_ts > 50 ) {
    adc_update_ts = millis();
    read_adc_chip( 5 ); // change this to whatever ADC number you want to test
    Serial.println();
  }
}



float convertToMilliV(int32_t i32data)
{
  return (float)((i32data * VFSR * 1000) / FULL_SCALE);
}
void read_adc_chip( int which ) {

  // Only run the following code if we have
  // been asked to read a sensible chip
  // number (e.g. 0,1,2,3,4,5)
  if ( which >= 0 && which < 6 ) {


    // read each channel for this chip
    int32_t reading;

    reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH0);
    adc_value[ which ][0] = convertToMilliV(reading);

    reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH1);
    adc_value[ which ][1] = convertToMilliV(reading);

    reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH2);
    adc_value[ which ][2] = convertToMilliV(reading);

    reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH3);
    adc_value[ which ][3] = convertToMilliV(reading);

    for ( int i = 0; i < 4; i++ ) {

      //check if the latest reading is very extreme
      if ( CALIBRATING == false ) {

        // not calibrating, did the value make an unusual big jump
        if ( abs(adc_value[which][i]) < (lpf_value[which][i] * 50 ) ) {
          // no, so update low pass filter
          lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 ) ;
        }
      } else {

        lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 );

      } // end of calibrating


      Serial.print( adc_value[which][i] );
      Serial.print(",");
    } // end of channel 0-3

  } // end of which < 6
} // end of function





/*
   This is written with the assumption that we
   are updating all motors, but setting the
   #define TEST_ONE_MOTOR true will mean that
   we can over-ride this behaviour to actually
   only operate one motor.
*/
void updateMotors() {

  // For each motor, check if it is time to
  // send a step.
  for ( int motor = 0; motor < 6; motor++ ) {



    if ( micros() - step_us_ts[motor] > step_delay[motor] ) {
      step_us_ts[motor] = micros();

      // Do we want to test just 1 motor?
      if ( TEST_ONE_MOTOR == true ) {
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
