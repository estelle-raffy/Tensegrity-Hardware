// MESSY version of Tensegrity_test_lowlevelV2(AUTOMATE)
// This code builds on the low-level code and adds a neighbour condition
// module = stepper motor + 2 cords not shared with single other modules on one end
// neighbours are modules sharing 2 cords at same end
// module condition : -1 = same total voltage; 1 = different total cord voltage

#include "Protocentral_ADS1220.h"
#include <SPI.h>


float results[100];

/***************************************************************

    ADC Chip variables and configuration below



 **************************************************************/
#define PGA          1                 // Programmable Gain = 1
#define VREF         2.048            // Internal reference of 2.048V
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1) // max ADC value (24-bit resolution of ADS1220)

const int cs_pin[6]    = {10, 11, 17, 16, 15, 14}; // pins for ADC, check pins 10 and 11 actually work!
const int drdy_pin[6]  = {2, 3, 18, 19, 20, 21};

Protocentral_ADS1220 adc_chip[6]; // create an array of ADCs

// 6 chips, each with 4 channels
float adc_value[6][4]; // stores ADC readings
float lpf_value[6][4]; // stored filtered ADC readings

boolean CALIBRATING = true; // indicates calibration is happening

volatile bool drdyIntrFlag0 = false; // these flags store whether an ADC chip has new data available
volatile bool drdyIntrFlag1 = false;
volatile bool drdyIntrFlag2 = false;
volatile bool drdyIntrFlag3 = false;
volatile bool drdyIntrFlag4 = false;
volatile bool drdyIntrFlag5 = false;

void drdyInterruptHndlr0() { //functions that set flaf to True whenever ADC chip signals new data
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
  attachInterrupt( digitalPinToInterrupt( drdy_pin[0]), drdyInterruptHndlr0, FALLING); // links ADC's data ready pin to respective interrupt handler
  attachInterrupt( digitalPinToInterrupt( drdy_pin[1]), drdyInterruptHndlr1, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[2]), drdyInterruptHndlr2, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[3]), drdyInterruptHndlr3, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[4]), drdyInterruptHndlr4, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[5]), drdyInterruptHndlr5, FALLING);
}

/***************************************************************

    stepper motor drivers and variables below



 **************************************************************/


#define TEST_ONE_MOTOR true // if true, only one motor is tested 
#define TEST_MOTOR 0

#define TEST_UPDATE_MS 10 // 6000 was default because used 6 motors??   

#define DEFAULT_STEP_DELAY_US 150 // 150 was the fastest we can step a motor; 1800 was default ; 800 was used in check motors


// How fast should we step the motors?
// We control this by making the delay between
// steps bigger or smaller.
unsigned long step_delay[6] = {}; // array storing the step delay for each of 6 motors

// to allow us to simply toggle the step pin
// state, we create 6 binary variables
boolean step_pin_state[6]; // arrays of boolean values, each correspond to a motor (e.g. step pin state HIGH/LOW for each )
boolean dir_pin_state[6];

// We need to keep track of when we last updated
// the step for each motor in microseconds (us)
// so we create a timestamp (_ts) for each.
unsigned long step_us_ts[6];

// Pins to operate motors
int step_pin[6] = { 45, 6, 7, 44,  4, 5};
int dir_pin[6] = { 42, 24, 9, 41, 22, 23};

int timeStep = 0;

// **************************************************** LOWER-LEVEL VARIABLES, WEIGHTS, PARAMETERS **************************

int local_demand = 4000; // 3700 not moving!
int local_threshold = 100; // 100 goes with 3700 for M0
int neighbour_condition = -1; // -1 same voltage; 1 different voltage
int neigh_threshold = 40; // based on data
int actuation_step = 10; // allows to see the motor move, 1 was too small
int local_weight = 1; // how much local affects bhv, 0 = OFF
int neigh_weight = 1; // how much neigh affects bhv, 0 = OFF

// **************************************************** END OF LOWER-LEVEL VARIABLES, WEIGHTS, PARAMETERS **************************

void setup() {

  // enable and configure each adc chip
  for ( int i = 0; i < 6; i++ ) {
    adc_chip[i].begin(cs_pin[i], drdy_pin[i]);
    adc_chip[i].set_data_rate(DR_330SPS); // sets a data rate of 330 samples/sec and gain of 1
    adc_chip[i].set_pga_gain(PGA_GAIN_1);
    adc_chip[i].set_VREF (1 << 6); //3 --> see notes input voltage to cord and ADC matching

    // Set initial adc values read to 0
    for ( int j = 0; j < 4; j++ ) {
      adc_value[i][j] = 0;
      lpf_value[i][j] = 0;
    }
  }

  // Do some initial readings to setup the low
  // pass filter
  for ( int i = 0; i < 100; i++ ) { // read ADC 100 times for calibration; LPF = average previous and current values to smooth out noise.
    for ( int chip = 2; chip < 6; chip++ ) {
      calibrate_all_adcs( chip );
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

  delay(100);
  Serial.begin(9600);

  adc_update_ts = millis();
}

//==================================== MAIN LOOP ===================================
void loop() {

  timeStep += 1;
  Serial.print("***************************************************************timeStep ");
  Serial.println(timeStep);

  //  //testing the cords are sending expected outputs
  //  Serial.println(read_adc_chip_ch( 0 , 3 ));
  //  Serial.print(",");
  //  Serial.println(read_adc_chip_ch( 0 , 1 ));
  //  Serial.print(",");
  //  Serial.println(read_adc_chip_ch( 0 , 2 ));
  //  Serial.print(",");
  //  Serial.println(read_adc_chip_ch( 0 , 0 ));
  //  Serial.print(",");


  //    float total_voltage = read_adc_chip_ch( 3 , 0 ) + read_adc_chip_ch( 5 , 2 );
  //    //Serial.println(" TOTAL VOLTAGE = ");
  //    Serial.print(total_voltage);
  //    Serial.println(',');

  // Move motor: this is non-blocking and
  // will handle it's own timing
  updateMotor0();
  updateMotor1();

}
//==================================== END MAIN LOOP ===================================



float convertToMilliV(int32_t i32data)
{
  return (float)((i32data * VFSR * 1000) / FULL_SCALE);
}
void calibrate_all_adcs(int which) {

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

      //if not calibrating, apply extra check: if the latest reading is very extreme and filtering it out so it doesn't affect the filter
      if ( CALIBRATING == false ) {

        // not calibrating, did the value make an unusual big jump
        if ( abs(adc_value[which][i]) < (lpf_value[which][i] * 50 ) ) { // if reading less than 50 times previous filtered value, then fine. if not, out so avoiding updating filter with these!
          // no, so update low pass filter
          lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 ) ; //Exponential Moving average: New filtered value = (old filtered value*0.9) + (New ADC value*0.1)
          // the 0.9 weight gives more importance to past readings (smoothing the signal through history) and slow 0.1 adaptation to new readings
        }
      } else {

        lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 ); // if calibration happening, always apply filter: no extreme jumps during calibration

      } // end of calibrating

      //      Serial.print("Calibration readings are: ");
      //      Serial.print( adc_value[which][i] );
      //      Serial.print(",");
    } // end of channel 0-3

  } // end of which < 6
} // end of function


float read_adc_chip_ch( int which, int channel ) {

  //Serial.println("Entering reading ADC function");
  // Only run the following code if we have
  // been asked to read a sensible chip
  // number (e.g. 0,1,2,3,4,5)
  if ( which >= 0 && which < 6 && channel >= 0 && channel < 4 ) {

    // read specific ADC and specific channel for one cord
    int32_t reading;

    if (channel == 0) {
      reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH0);
      //Serial.println("Reading channel 0");
    } else if (channel == 1) {
      reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH1);
    } else if (channel == 2) {
      reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH2);
    } else if (channel == 3) {
      reading = adc_chip[which].Read_SingleShot_SingleEnded_WaitForData(MUX_SE_CH3);
    }

    float voltage = convertToMilliV(reading);
    Serial.print("Voltage for ADC ");
    Serial.print(which);
    Serial.print("and channel ");
    Serial.print(channel);
    Serial.print(" is ");
    Serial.println(voltage);
    return voltage; // Return only the requested channel's voltage

    // is this still being done even though return has been made?
    for ( int i = 0; i < 4; i++ ) {

      //if not calibrating, apply extra check: if the latest reading is very extreme and filtering it out so it doesn't affect the filter
      if ( CALIBRATING == false ) {

        // not calibrating, did the value make an unusual big jump
        if ( abs(adc_value[which][i]) < (lpf_value[which][i] * 50 ) ) { // if reading less than 50 times previous filtered value, then fine. if not, out so avoiding updating filter with these!
          // no, so update low pass filter
          lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 ) ; //Exponential Moving average: New filtered value = (old filtered value*0.9) + (New ADC value*0.1)
          // the 0.9 weight gives more importance to past readings (smoothing the signal through history) and slow 0.1 adaptation to new readings
        }
      } else {

        lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 ); // if calibration happening, always apply filter: no extreme jumps during calibration

      } // end of calibrating
    } // end of channel 0-3

  } // end of which < 6
  else {
    Serial.print("Error value ADC or channel");
  }
} // end of function


// -------------------------------------- MOTOR 0 UPDATE FUNCTION ------------------------------------------
void updateMotor0() { // steps motors based on time intervals
  // steps the motors at right time using micros() and changes direction periodically using millis()
  // For each motor, check if it is time to send a step
  // and what total cords values are vs threshold

  //update ADCs readings before using them
  // Read and report adc periodically: reads whatever ADC want to read every 50ms
  float total_voltage;
  float total_voltage_neigh;

  if ( millis() - adc_update_ts > 10 ) { // was 50 in default
    adc_update_ts = millis();

    // Reset total voltage before summing up new readings
    total_voltage = 0;
    total_voltage_neigh = 0;

    // take voltage of module
    total_voltage += read_adc_chip_ch(3, 0); //cord L2 - F3
    total_voltage += read_adc_chip_ch(5, 1); // cord L4 - G4
    delayMicroseconds(50);

    // take voltage of neighbour
    total_voltage_neigh += read_adc_chip_ch(0, 1); //cord L1 - A2
    total_voltage_neigh += read_adc_chip_ch(5, 2); // cord L3 - B1
    delayMicroseconds(50);
  }

  // work out the error to total voltage
  float local_err = local_demand - total_voltage;
  Serial.print("----------------> MOTOR 0 local error is: ");
  Serial.print(local_err);
  Serial.println("\n");

  //work out the difference with neighbour
  float diff_neigh = total_voltage - total_voltage_neigh;
  Serial.print("-----------------> MOTOR 0 Neighbour Difference is: ");
  Serial.print(diff_neigh);
  Serial.println("\n");

  // work out if should move
  if ( micros() - step_us_ts[TEST_MOTOR] > step_delay[TEST_MOTOR] ) {
    // micros = time in microseconds since Arduino started (used for very short intervals),  step_us_ts = when last stepped, step_delay = how often should step
    // if enough time has passed, then send step signal
    step_us_ts[TEST_MOTOR] = micros(); // updating the last step time, storing current time


    //Working out actuation signal to send to motors depending on demand + neighbour
    int actuation_signal_up = 0; //will carry the local + neighbour actuation for shortening
    int actuation_signal_down = 0; //will carry the local + neighbour actuation for elongating
    int actuation_final = 0;

    if ( abs(local_err) < local_threshold ) {
      //stop moving
      Serial.println("MOTOR 0 LOCAL DEMAND ACHIEVED!");
    }
    else if (local_err > 0) {
      //moveStepsUp(actuation_step * local_weight, 0);
      actuation_signal_up += (actuation_step * local_weight);
      Serial.println("MOTOR 0 Increases voltage to local");
    }
    else {
      //moveStepsDown(actuation_step * local_weight, 0);
      actuation_signal_down += (actuation_step * local_weight);
      Serial.println("MOTOR 0 Decreases voltage to local");
    }

    // work out neighbour

    if ( abs(diff_neigh) < neigh_threshold ) {
      //stop moving
      Serial.println("MOTOR 0 NEIGHBOUR ACHIEVED");
    }
    else if (diff_neigh > 0) { // M0 bigger than M1
      if (neighbour_condition == -1) {
        //moveStepsUp(actuation_step * neigh_weight, 0); // shorten
        actuation_signal_up += (actuation_step * neigh_weight);
        Serial.println("MOTOR 0 Decreases voltage to converge");
      }
      else if (neighbour_condition == 1) {
        //moveStepsDown(actuation_step * neigh_weight, 0); // elongates
        actuation_signal_down += (actuation_step * neigh_weight);
        Serial.println("MOTOR 0 increase voltage to diverge");
      }
      else {
        Serial.println("MOTOR 0 INCORRECT NEIGHBOUR CONDITION");
      }  // end of neighbour_condition block

    }  // end of if (diff_neigh > 0) block

    else if (diff_neigh < 0) { // M1 bigger than M0
      if (neighbour_condition == -1) {
        //moveStepsDown(actuation_step * neigh_weight, 0);
        actuation_signal_down += (actuation_step * neigh_weight);
        Serial.println("MOTOR 0 Increase voltage to converge");
      }
      else if (neighbour_condition == 1) {
        //moveStepsUp(actuation_step * neigh_weight, 0);
        actuation_signal_up += (actuation_step * neigh_weight);
        Serial.println("MOTOR 0 Decrease voltage to diverge");
      }
      else {
        Serial.println("MOTOR 0 INCORRECT NEIGHBOUR CONDITION");
      }
    }  // end of the if (diff_neigh < 0) block

    // MOVE 
    actuation_final = actuation_signal_down - actuation_signal_up; //(elongate - shortening)

    if (actuation_final > 0) {
      moveStepsDown(actuation_final, 0);
    } else if (actuation_final < 0) {
      moveStepsUp(abs(actuation_final), 0);
    } else {
      Serial.println(" Motor 0 No movement needed.");
    }

    Serial.print("MOTOR 0 FINAL ACTUATION: ");
    Serial.println(actuation_final); // if negative means shortens

  } // end of check if can move

  Serial.print("===============> MOTOR 0 Updated total voltage is: ");
  Serial.println(total_voltage);
  Serial.print('\n');
} // end of function

// -------------------------------------- END OF MOTOR 0 MAIN UPDATE FUNCTION ------------------------------------------

// -------------------------------------- MOTOR 1 MAIN UPDATE FUNCTION ------------------------------------------

void updateMotor1() { // steps motors based on time intervals
  // steps the motors at right time using micros() and changes direction periodically using millis()
  // For each motor, check if it is time to send a step
  // and what total cords values are vs threshold

  //update ADCs readings before using them
  // Read and report adc periodically: reads whatever ADC want to read every 50ms
  float total_voltage;
  float total_voltage_neigh;

  if ( millis() - adc_update_ts > 10 ) { // was 50 in default
    adc_update_ts = millis();

    // Reset total voltage before summing up new readings
    total_voltage = 0;
    total_voltage_neigh = 0;

    // take voltage of module
    total_voltage += read_adc_chip_ch(0, 1); //cord A2 - L1
    total_voltage += read_adc_chip_ch(0, 3); // cord A1 - C1
    delayMicroseconds(50);

    // take voltage of neighbour
    total_voltage_neigh += read_adc_chip_ch(0, 2); //cord A3 - E1
    total_voltage_neigh += read_adc_chip_ch(0, 0); // cord A4 - F1
    delayMicroseconds(50);
  }

  // work out the error to total voltage
  float local_err = local_demand - total_voltage;
  Serial.print("----------------> MOTOR 1 local error is: ");
  Serial.print(local_err);
  Serial.println("\n");

  //work out the difference with neighbour
  float diff_neigh = total_voltage - total_voltage_neigh;
  Serial.print("-----------------> MOTOR 1 Neighbour Difference is: ");
  Serial.print(diff_neigh);
  Serial.println("\n");

  // work out if should move
  if ( micros() - step_us_ts[TEST_MOTOR] > step_delay[TEST_MOTOR] ) {
    // micros = time in microseconds since Arduino started (used for very short intervals),  step_us_ts = when last stepped, step_delay = how often should step
    // if enough time has passed, then send step signal
    step_us_ts[TEST_MOTOR] = micros(); // updating the last step time, storing current time


    //Working out actuation signal to send to motors depending on demand + neighbour
    int actuation_signal_up = 0; //will carry the local + neighbour actuation for shortening
    int actuation_signal_down = 0; //will carry the local + neighbour actuation for elongating
    int actuation_final = 0;

    if ( abs(local_err) < local_threshold ) {
      //stop moving
      Serial.println("MOTOR 1 LOCAL DEMAND ACHIEVED!");
    }
    else if (local_err > 0) {
      //moveStepsUp(actuation_step * local_weight, 1);
      actuation_signal_up += (actuation_step * local_weight);
      Serial.println("MOTOR 1 Increases voltage to local");
    }
    else {
      //moveStepsDown(actuation_step * local_weight, 1);
      actuation_signal_down += (actuation_step * local_weight);
      Serial.println("MOTOR 1 Decreases voltage to local");
    }

    // work out neighbour

    if ( abs(diff_neigh) < neigh_threshold ) {
      //stop moving
      Serial.println("MOTOR 1 NEIGHBOUR ACHIEVED");
    }
    else if (diff_neigh > 0) { // M1 bigger than M0
      if (neighbour_condition == -1) {
        //moveStepsUp(actuation_step * neigh_weight, 1); // shorten
        actuation_signal_up += (actuation_step * neigh_weight);
        Serial.println("MOTOR 1 Decreases voltage to converge");
      }
      else if (neighbour_condition == 1) {
        //moveStepsDown(actuation_step * neigh_weight, 1); // elongates
        actuation_signal_down += (actuation_step * neigh_weight);
        Serial.println("MOTOR 1 increase voltage to diverge");
      }
      else {
        Serial.println("MOTOR 1 INCORRECT NEIGHBOUR CONDITION");
      }  // end of neighbour_condition block

    }  // end of if (diff_neigh > 0) block

    else if (diff_neigh < 0) { // M0 bigger than M1
      if (neighbour_condition == -1) {
        //moveStepsDown(actuation_step * neigh_weight, 1);
        actuation_signal_down += (actuation_step * neigh_weight);
        Serial.println("MOTOR 1 Increase voltage to converge");
      }
      else if (neighbour_condition == 1) {
        //moveStepsUp(actuation_step * neigh_weight, 1);
        actuation_signal_up += (actuation_step * neigh_weight);
        Serial.println("MOTOR 1 Decrease voltage to diverge");
      }
      else {
        Serial.println("MOTOR 1 INCORRECT NEIGHBOUR CONDITION");
      }
    }  // end of the if (diff_neigh < 0) block

    // MOVE
    actuation_final = actuation_signal_down - actuation_signal_up; //(elongate - shortening)

    if (actuation_final > 0) {
      moveStepsDown(actuation_final, 1);
    } else if (actuation_final < 0) {
      moveStepsUp(abs(actuation_final), 1);
    } else {
      Serial.println(" Motor 1 No movement needed.");
    }

    Serial.print("MOTOR 1 FINAL ACTUATION: ");
    Serial.println(actuation_final); // if negative means shortens

  } // end of check if can move

  Serial.print("===============> MOTOR 1 Updated total voltage is: ");
  Serial.println(total_voltage);
  Serial.print('\n');
} // end of function

// -------------------------------------- END OF MOTOR 1 MAIN UPDATE FUNCTION ------------------------------------------


// -------------------------------------- MOVING THE MOTORS FUNCTIONS ------------------------------------------

void moveDown(int motor) {

  digitalWrite(dir_pin[motor], dir_pin_state[motor]);

  // TO make the motor move one step
  // we just toggle the step pin high
  // then low.
  digitalWrite(step_pin[motor], step_pin_state[motor]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US);
  digitalWrite(step_pin[motor], !step_pin_state[motor]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US);
}

void moveUp(int motor) {

  digitalWrite(dir_pin[motor], !dir_pin_state[motor]);

  // TO make the motor move one step
  // we just toggle the step pin high
  // then low.
  digitalWrite(step_pin[motor], step_pin_state[motor]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US);
  digitalWrite(step_pin[motor], !step_pin_state[motor]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US);
}

void moveStepsDown( int steps_to_move, int motor ) {
  int step_count = 0;

  // What is an illogical use of this function?
  // if 0 or less, return (do no more of this function).
  if ( steps_to_move <= 0 ) return;

  // Otherwise, move by the requested amount
  while ( step_count < steps_to_move ) {
    moveDown(motor);
    step_count++;

  }
}

void moveStepsUp( int steps_to_move, int motor ) {
  int step_count = 0;

  // What is an illogical use of this function?
  // if 0 or less, return (do no more of this function).
  if ( steps_to_move <= 0 ) return;

  // Otherwise, move by the requested amount
  while ( step_count < steps_to_move ) {
    moveUp(motor);
    step_count++;
  }
}
