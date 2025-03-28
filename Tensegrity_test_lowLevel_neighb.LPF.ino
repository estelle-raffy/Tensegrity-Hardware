// Low pass filter in progress
// This code builds on the low-level code and adds a neighbour condition
// module = stepper motor + 2 cords not shared with single other modules on one end
// neighbours are modules sharing 2 cords at same end
// module condition : -1 = same total voltage; 1 = different total cord voltage

#include "Protocentral_ADS1220.h"
#include <SPI.h>


/***************************************************************

    ADC Chip variables and configuration below



 **************************************************************/
#define PGA          1                 // Programmable Gain = 1
#define VREF         2.048            // Internal reference of 2.048V
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1) // max ADC value (24-bit resolution of ADS1220)

const int cs_pin[6]    = {10, 11, 17, 16, 15, 14}; // pins for ADC, check pins 10 and 11 actually work!
const int drdy_pin[6]  = {2, 3, 18, 19, 20, 21};

const int channel_key[4] = { MUX_SE_CH0, MUX_SE_CH1, MUX_SE_CH2, MUX_SE_CH3};

// ADCs and channels per motor >> depending on how many motors test
const int nb_motors = 2;

const int motor_ADC_own_1[nb_motors] = {3, 0};
const int motor_ADC_own_2[nb_motors] = {5, 0};
const int motor_channel_own_1[nb_motors] = {0, 3};
const int motor_channel_own_2[nb_motors] = {1, 1};
const int motor_ADC_neigh_1[nb_motors] = {0, 0};
const int motor_ADC_neigh_2[nb_motors] = {5, 0};
const int motor_channel_neigh_1[nb_motors] = {1, 2};
const int motor_channel_neigh_2[nb_motors] = {2, 0};
//const int motor_ADC_neigh[2] = {motor_ADC_neigh_1[motorIndex], motor_ADC_neigh_2[motorIndex]};

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

#define DEFAULT_STEP_DELAY_US 350 // 150 was the fastest we can step a motor; 1800 was default ; 800 was used in check motors


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

int local_demand = 3800; // 3700 not moving!
int local_threshold = 100; // goes with 3700
int neighbour_condition = -1; // -1 same voltage; 1 different voltage
int neigh_threshold = 40; // based on data
int actuation_step = 10; // allows to see the motor move, 1 was too small; default 10
int local_weight = 1; // how much local affects bhv, 0 = OFF
int neigh_weight = 0; // how much neigh affects bhv, 0 = OFF

// **************************************************** END OF LOWER-LEVEL VARIABLES, WEIGHTS, PARAMETERS **************************

float convertToMilliV(int32_t i32data)
{
  return (float)((i32data * VFSR * 1000) / FULL_SCALE);
}

void calibrate_all_adcs(int which) {


  Serial.println("Entered calibrate all() ");
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

    // for each channel, 0 to 3
    for ( int i = 0; i < 4; i++ ) {

      //if not calibrating, apply extra check: if the latest reading is very extreme and filtering it out so it doesn't affect the filter
      if ( CALIBRATING == false ) {

        // not calibrating, did the value make an unusual big jump
        if ( abs(adc_value[which][i]) < (lpf_value[which][i] * 50 ) ) { // if reading less than 50 times previous filtered value, then fine. if not, out so avoiding updating filter with these!
          // no, so update low pass filter
          Serial.print("Calibration values before main: ");
          Serial.println(lpf_value[which][i]);;
          lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 ) ; //Exponential Moving average: New filtered value = (old filtered value*0.9) + (New ADC value*0.1)
          // the 0.9 weight gives more importance to past readings (smoothing the signal through history) and slow 0.1 adaptation to new readings
        }
      } else {
        Serial.println(" calibrating is true");
        Serial.print("which: " );Serial.print(which);Serial.print("i:");Serial.println(i);
        
        Serial.print("before: ");
        Serial.println( lpf_value[which][i] );
        
        lpf_value[which][i] = ( lpf_value[which][i] * 0.9 ) + ( adc_value[which][i] * 0.1 ); // if calibration happening, always apply filter: no extreme jumps during calibration
        
        Serial.print("after: ");
        Serial.println( lpf_value[which][i] );
        
      } // end of calibrating

      Serial.print("Calibration readings are: ");
      Serial.print( adc_value[which][i] );
      Serial.print(",");
    } // end of channel 0-3
    Serial.println();
  } // end of which < 6
} // end of function


void setup() {

  Serial.begin(9600);
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
  for ( int i = 0; i < 10; i++ ) { // read ADC 100 times for calibration; LPF = average previous and current values to smooth out noise.
    for ( int chip = 0; chip < 6; chip++ ) {
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

  adc_update_ts = millis();
}

//==================================== MAIN LOOP ===================================
void loop() {

  timeStep += 1;
  Serial.print("***************************************************************timeStep ");
  Serial.println(timeStep);

  //  //testing the cords are sending expected outputs
  //  Serial.println(read_adc_chip_ch( 3 , 0 ));
  //  Serial.print(",");
  //  Serial.println(read_adc_chip_ch( 5 , 2 ));
  //  Serial.print(",");
  //  float total_voltage = read_adc_chip_ch( 3 , 0 ) + read_adc_chip_ch( 5 , 2 );
  //  //Serial.println(" TOTAL VOLTAGE = ");
  //  Serial.print(total_voltage);
  //  Serial.println(',');

  // Move motor: this is non-blocking and
  // will handle it's own timing
  //  unsigned long start_time = millis();
  updateMotor(0);
  updateMotor(1);
  //  unsigned long end_time = millis();
  //  Serial.println( (end_time - start_time));

}
//==================================== END MAIN LOOP ===================================

// -------------------------------------- MOVING THE MOTORS FUNCTIONS ------------------------------------------

void moveStepsDown(int motorIndex, int steps_to_move) {
  int step_count = 0;

  if (steps_to_move <= 0) return; // No illogical steps

  // Otherwise, move by the requested amount
  while (step_count < steps_to_move) {
    moveDown(motorIndex); // Specify motor index
    step_count++;
  }
}

void moveStepsUp(int motorIndex, int steps_to_move) {
  int step_count = 0;

  if (steps_to_move <= 0) return; // No illogical steps

  // Otherwise, move by the requested amount
  while (step_count < steps_to_move) {
    moveUp(motorIndex); // Specify motor index
    step_count++;
  }
}

void moveDown(int motorIndex) {
  digitalWrite(dir_pin[motorIndex], dir_pin_state[motorIndex]);

  // TO make the motor move one step (control specific motor)
  digitalWrite(step_pin[motorIndex], step_pin_state[motorIndex]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US); // Adds delay
  digitalWrite(step_pin[motorIndex], !step_pin_state[motorIndex]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US); // Adds delay
}

void moveUp(int motorIndex) {
  digitalWrite(dir_pin[motorIndex], !dir_pin_state[motorIndex]);

  // TO make the motor move one step (control specific motor)
  digitalWrite(step_pin[motorIndex], step_pin_state[motorIndex]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US);
  digitalWrite(step_pin[motorIndex], !step_pin_state[motorIndex]);
  delayMicroseconds(DEFAULT_STEP_DELAY_US); // Adds delay
}

void read_adc_MOTOR(int motorIndex, float & ownVoltage, float & neighVoltage) {
  ownVoltage = 0;
  neighVoltage = 0;

  // Read "own" ADC values
  const int motor_ADC_own[2] = {motor_ADC_own_1[motorIndex], motor_ADC_own_2[motorIndex]};
  const int motor_channel_own[2] = {motor_channel_own_1[motorIndex], motor_channel_own_2[motorIndex]};

  // Sequentially read own ADC channels
  for (int channel = 0; channel < 2; channel++) {
    int adcChip = motor_ADC_own[channel];   // ADC chip index
    int channel_mask = channel_key[ motor_channel_own[channel] ] ; // ADC channel index, check printing right channels

    // Ensure the ADC read is completed before continuing
    int32_t reading = adc_chip[adcChip].Read_SingleShot_SingleEnded_WaitForData(channel_mask);
//    Serial.print("RAW VALUES ");
//    Serial.println(reading); 
    float convertedVoltage = convertToMilliV(reading);
    //printing without lpf
    ownVoltage += convertedVoltage;
    Serial.print("own cords converted voltage: ");
    Serial.println(ownVoltage);

    // Apply filtering
    if (!CALIBRATING) {
      Serial.print("before abs(): ");
      Serial.println( lpf_value[adcChip][channel] );
      if (abs(convertedVoltage) < (lpf_value[adcChip][channel] * 50)) {
        
        lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
        Serial.println(lpf_value[adcChip][channel]);
        Serial.println("CALIBRATING.............");
        
      } else {
        Serial.println(" abs() was not true");
      }
    } else {
      lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
      Serial.print(";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;");
      Serial.println(lpf_value[adcChip][channel]);
    }
    Serial.print("paul: ");
    Serial.println(lpf_value[adcChip][channel]);
    ownVoltage += lpf_value[adcChip][channel];
    Serial.print("own voltage in ADC function: ");
    Serial.println(ownVoltage);

    // Optional: Add a small delay if needed to ensure enough time for the conversion to complete
    delay(10); // Example: delay by 10 milliseconds, adjust as needed
  }

  // Read "neighbor" ADC values
  const int motor_ADC_neigh[2] = {motor_ADC_neigh_1[motorIndex], motor_ADC_neigh_2[motorIndex]}; // 1,2
  const int motor_channel_neigh[2] = {motor_channel_neigh_1[motorIndex], motor_channel_neigh_2[motorIndex]};

  // Sequentially read neighbor ADC channels
  for (int channel = 0; channel < 2; channel++) {
    int adcChip = motor_ADC_neigh[channel];  // ADC chip index // 1
    int channel_mask = channel_key[ motor_channel_neigh[channel] ]; // ADC channel index // 1

    // Ensure the ADC read is completed before continuing
    int32_t reading = adc_chip[adcChip].Read_SingleShot_SingleEnded_WaitForData(channel_mask);
    float convertedVoltage = convertToMilliV(reading);
    //printing without lpf
    neighVoltage += convertedVoltage;
    Serial.print("neigh cords converted voltage: ");
    Serial.println(neighVoltage);

//    // Apply filtering
    if (!CALIBRATING) {
      Serial.println( "not calibrating");
      if (abs(convertedVoltage) < (lpf_value[adcChip][channel] * 50)) {
        Serial.println(convertedVoltage ,4);
        lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
         
      } else {
        Serial.println("value is bad?");
      }
    } else {
      Serial.println("calibrating");
      Serial.println(convertedVoltage ,4);
      lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
    }

    Serial.println( lpf_value[adcChip][channel], 4 );
    neighVoltage += lpf_value[adcChip][channel];
    Serial.print("neigh voltage in ADC function: ");
    Serial.println(neighVoltage);

    // Add a small delay if needed to ensure enough time for the conversion to complete
    delay(50); // Example: delay by 10 milliseconds, adjust as needed
  }
}

// -------------------------------------- MAIN UPDATE FUNCTION ------------------------------------------
void updateMotor(int motor) {
  // For each motor, check if it is time to send a step
  // and what total cord values are vs threshold
  float ownVoltage;
  float neighVoltage;

  // update ADC readings before using them
  if (millis() - adc_update_ts > 10) {  // was 50 in default
    adc_update_ts = millis();

    // Loop through each motor
    for (int motor = 0; motor < nb_motors; motor++) {
      ownVoltage = 0; // maybe before for loop?
      neighVoltage = 0;

      // Take own and neighbor voltage of motor
      read_adc_MOTOR(motor, ownVoltage, neighVoltage);

      Serial.print("Motor ");
      Serial.print(motor);
      Serial.print(" Own Voltage: ");
      Serial.println(ownVoltage);
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.print(" Neighbor Voltage: ");
      Serial.println(neighVoltage);

      // Calculate the error to total voltage
      float local_err = local_demand - ownVoltage;
      Serial.print("----------------> local error is: ");
      Serial.print(local_err);
      Serial.println("\n");

      // Work out the difference with neighbor
      float diff_neigh = ownVoltage - neighVoltage;
      Serial.print("-----------------> Neighbour Difference is: ");
      Serial.print(diff_neigh);
      Serial.println("\n");

      // work out if should move
      if ( micros() - step_us_ts[motor] > step_delay[motor] ) {
        // micros = time in microseconds since Arduino started (used for very short intervals),  step_us_ts = when last stepped, step_delay = how often should step
        // if enough time has passed, then send step signal
        step_us_ts[motor] = micros(); // updating the last step time, storing current time


        //Working out actuation signal to send to motors depending on demand + neighbour
        int actuation_signal_up = 0; //will carry the local + neighbour actuation for shortening
        int actuation_signal_down = 0; //will carry the local + neighbour actuation for elongating
        int actuation_final = 0;

        if ( abs(local_err) < local_threshold ) {
          //stop moving
          Serial.print(motor);
          Serial.println(" LOCAL DEMAND ACHIEVED!");
        }
        else if (local_err > 0) {
          //moveStepsUp(motor, actuation_step * local_weight);
          actuation_signal_up += (actuation_step * local_weight);
          Serial.print(motor);
          Serial.println(" Increases voltage to local");
        }
        else {
          //moveStepsDown(motor, actuation_step * local_weight);
          actuation_signal_down += (actuation_step * local_weight);
          Serial.print(motor);
          Serial.println(" Decreases voltage to local");
        }

        // work out neighbour

        if ( abs(diff_neigh) < neigh_threshold ) {
          //stop moving
          Serial.print(motor);
          Serial.println(" NEIGHBOUR ACHIEVED");
        }
        else if (diff_neigh > 0) { // M0 bigger than M1
          if (neighbour_condition == -1) {
            //moveStepsUp(motor, actuation_step * neigh_weight); // shorten
            actuation_signal_up += (actuation_step * neigh_weight);
            Serial.print(motor);
            Serial.println(" Decreases voltage to converge");
          }
          else if (neighbour_condition == 1) {
            //moveStepsDown(motor, actuation_step * neigh_weight); // elongates
            actuation_signal_down += (actuation_step * neigh_weight);
            Serial.print(motor);
            Serial.println(" increase voltage to diverge");
          }
          else {
            Serial.print(motor);
            Serial.println(" INCORRECT NEIGHBOUR CONDITION");
          }  // end of neighbour_condition block

        }  // end of if (diff_neigh > 0) block

        else if (diff_neigh < 0) { // M1 bigger than M0
          if (neighbour_condition == -1) {
            //moveStepsDown(motor, actuation_step * neigh_weight);
            actuation_signal_down += (actuation_step * neigh_weight);
            Serial.print(motor);
            Serial.println(" Increase voltage to converge");
          }
          else if (neighbour_condition == 1) {
            //moveStepsUp(motor, actuation_step * neigh_weight);
            actuation_signal_up += (actuation_step * neigh_weight);
            Serial.print(motor);
            Serial.println(" Decrease voltage to diverge");
          }
          else {
            Serial.print(motor);
            Serial.println(" INCORRECT NEIGHBOUR CONDITION");
          }
        }  // end of the if (diff_neigh < 0) block

        // MOVE
        actuation_final = actuation_signal_down - actuation_signal_up; //(elongate - shortening)

        if (actuation_final > 0) {
          moveStepsDown(motor, actuation_final);
        } else if (actuation_final < 0) {
          moveStepsUp(motor, abs(actuation_final));
        } else {
          Serial.print(motor);
          Serial.println(" No movement needed.");
        }

        Serial.print(motor);
        Serial.print(" FINAL ACTUATION: ");
        Serial.println(actuation_final); // if negative means shortens

      } // end of check if can move
    } // looping through motors
  } // checking time for ADC reading
} // end of function


// -------------------------------------- END OF MAIN UPDATE FUNCTION ------------------------------------------
