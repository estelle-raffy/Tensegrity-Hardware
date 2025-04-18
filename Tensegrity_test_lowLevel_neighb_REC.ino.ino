// This code records DATA : 70 timesteps => 70 max_results (~70% memory capacity), 60 seconds duration, Interval (=timestep) ~426.248ms, 
// This code also builds on the low-level code and adds a neighbour condition
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

// define 'container' to return mutiple variables from update function
struct MotorData {
  float ownVoltage;
  float neighVoltage;
  float local_err;
  float diff_neigh;
  int actuation_final;
};

const int cs_pin[6]    = {10, 11, 17, 16, 15, 14}; // pins for ADC, check pins 10 and 11 actually work!
const int drdy_pin[6]  = {2, 3, 18, 19, 20, 21};

const int channel_key[4] = { MUX_SE_CH0, MUX_SE_CH1, MUX_SE_CH2, MUX_SE_CH3};

// ADCs and channels per motor >> depending on how many motors test
const int nb_motors = 3;

const int motor_ADC_own_1[nb_motors] = {3, 0, 0};
const int motor_ADC_own_2[nb_motors] = {5, 0, 1};
const int motor_channel_own_1[nb_motors] = {0, 3, 2};
const int motor_channel_own_2[nb_motors] = {1, 1, 1};
const int motor_ADC_neigh_1[nb_motors] = {0, 0, 2};
const int motor_ADC_neigh_2[nb_motors] = {5, 0, 2};
const int motor_channel_neigh_1[nb_motors] = {1, 2, 2};
const int motor_channel_neigh_2[nb_motors] = {2, 0, 3};
//const int motor_ADC_neigh[2] = {motor_ADC_neigh_1[motorIndex], motor_ADC_neigh_2[motorIndex]};
// M3 own 0-2 & 1-1 ; M3 neigh 2-2 & 2-3

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

int local_demand = 3700; // 3700 not moving!
int local_threshold = 100; // goes with 3700
int neighbour_condition = 1; // -1 same voltage; 1 different voltage
int neigh_threshold_converge = 40; // based on data
int neigh_threshold_diverge = 200; 
int actuation_step = 10; // allows to see the motor move, 1 was too small; default 10
int local_weight = 1; // how much local affects bhv, 0 = OFF
int neigh_weight = 1; // how much neigh affects bhv, 0 = OFF

// **************************************************** END OF LOWER-LEVEL VARIABLES, WEIGHTS, PARAMETERS **************************

// like in simulation, we want to record variable for each module
#define MAX_RESULTS 70 // agree with memory capacity (~70%),try to take as much as possible 
#define VARIABLES 6 // what variable tracking?
float motor0_results[MAX_RESULTS][VARIABLES]; // Motor 1 results
float motor1_results[MAX_RESULTS][VARIABLES]; // Motor 2 results
float motor2_results[MAX_RESULTS][VARIABLES]; // Motor 3 results
int results_index; // track position of results in array from 0 to MAX_RESULTS

// State where motors should run/stop
# define STATE_RUNNING_EXPERIMENT  0
# define STATE_FINISHED_EXPERIMENT 1
int state;

// Time stamp to track whether the experiment
// duration has elapsed.
unsigned long experiment_start_ts;
# define EXPERIMENT_END_MS 60000 // 60 seconds

// We can try to automate the time interval
// of storing results.
unsigned long record_results_ts;
unsigned long results_interval_ms;

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

      Serial.print("Calibration readings are: ");
      Serial.print( adc_value[which][i] );
      Serial.print(",");
    } // end of channel 0-3

  } // end of which < 6
} // end of function


void setup() {

  results_index = 0;
  results_interval_ms = 426; // one timestep, update time of the 6 variables 

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

  state = STATE_RUNNING_EXPERIMENT;

  Serial.begin(9600);

  experiment_start_ts = millis();
  record_results_ts = millis();
  adc_update_ts = millis();
}

//==================================== MAIN LOOP ===================================
void loop() {

  if (state == STATE_RUNNING_EXPERIMENT) {

    timeStep += 1;
    Serial.print("***************************************************************timeStep ");
    Serial.println(timeStep);

    // Updates 
    unsigned long timeStep_starts = micros();
    MotorData motor0_data = updateMotor(0);
    MotorData motor1_data = updateMotor(1);
    MotorData motor2_data = updateMotor(2);
    unsigned long timeStep_ends = micros();
    Serial.println(timeStep_ends - timeStep_starts);

    unsigned long elapsed_time;
    elapsed_time = millis() - record_results_ts;

    if (elapsed_time > results_interval_ms) {
      // Move time stamp forwards for next iteration.
      record_results_ts = millis();

      // Let's be safe and check we haven't
      // filled up the results array already.
      if (results_index < MAX_RESULTS) {
        // Store motor 0 results
        motor0_results[results_index][0] = local_demand;
        motor0_results[results_index][1] = motor0_data.ownVoltage;
        motor0_results[results_index][2] = motor0_data.neighVoltage;
        motor0_results[results_index][3] = motor0_data.local_err;
        motor0_results[results_index][4] = motor0_data.diff_neigh;
        motor0_results[results_index][5] = motor0_data.actuation_final;

        // Store motor 1 results
        motor1_results[results_index][0] = local_demand;
        motor1_results[results_index][1] = motor1_data.ownVoltage;
        motor1_results[results_index][2] = motor1_data.neighVoltage;
        motor1_results[results_index][3] = motor1_data.local_err;
        motor1_results[results_index][4] = motor1_data.diff_neigh;
        motor1_results[results_index][5] = motor1_data.actuation_final;

        // Store motor 2 results
        motor2_results[results_index][0] = local_demand;
        motor2_results[results_index][1] = motor2_data.ownVoltage;
        motor2_results[results_index][2] = motor2_data.neighVoltage;
        motor2_results[results_index][3] = motor2_data.local_err;
        motor2_results[results_index][4] = motor2_data.diff_neigh;
        motor2_results[results_index][5] = motor2_data.actuation_final;

        // Increment result index for next time.
        results_index++;
        Serial.print("Results Index: ");
        Serial.println(results_index);

      } else {
        // If RESULTS_MAX has been reached, the experiment is finished.
        state = STATE_FINISHED_EXPERIMENT;
        return;
      }
    }

    // Has the experiment duration been reached?
    elapsed_time = millis() - experiment_start_ts;
    if (elapsed_time > EXPERIMENT_END_MS) {
      // Transition to finished state
      state = STATE_FINISHED_EXPERIMENT;
      return;
    }

  } else if (state == STATE_FINISHED_EXPERIMENT) {

    // Once the experiment is finished, stop the motors
    moveStepsDown(0, 0);
    moveStepsDown(1, 0);
    moveStepsDown(2, 0);

    // Loop through the results and print them
    int result;
    Serial.println("Sample, Motor, Local Demand, Own Voltage, Neighbour Voltage, Local Error, Neighbour Difference, Final Actuation");

    for (result = 0; result < MAX_RESULTS; result++) {
      // Print the sample number, use result + 1 for 1-based indexing
      Serial.print(result + 1); // Print sample number (1-based indexing)
      Serial.print(",");

      // Motor 0 Data
      Serial.print("Motor 0,");
      Serial.print(motor0_results[result][0]); // Local Demand
      Serial.print(",");
      Serial.print(motor0_results[result][1]); // Own Voltage
      Serial.print(",");
      Serial.print(motor0_results[result][2]); // Neighbour Voltage
      Serial.print(",");
      Serial.print(motor0_results[result][3]); // Local Error
      Serial.print(",");
      Serial.print(motor0_results[result][4]); // Neighbour Difference
      Serial.print(",");
      Serial.print(motor0_results[result][5]); // Final Actuation for Motor 0
      Serial.print("\n");  // Newline after Motor 0

      // Motor 1 Data (same sample number)
      Serial.print(result + 1);  // Reprint the sample number for Motor 1
      Serial.print(",");
      Serial.print("Motor 1,");
      Serial.print(motor1_results[result][0]); // Local Demand
      Serial.print(",");
      Serial.print(motor1_results[result][1]); // Own Voltage
      Serial.print(",");
      Serial.print(motor1_results[result][2]); // Neighbour Voltage
      Serial.print(",");
      Serial.print(motor1_results[result][3]); // Local Error
      Serial.print(",");
      Serial.print(motor1_results[result][4]); // Neighbour Difference
      Serial.print(",");
      Serial.print(motor1_results[result][5]); // Final Actuation for Motor 1
      Serial.print("\n");  // Newline after Motor 1

      // Motor 2 Data (same sample number)
      Serial.print(result + 1);  // Reprint the sample number for Motor 2
      Serial.print(",");
      Serial.print("Motor 2,");
      Serial.print(motor2_results[result][0]); // Local Demand
      Serial.print(",");
      Serial.print(motor2_results[result][1]); // Own Voltage
      Serial.print(",");
      Serial.print(motor2_results[result][2]); // Neighbour Voltage
      Serial.print(",");
      Serial.print(motor2_results[result][3]); // Local Error
      Serial.print(",");
      Serial.print(motor2_results[result][4]); // Neighbour Difference
      Serial.print(",");
      Serial.print(motor2_results[result][5]); // Final Actuation for Motor 2
      Serial.print("\n");  // Newline after Motor 2
    }
    // A delay to allow time for copying results
    delay(3000);
  }
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
  for (int i = 0; i < 2; i++) {
    int adcChip = motor_ADC_own[i];   // ADC chip index
    int channel = channel_key[ motor_channel_own[i] ] ; // ADC channel index, check printing right channels

    // Ensure the ADC read is completed before continuing
    int32_t reading = adc_chip[adcChip].Read_SingleShot_SingleEnded_WaitForData(channel);
    float convertedVoltage = convertToMilliV(reading);
    //printing without lpf
    ownVoltage += convertedVoltage;
    //Serial.print("own cords converted voltage: ");
    //Serial.println(ownVoltage);

    // Apply filtering
    //    if (!CALIBRATING) {
    //      if (abs(convertedVoltage) < (lpf_value[adcChip][channel] * 50)) {
    //        lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
    //        Serial.println(lpf_value[adcChip][channel]);
    //        Serial.println("CALIBRATING.............");
    //      }
    //    } else {
    //      lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
    //      Serial.print(";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;");
    //      Serial.println(lpf_value[adcChip][channel]);
    //    }
    //
    //    ownVoltage += lpf_value[adcChip][channel];
    //    Serial.print("own voltage in ADC function: ");
    //    Serial.println(ownVoltage);

    // Optional: Add a small delay if needed to ensure enough time for the conversion to complete
    delay(10); // Example: delay by 10 milliseconds, adjust as needed
  }

  // Read "neighbor" ADC values
  const int motor_ADC_neigh[2] = {motor_ADC_neigh_1[motorIndex], motor_ADC_neigh_2[motorIndex]}; // 1,2
  const int motor_channel_neigh[2] = {motor_channel_neigh_1[motorIndex], motor_channel_neigh_2[motorIndex]};

  // Sequentially read neighbor ADC channels
  for (int i = 0; i < 2; i++) {
    int adcChip = motor_ADC_neigh[i];  // ADC chip index // 1
    int channel = channel_key[ motor_channel_neigh[i] ]; // ADC channel index // 1

    // Ensure the ADC read is completed before continuing
    int32_t reading = adc_chip[adcChip].Read_SingleShot_SingleEnded_WaitForData(channel);
    float convertedVoltage = convertToMilliV(reading);
    //printing without lpf
    neighVoltage += convertedVoltage;
    //Serial.print("neigh cords converted voltage: ");
    //Serial.println(neighVoltage);

    //    // Apply filtering
    //    if (!CALIBRATING) {
    //      if (abs(convertedVoltage) < (lpf_value[adcChip][channel] * 50)) {
    //        lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
    //      }
    //    } else {
    //      lpf_value[adcChip][channel] = (lpf_value[adcChip][channel] * 0.9) + (convertedVoltage * 0.1);
    //    }

    //    neighVoltage += lpf_value[adcChip][channel];
    //    Serial.print("neigh voltage in ADC function: ");
    //    Serial.println(neighVoltage);

    // Add a small delay if needed to ensure enough time for the conversion to complete
    delay(50); // Example: delay by 10 milliseconds, adjust as needed
  }
}

// -------------------------------------- MAIN UPDATE FUNCTION ------------------------------------------
MotorData updateMotor(int motor) {
  MotorData data; // Declare struct 'container'

  // Initialize struct variables to avoid undefined values
  data.ownVoltage = 0;
  data.neighVoltage = 0;
  data.local_err = 0;
  data.diff_neigh = 0;
  data.actuation_final = 0;

  // update ADC readings before using them
  if (millis() - adc_update_ts > 10) {  // was 50 in default
    adc_update_ts = millis();

    // No loop here, just handle the specific motor passed to the function
    data.ownVoltage = 0;
    data.neighVoltage = 0;

    // Take own and neighbor voltage of the specified motor
    read_adc_MOTOR(motor, data.ownVoltage, data.neighVoltage);

//    Serial.print("Motor ");
//    Serial.print(motor);
//    Serial.print(" Own Voltage: ");
//    Serial.println(data.ownVoltage);
//    Serial.print("Motor ");
//    Serial.print(motor);
//    Serial.print(" Neighbor Voltage: ");
//    Serial.println(data.neighVoltage);

    // Calculate the error to total voltage
    data.local_err = local_demand - data.ownVoltage; //no longer defined as float since global
//    Serial.print("----------------> local error is: ");
//    Serial.print(data.local_err);
//    Serial.println("\n");

    // Work out the difference with neighbor
    data.diff_neigh = data.ownVoltage - data.neighVoltage; //no longer defined as float since global
//    Serial.print("-----------------> Neighbour Difference is: ");
//    Serial.print(data.diff_neigh);
//    Serial.println("\n");

    // work out if should move
    if (micros() - step_us_ts[motor] > step_delay[motor]) {
      // micros = time in microseconds since Arduino started (used for very short intervals),  step_us_ts = when last stepped, step_delay = how often should step
      // if enough time has passed, then send step signal
      step_us_ts[motor] = micros(); // updating the last step time, storing current time

      // Working out actuation signal to send to motors depending on demand + neighbour
      int actuation_signal_up = 0; // will carry the local + neighbour actuation for shortening
      int actuation_signal_down = 0; // will carry the local + neighbour actuation for elongating

      if (abs(data.local_err) < local_threshold) {
        // stop moving
        actuation_signal_up = 0;
        actuation_signal_down = 0;
//        Serial.print(motor);
//        Serial.println(" LOCAL DEMAND ACHIEVED!");
      } else if (data.local_err > 0) {
        //moveStepsUp(motor, actuation_step * local_weight);
        actuation_signal_up += (actuation_step * local_weight);
//        Serial.print(motor);
//        Serial.println(" Increases voltage to local");
      } else {
        //moveStepsDown(motor, actuation_step * local_weight);
        actuation_signal_down += (actuation_step * local_weight);
//        Serial.print(motor);
//        Serial.println(" Decreases voltage to local");
      }

      // work out neighbour
      if (neighbour_condition == -1) {  // Should be as close as possible
        // Check if local error is within threshold
        if (abs(data.diff_neigh) < neigh_threshold_converge) {
          // stop moving
          actuation_signal_up = 0;
          actuation_signal_down = 0;
//          Serial.print(motor);
//          Serial.println(" LOCAL DEMAND ACHIEVED!");
        } else if (data.diff_neigh > 0) {  // // M0 bigger than M1, decrease voltage to converge
          //moveStepsDown(motor, actuation_step * local_weight);  // elongates
          actuation_signal_down += (actuation_step * local_weight);
//          Serial.print(motor);
//          Serial.println(" decreases voltage to local");
        } else {  // M0 smaller than M1, increase voltage to converge
          //moveStepsUp(motor, actuation_step * local_weight);  // shortens
          actuation_signal_up += (actuation_step * local_weight);
//          Serial.print(motor);
//          Serial.println(" Increases voltage to local");
        }
      } else if (neighbour_condition == 1) {  // Should be as different as possible (increase the difference)
        // Check if the neighbour difference is within the allowable range
        if (abs(data.diff_neigh) > neigh_threshold_diverge) {
          // Stop moving if the difference exceeds 200
          actuation_signal_up = 0;
          actuation_signal_down = 0;
//          Serial.print(motor);
//          Serial.println(" Neighbour difference too large, no more movement allowed");
        } else if (data.diff_neigh > 0) { //M0 bigger than M1, keeps getting big
          actuation_signal_up += (actuation_step * neigh_weight);  // keep shortening
//          Serial.print(motor);
//          Serial.println(" Increases voltage to diverge");
        } else { // M0 smaller than M1, keeps getting small
          actuation_signal_down += (actuation_step * neigh_weight);  // keep elongating
//          Serial.print(motor);
//          Serial.println(" Decreases voltage to diverge");
        }
      } else {
//        Serial.print(motor);
//        Serial.println(" INCORRECT NEIGHBOUR CONDITION");
      }

      // MOVE
      data.actuation_final = actuation_signal_down - actuation_signal_up; // (elongate - shortening)

      if (data.actuation_final > 0) {
        moveStepsDown(motor, data.actuation_final); // elongates - increases voltage 
      } else if (data.actuation_final < 0) {
        moveStepsUp(motor, abs(data.actuation_final)); // shortens - decreases voltage
      } else {
//        Serial.print(motor);
//        Serial.println(" No movement needed.");
      }
//
//      Serial.print(motor);
//      Serial.print(" FINAL ACTUATION: ");
//      Serial.println(data.actuation_final); // if negative means shortens --> voltage increases 
    } // end of check if can move
  } // checking time for ADC reading

  return data; // Return data for the specific motor requested
}// end of function

// -------------------------------------- END OF MAIN UPDATE FUNCTION ------------------------------------------
