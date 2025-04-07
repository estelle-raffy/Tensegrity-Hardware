// Gets & logs current positions of pink endcap fed by python script
// **Need to fix** TIMING & EXCEL HANDLING 

// Records 35 MAXRESULTS (71% memory capacity) at interval 827.89ms (Timestep ~72 ), only 35 results = REC only half experiment...
// 6000s / 827.89 = 72 but cannot collect 72 results (1/timestep), so will stop at 35. 
// could work out a different interval: every 2 timestep: 6000/ 1656 = ~36, so longer interval may allow more results
// ... or could do shorter experiment: 35×827.89 = ~28976 ms (29s)


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
struct ExperimentData {
  float ownVoltage;
  float neighVoltage;
  float local_err;
  float newVoltage;
  float local_err_after_actuation;
  float diff_neigh;
  int actuation_final;
  float local_integrated_frustration;
  float error_from_python;
  float global_integrated_frustration;
  float neigh_weight;
  float motor0_weight;
  float motor1_weight;
  float motor2_weight;
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
#define DEBUG false   

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
int local_threshold = 100; // goes with 3700. can be adjusted to serve as a happiness range
int neigh_threshold_converge = 40; // 40 based on data
int neigh_threshold_diverge = 200;
float local_integrated_error_motors[3] = {0.0, 0.0, 0.0}; // will serve as the 'old error' for the leakage
float local_integrated_frustration_motors[3] = {0.0, 0.0, 0.0}; // cannot reach good local/neigh compromise will be integrated over time
float local_frustration_threshold = 300; // since taking the absolute value of error, no need for min_value
float local_recovery_threshold = 275; // lower to prevent oscillations at the frustration threshold, allows system to stabilize
float local_emotional_momentum = 2.5; // how sensitive to error/quickly frustrated? 0.5: slow, 2.5: quick, 1: reset 
float lambda_local = 0.8 ;// 1 = all the error leaks out!, want a faster leak for ST effect, LT for global so lower values
int actuation_step = 10; // allows to see the motor move, 1 was too small; default 10
float local_weight = 1.0; // how much local affects bhv, 0 = OFF
float neigh_weights[3] = {1, 1, 1}; // neigh influence, 0 = OFF; for each motor will be changed by selfish and pass through global check
float reset_neigh_weights[3] = {1, 1, 1}; // when global check turns off selfish --> reset neigh_weights to 1
int neighbour_condition = -1; // -1 same voltage; 1 different voltage

// **************************************************** END OF LOWER-LEVEL VARIABLES, WEIGHTS, PARAMETERS **************************

// **************************************************** HIGHER-LEVEL VARIABLES, WEIGHTS, PARAMETERS ************************** 
float error_from_python = 0.0;
int global_threshold = 100; // happiness range, depending on how stressed/relaxed needs to be
float global_integrated_error = 0; // will serve as the 'old error' for the leakage
float global_integrated_frustration = 0; // cannot reach global target state threshold
float global_frustration_threshold = 4000; // 4000 since taking the absolute value of error, no need for min_value
float global_recovery_threshold = 3800; // 3800 lower to prevent oscillations at the frustration threshold, allows system to stabilize
float global_emotional_momentum = 0.5; // how sensitive to error/quickly frustrated? 0.5: slow, 2.5: quick, 1: reset 
float lambda_global = 0.2 ; // 1 = all the error leaks out!, want a slower leak for LT effect than local
bool beingSelfish = true; // whether lower-system only focuses on local demand, if true, then turned OFF

// **************************************************** END OF HIGHER-LEVEL VARIABLES, WEIGHTS, PARAMETERS **************************

// like in simulation, we want to record variable for each module
#define MAX_RESULTS 35 // agree with memory capacity (<75%),try to take as much as possible
#define MOTOR_VARIABLES 10 // what variable tracking?
#define FRUSTRATION_VARIABLES 5
float motor0_results[MAX_RESULTS][MOTOR_VARIABLES]; // Motor 1 results
float motor1_results[MAX_RESULTS][MOTOR_VARIABLES]; // Motor 2 results
float motor2_results[MAX_RESULTS][MOTOR_VARIABLES]; // Motor 3 results
float frustration_results[MAX_RESULTS][FRUSTRATION_VARIABLES]; // python error + integrated frustration + final weights 
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
  float value = (float)((i32data * VFSR * 1000) / FULL_SCALE);
  if ( DEBUG) Serial.print("convertToMilliV: ");
  if ( DEBUG) Serial.println( value, 4);
  return value;
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

      if ( DEBUG )Serial.print("Calibration readings are: ");
      if ( DEBUG )Serial.print( adc_value[which][i] );
      if ( DEBUG )Serial.print(",");
    } // end of channel 0-3

  } // end of which < 6
} // end of function


void setup() {

  results_index = 0;
  results_interval_ms = 828; // = 1 timestep ~827.89ms

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

  //delay(90000); // takes camera ~90s to warm up

  // Just for debugging, comment out!
//  while( true ) {
//    parseSerialPort();
//    delay(10);
//  }
  
}

//==================================== MAIN LOOP ==============================================================
void loop() {

  if (state == STATE_RUNNING_EXPERIMENT) {

    timeStep += 1;
    if ( DEBUG )Serial.print("***************************************************************timeStep ");
    if ( DEBUG )Serial.println(timeStep);

    unsigned long timeStep_starts = micros();
    
    ExperimentData motor0_data; // 0
    ExperimentData motor1_data;
    ExperimentData motor2_data;

    ///////////// MOTOR 1 UPDATE + extracting local error & state
    motor0_data = updateMotor(0);
    motor0_results[results_index][1] = motor0_data.newVoltage; // state of M0 for global
    motor0_results[results_index][3] = motor0_data.local_err_after_actuation; // local error of M0 for selfishness
    ExperimentData motor0_frustration_local = being_selfish(0, motor0_data.local_err_after_actuation, local_integrated_error_motors[0], local_integrated_frustration_motors[0]);
    motor0_results[results_index][9] = motor0_frustration_local.neigh_weight; // get the local neigh weight from selfishness

    ///////////// MOTOR 1 UPDATE + extracting local error & state

    motor1_data = updateMotor(1);
    motor1_results[results_index][3] = motor1_data.local_err_after_actuation; // local error of M1
    motor1_results[results_index][1] = motor1_data.newVoltage; // State of M1
    ExperimentData motor1_frustration_local = being_selfish(1, motor1_data.local_err_after_actuation, local_integrated_error_motors[1], local_integrated_frustration_motors[1]);
    motor1_results[results_index][9] = motor1_frustration_local.neigh_weight;

    ///////////// MOTOR 2 UPDATE + extracting local error & state

    motor2_data = updateMotor(2);
    motor2_results[results_index][3] = motor2_data.local_err_after_actuation; // local error of M2
    motor2_results[results_index][1] = motor2_data.newVoltage; // state of M2
    ExperimentData motor2_frustration_local = being_selfish(2, motor2_data.local_err_after_actuation, local_integrated_error_motors[2], local_integrated_frustration_motors[2]);
    motor2_results[results_index][9] = motor2_frustration_local.neigh_weight;

    ///////////// GLOBAL modulation
    if ( DEBUG ) Serial.println("\n");
    if ( DEBUG ) Serial.println("&&&&&&&&&&&&  Entering GLOBAL &&&&&&&&&&&");

    // get current error from python (error to target position in space)
    receiveErrorFromPython();
    if ( DEBUG )Serial.print("Error from Python function call in main loop is: ");
    if ( DEBUG )Serial.println(error_from_python, 2);  // Print with 2 decimal places
    
    ExperimentData frustration_global = global_function(error_from_python);
    frustration_results[results_index][0] = frustration_global.error_from_python;
    if ( DEBUG )Serial.print("Error from Python after global call in main loop is: ");
    if ( DEBUG )Serial.println(error_from_python, 2);
    
    //frustration_results[results_index][3] = frustration_global.motor1_weight;
    //frustration_results[results_index][4] = frustration_global.motor2_weight;

    
    unsigned long timeStep_ends = micros();
    if ( DEBUG ) Serial.println(timeStep_ends - timeStep_starts);


    unsigned long elapsed_time;
    elapsed_time = millis() - record_results_ts;

    if (elapsed_time > results_interval_ms) {
      // Move time stamp forwards for next iteration.
      record_results_ts = millis();

      // Let's be safe and check we haven't
      // filled up the results array already.
      if (results_index < MAX_RESULTS) {
        // Store motor 0 results & frustration
        motor0_results[results_index][0] = local_demand;
        motor0_results[results_index][1] = motor0_data.ownVoltage;
        motor0_results[results_index][2] = motor0_data.neighVoltage;
        motor0_results[results_index][3] = motor0_data.local_err;
        motor0_results[results_index][4] = motor0_data.diff_neigh;
        motor0_results[results_index][5] = motor0_data.actuation_final;
        motor0_results[results_index][6] = motor0_data.newVoltage;
        motor0_results[results_index][7] = motor0_data.local_err_after_actuation;
        motor0_results[results_index][8] = motor0_frustration_local.local_integrated_frustration;
        motor0_results[results_index][9] = motor0_frustration_local.neigh_weight;

        // Store motor 1 results
        motor1_results[results_index][0] = local_demand;
        motor1_results[results_index][1] = motor1_data.ownVoltage;
        motor1_results[results_index][2] = motor1_data.neighVoltage;
        motor1_results[results_index][3] = motor1_data.local_err;
        motor1_results[results_index][4] = motor1_data.diff_neigh;
        motor1_results[results_index][5] = motor1_data.actuation_final;
        motor1_results[results_index][6] = motor1_data.newVoltage;
        motor1_results[results_index][7] = motor1_data.local_err_after_actuation;
        motor1_results[results_index][8] = motor1_frustration_local.local_integrated_frustration;
        motor1_results[results_index][9] = motor1_frustration_local.neigh_weight;

        // Store motor 2 results
        motor2_results[results_index][0] = local_demand;
        motor2_results[results_index][1] = motor2_data.ownVoltage;
        motor2_results[results_index][2] = motor2_data.neighVoltage;
        motor2_results[results_index][3] = motor2_data.local_err;
        motor2_results[results_index][4] = motor2_data.diff_neigh;
        motor2_results[results_index][5] = motor2_data.actuation_final;
        motor2_results[results_index][6] = motor2_data.newVoltage;
        motor2_results[results_index][7] = motor2_data.local_err_after_actuation;
        motor2_results[results_index][8] = motor2_frustration_local.local_integrated_frustration;
        motor2_results[results_index][9] = motor2_frustration_local.neigh_weight;


        // Store global frustration levels
        frustration_results[results_index][0] = frustration_global.error_from_python;
        frustration_results[results_index][1] = frustration_global.global_integrated_frustration;
        frustration_results[results_index][2] = frustration_global.motor0_weight;
        frustration_results[results_index][3] = frustration_global.motor1_weight;
        frustration_results[results_index][4] = frustration_global.motor2_weight;

        
        // Increment result index for next time.
        results_index++;
        if ( DEBUG ) Serial.print("Results Index: ");
        if ( DEBUG ) Serial.println(results_index);

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
    Serial.println("Sample, Motor, Local Demand, Own Voltage, Neigh Voltage, Local Error, Neigh Difference, Final Actuation, New Voltage, New error, Local Frustration, Local Neigh Weight, Error Python, Global Frustration, Gobal Neigh Weight");

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
      Serial.print(",");
      Serial.print(motor0_results[result][6]); // New Voltage for Motor 0
      Serial.print(",");
      Serial.print(motor0_results[result][7]); // New error for Motor 0
      Serial.print(",");
      Serial.print(motor0_results[result][8]); // Frustration levels for Motor 0
      Serial.print(",");
      Serial.print(motor0_results[result][9]); // Neighbour Weight for Motor 0
      Serial.print(",");
      Serial.print(frustration_results[result][0]); // Global Error from python
      Serial.print(",");
      Serial.print(frustration_results[result][1]); // Global frustration (higher-level function)
      Serial.print(",");
      Serial.print(frustration_results[result][2]); // Neighbour weight Motor 0 (higher-level function)
      Serial.print("\n");  // Newline after Motor


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
      Serial.print(",");
      Serial.print(motor1_results[result][6]); // New Voltage for Motor 0
      Serial.print(",");
      Serial.print(motor1_results[result][7]); // New Error for Motor 0
      Serial.print(",");
      Serial.print(motor1_results[result][8]); // Frustration levels for Motor 1
      Serial.print(",");
      Serial.print(motor1_results[result][9]); // Neighbour Weight for Motor 1
      Serial.print(",");
      Serial.print(frustration_results[result][0]); // Global Error from Python
      Serial.print(",");
      Serial.print(frustration_results[result][1]); // Global frustration (higher-level function)
      Serial.print(",");
      Serial.print(frustration_results[result][3]); // Neighbour weight (higher-level function)
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
      Serial.print(",");
      Serial.print(motor2_results[result][6]); // New Voltage for Motor 2
      Serial.print(",");
      Serial.print(motor2_results[result][7]); // New Error for Motor 2
      Serial.print(",");
      Serial.print(motor2_results[result][8]); // Frustration levels for Motor 2
      Serial.print(",");
      Serial.print(motor2_results[result][9]); // Neighbour Weight for Motor 2
      Serial.print(",");
      Serial.print(frustration_results[result][0]); // Global Error from Python
      Serial.print(",");
      Serial.print(frustration_results[result][1]); // Global frustration (higher-level function)
      Serial.print(",");
      Serial.print(frustration_results[result][4]); // Neighbour weight (higher-level function)
      Serial.print("\n");  // Newline after Motor 2
    }
    // A delay to allow time for copying results
    delay(3000);
  }
}
//==================================== END MAIN LOOP =========================================================

// -------------------------------------- Higher-level functions ----------------------------------------------

void receiveErrorFromPython() {
  // Check if there is data available in the serial buffer
  if (Serial.available() > 0) {
    // Read the incoming data from Python
    String input = Serial.readStringUntil('\n');  // Read until newline
    input.trim();  // Remove any extra spaces or newline characters

    // Convert the received string to a float (error distance)
    float received_error = input.toFloat();

    // If valid (not NaN), store and print it
    if (!isnan(received_error)) {
      error_from_python = received_error;
      if (DEBUG) {
        Serial.print("[Arduino] Received error: ");
        Serial.println(error_from_python);
      }
    } else {
      // If the data is invalid (NaN), set error_from_python to a default value (e.g., 0)
      error_from_python = 0.0;  // Default value when invalid data is received
      if (DEBUG) {
        Serial.println("[Arduino] Invalid data received, setting error to 0.");
      }
    }
  } else {
    // If no data is available from Python, set a default value (e.g., 0)
    error_from_python = 0.0;  // Default value when no data is received
    if (DEBUG) {
      Serial.println("[Arduino] No data received, setting error to 0.");
    }
  }
}

ExperimentData global_function(float error_from_python) {

  ExperimentData global_frustration_data;
  
  // get the 'new' error
  if ( DEBUG )Serial.print("------------------------------> Global error is ");
  if ( DEBUG )Serial.println(error_from_python);
  if ( DEBUG )Serial.println("\n");
  global_frustration_data.error_from_python = error_from_python;
  if ( DEBUG )Serial.println(global_frustration_data.error_from_python);
  
  // integrate the error
  global_integrated_error = global_integrated_error + global_emotional_momentum * (error_from_python - (lambda_global * global_integrated_error)) * 1;
  // global_integrated_error = old error, declared globally
  // (lambda_global * global_integrated_error) = portion of old error to simulate 'leakage'
  // timestep = 1
  if ( DEBUG )Serial.print("Global integrated error is ");
  if ( DEBUG )Serial.println(global_integrated_error);

  // update global_integrated_frustration
  global_integrated_frustration += global_integrated_error;
  global_frustration_data.global_integrated_frustration = global_integrated_frustration;
  if ( DEBUG )Serial.print("GLOBAL FRUSTRATION LEVELS: ");
  if ( DEBUG )Serial.println(global_frustration_data.global_integrated_frustration);

  // WORK OUT WHAT TO DO
  if (abs(error_from_python) < global_threshold) {
    // SUCCESS, higher-level doesn't need to do anything
    if ( DEBUG )Serial.println("GLOBAL SUCCESS ACHIEVED!");
  }
  else if (abs(global_frustration_data.global_integrated_frustration) < global_recovery_threshold) {
    // Recovery: Make sure selfishness is ON or turn selfishness back on
    if ( DEBUG )Serial.println("Below global recovery");
    if (!beingSelfish) {
      if ( DEBUG )Serial.println("SELFISHNESS ALLOWED");
    }
    beingSelfish = true; // Turning selfishness back on
  }
  else if (abs(global_frustration_data.global_integrated_frustration) < global_frustration_threshold) {
    // Still acceptable, do nothing
    if ( DEBUG )Serial.println("Frustration below global threshold acceptable");
  }
  else if (abs(global_frustration_data.global_integrated_frustration) > global_frustration_threshold) {
    // Frustration too high, turn selfishness off
    if (beingSelfish) {
      if ( DEBUG )Serial.println("OVERRULING SELFISHNESS!");
    }
    // reset the weights in the array to 1
    neigh_weights[0] = reset_neigh_weights[0];
    neigh_weights[1] = reset_neigh_weights[1];
    neigh_weights[2] = reset_neigh_weights[2];
    beingSelfish = false; // Keeping/turning off selfishness function

    // reset the frustration levels to 0 
    global_integrated_error = 0; 
    global_integrated_frustration = 0; 
    global_frustration_data.global_integrated_frustration = global_integrated_frustration;
    if ( DEBUG ) Serial.println("Threshold reached: Global Frustration level is reset");
    
  }
  else {
    if ( DEBUG )Serial.println("Error global function");
  }

  // log the changes from global function
  global_frustration_data.motor0_weight = neigh_weights[0];
  global_frustration_data.motor1_weight = neigh_weights[1];
  global_frustration_data.motor2_weight = neigh_weights[2];
  if ( DEBUG )Serial.print("Neighbour weights data finishing global are: ");
  if ( DEBUG )Serial.print(global_frustration_data.motor0_weight);
  if ( DEBUG )Serial.print(", ");
  if ( DEBUG )Serial.print(global_frustration_data.motor1_weight);
  if ( DEBUG )Serial.print(", ");
  if ( DEBUG )Serial.println(global_frustration_data.motor2_weight);

  //check the frustration levels 
  if ( DEBUG )Serial.println("Checking frustration levels at end of function ---> what gets logged: ");
  if ( DEBUG )Serial.println(global_frustration_data.global_integrated_frustration);
  return global_frustration_data;
}

// --------------------------------------- end of Higher-level function --------------------------------------

// ############################################ SELFISHNESS! ######################################################
ExperimentData being_selfish(int motor, float new_local_err, float local_integrated_error, float local_integrated_frustration) { //

  ExperimentData local_frustration_data;
  // local_frustration_data.???? = 0;
  // local_frustration_data.???? = 0;
  // local_frustration_data.???? = 0;
  // local_frustration_data.???? = 0;

  memset( &local_frustration_data, 0, sizeof( local_frustration_data) ); // initialize the entire structs to be full of zeros
 
  local_integrated_error = local_integrated_error + local_emotional_momentum * ( new_local_err - (lambda_global * local_integrated_error));
  local_integrated_frustration += local_integrated_error; 
  local_frustration_data.local_integrated_frustration += local_integrated_frustration;

  if ( DEBUG )Serial.print("Motor ");
  if ( DEBUG )Serial.print(motor);
  if ( DEBUG )Serial.print(" LOCAL FRUSTRATION LEVELS: ");
  if ( DEBUG )Serial.println(local_frustration_data.local_integrated_frustration);

  if (beingSelfish) {
    // look at frustration to decide if shut off neighbour
    if ( DEBUG )Serial.println("Selfishness function is on");
    if (abs(local_frustration_data.local_integrated_frustration) < local_threshold) {
      if ( DEBUG )Serial.print(motor);
      if ( DEBUG )Serial.println(" LOCAL ACHIEVED! No need for selfishness");
      //beingSelfish = false; // we may not need this, selfishness can be on and not do anything
    }
    else if (abs(local_frustration_data.local_integrated_frustration) < local_recovery_threshold) {
      // Recovery: Make sure neigh_weight is reset
      neigh_weights[motor] = reset_neigh_weights[motor];
      local_frustration_data.neigh_weight = neigh_weights[motor];// should we only reset here?
      if ( DEBUG )Serial.print(motor);
      if ( DEBUG )Serial.print(" plays collective below local recovery threshold, neighbour weight: ");
      if ( DEBUG )Serial.println(local_frustration_data.neigh_weight);
    }
    else if (abs(local_frustration_data.local_integrated_frustration) < local_frustration_threshold) {
      // do nothing, still acceptable
      // Make sure neigh_weight is active
      neigh_weights[motor] = reset_neigh_weights[motor];
      local_frustration_data.neigh_weight = neigh_weights[motor];
      if ( DEBUG )Serial.print(motor);
      if ( DEBUG )Serial.println(" plays collective below local frustration threshold, neighbour weight: ");
      if ( DEBUG )Serial.println(local_frustration_data.neigh_weight);
    }
    else if (abs(local_frustration_data.local_integrated_frustration) > local_frustration_threshold) {
      // frustration level not acceptable
      neigh_weights[motor] = 0.0; // need to access global array, not just inside struts
      local_frustration_data.neigh_weight = neigh_weights[motor]; // start with on/off and can then decide a proportion of it depending on magnitude of frustration?
      if ( DEBUG )Serial.print(motor);
      if ( DEBUG )Serial.print(" plays selfish, neighbour weight: ");
      if ( DEBUG )Serial.println(local_frustration_data.neigh_weight);

      // reset frustration level to 0 
      local_integrated_error = 0;
      local_integrated_frustration = 0; 
      local_frustration_data.local_integrated_frustration = local_integrated_frustration;
      if ( DEBUG )Serial.println("Threshold reached: Local Frustration level is reset");
    }
  }
  else {
    // function not active, do nothing
    neigh_weights[motor] = reset_neigh_weights[motor]; // so we print the weights being active, or else it will print as 0 (or last active?)
    local_frustration_data.neigh_weight = neigh_weights[motor];
    if ( DEBUG )Serial.print(motor);
    if ( DEBUG )Serial.print(" Selfish function disabled, neighbour weight: ");
    if ( DEBUG )Serial.print(local_frustration_data.neigh_weight);
  }
  return local_frustration_data;
}

// ############################################ end of SELFISHNESS! ######################################################

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
    //    Serial.print("own cords converted voltage: ");
    //    Serial.println(ownVoltage);

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
    //    Serial.print("neigh cords converted voltage: ");
    //    Serial.println(neighVoltage);

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
ExperimentData updateMotor(int motor) {
  ExperimentData data; // Declare struct 'container'

  // Initialize struct variables to avoid undefined values
  data.ownVoltage = 0;
  data.neighVoltage = 0;
  data.local_err = 0;
  data.local_err_after_actuation = 0;
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
    if ( DEBUG )Serial.println("\n");
    if ( DEBUG )Serial.print("Motor ");
    if ( DEBUG )Serial.print(motor);
    if ( DEBUG )Serial.print(" Own Voltage: ");
    if ( DEBUG )Serial.println(data.ownVoltage);
    if ( DEBUG )Serial.print("Motor ");
    if ( DEBUG )Serial.print(motor);
    if ( DEBUG )Serial.print(" Neighbor Voltage: ");
    if ( DEBUG )Serial.println(data.neighVoltage);

    // Calculate the error to total voltage
    data.local_err = local_demand - data.ownVoltage; //no longer defined as float since global
    if ( DEBUG )Serial.println("\n");
    if ( DEBUG )Serial.print("----------------> local error is: ");
    if ( DEBUG )Serial.print(data.local_err);
    if ( DEBUG )Serial.println("\n");

    // Work out the difference with neighbor
    data.diff_neigh = data.ownVoltage - data.neighVoltage; //no longer defined as float since global
    if ( DEBUG )Serial.print("-----------------> Neighbour Difference is: ");
    if ( DEBUG )Serial.print(data.diff_neigh);
    if ( DEBUG )Serial.println("\n");

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
        //actuation_signal_up = 0;
        //actuation_signal_down = 0;
        if ( DEBUG )Serial.print(motor);
        if ( DEBUG )Serial.println(" LOCAL DEMAND ACHIEVED!");
      } else if (data.local_err > 0) {
        //moveStepsUp(motor, actuation_step * local_weight);
        actuation_signal_up += (actuation_step * local_weight);
        if ( DEBUG )Serial.print(motor);
        if ( DEBUG )Serial.println(" Increases voltage to local");
      } else {
        //moveStepsDown(motor, actuation_step * local_weight);
        actuation_signal_down += (actuation_step * local_weight);
        if ( DEBUG )Serial.print(motor);
        if ( DEBUG )Serial.println(" Decreases voltage to local");
      }

      // work out neighbour
      if ( DEBUG )Serial.print("Neighbour Weights entering neighbour update are: ");
      if ( DEBUG )Serial.print(neigh_weights[0]);
      if ( DEBUG )Serial.print(", ");
      if ( DEBUG )Serial.print(neigh_weights[1]);
      if ( DEBUG )Serial.print(", ");
      if ( DEBUG )Serial.println(neigh_weights[2]);
      
      if (neighbour_condition == -1) {  // Should be as close as possible
        // Check if local error is within threshold
        if (abs(data.diff_neigh) < neigh_threshold_converge) {
          // stop moving
          if ( DEBUG )Serial.print(motor);
          if ( DEBUG )Serial.println(" NEIGH DEMAND ACHIEVED!");
        } else if (data.diff_neigh > 0) {  // // M0 bigger than M1, decrease voltage to converge
          actuation_signal_down += (actuation_step * neigh_weights[motor]);
          if ( DEBUG )Serial.print(motor);
          if ( DEBUG )Serial.print(" decreases voltage by "); 
          if ( DEBUG )Serial.print(actuation_step * neigh_weights[motor]); 
          if ( DEBUG )Serial.println(" to neigh");
        } else {  // M0 smaller than M1, increase voltage to converge
          actuation_signal_up += (actuation_step * neigh_weights[motor]);
          if ( DEBUG )Serial.print(motor);
          if ( DEBUG )Serial.print(" Increases voltage by "); 
          if ( DEBUG )Serial.print(actuation_step * neigh_weights[motor]); 
          if ( DEBUG )Serial.println(" to neigh");
        }
      } else if (neighbour_condition == 1) {  // Should be as different as possible (increase the difference)
        // Check if the neighbour difference is within the allowable range
        if (abs(data.diff_neigh) < neigh_threshold_diverge) {
          if (data.diff_neigh > 0) { //M0 bigger than M1, keeps getting big
            actuation_signal_up += (actuation_step * neigh_weights[motor]);  // keep shortening
            if ( DEBUG )Serial.print(motor);
            if ( DEBUG )Serial.println(" Increases voltage to diverge from neigh");
          } else if (data.diff_neigh < 0) { // M0 smaller than M1, keeps getting small
            actuation_signal_down += (actuation_step * neigh_weights[motor]);  // keep elongating
            if ( DEBUG )Serial.print(motor);
            if ( DEBUG )Serial.println(" Decreases voltage to diverge from neigh");
          }
        }
        else if (abs(data.diff_neigh) > neigh_threshold_diverge) {
          // Stop moving if the difference exceeds 200
          if ( DEBUG )Serial.print(motor);
          if ( DEBUG )Serial.println(" Neighbour difference too large, no more movement allowed");
        }

        else {
          if ( DEBUG )Serial.println("Incorrect neigh_diff value for motor ");
          if ( DEBUG )Serial.print(motor);
        }
      } else {
        if ( DEBUG )Serial.print(motor);
        if ( DEBUG )Serial.println(" INCORRECT NEIGHBOUR CONDITION");
      }

      // MOVE
      data.actuation_final = actuation_signal_down - actuation_signal_up; // (elongate - shortening)

      if (data.actuation_final > 0) {
        moveStepsDown(motor, data.actuation_final); // elongates - increases voltage
      } else if (data.actuation_final < 0) {
        moveStepsUp(motor, abs(data.actuation_final)); // shortens - decreases voltage
      } else {
        if ( DEBUG )Serial.print(motor);
        if ( DEBUG )Serial.println(" No movement needed, actuation = 0");
      }

      if ( DEBUG )Serial.print(motor);
      if ( DEBUG )Serial.print(" FINAL ACTUATION: ");
      if ( DEBUG )Serial.println(data.actuation_final); // if negative means shortens --> voltage increases

      // update ADC readings
      read_adc_MOTOR(motor, data.ownVoltage, data.neighVoltage);
      data.newVoltage = data.ownVoltage;

      // get the new error after actuation
      data.local_err_after_actuation = local_demand - data.newVoltage;

    } // end of check if can move
  } // checking time for ADC reading

  return data; // Return data for the specific motor requested
}// end of function
