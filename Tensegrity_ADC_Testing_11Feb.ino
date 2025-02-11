
/*
 * We wrote this to read a specific adc chip
 * We refactored the code to use arrays to try
 * to make reading each chip easier by iterating
 * through arrays
 * Currently chip 0 and 1 are not working because
 * the chip select pins are wired to the serial 
 * pins.
 * Disabling UART hasn't worked, so we'll rewire
 * them.
 * 
 * 
 * 
 */

#include "Protocentral_ADS1220.h"
#include <SPI.h>

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

void enableInterruptPin() {
  attachInterrupt( digitalPinToInterrupt( drdy_pin[0]), drdyInterruptHndlr0, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[1]), drdyInterruptHndlr1, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[2]), drdyInterruptHndlr2, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[3]), drdyInterruptHndlr3, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[4]), drdyInterruptHndlr4, FALLING);
  attachInterrupt( digitalPinToInterrupt( drdy_pin[5]), drdyInterruptHndlr5, FALLING);
}


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


  delay(100);
  Serial.begin(9600);

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

void loop() {

  // Report just one chip
  read_adc_chip( 2 ); // change this to whatever ADC number you want to test
  Serial.println();
  Serial.flush();

  // Report all readings of all chips
  //  for ( int i = 0; i < 6; i++ ) {
  //    read_adc_chip( i );
  //  }
  //  Serial.println();
  //  Serial.flush();

  delay(50);
} 
