#include "DFRobot_ADXL345.h"

#define cs_pin_1 10
#define cs_pin_2 9

#define SERIAL_PORT Serial

//DFRobot_ADXL345_I2C ADXL345(&Wire,0x53);
DFRobot_ADXL345_SPI ADXL345(&SPI, cs_pin_1);

DFRobot_ADXL345_SPI ADXL345_2(&SPI, cs_pin_2);

int accval[3];
int accval_2[3];

// Configuration for the trigger button and the LED indicator
const int triggerButtonPin = 8; // Pin for the trigger button
const int ledPin = 3; // Pin for the LED
int buttonState = 0; // Current state of the button
int previousButtonState = 0; // Previous state of the button
int samplecount = 1;

void setup() {
  pinMode(triggerButtonPin, INPUT); // Set the trigger button pin as input
  pinMode(ledPin, OUTPUT); // Set the LED pin as output
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(cs_pin_1, OUTPUT);
  pinMode(cs_pin_2, OUTPUT);
  digitalWrite(cs_pin_1,HIGH);
  digitalWrite(cs_pin_2,HIGH);
 
  SERIAL_PORT.begin(115200);

  init_adxl_sensors();
}

void init_adxl_sensors() {
  SERIAL_PORT.println("Initializing ADXL345 sensor");
  if (ADXL345.begin() && ADXL345_2.begin()) {
    ADXL345.powerOn();
    ADXL345_2.powerOn();
      /**
    * Give the range settings
    * Accepted values are 2g, 4g, 8g or 16g
    * Higher Values = Wider Measurement Range
    * Lower Values = Greater Sensitivity
    */
    ADXL345.setRangeSetting(16);
    ADXL345_2.setRangeSetting(16);

    ADXL345.setRate(ADXL345_BW_1600);
    ADXL345_2.setRate(ADXL345_BW_1600);
    SERIAL_PORT.println("SUCCESS: Both ADXL345 detected!");
  } else {
    SERIAL_PORT.println("ERROR: One or more ADXL345 NOT detected!");
  }
  
}

void loop() 
{
    // Read the state of the trigger button
  buttonState = digitalRead(triggerButtonPin);
  // Check if the button is pressed
  if (buttonState == HIGH){
    if(previousButtonState == LOW) {
      SERIAL_PORT.print("Samples");
      SERIAL_PORT.print(", ");
      SERIAL_PORT.print("ADXL-X1,ADXL-Y1,ADXL-Z1");
      SERIAL_PORT.print(", ");
      SERIAL_PORT.print("ADXL-X2,ADXL-Y2,ADXL-Z2");
      SERIAL_PORT.println();
    }
    previousButtonState = 1;
    // Button has just been pressed, turn on the LED
    digitalWrite(ledPin, HIGH);
    printAccelerometersReadings();
  } else if (buttonState == LOW && previousButtonState == HIGH) {
    previousButtonState = 0;
    // Button has just been released, turn off the LED
    digitalWrite(ledPin, LOW);
    SERIAL_PORT.println("=");
    samplecount = 1;
  }
}

void printAccelerometersReadings() {
  ADXL345.readAccel(accval);
  SERIAL_PORT.print(accval[0]);
  SERIAL_PORT.print(accval[1]);
  SERIAL_PORT.println(accval[2]);
  SERIAL_PORT.println();

  ADXL345_2.readAccel(accval);
  SERIAL_PORT.print(accval_2[0]);
  SERIAL_PORT.print(accval_2[1]);
  SERIAL_PORT.println(accval_2[2]);
}
