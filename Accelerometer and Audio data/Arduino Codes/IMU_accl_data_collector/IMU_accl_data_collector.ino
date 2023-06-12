#include "ICM_20948.h"

#include <Adafruit_Sensor.h>

#include <Adafruit_ADXL345_U.h>

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2 // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

//defining ADXL sensor object
Adafruit_ADXL345_Unified adxl_accelerometer = Adafruit_ADXL345_Unified(0x53);
//defining IMU sensor object
#ifdef USE_SPI
ICM_20948_SPI imu_accelerometer; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C imu_accelerometer; // Otherwise create an ICM_20948_I2C object
#endif
// Configuration for the trigger button and the LED indicator
const int triggerButtonPin = 3; // Pin for the trigger button
const int ledPin = 9; // Pin for the LED
int buttonState = 0; // Current state of the button
int previousButtonState = 0; // Previous state of the button

void setup() {
  pinMode(triggerButtonPin, INPUT); // Set the trigger button pin as input
  pinMode(ledPin, OUTPUT); // Set the LED pin as output

  //set the BAUD rate
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {};

  #ifdef USE_SPI
  SPI_PORT.begin();
  #else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  #endif

  init_adxl_sensor();
  init_imu_sensor();
}

void init_adxl_sensor() {
  SERIAL_PORT.println("Initializing ADXL345 sensor");
  if (!adxl_accelerometer.begin()) {
    SERIAL_PORT.println("ERROR: NO ADXL345 detected!");
    while (1);
  } else {
    SERIAL_PORT.println("SUCCESS: ADXL345 detected!");
  }
  
  adxl_accelerometer.setRange(ADXL345_RANGE_16_G);
  delay(1);
  adxl_accelerometer.setDataRate(ADXL345_DATARATE_3200_HZ);
}

void init_imu_sensor() {
  bool initialized = false;
  while (!initialized) {
    #ifdef USE_SPI
    imu_accelerometer.begin(CS_PIN, SPI_PORT);
    #else
    imu_accelerometer.begin(WIRE_PORT, AD0_VAL);
    #endif

    SERIAL_PORT.println("Initializing IMU sensor");
    if (imu_accelerometer.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
      SERIAL_PORT.println("SUCCESS: IMU sensor detected!");
      imu_accelerometer.setSampleMode(ICM_20948_Internal_Acc, ICM_20948_Sample_Mode_Continuous);
      //setting the sample/data rate of the IMU
      ICM_20948_smplrt_t mySmplrt;
      ICM_20948_fss_t myFSS;
      mySmplrt.a = 1; //specify the accel sample rate to maximum, set it to 1 : see Table 19 in datasheet DS-000189-ICM-20948-v1.3.pdf
      imu_accelerometer.setSampleRate(ICM_20948_Internal_Acc, mySmplrt);
      delay(1);
      myFSS.a= gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e) // gpm2, gpm4, gpm8, gpm16
      imu_accelerometer.setFullScale(ICM_20948_Internal_Acc, myFSS);
    }
  }
}

void loop() {
  // Read the state of the trigger button
  buttonState = digitalRead(triggerButtonPin);
  // Check if the button is pressed
  if (buttonState == HIGH) {
    if (previousButtonState == LOW) {
      SERIAL_PORT.print("IMU-X,IMU-Y,IMU-Z");
      SERIAL_PORT.print(", ");
      SERIAL_PORT.println("ADXL-X,ADXL-Y,ADXL-Z");
    }
    previousButtonState = 1;
    // Button has just been pressed, turn on the LED
    digitalWrite(ledPin, HIGH);
    if (imu_accelerometer.dataReady()) {
      imu_accelerometer.getAGMT(); // The values are only updated when you call 'getAGMT'
      printAccelerometersReadings( & imu_accelerometer);
    } else {
      SERIAL_PORT.println("Waiting for data");
      delay(500);
    }
  } else if (buttonState == LOW && previousButtonState == HIGH) {
    previousButtonState = 0;
    // Button has just been released, turn off the LED
    digitalWrite(ledPin, LOW);
    SERIAL_PORT.println("=");
  }
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    SERIAL_PORT.print("-");
  } else {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      SERIAL_PORT.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    SERIAL_PORT.print(-val, decimals);
  } else {
    SERIAL_PORT.print(val, decimals);
  }
}

void printAccelerometersReadings(ICM_20948_I2C * sensor) {
  printScaledIMUAxesReading(sensor); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
  printScaledADXLAxesReading();
}

// #ifdef USE_SPI
// void printScaledIMUAxesReading(ICM_20948_SPI * sensor) {
//     #else
void printScaledIMUAxesReading(ICM_20948_I2C * sensor) {
  //printFormattedFloat(sensor -> accX()/1000, 5, 2);
  // printFormattedFloat(sensor -> accY(), 5, 2);
  // SERIAL_PORT.print(", ");
  // printFormattedFloat(sensor -> accZ(), 5, 2);
  // SERIAL_PORT.print(", ");

  SERIAL_PORT.print(sensor -> accX()/100);
  SERIAL_PORT.print(", ");
   SERIAL_PORT.print(sensor -> accY()/100);
  SERIAL_PORT.print(", ");
   SERIAL_PORT.print(sensor -> accZ()/100);
  SERIAL_PORT.print(", ");
}

void printScaledADXLAxesReading() {
  sensors_event_t event;
  adxl_accelerometer.getEvent( & event);
  SERIAL_PORT.print(event.acceleration.x);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(event.acceleration.y);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(event.acceleration.z);
  SERIAL_PORT.println();
}