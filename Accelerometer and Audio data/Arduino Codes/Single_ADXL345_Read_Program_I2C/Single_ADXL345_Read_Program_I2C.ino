#include <Wire.h>
#include <SimpleKalmanFilter.h>

#define DEVICE (0x53)      //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ] ;        //6 bytes buffer for saving data read from the device
char str[512];              //string buffer to transform data before sending it to the serial port
int regAddress = 0x32;      //first axis-acceleration-data register on the ADXL345


// Configuration for the trigger button and the LED indicator
const int triggerButtonPin = 6; // Pin for the trigger button
const int ledPin = 7; // Pin for the LED
int buttonState = 0; // Current state of the button
int previousButtonState = 0; // Previous state of the button
int samplecount = 1;

unsigned long previousMillis = 0; // Variable to store the last time 'x' was incremented
unsigned long interval = 1000; // Interval in milliseconds (change this as needed)


double v = 0; // Initial velocity
double v_old = 0; // Previous velocity
unsigned long time_acceleration_last = 0;
unsigned long time_acceleration = 0;

unsigned long startMillis;

//Register addresses
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_BW_RATE 0x2C

/*
Kalman Filter properties
SimpleKalmanFilter(e_mea, e_est, q);
e_mea: Reflects the uncertainty in measurements taken by the sensor.
e_est: Represents the initial uncertainty or variance in the estimation before incorporating any measurements.
q: Accounts for the uncertainty or variance in the system due to unmodeled process noise.
*/
SimpleKalmanFilter simpleKalmanFilter(1, 5, 0.01);

void setup() {
  Wire.begin();         // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output

  pinMode(triggerButtonPin, INPUT); // Set the trigger button pin as input
  pinMode(ledPin, OUTPUT); // Set the LED pin as output

  initializeADXL345();
  
  Serial.println("ADXL-X,ADXL-Y,ADXL-Z");
}


void loop() {
  readAndPrintSensorData();

  // buttonState = digitalRead(triggerButtonPin);
  // // Check if the button is pressed
  // if (buttonState == HIGH){
  //   if(previousButtonState == LOW) {
  //     Serial.println("ADXL-X,ADXL-Y,ADXL-Z");
  //   }
  //   previousButtonState = 1;
  //   // Button has just been pressed, turn on the LED
  //   digitalWrite(ledPin, HIGH);
  //   startMillis = millis();
  //   readAndPrintSensorData();
  // } 
  // else if (buttonState == LOW && previousButtonState == HIGH) {
  //   previousButtonState = 0;
  //   // Button has just been released, turn off the LED
  //   digitalWrite(ledPin, LOW);
  //   Serial.println("=");
  //   v = 0; // Initial velocity
  //   v_old = 0; // Previous velocity
  //   time_acceleration_last = 0;
  //   time_acceleration = 0;
  // } 
}

void initializeADXL345(){
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
  writeTo(DEVICE, ADXL345_DATA_FORMAT, 0x03); // Full resolution, +/-16g range
  writeTo(DEVICE, ADXL345_BW_RATE, 0x0F);
  
  //CALCULATE OFFSET
}

void readAndPrintSensorData() {
  double velocity_filter;
  int x, y, z;                        
  float xg, yg, zg;

  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
                                              //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
                                              //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];

   //Convert the accelerometer value to G's.
  //With 10 bits measuring over a +/-16g = 16 - (-16) = 32; we can find how to convert by using the equation:
  //Gs = Measurement Value * (G-range/(2^10-1)) or Gs = Measurement Value * (32/1023)
  //convert G to m/s^2 by multiplying it with 9.81 m/s^2

  xg = x * 0.0312 * 9.81;
  yg = y * 0.0312 * 9.81;
  zg = z * 0.0312 * 9.81;
  //Filtered velocity
  velocity_filter = calculateAndPrintFilteredVelocity(calculateMagnitude(xg,yg,zg));
  //Raw velocity
  //velocity_y_raw = calculateAndPrintRawVelocity(yg);

  Serial.print(xg);Serial.print(", ");
  Serial.print(yg);Serial.print(", ");
  Serial.print(zg);Serial.println(", ");
  //Serial.print(velocity_y_raw);Serial.print(", ");
  //Serial.println(velocity_filter);
}

float calculateMagnitude(float xg, float yg, float zg){
  float mag = pow(pow(xg, 2) + pow(yg,2) + pow(zg,2),1/2);
  return mag;
}

double calculateAndPrintFilteredVelocity(float acceleration){ 
  double dv = acceleration * (time_acceleration - time_acceleration_last) / 1000.0; // Calculate change in velocity (dv) in seconds
  v = v_old + dv; // Calculate new velocity
  v_old = v; // Update the previous velocity for the next iteration
  time_acceleration_last = time_acceleration; // Update the time variable
  time_acceleration =millis();
  
  return simpleKalmanFilter.updateEstimate(v);
}

// double calculateAndPrintRawVelocity(float acceleration){
//   unsigned long time_acceleration = millis(); // Current time

//   double dv = acceleration * (time_acceleration - time_acceleration_last) / 1000.0; // Calculate change in velocity (dv) in seconds
//   v = v_old + dv; // Calculate new velocity

//   //double estimatedValue = filter.updateEstimate(v, 0); // Update the filter with the velocity measurement and control input (0 in this case)

//   v_old = v; // Update the previous velocity for the next iteration
//   time_acceleration_last = time_acceleration; // Update the time variable
//   return v;
//}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  {
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}