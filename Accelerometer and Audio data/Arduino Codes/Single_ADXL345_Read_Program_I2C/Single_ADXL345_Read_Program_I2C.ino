#include <Wire.h>

#define DEVICE (0x53)      //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ] ;        //6 bytes buffer for saving data read from the device
char str[512];              //string buffer to transform data before sending it to the serial port
int regAddress = 0x32;      //first axis-acceleration-data register on the ADXL345
int x, y, z;                        //three axis acceleration data
float xg, yg, zg;

// Configuration for the trigger button and the LED indicator
const int triggerButtonPin = 6; // Pin for the trigger button
const int ledPin = 7; // Pin for the LED
int buttonState = 0; // Current state of the button
int previousButtonState = 0; // Previous state of the button
int samplecount = 1;

//Register addresses
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_BW_RATE 0x2C

void setup() {
  Wire.begin();         // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output

  pinMode(triggerButtonPin, INPUT); // Set the trigger button pin as input
  pinMode(ledPin, OUTPUT); // Set the LED pin as output

  initializeADXL345();
}

void loop() {
  buttonState = digitalRead(triggerButtonPin);
  // Check if the button is pressed
  if (buttonState == HIGH){
    if(previousButtonState == LOW) {
      Serial.println("ADXL-X,ADXL-Y,ADXL-Z");
    }
    previousButtonState = 1;
    // Button has just been pressed, turn on the LED
    digitalWrite(ledPin, HIGH);
    readAndPrintSensorData();
  } 
  else if (buttonState == LOW && previousButtonState == HIGH) {
    previousButtonState = 0;
    // Button has just been released, turn off the LED
    digitalWrite(ledPin, LOW);
    Serial.println("=");
  } 
}

void initializeADXL345(){
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
  writeTo(DEVICE, ADXL345_DATA_FORMAT, 0x03); // Full resolution, +/-16g range
  writeTo(DEVICE, ADXL345_BW_RATE, 0x0F);
}

void readAndPrintSensorData() {
  
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

  Serial.print(xg);Serial.print(", ");
  Serial.print(yg);Serial.print(", ");
  Serial.println(zg);

}

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