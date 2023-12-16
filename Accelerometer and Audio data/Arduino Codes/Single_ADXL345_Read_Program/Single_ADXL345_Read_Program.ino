//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
//Assign the Chip Select signal to pin 10.
int CS=8;

unsigned long time;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char BW_RATE = 0x2C; //Data Rate Register
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
unsigned char values[10];

double v = 0; // Initial velocity
double v_old = 0; // Previous velocity
unsigned long time_acceleration_last = 0; // Variable to store the last time acceleration was measured


// Configuration for the trigger button and the LED indicator
const int triggerButtonPin = 6; // Pin for the trigger button
const int ledPin = 7; // Pin for the LED
int buttonState = 0; // Current state of the button
int previousButtonState = 0; // Previous state of the button
int samplecount = 1;

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(115200);

  pinMode(triggerButtonPin, INPUT); // Set the trigger button pin as input
  pinMode(ledPin, OUTPUT); // Set the LED pin as output
  initializeADXL345();  // Initialize ADXL345
}

void loop(){
  buttonState = digitalRead(triggerButtonPin);
  // Check if the button is pressed
  if (buttonState == HIGH){
    if(previousButtonState == LOW) {
      Serial.println("ADXL-X,ADXL-Y,ADXL-Z,Velocity-Y");
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

void initializeADXL345() {
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS,HIGH);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);//Measurement mode 
  delay(100);
  writeRegister(BW_RATE, 0b00001111); //BW set to 1600Hz
  delay(100);
  //Put the ADXL345 into +/- 16G range by writing the value 0x03 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x03);
  delay(100);
}

// Function to read and convert sensor data
void readAndPrintSensorData() {
  int x, y, z;
  float xg, yg, zg;
  double velocity_y;

  readRegister(DATAX0, 6, values);

  x = (int)(((unsigned int)(values[1] & 255)) << 8 | (unsigned int)(values[0]) & 255);
  y = (int)(((unsigned int)(values[3] & 255)) << 8 | (unsigned int)(values[2]) & 255);
  z = (int)(((unsigned int)(values[5] & 255)) << 8 | (unsigned int)(values[4]) & 255);

  //Convert the accelerometer value to G's.
  //With 10 bits measuring over a +/-16g = 16 - (-16) = 32; we can find how to convert by using the equation:
  //Gs = Measurement Value * (G-range/(2^10-1)) or Gs = Measurement Value * (32/1023)
  //convert G to m/s^2 by multiplying it with 9.81 m/s^2
  xg = x * 0.0312 * 9.81;
  yg = y * 0.0312 * 9.81;
  zg = z * 0.0312 * 9.81;

  velocity_y = calculateAndPrintVelocity(yg);

  Serial.print(xg);Serial.print(", ");
  Serial.print(yg);Serial.print(", ");
  Serial.print(zg);Serial.println(", ");
  //Serial.println(velocity_y);
}

double calculateAndPrintVelocity(float acceleration){
  unsigned long time_acceleration = millis(); // Current time

  double dv = acceleration * (time_acceleration - time_acceleration_last) / 1000.0; // Calculate change in velocity (dv) in seconds
  v = v_old + dv; // Calculate new velocity

  //double estimatedValue = filter.updateEstimate(v, 0); // Update the filter with the velocity measurement and control input (0 in this case)

  v_old = v; // Update the previous velocity for the next iteration
  time_acceleration_last = time_acceleration; // Update the time variable

  return v;
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, unsigned char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
  
}

