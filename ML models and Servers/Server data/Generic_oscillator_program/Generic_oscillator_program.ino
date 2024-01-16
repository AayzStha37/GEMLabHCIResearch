
#include "DaisyDuino.h"
#include <SPI.h>
// #include <SPIExtension.h>

DaisyHardware hw;

size_t num_channels;

static Oscillator osc_sine;
static Oscillator osc_tri;
static Oscillator osc_toPlay;

const String SINE = "sine";
const String TRI = "tri";

//SPI pins
// MOSI - 10
// MISO - 9
// SCK - 8
// CS - 7

const uint8_t PIN_CS = 7;
// const uint8_t NUM_DEVICE = 1;
// SPIExtension<PIN_CS, NUM_DEVICE> spi;

uint32_t rx3 = 11, tx3 = 12;

HardwareSerial  SerialPot(rx3, tx3); 

void MyCallback(float **in, float **out, size_t size) {
  Oscillator osc = osc_toPlay;
  for (size_t i = 0; i < size; i++) {
    float sig = osc.Process();

    for (size_t chn = 0; chn < num_channels; chn++) {
      out[chn][i] = sig;
    }
  }
}

void setup() {  
  Serial.begin(115200);
  SerialPot.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_CS, INPUT);
  // SPI.begin();
  // spi.attatch(SPI);
  float sample_rate;
  // Initialize for Daisy pod at 96kHz
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_96K);
  num_channels = hw.num_channels;
  sample_rate = DAISY.get_samplerate();

  initOscillators(sample_rate);
}

void initOscillators(float sample_rate){
  
  osc_sine.Init(sample_rate);
  osc_sine.SetFreq(440);
  osc_sine.SetAmp(0.5);
  osc_sine.SetWaveform(Oscillator::WAVE_SIN);
  
  osc_tri.Init(sample_rate);
  osc_tri.SetFreq(200);
  osc_tri.SetAmp(2);
  osc_tri.SetWaveform(Oscillator::WAVE_TRI);
}

void loop() {
  String recievedMsg;
  if (SerialPot.available()>0) {
    char inputChar = SerialPot.read();
    if (inputChar == '\n') {
      if (recievedMsg.equalsIgnoreCase(SINE)) {
        DAISY.end();
        Serial.println("Received command: Sine Wave");
        osc_toPlay = osc_sine;
        DAISY.begin(MyCallback);
        blinkLED();
      }else if (recievedMsg.equalsIgnoreCase(TRI)) {
        DAISY.end();
        Serial.println("Received command: Triangle Wave");
        osc_toPlay = osc_tri;      
        DAISY.begin(MyCallback);
      }else {
        Serial.println("Invalid command");
      }
      recievedMsg = "";  // Reset the message
    } else {
      recievedMsg += inputChar;  // Append characters to the message
    }
  }

  if(SerialPot.available()){
    char inputChar = SerialPot.read();
    if(inputChar == '1'){
        Serial.println("Received command: 1");
    }
  }

  if(SerialPot.available()>0){
    char inputChar = SerialPot.read();
    if(inputChar == '1'){
        Serial.println("Received command: 1'");
    }
  }
  //read data over SPI connection
  // for (uint8_t i = 0; i < spi.size(); ++i)
  // {
  //   Serial.println(spi.data(i), HEX); 
  // }

  // static int previousState = LOW; // Assume the initial state is LOW
  // int currentState = digitalRead(PIN_CS);

  // if (currentState != previousState) {
  //   // CS pin state has changed
  //   if (currentState == HIGH) {
  //     osc_toPlay = osc_sine;      
  //     DAISY.begin(MyCallback);
  //     Serial.println("CS pin is HIGH");
  //     blinkLED();
  //   } else {
  //     DAISY.end();
  //     Serial.println("CS pin is LOW");
  //   }

  //   // Update the previous state
  //   previousState = currentState;
  // }
}


void blinkLED(){
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);   
}
