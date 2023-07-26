#include <SD.h>
#include <Wire.h>

const int soundPin = 9;

// Define the frequency range (in Hz) you want to map the accelerometer data to
const int minFrequency = 50;
const int maxFrequency = 500;
// Define the array to store 200 samples of accelerometer data
const int numSamples = 200;
const float maxAcceleration = 16.0;
String startHapticsFlag = "startHaptics";
String stopHapticsFlag = "stopHaptics";

float accelerometerData[] = {
  -4.23866572, -1.10848172, -5.97839345, -3.65826733, 0.76842214,
  -5.23563836, -2.35738985, -2.33480944, -5.47982408, -2.73704628,
  -3.16778926, -3.97429501, -2.29628814, -2.33444975, -5.9093262,
  -0.91229532, 0.0567518, -6.55416879, -3.19509269, -3.34276004,
  -4.81432908, -1.90157445, -1.46106758, -1.19796868, -5.1369366,
  -0.65006208, -2.22431219, 6.09877078, 0.22625543, -2.4405955,
  5.01638267, -1.34743332, 7.94275368, -3.42275345, 0.74302466,
  3.22778064, 1.65642786, 1.37115858, 5.37599116, 3.80894943,
  -1.47819085, 16.85167369, 0.199048, 8.08260356, 17.305096,
  -0.96030852, 10.69790336, 4.45237219, 10.43443685, 17.47798644,
  9.50819257, 3.83821277, 14.74608708, 18.6944273, 14.65735156,
  7.11623839, 20.11037237, 17.83685788, -2.09144647, 24.6530609,
  20.86586234, 11.54637346, 28.76359013, -8.69536368, 1.34593474,
  28.24345732, 10.99388309, 14.40450579, 26.89353256, 1.68358712,
  -1.67721016, 33.62527495, -12.39965731, 10.51196172, 34.32987648,
  32.69281199, 19.91208325, 3.81626665, 22.07934572, 28.85717978,
  8.12350976, 17.700535, 14.22384928, 5.90905478, 19.44569319,
  5.24904725, 22.12709438, -5.62734001, 11.23638498, 3.33607663,
  5.76427356, 12.40836975, 3.51331352, 4.92997507, 7.22319561,
  6.13432404, 20.82631276, 5.62907187, 19.76455262, 8.29726877,
  3.04192518, 7.76558203, -11.34161651, 23.60305109, 12.50998804,
  5.58799011, 23.74155558, 15.13298489, -2.67908985, 22.35030428,
  13.969722, 27.20576433, 10.18596464, 13.89278425, 22.82280893,
  8.02352101, 25.17567008, 10.38380632, -0.20355922, 26.02756639,
  18.96165588, 4.53321649, 26.53508617, -8.05149315, 19.71572619,
  16.99643511, 19.60100987, 17.83883032, 5.46667583, 3.7335959,
  13.28721957, 16.55522256, 18.36375411, 13.22774953, -1.40094696,
  15.38374431, 15.31688634, 9.68112984, 22.77507594, -0.07839722,
  10.71661556, 9.06302737, 5.77660934, 11.90516172, 2.27153482,
  6.0004731, 7.32794489, -1.57717452, 8.26513062, 3.1268131,
  2.16943895, 5.16686238, -0.25421852, 5.58553821, 4.08730451,
  1.74661147, 0.38365132, 1.03027746, -3.31513441, 3.98824666,
  1.20492048, -1.61970446, 2.42546698, -1.64210794, -1.22677219,
  -1.45738998, -1.21325465, -2.27804904, -4.93017407, -4.24221744,
  -4.924263, -3.33311934, -3.45402674, -2.14631154
};


// Calculate the number of elements in the array
size_t numElements = sizeof(accelerometerData) / sizeof(float);


void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(soundPin, OUTPUT);
}

void loop() {
  // Iterate through the array and play each magnitude value as a sound
  if(Serial.available()){
    //to read input from the serial input via Unity
    String serialInput = Serial.readString();
    //Uncomment below 2 LOC to get input from the built-in serial monitor
    // String serialInput = Serial.readStringUntil('\n');
    // serialInput.trim();
    if(serialInput.equalsIgnoreCase(startHapticsFlag)){
        for (int i = 0; i < numElements; i++) {
          if (Serial.available() > 0) {
            serialInput = Serial.readStringUntil('\n');
            serialInput.trim();
          }
          if(serialInput.equals(stopHapticsFlag))
            break;
          float normalizedMagnitude = accelerometerData[i] / (maxAcceleration * 2);
          int frequency = map(normalizedMagnitude * 1000, 10, 3200, minFrequency, maxFrequency);
          Serial.println(i);
          Serial.println(frequency);
          if (frequency > 0)
            playTone(frequency, 100); // Play for 100 milliseconds
        }
    }
    else {
       Serial.println("start haptics error");
       Serial.println("Flag received - "+serialInput);
    }
  }
  
}

void playTone(int frequency, int duration) {
  //Calculate the period of the tone by converting the frequency from Hz (cycles per second) to the period in microseconds (µs)
  //Formula : Period (µs) = 1 / Frequency (Hz) * 1,000,000
  long period = 1000000L / frequency;  
  long pulseWidth = period / 2; // High and low durations for square wave
  //Convert the time duration from milliseconds (ms) to microseconds (µs)
  for (long i = 0; i < duration*1000L; i += period) {
    Serial.println("playing tone");
    digitalWrite(soundPin, HIGH); // Sound ON
    delayMicroseconds(pulseWidth);
    digitalWrite(soundPin, LOW);  // Sound OFF
    delayMicroseconds(pulseWidth);
  }
}
