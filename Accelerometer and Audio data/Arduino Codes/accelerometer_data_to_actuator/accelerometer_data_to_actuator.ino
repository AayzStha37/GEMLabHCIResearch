#include <Wire.h>

const int soundPin_adxl = 8;
const int soundPin_imu = 9;

// Define the frequency range (in Hz) you want to map the accelerometer data to
const int minFrequency = 50;
const int maxFrequency = 500;
// Define the array to store 200 samples of accelerometer data
const int numSamples = 200;
const float maxAcceleration = 16.0;
//haptics flag constants
const String startHapticsFlag = "startHaptics";
const String stopHapticsFlag = "stopHaptics";

//startHaptics_Sidewalk
//startHaptics_Road
const String sidwalkFlag = "Sidewalk";
const String roadFlag = "Road";
const String gravelFlag = "Gravel";
const String grassFlag = "Grass";


float sidewalk_accelerometerData[] = {11.9957814, -9.21458248, -23.5700082, 11.3311076, 2.10931239, -0.715526674, -25.9875513, 2.9975653, -6.87430486, -7.20617744, -16.2885319, 3.98144169, 6.07271322, 6.29952507, -11.7517073, 12.4924423, 4.59385452, 20.9898635, 16.8637806, 29.9948699, 8.31520837, -6.46487079, -5.91893128, -2.00704778, 11.2743066, -4.87936973, 8.71549064, 1.93152907, -0.00024213223, -8.28404816, -11.2460762, -11.9434974, -4.41131833, 0.905374799, 2.08265642, 18.2134851, 2.14743718, -14.6391512, -59.4081487, -9.7212467, -5.47009807, 29.9572038, -7.60662351, 37.6334092, -8.51491802, 29.3641215, 5.97559901, 17.5704724, -4.33064044, 46.1924686, 42.5017547, 15.2176744, 22.1951058, 33.4974066, 45.2259246, 23.0480323, 37.6955117, 27.7786835, 46.5047569, 34.0076926, 25.0109317, -0.867646158, -36.6897944, -116.113146, 15.7095141, -1.31613702, -9.60356609, 35.011701, 76.7573393, 61.8314583, -73.0270151, 17.1436269, 48.8594951, 39.9559273, 18.8887119, 12.0483438, 14.3824692, -122.291376, 136.06904, -78.8252051, 80.2496586, 3.08481888, 26.5704576, -11.9938283, 28.8715189, 114.884402, 38.6832423, -61.5113307, 0.505878752, -96.8500346, 23.3638941, 215.915491, 101.176699, -60.7742779, 27.4498507, 52.8265268, -18.3745432, 5.17737657, 60.5367632, -282.385604, 16.5759108, 34.9335958, -10.5350981, 67.4025087, 54.422634, 19.6948566, 38.7801071, -174.906868, -13.235666, 55.787524, -38.593609, -83.1914024, -21.7093806, 11.7212072, -6.55145649, -60.9866981, -21.9606595, -4.7360684, -5.57473964, 7.97172101, -32.6917866, 93.75189, 35.1112426, -13.6286771, 17.6733551, 44.3398415, -41.413385, -111.952928, -187.215947, 13.9578359, 2.1751687, -9.83531682, -13.3178638, 26.7919743, 12.3115783, -10.552322, -42.8370955, 14.2738686, 134.878727, -42.5618829, -2.36080394, -8.95442195, 7.81003306, -55.6771838, 22.782337, -0.695715284, 22.1655807, -13.3778042, -1.56005094, -11.116105, 24.7422776, -33.2086579, 8.47333649, 22.1948097, 41.4118854, 13.4665196, -12.2505865, -0.497795992, -3.81541039, 7.65603573, -14.5760736, -4.52887682, 12.2973116, -9.64113763, 14.9421895, -5.08907705, -2.26743596, -5.52588992, -4.66530117, 0.995700298, -0.0371005869, -2.42724234, 24.3664112, 3.25746252, -4.78725815, -9.98558795, 14.6514469, -20.7625331, 2.85499557, -5.85779184};
float road_accelerometerData[] = {-2.00441164, -11.30262114, -7.04176148, 6.10804368, -9.2677073, 0.85253779, -4.05841214, -8.82869467, 5.92913871, -13.17569853, -4.00686144, -12.65535475, 1.21160183, 3.19548214, 2.45824047, -9.91845318, -9.60866502, -9.50401968, 6.59015286, 3.07346011, -2.97964702, 5.87834398, 5.74604466, 2.12940048, 1.80735609, -8.76482576, -1.61352609, -8.22156302, 11.58111061, -2.49780166, 16.62957806, -5.42727276, -2.21121626, 8.85261625, -17.30798437, 2.37655641, -14.18867441, 9.28148094, 4.25120469, 19.496621, 11.88428677, -14.5332248, 3.74044842, 0.70053532, 6.73320703, -4.28217952, 18.69374606, -0.64765407, 36.0063441, 6.8282555, 2.9195438, 9.76870524, 10.00421264, 27.17075745, -3.20807223, 6.06000821, 1.05558071, -19.70747681, 15.28920397, 21.43149463, -4.63225087, 24.73815502, -11.14044741, -9.52868535, -2.60133967, 25.49128578, 36.29477487, 13.17706853, 2.25187254, -13.74184919, 39.53660156, 34.54398927, 3.9914109, 29.55484598, 9.90294442, 30.64573616, -17.9357516, 39.74270582, -17.61875958, 76.87537228, -19.52175682, 92.75288547, 20.6319604, 29.44954511, -17.90990864, -126.2107468, 69.28864535, -23.88311807, 69.74451545, 1.52272179, 13.33912835, -4.20693199, 4.36512736, -100.57436499, 56.51063744, 11.2976387, -6.09692474, -68.69224718, -11.71762327, 98.94914647, 47.82190242, 7.69068253, 67.31366043, -42.13324615, 30.39431905, 83.50846872, -28.18277376, 1.41439664, 7.4474355, -71.36617623, -20.35078097, 16.46456657, -116.78413394, -19.14835938, 41.38492695, 16.33631196, 19.41288459, 32.8105479, -17.59096275, 24.3050553, -54.32740784, 17.39426137, -22.94198288, -77.45198927, 10.3957197, 11.3806471, -35.88814471, 29.68253222, 74.28004332, 25.11461416, -47.24935578, -5.58841319, 5.79223815, 40.31429355, 28.33520565, -3.00138337, 15.16418121, 14.05431476, 14.46182794, 3.87699584, -42.66396339, 14.19580406, -46.85629698, 65.00536698, 9.22765432, 35.26709698, 45.47676465, 44.11916455, -25.56262755, 15.38981417, -34.71839717, -15.24306496, -37.40310453, 28.26062385, 13.68476158, 1.33665073, 5.6604951, -29.94963458, -13.52984517, 13.1189113, 37.42953683, 28.89611965, -3.39718132, 2.49758966, 0.7496962, -9.6963159, -0.15866861, -20.73846659, 0.14011414, 8.82446046, 10.29884761, 10.40730039, -10.91096613, 4.97341836, 10.36182148, 1.64654927, 19.83356872, -15.72880949, -10.9672644, -5.90643153, 6.2231467, -0.71653556, -14.76591962, -7.46574461, -11.66965741, 0.91191166, -1.06115452, 2.17769232, -11.3091628, 14.22459039, -2.76072821, -0.29734456, -17.91789769, -14.5164608, -4.15290747, 2.00053358, 4.83054997, 2.49683269, 0.88702996}; 
float gravel_accelerometerData[] = {-4.69967399, -5.6509253, -3.15117066, -3.68947705, -2.53251943, 0.39088592, -0.8582606, -4.02047801, -2.75695802, -1.74658639, -5.78735429, -2.04149525, -0.80428183, -6.99509834, -2.72960072, -1.98105141, 0.14633161, -2.90760392, -0.08176117, -0.85606646, 0.96421951, -2.59255348, 2.3974799, -2.71176703, 1.23666434, 0.85266183, -1.69646701, 5.79369349, -2.82715609, 6.09151516, -3.17143085, 4.80846217, 5.02108629, 9.62852406, 3.03013163, 3.47625003, 10.2496775, 5.08918595, 19.84406322, 19.66889358, 6.97247538, 14.90070175, 9.29373375, 14.5044693, -5.49366209, 12.29210784, -0.80112493, 1.83876413, 22.41381846, 15.22050263, -3.59360468, 18.56953471, 12.57158309, 7.86219442, 21.77272822, 4.67782171, 19.99038555, 22.43061661, 15.186432, 20.04583557, 8.02877198, 32.59524952, 8.98357555, -13.00872862, 2.2659087, 23.85562202, -1.82716583, 22.49663124, 15.07834368, 16.57979581, 5.62476866, 5.36566195, 18.27204343, 6.15406567, 15.60176224, 22.70390371, 11.1439136, 26.41463445, -4.51703998, 17.50122071, -6.00752234, 7.56839813, 9.78070694, 7.53891893, 6.1088925, 8.20182817, 10.27889251, 4.93837704, 6.76962565, 17.49914285, 0.19419603, 6.30638359, 11.9790604, 2.28763886, 21.33286202, -4.67370568, 16.75653259, 18.21807887, 17.75649148, 13.01500483, 27.50639277, -8.51781572, 16.52533165, 9.89737965, 15.47223124, 33.79658119, -12.43391131, 20.29205438, 30.31549556, -7.19884705, 16.95967639, 12.39561863, -2.10958726, 23.44832185, 27.03908321, 17.28726634, 25.8346833, 13.81356481, 2.42394266, 24.98817103, 32.87403574, 10.80902486, 13.62788693, 10.56105042, 7.54538174, 21.87750662, 28.19829364, 12.42098757, 20.19478423, 14.91488342, 17.0615701, 14.93836634, 12.53328477, 11.12044519, -1.00324925, 9.35055636, 6.34535786, 3.44333559, 6.81159866, 9.63380845, -8.82848315, -0.87720737, 7.24101047, -2.82038023, 7.97440495, -1.76706153, -3.28031799, 2.33134314, -6.03565883, 4.1597601, 4.33802843, -4.67626328, -0.17868194, -2.01168287, -0.04361508, -4.60759596, -2.1291735, 1.88262849, -4.01530902, -3.55945173, -1.59895221, -0.93818834, -1.41266978, -0.11224209, 0.27273554, -2.49645384, -3.10518316, -2.53704823, -2.92803908, -0.0671321, -3.78517869, -1.47134651};
// float grass_accelerometerData[] = {};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(soundPin_adxl, OUTPUT);
  pinMode(soundPin_imu, OUTPUT);
}

void loop() {
  // Iterate through the array and play each magnitude value as a sound
  if(Serial.available()){
    //to read input from the serial input via Unity
    String serialInput = Serial.readStringUntil('\n');
    serialInput.trim();
    String* tokens = splitString(serialInput, '_');
    if(startHapticsFlag.equalsIgnoreCase(tokens[0])){
       float* accelerometerData = selectArray(tokens[1]);
       size_t arrayLen = getArrayLength(tokens[1]);
       for (int i = 0; i < arrayLen ; i++) {
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
    digitalWrite(soundPin_adxl, HIGH); // Sound ON
    digitalWrite(soundPin_imu, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(soundPin_adxl, LOW);  // Sound OFF
    digitalWrite(soundPin_imu, LOW);
    delayMicroseconds(pulseWidth);
  }
}

String* splitString(String str, char delimiter) {
  int numTokens = 0; // Count the number of tokens
  for (int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == delimiter) {
      numTokens++;
    }
  }
  numTokens++; // Add one for the last token after the last delimiter

  String* tokens = new String[numTokens]; // Create an array to store the tokens

  char char_array[str.length() + 1]; // Create a character array to store the string
  str.toCharArray(char_array, sizeof(char_array)); // Convert the String to a character array

  char *token = strtok(char_array, "_"); // Find the first token separated by "_"
  int tokenIndex = 0;

  while (token != NULL) {
    tokens[tokenIndex] = String(token); // Store the token in the array
    tokenIndex++;
    token = strtok(NULL, "_"); // Find the next token separated by "_"
  }

  return tokens; // Return the array of tokens
}

float* selectArray(String surfaceFlag) {
  if(surfaceFlag.equals(sidwalkFlag))
     return sidewalk_accelerometerData;
  else if(surfaceFlag.equals(roadFlag))
     return sidewalk_accelerometerData;
  else if(surfaceFlag.equals(gravelFlag) || surfaceFlag.equals(grassFlag) )      
    return sidewalk_accelerometerData;
  else 
    return;
}

size_t getArrayLength(String surfaceFlag){
    if(surfaceFlag.equals(sidwalkFlag))
     return sizeof(sidewalk_accelerometerData)/sizeof(float);
  else if(surfaceFlag.equals(roadFlag))
     return sizeof(road_accelerometerData)/sizeof(float);
  else if(surfaceFlag.equals(gravelFlag) || surfaceFlag.equals(grassFlag) )      
     return sizeof(gravel_accelerometerData)/sizeof(float);
  else 
    return;
}

