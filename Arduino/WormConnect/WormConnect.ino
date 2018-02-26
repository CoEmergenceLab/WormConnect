/*************************************************************************************************************
 * This code is for the "WormConnect" project
 * Digital/Experimental Media Lab
 * Kansas State University
 * http://dxmedialab.org
 * 
 * It does three things:
 * 1. Controls the temperature of 2 Peltier thermoelectric coolers
 * using a PID a control algorithm (https://en.wikipedia.org/wiki/PID_controller)
 * 2. Reads the heart rate from a participant and changes the temperatures of the Peltier modules
 * 3. Controls relays attched to 8 LED light panels
 * 
 * 
 * The heart rate code is based upon software is based upon the Bristol Pulse of the City heart project,
 * built by Alan Senior and is documented here: 
 * http://www.instructables.com/id/Pulse-of-the-City-Bristol/
 * and is itself based upon software created for the San Francisco Urban Prototying Pulse of the City project,
 * created by George Zisiadis and Matt Ligon and documented here:
 * http://www.instructables.com/id/Pulse-of-the-City/
 * 
 * 
 * This code is inteneded for the Arduino Mega 2560 microcontroller
 * 
 * created October 13, 2017
 * by Carlos Castellanos

*************************************************************************************************************/

#include <PID_v1.h>
#include <RunningMedian.h>
#include <SPI.h>
#include <Adafruit_DotStar.h>


// ========== PID CONTROLLER FOR TWO PELTIER/TEC MODULES (labelled peltier0 & peltier1) ========== //

// === Temperature Sensor (thermistor) === //
#define TEMP0_SENSORPIN A0               // analog sensor pin
#define TEMP0_SUPPLYPIN 38               // digital pin that will serve as voltage source
#define TEMP1_SENSORPIN A1               // analog sensor pin
#define TEMP1_SUPPLYPIN 39               // digital pin that will serve as voltage source
#define TEMP_HI 20                       // high target temp (in C)
#define TEMP_LO 14                       // low target temp (in C)

// === FAN === //
#define FAN0_OUTPUT_PIN 5                // pwm output pin for fan (490 Hz)
#define FAN1_OUTPUT_PIN 12               // pwm output pin for fan (490 Hz)

// === PID === //
#define PID0_OUTPUT_PIN 4                // pwm output pin for peltier/pid controller (980 Hz)
#define PID1_OUTPUT_PIN 13               // pwm output pin for peltier/pid controller (980 Hz)
// Note: pins 13 and 4 on the Arduino Mega run at 980Hz (instead of the usual 490Hz). We will use these pins for the Peltier


// ========== Class representing a Peltier/TEC module ========== //

class Peltier {
  // === Class Member Variables === //
  private:
  uint8_t peltierPin;       // the number of the pin that outputs PWM to the Peltier
  uint8_t tempPin;          // the analog input pin to read the temperature
  uint8_t fanPin;           // the number of the pin that outputs PWM to the fan
  uint8_t tempSupplyPin;    // pin number for thermistor power (5V) source (using a digital pin is cleaner than using Arduino 5V pin)

  // Temperature - grab temperature from a thermistor (NTC B25/50: 3950) - ALL TEMPS ARE IN CELSIUS
  static const uint16_t TEMP_VCC = 5000;                          // VCC (in millivolts)
  static const uint8_t NUM_THERMISTOR_SAMPLES = 5;                // how many thermistor samples to take and average; more takes longer but is smoother
  uint16_t thermistorSamples[NUM_THERMISTOR_SAMPLES];             // array to hold the thermistor samples
  uint8_t numThermistorSamplesTaken = 0;                          // number of thermistor samples taken (before averaging)
  unsigned long previousMillis;                                   // store last time temperature was updated
  static const uint8_t THERMISTOR_INTERVAL = 10;                  // time interval (in ms) between thermistor samples
  static const uint16_t THERMISTOR_FIXED_RESISTOR_VAL = 10000;    // the thermistor is set up as a basic voltage divider circuit so we need the value of the fixed resistor
  static const uint16_t THERMISTOR_RESISTANCE_NOMINAL = 10000;    // resistance at 25 degrees C
  static const uint8_t THERMISTOR_TEMPERATURE_NOMINAL = 25;       // temp. for nominal resistance (almost always 25 C)
  static const uint16_t BCOEFFICIENT = 3950;                      // The beta coefficient of our thermistor (usually 3000-4000)
  static const uint8_t MAX_TEMP = 25;                             // Maximum allowed temerature for this Peltier (in C)
  static const uint8_t MIN_TEMP = 0;                              // Minimum allowed temerature for this Peltier (in C)

  // set the PWM speed (0-255) of the fan pin
  uint16_t fanSpeed = 200;

  // Define PID variables
  double Setpoint;                               // Setpoint is the desired temperature
  double Input;                                  // Input is current the current temperature
  double Output;                                 // Output is the PWM rate
  // Define aggressive and conservative Tuning Parameters
  // we set the controller to use Conservative Tuning Parameters
  // when we're near Setpoint and more aggressive Tuning
  // Parameters when we're farther away.
  double aggKp=3.9, aggKi=0.5, aggKd=1;
  double consKp=1, consKi=0.05, consKd=0.25;
  PID *pid;                                      // pointer to PID controller object

  public:
  // Constructor
  Peltier(uint8_t peltier_pin, uint8_t temp_pin, uint8_t temp_supply_pin, uint8_t fan_pin, double target_temp) {
    peltierPin = peltier_pin;                                       // pin that outputs PWM to this Peltier
    tempPin = temp_pin;                                             // analog input pin to read the temperature
    tempSupplyPin = temp_supply_pin;                                // pin number for thermistor power (5V) source
    fanPin = fan_pin;                                               // the number of the pin that outputs PWM to the fan
    
    pinMode(tempSupplyPin, OUTPUT);                                 // set the supply pin for the thermistor to be an output
    pinMode(fanPin, OUTPUT);                                        // set the PWM pin for the fan to be an output
    pinMode(peltierPin, OUTPUT);                                    // set the PWM pin for the Peltier to be an output
    
    // initialize the PID object and the variables we're linked to
    pid = new PID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE); // PID controller object (reverse direction by default)
    calculateTemp(tempPin);                                                     // current temperature (Input) of this Peltier
    if(Input < -200) Input = 22;                                                // default to room temperature
    setTargetTemp(target_temp);                                                 // target temperature (Setpoint) for this Peltier
    
    // turn this PID controller on
    pid->SetMode(AUTOMATIC);

    previousMillis = millis() - THERMISTOR_INTERVAL;
  }

  // the main PID update method
  bool updateController() {
    calculateTempAvg(tempPin);        // calculate temperature

    double gap = abs(Input-Setpoint); // distance away from Setpoint
    if(gap < 5) {
      // we're close to setpoint, use conservative tuning parameters
      pid->SetTunings(consKp, consKi, consKd);
      fanSpeed = 200;
    } else {
      // we're far from setpoint, use aggressive tuning parameters
      pid->SetTunings(aggKp, aggKi, aggKd);
      fanSpeed = 210;
    }
    
    bool x = pid->Compute();
    analogWrite(peltierPin, Output);  // PWM to peltier pin
    analogWrite(fanPin, fanSpeed);    // PWM to fan pin
    return x;
  }

  // returns the PID object
  /*
  PID *getPID() {
    return pid;
  }
  */

  // sets PID to AUTOMATIC/1 (on) or MANUAL/0 (off)
  void setControllerMode(int MODE) {
    pid->SetMode(MODE);
  }

  // returns PID mode (AUTOMATIC (1/on), MANUAL (0/off))
  int getControllerMode() {
    return pid->GetMode();
  }

  // sets PID to DIRECT (0) or REVERSE (1)
  void setControllerDirection(int DIRECTION) {
    pid->SetControllerDirection(DIRECTION);
  }

  // returns PID direction (DIRECT/0 or REVERSE/1)
  int getControllerDirection() {
    return pid->GetDirection();
  }

  // set how often the PID controller algorithm evaluates (default=100ms)
  void setControllerSampleTime(int sampleTime) {
    pid->SetSampleTime(sampleTime);
  }

  // set the PID controller's output limits (default range is 0-255: the arduino PWM range)
  void setControllerOutputLimits(double minimum, double maximum) {
    pid->SetOutputLimits(minimum, maximum);
  }

  // set the PID controller's tuning parameters; dictates the dynamic behavior of the PID (slow, fast, etc)
  void setControllerTunings(double Kp, double Ki, double Kd) {
    pid->SetTunings(Kp, Ki, Kd);
  }

  // same as above but also allows for Proportional on Measurement to be specified
  // P_ON_E (1) (Proportional on Error) is default, P_ON_M (0) Proportional on Measurement
  void setControllerTunings(double Kp, double Ki, double Kd, int P_ON) {
    pid->SetTunings(Kp, Ki, Kd, P_ON);
  }

  // these next three methods return the PID controller's current tuning parameters
  double getControllerKp() {
    return pid->GetKp();
  }

  double getControllerKi() {
    return pid->GetKi();
  }

  double getControllerKd() {
    return pid->GetKd();
  }
  
  // returns the current temperature of the Peltier module
  double getTemp() {
    return Input;
  }

  // set a new target temperature
  void setTargetTemp(double temp) {
    if(temp > MAX_TEMP) {
      Setpoint = MAX_TEMP;
    } else if(temp < MIN_TEMP) {
      Setpoint = MIN_TEMP;
    } else {
      Setpoint = temp;
    }
  }

  // returns the target temperature (in Celsius)
  double getTargetTemp() {
    return Setpoint;
  }

  // set the fan speed
  void setFanSpeed(uint16_t spd) {
    if(spd > 255) {
      fanSpeed = 255;
    } else if(spd < 0) {
      fanSpeed = 0;
    } else {
      fanSpeed = spd;
    }
    analogWrite(fanPin, fanSpeed);
  }

  uint16_t getFanSpeed() {
    return fanSpeed;
  }
  
  private:
  void calculateTemp(uint8_t tPin) {
    // This code is for an NTC 10k B25/50: 3950 Thermistor
    
    digitalWrite(tempSupplyPin, HIGH);                   // turn on voltage source
    float sample = analogRead(tPin);                     // read the analog pin
    digitalWrite(tempSupplyPin, LOW);                    // turn off voltage source
    
    // the thermistor is set up as a basic voltage divider circuit
    // we first need to determine the resistance of the thermistor in order to determine the temperature
    // so first convert the voltage coming from the analog pin into a resistance
    // using this formula: R = 10K / (1023/ADC - 1)
    if(sample == 0) sample = 1;
    sample = (1023 / sample) - 1;                        // (1023/ADC - 1) 
    sample = THERMISTOR_FIXED_RESISTOR_VAL / sample;     // 10K / (1023/ADC - 1) THERMISTOR_FIXED_RESISTOR_VAL is the value of the fixed resistor (in ohms)

    // now we need to convert to temperature
    // we are using the simplified B parameter version of the Steinhart-Hart equation: https://en.wikipedia.org/wiki/Thermistor.
    // this is fine since our temps will be within a fairly small range (15-25C)
    // this code is taken from: https://learn.adafruit.com/thermistor
    double currTemp;
    currTemp = sample / THERMISTOR_RESISTANCE_NOMINAL;           // (R/Ro)
    currTemp = log(currTemp);                                    // ln(R/Ro) Arduino's log() function actually calcualtes a natural log (base e, not base 10)
    currTemp /= BCOEFFICIENT;                                    // 1/B * ln(R/Ro)
    currTemp += 1.0 / (THERMISTOR_TEMPERATURE_NOMINAL + 273.15); // + (1/To)
    currTemp = 1.0 / currTemp;                                   // Invert
    currTemp -= 273.15;                                          // convert to Celsius

    // update the global variable Input and return the current temperature
    Input = currTemp;
  }

  void calculateTempAvg(uint8_t tPin) {
    // This code is for an NTC 10k B25/50: 3950 Thermistor

    // store current time
    unsigned long currentMillis = millis();
    
    if(currentMillis - previousMillis >= THERMISTOR_INTERVAL) {           // check to see if it's time to take a thermistor reading
      digitalWrite(tempSupplyPin, HIGH);                                  // turn on voltage source
      thermistorSamples[numThermistorSamplesTaken] = analogRead(tPin);    // read the analog pin & store value in array
      digitalWrite(tempSupplyPin, LOW);                                   // turn off voltage source
      numThermistorSamplesTaken++;                                        // increment sample count
      previousMillis = currentMillis;                                     // store the time
    } else {
      // return (without updating Input)
      return;
    }

    // after NUM_THERMISTOR_SAMPLES have been taken start averaging the values
    if(numThermistorSamplesTaken >= NUM_THERMISTOR_SAMPLES) {
      numThermistorSamplesTaken = 0;                        // reset the counter

      // now we average all the samples out
      float average = 0;
      for (uint8_t i=0; i<NUM_THERMISTOR_SAMPLES; i++) {
        average += thermistorSamples[i];
      }
      average /= NUM_THERMISTOR_SAMPLES;
      
      // the thermistor is set up as a basic voltage divider circuit
      // we first need to determine the resistance of the thermistor in order to determine the temperature
      // so first convert the voltage coming from the analog pin into a resistance
      // using this formula: R = 10K / (1023/ADC - 1)
      if(average == 0) average = 1;
      average = (1023 / average) - 1;                       // (1023/ADC - 1) 
      average = THERMISTOR_FIXED_RESISTOR_VAL / average;    // 10K / (1023/ADC - 1) THERMISTOR_FIXED_RESISTOR_VAL is the value of the fixed resistor (in ohms)
  
      // now we need to convert to temperature
      // we are using the simplified B parameter version of the Steinhart-Hart equation: https://en.wikipedia.org/wiki/Thermistor.
      // this is fine since our temps will be within a fairly small range (15-25C)
      // this code is taken from: https://learn.adafruit.com/thermistor
      double currTemp;
      currTemp = average / THERMISTOR_RESISTANCE_NOMINAL;          // (R/Ro)
      currTemp = log(currTemp);                                    // ln(R/Ro) Arduino's log() function actually calcualtes a natural log (base e, not base 10)
      currTemp /= BCOEFFICIENT;                                    // 1/B * ln(R/Ro)
      currTemp += 1.0 / (THERMISTOR_TEMPERATURE_NOMINAL + 273.15); // + (1/To)
      currTemp = 1.0 / currTemp;                                   // Invert
      currTemp -= 273.15;                                          // convert to Celsius
  
      // update the global variable Input
      Input = currTemp;
    } else {
      return;
    }
  }
};

// === PELTIER OBJECTS === //
// initialize peltier objects
// Peltier 0 is the one to the left of participant 
// Peltier 1 is to the right of participant
// one peltier is is made warmer when participant is connected to system & colder when not
// they toggle back & forth - after worms move to the cold side - that side becomes warm when the next particpant connects to the system
// (Also food will be dropped (via syringe pumps) on whichever side is being made cold)
Peltier peltier0(PID0_OUTPUT_PIN, TEMP0_SENSORPIN, TEMP0_SUPPLYPIN, FAN0_OUTPUT_PIN, TEMP_HI); // left side - warm by default
Peltier peltier1(PID1_OUTPUT_PIN, TEMP1_SENSORPIN, TEMP1_SUPPLYPIN, FAN1_OUTPUT_PIN, TEMP_LO); // right side - cold by default

// --- Peltier serial communication --- //
void sendPeltierSerial(uint8_t id, double temp) {
  Serial.print('t');
  Serial.print('e');
  Serial.print('c');
  Serial.write(32);   // space
  Serial.print(id);
  Serial.write(32);   // space
  Serial.println(temp);
}
// ====================================================================================================================== //


// ========== HEART RATE SENSOR ========== //

RunningMedian beatSamples = RunningMedian(4);   // take a running median of the heart rate using 4 most recent beats
float hrAvg = 0;
float hrMedian = 0;
volatile bool pulse = 0;                        // a heart pulse; declared volatile as they are used in the interrupt service routine
volatile unsigned long pulseTime = 0;           // time of the pulse
bool prevPulse = 0;
bool goodBPM = 0;
float bpm = 60;
unsigned long beatMillis = 0;
int beatPeriod = 1000;

// The main loop has different operating states as defined here:
byte heartMode = 0;
byte lastHeartMode = 99;
bool wormTravel = false;      // when worms travel all the way across toward one end
#define WAITING 0
#define CONNECTED 1
#define PLAY 2
#define MEDITATE 3
#define DISCONNECTED 4

// direction worms are traveling - basically which peltier is cold and which is warm
byte wormDir = 0;
#define LEFT_RIGHT 0
#define RIGHT_LEFT 1

// Is the detected heartbeat signal good or bad?
#define BEAT_GOOD 0
#define BEAT_BAD  1

// pins to display heartbeat on an led and to detect human contact (and begin hr monitoring)
#define PULSE_LED 24
#define CONTACT_PIN 22
#define CONTACT_CONTROL_PIN 23

// Interrupt pin for detecting a pulse
#define PULSE_INTERRUPT_PIN 2

// various time values used to sequence activities
unsigned long currentMillis = 0;
unsigned long contStartTime = 0;
unsigned long loopMillis = 0;
unsigned long detectMillis = 0;
unsigned long prevHRSerialMillis = 0;
#define HR_SERIAL_INTERVAL 100

// Interrupt handler for the heartbeat
void pulseInterrupt() {
  // Toggle the pulse flag to show we have a beat signal
  pulse = !pulse;
  // Maintain a running median of the beat period
  beatSamples.add(millis() - pulseTime);
  // Record the pulse time ready for the next period calculation
  pulseTime = millis();
}

void handlePulse() {
  prevPulse = pulse;
  // Change the state of an LED as a pulse indicator
  digitalWrite(PULSE_LED, pulse);
  //Serial.print(F("^")); // serial output to show a pulse has been detected

  // Signal contact, don't stop for a period after contact is lost to allow
  // to allow for temporary loss of ECG signal
  digitalWrite(CONTACT_CONTROL_PIN, HIGH);
  detectMillis = pulseTime + 4000;

  // if in connected, play or meditate mode make a heartbeat sound in time with the pulse
  if (heartMode == CONNECTED || heartMode == PLAY || heartMode == MEDITATE) {
    // Signal a beat - send serial data out (to Max, Processing, etc)
    sendHRSerial(heartMode, BEAT_GOOD);
    // delay the next false beat for longer than we would expect the next real one
    beatMillis = loopMillis + 1200;
  }

  // Get the median and average for comparison
  hrMedian = 60000/beatSamples.getMedian();
  hrAvg = 60000/beatSamples.getAverage();
  goodBPM = 0;
  
  // Check if median and average are close, this indicates reasonably consistant heartbeat periods
  if ((hrAvg > 50) && (hrAvg < 130) && (hrAvg < 4 + hrMedian) && (hrAvg + 4 > hrMedian)) {
    goodBPM = 1;
    bpm = hrMedian;
    // Print the rate for debug purposes
    //Serial.print(bpm);
    //Serial.println(F("bpm"));
  }
}

void flushBeatSamples() {
  beatSamples.add(9999); // Force the hearbeat samples to be a bad set
  beatSamples.add(9999); // Force the hearbeat samples to be a bad set
  beatSamples.add(9999); // Force the hearbeat samples to be a bad set
  beatSamples.add(9999); // Force the hearbeat samples to be a bad set
}

// --- Heart Rate serial communication --- //
void sendHRSerial(uint8_t mode, uint8_t beatState) {
  /*
   * Serial protocol:
   * hr [mode] [bpm] [good/bad bpm state]
   * 
   * Max will interpret any message that has a
   * CONNECTED, PLAY or MEDITATE mode as a heart beat
   */
  Serial.print('h');
  Serial.print('r');
  Serial.write(32);   // space
  Serial.print(mode);
  Serial.write(32);   // space
  Serial.print(bpm);
  Serial.write(32);   // space
  Serial.println(beatState);
}

void sendHRSerial(uint8_t mode) {
  Serial.print('h');
  Serial.print('r');
  Serial.write(32);   // space
  Serial.println(mode);
}

// --- Syringe pump serial communication --- //
void sendSPSerial(uint8_t whichPump) {
  // which syringe pump to drop food from
  Serial.print('s');
  Serial.print('p');
  Serial.write(32);   // space
  Serial.println(whichPump);
}

// ====================================================================================================================== //


// ========== LIGHTS/RELAY (BINARY COMMUNICATION) ========== //

// the relay board's input controls are Active LOW, meaning that setting a pin LOW (0) turns them ON.
// To assure that no relays activate at Reset or Power-up we want to set the pins to HIGH (1)
#define RELAY_ON 0
#define RELAY_OFF 1
#define NUM_BIN_LIGHTS 8
static const byte binLightPins[NUM_BIN_LIGHTS] = {25,26,27,28,29,30,31,32}; // pin numbers of the binary lights


// ====================================================================================================================== //


// ========== DARK-FIELD ILLUMINATOR LIGHTS (ADAFRUIT DOTSTAR LEDs) ========== //

#define NUM_DOTSTAR_LEDS 132        // Number of LEDs we are driving

// create a list of colors for the lights
// we are basically creating an HSV gradient here
#define NUM_DOTSTAR_COLORS 8
// list of colors (solid colors fading to dark)
//const uint32_t dotstarColorList[NUM_DOTSTAR_COLORS] = {0xFFFFFF, 0xDFDFDF, 0xBFBFBF, 0x9F9F9F, 0x7F7F7F, 0x5F5F5F, 0x3F3F3F, 0x1F1F1F};
// red worksbetter for dark-field illumination
const uint32_t dotstarColorList[NUM_DOTSTAR_COLORS] = {0xFF0000, 0xE50000, 0xCC0000, 0xB20000, 0x990000, 0x7F0000, 0x660000, 0x4C0000};

// Here's how to control the LEDs from any two pins:
// The below code is for software SPI on pins 8 & 9
//#define DATAPIN    8
//#define CLOCKPIN   9
/*Adafruit_DotStar strip = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);*/
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, which apprently doesn't work with the latest
// production runs. DOTSTAR_BGR worked for me.

// Hardware SPI is a little faster, but must be wired to specific pins (and that's what we're using!)
// (Arduino Uno & Pro/Pro Mini = pin 11 for data, 13 for clock, Mega 2560 = pin 51 for data, 52 for clock; other boards may be different).
Adafruit_DotStar ledStrip = Adafruit_DotStar(NUM_DOTSTAR_LEDS, DOTSTAR_BGR);

void setDotstarLEDColors(uint8_t colorIndex, uint8_t brightness) {
  // set the brightness
  ledStrip.setBrightness(brightness);

  uint32_t color;
  // select the correct color from the array
  if(colorIndex >= NUM_DOTSTAR_COLORS) {
    color = dotstarColorList[NUM_DOTSTAR_COLORS - 1]; // making sure we don't run over the array bounds
  } else {
    color = dotstarColorList[colorIndex];
  }
  
  // set the color for all the pixels/leds
  for (uint8_t i = 0; i < NUM_DOTSTAR_LEDS; ++i) {
      ledStrip.setPixelColor(i, color);
  }
  // show the updated pixels
  ledStrip.show();
}

// ====================================================================================================================== //


// +++++++++++++++++++++++++++++++ //
//  General Serial Communication   //
// +++++++++++++++++++++++++++++++ //
const int serialBufferSize = 64;
byte serialDataBuffer[serialBufferSize];
byte serialIndex = 0;

void startSerial() {
  while(Serial.available() <= 0) {
    Serial.println("0,0,0");   // send an initial string
    delay(300);
  }
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
} 

// +++++++++++++++++++++++++++++++++++++++++ //
//  Arduino setup() and main program loop()  //
// +++++++++++++++++++++++++++++++++++++++++ //

void setup() {
  // set up the binary communication light pins/relays
  for(byte i=0; i<NUM_BIN_LIGHTS; i++) {
    // the relay board's input controls are Active LOW, meaning that setting a pin LOW (0) turns them ON.
    // To assure that no relays activate at Reset or Power-up we want to set the pins to HIGH (1)
    digitalWrite(binLightPins[i], RELAY_OFF); // RELAY_OFF = HIGH (1)
    // set all the binary light pins to outputs
    pinMode(binLightPins[i], OUTPUT);
  }

  // Dotstar LEDs for dark-field illuminator
  ledStrip.begin();                  // Initialize LED pins for output
  ledStrip.clear();                  // Set all pixel data to zero
  ledStrip.show();                   // Turn all LEDs off ASAP
  // initiaize dotstars at full white/full brightness
  setDotstarLEDColors(0, 255);
  
  // initialize serial port connection at 57600 bps and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // send a byte to establish serial contact until receiver responds
  startSerial();

  // say hello!
  Serial.println(F("WormConnect v.0.1 initialized..."));
  
  // check if the peltier modules' PID controllers are on
  if(peltier0.getControllerMode() > 0) {
    Serial.println(F("Peltier module 0 is on!"));
  } else {
    Serial.println(F("Peltier module 0 is off! Something went wrong!"));
  }
  if(peltier1.getControllerMode() > 0) {
    Serial.println(F("Peltier module 1 is on!"));
  } else {
    Serial.println(F("Peltier module 1 is off! Something went wrong!"));
  }
  
  // Pin CONTACT_PIN is used to detect human contact with heart rate sensor
  pinMode(CONTACT_PIN, INPUT_PULLUP); // add a pullup so that there is a contact for testing
  pinMode(CONTACT_CONTROL_PIN, OUTPUT); // temporary control for CONTACT_PIN
  digitalWrite(CONTACT_CONTROL_PIN, LOW); // set it low for no contact

  // setup the heartbeat interrupt handler
  attachInterrupt(digitalPinToInterrupt(PULSE_INTERRUPT_PIN), pulseInterrupt, RISING); // set interrupt for pulse detection 
} // end setup()

void loop() {
  // --- Read from the Serial port --- //
  // if we get a valid byte on the serial port:
  while(Serial.available()) {
    // read a byte from the serial port and store it in data variable
    serialDataBuffer[serialIndex] = Serial.read();
    if(serialDataBuffer[serialIndex] == '\r') { // end of message, start parsing
      if(serialDataBuffer[0] == 'w') {
        wormTravel = serialDataBuffer[1]; // worm travel flag
      } else if(serialDataBuffer[0] == 'd') {
        // set the dotstar leds color & brightness
        setDotstarLEDColors(serialDataBuffer[1], serialDataBuffer[2]);
      } else {
        // turn on/off the binary communication lights
        // loop thru the message to get the pin numbers and their states
        for(byte i=0; i< NUM_BIN_LIGHTS*2; i+=2) {
          digitalWrite(serialDataBuffer[i], serialDataBuffer[i+1]);
        }
      }
      memset(serialDataBuffer, 0, sizeof(serialDataBuffer));   // Clear contents of buffer
      //serialFlush();    // empty serial buffer
      serialIndex = 0;  // reset index to 0
    } else {
      serialIndex++;
    }
  }
  
  // --- Heart Rate --- //
  // if a heartbeat interrupt has been detected...
  if (pulse != prevPulse) handlePulse();

//  // Report each operating mode transition for debug purposes
//  if (heartMode != lastHeartMode) {
//    lastHeartMode = heartMode;
//    if (heartMode == WAITING) Serial.println(F("Waiting..."));
//    if (heartMode == CONNECTED) Serial.println(F("Connected..."));
//    if (heartMode == PLAY) Serial.println(F("Playing..."));
//    if (heartMode == MEDITATE) Serial.println(F("Meditating..."));
//    if (heartMode == DISCONNECTED) Serial.println(F("Disconnected..."));
//  }
  
  // Debug info for loop speed check
  // Serial.println(millis()-loopMillis);
  loopMillis = millis();
  if (loopMillis > detectMillis) digitalWrite(CONTACT_CONTROL_PIN, LOW); // Heartbeat lost

  // Here we detect what operating mode we are in and do the appropriate thing
  switch (heartMode) {
    case WAITING:             // waiting for someone to connect
      goodBPM = 0;            // no heart beat
      flushBeatSamples();     // flush out the old heart rate samples so we always get a fresh set

      if(millis() - prevHRSerialMillis >= HR_SERIAL_INTERVAL) {
        prevHRSerialMillis = millis();  //"reset" counter
        sendHRSerial(WAITING);          // send serial data to Max
      }

      // Do we have contact, if so switch to sensing mode
      if (digitalRead(CONTACT_PIN) == 1) {
        heartMode = CONNECTED;
        // Choose a random heart rate in case we don't get a reliable ECG signal
        // this is just so the person does not walk away dissapointed!
        bpm = random(65,80);
        beatPeriod = 60000/bpm;
        beatMillis = loopMillis + beatPeriod;
      }
      // store the contact time so we can set a timer to control
      // how long we wait for a good heartbeat
      contStartTime = loopMillis; //get time at start of contact
      break;

    case CONNECTED:
      // Substitute a hearbeat in case no relaible ECG can be sensed
      if (beatMillis < loopMillis) {
        beatMillis = loopMillis + beatPeriod;
        sendHRSerial(CONNECTED, BEAT_BAD);    // send serial data to Max
        // debug and status message for a false beat...
        //Serial.print(F("False beat "));
      }
      // Wait here for a while for a steady heartbeat or a time-out of about 10 secs
      if ((millis() > (contStartTime + 12*beatPeriod)) || (goodBPM == 1)) {
        heartMode = PLAY;
        
        // --- Peltiers & Syringe Pumps --- //
        // set the temp of the peltiers based upon the direction we want the worms to move
        // and drop food via the approriate syringe pump
        if(wormDir == LEFT_RIGHT) {
          peltier0.setTargetTemp(TEMP_LO);
          peltier1.setTargetTemp(TEMP_HI);
          sendSPSerial(1);
        } else {
          peltier0.setTargetTemp(TEMP_HI);
          peltier1.setTargetTemp(TEMP_LO);
          sendSPSerial(0);
        }
          
        //Serial.println("");
      }
      // If contact is lost go back to waiting
      if (digitalRead(CONTACT_PIN) == 0) heartMode = WAITING;
      break;

    case PLAY:
      // Switch to the MEDITATE mode if the playing is over
      if(wormTravel) {
        heartMode = MEDITATE;
        beatMillis = loopMillis + (5*beatPeriod);
      }
      // Go to DISCONNECTED mode immediately if contact is lost
      if (digitalRead(CONTACT_PIN) == 0) heartMode = DISCONNECTED;
      break;
      
    case MEDITATE:
      // Go to DISCONNECTED mode immediately if contact is lost
      if (digitalRead(CONTACT_PIN) == 0) heartMode = DISCONNECTED;
      break;

    case DISCONNECTED:
      sendHRSerial(DISCONNECTED); // send serial data to Max
      if (beatMillis < loopMillis) {
        // Set up a time for the next random beat otherwise we get one immediately
        beatMillis = loopMillis + 1200 + 1000 * random(10);
        if(wormTravel) wormDir = !wormDir; // invert peltier target temperatures
        wormTravel = false;
        heartMode = WAITING;
      }
      break;
  } // end switch

  // --- Update Peltiers --- //
  if(peltier0.updateController()) {
    // debug
    //Serial.print(F("Peltier0 temp: "));
    //Serial.println(peltier0.getTemp());
    sendPeltierSerial(0, peltier0.getTemp()); // send temp over serial port
  }
  if(peltier1.updateController()) {
    // debug
    //Serial.print(F("Peltier1 temp: "));
    //Serial.println(peltier1.getTemp());
    sendPeltierSerial(1, peltier1.getTemp()); // send temp over serial port
  }
} // end loop()




