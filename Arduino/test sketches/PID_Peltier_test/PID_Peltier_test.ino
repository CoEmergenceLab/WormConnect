#include <PID_v1.h>


// ========== PID CONTROLLER FOR TWO PELTIER MODULES (labelled peltier0 & peltier1) ========== //

// === Temperature Sensor (thermistor) === //
#define TEMP0_SENSORPIN A0               // analog sensor pin
#define TEMP0_SUPPLYPIN 38               // digital pin that will serve as voltage source
#define TEMP1_SENSORPIN A1               // analog sensor pin
#define TEMP1_SUPPLYPIN 39               // digital pin that will serve as voltage source
#define TEMP_HI 20                       // high target temp (in C)
#define TEMP_LO 14                       // low target temp (in C)

// === FAN === //
#define FAN0_OUTPUT_PIN 5                // pwm output pin for fan
#define FAN1_OUTPUT_PIN 12               // pwm output pin for fan

// === PID === //
#define PID0_OUTPUT_PIN 4                // pwm output pin for peltier/pid controller
#define PID1_OUTPUT_PIN 13               // pwm output pin for peltier/pid controller


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
  double aggKp=3.9, aggKi=5, aggKd=1;
  double consKp=2, consKi=4, consKd=.75;
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
Peltier peltier0(PID0_OUTPUT_PIN, TEMP0_SENSORPIN, TEMP0_SUPPLYPIN, FAN0_OUTPUT_PIN, TEMP_LO);
//Peltier peltier1(PID1_OUTPUT_PIN, TEMP1_SENSORPIN, TEMP1_SUPPLYPIN, FAN1_OUTPUT_PIN, TEMP_HI);

// ====================================================================================================================== //


void setup() {
  // initialize serial port connection
  Serial.begin(57600);

  // check if the peltier modules' PID controllers are on
  if(peltier0.getControllerMode() > 0) {
    Serial.println(F("Peltier module 0 is on!"));
  } else {
    Serial.println(F("Peltier module 0 is off! Something went wrong!"));
  }
//  if(peltier1.getControllerMode() > 0) {
//    Serial.println(F("Peltier module 1 is on!"));
//  } else {
//    Serial.println(F("Peltier module 1 is off! Something went wrong!"));
//  }

}

void loop() {
  if(peltier0.updateController()) {
    Serial.print(F("Peltier0 temp: "));
    Serial.println(peltier0.getTemp());
  }
//  if(peltier1.updateController()) {
//    Serial.print(F("Peltier1 temp: "));
//    Serial.println(peltier1.getTemp());
//  }

}




