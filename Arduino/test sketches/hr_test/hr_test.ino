volatile bool pulse = 0;                        // a heart pulse; declared volatile as they are used in the interrupt service routine
bool prevPulse = 0;
#define PULSE_LED 13
#define PULSE_INTERRUPT_PIN 2

void setup() {
  Serial.begin(57600);
  pinMode(PULSE_LED, OUTPUT);
  // put your setup code here, to run once:
  // setup the heartbeat interrupt handler
  attachInterrupt(digitalPinToInterrupt(PULSE_INTERRUPT_PIN), pulseInterrupt, RISING); // set interrupt for pulse detection 
}

void loop() {
  // if a heartbeat interrupt has been detected...
  if (pulse != prevPulse) handlePulse();
}

// Interrupt handler for the heartbeat
void pulseInterrupt() {
  // Toggle the pulse flag to show we have a beat signal
  pulse = !pulse;
}

void handlePulse() {
  prevPulse = pulse;
  // Change the state of an LED as a pulse indicator
  digitalWrite(PULSE_LED, pulse);
  Serial.println(F("^")); // serial output to show a pulse has been detected
}


