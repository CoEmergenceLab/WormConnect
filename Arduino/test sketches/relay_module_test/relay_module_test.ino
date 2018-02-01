/* Example: Relay Control for 8-relay board
  Handles "Relay is active-low" to assure
  no relay activation from reset until
  application is ready.

  Based upon the example found here:
  http://arduino-info.wikispaces.com/ArduinoPower
*/


/*-----( Declare Constants )-----*/
#define RELAY_ON 0
#define RELAY_OFF 1
#define Relay_1  2  // Arduino Digital I/O pin number
#define Relay_2  3
#define Relay_3  4
#define Relay_4  5
#define Relay_5  6
#define Relay_6  7
#define Relay_7  8
#define Relay_8  9

void setup() {
//-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(Relay_1, RELAY_OFF);
  digitalWrite(Relay_2, RELAY_OFF);
  digitalWrite(Relay_3, RELAY_OFF);
  digitalWrite(Relay_4, RELAY_OFF);
  digitalWrite(Relay_5, RELAY_OFF);
  digitalWrite(Relay_6, RELAY_OFF);
  digitalWrite(Relay_7, RELAY_OFF);
  digitalWrite(Relay_8, RELAY_OFF);
  
//---( THEN set pins as outputs )----  
  pinMode(Relay_1, OUTPUT);   
  pinMode(Relay_2, OUTPUT);  
  pinMode(Relay_3, OUTPUT);  
  pinMode(Relay_4, OUTPUT);
  pinMode(Relay_5, OUTPUT);  
  pinMode(Relay_6, OUTPUT);  
  pinMode(Relay_7, OUTPUT);  
  pinMode(Relay_8, OUTPUT);  
  
  delay(4000); //Check that all relays are inactive at Reset

}//--(end setup )---


void loop() {
  // === Lets blink the first 5 prime numbers === //
  // note: order is right-to-left

  // 2 (00000010)
  digitalWrite(Relay_8, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_7, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF
  delay(50); // wait (blink)
  digitalWrite(Relay_8, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_7, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF
    
  delay(1000);

  // 3 (00000011)
  digitalWrite(Relay_8, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_7, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF
  delay(50); // wait (blink)
  digitalWrite(Relay_8, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_7, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF
  
  delay(1000);

  // 5 (00000101)
  digitalWrite(Relay_8, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_7, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_6, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF
  delay(50); // wait (blink)
  digitalWrite(Relay_8, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_7, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF

  delay(1000);

  // 7 (00000111)
  digitalWrite(Relay_8, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_7, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_6, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF
  delay(50); // wait (blink)
  digitalWrite(Relay_8, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_7, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF

  delay(1000);

  // 11 (00001011)
  digitalWrite(Relay_8, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_7, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_ON);  // set the Relay ON
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF
  delay(50); // wait (blink)
  digitalWrite(Relay_8, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_7, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_6, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_5, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_4, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_3, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_2, RELAY_OFF); // set the Relay OFF
  digitalWrite(Relay_1, RELAY_OFF); // set the Relay OFF

  delay(1000);
  
////---( Turn all 8 relays ON in sequence)---
//  digitalWrite(Relay_1, RELAY_ON);// set the Relay ON
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_2, RELAY_ON);// set the Relay ON
//  delay(1000);              // wait for a second  
//  digitalWrite(Relay_3, RELAY_ON);// set the Relay ON
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_4, RELAY_ON);// set the Relay ON
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_5, RELAY_ON);// set the Relay ON
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_6, RELAY_ON);// set the Relay ON
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_7, RELAY_ON);// set the Relay ON
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_8, RELAY_ON);// set the Relay ON
//
//  delay(4000);              // wait see all relays ON
//  
////---( Turn all 8 relays OFF in sequence)---  
//  digitalWrite(Relay_1, RELAY_OFF);// set the Relay OFF
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_2, RELAY_OFF);// set the Relay OFF
//  delay(1000);              // wait for a second  
//  digitalWrite(Relay_3, RELAY_OFF);// set the Relay OFF
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_4, RELAY_OFF);// set the Relay OFF
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_5, RELAY_OFF);// set the Relay OFF
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_6, RELAY_OFF);// set the Relay OFF
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_7, RELAY_OFF);// set the Relay OFF
//  delay(1000);              // wait for a second
//  digitalWrite(Relay_8, RELAY_OFF);// set the Relay OFF
//  
//  delay(4000);              // wait see all relays OFF  

} //--(end main loop )---



