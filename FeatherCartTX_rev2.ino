// rf69 demo tx rx.pde
// -*- mode: C++ -*-

// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif

/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

//define contstants
#define BTN_IDLE 0
#define BTN_PRESSED 1
#define BTN_HELD 2
#define BTN_RELEASED 3
#define REMOTE_START_FRAME  0xAAA1  
#define VBATPIN A7

//#define NO_PRESS_SLEEP_TIME 60000 

unsigned long lastSentMillis = millis();
unsigned long lastPressedMillis = millis();
unsigned long lastBlinkMillis = millis();

int buttonInputs[] ={15,15,15,16,16,10,10,16, 6,18,16, 5};
int buttonOutputs[]={17,10,16,17, 6,16, 6,18,17,17, 5,17};
int buttonStates[]={0,0,0,0,0,0,0,0,0,0,0,0};
int previousButtonStates[] = {0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long previousButtonMillis[] = {0,0,0,0,0,0,0,0,0,0,0,0};
char pressedChars[] = {'B','D','2','4','5','6','8','9','#','0','*','1'};
//0 -- Pulse back
//1 -- Pulse fwd
//2 -- Accelerate
//4 -- Turn Left
//5 -- Full Stop
//6 -- Turn Right
//7 -- N/A
//8 -- Decel
//9 -- Reverse
//B -- Freewheel
//D -- Motor Enguage
//* -- Beep
//# -- N/A

int debounceMillis = 40;
int repeatKeyMillis = 200;

//define varriables
char key = char('X');
char previousKey = char('X');

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

   //Set all button pins to inputs just to be safe
  for(int btn=0; btn<12; btn++){ // loop for all the btns
    pinMode(buttonInputs[btn],INPUT_PULLUP);
    pinMode(buttonOutputs[btn],INPUT_PULLUP);

 
    
  }

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

 
  //Set sync words 
  uint8_t syncWords[] = {0x01, 0x02};
  rf69.setSyncWords(syncWords,2);

  
 Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}



void loop() {

  readButtons();
  processButtons();
  sendData();
  ledBlink();
}

void ledBlink(){
  if(millis()-lastBlinkMillis >5000){
    digitalWrite(LED,LOW);
    delay(10);
    digitalWrite(LED,LOW);
    lastBlinkMillis = millis();
  }
  else{
    delay(10);
  }
}
void sendData(){
  if(millis()-lastPressedMillis > 60000){
    return;
  }
  if(key == previousKey && millis()-lastSentMillis < repeatKeyMillis){      //wait for keyRepeat time
       return;       
    }
    
  char radiopacket[2] = "X";
  radiopacket[0] = key;
  //itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  
  // Send a message!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  previousKey = key;
  lastSentMillis = millis();
  rf69.sleep();

  //report bat voltage
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
}



void readButtons(){
  for(int btn=0;btn<12;btn++){                                                // do this loop for all the btns
    previousButtonStates[btn]=buttonStates[btn];                              //copy the currentstate to the previous state
    pinMode(buttonOutputs[btn],OUTPUT);                                       // change one pin to a low output
    digitalWrite(buttonOutputs[btn],LOW);
    if(!digitalRead(buttonInputs[btn])){                                      // button is down so do something like set the button state
      if(previousButtonStates[btn] == BTN_IDLE){
        buttonStates[btn] = BTN_PRESSED;
        previousButtonMillis[btn]= millis();
      }
      else if(millis()-previousButtonMillis[btn] > debounceMillis ){  //debounce time is over transition to held
        buttonStates[btn]=BTN_HELD;
      }
    }
    else{                                                                     //button is up so do something like change the button state
      if(previousButtonStates[btn] == BTN_PRESSED){
        buttonStates[btn]=BTN_IDLE;
        previousButtonMillis[btn]= 0;
      }
      else if(previousButtonStates[btn] == BTN_HELD){
        buttonStates[btn]=BTN_RELEASED;
        previousButtonMillis[btn]= 0;
      }
      else if(previousButtonStates[btn] == BTN_RELEASED){
        buttonStates[btn]=BTN_IDLE;
        previousButtonMillis[btn]= 0;        
      }
    }
    digitalWrite(buttonOutputs[btn],HIGH);                //Set pin back the way we found it
    pinMode(buttonOutputs[btn],INPUT_PULLUP); 
  }
}

void processButtons(){
  for(int btn=0;btn<12;btn++){ 
    if (buttonStates[btn] == BTN_HELD){
    key= pressedChars[btn];
    if (key == char('#')){
      Serial.println("checking voltage");

       // Blink LED for battery voltage
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 4200; // convert to voltage
    Serial.println(measuredvbat);
    
    for(int blinkCount = 0; blinkCount<10; blinkCount++){
      digitalWrite(13,HIGH);
      delay(200);
      digitalWrite(13,LOW);
      delay(200);
    }
    }
    lastPressedMillis = millis();
    }
    
    if (buttonStates[btn] == BTN_RELEASED){
    key= char('X');
    
    
    }
  }
}
