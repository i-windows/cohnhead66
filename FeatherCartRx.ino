// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
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


//hoverboard control variables and contstants
boolean newData = false;
char keyData;
unsigned long lastRecMillis = 0;
unsigned long lastSentMillis;
unsigned long fullStopMillis;
int moveDirection = 1;

#define maxSpeed 300
#define zeroSpeed 0
#define minSpeed -300 // Reverse max speed
#define speedStep 10// how much to accelerate
#define maxTurnRate  60
int curSpeed = 0;
int curTurn = 0;
int curCtrlMode = 2; // 2 = Speed Mode, 3 = Torque Mode
int pulseSpeed = 0;
int beepFreq = 0;
int beepTime = 0;
bool fullStop = false;
void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

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
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


void loop() {
 if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf69.lastRssi(), DEC);

      keyData = (char)buf[0];
      Serial.println(keyData);
      newData = true;
     
    } else {
      Serial.println("Receive failed");
    }
    
  }

takeActionOnNewData(keyData); //Deal with the latest remote data
  
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void takeActionOnNewData(char cr){
  int turnStep;
  
    
 
  if (newData == true) {
    
     switch(cr){
      case '0':
        Serial.println(" K = 0 ");
        pulseSpeed= constrain(pulseSpeed-speedStep,-100,0);
      break;
  
      case '1':
        Serial.println(" K = 1 ");
        pulseSpeed= constrain(pulseSpeed+speedStep,0,100);
        
      break;
  
      case '2': //Accelerate
        
        curSpeed = min(curSpeed+speedStep,maxSpeed);
        
      break;
  

      case '4':
        
        curTurn=-maxTurnRate;
        
      break;
  
      case '5':
        
        fullStop = true;
        fullStopMillis=millis();
        curTurn = 0;
        
      break;
  
      case '6':
        
        curTurn=maxTurnRate;
        
      break;
  

      
      case '8':
        
        curSpeed = max(curSpeed - speedStep,0);
        
      break;
      
      case '9':
        
        if(curSpeed == 0){
          moveDirection = moveDirection*(-1);
          delay(200);
          beepFreq = 2;
          beepTime = 1;
          
        }
        
      break;
      
       
      case 'B':
        
        if(curSpeed == 0){
          curTurn = 0;
          curCtrlMode = 3; 
        }
        
      break;


      case 'D':
        
        curSpeed = 0;
        curTurn = 0;
        curCtrlMode = 2;
      break;

      case '*':
        
        beepFreq = 6;
        beepTime = 1;
      break;

      case '#':
        
        //delay(50);
        //Serial.println("Ready to calibrate");
        //delay(1000);

      break;
        
        
      case 'X':
          
          curTurn=0;
          pulseSpeed=0;
          beepFreq = 0;
          beepTime = 0;
      break;
       
     }
     
        //Serial.println(cr);
     
     newData=false;
  }
   
 
  // Process full stop slowly decel because remote comm was lost
  if(fullStop){
    if(millis()-fullStopMillis>50){
      curSpeed = max(curSpeed - 10,0);
      if(curSpeed == 0){
        fullStop = false;
      }
      fullStopMillis = millis();
    }
  }
  

}
