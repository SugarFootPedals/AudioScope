
/*
     AudiOscope DSO V4c Digital Storage Oscilloscope
    Original Code and Schematic from http://radiopench.blog96.fc2.com/
    
	Minor Code and Schematic/Circuit modifications by Vibrato, LLC. 09/22/2019
	Circuit Board Layout and Verified by Vibrato, LLC. 09/22/2019
  Deleted parts related to Switch Indicator LED (C52,R52,Q1,R53,LED,R54)- not necessary
  Added Power Circuit with Regulator and Power Switch. Added AC/DC Coupling with Offest Control.
  
  NOTE: Once the ProMini is programmed with this code, I found it will no longer accept any other code.
	
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128                // OLED display width
#define SCREEN_HEIGHT 64                // OLED display height
#define REC_LENGTH 200                  // 
#define MIN_TRIG_SWING 5                // 

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1      // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


const char vRangeName[10][5] PROGMEM = {"A50V", "A 5V", " 50V", " 20V", " 10V", "  5V", "  2V", "  1V", "0.5V", "0.2V"};  
const char * const vstring_table[] PROGMEM = {vRangeName[0], vRangeName[1], vRangeName[2], vRangeName[3], vRangeName[4], vRangeName[5], vRangeName[6], vRangeName[7], vRangeName[8], vRangeName[9]};
const char hRangeName[10][6] PROGMEM = {"200ms", "100ms", " 50ms", " 20ms", " 10ms", "  5ms", "  2ms", "  1ms", "500us", "200us"};  
const char * const hstring_table[] PROGMEM = {hRangeName[0], hRangeName[1], hRangeName[2], hRangeName[3], hRangeName[4], hRangeName[5], hRangeName[6], hRangeName[7], hRangeName[8], hRangeName[9]};

int waveBuff[REC_LENGTH];      
char chrBuff[8];               
char hScale[] = "xxxAs";       
char vScale[] = "xxxx";        

float lsb5V = 0.0055549;       
float lsb50V = 0.0503931;      

volatile int vRange;           //  0:A50V, 1:A 5V, 2:50V, 3:20V, 4:10V, 5:5V, 6:2V, 7:1V, 8:0.5V
volatile int hRange;           // 0:50m, 1:20m, 2:10m, 3:5m, 4;2m, 5:1m, 6:500u, 7;200u
volatile int trigD;            
volatile int scopeP;           
volatile boolean hold = false; 
volatile boolean switchPushed = false; 
volatile int saveTimer;        
int timeExec;                  // (ms)

int dataMin;                   // (min:0)
int dataMax;                   // (max:1023)
int dataAve;                   // (max:10230)
int rangeMax;                  
int rangeMin;                  
int rangeMaxDisp;              
int rangeMinDisp;              
int trigP;                     
boolean trigSync;              
int att10x;                    

void setup() {
  pinMode(2, INPUT_PULLUP);    
  pinMode(8, INPUT_PULLUP);    // Select
  pinMode(9, INPUT_PULLUP);    // Up
  pinMode(10, INPUT_PULLUP);   // Down
  pinMode(11, INPUT_PULLUP);   // Hold 
  pinMode(12, INPUT);          
  pinMode(13, OUTPUT);         

  //       Serial.begin(115200);        
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    //       Serial.println(F("SSD1306 failed"));
    for (;;);                               // Don't proceed, loop forever
  }
  auxFunctions();                           
  loadEEPROM();                             
  analogReference(INTERNAL);                
  attachInterrupt(0, pin2IRQ, FALLING);     // (int.0)
  startScreen();                            
}

void loop() {
  setConditions();                          
  digitalWrite(13, HIGH);
  readWave();                               
  digitalWrite(13, LOW);                    
  setConditions();                          
  dataAnalize();                            // (0.5-0.6ms)
  writeCommonImage();                       // (2.6ms)
  plotData();                               // (10-18ms)
  dispInf();                                // (6.5-8.5ms)
  display.display();                        // (37ms)
  saveEEPROM();                              
  while (hold == true) {                    // Hold
    dispHold();
    delay(10);
  }                                         
}

void setConditions() {            
  
  strcpy_P(hScale, (char*)pgm_read_word(&(hstring_table[hRange])));  
  strcpy_P(vScale, (char*)pgm_read_word(&(vstring_table[vRange])));  

  switch (vRange) {              
    case 0: {                    // Auto50V
        att10x = 1;              
        break;
      }
    case 1: {                    // Auto 5V
        att10x = 0;              
        break;
      }
    case 2: {                    // 50V
        rangeMax = 50 / lsb50V;  
        rangeMaxDisp = 5000;     
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 1;              
        break;
      }
    case 3: {                    // 20V
        rangeMax = 20 / lsb50V;  
        rangeMaxDisp = 2000;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 1;              
        break;
      }
    case 4: {                    // 10V
        rangeMax = 10 / lsb50V;  
        rangeMaxDisp = 1000;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 1;              
        break;
      }
    case 5: {                    // 5V
        rangeMax = 5 / lsb5V;    
        rangeMaxDisp = 500;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              
        break;
      }
    case 6: {                    // 2V
        rangeMax = 2 / lsb5V;    
        rangeMaxDisp = 200;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              
        break;
      }
    case 7: {                    // 1V
        rangeMax = 1 / lsb5V;    
        rangeMaxDisp = 100;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              
        break;
      }
    case 8: {                    // 0.5V
        rangeMax = 0.5 / lsb5V;  
        rangeMaxDisp = 50;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              
        break;
      }
    case 9: {                    // 0.2V
        rangeMax = 0.2 / lsb5V;  
        rangeMaxDisp = 20;
        rangeMin = 0;
        rangeMinDisp = 0;
        att10x = 0;              
        break;
      }
  }
}

void writeCommonImage() {                   
  display.clearDisplay();                   // (0.4ms)
  display.setTextColor(WHITE);              
  display.setCursor(86, 0);                 // Start at top-left corner
  display.println(F("av    V"));            
  display.drawFastVLine(26, 9, 55, WHITE);  
  display.drawFastVLine(127, 9, 55, WHITE); 

  display.drawFastHLine(24, 9, 7, WHITE);   
  display.drawFastHLine(24, 36, 2, WHITE);  
  display.drawFastHLine(24, 63, 7, WHITE);  

  display.drawFastHLine(51, 9, 3, WHITE);   
  display.drawFastHLine(51, 63, 3, WHITE);  

  display.drawFastHLine(76, 9, 3, WHITE);   
  display.drawFastHLine(76, 63, 3, WHITE);  

  display.drawFastHLine(101, 9, 3, WHITE);  
  display.drawFastHLine(101, 63, 3, WHITE); 

  display.drawFastHLine(123, 9, 5, WHITE);  
  display.drawFastHLine(123, 63, 5, WHITE); 

  for (int x = 26; x <= 128; x += 5) {
    display.drawFastHLine(x, 36, 2, WHITE); 
  }
  for (int x = (127 - 25); x > 30; x -= 25) {
    for (int y = 10; y < 63; y += 5) {
      display.drawFastVLine(x, y, 2, WHITE); // 
    }
  }
}

void readWave() {                            
  if (att10x == 1) {                         
    pinMode(12, OUTPUT);                     
    digitalWrite(12, LOW);                   
  } else {                                   
    pinMode(12, INPUT);                      
  }
  switchPushed = false;                      
  switch (hRange) {                          
    case 0: {                                // 200ms
        timeExec = 1600 + 60;                 
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x07;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(7888);           
          if (switchPushed == true) {        
            switchPushed = false;
            break;                           
          }
        }
        break;
      }
    case 1: {                                // 100ms
        timeExec = 800 + 60;                 
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x07;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(3888);           
          if (switchPushed == true) {        
            switchPushed = false;
            break;                           
          }
        }
        break;
      }
    case 2: {                                // 50ms
        timeExec = 400 + 60;                  
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x07;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(1888);           
          if (switchPushed == true) {        
            break;                           
          }
        }
        break;
      }
    case 3: {                                // 20ms
        timeExec = 160 + 60;                 
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x07;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(688);            
          if (switchPushed == true) {        
            break;                           
          }
        }
        break;
      }
    case 4: {                                // 10 ms
        timeExec = 80 + 60;                  
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x07;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(288);            
          if (switchPushed == true) {        
            break;                           
          }
        }
        break;
      }
    case 5: {                                // 5 ms
        timeExec = 40 + 60;                  
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x07;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(88);             
          if (switchPushed == true) {        
            break;                           
          }
        }
        break;
      }
    case 6: {                                // 2 ms
        timeExec = 16 + 60;                  
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x06;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(24);             
        }
        break;
      }
    case 7: {                                // 1 ms
        timeExec = 8 + 60;                   
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x05;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(12);             
        }
        break;
      }
    case 8: {                                // 500us
        timeExec = 4 + 60;                   
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x04;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          delayMicroseconds(4);              
          
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
          asm("nop"); asm("nop"); asm("nop");
        }
        break;
      }
    case 9: {                                // 200us
        timeExec = 2 + 60;                   
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x02;              
        for (int i = 0; i < REC_LENGTH; i++) {  
          waveBuff[i] = analogRead(0);       
          
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
          asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
        }
        break;
      }
  }
}

void dataAnalize() {                      
  int d;
  long sum = 0;

 
  dataMin = 1023;                         
  dataMax = 0;                            
  for (int i = 0; i < REC_LENGTH; i++) {  
    d = waveBuff[i];
    sum = sum + d;
    if (d < dataMin) {                    
      dataMin = d;
    }
    if (d > dataMax) {                    
      dataMax = d;
    }
  }

 
  dataAve = (sum + 10) / 20;               

  
  if (vRange <= 1) {                       
    rangeMin = dataMin - 20;               
    rangeMin = (rangeMin / 10) * 10;       
    if (rangeMin < 0) {
      rangeMin = 0;                        
    }
    rangeMax = dataMax + 20;               
    rangeMax = ((rangeMax / 10) + 1) * 10; 
    if (rangeMax > 1020) {
      rangeMax = 1023;                     
    }

    if (att10x == 1) {                            
      rangeMaxDisp = 100 * (rangeMax * lsb50V);   
      rangeMinDisp = 100 * (rangeMin * lsb50V);   
    } else {                                      
      rangeMaxDisp = 100 * (rangeMax * lsb5V);
      rangeMinDisp = 100 * (rangeMin * lsb5V);
    }
  } else {                                 
    
  }

 
  for (trigP = ((REC_LENGTH / 2) - 51); trigP < ((REC_LENGTH / 2) + 50); trigP++) { 
    if (trigD == 0) {                      
      if ((waveBuff[trigP - 1] < (dataMax + dataMin) / 2) && (waveBuff[trigP] >= (dataMax + dataMin) / 2)) {
        break;                             
      }
    } else {                               
      if ((waveBuff[trigP - 1] > (dataMax + dataMin) / 2) && (waveBuff[trigP] <= (dataMax + dataMin) / 2)) {
        break;
      }                                    
    }
  }
  trigSync = true;
  if (trigP >= ((REC_LENGTH / 2) + 50)) {  
    trigP = (REC_LENGTH / 2);              
    trigSync = false;                      
  }
  if ((dataMax - dataMin) <= MIN_TRIG_SWING) { 
    trigSync = false;                      
  }
}

void startScreen() {                         
  display.clearDisplay();
  display.setTextSize(1);                    
  display.setTextColor(WHITE);
  display.setCursor(15, 15);
  display.println(F("AudiOscope"));           
  display.setCursor(15, 35);
  display.println(F("DSO v1.1"));
  display.display();                         
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);                    
}

void dispHold() {                            
  display.fillRect(42, 11, 24, 8, BLACK);    
  display.setCursor(42, 11);
  display.print(F("Hold"));                  
  display.display();                         
}

void dispInf() {                             
  float voltage;
 
  display.setCursor(2, 0);                   
  display.print(vScale);                     
  if (scopeP == 0) {                         
    display.drawFastHLine(0, 7, 27, WHITE);  
    display.drawFastVLine(0, 5, 2, WHITE);
    display.drawFastVLine(26, 5, 2, WHITE);
  }


  display.setCursor(34, 0);                  
  display.print(hScale);                     
  if (scopeP == 1) {                         
    display.drawFastHLine(32, 7, 33, WHITE); 
    display.drawFastVLine(32, 5, 2, WHITE);
    display.drawFastVLine(64, 5, 2, WHITE);
  }


  display.setCursor(75, 0);                 
  if (trigD == 0) {
    display.print(char(0x18));               
  } else {
    display.print(char(0x19));              
  }
  if (scopeP == 2) {      
    display.drawFastHLine(71, 7, 13, WHITE); 
    display.drawFastVLine(71, 5, 2, WHITE);
    display.drawFastVLine(83, 5, 2, WHITE);
  }


  if (att10x == 1) {                         
    voltage = dataAve * lsb50V / 10.0;       
  } else {
    voltage = dataAve * lsb5V / 10.0;        
  }
  if (voltage < 10.0) {                      
    dtostrf(voltage, 4, 2, chrBuff);         
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);         
  }
  display.setCursor(98, 0);                  
  display.print(chrBuff);                    
  //  display.print(saveTimer);                  


  voltage = rangeMaxDisp / 100.0;            
  if (vRange == 1 || vRange > 4) {           
    dtostrf(voltage, 4, 2, chrBuff);         
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);         
  }
  display.setCursor(0, 9);
  display.print(chrBuff);                    

  voltage = (rangeMaxDisp + rangeMinDisp) / 200.0; 
  if (vRange == 1 || vRange > 4) {           
    dtostrf(voltage, 4, 2, chrBuff);         
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);         
  }
  display.setCursor(0, 33);
  display.print(chrBuff);                    

  voltage = rangeMinDisp / 100.0;            
  if (vRange == 1 || vRange > 4) {           
    dtostrf(voltage, 4, 2, chrBuff);         
  } else {
    dtostrf(voltage, 4, 1, chrBuff);         
  }
  display.setCursor(0, 57);
  display.print(chrBuff);                    


  if (trigSync == false) {                   
    display.fillRect(85, 11, 24, 8, BLACK);  
    display.setCursor(85, 11);               
    display.print(F("Unsync"));              
  }
}

void plotData() {                    
  long y1, y2;
  for (int x = 0; x <= 98; x++) {
    y1 = map(waveBuff[x + trigP - 50], rangeMin, rangeMax, 63, 9); 
    y1 = constrain(y1, 9, 63);                                     
    y2 = map(waveBuff[x + trigP - 49], rangeMin, rangeMax, 63, 9); 
    y2 = constrain(y2, 9, 63);                                     
    display.drawLine(x + 27, y1, x + 28, y2, WHITE);               
  }
}

void saveEEPROM() {                    
  if (saveTimer > 0) {                 
    saveTimer = saveTimer - timeExec;  
    if (saveTimer < 0) {               
      EEPROM.write(0, vRange);        
      EEPROM.write(1, hRange);
      EEPROM.write(2, trigD);
      EEPROM.write(3, scopeP);
    }
  }
}

void loadEEPROM() {                
  int x;
  x = EEPROM.read(0);             // vRange
  if ((x < 0) || (x > 9)) {        
    x = 3;                         
  }
  vRange = x;

  x = EEPROM.read(1);             // hRange
  if ((x < 0) || (x > 7)) {        
    x = 3;                         
  }
  hRange = x;
  x = EEPROM.read(2);             // trigD
  if ((x < 0) || (x > 1)) {        
    x = 1;                         
  }
  trigD = x;
  x = EEPROM.read(3);             // scopeP
  if ((x < 0) || (x > 2)) {        
    x = 1;                         
  }
  scopeP = x;
}

void auxFunctions() {                         
  float voltage;
  if (digitalRead(8) == LOW) {                // SELECT
    analogReference(DEFAULT);                 // ADC
    while (1) {                               
      voltage = 5.0 * analogRead(1) / 1023.0; 
      display.clearDisplay();                 
      display.setTextColor(WHITE);            
      display.setCursor(20, 16);              
      display.setTextSize(1);                 
      display.println(F("Battery voltage"));
      display.setCursor(35, 30);              
      display.setTextSize(2);                 
      dtostrf(voltage, 4, 2, chrBuff);        // x.xx 
      display.print(chrBuff);
      display.println(F("V"));
      display.display();
      delay(150);
    }
  }
  if (digitalRead(10) == LOW) {                // UP
    analogReference(INTERNAL);
    pinMode(12, INPUT);                       
    while (1) {                               
      digitalWrite(13, HIGH);                 // LED
      voltage = analogRead(0) * lsb5V;        
      display.clearDisplay();                 
      display.setTextColor(WHITE);            
      display.setCursor(26, 16);              
      display.setTextSize(1);                 
      display.println(F("DVM 5V Range"));
      display.setCursor(35, 30);              
      display.setTextSize(2);                 
      dtostrf(voltage, 4, 2, chrBuff);        // x.xx 
      display.print(chrBuff);
      display.println(F("V"));
      display.display();
      digitalWrite(13, LOW);
      delay(150);
    }
  }
  if (digitalRead(9) == LOW) {               // DOWN
    analogReference(INTERNAL);
    pinMode(12, OUTPUT);                      
    digitalWrite(12, LOW);                    
    while (1) {                               
      digitalWrite(13, HIGH);                 
      voltage = analogRead(0) * lsb50V;       
      display.clearDisplay();                 
      display.setTextColor(WHITE);            
      display.setCursor(26, 16);              
      display.setTextSize(1);                 
      display.println(F("DVM 50V Range"));
      display.setCursor(35, 30);              
      display.setTextSize(2);                 
      dtostrf(voltage, 4, 1, chrBuff);        
      display.print(chrBuff);
      display.println(F("V"));
      display.display();
      digitalWrite(13, LOW);
      delay(150);
    }
  }
}

void pin2IRQ() {                   
  
  

  int x;                           
  x = PINB;                        

  if ( (x & 0x07) != 0x07) {       
    saveTimer = 5000;              
    switchPushed = true;           
  }

  if ((x & 0x01) == 0) {           
    scopeP++;                      
    if (scopeP > 2) {              
      scopeP = 0;                  
    }
  }

  if ((x & 0x02) == 0) {           
    if (scopeP == 0) {             
      vRange++;                    
      if (vRange > 9) {            
        vRange = 9;
      }
    }
    if (scopeP == 1) {             
      hRange++;
      if (hRange > 9) {            
        hRange = 9;                
      }
    }
    if (scopeP == 2) {             
      trigD = 0;                   
    }
  }

  if ((x & 0x04) == 0) {           
    if (scopeP == 0) {             
      vRange--;                    
      if (vRange < 0) {            
        vRange = 0;                
      }
    }
    if (scopeP == 1) {             
      hRange--;                    
      if (hRange < 0) {            
        hRange = 0;                
      }
    }
    if (scopeP == 2) {             
      trigD = 1;                   
    }
  }

  if ((x & 0x08) == 0) {           
    hold = ! hold;                 
  }
}
