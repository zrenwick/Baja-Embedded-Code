#include <stdio.h>
#include <Wire.h>      // Include I2C library
#include "RTClib.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345.h>
#include <EEPROM.h>
#include <avr/wdt.h>   // Enable watchdog timer
#include <SdFat.h>
#include <SdFatUtil.h>
#include <avr/interrupt.h>  //enable interrupts
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

//FIFO_CTL register options
#define STREAM                           (0x80)     //Set accel to stream mode
#define FIFO_PULL_LENGTH                 23         // # Data to pull from FIFO at once (1-31)
#define SAMP_BYT_SZ                      19         //# of bytes per sample write

//DATA_FORMAT register options
#define RANGE_16G                        (0x03)     //Set accel to -16g --> 16g range       
#define FULL_RES                         (0x08)     // Set accel to full resolution
#define INT_INVERT                       (0x20)     //Invert interrupts to active low 

//INT_ENABLE register options
#define WATER_MARK                       (0x02)     //Enables watermark interrupt (when fifo is full)

//Data conversion constants

//Display addresses
#define  DISPLAY_ADDR1                   0x70
#define  DISPLAY_ADDR2                   0x71
#define LED_POWER_PIN                    5        // Pin that button ring LED is connected to
#define BUTTON_PIN                       8        // Pin that reads if the button is pressed
#define PHONE_DELAY                      15000

#define CHIP_SELECT                      10       // CS for SD card
#define TIME_ADDRESS                     1        // Address in EEPROM for filenum
#define PHONE_PIN                        9
#define CALL_PIN                         6        // This pin is written high to press the call button on the phone
#define END_PIN                          7        // This pin is written high to press the end button on the phone

SdFat sd;
SdFile dataFile;
RTC_DS1307 RTC;
Adafruit_ADXL345 accel = Adafruit_ADXL345(12345);
Adafruit_7segment displayTop = Adafruit_7segment();
Adafruit_7segment displayBottom = Adafruit_7segment();

//Initialization Variables
unsigned int fileNum = 0;
unsigned char address = 0;
char fileName[8];
boolean settime;

//Output Buffer
byte dataBuffer[(SAMP_BYT_SZ*FIFO_PULL_LENGTH)];

//Data Variables
unsigned long dataTime = 0;

//Accelerometer status variables
boolean data_ready = 0;
boolean overrun = 0;            //To hold overrun bit (overrun = 1 means data has been lost)

//Loop time counters
unsigned long t_start = 0;
unsigned long t_cycle = 0;
unsigned int loop_cnt = 0;

//Tachometer time counters
unsigned long eng_start_t = 0;
unsigned long eng_finish_t = 0;
unsigned long eng_cycle_t = 1;

//Spedometer time counters
unsigned long out_start_t = 0;
unsigned long out_finish_t = 0;
unsigned long out_cycle_t = 1;
unsigned char int_cnt = 0;

//Display variables
float eng_rpm = 0;
float car_speed = 0;
boolean phoneCall = 0;
boolean callinProgress = 0;
unsigned long callTime = 0;
boolean buttonPress= 0;
boolean pitAlert = 0;
unsigned long alertTime = 0;
boolean pitCall = 0;
unsigned long pitTime = 0;
const float MI_PER_ROT = 0.00124;


void setup(void){
  wdt_enable(WDTO_4S);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C communication
  RTC.begin();   // Initialize Real-Time clock
  displayTop.begin(DISPLAY_ADDR1);
  displayBottom.begin(DISPLAY_ADDR2);
  
 
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
  }
  
  settime = EEPROM.read(TIME_ADDRESS);
  if(!settime){ 
    RTC.adjust(DateTime(__DATE__,__TIME__));
    EEPROM.write(TIME_ADDRESS, 1);
  }
  SdFile::dateTimeCallback(dateTime); 
  DateTime now = RTC.now();  //Get current time;  
  
  
  //Setup File name
  fileNum = EEPROM.read(0);
  fileNum = fileNum+1;
  if(fileNum == 255){
    fileNum = 1;    //One Byte can only hold a value from 0 to 255, so if 255 is hit, start over at 1
  }
  
  EEPROM.write(address, fileNum);
  sprintf(fileName, "%d.txt", fileNum);
  Serial.println(fileName);

  //Set up the SD card  
  pinMode(CHIP_SELECT, OUTPUT);
  Serial.println("Initializing SD");
  
  // see if the card is present and can be initialized:
  if (!sd.begin(CHIP_SELECT, SPI_FULL_SPEED)) {
    Serial.println("Card failed, or not present... wait for reset");
    cardErrorDisplay(); // Display "CArd Err"
    delay(5000);  //Wait for 5 sec. to trigger watchdog reset (after 4 sec).
  }
  
  pinMode(PHONE_PIN, INPUT); 
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(CALL_PIN, OUTPUT);
  pinMode(END_PIN, OUTPUT);
  
  phoneOn();    //Put this write before the displays so phone has time to turn on
  bajaDisplay();
  fileDisplay(fileNum);  //Display File number on LED display
  phoneCheck();
  
  sei();  //global interrupt enable
  //INT0 is ATmega pin 4 Arduino pin 2
  //INT1 is ATmega pin 5 Arduino pin 3
  //PCINT on ATmega pin 6, Arduino pin 4
  pinMode(2, INPUT_PULLUP);  //Enable pullup resistors on pin 2
  //Pins 3 and 4 are pulled up via hardware
  EICRA  |= 0x0A;    // Interrupts on INT0 & INT1 caused by falling edge to catch accelerometer interrupt and wheel rotation interrupt respectively 
  EIMSK |= 0x03;     // Enable INT0 & INT1 interrupts to catch accelerometer and wheel interrupts
  PCICR |= 0x04;  // Enable use of interrupts on ATmega pins 0-7
  PCMSK2 |= 0x10; //Enable interrupts on ATmega pin 6 = arduino pin 4. 

  accel.begin();
  accel.setDataRate(ADXL345_DATARATE_200_HZ);     //400Hz max
  regWrite( ADXL345_ADDRESS, ADXL345_REG_FIFO_CTL, STREAM + FIFO_PULL_LENGTH);  //Set FIFO to stream mode and set watermark to 31 data points
  regWrite( ADXL345_ADDRESS, ADXL345_REG_DATA_FORMAT, (RANGE_16G + FULL_RES + INT_INVERT));
  regWrite( ADXL345_ADDRESS, ADXL345_REG_INT_ENABLE, WATER_MARK);
  for(int cnt=0; cnt<FIFO_PULL_LENGTH; cnt++){
    regRead(ADXL345_ADDRESS, ADXL345_REG_DATAX0, 6);
  }
}

void loop(void){
  wdt_reset();
  
  if(data_ready){
    data_ready = 0;
    t_cycle = millis()-t_start;
    t_start = millis();
   
    regRead(ADXL345_ADDRESS, ADXL345_REG_INT_SOURCE, 1);
    overrun = (Wire.read() & (0x01));
    
    //Empty accelerometer readings as quickly as possible
    for(int cnt=0; cnt<FIFO_PULL_LENGTH; cnt++){
      //Remember, the data is FIFO (first in, first out) so first data pulled is the oldest  
      regRead(ADXL345_ADDRESS, ADXL345_REG_DATAX0, 6);
      dataBuffer[SAMP_BYT_SZ*cnt+4] = Wire.read();  //read DataX0 register
      dataBuffer[SAMP_BYT_SZ*cnt+5] = Wire.read();  //read DataX1 register
      dataBuffer[SAMP_BYT_SZ*cnt+6] = Wire.read();  //read DataY0 register
      dataBuffer[SAMP_BYT_SZ*cnt+7] = Wire.read();  //read DataY1 register
      dataBuffer[SAMP_BYT_SZ*cnt+8] = Wire.read();  //read DataZ0 register
      dataBuffer[SAMP_BYT_SZ*cnt+9] = Wire.read();  //read DataZ1 register
    }
  
    
    boolean opencheck = dataFile.open(fileName, O_WRITE | O_CREAT | O_APPEND);
    if(opencheck){
      for(int cnt=0; cnt<FIFO_PULL_LENGTH; cnt++){
        dataTime = (t_start - ((t_cycle*(FIFO_PULL_LENGTH - cnt))/FIFO_PULL_LENGTH) );  //Last data pulled is the most recent, 
        dataBuffer[SAMP_BYT_SZ*cnt]   = dataTime;
        dataBuffer[SAMP_BYT_SZ*cnt+1] = (dataTime>>8);
        dataBuffer[SAMP_BYT_SZ*cnt+2] = (dataTime>>16);
        dataBuffer[SAMP_BYT_SZ*cnt+3] = (dataTime>>24);
        dataBuffer[SAMP_BYT_SZ*cnt+10]= eng_cycle_t;
        dataBuffer[SAMP_BYT_SZ*cnt+11]= (eng_cycle_t>>8);
        dataBuffer[SAMP_BYT_SZ*cnt+12]= (eng_cycle_t>>16);
        dataBuffer[SAMP_BYT_SZ*cnt+13]= (eng_cycle_t>>24);
        dataBuffer[SAMP_BYT_SZ*cnt+14]= out_cycle_t;
        dataBuffer[SAMP_BYT_SZ*cnt+15]= (out_cycle_t>>8);
        dataBuffer[SAMP_BYT_SZ*cnt+16]= (out_cycle_t>>16);
        dataBuffer[SAMP_BYT_SZ*cnt+17]= (out_cycle_t>>24);
        dataBuffer[SAMP_BYT_SZ*cnt+18]= overrun;
      }
      dataFile.write(dataBuffer, (SAMP_BYT_SZ*FIFO_PULL_LENGTH));
      dataFile.close();
      //Set everything to zero to see if something isn't getting set.
      for(int cnt=0; cnt<(SAMP_BYT_SZ*FIFO_PULL_LENGTH); cnt++){
        dataBuffer[cnt] = 0;
      }
    } 
    else {
      Serial.print("File failed to open. Wait for Reset");
      cardErrorDisplay(); // Display "CArd Err"
      delay(5000); //Wait for 5 sec. to trigger watchdog reset (after 4 sec).
    }
    //Display calculations  
    eng_rpm = rpmCalc();
    car_speed = speedCalc();    
    
    phoneCall= digitalRead(PHONE_PIN);
    if(millis()<30000){
      phoneCall = 0;
    }
    if(callinProgress&&phoneCall){
      callTime = millis();
      callinProgress = 0;
    }
    if((millis()-alertTime)<PHONE_DELAY || ((millis()-4*pitTime)<PHONE_DELAY) || ((millis()-callTime)<PHONE_DELAY)){
      phoneCall = 0;
    }
    buttonPress = (!digitalRead(BUTTON_PIN));
    
    if(phoneCall){  //Phone is ringing but button hasn't been pressed -->time to pit
      pitAlert = 1;    //Alert driver it is time to pit
      alertTime = millis();
    }
        
    if(buttonPress){      //button is pressed during alert --> call to confirm and stop alert    
      pitCall = 1;
      pitAlert = 0;
      pitTime = millis();
      alertTime = millis();
    }
    if(pitCall){
      if((millis()-pitTime) > 4000){
        pitCall = 0;
      }
      makeCall(pitTime);
      displayClear();
      displayTop.writeDigitRaw(0,0b01110011);    //P
      displayTop.writeDigitRaw(1,0b00000110);    //I
      displayTop.writeDigitRaw(3,0b00000111);    //T
      displayBottom.writeDigitRaw(0,0b00111001);      //C
      displayBottom.writeDigitRaw(1,0b01110111);      //A
      displayBottom.writeDigitRaw(3,0b00111000);      //L
      displayBottom.writeDigitRaw(4,0b00111000);      //L
      digitalWrite(LED_POWER_PIN, HIGH);  
    }    
    else if(pitAlert){
      answerCall(alertTime);
      if((loop_cnt%6) < 2){
        displayClear();
        displayTop.writeDigitRaw(0,0b01110011);    //P
        displayTop.writeDigitRaw(1,0b00000110);    //I
        displayTop.writeDigitRaw(3,0b00000111);    //T
        displayBottom.writeDigitRaw(0,0b01101101);  //S
        displayBottom.writeDigitRaw(1,0b00000111);  //T
        displayBottom.writeDigitRaw(3,0b00111111);  //O
        displayBottom.writeDigitRaw(4,0b01110011);  //P
        digitalWrite(LED_POWER_PIN, HIGH);
      }
      else{
        displayClear();
        digitalWrite(LED_POWER_PIN, LOW);
      }
    }
    else{
      displayClear();
      displayBottom.print(((int)eng_rpm),DEC);
      displayTop.print(((int)car_speed),DEC);
      digitalWrite(LED_POWER_PIN, LOW);
    }
    
    displayTop.writeDisplay();
    displayBottom.writeDisplay();
    
    loop_cnt++;
  }
} 

float rpmCalc(void){
  if((micros() - eng_start_t) < eng_cycle_t){   
    eng_rpm = 60000000*(1.0/((float)eng_cycle_t));
  }
  else{
    eng_rpm = 60000000*(1.0/((float)(micros() - eng_start_t)));
    if(eng_rpm < 1000){
      eng_rpm = 0;
    }
  }
  return eng_rpm;
}

float speedCalc(void){
  if((micros() - out_start_t) < out_cycle_t){
    car_speed = MI_PER_ROT*3600000000*(1.0/((float)(out_cycle_t)));
  }
  else{
    car_speed = MI_PER_ROT*3600000000*(1.0/((float)(micros() - out_start_t)));
    if(car_speed < 2){
      car_speed =0;
    }
  }
  return car_speed;
}


ISR(INT0_vect){
  data_ready = 1;
}    

//These interrupts need to be rethought since micros() won't work in interrupts

ISR(INT1_vect){
  if((micros()-eng_start_t)>15000){
    eng_finish_t = eng_start_t;
    eng_start_t = micros();
    eng_cycle_t = eng_start_t - eng_finish_t;
    int_cnt++;
  } 
}

ISR(PCINT2_vect){
  //TODO: remember to update software debounce once location for hall sensor is determined.
  //TODO: remember to calibrate speed multipliers
  if((PIND&0x10)==0){
    if((micros()-out_start_t)>15000){
      out_finish_t = out_start_t;
      out_start_t = micros();
      out_cycle_t = out_start_t - out_finish_t;
      int_cnt++;
    }
  }  
}

void dateTime(uint16_t* date, uint16_t* time) {
   DateTime now = RTC.now();
      // return date using FAT_DATE macro to format fields
   *date = FAT_DATE(now.year(), now.month(), now.day());
  
   // return time using FAT_TIME macro to format fields
   *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

boolean regWrite(byte address, byte reg, byte instruction){
  boolean success = true;
  Wire.beginTransmission(address);
  if(Wire.write(reg)!=1){
     success = false;    
  }    
  if(Wire.write(instruction)!=1){
     success = false;
  }
  Wire.endTransmission();
  return success;  
}

boolean regRead(byte address, byte reg, byte numbytes){
  boolean success = true;
  Wire.beginTransmission(address);
  if(Wire.write(reg)!=1){
    success =false;  
  }
  Wire.endTransmission();
  if(Wire.requestFrom(address,numbytes)!= numbytes){
    success = false;
  }
  return success;
}

void displayClear(void){
  displayBottom.writeDigitRaw(0,0);
  displayBottom.writeDigitRaw(1,0);
  displayBottom.writeDigitRaw(2,0);
  displayBottom.writeDigitRaw(3,0);
  displayBottom.writeDigitRaw(4,0);
  displayTop.writeDigitRaw(0,0);
  displayTop.writeDigitRaw(1,0);
  displayTop.writeDigitRaw(2,0);
  displayTop.writeDigitRaw(3,0);
  displayTop.writeDigitRaw(4,0);      
}

void fileDisplay(unsigned char fileNum){
  wdt_reset();
  displayClear();
  displayTop.writeDigitRaw(0, 0b01110001);  //F
  displayTop.writeDigitRaw(1, 0b00000110);  //I
  displayTop.writeDigitRaw(3, 0b00111000);  //L
  displayTop.writeDigitRaw(4, 0b01111001);  //E
  displayBottom.print(fileNum, DEC);             //fileNum
  displayTop.writeDisplay();
  displayBottom.writeDisplay();
  delay(3500);
  wdt_reset();
}

void bajaDisplay(void){
  wdt_reset();
  displayClear();
  displayTop.writeDigitRaw(1, 0b00111101);  //G?
  displayTop.writeDigitRaw(3, 0b00111111);  //O
  displayBottom.writeDigitRaw(0, 0b01111100);    //B
  displayBottom.writeDigitRaw(1, 0b01110111);    //A
  displayBottom.writeDigitRaw(3, 0b00011110);    //J
  displayBottom.writeDigitRaw(4, 0b01110111);    //A  
  displayTop.writeDisplay();
  displayBottom.writeDisplay();
  delay(3000);
  wdt_reset();
}

void cardErrorDisplay(void){
  Serial.println("What");
  wdt_reset();
  displayClear();
  displayTop.writeDigitRaw(0,0b00111001);  //C
  displayTop.writeDigitRaw(1,0b01110111);  //A
  displayTop.writeDigitRaw(3,0b01010000);  //r
  displayTop.writeDigitRaw(4,0b01011110);  //d
  displayBottom.writeDigitRaw(0, 0b01111001);   //E
  displayBottom.writeDigitRaw(1, 0b01010000);   //r
  displayBottom.writeDigitRaw(3, 0b01010000);   //r
  displayTop.writeDisplay();
  displayBottom.writeDisplay();
  delay(3000);
  wdt_reset();
}

void phoneErrorDisplay(void){
  wdt_reset();
  displayClear();
  displayTop.writeDigitRaw(0,0b01110011);  //P
  displayTop.writeDigitRaw(1,0b01110110);  //H
  displayTop.writeDigitRaw(3,0b00111111);  //O
  displayTop.writeDigitRaw(4,0b01111001);  //E
  displayBottom.writeDigitRaw(0, 0b01111001);   //E
  displayBottom.writeDigitRaw(1, 0b01010000);   //r
  displayBottom.writeDigitRaw(3, 0b01010000);   //r
  displayTop.writeDisplay();
  displayBottom.writeDisplay();
  delay(3000);
  wdt_reset();
}


void makeCall(unsigned long pitTime){
  unsigned int t_sincePit = millis()-pitTime;
  callinProgress = 1;
  if(t_sincePit < 500){
    digitalWrite(END_PIN, HIGH);
  }
  if((t_sincePit > 500)&&(t_sincePit < 1000)){
    digitalWrite(END_PIN,LOW);
    digitalWrite(CALL_PIN,HIGH);
  }
  if((t_sincePit > 1000)&&(t_sincePit < 1500)){
    digitalWrite(CALL_PIN,LOW);
  }
  if((t_sincePit > 1500)&&(t_sincePit < 2000)){
    digitalWrite(CALL_PIN,HIGH);
  }
  if(t_sincePit > 2000){
    digitalWrite(CALL_PIN,LOW);
  }
}
  
void answerCall(unsigned long alertTime){
  unsigned int t_sinceCall  = millis() - alertTime;
  if(t_sinceCall < 500){
    digitalWrite(END_PIN, HIGH);
  }
  if((t_sinceCall > 500)&&(t_sinceCall < 1000)){
    digitalWrite(END_PIN,LOW);
  }
  if((t_sinceCall > 1000)&&(t_sinceCall < 1500)){
    digitalWrite(END_PIN, HIGH);
  }
  if((t_sinceCall > 1500)&&(t_sinceCall < 2000)){
    digitalWrite(END_PIN,LOW);
  }  
}  
    
void phoneOn(void){
  digitalWrite(CALL_PIN,HIGH); //Turn phone on
  delay(500);
  digitalWrite(CALL_PIN,LOW); //Turn phone on
  digitalWrite(END_PIN,HIGH); //Turn phone on
  delay(500);
  digitalWrite(END_PIN,LOW); //Turn phone on
}

void phoneCheck(void){
   digitalWrite(END_PIN,LOW);
   delay(500);
   digitalWrite(END_PIN,HIGH);
   delay(500);
   digitalWrite(END_PIN,LOW);
   if(!digitalRead(PHONE_PIN)){
     phoneErrorDisplay();    
   }
}
