#include <SPI.h>
#include "SdFat.h"
#include <DS3231.h>
#include "PCD8544_SPI.h"

#define LCD 1

#define NONE 0b00011100
#define S1   0b00011000
#define S12  0b00010000
#define S123 0b00000000
#define S32  0b00000100
#define S3   0b00001100
#define CALI 0b10000000   

#define DELTA_L 0.2

enum States
{
  INIT,
  START_DRIVING_RIGHT,
  START_DRIVING_LEFT,
  END_DRIVING_RIGHT,
  END_DRIVING_LEFT,
  PRINT  
};

// Sd card reader pins (soft-spi)
const uint8_t SOFT_MISO_PIN = A0;
const uint8_t SOFT_MOSI_PIN = A1;
const uint8_t SOFT_SCK_PIN  = A2;
const uint8_t SD_CHIP_SELECT_PIN = A3;
SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> sd;
SdFile file;

PCD8544_SPI lcd;
DS3231 rtc(5,6);  //Data, Clock

unsigned long t = 0;
unsigned long t_start = 0;
unsigned long t_mid = 0;
unsigned long t_end = 0;

int state = 0;
byte sensor_old = NONE;
byte sensor_new = NONE;
double v_max = 0.0;
double v_mean = 0.0;
int car_cnt = 0;


void setup() {
  // sd init
  sd.begin(SD_CHIP_SELECT_PIN);
  
  // rtc init
  rtc.begin();

  // display init
  #if LCD
    lcd.begin();
  #else
    Serial.begin(9600); 
  #endif
  DDRD = DDRD | B00000000; // digital pin 2,3,4 as input

  // sensor test with pin 7 low
  while(!(PIND & CALI))
  {
     #if LCD
        lcd.clear();
        lcd.gotoXY(0,0);
        lcd.print((bool)(PIND & 0b00000100)); 
        lcd.print((bool)(PIND & 0b00001000)); 
        lcd.print((bool)(PIND & 0b00010000)); 
      #else
        Serial.print((bool)(PIND & 0b00000100)); 
        Serial.print((bool)(PIND & 0b00001000)); 
        Serial.println((bool)(PIND & 0b00010000)); 
      #endif 
      delay(100);
            
  }
  lcd.clear();
}

void loop() {
  
  if( (PIND & NONE) != sensor_new ) // if some sensor state has changed
  {
    t = micros(); // take the time
    sensor_old = sensor_new;
    sensor_new = (PIND & NONE);
  }
  
  switch(state)
  {
  case INIT:
  {
    if( sensor_new == S1 && sensor_old == NONE )
    {
      state = START_DRIVING_RIGHT;
      t_start = t;
    }
    if( sensor_new == S3 && sensor_old == NONE )
    {
      state = START_DRIVING_LEFT;
      t_start = t;
    }
    break;
  }
  case START_DRIVING_RIGHT:
  {
    if(sensor_old == S1 || sensor_new == S1)
    {
      if(sensor_new == S12 && sensor_old == S1)
      {
        state = END_DRIVING_RIGHT;
        t_mid = t;
      }
    }
    else
    {
      state = INIT; // an error happened, start again
    }
    break;
  }
  case START_DRIVING_LEFT:
  {
    if(sensor_old == S3 || sensor_new == S3)
    {
      if(sensor_new == S32 && sensor_old == S3)
      {
        state = END_DRIVING_LEFT;
        t_mid = t;
      }
    }
    else
    {
      state = INIT; // an error happened, start again
    }
    break;
  }
  case END_DRIVING_RIGHT:
  {
    if(sensor_old == S12 || sensor_new == S12)
    {
      if(sensor_new == S123 && sensor_old == S12)
      {
        state = PRINT;
        t_end = t;
      }
    }
    else
    {
      state = INIT; // an error happened, start again
    }
    break;
  }
  case END_DRIVING_LEFT:
  {
    if(sensor_old == S32 || sensor_new == S32)
    {
      if(sensor_new == S123 && sensor_old == S32)
      {
        state = PRINT;
        t_end = t;
      }
    }
    else
    {
      state = INIT; // an error happened, start again
    }
    break;
  }
  case PRINT:
  {
    state = INIT;

    // calculate velocity
    double v_start_mid = DELTA_L * 3.6 / ( 1e-6 * (t_mid-t_start) );       // km/h
    double v_end_mid = DELTA_L * 3.6 / ( 1e-6 * (t_end-t_mid) );         // km/h
    double v_start_end = 2.0 * DELTA_L * 3.6 / ( 1e-6 * (t_end-t_start) );   // km/h

    // check if velocity is plausible   
    if( v_start_end > 1.0   &&
        v_start_end < 150.0 && 
        abs(v_start_mid - v_end_mid) < 3.0  && 
        abs(v_end_mid - v_start_end) < 3.0  && 
        abs(v_start_mid - v_start_end) < 3.0 )
    {    
      car_cnt += 1;
      
      if(v_start_end > v_max)
      {
        v_max = v_start_end;
      }
  
      v_mean = (v_mean * (car_cnt - 1) + v_start_end) / car_cnt;

      // write to sd card
      file.open("SpeedData.txt", O_RDWR | O_CREAT | O_APPEND);
      file.print("time: ");
      file.print(rtc.getTimeStr());
      file.print("; car_cnt: ");
      file.print(car_cnt);
      file.print("; vel: ");
      file.println(v_start_end);
      file.close();

      // write to display
      #if LCD
        lcd.clear();
        lcd.gotoXY(0,0);
        lcd.print("car_cnt = ");
        lcd.print(car_cnt); 
        lcd.gotoXY(0,1);
        lcd.print("v = ");
        lcd.print(v_start_end); 
        lcd.gotoXY(0,2);
        lcd.print("vme = ");
        lcd.print(v_mean); 
        lcd.gotoXY(0,3);
        lcd.print("vma = ");
        lcd.print(v_max); 
      #else
        Serial.print("car_cnt = ");
        Serial.print(car_cnt);
        Serial.print(" vel = ");
        Serial.print(v_start_end);
        Serial.print(" vel_mean = ");
        Serial.print(v_mean);
        Serial.print(" vel_max = ");
        Serial.print(v_max);
        Serial.println(" km/h");
      #endif  

    }
        
    break;
  }
  }

}
