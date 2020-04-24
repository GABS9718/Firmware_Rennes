// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#define height 320
#define width 240
#define ch_width 6
#define ch_height 7

#include "RTClib.h"

RTC_DS1307 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
uint8_t minutes = 0;
uint8_t hours = 0;
uint8_t days = 0;
uint8_t months = 0;
uint16_t years = 0;

boolean RTC_Init(){
  rtc.begin();
  uint8_t number_of_ch = 7; // Error x
  uint8_t t_size = 1;
  uint8_t word_width = ch_width*number_of_ch*t_size;
  
  uint8_t x0 = (width-word_width)/2;
  uint16_t y0 = height - (3*(ch_height+2));
  
  if (!rtc.begin()){
    //tft.setCursor(x0,y0);
    //tft.print("Error 1");  
    return 0;  
  }
  if (! rtc.isrunning()) {
    //tft.setCursor(x0,y0);
    //tft.print("Error 2");
    return 0;
  }
    
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  return 1;
}

void RTC_Update(){ 
  DateTime now = rtc.now();
  unsigned long int agora;
  agora = now;
  Serial.print("O valor de agora é "); Serial.println(agora);
  days = now.day();
  months = now.month();
  years = now.year();
  
  minutes = now.minute();
  hours = now.hour();
  if (days<10){
    Serial.print("0");
  } Serial.print(days); Serial.print("/"); 
  if (months<10){
    Serial.print("0");
  } Serial.print(months); Serial.print("/");
  Serial.println(years);

  if (hours<10){
    Serial.print("0");
  } Serial.print(hours); Serial.print(":");
  if (minutes<10){
    Serial.print("0");
  }Serial.print(minutes); 
}

void setup () {
  RTC_Init();
  Serial.begin(9600);
}

void loop () {
  RTC_Update();
  delay(3000);
}
