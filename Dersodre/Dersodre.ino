//Include's section
#include <SPI.h>
#include <SD.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>      // Core graphics library
#include <TouchScreen.h>
#include <string.h>
#include "RTClib.h"
//#include <math.h>
//#include <Adafruit_TFTLCD.h> // Hardware-specific library

//===========================================================
//Define's section
#define tft_pin_CS 10 //tft means Thin Film Transistor, it's type of LCD. And the CS is the pin ChipSelect for the SPI protocol.
#define tft_pin_DC 9
#define tft_pin_rst -1 //definied as -1 when it is connector to arduino's reset pin

#define width 240 //Number of pixels in the width
#define height 320 //Number of pixels in the height
#define ch_width 6 //Number of width pixels for each character
#define ch_height 7//number of hight pixels for each character
#define sym_arrow_width 48 //The width of the arrow's symbol
#define square_settings_width 68
#define square_settings2_width 55

#define color_1 ORANGE //Primary color
#define color_2 BLACK //Secondary color
#define bg_color LIGHTGREY //background color
#define bg_color_sym_1 DARKGREY //symbol's background color type 1
#define bg_color_sym_2 LIGHTGREY //symbol's background color type 2
#define dont_measure_color color_1 //color to mean that the sensor won't be measured
#define ybac_color BLUE //color to mean that there is bacteria (Y)es (Bac)teria
#define nbac_color GREEN //color to mean that there is not bacteria (N)o (Bac)teria

#define y0_menu 34 //the initial position to start the word 'MENU' on page 0
#define touch_delay 100 //Time delay to wait until the next read of the touchsensor

#define num_columns 25 //=num_buttons of the matrix M_Button
#define num_lines 4 //Number of points needed to create a button

#define YP A2 //Pin (Y)(P)lus of the touchscreen sensor
#define XM A3 //Pin (X)(M)inus of the touchscreen sensor
#define YM 44 //Pin (Y)(M)inus of the touchscreen sensor
#define XP 46 //Pin (X)(P)lus of the touchscreen sensor
#define resistance_XM_XP 301 //resistence measured with a multimeter between the pins Xm and Xp to make it more accurate 

#define TS_MINX 110 //The minimum touchscreen point in the X axis measured by the example code 'touchscreendemo'
#define TS_MINY 70 //The minimum touchscreen point in the Y axis measured by the example code 'touchscreendemo'
#define TS_MAXX 925 //The maximum touchscreen point in the X axis measured by the example code 'touchscreendemo' 
#define TS_MAXY 913 //The maximmum touchscreen point in the Y axis measured by the example code 'touchscreendemo'

#define SHORT 0 //Integration Time as being "SHORT" //apagar isso aqui depois
#define LONG 1 //Integration Time as being "LONG" //apagar isso aqui depois

#define voltage_Level_PinA_MCP 5.000 //This is the voltage level applied to pin A in the MCP. 
#define voltage_Level_PinB_MCP 0 //This is the voltage level applied to pin B in the MCP. 
#define voltage_wiper_pos_0 0.0128 //This is the voltage read by the multimeter when the potentiometer is in the position 0, it shohuld be equal to the ground voltage, as it is connected directly to the GND (theoretically)
#define MCP_pin_CS 8 //Pin CS for the MCP41010
#define MCP_Resolution 256 //The number of steps that the wiper of the potentiometer have.
#define MPC_DP_instr B00010001 //Instruction sent to the MPC potentiometer
#define delay_bt_next_wiper_increase 5000 //This is the time delay between the next wiper increase
//======================================================================
//Global variables
uint8_t thickness_rect_notification = 30; //the thickness of the notification bar, it is the point in Y axis

uint8_t bat_level = 50; //This variable will store the percentage of the battery
uint8_t temp = 10; //This variable will store the value measured by the temperature sensor
uint8_t humidity = 36; //This variable will store the value measured by the humidity sensor
float current = 0;
float voltage_Out = 0;

int8_t button_clicked = -1; //To know the number of which button was clicked
int8_t current_page = -1; //To know which page the user is in the current moment
uint8_t which_sensor = 1; //This means that the results after clicking in ''see results'' is the first sensor in ascending order
int8_t old_page = -1; //Logic not made yet

float final_Voltage_Value = voltage_Level_PinA_MCP; //This is the default value to maximum voltage level of the output signal in the settings configuration.
float initial_Voltage_Value = 0; //This is the default value to minimum voltage level of the ouput signal in the settings configuration
float signal_Step = 0.200; //This is the default value for the signal step of the output signal level
float integration_Time = 1; //IDK in what this will act

uint8_t number_of_decimal_places_x = 1; //select how much algarisms will have after the point
int8_t number_of_decimal_places_y = 1;
float scale_initial_value_x = initial_Voltage_Value;
float scale_initial_value_y = 0;
float scale_step_x = signal_Step;
float scale_step_y = 1;
float scale_final_value_x = final_Voltage_Value;
float scale_final_value_y = 9;
char *dotted_lines_x = "YES";
char *dotted_lines_y = "NO";
boolean preview = 0; //0 is preview closed. 1 is preview open.
boolean size_error_x = 0;
boolean start_already_created=false;

uint8_t minutes = 0;
uint8_t minutes_now = 0;
uint8_t hours = 0;
//=======================================================================
//Matrix and global vectors
/*
   P00 _________________ P10   P01  _____________________ P11         Matrix of the buttons: M[lines][columns] ->Point line 0 and column 0 = P00
      |                 |          |                    |                               P00 P01 ... P0n
      |    Button 0     |          |      Button 1      |                               P10 P11 ... P1n
      |                 |          |                    |                               P20 P21 ... P2n
   P20|_________________| P30  P21 |____________________| P31                           P30 P31 ... P3n
                                                                      Lines = [P0-P3] e Columns = Number of buttons
*/

uint16_t M_Button[num_lines][num_columns]; //Global matrix to store the points needed to create a button. It is like the figures aboce
int8_t sensor_counter[15] = { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //This vector is to know the type of measured to be made in each sensor
// If it is 0, the sensor will not be measured. 1 the measurement is without bacteria, 2 is with bacteria.
// Example: sensor_counter={-1, 0, 0 , 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2}. Sensor 1-5 -> don't measure, Sensor 6-9 there is no bacteria, Sensor 1-14 there is bacteria
float vector_voltage_out[6] = {0, 1.0, 2.0, 3.0, 4.0, 5};

Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_pin_CS, tft_pin_DC); //Class to use the lcd
TouchScreen ts = TouchScreen(XP, YP, XM, YM, resistance_XM_XP); //Class to activate the touchscreen sensor
RTC_DS1307 rtc; char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

/* NOTEs:
   -> The initial reference that will be used for everything refers to the points in the right left (x0, y0);
   -> The word symbols refers to: the left arrow, the right arrow, the battery, the temperature and the humidity. Each one of them is a different symbol = sym.
   -> Every button will be with a edge that have the color_1. If there is not a edge in the word or number, it is just a text.

*/

void init_All() { //Initializates all necessary functions
  tft_Init();
  DP_MCP_Init();
  RTC_Init();
}
//=========================================================================================================
/*
  // O QUE FALTA NESSA FUNCAO:
    0. REFAZER A LÓGICA DESSA FUNÇÃO DE ACORDO COM A NOVA FUNÇÃO DE GERAR O SINAL DE SAÍDA QUE FOI REPROGRAMADA
    1. FAZER A FUNÇÃO DE QUANDO O SINAL EH DECRESCENTE
    2. FAZER A FUNÇÃO COM O MUX PRA SELECIONAR O PINO DE CADA SENSOR
    3. FAZER A FUNCAO PARA SALVAR OS DADOS NO SD DE ACORDO COM OS PARAMETROS QUE SAO PASSADOS (TEXTO/VARIAVEL) IGUAL SERIAL.PRINT
    4. FAZER A FUNCAO DE LER A CORRENTE
    5. FAZER A FUNCAO DE LER A TEMPERATURA
    6. FAZER A FUNCAO DE LER A HUMIDADE
    7. DECIDIR SE VAI LER O VALOR PRATICO DA TENSAO EM ALGUM PINO

  void brain(){ //FAZER A LÓGICA DEPOIS SE FOR SINAL DECRESCENTE. MAIS FÁCIL FAZER TUDO CERTO COM SINAL CRESCENTE E DEPOIS USAR O SEGUNDO IF DA FUNÇÃO GENERATE SIGNAL OUTPUT QUE JÁ ESTA PRONTA
  uint8_t sensor_number=1;
  uint8_t sensor_to_be_measured = 0; //This is the number of the sensor that is been measured in the moment
  uint8_t measurement_type=0; //If 1 = no Bacteria and if it is 2 = yes Bacteria
  float deltaV_Possible = voltage_Level_PinA_MCP - voltage_wiper_pos_0; //This is the range of the deltaV possible to be used, as these value are hardware connections.
  float min_Output_Signal_Step_Possible = deltaV_Possible/(float)MCP_Resolution; //This is the minimal value for the signal step output
  uint16_t wiper_pos_initial = initial_Voltage_Value/min_Output_Signal_Step_Possible;
  uint16_t wiper_pos = wiper_pos_initial;
  float wiper_Step = signal_Step/min_Output_Signal_Step_Possible;

  save_Data_To_SD("Initial Voltage Value, Final Voltage Value, Signal Step, Integration Time");
  save_Data_To_SD(initial_Voltage_Value, final_Voltage_Value, signal_Step, integration_Time);

  for (uint8_t counter=1; counter <= number_of_measurements(); counter++){
    sensor_to_be_measured = which_sensor_to_be_measured(sensor_number); //discovery which is the number of the first sensor to be measured
    MUX_selection(sensor_to_be_measured); //Activate the signal in the pin from the number of sensor to be measured
    measurement_type = sensor_counter[sensor_to_be_measured];

    save_Data_To_SD("Sensor Number, Measurement Type"); //save_Data_To_SD
    save_Data_To_SD(sensor_to_be_measured, measurement_type); //save_Data_To_SD
    save_Data_To_SD("wiper_pos, Voltage (V), Current (mA), Temperature (oC), Humidity (/100)"); //save_Data_To_SD

    do {
      select_MCP_Wiper_Position(wiper_pos);
      delay(delay_bt_next_wiper_increase); //não tenho certeza se o melhor lugar do delay é aqui. Tem que dar tempo de mudar a tensão na saída do potenciometro e ler o valor de corrente causado por ela

      voltage_Out = ((float)wiper_pos/(float)MCP_Resolution)*deltaV_Possible; //read_Voltage(); ler o valor da tensão aplicada em um pino ou não
      current = read_Current();
      temp = read_Temp();
      humidity = read_Humidity();
      save_Data_To_SD(wiper_pos, voltage, current, temp, humidity); //save_Data_To_SD

      wiper_pos = wiper_pos +(uint16_t)wiper_Step;
      if (wiper_pos > (final_Voltage_Value/min_Output_Signal_Step_Possible)){
        wiper_pos = final_Voltage_Value/min_Output_Signal_Step_Possible;
        select_MCP_Wiper_Position(wiper_pos);
        delay(delay_bt_next_wiper_increase); //não tenho certeza se o melhor lugar do delay é aqui. Tem que dar tempo de mudar a tensão na saída do potenciometro e ler o valor de corrente causado por ela

        voltage_Out = ((float)wiper_pos/(float)MCP_Resolution)*deltaV_Possible; //read_Voltage(); ler o valor da tensão aplicada de novo ou não?
        current = read_Current();
        temp = read_Temp();
        humidity = read_Humidity();
        save_Data_To_SD(wiper_pos, voltage, current, temp, humidity); //save_Data_To_SD
        break;
      }
    } while (wiper_pos < (final_Voltage_Value/min_Output_Signal_Step_Possible));

    select_MCP_Wiper_Position(0); //put the wiper in the position 0 to start the signal from the begining
    sensor_number++; //Go to next sensor to be measured
    wiper_pos = wiper_pos_initial; //start applying the same signal
  }
  progress_bar(100); //Can show the results
  }

*/
//========================================================================================================= ABOVE THE BRAIN FUNCTION
boolean RTC_Init() {
  rtc.begin();
  uint8_t number_of_ch = 7; // Error x
  uint8_t t_size = 1;
  uint8_t word_width = ch_width * number_of_ch * t_size;

  uint8_t x0 = (width - word_width) / 2;
  uint16_t y0 = height - (3 * (ch_height + 2));

  if (!rtc.begin()) {
    tft.setCursor(x0,y0);
    tft.print("Error 1");
    return 0;
  }
  if (! rtc.isrunning()) {
    tft.setCursor(x0,y0);
    tft.print("Error 2");
    return 0;
  }
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //This line should used one time to adjust the time. After adjusting it, it must be commented and the code compiled again.
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  return 1;
}

uint8_t *print_Time(uint16_t x0, uint16_t y0, boolean with_seconds){
  uint8_t vector_size = 3;
  uint8_t *vec_Values_Time = calloc(vector_size, sizeof(uint8_t));
  DateTime now = rtc.now();  
  tft.setCursor(x0, y0);
  if (now.hour()<10){
    tft.print("0");
  } tft.print(now.hour()); tft.print(':');
  vec_Values_Time[0] = now.hour(); 
  if (now.minute()<10){
    tft.print("0");
  }tft.print(now.minute());
  vec_Values_Time[1] = now.minute();
  if (with_seconds == true){
    if (now.second()<10){
      tft.print("0");
    }tft.print(now.second());
    vec_Values_Time[2] = now.second();
  }
  return vec_Values_Time;
}

uint16_t *print_Date(uint16_t x0, uint16_t y0){
  uint8_t vector_size = 3;
  uint16_t *vec_Values_Date = calloc(vector_size, sizeof(uint16_t));
  DateTime now = rtc.now();
  tft.setCursor(x0, y0);
  if (now.day()<10){
    tft.print("0");
  } tft.print(now.day()); tft.print("/"); 
  vec_Values_Date[0] = now.day();
  if (now.month()<10){
    tft.print("0");
  }tft.print(now.month()); tft.print("/");
  vec_Values_Date[1] = now.month();
  tft.print(now.year());
  vec_Values_Date[2] = now.year();
  return vec_Values_Date;
}

uint8_t print_DayWeek(uint16_t x0, uint16_t y0){
  DateTime now = rtc.now();
  tft.setCursor(x0, y0);
  tft.print(daysOfTheWeek[now.dayOfTheWeek()]);
  return now.dayOfTheWeek();
}

uint16_t *calculate_coordinates_RTC_Data(uint8_t pos_Time, uint8_t pos_Date, uint8_t pos_DayWeek) { //if 0, don't print
  boolean RTC_status = RTC_Init();
  uint8_t vector_size = ((pos_Time/pos_Time)*2)+((pos_Date/pos_Date)*2)+((pos_DayWeek/pos_DayWeek)*2); //discover if some one is 0, then doestn need to export the x0 and y0
  uint16_t *vec_coordinates_Time_Date_DayWeek = calloc(vector_size, sizeof(uint16_t));
  
  if (RTC_status == 1) { //It's working, so print. Else, do nothing.
    DateTime now = rtc.now();
    uint8_t t_size = 1;
    tft.setTextSize(t_size);
    tft.setTextColor(color_2);
    uint8_t number_of_ch = 4; // hh:mm
    uint8_t word_width = ch_width * number_of_ch * t_size;

    uint8_t x0 = (width - word_width) / 2;
    uint16_t y0 = height - (pos_Time * (ch_height + 2));
    vec_coordinates_Time_Date_DayWeek[0] = x0; //TIME
    vec_coordinates_Time_Date_DayWeek[1] = y0; //TIME
    
    number_of_ch = 10; //hh:mm
    word_width = ch_width * number_of_ch * t_size;
    x0 = (width - word_width)/2;
    y0 = height - (pos_Date *(ch_height+2));
    vec_coordinates_Time_Date_DayWeek[2] = x0; //DATE
    vec_coordinates_Time_Date_DayWeek[3] = y0; //DATE
    
    number_of_ch = strlen(daysOfTheWeek[now.dayOfTheWeek()]);
    word_width = ch_width * number_of_ch * t_size;
    x0 = (width - word_width)/2;
    y0 = height - (pos_DayWeek *(ch_height+2));
    vec_coordinates_Time_Date_DayWeek[4] = x0; //DayWeek
    vec_coordinates_Time_Date_DayWeek[5] = y0; //DayWeek
        
    //free(vec_coordinates_Time_Date_DayWeek); //check where this should be
    return vec_coordinates_Time_Date_DayWeek;
  }
}

void update_RTC_Time_Menu(boolean with_seconds){ //think better in this
  DateTime now = rtc.now();
  uint16_t *coordinates_Info_RTC = calculate_coordinates_RTC_Data(3, 2, 1);
  uint8_t x0_Time = coordinates_Info_RTC[0]; uint16_t y0_Time = coordinates_Info_RTC[1];
  uint8_t first_algarism_minutes_old = (minutes%10);
  uint8_t second_algarism_minutes_old = (minutes/10);
  uint8_t first_algarism_hour_old = (hours%10);
  uint8_t second_algarism_hour_old = (hours/10); 
  uint8_t hours_now = now.hour();
  uint8_t first_algarism_minutes_now = (minutes_now%10);
  uint8_t second_algarism_minutes_now = (minutes_now/10);
  uint8_t first_algarism_hour_now = (hours_now%10);
  uint8_t second_algarism_hour_now = (hours_now/10);
  uint8_t number_of_algarisms_differents = 0;
  if (minutes!= minutes_now){
    //19:59 -> 20:00
    if ((first_algarism_minutes_old!=first_algarism_minutes_now)&&(second_algarism_minutes_old!=second_algarism_minutes_now)&&(first_algarism_hour_old!=first_algarism_hour_now)&&( second_algarism_hour_old!=second_algarism_hour_now )){
      number_of_algarisms_differents = 4;
      tft.fillRect(x0_Time, y0_Time, 2*ch_width, ch_height, bg_color);
      tft.fillRect(x0_Time+3*ch_width, y0_Time, 2*ch_width, ch_height, bg_color);
      print_Time(x0_Time, y0_Time, with_seconds);
    }
    //13:59 -> 14:00
    if ((first_algarism_minutes_old!=first_algarism_minutes_now)&&(second_algarism_minutes_old!=second_algarism_minutes_now)&&(first_algarism_hour_old!=first_algarism_hour_now)){
      number_of_algarisms_differents = 3;
      tft.fillRect(x0_Time+1*ch_width, y0_Time, 1*ch_width, ch_height, bg_color);
      tft.fillRect(x0_Time+3*ch_width, y0_Time, 2*ch_width, ch_height, bg_color);   
      print_Time(x0_Time, y0_Time, with_seconds); 
    }
    //13:49 -> 13:50 
    if ((first_algarism_minutes_old!=first_algarism_minutes_now)&&(second_algarism_minutes_old!=second_algarism_minutes_now)){
      number_of_algarisms_differents = 2;
      tft.fillRect(x0_Time+3*ch_width, y0_Time, 2*ch_width, ch_height, bg_color);
      print_Time(x0_Time, y0_Time, with_seconds);
    }
    //13:40 -> 13:41
    if (first_algarism_minutes_old!=first_algarism_minutes_now){
      number_of_algarisms_differents = 1;
      tft.fillRect(x0_Time+4*ch_width, y0_Time, ch_width, ch_height, bg_color);
      print_Time(x0_Time, y0_Time, with_seconds);
    }
    minutes = minutes_now;
  }

  
}
//========================================================================================================= ABOVE RTC
void DP_MCP_Init() {
  pinMode (MCP_pin_CS, OUTPUT);  //Defining the pin CD of the MPC41010 as a digital output
  select_MCP_Wiper_Position(0); //Put in this position always when the MCP is initializate
}

void select_MCP_Wiper_Position(uint16_t wiper_position) {
  //NEVER FORGET TO TURN OFF THE OTHERS SPI SLAVES AND JUST LET THE CHANEL TURN ON OF THAT ONE THAT YOU WILL BE TALK
  digitalWrite(tft_pin_CS, HIGH); //Turn off the SPI communiation with the channel from the tft
  digitalWrite(MCP_pin_CS, LOW);  //Turn on the SPI communication with the channel from the MCP

  SPI.transfer(MPC_DP_instr);
  SPI.transfer(wiper_position);

  digitalWrite(MCP_pin_CS, HIGH); //Turn off the SPI communication with the channel from the MCP
  digitalWrite(tft_pin_CS, LOW); //Turn on the SPI communication with the channel from the tft
}

float *create_Voltage_Vector() { //This will create the voltage vector of the signal output, as it uses the same code but whitout the delay and the serial prints
  uint16_t vector_size = number_of_points(initial_Voltage_Value, signal_Step, final_Voltage_Value);
  uint16_t pos_vect_Voltage_Out = 0;
  float *vect_Voltage_Out = calloc(vector_size, sizeof(float));

  float min_Output_Signal_Step_Possible = (voltage_Level_PinA_MCP / (float)MCP_Resolution); //This is the minimal value for the signal step output
  uint16_t wiper_pos = 0;
  voltage_Out = initial_Voltage_Value; //my first voltage out is the initial voltage

  if (initial_Voltage_Value < final_Voltage_Value) { //growing signal
    do {
      vect_Voltage_Out[pos_vect_Voltage_Out] = voltage_Out;
      pos_vect_Voltage_Out++;

      voltage_Out = voltage_Out + signal_Step;
      if (voltage_Out > final_Voltage_Value) {
        voltage_Out = final_Voltage_Value;
        vect_Voltage_Out[vector_size - 1] = voltage_Out;
        break;
      }
    } while (voltage_Out <= final_Voltage_Value);

    free(vect_Voltage_Out); //Check if this should be kept here
    return vect_Voltage_Out;
  }
}


void generate_Output_Signal() {
  Serial.print("As configuracoes do sinal de saída são: (");
  Serial.print(initial_Voltage_Value, 10); Serial.print(", ");
  Serial.print(signal_Step, 10); Serial.print(", ");
  Serial.print(final_Voltage_Value, 10); Serial.println(")");

  float min_Output_Signal_Step_Possible = (voltage_Level_PinA_MCP / (float)MCP_Resolution); //This is the minimal value for the signal step output
  uint16_t wiper_pos = 0;
  voltage_Out = initial_Voltage_Value; //my first voltage out is the initial voltage
  Serial.print("O minimo do step do sinal é "); Serial.println(min_Output_Signal_Step_Possible, 5);
  if (initial_Voltage_Value < final_Voltage_Value) { //growing signal
    do {
      wiper_pos = voltage_Out / min_Output_Signal_Step_Possible;
      Serial.print("A posicao crescente do wiper eh ");
      Serial.println(wiper_pos);

      select_MCP_Wiper_Position(wiper_pos);
      Serial.print("O valor da tensao de saida é ");
      Serial.println(voltage_Out, 5);
      //delay(delay_bt_next_wiper_increase);
      voltage_Out = voltage_Out + signal_Step;
      if (voltage_Out > final_Voltage_Value) {
        voltage_Out = final_Voltage_Value;
        wiper_pos = (voltage_Out / min_Output_Signal_Step_Possible) - 1;
        Serial.print("A ULTLIMA posicao do wiper eh ");
        Serial.println(wiper_pos);
        select_MCP_Wiper_Position(wiper_pos);
        Serial.print("O valor da tensao de saida é ");
        Serial.println(voltage_Out, 5);
        //delay(delay_bt_next_wiper_increase);
        break;
      }
    } while (voltage_Out <= final_Voltage_Value);
  } else if (initial_Voltage_Value > final_Voltage_Value) { //degrowing signal
    do {
      wiper_pos = voltage_Out / min_Output_Signal_Step_Possible;
      Serial.print("A posicao decrescente do wiper eh ");
      Serial.println(wiper_pos);
      select_MCP_Wiper_Position(wiper_pos);
      Serial.print("O valor da tensao de saida é ");
      Serial.println(voltage_Out, 5);
      delay(delay_bt_next_wiper_increase);
      voltage_Out = voltage_Out - signal_Step;
      if (voltage_Out < final_Voltage_Value) {
        voltage_Out = final_Voltage_Value;
        wiper_pos = (voltage_Out / min_Output_Signal_Step_Possible) - 1;
        Serial.print("A ULTLIMA posicao decrescente do wiper eh ");
        Serial.println(wiper_pos);
        select_MCP_Wiper_Position(wiper_pos);
        Serial.print("O valor da tensao de saida é ");
        Serial.println(voltage_Out, 5);
        delay(delay_bt_next_wiper_increase);
        break;
      }
    } while (voltage_Out >= final_Voltage_Value);
  }
  select_MCP_Wiper_Position(0); //After generating the output signal must put it on 0V = wiper0
  progress_bar(100);
}

//=============================================================================== ABOVE JUST DIGITAL POTENTIOMETER
void tft_Init() {
  tft.begin(); //Need to initializate the screen
  tft.setRotation(2); //Put the point (0,0) in the right left (rf) ^<.
  tft.fillScreen(bg_color); //Fill all the screen with the background color
  tft.setTextWrap(1); //This fucntion keeps the next in the lines below with it is bigger than the width
  //read_sensors(); //This function will read the values of the sensors (battery, temperature and humidity) as soon as the device is turn on
}

/*Action: create the layout of the notification bar
  Parameters:
            x0 -> the initial point x0 of  width where the line should begin
            y0 -> the initial point y0 of the right left height where the line should begin
            thickness -> the thickness/height of the bar
            color -> the color of the line
            line_thickness -> the thickness of the line that separetes the bar
  Return: nothing
*/

void draw_VLine(uint8_t x0, uint8_t y0, uint8_t thickness, uint8_t line_thickness, uint16_t color) {
  uint8_t counter = 0;
  for (counter = 1; counter <= line_thickness; counter++) { //Improve the thickness of the vertical line by incremeting the pixels in the vertical
    tft.drawFastHLine(x0, y0 + thickness + counter, width, color);
  }
}

/*Action: draw just the background rectangles that will stand behind each symbols
  Parameters:
            x0 -> the point of the right left width where the rectangles begins
            y0 -> the point of the right left height where the rectangles begins
            r_height -> the rectangle height
            symbol_number -> how many symbol's there will be
  Return: nothing
*/
void draw_Rect_Bg_Not_Bar (uint8_t x0, uint8_t y0, uint8_t r_height, uint8_t symbol_number, boolean first_rect_button, boolean last_rect_button) {
  uint8_t r_width = width / symbol_number;
  uint8_t columns = 0;
  for (uint8_t counter = 1; counter <= symbol_number; counter++) {
    if (counter % 2 == 0) { //When the rectangle's number is pair, colors of the background color symbol 1 color
      tft.fillRect(x0, y0, r_width, r_height, bg_color_sym_2);
    }
    if (counter % 2 != 0) { //When the rectangle's number is odd, colors of another color (background color symbols 2)
      tft.fillRect(x0, y0, r_width, r_height, bg_color_sym_1);
    }
    if (symbol_number == 4 || symbol_number == 5) { //we have just to store in the matrix when the symbol number is 4 or 5. When it is 3, we dont have buttons in the notification bar.
      if (first_rect_button == true && counter == 1) { //save the first rectangle as the button 0
        columns=0;
        M_Button[0][columns] = x0;
        M_Button[1][columns] = x0 + r_width;
        M_Button[2][columns] = y0;
        M_Button[3][columns] = y0 + r_height;
      }
      if (last_rect_button == true && counter == symbol_number) { //save the last rect as the button 0 or as the button 1
        columns=1; 
        M_Button[0][columns] = x0;
        M_Button[1][columns] = x0 + r_width;
        M_Button[2][columns] = y0;
        M_Button[3][columns] = y0 + r_height; 
      }

    }
    x0 = x0 + r_width;
  }
  tft.drawRect(0, 0, width, height, color_1); //Every time that the notification bar is created, create the edge in all screen.
}

/*Action: draw the left arrow
  Parameters:
            symbol_pos -> what vertical position the symbol will be
            arrow_width -> the total width of the arrow, including the triangle and the rectangle width
            string -> what will be writen in the arrow
            symbol_number -> how many symbols will be in the = how many vertical divisions
  Return: nothing
*/

uint8_t draw_Left_Arrow(uint8_t symbol_pos, uint8_t arrow_width, char *what_to_write, uint8_t symbol_number) {
  if (symbol_pos == 0) {
    return 0;
  }
  uint8_t bg_r_width = width / symbol_number;
  uint8_t x0 = 1 + (bg_r_width * (symbol_pos - 1)) + ((bg_r_width - arrow_width) / 2); //(1)+(begin of rectangle)+(centralize)centralize in the x coordanate
  uint8_t y0 = thickness_rect_notification / 2;

  uint8_t triangle_height = thickness_rect_notification - 2;
  uint8_t triangle_width = thickness_rect_notification - (0.5 * thickness_rect_notification);
  uint8_t x1 = x0 + triangle_width;
  uint8_t y1 = y0 + (triangle_height / 2);
  uint8_t x2 = x1;
  uint8_t y2 = y0 - (triangle_height / 2);
  uint8_t r_x0 = x1;
  uint8_t r_y0 = y0 - (triangle_height / 4);
  uint8_t r_width = arrow_width - triangle_width; //r_width != bg_r_width. The first is just the rectangle of the arrow
  uint8_t r_height = triangle_height / 2;
  uint8_t number_of_caracters = strlen(what_to_write);
  uint8_t string_x0 = r_x0 + (r_width - (ch_width * number_of_caracters)) / 2; //Centralize the rectangle
  uint8_t string_y0 = r_y0 + (r_height - ch_height) / 2;

  tft.fillTriangle(x0, y0, x1, y1, x2, y2, color_2);
  tft.drawTriangle(x0, y0, x1, y1, x2, y2, color_1);

  tft.fillRect(r_x0, r_y0, r_width, r_height, color_2);
  tft.drawRect(r_x0, r_y0, r_width, r_height, color_1);
  tft.drawFastVLine(r_x0, r_y0 + 1, r_height - 2, color_2); //Erase the comum point between the triangle and the rectangle

  tft.setCursor(string_x0, string_y0); //Coordinates where the string will begin to be writen
  tft.setTextColor(color_1);
  tft.setTextSize(1); // tamanho da letra
  tft.println(what_to_write);
}

/*Action: draw the right arrow
  Parameters:
            symbol_pos -> what vertical position the symbol will be
            arrow_width -> the total width of the arrow, including the triangle and the rectangle width
            string -> what will be writen in the arrow
            symbol_number -> how many symbols will be in the = how many vertical divisions
  Return: nothing
*/
uint8_t draw_Right_Arrow(uint8_t symbol_pos, uint8_t arrow_width, char *what_to_write, uint8_t symbol_number) {
  if (symbol_pos == 0) { //when it isnt to print the arrow
    return 0;
  }
  uint8_t bg_r_width = width / symbol_number;
  uint8_t triangle_height = thickness_rect_notification - 2;
  uint8_t triangle_width = (thickness_rect_notification - (0.5 * thickness_rect_notification));

  uint8_t r_x0 = 1 + (bg_r_width * (symbol_pos - 1)) + ((bg_r_width - arrow_width) / 2);
  uint8_t r_width = arrow_width - triangle_width; //r_width != bg_r_width. The first is just the rectangle of the arrow
  uint8_t x0 = r_x0 + r_width + triangle_width - 3;
  uint8_t y0 = thickness_rect_notification / 2;
  uint8_t x1 = x0 - triangle_width;
  uint8_t y1 = y0 + (triangle_height / 2);
  uint8_t x2 = x1;
  uint8_t y2 = y0 - (triangle_height / 2);
  uint8_t r_y0 = y0 - (triangle_height / 4);
  uint8_t r_height = triangle_height / 2;

  uint8_t number_of_caracters = strlen(what_to_write);
  uint8_t string_x0 = r_x0 + ((r_width - (ch_width * number_of_caracters)) / 2) + 1; //Centralize the rectangle
  uint8_t string_y0 = r_y0 + (r_height - ch_height) / 2;

  tft.fillTriangle(x0, y0, x1, y1, x2, y2, color_2);
  tft.drawTriangle(x0, y0, x1, y1, x2, y2, color_1);

  tft.fillRect(r_x0, r_y0, r_width, r_height, color_2);
  tft.drawRect(r_x0, r_y0, r_width, r_height, color_1);
  tft.fillRect(x1, r_y0, 3, r_height, color_2); //Erase the comum point between the triangle and the rectangle

  tft.setCursor(string_x0, string_y0); //Coordinates where the string will begin to be writen
  tft.setTextColor(color_1);
  tft.setTextSize(1);
  tft.println(what_to_write);
}

/*Action: draw the battery symbol
  Parameters:
            symbol_pos -> what vertical position the symbol will be
            symbol_number -> how many symbols there will be in the notification bar
            rect_percentage -> what is the percentage of that should be filled in the rectangle, it represents the same percentage of battery
  Return: nothing
*/

void draw_Battery(uint8_t symbol_pos, uint8_t symbol_number, uint8_t rect_percentage) {
  if (symbol_pos == 0) {
    return 0;
  }
  uint8_t bg_r_width = width / symbol_number;
  uint8_t r_width = 42; //Let the battery with a definied length
  uint8_t x0 = (bg_r_width * (symbol_pos - 1)) + ((bg_r_width - r_width) / 2);
  uint8_t y0 = 3;
  uint8_t r_height = thickness_rect_notification - 5;

  tft.drawRect(x0, y0, r_width, r_height , color_2);
  tft.drawFastVLine(x0 + r_width + 1, y0 + 7, r_height - 14, color_2); //make a visual effect at the end of the rectangle
  tft.drawFastVLine(x0 + r_width + 2, y0 + 7, r_height - 14, color_2); //make a visual effect at the end of the rectangle
  //this below will change when you have the value of voltage of the battery. So you can know the percentage directly, without all those if. THis is just for testing
  if (rect_percentage <= 100 && rect_percentage > 80) {
    tft.fillRect(x0 + 2, y0 + 2, (1 * r_width) - 4, r_height - 4, GREEN);
  } else if (rect_percentage <= 80 && rect_percentage > 60) {
    tft.fillRect(x0 + 2, y0 + 2, (0.8 * r_width) - 4, r_height - 4, GREEN);
  }
  else if (rect_percentage <= 60 && rect_percentage > 40) {
    tft.fillRect(x0 + 2 , y0 + 2, (0.6 * r_width) - 4, r_height - 4, YELLOW); //Always when u add some value in the x, to keep the same proportional using the r_width, you have to subtract the double of the value (2 -> 4)
  }
  else if (rect_percentage <= 40 && rect_percentage > 20) {
    tft.fillRect(x0 + 2, y0 + 2, (0.4 * r_width) - 4, r_height - 4, YELLOW);
  }
  else if (rect_percentage <= 20 && rect_percentage >= 0) {
    tft.fillRect(x0 + 2, y0 + 2, (0.2 * r_width) - 4, r_height - 4, RED);
  }
  else if (rect_percentage < 0 || rect_percentage > 100) {
    tft.fillRect(x0 + 2, y0 + 2, (1 * r_width) - 4, r_height - 4, RED); //If the battery is out of the limit 0-100% something is wrong
  }
}

/*Action: draw the temperature value
  Parameters:
            symbol_pos -> what vertical position the symbol will be
            symbol_number -> how many symbol's there is in the notification bar
  Return: nothing
*/
void draw_Temp(uint8_t symbol_pos, uint8_t symbol_number) {
  if (symbol_pos == 0) {
    return 0;
  }
  uint8_t r_width = width / symbol_number;
  uint8_t t_size = 2;
  uint8_t degree_r = 2;
  uint8_t number_of_caracters = 3; //It will always have 3 characters
  uint8_t x0 = (r_width * (symbol_pos - 1)) + ((r_width - ((t_size * number_of_caracters * ch_width) + (2 * degree_r))) / 2);
  uint8_t y0 = (thickness_rect_notification - (t_size * 7)) / 2;
  uint8_t degree_x0 = x0 + (t_size * ch_width * 2) + degree_r; //(t_size*ch_width*2) width of the two characters
  uint8_t degree_y0 = y0 + degree_r;
  tft.setTextSize(t_size);
  tft.setTextColor(color_1);

  if (temp >= 10 && temp < 100) {
    tft.setCursor(x0, y0); //Coordinates where the string will began to be writen
    tft.print(temp);
    tft.drawCircle(degree_x0, degree_y0, degree_r, color_1);
    tft.setCursor(degree_x0 + 2 * degree_r, y0);
    tft.print("C");
  } else if (temp < 10 && temp > 0) {
    tft.setCursor(x0, y0); //Coordinates where the string will began to be writen
    tft.print("0"); //Printing a number 0 to stay with 3 characters
    tft.print(temp);
    tft.drawCircle(degree_x0, degree_y0, degree_r, color_1);
    tft.print("C");
  } else if (temp < 0 || temp >= 100) { //It will be a impossible value. So, this is to make easier debugging an error.
    tft.setCursor(x0, y0); //Coordinates where the string will began to be writen
    tft.setTextColor(RED);
    tft.print("xxx");
  }
}

/*Action: draw the humidity value
  Parameters:
            symbol_pos -> what vertical position the symbol will be
            symbol_number -> how many symbol's there is in the notification bar
  Return: nothing
*/

void draw_Humi(uint8_t symbol_pos, uint8_t symbol_number) {
  if (symbol_pos == 0) {
    return 0;
  }
  uint8_t r_width = width / symbol_number;
  uint8_t t_size = 2;
  uint8_t number_of_caracters = 3; //It will always have 3 characters. 2 for a value between 0 and 99 and the %.
  uint8_t x0 = (r_width * (symbol_pos - 1)) + ((r_width - (t_size * number_of_caracters * ch_width)) / 2);
  uint8_t y0 = (thickness_rect_notification - (t_size * 7)) / 2;

  tft.setTextSize(t_size);
  tft.setTextColor(color_1);

  if (humidity >= 10 && humidity < 100) { //Value in the normal range
    tft.setCursor(x0, y0);
    tft.print(humidity);
    tft.print("%");
  }  else if (humidity < 10 && humidity > 0) { //Value just with one caracter
    tft.setCursor(x0, y0);
    tft.print("0");
    tft.print(humidity);
    tft.print("%");
  } else if (humidity < 0 || humidity >= 99) { //Impossible value for humidity. So, this is to make easier debugging an error.
    tft.setCursor(x0, y0);
    tft.setTextColor(RED);
    tft.print("xxx");
  }
}

/*Action: draw the notification bar
  Parameters:
            mode -> each mode corresponds to a type of a notification bar. That is the symbol can change the place, have different number of symbols and differentes symbols.
  Return: nothing
*/

void notific_Bar (uint8_t number_of_symbols, uint8_t pos_left_arrow, char *text_left_arrow, uint8_t pos_hum, uint8_t pos_temp, uint8_t pos_bat, uint8_t pos_right_arrow, char *text_right_arrow) {
  draw_VLine(0, 0, thickness_rect_notification, 1, color_1);
  draw_Rect_Bg_Not_Bar(1, 1, thickness_rect_notification, number_of_symbols, pos_left_arrow, pos_right_arrow);

  draw_Left_Arrow(pos_left_arrow, sym_arrow_width, text_left_arrow, number_of_symbols);
  draw_Right_Arrow(pos_right_arrow, sym_arrow_width, text_right_arrow, number_of_symbols);
  draw_Temp(pos_temp, number_of_symbols);
  draw_Humi(pos_hum, number_of_symbols);
  draw_Battery(pos_bat, number_of_symbols, bat_level);
}

/*Action: create the screen Menu, that represents the initial page
  Functions:
            printMenu() -> Print in the screen the word 'MENU'
            drawRects -> draw the rectangles that represents the buttons: MEASURE, SETTINGS, SD CARD and INFORMATIONS
  Return: nothing
*/
void screen_Menu() {
  //reset_sensor_counter(); think if its better if dont reset the sensor counter, because like that, the user can change the pages and when go back, the previously configurations are selected. Maybe its better to reset after showing results only
  tft.drawRect(0, 0, width, height, color_1); //Draw a rect in all the edge. Just to make a good visual effect
  printMenu();
  drawRects();
  print_RTC_Data_Menu_Screen(3, 2, 1, false);
}

uint16_t print_RTC_Data_Menu_Screen(uint8_t pos_Time, uint8_t pos_Date, uint8_t pos_DayWeek, boolean with_seconds){
  uint16_t *coordinates = calculate_coordinates_RTC_Data(pos_Time, pos_Date, pos_DayWeek);     
  uint8_t *Time = print_Time(coordinates[0], coordinates[1], with_seconds);  
  hours = Time[0];
  minutes = Time[1];
  minutes_now = minutes;
  print_Date(coordinates[2], coordinates[3]);
  print_DayWeek(coordinates[4], coordinates[5]);
}
/*Action: Print in the screen the word 'MENU'
  Parameters:
  Return: t_size = the text size of the word MENU
*/

uint8_t printMenu() {
  uint8_t t_size = 5;
  uint8_t number_of_characters = strlen("MENU");;
  uint8_t x0 = (width - (ch_width * t_size * number_of_characters)) / 2;
  uint8_t y0 = y0_menu;
  tft.setCursor(x0, y0);
  tft.setTextSize(t_size);
  tft.setTextColor(color_2);
  tft.println("MENU");
  tft.setCursor(x0 - 2, y0);
  tft.setTextColor(color_1);
  tft.println("MENU");
  return t_size;
}

/*Action: create the four initial buttons in the screen Menu by drawing the background rectangles
  Parameters:
  Return:
*/

void drawRects() {
  char *what_to_write[] = {"MEASURE", "SETTINGS", "SD CARD", "INFORMATIONS"};
  uint8_t number_of_rect = 4;
  uint8_t number_of_characters_of_biggest_word = 12; //INFORMATIONS
  uint8_t t_size = 3;
  uint8_t radius = 3;
  uint8_t height_btw_menu_rect = 35;
  uint8_t height_btw_rect_rect = 10;
  uint8_t x0 = ((width - (number_of_characters_of_biggest_word * ch_width)) / 2) - (10 * ch_width); //begin the rect
  uint8_t c_x0 = 0; //character_x0
  uint8_t c_y0 = 0; //character_y0
  uint8_t menu_t_size = printMenu();
  uint8_t y0 = y0_menu + (ch_height * menu_t_size) + height_btw_menu_rect;
  uint8_t r_width = (number_of_characters_of_biggest_word * ch_width) + (20 * ch_width);
  uint8_t r_height = (t_size * ch_height) + height_btw_rect_rect;
  uint8_t column = 0;

  for (uint8_t counter = 0; counter < number_of_rect; counter++) {
    tft.fillRoundRect(x0, y0, r_width, r_height, radius, color_2);
    tft.drawRoundRect(x0, y0, r_width, r_height, radius, color_1);

    //Buttons: 0 = MEASURE, 1 = SEETINGS, 2 = SD CARD, 3 = INFORMATIONS
    if (M_Button[0][column] == 0) { //This is just to storage the points in the Matrix one time. Hoping that make the code to be faster.
      M_Button[0][column] = x0;
      M_Button[1][column] = x0 + r_width;
      M_Button[2][column] = y0;
      M_Button[3][column] = y0 + r_height;
      column = column + 1;
    }

    c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[counter]))) / 2; //centralizar em x o texto
    c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

    tft.setTextSize(t_size - 1);
    tft.setCursor(c_x0, c_y0);
    tft.print(what_to_write[counter]);

    y0 = y0 + r_height + height_btw_rect_rect;
  }
}

/*Action: create the central circle that represents all the sensors
  Parameters: -
  Return: -
*/

void central_circle() {
  uint8_t sensor_number = 1;
  uint8_t c_x0 = width / 2; //central point x0
  uint8_t c_y0 = thickness_rect_notification + (height - thickness_rect_notification) / 2; //central point y0 in the screen counting with the thickness of the notification bar
  uint8_t c_r = (width / 2) - 3; //circunference's radius
  uint8_t column = 1;
  tft.fillCircle(c_x0, c_y0, c_r, bg_color_sym_1);
  tft.drawCircle(c_x0, c_y0, c_r, color_1);

  uint8_t t_size = 2;
  uint8_t t_x0, t_y0;

  uint8_t counter, r_width, r_height;
  double x0, y0;
  r_width = (2 * c_r) / 6; //The width of each square is the diameter divided by the biggest number of squares, that is 6.
  r_height = r_width; //Make the height equal to the width, so it's become a square

  //Draw the first four squares
  y0 = c_y0 - c_r + 30; //The number 30 will be explain the code's manual.
  x0 = 3 + (c_x0 - (sqrt(pow(c_r, 2) - (pow(y0 - c_y0, 2))))); //We have the distance between two points and 3 of the 4 coordinates. So, using distance's formula, we can find the last coordinate.
  for (counter = 0; counter < 4; counter++) {
    tft.fillRect(x0, y0, r_width, r_height, color_2);    
    tft.drawRect(x0, y0, r_width, r_height, dont_measure_color);

    M_Button[0][column] = x0;
    M_Button[1][column] = x0 + r_width;
    M_Button[2][column] = y0;
    M_Button[3][column] = y0 + r_height;
    column++;

    t_x0 = x0 + (r_width - (t_size*1*ch_width)) / 2; //centralize in the x axis the text
    t_y0 = y0 + (r_height - (t_size) * ch_height) / 2; //centralize in the y axis the text
    tft.setTextSize(t_size);
    tft.setCursor(t_x0, t_y0);
    tft.print(sensor_number);

    x0 = x0 + r_width - 1;
    sensor_number++;
  }

  //Draw the 6 squares that are in the centre
  y0 = c_y0 - ((2 * c_r) / 12);
  x0 = 2 + (c_x0 - (sqrt(pow(c_r, 2) - (pow(y0 - c_y0, 2)))));
  for (counter = 4; counter < 10; counter++) {
    tft.fillRect(x0, y0, r_width, r_height, color_2);
    tft.drawRect(x0, y0, r_width, r_height, dont_measure_color);
    
    M_Button[0][column] = x0;
    M_Button[1][column] = x0 + r_width;
    M_Button[2][column] = y0;
    M_Button[3][column] = y0 + r_height;
    column++;
    if (sensor_number == 10){
      t_x0 = x0 + (r_width - (t_size * 2*ch_width)) / 2; //centralize in the x axis the text
    } else{
      t_x0 = x0 + (r_width - (t_size * 1*ch_width)) / 2; //centralize in the x axis the text
    }
    t_y0 = y0 + (r_height - (t_size) * ch_height) / 2; //centralize in the y the text
    tft.setTextSize(t_size);
    tft.setCursor(t_x0, t_y0);
    tft.print(sensor_number);

    x0 = x0 + r_width - 1;
    sensor_number++;
  }

  //Draw the last four squares
  y0 = c_y0 - c_r + 30;
  x0 = 3 + (c_x0 - (sqrt(pow(c_r, 2) - (pow(y0 - c_y0, 2)))));
  y0 = c_y0 - c_r + 30 + (r_width + 30 + r_width + 30);
  for (counter = 10; counter < 14; counter++) {
    tft.fillRect(x0, y0, r_width, r_height, color_2);
    tft.drawRect(x0, y0, r_width, r_height, dont_measure_color);
    
    M_Button[0][column] = x0;
    M_Button[1][column] = x0 + r_width;
    M_Button[2][column] = y0;
    M_Button[3][column] = y0 + r_height;
    column++;

    t_x0 = x0 + (r_width - (t_size*2*ch_width)) / 2; //centralizar em x o texto
    t_y0 = y0 + (r_height - (t_size) * ch_height) / 2; //centralizar em y o texto que está sendo escrito
    tft.setTextSize(t_size);
    tft.setCursor(t_x0, t_y0);
    tft.print(sensor_number);

    x0 = x0 + r_width - 1;
    sensor_number++;
  }
  
  for (counter = 1; counter<=14; counter++){
    if (sensor_counter[counter] == 1) {
      tft.drawRect(M_Button[0][counter], M_Button[2][counter], M_Button[1][counter] - M_Button[0][counter], M_Button[3][counter] - M_Button[2][counter], nbac_color);
    } else if (sensor_counter[counter] == 2) {
      tft.drawRect(M_Button[0][counter], M_Button[2][counter], M_Button[1][counter] - M_Button[0][counter], M_Button[3][counter] - M_Button[2][counter], ybac_color);
    }
  }  
}

/*Action: print some text/word.
  Parameters:
              which_text -> Represents the word that will be writen
              disable -> Sometimes we need to erase the text in the screen to write something up. So, we need to disable (1) that text. Otherwise, disable = 0.
              sensor_number -> Will be used to know the order and the number of sensors that should be measured
  Return: -
*/

uint16_t print_Text (uint8_t which_text, boolean disable, uint8_t sensor_number, uint8_t settings_page) {
  char *what_to_write[] = {"CLICK TO SELECT", "LOADING...", "DONE!", "SENSOR ", "OUTPUT SIGNAL", "FINAL VALUE (V)", "INITIAL VALUE (V)", "SIGNAL STEP", "INTEGRATION TIME", "Graph V(V) x I(uA)", "GRAPH LAYOUT", "AXIS", "X", "Y", "DECIMAL PLACES", "SCALE INITIAL VALUE", "SCALE STEP", "SCALE FINAL VALUE", "DOTTED LINES"};
  uint8_t r = 3;
  uint8_t t_size = 3; //This will be always +1 bigger than the real character size to centralize the text in the rectangle
  if (which_text == 0) { //CLICK TO SELECT
    uint8_t word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_text]);
    uint8_t r_width = (word_width) + (0.2) * word_width;
    uint8_t x0 = (width - (r_width)) / 2;
    uint8_t y0 = thickness_rect_notification + 3;
    uint8_t r_height =  (ch_height * t_size) + t_size;
    if (disable == false) {
      //If those 2 lines below the text seemns like a button. It is better without them? Wait for choose.
      //tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      //tft.drawRoundRect(x0, y0, r_width, r_height, r, text_edge_color);

      uint8_t c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_text]))) / 2; //centralize the text in x axis
      uint8_t c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralize in the y axis

      tft.setTextSize(t_size - 1);
      tft.setTextColor(color_2);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_text]);

      tft.setCursor(c_x0 - 2, c_y0);
      tft.setTextColor(color_1);
      tft.print(what_to_write[which_text]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
    }
  } else if (which_text == 1) { //LOADING...
    uint8_t word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_text]);
    uint8_t r_width = (word_width) + (0.2) * word_width;
    uint8_t x0 = (width - (r_width)) / 2;
    uint8_t y0 = thickness_rect_notification + 3;
    uint8_t r_height =  (ch_height * t_size) + t_size;
    if (disable == false) {
      //If those 2 lines below the text seemns like a button. It is better without them? Wait for choose.
      //tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      //tft.drawRoundRect(x0, y0, r_width, r_height, r, text_edge_color);

      uint8_t c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_text]))) / 2; //centralizar em x o texto
      uint8_t c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

      tft.setTextSize(t_size - 1);
      tft.setTextColor(color_2);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_text]);

      tft.setCursor(c_x0 - 2, c_y0);
      tft.setTextColor(color_1);
      tft.print(what_to_write[which_text]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
    }
  } else if (which_text == 2) { //DONE!
    uint8_t word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_text]);
    uint8_t r_width = (word_width) + (0.2) * word_width;
    uint8_t x0 = (width - (r_width)) / 2;
    uint8_t y0 = thickness_rect_notification + 3;
    uint8_t r_height =  (ch_height * t_size) + t_size;
    if (disable == false) {
      //If those 2 lines below the text seemns like a button. It is better without them? Wait for choose.
      //tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      //tft.drawRoundRect(x0, y0, r_width, r_height, r, text_edge_color);

      uint8_t c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_text]))) / 2; //centralizar em x o texto
      uint8_t c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

      tft.setTextSize(t_size - 1);
      tft.setTextColor(color_2);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_text]);

      tft.setCursor(c_x0 - 2, c_y0);
      tft.setTextColor(color_1);
      tft.print(what_to_write[which_text]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
    }
  } else if (which_text == 3) { //SENSOR X
    uint8_t word_width = ch_width * (t_size - 1) * (strlen(what_to_write[which_text]) + 2); //The number 2 referes to the 2 algarisms that can have of the sensor (1-14)

    if (disable == false) {
      uint8_t c_x0 = ((width - word_width) / 2); //centralizar em x o texto
      uint8_t c_y0 = thickness_rect_notification +  (((ch_height * t_size) + t_size) - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito. Explicar essa zona depois.

      tft.setTextSize(t_size - 1);
      tft.setTextColor(color_2);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_text]);
      if (which_sensor_to_be_measured(sensor_number) < 10) { //to put a 0 before the number when it have just 1 algarism
        tft.print("0");
      }
      tft.print(which_sensor_to_be_measured(sensor_number));

      tft.setCursor(c_x0 - 2, c_y0);
      tft.setTextColor(color_1);
      tft.print(what_to_write[which_text]);
      if (which_sensor_to_be_measured(sensor_number) < 10) {
        tft.print("0");
      }
      tft.print(which_sensor_to_be_measured(sensor_number));

      word_width = ch_width * (t_size - 2) * (strlen(what_to_write[which_text + 6]));
      c_x0 = ((width - word_width) / 2);
      c_y0 = c_y0 + (ch_height * (t_size - 1)) + 2;

      tft.setTextSize(t_size - 2);
      tft.setTextColor(color_2);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_text + 6]);
    } else if (disable == true) {
      //tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
    }
  } else if (which_text == 4 || which_text == 10) { //OUTPUT SIGNAL or GRAPHICHS LAYOUT
    if (settings_page == 1) { //if it's in the first settings page, print the OUTPUT SIGNAL
      which_text = 4 ;
    } else if (settings_page == 2) { //If it's in the second settings page, print GRAPHICHS LAYOUT
      which_text = 10;
    }
    t_size = 3;
    uint8_t word_width = ch_width * (t_size) * (strlen(what_to_write[which_text]));
    if (disable == false) {
      uint8_t c_x0 = 2 + ((width - (word_width)) / 2); //centralizar em x o texto
      uint8_t c_y0 = thickness_rect_notification + 5; //centralizar em y o texto que está sendo escrito. Explicar essa zona depois.

      tft.setTextSize(t_size);
      tft.setTextColor(color_2);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_text]);

      tft.setCursor(c_x0 - 2, c_y0);
      tft.setTextColor(color_1);
      tft.print(what_to_write[which_text]);

      if (settings_page == 2) {
        t_size = 2;
        word_width = ch_width * (t_size) * (strlen(what_to_write[11])); //the word AXIS
        c_x0 = (width - word_width) / 2; //centralize in the all screen
        c_y0 = c_y0 + ((t_size + 1) * ch_height) + 3;
        tft.setTextSize(t_size);
        tft.setCursor(c_x0, c_y0);
        tft.setTextColor(color_1);
        tft.print(what_to_write[11]);

        tft.setCursor(c_x0 - 2, c_y0);
        tft.setTextColor(color_2);
        tft.print(what_to_write[11]);

        word_width = ch_width * (t_size) * (strlen(what_to_write[12]));
        c_x0 = (width / 2) - (width / 4) - (word_width / 2); //centralize in the first half
        tft.setCursor(c_x0, c_y0);
        tft.setTextColor(color_2);
        tft.print(what_to_write[12]);

        c_x0 = (width / 2) + (width / 4) - (word_width / 2); //centralize in the second half
        tft.setCursor(c_x0, c_y0);
        tft.setTextColor(color_2);
        tft.print(what_to_write[13]);

      }
    } else if (disable == true) {
      //tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
    }
  } else if (which_text == 5 || which_text == 14) { //begin with FINAL VALUE (V) -> INITIAL VALUE (V) -> SIGNAL STEP and ends with INTEGRATION TIME
    t_size = 2;
    uint8_t word_width = 0;
    uint8_t number_of_configurable_settings = 4;
    uint8_t distance_bt_words = 0;
    uint8_t c_x0 = 0;
    uint8_t definied_y0_value = 45;
    uint16_t c_y0 = 0;
    uint8_t counter = 0;

    if (which_text == 5) {
      word_width = ch_width * (t_size) * (strlen(what_to_write[which_text]));
      number_of_configurable_settings = 4;
      distance_bt_words = (t_size * ch_height) + 40; //40 was definied testing
      c_x0 = ((width - (word_width)) / 2); //centralizar em x o texto
      definied_y0_value = 45;
      c_y0 = thickness_rect_notification + definied_y0_value; //centralizar em y o texto que está sendo escrito. Explicar essa zona depois.

      if (disable == false) {
        for (counter = 1; counter <= number_of_configurable_settings; counter++) {
          tft.setTextSize(t_size);
          tft.setTextColor(color_2);
          tft.setCursor(c_x0, c_y0);
          tft.print(what_to_write[which_text]);

          which_text++;
          c_y0 = c_y0 + distance_bt_words;
          word_width = ch_width * (t_size) * (strlen(what_to_write[which_text]));
          c_x0 = ((width - (word_width)) / 2);
        }
      } else if (disable == true) {
        //tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      }
    } else if (which_text == 14) { //DECIMAL PLACES -> SCALE INITIAL VALE -> SCALE STEP -> SCALE FINAL VALUE -> DOTTED LINES
      word_width = ch_width * (t_size) * (strlen(what_to_write[which_text]));
      number_of_configurable_settings = 5;
      distance_bt_words = (t_size * ch_height) + 30; //30 was definied testing
      c_x0 = ((width - (word_width)) / 2); //centralizar em x o texto
      definied_y0_value = 47;
      c_y0 = thickness_rect_notification + definied_y0_value; //centralizar em y o texto que está sendo escrito. Explicar essa zona depois.

      if (disable == false) {
        for (counter = 1; counter <= 5; counter++) {
          tft.setTextSize(t_size);
          tft.setTextColor(color_2);
          tft.setCursor(c_x0, c_y0);
          tft.print(what_to_write[which_text]);

          which_text++;
          c_y0 = c_y0 + distance_bt_words;
          word_width = ch_width * (t_size) * (strlen(what_to_write[which_text]));
          c_x0 = ((width - (word_width)) / 2);
        }
      } else if (disable == true) {
        //tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      }
    }
  }
  return thickness_rect_notification + 45 + (t_size * ch_height); //45 is the value on variable "definied_y0_value" -> gambiarra
}


/*Action: create a button. Button is know for having some action when the touchscreen is select, instead of the print_Text, that don't recognize the touch in the screen.
  Parameters:
              which_button -> Represents the button that will be created
              disable -> Sometimes we need to erase the text in the screen to write something up. So, we need to disable (1) that button. Otherwise, disable = 0.
  Return: -
*/
void create_button (uint8_t which_button, boolean disable) {
  //The space betwwen the begining of the rectangle and the beggining of the word that will be writen inside it will be defined as 20% of the width occupied by the word |0,2x   x   0,2x| -> |xx xxxxxxxxxx xx|
  //O espaço entre o inicio do retangulo e o ínicio da palavra vai ser definido como 20% do comprimento ocupado pela palavra
  char *what_to_write[] = {"START", "CANCEL", "SEE RESULTS", "MENU", "PREVIEW", "CLOSE"};
  uint8_t r = 3;
  uint8_t t_size = 3; //This will be always +1 bigger than the real character size to centralize the text in the rectangle
  uint8_t column = 19;
  uint8_t word_width = 0;
  uint8_t r_width = 0;
  uint8_t x0 = 0;
  uint8_t r_height = 0;
  uint16_t y0 = 0;
  uint8_t c_x0 = 0;
  uint16_t c_y0 = 0;

  tft.setTextColor(color_1);

  if (which_button == 0) { //create the START 
    word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_button]);
    r_width = (word_width) + (0.2) * word_width;
    x0 = (width - (r_width)) / 2;
    r_height =  (ch_height * t_size) + t_size;
    y0 = height - r_height - 2;
    column = 15;
    if (disable == false) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

      M_Button[0][column] = x0;
      M_Button[1][column] = x0 + r_width;
      M_Button[2][column] = y0;
      M_Button[3][column] = y0 + r_height;
    

      c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_button]))) / 2; //centralizar em x o texto
      c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

      tft.setTextSize(t_size - 1);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_button]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      //the column of button START is 15.
      M_Button[0][column] = 0;
      M_Button[1][column] = 0;
      M_Button[2][column] = 0;
      M_Button[3][column] = 0;
    }
  } else if (which_button == 1) { //CREATE THE CANCEL BUTTON
    word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_button]);
    r_width = (word_width) + (0.2) * word_width;
    x0 = (width - (r_width)) / 2;
    r_height =  (ch_height * t_size) + t_size;
    y0 = height - r_height - 35;
    column = 0;
    if (disable == false) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);
      
      M_Button[0][column] = x0;
      M_Button[1][column] = x0 + r_width;
      M_Button[2][column] = y0;
      M_Button[3][column] = y0 + r_height;
    
      c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_button]))) / 2; //centralizar em x o texto
      c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito
      tft.setTextSize(t_size - 1);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_button]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      //the column of button CANCEL is 20.
      M_Button[0][column] = 0;
      M_Button[1][column] = 0;
      M_Button[2][column] = 0;
      M_Button[3][column] = 0;
    }
  } else if (which_button == 2) { //CREATE THE 'SEE RESULTS' BUTTON
    word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_button]);
    r_width = (word_width) + (0.2) * word_width;
    x0 = (width - (r_width)) / 2;
    r_height =  (ch_height * t_size) + t_size;
    y0 = height - r_height - 35;
    column = 1;
    if (disable == false) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

      M_Button[0][column] = x0;
      M_Button[1][column] = x0 + r_width;
      M_Button[2][column] = y0;
      M_Button[3][column] = y0 + r_height;
    
      c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_button]))) / 2; //centralizar em x o texto
      c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

      tft.setTextSize(t_size - 1);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_button]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      //the column of button SEE RESULTS is 2.
      M_Button[0][column] = 0;
      M_Button[1][column] = 0;
      M_Button[2][column] = 0;
      M_Button[3][column] = 0;
    }
  } else if (which_button == 3) { //CREATE THE 'MENU' BUTTON
    word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_button]);
    r_width = (word_width) + (0.2) * word_width;
    x0 = (width - (r_width)) / 2;
    r_height =  (ch_height * t_size) + t_size;
    y0 = height - r_height - 2; //y0 = 294
    column = 2;
    if (disable == false) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

      M_Button[0][column] = x0;
      M_Button[1][column] = x0 + r_width;
      M_Button[2][column] = y0;
      M_Button[3][column] = y0 + r_height;
    
      c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_button]))) / 2; //centralizar em x o texto
      c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

      tft.setTextSize(t_size - 1);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_button]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      //the column of button SEE RESULTS is 21.
      M_Button[0][column] = 0;
      M_Button[1][column] = 0;
      M_Button[2][column] = 0;
      M_Button[3][column] = 0;
    }
  } else if (which_button == 4) { //CREATE THE 'PREVIEW' BUTTON
    word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_button]);
    r_width = (word_width) + (0.2) * word_width;
    x0 = (width - (r_width)) / 2;
    r_height =  (ch_height * t_size) + t_size;
    y0 = height - r_height - 2;
    column = 22;
    if (disable == false) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

      M_Button[0][column] = x0;
      M_Button[1][column] = x0 + r_width;
      M_Button[2][column] = y0;
      M_Button[3][column] = y0 + r_height;
    
      c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_button]))) / 2; //centralizar em x o texto
      c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

      tft.setTextSize(t_size - 1);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_button]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      //the column of button SEE RESULTS is 21.
      M_Button[0][column] = 0;
      M_Button[1][column] = 0;
      M_Button[2][column] = 0;
      M_Button[3][column] = 0;
    }
  } else if (which_button == 5) { //CREATE THE 'CLOSE' BUTTON
    word_width = ch_width * (t_size - 1) * strlen(what_to_write[which_button]);
    r_width = (word_width) + (0.2) * word_width;
    x0 = (width - (r_width)) / 2;
    r_height =  (ch_height * t_size) + t_size;
    y0 = height - r_height - 2;
    column = 23;
    if (disable == false) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

      M_Button[0][column] = x0;
      M_Button[1][column] = x0 + r_width;
      M_Button[2][column] = y0;
      M_Button[3][column] = y0 + r_height;
    
      c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(what_to_write[which_button]))) / 2; //centralizar em x o texto
      c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

      tft.setTextSize(t_size - 1);
      tft.setCursor(c_x0, c_y0);
      tft.print(what_to_write[which_button]);
    } else if (disable == true) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
      M_Button[0][column] = 0;
      M_Button[1][column] = 0;
      M_Button[2][column] = 0;
      M_Button[3][column] = 0;
    }
  }
}

/*Action: this function will know which button was pressed by discovering it's column
  Parameters: -
  Return: -
*/

void which_Button_Pressed () { //This function will be always be checking which button is pressed by selecting which page it is to be faster
  uint16_t columns = 0;
  uint8_t lines = 0;
  TSPoint p = ts.getPoint();
  p.x = map(p.x, TS_MAXX, TS_MINX, 0, 240);
  p.y = map(p.y, TS_MAXY, TS_MINY, 0, 320);
  for (columns = 0; columns < num_columns; columns++) {
    if (p.x > M_Button[lines][columns] && p.x < M_Button[lines + 1][columns] && p.y > M_Button[lines + 2][columns] && p.y < M_Button[lines + 3][columns]) {
      button_clicked = columns;
      if (current_page == 17 || current_page == 18){ //because in this case, the edge of the button must back to normal and dont keep as white
        tft.drawRect(M_Button[lines][columns]-1, M_Button[lines + 2][columns]-1, M_Button[lines + 1][columns] - M_Button[lines][columns]+2, M_Button[lines + 3][columns] - M_Button[lines + 2][columns]+2, WHITE);
        delay(100);
        tft.drawRect(M_Button[lines][columns]-1, M_Button[lines + 2][columns]-1, M_Button[lines + 1][columns] - M_Button[lines][columns]+2, M_Button[lines + 3][columns] - M_Button[lines + 2][columns]+2, bg_color);
      } else{
        tft.drawRect(M_Button[lines][columns], M_Button[lines + 2][columns], M_Button[lines + 1][columns] - M_Button[lines][columns], M_Button[lines + 3][columns] - M_Button[lines + 2][columns], WHITE);
      }
      break; 
    }
  }
}

void selected_sensors() {
  uint8_t sensor_selected = button_clicked;
  uint8_t lines = 0;
  sensor_counter[sensor_selected] = sensor_counter[sensor_selected] + 1; //Each time that the button is clicked, increment in the position of the sensor_counter
  if (sensor_counter[sensor_selected] > 2) { //If it is bigger the 2, re start of the begining
    sensor_counter[sensor_selected] = 0;
  }
  if (sensor_counter[sensor_selected] == 0) {
    tft.drawRect(M_Button[lines][button_clicked], M_Button[lines + 2][button_clicked], M_Button[lines+1][button_clicked] - M_Button[lines][button_clicked], M_Button[lines+3][button_clicked] - M_Button[lines + 2][button_clicked], dont_measure_color);
  } else if (sensor_counter[sensor_selected] == 1) {
    tft.drawRect(M_Button[lines][button_clicked], M_Button[lines + 2][button_clicked], M_Button[lines+1][button_clicked] - M_Button[lines][button_clicked], M_Button[lines+3][button_clicked] - M_Button[lines + 2][button_clicked], nbac_color);
  } else if (sensor_counter[sensor_selected] == 2) {
    tft.drawRect(M_Button[lines][button_clicked], M_Button[lines + 2][button_clicked], M_Button[lines+1][button_clicked] - M_Button[lines][button_clicked], M_Button[lines+3][button_clicked] - M_Button[lines + 2][button_clicked], ybac_color);
  }
}
/*Action: this function will create the screen after clicking in the button "MEASURE"
  Functions:
            -> notific_Bar():
            -> print_Text(0,0,0): print the text "CLICK TO SELECT"
            -> central_circle(): draw the central circle with the 14 sensors
  Return: -
*/
void screen_Measure() {
  uint8_t number_of_symbols = 4;
  uint8_t left_arrow_position = 1; uint8_t right_arrow_position = 0; uint8_t humidity_position = 2; uint8_t temperature_position = 3; uint8_t battery_position = 4;

  notific_Bar(number_of_symbols, left_arrow_position, "MENU", humidity_position, temperature_position, battery_position, right_arrow_position, NULL);
  print_Text(0, 0, 0, 0);
  central_circle();
  if (number_of_measurements() > 0) { //create the START button just when there is some sensor selected
    create_button(0, 0);
    start_already_created = true;
    Serial.println("ENTROU NESSA BUCETA");
  } else if (number_of_measurements() == 0){
    create_button(0, 1);
    start_already_created = false;
  }
}

/*Action: Just reset the vector sensor counter when clicking to go back to "MENU" or in the button "CANCEL"
  Functions: -
  Return: -
*/
void reset_sensor_counter () {
  for (uint8_t counter = 1; counter <= 14; counter++) {
    sensor_counter[counter] = 0;
  }
}

void reset_M_Button() {
  for (uint8_t lines = 0; lines < 4; lines++) {
    for (uint8_t columns = 0; columns < num_columns; columns++) {
      M_Button[lines][columns] = 0;
    }
  }
}
/*Action: will fill the progress bar to represent how much left to finish the process of colecting data
  Parameters:
            percentage_progress: to know how much in percentage it is already done in the process of colecting the data
  Return: -
*/
void progress_bar(uint8_t percentage_progress) {
  uint8_t t_size = 3; //To keep the same logic as used before.
  uint8_t r_height = (ch_height * t_size) + t_size; //to keep the same logic as used before.
  uint8_t r_width = width - 5; // To use almost the inteire width
  uint8_t x0 = 3; //To use almost the inteire width
  uint8_t y0 = 60; //print_Loading()+ 10; //10 was a definied valuer

  tft.drawRect(x0, y0, r_width, r_height , color_2);
  x0 = x0 + 2; //Just to make a visual effect
  y0 = y0 + 2; //Just to make a visual effect: one pixel is for the drawRect and the other to be like the background color
  if (percentage_progress < 2) { //GAMBIARRA PRA CORRIGIR O BUG DE QUANDO COMECA COM 0 E 1% NÃO SEI POR QUE
    percentage_progress = 2;
  }
  r_width = (r_width * percentage_progress / 100) - 4; //Just to make a visual effect
  r_height = r_height - 4; //Just to make a visual effect

  tft.fillRect(x0, y0, r_width, r_height , color_1);
  if (percentage_progress == 100) {
    print_Text(1, 1, 0, 0); //Disable the Loading...
    print_Text(2, 0, 0, 0); //Print that it is done!
    create_button(1, 1); //Desactivate the CANCEL button
    create_button(2, 0); //Create the SEE RESULTS button
  } else {
    print_Text(1, 0, 0, 0); //Print the word 'LOADING'
    create_button(2, 1); //Disable the SEE RESULTS button
    create_button(1, 0); //Create the CANCEL button
  }
}

/*Action: create the screen where the user will wait until the process is done
  Functions:
            -> notific_Bar():
            -> progress_bar(): creates the progress bar
  Return: -
*/
void screen_Loading() {
  uint8_t number_of_symbols = 3;
  uint8_t left_arrow_position = 0; uint8_t right_arrow_position = 0; uint8_t humidity_position = 1; uint8_t temperature_position = 3; uint8_t battery_position = 2;
  notific_Bar(number_of_symbols, left_arrow_position, NULL, humidity_position, temperature_position, battery_position, right_arrow_position, NULL);
  progress_bar(0);
}

uint16_t number_of_points(float initial_value, float interval, float final_value) { //discover how much points fit in some interval
  uint16_t number_of_points = 0;
  float value_now = initial_value;
  do {
    value_now = value_now + interval;
    number_of_points++;
    if (value_now > final_value) {
      number_of_points++;
      break;
    }
  } while (value_now <= final_value);
  return number_of_points;
}

void graph(int8_t sensor_number) { //sensor_number is the ordinary number for the number of measurements
  uint8_t t_size = 1;
  tft.setTextSize(t_size);
  tft.setTextColor(color_2);

  uint8_t number_of_axies_division = number_of_points(scale_initial_value_x, scale_step_x, scale_final_value_x) - 1 ;
  uint8_t number_of_ch_y = 4; //Do a function to calculate the biggest number of caracters read by the current data collect. Example: (0, 1, 3, 50, 1343); number_of_ch_y = 4;

  uint8_t space_x0, space_width, g_x0, scale_x_x0, scale_x_xfinal, scale_x_width, scale_y_x0, width_axies_x, number_useless_zero;
  uint16_t space_y0, space_height, g_y0, scale_x_y0, scale_y_y0, scale_y_yfinal, scale_y_height, height_axies_y;
  float real_word_width;

  number_useless_zero = number_of_useless_zero(scale_final_value_x, number_of_decimal_places_x); //discover how many caracters the last value of the scale will have
  real_word_width = t_size * ch_width * real_number_of_ch(number_of_decimal_places_x, number_useless_zero);

  space_x0 = number_of_ch_y * t_size * ch_width;
  space_y0 = 60;
  space_height = 224;
  g_x0 = space_x0; //the point x0 of the graph is the same point that the space begins to be draw

  scale_x_x0 = g_x0; //the point x0 of the grpah is the same poiont where the scale will begin
  scale_x_y0 = space_y0 + space_height;
  scale_x_xfinal = width - 1;
  while ((scale_x_xfinal - scale_x_x0) % number_of_axies_division != 0 || (width - 1) - scale_x_xfinal < (real_word_width / 2)) { //Discover the biggest value of x that is multiple of the number of divisions and fit all caracters that depends of decimal places
    scale_x_xfinal--;
  }
  scale_x_width = scale_x_xfinal - scale_x_x0;
  space_width = scale_x_width;

  draw_graph_Space(space_x0, space_y0, space_width, space_height);
  print_Scale_X(scale_x_x0, scale_x_y0, scale_x_width, space_y0, space_y0 + space_height, sensor_number);
  //print_Values_Scale_Y(); //draw_Dotted_Lines_Y();
  print_Points(scale_x_x0, scale_x_y0, scale_x_width, scale_y_height, sensor_number);  //convert_Current_To_Pixels(sensor_number);

}

void print_Scale_X(uint8_t x0, uint16_t y0, uint8_t scale_width, uint16_t dotted_line_begin, uint16_t dotted_line_end, int8_t show_error) { //adicionei depois q tentei arrumar o gráfico pra parte negativa (just_x_pos)
  uint8_t t_size = 1;
  tft.setTextSize(t_size);
  tft.setTextColor(color_2);
  float ch_x = x0; //x0 is the begining of the graph, but to write this point must be in the centre of the what will be writen
  uint16_t ch_y = y0 + 2; //to separate the line of y=0 and where we begin to write the numbers
  float x_value_now = scale_initial_value_x;
  x_value_now = truncate(x_value_now, number_of_decimal_places_x);

  uint8_t number_useless_zero = number_of_useless_zero(x_value_now, number_of_decimal_places_x);

  float real_word_width = t_size * ch_width * real_number_of_ch(number_of_decimal_places_x, number_useless_zero); //it means that the 0 that are in the right wont me printed, as they mean nothing
  Serial.print("A quantidade de pontos no intervalo é de "); Serial.println(number_of_points(scale_initial_value_x, scale_step_x, scale_final_value_x));
  Serial.print("O valor inicial do gráfico é "); Serial.println(ch_x);
  Serial.print("O tamanho da escala em x é "); Serial.println(scale_width);

  //Print the initial value of the scale
  tft.setCursor(ch_x - (real_word_width / 2) + 1, ch_y); //+1 was testing
  tft.print(abs(x_value_now), number_of_decimal_places_x - number_useless_zero);
  Serial.print("O valor INICIAL de x é "); Serial.print(x_value_now); Serial.print(" escrito em "); Serial.print(ch_x); Serial.println(" (pixels)");
  number_useless_zero = number_of_useless_zero(scale_final_value_x, number_of_decimal_places_x);
  real_word_width = t_size * ch_width * real_number_of_ch(number_of_decimal_places_x, number_useless_zero);
  Serial.print("O comprimento do ultimo ponto em pixels é "); Serial.println(real_word_width);
  uint8_t ch_x_to_final_value = (x0 + scale_width) - (real_word_width / 2) - 1;
  Serial.print("O ultimo ponto comeca a ser escrito em "); Serial.println(ch_x_to_final_value);
  if (width_of_the_interval(x0 + scale_width, scale_initial_value_x, scale_step_x, scale_final_value_x, number_of_decimal_places_x) <= scale_width) { //there is enough space in the axy x to plot every points
    if (scale_initial_value_x < 0 && scale_final_value_x > 0) { //draw the axies y (where the x value is 0)
      ch_x = x0 + ((0 - scale_initial_value_x) / (scale_final_value_x - scale_initial_value_x)) * scale_width; //the pixel that will be plotted is calculated in percentage of the lenght of the axies x add the initial point
      tft.drawFastVLine(ch_x, dotted_line_begin, (dotted_line_end - dotted_line_begin), color_2);
    }
    do {
      x_value_now = truncate(x_value_now + scale_step_x, number_of_decimal_places_x);

      if (x_value_now >= scale_final_value_x) { //the last value to print
        x_value_now = truncate(scale_final_value_x, number_of_decimal_places_x);
      }

      if (ch_x + (real_word_width / 2) >= ch_x_to_final_value) { //when the biggest value lower than the final value and the final value are printed in the same pixels
        number_useless_zero = number_of_useless_zero(scale_final_value_x, number_of_decimal_places_x);
        real_word_width = t_size * ch_width * real_number_of_ch(number_of_decimal_places_x, number_useless_zero);
        if ((width - 3) - (x0 + scale_width) >= real_word_width) { //this is when we can write the last value after the graph space
          ch_x = x0 + scale_width + 1 + real_word_width / 2;
          ch_y = y0 - ch_height;
        } else {
          ch_x = x0 + scale_width;
          ch_y = ch_y + ch_height + 2;  //this is when we have to give a "enter" in the last number,
        }
      } else { //if not, keep increasing the ch_x as usual
        ch_x = x0 + ((x_value_now - scale_initial_value_x) / (scale_final_value_x - scale_initial_value_x)) * scale_width; //the pixel that will be plotted is calculated in percentage of the lenght of the axies x add the initial point
      }

      number_useless_zero = number_of_useless_zero(x_value_now, number_of_decimal_places_x);
      real_word_width = t_size * ch_width * real_number_of_ch(number_of_decimal_places_x, number_useless_zero);
      if (ch_x < x0 + scale_width) {
        tft.drawFastVLine(ch_x, y0, -4, color_2); //plot the litle straight in the below rectangle part
        draw_Dotted_Lines_X(dotted_lines_x, ch_x, dotted_line_begin, dotted_line_end);
      }
      tft.setCursor(ch_x - (real_word_width / 2) + 1, ch_y); //+1 was testing
      tft.print(abs(x_value_now), number_of_decimal_places_x - number_useless_zero);

      Serial.print("O valor de x é "); Serial.print(x_value_now, 6); Serial.print(" escrito em "); Serial.print(ch_x); Serial.println(" (pixels)");
      //return 1;
    } while (x_value_now < scale_final_value_x);

    if (size_error_x == true && show_error == -1) { //if there was the size error in the scale, print this advertising message to the user but just if it is in the preview screen
      tft.setTextColor(color_1, color_2);
      ch_x = x0 + 2;
      ch_y = dotted_line_begin + (dotted_line_end - dotted_line_begin) / 2; //print in the middle of the graph
      tft.setCursor(ch_x, ch_y); tft.print("The previously options for x dont");
      ch_y = ch_y + ch_height + 1;
      tft.setCursor(ch_x, ch_y); tft.print("fit. The  system  calculated  the");
      ch_y = ch_y + ch_height + 1;
      tft.setCursor(ch_x, ch_y); tft.print("best one. If you dont accept this");
      ch_y = ch_y + ch_height + 1;
      tft.setCursor(ch_x, ch_y); tft.print("close the preview and change back");
    }
    size_error_x = 0;
  } else { //when the number of points that fit in the inverval are lower than the maxim, find the step closest to that own setted by user. If it don't fit, means that the step is very low, so must be increased.
    size_error_x = 1;
    scale_step_x = scale_step_x + 0.05;
    print_Scale_X(x0, y0, scale_width, dotted_line_begin, dotted_line_end, show_error);
  }
}

void draw_graph_Space(uint8_t x0, uint16_t y0, uint8_t g_space_width, uint16_t g_space_height) {
  //use the function drawRect it was not good. IDK why
  //x0 and y0 are the below left point of the rectangle
  tft.fillRect(x0, y0, g_space_width, g_space_height, WHITE);
  tft.drawFastHLine(x0, y0, g_space_width, color_2);
  tft.drawFastVLine(x0 + g_space_width, y0, g_space_height, color_2);
  tft.drawFastHLine(x0 + g_space_width, y0 + g_space_height, -g_space_width, color_2);
  tft.drawFastVLine(x0, y0 + g_space_height, -g_space_height, color_2);
}

uint16_t width_of_the_interval(uint8_t total_width, float initial_value, float step_value, float final_value, uint8_t decimal_places) {
  uint8_t min_dist_bt_end_and_begin_word = 3; //the minimum quantity of pixels between the end of the arrow and the begining of the next one
  float max_points = 0;
  uint8_t number_useless_zero = 0;
  uint8_t real_word_width = 0;
  float value_now = initial_value;
  number_useless_zero = number_of_useless_zero(initial_value, decimal_places);
  real_word_width = 1 * ch_width * real_number_of_ch(number_of_decimal_places_x, number_useless_zero);
  uint16_t number_of_pixels_used = real_word_width / 2; //the first point the number of pixels used is divided by 2 as it is in the midle of the number
  do {
    value_now = truncate(value_now + step_value, decimal_places);
    number_useless_zero = number_of_useless_zero(value_now, decimal_places);
    real_word_width = 1 * ch_width * real_number_of_ch(number_of_decimal_places_x, number_useless_zero);
    if (value_now >= final_value) { //the last value to print
      break; //the last value in the axies x is already calculate to know where it should be ploted. So, there is no need to consider the pixels of the last number
    }
    number_of_pixels_used = number_of_pixels_used + real_word_width;
  } while (value_now < final_value);

  number_of_pixels_used = number_of_pixels_used + (min_dist_bt_end_and_begin_word * (number_of_points(scale_initial_value_x, scale_step_x, scale_final_value_x) - 1));
  return number_of_pixels_used;
}

void print_Points(uint8_t x0, uint16_t y0, uint8_t scale_width, uint16_t scale_height, uint8_t ordinary_sensor) {
  float *voltage_vector = create_Voltage_Vector();
  uint16_t p_x = x0 + ((voltage_vector[0] - scale_initial_value_x) / (scale_final_value_x - scale_initial_value_x)) * scale_width; //coordinate in the axies x for the point
  uint16_t p_y = 160; //coordinate in the axies y for the point
  uint16_t p_x_old = 0;
  uint16_t p_y_old = 160;
  uint8_t counter = 0;
  for (counter = 0; counter < number_of_points(initial_Voltage_Value, signal_Step, final_Voltage_Value); counter++) {
    p_x_old = p_x;
    p_x = x0 + ((voltage_vector[counter] - scale_initial_value_x) / (scale_final_value_x - scale_initial_value_x)) * scale_width;
    if (p_x >= x0 && p_x <= x0 + scale_width) { //if it is out of this interval, can't be plotted and the scale must be adjusted
      if (p_x_old >= x0 && p_x_old <= x0 + scale_width) {
        tft.drawLine(p_x_old + 2, p_y_old, p_x, p_y, color_2); //This will join the points to form a curve
      }
      create_Point_Square(p_x, p_y);
      Serial.print("O valor do ponto x antigo é "); Serial.println(p_x_old);
    }
    Serial.print("O valor em pixels do ponto eh "); Serial.println(p_x);
  }
}

void create_Point_Square(uint8_t x, uint16_t y) {
  tft.drawPixel(x, y, color_1);
  tft.drawPixel(x + 1, y, color_1);
  tft.drawPixel(x, y - 1, color_1);
  tft.drawPixel(x + 1, y - 1, color_1);
}

void draw_Dotted_Lines_X (char *activated_or_disabled, float x, uint16_t initial_y, uint16_t final_y) {
  if (activated_or_disabled == "YES") {
    uint8_t step_bt_pixels = 5;
    uint16_t y_now = initial_y;
    tft.drawFastVLine(x, initial_y, 4, color_2); //plot the litle straight in the above rectangle part
    do { //plot the litle pixels to help see the scale
      tft.drawPixel(x, y_now, color_2);
      y_now = y_now + step_bt_pixels;
    } while (y_now <= final_y);
  }
}

/*Action: will show the screen results of the corresponding sensor
  Parameters:
             -> sensor_number: it represents the ordinary number of the sensor selected to be measured. So, if it is the first, second, third, fourth, fifth, sixth and so on.
  Return: -
*/
void screen_Results(uint8_t sensor_number) {
  uint8_t number_of_symbols, left_arrow_position, right_arrow_position, humidity_position, temperature_position, battery_position;
  if (sensor_number == 1) { //the first measurement
    if (number_of_measurements() == 1) { //if there is just one measurement, it isn't necessary to create the button for next, as there is not another measurement
      number_of_symbols = 3; left_arrow_position = 0; right_arrow_position = 0; humidity_position = 1; temperature_position = 3; battery_position = 2;
      notific_Bar(number_of_symbols, left_arrow_position, NULL, humidity_position, temperature_position, battery_position, right_arrow_position, NULL);
    } else { //if the number of measurements is bigger than 1, it is necessary to create the bar with the arrow of next
      number_of_symbols = 4; left_arrow_position = 0; right_arrow_position = 4; humidity_position = 1; temperature_position = 3; battery_position = 2;
      notific_Bar(number_of_symbols, left_arrow_position, NULL, humidity_position, temperature_position, battery_position, right_arrow_position, "NEXT");
    }
    print_Text(3, 0, sensor_number, 0);
    create_button(3, 0); //Create the MENU button
    graph(sensor_number);

  }  else if (sensor_number == number_of_measurements()) { //the last measurement
    number_of_symbols = 4; left_arrow_position = 1; right_arrow_position = 0; humidity_position = 2; temperature_position = 3; battery_position = 4;

    notific_Bar(number_of_symbols, left_arrow_position, "BACK", humidity_position, temperature_position, battery_position, right_arrow_position, NULL);
    print_Text(3, 0, sensor_number, 0);
    create_button(3, 0); //Create the menu button
    graph(sensor_number);
  } else if (sensor_number > 1 && sensor_number < number_of_measurements()) { //the measurement that are not neither the first either the last
    number_of_symbols = 5; left_arrow_position = 1; right_arrow_position = 5; humidity_position = 2; temperature_position = 3; battery_position = 4;

    notific_Bar(number_of_symbols, left_arrow_position, "BACK", humidity_position, temperature_position, battery_position, right_arrow_position, "NEXT");
    print_Text(3, 0, sensor_number, 0);
    create_button(3, 0); //Create the menu button
    graph(sensor_number);
  }
}

/*Action: discover how much measurements should be done
  Parameters: -
  Return: -
*/
uint8_t number_of_measurements() {
  uint8_t counter = 0;
  uint8_t quantitity = 0;
  for (counter = 1; counter <= 14; counter++) {
    if (sensor_counter[counter] != 0) {
      quantitity++;
    }
  }
  return quantitity;
}

/*Action: discover which sensor should be measured (first, second, third, fourth...)
  Parameters:
             ordinary_sensor_number: first until fourteenth
  Return: -
*/
uint8_t which_sensor_to_be_measured(uint8_t ordinary_sensor_number) {
  uint8_t sensor_to_be_measured = 0;
  uint8_t counter = 0;
  uint8_t counter_2 = 1;
  for (counter = 1; counter <= 14; counter++) {
    if (sensor_counter[counter] != 0) {
      sensor_to_be_measured = counter;
      if (counter_2 == ordinary_sensor_number) {
        break;
      }
      counter_2++;
    }
  }
  return sensor_to_be_measured;
}

void create_Yes_No_Buttons(char *which_word, boolean disable, char  *axies) {
  uint8_t r = 3;
  uint8_t t_size = 3; //This will be always +1 bigger than the real character size to centralize the text in the rectangle
  uint8_t word_width = ch_width * (t_size - 1) * strlen(which_word);
  uint16_t y0 = 269;
  uint8_t r_width = (word_width) + (0.2) * word_width;
  uint8_t x0 = 0;
  uint8_t r_height =  (ch_height * t_size) + t_size;
  uint8_t columns = 0; //conferir se vai ser 50 aqui msm
  uint8_t c_x0 = 0;
  uint16_t c_y0 = 0;
  if (axies == "X") { //AXIES X
    x0 = (width / 2) - (width / 4) - (r_width / 2);
    if (which_word == "NO") { //THE WORD NO IN THE AXIES X
      columns = 19;
      if (disable == false) {
        dotted_lines_x = "NO";
        tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
        tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

        M_Button[0][columns] = x0;
        M_Button[1][columns] = x0 + r_width;
        M_Button[2][columns] = y0;
        M_Button[3][columns] = y0 + r_height;
      
        c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(which_word))) / 2; //centralizar em x o texto
        c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

        tft.setTextSize(t_size - 1);
        tft.setTextColor(color_1);
        tft.setCursor(c_x0, c_y0);
        tft.print(which_word);
      } else if (disable == true) {
        tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
        M_Button[0][columns] = 0;
        M_Button[1][columns] = 0;
        M_Button[2][columns] = 0;
        M_Button[3][columns] = 0;
      }
    } else if (which_word == "YES") { //THE WORD "YES" IN THE AXIES X
      dotted_lines_x = "YES";
      columns = 18;
      if (disable == false) {
        tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
        tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

        M_Button[0][columns] = x0;
        M_Button[1][columns] = x0 + r_width;
        M_Button[2][columns] = y0;
        M_Button[3][columns] = y0 + r_height;
      
        c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(which_word))) / 2; //centralizar em x o texto
        c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

        tft.setTextSize(t_size - 1);
        tft.setTextColor(color_1);
        tft.setCursor(c_x0, c_y0);
        tft.print(which_word);
      } else if (disable == true) {
        tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
        M_Button[0][columns] = 0;
        M_Button[1][columns] = 0;
        M_Button[2][columns] = 0;
        M_Button[3][columns] = 0;
      }
    }
  } else if (axies == "Y") { //AXIES Y
    x0 = (width / 2) + (width / 4) - (r_width / 2);
    if (which_word == "NO") { //THE WORD NO IN THE AXIES Y
      columns = 21;
      if (disable == false) {
        dotted_lines_y = "NO";
        tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
        tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

        M_Button[0][columns] = x0;
        M_Button[1][columns] = x0 + r_width;
        M_Button[2][columns] = y0;
        M_Button[3][columns] = y0 + r_height;
      
        c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(which_word))) / 2; //centralizar em x o texto
        c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

        tft.setTextSize(t_size - 1);
        tft.setTextColor(color_1);
        tft.setCursor(c_x0, c_y0);
        tft.print(which_word);
      } else if (disable == true) {
        tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
        M_Button[0][columns] = 0;
        M_Button[1][columns] = 0;
        M_Button[2][columns] = 0;
        M_Button[3][columns] = 0;
      }
    } else if (which_word == "YES") { //THE WORD "YES" IN THE AXIES Y
      columns = 20; //conferir se vai ser 51 aqui msm
      if (disable == false) {
        dotted_lines_y = "YES";
        tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
        tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

        if (M_Button[0][columns] == 0) { //just to save one time
          M_Button[0][columns] = x0;
          M_Button[1][columns] = x0 + r_width;
          M_Button[2][columns] = y0;
          M_Button[3][columns] = y0 + r_height;
        }
        c_x0 = x0 + (r_width - ((t_size - 1) * ch_width * strlen(which_word))) / 2; //centralizar em x o texto
        c_y0 = y0 + (r_height - (t_size - 1) * ch_height) / 2; //centralizar em y o texto que está sendo escrito

        tft.setTextSize(t_size - 1);
        tft.setTextColor(color_1);
        tft.setCursor(c_x0, c_y0);
        tft.print(which_word);
      } else if (disable == true) {
        tft.fillRoundRect(x0, y0, r_width, r_height, r, bg_color);
        M_Button[0][columns] = 0;
        M_Button[1][columns] = 0;
        M_Button[2][columns] = 0;
        M_Button[3][columns] = 0;
      }
    }
  }
}

void create_Down_Arrow(uint8_t settings_page) {
  //creating the background rectangle
  uint8_t distance_bt_edge_and_arrow = 0;
  uint8_t r_width = 0;
  uint8_t r = 0;
  uint8_t r_height = 0;
  uint8_t x0 = 0;
  uint16_t y0 = 0;
  //creating the inside (a)rrow
  uint8_t a_x0 = 0;
  uint16_t a_y0 = 0;
  uint8_t a_x1 = 0;
  uint16_t a_y1 = 0;
  uint8_t a_x2 = 0;
  uint16_t a_y2 = 0;
  uint8_t columns = 0;
  if (settings_page == 1) {
    distance_bt_edge_and_arrow = 5; //distance between the edge of the circle and the arrow
    r_width = 35;
    r = r_width / 2;
    r_height = r_width;
    x0 = (width / 2) - (square_settings_width / 2) - 5 - r_width; //5 was a difinied value
    y0 = print_Text(5, 0, 0, 0) + 2;

    a_x0 =  x0 + r_width / 2;
    a_y0 = y0 + r_height - distance_bt_edge_and_arrow;
    a_x1 = x0 + distance_bt_edge_and_arrow;
    a_y1 = y0 + 2 * distance_bt_edge_and_arrow;
    a_x2 = x0 + r_width - distance_bt_edge_and_arrow - 1;
    a_y2 = a_y1;
    columns = 2;

    for (uint8_t counter = 0; counter <= 3; counter++) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

      M_Button[0][columns] = x0;
      M_Button[1][columns] = x0 + r_width;
      M_Button[2][columns] = y0;
      M_Button[3][columns] = y0 + r_height;
      columns = columns + 2;
    
      tft.drawLine(a_x0, a_y0, a_x1, a_y1, color_1);
      tft.drawLine(a_x0, a_y0 - 1, a_x1 + 1, a_y1, color_1);
      tft.drawLine(a_x0, a_y0 - 2, a_x1 + 2, a_y1, color_1);

      tft.drawLine(a_x0, a_y0, a_x2, a_y2, color_1);
      tft.drawLine(a_x0, a_y0 - 1, a_x2 - 1, a_y2, color_1);
      tft.drawLine(a_x0, a_y0 - 2, a_x2 - 2, a_y2, color_1);
      y0 = y0 + r_height + 2 + (2 * ch_height) + 3;
      a_y0 = y0 + r_height - distance_bt_edge_and_arrow;
      a_y1 = y0 + 2 * distance_bt_edge_and_arrow;
      a_y2 = a_y1;
    }
  } else if (settings_page == 2) {
    distance_bt_edge_and_arrow = 5; //distance between the edge of the circle and the arrow
    r_width = 25;
    r = r_width / 2;
    r_height = r_width;
    x0 = 5; //8 was a difinied value
    y0 = 93; //definied
    columns = 2;

    for (uint8_t counter = 1; counter <= 4; counter++) {
      for (uint8_t counter2 = 1; counter2 <= 2; counter2++) {
        a_x0 =  x0 + r_width / 2;
        a_y0 = y0 + r_height - distance_bt_edge_and_arrow;
        a_x1 = x0 + distance_bt_edge_and_arrow;
        a_y1 = y0 + 2 * distance_bt_edge_and_arrow;
        a_x2 = x0 + r_width - distance_bt_edge_and_arrow - 1;
        a_y2 = a_y1;
        tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
        tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

        M_Button[0][columns] = x0;
        M_Button[1][columns] = x0 + r_width;
        M_Button[2][columns] = y0;
        M_Button[3][columns] = y0 + r_height;
        columns = columns + 2;
      
        tft.drawLine(a_x0, a_y0, a_x1, a_y1, color_1);
        tft.drawLine(a_x0, a_y0 - 1, a_x1 + 1, a_y1, color_1);
        tft.drawLine(a_x0, a_y0 - 2, a_x1 + 2, a_y1, color_1);

        tft.drawLine(a_x0, a_y0, a_x2, a_y2, color_1);
        tft.drawLine(a_x0, a_y0 - 1, a_x2 - 1, a_y2, color_1);
        tft.drawLine(a_x0, a_y0 - 2, a_x2 - 2, a_y2, color_1);

        x0 = (width / 2) + 5;
      }
      x0 = 5;
      y0 = y0 + r_height + 2 + (2 * ch_height) + 3;
    }
  }

}

void create_Up_Arrow(uint8_t settings_page) {
  //creating the background rectangle
  uint8_t distance_bt_edge_and_arrow = 0;
  uint8_t x0 = 0;
  uint16_t y0 = 0;
  uint8_t r_width = 0;
  uint8_t r_height = 0;
  uint8_t r = 0;
  //creating the inside (a)rrow
  uint8_t a_x0 = 0;
  uint16_t a_y0 = 0;
  uint8_t a_x1 = 0;
  uint16_t a_y1 = 0;
  uint8_t a_x2 = 0;
  uint16_t a_y2 = 0;
  uint8_t columns = 0;
  if (settings_page == 1) {
    distance_bt_edge_and_arrow = 5;
    x0 = (width / 2) + (square_settings_width / 2) + 5; //5 was a difinied value
    y0 = print_Text(5, 0, 0, 0) + 2;
    r_width = 35; //definied
    r_height = r_width; //definied
    r = r_width / 2;
    //creating the inside (a)rrow
    a_x0 = x0 + r;
    a_y0 = y0 + distance_bt_edge_and_arrow;
    a_x1 = x0 + distance_bt_edge_and_arrow;
    a_y1 = y0 + r_height - 2 * distance_bt_edge_and_arrow;
    a_x2 = x0 + r_width - distance_bt_edge_and_arrow - 1;
    a_y2 = a_y1;
    columns = 3;

    for (uint8_t counter = 0; counter <= 3; counter++) {
      tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
      tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

      M_Button[0][columns] = x0;
      M_Button[1][columns] = x0 + r_width;
      M_Button[2][columns] = y0;
      M_Button[3][columns] = y0 + r_height;
      columns = columns + 2;
   

      tft.drawLine(a_x0, a_y0, a_x1, a_y1, color_1);
      tft.drawLine(a_x0, a_y0 + 1, a_x1 + 1, a_y1, color_1);
      tft.drawLine(a_x0, a_y0 + 2, a_x1 + 2, a_y1, color_1);

      tft.drawLine(a_x0, a_y0, a_x2, a_y2, color_1);
      tft.drawLine(a_x0, a_y0 + 1, a_x2 - 1, a_y2, color_1);
      tft.drawLine(a_x0, a_y0 + 2, a_x2 - 2, a_y2, color_1);
      y0 = y0 + r_height + 2 + (2 * ch_height) + 3;
      a_y0 = y0 + distance_bt_edge_and_arrow;
      a_y1 = y0 + r_height - 2 * distance_bt_edge_and_arrow;
      a_y2 = a_y1;
    }
  } else if (settings_page == 2) {
    distance_bt_edge_and_arrow = 5;
    x0 = (width / 2) - (width / 4) + (square_settings2_width / 2) + 5; //5 was a difinied value
    y0 = 93;
    r_width = 25; //definied
    r_height = r_width; //definied
    r = r_width / 2;
    columns = 3;
    for (uint8_t counter = 1; counter <= 4; counter++) {
      for (uint8_t counter2 = 1; counter2 <= 2; counter2++) {
        a_x0 = x0 + r;
        a_y0 = y0 + distance_bt_edge_and_arrow;
        a_x1 = x0 + distance_bt_edge_and_arrow;
        a_y1 = y0 + r_height - 2 * distance_bt_edge_and_arrow;
        a_x2 = x0 + r_width - distance_bt_edge_and_arrow - 1;
        a_y2 = a_y1;
        tft.fillRoundRect(x0, y0, r_width, r_height, r, color_2);
        tft.drawRoundRect(x0, y0, r_width, r_height, r, color_1);

        M_Button[0][columns] = x0;
        M_Button[1][columns] = x0 + r_width;
        M_Button[2][columns] = y0;
        M_Button[3][columns] = y0 + r_height;
        columns = columns + 2;
        
        tft.drawLine(a_x0, a_y0, a_x1, a_y1, color_1);
        tft.drawLine(a_x0, a_y0 + 1, a_x1 + 1, a_y1, color_1);
        tft.drawLine(a_x0, a_y0 + 2, a_x1 + 2, a_y1, color_1);

        tft.drawLine(a_x0, a_y0, a_x2, a_y2, color_1);
        tft.drawLine(a_x0, a_y0 + 1, a_x2 - 1, a_y2, color_1);
        tft.drawLine(a_x0, a_y0 + 2, a_x2 - 2, a_y2, color_1);

        x0 = (width / 2) + (width / 4) + (square_settings2_width / 2) + 5;
      }
      x0 = (width / 2) - (width / 4) + (square_settings2_width / 2) + 5; //5 was a difinied value
      y0 = y0 + r_height + 2 + (2 * ch_height) + 3;

    }
  }

}

uint16_t create_Rectangles_Settings(uint8_t settings_page) {
  uint8_t r_width = 0;
  uint8_t r_height = 0;
  uint8_t x0 = 0;
  uint16_t y0 = 0;
  uint8_t t_size = 2;
  uint8_t counter = 0;
  if (settings_page == 1) {
    r_width = square_settings_width;
    r_height = 35; //definied
    x0 = (width / 2) - (r_width / 2);
    y0 = print_Text(5, 0, 0, 0) + 2; //2 is just to stay a little behind the text above
    for (counter = 0; counter <= 3; counter++) {
      tft.fillRect(x0, y0, r_width, r_height, color_2);
      tft.drawRect(x0, y0, r_width, r_height, color_1);
      y0 = y0 + r_height + 2 + (t_size * ch_height) + 3;
    }
  } else if (settings_page == 2) {
    r_width = square_settings2_width;
    r_height = 25;
    y0 = 93;
    for (counter = 1; counter <= 4; counter++) {
      x0 = (width / 2) - (width / 4) - (r_width / 2);
      tft.fillRect(x0, y0, r_width, r_height, color_2);
      tft.drawRect(x0, y0, r_width, r_height, color_1);
      x0 = (width / 2) + (width / 4) - (r_width / 2);
      tft.fillRect(x0, y0, r_width, r_height, color_2);
      tft.drawRect(x0, y0, r_width, r_height, color_1);
      y0 = y0 + r_height + 2 + (t_size * ch_height) + 3;
    }
  }
  return y0;
}

void create_Buttons_Settings(uint8_t settings_page) {
  create_Down_Arrow(settings_page);
  create_Up_Arrow(settings_page);
}

void screen_Settings(uint8_t ordinary_number_of_the_page) { //to see if it's the first settings page, or the second and so on
  uint8_t number_of_symbols, left_arrow_position, right_arrow_position, humidity_position, temperature_position, battery_position;

  if (ordinary_number_of_the_page == 1) {
    number_of_symbols = 5; left_arrow_position = 1; right_arrow_position = 5; humidity_position = 2; temperature_position = 3; battery_position = 4;
    notific_Bar(number_of_symbols, left_arrow_position, "BACK", humidity_position, temperature_position, battery_position, right_arrow_position, "NEXT");
    print_Text(4, 0, 0, ordinary_number_of_the_page); //print OUTPUT SIGNAL
    print_Text(5, 0, 0, ordinary_number_of_the_page); //print: Final Value (V), Initial Value (V), Signal Step, Integration Time
    create_Rectangles_Settings(ordinary_number_of_the_page); //create the rectangles that you show the results
    create_Buttons_Settings(ordinary_number_of_the_page); //create the up and down arrows to control each parameters
    print_Max_Limit_Voltage(0); //just print the default value for the final_Voltage_Value (5)
    print_Min_Limit_Voltage(0); //just print the default value for the initial_Voltage_Value (0)
    print_Step_Signal(0); //just to print the default value for the signal step (0.2)
    print_Integration_Time(0); //just to print the default value for the integration Time (1)
  } else if (ordinary_number_of_the_page == 2) {
    if (button_clicked == 22) { //In this case, just erase the middle of the screen
      tft.fillRect(1, 60, width - 2, height - 2, bg_color); //erase what was writen
    } else { //in this case, must create all the screen again
      tft.fillScreen(bg_color);
      number_of_symbols = 5; left_arrow_position = 1; right_arrow_position = 5; humidity_position = 2; temperature_position = 3; battery_position = 4;
      notific_Bar(number_of_symbols, left_arrow_position, "BACK", humidity_position, temperature_position, battery_position, right_arrow_position, "MEAS.");
    }
    print_Text(10, 0, 0, ordinary_number_of_the_page);
    print_Text(14, 0, 0, ordinary_number_of_the_page); //print: DECIMAL PLACES NUMBER, SCALE INITIAL VALUE, SCALE STEP, SCALE FINAL VALUE, ACTIVATE DOTTED LINES
    create_Rectangles_Settings(ordinary_number_of_the_page); //create the rectangles that you show the results
    create_Buttons_Settings(ordinary_number_of_the_page); //create the up and down arrows to control each parameters

    print_Decimal_Places(0, 0); //0 = just print the default value, 0=axies x
    print_Decimal_Places(0, 1); //0 = just print the default value, 1=axies y

    print_Scale_Initial_Value(0, 0); //0 = just print the default value, 0=axies x
    print_Scale_Initial_Value(0, 1); //0 = just print the default value, 1=axies y

    print_Scale_Step(0, 0);
    print_Scale_Step(0, 1);

    print_Scale_Final_Value(0, 0);
    print_Scale_Final_Value(0, 1);

    create_Yes_No_Buttons(dotted_lines_x, 0, "X"); //create the button YES, activate in the axies X
    create_Yes_No_Buttons(dotted_lines_y, 0, "Y"); //create the button no, activate in the AXIES Y

    create_button(4, 0); //Create the PREVIEW button
  } else if (ordinary_number_of_the_page == 3) { //open the graph preview
    tft.fillRect(1, 58, width - 2, height - 2, bg_color); //erase what was writen
    create_button(4, 1); //Disable the preview button

    graph(-1); //the -1 means that is just a preview, it don't have the data of any sensor
    create_button(5, 0); //create the cancel button
  }
}

void print_Decimal_Places(uint8_t decrease_or_increase, uint8_t axies) {
  uint8_t maximum_value = 2;
  uint8_t y0 = 93;
  uint8_t x0 = 0;
  uint8_t r_height = 25;
  uint8_t r_width = square_settings2_width;
  uint8_t t_size = 2;
  uint16_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = 0;
  if (axies == 0) { //AXIES X
    x0 = (width / 2) - (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 2) { //DECREASE
      number_of_decimal_places_x--;
      if (number_of_decimal_places_x < 1) { //if it is a too small value, begin of the biggest value
        number_of_decimal_places_x = maximum_value;
      }
    } else if (decrease_or_increase == 3) { //INCREASE
      number_of_decimal_places_x++;
      if (number_of_decimal_places_x > maximum_value) {
        number_of_decimal_places_x = 1; //if it is a too big value, begin from the smallest
      }
    }
    c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 1 / 2);

    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(number_of_decimal_places_x);
  } else if (axies == 1) { //AXIES Y
    x0 = (width / 2) + (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 4) { //DECREASE
      number_of_decimal_places_y--;
      if (number_of_decimal_places_y < 0) { //if it is a too small value, begin of the biggest value
        number_of_decimal_places_y = maximum_value;
      }
    } else if (decrease_or_increase == 5) { //INCREASE
      number_of_decimal_places_y++;
      if (number_of_decimal_places_y > maximum_value) {
        number_of_decimal_places_y = 0; //if it is a too big value, begin from the smallest
      }
    }
    c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 1 / 2);

    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(number_of_decimal_places_y);
  }
}

void print_Scale_Initial_Value(uint8_t decrease_or_increase, uint8_t axies) {
  int8_t maximum_value = -5; //voltage_Level_PinB_MCP; //the minimum value for the scale x is the same value that goes in the pin B of the DP
  float step_for_decreasing_or_increasing = 0.5;
  uint8_t y0 = 137;
  uint8_t x0 = 0;
  uint8_t r_height = 25;
  uint8_t r_width = square_settings2_width;
  uint8_t t_size = 2;
  uint16_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = 0;
  if (axies == 0) { //AXIES X
    x0 = (width / 2) - (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 6) { //DECREASE
      scale_initial_value_x = scale_initial_value_x - step_for_decreasing_or_increasing;
      if (scale_initial_value_x < maximum_value) { //if it is a too small value, begin of the biggest value
        scale_initial_value_x = -maximum_value;
      }
    } else if (decrease_or_increase == 7) { //INCREASE
      scale_initial_value_x = scale_initial_value_x + step_for_decreasing_or_increasing;
      if (scale_initial_value_x > -maximum_value) {
        scale_initial_value_x = maximum_value; //if it is a too big value, begin from the smallest
      }
    }
    if (scale_initial_value_x < 0) { // this both if are just to centralize the text in the rectangle as when it is negative there another character (-)
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 4 / 2);
    } else if (scale_initial_value_x >= 0) {
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 3 / 2);
    }

    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(scale_initial_value_x, 1);
  } else if (axies == 1) { //AXIES Y
    x0 = (width / 2) + (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 8) { //DECREASE
      scale_initial_value_y = scale_initial_value_y - step_for_decreasing_or_increasing;
      if (scale_initial_value_y < maximum_value) { //if it is a too small value, begin of the biggest value
        scale_initial_value_y = maximum_value;
      }
    } else if (decrease_or_increase == 9) { //INCREASE
      scale_initial_value_y = scale_initial_value_y + step_for_decreasing_or_increasing;
      if (scale_initial_value_y > -maximum_value) {
        scale_initial_value_y = maximum_value; //if it is a too big value, begin from the smallest
      }
    }
    if (scale_initial_value_y < 0) { // this both if are just to centralize the text in the rectangle as when it is negative there another character (-)
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 4 / 2);
    } else if (scale_initial_value_y >= 0) {
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 3 / 2);
    }
    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(scale_initial_value_y, 1);
  }
}

void print_Scale_Step(uint8_t decrease_or_increase, uint8_t axies) {
  int8_t maximum_value = 3; //think better on this
  float step_for_decreasing_or_increasing_x = 0.05;
  float step_for_decreasing_or_increasing_y = 0.10;
  uint8_t y0 = 181;
  uint8_t x0 = 0;
  uint8_t r_height = 25;
  uint8_t r_width = square_settings2_width;
  uint8_t t_size = 2;
  uint16_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = 0;
  if (axies == 0) { //AXIES X
    x0 = (width / 2) - (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 10) { //DECREASE
      scale_step_x = scale_step_x - step_for_decreasing_or_increasing_x;
      if (scale_step_x < step_for_decreasing_or_increasing_x) { //if it is a too small value, begin of the biggest value
        scale_step_x = maximum_value;
      }
    } else if (decrease_or_increase == 11) { //INCREASE
      scale_step_x = scale_step_x + step_for_decreasing_or_increasing_x;
      if (scale_step_x > maximum_value) {
        scale_step_x = step_for_decreasing_or_increasing_x; //if it is a too big value, begin from the smallest
      }
    }
    Serial.print("O valor do step da escala no eixo x é ");
    Serial.println(scale_step_x, 5);
    c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 4 / 2) + 2;

    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(scale_step_x, 2);
  } else if (axies == 1) { //AXIES Y
    x0 = (width / 2) + (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 12) { //DECREASE
      scale_step_y = scale_step_y - step_for_decreasing_or_increasing_y;
      if (scale_step_y < step_for_decreasing_or_increasing_y) { //if it is a too small value, begin of the biggest value
        scale_step_y = maximum_value;
      }
    } else if (decrease_or_increase == 13) { //INCREASE
      scale_step_y = scale_step_y + step_for_decreasing_or_increasing_y;
      if (scale_step_y > maximum_value) {
        scale_step_y = step_for_decreasing_or_increasing_y; //if it is a too big value, begin from the smallest
      }
    }
    c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 3 / 2);
    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(scale_step_y, 1);
  }
}

void print_Scale_Final_Value(uint8_t decrease_or_increase, uint8_t axies) {
  int8_t maximum_value = 5; //voltage_Level_PinB_MCP; //the minimum value for the scale x is the same value that goes in the pin B of the DP
  float step_for_decreasing_or_increasing = 0.5;
  uint8_t y0 = 225;
  uint8_t x0 = 0;
  uint8_t r_height = 25;
  uint8_t r_width = square_settings2_width;
  uint8_t t_size = 2;
  uint16_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = 0;
  if (axies == 0) { //AXIES X
    x0 = (width / 2) - (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 14) { //DECREASE
      scale_final_value_x = scale_final_value_x - step_for_decreasing_or_increasing;
      if (scale_final_value_x < -maximum_value) { //if it is a too small value, begin of the biggest value
        scale_final_value_x = maximum_value;
      }
    } else if (decrease_or_increase == 15) { //INCREASE
      scale_final_value_x = scale_final_value_x + step_for_decreasing_or_increasing;
      if (scale_final_value_x > maximum_value) {
        scale_final_value_x = -maximum_value; //if it is a too big value, begin from the smallest
      }
    }
    if (scale_final_value_x < 0) { // this both if are just to centralize the text in the rectangle as when it is negative there another character (-)
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 4 / 2);
    } else if (scale_final_value_x >= 0) {
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 3 / 2);
    }

    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(scale_final_value_x, 1);
  } else if (axies == 1) { //AXIES Y
    x0 = (width / 2) + (width / 4) - (square_settings2_width / 2);
    tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
    if (decrease_or_increase == 16) { //DECREASE
      scale_final_value_y = scale_final_value_y - step_for_decreasing_or_increasing;
      if (scale_final_value_y < -maximum_value) { //if it is a too small value, begin of the biggest value
        scale_final_value_y = maximum_value;
      }
    } else if (decrease_or_increase == 17) { //INCREASE
      scale_final_value_y = scale_final_value_y + step_for_decreasing_or_increasing;
      if (scale_final_value_y > maximum_value) {
        scale_final_value_y = -maximum_value; //if it is a too big value, begin from the smallest
      }
    }
    if (scale_final_value_y < 0 || scale_final_value_y >= 10) { // this both if are just to centralize the text in the rectangle as when it is negative there another character (-)
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 4 / 2);
    } else if (scale_final_value_y >= 0 && scale_final_value_y < 10) {
      c_x0 = x0 + (r_width / 2) - (t_size * ch_width * 3 / 2);
    }
    tft.setCursor(c_x0, c_y0);
    tft.setTextSize(t_size);
    tft.setTextColor(color_1);
    tft.print(scale_final_value_y, 1);
  }
}

/*Action: choose which page should be created by knowing which button was selected.
  Parameters: -
  Return: -
*/
void choose_page() {
  Serial.print("O botao clicado foi B"); Serial.print(button_clicked); Serial.print(" da página "); Serial.println(current_page);
  //--------------------- PAGES FOR MEASURE BUTTON ------------------------------------
  //fazer uma funcao q sempre q a pagina antiga for diferente da pagina nova tem q fillscreen, zerar matriz botões... -> economizar linhas de código
  if (current_page == 0) { //It's in the MENU PAGE
    if (button_clicked == 0) { //MEASURE
      tft.fillScreen(bg_color);
      reset_M_Button();
      current_page++;
      screen_Measure();
    } else if(button_clicked == 1){ //SETTINGS
      tft.fillScreen(bg_color);
      current_page = 17;
      reset_M_Button();
      screen_Settings(1);
    } else if (button_clicked == 2){ //SD CARD
      
    } else if(button_clicked == 3){ //INFORMATIONS
      
    }
  } else if (current_page == 1) { //It's in the MEASURE page
    if (button_clicked == 0) { //MENU
      reset_M_Button();
      tft.fillScreen(bg_color);
      current_page--;
      screen_Menu();
    } else if (button_clicked >= 1 && button_clicked <= 14) { //some sensor
      selected_sensors();
      if (number_of_measurements() > 0 && start_already_created == false) { //create the START button just when there is some sensor selected
        create_button(0, 0);
        start_already_created = true;
      } else if (number_of_measurements() == 0){
        create_button(0, 1);
        start_already_created = false;
      }
    } else if(button_clicked == 15){ //START
      tft.fillScreen(bg_color);
      current_page++;
      reset_M_Button();
      start_already_created = false;
      screen_Loading();      
      generate_Output_Signal();
      progress_bar(100);
    }
  } else if (current_page == 2){ //It's in the loading page
    if(button_clicked == 0){ //CANCEL
      
    } else if(button_clicked == 1){ //SEE RESULTS
      tft.fillScreen(bg_color);
      reset_M_Button();
      current_page++;
      screen_Results(which_sensor);      
    }
  } else if (current_page >=3 && current_page <= 16){ //RESULTS
    if (button_clicked == 0){ //BACK
      tft.fillScreen(bg_color);
      reset_M_Button();
      current_page--;
      which_sensor--;
      screen_Results(which_sensor);
    } else if(button_clicked == 1){ //NEXT
      tft.fillScreen(bg_color);
      reset_M_Button();
      current_page++;
      which_sensor++;
      screen_Results(which_sensor);
    } else if(button_clicked == 2){ //MENU
      reset_M_Button();
      reset_sensor_counter(); 
      tft.fillScreen(bg_color);
      current_page = 0;
      which_sensor = 1;
      screen_Menu();      
    }
  } else if (current_page == 17){ //Settings 1
    if (button_clicked == 0){ //BACK
        reset_M_Button();
        reset_sensor_counter(); 
        tft.fillScreen(bg_color);
        current_page = 0;
        which_sensor = 1;
        screen_Menu();
    } else if (button_clicked == 1){ //NEXT
        reset_M_Button();
        tft.fillScreen(bg_color);
        current_page = 18;
        screen_Settings(2);      
    } else if (button_clicked == 2 || button_clicked == 3) { //Increase or decrease the Max Voltage
        print_Max_Limit_Voltage(button_clicked);
    } else if (button_clicked == 4 || button_clicked == 5) { //Increase or decrease the Min Voltage
        print_Min_Limit_Voltage(button_clicked);
    } else if (button_clicked == 6 || button_clicked == 7) { //Increase or decrease the Step Signal
        print_Step_Signal(button_clicked);
    } else if (button_clicked == 8 || button_clicked == 9) { //Increase or decrease the integration time
        print_Integration_Time(button_clicked);
    }
  } else if(current_page == 18){
    if (button_clicked == 0){ //BACK
        reset_M_Button();
        tft.fillScreen(bg_color);
        current_page = 17;
        screen_Settings(1);
    } else if(button_clicked == 1){ //MEASUREMENT
        reset_M_Button();
        tft.fillScreen(bg_color);
        current_page = 1;
        screen_Measure();
    }else if (button_clicked == 2 || button_clicked == 3) { //Increase or decrease the decimal places in axies X
        print_Decimal_Places(button_clicked, 0);
    } else if (button_clicked == 4 || button_clicked == 5) { //Increase or decrease the decimal places in axies Y
        print_Decimal_Places(button_clicked, 1);
    } else if (button_clicked == 6 || button_clicked == 7) { //Increase or decrease the Initial Value of the Scale in axies X
        print_Scale_Initial_Value(button_clicked, 0);
    } else if (button_clicked == 8 || button_clicked == 9) { //Increase or decrease the Initial Value of the Scale in axies Y
        print_Scale_Initial_Value(button_clicked, 1);
    } else if (button_clicked == 10 || button_clicked == 11) { //Increase or decrease the Step Value of the Scale in axies X 
        print_Scale_Step(button_clicked, 0);
    } else if (button_clicked == 12 || button_clicked == 13) { //Increase or decrease the Step Value of the Scale in axies Y
        print_Scale_Step(button_clicked, 1);
    } else if (button_clicked == 14 || button_clicked == 15) { //Increase or decrease the Final Value of the Scale in axies X
        print_Scale_Final_Value(button_clicked, 0);
    } else if (button_clicked == 16 || button_clicked == 17) { //Increase or decrease the Final Value of the Scale in axies Y
        print_Scale_Final_Value(button_clicked, 1);
    } else if (button_clicked == 18) { //YES in the axies X
        create_Yes_No_Buttons("YES", 1, "X");
        create_Yes_No_Buttons("NO", 0, "X");
    } else if (button_clicked == 19) { //NO in the axies X
        create_Yes_No_Buttons("NO", 1, "X");
        create_Yes_No_Buttons("YES", 0, "X");
    } else if (button_clicked == 20) { //YES in the axies Y
        create_Yes_No_Buttons("YES", 1, "Y");
        create_Yes_No_Buttons("NO", 0, "Y");
    } else if (button_clicked == 21) { //NO in the axies Y
        create_Yes_No_Buttons("NO", 1, "Y");
        create_Yes_No_Buttons("YES", 0, "Y");
    } else if (button_clicked == 22) {
        screen_Settings(3);
    } else if (button_clicked == 23) {
        screen_Settings(2);
    }
  }
  button_clicked = -1;
}

void print_Max_Limit_Voltage(uint8_t decrease_or_increase) {
  float step_for_decreasing_or_increasing = 0.5;
  float maximum_value = voltage_Level_PinA_MCP;
  uint8_t y0 = 91;
  uint8_t x0 = (width / 2) - (square_settings_width / 2);
  uint8_t r_height = 35;
  uint8_t r_width = square_settings_width;
  uint8_t t_size = 2;
  uint8_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = 0;
  tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge
  if (decrease_or_increase == 2) { //DECREASE
    final_Voltage_Value = final_Voltage_Value - step_for_decreasing_or_increasing;
    if (final_Voltage_Value < -maximum_value) { //if it is a too small value, begin of the biggest value
      final_Voltage_Value = maximum_value;
    }
  } else if (decrease_or_increase == 3) { //INCREASE
    final_Voltage_Value = final_Voltage_Value + step_for_decreasing_or_increasing;
    if (final_Voltage_Value > maximum_value) {
      final_Voltage_Value = -maximum_value; //if it is a too big value, begin from the smallest
    }
  }
  if (final_Voltage_Value < 0) { // this both if are just to centralize the text in the rectangle as when it is negative there another character (-)
    c_x0 = x0 + (t_size * ch_width / 2) - 1;
  } else if (final_Voltage_Value >= 0) {
    c_x0 = x0 + (t_size * ch_width / 2) + 5;
  }
  tft.setCursor(c_x0, c_y0);
  tft.setTextSize(t_size);
  tft.setTextColor(color_1);
  tft.print(final_Voltage_Value);
}

void print_Min_Limit_Voltage(uint8_t decrease_or_increase) {
  float step_for_decreasing_or_increasing = 0.5;
  float maximum_value = voltage_Level_PinA_MCP; //this is a value in module
  uint8_t y0 = 145;
  uint8_t x0 = (width / 2) - (square_settings_width / 2);
  uint8_t r_height = 35;
  uint8_t r_width = square_settings_width;
  uint8_t t_size = 2;
  uint8_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = 0;
  tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge

  if (decrease_or_increase == 4) { //DECREASE
    initial_Voltage_Value = initial_Voltage_Value - step_for_decreasing_or_increasing;
    if (initial_Voltage_Value < -maximum_value) { //if it is a too small value, begin of the biggest value
      initial_Voltage_Value = maximum_value;
    }
  } else if (decrease_or_increase == 5) { //INCREASE
    initial_Voltage_Value = initial_Voltage_Value + step_for_decreasing_or_increasing;
    if (initial_Voltage_Value > maximum_value) {
      initial_Voltage_Value = -maximum_value; //if it is a too big value, begin from the smallest
    }
  }
  if (initial_Voltage_Value < 0) { // this both if are just to centralize the text in the rectangle as when it is negative there another character (-)
    c_x0 = x0 + (t_size * ch_width / 2) - 1;
  } else if (initial_Voltage_Value >= 0) {
    c_x0 = x0 + (t_size * ch_width / 2) + 5;
  }
  tft.setCursor(c_x0, c_y0);
  tft.setTextSize(t_size);
  tft.setTextColor(color_1);
  tft.print(initial_Voltage_Value);
}

void print_Step_Signal(uint8_t decrease_or_increase) {
  float min_Output_Signal_Step_Possible =  voltage_Level_PinA_MCP / (float)MCP_Resolution; //This is the minimal value for the signal step output
  min_Output_Signal_Step_Possible = truncate(min_Output_Signal_Step_Possible, 3);
  float maximum_value = 54 * min_Output_Signal_Step_Possible; //this is a value in module
  uint8_t y0 = 199; //this value was obtained by the function drawRectangles (return y0 in the loop for)
  uint8_t x0 = (width / 2) - (square_settings_width / 2);
  uint8_t r_height = 35;
  uint8_t r_width = square_settings_width;
  uint8_t t_size = 2;
  uint8_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = 0;
  tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge

  if (decrease_or_increase == 6) { //DECREASE
    signal_Step = signal_Step - min_Output_Signal_Step_Possible;
    if (signal_Step < min_Output_Signal_Step_Possible) { //The value cant be smaller than 0
      signal_Step = maximum_value;
    }
  } else if (decrease_or_increase == 7) { //INCREASE
    signal_Step = signal_Step + min_Output_Signal_Step_Possible;
    if (signal_Step > maximum_value) {
      signal_Step = min_Output_Signal_Step_Possible; //if it is a too big value, begin from the smallest. In this case, the smaller value is the min_Signal_Step_Possible
    }
  }
  c_x0 = x0 + (t_size * ch_width / 2) - 1;
  tft.setCursor(c_x0, c_y0);
  tft.setTextSize(t_size);
  tft.setTextColor(color_1);
  tft.print(signal_Step, 3); //print with 3 decimal numbers
}

void print_Integration_Time(uint8_t decrease_or_increase) { //create_Rectangles_Settings
  float maximum_value = 3; //this is a value in module
  float minimum_value = 0;
  float step_for_decreasing_or_increasing = 1;
  uint8_t numb_of_ch = 4; //if there is two decimal houses it is the unit + point + decimal houses
  uint16_t y0 = 253; //this value was obtained by the function create_Rectangles_Settings (return y0 in the loop for)
  uint8_t x0 = (width / 2) - (square_settings_width / 2);
  uint8_t r_height = 35;
  uint8_t r_width = square_settings_width;
  uint8_t t_size = 2;
  uint16_t c_y0 = y0 + ((r_height - (t_size * ch_height)) / 2);
  uint8_t c_x0 = x0 + (t_size * ch_width / 2) + 5; //5 was testing
  tft.fillRect(x0 + 1, y0 + 1, r_width - 2, r_height - 2, color_2); //Erase what was writen and dont take the edge

  if (decrease_or_increase == 8) { //DECREASE
    integration_Time = integration_Time - step_for_decreasing_or_increasing;
    if (integration_Time < minimum_value) { //The value cant be smaller than 0
      integration_Time = maximum_value;
    }
  } else if (decrease_or_increase == 9) { //INCREASE
    integration_Time = integration_Time + step_for_decreasing_or_increasing;
    if (integration_Time > maximum_value) {
      integration_Time = minimum_value; //if it is a too big value, begin from the smallest. In this case, the smaller value is the min_Signal_Step_Possible
    }
  }
  tft.setCursor(c_x0, c_y0);
  tft.setTextSize(t_size);
  tft.setTextColor(color_1);
  tft.print(integration_Time, 2); //print with 2 decimal numbers
}

void print_M_Button() { //This functions print all the Matrix M_Button in the monitor serial to make it easier to debug
  uint8_t columns = 0;
  uint8_t lines = 0;
  Serial.println("Button's Matrix = ");
  for (lines = 0; lines < num_lines; lines++) {
    for (columns = 0; columns < num_columns; columns++) {
      if (M_Button[lines][columns] == 0){
        break; //just to dont lost time printing where it is 0 as it is not a button
      }
      Serial.print(M_Button[lines][columns]);
      if (M_Button[lines][columns] >= 0 && M_Button[lines][columns] < 10) { //There is just one digit
        Serial.print("   "); //3 spaces
      }
      if (M_Button[lines][columns] > 9 && M_Button[lines][columns] < 100) { //There is two digits
        Serial.print("  "); //2 spaces
      }
      if (M_Button[lines][columns] > 99) { //There is three digits
        Serial.print(" "); //1 space
      }
    }
    Serial.println();
  }
}

void print_V_and_I_Vectors() { //This functions print all the Matrix M_Button in the monitor serial to make it easier to debug
  float *voltage_vector = create_Voltage_Vector();
  uint16_t columns = 0;
  Serial.print("Voltage's Vector = (");
  for (columns = 0; columns < number_of_points(initial_Voltage_Value, signal_Step, final_Voltage_Value); columns++) {
    Serial.print(voltage_vector[columns], 3);
    Serial.print(", ");
  }
  Serial.println("}");
}

uint8_t number_of_useless_zero(float value, uint8_t decimal_places) {
  value = abs(value); //when the number is negative
  uint8_t number_of_zeros = 0;
  uint8_t expo = 1;
  uint16_t value_int = 0;
  uint16_t quoc = pot_10(expo);
  value_int = value * pot_10(decimal_places); //28200
  if (decimal_places == 0) { //without this if the function always enter in the do while and stays there until overflow
    return number_of_zeros;
  }
  do {
    if (value_int % quoc == 0) {
      number_of_zeros++;
    } else {
      break;
    }
    expo++;
    quoc = pot_10(expo);
    decimal_places--;
  } while (decimal_places != 0);
  return number_of_zeros;
}

uint8_t real_number_of_ch(uint8_t decimal_places, uint8_t number_of_useless_zero) {
  uint8_t real_number_of_ch = 0;
  if (number_of_useless_zero == decimal_places) { //numero é inteiro
    real_number_of_ch = 1;
  } else {
    real_number_of_ch = 2 + (decimal_places - number_of_useless_zero);
  }
  return real_number_of_ch;
}

uint32_t pot_10(uint8_t exponent) {
  uint32_t result = 1;
  for (uint8_t counter = 1; counter <= exponent; counter++) {
    result = result * 10;
  }
  return result;
}

float truncate(float val, byte dec) //round some function to the number of "DEC" places
{
  float x = val * pow(10, dec);
  float y = round(x);
  float z = x - y;
  if ((int)z == 5)
  {
    y++;
  } else {}
  x = y / pow(10, dec);
  return x;
}

//Ignore this function for now
void debugger() {
  uint8_t sensor_to_be_measured = 0;
  uint8_t counter_2 = 1;
  uint8_t ordinary_sensor_number = 1;
  uint8_t measurement_type = 0;

  Serial.print("A página atual é ");
  Serial.println(current_page);

  Serial.print("O vetor do tipo de medição é {");
  for (uint8_t counter_3 = 1; counter_3 < 15; counter_3++) {
    Serial.print(sensor_counter[counter_3]);
    Serial.print(",");
  }
  Serial.println("}");

  Serial.print("O numero de sensores selecionados é ");
  Serial.println(number_of_measurements());

  for (uint8_t counter_1 = 1; counter_1 <= number_of_measurements(); counter_1++) {
    for (uint8_t counter = 1; counter <= 14; counter++) {
      if (sensor_counter[counter] > 0) {
        sensor_to_be_measured = counter;
        measurement_type = sensor_counter[counter];
        if (counter_2 == ordinary_sensor_number) {
          break;
        }
        counter_2++;
      }
    }
    counter_2 = 1;
    Serial.print("O ");
    Serial.print(ordinary_sensor_number);
    Serial.print("º sensor a ser medido é ");
    Serial.print(sensor_to_be_measured);
    Serial.print(" com o tipo de medição ");
    Serial.println(measurement_type);
    ordinary_sensor_number++;
  }
  Serial.println("As configuracoes do layout do grafico sao ");
  Serial.print("EIXO X: (");
  Serial.print(scale_initial_value_x, 10);
  Serial.print(", ");
  Serial.print(scale_step_x, 10);
  Serial.print(", ");
  Serial.print(scale_final_value_x, 10);
  Serial.print(")");
  Serial.print(" com ");
  Serial.print(number_of_decimal_places_x, 10);
  Serial.print(" casas decimais e ");
  Serial.print(dotted_lines_x);
  Serial.println(" para linhas pontilhadas");
  Serial.print("EIXO Y: (");
  Serial.print(scale_initial_value_y, 10);
  Serial.print(", ");
  Serial.print(scale_step_y, 10);
  Serial.print(", ");
  Serial.print(scale_final_value_y, 10);
  Serial.print(")");
  Serial.print(" com ");
  Serial.print(number_of_decimal_places_y, 10);
  Serial.print(" casas decimais e ");
  Serial.print(dotted_lines_y);
  Serial.println(" para linhas pontilhadas");

  print_V_and_I_Vectors();
  print_M_Button(); //This print all the M_Button Matrix, it is good to debug the code
  Serial.println("____________________________________________________________________");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  init_All();
  reset_M_Button();
  screen_Menu();
  current_page = 0;
}

void update_RTC(){
  DateTime now = rtc.now();
  minutes_now = now.minute();
  uint8_t hour_now = now.hour();
  uint8_t second_now = now.second();
  if ((current_page == 0) && (minutes_now != minutes)){
    update_RTC_Time_Menu(false);
    minutes = minutes_now;
  } 
  if ((current_page == 0) && (hour_now == 0) && (minutes_now == 0) && (second_now == 0)){ //It's midnight of one day. Clean everything and print again
     tft.fillRect(0, 0, width, height, bg_color);
     screen_Menu();
  }
}

void touch_Pressed(){
  TSPoint p = ts.getPoint(); //get touch point
  if (p.z > ts.pressureThreshhold) {
    which_Button_Pressed(); //Discovery which button was pressed in the screen
    delay(touch_delay); //Create a delay because without it, the sensor recognize more than one click
    choose_page(); //Knowing which button was pressed, choose what page should be created
    debugger(); //ignore this function for now
  }  
}

void loop() {
  // put your main code here, to run repeatedly:
  touch_Pressed();
  update_RTC();
}
