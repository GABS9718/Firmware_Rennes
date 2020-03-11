//--- Section of includes ---
#include <SPI.h>  //This library is necessary for using the communication protocol call SPI.

//--- Section of defines ---
//Digital Potentiometer (#MCP41010)
#define voltage_input 5 //Value of the voltage input of the output of arduino applied to the voltage division
#define MCP_pin_CS 49 //Pin CS for the MCP41010
#define number_of_step_DP_MPC 256 //Resolution of the steps from the MPC potentiometer
#define MPC_DP_instr B00010001 //Instruction sent to the MPC potentiometer
//MUX (#CD4051BE)
#define MUX_pin_A 30
#define MUX_pin_B 32
#define MUX_pin_C 34

/*Action: Initializes all necessary functions
 *Functions:  
 *          DP_MPC -> Initializes de digital potentiometer MPC communication
*           Serial_init -> initializes the serial communication to USB/Monitor Serial
 * Return: nothing
 */
void Init_all () {
  DP_MCP_Init();
  Serial_Init();
  MUX_Init();
}

void DP_MCP_Init(){
  pinMode (MCP_pin_CS, OUTPUT);  //Defining the pin CD of the MPC41010 as a digital output
  SPI.begin(); //Boot of the library necessary for the SPI protocol
}

void Serial_Init(){
  Serial.begin(9600);
}

void MUX_Init(){
  pinMode(MUX_pin_A, OUTPUT); //pin A of the multiplexer as a OUTPUT to generate a digital signal with 3 its
  pinMode(MUX_pin_B, OUTPUT); // = B
  pinMode(MUX_pin_C, OUTPUT); // = C
}

/*Action: write in the selected potentiometer a value of resistence by selection the wipe position
 *Inputs: 
 *        instruction -> select what kind of instruction the potentiometer should do (datasheet)
 *        wiper_value_selection -> selected where the wipe gonna be
 * Return: nothing
 */
void change_R_DP_MPC (uint8_t instruction, uint8_t wiper_value_selection){
  digitalWrite(MCP_pin_CS,LOW);
  SPI.transfer(instruction); // This tells the chip to set the pot
  SPI.transfer(wiper_value_selection);     // This tells it the pot position
  digitalWrite(MCP_pin_CS,HIGH);
  delay(10);
}

/*Action: Generate the ramp as the resistence of the potentiometer variates.
 *Inputs: 
 *        deltaT -> time in milliseconds to increase the value of the voltage
 * Return: nothing
 */
int pos_wiper_MPC[26]={0, 10, 20, 30, 41, 51, 61, 72, 82, 92, 103, 113, 123, 133, 144, 154, 164, 175, 185, 195, 206, 216, 226, 237, 247, 255};
// This corresponds to the MEASURED vector of voltage [0.00131, 0.20756, 0.40142, 0.5959, 0.80907, 1.00292, 1.19753, 1.41031, 1.60413, 1.79869, 2.01106, 2.20400, 2.3985, 2.5922, 2.803, 2.9997, 3.1936, 3.4006, 
//                                                     3.6001, 3.795, 4.008, 4.2016, 4.396, 4.6093, 4.8031, 4.9599]
  
void generate_ramp_MPC (uint16_t deltaT){
   int vect_counter=0;
   for (vect_counter=0; vect_counter<=25; vect_counter++){
    change_R_DP_MPC(MPC_DP_instr, pos_wiper_MPC[vect_counter]);
    delay(deltaT);  
  } 
}

/*Action: select the pin of the multiplexer that should be connect with the pin COM
 *Inputs: 
 *        pin_choosed -> number of the pin choosed
 * Return: nothing
 */
void MUX_selection(uint8_t pin_choosed){
  switch(pin_choosed){
    case 0:
      digitalWrite(MUX_pin_A, LOW); // 000 = pin0
      digitalWrite(MUX_pin_B, LOW);
      digitalWrite(MUX_pin_C, LOW);    
      break;  
    case 1: 
      digitalWrite(MUX_pin_A, HIGH); //001 = pin1
      digitalWrite(MUX_pin_B, LOW);
      digitalWrite(MUX_pin_C, LOW);  
      break;  
  case 2:  
      digitalWrite(MUX_pin_A, LOW);  //010 = pin2
      digitalWrite(MUX_pin_B, HIGH);
      digitalWrite(MUX_pin_C, LOW);
      break;  
  case 3: 
      digitalWrite(MUX_pin_A, HIGH);  //011 = pin3
      digitalWrite(MUX_pin_B, HIGH);
      digitalWrite(MUX_pin_C, LOW); 
      break;
  case 4: 
      digitalWrite(MUX_pin_A, LOW); //100 = pin4
      digitalWrite(MUX_pin_B, LOW);
      digitalWrite(MUX_pin_C, HIGH);
      break;  
  case 5: 
      digitalWrite(MUX_pin_A, HIGH); //101 = pin5
      digitalWrite(MUX_pin_B, LOW);
      digitalWrite(MUX_pin_C, HIGH);
      break;
  case 6: 
      digitalWrite(MUX_pin_A, LOW); //110 = pin6
      digitalWrite(MUX_pin_B, HIGH);
      digitalWrite(MUX_pin_C, HIGH);
      break;  
  case 7:  
      digitalWrite(MUX_pin_A, HIGH); //111 = pin7
      digitalWrite(MUX_pin_B, HIGH);
      digitalWrite(MUX_pin_C, HIGH);
      break;   
  default:
  Serial.println("Erro 01: variável do pino do MUX com valor incompatível.");
  break;
  }
}
void setup() {
  // put your setup code here, to run once:
  Init_all(); //Initializes all necessary functions
  generate_ramp_MPC(2000); //Generate the ramp in the MPC digital potentiometer with a delay of (x) milliseconds
  MUX_selection(0);
}

void loop() {
  // put your main code here, to run repeatedly:


}
