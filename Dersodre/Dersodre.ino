//--- Section of includes ---
#include <Wire.h> //This library is necessary for using the communication protocol call I2C.

//--- Section of defines ---
//Note: the "B" before the name of the variables means that the number must be binary
#define DP_slave_adress B00101000 //Setting the adress of the slave DP (Digital Potentiometer)
#define DP_instr B10100000 // |A B C D|E F|G H| Setting the data of the slave DP (A-D = INSTRUCTIONS)
                           // (E-F = NUMBER OF POTENTIOMETER)
                          //  (G-H = WHICH REGISTER)
//#define DP_data B00111111 // |A B| C D E F G H| (A-B = 00) (C-H = WIPER POSITION)
#define pot_0 B10100000
#define pot_1 B10100100
#define pot_2 B10101000
#define pot_3 B10101100

#define voltage_input 5 //Value of the voltage input of the output of arduino applied to the voltage division
#define number_of_step_DP 64 //The number of steps from the digital potentiometer

/*Action: write in the selected potentiometer a value of resistence by selection the wipe position
 *Inputs: 
 *        which_Pot -> selecte which potentiometer must have it resistence value change
 *        wiper_value_selection -> selected where the wipe gonna be
 * Return: --
 */
void change_R_DP (uint8_t which_Pot, uint8_t wiper_value_selection){
  Serial.println("Entrou na funcao change_R_DP");
  Wire.beginTransmission(DP_slave_adress); //START the communication
  Wire.write(which_Pot); //Write in the DP this instruction
  Wire.write(wiper_value_selection); //Write in the DP this value 
  delay(10);
  Wire.endTransmission(); //End the transmission
}

/*Action: Generate the ramp as the resistence of the potentiometer variates.
 *Inputs: 
 *        deltaT -> time in milliseconds to increase the value of the voltage
 *        deltaV -> value in voltage of it's step 
 * Return: --
 */
void generate_ramp (uint16_t deltaT, float deltaV){
  float step_voltage = float(voltage_input)/float(number_of_step_DP); //This calculate the resolution of the voltage that can be applied.
  int step_wiper=deltaV/step_voltage; //Calculate the step that the wiper must increase
  uint8_t wiper_position=0; //New variable to control the position of the wiper by adding the step
  round(step_wiper); //Round the step, as it just can be a integer number between 0-64
  do {      
      change_R_DP(pot_0, wiper_position); //Variate the resistence of the selected potentiometer by selecting the position of the wiper
      wiper_position=wiper_position+step_wiper; //Increasinig the wiper's position according to the step of the definied by the user
      delay(deltaT);
  }while(wiper_position<number_of_step_DP);
  change_R_DP(pot_0, 0); //Not shure if this should keep here: when the measurement comes to the end, the wiper goes to position 0.
  delay(deltaT);
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //Boot of the library and setting the arduino as master. This is a default option: when there is nothing written inside the parenthesis, the device is a master.
  Serial.begin(9600);
  generate_ramp(1000, 0.234375);
}

void loop() {
  // put your main code here, to run repeatedly:

}
