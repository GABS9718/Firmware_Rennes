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

#define step_voltage 0.078125 //5V divided per 64 (wiper resolution)

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
  int step_wiper=deltaV/step_voltage; //Calculate the step that the wiper must increase
  uint8_t wiper_position=0; //New variable to control the position of the wiper by adding the step
  round(step_wiper); //Round the step, as it just can be a integer number between 0-64
  do {      
      change_R_DP(pot_0, wiper_position); //Variate the resistence of the selected potentiometer by selecting the position of the wiper
      wiper_position=wiper_position+step_wiper; //Increasinig the wiper's position according to the step of the definied by the user
      delay(deltaT);
  }while(wiper_position<64);
  change_R_DP(pot_0, 0);
  delay(deltaT);
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //Boot of the library and setting the arduino as master. This is a default option: when there is nothing written inside the parenthesis, the device is a master.
  Serial.begin(9600);
  generate_ramp(500, 0.234375);
}

void loop() {
  // put your main code here, to run repeatedly:

}
