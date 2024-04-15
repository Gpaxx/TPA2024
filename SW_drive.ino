#include"motor.h"
#include"position.h"

uint64_t PrevTime = 0; 

uint8_t INA[3] = {41,39,35}; // {INA_1 , INA_2 ,INA_3}
uint8_t INB[3] = {40,38,33}; // {INB_1 , INB_2 ,INB_3}
uint8_t PWM[3] = {37,36,33}; // {PWM_1 , PWM_2 ,PWM_3}

uint8_t ENCA[3] = {2,4,6};  // {ENCA_1 , ENCA_2 ,ENCA_3}
uint8_t ENCB[3] = {3,5,7};  // {ENCB_1 , ENCB_2 ,ENCB_3}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogWriteResolution(10);
  initial_sensor();
  mpose_F.init_motor(INA[0],INB[0],PWM[0],ENCA[0],ENCB[0],200);
  mpose_BR.init_motor(INA[1],INB[1],PWM[1],ENCA[1],ENCB[1],200);
  mpose_BL.init_motor(INA[2],INB[2],PWM[2],ENCA[2],ENCB[2],200);
}

void loop() {
  // put your main code here, to run repeatedly
     
}
