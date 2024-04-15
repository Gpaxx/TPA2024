#define prox_F 30
#define prox_BR 31
#define prox_BL 32

#define red_sw 10
#define yellow_sw 11
#define black_sw 12

const bool prox_debug = false;
const bool sw_debug = false;

void initial_sensor()
{
  pinMode(prox_F , INPUT);
  pinMode(prox_BR , INPUT);
  pinMode(prox_BL , INPUT);
  Serial.println("Prox ready!!");

  pinMode(red_sw , INPUT);
  pinMode(yellow_sw , INPUT);
  pinMode(black_sw , INPUT);
//    pinMode(toggle_sw , INPUT);
  Serial.println("SW ready!!");
}

void debug()
{
  if (prox_debug)Serial.printf("prox_F : %d , prox_BR : %d , prox_BL : %d \n" , digitalRead(prox_F) , digitalRead(prox_BR) , digitalRead(prox_BL));
  if (sw_debug)Serial.printf("yellow_sw :  %d , red_sw : %d , black : %d \n", digitalRead(yellow_sw) , digitalRead(red_sw) , digitalRead(black_sw));
}
