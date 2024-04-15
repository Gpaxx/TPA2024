class motor
{
  public:

    uint8_t INA;
    uint8_t INB;
    uint8_t PWM;

    static uint8_t ENCA;
    static uint8_t ENCB;

    static uint16_t pulse_resolution;

    static uint32_t pulse;
    static uint32_t dt;

    static float coff_time_to_rpm;
    static float coff_pulse_to_rad;

    float rpm = 0;

    struct gain
    {
      float kp;
      float ki;
      float kd;
      float abs_max_sum;
      float prev_error;

      float pid_controller(float error )
      {
        static float P, I, D;

        P = kp * error;
        I += ki * error;
        D = kd * (error - prev_error);

        return P + I + D;
      }

    } speed, position;



    void init_motor(uint8_t pin_INA , uint8_t pin_INB , uint8_t pin_PWM , uint8_t pin_ENCA , uint8_t pin_ENCB , uint16_t pulse_resolution_1_disc)
    {
      INA = pin_INA;
      INB = pin_INB;
      PWM = pin_PWM;
      ENCA = pin_ENCA;
      ENCB = pin_ENCB;
      pulse_resolution = pulse_resolution_1_disc;

      coff_time_to_rpm = 60000000 / pulse_resolution;

      pinMode(INA , OUTPUT);
      pinMode(INB , OUTPUT);
      pinMode(PWM , OUTPUT);
      pinMode(ENCA , INPUT);
      pinMode(ENCB , INPUT);
      attachInterrupt(ENCB , callback_pulse , RISING);
      attachInterrupt(ENCA , callback_dt , CHANGE);
    }

    void drive(int pwm , bool dir)
    {
      if (dir && abs(pwm) > 0 )
      {
        digitalWrite(INA , HIGH);
        digitalWrite(INB , LOW);
        analogWrite(PWM  , abs(pwm));
      }
      else if (!dir && abs(pwm) > 0)
      {
        digitalWrite(INA , LOW);
        digitalWrite(INB , HIGH);
        analogWrite(PWM  , abs(pwm));
      }
      else
      {
        digitalWrite(INA , HIGH);
        digitalWrite(INB , HIGH);
        analogWrite(PWM  , 0);
      }
    }

    //dosen t approve this function
    void test_drive(uint8_t pin_sw1 , uint8_t pin_sw2 , uint16_t pwm)
    {
      if (digitalRead(pin_sw1) && !digitalRead(pin_sw2))drive(pwm , 1);
      else if (!digitalRead(pin_sw1) && digitalRead(pin_sw2))drive(pwm , 0);
      else drive(0, 1);
    }

  private:
    static void callback_dt()
    {
      static uint32_t prev_time = 0;
      dt = micros() - prev_time;
      prev_time = micros();
    }

    static void callback_pulse()
    {
      if (digitalRead(ENCB))pulse++;
      else if (!digitalRead(ENCB))pulse--;
    }

    float cal_rpm()
    {
      return  coff_time_to_rpm * 1 / dt;
    }

    float cal_pose()
    {
      return coff_pulse_to_rad * pulse;
    }

} mpose_F, mpose_BR, mpose_BL;

uint32_t motor::dt = 0;
uint32_t motor::pulse = 0;
uint8_t motor::ENCA;
uint8_t motor::ENCB;
uint16_t motor::pulse_resolution;
float motor::coff_time_to_rpm;
