#include <Arduino.h>
#include "crsf.h"

// CRSF settings
#define SBUS_BUFFER_SIZE 32
uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};
uint16_t _old_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

#define CRSF_MIN_VALUE 1000
#define CRSF_MID_VALUE 1500
#define CRSF_MAX_VALUE 2000
//END CRSF SETTINGS

// I/O SETTINGS
#define RX_PIN 26
#define TX_PIN 27

#define ESC_OUTPUT_PIN_1 23
#define ESC_OUTPUT_PIN_2 17
#define ESC_OUTPUT_PIN_3 22
#define ESC_OUTPUT_PIN_4 16

#define SERVO_OUTPUT_PIN -1

#define LEFT_JOYSTICK_X_CHANNEL 1
#define RIGHT_JOYSTICK_Y_CHANNEL 0
#define BUTTON_D_CHANNEL 6 
#define SWITCH_F_CHANNEL 5

#define MIN_FILTER 3
#define CENTER_FILTER_COEF 30
#define MOVEMENT_TURN_COEF 0.7

#define FULL_SPEED_COEF 1.0
#define LOW_SPEED_COEF 0.4

int lowerBorder = CRSF_MID_VALUE - CENTER_FILTER_COEF;
int higherBorder = CRSF_MID_VALUE + CENTER_FILTER_COEF;

int rightWheelOneChannel = 0; //antenna tube is top left
int leftWheelOneChannel = 1;
int rightWheelTwoChannel = 2;
int leftWheelTwoChannel = 3;

int rightWheelOneMapped;
int rightWheelTwoMapped;
int leftWheelOneMapped;
int leftWheelTwoMapped;

int yCmd = 0;
char xCmd = 0;
int bCmd = 0;
int sCmd = 0;

float speedCoef = FULL_SPEED_COEF;

bool lostConnectionFlag = false;
//END I/O SETTINGS

//DEBUG SETTINGS
#define PRINT_PWM_INFO 0
#define PRINT_CHANNELS 0
#define PRINT_COMMANDS 0
//END DEBUG SETTINGS

void SetServoPos(float percent, int pwmChannel)
{
    // 50 cycles per second 1,000ms / 50 = 100 /5 = 20ms per cycle
    // 1ms / 20ms = 1/20 duty cycle
    // 2ms / 20ms = 2/20 = 1/10 duty cycle
    // using 16 bit resolution for PWM signal convert to range of 0-65536 (0-100% duty/on time)
    // 1/20th of 65536 = 3276.8
    // 1/10th of 65536 = 6553.6

    uint32_t duty = map(percent, 0, 100, 3276.8, 6553.6);
    if (!lostConnectionFlag) {
      ledcWrite(pwmChannel, duty);
    }
    
    if (PRINT_PWM_INFO) {
      Serial.printf("%u% || %u%\n", pwmChannel, duty);
    }
}

int getBCmd() {
  if (_raw_rc_values[BUTTON_D_CHANNEL] > _old_rc_values[BUTTON_D_CHANNEL] && _old_rc_values[BUTTON_D_CHANNEL] != 0) {
    return 1; //change speed mode
  }
  return 0;
}

int getSCmd() {
  if (0) {return 1;}
  return 0;
}

char getXCmd() {

  bool val_in_the_middle = ((lowerBorder < _raw_rc_values[LEFT_JOYSTICK_X_CHANNEL]) && (_raw_rc_values[LEFT_JOYSTICK_X_CHANNEL] < higherBorder));

  if ((_raw_rc_values[LEFT_JOYSTICK_X_CHANNEL] - _old_rc_values[LEFT_JOYSTICK_X_CHANNEL] >= MIN_FILTER)) {
    if(_raw_rc_values[LEFT_JOYSTICK_X_CHANNEL]>CRSF_MID_VALUE) return  'R'; //right turn
    else return 'N';
  } else if (_old_rc_values[LEFT_JOYSTICK_X_CHANNEL] - _raw_rc_values[LEFT_JOYSTICK_X_CHANNEL] >= MIN_FILTER) {
    if(_raw_rc_values[LEFT_JOYSTICK_X_CHANNEL]<CRSF_MID_VALUE) return 'L'; //left turn
    else return 'N';
  }

  if (val_in_the_middle) return 'N';  
  return xCmd;
}

int getYCmd() {

  bool val_in_the_middle = ((lowerBorder < _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL]) && (_raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL] < higherBorder));

  if (_raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL] - _old_rc_values[RIGHT_JOYSTICK_Y_CHANNEL] >= MIN_FILTER) {
    if(_raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL]>CRSF_MID_VALUE){
      return 1; // fwd
    }else{
      return 0;
    } 
  } else if (_old_rc_values[RIGHT_JOYSTICK_Y_CHANNEL] - _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL] >= MIN_FILTER) {
    if(_raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL]<CRSF_MID_VALUE){
      return 2; // back
    }else{
      return 0;
    }    
  }

  if (val_in_the_middle) {
    return 0;
  }  
  return yCmd;
}

long move(String side, String direction, uint16_t val) {
  
  long rel_zero = 50-50*speedCoef;
  long rel_hundred = 50+50*speedCoef; 
  
  String cmd = side+"->"+direction;
  
  if(cmd=="R1->F"||cmd=="R2->B"){
      return map(val, CRSF_MAX_VALUE, CRSF_MIN_VALUE, rel_hundred, rel_zero); // AIHZ
  } else
  if(cmd=="R1->B"||cmd=="R2->F"){
    return map(val, CRSF_MIN_VALUE, CRSF_MAX_VALUE, rel_zero, rel_hundred); // IAZH
  } else
  if(cmd=="L1->B"||cmd=="L2->F"||cmd=="R1->R"||cmd=="R2->L"){
    return map(val, CRSF_MAX_VALUE, CRSF_MIN_VALUE, rel_zero, rel_hundred); // AIZH
  } else 
  if(cmd=="L1->F"||cmd=="L2->B"||cmd=="R1->L"||cmd=="R2->R"){
    return map(val, CRSF_MIN_VALUE, CRSF_MAX_VALUE, rel_hundred, rel_zero); // IAHZ
  } else {
    return map(val, CRSF_MIN_VALUE, CRSF_MAX_VALUE, rel_zero, rel_hundred);
  }
} 

void lostConnection() {
  
  long val = move("R1", "N", CRSF_MID_VALUE);
  
  SetServoPos( val, rightWheelOneChannel);
  SetServoPos( val, rightWheelTwoChannel); 
        
  SetServoPos( val, leftWheelOneChannel);
  SetServoPos( val, leftWheelTwoChannel);

  lostConnectionFlag = true;
  
  for (int i = 0; i < SBUS_BUFFER_SIZE; i++) {
    _rcs_buf[i] = 0;
  }

  for (int i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
    _raw_rc_values[i] = CRSF_MID_VALUE;
  }
}

uint16_t getCombinedValue(char direction, int position) {
  
  int diff = abs(_raw_rc_values[LEFT_JOYSTICK_X_CHANNEL] - CRSF_MID_VALUE);
  int val;

  if (direction == 'F' && position == 1) {
    val = min(CRSF_MAX_VALUE, _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL] + diff);
  } else if (direction == 'F' && position == 2) {  
    val = min((double)CRSF_MAX_VALUE, CRSF_MID_VALUE + (diff * MOVEMENT_TURN_COEF));
  } else if (direction == 'B' && position == 1) {
    val = max(CRSF_MIN_VALUE, _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL] - diff);
  } else if (direction == 'B' && position == 2) {
    val = max((double)CRSF_MIN_VALUE, CRSF_MID_VALUE - (diff * MOVEMENT_TURN_COEF));
  }

  return val;
}

void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  
  Serial.begin(9600);
  Serial2.begin(420000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  ledcSetup(rightWheelOneChannel,50,16);
  ledcSetup(rightWheelTwoChannel,50,16);
  ledcSetup(leftWheelOneChannel,50,16);
  ledcSetup(leftWheelTwoChannel,50,16);

  ledcAttachPin(ESC_OUTPUT_PIN_1, rightWheelOneChannel);
  ledcAttachPin(ESC_OUTPUT_PIN_2, rightWheelTwoChannel);
  ledcAttachPin(ESC_OUTPUT_PIN_3, leftWheelOneChannel);
  ledcAttachPin(ESC_OUTPUT_PIN_4, leftWheelTwoChannel);
}

void loop() { //Choose Serial1 or Serial2 as required
  
  while (true) {
    if (!Serial2.available()) {
      lostConnection();
      break;
    } else {
      size_t numBytesRead = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
      if(numBytesRead == 0) {
        lostConnection();
        break;
      } else {
        lostConnectionFlag = false;
        memcpy(_old_rc_values, _raw_rc_values, sizeof(_raw_rc_values));
        crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );
        
        yCmd = getYCmd();
        xCmd = getXCmd();
        bCmd = getBCmd();
        sCmd = getSCmd();

        uint16_t l_crsf_val = _raw_rc_values[LEFT_JOYSTICK_X_CHANNEL];
        uint16_t r_crsf_val = _raw_rc_values[LEFT_JOYSTICK_X_CHANNEL];
        String dirR="N";
        String dirL="N";

        if (PRINT_CHANNELS) {
          Serial.print("Ch 1: ");
          Serial.print(_raw_rc_values[0]);
          Serial.print("\tCh 2: ");
          Serial.print(_raw_rc_values[1]);
          Serial.print("\tCh 3: ");
          Serial.print(_raw_rc_values[2]);
          Serial.print("\tCh 4: ");
          Serial.print(_raw_rc_values[3]);
          Serial.print("\tCh 5: ");
          Serial.print(_raw_rc_values[4]);
          Serial.print("\tCh 6: ");
          Serial.print(_raw_rc_values[5]);
          Serial.print("\tCh 7: ");
          Serial.println(_raw_rc_values[6]);
        }
        if (PRINT_COMMANDS) {
          Serial.printf("Speed coef: %f% || ", speedCoef);
          Serial.printf("%u || ", bCmd);
          Serial.printf("%u || ", sCmd);
          Serial.printf("%u || ", yCmd)
          Serial.printf("%c\n", sCmd);
        }

        switch(bCmd){
          case 1:
            speedCoef == FULL_SPEED_COEF?speedCoef = LOW_SPEED_COEF:speedCoef = FULL_SPEED_COEF;
            break;
          default:
            break;
        }
    
        switch(sCmd) {
          case 1:
            break;
          default:
            break;
        }

        switch(xCmd){
          case 'R':
            dirR="R";
            dirL="F";
            switch (yCmd) {
              case 1:
                dirR = "F";
                l_crsf_val = getCombinedValue('F', 1);
                r_crsf_val = getCombinedValue('F', 2);
                break;
              case 2:
                dirR = "B";
                l_crsf_val = getCombinedValue('B', 1);
                r_crsf_val = getCombinedValue('B', 2);
                break;
              default:
                break;
            }
            break;

          case 'L':
            dirR="L";
            dirL="B";
            switch (yCmd) {
              case 1:
                dirR = "F";
                l_crsf_val = getCombinedValue('F', 2);
                r_crsf_val = getCombinedValue('F', 1);
                break;
              case 2:
                dirR ="B";
                l_crsf_val = getCombinedValue('B', 2);
                r_crsf_val = getCombinedValue('B', 1);
                break;              
              default:
                break;
            }
            break;

          /**/
          default:
            switch (yCmd) {
              case 1: 
                l_crsf_val = _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL];
                r_crsf_val = _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL];
                dirR="F";
                dirL="F";
                break;
              case 2: 
                l_crsf_val = _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL];
                r_crsf_val = _raw_rc_values[RIGHT_JOYSTICK_Y_CHANNEL];
                dirR="B";
                dirL="B";
                break;
              default:
                dirR="N";
                dirL="N";
                l_crsf_val = CRSF_MID_VALUE;
                r_crsf_val = CRSF_MID_VALUE;
            }
          
        }
        
        SetServoPos( move("R1", dirR, r_crsf_val), rightWheelOneChannel);
        SetServoPos( move("L1", dirL, l_crsf_val), leftWheelOneChannel);
        
        SetServoPos( move("R2", dirR, r_crsf_val), rightWheelTwoChannel);
        SetServoPos( move("L2", dirL, l_crsf_val), leftWheelTwoChannel);

      }
    }
  }
}