#include <Arduino.h>
#include "crsf.h"

#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25] {};
uint16_t _old_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};


#define RX_PIN 27
#define TX_PIN 26

#define OUTPUT_PIN_1 22
#define OUTPUT_PIN_2 23
#define OUTPUT_PIN_3 -1
#define OUTPUT_PIN_4 -1

#define CRSF_MIN_VALUE 1000
#define CRSF_MID_VALUE 1500
#define CRSF_MAX_VALUE 2000

#define LEFT_JOYSTICK_X 1
#define LEFT_JOYSTICK_Y -1
#define RIGHT_JOYSTICK_X -1
#define RIGHT_JOYSTICK_Y 0

#define BUTTON_B 6 

#define MIN_FILTER 3

#define FULL_SPEED_COEF 1.0
#define LOW_SPEED_COEF 0.3

int topRightWheelChannel = 1; //antenna tube is top left
int botttomRightWheelChannel = 2;

/*
int topLeftWheelChannel = 1;
int bottomLeftWheelChannel = 1;
*/

int y_cmd = 0;
int x_cmd = 0;
int b_cmd = 0;

float speed_coef = FULL_SPEED_COEF;

void SetServoPos(float percent, int pwmChannel)
{
    // 50 cycles per second 1,000ms / 50 = 100 /5 = 20ms per cycle
    // 1ms / 20ms = 1/20 duty cycle
    // 2ms / 20ms = 2/20 = 1/10 duty cycle
    // using 16 bit resolution for PWM signal convert to range of 0-65536 (0-100% duty/on time)
    // 1/20th of 65536 = 3276.8
    // 1/10th of 65536 = 6553.6

    uint32_t duty = map(percent, 0, 100, 3276.8, 6553.6);
    ledcWrite(pwmChannel, duty);
    //Serial.print(pwmChannel);
    //Serial.print(" ");
    //Serial.println(duty);
}

int getBCmd() {
  if (_raw_rc_values[BUTTON_B] != _old_rc_values[BUTTON_B]) {
    return 1; //change speed mode
  }
  return 0;
}

int getXCmd() {
  if (_raw_rc_values[LEFT_JOYSTICK_X] - _old_rc_values[LEFT_JOYSTICK_X] >= MIN_FILTER) {
    return  1; //right turn
  } else if (_raw_rc_values[LEFT_JOYSTICK_X] - _old_rc_values[LEFT_JOYSTICK_X] >= MIN_FILTER) {
    return 2; //left turn
  }
  return 0;
}

int getYCmd() {
  if (_raw_rc_values[RIGHT_JOYSTICK_Y] - _old_rc_values[RIGHT_JOYSTICK_Y] >= MIN_FILTER) {
    //Serial.println("+");
    return 1; // accelerate
  } else if (_old_rc_values[RIGHT_JOYSTICK_Y] - _raw_rc_values[RIGHT_JOYSTICK_Y] >= MIN_FILTER) {
    //Serial.println("-");
    return 2;// break
  } 
  return y_cmd;
}

long move(char wheel, char direction, uint16_t val) {
  if (wheel == 't' && direction == 'f') {
    return map(val, CRSF_MAX_VALUE, CRSF_MIN_VALUE, 100, 0) * speed_coef;
  } else if (wheel == 't' && direction == 'b') {
    return map(val, CRSF_MIN_VALUE, CRSF_MAX_VALUE, 0, 100) * speed_coef;
  } else if (wheel == 'b' && direction == 'f') {
    return map(val, CRSF_MAX_VALUE, CRSF_MIN_VALUE, 0, 100) * speed_coef;    
  } else if (wheel == 'b' && direction == 'b') {
    return map(val, CRSF_MIN_VALUE, CRSF_MAX_VALUE, 100, 0) * speed_coef;
  } else if (wheel == 't' && direction == 'n') {
    return map(val, CRSF_MIN_VALUE, CRSF_MAX_VALUE, 0, 100) * speed_coef;
  } else if (wheel == 'b' && direction == 'n') {
    return map(val, CRSF_MIN_VALUE, CRSF_MAX_VALUE, 100, 0) * speed_coef;
  }
}

void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  
  Serial.begin(9600);
  Serial2.begin(420000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  ledcSetup(topRightWheelChannel,50,16);
  ledcSetup(botttomRightWheelChannel,50,16);
  /*
  ledcSetup(topLeftWheelChannel,50,16);
  ledcSetup(bottomLeftWheelChannel,50,16);
  */


  ledcAttachPin(OUTPUT_PIN_1, topRightWheelChannel);
  ledcAttachPin(OUTPUT_PIN_2, botttomRightWheelChannel);
  /*
  ledcAttachPin(OUTPUT_PIN_3, topLeftWheelChannel);
  ledcAttachPin(OUTPUT_PIN_4, bottomLeftWheelChannel);
  */
}


void loop() { //Choose Serial1 or Serial2 as required
  
  while (Serial2.available()) {
    size_t numBytesRead = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if(numBytesRead > 0)
    {
      memcpy(_old_rc_values, _raw_rc_values, sizeof(_raw_rc_values));
      crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );
      /*Serial.print("Ch 1: ");
      Serial.print(_raw_rc_values[0]);
      Serial.print("\tCh 2: ");
      Serial.print(_raw_rc_values[1]);
      Serial.print("\tCh 3: ");
      Serial.print(_raw_rc_values[2]);
      Serial.print("\tCh 4: ");
      Serial.print(_raw_rc_values[3]);
      Serial.print("\tCh 5: ");
      Serial.println(_raw_rc_values[4]); */

      b_cmd = getBCmd();
      y_cmd = getYCmd();
      x_cmd = getXCmd();
      
      int topRightWheelMapped;
      int bottomRightWheelMapped;
      /*
      int topLeftWheelMapped;
      int bottomLeftWheelMapped;
      */
     
      switch (b_cmd) {
        case 1:
          speed_coef == FULL_SPEED_COEF?speed_coef = LOW_SPEED_COEF:speed_coef = FULL_SPEED_COEF;
          break;
        default:
          break;        
      }

      char direction = 'f';
      if (y_cmd == 2) direction = 'b';
      switch (x_cmd)
      {
        case 1:
            topRightWheelMapped = move('t', direction, _raw_rc_values[LEFT_JOYSTICK_X]);
            /*
            bottomLeftWheelMapped = move('b', direction _raw_rc_values[LEFT_JOYSTICK_X]);
            */

          SetServoPos(topRightWheelMapped, topRightWheelChannel);
          /*
          SetServoPos(bottomLeftWheelMapped, botttomLeftWheelChannel);
          */

        case 2:
          /*
          topLefttWheelMapped = move('t', direction, _raw_rc_values[LEFT_JOYSTICK_X]);;
          */
          bottomRightWheelMapped = move('b', direction, _raw_rc_values[LEFT_JOYSTICK_X]);
          /*
          SetServoPos(topLeftWheelMapped, topLeftWheelChannel);
          */
          SetServoPos(bottomRightWheelMapped, botttomRightWheelChannel);
        
        default:
          break;
      }

      switch (y_cmd)
      {
        case 1:
          topRightWheelMapped = move('t', 'f', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          bottomRightWheelMapped = move('b', 'f', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          /*
          topLeftWheelMapped = move('t', 'f', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          bottomLeftWheelMapped = move('t', 'f', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          */

          SetServoPos(topRightWheelMapped, topRightWheelChannel);
          SetServoPos(bottomRightWheelMapped, botttomRightWheelChannel);

          /*
          SetServoPos(topLeftWheelMapped, topLeftWheelChannel);
          SetServoPos(bottomLeftWheelMapped, bottomLeftWheelChannel);
          */
          break;

        case 2:
          topRightWheelMapped = move('t', 'b', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          bottomRightWheelMapped = move('b', 'b', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          /*
          topLeftWheelMapped = move('t', 'b', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          bottomLeftWheelMapped = move('b', 'b', _raw_rc_values[RIGHT_JOYSTICK_Y]);
          */

          SetServoPos(topRightWheelMapped, topRightWheelChannel);
          SetServoPos(bottomRightWheelMapped, botttomRightWheelChannel);
          /*
          SetServoPos(topLeftWheelMapped, topLeftWheelChannel);
          SetServoPos(bottomLeftWheelMapped, bottomLeftWheelChannel);          
          */
          break;

        default:
          topRightWheelMapped = move('t', 'n', CRSF_MID_VALUE);
          bottomRightWheelMapped = move('b', 'n', CRSF_MID_VALUE);
          /*
          topLeftWheelMapped = move('t', 'n', CRSF_MID_VALUE);
          bottomLeftWheelMapped = move('b', 'n', CRSF_MID_VALUE);
          */

          SetServoPos(topRightWheelMapped, topRightWheelChannel);
          SetServoPos(bottomRightWheelMapped, botttomRightWheelChannel);
          /*
          SetServoPos(topLeftWheelMapped, topLeftWheelChannel);
          SetServoPos(bottomLeftWheelMapped, bottomLeftWheelChannel)
          */
      }
    }
  }
}