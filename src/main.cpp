#include <Arduino.h>
#include "crsf.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);


#define RXD2 16
#define TXD2 17

#define SBUS_BUFFER_SIZE 25
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t _rcs_buf[SBUS_BUFFER_SIZE] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

// 9685 PWM Pins (sent as control signals over I2C)
int twistNeckPin = 0;

int leftEyeBlinkPin = 2;
int rightEyeBlinkPin = 3;

int leftEyeHorizontalPin = 4;
int rightEyeHorizontalPin = 5;

int leftNeckPin = 6;
int rightNeckPin = 7;


void SetServoPos(float percent, int pwmChannel)
{
    // 50 cycles per second 1,000ms / 50 = 100 /5 = 20ms per cycle
    // 1ms / 20ms = 1/20 duty cycle
    // 2ms / 20ms = 2/20 = 1/10 duty cycle


    // using 16 bit resolution for PWM signal convert to range of 0-65536 (0-100% duty/on time)
    // 1/20th of 65536 = 3276.8
    // 1/10th of 65536 = 6553.6

    // For PCA9685 control the values range from 0-4096 for the duty on time but percentage is the same as above

    uint32_t duty = map(percent, -1, 100, 1.0/20.0*4096, 2.0/20.0*4096);

    if(duty > 4095) duty = 4095;
    if(duty <= 1 ) duty = 1;
    // Serial.println("duty: "+String(duty));
    
    pwm.setPWM(pwmChannel, 0, duty);
    // ledcWrite(pwmChannel, duty);
}

void centerAllServos() {
  for (int i = 0; i < 15; i++)
  {
          SetServoPos(50, i);
  }
}

void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  // Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(420000, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
  
  Wire.begin();
  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  // pwm.setOscillatorFrequency(27000000);
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  centerAllServos();
  Serial.begin(460800);
}


void logChannelValues() {
  Serial.print("Channel 1: ");
  Serial.print(_raw_rc_values[0]);
  Serial.print("\tChannel 2: ");
  Serial.print(_raw_rc_values[1]);
  Serial.print("\tChannel 3: ");
  Serial.print(_raw_rc_values[2]);
  Serial.print("\tChannel 4: ");
  Serial.print(_raw_rc_values[3]);
  Serial.print("\tChannel 5: ");
  Serial.println(_raw_rc_values[4]);
}


void loop() { //Choose Serial1 or Serial2 as required
  // Serial.println("looping");
  while (Serial2.available()) {
    size_t numBytesRead = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if(numBytesRead > 0)
    {
      crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );

      logChannelValues();

      int aileronsMapped = map(_raw_rc_values[0], 1000, 2000, 0, 100);
      int elevatorMapped = map(_raw_rc_values[1], 1000, 2000, 0, 100);
      int throttleMapped = map(_raw_rc_values[2], 1000, 2000, 0, 100);
      int rudderMapped = map(_raw_rc_values[3], 1000, 2000, 0, 100);
      int switchMapped = map(_raw_rc_values[4], 1000, 2000, 0, 100);

      SetServoPos(aileronsMapped, leftEyeHorizontalPin);
      SetServoPos(aileronsMapped, rightEyeHorizontalPin);

      SetServoPos(elevatorMapped, leftNeckPin);
      SetServoPos(elevatorMapped, rightNeckPin);

      SetServoPos(100-throttleMapped, leftEyeBlinkPin);
      SetServoPos(throttleMapped, rightEyeBlinkPin);
      SetServoPos(rudderMapped, twistNeckPin);
    }
  }
}