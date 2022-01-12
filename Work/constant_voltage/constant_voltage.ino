/***********************************************************
 * constatnt_voltage
 * 
 * Sets constant voltage via Arduino PWM for Condi controller
 * 
 * 15.11.2021 ZT
 ***********************************************************/
#include <zt_io.h>

const unsigned int outputPwmPin = 3;   // Output PWM pin
int count = 0;

void setup() {
  TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  Serial.begin(9600);
  sleep(2);
  Serial.println("===== Starting 1.8V...");
  unsigned int pwm = 91;
  unsigned int count = 0;
//  analogWrite(outputPwmPin, pwm);
//  for (int i=0; i<5; i++) {
//    Serial.println(pwm);
//    sleep(5);
//  }
}

void loop() {
  count++;
  unsigned int pwmUnits = 91;   //  1.8V -> 10%
  analogWrite(outputPwmPin, pwmUnits);
  serial_printf(Serial, "Count: %i: PWM = %i\n", count, pwmUnits);
  sleep(60);
}
