/***********************************************************
 * enter_voltage
 * 
 * Sets voltage via Arduino PWM for Condi controller
 * The voltage value is read from the Serial port
 * 
 * 15.07.2021 ZT
 * 30.07.2021 Update: library functions are used
 * 10.08.2021 Last update
 ***********************************************************/
#include <zt_io.h>

const unsigned int outputPwmPin = 3;   // Output PWM pin
int count = 0;

void setup() {
  TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  Serial.begin(9600);
  sleep(2);
  Serial.println("++++ ZT! Go...");
  unsigned int pwm = 92;
  analogWrite(outputPwmPin, pwm);
  for (int i=0; i<5; i++) {
    Serial.println(pwm);
    sleep(5);
  }
  Serial.println("===== Starting: enter Voltage (float number)");
}

void loop() {
  double voltage        = atof(getSerialInput());
  unsigned int pwmUnits = voltage * 255 /5.0;

  if (pwmUnits != 0) {
    count++;
    analogWrite(outputPwmPin, pwmUnits);
    serial_printf(Serial, "Count: %i: V = %2f; PWM = %i\n", count, voltage, pwmUnits);
    sleep(5);
  }
  
  sleep(10);
  Serial.println("Next");
}
