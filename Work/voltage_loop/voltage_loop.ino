/***********************************************************
 * voltage_loop
 * 
 * Loop to set Voltage to Condi via Arduino
 * 
 * 30.07.2021 ZT
 * 10.08.2021 Last update
 ***********************************************************/
//#include <zt_io.h>
#include <zt_venti_condi.h>

const unsigned int baudRate     = 9600;

const unsigned int  inputPin = A0; // Analog input  pin (Venti System)
const unsigned int outputPin = 3;  // Analog output pin (PWM for Condi)

const unsigned int  inputUnitMin = 0;
const unsigned int  inputUnitMax = 1023;
const unsigned int outputUnitMax = 255;

const float  outputVoltageMin = 1.0;
const float  outputVoltageMax = 5.0;
const float  voltageIncrement = 0.5;

const unsigned int ARRAY_SIZE = 4;

unsigned int pwms[ARRAY_SIZE] = {0};
int    trend   = 1;
double voltage = outputVoltageMin;
double unitFactor;
unsigned int outputUnitMin = outputVoltageMin * unitFactor;

void setup() {
  TCCR2B = TCCR2B & B11111000 | B00000010; // PWM frequency: 3921.16 Hz (pins 3 & 11)
  Serial.begin(baudRate);
  setOutputMode(outputPin);
  sleep(2);
  Serial.println("===== Starting Voltage loop");
  
  unitFactor = outputUnitMax / outputVoltageMax;
  unsigned int pwmOn = outputVoltageMin * unitFactor;
  condiOn(outputPin, pwmOn, 30);
}

void loop() {
  unsigned int pwmUnits;
  
  voltage  = voltage + voltageIncrement * trend;
  pwmUnits = voltage * outputUnitMax / outputVoltageMax;
  
  if (pwmUnits >= outputUnitMax) {
    pwmUnits = outputUnitMax;
    voltage  = outputVoltageMax;
    trend = -1;
  } else if (pwmUnits <= outputUnitMin) {
    pwmUnits = outputUnitMin;
    voltage  = outputVoltageMin;
    trend = 1;
  } else {
    // Nothing...
  }
  analogWrite(outputPin, pwmUnits);
  serial_printf(Serial, "V / PWM: %3f / %d\n", voltage, pwmUnits);
  sleep(4);
}
