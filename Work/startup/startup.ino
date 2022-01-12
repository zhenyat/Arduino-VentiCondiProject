/***************************************************************************
 *  VentiCondi: Connection of Ventilation System (Venti) Output voltage 
 *              with Conditioner Controller (Condi) Input
 * 
 *  Input Analog signal:
 *      Venti Voltage range [0...10] V, converted to [0...5] V by the potentiometer (10kΩ)
 *  
 *  Output Analog signal:
 *      7 levels of Conditioner Power (Performance)
 *   
 *   Analog output PWM frequency is increased from default 490 Hz to 3921.16 Hz
 *    see:  https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
 *  
 *   Level  Condi_Control_Voltage  Condi_Power  Coded_Voltage  PWM_Units (of 255 as 100%)
 *            Min   Mean  Max                      Up  Down     Up  Down
 *      0     0.00        1.25       0%         (1.20) 1.00    (61)   51 
 *      1           1.75            10%          1.80  1.40     92    71
 *      2           2.25            20%          2.50  2.00    128   102
 *      3           2.75            30%          3.00  2.40    153   122
 *      4           3.25            50%          3.50  2.90    179   147
 *      5           3.75            70%          4.00  3.50    204   178
 *      6           4.25            80%          4.40  4.20    224   214
 *      7     4.75        5.00     100%          4.80 (5.00)   245  (255)
 *
 *   02.08.2021  0.3.0  ZT
 * /***************************************************************************/
#include <zt_venti_condi.h>

/*  App params  */
const bool DEBUG = true;

/*  Arduino params  */
const unsigned int BAUD_RATE = 9600;
const unsigned int VENTI_PIN = A0; // Analog input  pin (Venti System)
const unsigned int CONDI_PIN = 3;  // Analog output pin (PWM for Condi)

/*  Venti params  */
const unsigned int VENTI_UNIT_MIN = 0;
const unsigned int VENTI_UNIT_MAX = 1023;

/* Condi params */
const unsigned int CONDI_UNIT_MAX = 255;

const unsigned long DELAY_AFTER_CONDI_OFF = 180;  // Delay after Condi OFF in sec
const unsigned long DELAY_AFTER_CONDI_ON  =  60;  // Delay after Condi On  in sec
const unsigned long DELAY_AFTER_LEVEL_SET =  10;  // Delay after Condi On  in sec

const float CONDI_VOLTAGE_MIN = 1.0;
const float CONDI_VOLTAGE_MAX = 5.0;

/*  Trend params  */
const unsigned int REGISTER_SIZE = 4;
unsigned int VENTI_REGISTER[REGISTER_SIZE] = {0}; // Venti PWM array for trend definition

/*  Venti Levels  */
const unsigned int VENTI_LEVELS         = 9;
const unsigned int VENTI_VOLTAGE[]      = {0.0, 0.1, 1.25, 2.25, 3.0, 3.75, 4.25, 4.75, 5.00};
const unsigned int VENTI_PWM[]          = {  0, 205,  255,  460, 613,  767,  869,  971, 1023};

/*  Condi Levels                                  1     2     3    4     5     6     7 */
const unsigned int CONDI_LEVELS         = 8;
const unsigned int CONDI_POWER[]        = {  0,  10,   20,   30,  50,   70,   80,  100};

const float CONDI_VOLTAGE_UP_MAX[]      = {1.2, 1.8,  2.5,  3.0, 3.5,  4.0,  4.4,  4.8};
const unsigned int CONDI_PWM_UP_MAX[]   = { 61,  92,  128,  153, 179,  204,  224,  245};

const float CONDI_VOLTAGE_DOWN_MIN[]    = {1.0, 1.4,  2.0,  2.4, 2.9,  3.5,  4.2, 5.0};
const unsigned int CONDI_PWM_DOWN_MIN[] = { 51,  71,  102,  122, 148,  179,  214, 255};

/****************
 * Setup Phase:
 *  - Increase the PWM frequency on Condi Pin
 *  - Switch OFF / On Condi with a proper delay time
 *  - Inc increasing Condi power smoothly if Venti requires maximum 
 * 
 ***************/
void setup() 
{
  TCCR2B = TCCR2B & B11111000 | B00000010; // PWM frequency: 3921.16 Hz (pins 3 & 11)

  Serial.begin(BAUD_RATE);   // initialize serial communications at BAUD_RATE bps
  setOutputMode(CONDI_PIN);  // Reset to LOW
  sleep(2);                  // Just to avoid printing twice
  Serial.println("===== VentiCondi starting...");

  condiOff(CONDI_PIN, DELAY_AFTER_CONDI_OFF);  // Condi to OFF at start!
  
  //unsigned int pwmOn = CONDI_VOLTAGE_MIN * CONDI_UNIT_MAX / CONDI_VOLTAGE_MAX;
  unsigned int pwmOn = CONDI_PWM_UP_MAX[1];
  condiOn(CONDI_PIN, pwmOn, DELAY_AFTER_CONDI_ON);   // Condi is ON with starting PWM Value

  /*  If Venti sends 10 V increasing Condi power smoothly   */
  unsigned int level = 0;
  while (isVentiOnTop(VENTI_PIN) && level < CONDI_LEVELS-1) {
    level++;
    analogWrite(CONDI_PIN, CONDI_PWM_UP_MAX[level]);   // Increase Condi power smoothly
    if (DEBUG) {
      serial_printf(Serial, "*****  Increasing smoothly Condi level: %i; Voltage = %3f\n", level, CONDI_VOLTAGE_UP_MAX[level]);
    }
    sleep(DELAY_AFTER_LEVEL_SET);
  }
  serial_printf(Serial, "===== Initial Condi Power: %i\n", CONDI_POWER[level]);
}

void loop() 
{
  /*  Get and handle the next value from Venti  */
  unsigned int ventiPwm = analogRead(VENTI_PIN);
  double ventiVoltage = ventiPwm * 10.0 / VENTI_UNIT_MAX;
  shiftArrayLeft(VENTI_REGISTER, REGISTER_SIZE, ventiPwm);
  String trend = getTrend(VENTI_REGISTER, REGISTER_SIZE);
  if (DEBUG) serial_printf(Serial, "-- Venti: PWM = %i, Voltage = %2f; Trend: %s", ventiPwm, ventiVoltage,  trend.c_str());

  if (trend == 0) {
    // Standby
  } else {
    int level = GetLevel(ventiPwm);
    if (trend == 1) {
      analogWrite(CONDI_PIN, CONDI_PWM_UP_MAX[level]);   // Increase Condi power
      if (DEBUG) serial_printf(Serial, "\t-- Condi: Level = %i; Voltage = %2f; Power = %d\n", level, CONDI_VOLTAGE_UP_MAX[level], CONDI_POWER[level]);
    } else {
      analogWrite(CONDI_PIN, CONDI_PWM_DOWN_MIN[level]);   // Increase Condi power
      if (DEBUG) serial_printf(Serial, "\t-- Condi: Level = %i; Voltage = %2f; Power = %d\n", level, CONDI_VOLTAGE_DOWN_MIN[level], CONDI_POWER[level]);
    }
  }

  sleep(DELAY_AFTER_LEVEL_SET);
}

/***  Identifies Level according to Venti PWM signal  ***/
int GetLevel(unsigned int pwm) {
  for (int i=0; i < VENTI_LEVELS; i++) {
    if (VENTI_PWM[i] <= pwm && pwm < VENTI_PWM[i+1]) return i;
  }
  return -1;
}
