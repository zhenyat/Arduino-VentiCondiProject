/***************************************************************************
 *  VentiCondiApp: Connection of Ventilation System (Venti) Output voltage 
 *                 to Conditioner Controller (Condi) Input via Arduino Uno
 *
 *  Schema:   
 *                                      -----------------------
 *       + 0...10V                      |     Arduino Uno     |
 *       o-------------                 |                     |
 *                    |                 |               Pin 3 |    + 0...5 V                   
 *                    --                |                 o-----------o
 *     Venti          ||      + 0...5 V |                     |    0...255 PWM
 *                 R  ||<---------------|--o Pin A0           |
 *               10kΩ ||   0...1023 PWM |                     |      Condi
 *                    --                |                     |
 *       GND          |                 |  GND            GND |      GND
 *       o------------------------------|--o--------------o---|-------o                    
 *                                      -----------------------
 *            
 *  Input Analog signal:
 *    - Voltage range 0...5 V converted by the potentiometer (10kΩ) 
 *      from Venti Voltage range 0...10 V 
 *    - PWM range 0...1023
 *  
 *  Output Analog signal: 
 *    - Condi Voltage range 0...5 V
 *    -       PWM range     0...255
 *    - 7 levels of Conditioner Power (Performance)
 *   
 *        Input: Venti                           Output: Condi 
 *   ======================  ------------------------------------------------------    
 *   Original    Divided     Level  Control_Voltage   Power Coded_Voltage PWM_Units 
 *   Voltage  Voltage Units         Min  Mean   Max           Up  Down    Up  Down
 *     0.0       0.0     0     0   0.00        1.25    0   (1.20) 1.00   (61)   51
 *     0.1      0.05    10     1         1.75         10%   1.80  1.40    92    71  
 *     2.5      1.25   255     2         2.25         20%   2.50  2.00   128   102   
 *     4.5      2.25   460     3         2.75         30%   3.00  2.40   153   122
 *     6.0      3.00   613     4         3.25         50%   3.50  2.90   179   147     
 *     7.5      3.75   767     5         3.75         70%   4.00  3.50   204   178
 *     8.5      4.25   69      6         4.25         80%   4.40  4.20   224   214
 *     9.5      4.75   971     7   4.75        5.00  100%   4.80 (5.00)  245  (255)
 *   (10.0)    (5.00)(1023)
 *   
 *  Remarks to the Table:
 *  1) Coded_Voltage / PWM_Units to control the Condi are defined by measurements
 *  2) Values in parantheses () are not used in the Code below - just to complete arrays
 *
 *    Libraries used:
 *        ~/Arduino/libraries/
 *                       |
 *                       -- zt_venti_code
 *                       |
 *                       -- zt_io
 *    
 *    Analog output PWM frequency is increased from default 490 Hz to 3921.16 Hz
 *    see:  https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
 *                                                               
 *    02.08.2021  1.0.0   ZT    
 *    23.08.2021  3.3.2   Bug fixed
 *    31.08.2021  3.3.3   Tested & finalized
 *************************************************************************/
#include <zt_venti_condi.h>

/*  App params  */
const bool DEBUG = true;
const char* APP_VERSION = "3.3.3_1 (23.08.2021)";
char printIntValue[4];       // to print integer values with '%4d' format

/*  Arduino params  */
const unsigned int BAUD_RATE = 9600;
const unsigned int VENTI_PIN = A0; // Analog input  pin (Venti System)
const unsigned int CONDI_PIN = 3;  // Analog output pin (PWM for Condi)

/*  Venti params  */
const float VENTI_ORIGINAL_VOLTAGE_MAX = 10.0;
const unsigned int VENTI_UNIT_MAX = 1023;
const unsigned int VENTI_BOTTOM_PWM = 10; // 0.10- V meams that Venti requests   0% Power
//const unsigned int VENTI_TOP_PWM  = 1002; // 4.90+ V meams that Venti requests 100% Power 
const unsigned int VENTI_CHECK_BOTTOM_ATTEMPTS = 4;

/* Condi params */
bool isCondiOff = true;

/*  Times in seconds*/
const unsigned int DELAY_AFTER_CONDI_OFF      = 180;  // Minimum is 3 min  (Docs)
const unsigned int DELAY_AFTER_CONDI_ON       =  60;  // Minimum is 30 sec (experimental)
const unsigned int LOOP_DELAY                 =  10;
const unsigned int DELAY_WHILE_VENTI_SLEEPING = 300;
const unsigned int TIME_CHECKING_VENTI_STATUS =  10;

/*  Trend params  */
const unsigned int REGISTER_SIZE = 4;
unsigned int VENTI_REGISTER[REGISTER_SIZE] = {0}; // Venti PWM array for trend monitoring
String trend;

/*  Venti Levels  */
const unsigned int VENTI_LEVELS    = 9;
/* Ref:  Venti Original Voltage, V      0   0.1  2.50  4.50  6.0  7.50  8.50  9.50  10.00 */
const float VENTI_VOLTAGE[]        = {0.0, 0.05, 1.25, 2.25, 3.0, 3.75, 4.25, 4.75,  5.00};
const unsigned int VENTI_PWM[]     = {  0,   10,  255,  460, 613,  767,  869,  971,  1023};

/* Ref:  Condi Levels                         1     2     3    4     5     6     7 */
const unsigned int CONDI_LEVELS    = 8;
const unsigned int CONDI_POWER[]   = {  0,   10,   20,   30,  50,   70,   80,  100};

int actualCondiLevel;  // Condi level set on CONDI_PIN
int targetCondiLevel;  // Condi level requested by Venti

// Up:   Values to increase Level
const float CONDI_VOLTAGE_UP[]     = { 1.2, 1.8,  2.5,  3.0, 3.5,  4.0,  4.4,  4.8};
const unsigned int CONDI_PWM_UP[]  = {  61,  92,  128,  153, 179,  204,  224,  245};

// Down: Values to decrease Level
const float CONDI_VOLTAGE_DOWN[]    = {1.0, 1.4,  2.0,  2.4, 2.9,  3.5,  4.2,  4.8};
const unsigned int CONDI_PWM_DOWN[] = { 51,  71,  102,  122, 148,  179,  214,  245};

/****************
 * Setup Phase:
 *  - Increase the PWM frequency on Condi Pin
 *  - Reset Condi (switch Condi OFF & On with a proper delay time) on Venti request
 *  - Increasing Condi power smoothly to meet Venti request 
 ***************/
void setup() 
{
  TCCR2B = TCCR2B & B11111000 | B00000010; // PWM frequency: 3921.16 Hz (pins 3 & 11)

  /*  Initializing  */
  Serial.begin(BAUD_RATE);   // initialize serial communications at BAUD_RATE bps
  setOutputMode(CONDI_PIN);  // Reset to LOW
  sleep(2);                  // Just to avoid printing twice
  Printf("\n===== VentiCondiApp version %s starting...\n", APP_VERSION);

  /* Check the initial Venti status, waiting till Venti is active  */
  while (isVentiOnBottom(VENTI_PIN, VENTI_BOTTOM_PWM, VENTI_CHECK_BOTTOM_ATTEMPTS, TIME_CHECKING_VENTI_STATUS)) {
    if (DEBUG) Serial.println("..... Setup: Venti is at the Bottom...");
    sleep(DELAY_WHILE_VENTI_SLEEPING);  // Do nothing - just waiting...
  }
  
  /*  Reset Condi if Venti requests power */
  condiOff(CONDI_PIN, DELAY_AFTER_CONDI_OFF);
  Serial.println("+++++ ZT! After Condi OFF...\n");
  condiOn(CONDI_PIN, CONDI_PWM_UP[1], DELAY_AFTER_CONDI_ON);
  Serial.println("+++++ ZT! After Condi ON...\n");
  isCondiOff = false;
  actualCondiLevel = 1;

  unsigned int ventiInitialPwm = analogRead(VENTI_PIN);
  float ventiPowerRequest = ventiInitialPwm * VENTI_ORIGINAL_VOLTAGE_MAX / VENTI_UNIT_MAX * 10.;
  serial_printf(Serial, "===== Venti request at start: %1f\%%", ventiPowerRequest);

  /*  Increasing Condi power smoothly   */
  targetCondiLevel = getCondiLevel(ventiInitialPwm, VENTI_PWM, VENTI_LEVELS);

  if (targetCondiLevel > 0) {
    for (int level = 1; level <= targetCondiLevel; level++) {
      analogWrite(CONDI_PIN, CONDI_PWM_UP[level]);
      if (DEBUG) serial_printf(Serial,"\n----- Smooth Condi power increasing: %d\%%", CONDI_POWER[level]);
      sleep(LOOP_DELAY);
    }
  }
  actualCondiLevel = targetCondiLevel;
  Printf("\n===== VentiCondiApp is in the Loop mode now...\n");
}

void loop() 
{
  unsigned int ventiPwm   = 0;  // Value read from Analog Input
  float ventiPowerRequest = 0.; // Condi Power requested by Venti (%)
  Serial.println("+++++ ZT! Starting LOOP...\n");
  if (isVentiOnBottom(VENTI_PIN, VENTI_BOTTOM_PWM, VENTI_CHECK_BOTTOM_ATTEMPTS, TIME_CHECKING_VENTI_STATUS)) {
    if (DEBUG) Serial.println("\n..... Loop-start: Venti is at the Bottom...");
    if (!isCondiOff) {
      condiOff(CONDI_PIN, DELAY_AFTER_CONDI_OFF);
      isCondiOff = true;
      actualCondiLevel = 0;
    }
    if (DEBUG) {
      ventiPwm = analogRead(VENTI_PIN);
      serial_printf(Serial,"\n..... App is sleeping %d seconds, while Venti PWM = %d", DELAY_WHILE_VENTI_SLEEPING, ventiPwm);
    }
    sleep(DELAY_WHILE_VENTI_SLEEPING);
    
  } else {
    
    if (isCondiOff) {
      condiOn(CONDI_PIN, CONDI_PWM_UP[1], DELAY_AFTER_CONDI_ON);
      isCondiOff = false;
      actualCondiLevel = 1;
//      serial_printf(Serial, "\n!!!!! 1-After condiON: actualCondiLevel = %d\n", actualCondiLevel);
    }
    
    /*  Get and handle the next value from Venti  */
    ventiPwm          = analogRead(VENTI_PIN);
    ventiPowerRequest = ventiPwm * VENTI_ORIGINAL_VOLTAGE_MAX / VENTI_UNIT_MAX * 10.;
    
    shiftArrayLeft(VENTI_REGISTER, REGISTER_SIZE, ventiPwm);
    trend = getTrend(VENTI_REGISTER, REGISTER_SIZE);
    if (DEBUG) {
      sprintf(printIntValue, "%4d", ventiPwm);
      serial_printf(Serial, "\n-- Venti: PWM = %s, Request = %1f\%%; Trend: %s", printIntValue, ventiPowerRequest, trend.c_str());
    }

    targetCondiLevel = getCondiLevel(ventiPwm, VENTI_PWM, VENTI_LEVELS);
//    serial_printf(Serial, "\n!!!!! 2-After getCondiLevel: targetCondiLevel = %d, actualCondiLevel = %d\n", targetCondiLevel, actualCondiLevel);

    if (trend == "Standby") {
      if (DEBUG) serial_printf(Serial, "\t-- Condi: actual Power = %i\%%", CONDI_POWER[actualCondiLevel]);

    } else {

      if (trend == "Up") {
        if (targetCondiLevel > actualCondiLevel) {
          analogWrite(CONDI_PIN, CONDI_PWM_UP[targetCondiLevel]);   // Increase Condi power
          actualCondiLevel = targetCondiLevel;
          if (DEBUG) {
            sprintf(printIntValue, "%3d", CONDI_POWER[actualCondiLevel]);
            serial_printf(Serial, "\t-- Condi: Level = %i; Voltage = %2f; Power = %s\%%", actualCondiLevel, CONDI_VOLTAGE_UP[actualCondiLevel], printIntValue);
          }
        } else if (targetCondiLevel < actualCondiLevel) {   // Abnormal situation?
          if (DEBUG) serial_printf(Serial, "\t!!! While UP target level (%i) < actual level (%i)", targetCondiLevel, actualCondiLevel);
          analogWrite(CONDI_PIN, CONDI_PWM_DOWN[targetCondiLevel]);   // ! Decrease Condi power to required
          actualCondiLevel = targetCondiLevel;
          if (DEBUG) {
            sprintf(printIntValue, "%3d", CONDI_POWER[actualCondiLevel]);
            serial_printf(Serial, "\n\t\t\t\t\t\t-- Condi: Level = %i; Voltage = %2f; Power = %s\%%", actualCondiLevel, CONDI_VOLTAGE_DOWN[actualCondiLevel], printIntValue);
          }
        } else {  // targetCondiLevel = actualCondiLevel: do nothing
          if (DEBUG) serial_printf(Serial, "\t-- Condi: actual Power = %i\%%", CONDI_POWER[actualCondiLevel]);
        }
      } else {    // Down

        if (targetCondiLevel == 0) {                   // if Venti sends 0 V, don't rush to OFF Condi...
          if(isVentiOnBottom(VENTI_PIN, VENTI_BOTTOM_PWM, VENTI_CHECK_BOTTOM_ATTEMPTS, TIME_CHECKING_VENTI_STATUS)) {
           if (DEBUG) Serial.println("\n..... Loop-checking: Venti is on Bottom...\n");
           if (!isCondiOff) {
             condiOff(CONDI_PIN, DELAY_AFTER_CONDI_OFF);
             isCondiOff =  true;
             if (DEBUG) serial_printf(Serial,"\n..... App is sleeping %d seconds while Venti sleeping...", DELAY_WHILE_VENTI_SLEEPING);
             sleep(DELAY_WHILE_VENTI_SLEEPING);
           }
          }
        } else {
          if (targetCondiLevel < actualCondiLevel) {
            analogWrite(CONDI_PIN, CONDI_PWM_DOWN[targetCondiLevel]);  // Decrease Condi power
            actualCondiLevel = targetCondiLevel;
            if (DEBUG) {
              sprintf(printIntValue, "%3d", CONDI_POWER[actualCondiLevel]);
              serial_printf(Serial, "\t-- Condi: Level = %i; Voltage = %2f; Power = %s\%%", actualCondiLevel, CONDI_VOLTAGE_DOWN[actualCondiLevel], printIntValue);
            }
          } else if (targetCondiLevel > actualCondiLevel) {
            if (DEBUG) serial_printf(Serial, "\t!!! While DOWN target level (%i) > actual level (%i) - IGNORE!", targetCondiLevel, actualCondiLevel);
//            analogWrite(CONDI_PIN, CONDI_PWM_UP[targetCondiLevel]);   // ! Increase Condi power to required
//            actualCondiLevel = targetCondiLevel;
//            if (DEBUG) {
//              sprintf(printIntValue, "%3d", CONDI_POWER[actualCondiLevel]);
//              serial_printf(Serial, "\n\t\t\t\t\t-- Condi: Level = %i; Voltage = %2f; Power = %s\%%", actualCondiLevel, CONDI_VOLTAGE_DOWN[actualCondiLevel], printIntValue);
//            }
            // do nothing!!! 
          } else {    // targetCondiLevel = actualCondiLevel: do nothing
            if (DEBUG) serial_printf(Serial, "\t\-- Condi: actual Power = %i\%%", CONDI_POWER[actualCondiLevel]);
          }
        }
      }
    }
  }
  sleep(LOOP_DELAY);
}
