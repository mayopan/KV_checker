/*
    KV checker for R/C
    This program is for the original KV checker board using Seeed XIAO.
    Schematic of the board is shown in github -> https://github.com/mayopan/KV_checker

    It counts rotation of tire using photo reflector. You have to put some light color musking tape on the tire.
    It also measures voltage of battery with adc.
    Parameters listed below is modified by Setting mode with UP/DOWN/SELECT buttons.

      *Pinion Gear Teeth number
      *Spar Gear Teeth number
      .1st Gear ratio (calculated with the two gear's number. Not set manually)
      *2nd Gear ratio
      *Tire dia

    KV is calculated from tire rotation speed with gear ratio parameter.
    So you have to set the gear ratio parameter correctly to get true motor KV.
    KVT is easier to check the equality of the top speed performance per voltage that includes motor KV and gear ratio.
    For example, if the race regulation says KV < 2800rpm/V and gear ratio > 7,
    KVT < 2800 / 7 = 400.

    It has 6 mode for measure listed below.

      *KV       [rpm/V]     ...Rotation speed of Motor / Battery voltage
      *KVT      [rpm/V]     ...Rotation speed of Tire / Battery voltage
      *VOLTAGE  [V]         ...Battery voltage
      *SPD      [km/h]      ...Speed
      *TIRE RPM [rpm]       ...Rotation speed of tire
      *MOTOR RPM[rpm]       ...Rotation speed of motor

*/

#include <Arduino.h>
#include <TM1637Display.h>
#include <TimerTC3.h>

//---------------------- Pin layout ----------------------//
#define CLK 9
#define DIO 10
#define VBAT 3
#define BUTTON_DOWN 6
#define BUTTON_UP 5
#define BUTTON_SELECT 4
#define ROT 7

#define AVERAGE_COUNT 10 // durations = 1.7ms/r @ 40km/h, 62mm tire. avarage 10 round takes 17ms.
#define VOLT_AVERAGE 40
#define VOLT_MEASURE_DURATION 1000 // us for timer interrupt = 1kHz
#define FPS 5                      // Frame rate for display

// Max Voltage of ADR is set to 1V. Battery voltage: 0-8.5V
// The voltage divider resistors are R1:6.8k & R2:51k  (51+6.8)/6.8=8.5
#define VOLTAGE_DIVIDER_RATIO 8.5

// Duration in 1 round with 62mm dia tire sould be larger than 7000us when VMAX < 80[km/h]
// VMAX 80[km/h] / 3600[s/h] * 1e6[mm/km] / (62 * 3.14)[mm/r] = 114[r/s] = 8772[us/r]
// So shorter detection than 7000us will be noise and ignored.
#define IGNORE_DURATION 7000

enum measure_mode
{
  MODE_KV,
  MODE_KVT,
  MODE_VOLTAGE,
  MODE_SPD,
  MODE_RPM_TIRE,
  MODE_RPM_MOTOR,
  MODE_GRSET
};

enum parameter_set_mode
{
  SET_PINION,
  SET_SPAR,
  COMFIRM_FIRST_GEAR_RATIO,
  SET_SECOND_GEAR_RATIO,
  SET_TIRE_DIA,
  COMFIRM_FINAL_GEAR_RATIO
};

volatile uint32_t lastTime = 0;
volatile int rotation_count = 0;
volatile float volt = 0;
volatile uint32_t d = 1;
uint32_t disp_lasttime = 0;
volatile int mode = MODE_KV;
int set_mode = COMFIRM_FINAL_GEAR_RATIO;

class Parameter
{
public:
  float pinion_T = 1;
  float spar_T = 1;
  float first_gear_ratio = 1;
  float second_gear_ratio = 1.0;
  float total_gear_ratio = 1.0;
  int tire_dia = 63;

  void init(float pinion, float spar, float second_gr, int tire)
  {
    pinion_T = pinion;
    spar_T = spar;
    second_gear_ratio = second_gr;
    tire_dia = tire;
    final_gr_calc();
  };

  float final_gr_calc()
  {
    first_gear_ratio = spar_T / pinion_T;
    total_gear_ratio = first_gear_ratio * second_gear_ratio;
    return total_gear_ratio;
  };
};

const uint8_t SEG_KV[] = {
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // K
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // V
    0, 0};
const uint8_t SEG_KVT[] = {
    SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // K
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // V
    SEG_D | SEG_E | SEG_F | SEG_G,         // t
    0};
const uint8_t SEG_GR[] = {
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F,         // G
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    0, 0};
const uint8_t SEG_SPD[] = {
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G, // S
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G, // P
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, // d
    0};
const uint8_t SEG_VOLT[] = {
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,         // V
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O
    SEG_D | SEG_E | SEG_F,                         // L
    SEG_D | SEG_E | SEG_F | SEG_G                  // t
};
const uint8_t SEG_TIRE[] = {
    SEG_D | SEG_E | SEG_F | SEG_G,                 // t
    SEG_E | SEG_F,                                 // I
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G          // E
};
const uint8_t SEG_RPM_T[] = {
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,         // P
    SEG_C | SEG_E | SEG_G,                         // m
    SEG_D | SEG_E | SEG_F | SEG_G                  // t
};
const uint8_t SEG_RPM_M[] = {
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,         // P
    SEG_C | SEG_E | SEG_G,                         // m
    SEG_C | SEG_E | SEG_G                          // m
};
const uint8_t SEG_PINION[] = {
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G, // P
    SEG_E | SEG_F,                         // I
    SEG_C | SEG_E | SEG_G,                 // n
    0};
const uint8_t SEG_SPAR[] = {
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,         // S
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,         // P
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    0};
const uint8_t SEG_FIRST_GEAR_RATIO[] = {
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F,         // G
    SEG_B | SEG_E | SEG_G,                         //'/'
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    SEG_B | SEG_C,                                 // 1
};
const uint8_t SEG_SECOND_GEAR_RATIO[] = {
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F,         // G
    SEG_B | SEG_E | SEG_G,                         //'/'
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,         // 2
};
const uint8_t SEG_TOTAL_GEAR_RATIO[] = {
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F,         // G
    SEG_B | SEG_E | SEG_G,                         //'/'
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G, // R
    SEG_C | SEG_D | SEG_E | SEG_G,                 // o
};

TM1637Display display(CLK, DIO);
Parameter reduction;

void showMode()
{
  switch (mode)
  {
  case MODE_KV:
    display.setSegments(SEG_KV, 4);
    break;
  case MODE_KVT:
    display.setSegments(SEG_KVT, 4);
    break;
  case MODE_GRSET:
    display.setSegments(SEG_GR, 4);
    break;
  case MODE_SPD:
    display.setSegments(SEG_SPD, 4);
    break;
  case MODE_VOLTAGE:
    display.setSegments(SEG_VOLT, 4);
    break;
  case MODE_RPM_TIRE:
    display.setSegments(SEG_RPM_T, 4);
    break;
  case MODE_RPM_MOTOR:
    display.setSegments(SEG_RPM_M, 4);
    break;
  default:
    break;
  }
  return;
}

void show_parameter()
{
  switch (set_mode)
  {
  case SET_PINION:
    display.setSegments(SEG_PINION, 4);
    delay(1000);
    display.showNumberDec(int(reduction.pinion_T), false);
    break;
  case SET_SPAR:
    display.setSegments(SEG_SPAR, 4);
    delay(1000);
    display.showNumberDec(int(reduction.spar_T), false);
    break;
  case COMFIRM_FIRST_GEAR_RATIO:
    display.setSegments(SEG_FIRST_GEAR_RATIO, 4);
    delay(1000);
    display.showNumberDec(int(reduction.first_gear_ratio), false);
    break;
  case SET_SECOND_GEAR_RATIO:
    display.setSegments(SEG_SECOND_GEAR_RATIO, 4);
    delay(1000);
    display.showNumberDecEx(int(reduction.second_gear_ratio * 10), 0b00100000, false);
    break;
  case SET_TIRE_DIA:
    display.setSegments(SEG_TIRE, 4);
    delay(1000);
    display.showNumberDec(reduction.tire_dia, false);
    break;
  case COMFIRM_FINAL_GEAR_RATIO:
    display.setSegments(SEG_TOTAL_GEAR_RATIO, 4);
    delay(1000);
    display.showNumberDecEx(int(reduction.total_gear_ratio * 1000), 0b10000000, false);
    break;
  default:
    break;
  }
  return;
}

// Interrupt handler //
void rotation_Counter() // Interrupt from counter
{
  volatile uint32_t t_micro = micros();

  if (t_micro - lastTime > IGNORE_DURATION)
  {
    d = (d * (AVERAGE_COUNT - 1) + t_micro - lastTime) / AVERAGE_COUNT;
    lastTime = t_micro;
    rotation_count++;
  }
}

void measure_mode_change() // Interrupt from button input
{
  if (mode < MODE_GRSET)
  {
    mode++;
    showMode();
  }
}

void getVoltage() // Interrupt for adc of voltage
{
  volt = (volt * (VOLT_AVERAGE - 1) + (float)analogRead(VBAT) / 4096 * VOLTAGE_DIVIDER_RATIO) / VOLT_AVERAGE;
}

void set_parameter()
{
  show_parameter();
  while (set_mode < COMFIRM_FINAL_GEAR_RATIO)
  {
    while (digitalRead(BUTTON_SELECT))
    {
      switch (set_mode)
      {
      case SET_PINION:
        if (!digitalRead(BUTTON_DOWN))
        {
          reduction.pinion_T -= 1.0;
          display.showNumberDec(int(reduction.pinion_T), false);
          delay(500);
        }
        else if (!digitalRead(BUTTON_UP))
        {
          reduction.pinion_T += 1.0;
          display.showNumberDec(int(reduction.pinion_T), false);
          delay(500);
        }
        break;
      case SET_SPAR:
        if (!digitalRead(BUTTON_DOWN))
        {
          reduction.spar_T -= 1.0;
          display.showNumberDec(int(reduction.spar_T), false);
          delay(500);
        }
        else if (!digitalRead(BUTTON_UP))
        {
          reduction.spar_T += 1.0;
          display.showNumberDec(int(reduction.spar_T), false);
          delay(500);
        }
        break;
      case COMFIRM_FIRST_GEAR_RATIO:
        reduction.final_gr_calc();
        display.showNumberDecEx(int(reduction.first_gear_ratio * 1000), 0b10000000, false);
        delay(500);
        break;
      case SET_SECOND_GEAR_RATIO:
        if (!digitalRead(BUTTON_DOWN))
        {
          reduction.second_gear_ratio -= 0.1;
          display.clear();
          display.showNumberDecEx(int(reduction.second_gear_ratio * 10), 0b00100000, false);
          delay(500);
        }
        else if (!digitalRead(BUTTON_UP))
        {
          reduction.second_gear_ratio += 0.1;
          display.clear();
          display.showNumberDecEx(int(reduction.second_gear_ratio * 10), 0b00100000, false);
          delay(500);
        }
        display.showNumberDecEx(int(reduction.second_gear_ratio * 10), 0b00100000, false);
        delay(50);
        break;
      case SET_TIRE_DIA:
        if (!digitalRead(BUTTON_DOWN))
        {
          reduction.tire_dia -= 1;
          display.showNumberDec(reduction.tire_dia, false);
          delay(500);
        }
        else if (!digitalRead(BUTTON_UP))
        {
          reduction.tire_dia += 1;
          display.showNumberDec(reduction.tire_dia, false);
          delay(500);
        }
        break;
      default:
        break;
      }
    }
    reduction.final_gr_calc();
    set_mode++;
    show_parameter();
  }

  while (digitalRead(BUTTON_SELECT))
  {
  }
  set_mode = SET_PINION;
  mode = MODE_KV;
  delay(500);
  attachInterrupt(BUTTON_SELECT, measure_mode_change, FALLING);
  attachInterrupt(ROT, rotation_Counter, RISING);
  TimerTc3.attachInterrupt(getVoltage);
  return;
}

void setup()
{
  pinMode(ROT, INPUT);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  pinMode(VBAT, INPUT);

  //------------------- Initializing ADC ---------------------//
  /* Loading caliblation data */
  uint32_t bias = (*((uint32_t *)ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *)ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *)ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY)
  {
  };
  /* Write the calibration data. */
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  /* Set 12bit resolution on 1V full scale. It means LSB = 0.244mV (at ADC) -> 0.244 x 8.5 (voltage divider ratio) = 2.074mV (at Input)*/
  analogReference(AR_INTERNAL1V0);
  analogReadResolution(12);

  //-------------------- Set measuring parameter --------------//
  reduction.init(28, 103, 1.9, 63);

  /* Display parameter (Total gear ratio only) */
  display.setBrightness(7);
  display.clear();
  display.setSegments(SEG_TOTAL_GEAR_RATIO, 4);
  delay(1000);
  display.showNumberDecEx(int(reduction.total_gear_ratio * 1000), 0b10000000, false);
  delay(2000);

  //--------------------- Interrupt task set -------------------//
  attachInterrupt(ROT, rotation_Counter, RISING);
  attachInterrupt(BUTTON_SELECT, measure_mode_change, FALLING);
  TimerTc3.initialize(VOLT_MEASURE_DURATION);
  TimerTc3.attachInterrupt(getVoltage);
  disp_lasttime = millis();
}

void loop()
{
  int rpm_tire;
  int rpm_motor;
  float spd;
  int kv;
  int kvtx10;
  if (rotation_count >= AVERAGE_COUNT) // Triggered by rotation_Counter interrupt when AVERAGE_COUNT round is detected
  {
    rpm_tire = (int)(1e6 / d * 60);
    kvtx10=(int)(rpm_tire*10/volt);
    rpm_motor = (int)(rpm_tire * reduction.total_gear_ratio);
    kv = (int)(rpm_motor / volt);
    spd = 3.1415 * reduction.tire_dia / 1000000 * rpm_tire * 60;

    switch (mode)
    {
    case MODE_KV:
      display.showNumberDec(kv, false);
      break;
    case MODE_KVT:
      display.showNumberDecEx(kvtx10,0b00100000, false);
      break;
    case MODE_VOLTAGE:
      display.showNumberDecEx((int)(volt * 1000), 0b10000000, true);
      break;
    case MODE_RPM_TIRE:
      display.showNumberDec(rpm_tire, false);
      break;
    case MODE_RPM_MOTOR:
      if (rpm_motor < 10000)
        display.showNumberDec(rpm_motor, false);
      else if (rpm_motor < 100000)
        display.showNumberDecEx(rpm_motor / 10, 0b01000000, false);
      else
        display.showNumberDecEx(rpm_motor / 100, 0b11110000, false);
      break;
    case MODE_SPD:
      display.showNumberDecEx((int)(spd * 100), 0b01000000, false);
      break;
    default:
      break;
    }
    disp_lasttime = millis();
    rotation_count = 0;
  }
  else
  {
    if (mode == MODE_GRSET)
    {
      /* In setting mode, all interrupts are stopped */
      detachInterrupt(ROT);
      detachInterrupt(BUTTON_SELECT);
      TimerTc3.detachInterrupt();
      showMode();
      delay(1000);
      set_parameter();
    }
    else if (millis() - disp_lasttime > 1000 / FPS)
    {
      if (mode != MODE_VOLTAGE)
      {
        showMode();
      }
      else
      {
        display.showNumberDecEx((int)(volt * 1000), 0b10000000, true);
      }
      disp_lasttime = millis();
    }
  }
  delay(1);
}