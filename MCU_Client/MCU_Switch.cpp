/*
Copyright © 2017 Silvair Sp. z o.o. All Rights Reserved.
 
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*******************************************
 * INCLUDES                                *
 *******************************************/

#include "MCU_Switch.h"
#include "Arduino.h"
#include "Config.h"
#include "LCD.h"
#include "Mesh.h"
#include <limits.h>
#include <stdint.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Buttons placement definitions
 */
#define PB_ON                       PIN_SW_1          /**< Defines ON button location. */
#define PB_OFF                      PIN_SW_2          /**< Defines OFF button location. */
#define PB_LC_MODE_ON               PIN_ENCODER_SW    /**< Defines LC MODE button location. */
#define PB_ENCODER_A                PIN_ENCODER_A     /**< Defines Encoder A pin location. */
#define PB_ENCODER_B                PIN_ENCODER_B     /**< Defines Encoder B pin location. */

/**
 * Light Lightness Client configuration
 */
#define LIGHTNESS_MIN               0            /**< Defines lower range of light lightness. */
#define LIGHTNESS_MAX               UINT16_MAX   /**< Defines upper range of light lightness. */
#define LIGHTNESS_MIN_CHANGE        0x500        /**< Defines the lowest reported changed. */
#define LIGHTNESS_INTVL_MS          100          /**< Defines the shortest interval beetwen two LightLightness messages. */

/**
 * On Off communication properties
 */
#define ON_OFF_TRANSITION_TIME_MS   0            /**< Defines on off transition time */
#define ON_OFF_NUMBER_OF_REPEATS    6            /**< Defines on off number of repeats while sending mesh message request */

/**
 * Generic Delta Client step
 */
#define DELTA_STEP_VALUE            0x1500       /**< Defines Generic Delta minimal step */
#define DELTA_INTVL_MS              100          /**< Defines the shortest interval beetwen two Delta Set messages. */
#define DELTA_NEW_TID_INTVL         350          /**< Defines the shortest interval beetwen generation of new TID. */

/**
 * Default communication properties
 */
#define DEFAULT_NUMBER_OF_REPEATS   2            /**< Defines default number of repeats while sending mesh message request */
#define DEFAULT_TRANSITION_TIME_MS  100          /**< Defines default transition time */
#define DEFAULT_DELAY_TIME_MS       50           /**< Defines default delay time (implies repeats) */


#define LIGHT_LC_MODE_OFF           0x00         /**< Defines Light LC Mode Off payload */
#define LIGHT_LC_MODE_ON            0x01         /**< Defines Light LC Mode On payload */
#define GENERIC_OFF                 0x00         /**< Defines Generic OnOff Off payload */
#define GENERIC_ON                  0x01         /**< Defines Generic OnOff On payload */

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static volatile bool LightLcMode              = false;               /**< Implies if LightLC Mode button has been pushed */
static volatile bool On                       = false;               /**< Implies if Generic ON button has been pushed */
static volatile bool Off                      = false;               /**< Implies if Generic OFF button has been pushed */
static volatile int  Encoder                  = 0;                   /**< Implies if Encoder position has changed */
static uint8_t       LightLcClientInstanceIdx = INSTANCE_INDEX_UNKNOWN;

/********************************************
 * LOCAL FUNCTION PROTOTYPES                *
 ********************************************/

/*
 *  Light Controller Mode On button interrupt handler
 */
static void InterruptLCModeOnPBClick(void);

/*
 *  Generic On button interrupt handler
 */
static void InterruptOnPBClick(void);

/*
 *  Generic Off button interrupt handler
 */
static void InterruptOffPBClick(void);

/*
 *  Encoder pin A interrupt handler
 */
static void InterruptEncoderA(void);

/*
 *  Encoder pin B interrupt handler
 */
static void InterruptEncoderB(void);

/*
 *  Check if lightness need an update
 *
 *  @param lightness_actual  Actual lightness value
 */
static bool HasLightnessChanged(int lightness_actual);

/*
 *  Get set lightness value
 */
static uint16_t LightnessGet(void);

/*
 *  Print lightness on debug interface
 *
 *  @param lightness_actual  Actual lightness value
 */
static void PrintLightness(unsigned lightness_actual);

/********************************************
 * LOCAL FUNCTIONS DEFINITIONS              *
 ********************************************/

static void InterruptLCModeOnPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_LC_MODE_ON)) return;

  LightLcMode = true;
}

static void InterruptOnPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_ON)) return;

  On = true;
}

static void InterruptOffPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_OFF)) return;

  Off = true;
}

static void InterruptEncoderA(void)
{
  if (digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  delayMicroseconds(ENCODER_DEBOUNCE_TIME_US);
  if (digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  Encoder--;

  while(!(digitalRead(PB_ENCODER_A) && digitalRead(PB_ENCODER_B))) { }
}

static void InterruptEncoderB(void)
{
  if (digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  delayMicroseconds(ENCODER_DEBOUNCE_TIME_US);
  if (digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  Encoder++;

  while(!(digitalRead(PB_ENCODER_A) && digitalRead(PB_ENCODER_B))) { }
}

static bool HasLightnessChanged(int lightness_actual)
{
  static int  prev_lightness          = LIGHTNESS_MIN;
  static bool is_prev_lightness_valid = false;

  if (!is_prev_lightness_valid)
  {
    prev_lightness          = lightness_actual;
    is_prev_lightness_valid = true;
    return false;
  }
  else if ((abs(lightness_actual - prev_lightness) > LIGHTNESS_MIN_CHANGE)
           || ((lightness_actual == 0) && (prev_lightness != 0)))
  {
    prev_lightness = lightness_actual;
    return true;
  }
  else
  {
    return false;
  }
}

static uint16_t LightnessGet(void)
{
  const float coefficient        = (float)(LIGHTNESS_MAX-LIGHTNESS_MIN) / (float)(ANALOG_MAX-ANALOG_MIN);
  uint16_t    analog_measurement = analogRead(PIN_ANALOG);

  return coefficient * (analog_measurement - ANALOG_MIN) + LIGHTNESS_MIN;
}

static void PrintLightness(unsigned lightness_actual)
{
  unsigned percentage = (lightness_actual * 100.0) / LIGHTNESS_MAX;
  DEBUG_INTERFACE.printf("Current Lightness is %d (%d%)\n", lightness_actual, percentage);
}

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void SetInstanceIdxSwitch(uint8_t idx)
{
  LightLcClientInstanceIdx = idx;
}

uint8_t GetInstanceIdxSwitch(void)
{
  return LightLcClientInstanceIdx;
}

void SetupSwitch(void)
{
  DEBUG_INTERFACE.println("Switch initialization.\n");
  pinMode(PB_ON,         INPUT_PULLUP);
  pinMode(PB_OFF,        INPUT_PULLUP);
  pinMode(PB_LC_MODE_ON, INPUT_PULLUP);
  pinMode(PB_ENCODER_A,  INPUT_PULLUP);
  pinMode(PB_ENCODER_B,  INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(PB_ON),         InterruptOnPBClick,        FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_OFF),        InterruptOffPBClick,       FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_LC_MODE_ON), InterruptLCModeOnPBClick,  FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_ENCODER_A),  InterruptEncoderA,         FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_ENCODER_B),  InterruptEncoderB,         FALLING);
}

void LoopSwitch(void)
{
  if (LightLcMode)
  {
    LightLcMode = false;
    DEBUG_INTERFACE.println("Light LC mode ON\n");
    Mesh_SendLightLightnessControllerModeSet(LightLcClientInstanceIdx, 
                                             LIGHT_LC_MODE_ON, 
                                             DEFAULT_NUMBER_OF_REPEATS);
  }

  if (On)
  {
    On = false;
    DEBUG_INTERFACE.println("Generic ON\n");
    Mesh_SendGenericOnOffSet(LightLcClientInstanceIdx, 
                             GENERIC_ON, 
                             ON_OFF_TRANSITION_TIME_MS,
                             DEFAULT_DELAY_TIME_MS);
  }

  if (Off)
  {
    Off = false;
    DEBUG_INTERFACE.println("Generic OFF\n");
    Mesh_SendGenericOnOffSet(LightLcClientInstanceIdx, 
                             GENERIC_OFF,
                             ON_OFF_TRANSITION_TIME_MS,
                             DEFAULT_DELAY_TIME_MS);
  }

  static unsigned long last_message_time = ULONG_MAX;
  
  if (Encoder != 0)
  {
    if (last_message_time + DELTA_INTVL_MS <= millis())
    {
      static uint32_t last_delta_message_time = UINT32_MAX;
      static int      delta                   = 0;

      bool is_new_tid = (last_delta_message_time + DELTA_NEW_TID_INTVL) < millis();
      if (is_new_tid)
          delta = 0;

      delta += Encoder;
      Mesh_SendGenericDeltaSet(LightLcClientInstanceIdx,
                               DELTA_STEP_VALUE * delta,
                               is_new_tid,
                               DEFAULT_TRANSITION_TIME_MS,
                               DEFAULT_DELAY_TIME_MS);

      if (is_new_tid)
        DEBUG_INTERFACE.printf("Delta Continue %d \n\n", delta);
      else
        DEBUG_INTERFACE.printf("Delta Start %d \n\n", delta);

      Encoder                 = 0;
      last_delta_message_time = millis();
      last_message_time       = millis();
    }
  }

  if (last_message_time + LIGHTNESS_INTVL_MS <= millis())
  {
    uint16_t lightness_actual = LightnessGet();
    if (HasLightnessChanged(lightness_actual))
    {
      PrintLightness(lightness_actual);
      Mesh_SendLightLightnessSet(LightLcClientInstanceIdx,
                                 lightness_actual,
                                 DEFAULT_TRANSITION_TIME_MS,
                                 DEFAULT_DELAY_TIME_MS);
      last_message_time = millis();
    }
  }
}
