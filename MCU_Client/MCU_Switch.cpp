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
#include "UART.h"
#include "Encoder.h"
#include <limits.h>
#include <stdint.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Buttons placement definitions
 */
#define PB_ON_1                             PIN_SW_1       /**< Defines ON button location. */
#define PB_OFF_1                            PIN_SW_2       /**< Defines OFF button location. */
#define PB_ON_2                             PIN_SW_3       /**< Defines ON button location. */
#define PB_OFF_2                            PIN_SW_4       /**< Defines OFF button location. */
#define PB_ENCODER_A                        PIN_ENCODER_A  /**< Defines Encoder A pin location. */
#define PB_ENCODER_B                        PIN_ENCODER_B  /**< Defines Encoder B pin location. */

/**
 * Light Lightness Client configuration
 */
#define LIGHTNESS_MIN                       0u             /**< Defines lower range of light lightness. */
#define LIGHTNESS_MAX                       UINT16_MAX     /**< Defines upper range of light lightness. */
#define LIGHTNESS_MIN_CHANGE                0x500u         /**< Defines the lowest reported changed. */
#define LIGHTNESS_INTVL_MS                  50u            /**< Defines the shortest interval between two LightLightness messages. */

/**
 * On Off communication properties
 */
#define ON_OFF_TRANSITION_TIME_MS           0              /**< Defines transition time */
#define ON_OFF_NUMBER_OF_REPEATS            2              /**< Defines number of repeats while sending mesh message request */
#define ON_OFF_DELAY_TIME_MS                0              /**< Defines message delay in milliseconds */

#define GENERIC_OFF                         0x00           /**< Defines Generic OnOff Off payload */
#define GENERIC_ON                          0x01           /**< Defines Generic OnOff On payload */

/**
 * Generic Delta Client step
 */
#define DELTA_STEP_VALUE                    0x500          /**< Defines Generic Delta minimal step */
#define DELTA_INTVL_MS                      100            /**< Defines the shortest interval beetwen two Delta Set messages. */
#define DELTA_NEW_TID_INTVL                 350            /**< Defines the shortest interval beetwen generation of new TID. */
#define DELTA_TRANSITION_TIME_MS            100            /**< Defines transition time */
#define DELTA_DELAY_TIME_MS                 40             /**< Defines message delay in milliseconds */
#define DELTA_DELAY_TIME_MS_LAST            0              /**< Defines message delay in milliseconds */
#define DELTA_NUMBER_OF_REPEATS             0              /**< Defines number of repeats while sending mesh message request */
#define DELTA_NUMBER_OF_REPEATS_LAST        2              /**< Defines number of repeats while sending mesh message request */

/**
 * Default communication properties
 */
#define LIGHT_LIGHTNESS_NUMBER_OF_REPEATS   0              /**< Defines default number of repeats while sending mesh message request */
#define LIGHT_LIGHTNESS_TRANSITION_TIME_MS  100            /**< Defines default transition time */
#define LIGHT_LIGHTNESS_DELAY_TIME_MS       0              /**< Defines default delay time */

#define POT_DEADBAND                        20             /**< Potentiometer deadband. About 2% of full range */

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static volatile bool     On1                       = false;       /**< Implies if Generic ON button has been pushed */
static volatile bool     Off1                      = false;       /**< Implies if Generic OFF button has been pushed */
static volatile bool     On2                       = false;       /**< Implies if Generic ON button has been pushed */
static volatile bool     Off2                      = false;       /**< Implies if Generic OFF button has been pushed */
static volatile bool     Reinit                    = false;       /**< Implies if LCD reinit button has been pushed */
static          uint8_t  LightLcClient1InstanceIdx = INSTANCE_INDEX_UNKNOWN;
static          uint8_t  LightLcClient2InstanceIdx = INSTANCE_INDEX_UNKNOWN;
static          Encoder  DeltaEncoder(PB_ENCODER_B, PB_ENCODER_A);

/********************************************
 * LOCAL FUNCTION PROTOTYPES                *
 ********************************************/

/*
 *  Generic On 1 button interrupt handler
 */
static void InterruptOn1PBClick(void);

/*
 *  Generic Off 1 button interrupt handler
 */
static void InterruptOff1PBClick(void);

/*
 *  Generic On 2 button interrupt handler
 */
static void InterruptOn2PBClick(void);

/*
 *  Generic Off 2 button interrupt handler
 */
static void InterruptOff2PBClick(void);

/*
 *  LCD reinit button interrupt handler
 */
static void InterruptLcdReinit(void);

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

static void InterruptOn1PBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_ON_1)) return;

  On1 = true;
}

static void InterruptOff1PBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_OFF_1)) return;

  Off1 = true;
}

static void InterruptOn2PBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_ON_2)) return;

  On2 = true;
}

static void InterruptOff2PBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_OFF_2)) return;

  Off2 = true;
}

static void InterruptLcdReinit(void){
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PIN_ENCODER_SW)) return;

  Reinit = true;
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
  const uint32_t coefficient        = ((LIGHTNESS_MAX-LIGHTNESS_MIN) * UINT16_MAX) / (ANALOG_MAX-ANALOG_MIN);
  uint16_t       analog_measurement = analogRead(PIN_ANALOG);

  if (analog_measurement < POT_DEADBAND) analog_measurement = 0;

  return (coefficient * (analog_measurement - ANALOG_MIN)) / UINT16_MAX + LIGHTNESS_MIN;
}

static void PrintLightness(unsigned lightness_actual)
{
  unsigned percentage = (lightness_actual * 100.0) / LIGHTNESS_MAX;
  INFO("Current Lightness is %d (%d%)\n", lightness_actual, percentage);
}

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void SetInstanceIdxSwitch1(uint8_t idx)
{
  LightLcClient1InstanceIdx = idx;
}

uint8_t GetInstanceIdxSwitch1(void)
{
  return LightLcClient1InstanceIdx;
}

void SetInstanceIdxSwitch2(uint8_t idx)
{
  LightLcClient2InstanceIdx = idx;
}

uint8_t GetInstanceIdxSwitch2(void)
{
  return LightLcClient2InstanceIdx;
}

void SetupSwitch(void)
{
  INFO("Switch initialization.\n");
  pinMode(PB_ON_1,        INPUT_PULLUP);
  pinMode(PB_OFF_1,       INPUT_PULLUP);
  pinMode(PB_ON_2,        INPUT_PULLUP);
  pinMode(PB_OFF_2,       INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(PB_ON_1),        InterruptOn1PBClick,        FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_OFF_1),       InterruptOff1PBClick,       FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_ON_2),        InterruptOn2PBClick,        FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_OFF_2),       InterruptOff2PBClick,       FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), InterruptLcdReinit,         FALLING);
}

void LoopSwitch(void)
{
  if (On1)
  {
    On1 = false;
    INFO("Generic ON 1\n");
    Mesh_SendGenericOnOffSet(LightLcClient1InstanceIdx,
                             GENERIC_ON,
                             ON_OFF_TRANSITION_TIME_MS,
                             ON_OFF_DELAY_TIME_MS,
                             ON_OFF_NUMBER_OF_REPEATS,
                             true);
  }

  if (Off1)
  {
    Off1 = false;
    INFO("Generic OFF 1\n");
    Mesh_SendGenericOnOffSet(LightLcClient1InstanceIdx,
                             GENERIC_OFF,
                             ON_OFF_TRANSITION_TIME_MS,
                             ON_OFF_DELAY_TIME_MS,
                             ON_OFF_NUMBER_OF_REPEATS,
                             true);
  }

  if (On2)
  {
    On2 = false;
    INFO("Generic ON 2\n");
    Mesh_SendGenericOnOffSet(LightLcClient2InstanceIdx,
                             GENERIC_ON,
                             ON_OFF_TRANSITION_TIME_MS,
                             ON_OFF_DELAY_TIME_MS,
                             ON_OFF_NUMBER_OF_REPEATS,
                             true);
  }

  if (Off2)
  {
    Off2 = false;
    INFO("Generic OFF 2\n");
    Mesh_SendGenericOnOffSet(LightLcClient2InstanceIdx,
                             GENERIC_OFF,
                             ON_OFF_TRANSITION_TIME_MS,
                             ON_OFF_DELAY_TIME_MS,
                             ON_OFF_NUMBER_OF_REPEATS,
                             true);
  }

  if (Reinit)
  {
    Reinit = false;
    INFO("LCD Reinit\n");

    LCD_Reinit();
  }

  static unsigned long last_message_time = ULONG_MAX;

  long encoder_pos;
  encoder_pos = DeltaEncoder.read();
  
  if (encoder_pos != 0)
  {
    if (last_message_time + DELTA_INTVL_MS <= millis())
    {
      static uint32_t last_delta_message_time = UINT32_MAX;
      static int      delta                   = 0;

      bool is_new_tid = (last_delta_message_time + DELTA_NEW_TID_INTVL) < millis();
      if (is_new_tid)
          delta = 0;

      delta += encoder_pos;
      Mesh_SendGenericDeltaSet(LightLcClient1InstanceIdx,
                               DELTA_STEP_VALUE * delta,
                               DELTA_TRANSITION_TIME_MS,
                               DELTA_DELAY_TIME_MS,
                               DELTA_NUMBER_OF_REPEATS,
                               is_new_tid);

      if (is_new_tid)
        INFO("Delta Continue %d \n\n", delta);
      else
        INFO("Delta Start %d \n\n", delta);

      DeltaEncoder.write(0);
      last_delta_message_time = millis();
      last_message_time       = millis();
    }
  }

  if (last_message_time + LIGHTNESS_INTVL_MS <= millis())
  {
    uint16_t lightness_actual = LightnessGet();
    if (HasLightnessChanged(lightness_actual))
    {
      INFO("lightness changed");
      PrintLightness(lightness_actual);
      Mesh_SendLightLightnessSet(LightLcClient1InstanceIdx,
                                 lightness_actual,
                                 LIGHT_LIGHTNESS_TRANSITION_TIME_MS,
                                 LIGHT_LIGHTNESS_DELAY_TIME_MS,
                                 LIGHT_LIGHTNESS_NUMBER_OF_REPEATS,
                                 true);

      last_message_time = millis();
    }
  }
}
