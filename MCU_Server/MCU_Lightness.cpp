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

/********************************************
* INCLUDES                                 *
********************************************/

#include "Config.h"
#include "Mesh.h"
#include "UART.h"
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Light Lightness Controller Server configuration
 */
#define DIMM_INTERRUPT_TIME_MS                   5            /**< Dimming control interrupt interval definition [ms]. */
#define DIMM_INTERRUPT_TIME_US                   (DIMM_INTERRUPT_TIME_MS * 1000)
#define CALC_SLOPE(current,target,steps)         (((target)-(current))/(steps))
#define CALC_NEW_LIGHTNESS(target, slope, steps) ((target) - ((steps) * (slope)))
#define CALC_PWM_OUTPUT(val)                     (((uint32_t)(val)*(val))>>24)

/********************************************
 * LOCAL TYPES DEFINITIONS                  *
 ********************************************/

struct DimLight
{
  uint16_t target;
  uint16_t current;
  uint16_t dimming_steps;
  int16_t  slope;
};

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/*
 *  Dimming interrupt handler.
 */
static void DimmInterrupt(void);

/*
 *  Sets PWM output to specific lightness
 *
 *  @param val     Lightness value
 */
static void PWMLightnessOutput(uint16_t val);

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static DimLight      Light;
static uint8_t       LightLightnessServerIdx = INSTANCE_INDEX_UNKNOWN;
static volatile bool AttentionLedState       = false;

/********************************************
 * LOCAL FUNCTIONS DEFINITIONS              *
 ********************************************/

static void DimmInterrupt(void)
{
  if (!AttentionLedState)
  {
    if (Light.dimming_steps)
    {
      Light.dimming_steps--;
    }

    // Default algorithm to calculate average slope of characteristic for Lightness
    Light.current = CALC_NEW_LIGHTNESS(Light.target, Light.slope, Light.dimming_steps);
    PWMLightnessOutput(Light.current);
  }
}

static void PWMLightnessOutput(uint16_t val)
{
  uint32_t pwm_out = CALC_PWM_OUTPUT(val);
  analogWrite(PIN_PWM, pwm_out);
}

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void SetLightnessServerIdx(uint8_t idx)
{
  LightLightnessServerIdx = idx;
}

uint8_t GetLightnessServerIdx(void)
{
  return LightLightnessServerIdx;
}

void IndicateAttentionLightness(bool attention_state, bool led_state)
{
  if (attention_state)
  {
    PWMLightnessOutput(0xFFFF * led_state);
  }
  AttentionLedState = attention_state;
}

void ProcessTargetLightness(uint16_t val, uint32_t transition_time)
{
  DEBUG_INTERFACE.printf("val: %d, transition_time %d\n", val, transition_time);
  noInterrupts();
  Light.target        = val;
  Light.dimming_steps = transition_time / DIMM_INTERRUPT_TIME_MS;
  if (Light.dimming_steps)
  {
    // Coefficient for lightness value
    Light.slope = CALC_SLOPE(Light.current, Light.target, Light.dimming_steps);
  }
  else
  {
    Light.current = Light.target;
  }
  interrupts();
}

void SetupLightnessServer(void)
{
  pinMode(PIN_PWM, OUTPUT);
  Timer1.initialize(DIMM_INTERRUPT_TIME_US);
  Timer1.attachInterrupt(DimmInterrupt);
}

void LoopLightnessServer(void)
{
}
