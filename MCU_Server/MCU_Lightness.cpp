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

#include "MCU_Lightness.h"
#include "Config.h"
#include "Mesh.h"
#include "UART.h"
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define PWM_OUTPUT_MAX UINT16_MAX

#if ENABLE_1_10_V
#define PWM_OUTPUT_MIN (uint16_t)(0.12 * PWM_OUTPUT_MAX)
#else
#define PWM_OUTPUT_MIN 0u
#endif
/**
 * Light Lightness Controller Server configuration
 */
#define DIMM_INTERRUPT_TIME_MS 5u            /**< Dimming control interrupt interval definition [ms]. */
#define DIMM_INTERRUPT_TIME_US (DIMM_INTERRUPT_TIME_MS * 1000)
#define POW(a)                 ((a) * (a))

#define ATTENTION_LIGHTNESS_ON  0xFFFF
#define ATTENTION_LIGHTNESS_OFF (0xFFFF * 4 / 10)

#define DEVICE_STARTUP_SEQ_STAGE_DURATION_MS    300
#define DEVICE_STARTUP_SEQ_STAGE_1_LIGHTNESS    0xFFFF
#define DEVICE_STARTUP_SEQ_STAGE_2_LIGHTNESS    0x0000
#define DEVICE_STARTUP_SEQ_STAGE_3_LIGHTNESS    0xFFFF
#define DEVICE_STARTUP_SEQ_STAGE_4_LIGHTNESS    0x0000
#define DEVICE_STARTUP_SEQ_STAGE_5_LIGHTNESS    0x7FFF
#define DEVICE_STARTUP_SEQ_STAGE_6_LIGHTNESS    0x0000
#define DEVICE_STARTUP_SEQ_STAGE_OFF_LIGHTNESS  0xFFFF

/********************************************
 * LOCAL TYPES DEFINITIONS                  *
 ********************************************/

struct Transition
{
  uint16_t target_value;
  uint16_t start_value;
  uint32_t start_timestamp;
  uint32_t transition_time;
};

typedef enum
{
  DEVICE_SEQUENCE_STAGE_1,
  DEVICE_SEQUENCE_STAGE_2,
  DEVICE_SEQUENCE_STAGE_3,
  DEVICE_SEQUENCE_STAGE_4,
  DEVICE_SEQUENCE_STAGE_5,
  DEVICE_SEQUENCE_STAGE_6,
  DEVICE_SEQUENCE_STAGE_OFF,
} DeviceStartupSequence_T;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/*
 *  Convert Lightness Actual to Lightness Linear (based on Spec Model, chapter 6.1.2.2.1)
 *
 *  @param val     Lightness Actual value
 */
static inline uint32_t ConvertLightnessActualToLinear(uint16_t val);

/*
 *  Dimming interrupt handler.
 */
static void DimmInterrupt(void);

/*
 *  Calculate current transition value
 *
 *  @param p_transition     Pointer to transition
 */
static uint16_t GetCurrentValue(Transition * p_transition);

/*
 *  Calculate slope ans sets PWM output to specific lightness
 *
 *  @param val     Lightness value
 */
static void SetLightnessOutput(uint16_t val);

/*
 *  Calculate new transition
 *
 *  @param current          Current value
 *  @param target           Target value
 *  @param transition_time  Transition time
 *  @param p_transition     Pointer to transition
 */
static void UpdateTransition(uint16_t current, uint16_t target, uint32_t transition_time, Transition * p_transition);

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static Transition Light = {
  .target_value    = 0,
  .start_value     = 0,
  .start_timestamp = 0,
  .transition_time = 0,
};

static Transition Temperature = {
  .target_value    = 0,
  .start_value     = UINT16_MAX/2,
  .start_timestamp = 0,
  .transition_time = 0,
};

static bool          IsEnabled                       = false;
static bool          CTLSupport                      = false;
static uint8_t       LightLightnessServerIdx         = INSTANCE_INDEX_UNKNOWN;
static volatile bool AttentionLedState               = false;
static bool          UnprovisionedSequenceEnableFlag = false;

/********************************************
 * LOCAL FUNCTIONS DEFINITIONS              *
 ********************************************/

static inline uint32_t ConvertLightnessActualToLinear(uint16_t val)
{
  return (LIGHTNESS_MAX * POW((val * UINT8_MAX)/LIGHTNESS_MAX)) / UINT16_MAX; 
}

static void DimmInterrupt(void)
{
  if (!AttentionLedState)
  {
    SetLightnessOutput(GetCurrentValue(&Light));
  }
}

static uint16_t GetCurrentValue(Transition * p_transition)
{
  uint32_t time       = millis();
  uint32_t delta_time = time - p_transition->start_timestamp;

  if (delta_time > p_transition->transition_time)
  {
    p_transition->start_value = p_transition->target_value;
    return p_transition->start_value;
  }

  int32_t  delta_transition = ((int64_t)p_transition->target_value - p_transition->start_value) * delta_time / p_transition->transition_time;

  return p_transition->start_value + delta_transition;
}

static void SetLightnessOutput(uint16_t val)
{
  const uint32_t coefficient = ((uint32_t) (PWM_OUTPUT_MAX-PWM_OUTPUT_MIN) * UINT16_MAX) / (LIGHTNESS_MAX-LIGHTNESS_MIN);
  uint32_t pwm_out;
  if (val == 0)
  {
    pwm_out = 0; 
  }
  else
  {
    // calc value to PWM OUTPUT
    pwm_out = ConvertLightnessActualToLinear(val);
    // Calculate value depend of 0-10 V 
    pwm_out = ((coefficient * (pwm_out - LIGHTNESS_MIN)) / UINT16_MAX) + PWM_OUTPUT_MIN;
  }

  if (CTLSupport)
  {
    uint64_t warm;
    uint64_t cold;

    uint16_t temperature = GetCurrentValue(&Temperature);

    cold  = temperature * pwm_out;
    cold /= UINT16_MAX;

    warm  = (UINT16_MAX - temperature) * pwm_out;
    warm /= UINT16_MAX;

    analogWrite(PIN_PWM_WARM, warm);
    analogWrite(PIN_PWM_COLD, cold);
  }
  else
  {
    analogWrite(PIN_PWM_COLD, pwm_out);
    analogWrite(PIN_PWM_WARM, 0);
  }
}

static void UpdateTransition(uint16_t current, uint16_t target, uint32_t transition_time, Transition * p_transition)
{
  noInterrupts();
  p_transition->start_value     = current;
  p_transition->target_value    = target;
  p_transition->transition_time = transition_time;
  p_transition->start_timestamp = millis();
  interrupts();
}

static void PerformStartupSequenceIfNeeded(void)
{

  static DeviceStartupSequence_T current_startup_sequence_stage =   DEVICE_SEQUENCE_STAGE_OFF;
  static long                    sequence_start                 =   UINT32_MAX;
  uint16_t                       startup_sequence_lightness[]   = { DEVICE_STARTUP_SEQ_STAGE_1_LIGHTNESS,
                                                                    DEVICE_STARTUP_SEQ_STAGE_2_LIGHTNESS,
                                                                    DEVICE_STARTUP_SEQ_STAGE_3_LIGHTNESS,
                                                                    DEVICE_STARTUP_SEQ_STAGE_4_LIGHTNESS,
                                                                    DEVICE_STARTUP_SEQ_STAGE_5_LIGHTNESS,
                                                                    DEVICE_STARTUP_SEQ_STAGE_6_LIGHTNESS,
                                                                    DEVICE_STARTUP_SEQ_STAGE_OFF_LIGHTNESS };
  if(UnprovisionedSequenceEnableFlag)
  {
    sequence_start                  = millis();
    UnprovisionedSequenceEnableFlag = false;
    current_startup_sequence_stage     = DEVICE_SEQUENCE_STAGE_1;

    ProcessTargetLightness(0, startup_sequence_lightness[current_startup_sequence_stage], 0);
  }

  unsigned long           sequence_duration = millis() - sequence_start;
  DeviceStartupSequence_T calculated_stage  =
    (DeviceStartupSequence_T)(sequence_duration / DEVICE_STARTUP_SEQ_STAGE_DURATION_MS);

  if (current_startup_sequence_stage != DEVICE_SEQUENCE_STAGE_OFF
   && current_startup_sequence_stage != calculated_stage)
  {
    current_startup_sequence_stage = calculated_stage;
    ProcessTargetLightness(0, startup_sequence_lightness[current_startup_sequence_stage], 0);
  }
}

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void SetLightnessServerIdx(uint8_t idx)
{
  if (!IsEnabled) return;
  LightLightnessServerIdx = idx;
}

void SetLightCTLSupport(bool support)
{
  if (!IsEnabled) return;
  CTLSupport = support;
}

uint8_t GetLightnessServerIdx(void)
{
  return LightLightnessServerIdx;
}

void IndicateAttentionLightness(bool attention_state, bool led_state)
{
  if (!IsEnabled) return;

  if (attention_state)
  {
    uint16_t led_lightness = led_state ? ATTENTION_LIGHTNESS_ON : ATTENTION_LIGHTNESS_OFF;
    SetLightnessOutput(led_lightness);
  }
  AttentionLedState = attention_state;
}

void ProcessTargetLightness(uint16_t current, uint16_t target, uint32_t transition_time)
{
  if (!IsEnabled) return;

  INFO("Lightness: %d -> %d, transition_time %d\n", current, target, transition_time);

  UpdateTransition(current, target, transition_time, &Light);
}

void ProcessTargetLightnessTemp(uint16_t current, uint16_t target, uint32_t transition_time)
{
  if (!IsEnabled) return;

  INFO("Temperature: %d-> %d, transition_time %d\n", current, target, transition_time);

  UpdateTransition(current, target, transition_time, &Temperature);
}

void SetupLightnessServer(void)
{
  IsEnabled = true;
  pinMode(PIN_PWM_WARM, OUTPUT);
  pinMode(PIN_PWM_COLD, OUTPUT);
  analogWriteResolution(PWM_RESOLUTION);
  Timer1.initialize(DIMM_INTERRUPT_TIME_US);
  Timer1.attachInterrupt(DimmInterrupt);
}

void LoopLightnessServer(void)
{
  if (!IsEnabled) return;

  PerformStartupSequenceIfNeeded();
}

void EnableStartupSequence(void)
{
  if (!IsEnabled) return;

  UnprovisionedSequenceEnableFlag = true;
}

void SynchronizeLightness(void)
{
  if (!IsEnabled) return;

  Mesh_SendLightLightnessGet(GetLightnessServerIdx());
}
