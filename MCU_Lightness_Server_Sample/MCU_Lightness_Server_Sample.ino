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

#include "UART.h"
#include "Mesh.h"
#include "Common.h"
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Light Lightness Client configuration
 */
#define LIGHTNESS_MIN               0            /**< Defines lower range of light lightness */
#define LIGHTNESS_MAX               UINT16_MAX   /**< Defines upper range of light lightness */
#define DIMM_INTERRUPT_TIME_MS      5            /**< Dimming control interrupt interval definition [ms]. */
#define DIMM_INTERRUPT_TIME_US      (DIMM_INTERRUPT_TIME_MS * 1000)
#define CALC_SLOPE(current,target,steps)              (((target)-(current))/(steps))
#define CALC_NEW_LIGHTNESS(target, slope, steps)      ((target) - ((steps) * (slope)))
#define CALC_PWM_OUTPUT(val)                          (((uint32_t)(val)*(val))>>24)

/*************************************************************************
 *
 * Structures
 *
 *************************************************************************/

struct DimLight
{
  uint16_t target;
  uint16_t current;
  uint16_t dimming_steps;
  int16_t  slope;
};

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static DimLight      Light;
static bool          AttentionState;
static bool          AttentionLedValue;
static ModemState_t  ModemState  = MODEM_STATE_UNKNOWN;

void SetupDebug(void)
{
  DEBUG_INTERFACE.begin(DEBUG_INTERFACE_BAUDRATE);
  unsigned long timestamp = millis();
  while(!DEBUG_INTERFACE && (timestamp + 1000 > millis()));
}

void SetupAttention(void)
{
  pinMode(PIN_LED, OUTPUT);

  Timer3.initialize(ATTENTION_TIME_US);
  Timer3.attachInterrupt(IndicateAttention);
}

void SetupHW(void)
{
  pinMode(PIN_PWM, OUTPUT);
  Timer1.initialize(DIMM_INTERRUPT_TIME_US);
  Timer1.attachInterrupt(DimmInterrupt);
}

void DimmInterrupt(void)
{
  if (!AttentionState)
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

void PWMLightnessOutput(uint16_t val)
{
  uint32_t pwm_out = CALC_PWM_OUTPUT(val);
  analogWrite(PIN_PWM, pwm_out);
}

void ProcessEnterInitDevice(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Device State.");
  ModemState     = MODEM_STATE_INIT_DEVICE;
  AttentionState = false;

  Mesh_ResetRegisteredModelId();

  if (!Mesh_IsModelAvailable(payload, len, MESH_MODEL_ID_LIGHT_LIGHTNESS_SERVER))
  {
    DEBUG_INTERFACE.println("Modem does not support Light Lightness Server.\n");
    return;
  }

  uint8_t model_ids[] = {
    lowByte (MESH_MODEL_ID_LIGHT_LIGHTNESS_SERVER),
    highByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_SERVER),
  };
  UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
}

void ProcessEnterDevice(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Device State.\n");
  ModemState = MODEM_STATE_DEVICE;
}

void ProcessEnterInitNode(uint8_t * payload, uint8_t len)
{
  Mesh_ResetRegisteredModelId();
  DEBUG_INTERFACE.println("Init Node State.\n");

  ModemState     = MODEM_STATE_INIT_NODE;
  AttentionState = false;

  for (size_t index = 0; index < len;)
  {
    uint16_t model_id = ((uint16_t)payload[index++]);
    model_id         |= ((uint16_t)payload[index++] << 8);
    Mesh_AddRegisteredModelId(model_id);
  }
  Mesh_SendLightLightnessGet();
  UART_StartNodeRequest();
}

void ProcessEnterNode(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Node State.\n");
  ModemState = MODEM_STATE_NODE;
}

void ProcessMeshCommand(uint8_t * payload, uint8_t len)
{
  Mesh_ProcessMeshCommand(payload, len);
}

void ProcessAttention(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Attention State %d\n\n.", payload[0]);
  AttentionState = (payload[0] == 0x01);
  if (!AttentionState)
  {
    AttentionLedValue = false;
    digitalWrite(PIN_LED, AttentionLedValue);
  }
}

void ProcessError(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Error %d\n\n.", payload[0]);
}

void ProcessTargetLightness(uint16_t val, uint32_t transition_time)
{
  DEBUG_INTERFACE.printf("val: %d, transition_time %d\n",val,transition_time);
  noInterrupts();
  Light.target = val;
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

void IndicateAttention(void)
{
  AttentionLedValue = AttentionState ? !AttentionLedValue : false;
  digitalWrite(PIN_LED, AttentionLedValue);
  if (AttentionState)
  {
    PWMLightnessOutput(0xFFFF*AttentionLedValue);
  }
}

void setup()
{
  SetupDebug();
  DEBUG_INTERFACE.println("Lightness Server Sample.\n");
  SetupHW();
  SetupAttention();
  UART_Init();
  UART_SendSoftwareResetRequest();
}

void loop()
{
  UART_ProcessIncomingCommand();
}
