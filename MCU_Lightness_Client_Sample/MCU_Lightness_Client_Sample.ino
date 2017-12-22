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
#include <TimerThree.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Buttons placement definitions
 */
#define PB_ON                       PIN_PB_0     /**< Defines ON button location. */
#define PB_OFF                      PIN_PB_1     /**< Defines OFF button location. */
#define PB_LC_MODE_ON               PIN_PB_4     /**< Defines LC MODE button location. */
#define PB_ENCODER_A                PIN_PB_5     /**< Defines Encoder A pin location. */
#define PB_ENCODER_B                PIN_PB_6     /**< Defines Encoder B pin location. */

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

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static volatile bool LightLcMode       = false;               /**< Implies if LightLC Mode button has been pushed */
static volatile bool On                = false;               /**< Implies if Generic ON button has been pushed */
static volatile bool Off               = false;               /**< Implies if Generic OFF button has been pushed */
static volatile int  Encoder           = 0;                   /**< Implies if Encoder position has changed */
static bool          AttentionState    = false; 
static bool          AttentionLedValue = false;
static ModemState_t  ModemState        = MODEM_STATE_UNKNOWN;

/********************************************
 * FUNCTION DEFINITIONS                     *
 ********************************************/

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

void InterruptLCModeOnPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if(digitalRead(PB_LC_MODE_ON)) return;

  LightLcMode = true;
}

void InterruptOnPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if(digitalRead(PB_ON)) return;

  On = true;
}

void InterruptOffPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_OFF)) return; 

  Off = true;
}

void InterruptEncoderA(void)
{
  if(digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  delayMicroseconds(ENCODER_DEBOUNCE_TIME_US);
  if(digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  Encoder--;

  while(!(digitalRead(PB_ENCODER_A) && digitalRead(PB_ENCODER_B))) { }
}

void InterruptEncoderB(void)
{
  if(digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  delayMicroseconds(ENCODER_DEBOUNCE_TIME_US);
  if(digitalRead(PB_ENCODER_A) || digitalRead(PB_ENCODER_B)) return;
  Encoder++;

  while(!(digitalRead(PB_ENCODER_A) && digitalRead(PB_ENCODER_B))) { }
}

void ProcessEnterInitDevice(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Device State.\n");
  ModemState     = MODEM_STATE_INIT_DEVICE;
  AttentionState = false;

  Mesh_ResetRegisteredModelId();

  if (!Mesh_IsModelAvailable(payload, len, MESH_MODEL_ID_GENERIC_ONOFF_CLIENT))
  {
    DEBUG_INTERFACE.println("Modem does not support Generic On Off Client.\n");
    return;
  }

  if (!Mesh_IsModelAvailable(payload, len, MESH_MODEL_ID_GENERIC_LEVEL_CLIENT))
  {
    DEBUG_INTERFACE.println("Modem does not support Generic Level Client.\n");
    return;
  }

  if (!Mesh_IsModelAvailable(payload, len, MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT))
  {
    DEBUG_INTERFACE.println("Modem does not support Light Lightness Client.\n");
    return;
  }

  if (!Mesh_IsModelAvailable(payload, len, MESH_MODEL_ID_LIGHT_LC_CLIENT))
  {
    DEBUG_INTERFACE.println("Modem does not support Light Lightness Controler Client.\n");
    return;
  }

  uint8_t model_ids[] = {
    lowByte (MESH_MODEL_ID_GENERIC_ONOFF_CLIENT),
    highByte(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT),
    lowByte (MESH_MODEL_ID_GENERIC_LEVEL_CLIENT),
    highByte(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT),
    lowByte (MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT),
    highByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT),
    lowByte (MESH_MODEL_ID_LIGHT_LC_CLIENT),
    highByte(MESH_MODEL_ID_LIGHT_LC_CLIENT)
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
}

void ProcessError(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Error %d\n\n.", payload[0]);
}

bool HasLightnessChanged(int lightness_actual)
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

uint16_t LightnessGet(void)
{
  const float coefficient        = (float)(LIGHTNESS_MAX-LIGHTNESS_MIN) / (float)(ANALOG_MAX-ANALOG_MIN);
  uint16_t    analog_measurement = analogRead(PIN_ANALOG);

  return coefficient * (analog_measurement - ANALOG_MIN) + LIGHTNESS_MIN;
}

void PrintLightness(unsigned lightness_actual)
{
  unsigned percentage = (lightness_actual * 100.0) / LIGHTNESS_MAX;
  DEBUG_INTERFACE.printf("Current Lightness is %d (%d%)\n", lightness_actual, percentage);
}

void IndicateAttention(void)
{
  AttentionLedValue = AttentionState ? !AttentionLedValue : false;
  digitalWrite(PIN_LED, AttentionLedValue);
}

/********************************************
 * MAIN FUNCTION DEFINITIONS                *
 ********************************************/

void setup()
{
  SetupDebug();
  DEBUG_INTERFACE.println("Lightness Client Sample.\n");
  SetupHW();
  SetupAttention();
  UART_Init();
  UART_SendSoftwareResetRequest();
}

void loop()
{
  UART_ProcessIncomingCommand();

  if (MODEM_STATE_NODE != ModemState) return;

  if (LightLcMode)
  {
    LightLcMode = false;
    DEBUG_INTERFACE.println("Light LC mode ON\n");
    Mesh_SendLightLightnessControllerModeSet(0x01, DEFAULT_NUMBER_OF_REPEATS);
  }

  if (On)
  {
    On = false;
    DEBUG_INTERFACE.println("Generic ON\n");
    Mesh_SendGenericOnOffSet(0x01, 
                             ON_OFF_TRANSITION_TIME_MS,
                             DEFAULT_DELAY_TIME_MS);
  }

  if (Off)
  {
    Off = false;
    DEBUG_INTERFACE.println("Generic OFF\n");
    Mesh_SendGenericOnOffSet(0x00,
                             ON_OFF_TRANSITION_TIME_MS,
                             DEFAULT_DELAY_TIME_MS);
  }

  static uint32_t last_message_time = UINT32_MAX;

  if (Encoder != 0 && (last_message_time + DELTA_INTVL_MS <= millis()))
  {
    static uint32_t last_delta_message_time = UINT32_MAX;
    static int      delta             = 0;
    if (last_delta_message_time + DELTA_NEW_TID_INTVL > millis())
    {
      delta += Encoder;
      Mesh_SendGenericDeltaSet(DELTA_STEP_VALUE * delta, 
                               false, 
                               DEFAULT_TRANSITION_TIME_MS,
                               DEFAULT_DELAY_TIME_MS);

      DEBUG_INTERFACE.printf("Delta Continue %d \n\n", delta);
    }
    else
    {
      delta = Encoder;
      Mesh_SendGenericDeltaSet(DELTA_STEP_VALUE * delta, 
                               true,
                               DEFAULT_TRANSITION_TIME_MS,
                               DEFAULT_DELAY_TIME_MS);

      DEBUG_INTERFACE.printf("Delta Start %d \n\n", delta);
    }

    Encoder           = 0;
    last_delta_message_time = millis();
    last_message_time = millis();
  }

  uint16_t lightness_actual = LightnessGet();
  if ((last_message_time + LIGHTNESS_INTVL_MS <= millis()))
  {
    if (HasLightnessChanged(lightness_actual))
    {
      PrintLightness(lightness_actual);
      Mesh_SendLightLightnessSet(lightness_actual,
                                 DEFAULT_TRANSITION_TIME_MS,
                                 DEFAULT_DELAY_TIME_MS);
      last_message_time = millis();
    }
  }
}
