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

#include "Mesh.h"
#include "Common.h"
#include "Arduino.h"
#include "UART.h"

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Supported Mesh Opcodes definitions
 */
#define MESH_MESSAGE_GENERIC_ONOFF_GET                      0x8201
#define MESH_MESSAGE_GENERIC_ONOFF_SET                      0x8202
#define MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED       0x8203
#define MESH_MESSAGE_GENERIC_ONOFF_STATUS                   0x8204

#define MESH_MESSAGE_GENERIC_LEVEL_GET                      0x8205
#define MESH_MESSAGE_GENERIC_LEVEL_SET                      0x8206
#define MESH_MESSAGE_GENERIC_LEVEL_SET_UNACKNOWLEDGED       0x8207
#define MESH_MESSAGE_GENERIC_LEVEL_STATUS                   0x8208
#define MESH_MESSAGE_GENERIC_DELTA_SET                      0x8209
#define MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED       0x820A

#define MESH_MESSAGE_LIGHT_LIGHTNESS_GET                    0x824B
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET                    0x824C
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED     0x824D
#define MESH_MESSAGE_LIGHT_LIGHTNESS_STATUS                 0x824E

#define MESH_MESSAGE_LIGHT_LC_MODE_GET                      0x8291
#define MESH_MESSAGE_LIGHT_LC_MODE_SET                      0x8292
#define MESH_MESSAGE_LIGHT_LC_MODE_SET_UNACKNOWLEDGED       0x8293
#define MESH_MESSAGE_LIGHT_LC_MODE_STATUS                   0x8294

/**
 * Default communication properties
 */
#define MESH_REPEATS_INTERVAL_MS                            50

/**
 * Used Mesh Messages len
 */
#define MESH_MESSAGE_GENERIC_ONOFF_SET_LEN                  8
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET_LEN                9
#define MESH_MESSAGE_GENERIC_DELTA_SET_LEN                  11
#define MESH_MESSAGE_LIGHT_LC_MODE_SET_LEN                  5

/*
 * Mesh time conversion definitions
 */
#define MESH_NUMBER_OF_MS_IN_100_MS                         (100UL)
#define MESH_NUMBER_OF_MS_IN_1S                             (10*MESH_NUMBER_OF_MS_IN_100_MS)
#define MESH_NUMBER_OF_MS_IN_10S                            (10*MESH_NUMBER_OF_MS_IN_1S)
#define MESH_NUMBER_OF_MS_IN_10MIN                          (60*MESH_NUMBER_OF_MS_IN_10S)

#define MESH_TRANSITION_TIME_STEP_RESOLUTION_MASK           0xC0
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS         0x00
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S            0x40
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S           0x80
#define MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN         0xC0

#define MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK           0x3F
#define MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE  0x3F

#define MESH_DELAY_TIME_STEP_MS                             5

#define MAX_NUMBER_OF_REGISTERED_MODELS                     10

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static uint16_t RegisteredModelIds[MAX_NUMBER_OF_REGISTERED_MODELS];
static size_t   RegisteredModelIdsNum;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

static uint8_t MeshInternal_GetInstanceIndexBasedOnModelId(uint16_t model_id);
static void MeshInternal_SendGenericOnOffSet(bool value, uint8_t transition_time, uint8_t delay_ms, bool is_new);
static void MeshInternal_SendLightLightnessSet(uint16_t value, uint8_t transition_time, uint8_t delay_ms, bool is_new);
static void MeshInternal_SendGenericDeltaSet(int32_t value, uint8_t transition_time, uint8_t delay_ms, bool is_new);
static void MeshInternal_SendLightLightnessControllerModeSet(bool value);
static uint8_t MeshInternal_ConvertFromMsToMeshFormatTransitionTime(uint32_t time_ms);

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

bool Mesh_IsModelAvailable(uint8_t * payload, uint8_t len, uint16_t expected_model_id)
{
  for (size_t index = 0; index < len;)
  {
    uint16_t model_id = ((uint16_t)payload[index++]);
    model_id         |= ((uint16_t)payload[index++] << 8);

    if (expected_model_id == model_id)
    {
      return true;
    }
  }
  return false;
}

void Mesh_AddRegisteredModelId(uint16_t model_id)
{
  if (RegisteredModelIdsNum >= MAX_NUMBER_OF_REGISTERED_MODELS)
  {
    DEBUG_INTERFACE.println("The maximum number of models has been exceeded!");
    return;
  }
  RegisteredModelIds[RegisteredModelIdsNum++] = model_id;
}

void Mesh_ResetRegisteredModelId(void)
{
  RegisteredModelIdsNum = 0;
}

void Mesh_ProcessMeshCommand(uint8_t * payload, size_t len)
{
  size_t   index             = 0;
  uint8_t  instance_index    = payload[index++];
  uint8_t  instance_subindex = payload[index++];
  uint16_t mesh_cmd          = ((uint16_t)payload[index++]);
  mesh_cmd                  |= ((uint16_t)payload[index++] << 8);

  DEBUG_INTERFACE.printf("Process Mesh Command [%d %d 0x%02X]\n",
                         instance_index,
                         instance_subindex,
                         mesh_cmd);
}

void Mesh_SendGenericOnOffSet(bool value, unsigned transition_time, unsigned delay_ms)
{
  uint8_t transition_mesh_format = 
    MeshInternal_ConvertFromMsToMeshFormatTransitionTime(transition_time);
  MeshInternal_SendGenericOnOffSet(value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, true);
  while(delay_ms >= MESH_REPEATS_INTERVAL_MS)
  {
    delay_ms -= MESH_REPEATS_INTERVAL_MS;
    MeshInternal_SendGenericOnOffSet(value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, false);
  }
}

void Mesh_SendLightLightnessSet(uint16_t value, unsigned transition_time, unsigned delay_ms)
{
  uint8_t transition_mesh_format = 
    MeshInternal_ConvertFromMsToMeshFormatTransitionTime(transition_time);
  MeshInternal_SendLightLightnessSet(value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, true);
  while(delay_ms >= MESH_REPEATS_INTERVAL_MS)
  {
    delay_ms -= MESH_REPEATS_INTERVAL_MS;
    MeshInternal_SendLightLightnessSet(value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, false);
  }
}

void Mesh_SendGenericDeltaSet(int32_t value, bool is_new, unsigned transition_time, unsigned delay_ms)
{
  uint8_t transition_mesh_format = 
    MeshInternal_ConvertFromMsToMeshFormatTransitionTime(transition_time);
  MeshInternal_SendGenericDeltaSet(value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, is_new);
  while(delay_ms >= MESH_REPEATS_INTERVAL_MS)
  {
    delay_ms -= MESH_REPEATS_INTERVAL_MS;
    MeshInternal_SendGenericDeltaSet(value, transition_mesh_format, delay_ms/MESH_DELAY_TIME_STEP_MS, false);
  }
}

void Mesh_SendLightLightnessControllerModeSet(bool value, unsigned repeats)
{
  MeshInternal_SendLightLightnessControllerModeSet(value);
  for(size_t i = 0; i < repeats; i++)
  {
    MeshInternal_SendLightLightnessControllerModeSet(value);
  }
}

/********************************************
 * LOCAL FUNCTION DEFINITIONS               *
 *******************************************/

static uint8_t MeshInternal_GetInstanceIndexBasedOnModelId(uint16_t model_id)
{
  for (size_t index = 0; index < RegisteredModelIdsNum; index++)
  {
    if (RegisteredModelIds[index] == model_id)
    {
      return index + 1;
    }
  }
  return 0;
}

static void MeshInternal_SendGenericOnOffSet(bool value, uint8_t transition_time, uint8_t delay_ms, bool is_new)
{
  uint8_t        buf[MESH_MESSAGE_GENERIC_ONOFF_SET_LEN];
  size_t         index = 0;
  static uint8_t tid   = 0;

  buf[index++] = MeshInternal_GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT);
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
  buf[index++] = (value) ? 0x01 : 0x00;
  buf[index++] = is_new ? ++tid : tid;
  buf[index++] = transition_time;
  buf[index++] = delay_ms;

  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendLightLightnessSet(uint16_t value, uint8_t transition_time, uint8_t delay_ms, bool is_new)
{
  uint8_t        buf[MESH_MESSAGE_LIGHT_LIGHTNESS_SET_LEN];
  size_t         index = 0;
  static uint8_t tid   = 0;

  buf[index++] = MeshInternal_GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT);
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED);
  buf[index++] = lowByte(value);
  buf[index++] = highByte(value);
  buf[index++] = is_new ? ++tid : tid;
  buf[index++] = transition_time;
  buf[index++] = delay_ms;

  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendGenericDeltaSet(int32_t value, uint8_t transition_time, uint8_t delay_ms, bool is_new)
{
  uint8_t         buf[MESH_MESSAGE_GENERIC_DELTA_SET_LEN];
  size_t          index = 0;
  static uint8_t  tid   = 0;

  buf[index++] = MeshInternal_GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT);
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
  buf[index++] = ((value >> 0) & 0xFF);
  buf[index++] = ((value >> 8) & 0xFF);
  buf[index++] = ((value >> 16) & 0xFF);
  buf[index++] = ((value >> 24) & 0xFF);
  buf[index++] = is_new ? ++tid : tid;
  buf[index++] = transition_time;
  buf[index++] = delay_ms;
  
  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static void MeshInternal_SendLightLightnessControllerModeSet(bool value)
{
  uint8_t buf[MESH_MESSAGE_LIGHT_LC_MODE_SET_LEN];
  size_t  index = 0;

  buf[index++] = MeshInternal_GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_LIGHT_LC_CLIENT);
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_LIGHT_LC_MODE_SET_UNACKNOWLEDGED);
  buf[index++] = highByte(MESH_MESSAGE_LIGHT_LC_MODE_SET_UNACKNOWLEDGED);
  buf[index++] = (value) ? 0x01 : 0x00;

  UART_SendMeshMessageRequest(buf, sizeof(buf));
}

static uint8_t MeshInternal_ConvertFromMsToMeshFormatTransitionTime(uint32_t time_ms)
{
  if((time_ms/MESH_NUMBER_OF_MS_IN_100_MS) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS | (time_ms/MESH_NUMBER_OF_MS_IN_100_MS));
  }
  else if((time_ms/MESH_NUMBER_OF_MS_IN_1S) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S | (time_ms/MESH_NUMBER_OF_MS_IN_1S));
  }
  else if((time_ms/MESH_NUMBER_OF_MS_IN_10S) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S | (time_ms/MESH_NUMBER_OF_MS_IN_10S));
  }
  else if((time_ms/MESH_NUMBER_OF_MS_IN_10MIN) < MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN | (time_ms/MESH_NUMBER_OF_MS_IN_10MIN));
  }
  else
  {
    return (MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN | 
      (MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE & MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
  }
}
