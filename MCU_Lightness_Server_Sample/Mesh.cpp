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
#include "UART.h"
#include "Common.h"
#include "Arduino.h"

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Supported Mesh Opcodes definitions
 */
#define MESH_MESSAGE_LIGHT_LIGHTNESS_GET                     0x824B
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET                     0x824C
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED      0x824D
#define MESH_MESSAGE_LIGHT_LIGHTNESS_STATUS                  0x824E

/**
 * Used Mesh Messages len
 */
#define MESH_MESSAGE_LIGHT_LIGHTNESS_GET_LEN                 4

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

#define MAX_NUMBER_OF_REGISTERED_MODELS                     10

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static uint16_t RegisteredModelIds[MAX_NUMBER_OF_REGISTERED_MODELS];
static size_t   RegisteredModelIdsNum;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

static bool MeshInternal_ConvertFromMeshFormatToMsTransitionTime(uint8_t time_mesh_format, uint32_t * time_ms);
static void MeshInternal_ProcessLightLightnessStatus(uint8_t * payload, size_t len);
static uint8_t MeshInternal_GetInstanceIndexBasedOnModelId(uint16_t model_id);

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
  switch (mesh_cmd)
  {
    case MESH_MESSAGE_LIGHT_LIGHTNESS_STATUS:
    {
      MeshInternal_ProcessLightLightnessStatus(payload + index, len - index);
      break;
    }
  }
}

void Mesh_SendLightLightnessGet(void)
{
  uint8_t        buf[MESH_MESSAGE_LIGHT_LIGHTNESS_GET_LEN];
  size_t         index = 0;
  static uint8_t tid   = 0;

  buf[index++] = MeshInternal_GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_LIGHT_LIGHTNESS_SERVER);
  buf[index++] = 0x00;
  buf[index++] = lowByte(MESH_MESSAGE_LIGHT_LIGHTNESS_GET);
  buf[index++] = highByte(MESH_MESSAGE_LIGHT_LIGHTNESS_GET);

  UART_SendMeshMessageRequest(buf, sizeof(buf));
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

static void MeshInternal_ProcessLightLightnessStatus(uint8_t * payload, size_t len)
{
  size_t index = 0;
  uint16_t present_value;
  uint16_t target_value;
  uint32_t transition_time_ms;

  present_value  = ((uint16_t)payload[index++]);
  present_value |= ((uint16_t)payload[index++] << 8);


  if (index < len)
  {
    target_value  = ((uint16_t)payload[index++]);
    target_value |= ((uint16_t)payload[index++] << 8);

    bool is_valid = MeshInternal_ConvertFromMeshFormatToMsTransitionTime(payload[index++], &transition_time_ms);
    if (!is_valid)
    {
      DEBUG_INTERFACE.println("Rejected Transition Time");
      return;
    }
  }
  else
  {
    target_value       = present_value;
    transition_time_ms = 0;
  }

  ProcessTargetLightness(target_value, transition_time_ms);
}

static bool MeshInternal_ConvertFromMeshFormatToMsTransitionTime(uint8_t time_mesh_format, uint32_t * time_ms)
{
  uint32_t number_of_steps = (time_mesh_format & MESH_TRANSITION_TIME_NUMBER_OF_STEPS_MASK);
  uint32_t step_resolution = (time_mesh_format & MESH_TRANSITION_TIME_STEP_RESOLUTION_MASK);

  if (MESH_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE == number_of_steps)
  {
    *time_ms = 0;
    return false;
  }
  else
  {
    switch (step_resolution)
    {
    case MESH_TRANSITION_TIME_STEP_RESOLUTION_10_MIN:
      *time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_10MIN * number_of_steps;
      break;

    case MESH_TRANSITION_TIME_STEP_RESOLUTION_10_S:
      *time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_10S * number_of_steps;
      break;

    case MESH_TRANSITION_TIME_STEP_RESOLUTION_1_S:
      *time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_1S * number_of_steps;
      break;

    case MESH_TRANSITION_TIME_STEP_RESOLUTION_100_MS:
      *time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_100_MS * number_of_steps;
      break;

    default:
      *time_ms = (uint32_t)MESH_NUMBER_OF_MS_IN_100_MS * number_of_steps;
      break;
    }
  }
  return true;
}
