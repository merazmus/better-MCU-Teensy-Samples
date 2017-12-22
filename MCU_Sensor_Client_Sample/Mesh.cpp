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

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**
 * Supported Mesh Opcodes definitions
 */
#define MESH_MESSAGE_SENSOR_STATUS        0x0052

/**
 * Sensor status description
 */
#define SS_FORMAT_MASK                    0x01
#define SS_SHORT_LEN_MASK                 0x1E
#define SS_SHORT_LEN_OFFSET               1
#define SS_SHORT_PROPERTY_ID_LOW_MASK     0xE0
#define SS_SHORT_PROPERTY_ID_LOW_OFFSET   5
#define SS_SHORT_PROPERTY_ID_HIGH_OFFSET  3
#define SS_LONG_LEN_MASK                  0xFE
#define SS_LONG_LEN_OFFSET                1

/**
 * Property IDs description
 */
#define PRESENCE_DETECTED_PROPERTY_ID     0x004D
#define PRESENT_AMBIENT_LIGHT_LEVEL       0x004E

#define MAX_NUMBER_OF_REGISTERED_MODELS   10

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static uint16_t RegisteredModelIds[MAX_NUMBER_OF_REGISTERED_MODELS];
static size_t   RegisteredModelIdsNum;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

static void MeshInternal_ProcessSensorStatus(uint8_t * payload, size_t len);
static void MeshInternal_ProcessSensorProperty(uint16_t property_id, uint8_t * payload, size_t len, uint16_t src_addr);
static void MeshInternal_ProcessPresenceDetected(uint8_t * payload, size_t len, uint16_t src_addr);
static void MeshInternal_ProcessPresentAmbientLightLevel(uint8_t * payload, size_t len, uint16_t src_addr);

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
    case MESH_MESSAGE_SENSOR_STATUS:
    {
      MeshInternal_ProcessSensorStatus(payload + index, len - index);
      break;
    }
  }
}

/********************************************
 * LOCAL FUNCTION DEFINITIONS               *
 *******************************************/

static void MeshInternal_ProcessSensorStatus(uint8_t * payload, size_t len)
{
  if (len < 2) 
  {
    DEBUG_INTERFACE.println("Received empty Sensor Status message");
    return;
  }

  uint16_t src_addr = ((uint16_t)payload[len-2]);
  src_addr         |= ((uint16_t)payload[len-1] << 8);

  if (len <= 2)
  {
    DEBUG_INTERFACE.printf("Received empty Sensor Status message from: %d\n", src_addr);
    return;
  }

  len -= 2;
  size_t index = 0;
  while(index < len)
  {
    DEBUG_INTERFACE.printf("ProcessSensorStatus index: %d\n", index);
    if (payload[index] & SS_FORMAT_MASK)
    {
      size_t   message_len = (payload[index++] & SS_LONG_LEN_MASK) >> SS_LONG_LEN_OFFSET;
      uint16_t property_id = ((uint16_t)payload[index++]);
      property_id         |= ((uint16_t)payload[index++] << 8);

      MeshInternal_ProcessSensorProperty(property_id, payload + index, message_len, src_addr);
      index += message_len;
    }
    else
    {
      size_t   message_len = (payload[index] & SS_SHORT_LEN_MASK) >> SS_SHORT_LEN_OFFSET;
      uint16_t property_id = (payload[index++] & SS_SHORT_PROPERTY_ID_LOW_MASK) >> SS_SHORT_PROPERTY_ID_LOW_OFFSET;
      property_id         |= ((uint16_t) payload[index++]) << SS_SHORT_PROPERTY_ID_HIGH_OFFSET;

      MeshInternal_ProcessSensorProperty(property_id, payload + index, message_len, src_addr);
      index += message_len;
    }
  }
}

static void MeshInternal_ProcessSensorProperty(uint16_t property_id, uint8_t * payload, size_t len, uint16_t src_addr)
{
  switch(property_id)
  {
    case PRESENCE_DETECTED_PROPERTY_ID:
    {
      MeshInternal_ProcessPresenceDetected(payload, len, src_addr);
      break;
    }
    case PRESENT_AMBIENT_LIGHT_LEVEL:
    {
      MeshInternal_ProcessPresentAmbientLightLevel(payload, len, src_addr);
      break;
    }
    default:
    {
      DEBUG_INTERFACE.println("Invalid property id");
    }
  }
}

static void MeshInternal_ProcessPresenceDetected(uint8_t * payload, size_t len, uint16_t src_addr)
{
  if (len != 1) 
  {
    DEBUG_INTERFACE.println("Invalid Length Sensor Status message");
    return;
  }

  size_t  index = 0;
  uint8_t value = payload[index++];

  if (value <= 1)
  {
    ProcessPresenceDetected(src_addr, (value == 1));
  }
  else
  {
    DEBUG_INTERFACE.printf("Decoded Sensor Status message from 0x%04X, PRESENCE DETECTED with prohibited value\n", src_addr);
  }
}

static void MeshInternal_ProcessPresentAmbientLightLevel(uint8_t * payload, size_t len, uint16_t src_addr)
{
  if (len != 3) 
  {
    DEBUG_INTERFACE.println("Invalid Length Sensor Status message");
    return;
  }

  size_t   index = 0;
  uint32_t value_clux = ((uint32_t)payload[index++]);
  value_clux         |= ((uint32_t)payload[index++] << 8);
  value_clux         |= ((uint32_t)payload[index++] << 16);

  float value = ((float)value_clux) / 100;
  ProcessPresentAmbientLightLevel(src_addr, value);
}
