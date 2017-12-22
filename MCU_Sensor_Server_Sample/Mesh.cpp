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

/********************************************
 * LOCAL FUNCTION DEFINITIONS               *
 *******************************************/
