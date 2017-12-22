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

#ifndef MESH_H_
#define MESH_H_

/********************************************
 * INCLUDES                                 *
 ********************************************/

#include "stdint.h"
#include "stddef.h"

/********************************************
 * EXPORTED #define CONSTANTS AND MACROS    *
 ********************************************/

/**
 * Supported Mesh Model IDs definitions
 */
#define MESH_MODEL_ID_GENERIC_ONOFF_CLIENT                   0x1001
#define MESH_MODEL_ID_GENERIC_LEVEL_CLIENT                   0x1003
#define MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT                 0x1302
#define MESH_MODEL_ID_LIGHT_LC_CLIENT                        0x1311

/********************************************
 * EXPORTED FUNCTIONS PROTOTYPES            *
 ********************************************/

bool Mesh_IsModelAvailable(uint8_t * payload, uint8_t len, uint16_t expected_model_id);
void Mesh_AddRegisteredModelId(uint16_t model_id);
void Mesh_ResetRegisteredModelId(void);
void Mesh_ProcessMeshCommand(uint8_t * payload, size_t len);
void Mesh_SendGenericOnOffSet(bool value, unsigned transition_time, unsigned delay_ms);
void Mesh_SendLightLightnessSet(uint16_t value, unsigned transition_time, unsigned delay_ms);
void Mesh_SendGenericDeltaSet(int32_t value, bool is_new, unsigned transition_time, unsigned delay_ms);
void Mesh_SendLightLightnessControllerModeSet(bool value, unsigned repeats);

#endif // MESH_H_