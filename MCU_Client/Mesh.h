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

#include "stddef.h"
#include "stdint.h"

/********************************************
 * EXPORTED #define CONSTANTS AND MACROS    *
 ********************************************/

/**
 * Supported Mesh Model IDs definitions
 */
#define MESH_MODEL_ID_GENERIC_ONOFF_CLIENT      0x1001
#define MESH_MODEL_ID_SENSOR_CLIENT             0x1102
#define MESH_MODEL_ID_GENERIC_LEVEL_CLIENT      0x1003
#define MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT    0x1302
#define MESH_MODEL_ID_LIGHT_LC_CLIENT           0x1311

/********************************************
 * EXPORTED FUNCTIONS PROTOTYPES            *
 ********************************************/

/*
 *  Search for model ID in a message
 *
 *  @param p_payload            Pointer to p_payload
 *  @param len                  Payload len
 *  @param expected_model_id    Expected model ID
 *  @return                     True if found, false otherwise
 */
bool Mesh_IsModelAvailable(uint8_t * p_payload, uint8_t len, uint16_t expected_model_id);

/*
 *  Process Mesh Message Request command
 *
 *  @param p_payload    Pointer to p_payload
 *  @param len          Payload len
 */
void Mesh_ProcessMeshCommand(uint8_t * p_payload, size_t len);

/*
 *  Send Generic OnOff Set message with repeats
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 */
void Mesh_SendGenericOnOffSet(uint8_t  instance_idx,
                              bool     value,
                              unsigned transition_time,
                              unsigned delay_ms);

/*
 *  Send Light Lightness Set message with repeats
 *
 *  @param instance_idx      Instance index
 *  @param value             Light Lightness value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
void Mesh_SendLightLightnessSet(uint8_t  instance_idx,
                                uint16_t value,
                                unsigned transition_time,
                                unsigned delay_ms);

/*
 *  Send Generic Delta Set message with repeats
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
void Mesh_SendGenericDeltaSet(uint8_t  instance_idx,
                              int32_t  value,
                              bool     is_new,
                              unsigned transition_time,
                              unsigned delay_ms);

/*
 *  Send Light Lightness Controller Mode Set message with repeats
 *
 *  @param instance_idx      Instance index
 *  @param value             Generic OnOff value
 *  @param transition_time   Transition time (mesh format)
 *  @param delay_ms          Delay in miliseconds
 *  @param is_new            Is it a new transation?
 */
void Mesh_SendLightLightnessControllerModeSet(uint8_t instance_idx, bool value, unsigned repeats);

/*
 *  Process ALS update
 *
 *  @param value       New ALS value
 *  @param src_addr    Source address
 */
extern void ProcessPresentAmbientLightLevel(uint16_t src_addr, float value);

/*
 *  Process PIR update
 *
 *  @param value       New PIR value
 *  @param src_addr    Source address
 */
extern void ProcessPresenceDetected(uint16_t src_addr, bool value);

#endif  // MESH_H_
