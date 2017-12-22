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

#ifndef UART_H_
#define UART_H_

/********************************************
 * INCLUDES                                 *
 ********************************************/

#include "stdint.h"

/********************************************
 * EXPORTED #define CONSTANTS AND MACROS    *
 ********************************************/

/**< Defines maximum data length in frame */
#define MAX_PAYLOAD_SIZE      127

/********************************************
 * EXPORTED FUNCTIONS PROTOTYPES            *
 ********************************************/

void UART_Init(void);
void UART_SendPingRequest(void);
void UART_SendPongResponse(void);
void UART_SendSoftwareResetRequest(void);
void UART_SendCreateInstancesRequest(uint8_t * model_id, uint8_t len);
void UART_SendMeshMessageRequest(uint8_t * payload, uint8_t len);
void UART_SendSensorUpdateRequest(uint8_t * payload, uint8_t len);
void UART_StartNodeRequest(void);
void UART_ProcessIncomingCommand(void);

extern void ProcessEnterInitDevice(uint8_t * payload, uint8_t len);
extern void ProcessEnterDevice(uint8_t * payload, uint8_t len);
extern void ProcessEnterInitNode(uint8_t * payload, uint8_t len);
extern void ProcessEnterNode(uint8_t * payload, uint8_t len);
extern void ProcessMeshCommand(uint8_t * payload, uint8_t len);
extern void ProcessAttention(uint8_t * payload, uint8_t len);
extern void ProcessError(uint8_t * payload, uint8_t len);

#endif