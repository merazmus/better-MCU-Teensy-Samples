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
#define MAX_PAYLOAD_SIZE 127

/********************************************
 * EXPORTED FUNCTIONS PROTOTYPES            *
 ********************************************/

/*
 *  Setup UART hardware
 */
void UART_Init(void);

/*
 *  Enables Ping Requests and Responses
 */
void UART_EnablePings(void);

/*
 *  Disables Ping Requests and Responses
 */
void UART_DisablePings(void);

/*
 *  Send Ping Request command
 */
void UART_SendPingRequest(void);

/*
 *  Send Pong Response command
 */
void UART_SendPongResponse(void);

/*
 *  Send Software Reset Request command
 */
void UART_SendSoftwareResetRequest(void);

/*
 *  Send Create Instances Request command
 *
 *  @param * model_id   Pointer to model ids list
 *  @param len          Model ids list length
 */
void UART_SendCreateInstancesRequest(uint8_t * model_id, uint8_t len);

/*
 *  Send Mesh Message Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void UART_SendMeshMessageRequest(uint8_t * p_payload, uint8_t len);

/*
 *  Send Sensor Update Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void UART_SendSensorUpdateRequest(uint8_t * p_payload, uint8_t len);

/*
 *  Send Start Node Request command
 */
void UART_StartNodeRequest(void);

/*
 *  Send Set Fault Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void UART_SendSetFaultRequest(uint8_t * p_payload, uint8_t len);

/*
 *  Send Clear Fault Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void UART_SendClearFaultRequest(uint8_t * p_payload, uint8_t len);

/*
 *  Send Test Start Response command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void UART_SendTestStartResponse(uint8_t * p_payload, uint8_t len);

/*
 *  Send Test Finished Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void UART_SendTestFinishedRequest(uint8_t * p_payload, uint8_t len);

/*
 *  Receive and process incoming UART command
 */
void UART_ProcessIncomingCommand(void);

/*
 *  Process Init Device Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessEnterInitDevice(uint8_t * p_payload, uint8_t len);

/*
 *  Process Create Instances Response command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessEnterDevice(uint8_t * p_payload, uint8_t len);

/*
 *  Process Init Node Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessEnterInitNode(uint8_t * p_payload, uint8_t len);
/*
 *  Process Start Node Response command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessEnterNode(uint8_t * p_payload, uint8_t len);

/*
 *  Process Mesh Message Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessMeshCommand(uint8_t * p_payload, uint8_t len);

/*
 *  Process Attention Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessAttention(uint8_t * p_payload, uint8_t len);

/*
 *  Process Error command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessError(uint8_t * p_payload, uint8_t len);

/*
 *  Process Start Test command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
extern void ProcessStartTest(uint8_t * p_payload, uint8_t len);

#endif  // UART_H_
