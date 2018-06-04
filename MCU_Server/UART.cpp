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
#include "Arduino.h"
#include "Config.h"

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/**< UART Command Codes definitions */
#define UART_CMD_PING_REQUEST               0x01u
#define UART_CMD_PONG_RESPONSE              0x02u
#define UART_CMD_INIT_DEVICE_EVENT          0x03u
#define UART_CMD_CREATE_INSTANCES_REQUEST   0x04u
#define UART_CMD_CREATE_INSTANCES_RESPONSE  0x05u
#define UART_CMD_INIT_NODE_EVENT            0x06u
#define UART_CMD_MESH_MESSAGE_REQUEST       0x07u
#define UART_CMD_START_NODE_REQUEST         0x09u
#define UART_CMD_START_NODE_RESPONSE        0x0Bu
#define UART_CMD_FACTORY_RESET_REQUEST      0x0Cu
#define UART_CMD_FACTORY_RESET_RESPONSE     0x0Du
#define UART_CMD_FACTORY_RESET_EVENT        0x0Eu
#define UART_CMD_MESH_MESSAGE_RESPONSE      0x0Fu
#define UART_CMD_CURRENT_STATE_REQUEST      0x10u
#define UART_CMD_CURRENT_STATE_RESPONSE     0x11u
#define UART_CMD_ERROR                      0x12u
#define UART_CMD_FIRMWARE_VERSION_REQUEST   0x13u
#define UART_CMD_FIRMWARE_VERSION_RESPONSE  0x14u
#define UART_CMD_SENSOR_UPDATE_REQUEST      0x15u
#define UART_CMD_ATTENTION_EVENT            0x16u
#define UART_CMD_SOFTWARE_RESET_REQUEST     0x17u
#define UART_CMD_SOFTWARE_RESET_RESPONSE    0x18u
#define UART_CMD_SENSOR_UPDATE_RESPONSE     0x19u
#define UART_CMD_DEVICE_UUID_REQUEST        0x1Au
#define UART_CMD_DEVICE_UUID_RESPONSE       0x1Bu
#define UART_CMD_SET_FAULT_REQUEST          0x1Cu
#define UART_CMD_SET_FAULT_RESPONSE         0x1Du
#define UART_CMD_CLEAR_FAULT_REQUEST        0x1Eu
#define UART_CMD_CLEAR_FAULT_RESPONSE       0x1Fu
#define UART_CMD_START_TEST_REQ             0x20u
#define UART_CMD_START_TEST_RESP            0x21u
#define UART_CMD_TEST_FINISHED_REQ          0x22u
#define UART_CMD_TEST_FINISHED_RESP         0x23u


/**< CRC configuration */
#define CRC_POLYNOMIAL                      0x8005u
#define CRC_INIT_VAL                        0xFFFFu

/**< Preamble definition */
#define PREAMBLE_BYTE_1                     0xAAu
#define PREAMBLE_BYTE_2                     0x55u

/**< UART Message description */
#define PREAMBLE_BYTE_1_OFFSET              0u
#define PREAMBLE_BYTE_2_OFFSET              1u
#define LEN_OFFSET                          2u
#define CMD_OFFSET                          3u
#define PAYLOAD_OFFSET                      4u
#define CRC_BYTE_1_OFFSET(len)              (PAYLOAD_OFFSET+(len))
#define CRC_BYTE_2_OFFSET(len)              (PAYLOAD_OFFSET+(len)+1)

/** @brief Counts number of elements inside the array. */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/********************************************
 * EXPORTED TYPES DEFINITIONS               *
 ********************************************/

struct RxFrame_t
{
  uint8_t len;
  uint8_t cmd;
  uint8_t p_payload[MAX_PAYLOAD_SIZE];
};

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static bool UART_PingsEnabled = true; /**< If true, device will send and respond to pings. Default it should work */

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/*
 *  Received data from UART
 *
 *  @param rx_frame    Pointer to frame to be filled with received data
 *  @return            True if frame's CRC is valid, false otherwise
 */
static bool UARTInternal_Receive(RxFrame_t * rx_frame);

/*
 *  Send message over UART
 *
 *  @param len        Message length
 *  @param cmd        Message command
 *  @param p_payload  Message payload
 */
static void UARTInternal_Send(uint8_t len, uint8_t cmd, uint8_t * p_payload);

/*
 *  Print debug message
 *
 *  @param *dir    Direction description
 *  @param len     Command length
 *  @param cmd     Command code
 *  @param *buf    Pointer to message
 *  @param crc     CRC
 */
static void PrintDebug(const char * dir, uint8_t len, uint8_t cmd, uint8_t * buf, uint16_t crc);

/*
 *  Calc CRC16
 *
 *  @param len    Data length
 *  @param cmd    Command code
 *  @param *data  Pointer to data buffer
 */
static uint16_t CalcCRC16(uint8_t len, uint8_t cmd, uint8_t * data);

/*
 *  Add byte to calculated CRC
 *
 *  @param data   New byte
 *  @param crc    Data length
 */
static uint16_t __calcCRC(uint8_t data, uint16_t crc);

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void UART_Init(void)
{
  UART_INTERFACE.begin(UART_INTERFACE_BAUDRATE);
  while(!UART_INTERFACE);
}

void UART_EnablePings(void)
{
  DEBUG_INTERFACE.println("Pings enabled \n");
  UART_PingsEnabled = true;
}

void UART_DisablePings(void)
{
  DEBUG_INTERFACE.println("Pings disabled \n");
  UART_PingsEnabled = false;
}

void UART_SendPingRequest(void)
{
  if(UART_PingsEnabled)
  {
    UARTInternal_Send(0, UART_CMD_PING_REQUEST, NULL);
  }
}

void UART_SendPongResponse(void)
{
  if(UART_PingsEnabled)
  {
    UARTInternal_Send(0, UART_CMD_PONG_RESPONSE, NULL);
  }
}

void UART_SendSoftwareResetRequest(void)
{
  UARTInternal_Send(0, UART_CMD_SOFTWARE_RESET_REQUEST, NULL);
}

void UART_SendCreateInstancesRequest(uint8_t * model_id, uint8_t len)
{
  UARTInternal_Send(len, UART_CMD_CREATE_INSTANCES_REQUEST, model_id);
}

void UART_SendMeshMessageRequest(uint8_t * p_payload, uint8_t len)
{
  UARTInternal_Send(len, UART_CMD_MESH_MESSAGE_REQUEST, p_payload);
}

void UART_SendSensorUpdateRequest(uint8_t * p_payload, uint8_t len)
{
  UARTInternal_Send(len, UART_CMD_SENSOR_UPDATE_REQUEST, p_payload);
}

void UART_StartNodeRequest(void)
{
  UARTInternal_Send(0, UART_CMD_START_NODE_REQUEST, NULL);
}

void UART_SendSetFaultRequest(uint8_t * p_payload, uint8_t len)
{
  UARTInternal_Send(len, UART_CMD_SET_FAULT_REQUEST, p_payload);
}

void UART_SendClearFaultRequest(uint8_t * p_payload, uint8_t len)
{
  UARTInternal_Send(len, UART_CMD_CLEAR_FAULT_REQUEST, p_payload);
}

void UART_SendTestStartResponse(uint8_t * p_payload, uint8_t len)
{
  UARTInternal_Send(len, UART_CMD_START_TEST_RESP, p_payload);
}

void UART_SendTestFinishedRequest(uint8_t * p_payload, uint8_t len)
{
  UARTInternal_Send(len, UART_CMD_TEST_FINISHED_REQ, p_payload);
}

void UART_ProcessIncomingCommand(void)
{
  static RxFrame_t rx_frame;

  if (!UARTInternal_Receive(&rx_frame)) return;

  switch (rx_frame.cmd)
  {
    case UART_CMD_PING_REQUEST:
    {
      UART_SendPongResponse();
      break;
    }
    case UART_CMD_INIT_DEVICE_EVENT:
    {
      ProcessEnterInitDevice(rx_frame.p_payload, rx_frame.len);
      break;
    }
    case UART_CMD_CREATE_INSTANCES_RESPONSE:
    {
      ProcessEnterDevice(rx_frame.p_payload, rx_frame.len);
      break;
    }
    case UART_CMD_INIT_NODE_EVENT:
    {
      ProcessEnterInitNode(rx_frame.p_payload, rx_frame.len);
      break;
    }
    case UART_CMD_START_NODE_RESPONSE:
    {
      ProcessEnterNode(rx_frame.p_payload, rx_frame.len);
      break;
    }
    case UART_CMD_MESH_MESSAGE_REQUEST:
    {
      ProcessMeshCommand(rx_frame.p_payload, rx_frame.len);
      break;
    }
    case UART_CMD_ATTENTION_EVENT:
    {
      ProcessAttention(rx_frame.p_payload, rx_frame.len);
      break;
    }
    case UART_CMD_ERROR:
    {
      ProcessError(rx_frame.p_payload, rx_frame.len);
      break;
    }
    case UART_CMD_START_TEST_REQ:
    {
      ProcessStartTest(rx_frame.p_payload, rx_frame.len);
      break;
    }
  }
}

/********************************************
 * LOCAL FUNCTION DEFINITIONS               *
 *******************************************/

static bool UARTInternal_Receive(RxFrame_t * rx_frame)
{
  bool            isCRCValid = false;
  static uint16_t crc        = 0;
  static size_t   count      = 0;

  while (UART_INTERFACE.available())
  {
    uint8_t received_byte = UART_INTERFACE.read();

    if (count == PREAMBLE_BYTE_1_OFFSET)
    {
      if (received_byte == PREAMBLE_BYTE_1)
      {
        count++;
      }
      else
      {
        count = 0;
      }
    }
    else if (count == PREAMBLE_BYTE_2_OFFSET)
    {
      if (received_byte == PREAMBLE_BYTE_2)
      {
        count++;
      }
      else
      {
        count = 0;
      }
    }
    else if (count == LEN_OFFSET)
    {
      rx_frame->len = received_byte;
      count++;
    }
    else if (count == CMD_OFFSET)
    {
      rx_frame->cmd = received_byte;
      count++;
    }
    else if ((CMD_OFFSET < count) && (count < CRC_BYTE_1_OFFSET(rx_frame->len)))
    {
      rx_frame->p_payload[count - PAYLOAD_OFFSET] = received_byte;
      count++;
    }
    else if (count == CRC_BYTE_1_OFFSET(rx_frame->len))
    {
      crc = received_byte;
      count++;
    }
    else if (count == CRC_BYTE_2_OFFSET(rx_frame->len))
    {
      crc       += ((uint16_t) received_byte) << 8;
      isCRCValid = (crc == CalcCRC16(rx_frame->len, rx_frame->cmd, rx_frame->p_payload));
      count      = 0;

      if (isCRCValid)
        break;
    }
  }

  if (isCRCValid)
  {
    PrintDebug("Received", rx_frame->len, rx_frame->cmd, rx_frame->p_payload, crc);
  }

  return isCRCValid;
}

static void UARTInternal_Send(uint8_t len, uint8_t cmd, uint8_t * p_payload)
{
  uint16_t crc;

  UART_INTERFACE.write(PREAMBLE_BYTE_1);
  UART_INTERFACE.write(PREAMBLE_BYTE_2);
  UART_INTERFACE.write(len);
  UART_INTERFACE.write(cmd);

  for (int i = 0; i < len; i++)
  {
    UART_INTERFACE.write(p_payload[i]);
  }

  crc = CalcCRC16(len, cmd, p_payload);
  UART_INTERFACE.write(lowByte(crc));
  UART_INTERFACE.write(highByte(crc));

  PrintDebug("Sent", len, cmd, p_payload, crc);
}

static void PrintDebug(const char * dir, uint8_t len, uint8_t cmd, uint8_t * buf, uint16_t crc)
{
  const char * cmdName[] = {
    "Unknown",
    "PingRequest",
    "PongResponse",
    "InitDeviceEvent",
    "CreateInstancesRequest",
    "CreateInstancesResponse",
    "InitNodeEvent",
    "MeshMessageRequest",
    "Unknown",
    "StartNodeRequest",
    "Unknown",
    "StartNodeResponse",
    "FactoryResetRequest",
    "FactoryResetResponse",
    "FactoryResetEvent",
    "MeshMessageResponse",
    "CurrentStateRequest",
    "CurrentStateResponse",
    "Error",
    "FirmwareVersionRequest",
    "FirmwareVersionResponse",
    "SensorUpdateRequest",
    "AttentionEvent",
    "SoftwareResetRequest",
    "SoftwareResetResponse",
    "SensorUpdateResponse",
    "DeviceUUIDRequest",
    "DeviceUUIDResponse",
    "SetFaultRequest",
    "SetFaultResponse",
    "ClearFaultRequest",
    "ClearFaultResponse"
  };

  DEBUG_INTERFACE.printf("%s %s command\n", dir, (cmd < ARRAY_SIZE(cmdName)) ? cmdName[cmd] : "Unknown");
  DEBUG_INTERFACE.printf("\t Len: 0x%02X\n", len);
  DEBUG_INTERFACE.printf("\t Cmd: 0x%02X\n", cmd);
  DEBUG_INTERFACE.printf("\t Data: ");
  for (size_t i = 0; i < len; i++)
  {
    DEBUG_INTERFACE.printf("0x%02X ", buf[i]);
  }
  DEBUG_INTERFACE.println();
  DEBUG_INTERFACE.printf("\t CRC: 0x%02X%02X\n\n", lowByte(crc), highByte(crc));
}

static uint16_t CalcCRC16(uint8_t len, uint8_t cmd, uint8_t * data)
{
  uint16_t crc = CRC_INIT_VAL;
  crc          = __calcCRC(len, crc);
  crc          = __calcCRC(cmd, crc);

  for (size_t i = 0; i < len; i++)
  {
    crc = __calcCRC(data[i], crc);
  }

  return crc;
}

static uint16_t __calcCRC(uint8_t data, uint16_t crc)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if (((crc & 0x8000) >> 8) ^ (data & 0x80))
    {
      crc = (crc << 1) ^ CRC_POLYNOMIAL;
    }
    else
    {
      crc = (crc << 1);
    }
    data <<= 1;
  }

  return crc;
}