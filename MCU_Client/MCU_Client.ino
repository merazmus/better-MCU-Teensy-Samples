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

#include "Config.h"
#include "LCD.h"
#include "MCU_Sensor.h"
#include "MCU_Switch.h"
#include "Mesh.h"
#include "UART.h"
#include <TimerThree.h>
#include <limits.h>

/********************************************
 * EXPORTED #define CONSTANTS AND MACROS    *
 ********************************************/

#define FIRMWARE_VERSION_EXPECTED_LEN     6
#define FIRMWARE_VERSION_VID_OFFSET_LOW   4
#define FIRMWARE_VERSION_VID_OFFSET_HIGH  5
#define FIRMWARE_VERSION_VID_UNKNOWN      0xFFFF

/********************************************
 * EXPORTED TYPES DEFINITIONS               *
 ********************************************/

enum ModemState_t
{
  MODEM_STATE_INIT_DEVICE,
  MODEM_STATE_DEVICE,
  MODEM_STATE_INIT_NODE,
  MODEM_STATE_NODE,
  MODEM_STATE_UNKNOWN = 0xFF,
};

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static bool         AttentionState    = false;
static bool         AttentionLedValue = false;
static ModemState_t ModemState        = MODEM_STATE_UNKNOWN;
static uint16_t     McuVid            = FIRMWARE_VERSION_VID_UNKNOWN;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/*
 *  Setup debug interface
 */
void SetupDebug(void);

/*
 *  Setup attention hardware
 */
void SetupAttention(void);

/*
 *  Process Init Device Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterInitDevice(uint8_t * p_payload, uint8_t len);

/*
 *  Process Create Instances Response command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterDevice(uint8_t * p_payload, uint8_t len);

/*
 *  Process Init Node Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterInitNode(uint8_t * p_payload, uint8_t len);

/*
 *  Process Start Node Response command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessEnterNode(uint8_t * p_payload, uint8_t len);

/*
 *  Process Mesh Message Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessMeshCommand(uint8_t * p_payload, uint8_t len);

/*
 *  Process Attention Event command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessAttention(uint8_t * p_payload, uint8_t len);

/*
 *  Process Error command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessError(uint8_t * p_payload, uint8_t len);

/*
 *  Indicate attention
 */
void IndicateAttention(void);

/*
 *  Print current UART Modem state on LCD
 */
void PrintStateOnLCD(void);

/*
 *  Main Arduino setup
 */
void setup();

/*
 *  Main Arduino loop
 */
void loop();

/********************************************
 * COMMON FUNCTION DEFINITIONS              *
 ********************************************/

void SetupDebug(void)
{
  DEBUG_INTERFACE.begin(DEBUG_INTERFACE_BAUDRATE);
  // Waits for debug interface initialization.
  delay(1000);
}

void SetupAttention(void)
{
  pinMode(PIN_LED_STATUS, OUTPUT);

  Timer3.initialize(ATTENTION_TIME_US);
  Timer3.attachInterrupt(IndicateAttention);
}

void ProcessEnterInitDevice(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Device State.\n");
  ModemState     = MODEM_STATE_INIT_DEVICE;
  AttentionState = false;

  SetInstanceIdxSwitch1(INSTANCE_INDEX_UNKNOWN);
  SetInstanceIdxSwitch2(INSTANCE_INDEX_UNKNOWN);
  SetInstanceIdxSensor(INSTANCE_INDEX_UNKNOWN);

  if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_LC_CLIENT))
  {
    DEBUG_INTERFACE.println("Modem does not support Light Lightness Controler Client.\n");
    return;
  }

  if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_SENSOR_CLIENT))
  {
    DEBUG_INTERFACE.println("Modem does not support Sensor Client.\n");
    return;
  }

  uint8_t model_ids[] = {
    // First Light Lightness controller client
    lowByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),
    highByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),
    // Second Light Lightness controller client
    lowByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),
    highByte(MESH_MODEL_ID_LIGHT_LC_CLIENT),
    // Sensor client
    lowByte(MESH_MODEL_ID_SENSOR_CLIENT),
    highByte(MESH_MODEL_ID_SENSOR_CLIENT),
  };
  
  UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
  UART_FirmwareVersionRequest();
}

void ProcessEnterDevice(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Device State.\n");
  ModemState = MODEM_STATE_DEVICE;
}

void ProcessEnterInitNode(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Node State.\n");
  ModemState     = MODEM_STATE_INIT_NODE;
  AttentionState = false;

  SetInstanceIdxSwitch1(INSTANCE_INDEX_UNKNOWN);
  SetInstanceIdxSwitch2(INSTANCE_INDEX_UNKNOWN);
  SetInstanceIdxSensor(INSTANCE_INDEX_UNKNOWN);

  for (size_t index = 0; index < len;)
  {
    uint16_t model_id = ((uint16_t)p_payload[index++]);
    model_id         |= ((uint16_t)p_payload[index++] << 8);
    uint16_t current_model_id_instance_index = index/2;

    if (MESH_MODEL_ID_LIGHT_LC_CLIENT == model_id)
    {
      if (GetInstanceIdxSwitch1() == INSTANCE_INDEX_UNKNOWN)
      {
        SetInstanceIdxSwitch1(current_model_id_instance_index);
      }
      else
      {
        SetInstanceIdxSwitch2(current_model_id_instance_index);
      }
    }

    if (MESH_MODEL_ID_SENSOR_CLIENT == model_id)
    {
      SetInstanceIdxSensor(current_model_id_instance_index);
    }
  }

  if (GetInstanceIdxSwitch1() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    DEBUG_INTERFACE.println("First Light Lightness Controller Client model id not found in init node message");
    return;
  }

  if (GetInstanceIdxSwitch2() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    DEBUG_INTERFACE.println("Second Light Lightness Controller Client model id not found in init node message");
    return;
  }

  if (GetInstanceIdxSensor() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    DEBUG_INTERFACE.println("Sensor Client model id not found in init node message");
    return;
  }

  UART_StartNodeRequest();
  UART_FirmwareVersionRequest();
}

void ProcessEnterNode(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Node State.\n");
  ModemState = MODEM_STATE_NODE;
}

void ProcessMeshCommand(uint8_t * p_payload, uint8_t len)
{
  Mesh_ProcessMeshCommand(p_payload, len);
}

void ProcessAttention(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Attention State %d\n\n.", p_payload[0]);
  AttentionState = (p_payload[0] == 0x01);
}

void ProcessError(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Error %d\n\n.", p_payload[0]);
}

void ProcessFirmwareVersion(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Process Firmware Version");
  if (len == FIRMWARE_VERSION_EXPECTED_LEN)
  {
    McuVid  = p_payload[FIRMWARE_VERSION_VID_OFFSET_LOW];
    McuVid |= (uint16_t) p_payload[FIRMWARE_VERSION_VID_OFFSET_HIGH] << 8;
  }
}

void IndicateAttention(void)
{
  AttentionLedValue = AttentionState ? !AttentionLedValue : false;
  digitalWrite(PIN_LED_STATUS, AttentionLedValue);
}

void PrintVersionOnLCD(void)
{
  if(McuVid != FIRMWARE_VERSION_VID_UNKNOWN)
  {
    char text[LCD_COLUMNS];
    sprintf(text, "Build #%d", McuVid);
    WriteLineLCD(LCD_VERSION_LINE, text);
  }
}

void PrintStateOnLCD(void)
{
  switch (ModemState)
  {
    case MODEM_STATE_INIT_DEVICE:
    {
      ClearLCD();
      PrintVersionOnLCD();
      WriteLineLCD(LCD_DEVICE_STATE_LINE, "Init device state");
      break;
    }
    case MODEM_STATE_DEVICE:
    {
      ClearLCD();
      PrintVersionOnLCD();
      WriteLineLCD(LCD_DEVICE_STATE_LINE, "Device state");
      break;
    }
    case MODEM_STATE_INIT_NODE:
    {
      ClearLCD();
      PrintVersionOnLCD();
      WriteLineLCD(LCD_DEVICE_STATE_LINE, "Init node state");
      break;
    }
    case MODEM_STATE_NODE:
    {
      WriteLineLCD(LCD_DEVICE_STATE_LINE, "Node state");
      break;
    }
    default:
    {
      ClearLCD();
      PrintVersionOnLCD();
      WriteLineLCD(LCD_DEVICE_STATE_LINE, "Unknown state");
      break;
    }
  }
}

/********************************************
 * MAIN FUNCTION DEFINITIONS                *
 ********************************************/

void setup()
{
  SetupDebug();
  SetupAttention();
  SetupLCD();

  SetupSwitch();
  SetupSensor();

  UART_Init();
  UART_SendSoftwareResetRequest();
}

void loop()
{
  UART_ProcessIncomingCommand();
  PrintStateOnLCD();
  LoopLCD();

  if (MODEM_STATE_NODE != ModemState) return;

  LoopSwitch();
  LoopSensor();
}
