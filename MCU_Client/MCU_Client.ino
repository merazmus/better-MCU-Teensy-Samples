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
#include "MCU_DFU.h"
#include "MCU_Attention.h"
#include "Mesh.h"
#include "UART.h"
#include <limits.h>
#include <string.h>
#include "MCU_Definitions.h"

/********************************************
 * EXPORTED #define CONSTANTS AND MACROS    *
 ********************************************/

/********************************************
 * EXPORTED TYPES DEFINITIONS               *
 ********************************************/

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static ModemState_t ModemState        = MODEM_STATE_UNKNOWN;
static bool         LastDfuInProgress = false;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/*
 *  Setup debug interface
 */
void SetupDebug(void);

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
 *  Process Error command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessError(uint8_t * p_payload, uint8_t len);

/*
 *  Send Firmware Version Set request
 */
void SendFirmwareVersionSetRequest(void);

/*
 *  Process FactoryResetEvent
 */
void ProcessFactoryResetEvent(void);

/*
 *  Print current UART Modem state on LCD
 */
void PrintStateOnLCD(void);

/*
 *  Update everything on LCD
 */
void UpdateLCD(void);

/*
 *  Check if DFU state was changed.
 */
bool IsDfuStateChanged(void);

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

void ProcessEnterInitDevice(uint8_t * p_payload, uint8_t len)
{
  INFO("Init Device State.\n");
  ModemState = MODEM_STATE_INIT_DEVICE;
  LCD_UpdateModemState(ModemState);
  AttentionStateSet(false);

  SetInstanceIdxSwitch1(INSTANCE_INDEX_UNKNOWN);
  SetInstanceIdxSwitch2(INSTANCE_INDEX_UNKNOWN);
  SetInstanceIdxSensor(INSTANCE_INDEX_UNKNOWN);

  if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_LC_CLIENT))
  {
    INFO("Modem does not support Light Lightness Controler Client.\n");
    return;
  }

  if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_SENSOR_CLIENT))
  {
    INFO("Modem does not support Sensor Client.\n");
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
  
  SendFirmwareVersionSetRequest();
  UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
  UART_ModemFirmwareVersionRequest();
}

void ProcessEnterDevice(uint8_t * p_payload, uint8_t len)
{
  INFO("Device State.\n");
  ModemState = MODEM_STATE_DEVICE;
  LCD_UpdateModemState(ModemState);
}

void ProcessEnterInitNode(uint8_t * p_payload, uint8_t len)
{
  INFO("Init Node State.\n");
  ModemState = MODEM_STATE_INIT_NODE;
  LCD_UpdateModemState(ModemState);
  AttentionStateSet(false);

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
    LCD_UpdateModemState(ModemState);
    INFO("First Light Lightness Controller Client model id not found in init node message\n");
    return;
  }

  if (GetInstanceIdxSwitch2() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    LCD_UpdateModemState(ModemState);
    INFO("Second Light Lightness Controller Client model id not found in init node message\n");
    return;
  }

  if (GetInstanceIdxSensor() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    LCD_UpdateModemState(ModemState);
    INFO("Sensor Client model id not found in init node message\n");
    return;
  }

  SendFirmwareVersionSetRequest();
  UART_StartNodeRequest();
  UART_ModemFirmwareVersionRequest();
}

void ProcessEnterNode(uint8_t * p_payload, uint8_t len)
{
  INFO("Node State.\n");
  ModemState = MODEM_STATE_NODE;
  LCD_UpdateModemState(ModemState);
}

void ProcessMeshCommand(uint8_t * p_payload, uint8_t len)
{
  Mesh_ProcessMeshCommand(p_payload, len);
}

void ProcessError(uint8_t * p_payload, uint8_t len)
{
  INFO("Error %d\n\n.", p_payload[0]);
}

void ProcessModemFirmwareVersion(uint8_t * p_payload, uint8_t len)
{
  INFO("Process Modem Firmware Version\n");
  LCD_UpdateModemFwVersion((char*)p_payload, len);
}

bool IsDfuStateChanged(void)
{
  return LastDfuInProgress ^ MCU_DFU_IsInProgress();
}

void SendFirmwareVersionSetRequest(void)
{
  const char * p_firmware_version = BUILD_NUMBER;

  UART_SendFirmwareVersionSetRequest((uint8_t *)p_firmware_version, strlen(p_firmware_version));
}

void ProcessFactoryResetEvent(void)
{
  LCD_EraseSensorsValues();
}

/********************************************
 * MAIN FUNCTION DEFINITIONS                *
 ********************************************/

void setup()
{
  SetupDebug();

  SetupAttention();
  LCD_Setup();

  SetupSwitch();
  SetupSensor();

  UART_Init();
  UART_SendSoftwareResetRequest();

  SetupDFU();
}

void loop()
{
  Mesh_Loop();
  LCD_Loop();
  LoopAttention();
  UART_ProcessIncomingCommand();

  switch (ModemState)
  {
    case MODEM_STATE_UNKNOWN:
    case MODEM_STATE_INIT_DEVICE:
    case MODEM_STATE_INIT_NODE:
      break;

    case MODEM_STATE_DEVICE:
    case MODEM_STATE_NODE:

      if (IsDfuStateChanged())
      {
        LastDfuInProgress = MCU_DFU_IsInProgress();
        LCD_UpdateDfuState(LastDfuInProgress);
      }

      if (!MCU_DFU_IsInProgress())
      {
        LoopSwitch();
      }
      break;
  }
}
