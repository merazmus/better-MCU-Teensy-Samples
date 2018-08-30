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
#include "MCU_Health.h"
#include "MCU_Lightness.h"
#include "MCU_Sensor.h"
#include "MCU_DFU.h"
#include "Mesh.h"
#include "UART.h"
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>
#include <string.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/
 
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

static ModemState_t ModemState = MODEM_STATE_UNKNOWN;
static bool         AttentionState;
static bool         AttentionLedValue;

#if ENABLE_CTL==0
static const bool CTLEnabled = false;
#else
static const bool CTLEnabled = true;
#endif

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
 *  Process Start Test Request command
 *
 *  @param * p_payload   Command payload
 *  @param len           Payload len
 */
void ProcessStartTest(uint8_t * p_payload, uint8_t len);

/*
 *  Send Firmware Version Set request
 */
void SendFirmwareVersionSet(void);

/*
 *  Process Firmware Version set response
 */
void ProcessFirmwareVersionSet(void);

/*
 *  Timer3 Tick
 */
void Timer3Tick(void);

/*
 *  Indicate attention
 */
void IndicateAttention(void);

/*
 *  Main Arduino setup
 */
void setup(void);

/*
 *  Main Arduino loop
 */
void loop(void);

/********************************************
 * FUNCTION DEFINITIONS                     *
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

  Timer3.initialize(TIMER_THREE_PERIOD);
  Timer3.attachInterrupt(Timer3Tick);
}

void ProcessEnterInitDevice(uint8_t * p_payload, uint8_t len)
{
  INFO("Init Device State.\n");
  ModemState     = MODEM_STATE_INIT_DEVICE;
  AttentionState = false;

  if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_SENSOR_SERVER))
  {
    INFO("Modem does not support Sensor Server.\n");
    return;
  }

  if (Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_CTL_SERVER) && CTLEnabled)
  {
    uint8_t model_ids[] = {
      // Light CTL server
      lowByte(MESH_MODEL_ID_LIGHT_CTL_SERVER),
      highByte(MESH_MODEL_ID_LIGHT_CTL_SERVER),
      lowByte(0),
      highByte(0),
      lowByte(0),
      highByte(0),

      // PIR Sensor Server
      lowByte(MESH_MODEL_ID_SENSOR_SERVER),
      highByte(MESH_MODEL_ID_SENSOR_SERVER),
      0x01,
      lowByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
      highByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
      lowByte(PIR_POSITIVE_TOLERANCE),
      highByte(PIR_POSITIVE_TOLERANCE),
      lowByte(PIR_NEGATIVE_TOLERANCE),
      highByte(PIR_NEGATIVE_TOLERANCE),
      PIR_SAMPLING_FUNCTION,
      PIR_MEASUREMENT_PERIOD,
      PIR_UPDATE_INTERVAL,

      // ALS Sensor Server
      lowByte(MESH_MODEL_ID_SENSOR_SERVER),
      highByte(MESH_MODEL_ID_SENSOR_SERVER),
      0x01,
      lowByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
      highByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
      lowByte(ALS_POSITIVE_TOLERANCE),
      highByte(ALS_POSITIVE_TOLERANCE),
      lowByte(ALS_NEGATIVE_TOLERANCE),
      highByte(ALS_NEGATIVE_TOLERANCE),
      ALS_SAMPLING_FUNCTION,
      ALS_MEASUREMENT_PERIOD,
      ALS_UPDATE_INTERVAL,

      // Health Server
      lowByte(MESH_MODEL_ID_HEALTH_SERVER),
      highByte(MESH_MODEL_ID_HEALTH_SERVER),
      0x01,
      lowByte(SILVAIR_ID),
      highByte(SILVAIR_ID),
    };

    SendFirmwareVersionSet();
    UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
  }
  else if (Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_LC_SERVER))
  {
    uint8_t model_ids[] = {
      // Light CTL server
      lowByte(MESH_MODEL_ID_LIGHT_LC_SERVER),
      highByte(MESH_MODEL_ID_LIGHT_LC_SERVER),

      // PIR Sensor Server
      lowByte(MESH_MODEL_ID_SENSOR_SERVER),
      highByte(MESH_MODEL_ID_SENSOR_SERVER),
      0x01,
      lowByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
      highByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
      lowByte(PIR_POSITIVE_TOLERANCE),
      highByte(PIR_POSITIVE_TOLERANCE),
      lowByte(PIR_NEGATIVE_TOLERANCE),
      highByte(PIR_NEGATIVE_TOLERANCE),
      PIR_SAMPLING_FUNCTION,
      PIR_MEASUREMENT_PERIOD,
      PIR_UPDATE_INTERVAL,

      // ALS Sensor Server
      lowByte(MESH_MODEL_ID_SENSOR_SERVER),
      highByte(MESH_MODEL_ID_SENSOR_SERVER),
      0x01,
      lowByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
      highByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
      lowByte(ALS_POSITIVE_TOLERANCE),
      highByte(ALS_POSITIVE_TOLERANCE),
      lowByte(ALS_NEGATIVE_TOLERANCE),
      highByte(ALS_NEGATIVE_TOLERANCE),
      ALS_SAMPLING_FUNCTION,
      ALS_MEASUREMENT_PERIOD,
      ALS_UPDATE_INTERVAL,

      // Health Server
      lowByte(MESH_MODEL_ID_HEALTH_SERVER),
      highByte(MESH_MODEL_ID_HEALTH_SERVER),
      0x01,
      lowByte(SILVAIR_ID),
      highByte(SILVAIR_ID),
    };

    SendFirmwareVersionSet();
    UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
  }
  else
  {
    INFO("Modem does support neither Light CTL Server nor Light LC Server.\n");
    return;
  }
}

void ProcessEnterDevice(uint8_t * p_payload, uint8_t len)
{
  INFO("Device State.\n");

  // Setting Lightness value in Device state to 50 % of maximum lightness
  ProcessTargetLightness(0xFFFF/2, 0xFFFF/2, 0); 

  ModemState = MODEM_STATE_DEVICE;
}

void ProcessEnterInitNode(uint8_t * p_payload, uint8_t len)
{
  INFO("Init Node State.\n");
  ModemState     = MODEM_STATE_INIT_NODE;
  AttentionState = false;

  SetLightnessServerIdx(INSTANCE_INDEX_UNKNOWN);
  SetSensorServerPIRIdx(INSTANCE_INDEX_UNKNOWN);
  SetSensorServerALSIdx(INSTANCE_INDEX_UNKNOWN);

  uint8_t sensor_server_model_id_occurency = 0;

  for (size_t index = 0; index < len;)
  {
    uint16_t model_id = ((uint16_t)p_payload[index++]);
    model_id         |= ((uint16_t)p_payload[index++] << 8);

    if (MESH_MODEL_ID_LIGHT_CTL_SERVER == model_id || MESH_MODEL_ID_LIGHT_LC_SERVER == model_id)
    {
      uint16_t current_model_id_instance_index = index/2;
      SetLightnessServerIdx(current_model_id_instance_index);
      SetLightCTLSupport(model_id == MESH_MODEL_ID_LIGHT_CTL_SERVER);
    }

    if (model_id == MESH_MODEL_ID_SENSOR_SERVER)
    {
      uint16_t current_model_id_instance_index = index/2;
      sensor_server_model_id_occurency++;

      if (sensor_server_model_id_occurency == PIR_REGISTRATION_ORDER)
      {
        SetSensorServerPIRIdx(current_model_id_instance_index);
      }
      else if (sensor_server_model_id_occurency == ALS_REGISTRATION_ORDER)
      {
        SetSensorServerALSIdx(current_model_id_instance_index);
      }
    }

    if (model_id == MESH_MODEL_ID_HEALTH_SERVER)
    {
      uint16_t current_model_id_instance_index = index/2;
      SetHealthServerIdx(current_model_id_instance_index);
    }
  }

  if (GetLightnessServerIdx() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    INFO("Light CTL Server model id not found in init node message\n");
    return;
  }

  if (GetSensorServerPIRIdx() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    INFO("Sensor server (PIR) model id not found in init node message\n");
    return;
  }

  if (GetSensorServerALSIdx() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    INFO("Sensor server (ALS) model id not found in init node message\n");
    return;
  }

   if (GetHealthServerIdx() == INSTANCE_INDEX_UNKNOWN)
   {
     ModemState = MODEM_STATE_UNKNOWN;
     INFO("Health Server model id not found in init node message\n");
     return;
   }

  SendFirmwareVersionSet();
  UART_StartNodeRequest();
}

void ProcessEnterNode(uint8_t * p_payload, uint8_t len)
{
  INFO("Node State.\n");
  ModemState = MODEM_STATE_NODE;
  
  Mesh_SendLightLightnessGet(GetLightnessServerIdx());
}

void ProcessMeshCommand(uint8_t * p_payload, uint8_t len)
{
  Mesh_ProcessMeshCommand(p_payload, len);
}

void ProcessAttention(uint8_t * p_payload, uint8_t len)
{
  INFO("Attention State %d\n\n.", p_payload[0]);
  AttentionState = (p_payload[0] == 0x01);
  if (!AttentionState)
  {
    AttentionLedValue = false;
    digitalWrite(PIN_LED_STATUS, AttentionLedValue);
  }
}
 
void ProcessError(uint8_t * p_payload, uint8_t len)
{
  INFO("Error %d\n\n.", p_payload[0]);
}

void Timer3Tick(void)
{
  IndicateAttention();
  IndicateHealth();
}

void IndicateAttention(void)
{
  if(!IsTestInProgress())
  {
    AttentionLedValue = AttentionState ? !AttentionLedValue : false;
    digitalWrite(PIN_LED_STATUS, AttentionLedValue);
    IndicateAttentionLightness(AttentionState, AttentionLedValue);
  }
}

void SendFirmwareVersionSet(void)
{
  const char * p_firmware_version = FIRMWARE_VERSION;

  UART_SendFirmwareVersionSetRequest((uint8_t *)p_firmware_version, strlen(p_firmware_version));
}

void ProcessFirmwareVersionSet(void)
{
}

void setup(void)
{
  SetupDebug();
  INFO("Server Sample.\n");
  SetupAttention();
  SetupHealth();

  SetupLightnessServer();
  SetupSensorServer();

  UART_Init();
  UART_SendSoftwareResetRequest();

  SetupDFU();
}

void loop(void)
{
  UART_ProcessIncomingCommand();

  if (!MCU_DFU_IsInProgress())
  {
    LoopHealth();

    if (MODEM_STATE_NODE == ModemState)
    {
      LoopSensorSever();
    }
  }
}
