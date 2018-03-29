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
#include "MCU_Lightness.h"
#include "MCU_Sensor.h"
#include "Mesh.h"
#include "UART.h"
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

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
  pinMode(PIN_LED, OUTPUT);

  Timer3.initialize(ATTENTION_TIME_US);
  Timer3.attachInterrupt(IndicateAttention);
}

void ProcessEnterInitDevice(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Device State.");
  ModemState     = MODEM_STATE_INIT_DEVICE;
  AttentionState = false;

  if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_LIGHT_LC_SERVER))
  {
    DEBUG_INTERFACE.println("Modem does not support Light Lightness Controller Server.\n");
    return;
  }

  if (!Mesh_IsModelAvailable(p_payload, len, MESH_MODEL_ID_SENSOR_SERVER))
  {
    DEBUG_INTERFACE.println("Modem does not support Sensor Server.\n");
    return;
  }

  uint8_t model_ids[] = {
    // Light Lightness Controller server
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
  };

  UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
}

void ProcessEnterDevice(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Device State.\n");

  // Setting Lightness value in Device state to 50 % of maximum lightness
  ProcessTargetLightness(0xFFFF/2, 0); 

  ModemState = MODEM_STATE_DEVICE;
}

void ProcessEnterInitNode(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Node State.\n");
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

    if (MESH_MODEL_ID_LIGHT_LC_SERVER == model_id)
    {
      uint16_t current_model_id_instance_index = index/2;
      SetLightnessServerIdx(current_model_id_instance_index);
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
  }

  if (GetLightnessServerIdx() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    DEBUG_INTERFACE.println("Light Lightness Server model id not found in init node message");
    return;
  }

  if (GetSensorServerPIRIdx() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    DEBUG_INTERFACE.println("Sensor server (PIR) model id not found in init node message");
    return;
  }

  if (GetSensorServerALSIdx() == INSTANCE_INDEX_UNKNOWN)
  {
    ModemState = MODEM_STATE_UNKNOWN;
    DEBUG_INTERFACE.println("Sensor server (ALS) model id not found in init node message");
    return;
  }

  UART_StartNodeRequest();
}

void ProcessEnterNode(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Node State.\n");
  ModemState = MODEM_STATE_NODE;
  
  Mesh_SendLightLightnessGet(GetLightnessServerIdx());
}

void ProcessMeshCommand(uint8_t * p_payload, uint8_t len)
{
  Mesh_ProcessMeshCommand(p_payload, len);
}

void ProcessAttention(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Attention State %d\n\n.", p_payload[0]);
  AttentionState = (p_payload[0] == 0x01);
  if (!AttentionState)
  {
    AttentionLedValue = false;
    digitalWrite(PIN_LED, AttentionLedValue);
  }
}

void ProcessError(uint8_t * p_payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Error %d\n\n.", p_payload[0]);
}

void IndicateAttention(void)
{
  AttentionLedValue = AttentionState ? !AttentionLedValue : false;
  digitalWrite(PIN_LED, AttentionLedValue);
  IndicateAttentionLightness(AttentionState, AttentionLedValue);
}

void setup(void)
{
  SetupDebug();
  DEBUG_INTERFACE.println("Server Sample.\n");
  SetupAttention();

  SetupLightnessServer();
  SetupSensorServer();

  UART_Init();
  UART_SendSoftwareResetRequest();
}

void loop(void)
{
  UART_ProcessIncomingCommand();
  if (MODEM_STATE_NODE != ModemState) return;

  LoopSensorSever();
}
