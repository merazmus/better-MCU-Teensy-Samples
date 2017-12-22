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
#include "Mesh.h"
#include "Common.h"
#include <TimerThree.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define PIN_PIR                     20           /**< Defines PIR sensor location */
#define PIN_ALS                     18           /**< Defines ALS sensor location */

#define ALS_CONVERSION_COEFFICIENT  1600UL       /**< Defines light sensor coefficient */
#define PIR_DEBOUNCE_TIME_MS        20           /**< Defines PIR debounce time in miliseconds */

#define PIR_INERTIA_MS              4000         /**< Defines PIR inertia in miliseconds */
/*
*  Sensor properties
*/
#define PIR_POSITIVE_TOLERANCE                            0x0000
#define PIR_NEGATIVE_TOLERANCE                            0x0000
#define PIR_SAMPLING_FUNCTION                             0x01
#define PIR_MEASUREMENT_PERIOD                            0x40
#define PIR_UPDATE_INTERVAL                               0x40
#define ALS_POSITIVE_TOLERANCE                            0x0000
#define ALS_NEGATIVE_TOLERANCE                            0x0000
#define ALS_SAMPLING_FUNCTION                             0x01
#define ALS_MEASUREMENT_PERIOD                            0x40
#define ALS_UPDATE_INTERVAL                               0x40

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static volatile uint32_t PirTimestamp      = 0;
static volatile bool     AttentionState    = false; 
static bool              AttentionLedValue = false;
static ModemState_t      ModemState        = MODEM_STATE_UNKNOWN;

/********************************************
 * FUNCTION DEFINITIONS                     *
 ********************************************/

void SetupDebug(void)
{
  DEBUG_INTERFACE.begin(DEBUG_INTERFACE_BAUDRATE);
  unsigned long timestamp = millis();
  while(!DEBUG_INTERFACE && (timestamp + 1000 > millis()));
}

void SetupAttention(void)
{
  pinMode(PIN_LED, OUTPUT);

  Timer3.initialize(ATTENTION_TIME_US);
  Timer3.attachInterrupt(IndicateAttention);
}

void SetupPIR(void)
{
  pinMode(PIN_PIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_PIR), InterruptPIR, RISING);
}

void ProcessEnterInitDevice(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Device State.\n");
  ModemState     = MODEM_STATE_INIT_DEVICE;
  AttentionState = false;

  Mesh_ResetRegisteredModelId();

  if (!Mesh_IsModelAvailable(payload, len, MESH_MODEL_ID_SENSOR_SERVER))
  {
    DEBUG_INTERFACE.println("Modem does not support Sensor Server.\n");
    return;
  }

  uint8_t model_ids[] = 
  {
    // PIR Sensor Server
    lowByte (MESH_MODEL_ID_SENSOR_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SERVER),
    0x01,
    lowByte (MESH_PROPERTY_ID_PRESENCE_DETECTED),
    highByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
    lowByte (PIR_POSITIVE_TOLERANCE),
    highByte(PIR_POSITIVE_TOLERANCE),
    lowByte (PIR_NEGATIVE_TOLERANCE),
    highByte(PIR_NEGATIVE_TOLERANCE),
    PIR_SAMPLING_FUNCTION,
    PIR_MEASUREMENT_PERIOD,
    PIR_UPDATE_INTERVAL,

    // PIR Sensor Setup Server
    lowByte (MESH_MODEL_ID_SENSOR_SETUP_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SETUP_SERVER),

    // ALS Sensor Server
    lowByte (MESH_MODEL_ID_SENSOR_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SERVER),
    0x01,
    lowByte (MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
    highByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
    lowByte (ALS_POSITIVE_TOLERANCE),
    highByte(ALS_POSITIVE_TOLERANCE),
    lowByte (ALS_NEGATIVE_TOLERANCE),
    highByte(ALS_NEGATIVE_TOLERANCE),
    ALS_SAMPLING_FUNCTION,
    ALS_MEASUREMENT_PERIOD,
    ALS_UPDATE_INTERVAL,

    // ALS Sensor Setup Server
    lowByte (MESH_MODEL_ID_SENSOR_SETUP_SERVER),
    highByte(MESH_MODEL_ID_SENSOR_SETUP_SERVER),
  };
  UART_SendCreateInstancesRequest(model_ids, sizeof(model_ids));
}

void ProcessEnterDevice(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Device State.\n");
  ModemState = MODEM_STATE_DEVICE;
}

void ProcessEnterInitNode(uint8_t * payload, uint8_t len)
{
  Mesh_ResetRegisteredModelId();
  DEBUG_INTERFACE.println("Init Node State.\n");
  ModemState     = MODEM_STATE_INIT_NODE;
  AttentionState = false;

  for (size_t index = 0; index < len;)
  {
    uint16_t model_id = ((uint16_t)payload[index++]);
    model_id         |= ((uint16_t)payload[index++] << 8);
    Mesh_AddRegisteredModelId(model_id);
  }

  UART_StartNodeRequest();
}

void ProcessEnterNode(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Node State.\n");
  ModemState = MODEM_STATE_NODE;
}

void ProcessMeshCommand(uint8_t * payload, uint8_t len)
{
  Mesh_ProcessMeshCommand(payload, len);
}

void ProcessAttention(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Attention State %d\n\n.", payload[0]);
  AttentionState = (payload[0] == 0x01);
}

void ProcessError(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.printf("Error %d\n\n.", payload[0]);
}

void IndicateAttention(void)
{
  AttentionLedValue = AttentionState ? !AttentionLedValue : false;
  digitalWrite(PIN_LED, AttentionLedValue);
}

void InterruptPIR(void)
{
  PirTimestamp = millis();
}

void setup()
{
  SetupDebug();
  DEBUG_INTERFACE.println("Sensor Server Sample.\n");
  SetupPIR();
  SetupAttention();
  UART_Init();
  UART_SendSoftwareResetRequest();
}

void loop()
{
  UART_ProcessIncomingCommand();

  if (MODEM_STATE_NODE != ModemState) return;

  delay(1000);
  bool    pir       = digitalRead(PIN_PIR) || (millis() < (PirTimestamp + PIR_INERTIA_MS));

  uint8_t pir_buf[] = 
  {
    0x01,
    lowByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
    highByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
    pir,
  };
  UART_SendSensorUpdateRequest(pir_buf, sizeof(pir_buf));

  uint32_t als       = ALS_CONVERSION_COEFFICIENT * analogRead(PIN_ALS);
  uint8_t  als_buf[] = 
  {
    0x03,
    lowByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
    highByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
    (uint8_t) als,
    (uint8_t) (als >> 8),
    (uint8_t) (als >> 16),
  };
  UART_SendSensorUpdateRequest(als_buf, sizeof(als_buf));
}
