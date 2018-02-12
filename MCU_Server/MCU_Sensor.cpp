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
#include "Mesh.h"
#include "UART.h"
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define PIN_PIR                     5            /**< Defines PIR sensor location */
#define PIN_ALS                     18           /**< Defines ALS sensor location */

#define ALS_CONVERSION_COEFFICIENT  16UL         /**< Defines light sensor coefficient */
#define PIR_DEBOUNCE_TIME_MS        20           /**< Defines PIR debounce time in milliseconds */
#define PIR_INERTIA_MS              4000         /**< Defines PIR inertia in milliseconds */
#define SENSOR_UPDATE_INTV          1000         /**< Defines sensor update in milliseconds */
#define ALS_REPORT_THRESHOLD        500

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static volatile uint32_t PirTimestamp       = 0;
static uint8_t           SensorServerPirIdx = INSTANCE_INDEX_UNKNOWN;
static uint8_t           SensorServerAlsIdx = INSTANCE_INDEX_UNKNOWN;

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void SetSensorServerALSIdx(uint8_t idx)
{
  SensorServerAlsIdx = idx;
}

uint8_t GetSensorServerALSIdx(void)
{
  return SensorServerAlsIdx;
}

void SetSensorServerPIRIdx(uint8_t idx)
{
  SensorServerPirIdx = idx;
}

uint8_t GetSensorServerPIRIdx(void)
{
  return SensorServerPirIdx;
}

void InterruptPIR(void)
{
  PirTimestamp = millis();
}

void SetupSensorServer(void)
{
  pinMode(PIN_PIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_PIR), InterruptPIR, RISING);
}

void LoopSensorSever(void)
{
  static unsigned long timestamp = 0;

  if (timestamp + SENSOR_UPDATE_INTV < millis())
  {
    bool pir = digitalRead(PIN_PIR) || (millis() < (PirTimestamp + PIR_INERTIA_MS));

    uint8_t pir_buf[] = {
      SensorServerPirIdx,
      lowByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
      highByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
      pir,
    };
    UART_SendSensorUpdateRequest(pir_buf, sizeof(pir_buf));

    uint32_t als = ALS_CONVERSION_COEFFICIENT * analogRead(PIN_ALS);
    /* 
     * Sensor server can be configured to report on change. In one mode report is triggered by
     * percentage change from the actual value. In case of small measurement, it can generate heavy traffic.
     */
    if (als < ALS_REPORT_THRESHOLD)
    {
      als = 0;
    }

    uint8_t als_buf[] = {
      SensorServerAlsIdx,
      lowByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
      highByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
      (uint8_t)als,
      (uint8_t)(als >> 8),
      (uint8_t)(als >> 16),
    };

    UART_SendSensorUpdateRequest(als_buf, sizeof(als_buf));
    timestamp = millis();
  }
}
