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
#include "SDM.h"
#include "MCU_Sensor.h"
#include <TimerOne.h>
#include <TimerThree.h>
#include <math.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define PIN_PIR                     5            /**< Defines PIR sensor location */
#define PIN_ALS                     17           /**< Defines ALS sensor location */

#define ALS_CONVERSION_COEFFICIENT  5UL          /**< Defines light sensor coefficient [centilux / millivolt]*/
#define PIR_DEBOUNCE_TIME_MS        20           /**< Defines PIR debounce time in milliseconds */
#define PIR_INERTIA_MS              4000         /**< Defines PIR inertia in milliseconds */
#define SENSOR_UPDATE_INTV          1000         /**< Defines sensor update in milliseconds */
#define ALS_REPORT_THRESHOLD        500          /**< Defines sensor threshold in centilux */

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static bool              IsEnabled                = false;
static volatile uint32_t PirTimestamp             = 0;
static uint8_t           SensorServerPirIdx       = INSTANCE_INDEX_UNKNOWN;
static uint8_t           SensorServerAlsIdx       = INSTANCE_INDEX_UNKNOWN;
static uint8_t           SensorServerVoltCurrIdx  = INSTANCE_INDEX_UNKNOWN;
static uint8_t           SensorServerPowEnergyIdx = INSTANCE_INDEX_UNKNOWN;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/**
 * Convert floating point value to Voltage Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint16_t ConvertFloatToVoltage(float voltage);

/**
 * Convert floating point value to Electric Current Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint16_t ConvertFloatToCurrent(float voltage);

/**
 * Convert floating point value to Power Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint32_t ConvertFloatToPower(float power);

/**
 * Convert floating point value to Energy Characteristic
 *
 * @param voltage   value
 * @return          encoded value
 */
static uint32_t ConvertFloatToEnergy(float energy);

/**
 * Process PIR update
 */
static void ProcessPIR(void);

/**
 * Process ALS update
 */
static void ProcessALS(void);

/**
 * Process voltage, current update
 */
static void ProcessVoltCurr(void);

/**
 * Process power and energy update
 */
static void ProcessPowEnergy(void);

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

void SetSensorServerVoltCurrIdx(uint8_t idx)
{
  SensorServerVoltCurrIdx = idx;
}

uint8_t GetSensorServerVoltCurrIdx(void)
{
  return SensorServerVoltCurrIdx;
}

void SetSensorServerPowEnergyIdx(uint8_t idx)
{
  SensorServerPowEnergyIdx = idx;
}

uint8_t GetSensorServerPowEnergyIdx(void)
{
  return SensorServerPowEnergyIdx;
}

void InterruptPIR(void)
{
  PirTimestamp = millis();
}

void SetupSensorServer(void)
{
  pinMode(PIN_PIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_PIR), InterruptPIR, RISING);
  IsEnabled = true;
}

void LoopSensorSever(void)
{
  if (!IsEnabled) return;
  static unsigned long timestamp = 0;

  if (timestamp + SENSOR_UPDATE_INTV < millis())
  {
    ProcessPIR();
    ProcessALS();
    ProcessVoltCurr();
    ProcessPowEnergy();
    timestamp = millis();
  }
}

/********************************************
 * LOCAL FUNCTION DEFINITIONS               *
 *******************************************/

static void ProcessPIR(void)
{
  if(GetSensorServerPIRIdx() != INSTANCE_INDEX_UNKNOWN)
  {
    bool pir = digitalRead(PIN_PIR) || (millis() < (PirTimestamp + PIR_INERTIA_MS));

    uint8_t pir_buf[] = {
        SensorServerPirIdx,
        lowByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
        highByte(MESH_PROPERTY_ID_PRESENCE_DETECTED),
        pir,
    };
    UART_SendSensorUpdateRequest(pir_buf, sizeof(pir_buf));
  }
}

static void ProcessALS(void)
{
  if(GetSensorServerALSIdx() != INSTANCE_INDEX_UNKNOWN)
  {
    uint32_t als_adc_val    = analogRead(PIN_ALS);
    uint32_t als_millivolts = (als_adc_val * ANALOG_REFERENCE_VOLTAGE_MV) / ANALOG_MAX;
    uint32_t als_centilux   = als_millivolts * ALS_CONVERSION_COEFFICIENT;

    /*
     * Sensor server can be configured to report on change. In one mode report is triggered by
     * percentage change from the actual value. In case of small measurement, it can generate heavy traffic.
     */
    if (als_centilux < ALS_REPORT_THRESHOLD)
    {
      als_centilux = 0;
    }

    uint8_t als_buf[] = {
        SensorServerAlsIdx,
        lowByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
        highByte(MESH_PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL),
        (uint8_t)als_centilux,
        (uint8_t)(als_centilux >> 8),
        (uint8_t)(als_centilux >> 16),
    };

    UART_SendSensorUpdateRequest(als_buf, sizeof(als_buf));
  }
}

static void ProcessVoltCurr(void)
{
  uint16_t voltage;
  uint16_t current;

  const SDM_State_T * p_sdm_state = SDM_GetState();

  if (p_sdm_state != NULL)
  {
    voltage = ConvertFloatToVoltage(p_sdm_state->voltage);
    current = ConvertFloatToCurrent(p_sdm_state->current);
  }
  else
  {
    voltage = MESH_PROPERTY_PRESENT_INPUT_VOLTAGE_UNKNOWN_VAL;
    current = MESH_PROPERTY_PRESENT_INPUT_CURRENT_UNKNOWN_VAL;
  }

  if(GetSensorServerVoltCurrIdx() != INSTANCE_INDEX_UNKNOWN)
  {
    uint8_t voltcurr_buf[] = {
        SensorServerVoltCurrIdx,
        lowByte(MESH_PROPERTY_ID_PRESENT_INPUT_VOLTAGE),
        highByte(MESH_PROPERTY_ID_PRESENT_INPUT_VOLTAGE),
        (uint8_t)voltage,
        (uint8_t)(voltage >> 8),
        lowByte(MESH_PROPERTY_ID_PRESENT_INPUT_CURRENT),
        highByte(MESH_PROPERTY_ID_PRESENT_INPUT_CURRENT),
        (uint8_t)current,
        (uint8_t)(current >> 8),
    };
    UART_SendSensorUpdateRequest(voltcurr_buf, sizeof(voltcurr_buf));
  }
}

static void ProcessPowEnergy(void)
{
  uint32_t power;
  uint32_t energy;

  const SDM_State_T * p_sdm_state = SDM_GetState();

  if (p_sdm_state != NULL)
  {
    power  = ConvertFloatToPower(p_sdm_state->active_power);
    energy = ConvertFloatToEnergy(p_sdm_state->total_active_energy);
  }
  else
  {
    power  = MESH_PROPERTY_PRESENT_DEVICE_INPUT_POWER_UNKNOWN_VAL;
    energy = MESH_PROPERTY_TOTAL_DEVICE_ENERGY_USE_UNKNOWN_VAL;
  }

  if (GetSensorServerPowEnergyIdx() != INSTANCE_INDEX_UNKNOWN)
  {
    uint8_t powenergy_buf[] = {
        SensorServerPowEnergyIdx,
        lowByte(MESH_PROPERTY_ID_PRESENT_DEVICE_INPUT_POWER),
        highByte(MESH_PROPERTY_ID_PRESENT_DEVICE_INPUT_POWER),
        (uint8_t)power,
        (uint8_t)(power >> 8),
        (uint8_t)(power >> 16),
        lowByte(MESH_PROPERTY_ID_TOTAL_DEVICE_ENERGY_USE),
        highByte(MESH_PROPERTY_ID_TOTAL_DEVICE_ENERGY_USE),
        (uint8_t)energy,
        (uint8_t)(energy >> 8),
        (uint8_t)(energy >> 16),
    };
    UART_SendSensorUpdateRequest(powenergy_buf, sizeof(powenergy_buf));
  }
}

static uint16_t ConvertFloatToVoltage(float voltage)
{
  return (uint16_t) (voltage * 64);
}

static uint16_t ConvertFloatToCurrent(float voltage)
{
  return (uint16_t) (voltage * 100);
}

static uint32_t ConvertFloatToPower(float power)
{
  return (uint32_t) (power * 10);
}

static uint32_t ConvertFloatToEnergy(float energy)
{
  return (uint32_t) energy;
}
