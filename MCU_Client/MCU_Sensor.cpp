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

/*******************************************
 * INCLUDES                                *
 *******************************************/

#include "MCU_Sensor.h"
#include "Arduino.h"
#include "Config.h"
#include "LCD.h"
#include "UART.h"
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static uint32_t      AlsValueCLux    = 0;
static unsigned long AlsTimestamp    = (ULONG_MAX - DATA_VALIDITY_PERIOD_MS);
static bool          PirValue        = false;
static unsigned long PirTimestamp    = (ULONG_MAX - DATA_VALIDITY_PERIOD_MS);
static uint8_t       SensorClientIdx = INSTANCE_INDEX_UNKNOWN;

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void SetInstanceIdxSensor(uint8_t idx)
{
  SensorClientIdx = idx;
}

uint8_t GetInstanceIdxSensor(void)
{
  return SensorClientIdx;
}

void ProcessPresentAmbientLightLevel(uint16_t src_addr, uint32_t value_clux)
{
  AlsTimestamp = millis();
  AlsValueCLux = value_clux;
  INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRESENT AMBIENT LIGHT LEVEL with value of: %d.%02d\n\n", src_addr, AlsTimestamp, value_clux/100, value_clux%100);
}

void ProcessPresenceDetected(uint16_t src_addr, bool value)
{
  PirTimestamp = millis();
  PirValue     = value;
  INFO("Decoded Sensor Status message from 0x%04X [%d ms], PRESENCE DETECTED with value of: %d\n\n", src_addr, PirTimestamp, PirValue);
}

void SetupSensor(void)
{
  INFO("Sensor client initialization.\n");
}

void LoopSensor(void)
{
  char          text[LCD_COLUMNS];
  unsigned long timestamp = millis();

  const int integral   = AlsValueCLux/100;
  const int fractional = AlsValueCLux%100;

  strcpy(text, "ALS: ");
  itoa(integral, text + strlen(text), 10);
  strcpy(text + strlen(text), ".00");
  itoa(fractional, text + strlen(text) - (fractional < 10 ? 1 : 2), 10);

  if (((AlsTimestamp + DATA_VALIDITY_PERIOD_MS) <= timestamp) || (AlsTimestamp >= timestamp))
  {
    strcpy(text + strlen(text), " (N/A)");
  }

  WriteLineLCD(LCD_SENSOR_ALS_LINE, text);

  strcpy(text, "PIR: ");
  itoa(PirValue, text + strlen(text), 10);

  if (((PirTimestamp + DATA_VALIDITY_PERIOD_MS) <= timestamp) || (PirTimestamp >= timestamp))
  {
    strcpy(text + strlen(text), " (N/A)");
  }

  WriteLineLCD(LCD_SENSOR_PIR_LINE, text);
}
