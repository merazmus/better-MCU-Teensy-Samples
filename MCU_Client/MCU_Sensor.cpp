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
#include <limits.h>
#include <stdint.h>
#include <stdio.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define ALS_STRING_WIDTH      4         /**< Defines ALS string width */
#define ALS_STRING_PRECISION  2         /**< Defines ALS string precision */

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static float         AlsValue        = 0.0;
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

void ProcessPresentAmbientLightLevel(uint16_t src_addr, float value)
{
  AlsTimestamp = millis();
  AlsValue     = value;
  DEBUG_INTERFACE.printf("Decoded Sensor Status message from 0x%04X [%d ms], PRESENT AMBIENT LIGHT LEVEL with value of: %d\n\n", src_addr, AlsTimestamp, int(AlsValue));
}

void ProcessPresenceDetected(uint16_t src_addr, bool value)
{
  PirTimestamp = millis();
  PirValue     = value;
  DEBUG_INTERFACE.printf("Decoded Sensor Status message from 0x%04X [%d ms], PRESENCE DETECTED with value of: %d\n\n", src_addr, PirTimestamp, PirValue);
}

void SetupSensor(void)
{
  DEBUG_INTERFACE.println("Sensor client initialization.\n");
}

void LoopSensor(void)
{
  char          text[LCD_COLUMNS];
  char          als_text[ALS_STRING_WIDTH + ALS_STRING_PRECISION];
  unsigned long timestamp = millis();
  dtostrf(AlsValue, ALS_STRING_WIDTH, ALS_STRING_PRECISION, als_text);

  if (((AlsTimestamp + DATA_VALIDITY_PERIOD_MS) > timestamp) && (AlsTimestamp < timestamp))
  {
    sprintf(text, "ALS: %s", als_text);
  }
  else
  {
    sprintf(text, "ALS: %s (N/A)", als_text);
  }
  WriteLineLCD(LCD_SENSOR_ALS_LINE, text);


  if (((PirTimestamp + DATA_VALIDITY_PERIOD_MS) > timestamp) && (PirTimestamp < timestamp))
  {
    sprintf(text, "PIR: %d", PirValue);
  }
  else
  {
    sprintf(text, "PIR: %d (N/A)", PirValue);
  }
  WriteLineLCD(LCD_SENSOR_PIR_LINE, text);
}
