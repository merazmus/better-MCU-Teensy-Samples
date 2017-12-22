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
#include <LiquidCrystal.h>
#include <limits.h>
#include <TimerThree.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define PIN_LCD_D4              5
#define PIN_LCD_D5              4
#define PIN_LCD_D6              3
#define PIN_LCD_D7              2
#define PIN_LCD_RS              12
#define PIN_LCD_EN              11
#define LCD_REFRESH_INTV_MS     500
#define DATA_VALIDITY_PERIOD    3000

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static LiquidCrystal Lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
static float         AlsValue          = 0.0;
static unsigned long AlsTimestamp      = (ULONG_MAX - DATA_VALIDITY_PERIOD);
static bool          PirValue          = false;
static unsigned long PirTimestamp      = (ULONG_MAX - DATA_VALIDITY_PERIOD);
static bool          AttentionState    = false; 
static bool          AttentionLedValue = false;
static ModemState_t  ModemState        = MODEM_STATE_UNKNOWN;

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

void SetupLCD(void)
{
  // Set up the LCD's number of columns and rows:
  Lcd.begin(16, 2);
}

void UpdateLCD(void)
{
  static unsigned long last_displayed_timestamp = 0;
  unsigned long current_timestamp = millis();

  if ((current_timestamp - last_displayed_timestamp) < LCD_REFRESH_INTV_MS) return;
  last_displayed_timestamp = current_timestamp;

  Lcd.clear();

  if (ModemState == MODEM_STATE_NODE)
  {
    Lcd.setCursor(0,0);
    Lcd.print("ALS: ");
    if ((AlsTimestamp + DATA_VALIDITY_PERIOD) > millis() && AlsTimestamp < millis())
    {
      Lcd.print(AlsValue);
    }
    else
    {
      Lcd.print("N/A");
    }

    Lcd.setCursor(0,1);
    Lcd.print("PIR: ");
    if ((PirTimestamp + DATA_VALIDITY_PERIOD) > millis() && PirTimestamp < millis())
    {
      Lcd.print(PirValue);
    }
    else
    {
      Lcd.print("N/A");
    }
  }
  else
  {
    Lcd.setCursor(0,0);
    Lcd.print("Not ready.");
  }
}

void ProcessEnterInitDevice(uint8_t * payload, uint8_t len)
{
  DEBUG_INTERFACE.println("Init Device State.\n");
  ModemState     = MODEM_STATE_INIT_DEVICE;
  AttentionState = false;

  Mesh_ResetRegisteredModelId();

  if (!Mesh_IsModelAvailable(payload, len, MESH_MODEL_ID_SENSOR_CLIENT))
  {
    DEBUG_INTERFACE.println("Modem does not support Sensor Client.\n");
    return;
  }

  uint8_t model_ids[] = 
  {
    lowByte (MESH_MODEL_ID_SENSOR_CLIENT),
    highByte(MESH_MODEL_ID_SENSOR_CLIENT),
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

void IndicateAttention(void)
{
  AttentionLedValue = AttentionState ? !AttentionLedValue : false;
  digitalWrite(PIN_LED, AttentionLedValue);
}

/********************************************
 * MAIN FUNCTION DEFINITIONS                *
 ********************************************/

void setup()
{
  SetupDebug();
  DEBUG_INTERFACE.println("Sensor Client Sample.\n");
  SetupLCD();
  SetupAttention();
  UART_Init();
  UART_SendSoftwareResetRequest();
}

void loop()
{
  UART_ProcessIncomingCommand();
  UpdateLCD();
}
