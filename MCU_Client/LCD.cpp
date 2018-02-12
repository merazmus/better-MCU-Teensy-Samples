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

#include "Arduino.h"
#include "Config.h"
#include "LiquidCrystal_I2C.h"
#include <limits.h>
#include <string.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define PB_REINIT PIN_SW_4 /**< Defines LCD reinit button */

/********************************************
 * STATIC VARIABLES                         *
 ********************************************/

static LiquidCrystal_I2C Lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
static char              LcdBuffer[LCD_ROWS][LCD_COLUMNS] = {0};
static unsigned long     LcdTimestamp                     = 0;
static volatile bool     UpToDate                         = false;
static volatile bool     LcdNeedReinit                    = false;

/********************************************
 * LOCAL FUNCTIONS PROTOTYPES               *
 ********************************************/

/*
 *  LCD reset button interrupt handler
 */
static void InterruptLcdPBClick(void);

/*
 *  Reinit LCD
 */
static void ReinitLCD(void);

/********************************************
 * EXPORTED FUNCTION DEFINITIONS            *
 ********************************************/

void SetupLCD(void)
{
  DEBUG_INTERFACE.println("LCD initialization.\n");
  Lcd.begin(LCD_COLUMNS, LCD_ROWS);
  pinMode(PB_REINIT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PB_REINIT), InterruptLcdPBClick, FALLING);
}

void ClearLCD(void)
{
  memset(LcdBuffer, 0, sizeof(LcdBuffer));
  UpToDate = false;
}

void WriteLineLCD(size_t line, char * text)
{
  if (strcmp(LcdBuffer[line], text) != 0)
  {
    if (strlen(text) > LCD_COLUMNS)
    {
      DEBUG_INTERFACE.printf("Trying to write too long string on LCD: %s", text);
      return;
    }
    strcpy(LcdBuffer[line], text);
    UpToDate = false;
  }
}

void LoopLCD(void)
{
  if (!UpToDate && (LcdTimestamp + LCD_UPDATE_INTV) < millis())
  {
    Lcd.clear();
    for (int row = 0; row < LCD_ROWS; row++)
    {
      Lcd.setCursor(0, row);
      Lcd.print(LcdBuffer[row]);
    }
    LcdTimestamp = millis();
    UpToDate     = true;
  }

  if (LcdNeedReinit)
  {
    ReinitLCD();
    LcdNeedReinit = false;
    UpToDate      = false;
  }
}

/********************************************
 * LOCAL FUNCTION DEFINITIONS               *
 *******************************************/

static void InterruptLcdPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);
  if (digitalRead(PB_REINIT)) return;

  LcdNeedReinit = true;
}

static void ReinitLCD(void)
{
  Lcd.begin(LCD_COLUMNS, LCD_ROWS);
}
