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

#ifndef CONFIG_H_
#define CONFIG_H_

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

#define INSTANCE_INDEX_UNKNOWN      UINT8_MAX /**< Defines unknown instance index value. */
#define ANALOG_MIN                  0         /**< Defines lower range of analog measurements. */
#define ANALOG_MAX                  1023      /**< Defines uppper range of analog measurements. */

#define DEBUG_INTERFACE             (Serial)  /**< Defines serial port to print debug messages. */
#define DEBUG_INTERFACE_BAUDRATE    115200    /**< Defines baudrate of debug interface. */
#define UART_INTERFACE              (Serial2) /**< Defines serial port to communicate with modem. */
#define UART_INTERFACE_BAUDRATE     57600     /**< Defines baudrate of modem interface. */

#define PIN_LED                     13  /**< Defines led pin. */
#define PIN_PWM                     16  /**< Defines pwm pin. */
#define PIN_SW_1                    22  /**< Defines switch 1 pin. */
#define PIN_SW_2                    21  /**< Defines switch 2 pin. */
#define PIN_SW_3                    20  /**< Defines switch 3 pin. */
#define PIN_SW_4                    15  /**< Defines switch 4 pin. */
#define PIN_ENCODER_SW              14  /**< Defines encoder switch pin. */
#define PIN_ENCODER_A               6   /**< Defines encoder A pin. */
#define PIN_ENCODER_B               4   /**< Defines encoder B pin. */
#define PIN_ANALOG                  9   /**< Defines analog measurement pin. */

#define BUTTON_DEBOUNCE_TIME_MS     20      /**< Defines buttons debounce time in miliseconds. */
#define ENCODER_DEBOUNCE_TIME_US    300     /**< Defines encoder debounce time in microseconds. */
#define ATTENTION_TIME_US           500000  /**< Defines attention state change time in microseconds. */
#define DATA_VALIDITY_PERIOD_MS     3000    /**< Defines sensor data validity period. */

#define LCD_ROWS                    4                           /**< Defines number of LCD rows. */
#define LCD_COLUMNS                 20                          /**< Defines number of LCD columns. */
#define LCD_UPDATE_INTV             500                         /**< Defines LCD update interval. */
#define LCD_DEVICE_STATE_LINE       0                           /**< Defines LCD line content. */
#define LCD_VERSION_LINE            (LCD_DEVICE_STATE_LINE + 1) /**< Defines LCD line content. */
#define LCD_SENSOR_PIR_LINE         (LCD_VERSION_LINE + 1)      /**< Defines LCD line content. */
#define LCD_SENSOR_ALS_LINE         (LCD_SENSOR_PIR_LINE + 1)   /**< Defines LCD line content. */

#endif // CONFIG_H_
