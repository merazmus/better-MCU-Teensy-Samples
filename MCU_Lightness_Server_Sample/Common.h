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

#ifndef COMMON_H_
#define COMMON_H_

/********************************************
 * EXPORTED #define CONSTANTS AND MACROS    *
 ********************************************/

/** Debug definition. */
#define DEBUG_INTERFACE             (Serial)  /**< Defines serial port to print debug messages */
#define DEBUG_INTERFACE_BAUDRATE    115200    /**< Defines baudrate of debug interface */

/** Pinout definitions. */
#define PIN_LED                     13        /**< Defines led pin */
#define PIN_PWM                     16        /**< Defines pwm pin */
#define PIN_PB_0                    22        /**< Defines button 0 pin */
#define PIN_PB_1                    21        /**< Defines button 1 pin */
#define PIN_PB_2                    20        /**< Defines button 2 pin */
#define PIN_PB_3                    15        /**< Defines button 3 pin */
#define PIN_PB_4                    14        /**< Defines button 4 pin */
#define PIN_PB_5                    4         /**< Defines button 5 pin */
#define PIN_PB_6                    6         /**< Defines button 6 pin */
#define PIN_ANALOG                  9         /**< Defines analog measurement pin */

/** Analog range definitions */
#define ANALOG_MIN                  0         /**< Defines lower range of analog measurements */
#define ANALOG_MAX                  1023      /**< Defines uppper range of analog measurements */

/** Debounce time delay definition */
#define BUTTON_DEBOUNCE_TIME_MS     20        /**< Defines buttons debounce time in miliseconds */
#define ENCODER_DEBOUNCE_TIME_US    300       /**< Defines encoder debounce time in microseconds */
#define ATTENTION_TIME_US           500000    /**< Defines attention state change time in microseconds. */

/** @brief Counts number of elements inside the array. */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

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

#endif // COMMON_H_
