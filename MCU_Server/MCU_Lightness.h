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

#ifndef MCU_LIGHTNESS_SERVER_H
#define MCU_LIGHTNESS_SERVER_H

/*******************************************
* INCLUDES                                 *
********************************************/

#include <stdint.h>

/********************************************
 * EXPORTED #define CONSTANTS AND MACROS    *
 ********************************************/

#define LIGHTNESS_MIN                            0            /**< Defines lower range of light lightness */
#define LIGHTNESS_MAX                            UINT16_MAX   /**< Defines upper range of light lightness */

/********************************************
 * FUNCTION PROTOTYPES                      *
 ********************************************/

/*
 *  Set Lightness LC Server instance index
 *
 *  @param idx  Lightness value
 */
void SetLightnessServerIdx(uint8_t idx);

/*
 *  Get Lightness LC Server instance index
 *
 *  @return     Lightness value
 */
uint8_t GetLightnessServerIdx(void);

/*
 *  Process new target lightness
 *
 *  @param val                 Lightness value
 *  @param transition_time     Transition time
 */
void ProcessTargetLightness(uint16_t val, uint32_t transition_time);

/*
 *  Setup light lightness server hardware
 */
void SetupLightnessServer(void);

/*
 *  Lightness server main function, should be called in Arduino main loop
 */
void LoopLightnessServer(void);

/*
 *  Indicate attention using lightness output
 *
 *  @param attention_state     Attention state
 *  @param led_state           Attention led state
 */
void IndicateAttentionLightness(bool attention_state, bool led_state);


#endif  // MCU_LIGHTNESS_SERVER_H
