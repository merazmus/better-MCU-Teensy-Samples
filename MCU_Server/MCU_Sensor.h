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

#ifndef MCU_SENSOR_SERVER_H
#define MCU_SENSOR_SERVER_H

/********************************************
* INCLUDES                                 *
********************************************/

#include <stdint.h>

/********************************************
 * LOCAL #define CONSTANTS AND MACROS       *
 ********************************************/

/*
 *  Sensor properties
 */
#define PIR_POSITIVE_TOLERANCE      0x0000
#define PIR_NEGATIVE_TOLERANCE      0x0000
#define PIR_SAMPLING_FUNCTION       0x01
#define PIR_MEASUREMENT_PERIOD      0x40
#define PIR_UPDATE_INTERVAL         0x40
#define ALS_POSITIVE_TOLERANCE      0x0000
#define ALS_NEGATIVE_TOLERANCE      0x0000
#define ALS_SAMPLING_FUNCTION       0x01
#define ALS_MEASUREMENT_PERIOD      0x40
#define ALS_UPDATE_INTERVAL         0x40

/********************************************
 * FUNCTION PROTOTYPES                      *
 ********************************************/

/*
 *  Sensor Server ALS instance index setter
 *
 *  @param idx  Instance index
 */
void SetSensorServerALSIdx(uint8_t idx);

/*
 *  Sensor Server ALS instance index getter
 *
 *  @return  Instance index
 */
uint8_t GetSensorServerALSIdx(void);

/*
 *  Sensor Server PIR instance index setter
 *
 *  @param idx  Instance index
 */
void SetSensorServerPIRIdx(uint8_t idx);

/*
 *  Sensor Server PIR instance index getter
 *
 *  @return  Instance index
 */
uint8_t GetSensorServerPIRIdx(void);

/*
 *  Setup sensor server hardware
 */
void SetupSensorServer(void);

/*
 *  Sensor server main function, should be called in Arduino main loop
 */
void LoopSensorSever(void);

#endif  // MCU_SENSOR_SERVER_H
