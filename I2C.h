/*/***************************************************************************//**
 * @Yasir Aslam Shah
 * @IOT 2018 Spring
 *
 *******************************************************************************
 * # License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 */
#ifndef I2C_H
#define I2C_H

#include "em_I2C.h"
#include "I2C.h"
#include "letimer.h"
#include "gpio.h"
#include "cmu.h"
#include "em_core.h"
#include "sleep.h"
#include "spi.h"
#include "em_letimer.h"
#include <stdbool.h>

#define DEF_TEMP    30

#define I2C_SlaveAddr 0x40
#define I2C_writeBit 0x00
#define I2C_ReadBit 0x01
#define I2C_DeviceID 0xE3


extern uint16_t RH_Hum_Data;//degree variable

extern float Sensor_Lux;
extern int tempc2;

void I2C_Init1(void);
void I2C_Route_Init(void);
void I2C_gpio(void);

void Disable_Sensor(void);
uint16_t Read_Hum(void);


void i2c_Reg(uint8_t ,uint8_t);
void i2c_light_Setup();
int Read_temp(void);
uint8_t adc_values(uint8_t);

#endif /* SRC_I2C_H_ */
