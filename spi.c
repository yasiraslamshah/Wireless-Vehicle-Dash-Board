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

#include "em_i2c.h"
#include "I2C.h"
#include "letimer.h"
#include "gpio.h"
#include "cmu.h"
#include "em_core.h"
#include "sleep.h"
#include "em_letimer.h"
#include "em_usart.h"
#include "spi.h"

//	#include "em_uart.h"

uint8_t Accel_X = 0xAA;
uint8_t Accel_Y = 0xAA;
uint8_t Accel_Z = 0xAA;
uint8_t move;

//initailise spi for BMA 280acceleerometer
void SPI_init()
{
  // EMLIB documentation online:
  // PIN PC9 = SPI_CS
  GPIO_PinModeSet(gpioPortC, 9, gpioModePushPull, 1);
  // PIN PC8 = SPI_SCK
  GPIO_PinModeSet(gpioPortC, 8, gpioModePushPull, 1);
  // PIN PC7 = SPI_MISO
  GPIO_PinModeSet(gpioPortC, 7, gpioModeInput, 0);
  // PIN PC6 = SPI_MOSI
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 1);
  USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;

  CMU_ClockEnable(cmuClock_HFPER, true);		// Enable HF peripheral clock
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  config.enable = usartDisable;
  config.baudrate = 100000; /* 100 kbits/s. */
  config.clockMode = usartClockMode3;   /* Clock idle high, sample on rising edge. */
  config.msbf = true; /* Send most significant bit first. */

  config.autoCsEnable = true;

  USART_InitSync(USART1, &config);


  USART1->ROUTELOC0 = USART_ROUTELOC0_CLKLOC_LOC11 | USART_ROUTELOC0_TXLOC_LOC11 | USART_ROUTELOC0_RXLOC_LOC11 | USART_ROUTELOC0_CSLOC_LOC11;
  USART1->ROUTEPEN = USART_ROUTEPEN_CLKPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_CSPEN;

  USART_Enable(USART1, usartEnable);

}


uint8_t SPI_read(USART_TypeDef *usart, uint8_t offset)
{
	uint16_t data;
	uint32_t tmp;
	data = 0x0080; // note: byte order when sending is 0x80 0x00 . MSB is set to indicate read for BMA280 chip
	data = data | offset;  // set offset field

  while (!(usart->STATUS & USART_STATUS_TXBL))
    ;
  usart->TXDOUBLE = (uint32_t)data;
  while (!(usart->STATUS & USART_STATUS_TXC))
    ;
  tmp = usart->RXDOUBLE;
  tmp = tmp >> 8;
  return (uint8_t)tmp;
}


void SPI_accel()
{
	GPIO_PinOutSet(LED2_port,LED2_pin);
/*
	Accel_X = 0xAA;
	Accel_Y = 0xAA;
	Accel_Z = 0xAA;*/

	Accel_X = SPI_read(USART1, 0x03);
	//gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_X_Axis_Measured_Value, 1, &accelerationx);

	Accel_Y = SPI_read(USART1, 0x05);
	//gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Y_Axis_Measured_Value, 1, &accelerationy);

	Accel_Z = SPI_read(USART1, 0x07);
	//gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Z_Axis_Measured_Value, 1, &accelerationz);

	//printf("\nACC X%d, Y%d, Z%d", accelerationx,accelerationy,accelerationz);

	GPIO_PinOutClear(LED2_port,LED2_pin);
	//my_flag=0;
	if(Accel_Y>190 && Accel_Y<250){move=1;}
	if(Accel_Y>6 && Accel_Y<64){move=0;}

	// gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Move, 1, &mov);
}

//function to print Accelerometer values on LCD
void SPI_accel_print()
{
				char buf1[16] = {0};

				GRAPHICS_AppendString("\n    XX:YY:ZZ");
	        	snprintf(buf1,16,"\n    %d %d %d",Accel_X,Accel_Y,Accel_Z);
	        	GRAPHICS_AppendString(buf1);
	        	GRAPHICS_Update();

	        	if(Accel_X>190 && Accel_X<250)
	        	{GRAPHICS_AppendString("\n     LEFT!");
	        		GRAPHICS_Update();}
	        	if(Accel_X>6 && Accel_X<64)
	        	{GRAPHICS_AppendString("\n     RIGHTT!");
	        	    GRAPHICS_Update();}

	        	if(Accel_Y>190 && Accel_Y<250)
	        		        	{GRAPHICS_AppendString("\n     UP!");
	        		        	GRAPHICS_Update();}
	        	if(Accel_Y>6 && Accel_Y<64)
	        		        	{GRAPHICS_AppendString("\n     DOWN!");
	        		        	GRAPHICS_Update();}

	        	if((Accel_X>-1 && Accel_X<5)&&(Accel_Y>0 && Accel_Y<6)&&(Accel_Z>60 && Accel_Z<70))
								{GRAPHICS_AppendString("\n    !Stable!");
								GRAPHICS_Update();}
}
