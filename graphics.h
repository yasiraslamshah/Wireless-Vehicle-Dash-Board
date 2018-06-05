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
	 *//**************************************************************************//**
	 * @brief Draws the graphics on the display
	 * @version 3.20.5
	 *******************************************************************************
	 * @section License
	 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
	 *******************************************************************************
	 *
	 * This file is licensed under the Silabs License Agreement. See the file
	 * "Silabs_License_Agreement.txt" for details. Before using this software for
	 * any purpose, you must agree to the terms of that agreement.
	 *
	 ******************************************************************************/

	#ifndef __GRAPHICS_H
	#define __GRAPHICS_H

	#include <stdint.h>
	#include <stdbool.h>
	#include "glib.h"

	#ifdef __cplusplus
	extern "C" {
	#endif

	/*******************************************************************************
	 *****************************   PROTOTYPES   **********************************
	 ******************************************************************************/
	void GRAPHICS_Init(void);
	void GRAPHICS_Sleep(void);
	void GRAPHICS_Wakeup(void);
	void GRAPHICS_Update(void);
	void GRAPHICS_AppendString(char *str);
	void GRAPHICS_Clear(void);
	void GRAPHICS_InsertTriangle(uint32_t x,
								 uint32_t y,
								 uint32_t size,
								 bool up,
								 int8_t fillPercent);

	#ifdef __cplusplus
	}
	#endif


	#endif /* __GRAHPHICS_H */
