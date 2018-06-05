/***************************************************************************//**
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
 /***********************************************************************************
 Include files
***********************************************************************************/
#ifndef CMU_H_
#define CMU_H_

#include "main.h"
#include "em_cmu.h"
#include "sleep.h"
/***********************************************************************************
 function prototypes
***********************************************************************************/
//void cmu_init(void);/*FUNCTION TO INITALISE APPROPIATE CLOCK */

void CMU_Setup(void);/*FUNCTION TO SELECT APPROPIATE OSCILATOR AND CLOCK FOR ENERGY MODE 0,1,2,3*/

#endif /*CMU_H_*/




