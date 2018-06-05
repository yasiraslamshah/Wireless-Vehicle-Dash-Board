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

#ifndef LETIMER_H
#define LETIMER_H

#include "em_cmu.h"      /* include statements */
#include "em_letimer.h"
#include "sleep.h"
#include "gpio.h"
#include "em_device.h"


#define DUTY_PERIOD (4) //TIME SECOND OF 2 SECONDS
#define LCOMP0 (0)
#define LCOMP1 (1)
#define EVENT1 (1)//SCHEDULE VARIABLE FLAGS
#define EVENT2 (2)//SCHEDULE VARIABLE FLAGS
#define POWER (0.80)//POR TIME OF 80mS
#define ULF ((uint16_t)0x3E8)
#define LCOUNT ((uint16_t)0x7FFF)
/*VARIABLES*/
extern volatile uint8_t schedule_event;
//uint8_t EVENT1,EVENT2;

struct gecko_msg_system_external_signal_evt_t *evt1;

/*FUNCTION */

void LETIMER0_Cal_Prescaler(void);

void LETIMER0_Set_Enable(void);
void LETIMER0_Set_Disable(void);
void LETIMER_init1(void);

void LETIMER0_IRQHandler(void);
#endif





