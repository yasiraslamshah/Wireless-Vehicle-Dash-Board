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

#ifndef SLEEP_H_
#define SLEEP_H_
#include "em_emu.h" 		// to include energy mgmt unit functions


#define ENERGY_MODE		(0)       //Macro for Minimum energy mode for the system

unsigned char sleep_block_counter[4];  //Global of array sleep block counter for each energy mode

typedef enum {          //Enum for Energy Modes
    EM0 = 0,
    EM1 = 1,
    EM2 = 2,
    EM3 = 3,
    EM4 = 4
}SleepState;

void UnblockSleepMode(SleepState minimumMode);	//Function to unblock sleep modes

void BlockSleepMode(SleepState minimumMode); //Function to block sleep modes

void Sleep(void);




#endif /* SLEEP_H_ */
