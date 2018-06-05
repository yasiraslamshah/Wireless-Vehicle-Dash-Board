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


#include "sleep.h"
#include "em_core.h"

void unblockSleepMode(SleepState minimumMode)
{
    CORE_ATOMIC_IRQ_DISABLE();                            //Disabling interrupts to make the operation atomic
    if(sleep_block_counter[minimumMode] > 0)
    {
        sleep_block_counter[minimumMode]--;           //decreasing the energy mode or unblocking sleep mode
    }
    CORE_ATOMIC_IRQ_ENABLE();                             //Re-enabling interrupts
}

void BlockSleepMode(SleepState minimumMode)
{
	CORE_ATOMIC_IRQ_DISABLE();                            //Disabling interrupts to make the operation atomic
    sleep_block_counter[minimumMode]++;         //increasing the energy mode or blocking sleep mode.
    CORE_ATOMIC_IRQ_ENABLE();                              //Re-enabling interrupts
}
void Sleep(void)
{
	if(sleep_block_counter[0]>0){
		return;     //stay in EM0
	}
	else if(sleep_block_counter[1]>0){
		EMU_EnterEM1(); //EM2 is blocked, Go to EM1

	}
	else if (sleep_block_counter [2]>0){
		EMU_EnterEM2(true); //EM2 is blocked, Go to EM1
	}

else {
		EMU_EnterEM3(true); //Go to EM3 ,no EM4 is allowed
}return;
}
