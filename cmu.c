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
 /******************************************************************************/
//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"
#include "sleep.h"
#include "em_cmu.h"

/*FUNCTION TO INITALISE APPROPIATE CLOCK */
/*FUNCTION TO SELECT APPROPIATE OSCILATOR AND CLOCK FOR ENERGY MODE 0,1,2,3*/
void CMU_Setup(void){
	if(ENERGY_MODE==EM3){
	    	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO); //enable ultra Low Frequency RC oscillator
	    	CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);//ENABLE ULTRA LOW FREQUENCY OSCILLATOR
	    }
	    else{
	    	CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
	    	CMU_OscillatorEnable(cmuOsc_LFXO,true,true); //enable Low Frequency External oscillator
	    	//CMU_ClockDivSet(cmuClock_LETIMER0,2);//divide by two(Prescaled)
	    }

    CMU_ClockEnable(cmuClock_LETIMER0, true);//ENABLING LETIMER0 CLOCK
    CMU_ClockEnable(cmuClock_GPIO, true);//ENABLING GPIO CLOCK
    CMU_ClockEnable(cmuClock_HFLE, true);//ENABLING HFLE CLOCK
}









