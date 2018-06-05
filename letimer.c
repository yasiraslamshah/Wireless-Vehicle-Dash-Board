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
#include "letimer.h"
#include "gpio.h"
#include "cmu.h"
#include "em_core.h"
#include "sleep.h"
#include "em_letimer.h"
#include "native_gecko.h"

#include "main.h"
void LETIMER_init1(void){


	//CMU_Setup();
	LETIMER_Init_TypeDef letimerInit =               //configuration for letimer0
	    {
	    	.enable         = true,                 //Starts to count after init is completed
	        .debugRun       = false,                 //Counter would not run while its stopped for debug
	        .comp0Top       = true,                 //COMP0 is used as TOP and loaded into CNT after counter underflow
	        .bufTop         = false,                //it will not load COMP1 into COMP0 when REP0 is 0
	        .out0Pol        = 0,                    //output 0 idle value.
	        .out1Pol        = 0,                    //output 1 idle value.
	        .ufoa0          = letimerUFOANone,      //PWM output is on output 0
	        .ufoa1          = letimerUFOANone,      //Pulse output is on output 1
	        .repMode        = letimerRepeatFree     //keep on Counting until stopped
	    };

    LETIMER0_Cal_Prescaler();



	uint32_t val_1;
	uint32_t val_2;
	uint32_t pres = CMU->LFAPRESC0;
	uint32_t pres_1 = 1;

if (ENERGY_MODE == 3)

	{val_1 = (((ULF)/(pres_1 << pres))*(DUTY_PERIOD));//VALUE FOR COMP0 FOR ENERGY MODE 3
	val_2 = val_1-(((ULF)/(pres_1 << pres))*(POWER));}//VALUE FOR COMP1 FOR ENERGY MODE 3


else
    {val_1 = (((LCOUNT)/(pres_1 << pres))*(DUTY_PERIOD));//VALUE FOR COMP0 FOR ENERGY MODE 0-2
	val_2 = val_1-(((LCOUNT)/(pres_1 << pres))*(POWER));}//VALUE FOR COMP0 FOR ENERGY MODE 0-2
//changes    val_2 = val_1-(((LCOUNT)/(pres_1 << pres))*(POWER));}//VALUE FOR COMP0 FOR ENERGY MODE 0-2
	LETIMER0->COMP0 = val_1;//SETTING VALUE OF COMP0 AS val_1
	LETIMER0->COMP1 = val_2;//SETTING VALUE OF COMP1 AS val_2

	while((LETIMER0->SYNCBUSY));

	NVIC_ClearPendingIRQ(LETIMER0_IRQn);//CLEAR ALL NVIC PENDING INTERUPPTS FOR LETIMER0


	LETIMER0->IEN &= ~((LETIMER_IEN_COMP0)|(LETIMER_IEN_COMP1));//DISABLING INTERUPT FOR COMP0 AND COMP1
	LETIMER0->IEN |= (LETIMER_IEN_COMP0);//ENABLING INTERUPT FOR COMP0
	LETIMER0->IEN |= (LETIMER_IEN_COMP1);//ENABLING INTERUPT FOR COMP1


	NVIC_EnableIRQ(LETIMER0_IRQn);//ENABLING NVIC INTERRUPTS FOR LETIMER0



    LETIMER_Init(LETIMER0, &letimerInit);
    LETIMER0->CMD = LETIMER_CMD_START;//START LETIMER0

}

volatile uint8_t schedule_event=0;//VARIABLE FOR SCHEDULING EVENTS

//PRESCALOR function TO CALCULATE THE VALYE OF PRESCALOR FOR ANY GIVEN FREQUENCY FOR ANY ENERGY MODE
void LETIMER0_Cal_Prescaler(void){
	uint8_t base =0;
	uint16_t expo =1;

	while(expo< DUTY_PERIOD){
		expo = expo*2;
		base++;
	}
	CMU->LFAPRESC0 = base + 1;//ADDING PRESCALOR VALE TO LFAPRESC0 REGISTER
}


/*FUNCTION TO START LETIMER*/
void LETIMER0_Set_Enable(void){
	LETIMER0->CMD = ((LETIMER0->CMD) | (LETIMER_CMD_START));
}

/*FUNCTION TO STOP LETIMER*/
void LETIMER0_Set_Disable(void){
	LETIMER0->CMD = ((LETIMER0->CMD) | (LETIMER_CMD_STOP));
}

/*LETIMER IRQ HANDLER*/
void LETIMER0_IRQHandler(void){

	uint32_t intFlags;		// get interrupt flags
	CORE_ATOMIC_IRQ_DISABLE();                       //make the operation atomic
	intFlags = LETIMER0->IF;//INTERUPTS FLAGS
	LETIMER0->IFC= intFlags;

	if(intFlags & LETIMER_IF_COMP0){        		//COMP0 FLAG is set after 2000ms
	gecko_external_signal(1);
	//evt1->extsignals = 1;
	LETIMER0->IFC|=(LETIMER_IFC_COMP0);//CLEAR THE COMP0 INTERRUPT
	}

	if(intFlags & LETIMER_IF_COMP1){        		//COMP1 FLAG is set
	schedule_event=0x24;
	LETIMER0->IFC|=(LETIMER_IFC_COMP1);//CLEAR THE COMP1 INTERRUPT
	}
	gecko_cmd_le_connection_get_rssi(BlueLE_Connect);

	CORE_ATOMIC_IRQ_ENABLE();                         //Re-enabling interrupts
}

