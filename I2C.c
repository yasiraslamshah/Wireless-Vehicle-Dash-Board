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

#define I2C_SlaveAddr 0x40
#define I2C_writeBit 0x00
#define I2C_ReadBit 0x01
#define I2C_DeviceID 0xE3
#define Threshold 15
uint16_t RH_Hum_Data = 0;//degree variable

#define I2C1_IFC_ACK 0x40
#define I2C1_IFC_NACK 0x80
#define I2C1_IEN_ACK 0x40
#define I2C1_IEN_NACK 0x80
#define slave_addr 0x39
#define Write 0x0
#define Read 0x1
#define THLow 0x000f
#define Word_mode 0x80
#define INT_PIN 0x0200
#define Power_Pin  0
#define Interrupt_Pin  1
#define I2C_CTRL 0x0
#define I2C_INT  0x06
#define ADC0_LOW 0x0C
#define ADC0_HIGH 0x0D
#define ADC1_LOW 0x0E
#define ADC1_HIGH 0x0F
#define I2C_TIMING 0x01
#define Power_up 0x03
#define TIMING_Value 0x01
float Sensor_Lux;
int tempc2;
//setting I2C
	void I2C_Init1(){
		//CMU_ClockEnable(cmuClock_HFPER, true);//enable Clock tree
		//CMU_ClockEnable(cmuClock_I2C0, true);//enable Clock tree
		//I2C_Route();//Routing the pins
		/*NVIC_ClearPendingIRQ(I2C0_IRQn);
		NVIC_EnableIRQ(I2C0_IRQn); // Enables interrupt*/
		//I2C_Abort();
		I2C_gpio();//specify gpio pins for SCL and SDA
		//resetting I2C
		for(uint8_t p=0;p<9;p++){
		GPIO_PinOutClear(I2C0_SCL_Port,I2C0_SCL_Pin);
		GPIO_PinOutSet(I2C0_SCL_Port,I2C0_SCL_Pin);
		}
        I2C0->IFC=0X07FF;
		I2C0->IEN |= (I2C_IEN_ACK)|(I2C_IEN_RXDATAV);
        I2C_Enable(I2C0, true);
	}


	//-----------------------------------------------------------
	//function to route the SCl and SDA pins
	void I2C_Route_Init(void){
		//GPIO_PinOutSet(gpioPortF,5);
	    CMU_ClockEnable(cmuClock_HFPER, true);//enable Clock tree
		CMU_ClockEnable(cmuClock_I2C0, true);
		//routing SCl and SDA
        I2C0->ROUTEPEN = I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN;
		I2C0->ROUTELOC0 = _I2C_ROUTELOC0_RESETVALUE;
		I2C0->ROUTELOC0 |= I2C_ROUTELOC0_SCLLOC_LOC14 | I2C_ROUTELOC0_SDALOC_LOC16;

		I2C_Init_TypeDef temperature_init = I2C_INIT_DEFAULT;//
        I2C_Init(I2C0, &temperature_init);
        //abort
        if(I2C0->STATE & I2C_STATE_BUSY){
			I2C0->CMD = I2C_CMD_ABORT;
		}
		//GPIO_PinOutClear(gpioPortF,5);
	}

	//-----------------------------------------------------------
	//function to initialise I2C gpio pins
	void I2C_gpio(void){
	GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeWiredAnd, 1);//gpio pin for SCL
	GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeWiredAnd, 1);//gpio pin for SDA
	}
	void Disable_Sensor(void){
        GPIO_PinModeSet(I2C0_SCL_Port, I2C0_SCL_Pin, gpioModeDisabled, 0);
        GPIO_PinModeSet(I2C0_SDA_Port, I2C0_SDA_Pin, gpioModeDisabled, 0);

        GPIO_PinOutClear(gpioPortD, 9);
	}
	//Function for I2C Start

//Function to read humidity
	uint16_t Read_Hum(void){
		uint8_t SlaveAddr=I2C_SlaveAddr;//variable for slave address
		uint16_t MSB_Hum_Data=0;//MSB bit
		uint16_t LSB_Hum_Data=0;//MSB bit
      //int16_t Temp_Celsius_Data = 0;//degree variable
        uint16_t Hum_Data = 0;
//Funtion to abort the previous state of I2C
        if(I2C0->STATE & I2C_STATE_BUSY){
		I2C0->CMD = I2C_CMD_ABORT;}

	I2C0->TXDATA = ((SlaveAddr << 1)|(I2C_writeBit));//writing slave address and write bit
	I2C0->CMD = I2C_CMD_START;//starting I2C
	while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ack
	I2C0->IFC = I2C_IFC_ACK;//clear the ack pin

	//I2C0->TXDATA = I2C_DeviceID;//send the temp sensor sleave address of 0xE3 for hold master mode

	I2C0->TXDATA = 0XE5;//send the temp sensor sleave address of 0xE3 for hold master mode
	while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ACK
	I2C0->IFC = I2C_IFC_ACK;//clear the ack pin

	I2C0->CMD = I2C_CMD_START;//RESTART THE I2C
	I2C0->TXDATA = ((SlaveAddr << 1)|(I2C_ReadBit));//SEND SLAVE ADDRESS WITH WRITE BIT
	while((I2C0->IF & I2C_IF_ACK) == 0);//WAIT FOR ACK
	I2C0->IFC = I2C_IFC_ACK;//CLEAR THE ACK PIN

	while( ((I2C0->STATUS) & I2C_STATUS_RXDATAV)==0 );//CHECK FOR RX BUFFER
	MSB_Hum_Data = I2C0->RXDATA;//STORE MSB IN MSB_TEMP_DATA
	I2C0->CMD = I2C_CMD_ACK;//SEND ACK
	LSB_Hum_Data = I2C0->RXDATA;//STORE LSB IN LSB_TEMP_DATA

	I2C0->CMD = I2C_CMD_NACK;//SEND NACK
	I2C0->CMD = I2C_CMD_STOP;//SEND STOP

	Hum_Data = ((MSB_Hum_Data<<8)|(LSB_Hum_Data & 0xFE));//SAVE BOTH LSB AND MSB AND CLEAR LAST BIT
//	Temp_Celsius_Data = (int16_t)((((17572*(Temp_Data))>>16)-4685)/100);//FORMULA TO CALCULATE CELSIUS TEMP
	RH_Hum_Data = (int16_t)(((125*(Hum_Data))>>16)-6);//FORMULA TO CALCULATE CELSIUS TEMP

	if(RH_Hum_Data < 30){
		GPIO_PinOutSet(gpioPortF, 5);//TURN LED OFF AF TEMP GREATER THAN 15
		//GPIO_PinOutClear(gpioPortF, 4);
		//GPIO_PinOutClear(gpioPortF, 4);//TURN LED OFF AF TEMP GREATER THAN 15
	}
	else if(RH_Hum_Data > 30)
	{
		GPIO_PinOutClear(gpioPortF, 5);
		//GPIO_PinOutSet(gpioPortF, 4);//TURN LED ON FOR TEMP LESS THAN 15
		//GPIO_PinOutClear(gpioPortF, 4);//TURN LED ON FOR TEMP LESS THAN 15
	}
	return Hum_Data ;
	}



	//light sensor functions
	//FUNCTION TO WRITE TO I2C
	//called from light init
	void i2c_Reg(uint8_t addr_reg,uint8_t reg_value) //ADDR AND REG_VALUE ARE TWO VARAIBLES AS ARGUMENTS
	{
		uint8_t Addr;
			uint8_t value_reg = (0x80|addr_reg);//0X80 IS COMMAND ORED WITH ADDRESS VARAIBLE 80
			Addr =(slave_addr<<1)|Write;//SLAVE ADDRESS
			I2C0->TXDATA=Addr;//SEND SLAVE ADDRESS|WRITE BIT

			I2C0->CMD = I2C_CMD_START;//START
	//		I2C0->IFC = I2C_IFC_START;//CLEAR START IF
	//		I2C0->IFC = I2C_IFC_ACK;//clear the ack pin

			//WAIT FOR ACK
			while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ACK
			I2C0->IFC = I2C_IFC_ACK;//clear the ack pin

			I2C0->TXDATA = value_reg;//SEND COMMAND 80
			//WAIT FOR ACK
			while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ACK
			I2C0->IFC = I2C_IFC_ACK;//clear the ack pin

			I2C0->TXDATA = reg_value;//SEND DATA COMMAND
			//WAIT FOR ACK
			while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ACK
					    			I2C0->IFC = I2C_IFC_ACK;//clear the ack pin
			//STOP COMMAND
			I2C0->CMD = I2C_CMD_STOP;
			while((I2C0->IF & I2C_IF_MSTOP)==0);
			I2C0->IFC = I2C_IFC_MSTOP;//CLEAR STOP IF FLAG
	}



	//called from main()
	void i2c_light_Setup()
	{
	   	i2c_Reg(0x80,Power_up);//FUNCTION TO POWER UP 0X00|0X03
		i2c_Reg(I2C_TIMING,TIMING_Value);//0X01|0X01
	}

		//Function for I2C Start
		//read by temperature measure
		//Function to read light sensor values
	int Read_temp(void){
		uint8_t ch0_lsb;
		float ch1_ch0;
		uint8_t ch0_msb = adc_values(ADC0_HIGH);
		uint8_t ch1_lsb = adc_values(ADC1_LOW);
		uint8_t ch1_msb = adc_values(ADC1_HIGH);

		double yel;
		float exp;
		exp=1.4;
		ch0_lsb = adc_values(ADC0_LOW);

		uint8_t ch0_lux_value= ch0_msb<<8 | ch0_lsb;//SAVE SENSOR VALUE IN LUX AS MSB AND LSB
		uint8_t ch1_lux_value= ch1_msb<<8 | ch1_lsb;//SAVE SENSOR VALUE IN LUX AS MSB AND LSB

		ch1_ch0= ((float)ch1_lux_value/(float)ch0_lux_value);

		yel=pow((double) ch1_ch0, (double) exp);

		if((ch1_ch0 >=0.0) && (ch1_ch0<0.50))
			{Sensor_Lux= (  (((304.0)*ch0_lux_value) /(10000.0)) - (((62.0)*ch0_lux_value)/(1000.0)) * (yel) );}

			else if((ch1_ch0 >=(0.50)) && (ch1_ch0 <(0.61)))
			{Sensor_Lux=  (((0224*ch0_lux_value)/10000)- ((031*ch1_lux_value)/1000));}

			else if((ch1_ch0 >=(0.61)) && (ch1_ch0 <(0.80))){
			Sensor_Lux=  (((128*ch0_lux_value)/10000) - ((0153*ch1_lux_value)/10000));}

			else if((ch1_ch0 >=(0.80)) && (ch1_ch0 <(1.30))){
			Sensor_Lux= (((146*ch0_lux_value)/100000) - ((112*ch1_lux_value)/100000));}

			else if((ch1_ch0 >=(1.30))){
			Sensor_Lux=  ((ch1_ch0)*19)/10;}

			tempc2=(100.0)*Sensor_Lux;
			if(tempc2<50.00)//COMPARE LUX VALUE
				GPIO_PinOutSet(gpioPortF,4);//FINDOUT
			else
				GPIO_PinOutClear(gpioPortF,4);//FINDOUT
			return tempc2;
	}


	//read by read temp
	//adc values to communicate using i2c for ligyt sensor
	uint8_t adc_values(uint8_t abc)//ADDR AND REG_VALUE ARE TWO VARAIBLES AS ARGUMENTS
	{
			uint8_t valueadc = (Word_mode|abc);//0X80 IS COMMAND ORED WITH ADDRESS VARAIBLE
			I2C0->TXDATA=((slave_addr<<1)|Write);//write slave address of 0x39 and write bit
			I2C0->CMD =I2C_CMD_START;//INITAILISE START COMMAND
			while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ACK
						I2C0->IFC = I2C_IFC_ACK;//clear the ack pin
			I2C0->TXDATA= valueadc;  //adc low/high/cho/ch1
			while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ACK
						I2C0->IFC = I2C_IFC_ACK;//clear the ack pin
			I2C0->CMD =I2C_CMD_START;//INITAILISE START COMMAND
			I2C0->TXDATA=((slave_addr<<1)|Read);//write slave address of 0x39 and READ bit
			while((I2C0->IF & I2C_IF_ACK) == 0);//wait for ACK
						I2C0->IFC = I2C_IFC_ACK;//clear the ack pin
			//RECEIVE
						while( ((I2C0->STATUS) & I2C_STATUS_RXDATAV)==0 );//CHECK FOR RX BUFFER

						//while(!(I2C0->IF & I2C_IF_RXDATAV));//WAIT FOR RECIEVE DATA
			uint8_t tempadc = I2C0->RXDATA;//SAVE DATA IN temp0
			I2C0->CMD = I2C_CMD_NACK;//SEND NACK FROM MASTER
			I2C0->CMD = I2C_CMD_STOP;//ISSUE STOP COMMAND
			return tempadc;
			}
