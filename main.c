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

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "ble-configuration.h"
#include "board_features.h"
/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
/* Device initialization header */
#include "hal-config.h"
#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};
// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;
//***********************************************************************************
// Include files
//***********************************************************************************
#include <stdint.h>
#include <stdbool.h>
//***********************************************************************************
// Include files
//***********************************************************************************

#include "main.h"
#include "gpio.h"
#include "sleep.h"
#include "letimer.h"
#include "cmu.h"
#include "I2C.h"
#include "spi.h"
#include "infrastructure.h"
#include "graphics.h"
#include "retargetserial.h"
#include "em_usart.h"


//***********************************************************************************
// defined files
//***********************************************************************************

#define TX_MAX		(80)
#define TX_MIN		(-260)
#define Connection_Interval		(75)
#define Slave_Interval 		(450)
#define Slave_Latency		((Slave_Interval/Connection_Interval)-1)
#define Connection_Value		(Connection_Interval/1.25)
#define Adv_Interval		(539)
#define Chanl3 (7)



//***********************************************************************************
// global variables
//***********************************************************************************
int16_t my_flag=0;
int16_t my_flag1=0;

//***********************************************************************************
// Persistent memory
//***********************************************************************************
struct temp{
	int v;
};

struct temp t;
//function to save in persistent memory
void save_Value_PM(int value)
{
    t.v = value;
    struct gecko_msg_flash_ps_save_rsp_t *save_flash_temp;
    save_flash_temp = gecko_cmd_flash_ps_save(0x4001,sizeof(t),(const uint8*)&t);
    //if(save_flash_temp->result == 0)
    //GPIO_PinOutSet(LED0_port, LED0_pin);
}
//function to load in persistent memory
int loadValue()
{
    struct gecko_msg_flash_ps_load_rsp_t * load_flash_temp;
    load_flash_temp = gecko_cmd_flash_ps_load(0x4001);
    uint8_t temp;
    if(load_flash_temp->result == 0)
    {
        temp = load_flash_temp->value.data[0];
        //GPIO_PinOutSet(LED0_port, LED0_pin);
    }
    return temp;
}

typedef struct{
int16_t humid;
}temp_flash;

//***********************************************************************************
// functions
//***********************************************************************************
//function to read Humidity from si7021
void HumidityMeasure()
  {
	uint32_t temper;
    int Data_Temperature=0;
    uint8_t htmTempBuffer[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
    uint8_t flags = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
    //uint32_t temperature;   /* Stores the temperature data read from the sensor in the correct format */
    uint8_t *s = htmTempBuffer; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
    int tempc=0;
    //I2C_Route_Init();
    GPIO_PinModeSet(gpioPortD,9,gpioModePushPull,1);
    while(schedule_event!=0x24);
    schedule_event=0;
    I2C_Init1();//INITAILSIE I2C
               //Read_temp();//READ TEMPERATURE
    			UINT8_TO_BITSTREAM(s,flags);
    			tempc=Read_Hum();
    			Data_Temperature=(((tempc)*21965L)>>13) - 46850;
    			temper=FLT_TO_UINT32(Data_Temperature,-3);
    			UINT32_TO_BITSTREAM(s, temper);
   Disable_Sensor();//DISABLE SENSOR
   //my_flag=1;
   save_Value_PM(RH_Hum_Data);
   gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Hum_value, 2,&RH_Hum_Data);
//   gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Hum_value, 5, htmTempBuffer);

   // gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_temperature_measurement, 5, htmTempBuffer);
  }

//function to read LUX values from ADPS
void temperatureMeasure()
  {
	char yasir;
	uint32_t temper1;
    int Data_Temperature1=0;
    uint8_t htmTempBuffer1[5]; /* Stores the temperature data in the Health Thermometer (HTM) format. */
    uint8_t flags1 = 0x00;   /* HTM flags set as 0 for Celsius, no time stamp and no temperature type. */
    //uint32_t temperature;   /* Stores the temperature data read from the sensor in the correct format */
    uint8_t *s1 = htmTempBuffer1; /* Pointer to HTM temperature buffer needed for converting values to bitstream. */
    int tempc1;
    //I2C_Route_Init();
    GPIO_PinModeSet(gpioPortD,9,gpioModePushPull,1);
    while(schedule_event!=0x24);
    schedule_event=0;
    I2C_Init1();//INITAILSIE I2C
               //Read_temp();//READ TEMPERATURE
    i2c_light_Setup();
    UINT8_TO_BITSTREAM(s1,flags1);
    			tempc1=Read_temp();

    			Data_Temperature1=(((tempc1)*21965L)>>13) - 46850;
    			temper1=FLT_TO_UINT32(Data_Temperature1,-3);
    			UINT32_TO_BITSTREAM(s1, temper1);
    			my_flag=0;
   Disable_Sensor();//DISABLE SENSOR
if(tempc2>50){yasir='A';}
else yasir='B';

   gecko_cmd_gatt_server_send_characteristic_notification(0xFF, gattdb_Lux_Value, 1,&yasir);
  }

//function to display on LCD
void display(char * string)
{
    GRAPHICS_Init();
    GRAPHICS_Clear();
    GRAPHICS_AppendString(string);
    GRAPHICS_Update();
}

//***********************************************************************************
// main
//***********************************************************************************

int main(void)
{
int y,x;

  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize stack
  /* Initialize GPIO */
  gpio_init();
  CMU_Setup();

  gecko_init(&config);
  LETIMER_init1();

  I2C_Route_Init();
  GRAPHICS_Init();
  GRAPHICS_Clear();
  //display("\n\n\nSERVER:");

  while (1) {
	  /* Event pointer for handling events */
	      struct gecko_cmd_packet* evt;

	      /* Check for stack event. */
	      evt = gecko_wait_event();//

	      /* Handle events */
	      switch (BGLIB_MSG_ID(evt->header)) {
	        /* This boot event is generated when the system boots up after reset.
	         * Do not call any stack commands before receiving the boot event.
	         * Here the system is set to start advertising immediately after boot procedure. */
	        case gecko_evt_system_boot_id:

	         //display persistent data and compare with new values and compare to display relative humdity
			 x=loadValue();
			 /*if(x>RH_Hum_Data){GRAPHICS_AppendString("\nRelatively Dry!\n");GRAPHICS_Update();}
			 else if(x<RH_Hum_Data){GRAPHICS_AppendString("\nRelatively Humid!\n");GRAPHICS_Update();}

			 /* delete all bondings to force the pairing process */
			 gecko_cmd_sm_delete_bondings();
			//initailise MITM to passkey match only

			 gecko_cmd_sm_configure(0x0F, sm_io_capability_displayyesno); /* Numeric comparison */

			 /* enable bondable to accommodate certain mobile OS */

			 gecko_cmd_sm_set_bondable_mode(1);
		     /* Set advertising parameters. 100ms advertisement interval. All channels used.
		      * The first two parameters are minimum and maximum advertising interval, both in
		      * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
		      gecko_cmd_le_gap_set_adv_parameters(Adv_Interval, Adv_Interval, Chanl3);
		     /* Start general advertising and enable connections. */
		     gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
		   break;

	       case gecko_evt_le_connection_closed_id:
	          /* Check if need to boot to dfu mode */
	          if (boot_to_dfu) {
	            /* Enter to DFU OTA mode */
	            gecko_cmd_system_reset(2);
	          } else {
	            /* Restart advertising after client has disconnected */
	            gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
	          }
	          break;

	        case gecko_evt_le_connection_opened_id:

	        	/* The HTM service typically indicates and indications cannot be given an encrypted property so

	        	           * force encryption immediately after connecting */
	        	BlueLE_Connect = evt->data.evt_le_connection_opened.connection;
	        	gecko_cmd_sm_increase_security(BlueLE_Connect);

	        break;
	        //display code on LCD
	       case gecko_evt_sm_passkey_display_id:
	    	display("\nPasskey display\n");
	    	char buffff[16] = {0};
	    	GRAPHICS_Init();
	    	GRAPHICS_Clear();
	    	snprintf(buffff,16,"\n code:%d\n",evt->data.evt_sm_passkey_display.passkey);
			GRAPHICS_AppendString(buffff);
			GRAPHICS_Update();
			break;

	        case gecko_evt_sm_confirm_passkey_id:

	        	GRAPHICS_Init();
	        	GRAPHICS_Clear();
	        	snprintf(buffff,16," CODE?: %d\n",evt->data.evt_sm_confirm_passkey.passkey);
	        	GRAPHICS_AppendString(buffff);
	        	GRAPHICS_Update();
	        break;

	        case gecko_evt_system_external_signal_id:
	        	I2C_Route_Init();
	        	if(my_flag==0)
	        	{
	        		HumidityMeasure();
                    my_flag=1;
	        	}
	        	else if(my_flag==1)
	        	{//I2C_Route_Init();
	        		temperatureMeasure();
	        		char bufff[16] = {0};
	        		GRAPHICS_AppendString("  \n\n\n\nSERVER");
	        		char buf[16] = {0};
	        		snprintf(buf,16,"\n\n\n\n\n\n\nHUM:%dRH\n",RH_Hum_Data);
					GRAPHICS_AppendString(buf);
					GRAPHICS_Update();
					snprintf(bufff,16,"\n\n\n\n\n\n\n\nLUX:%d\n",tempc2);
					GRAPHICS_AppendString(bufff);
					GRAPHICS_Update();
					if(x>RH_Hum_Data){GRAPHICS_AppendString("\n\nRelatively Dry!\n");GRAPHICS_Update();}
						        	 else if(x<RH_Hum_Data){GRAPHICS_AppendString("\n\nRelatively Humid!\n");GRAPHICS_Update();}
					GRAPHICS_Update();
					GRAPHICS_Clear();
					my_flag=0;
	        	}
	            break;

	        case gecko_evt_le_connection_rssi_id:

	        	gecko_cmd_system_halt(1);
	       	        if(evt->data.evt_le_connection_rssi.rssi > -35){
	       	        	gecko_cmd_system_set_tx_power(TX_MIN);
	       	        }
	       	        else if((evt->data.evt_le_connection_rssi.rssi > -45) && (evt->data.evt_le_connection_rssi.rssi > -35)){
	       	        	gecko_cmd_system_set_tx_power(-200);
	       	        }
	       	        else if((evt->data.evt_le_connection_rssi.rssi > -55) && (evt->data.evt_le_connection_rssi.rssi > -45)){
	       	       	        	gecko_cmd_system_set_tx_power(-150);
	       	       	        }
	       	        else if((evt->data.evt_le_connection_rssi.rssi > -65) && (evt->data.evt_le_connection_rssi.rssi > -55)){
	       	       	        	gecko_cmd_system_set_tx_power(-50);
	       	       	        }
	       	        else if((evt->data.evt_le_connection_rssi.rssi > -75) && (evt->data.evt_le_connection_rssi.rssi > -65)){
	       	       	        	gecko_cmd_system_set_tx_power(0);
	       	       	        }
	       	        else if((evt->data.evt_le_connection_rssi.rssi > -85) && (evt->data.evt_le_connection_rssi.rssi > -75)){
	       	       	        	gecko_cmd_system_set_tx_power(50);
	       	       	        }
	       	        else{
	       	       	       	    gecko_cmd_system_set_tx_power(TX_MAX);
	       	       	        }
	       	        gecko_cmd_system_halt(0);
	       	       	break;

	        /* Events related to OTA upgrading
	           ----------------------------------------------------------------------------- */

	        /* Check if the user-type OTA Control Characteristic was written.
	         * If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
	        case gecko_evt_gatt_server_user_write_request_id:

	          if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
	            /* Set flag to enter to OTA mode */
	            boot_to_dfu = 1;
	            /* Send response to Write Request */
	            gecko_cmd_gatt_server_send_user_write_response(
	              evt->data.evt_gatt_server_user_write_request.connection,
	              gattdb_ota_control,
	              bg_err_success);

	            /* Close connection to enter to DFU OTA mode */
	            gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
	          }
	          break;


	        default:
	        	break;
}
	      }
	    }

#if 0
Sleep();

//SCHEDULAR TO
	else if(schedule_event == EVENT2){

        GPIO_PinModeSet(gpioPortD, 9, gpioModePushPull, 1);//SENSOR ENABLE
        CORE_ATOMIC_IRQ_DISABLE();
        schedule_event = 0;
		  CORE_ATOMIC_IRQ_ENABLE();

	}

	else{

          I2C_Init1();//INITAILSIE I2C
           Read_temp();//READ TEMPERATURE
			Disable_Sensor();//DISABLE SENSOR
        CORE_ATOMIC_IRQ_DISABLE();
        schedule_event = 0;
		  CORE_ATOMIC_IRQ_ENABLE();
	}
#endif
