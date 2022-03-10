/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "sdk_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_uart.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "app_error.h"
#include <string.h>
#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "uart.h"
	
//-----------------dw1000----------------------------

static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
#define PRE_TIMEOUT 1000

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100 

/*Should be accurately calculated during calibration*/
#define TX_ANT_DLY 16300
#define RX_ANT_DLY 16456	

//--------------dw1000---end---------------


#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      2000          /**< Timer period. LED1 timer will expire after 1000 ms */



#ifdef USE_FREERTOS

TaskHandle_t  ss_initiator_task_handle;   /**< Reference to SS TWR Initiator FreeRTOS task. */
extern void ss_initiator_task_function (void * pvParameter);
TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
#endif

#ifdef USE_FREERTOS

/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void led_toggle_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    LEDS_INVERT(BSP_LED_0_MASK);
    /* Delay a task for a given number of ticks */
    vTaskDelay(TASK_DELAY);
    /* Tasks must be implemented to never return... */
  }
}

/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void led_toggle_timer_callback (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  LEDS_INVERT(BSP_LED_1_MASK);
}
#else

  struct return_values {
			uint8 control, position,reset;
	};

	typedef struct return_values Struct;
	
	static uint8 NODE_ID = 1;

  extern Struct ss_init_run(uint8 source_node,uint8 target_node, uint8 tdoa_step, uint8 position, uint8 control);
	extern Struct ss_resp_run(uint8 NODE_ID, uint8 tdoa_step, uint8 position, uint8 control);
	extern int ss_reset(void);
	static Struct get_dist_diff_init(uint8 node_id, uint8 target_node, uint8 position);
	static Struct get_dist_diff_resp(uint8 node_id, uint8 target_node, uint8 position);
	
	//static uint8 sequence[]= {1,2, 2,3, 3,4, 4,1};//, 3,4};
	//static uint8 sequence[]= {1,2, 2,3, 3,1, 1,4, 4,3, 3,1};
	//static uint8 sequence[]= {1,2, 2,3, 3,1, 1,4, 4,3, 3,2, 2,4, 4,1};
	static uint8 sequence[]= {1,2, 2,3, 3,4, 4,5, 5,6, 6,1, 1,5, 5,3, 3,1, 1,4, 4,6, 6,2, 2,4, 4,5, 5,2, 2,3, 3,6, 6,1};
		
	//static uint8 sequence[]= {1,2, 2,3, 3,4, 4,5, 5,6, 6,1, 1,5, 5,3, 3,1, 1,4, 4,6, 6,2, 2,4, 4,5, 5,2, 2,1};
	static int lenght_sequence = sizeof(sequence)/sizeof(sequence[0]);

#endif   // #ifdef USE_FREERTOS

int main(void)
{
  /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );

  #ifdef USE_FREERTOS
    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    /* Create task for SS TWR Initiator set to 2 */
    UNUSED_VARIABLE(xTaskCreate(ss_initiator_task_function, "SSTWR_INIT", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_initiator_task_handle));
  #endif // #ifdef USE_FREERTOS
  
  //-------------dw1000  ini------------------------------------	

  /* Setup DW1000 IRQ pin */  
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		//irq
  
  /*Initialization UART*/
  boUART_Init ();
  printf("Singled Sided Two Way Ranging Responder Example \r\n");
	
	
  
  /* Reset DW1000 */
  reset_DW1000(); 

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();			
  
  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    //Init of DW1000 Failed
    while (1) {};
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();

  /* Configure DW1000. */
  dwt_configure(&config);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set preamble timeout for expected frames. See NOTE 3 below. */
  //dwt_setpreambledetecttimeout(0); // PRE_TIMEOUT
          
  /* Set expected response's delay and timeout. 
  * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(65000); // Maximum value timeout with DW1000 is 65ms  

  //-------------dw1000  ini------end---------------------------	
  // IF WE GET HERE THEN THE LEDS WILL BLINK

  #ifdef USE_FREERTOS		
    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();	

    while(1) 
    {};
  #else
	// No RTOS task here so just call the main loop here.
	// Loop forever responding to ranging requests.

	char read_serial[2];
	char serial_aux[2] = { 's', 'r' };


	Struct values;
	values.position = 0;
	values.control = 2;
	values.reset = 0;

	uint8 target = sequence[values.position + 1];
	uint8 source;
	uint8 tdoa_step = 0;
	uint8 prev_pos = values.position;

	bool loop = true;


	// NODE_ID = 3;
	printf("NODE_ID %d\r\n", NODE_ID);

	bool change_values = true;
	int counter = 0;
	
	//Reseting values for init sequence
	if (NODE_ID == 1)
	{
		values.position = 0;
		values.control = 2;
		dwt_setrxtimeout(65000);
		ss_reset();
	}
	
	while (loop)
	{
		//Check end sequence
		if (values.position >= lenght_sequence)
		{
			values.position = 0;
			counter++;
			printf("%d\r\n", counter);
		}

		///Sending (Node working as the initiator node)
		if (NODE_ID == sequence[values.position])
		{
			if (NODE_ID == 1 && values.position == 0)
			{
				while (1)
				{
					boUART_getc(read_serial);
					if (read_serial[0] == serial_aux[0])
					{
						read_serial[0] = 'E';
						printf("S\r\n");
						break;
					}
					else if (read_serial[0] == serial_aux[1])
					{
						values.position = 0;
						values.control = 2;
						values.reset = 0;
						change_values = true;
						prev_pos = values.position;
						tdoa_step = 0;
						dwt_setrxtimeout(65000);
						ss_reset();
					}

					printf("-");
					deca_sleep(100);
				}
			}

			source = NODE_ID;

			if (change_values)
			{
				if (values.control == 0 || values.control == 2)
				{

					target = sequence[values.position + 1];
					values.control = 0;
					tdoa_step = 0;
				}
				else
				{

					target = sequence[values.position - 1];
					values.control = 0;
					tdoa_step = 1;
				}
			}

			deca_sleep(1);
			prev_pos = values.position;
			
			dwt_setrxtimeout(65000);
			values = ss_init_run(NODE_ID, target, tdoa_step, values.position, values.control);

			if (prev_pos == values.position)
			{
				change_values = false;
			}
			else
			{
				change_values = true;
			}

			if (values.reset == 1)
			{
				values.position = 0;
				values.control = 2;
				values.reset = 0;
				change_values = true;
				prev_pos = values.position;
				tdoa_step = 0;
			}

			//printf("INIT values con %d, pos%d\r\n",values.control,values.position);
		}

		////Responding (Node working as the responder node)
		else if (NODE_ID != sequence[values.position])
		{

			values.control = 10;
			dwt_setrxtimeout(0);
			values = ss_resp_run(NODE_ID, tdoa_step, values.position, values.control);
			
			if (values.reset == 1)
			{
				values.position = 0;
				values.control = 2;
				values.reset = 0;
				change_values = true;
				prev_pos = values.position;
				tdoa_step = 0;;
			}
		}
	}
		
  #endif
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.  NB:SEE ALSO NOTE 11.
 * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate. For a real production application, each
 *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
 * 3. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 200 µs).
 * 4. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 *
 ****************************************************************************************************************************************************/




