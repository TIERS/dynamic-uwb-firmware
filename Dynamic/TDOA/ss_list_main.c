/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           This is a simple code example which acts as the responder in a SS TWR distance measurement exchange. 
*           This application waits for a "poll" message (recording the RX time-stamp of the poll) expected from 
*           the "SS TWR initiator" example code (companion to this application), and
*           then sends a "response" message recording its TX time-stamp.
*
*           Notes at the end of this file, to expand on the inline comments.
*
* @attention
*
* Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include "sdk_config.h" 
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

/* Inter-ranging delay period, in milliseconds. See NOTE 1*/
#define RNG_DELAY_MS 80

/* Frames used in the ranging process. See NOTE 2,3 below. */
//////////////////////////       0     1    2   3     4     5    6    7    8    9   10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 #check sum 30 31
static uint8 init_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 resp_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	
static uint8 reset_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	
	
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4	

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 32
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
//static uint64 poll_rx_ts;

/* Declaration of static functions. */
//static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
//static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
//static uint64 poll_rx_ts;
//static uint64 resp_tx_ts;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);

/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter

/*char buffer0[50];
char buffer1[50];
char buffer2[50];
char buffer3[50];
char buffer4[50];
char buffer5[50];
char buffer6[50];

char buffer7[50];
char buffer8[50];
char buffer9[50];
char buffer10[50];
char buffer11[50];
char buffer12[50];
char buffer13[50];*/

static int control_var=0;

#define TDOA_MSG_IDX 18
#define INIT_MSG_IDX 19
#define CLOCK_OFFSET_IDX 20
#define DISTANCE_IDX 24
#define SEQUENCE_POS_IDX 26

#define POLL_SOUR_IDX 7
#define POLL_DEST_IDX 5

#define RESP_SOUR_IDX 5
#define RESP_DEST_IDX 7

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
//static double comp;
static double distance_calc;

static double distance_msg;

static double distance_diff, distance_diff2;

static double distance_calc_vec[2];
static double distance_msg_vec[2];
static double distance_diff_vec[2];

//static uint32 time_init_tx, time_init_tx, time_resp_tx, time_resp_rx;
static uint32 poll_tx_ts, poll_rx_ts, resp_tx_ts, resp_rx_ts;

static uint32 list_init_rx_ts, list_resp_rx_ts, list_check_rx_ts;

int aux_cont=0;

#define TOTAL_NODES 8

double distance_calc_matrix[TOTAL_NODES][TOTAL_NODES] = {0};
double distance_msg_matrix[TOTAL_NODES][TOTAL_NODES] = {0};
double distance_diff_matrix[TOTAL_NODES][TOTAL_NODES] = {0};

/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Listener, read all receives messages, computes distances and difference of distances
*
* @param  none
*
* @return none
*/

int ss_list_run()
{
	/*Activate reception immediately. */
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	/*Poll for reception of a frame or error/timeout. See NOTE 5 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &(SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) 
	{};

	//printf("reception \r\n");
		#if 0	// Include to determine the type of timeout if required.
		int temp = 0;
		// (frame wait timeout and preamble detect timeout)
		if (status_reg & SYS_STATUS_RXRFTO)
			temp = 1;
		else if (status_reg & SYS_STATUS_RXPTO)
			temp = 2;
		#endif

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/*Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/*A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) &RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
			aux_cont++;
			//printf("%d\r\n",rx_buffer[POLL_SOUR_IDX]);
		}

		/*Check that the frame is a poll sent by "SS TWR initiator" example.
		 *As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0;

		uint8 sender;
		uint8 responder;
		
		sender = rx_buffer[7];
		responder = rx_buffer[5];
		
		init_poll_msg[POLL_SOUR_IDX] = sender;
		init_poll_msg[POLL_SOUR_IDX + 1] = sender;
		init_poll_msg[POLL_DEST_IDX] = responder;
		init_poll_msg[POLL_DEST_IDX + 1] = responder;

		resp_resp_msg[RESP_SOUR_IDX] = responder;
		resp_resp_msg[RESP_SOUR_IDX + 1] = responder;
		resp_resp_msg[RESP_DEST_IDX] = sender;
		resp_resp_msg[RESP_DEST_IDX + 1] = sender;

		//RESET message
		if (rx_buffer[POLL_SOUR_IDX] == 0)
		{
			printf("RESET %d %d, ori %d des %d pos %d\r\n", rx_count, rx_buffer[TDOA_MSG_IDX], rx_buffer[7], rx_buffer[5], rx_buffer[SEQUENCE_POS_IDX]);
		}
		else
		{
			// Poll message
			if (memcmp(rx_buffer, init_poll_msg, ALL_MSG_COMMON_LEN) == 0 &rx_buffer[INIT_MSG_IDX] == 0)
			{
				rx_count++;

				list_init_rx_ts = dwt_readrxtimestamplo32();

			}
			// Response message
			else if (memcmp(rx_buffer, resp_resp_msg, ALL_MSG_COMMON_LEN) == 0 &rx_buffer[INIT_MSG_IDX] == 0)
			{
				rx_count++;

				list_resp_rx_ts = dwt_readrxtimestamplo32();

				/*Get timestamps embedded in response message. */
				resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
				resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

			}
			// Data message
			else if (memcmp(rx_buffer, init_poll_msg, ALL_MSG_COMMON_LEN) == 0 &rx_buffer[INIT_MSG_IDX] == 1)
			{
				//printf("Initiator received\r\n");
				rx_count++;

				list_check_rx_ts = dwt_readrxtimestamplo32();

				int32 rtd_init, rtd_resp, rtd_list, rtd_check;
				uint8 dist_lsb, dist_msb;
				float clockOffsetRatio;
				float clockOffsetRatio2;
				uint32 readcarriervalue_unsigned;
				clockOffsetRatio = 0;

				float average_diff = 0;

				/*Get timestamps embedded in response message. */
				resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_tx_ts);
				resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_rx_ts);

				dist_msb = rx_buffer[DISTANCE_IDX];
				dist_lsb = rx_buffer[DISTANCE_IDX + 1];
				//readcarriervalue = rx_buffer[CLOCK_OFFSET_IDX];

				/*Get timestamps embedded in response message. */
				resp_msg_get_ts(&rx_buffer[CLOCK_OFFSET_IDX], &readcarriervalue_unsigned);

				int32 readcarriervalue = (int32) readcarriervalue_unsigned;

				distance_msg = ((float)(dist_msb *256) / 100) + ((float)(dist_lsb) / 100);

				/*Read carrier integrator value and calculate clock offset ratio. See NOTE 7 below. */
				//clockOffsetRatio = readcarriervalue *(FREQ_OFFSET_MULTIPLIER *HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;
				clockOffsetRatio = readcarriervalue *-5.7e-10;	//;((FREQ_OFFSET_MULTIPLIER *(HERTZ_TO_PPM_MULTIPLIER_CHAN_5))/ 1.0e6) ;
				clockOffsetRatio2 = readcarriervalue *-5.7e-10;	//;((FREQ_OFFSET_MULTIPLIER *(HERTZ_TO_PPM_MULTIPLIER_CHAN_5))/ 1.0e6) ;

				/*Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
				rtd_init = resp_rx_ts - poll_tx_ts;
				rtd_resp = resp_tx_ts - poll_rx_ts;


				//DISTANCE
				tof = ((rtd_init - rtd_resp *(1.0f - clockOffsetRatio)) / 2.0f) *DWT_TIME_UNITS;	// Specifying 1.0f and 2.0f are floats to clear warning 
				distance_calc = tof * SPEED_OF_LIGHT;

			
				double tof2;
				uint32 z;
				double tdoa,tdoa2;

				tof2=tof/DWT_TIME_UNITS;

				rtd_list=list_resp_rx_ts-list_init_rx_ts;

				tdoa = rtd_list-rtd_resp;
				tdoa = tdoa-tof2;

				distance_diff = tdoa*DWT_TIME_UNITS;
				distance_diff = distance_diff*SPEED_OF_LIGHT;

				rtd_check=list_check_rx_ts-list_resp_rx_ts;

				tdoa2 = rtd_check-rtd_init;
				tdoa2 = tdoa2-tof;
				
				//DIFFERENCE OF DISTANCE
				distance_diff2 = tdoa2*DWT_TIME_UNITS;
				distance_diff2 = distance_diff2*SPEED_OF_LIGHT;

				distance_calc_vec[rx_buffer[TDOA_MSG_IDX]]=distance_calc;
				distance_msg_vec[rx_buffer[TDOA_MSG_IDX]]=distance_msg;
				distance_diff_vec[rx_buffer[TDOA_MSG_IDX]]=distance_diff;	

				if (rx_buffer[TDOA_MSG_IDX]==0)
				{
					control_var=1;
				}

				else if (rx_buffer[TDOA_MSG_IDX]==1)
				{
					control_var=0;

					distance_calc_matrix[sender][responder] = distance_calc_vec[0];
					distance_calc_matrix[responder][sender] = distance_calc_vec[1];

					distance_msg_matrix[sender][responder] = distance_msg_vec[0];
					distance_msg_matrix[responder][sender] = distance_msg_vec[1];

					distance_diff_matrix[sender][responder] = distance_diff_vec[0];
					distance_diff_matrix[responder][sender] = distance_diff_vec[1];

				}	
				// Print all information by serial interfce
				if (rx_buffer[TDOA_MSG_IDX]==1)
				{
					printf("%d;%d;%.2f;%.2f;%.2f;%.2f\r\n",responder,sender,distance_msg_matrix[sender][responder],distance_msg_matrix[responder][sender],distance_diff_matrix[sender][responder],distance_diff_matrix[responder][sender]);
				
				}

				
			}
			// ACK message
			else if (memcmp(rx_buffer, resp_resp_msg, ALL_MSG_COMMON_LEN) == 0 &rx_buffer[INIT_MSG_IDX] == 1)
			{
				int i;
			}
		}
	}
	else
	{
		/*Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

		/*Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
	}

	return (control_var);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}


/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_responder_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);

  while (true)
  {
    ss_list_run(1,2);//check parameters for correct fuctioning
    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}

/**************************************************************************************************************/
/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. This is the task delay when using FreeRTOS. Task is delayed a given number of ticks. Useful to be able to define this out to see the effect of the RTOS
*    on timing.
* 2. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 3 below.
*     - byte 7/8: source address, see NOTE 3 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 4. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 5. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 6. POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors. For slower platforms where the SPI is at a slower speed 
*    or the processor is operating at a lower frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
*    Knowing the exact time when the responder is going to send its response is vital for time of flight calculation. The specification of the time of 
*    respnse must allow the processor enough time to do its calculations and put the packet in the Tx buffer. So more time required for a slower
*    system(processor).
* 7. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
*    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
*    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
*    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
*    8 bits.
* 8. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
*    time-of-flight computation) can be handled by a 32-bit subtraction.
* 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
*10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*    DW1000 API Guide for more details on the DW1000 driver functions.
*
****************************************************************************************************************************************************/
 