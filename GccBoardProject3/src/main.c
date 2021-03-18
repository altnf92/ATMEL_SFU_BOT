/**
 * \file
 *
 * \brief Empty user application template
 *
 */

 

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */
/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

 

#include <string.h>
#include <asf.h>
#include <custom_pin_map.h>
#include <ADC_4_Channel.h>
 

/* CMD byte received from USART (top board)
 * CMD_OPT_MAP : option MSG to set combination action between selected actuator and sensor
 * CMD_OPT_ACTUATOR : option MSG to set activation method of selected actuator
 * CMD_OPT_PERIOD : option MSG to set RTC period
 * CMD_ACT_REQ : request MSG to activate selected actuator
 * CMD_RES : unused
 * CMD_SEN_REQ : request MSG to get sensor data of bottom board
 * CMD_EXT_SEN : respond MSG which includes external sensor data
 * CMD_OPT_LOCAL_CLK : option MSG to set local clock time
 * CMD_REQ_LOCAL_CLK : request MSG to get local clock time
 * CMD_OPT_ACT_SCH : option MSG to set act schedule */
#define CMD_OPT_MAP			0x01
#define CMD_OPT_ACTUATOR	0x02
#define CMD_OPT_PERIOD		0x03
#define CMD_ACT_REQ			0x04
#define CMD_RES				0x05
#define CMD_SEN_REQ			0x06
#define CMD_EXT_SEN			0x07
#define CMD_OPT_LOCAL_CLK	0x08
#define CMD_REQ_LOCAL_CLK	0x09
#define CMD_OPT_ACT_SCH		0x0B


/* Respond byte for USART packet
 * NACK : successfully received MSG, but act has bad issue
 * ACK : successfully received MSG and act
 * HISTORY_CMD : send sensor data and act history to top board
 * EXT_SEN_REQ : send request MSG to server to get external sensor data
 * REQ_CLK_RESPOND : send local clock time
*/
#define NACK				0xdd
#define ACK					0xaa
#define HISTORY_CMD			0xbb
#define EXT_SEN_REQ			0xcc
#define REQ_CLK_RESPOND		0xee

// sensor wormup time (sec) (ex vegetronix soil temp sensor needs 1 sec as worm up time)
#define SEN_WORMUP		1

// Maximum number of sensor(including upper board sensor)
#define SENSOR_NUM		8

// actuator actuator_list[ACTUATOR_NUM]
// Maximum number of actuator
#define ACTUATOR_NUM	3

// act_info actuator_sensor_mapping[MAPPING_PAGE_NUM]
// Maximum number of action information pages 
#define MAPPING_PAGE_NUM			8

// Maximum length of cmd msg from upper board
#define MAX_CMD_LENTH				9

// Basic rtc step (min)
#define BASIC_RTC_STEP				10

// Maximum time schedule page number
#define TIME_SCHEDULE_NUM			4

// USART BUFFER - 1 byte ~ incompatible between two boards' UART
#define MAX_UART_BUF_SIZE		1




// RTC Instance
struct rtc_module rtc_instance;


// rtc period
// rtc period will be : rtc_period_steps * BASIC_RTC_STEP (minimum : 1 min)
uint16_t rtc_period_steps = 1;		// this var indicates how many steps should be passed to trigger RTC routine.
									// if this var = 3 and BASIC_RTC_STEP = 10(min), RTC routine period is 30 min.
uint16_t tmp_period_steps;			// this var is used as temp counter to trigger RTC routine.


// local time
uint8_t local_hour = 0;
uint8_t local_min = 0;



// RTC routine activation length variables
// rtc ISR loop cnt
uint8_t act_cnt = 2;
// rtc ISR actuating sec
uint8_t internal_act_sec = 5;
// if act_cnt = 3, act_sec = 2(sec), in the RTC routine, actuators act for 2 sec, and check sensors. this small routine will go 3 times.




// ACT time schedule
// this page indicates scheduled act. if start time = 9:00, end time = 18:00, actuator number = 2, 
// actuator number 2 will start act at 9:00 and end at 18:00
typedef struct _sch{
	uint8_t start_hour;			
	uint8_t start_min;

	uint8_t end_hour;
	uint8_t end_min;
	
	uint8_t actuator_number;
}time_schedule;
time_schedule time_schedule_list[TIME_SCHEDULE_NUM];


// Actuator info struct
typedef struct _actu{
	uint8_t act_durability;		// else : till next RTC intr / 0 : follow the internal execution time
	uint8_t sch_act_flag;		// if this actuator is scheduled on schedule list, RTC ISR should not off this actu.

	uint8_t initial_state;		// if 0, default state is clear / if 1, default state is set
	uint8_t act_GPIO;
}actuator;
actuator actuator_list[ACTUATOR_NUM];

// Acting info ~ sensor actuator mapping etc
typedef struct _act{
	uint8_t mapped_sensor;
	uint8_t mapped_actuator;	// sensor index associated with this actuator
	uint8_t trigger_method;
	// 1	: if sensor val is over the threshold, activate gpio
	// 0	: if sensor val is under the threshold, activate gpio
	int32_t threshold;
}act_info;
act_info actuator_sensor_mapping[MAPPING_PAGE_NUM];

// actual page number that created
uint8_t map_page_count = 0;



// acted history - 1 actuator per 1 bit
uint8_t acted = 0;
// ex) 0000 0101   ~> 0, 2 actuator acted in last rtc routine



// ADC sensor buffer 
volatile uint16_t sensor_data[4];

// External sensor buffer(from Top board)
int32_t ext_sensor_data[4];
// External sensor get flag
uint8_t ext_sensor_get_bool = 0;


// USART Instance
struct usart_module usart_instance;
// UART buffer unprocessed
volatile uint8_t rx_buffer[MAX_UART_BUF_SIZE];
// USART buffer for saving CMD
uint8_t cmd_from_upper[10];
// RTC ACT REQ CMD
uint16_t cmd_for_req[2];
// ACK CMD
uint8_t ack_to_upper;






/* @ config RTC as counter mode. (global instance : rtc_instance)
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void configure_rtc_count(void);

/* @ config RTC callback functions
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void configure_rtc_callbacks(void);



/* @ RTC counter overflow callback.
 *		This function will send evt message to task2. 
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void rtc_overflow_callback(void);

/* @ RTC routine. 
 *		When rtc_overflow_callback sends evt msg to task2, this function will be called.
 *		It includes activation, sensing, etc.
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void rtc_routine(void);



/* @ ACT request routine.
 *		When ACT request comes, this function will be called.
 *		It will run on task 2, and includes actuator activation seq.
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void req_routine(void);


/* @ config USART (global instance : usart_instance)
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void configure_usart(void);

/* @ config USART callback functions (read, write)
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void configure_usart_callbakcs(void);


/* @ USART read callback function.
 *		It will send evt message to task 1. 
 * @ updated : 2021.02.24
 * @ return void
 * @ param usart_module : USART handle instance */
void usart_read_callback(const struct usart_module *const usart_module);

/* unused */
void usart_write_callback(const struct usart_module *const usart_module);


/* @ USART endian trans.
 *		Top board and bottom board have a different endian order at U(S)ART. 
 *		So it should be translated into a different endian order.
 * @ updated : 2021.02.24
 * @ return void
 * @ param original : Original data to be processed. */
uint8_t endian_trans(uint8_t original);

/* @ usart_read_char_static
 *		This function reads data one character at a time from the USART buffer.
 *		Progress will be saved statically. Data will be saved at 'cmd_from_upper'
 * @ updated : 2021.02.24
 * @ return void
 * @ param void */
void usart_read_char_static(void);

/* @ CMD branch function.
 *		This function serves as a branch of the UART message from the top board.
 *		(UART message : cmd_from_upper[] ~ global variable buffer)
 * @ updated : 2021.02.24 
 *		recent update : return type uint8_t -> void
 * @ return void
 * @ param void */
void cmd_func(void);


/* @ Send sensor data. Bottom board has 4 analog sensor ports. This function sends them to top board.
 * @ updated : 2021.02.24
 * @ return void
 * @ param acted_actu : actuator act history, each bit represents actuator number */
void sensor_data_send(uint8_t acted_actu);




// close-loop settings
void threshold_set(uint8_t index, uint16_t _high, uint16_t _low);
void actuator_set(uint8_t index, uint8_t durability, uint8_t initial_state, uint8_t GPIO);
void act_info_set(uint8_t map_index, uint8_t sensor_index, uint8_t act_index, uint8_t trig_method, int32_t thres);

// initial close-loop setting
void feedback_param_init(void);



// FreeRTOS parameters
xTaskHandle task1_id, task2_id;
xQueueHandle Queue_id, Queue_id_rtc;

// EVT macro
enum{
	EVT_UART_GET,
	EVT_MAIN_UART_DONE,
	EVT_MAIN_ACK,
	EVT_MAIN_NACK,
	
	EVT_RTC_CALLBACK,
	EVT_ACT_REQ,
	EVT_SEN_REQ,
	
	EVT_EXT_SEN,
	EVT_EXT_SEN_DONE,
	
	EVT_LOCAL_CLK_REQ,
};

 
 
 
 
 //==============================================================================================
 //===========================main & tasks=======================================================
 //==============================================================================================
// main task
static void task1(void* pvParameters){
	uint8_t queue_msg[2];	
	uint8_t ack_buffer;
	
	config_adc_func();
	// initialize adc module
	
	configure_usart();
	configure_usart_callbakcs();
	// initialize USART Module, register callbacks


	tmp_period_steps = rtc_period_steps;
	// temp variable for RTC ISR	
	configure_rtc_count();
	rtc_count_set_compare(&rtc_instance, (BASIC_RTC_STEP * 60) - 1, RTC_COUNT_COMPARE_0);
	// * 60 ~ sec to min / - 1 ~ rtc period bias 1 sec
	configure_rtc_callbacks();
	// config RTC Module, set RTC Timer, register RTC timer callback
	
	memset(rx_buffer, 0, sizeof(rx_buffer));
	usart_read_buffer_job(&usart_instance, (uint8_t*)rx_buffer, MAX_UART_BUF_SIZE);
	// trigger USART Read callback (after this trigger, callback will be triggered at the end of callback)

	
	for(;;){
		xQueueReceive(Queue_id, queue_msg, portMAX_DELAY);	
		switch(queue_msg[0]){
			
			case EVT_UART_GET :
				usart_read_char_static();
				break;
				
			// USART Read callback publishes this evt
			case EVT_MAIN_UART_DONE :
				cmd_func();		// distinguish CMD 
			case EVT_EXT_SEN :
			case EVT_EXT_SEN_DONE :
				memset(cmd_from_upper, 0, sizeof(cmd_from_upper));
				break;
				
			// after cmd_func() ~ ACK(can recognize CMD, ETC) or NACK(cant recog CMD, ETC)	
			case EVT_MAIN_ACK :
				ack_buffer = endian_trans(ACK);
				usart_write_wait(&usart_instance, ack_buffer);
				ack_buffer = endian_trans(ack_to_upper);
				usart_write_wait(&usart_instance, ack_buffer);
				// send ACK + CMD
				break;
			case EVT_MAIN_NACK :				
				ack_buffer = endian_trans(NACK);
				usart_write_wait(&usart_instance, ack_buffer);
				ack_buffer = endian_trans(ack_to_upper);
				usart_write_wait(&usart_instance, ack_buffer);
				// send NACK + CMD

				break;
			case EVT_LOCAL_CLK_REQ :
				ack_buffer = endian_trans(REQ_CLK_RESPOND);
				usart_write_wait(&usart_instance, ack_buffer);
				ack_buffer = endian_trans(local_hour);
				usart_write_wait(&usart_instance, ack_buffer);
				ack_buffer = endian_trans(local_min);
				usart_write_wait(&usart_instance, ack_buffer);
				// send
			
				break;
				
				
			
			default : break;
		}
	}
	vTaskDelete(NULL);
} 

static void task2(void* pvParameters){
	uint8_t queue_msg[2];	
	for(;;){
		xQueueReceive(Queue_id_rtc, queue_msg, portMAX_DELAY);
		switch(queue_msg[0]){

			// RTC Timer callback publishes this evt.
			case EVT_RTC_CALLBACK :
				rtc_routine();
				break;
			
			// cmd_for_req[0] ~ actu num / [1] ~ act sec
			case EVT_ACT_REQ :
				req_routine();				
				break;

			case EVT_SEN_REQ :
				sensor_on();
				vTaskDelay(SEN_WORMUP * 1000);
				
				get_sen_data(sensor_data);
				sensor_data_send(acted);

				sensor_off();
				break;
						
			default : break;
		}
	}
	vTaskDelete(NULL);
} 
// init
int main (void){
	system_init();			// sys init
	system_interrupt_enable_global();
 
	pin_configure();
	feedback_param_init();
	// pin config & feedback parameter initialize

	xTaskCreate(task1, "task 1", configMINIMAL_STACK_SIZE + 1000, NULL, 2, &task1_id);
	// main task - UART EVT ...
	xTaskCreate(task2, "task 2", configMINIMAL_STACK_SIZE + 1000, NULL, 2, &task2_id);	
	// RTC, Actuation task

	Queue_id		= xQueueCreate(1, sizeof(uint8_t));
	Queue_id_rtc	= xQueueCreate(1, sizeof(uint8_t));
	// MSGQ

	vTaskStartScheduler();
	while(1);
}
 //==============================================================================================
 //===========================main & tasks=======================================================
 //==============================================================================================


















//==============================================================================================
//===========================close loop & pin===================================================
//==============================================================================================
void actuator_set(uint8_t index, uint8_t durability, uint8_t initial_state, uint8_t GPIO){
	actuator_list[index].act_durability	= durability;

	actuator_list[index].initial_state	= initial_state;
	actuator_list[index].act_GPIO		= GPIO;
	
}
void act_info_set(uint8_t map_index, uint8_t sensor_index, uint8_t act_index,
					uint8_t trig_method, int32_t thres){
	actuator_sensor_mapping[map_index].mapped_actuator	= act_index;
	actuator_sensor_mapping[map_index].mapped_sensor	= sensor_index;

	actuator_sensor_mapping[map_index].trigger_method = trig_method;
	actuator_sensor_mapping[map_index].threshold = thres;
}
void feedback_param_init(void){
	actuator_set(0, 0, 0, SOL1_EN);
	actuator_set(1, 0, 0, SOL2_EN);
	actuator_set(2, 0, 0, SOL3_EN);
	port_pin_set_output_level(SOL1_EN, false);
	port_pin_set_output_level(SOL2_EN, false);
	port_pin_set_output_level(SOL3_EN, false);

	
	act_info_set(0, 0, 4, 1, 1500);
	act_info_set(1, 1, 4, 1, 1500);
	act_info_set(2, 2, 4, 1, 1500);
	act_info_set(3, 3, 4, 1, 1500);
	
	map_page_count = 4;
}
//==============================================================================================
//===========================close loop & pin===================================================
//==============================================================================================



















//==============================================================================================
//===========================RTC & ACT==========================================================
//==============================================================================================
void configure_rtc_count(void){
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);

	config_rtc_count.prescaler				= RTC_COUNT_PRESCALER_DIV_1024;
	config_rtc_count.mode					= RTC_COUNT_MODE_32BIT;
	config_rtc_count.continuously_update	= true;

	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	rtc_count_enable(&rtc_instance);
}
void configure_rtc_callbacks(void){
	rtc_count_register_callback(&rtc_instance, rtc_overflow_callback, RTC_COUNT_CALLBACK_COMPARE_0);
	rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_COMPARE_0);
}

void rtc_overflow_callback(void){
	uint8_t evt_msg, tmp;
	
	rtc_count_set_count(&rtc_instance, 0);
		
	local_min += BASIC_RTC_STEP;
	local_hour = (local_hour + local_min / 60) % 24;
	local_min = local_min % 60;
	
	
	for(uint8_t i = 0; i < TIME_SCHEDULE_NUM; i++){
		tmp = time_schedule_list[i].actuator_number;
		if(local_hour == time_schedule_list[i].start_hour && local_min == time_schedule_list[i].start_min){
			port_pin_set_output_level(actuator_list[tmp].act_GPIO, actuator_list[tmp].initial_state ^ 1);
			actuator_list[tmp].sch_act_flag = 1;
		}
		else if(local_hour == time_schedule_list[i].end_hour && local_min == time_schedule_list[i].end_min){
			port_pin_set_output_level(actuator_list[tmp].act_GPIO, actuator_list[tmp].initial_state);			
			actuator_list[tmp].sch_act_flag = 0;
		}
	}

	
	tmp_period_steps--;
	if(	tmp_period_steps == 0){
		tmp_period_steps = rtc_period_steps;
		
		evt_msg = EVT_RTC_CALLBACK;
		xQueueSend(Queue_id_rtc, &evt_msg, 0);
	}
	
				
	//time_schedule_list / actuator_list
	// TIME_SCHEDULE_NUM
	// 시간 단위는 10분으로 통일될 것 ~ 조건을 이상, 이하로 할 필요 없이 ==로 하면 된다.
	
	return;
}
void rtc_routine(void){
	uint8_t i, j, bit_mask;
	uint8_t status, tmp_acted, tmp_uint8;
	uint8_t buffer;

	act_info* tmp;

	
	sensor_on();
	vTaskDelay(SEN_WORMUP * 1000);	// sec ~> msec
				
	get_sen_data(sensor_data);
//	sensor_data_send(acted);
	
	// req!!!	
	buffer = endian_trans(EXT_SEN_REQ);
	usart_write_wait(&usart_instance, buffer);				
	for(uint8_t di = 0; di < 50; di++){
		if(ext_sensor_get_bool == 1)	break;
		else							vTaskDelay(100);
	}
	ext_sensor_get_bool = 0;
				
	
	
	tmp_acted = 0;

	for(j = 0; j < act_cnt; j++){
		for(i = 0; i < map_page_count; i++){
			tmp = &(actuator_sensor_mapping[i]);
			uint8_t sensor_index = tmp->mapped_sensor;
			
			if(sensor_index < SENSOR_NUM){
				if(sensor_data[sensor_index] > tmp->threshold && tmp->trigger_method == 1)	status = 1;
				else if(sensor_data[sensor_index] <= tmp->threshold && tmp->trigger_method == 0)	status = 1;
				else	status = 0;
			}
			else{
				sensor_index = sensor_index - SENSOR_NUM;
				if(ext_sensor_data[sensor_index] > tmp->threshold && tmp->trigger_method == 1)	status = 1;
				else if(ext_sensor_data[sensor_index] <= tmp->threshold && tmp->trigger_method == 0)	status = 1;
				else	status = 0;				
			}
			// trigger method	~> over than 0 : it will operate when sensor > thres
			//					~> less than 0 : " when sensor <= thres
			// status : if trigger region and actual sensor val region is same, it will become 1 / else, 0
			
			if(actuator_list[tmp->mapped_actuator].sch_act_flag != 1){
				if(status == 1)	{
					port_pin_set_output_level(actuator_list[tmp->mapped_actuator].act_GPIO,
					actuator_list[tmp->mapped_actuator].initial_state ^ 1);
					
					tmp_acted = tmp_acted | (1 << tmp->mapped_actuator);
				}
				else{
					port_pin_set_output_level(actuator_list[tmp->mapped_actuator].act_GPIO,
					actuator_list[tmp->mapped_actuator].initial_state);
				}
			}
			
		}
		
		vTaskDelay(internal_act_sec * 1000); // sec ~> msec
		get_sen_data(sensor_data);
		
		buffer = endian_trans(EXT_SEN_REQ);
		usart_write_wait(&usart_instance, buffer);
		for(uint8_t di = 0; di < 50; di++){
			if(ext_sensor_get_bool == 1)	break;
			else							vTaskDelay(100);
		}
		ext_sensor_get_bool = 0;
	}
	
	sensor_data_send(tmp_acted);
	
	sensor_off();
	acted = 0;




	for(i = 0; i < map_page_count; i++){
		tmp_uint8 = actuator_sensor_mapping[i].mapped_actuator;
		if(actuator_list[tmp_uint8].act_durability == 0 && actuator_list[tmp_uint8].sch_act_flag == 0){
			port_pin_set_output_level(actuator_list[tmp_uint8].act_GPIO, actuator_list[tmp_uint8].initial_state);
		}
		else{
			acted = acted | (tmp_acted & (0x01 << tmp_uint8));
		}
	}
}

void req_routine(void){
	 // cmd_for_req[0] ~ actu num / [1] ~ act sec

	 // act
	 port_pin_set_output_level(actuator_list[cmd_for_req[0]].act_GPIO,
	 actuator_list[cmd_for_req[0]].initial_state ^ 1);
	 vTaskDelay(cmd_for_req[1] * 1000);

	 // data send after act
	 get_sen_data(sensor_data);
	 sensor_data_send( acted | (1 << cmd_for_req[0]) );

	 // after act behavior
	 if(actuator_list[cmd_for_req[0]].act_durability == 0){
		 port_pin_set_output_level(actuator_list[cmd_for_req[0]].act_GPIO,
		 actuator_list[cmd_for_req[0]].initial_state);
	 }
	 else{
		 acted = acted | (1 << cmd_for_req[0]);
	 }
	 
	 return;
 }
//==============================================================================================
//===========================RTC & ACT==========================================================
//==============================================================================================
























//==============================================================================================
//===========================USART==============================================================
//==============================================================================================
void configure_usart(void){
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	
	config_usart.character_size = USART_CHARACTER_SIZE_8BIT;
	config_usart.transfer_mode = USART_TRANSFER_ASYNCHRONOUSLY;
	config_usart.baudrate = 9600;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;
	

	while (usart_init(&usart_instance, SERCOM4, &config_usart) != STATUS_OK);
	usart_enable(&usart_instance);
}
void configure_usart_callbakcs(void){
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	
	usart_register_callback(&usart_instance, usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
}

void usart_read_callback(const struct usart_module *const usart_module){
	uint8_t evt_msg = EVT_UART_GET;
	xQueueSend(Queue_id, &evt_msg, 0);
}
void usart_write_callback(const struct usart_module *const usart_module){}
void usart_read_char_static(){
	uint8_t evt_msg;
	static uint8_t index = 0;
	
	// while (usart_write_wait(&usart_instance, (uint8_t*)(rx_buffer)[ret++]) != STATUS_OK);
	if(index == 0 && rx_buffer[0] == 0) goto Label;
	cmd_from_upper[index++] = endian_trans(rx_buffer[0]);

	if(index == cmd_from_upper[0] && index != 0){
		index = 0;
		evt_msg = EVT_MAIN_UART_DONE;
		xQueueSend(Queue_id, &evt_msg, 0);
	}
Label:
	memset(rx_buffer, 0, sizeof(rx_buffer));
	usart_read_buffer_job(&usart_instance, (uint8_t*)rx_buffer, MAX_UART_BUF_SIZE);
}

void cmd_func(void){
	uint8_t evt_msg = -1;
	
	ack_to_upper = 0xff;

	switch (cmd_from_upper[1]){
		
		
		case CMD_OPT_MAP :
		// LEN|CMD|Profile number|mapping(upper 4bit-sensor/lower-actu)|Thres1|Thres2|Thres3|Thres4|Trigger
			ack_to_upper = CMD_OPT_MAP;

			if(cmd_from_upper[2] >= map_page_count && cmd_from_upper[2] < MAPPING_PAGE_NUM) {
				map_page_count++;
			}
			else if(cmd_from_upper[2] >= MAPPING_PAGE_NUM)			{goto NACK_LABEL;}
			if( ((cmd_from_upper[3] >> 4) & 0x0f) >= SENSOR_NUM )	{goto NACK_LABEL;}
			if( (cmd_from_upper[3] & 0x0f) >= ACTUATOR_NUM )		{goto NACK_LABEL;}
			if( cmd_from_upper[8] != 1 && cmd_from_upper[8] != 0)	{goto NACK_LABEL;}
		
			actuator_sensor_mapping[cmd_from_upper[2]].mapped_actuator	= cmd_from_upper[3] & 0x0f;
			actuator_sensor_mapping[cmd_from_upper[2]].mapped_sensor	= (cmd_from_upper[3] >> 4) & 0x0f;
			actuator_sensor_mapping[cmd_from_upper[2]].threshold		= (cmd_from_upper[4] << 24) + (cmd_from_upper[5] << 16)
																	+ (cmd_from_upper[6] << 8) + (cmd_from_upper[7]);
			actuator_sensor_mapping[cmd_from_upper[2]].trigger_method	= cmd_from_upper[8];
		
			evt_msg = EVT_MAIN_ACK;
		
			break;
		
		case CMD_OPT_ACTUATOR :
		// LEN|CMD|actuator number|initial state|toggle method
			ack_to_upper = CMD_OPT_ACTUATOR;

			if( ((cmd_from_upper[2] >> 4) &0x0f) >= ACTUATOR_NUM ) goto NACK_LABEL;
			if( cmd_from_upper[3] != 0 && cmd_from_upper[3] != 1 ) goto NACK_LABEL;
			if( cmd_from_upper[4] != 0 && cmd_from_upper[4] != 1 ) goto NACK_LABEL;

			actuator_list[cmd_from_upper[2]].initial_state = cmd_from_upper[3];
			actuator_list[cmd_from_upper[2]].act_durability = cmd_from_upper[4];
		
			port_pin_set_output_level(actuator_list[cmd_from_upper[2]].act_GPIO, actuator_list[cmd_from_upper[2]].initial_state);

			evt_msg = EVT_MAIN_ACK;
			break;

		case CMD_OPT_PERIOD :
		// LEN|CMD|Period steps high|Period steps low|act sec|act cnt
			ack_to_upper = CMD_OPT_PERIOD;

		uint16_t tmp;

			tmp = (cmd_from_upper[2] << 8) + cmd_from_upper[3];
			internal_act_sec = cmd_from_upper[4];
			act_cnt = cmd_from_upper[5];

			if(tmp * BASIC_RTC_STEP * 60 * 0.8 <= internal_act_sec * act_cnt ) {
				// 80% of RTC ISR Period should be bigger than [internal_act_sec * act_cnt]
				goto NACK_LABEL;
			}
			else {
				rtc_period_steps = tmp;
				tmp_period_steps = tmp;
			}

			evt_msg = EVT_MAIN_ACK;
			break;

		case CMD_ACT_REQ :
		// LEN|CMD|actuator number|act sec1|act sec2
			ack_to_upper = CMD_ACT_REQ;

			if(cmd_from_upper[2] >= ACTUATOR_NUM) goto NACK_LABEL;
			cmd_for_req[0] = cmd_from_upper[2];
			cmd_for_req[1] = (cmd_from_upper[3] << 8) + cmd_from_upper[4];
		
			evt_msg = EVT_ACT_REQ;
			xQueueSend(Queue_id_rtc, &evt_msg, 0);
		
			evt_msg = EVT_MAIN_ACK;
			break;
		
		case CMD_SEN_REQ :
		// LEN|CMD
			ack_to_upper = CMD_SEN_REQ;
		
			evt_msg = EVT_SEN_REQ;
			xQueueSend(Queue_id_rtc, &evt_msg, 0);
		
			evt_msg = EVT_MAIN_ACK;
			break;
		
		case CMD_EXT_SEN :
		// LEN|CMD|LABEL|DATA1|DATA2|DATA3|DATA4
			ack_to_upper = CMD_EXT_SEN;
			uint8_t ext_sensor_num;
			// sensor number
			if(cmd_from_upper[2] == 0xff)	{
				ext_sensor_get_bool = 1;
			
				evt_msg = EVT_EXT_SEN_DONE;
				break;
			}
			else {
				ext_sensor_num = cmd_from_upper[2];
				ext_sensor_data[ext_sensor_num - SENSOR_NUM] =
				(cmd_from_upper[3] << 24) + (cmd_from_upper[4] << 16) +
				(cmd_from_upper[5] << 8) + (cmd_from_upper[6]);

				evt_msg = EVT_EXT_SEN;
				break;
			}
		
		case CMD_OPT_LOCAL_CLK :
		// LEN|CMD|HOUR|MINUTE_DIV_10
			ack_to_upper = CMD_OPT_LOCAL_CLK;

			local_hour = cmd_from_upper[2];
			local_min = cmd_from_upper[3] * BASIC_RTC_STEP;

			evt_msg = EVT_MAIN_ACK;
			break;
			
		case CMD_REQ_LOCAL_CLK:
		// LEN|CMD
			ack_to_upper = CMD_OPT_LOCAL_CLK;
			evt_msg = EVT_LOCAL_CLK_REQ;		
			break;
		
		case CMD_OPT_ACT_SCH:
		// LEN|CMD|START_HOUR|START_MIN|END_HOUR|END_MIN|ACTUATOR_NUM|SCH_NUM
		
			ack_to_upper = CMD_OPT_ACT_SCH;
			
			time_schedule_list[cmd_from_upper[7]].start_hour = cmd_from_upper[2];
			time_schedule_list[cmd_from_upper[7]].start_min = cmd_from_upper[3];
			time_schedule_list[cmd_from_upper[7]].end_hour = cmd_from_upper[4];
			time_schedule_list[cmd_from_upper[7]].end_min = cmd_from_upper[5];
			time_schedule_list[cmd_from_upper[7]].actuator_number = cmd_from_upper[6];


			// 1. 시작 시간보다 현재 시간이 큰 경우							:: 시작 < 현재
			//  1) 끝 시간이 0 이전인 경우 ~ 현재 시간이 끝시간보다 작아야함			:: 시작 < 끝 && 현재 < 끝
			//  2) 끝 시간이 0 이후인 경우 ex) 시작 18시 현재 21시 끝 1시			:: 끝 < 시작이면 무조건 ok

			// 2. 현재 시간이 시작 시간보다 작을 경우 ex) 시작 20시 현재 1시	:: 시작 > 현재 
			//																	:: 끝 > 현재 && 시작 > 끝

			// 3. else 
			
			uint16_t tp_time = local_hour * 60 + local_min;
			uint16_t st_time = cmd_from_upper[2] * 60 + cmd_from_upper[3];
			uint16_t ed_time = cmd_from_upper[4] * 60 + cmd_from_upper[5];
			uint8_t tmp_flag = 0;

			if(st_time < tp_time){
				if(st_time < ed_time && tp_time < ed_time)	tmp_flag = 1;
				else if(ed_time < st_time)					tmp_flag = 1;
			}
			else if(tp_time < st_time){
				if(ed_time > tp_time && st_time > ed_time)	tmp_flag = 1;
			}
			else tmp_flag = 1;
			
			if(tmp_flag == 1){
				port_pin_set_output_level(actuator_list[cmd_from_upper[6]].act_GPIO, actuator_list[cmd_from_upper[6]].initial_state ^ 1);
				actuator_list[cmd_from_upper[6]].sch_act_flag = 1;	
			}

/*			
			uint16_t tp_time = local_hour * 60 + local_min;
			uint16_t st_time = cmd_from_upper[2] * 60 + cmd_from_upper[3];
			uint16_t ed_time = cmd_from_upper[4] * 60 + cmd_from_upper[5];

			for(uint8_t idx = 0; idx < 144; idx++){
				tp_time = (tp_time + BASIC_RTC_STEP) % 1440;
				
				if(tp_time == st_time) break;
				else if(tp_time == ed_time){
					port_pin_set_output_level(actuator_list[cmd_from_upper[6]].act_GPIO, actuator_list[cmd_from_upper[6]].initial_state ^ 1);
					actuator_list[cmd_from_upper[6]].sch_act_flag = 1;
					break;
				}
			}
*/		
			evt_msg = EVT_MAIN_ACK;
			break;
		
		
NACK_LABEL:
		default :
			evt_msg = EVT_MAIN_NACK;
			break;
	}
	
	
	xQueueSend(Queue_id, &evt_msg, 0);
}
uint8_t endian_trans(uint8_t original){
	uint8_t ret = 0;
	
	ret += (original >> 7) & 0b00000001;
	ret += (original >> 5) & 0b00000010;
	ret += (original >> 3) & 0b00000100;
	ret += (original >> 1) & 0b00001000;
	ret += (original << 1) & 0b00010000;
	ret += (original << 3) & 0b00100000;
	ret += (original << 5) & 0b01000000;
	ret += (original << 7) & 0b10000000;
	
	return ret;
}
// send sensor data and act history(which actuators are acted)
void sensor_data_send(uint8_t acted_actu){
	uint8_t tmp[] = {HISTORY_CMD, acted_actu,
		(sensor_data[0] >> 8) & 0xff,
		(sensor_data[0]) & 0xff,
		(sensor_data[1] >> 8) & 0xff,
		(sensor_data[1]) & 0xff,
		(sensor_data[2] >> 8) & 0xff,
		(sensor_data[2]) & 0xff,
		(sensor_data[3] >> 8) & 0xff,
	(sensor_data[3]) & 0xff};
	uint8_t send_byte;
	
	for(uint8_t i = 0; i < 10; i++){
		send_byte = endian_trans(tmp[i]);
		usart_write_wait(&usart_instance, send_byte);
	}
}
//==============================================================================================
//===========================USART==============================================================
//==============================================================================================

