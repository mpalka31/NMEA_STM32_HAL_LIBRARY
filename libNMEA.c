/*
 * libNMEA.c
 *
 *  Created on: Nov 23, 2020
 *      Author: Michał Pałka
 */


#include "libNMEA.h"

static uint8_t NMEA__UART_buffer[NMEA_UART_BUFFER_SIZE]={0};
static uint8_t UART_char_tmp;
static uint8_t NMEA__working_buffer[NMEA_WORKING_BUFFER_SIZE]={0};
static uint8_t UART_buffer_head=0;
static uint8_t UART_buffer_tail=0;
static uint8_t UART_buffer_lines=0;

static void (*speed_change_CB_fun_ptr)(void);
static float  speed_change_tolerance;

static void (*speed_raise_barrier_CB_fun_ptr)(void);
static float speed_raise_barrier;

static void (*speed_fall_barrier_CB_fun_ptr)(void);
static float speed_fall_barrier;

static void default_CB(void){}

NMEA_status NMEA_CB_register(void (*CB_fun)(void),NMEA_CB_ID CB_id,float barrier){
	switch (CB_id) {
		case SPEED_CHANGE_CB:
			if (barrier < 0) return NMEA_WRONG_DATA;
			speed_change_CB_fun_ptr = CB_fun;
			speed_change_tolerance = barrier;
			return NMEA_OK;
		case SPEED_RISE_BARRIER_CB:
			if (barrier < 0) return NMEA_WRONG_DATA;
			speed_raise_barrier_CB_fun_ptr = CB_fun;
			speed_raise_barrier = barrier;
			return NMEA_OK;
		case SPEED_FALL_BARRIER_CB:
			if (barrier < 0) return NMEA_WRONG_DATA;
			speed_fall_barrier_CB_fun_ptr = CB_fun;
			speed_fall_barrier = barrier;
			return NMEA_OK;
		default:
			return NMEA_WRONG_CB_ID;
	}
	return NMEA_ERROR;
}

NMEA_status NMEA_CB_unregister(NMEA_CB_ID CB_id){
	switch (CB_id) {
		case SPEED_CHANGE_CB:
			speed_change_tolerance = 0;
			speed_change_CB_fun_ptr = default_CB;
			return NMEA_OK;
		case SPEED_RISE_BARRIER_CB:
			speed_raise_barrier = 0;
			speed_raise_barrier_CB_fun_ptr = default_CB;
			return NMEA_OK;
		case SPEED_FALL_BARRIER_CB:
			speed_fall_barrier = 0;
			speed_fall_barrier_CB_fun_ptr = default_CB;
			return NMEA_OK;
		default:
			return NMEA_WRONG_CB_ID;
	}
	return NMEA_ERROR;
}

static void 	NMEA_parser(char *message){

	NMEA_data previous_data = nmea_data;

	int num = 0;
	char *fields[32]={NULL};
	fields[num++]=message;
	while ((message = strchr(message, ','))) {
		*message++ = 0;
		fields[num++]=message;
	}

	if(strcmp(fields[0],"$GPGLL")==0){

		nmea_data.latitude = atof(fields[1]);
		nmea_data.latitude_direction = *fields[2];
		nmea_data.longitude = atof(fields[3]);
		nmea_data.longitude_direction = *fields[4];

	}else if(strcmp(fields[0],"$GPRMC")==0){

		nmea_data.UTC_time = atof(fields[1]);
		nmea_data.UT_date = atoi(fields[9]);

		nmea_data.latitude = atof(fields[3]);
		nmea_data.latitude_direction = *fields[4];
		nmea_data.longitude = atof(fields[5]);
		nmea_data.longitude_direction = *fields[6];


	}else if(strcmp(fields[0],"$GPVTG")==0){

		nmea_data.speed_knots =  atoi(fields[5]);
		nmea_data.speed_kmph =  atoi(fields[7]);

	}else if(strcmp(fields[0],"$GPGGA")==0){

		nmea_data.UTC_time = atof(fields[1]);

		nmea_data.latitude = atof(fields[2]);
		nmea_data.latitude_direction = *fields[3];
		nmea_data.longitude = atof(fields[4]);
		nmea_data.longitude_direction = *fields[5];

		nmea_data.fix = atoi(fields[6]);
		nmea_data.sat_in_use = atoi(fields[7]);
		nmea_data.HDOP = atof(fields[8]);

		nmea_data.altitude = atof(fields[9]);
		nmea_data.geoidal_separation = atof(fields[11]);

	}else if(strcmp(fields[0],"$GPGSA")==0){

		nmea_data.fix_mode = atoi(fields[2]);

		nmea_data.PDOP = atof(fields[15]);
		nmea_data.HDOP = atof(fields[16]);
		nmea_data.VDOP = atof(fields[17]);

	}else if(strcmp(fields[0],"$GPGSV")==0){
		nmea_data.sat_in_view = atoi(fields[3]);
	}

	if (abs(nmea_data.speed_kmph - previous_data.speed_kmph) > speed_change_tolerance){
		speed_change_CB_fun_ptr();
	}

	if (nmea_data.speed_kmph > speed_raise_barrier && previous_data.speed_kmph <= speed_raise_barrier){
		speed_raise_barrier_CB_fun_ptr();
	}

	if (nmea_data.speed_kmph < speed_raise_barrier && previous_data.speed_kmph >= speed_raise_barrier){
		speed_fall_barrier_CB_fun_ptr();
	}
}

static uint8_t hx2int(uint8_t n2, uint8_t n1){
	if (n2 <= '9') n2-='0';
	else n2=n2-'A'+10;

	if (n1 <= '9') n1-='0';
	else n1=n1-'A'+10;

	return n2*16+n1;

}
static NMEA_status NMEA_checksum_clc(uint8_t * message){
	uint8_t index = 1;
	uint8_t checksum_clc =0;
	while (message[index]!='*'){
		checksum_clc^=message[index++];
	}

	uint8_t checksum = hx2int(message[index+1],message[index+2]);
	if (checksum!=checksum_clc){
		return NMEA_CHECKSUM_ERROR;
	}
	return NMEA_OK;


}

/*
 * NMEA_read_line is function that reads one NMEA message line to NMEA__working_buffer.
 */
static void NMEA_read_line(void){
	uint8_t index = 0;
	while (index < NMEA_WORKING_BUFFER_SIZE) NMEA__working_buffer[index++]=0;	// Clean up working buffer.

	index = 0;
	while(NMEA__UART_buffer[UART_buffer_tail]!='\n'){
		NMEA__working_buffer[index]=NMEA__UART_buffer[UART_buffer_tail];
		NMEA__UART_buffer[UART_buffer_tail] = 0;
		++UART_buffer_tail;
		++index;
	}
	NMEA__working_buffer[index]=NMEA__UART_buffer[UART_buffer_tail];
	++UART_buffer_tail;
	++index;
	--UART_buffer_lines;

}

/*
 * Library initialization
 */
void NMEA_init(UART_HandleTypeDef *huart){
	NMEA_huart=huart;
	HAL_UART_Receive_IT(NMEA_huart, &UART_char_tmp, 1);

	speed_change_CB_fun_ptr = &default_CB;
	speed_raise_barrier_CB_fun_ptr = &default_CB;
	speed_fall_barrier_CB_fun_ptr = &default_CB;
}

/*
 * add char to buffer
 */
NMEA_status NMEA_UART_get_char(void){
	uint8_t position = (UART_buffer_head + 1)%NMEA_UART_BUFFER_SIZE;
	NMEA_status stat=NMEA_OK;

	if (position == UART_buffer_tail){		//buffer overflowed! make space for new message
		while (NMEA__UART_buffer[UART_buffer_tail]!='\n'){//&&NMEA__UART_buffer[UART_buffer_tail]!=0){
			NMEA__UART_buffer[UART_buffer_tail]=0;
			UART_buffer_tail=(UART_buffer_tail + 1)%NMEA_UART_BUFFER_SIZE;
		}
		NMEA__UART_buffer[UART_buffer_tail]=0;
		UART_buffer_tail=(UART_buffer_tail + 1)%NMEA_UART_BUFFER_SIZE;
		stat=NMEA_BUFFER_OVERFLOWED;
	}

	NMEA__UART_buffer[UART_buffer_head]=UART_char_tmp;

	UART_buffer_head=position;

	if(UART_char_tmp=='\n'){
		++UART_buffer_lines;	//increment lines counter
	}

	HAL_UART_Receive_IT(NMEA_huart, &UART_char_tmp, 1);
	return stat;
}
/*
 * NMEA_process_task is the function that process all data.
 * You have to place in inside your main loop!
 */
NMEA_status NMEA_process_task(void){
	NMEA_status stat = NMEA_OK;
	while(UART_buffer_lines>0) {
		NMEA_read_line();
		NMEA_checksum_clc(NMEA__working_buffer);
		NMEA_parser((char *)NMEA__working_buffer);

	}
	return stat;
}








