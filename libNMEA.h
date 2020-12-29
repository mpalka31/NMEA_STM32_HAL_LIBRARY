/*
 * libNMEA.h
 *
 *  Created on: Nov 23, 2020
 *      Author: Michał Pałka
 *
 * 	This is the header file of the library created to handling communication with GPS devices by NMEA protocol.
 * 	It is created for STM32 MCUs family.
 */

#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>
/*
 * Circular buffer size definition
 */
#define NMEA_UART_BUFFER_SIZE		256
/*
 * Working buffer size definition
 */
#define NMEA_WORKING_BUFFER_SIZE	128
/*
 * Pointer to UART handler structure using to communication with GPS module.
 */
UART_HandleTypeDef *NMEA_huart;
/*
 * Definition of ERROR codes (NMEA_status).
 *
 * MNEA_OK 					-	operation success
 * MNEA_BUFFER_OVERFLOWED	-	circular buffer overflowed
 * NMEA_ERROR				-	something went wrong
 *
 * More specific codes in the future.
 */
 typedef enum {
	NMEA_OK=0,
	NMEA_BUFFER_OVERFLOWED,
	NMEA_CHECKSUM_ERROR,
	NMEA_WRONG_DATA,
	NMEA_ERROR

}NMEA_status;

/*
 * NMEA_data is a structure caring all useful received data.
 */
typedef struct{
	float		UTC_time;
	int			UT_date;

	float 		latitude;
	char 		latitude_direction;
	float 		longitude;
	char 		longitude_direction;
	float 		altitude;
	float		geoidal_separation;

	float		speed_kmph;
	float		speed_knots;

	uint8_t		sat_in_view;
	uint8_t		sat_in_use;
	uint8_t		fix;
	uint8_t		fix_mode;
	float		PDOP;
	float		HDOP;
	float		VDOP;

}NMEA_data;
/*
 * NMEA_data instantion declaration.
 */
NMEA_data nmea_data;
/*
 * NMEA_init is a function to initialize library.
 */
void NMEA_init(UART_HandleTypeDef *huart);
/*
 * NMEA_get_UART_char is a function that receives one data byte from UART.
 * You have to place it in UART interrupt routine.
 */
NMEA_status NMEA_UART_get_char(void);
NMEA_status NMEA_process_task(void);
NMEA_status NMEA_speed_CB_register(void (*speed_CB)(void),float speed_bar);
void NMEA_speed_CB_unregister(void);

















