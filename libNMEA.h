/*
 * libNMEA.h
 *
 *  Created on: Nov 23, 2020
 *      Author: Michał Pałka
 *
 * 	This is the header file of the library created to handling communication with GPS devices by NMEA protocol.
 * 	It is created for STM32 MCUs family.
 * 	It should be included in "main.h" file
 */

#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include <string.h>
/*
 * UART DMA buffer size definition
 */
#define NMEA_UART_DMA_BUFFER_SIZE	512
/*
 * Circular buffer size definition
 */
#define NMEA_UART_BUFFER_SIZE		1024
/*
 * Working buffer size definition
 */
#define NMEA_WORKING_BUFFER_SIZE	128
/*
 * Pointer to UART handler structure using to communication with GPS module.
 */
UART_HandleTypeDef	*NMEA_huart;
/*
 * Pointer to DMA handler structure using to receive messages from GPS module.
 */
DMA_HandleTypeDef	*NMEA_DMA;

/*
 * Definition of ERROR codes (NMEA_status).
 *
 * MNEA_OK 					-	operation success
 * MNEA_BUFFER_OVERFLOWED	-	circular buffer overflowed
 * NMEA_CHECKSUM_ERROR		-	invalid data received
 * NMEA_WRONG_DATA			-	wrong data passed by user (of the library)
 * NMEA_WRONG_CB_ID			-	invalid CB ID
 * NMEA_ERROR				-	something went wrong
 *
 * More specific codes in the future.
 */
typedef enum {
	NMEA_OK=0,
	NMEA_BUFFER_OVERFLOWED,
	NMEA_CHECKSUM_ERROR,
	NMEA_WRONG_DATA,
	NMEA_WRONG_CB_ID,
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
 * May be?
 */
typedef enum {
	SPEED_CHANGE_CB=0,
	SPEED_RISE_BARRIER_CB,
	SPEED_FALL_BARRIER_CB
}NMEA_CB_ID;

/*
 * NMEA_data instantion declaration.
 */
NMEA_data nmea_data;
/*
 * NMEA_init is a function to initialize library.
 */
void NMEA_init(UART_HandleTypeDef *huart,DMA_HandleTypeDef	*DMA);
/*
 * NMEA_get_UART_char is a function that receives one data byte from UART.
 * You have to place it in UART interrupt routine.
 */
//NMEA_status NMEA_UART_get_char(void);
NMEA_status NMEA_UART_DMA_copy_buffer(void);
NMEA_status NMEA_process_task(void);
NMEA_status NMEA_CB_register(void (*CB_fun)(void),NMEA_CB_ID CB_id,float barier);
NMEA_status NMEA_CB_unregister(NMEA_CB_ID CB_id);

NMEA_status user_UART_IDLE_IT_handler(void);












