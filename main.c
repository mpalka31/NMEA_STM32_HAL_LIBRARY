/*
 * main.c
 *
 *  Created on: Dec 13, 2020
 *      Author: Michał Pałka
 *
 * 	This is the example how to use libNMEA library.
 * 	It shows how to integrate the library with hour project.
 * 	IT IS NOT REDY TO COMPILE FILE!
 */
...
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "libNMEA.h"
/* USER CODE END Includes */
...
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == NMEA_huart) NMEA_UART_get_char();	/*add NMEA_UART_get_char() to IT routine*/
}

int time;	/*time is a acceleration measurement var*/
/*
 * stop_measure_speed_CB() is a callback function that ends measurement and unregister CB
 */
void stop_measure_speed_CB(void){
	time = nmea_data.UTC_time - time;
	NMEA_CB_unregister(SPEED_RISE_BARRIER_CB);
}
/*
* start_measure_speed_CB() is a callback function that starts measurement and register stop_measure_speed_CB() CB function
*/
void start_measure_speed_CB(void){
	time = nmea_data.UTC_time;
	NMEA_CB_register(&stop_measure_speed_CB, SPEED_RISE_BARRIER_CB, 60);
}
/* USER CODE END 0 */
...
int main(void)
{
  ...
  /* USER CODE BEGIN 2 */

  NMEA_init(&huart4);	/*library initialization. Pass the UART handler structure*/
  NMEA_CB_register(&start_measure_speed_CB, SPEED_RISE_BARRIER_CB, 10);	/*CB function registration*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  NMEA_process_task();	/*library process. PLACE IT IN THE MAIN INF LOOP! Note that you should handle the status codes.*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
...

