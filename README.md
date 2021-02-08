# NMEA STM32 HAL LIBRARY

libNMEA is a library created for the purpose of easy implementation of GPS features in your embedded system based on STM32 MCU's. It is compatible with GPS modules which are using NMEA protocol and UART communication. The library uses HAL functions so HAL must be included in your project. I used STM32CubeIDE to create this library and CubeMX to initialize MCU.

## Why you should consider this library

### Comparison to other similar libraries

I have seen a lot of similar libraries on the GitHub. Unfortunately all of them have some important issues like:

- Properly parsing only full message (with no empty fields)
- UART interrupt for every single incoming character 
- Lack of checksum control
- Lack of error/status codes to interact with oder parts of the software

### Key features

#### Parsing messages with empty fields

I have implemented a custom parser that can return a pointer to NULL value. Thanks to it every message is parsed properly even if it contains blank fields.
#### UART with DMA

Thanks for custom function detecting IDLE UART flag (not included in HAL) it was possible to implement data receiving with DMA. It generates only one IT for single transmission.
#### Checksum control

Special function is calculating checksum for every incoming NMEA message and compares it with the checksum given.
#### Status codes returning by the functions

Library functions are returning special status codes:

```C
typedef enum {
	NMEA_OK=0,		/**< operation success*/
	NMEA_BUFFER_OVERFLOWED,	/**< circular buffer overflowed*/
	NMEA_CHECKSUM_ERROR,	/**< invalid data received*/
	NMEA_WRONG_DATA,	/**< wrong data passed by user (of the library)*/
	NMEA_WRONG_CB_ID,	/**< invalid CB ID*/
	NMEA_ERROR		/**< something went wrong*/

}NMEA_status;
```

#### Callback functions for events

I have implemented a system of callback functions for some events. Using it you can realize measurements, warnings etc. At the moment you can register callbacks for 3 events:
- speed change
- upward speed limit violation
- downward speed limit violation

For more information see code documentation.

#### All data close at hand

nmea_data is instance of NMEA_data structer. You have acces to it in your code to acces important and actual data whenever you need.
```C
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

NMEA_data nmea_data;
```
For more information see code documentation.

## Integrating library with your code

In CubeMX follow these steps:
- Select UART/USART port for your GPS module
- Select async mode
- Turn on global interrupt
- Select DMA channel for UART/USART RX
- Generate Code

In the "libNMEA.h" file change 

```C
#include "stm32l4xx_hal.h"
```

to file specific to your hardware.

In the "main.h" add

```C
#include "libNMEA.h"
```

in user includes section

In file "stm32XXxx_it.c" specific for your hardware place

```C
void UART4_IRQHandler(void)
{
	//USER CODE BEGIN UART4_IRQn 0
	
	user_UART_IDLE_IT_handler();
	
	//USER CODE END UART4_IRQn 0

  	...
```
In the main function after peripheria initialization place

```C
  NMEA_init(&huart4, &hdma_uart4_rx);	/*library initialization. Pass the UART and DMA handler structures*/
```

with handlers to defined structures and

```C
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  NMEA_process_task();	/*library process. PLACE IT IN THE MAIN LOOP! Note that you should handle the status codes.*/
```

inside the infinitive loop.

Thats it! Now you are ready to use GPS features in your embedded system!

Check out the example "main.c" to understand how to play with callback functions.

I highly recommend to generate and read the documentation when you clone the repository using Doxygen and read the source before integration with the system process. 

For more complex example check out [this project](https://github.com/mpalka31/NMEA_test)

