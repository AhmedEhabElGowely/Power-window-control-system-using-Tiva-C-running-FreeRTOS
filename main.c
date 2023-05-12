/*
 ============================================================================
 Name        : main.c
 Description : Power window control system using Tiva C running FreeRTOS
 Date        : 15/05/2023
 ============================================================================
 */

/*The Library for this application*/
#include "Window.h"

/*Main Function*/
int main()
{
	/*Creating Queue*/
	Motor_Queue = xQueueCreate( 10, sizeof( state ) );
	/*Creating Mutex*/
	Motor_mutex = xSemaphoreCreateMutex();
	/*Creating Semaphores*/
	vSemaphoreCreateBinary( Driver_up_sem );
	vSemaphoreCreateBinary( Driver_down_sem );
	vSemaphoreCreateBinary( Passenger_up_sem );
	vSemaphoreCreateBinary( Passenger_down_sem );
	vSemaphoreCreateBinary( Lock_sem );
	vSemaphoreCreateBinary( Obstacle_sem );
	vSemaphoreCreateBinary( Limit_sem );
	
	/*Initialization Function*/
	init();
	/*Tasks Creation*/
	xTaskCreate (Lock_Window , "Lock" , 200 , NULL , 4 , NULL);
	xTaskCreate (Obstacle_detect , "Jam" , 200 , NULL , 3 , NULL);
	
	xTaskCreate (Limit_Task, "LimitTask" , 200 , NULL , 2 , NULL);
	
	xTaskCreate (Driver_Check_Up , "Driver1" , 200 , NULL , 1 , NULL);
	xTaskCreate (Driver_Check_Down , "Driver2" , 200 , NULL , 1 , NULL);	
	xTaskCreate (Passenger_Check_Up , "Pass1" , 200, NULL , 1 , NULL);
	xTaskCreate (Passenger_Check_Down , "Pass2" , 200 , NULL , 1 , NULL);
	
	xTaskCreate (Motor_Move , "MotorMove" , 200 , NULL , 1 , NULL);
	
	// Startup of the FreeRTOS scheduler.  The program should block here.  
	vTaskStartScheduler();
	// The following line should never be reached.  
	//Failure to allocate enough memory from the heap could be a reason.
	for (;;);	
}
