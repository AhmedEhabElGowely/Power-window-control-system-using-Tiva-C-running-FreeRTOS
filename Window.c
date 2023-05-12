/******************************************************************************
 *
 * Module: Window
 *
 * File Name: Window.h
 *
 * Description: Source file for the Passenger Window driver
 *
 *******************************************************************************/
 
/*******************************************************************************
 *                          Libraries Included                                 *
 *******************************************************************************/
/* C standard Libraries*/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/*Tivaware Libraries*/
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

/*Common Macros used for bitwise operations*/
#include "bitwise_operation.h"

#include "Window.h"

/*******************************************************************************
 *                         Shared Global Variables                             *
 *******************************************************************************/
/*Creating Semaphore Handles*/
 xSemaphoreHandle Driver_up_sem; 
 xSemaphoreHandle Driver_down_sem; 
 xSemaphoreHandle Passenger_up_sem; 
 xSemaphoreHandle Passenger_down_sem; 
 xSemaphoreHandle Lock_sem; 
 xSemaphoreHandle Obstacle_sem;
 xSemaphoreHandle Limit_sem;
/*Creating the mutex handle*/
 xSemaphoreHandle Motor_mutex;
/*CReating Queue Handle*/
 xQueueHandle Motor_Queue;
 
/*******************************************************************************
 *                         Private Global Variables                            *
 *******************************************************************************/
unsigned char Control_flag = 0 , Change_flag_down = 0 , Change_flag_up = 0;

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/
/*
 * Description :
 * Interrupt Handler for the Driver Buttons:
 * 1. Driver Up.
 * 2. Driver Down.
 */
void Driver_ISR (void)
{
	/*Variable which will be set to true if the task woken by semaohore is Equal
	or Higher priority than the task that was interrupted */
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/*checking which Button gave the interrupt*/
	if ( BIT_IS_CLEAR(GPIOPinRead(DRIVER_PORT,DRIVER_UP) , DRIVER_UP_NUM) )
	{
		/* Give Semaphore to the corresponding task */
		xSemaphoreGiveFromISR( Driver_up_sem, &xHigherPriorityTaskWoken);
		/* Clearing Interrupt Flag */
		GPIOIntClear(DRIVER_PORT, DRIVER_UP);
	}
	/*checking which Button gave the interrupt*/
	else if ( BIT_IS_CLEAR(GPIOPinRead(DRIVER_PORT,DRIVER_DOWN) , DRIVER_DOWN_NUM) )
	{
		/* Give Semaphore to the corresponding task */
		xSemaphoreGiveFromISR( Driver_down_sem, &xHigherPriorityTaskWoken);
		/* Clearing Interrupt Flag */
		GPIOIntClear(DRIVER_PORT, DRIVER_DOWN);		
	}
	/* Clearing Interrupt Flag */
	GPIOIntClear(DRIVER_PORT, DRIVER_UP);
	GPIOIntClear(DRIVER_PORT, DRIVER_DOWN);
	/* Forcing a context switch if the task woken by semaphore 
	is higher or equal in priority that the task that was interrupted */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*
 * Description :
 * Checks on the Driver Up button and moves the window accordingly.
 */
void Driver_Check_Up (void *pvParameters)
{
	/* Variable which is sent in the queue*/
	state Motor_state = STOP;
	/*Local flags*/
	uint8_t  Driver_flag_up = 0 , prevState_up = 0 , sem_flag = 0;
	for(;;)
	{
		/*Ensuring that once the button is pressed and the semaphore is taken,
		the task will not be blocked on the semaphore again until the button is released.
		Taking the Mutex which ensures that only one button controls the motor at a time*/
		if (sem_flag == 0)
		{
			xSemaphoreTake( Driver_up_sem, portMAX_DELAY );
			xSemaphoreTake( Motor_mutex, portMAX_DELAY );
			sem_flag = 1;
		}
		else
		{
			/* Debouncing*/
		if ( BIT_IS_CLEAR(GPIOPinRead(DRIVER_PORT,DRIVER_UP) , DRIVER_UP_NUM) )
		{
			/* Debouncing*/
			vTaskDelay( 30/portTICK_RATE_MS);
			if ( BIT_IS_CLEAR(GPIOPinRead(DRIVER_PORT,DRIVER_UP) , DRIVER_UP_NUM) )
			{
				/*Enter here only once while the button is pressed*/
				if (Driver_flag_up == 0)
				{
					/*Flag used to override the control in case of Auto Down*/
					Change_flag_up = 1;
					/*Sending the data to the queue*/
					Motor_state = UP;
					xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
					/*Set the button flag value to 1 to not enter here again until the button is released.*/
					Driver_flag_up = 1;
					/*Flag which used to check for automatic operation*/
					prevState_up = 1;
					/*Delaying 1 second before checking again (For Automatic mode)*/
					vTaskDelay( 1000/portTICK_RATE_MS);
				}
				else 
				{
					/*Button is still pressed after 1 second (Manual Mode)*/
					prevState_up = 0;
				}
			}
		}
		/*Checking for automatic mode*/
		else if( ( BIT_IS_SET(GPIOPinRead(DRIVER_PORT,DRIVER_UP) , DRIVER_UP_NUM) ) &&  prevState_up )
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Motor_state = AUTO_UP;
			prevState_up = 0;
			Driver_flag_up = 0;
			sem_flag = 0;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}
		/*Manual Mode Ending*/
		else 
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Motor_state = STOP;
			sem_flag = 0;
			Driver_flag_up = 0;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}			
	}
}
}

/*
 * Description :
 * Checks on the Driver Down button and moves the window accordingly.
 */
void Driver_Check_Down (void *pvParameters)
{
	/* Variable which is sent in the queue*/
	state Motor_state = STOP;
	/*Local flags*/
	uint8_t Driver_flag_down = 0 , prevState_down = 0 , sem_flag = 0;;
	for(;;)
	{
		/*Ensuring that once the button is pressed and the semaphore is taken,
		the task will not be blocked on the semaphore again until the button is released.
		Taking the Mutex which ensures that only one button controls the motor at a time*/
		if (sem_flag == 0)
		{
			xSemaphoreTake( Driver_down_sem, portMAX_DELAY );
			xSemaphoreTake( Motor_mutex, portMAX_DELAY );
			sem_flag = 1;
		}
		else
		{
			/* Debouncing*/
		if ( BIT_IS_CLEAR(GPIOPinRead(DRIVER_PORT,DRIVER_DOWN) , DRIVER_DOWN_NUM) )
		{
			/* Debouncing*/
			vTaskDelay( 30/portTICK_RATE_MS);
			if ( BIT_IS_CLEAR(GPIOPinRead(DRIVER_PORT,DRIVER_DOWN) , DRIVER_DOWN_NUM) )
			{
				/*Enter here only once while the button is pressed*/
				if (Driver_flag_down == 0)
				{
					/*Flag used to override the control in case of Auto Up*/
					Change_flag_down = 1;
					/*Sending the data to the queue*/
					Motor_state = DOWN;
					xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
					/*Set the button flag value to 1 to not enter here again until the button is released.*/
					Driver_flag_down = 1;
					/*Flag which used to check for automatic operation*/
					prevState_down = 1;
					/*Delaying 1 second before checking again (For Automatic mode)*/
					vTaskDelay( 1000/portTICK_RATE_MS);
				}
				else 
				{
					/*Button is still pressed after 1 second (Manual Mode)*/
					prevState_down = 0;
				}
			}
		}
		/*Checking for automatic mode*/
		else if( ( BIT_IS_SET(GPIOPinRead(DRIVER_PORT,DRIVER_DOWN) , DRIVER_DOWN_NUM) ) &&  prevState_down )
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Motor_state = AUTO_DOWN;
			prevState_down = 0;
			Driver_flag_down = 0;
			sem_flag = 0;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}
		/*Manual Mode Ending*/
		else 
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Motor_state = STOP;
			sem_flag = 0;
			Driver_flag_down = 0;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}		
	}
}
}

/*
 * Description :
 * Interrupt Handler for the Passenger Buttons:
 * 1. Passenger Up.
 * 2. Passenger Down.
 */
void Passenger_ISR (void)
{
	/*Variable which will be set to true if the task woken by semaohore is Equal
	or Higher priority than the task that was interrupted */
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	/*checking which Button gave the interrupt*/
	if ( BIT_IS_CLEAR(GPIOPinRead(PASSENGER_PORT,PASSENGER_UP) , PASSENGER_UP_NUM) )
	{
		/* Give Semaphore to the corresponding task */
		xSemaphoreGiveFromISR( Passenger_up_sem, &xHigherPriorityTaskWoken);
		/* Clearing Interrupt Flag */
		GPIOIntClear(PASSENGER_PORT,PASSENGER_UP);
	}
	/*checking which Button gave the interrupt*/
	else if ( BIT_IS_CLEAR(GPIOPinRead(PASSENGER_PORT,PASSENGER_DOWN) , PASSENGER_DOWN_NUM) )
	{
		/* Give Semaphore to the corresponding task */
		xSemaphoreGiveFromISR( Passenger_down_sem, &xHigherPriorityTaskWoken);
		/* Clearing Interrupt Flag */
		GPIOIntClear(PASSENGER_PORT,PASSENGER_DOWN);		
	}
	/* Clearing Interrupt Flag */
	GPIOIntClear(PASSENGER_PORT,PASSENGER_UP);
	GPIOIntClear(PASSENGER_PORT,PASSENGER_DOWN);
	/* Forcing a context switch if the task woken by semaphore 
	is higher or equal in priority that the task that was interrupted */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*
 * Description :
 * Checks on the Passenger Up button and moves the window accordingly.
 */
void Passenger_Check_Up (void *pvParameters)
{
	/* Variable which is sent in the queue*/
	state Motor_state = STOP;
	/*Local flags*/
	uint8_t Passenger_flag_up = 0 , prevState_up = 0, sem_flag = 0 ;	
	for(;;)
	{
		/*Ensuring that once the button is pressed and the semaphore is taken,
		the task will not be blocked on the semaphore again until the button is released.
		Taking the Mutex which ensures that only one button controls the motor at a time*/
		if (sem_flag == 0)
		{
			xSemaphoreTake( Passenger_up_sem, portMAX_DELAY );
			xSemaphoreTake( Motor_mutex, portMAX_DELAY );
			sem_flag = 1;
		}
		else
		{
			/* Debouncing*/
		if ( BIT_IS_CLEAR(GPIOPinRead(PASSENGER_PORT,PASSENGER_UP) , PASSENGER_UP_NUM) )
		{
			/* Debouncing*/
			vTaskDelay( 30/portTICK_RATE_MS);
			if ( BIT_IS_CLEAR(GPIOPinRead(PASSENGER_PORT,PASSENGER_UP) , PASSENGER_UP_NUM) )
			{
				/*Enter here only once while the button is pressed*/
				if (Passenger_flag_up == 0)
				{
					/*Flag used to override the control in case of Auto Down*/
					Change_flag_up = 1;
					/*Sending the data to the queue*/
					Motor_state = UP;
					xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
					/*Set the button flag value to 1 to not enter here again until the button is released.*/
					Passenger_flag_up = 1;
					/*Flag which used to check for automatic operation*/
					prevState_up = 1;
					/*Delaying 1 second before checking again (For Automatic mode)*/
					vTaskDelay( 1000/portTICK_RATE_MS);
				}
				else 
				{
					/*Button is still pressed after 1 second (Manual Mode)*/
					prevState_up = 0;
				}
			}
		}
		/*Checking for automatic mode*/
		else if( ( BIT_IS_SET(GPIOPinRead(PASSENGER_PORT,PASSENGER_UP) , PASSENGER_UP_NUM) ) &&  prevState_up )
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Motor_state = AUTO_UP;
			prevState_up = 0;
			sem_flag = 0;
			Passenger_flag_up = 0;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}
		/*Manual Mode Ending*/
		else 
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Passenger_flag_up = 0;
			sem_flag = 0;
			Motor_state = STOP;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}		
	}
}
}
/*
 * Description :
 * Checks on the Passsenger Down button and moves the window accordingly.
 */
void Passenger_Check_Down (void *pvParameters)
{
	/* Variable which is sent in the queue*/
	state Motor_state = STOP;
	/*Local flags*/
	uint8_t Passenger_flag_down = 0 , prevState_down = 0 , sem_flag = 0;
	for(;;)
	{
		/*Ensuring that once the button is pressed and the semaphore is taken,
		the task will not be blocked on the semaphore again until the button is released.
		Taking the Mutex which ensures that only one button controls the motor at a time*/
		if (sem_flag == 0)
		{
			xSemaphoreTake( Passenger_down_sem, portMAX_DELAY );
			xSemaphoreTake( Motor_mutex, portMAX_DELAY );
			sem_flag = 1;
		}
		else
		{
			/* Debouncing*/
		if ( BIT_IS_CLEAR(GPIOPinRead(PASSENGER_PORT, PASSENGER_DOWN) , PASSENGER_DOWN_NUM) )
		{
			/* Debouncing*/
			vTaskDelay( 30/portTICK_RATE_MS);
			if ( BIT_IS_CLEAR(GPIOPinRead(DRIVER_PORT, PASSENGER_DOWN) , PASSENGER_DOWN_NUM) )
			{
				/*Enter here only once while the button is pressed*/
				if (Passenger_flag_down == 0)
				{
					/*Flag used to override the control in case of Auto Up*/
					Change_flag_down = 1;
					/*Sending the data to the queue*/
					Motor_state = DOWN;
					xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
					/*Set the button flag value to 1 to not enter here again until the button is released.*/
					Passenger_flag_down = 1;
					/*Flag which used to check for automatic operation*/
					prevState_down = 1;
					/*Delaying 1 second before checking again (For Automatic mode)*/
					vTaskDelay( 1000/portTICK_RATE_MS);
				}
				else 
				{
					/*Button is still pressed after 1 second (Manual Mode)*/
					prevState_down = 0;
				}
			}
		}
		/*Checking for automatic mode*/
		else if( ( BIT_IS_SET(GPIOPinRead(PASSENGER_PORT,PASSENGER_DOWN) , PASSENGER_DOWN_NUM) ) &&  prevState_down )
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Motor_state = AUTO_DOWN;
			prevState_down = 0;
			sem_flag = 0;
			Passenger_flag_down = 0;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}
		/*Manual Mode Ending*/
		else 
		{
			/*Sending data to motor queue and clearing all flags and giving the Mutex*/
			Motor_state = STOP;
			Passenger_flag_down = 0;
			sem_flag = 0;
			xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
			xSemaphoreGive(Motor_mutex);
		}
	}
}
}

/*
 * Description :
 * Responsible for moving window after recieving order in Queue from the check functions.
 */
void Motor_Move (void *pvParameters)
{
	/* Variable which recieves data from queue*/
	state Motor_state = STOP;
	for(;;)
	{
		/*Recieving data from queue*/
		xQueueReceive( Motor_Queue, &Motor_state, portMAX_DELAY );
	/*Checking if the data is STOP*/
	if (Motor_state == STOP)
	{
		/*Stopping the motor and turn off the leds*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0x00 );
	}
	/*Checking if the data is Down*/
	else if ( Motor_state == DOWN )
	{	
		/* Chcecking if the down limit switch is pressed*/
		if ( BIT_IS_CLEAR(GPIOPinRead(LIMIT_PORT,LIMIT_DOWN) , LIMIT_DOWN_NUM) )
		{
			/*Stopping the motor and turn off the leds*/
			GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
			GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0x00 );
		}
		else
		{
			/*Moving Motor in the Downward Direction and turining on the down led*/
			/*Clearing the overide flag*/
			Change_flag_up = 0 ;
			GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , MOTOR_IN1 );
			GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , MOTOR_DOWN_LED );
		}
	}
	/*Checking if the data is UP and Ensuring that there is no obstacle*/
	else if ( Motor_state == UP && Control_flag == 0 )
	{
		/* Chcecking if the down limit switch is pressed*/
		if ( BIT_IS_CLEAR(GPIOPinRead(LIMIT_PORT,LIMIT_UP) , LIMIT_UP_NUM) ) 
		{
			/*Stopping the motor and turn off the leds*/
			GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
			GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0x00 );
		}
		else 
		{
			/*Moving Motor in the Upward Direction and turining on the down led*/
			/*Clearing the overide flag*/
			Change_flag_down = 0 ;
			GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , MOTOR_IN2 );
			GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , MOTOR_UP_LED );
		}
	}
	/*Checking if the data is Auto down*/
	else if ( Motor_state == AUTO_DOWN )
	{
		/*As long as the limit is not reached Move down and No obstacle and no overide*/
		while ( BIT_IS_SET(GPIOPinRead(LIMIT_PORT,LIMIT_DOWN) , LIMIT_DOWN_NUM) && Control_flag == 0 && Change_flag_up == 0) 
		{
			/*Moving Motor in the Downward Direction and turining on the down led*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , MOTOR_IN1 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , MOTOR_DOWN_LED );
		}
		/*When switch is reached Stop motor and clear all flags*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0x00 );
		Control_flag = 0;
		Change_flag_up = 0 ;
	}	
	/*Checking if the data is Auto Up*/
	else if ( Motor_state == AUTO_UP  )
	{
		/*As long as the limit is not reached Move Up and No obstacle and no overide*/
		while ( BIT_IS_SET(GPIOPinRead(LIMIT_PORT,LIMIT_UP) , LIMIT_UP_NUM) && Control_flag == 0 && Change_flag_down == 0) 
		{
			/*Moving Motor in the Upward Direction and turining on the down led*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , MOTOR_IN2 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , MOTOR_UP_LED );
		}
		/*Clear override flag*/
		Change_flag_down = 0 ;
	}	
}	
}

/*
 * Description :
 * Interrupt Handler for the Control Switches:
 * 1. Lock Toggle.
 * 2. Obstacle Detection.
 */
void Control_ISR(void)
{
	/*Variable which will be set to true if the task woken by semaohore is Equal
	or Higher priority than the task that was interrupted */
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	/*checking which Button gave the interrupt*/
	if ( BIT_IS_CLEAR(GPIOPinRead(CONTROL_PORT,LOCK_TOGGLE) , LOCK_TOGGLE_NUM) )
	{
		/*Setting the flag which used to exit the Automatic Operation*/
		Control_flag = 1;
		/* Give Semaphore to the corresponding task */
		xSemaphoreGiveFromISR( Lock_sem, &xHigherPriorityTaskWoken);
		/* Clearing Interrupt Flag */
		GPIOIntClear(CONTROL_PORT,LOCK_TOGGLE);
	}
	/*checking which Button gave the interrupt*/
	else if( BIT_IS_CLEAR(GPIOPinRead(CONTROL_PORT,OBSTACLE_SWITCH) , OBSTACLE_SWITCH_NUM) )
	{
		/*Setting the flag which used to exit the Automatic Operation*/
		Control_flag = 1;
		/* Give Semaphore to the corresponding task */
		xSemaphoreGiveFromISR( Obstacle_sem , &xHigherPriorityTaskWoken);
		/* Clearing Interrupt Flag */
		GPIOIntClear(CONTROL_PORT,OBSTACLE_SWITCH);
	}
	/* Clearing Interrupt Flag */
	GPIOIntClear(CONTROL_PORT,LOCK_TOGGLE);
	GPIOIntClear(CONTROL_PORT,OBSTACLE_SWITCH);
	/* Forcing a context switch if the task woken by semaphore 
	is higher or equal in priority that the task that was interrupted */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*
 * Description :
 * Prevent the window from moving and disabling all Up and Down buttons.
 */
void Lock_Window (void *pvParameters)
{
	/* Variable which is sent in the queue*/
	state Motor_state = STOP;
	for(;;)
	{
		/* Taking the semaphore to unblock the task*/
		xSemaphoreTake(Lock_sem , portMAX_DELAY );
		/*Reseting the queue to ensure it is empty after exiting this task*/
		xQueueReset( Motor_Queue );
		/*Sending STOP to ensure that the motor is stopped when exiting the task*/
		xQueueSendToBack( Motor_Queue , &Motor_state, 0 );
		/*Stopping the motor and turning off the leds*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0x00 );
		vTaskDelay (100/portTICK_RATE_MS);
		/*While the button is pressed put microcontroller in sleep mode*/
		while ( BIT_IS_CLEAR(GPIOPinRead(CONTROL_PORT,LOCK_TOGGLE) , LOCK_TOGGLE_NUM) )
		{
			__WFI();
		}
		/*Clearing th control flag*/
		Control_flag = 0;
	}
}

/*
 * Description :
 * Prevent the window from moving upwards and retracts the window a little.
 */
void Obstacle_detect (void *pvParameters)
{
	for(;;)
	{
		/* Taking the semaphore to unblock the task*/
		xSemaphoreTake(Obstacle_sem , portMAX_DELAY );
		/*Reseting the queue to ensure it is empty after exiting this task*/
		xQueueReset( Motor_Queue );
		/*Checking if Limit switch is pressed*/
		if ( BIT_IS_SET(GPIOPinRead(LIMIT_PORT,LIMIT_DOWN) , LIMIT_DOWN_NUM) )
		{
			/*Moving window downward for 0.5 seconds*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , MOTOR_IN1 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , MOTOR_DOWN_LED );
		vTaskDelay (500/portTICK_RATE_MS);
		}
		else
		{
			/*Stopping the motor if limit switch is reached*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0xFF );
		vTaskDelay (500/portTICK_RATE_MS);
		}	
		/*Stopping the motor and clearing the control flag*/
		GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
		GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0x00);
		Control_flag = 0;
	}
}

/*
 * Description :
 * Interrupt Handler for the Limit Switches:
 * 1. Limit Up.
 * 2. Limit Down.
 */
void Limit_ISR (void)
{
	/*Variable which will be set to true if the task woken by semaohore is Equal
	or Higher priority than the task that was interrupted */
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	/* Give Semaphore to the corresponding task */
	xSemaphoreGiveFromISR( Limit_sem , &xHigherPriorityTaskWoken);
	/* Clearing Interrupt Flag */
	GPIOIntClear(LIMIT_PORT,LIMIT_UP);
	GPIOIntClear(LIMIT_PORT,LIMIT_DOWN);
	/* Forcing a context switch if the task woken by semaphore 
	is higher or equal in priority that the task that was interrupted */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

/*
 * Description :
 * Prevent the window from moving when the window reaches the limit switch.
 */
void Limit_Task (void *pvParameters)
{
	for(;;)
	{
		/* Taking the semaphore to unblock the task*/
	xSemaphoreTake(Limit_sem , portMAX_DELAY );
		/*Stopping the motor and clearing the control flag*/
	GPIOPinWrite(MOTOR_PORT , (MOTOR_IN1 | MOTOR_IN2) , 0x00 );
	GPIOPinWrite(CONTROL_PORT , (MOTOR_UP_LED | MOTOR_DOWN_LED) , 0x00);
	vTaskDelay (20/portTICK_RATE_MS);
	Control_flag = 0;
	}
}

/*
 * Description :
 * Initialize The window Control system:
 * 1. Enable clock for GPIO ports.
 * 2. Specifying pins directions and configurations.
 * 3. Enabling Intterups on GPIO pins needed.
 * 4. Specify Interrupt Handler for each Port.
 */
void init (void)
{
	/*Enable CLock*/
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
	/*Unlock Pins*/
GPIOUnlockPin(CONTROL_PORT  ,  OBSTACLE_SWITCH | LOCK_TOGGLE | MOTOR_UP_LED | MOTOR_DOWN_LED );
	/*Specifying pins directions and configurations*/
GPIOPinTypeGPIOInput(CONTROL_PORT , OBSTACLE_SWITCH | LOCK_TOGGLE);
GPIOPadConfigSet(CONTROL_PORT , ( OBSTACLE_SWITCH | LOCK_TOGGLE ) ,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
GPIOPinTypeGPIOOutput(CONTROL_PORT , MOTOR_UP_LED | MOTOR_DOWN_LED);
GPIOPinWrite(CONTROL_PORT,(MOTOR_UP_LED | MOTOR_DOWN_LED), 0x00 );
	/*Enabling interrupts for pins and setting their configurations and priorities and specifying the ISR for the interrupt*/
GPIOIntRegister(CONTROL_PORT  ,Control_ISR);
IntPrioritySet( INT_GPIOF_TM4C123 , 0x50);
GPIOIntEnable(CONTROL_PORT , ( OBSTACLE_SWITCH | LOCK_TOGGLE  ));
GPIOIntTypeSet(CONTROL_PORT ,( OBSTACLE_SWITCH | LOCK_TOGGLE ),GPIO_FALLING_EDGE);
	
	/*Enable CLock*/
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
	/*Unlock Pins*/
GPIOUnlockPin(MOTOR_PORT  ,  MOTOR_IN1 | MOTOR_IN2);
	/*Specifying pins directions and configurations*/
GPIOPinTypeGPIOOutput(MOTOR_PORT , MOTOR_IN1 | MOTOR_IN2);
GPIOPinWrite(MOTOR_PORT,(MOTOR_IN1 | MOTOR_IN2), 0x00 );
	
	/*Enable CLock*/
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
/*Unlock Pins*/
GPIOUnlockPin(LIMIT_PORT  ,  LIMIT_UP | LIMIT_DOWN);
/*Specifying pins directions and configurations*/
GPIOPinTypeGPIOInput(LIMIT_PORT , LIMIT_UP | LIMIT_DOWN);
GPIOPadConfigSet(LIMIT_PORT , ( LIMIT_UP | LIMIT_DOWN ) ,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
/*Enabling interrupts for pins and setting their configurations and priorities and specifying the ISR for the interrupt*/
GPIOIntRegister(LIMIT_PORT ,Limit_ISR);
IntPrioritySet( INT_GPIOE_TM4C123 , 0xD0);
GPIOIntEnable(LIMIT_PORT , ( LIMIT_UP | LIMIT_DOWN )); 
GPIOIntTypeSet(LIMIT_PORT ,( LIMIT_UP | LIMIT_DOWN ),GPIO_FALLING_EDGE); 

/*Enable CLock*/
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
/*Unlock Pins*/
GPIOUnlockPin(DRIVER_PORT  ,  DRIVER_UP | DRIVER_DOWN);
/*Specifying pins directions and configurations*/
GPIOPinTypeGPIOInput(DRIVER_PORT , DRIVER_UP | DRIVER_DOWN);
GPIOPadConfigSet(DRIVER_PORT , ( DRIVER_UP | DRIVER_DOWN ) ,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
/*Enabling interrupts for pins and setting their configurations and priorities and specifying the ISR for the interrupt*/
GPIOIntRegister(DRIVER_PORT ,Driver_ISR);
IntPrioritySet( INT_GPIOB_TM4C123 , 0xE0);
GPIOIntEnable(DRIVER_PORT , ( DRIVER_UP | DRIVER_DOWN ));
GPIOIntTypeSet(DRIVER_PORT ,( DRIVER_UP | DRIVER_DOWN ),GPIO_FALLING_EDGE); 

/*Enable CLock*/
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
/*Unlock Pins*/
GPIOUnlockPin(PASSENGER_PORT  ,  PASSENGER_UP | PASSENGER_DOWN);
/*Specifying pins directions and configurations*/
GPIOPinTypeGPIOInput(PASSENGER_PORT , PASSENGER_UP | PASSENGER_DOWN);
GPIOPadConfigSet(PASSENGER_PORT , ( PASSENGER_UP | PASSENGER_DOWN ) ,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
/*Enabling interrupts for pins and setting their configurations and priorities and specifying the ISR for the interrupt*/
GPIOIntRegister(PASSENGER_PORT ,Passenger_ISR);
IntPrioritySet( INT_GPIOA_TM4C123 , 0xE0);
GPIOIntEnable(PASSENGER_PORT , ( PASSENGER_UP | PASSENGER_DOWN )); 
GPIOIntTypeSet(PASSENGER_PORT ,( PASSENGER_UP | PASSENGER_DOWN ),GPIO_FALLING_EDGE); 
}
/*
 * Description :
 * Idle function which puts the Microcontroller in Sleep mode when all other tasks are blocked.
 */

void vApplicationIdleHook( void )
{
	/*SLEEP MODE*/
	__WFI();
}
