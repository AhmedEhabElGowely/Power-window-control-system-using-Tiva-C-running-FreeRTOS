/******************************************************************************
 *
 * Module: Window
 *
 * File Name: Window.h
 *
 * Description: Header file for the Passenger Window driver
 *
 *******************************************************************************/

#ifndef WINDOW_H_
#define WINDOW_H_

/*******************************************************************************
 *                          Libraries Included                                 *
 *******************************************************************************/

/*FreeRTOS standard Libraries*/
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>


/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/

/* ALL INPUTS ARE CONNECTED SUCH THAT PRESSED = CLEAR(0)   AND   UNPRESSED = SET(1)*/
/*Connected in PULL UP (Internal)*/

#define CONTROL_PORT 			GPIO_PORTF_BASE 
#define OBSTACLE_SWITCH 	GPIO_PIN_0	//IN 
#define LOCK_TOGGLE				GPIO_PIN_4	//IN 
#define MOTOR_UP_LED			GPIO_PIN_1	//OUT
#define MOTOR_DOWN_LED		GPIO_PIN_3	//OUT 

#define OBSTACLE_SWITCH_NUM 	0	//IN 
#define LOCK_TOGGLE_NUM				4	//IN 
#define MOTOR_UP_LED_NUM			1	//OUT 
#define MOTOR_DOWN_LED_NUM		3	//OUT 

#define MOTOR_PORT				GPIO_PORTD_BASE	
#define MOTOR_IN1					GPIO_PIN_2	//OUT 
#define MOTOR_IN2					GPIO_PIN_3	//OUT 

#define MOTOR_IN1_NUM					2	//OUT 
#define MOTOR_IN2_NUM					3	//OUT 

#define LIMIT_PORT				GPIO_PORTE_BASE
#define LIMIT_UP					GPIO_PIN_1	//IN 
#define LIMIT_DOWN				GPIO_PIN_2	//IN

#define LIMIT_UP_NUM					1	//IN
#define LIMIT_DOWN_NUM				2	//IN

#define DRIVER_PORT				GPIO_PORTB_BASE	
#define DRIVER_UP					GPIO_PIN_0	//IN
#define DRIVER_DOWN				GPIO_PIN_1	//IN

#define DRIVER_UP_NUM					0	//IN
#define DRIVER_DOWN_NUM				1	//IN

#define PASSENGER_PORT		GPIO_PORTA_BASE	
#define PASSENGER_UP			GPIO_PIN_6	//IN
#define PASSENGER_DOWN		GPIO_PIN_7	//IN

#define PASSENGER_UP_NUM			6	//IN
#define PASSENGER_DOWN_NUM		7	//IN

/*******************************************************************************
 *                          Type Definitions                                   *
 *******************************************************************************/

typedef enum 
{ 
    STOP, UP, DOWN ,AUTO_DOWN , AUTO_UP
} state;

/*******************************************************************************
 *                         Shared Global Variables                             *
 *******************************************************************************/

extern xSemaphoreHandle Driver_up_sem; 
extern xSemaphoreHandle Driver_down_sem; 
extern xSemaphoreHandle Passenger_up_sem; 
extern xSemaphoreHandle Passenger_down_sem; 
extern xSemaphoreHandle Lock_sem; 
extern xSemaphoreHandle Obstacle_sem;
extern xSemaphoreHandle Limit_sem;

extern xSemaphoreHandle Motor_mutex;

extern xQueueHandle Motor_Queue;


/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/


/*
 * Description :
 * Interrupt Handler for the Driver Buttons:
 * 1. Driver Up.
 * 2. Driver Down.
 */
static void Driver_ISR (void);

/*
 * Description :
 * Checks on the Driver Up button and moves the window accordingly.
 */
void Driver_Check_Up (void *pvParameters);

/*
 * Description :
 * Checks on the Driver Down button and moves the window accordingly.
 */
void Driver_Check_Down (void *pvParameters);

/*
 * Description :
 * Interrupt Handler for the Passenger Buttons:
 * 1. Passenger Up.
 * 2. Passenger Down.
 */
static void Passenger_ISR (void);

/*
 * Description :
 * Checks on the Passenger Up button and moves the window accordingly.
 */
void Passenger_Check_Up (void *pvParameters);

/*
 * Description :
 * Checks on the Passsenger Down button and moves the window accordingly.
 */
void Passenger_Check_Down (void *pvParameters);

/*
 * Description :
 * Responsible for moving window after recieving order in Queue from the check functions.
 */
void Motor_Move (void *pvParameters);

/*
 * Description :
 * Interrupt Handler for the Control Switches:
 * 1. Lock Toggle.
 * 2. Obstacle Detection.
 */
static void Control_ISR(void);
/*
 * Description :
 * Prevent the window from moving and disabling all Up and Down buttons.
 */
void Lock_Window (void *pvParameters);

/*
 * Description :
 * Prevent the window from moving upwards and retracts the window a little.
 */
void Obstacle_detect (void *pvParameters);

/*
 * Description :
 * Interrupt Handler for the Limit Switches:
 * 1. Limit Up.
 * 2. Limit Down.
 */
static void Limit_ISR (void);

/*
 * Description :
 * Prevent the window from moving when the window reaches the limit switch.
 */
void Limit_Task (void *pvParameters);

/*
 * Description :
 * Initialize The window Control system:
 * 1. Enable clock for GPIO ports.
 * 2. Specifying pins directions and configurations.
 * 3. Enabling Intterups on GPIO pins needed.
 * 4. Specify Interrupt Handler for each Port.
 */
void init (void);

/*
 * Description :
 * Idle function which puts the Microcontroller in Sleep mode when all other tasks are blocked.
 */
void vApplicationIdleHook( void );

#endif