//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "timers.h"
#include "semphr.h"
//******************************************************************************

/*
*
*/
#define BUTTON_PRIORITY		1 
#define DEBOUNCE_DELAY	( 100 / portTICK_RATE_MS )
#define DOUBLE_CLICK_TIME ( 250 / portTICK_RATE_MS)

void vLedBlinkBlue(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vLedBlinkGreen(void *pvParameters);
void vLedBlinkOrange(void *pvParameters);
//void Delay(uint32_t val);


int currLed = 0;
int leds[] = {LED_GREEN, LED_ORANGE, LED_RED, LED_BLUE};
int click_count = 0;
TimerHandle_t xTimers[2];
SemaphoreHandle_t xDebounceLock;
SemaphoreHandle_t xLEDCycleLock;

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

void vButtonDebounce( TimerHandle_t xTimer )
 {
	 //Incriment the click count since this was a legitemate button press
	 click_count++;
	 
	 //Once the timer resets, give the semaphore back so that we can unblock the button
	 xSemaphoreGive(xDebounceLock);
	 
   xTimerStop( xTimer, 0 );
 }
 
 void vDoubleClickTimer( TimerHandle_t xTimer)
 {
		if(click_count == 2) {
			//Trigger double click
		} else if(click_count == 1) {
			//Trigger single click
			//Run the button press routines by giving the semaphore and letting the button task execute
			xSemaphoreGive(xLEDCycleLock);
		}
		
		//Reset the click count
		click_count = 0;
	 
		//Stop the timer
	  xTimerStop( xTimer, 0 );
 }

static void vButtonTask( void *pvParameters )
{
	for( ;; )
	{
		//Poll the semaphore to see when it unlocks
		if(xSemaphoreTake(xLEDCycleLock, (TickType_t) 0) == pdTRUE) {
			//Once the button timer reset has given us the semaphore, change the current LED
			STM_EVAL_LEDToggle(leds[currLed]);
			if(currLed == 3) {
				currLed = 0;
			} else {
				currLed++;
			}
			
			STM_EVAL_LEDToggle(leds[currLed]);
		}
	}
}

void EXTI0_IRQHandler( void )
{
	 // Checks whether the interrupt from EXTI0 or not
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {      
				//Button press ocurred, check if we can access the semaphore
				//If the semaphore is not available it means we need to ignore the button debounce
				if(xSemaphoreTakeFromISR(xDebounceLock, NULL) == pdTRUE) {
					//Start the debounce timer to block debounce
					xTimerStartFromISR(xTimers[0],0);
					//Start the click count timer to handle button press actions
					xTimerStartFromISR(xTimers[1],0);
				}
			
			// Clears the EXTI line pending bit
      EXTI_ClearITPendingBit(EXTI_Line0);			
    }
}

//******************************************************************************
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */
	
	/*!< Most systems default to the wanted configuration, with the noticeable 
		exception of the STM32 driver library. If you are using an STM32 with 
		the STM32 driver library then ensure all the priority bits are assigned 
		to be preempt priority bits by calling 
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
	*/
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER ,BUTTON_MODE_EXTI);
	
	STM_EVAL_LEDToggle(LED_GREEN);
	
	//Create the task to cycle the leds
	xTaskCreate( vButtonTask, "Button", configMINIMAL_STACK_SIZE, NULL, BUTTON_PRIORITY, NULL );	
	//Create the timer to ignore the button debounce
	xTimers[0] = xTimerCreate("Debounce Timer", DEBOUNCE_DELAY, pdTRUE, (void *) 0, vButtonDebounce);
	//Create the timer to measure double clicks
	xTimers[1] = xTimerCreate("Double Click Timer", DOUBLE_CLICK_TIME, pdTRUE, (void *) 0, vDoubleClickTimer);
	//Create semaphore to block debouncing
	xDebounceLock = xSemaphoreCreateBinary();
	//Create semaphore to block the led cycling
	xLEDCycleLock = xSemaphoreCreateBinary();
	
	//Initally take the led semaphore so that when the task starts it's stuck
	xSemaphoreTake(xLEDCycleLock, (TickType_t) 0);
	//Initially have the debounce semaphore open so the first button press can take the lock in the ISR
	xSemaphoreGive(xDebounceLock);
	
	vTaskStartScheduler();
	
	for(;;);
  return 0;
}

void vLedBlinkBlue(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_BLUE);
		vTaskDelay( 1000 / portTICK_RATE_MS );
	}
}

void vLedBlinkRed(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_RED);
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void vLedBlinkGreen(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_GREEN);
		vTaskDelay( 200 / portTICK_RATE_MS );
	}
}

void vLedBlinkOrange(void *pvParameters)
{
	for(;;)
	{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay( 300 / portTICK_RATE_MS );
	}
}
//******************************************************************************
