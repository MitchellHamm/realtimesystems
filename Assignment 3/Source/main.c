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
#include "codec.h"
//******************************************************************************

/*
*
*/
#define BUTTON_PRIORITY		1
#define BREW_PRIORITY 1
//This constant defines how long the board will ignore subsequent button presses for
#define DEBOUNCE_DELAY	( 150 / portTICK_RATE_MS )
//This constant defines how long the board will wait until processing button presses
#define DOUBLE_CLICK_TIME ( 500 / portTICK_RATE_MS)
#define GREEN_POSITION 0
#define ORANGE_POSITION 1
#define RED_POSITION 2
#define BLUE_POSITION 3
#define NOTEFREQUENCY 0.015
#define NOTEAMPLITUDE 500.0

// Sched types: 0=FPS, 1=EDF, 2=LLF
int SCHED_TYPE = 0;
// Brew types: 0=same start time, 1=diff start time
int COFFEE_START = 0;

int schedPrio = 5;
int brewPrio = 1;

typedef struct {
	float tabs[8];
	float params[8];
	uint8_t currIndex;
} fir_8;

typedef struct {
	int led;
	int duration;
	int deadline;
	int period;
	int priority;
} coffee;

typedef struct {
	coffee coffee;
	int state; // 0=running, 1=ready, 2=blocked
	int timeLeft;
} brewTask;

coffee espresso;
coffee lattee;
coffee cappuccino;
coffee mocha;

brewTask eTask;
brewTask lTask;
brewTask cTask;
brewTask mTask;

QueueHandle_t prio1, prio2, prio3;
brewTask coffees[4];

int prio1Count = 1;
int prio2Count = 2;
int prio3Count = 1;
int running = 0;
int ticks = 0;
int coffeeCount = 0;


int currLed = 0;
int codecInit = 0;
int leds[] = {LED_GREEN, LED_ORANGE, LED_RED, LED_BLUE};
int brew_times[] = {2, 21, 22, 23};
volatile uint32_t sampleCounter = 0;
volatile int16_t sample = 0;
fir_8 filt;

double sawWave = 0.0;

float filteredSaw = 0.0;

float updateFilter(fir_8* theFilter, float newValue);

void initFilter(fir_8* theFilter);
SemaphoreHandle_t xDebounceLock;
SemaphoreHandle_t xSchedulerLock;

uint32_t multiplier;

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

void TM_Delay_Init(void) {
    RCC_ClocksTypeDef RCC_Clocks;
    
    /* Get system clocks */
    RCC_GetClocksFreq(&RCC_Clocks);
    
    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    multiplier = RCC_Clocks.HCLK_Frequency / 4000000;
}

void TM_DelayMillis(uint32_t millis) {
    /* Multiply millis with multipler */
    /* Substract 10 */
    millis = 1000 * millis * multiplier - 10;
    /* 4 cycles for one loop */
    while (millis--);
}

void playSound() {
	int j = 0;
	codec_init();
		codec_ctrl_init();
		I2S_Cmd(CODEC_I2S, ENABLE);
		initFilter(&filt);
		codecInit = 1;
	
	while(j < 8400000) {
		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)){
			SPI_I2S_SendData(CODEC_I2S, sample);

			if (sampleCounter & 0x00000001){
				sawWave += NOTEFREQUENCY;
				if (sawWave > 1.0)
					sawWave -= 2.0;

				filteredSaw = updateFilter(&filt, sawWave);
				sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
			}
			sampleCounter++;
		}
		j++;
	}
}

static void vBrewTask0(void *pvParameters) {
	for(;;) {
				if(xSemaphoreTake(xSchedulerLock, (TickType_t) 10) == pdTRUE) {
		// Pull the brew task from the parameters
	  brewTask *p = (brewTask*) &(*pvParameters);
	  brewTask curr = *p;
		STM_EVAL_LEDOn(curr.coffee.led);
		
		curr.timeLeft--;
		
		if(curr.timeLeft == 0) {
			curr.timeLeft = curr.coffee.duration;
			curr.state = 2;
			playSound();
		} else {
			curr.state = 1;
		}
		
		if(curr.coffee.priority == 3)
			xQueueSendToBack(prio3, (void*)&curr, (TickType_t) 10);
		if(curr.coffee.priority == 2)
			xQueueSendToBack(prio2, (void*)&curr, (TickType_t) 10);
		if(curr.coffee.priority == 1)
			xQueueSendToBack(prio1, (void*)&curr, (TickType_t) 10);
		
		TM_DelayMillis(500);

		STM_EVAL_LEDOff(curr.coffee.led);
		
		TM_DelayMillis(500);
		
		xSemaphoreGive(xSchedulerLock);
		vTaskDelete(NULL);
	}
}
}

static void vBrewTask1(void *pvParameters) {
	for(;;) {
				if(xSemaphoreTake(xSchedulerLock, (TickType_t) 10) == pdTRUE) {
		// Pull the brew task from the parameters
	  int *p = (int*) &(*pvParameters);
	  int index = *p;
					
		STM_EVAL_LEDOn(coffees[index].coffee.led);
		
		coffees[index].timeLeft--;
		
		if(coffees[index].timeLeft == 0) {
			coffees[index].timeLeft = coffees[index].coffee.duration;
			coffees[index].state = 2;
			playSound();
		} else {
			coffees[index].state = 1;
		}
		
		TM_DelayMillis(500);

		STM_EVAL_LEDOff(coffees[index].coffee.led);
		
		TM_DelayMillis(500);
		xSemaphoreGive(xSchedulerLock);
		vTaskDelete(NULL);
	}
}
}

static void vScheduler(void *pvParameters) {
	for(;;)
	{
		int i = 0;
		int found = 0;
		brewTask task1, task2, task3;
		int j = 0;
		int earliest = 1000;
		int foundIndex = -1;
		int laxity = 1000;
		if(xSemaphoreTake(xSchedulerLock, (TickType_t) 10) == pdTRUE) {
					
			if(COFFEE_START == 1) {
				if(ticks == 10) {
					xQueueSend(prio1, (void*)&coffees[1], (TickType_t) 0);
					coffees[1] = lTask;
					coffeeCount++;
				} else if(ticks == 20) {
					xQueueSend(prio2, (void*)&coffees[2], (TickType_t) 0);
					coffees[2] = cTask;
					coffeeCount++;
				} else if(ticks == 30) {
					xQueueSend(prio2, (void*)&coffees[3], (TickType_t) 0);
					coffees[3] = mTask;
					coffeeCount++;
				}
			}
		/************************************************
		* FIXED PRIORITY SCHEDULER
		* Use priority queues to determine what coffee 
		* to brew next
		************************************************/
		if(SCHED_TYPE == 0) {
			
			// Update any task states if necessary
			while(j < prio1Count) {
				xQueueReceive(prio1, &task1, (TickType_t)10);
				if(ticks % task1.coffee.period == 0) {
					task1.state = 1;
				}
				xQueueSendToBack(prio1, (void*)&task1, (TickType_t) 0);
				j++;
			}
		
			j = 0;
			while(j < prio2Count) {
				xQueueReceive(prio2, &task2, (TickType_t)10);
				if(ticks % task2.coffee.period == 0) {
					task2.state = 1;
				}
				xQueueSendToBack(prio2, (void*)&task2, (TickType_t) 0);
				j++;
			}
		
			j = 0;
			while(j < prio3Count) {
				xQueueReceive(prio3, &task3, (TickType_t)10);
				if(ticks % task3.coffee.period == 0) {
					task3.state = 1;
				}
				xQueueSendToBack(prio3, (void*)&task3, (TickType_t) 0);
				j++;
			}
			
			while(found == 0 && i < prio3Count) {
			  xQueueReceive(prio3, &task1, (TickType_t)10);
			  if(task1.state == 1) {
				  // Task is ready, run it
					found = 1;
					task1.state = 0;
					xTaskCreate(vBrewTask0, "Brew task", configMINIMAL_STACK_SIZE, (void*)&task1, brewPrio, NULL);
				} else {
					// Add to the back of the queue
				  xQueueSendToBack(prio3, (void*)&task1, (TickType_t) 10);
				}
			  i++;
		  }
			
			if(found == 0) {
				i = 0;
			  while(found == 0 && i < prio2Count) {
				  xQueueReceive(prio2, &task2, (TickType_t)10);
				  if(task2.state == 1) {
					  // Task is ready, run it
					  found = 1;
					  task2.state = 0;
						xTaskCreate(vBrewTask0, "Brew task", configMINIMAL_STACK_SIZE, (void*)&task2, brewPrio, NULL);
				  } else {
						// Add to the back of the queue
					  xQueueSendToBack(prio2, (void*)&task2, (TickType_t) 10);
					}
				  i++;
			  }
			}
				
			if(found == 0) {
				i = 0;
			  while(found == 0 && i < prio1Count) {
					xQueueReceive(prio1, &task3, (TickType_t)10);
				  if(task3.state == 1) {
				    // Task is ready, run it
				    found = 1;
				    task3.state = 0;
					  xTaskCreate(vBrewTask0, "Brew task", configMINIMAL_STACK_SIZE, (void*)&task3, brewPrio, NULL);
				   } else {
					  // Add to the back of the queue
				    xQueueSendToBack(prio1, (void*)&task3, (TickType_t) 10);
				   }
				   i++;
			   }
			 }
		}
		
		/************************************************
		* EARLIEST DEADLINE FIRST SCHEDULER
		* Use earliest deadline to determine which coffee 
		* to brew next
		************************************************/
		else if(SCHED_TYPE == 1) {
			// Update any task states if necessary
			  for(i = 0; i < coffeeCount; i++) {
					if(ticks % coffees[i].coffee.period == 0) {
						coffees[i].state = 1;
					}						
				}
				
				i = 0;
				while(found == 0 && i < coffeeCount) {
				  if(coffees[i].state == 1 && coffees[i].coffee.deadline < earliest) {
						foundIndex = i;
						earliest = coffees[i].coffee.deadline;
				  } 
				  i++;
			  }
					
				if(foundIndex != -1) {
				  // Task is ready, run it
				  found = 1;
					coffees[foundIndex].state = 0;
					xTaskCreate(vBrewTask1, "Brew task", configMINIMAL_STACK_SIZE, (void*)&foundIndex, brewPrio, NULL);
				}
		}
		
		/************************************************
		* LEAST LAXITY FIRST SCHEDULER
		*  
		* 
		************************************************/
		else if(SCHED_TYPE == 2) {
			
			// Update any task states if necessary
			for(i = 0; i < coffeeCount; i++) {
					if(ticks % coffees[i].coffee.period == 0) {
						coffees[i].state = 1;
					}						
				}
				
				i = 0;
			while(found == 0 && i < coffeeCount) {
				  if(coffees[i].state == 1 && coffees[i].coffee.deadline - coffees[i].timeLeft <= laxity) {
						foundIndex = i;
						laxity = coffees[i].coffee.deadline - coffees[i].timeLeft;
				  } 
				  i++;
			  }
					
				// Only want to switch tasks if we have a ready task with a earlier deadline
				if(foundIndex != -1) {
				  // Task is ready, run it
				  found = 1;
					coffees[foundIndex].state = 0;
					xTaskCreate(vBrewTask1, "Brew task", configMINIMAL_STACK_SIZE, (void*)&foundIndex, brewPrio, NULL);
				}
		}
		
		// Increment the tick count
		xSemaphoreGive(xSchedulerLock);
		vTaskDelay(1000);
		ticks++;
	}
}
}

void EXTI0_IRQHandler( void )
{
	 static signed portBASE_TYPE xHigherPriorityTaskWoken;
	 // Checks whether the interrupt from EXTI0 or not
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {      
				xHigherPriorityTaskWoken = pdFALSE;
				//Button press ocurred, check if we can access the semaphore
				//If the semaphore is not available it means we need to ignore the button debounce
				if(xSemaphoreTakeFromISR(xDebounceLock, NULL) == pdTRUE) {
					//Crude loop to ignore button debounce
					int i = 0;
					while( i < 6000000) {
						i++;
					}
					
					//Start or stop the scheduler
					if(running == 0) {
						xSemaphoreGiveFromISR(xSchedulerLock, &xHigherPriorityTaskWoken);
						running = 1;
					} else {
						xSemaphoreTakeFromISR(xSchedulerLock, &xHigherPriorityTaskWoken);
						running = 0;
					}
	 
					//give the semaphore back so that we can unblock the button
					xSemaphoreGiveFromISR(xDebounceLock, &xHigherPriorityTaskWoken);
				}
			
			// Clears the EXTI line pending bit
      EXTI_ClearITPendingBit(EXTI_Line0);			
    }
}

void initPlayback(){
	GPIO_InitTypeDef I2C_InitStructure;
	I2S_InitTypeDef I2S_InitType;
	I2C_InitTypeDef I2C_InitType;
	
	I2C_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);
	I2C_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	I2C_InitStructure.GPIO_OType = GPIO_OType_PP;
	I2C_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	I2S_InitType.I2S_AudioFreq = I2S_AudioFreq_48k;
	I2S_InitType.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
	I2S_InitType.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitType.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitType.I2S_Standard = I2S_Standard_Phillips;
	I2S_InitType.I2S_CPOL = I2S_CPOL_Low;
	I2S_Init(SPI3, &I2S_InitType); 
	I2S_Cmd(SPI3, ENABLE);
	
	I2C_InitType.I2C_ClockSpeed = 100000;
	I2C_InitType.I2C_Mode = I2C_Mode_I2C;
	I2C_InitType.I2C_OwnAddress1 = 99;
	I2C_InitType.I2C_Ack = I2C_Ack_Enable;
	I2C_InitType.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitType.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_Init(I2C1, &I2C_InitType);   //initialize the I2C peripheral 
	I2C_Cmd(I2C1, ENABLE);          //turn it on
}

void initDelayCoffees() {
	espresso.led = LED_BLUE;
	espresso.duration = 3;
	espresso.deadline = 5;
	espresso.period = 20;
	espresso.priority = 3;
	
	lattee.led = LED_GREEN;
	lattee.duration = 4;
	lattee.deadline = 10;
	lattee.period = 30;
	lattee.priority = 1;
	
	cappuccino.led = LED_ORANGE;
	cappuccino.duration = 4;
	cappuccino.deadline = 10;
	cappuccino.period = 40;
	cappuccino.priority = 2;
	
	mocha.led = LED_RED;
	mocha.duration = 6;
	mocha.deadline = 15;
	mocha.period = 40;
	mocha.priority = 2;
	
	eTask.coffee = espresso;
	eTask.state = 1;
	eTask.timeLeft = 3;
	lTask.coffee = lattee;
	lTask.state = 1;
	lTask.timeLeft = 4;
	cTask.coffee = cappuccino;
	cTask.state = 1;
	cTask.timeLeft = 4;
	mTask.coffee = mocha;
	mTask.state = 1;
	mTask.timeLeft = 6;
	
	prio1 = xQueueCreate(4, sizeof(brewTask));
	prio2 = xQueueCreate(4, sizeof(brewTask));
	prio3 = xQueueCreate(4, sizeof(brewTask));
	
	coffees[0] = eTask;
	
	xQueueSend(prio3, (void*)&eTask, (TickType_t) 0);
	
	ticks = 0;
	coffeeCount = 1;
}

void initCoffees() {
	espresso.led = LED_BLUE;
	espresso.duration = 3;
	espresso.deadline = 5;
	espresso.period = 20;
	espresso.priority = 3;
	
	lattee.led = LED_GREEN;
	lattee.duration = 4;
	lattee.deadline = 10;
	lattee.period = 30;
	lattee.priority = 1;
	
	cappuccino.led = LED_ORANGE;
	cappuccino.duration = 4;
	cappuccino.deadline = 10;
	cappuccino.period = 40;
	cappuccino.priority = 2;
	
	mocha.led = LED_RED;
	mocha.duration = 6;
	mocha.deadline = 15;
	mocha.period = 40;
	mocha.priority = 2;
	
	eTask.coffee = espresso;
	eTask.state = 1;
	eTask.timeLeft = 3;
	lTask.coffee = lattee;
	lTask.state = 1;
	lTask.timeLeft = 4;
	cTask.coffee = cappuccino;
	cTask.state = 1;
	cTask.timeLeft = 4;
	mTask.coffee = mocha;
	mTask.state = 1;
	mTask.timeLeft = 6;
	
	prio1 = xQueueCreate(4, sizeof(brewTask));
	prio2 = xQueueCreate(4, sizeof(brewTask));
	prio3 = xQueueCreate(4, sizeof(brewTask));
	
	coffees[0] = eTask;
	coffees[1] = lTask;
	coffees[2] = cTask;
	coffees[3] = mTask;
	
	xQueueSend(prio3, (void*)&eTask, (TickType_t) 0);
	xQueueSend(prio1, (void*)&lTask, (TickType_t) 0);
	xQueueSend(prio2, (void*)&cTask, (TickType_t) 0);
	xQueueSend(prio2, (void*)&mTask, (TickType_t) 0);
	
	ticks = 0;
	coffeeCount = 4;
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

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; //I2S SCL and SDA pins
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	SysTick_Config(SystemCoreClock/1000);

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	codec_init();
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER ,BUTTON_MODE_EXTI);
	
	TM_Delay_Init();
	
	if(COFFEE_START == 0)
	  initCoffees();
	else if(COFFEE_START == 1)
		initDelayCoffees();
	
	//STM_EVAL_LEDToggle(LED_GREEN);
	
	
	
	//Create the task to cycle the leds
	//Create the brew tasks for each led
	xTaskCreate( vScheduler, "Scheduler", configMINIMAL_STACK_SIZE, NULL, schedPrio, NULL);
	
	//Create semaphore to block debouncing
	xDebounceLock = xSemaphoreCreateBinary();
	//Create semaphore to block the led cycling
  xSchedulerLock = xSemaphoreCreateBinary();
	
	
	//Initally take the led semaphore so that when the task starts it's stuck
	//xSemaphoreTake(xLEDCycleLock, (TickType_t) 0);
	//Initially have the debounce semaphore open so the first button press can take the lock in the ISR
	xSemaphoreGive(xDebounceLock);
	
	vTaskStartScheduler();
	
	for(;;);
  return 0;
}

float updateFilter(fir_8* filt, float val)
{
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter)
{
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}
//******************************************************************************
