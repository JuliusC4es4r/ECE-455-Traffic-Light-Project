#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"

#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define ADC_PIN GPIO_Pin_3
#define RED_PIN GPIO_Pin_0
#define YELLOW_PIN GPIO_Pin_1
#define GREEN_PIN GPIO_Pin_2
#define DATA_PIN GPIO_Pin_6
#define CLOCK_PIN GPIO_Pin_7
#define RESET_PIN GPIO_Pin_8

#define RED_STATE 0x01
#define YELLOW_STATE 0x02
#define GREEN_STATE 0x03



/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

static void TrafficFlowAdjustment( void *pvParameters );
static void TrafficGenerator( void *pvParameters );
static void LightState( void *pvParameters );
static void SystemDisplay( TimerHandle_t xTimer );
static void adcInit(void);
static void gpioInit(void);
static uint16_t getADC(void);

SemaphoreHandle_t trafficOutputMutex;
uint32_t trafficOutput = 0x00000000;

SemaphoreHandle_t potValueMutex;
uint32_t potValue = 0;

SemaphoreHandle_t trafficLightMutex;
uint8_t trafficLight = RED_STATE;

TimerHandle_t displayTimer;


/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	adcInit();
	gpioInit();

	displayTimer = xTimerCreate(
			"displayTimer",
			pdMS_TO_TICKS(1000 / portTICK_RATE_MS),
			pdTRUE,
			(void*)0,
			SystemDisplay
	);

	xTimerStart(displayTimer, portMAX_DELAY);

	trafficOutputMutex = xSemaphoreCreateMutex();
	xSemaphoreGive(trafficOutputMutex);

	potValueMutex = xSemaphoreCreateMutex();
	xSemaphoreGive(potValueMutex);

	trafficLightMutex = xSemaphoreCreateMutex();
	xSemaphoreGive(trafficLightMutex);

	xTaskCreate( TrafficFlowAdjustment, "Traffic Flow", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( TrafficGenerator, "Traffic Generator", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( LightState, "Traffic Light", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
//	xTaskCreate( SystemDisplay, "System Display", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}


/*-----------------------------------------------------------*/

static void adcInit(void)
{

	GPIO_InitTypeDef gpioInit =
	{
			.GPIO_Pin = ADC_PIN,
			.GPIO_Mode = GPIO_Mode_AN,
			.GPIO_Speed = GPIO_Speed_100MHz,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_NOPULL
	};

	GPIO_Init(GPIOC, &gpioInit);

	ADC_InitTypeDef adcInit =
	{
			.ADC_Resolution = ADC_Resolution_10b,
			.ADC_ContinuousConvMode = ENABLE,
			.ADC_DataAlign = ADC_DataAlign_Right
	};

	ADC_Init(ADC1, &adcInit);

	ADC_Cmd(ADC1, ENABLE);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 0x00000001, ADC_SampleTime_15Cycles);

}

static void gpioInit(void)
{

	GPIO_InitTypeDef gpioInit =
	{
			.GPIO_Pin = RED_PIN |
						YELLOW_PIN |
						GREEN_PIN |
						DATA_PIN |
						CLOCK_PIN |
						RESET_PIN,
			.GPIO_Mode = GPIO_Mode_OUT,
			.GPIO_Speed = GPIO_Speed_100MHz,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_NOPULL

	};

	GPIO_Init(GPIOC, &gpioInit);

}

static uint16_t getADC(void)
{
	ADC_SoftwareStartConv(ADC1);

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
	{
		vTaskDelay(100);
	}

	return ADC_GetConversionValue(ADC1);
}

static void TrafficFlowAdjustment( void *pvParameters )
{
	uint16_t rawADC;
	while(1)
	{
		rawADC = getADC();
//		printf("%d\n", rawADC);

		xSemaphoreTake(potValueMutex, portMAX_DELAY);
		potValue = rawADC;
		xSemaphoreGive(potValueMutex);

		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

/*-----------------------------------------------------------*/

static void TrafficGenerator( void *pvParameters )
{
	while(1)
	{
		uint8_t tempState;
		xSemaphoreTake(trafficLightMutex, portMAX_DELAY);
		tempState = trafficLight;
		xSemaphoreGive(trafficLightMutex);

		uint16_t tempADC;
		xSemaphoreTake(potValueMutex, portMAX_DELAY);
		tempADC = potValue;
		xSemaphoreGive(potValueMutex);

		int flowRate = (int) ( 6500 - (5.5 * tempADC) );
		float tempRand = (rand() % 6 ) * 50;

		int generationDelay = (int)flowRate + tempRand;

		printf("%d \n", generationDelay);

		vTaskDelay(generationDelay / portTICK_RATE_MS);

		xSemaphoreTake(trafficOutputMutex, portMAX_DELAY);
		trafficOutput |= 0x0001;
		xSemaphoreGive(trafficOutputMutex);
	}
}


/*-----------------------------------------------------------*/

static void LightState( void *pvParameters )
{
	while(1)
	{
		uint8_t tempState;
		xSemaphoreTake(trafficLightMutex, portMAX_DELAY);
		tempState = trafficLight;
		xSemaphoreGive(trafficLightMutex);

		uint16_t tempADC;
		xSemaphoreTake(potValueMutex, portMAX_DELAY);
		tempADC = potValue;
		xSemaphoreGive(potValueMutex);

		int flowRate = (int) ( 7500 - (5 * tempADC) );
		int inverseFlowRate = (int) ( 2500 + (5 * tempADC) );

		printf("%d, %d \n", flowRate, inverseFlowRate);

		switch (tempState)
		{
			case(RED_STATE):

				GPIO_ResetBits(GPIOC, RED_PIN | YELLOW_PIN | GREEN_PIN);
				GPIO_SetBits(GPIOC, RED_PIN);
				vTaskDelay( flowRate / portTICK_RATE_MS);

				xSemaphoreTake(trafficLightMutex, portMAX_DELAY);
				trafficLight = GREEN_STATE;
				xSemaphoreGive(trafficLightMutex);

				break;

			case(YELLOW_STATE):

				GPIO_ResetBits(GPIOC, RED_PIN | YELLOW_PIN | GREEN_PIN);
				GPIO_SetBits(GPIOC, YELLOW_PIN);

				vTaskDelay(2000 / portTICK_RATE_MS);

				xSemaphoreTake(trafficLightMutex, portMAX_DELAY);
				trafficLight = RED_STATE;
				xSemaphoreGive(trafficLightMutex);

				break;

			case(GREEN_STATE):

				GPIO_ResetBits(GPIOC, RED_PIN | YELLOW_PIN | GREEN_PIN);
				GPIO_SetBits(GPIOC, GREEN_PIN);
				vTaskDelay(inverseFlowRate / portTICK_RATE_MS);

				xSemaphoreTake(trafficLightMutex, portMAX_DELAY);
				trafficLight = YELLOW_STATE;
				xSemaphoreGive(trafficLightMutex);

				break;

			default:
				break;
		}
	}
}

/*-----------------------------------------------------------*/

static void SystemDisplay( TimerHandle_t xTimer )
{
	while(1)
	{
		GPIO_ResetBits(GPIOC, RESET_PIN);
		vTaskDelay(1 / portTICK_RATE_MS);
		GPIO_SetBits(GPIOC, RESET_PIN);

		uint32_t tempTraffic;
		xSemaphoreTake(trafficOutputMutex, portMAX_DELAY);
		tempTraffic = trafficOutput;
		xSemaphoreGive(trafficOutputMutex);

		for(int i = 0; i < 20; i++)
		{
			GPIO_SetBits(GPIOC, CLOCK_PIN);

			if(tempTraffic & (0x00000001 << i))
			{
				GPIO_SetBits(GPIOC, DATA_PIN);
			}
			else
			{
				GPIO_ResetBits(GPIOC, DATA_PIN);
			}

			GPIO_ResetBits(GPIOC, CLOCK_PIN);
		}
		GPIO_ResetBits(GPIOC, DATA_PIN);

		uint8_t tempState;
		xSemaphoreTake(trafficLightMutex, portMAX_DELAY);
		tempState = trafficLight;
		xSemaphoreGive(trafficLightMutex);

		if(tempState == GREEN_STATE)
		{
			xSemaphoreTake(trafficOutputMutex, portMAX_DELAY);
			trafficOutput = trafficOutput << 1;
			xSemaphoreGive(trafficOutputMutex);
		}
		else
		{
			xSemaphoreTake(trafficOutputMutex, portMAX_DELAY);

			uint32_t postLight = trafficOutput & 0xFFFFFF00;
			uint8_t preLight = trafficOutput & 0xFF;

			for(int i = 7; i >= 0; i--)
			{
				if( !(preLight & (0x01 << i) ))
				{
//					preLight = ((preLight & (((0x00FF << (i-1)) & 0xFF00))) << 1) & (0xFF00 >> (8-i));
					preLight = ((preLight & ((((0x00FF << (i-1)) & 0xFF00) >> 8 )) << 1) & (0xFF00 >> (8-i)) );
				}
			}

			trafficOutput = (postLight << 1) | preLight;

			xSemaphoreGive(trafficOutputMutex);
		}

		vTaskDelay(1000);
	}
}


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

