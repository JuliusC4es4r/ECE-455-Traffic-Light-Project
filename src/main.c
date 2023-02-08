#include <stdint.h>
#include <stdio.h>
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
#define AMBER_PIN GPIO_Pin_1
#define GREEN_PIN GPIO_Pin_2
#define DATA_PIN GPIO_Pin_6
#define CLOCK_PIN GPIO_Pin_7
#define RESET_PIN GPIO_Pin_8


/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

static void TrafficFlowAdjustment( void *pvParameters );
static void TrafficGenerator( void *pvParameters );
static void LightState( void *pvParameters );
static void SystemDisplay( void *pvParameters );
static void adcInit(void);
static void gpioInit(void);
static void timerInit(void);
static uint16_t getADC(void);

uint32_t shift_output = 0x00000000;


/*-----------------------------------------------------------*/

int main(void)
{
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	adcInit();
	gpioInit();
	timerInit();

	xTaskCreate( TrafficFlowAdjustment, "Traffic Flow", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( TrafficGenerator, "Traffic Generator", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( LightState, "Traffic Light", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( SystemDisplay, "System Display", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

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

	GPIO_PinAFConfig(GPIOC, CLOCK_PIN, GPIO_AF_TIM3);

	GPIO_InitTypeDef gpioInit =
	{
			.GPIO_Pin = RED_PIN |
						AMBER_PIN |
						GREEN_PIN |
						DATA_PIN |
						RESET_PIN,
			.GPIO_Mode = GPIO_Mode_OUT,
			.GPIO_Speed = GPIO_Speed_100MHz,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_NOPULL

	};

	GPIO_Init(GPIOC, &gpioInit);

	GPIO_InitTypeDef gpioTimInit =
	{
		.GPIO_Pin = CLOCK_PIN,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_100MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL

	};

	GPIO_Init(GPIOC, &gpioTimInit);

}

static void timerInit(void)
{
	TIM_TimeBaseInitTypeDef timerInit =
	{
			.TIM_Prescaler = 128,
			.TIM_CounterMode = TIM_CounterMode_Up,
			.TIM_Period = 8400,
			.TIM_ClockDivision = TIM_CKD_DIV1

	};

	TIM_TimeBaseInit(TIM3, &timerInit);
	TIM_Cmd(TIM3, ENABLE);

	TIM_OCInitTypeDef tim_oc =
	{
		.TIM_OCMode = TIM_OCMode_PWM1,
		.TIM_OutputState = TIM_OutputState_Enable,
		.TIM_OCPolarity = TIM_OCPolarity_High,
		.TIM_Pulse = 2099
	};

	/* 25% duty cycle */
	TIM_OC1Init(TIM4, &tim_oc);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

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
		printf("%d\n", rawADC);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

/*-----------------------------------------------------------*/

static void TrafficGenerator( void *pvParameters )
{
	while(1)
	{
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}


/*-----------------------------------------------------------*/

static void LightState( void *pvParameters )
{
	while(1)
	{
		GPIO_SetBits(GPIOC, RED_PIN | AMBER_PIN | GREEN_PIN);
		vTaskDelay(500 / portTICK_RATE_MS);
		GPIO_ResetBits(GPIOC, RED_PIN | AMBER_PIN | GREEN_PIN);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

/*-----------------------------------------------------------*/

static void SystemDisplay( void *pvParameters )
{
	while(1)
	{
		vTaskDelay(500 / portTICK_RATE_MS);
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

