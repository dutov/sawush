/* Includes from SPL */
//	#include "stm32f4xx_adc.h"
//	#include "stm32f4xx_can.h"
//	#include "stm32f4xx_crc.h"
//	#include "stm32f4xx_cryp.h"
//	#include "stm32f4xx_dac.h"
//	#include "stm32f4xx_dbgmcu.h"
//	#include "stm32f4xx_dcmi.h"
//	#include "stm32f4xx_dma.h"
//	#include "stm32f4xx_exti.h"
//	#include "stm32f4xx_flash.h"
//	#include "stm32f4xx_fsmc.h"
//	#include "stm32f4xx_hash.h"
	#include "stm32f4xx_gpio.h"
//	#include "stm32f4xx_i2c.h"
//	#include "stm32f4xx_iwdg.h"
//	#include "stm32f4xx_pwr.h"
	#include "stm32f4xx_rcc.h"
//	#include "stm32f4xx_rng.h"
//	#include "stm32f4xx_rtc.h"
	#include "stm32f4xx_sdio.h"
//	#include "stm32f4xx_spi.h"
//	#include "stm32f4xx_syscfg.h"
//	#include "stm32f4xx_tim.h"
//	#include "stm32f4xx_usart.h"
//	#include "stm32f4xx_wwdg.h"
	#include "misc.h"

/* Includes from FreeRTOS */
	#include "FreeRTOS.h"
	#include "task.h"
	#include "queue.h"

/* User's functions */
static void prvSetupHardware( void );

/* Test user's functions */
static void prvLedBlink1( void *pvParameters );
static void prvLedBlink2( void *pvParameters );
static void prvLedBlink3( void *pvParameters );
static void prvLedBlink4( void *pvParameters );

void prvSetupHardware()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void prvLedBlink1( void *pvParameters )
{
	while(1){
		/* PD12 to be toggled */
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		/* Insert delay */
		vTaskDelay(250 / portTICK_RATE_MS);
		/* PD12 to be toggled */
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		/* Insert delay */
		vTaskDelay(250 / portTICK_RATE_MS);
	}
}

void prvLedBlink2( void *pvParameters )
{
	while(1){
		vTaskDelay(63 / portTICK_RATE_MS);
		/* PD12 to be toggled */
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		/* Insert delay */
		vTaskDelay(250 / portTICK_RATE_MS);
		/* PD12 to be toggled */
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		/* Insert delay */
		vTaskDelay(187 / portTICK_RATE_MS);
	}
}

void prvLedBlink3( void *pvParameters )
{
	while(1){
		vTaskDelay(125 / portTICK_RATE_MS);
		/* PD12 to be toggled */
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		/* Insert delay */
		vTaskDelay(250 / portTICK_RATE_MS);
		/* PD12 to be toggled */
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		/* Insert delay */
		vTaskDelay(125 / portTICK_RATE_MS);
	}
}

void prvLedBlink4( void *pvParameters )
{
	while(1){
		vTaskDelay(187 / portTICK_RATE_MS);
		/* PD12 to be toggled */
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		/* Insert delay */
		vTaskDelay(250 / portTICK_RATE_MS);
		/* PD12 to be toggled */
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		/* Insert delay */
		vTaskDelay(63 / portTICK_RATE_MS);
	}
}

/* The main function */
int main(void)
{
	prvSetupHardware();

	xTaskCreate(prvLedBlink1,(signed char*)"LED",configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(prvLedBlink2,(signed char*)"LED",configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(prvLedBlink3,(signed char*)"LED",configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY + 3, NULL);

	xTaskCreate(prvLedBlink4,(signed char*)"LED",configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY + 4, NULL);
	/* Start the scheduler. */
	vTaskStartScheduler();

    while(1);
}
