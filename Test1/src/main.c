/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO(R)
 **                STMicroelectronics STM32F4xx Standard Peripherals Library
 **
 **  Distribution: The file is distributed as is, without any warranty
 **                of any kind.
 **
 **  (c)Copyright Atollic AB.
 **  You may use this file as-is or modify it according to the needs of your
 **  project. Distribution of this file (unmodified or modified) is not
 **  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
 **  rights to distribute the assembled, compiled & linked contents of this
 **  file as part of an application binary file, provided that it is built
 **  using the Atollic TrueSTUDIO(R) toolchain.
 **
 **
 *****************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "utils.h"
#include "debug.h"
#include "mems.h"

xQueueHandle delayQueue;

#define GREENLED LED4
#define ORANGELED LED3
#define BLUELED LED6
#define IDLE_PRIORITY tskIDLE_PRIORITY /* Lowest priority */
#define MIN_STACK_SIZE configMINIMAL_STACK_SIZE
//STUFF ADDED FOR MARBEX
#define INIT_FREQ 1000

/* Initializations */
void GPIO_Configuration(void);
void RCC_Configuration(void);
void DMA_Configuration(void);
void ADC_Configuration(void);
/* Tasks */
//STUFF ADDED FOR MARBEX
static void vRowReadTask(void *pvparameters );
static void vLineHighTask( void *pvparameters );
static void vFreqChangeTask(void *pvparameters);
static void vVolChangeTask(void *pvparameters);
void PlaySound(uint8_t output);
void delay_ms(uint32_t milli);

/* Private variables */
uint64_t tickTime=0;        // Counts OS ticks (default = 1000Hz).
uint64_t u64IdleTicks=0;    // Value of u64IdleTicksCnt is copied once per sec.
uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
//STUFF ADDED FOR MARBEX
uint64_t frequency = INIT_FREQ;
uint8_t counter = 0;

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

// ============================================================================
int main( void )
{

	/* Initializations */
	RCC_Configuration();
	GPIO_Configuration(); // SET: PA4 - OUT; PA1, PA2, PA3 - IN;
	DMA_Configuration();
	ADC_Configuration();
	/* Create Tasks */
	xTaskCreate( vRowReadTask, ( signed char * ) "Read Row Task", MIN_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate( vLineHighTask, ( signed char * ) "Line High Task", MIN_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate( vFreqChangeTask, ( signed char * ) "Freq Change Task", MIN_STACK_SIZE, NULL, 5, NULL);
	xTaskCreate( vVolChangeTask, ( signed char * ) "Vol Change Task", MIN_STACK_SIZE, NULL, 4, NULL);
	vTaskStartScheduler();

    // Will only get here if there was insufficient memory to create
    // the idle task.
    for( ;; );  
}

/*
 * Row Read Task
 */
static void vRowReadTask( void *pvparameters )
{
	uint8_t value0;
	uint8_t value1;
	uint8_t value2;
	uint8_t value3;

	for(;;)
	{
		// SET ENABLE BITS, maybe do this in wiring?
		if(counter & 00000001){
			// SET PIN 0 HIGH FOR ALL MUX
			GPIO_SetBits(GPIOA, GPIO_Pin_1);
			//GPIO_SetBits(GPIOB, GPIO_Pin_1);
			//GPIO_SetBits(GPIOC, GPIO_Pin_1);
			//GPIO_SetBits(GPIOD, GPIO_Pin_1);
		}
		if(counter & 00000010){
			// SET PIN 1 HIGH FOR ALL MUX
			GPIO_SetBits(GPIOA, GPIO_Pin_2);
			//GPIO_SetBits(GPIOB, GPIO_Pin_2);
			//GPIO_SetBits(GPIOC, GPIO_Pin_2);
			//GPIO_SetBits(GPIOD, GPIO_Pin_2);
		}
		if(counter & 00000001){
			// SET PIN 2 HIGH FOR ALL MUX
			GPIO_SetBits(GPIOA, GPIO_Pin_3);
			//GPIO_SetBits(GPIOB, GPIO_Pin_3);
			//GPIO_SetBits(GPIOC, GPIO_Pin_3);
			//GPIO_SetBits(GPIOD, GPIO_Pin_3);
		}
		delay_ms(10);
		// READ INPUT FOR ALL MUX

		value0 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		value1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
		value2 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
		value3 = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0);

		// READ HIGH ??
		value0 = value0 & 00000001;
		value1 = value1 & 00000001;
		value2 = value2 & 00000001;
		value3 = value3 & 00000001;

		// WILL THIS WORK?
		uint8_t output = value0 + (value1 << 1) + (value2 << 2) + (value3 << 3);
		//THIS IS WHERE TO SEND TO CORRESPONDING SOUND OUTPUT
		PlaySound(output);
		vTaskDelay(frequency);
	}
}

void PlaySound(uint8_t output){

}

/*
 * Line High Task
 */
static void vLineHighTask( void *pvparameters )
{
	for(;;)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		delay_ms(10);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		if(counter == 7){
			counter = 0;
		}else{
			counter++;
		}
		vTaskDelay(frequency);
	}
}

/*
 * Frequency Adjustment Task
 */
static void vFreqChangeTask(void *pvparameters){
	for(;;){

	}
}
/*
 * Volume Adjustment Task
 */
static void vVolChangeTask(void *pvparameters){

}
/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */
void RCC_Configuration(void)
{
	/* Enable DMA and GPIOA Clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA, ENABLE);

	/* Enable DAC1 and TIM6 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC | RCC_APB1Periph_TIM6, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Pack the struct */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* Call Init function */
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitTypeDef GPIO_InitStruct2;

	/* Pack the struct */
	GPIO_InitStruct2.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct2.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct2.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct2.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* Call Init function */
	GPIO_Init(GPIOA, &GPIO_InitStruct2);
}

/**
  * @brief  Configures the DMA.
  * @param  None
  * @retval : None
  */
void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	//Initialize the structure to default values
	DMA_StructInit(&DMA_InitStructure);

	/* PACK YOUR INIT STRUCT HERE */

	/* Call Init function */
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	/* Enable DMA */
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

/**
  * @brief  Configures the ADC.
  * @param  None
  * @retval : None
  */
void ADC_Configuration(void){
	// ADC connected to pin PC1 according to datasheet

	GPIO_InitTypeDef GPIOInit;

	/* Pack the struct */
	GPIOInit.GPIO_Mode = GPIO_Mode_AN;
	GPIOInit.GPIO_Pin = GPIO_Pin_1;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInit.GPIO_OType = GPIO_OType_PP;
	GPIOInit.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* Call Init function */
		GPIO_Init(GPIOC, &GPIOInit);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; //??;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;; //??;
	ADC_InitStructure.ADC_NbrOfConversion = 1;          /*!< Specifies the number of ADC conversions
	                                               that will be done using the sequencer for
	                                               regular channel group.
	                                               This parameter must range from 1 to 16. */
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,1,ADC_SampleTime_480Cycles);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA
	ADC_Cmd(ADC1, ENABLE); // Enable ADC1
	ADC_SoftwareStartConv(ADC1); // Start ADC1 conversion

}
// This FreeRTOS callback function gets called once per tick (default = 1000Hz).
// ---------------------------------------------------------------------------- 
void vApplicationTickHook( void ) {
    ++tickTime;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ---------------------------------------------------------------------------- 
void vApplicationIdleHook( void ) {
    ++u64IdleTicksCnt;
}

// A required FreeRTOS function.
// ---------------------------------------------------------------------------- 
void vApplicationMallocFailedHook( void ) {
    configASSERT( 0 );  // Latch on any failure / error.
}

void delay_ms(uint32_t milli)
{
	uint32_t delay = milli * 17612;              // approximate loops per ms at 168 MHz, Debug config
	for(; delay != 0; delay--);
}


