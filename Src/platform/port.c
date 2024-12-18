/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port.h"
#include "deca_device_api.h"
#include "stm32f1xx_hal_conf.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
static volatile uint32_t signalResetDone;
static port_dwic_isr_t port_dwic_isr = NULL;
volatile int32_t sys_time_diff = 0;

 
/* @fn 	  portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 * 		  CLOCKS_PER_SEC frequency.
 * 		  The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE unsigned long
portGetTickCnt(void)
{

	return HAL_GetTick() - sys_time_diff;

}


/* @fn	  usleep
 * @brief precise usleep() delay
 * */
#pragma GCC optimize ("O0")
int usleep(unsigned long usec)
{
	int i,j;
#pragma GCC ivdep
	for(i=0;i<usec;i++)
	{
#pragma GCC ivdep
		for(j=0;j<2;j++)
		{
			__NOP();
			__NOP();
		}
	}
	return 0;
}


/* @fn 	  Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
	HAL_Delay(x);
}

void wakeup_device_with_io(void)
{
    SET_WAKEUP_PIN_IO_HIGH;
    WAIT_500uSEC;
    SET_WAKEUP_PIN_IO_LOW;
}

/* @fn 	  peripherals_init
 * */
int peripherals_init (void)
{
	/* All has been initialized in the CubeMx code, see main.c */
	return 0;
}


/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t x)
{
	return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] &\
		    (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL)) ) == (uint32_t)RESET)?(RESET):(SET);
}


/* @fn		reset_DWIC
 * @brief	DW_RESET pin on DW1000 has 2 functions
 * 			In general it is output, but it also can be used to reset the digital
 * 			part of DW1000 by driving this pin low.
 * 			Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DWIC(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;

	// Enable GPIO used for DW1000 reset as open collector output
	GPIO_InitStruct.Pin = DW_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);

	usleep(1);
	//put the pin back to output open-drain (not active)
	setup_DWICRSTnIRQ(0);

	Sleep(2);

}

/* @fn		setup_DWICRSTnIRQ
 * @brief	setup the DW_RESET pin mode
 * 			0 - output Open collector mode
 * 			!0 - input mode with connected EXTI0 IRQ
 * */
void setup_DWICRSTnIRQ(int enable)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(enable)
	{
		// Enable GPIO used as DECA RESET for interrupt
		GPIO_InitStruct.Pin = DW_RESET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

		HAL_NVIC_EnableIRQ(EXTI0_IRQn);		//pin #0 -> EXTI #0
		HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
		
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);	//pin #0 -> EXTI #0

		//put the pin back to tri-state ... as 
		//output open-drain (not active)
		GPIO_InitStruct.Pin = DW_RESET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_SET);
	}
}


void motor_on(void)
{
	HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
}

void motor_off(void)
{
	HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
}

/* @fn		switch_is_on
 * @brief	check the switch status.
 * 			when switch (S1) is 'on' the pin is low
 * @return  1 if ON and 0 for OFF
 * */
int switch_is_on(uint16_t GPIOpin)
{
	return ((GPIO_ReadInputDataBit(SW_GPIO, GPIOpin))?(0):(1));
}

void expr_on(void)
{
	//HAL_GPIO_TogglePin(EXPR_GPIO_Port, EXPR_Pin);
	HAL_GPIO_WritePin(EXPR_GPIO_Port, EXPR_Pin, GPIO_PIN_SET);
}


void led_toggle (led_t led)
{
	switch (led)
	{
		case LED1:
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			break;
		case LED2:
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			break;
		case LED3:
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			break;
		case LED_ALL:
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			break;
		default:
			break;
	}
}

void led_on (led_t led)
{
	switch (led)
	{
		case LED1:
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			break;
		case LED2:
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			break;
		case LED3:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			break;
		case LED_ALL:
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}


void led_off (led_t led)
{
	switch (led)
	{
		case LED1:
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			break;
		case LED2:
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			break;
		case LED3:
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			break;
		case LED_ALL:
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			break;
		default:
			break;
	}
}


/* @fn		port_wakeup_IC
 * @brief	"slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_IC(void)
{
	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);
    Sleep(1);
	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);
    Sleep(7);						//wait 7ms for DW1000 XTAL to stabilise
}

/* @fn		port_wakeup_IC_fast
 * @brief	waking up of DW1000 using DW_CS and DW_RESET pins.
 * 			The DW_RESET signalling that the DW1000 is in the INIT state.
 * 			the total fast wakeup takes ~2.2ms and depends on crystal startup time
 * */

void port_wakeup_IC_fast(void)
{
	#define WAKEUP_TMR_MS	(10)

	uint32_t x = 0;
	uint32_t timestamp = HAL_GetTick();	//protection

	setup_DWICRSTnIRQ(0); 			//disable RSTn IRQ
	signalResetDone = 0;			//signalResetDone connected to RST_PIN_IRQ
	setup_DWICRSTnIRQ(1); 			//enable RSTn IRQ
	port_SPIx_clear_chip_select();  //CS low

	//need to poll to check when the DW1000 is in the IDLE, the CPLL interrupt is not reliable
	//when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL lock (in 5 us)

	while((signalResetDone == 0) && ((HAL_GetTick() - timestamp) < WAKEUP_TMR_MS))
	{
		x++;	 //when DW1000 will switch to an IDLE state RSTn pin will high
	}
	//usleep(100);
	setup_DWICRSTnIRQ(0); 		//disable RSTn IRQ
	port_SPIx_set_chip_select();  	//CS high

	//usleep(200);
	//it takes ~35us in total for the DW1000 to lock the PLL, download AON and go to IDLE state
	usleep(35);
}



/* @fn		port_set_dwIC_slowrate
 * @brief	set 2.25MHz
 * 			note: hspi1 is clocked from 72MHz
 * */
void port_set_dwIC_slowrate(void)
{
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	HAL_SPI_Init(&hspi1);
}

/* @fn		port_set_dw_ic_spi_fastrate
 * @brief	set 18MHz
 * 			note: hspi1 is clocked from 72MHz
 * */
void port_set_dw_ic_spi_fastrate(void)
{
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&hspi1);
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_dwic_isr()
 *
 * @brief This function is used to install the handling function for DW IC IRQ.
 *
 * NOTE:
 *   - The user application shall ensure that a proper handler is set by calling this function before any DW IC IRQ occurs.
 *   - This function deactivates the DW IC IRQ line while the handler is installed.
 *
 * @param deca_isr function pointer to DW IC interrupt handler to install
 *
 * @return none
 */
void port_set_dwic_isr(port_dwic_isr_t dwic_isr)
{
    /* Check DW IC IRQ activation status. */
    ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW IC IRQ during the installation of the new handler. */
    port_DisableEXT_IRQ();

    port_dwic_isr = dwic_isr;

    if (!en)
    {
        port_EnableEXT_IRQ();
    }
}



/* @fn		port_DisableEXT_IRQ
 * @brief	wrapper to disable DW_IRQ pin IRQ
 * 			in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
	NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn		port_EnableEXT_IRQ
 * @brief	wrapper to enable DW_IRQ pin IRQ
 * 			in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
	NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}


/* @fn		port_GetEXT_IRQStatus
 * @brief	wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
	return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}


/* @fn		port_CheckEXT_IRQ
 * @brief	wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
	return HAL_GPIO_ReadPin(DECAIRQ_GPIO, DW_IRQn_Pin);
}




/* @fn         HAL_GPIO_EXTI_Callback
 * @brief      EXTI line detection callback from HAL layer
 * @param      GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 *             i.e. DW_RESET_Pin and DW_IRQn_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch ( GPIO_Pin )
    {

    case DW_RESET_Pin :
        signalResetDone = 1;
        break;

    case DW_IRQn_Pin :
        {
            //while(HAL_GPIO_ReadPin(DECAIRQ_GPIO, DW_IRQn_Pin) == GPIO_PIN_SET)
            {
               process_deca_irq();
               //dwt_isr();
            }

            break;
        }

    default :
        break;
    }
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW3000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW3000 will clear the IRQ line.
 * */
__INLINE void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {

		if(port_dwic_isr)
        {
            port_dwic_isr();
        }
    } //while DW3000 IRQ line active
}


 
