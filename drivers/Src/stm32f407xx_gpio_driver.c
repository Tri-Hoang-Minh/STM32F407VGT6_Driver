/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 28, 2023
 *      Author: Hoang Tri
 */
#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
    	if (pGPIOx == GPIOA)
    	{
    		GPIOA_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOB)
    	{
    		GPIOB_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOC)
    	{
    		GPIOC_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOD)
    	{
    		GPIOD_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOE)
    	{
    		GPIOE_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOF)
    	{
    		GPIOF_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOG)
    	{
    		GPIOG_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOH)
    	{
    		GPIOH_PCLK_EN();
    	}
    	else if (pGPIOx == GPIOI)
    	{
    		GPIOI_PCLK_EN();
    	}
    }
    else
    {
    	if (pGPIOx == GPIOA)
    	{
    		GPIOA_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOB)
    	{
    	    GPIOB_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOC)
    	{
    		GPIOC_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOD)
    	{
    		GPIOD_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOE)
    	{
    		GPIOE_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOF)
    	{
    		GPIOF_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOG)
    	{
    		GPIOG_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOH)
    	{
    		GPIOH_PCLK_DI();
    	}
    	else if (pGPIOx == GPIOI)
    	{
    		GPIOI_PCLK_DI();
    	}


    }
}


/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

}
