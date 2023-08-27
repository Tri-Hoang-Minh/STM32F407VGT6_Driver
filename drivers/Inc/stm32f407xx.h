/*
 * stm32f407xx.h
 *
 *  Created on: Aug 28, 2023
 *      Author: Hoang Tri
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>   // recognize for uint_32
/*
 * Step 1
 * Base address of Flash and SRAM memories.
*/
#define __vo volatile

#define FLASH_BASEADDR      0x08000000U        /* Address of Flash Memory in STM32F407 MCU*/
#define SRAM1_BASEADDR      0x20000000U        /* Address of SRAM1 Memory in STM32F407 MCU*/
#define SRAM2_BASEADDR      0x2001C000U        /* Address of SRAM2 Memory in STM32F407 MCU*/
#define ROM_BASEADRR        0x1FFF0000U        /* Address of System Memory (ROM) in STM32F407 MCU*/
#define SRAM                SRAM1_BASEADDR    /* Base address of SRAM memory*/

/*
 * Step 2
 * AHBx and AHBx Bus Peripheral base addresses.
*/

#define PERIPH_BASE               0x40000000U
#define APB1PERIPH_BASEADDR       PERIPH_BASE
#define APB2PERIPH_BASEADDR       0x40010000U
#define AHB1PERIPH_BASEADDR       0x40020000U
#define AHB2PERIPH_BASEADDR       0x50000000U
/*
 * Base address of peripherals which are hanging in AHB1 bus
 * Step 3
 * TODO: Complete for all other peripheral
 * GPIO is managed of AHB1 bus.
 */
#define GPIOA_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR       (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR       (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR       (AHB1PERIPH_BASEADDR + 0x2000)
/*
 * Base address of peripherals which are hanging in APB1 bus
 * Step 4
 * TODO: Complete for all other peripheral
 */
#define I2C1_BASEADDR         (APB1PERIPH_BASEADDR  + 0x5400)
#define I2C2_BASEADDR         (APB1PERIPH_BASEADDR  + 0x5800)
#define I2C3_BASEADDR         (APB1PERIPH_BASEADDR  + 0x5C00)

#define SPI2_BASEADDR         (APB1PERIPH_BASEADDR  + 0x3800)
#define SPI3_BASEADDR         (APB1PERIPH_BASEADDR  + 0x3C00)

#define USART2_BASEADDR       (APB1PERIPH_BASEADDR  + 0x4400)
#define USART3_BASEADDR       (APB1PERIPH_BASEADDR  + 0x4800)
#define UART4_BASEADDR        (APB1PERIPH_BASEADDR  + 0x4C00)
#define UART5_BASEADDR        (APB1PERIPH_BASEADDR  + 0x5000)

/*
 * Step 5
 * Base address of peripherals which are hanging in APB2 bus
 * TODO: Complete for all other peripheral
 */
#define EXTI_BASEADDR         (APB2PERIPH_BASEADDR  + 0x3C00)
#define SPI1_BASEADDR         (APB2PERIPH_BASEADDR  + 0x3000)
#define SYSCFG_BASEADDR       (APB2PERIPH_BASEADDR  + 0x3800)
#define USART1_BASEADDR       (APB2PERIPH_BASEADDR  + 0x1000)
#define USART6_BASEADDR       (APB2PERIPH_BASEADDR  + 0x1400)


/*
 *  Note: Registers of peripheral are specific to MCU
 *  Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;     /* GPIO port mode register, Address offset:  */
	__vo uint32_t OTYPER;    /* GPIO port mode register, Address offset:  */
	__vo uint32_t OSPEEDR;   /* GPIO port mode register, Address offset:  */
	__vo uint32_t PUPDR;     /* GPIO port mode register, Address offset:  */
	__vo uint32_t IDR;       /* GPIO port mode register, Address offset:  */
	__vo uint32_t ODR;       /* GPIO port mode register, Address offset:  */
	__vo uint32_t BSRR;      /* GPIO port mode register, Address offset:  */
	__vo uint32_t LCKR;      /* GPIO port mode register, Address offset:  */
	__vo uint32_t AFR[2];    /*AF[0] GPIO alternate function low register, AF[1]: GPIO alternate function high register */
} GPIO_RegDef_t;

/*
 * HW: typde structure for RCC driver
 */


#endif /* INC_STM32F407XX_H_ */
