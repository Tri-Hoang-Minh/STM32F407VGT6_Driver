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
 * TODO: Complete for all other peripheral
 * RCC is managed of AHB1 bus.
 */
#define RCC_BASEADDR              (AHB1PERIPH_BASEADDR + 0x3800)
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
	__vo uint32_t MODER;     /* GPIO port mode register, Address offset: 0x00 */
	__vo uint32_t OTYPER;    /* GPIO port output type register, Address offset: 0x04 */
	__vo uint32_t OSPEEDR;   /* GPIO port output speed register, Address offset:0x08  */
	__vo uint32_t PUPDR;     /* GPIO port pullup/pulldown register, Address offset: 0x0C  */
	__vo uint32_t IDR;       /* GPIO port input data register, Address offset: 0x10  */
	__vo uint32_t ODR;       /* GPIO port ouput data register, Address offset: 0x14 */
	__vo uint32_t BSRR;      /* GPIO port set/reset register, Address offset: 0x18 */
	__vo uint32_t LCKR;      /* GPIO port lock register, Address offset: 0x1C */
	__vo uint32_t AFR[2];    /*AF[0] GPIO alternate function low register, AF[1]: GPIO alternate function high register, Address offset: 0x20 */
} GPIO_RegDef_t;

/*
 * HW: typde structure for RCC driver
 */
typedef struct
{
	__vo uint32_t CR;         /* RCC clock control register, Address offset: 0x00  */
	__vo uint32_t PLLCFGR;    /* RCC configuration register, Address offset: 0x04 */
	__vo uint32_t CFGR;       /* RCC clock configuaration register, Address offset: 0x08 */
	__vo uint32_t CIR;        /* RCC clock interrupt register, Address offset: 0x0C  */
	__vo uint32_t AHB1RSTR;   /* RCC  AHB1 peripheral reset register, Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;   /* RCC  AHB2 peripheral reset register, Address offset: 0x14  */
	__vo uint32_t AHB3RSTR;   /* RCC  AHB3 peripheral reset register, Address offset: 0x18 */
	     uint32_t RESERVED0;  /* Reserved 0x1C */
	__vo uint32_t APB1RSTR;   /* RCC  APB1 peripheral reset  register, Address offset:0x20  */
	__vo uint32_t APB2RSTR;   /* RCC  APB2 peripheral reset register, Address offset: 0x24*/
         uint32_t RESERVED1[2];  /* Reserved 0x28 and 0x2C */
	__vo uint32_t AHB1ENR ;   /* RCC  AHB1 peripheral clock enable register, Address offset: 0x30 */

	__vo uint32_t AHB2ENR;    /* RCC AHB2 peripheral clock enable register Address offset: 0x34  */
	__vo uint32_t AHB3ENR;    /* RCC AHB3 peripheral clock enable register, Address offset: 0x38 */
         uint32_t RESERVED2;  /* Reserved 0x3C */
	__vo uint32_t APB1ENR;    /* RCC APB1 peripheral clock enable register, Address offset: 0x40  */
	__vo uint32_t APB2ENR;    /* RCC APB2 peripheral clock enable register, Address offset: 0x44 */
         uint32_t RESERVED3[2];  /* Reserved 0x48 and 0x4C */
	__vo uint32_t AHB1LPENR;  /* RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;  /* RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54  */
	__vo uint32_t AHB3LPENR;  /* RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
         uint32_t RESERVED4;  /* Reserved 0x5C */
	__vo uint32_t APB1LPENR;  /* RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	__vo uint32_t APB2LPENR;  /* RCC APB2 peripheral clock enabled in low power mode register, Address offset: 0x64 */
         uint32_t RESERVED5[2];  /* Reserved 0x68 and 0x6C */
	__vo uint32_t BDCR ;      /* RCC Backup domain control register, Address offset: 0x70 */

	__vo uint32_t CSR;        /* RCC clock control & status register, Address offset: 0x74  */
         uint32_t RESERVED6[2];  /* Reserved 0x78 and 0x7C */
	__vo uint32_t SSCGR;      /* RCC spread spectrum clock generation register, Address offset: 0x80  */
    __vo uint32_t PLLI2SCFGR; /* RCC PLLI2S configuration registerr, Address offset: 0x84 */
} RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t)
 */
#define GPIOA                   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                   ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                   ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                   ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI                   ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC                      ((RCC_RegDef_t*)RCC_BASEADDR)



/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))
/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))
/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))

/*
 * Clock Enable Macros for SYSCFG peripheral
 * SYSCFG: System configuration controller
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


#endif /* INC_STM32F407XX_H_ */
