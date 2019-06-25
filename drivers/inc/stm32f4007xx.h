/*
 * stm32f4007xx.h
 *
 *  Created on: Jun 18, 2019
 *      Author: subhe
 */

#ifndef INC_STM32F4007XX_H_
#define INC_STM32F4007XX_H_

#include <stdint.h>

#define __vo volatile
/* Define addresses of flash and SRAM memories */

#define FLASH_BASEADDR			0x00000000U
#define SRAM1_BASEADDR			0x20000000U //112 kb
#define SRAM2_BASEADDR			0x20001C00U
#define ROM_BASEADDR			0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

/* define  bus addresses */
#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/* define bus addresses of peripherals hanging on AHB1*/

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR+0x1000)
#define GPIOF_BASEADDR 			(AHB1PERIPH_BASEADDR+0x1400)
#define GPIOG_BASEADDR 			(AHB1PERIPH_BASEADDR+0x1800)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR+0x1C00)
#define GPIOI_BASEADDR 			(AHB1PERIPH_BASEADDR+0x2000)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR+0x3800)

/* define bus addresses of peripherals hanging on APB1 bus*/

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR+0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR+0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR+0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR+0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR+0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR+0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR+0x5000)

/* define bus addresses of peripherals hanging on APB2 bus*/
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR+0x3C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR+0x3000)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR+0x3800)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR+0x1400)


/* structure for peripheral structures */

typedef struct
{
	__vo uint32_t MODER; 		/* refers to the base GPIO address ;GPIO port mode register*/
	__vo uint32_t OTYPER;	 	/*address offset 0x04; GPIO port output type register */
	__vo uint32_t OSPEEDR;		/*address offset 0x08; GPIO port output speed register*/
	__vo uint32_t PUPDR;		/*address offset 0x0C;GPIO port pull-up/pull-down register*/
	__vo uint32_t IDR;			/*address offset 0x10;GPIO port input data register*/
	__vo uint32_t ODR;			/*address offset 0x14;GPIO port output data register*/
	__vo uint32_t BSRR;			/*address offset 0x18; GPIO port bit set/reset register*/
	__vo uint32_t LCKR;			/*address offset 0x1C; GPIO port configuration lock register*/
	__vo uint32_t AFR[2];		/*address offset 0x20 0x24; GPIO alternate function low register [0]-low and [1]High*/

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR; /*point to base RCC register 0x00*/
	__vo uint32_t PLLCFGR; 				/* offset 0x04 */
	__vo uint32_t RCC_CFGR; 			/* offset 0x08 */
	__vo uint32_t RCC_CIR; 				/* offset 0x0C*/
	__vo uint32_t AHB1RSTR; 			/* offset 0x10 */
	__vo uint32_t AHB2RSTR; 			/* offset 0x14 */
	__vo uint32_t AHB3RSTR;				 /* offset 0x18 */
	 uint32_t RESERVED0;				/* offset 0x1C*/
	__vo uint32_t APB1RSTR; 			/* offset 0x20 */
	__vo uint32_t APB2RSTR; 			/* offset 0x24 */
	 uint32_t RESERVED1[2];			/* reserved 0x28-0x2C*/
	__vo uint32_t AHB1ENR; 				/* offset 0x30 */
	__vo uint32_t AHB2ENR;				 /* offset 0x34 */
	__vo uint32_t AHB3ENR;				 /* offset 0x38 */
	__vo uint32_t APB1ENR;				 /* offset 0x40 */
	__vo uint32_t APB2ENR;				 /* offset 0x44 */
	 uint32_t RESERVED2[2];			/* reserved 0x48-0x4C*/
	__vo uint32_t AHB1LPENR;				 /* offset 0x50 */
	__vo uint32_t AHB2LPENR;				 /* offset 0x54 */
	__vo uint32_t AHB3LPENR;				 /* offset 0x58 */
	uint32_t RESERVED3;			/* reserved 0x28-0x5C*/
	__vo uint32_t APB1LPENR;				 /* offset 0x60 */
	__vo uint32_t APB2LPENR;				 /* offset 0x64 */
	uint32_t RESERVED4[2];			/* reserved 0x68-0x6C*/
	__vo uint32_t BDCR;				 /* offset 0x70 */
	__vo uint32_t CSR;				 /* offset 0x74 */
	uint32_t RESERVED5[2];			/* reserved 0x78-0x7C*/
	__vo uint32_t SSCGR;				 /* offset 0x80 */
	__vo uint32_t PLLI2SCFGR;				 /* offset 0x84 */


} RCC_RegDef_t;


/* Peripheral definition (Peripheral base address type casted to the xxx_REgDef_t */

#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

/*
* clock enable macro for GPIOx peripherals
*/

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR|=(0x1<<8))
/*
* clock enable macro for I2Cx peripherals
*/
#define IC21_PCLK_EN() (RCC->APB1ENR|=(0x1<<21))
#define IC22_PCLK_EN() (RCC->APB1ENR|=(0x1<<22))
#define IC23_PCLK_EN() (RCC->APB1ENR|=(0x1<<23))

/*
* clock enable macro for SPIx peripherals
*/
#define SPI2_PCLK_EN() (RCC->APB1ENR|=(0x1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR|=(0x1<<15))
#define SPI1_PCLK_EN() (RCC->APB2ENR|=(0x1<<12))

/*
* clock enable macro for USARTx peripherals
*/
#define UASRT2_PCCK_EN() (RCC->APB1ENR|=(0x1<<17))
#define UASRT3_PCCK_EN() (RCC->APB1ENR|=(0x1<<18))
#define UASRT1_PCCK_EN() (RCC->APB2ENR|=(0x1<<4))
#define UART4_PCCK_EN()  (RCC->APB1ENR)|=(0x1<<19)
#define UART5_PCCK_EN()  (RCC->APB1ENR)|=(0x1<<20)
#define UASRT6_PCCK_EN() (RCC->APB2ENR|=(0x1<<5))

/*
* clock enable macro for SYSCFG peripherals
*/

#define SYSCFG_PCLK_EN 	(RCC->APB2ENR |=(0x1<<14))

/*
*  Macros to di Init  for GPIOx peripherals
*/
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR|=(0x1<<0)); (RCC->AHB1RSTR &=~(0x1<<0));} while(0)
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<1)); (RCC->AHB1RSTR &=~(0x1<<1));}  while(0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<2)); (RCC->AHB1RSTR &=~(0x1<<2));} while(0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<3)); (RCC->AHB1RSTR &=~(0x1<<3));} while(0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<4)); (RCC->AHB1RSTR &=~(0x1<<4));} while(0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<5)); (RCC->AHB1RSTR &=~(0x1<<5));} while(0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<6)); (RCC->AHB1RSTR &=~(0x1<<6));} while(0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<7)); (RCC->AHB1RSTR &=~(0x1<<7));} while(0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR|=(0x1<<8)); (RCC->AHB1RSTR &=~(0x1<<8));} while(0)
/*
* clock dis-able macro for I2Cx peripherals
*/
#define IC21_PCLK_DI() (RCC->APB1ENR &=~(0x1<<21))
#define IC22_PCLK_DI() (RCC->APB1ENR &=~(0x1<<22))
#define IC23_PCLK_DI() (RCC->APB1ENR &=~(0x1<<23))

/*
* clock dis-able macro for SPIx peripherals
*/
#define SPI2_PCLK_DI() (RCC->APB1ENR &=~(0x1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &=~(0x1<<15))
#define SPI1_PCLK_DI() (RCC->APB2ENR &=~(0x1<<12))

/*
* clock dis-able macro for USARTx peripherals
*/
#define UASRT2_PCLK_DI() (RCC->APB1ENR &=~(0x1<<17))
#define UASRT3_PCLK_DI() (RCC->APB1ENR &=~(0x1<<18))
#define UASRT1_PCLK_DI() (RCC->APB2ENR &=~(0x1<<4))
#define UASRT6_PCLK_DI() (RCC->APB2ENR &=~(0x1<<5))


/*
* clock dis-able macro for SYSCFG peripherals
*/
#define SYSCFG_PCLK_DI 	(RCC->APB2ENR &=~(0x1<<14))

/*
 * Some generic macro
 *
 */
#define ENABLE		 	1
#define DISABLE		 	0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */

