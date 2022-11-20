/*
 * MyHeader.h
 *
 *  Created on: Mar 22, 2022
 */

#ifndef MYHEADER_H_
#define MYHEADER_H_

#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
#define APB1PERIPH_BASE       PERIPH_BASE
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)

#define RCC_ADDR              (AHB1PERIPH_BASE + 0x3800UL)

#define RCC_AHB1ENR_GPIOEEN_Pos            (4U)
#define RCC_AHB1ENR_GPIOEEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOEEN_Pos)   /*!< 0x00000010 */
#define RCC_AHB1ENR_GPIOEEN                RCC_AHB1ENR_GPIOEEN_Msk

typedef struct
{
  volatile unsigned long CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  volatile unsigned long PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  volatile unsigned long CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  volatile unsigned long CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  volatile unsigned long AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  volatile unsigned long AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  volatile unsigned long AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  unsigned long      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  volatile unsigned long APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  volatile unsigned long APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  unsigned long      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  volatile unsigned long AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  volatile unsigned long AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  volatile unsigned long AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  unsigned long      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  volatile unsigned long APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  volatile unsigned long APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  unsigned long      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  volatile unsigned long AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  volatile unsigned long AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  volatile unsigned long AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  unsigned long      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  volatile unsigned long APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  volatile unsigned long APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  unsigned long      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  volatile unsigned long BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  volatile unsigned long CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  unsigned long      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  volatile unsigned long SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  volatile unsigned long PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  volatile unsigned long PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  volatile unsigned long DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
  volatile unsigned long CKGATENR;      /*!< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
  volatile unsigned long DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */
} RCC;

#endif /* MYHEADER_H_ */


#define GPIO_BSRR_BS2_Pos                (2U)
#define GPIO_BSRR_BS2_Msk                (0x1UL << GPIO_BSRR_BS2_Pos)           /*!< 0x00000020 */
#define GPIO_BSRR_BS2                    GPIO_BSRR_BS2_Msk


typedef struct
{
  volatile unsigned long MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  volatile unsigned long OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  volatile unsigned long OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  volatile unsigned long PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile unsigned long IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  volatile unsigned long ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  volatile unsigned long BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  volatile unsigned long LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  volatile unsigned long AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO;

#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOE              	  ((GPIO *) GPIOE_BASE)

#define RCC_BASE                 ((RCC *) RCC_ADDR)

#define GPIO_MODER_OUTPUT		 ( 0x1 )
#define ENABLE_CLOCK_APB2( PERIPH ) { RCC_BASE->APB2ENR |= PERIPH; }
#define ENABLE_CLOCK_AHB1( PERIPH ) { RCC_BASE->AHB1ENR |= PERIPH; }
