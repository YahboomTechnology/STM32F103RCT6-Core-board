#ifndef _BSP_SPI_H
#define _BSP_SPI_H

#include "AllHeader.h"

#define FLASH_SPI_CLK RCC_APB1Periph_SPI2
#define FLASH_SPIx SPI2

#define FLASH_SPI_GPIO RCC_APB2Periph_GPIOB

#define FLASH_SPI_CS_PIN 			GPIO_Pin_12
#define FLASH_SPI_CS_PORT 		GPIOB

#define FLASH_SPI_SCK_PIN 		GPIO_Pin_13
#define FLASH_SPI_SCK_PORT 		GPIOB

#define FLASH_SPI_MISO_PIN 		GPIO_Pin_14
#define FLASH_SPI_MISO_PORT 	GPIOB

#define FLASH_SPI_MOSI_PIN 		GPIO_Pin_15
#define FLASH_SPI_MOSI_PORT 	GPIOB

#define SPI_FLASH_CS_HIGH() GPIO_SetBits(GPIOB,FLASH_SPI_CS_PIN);
#define SPI_FLASH_CS_LOW()  GPIO_ResetBits(GPIOB,FLASH_SPI_CS_PIN);

void SPI_FLASH_Init(void);
u8 SPI_ReadWriteByte(u8 TxData);
void SPI1_SetSpeed(u8 SpeedSet);

#endif
