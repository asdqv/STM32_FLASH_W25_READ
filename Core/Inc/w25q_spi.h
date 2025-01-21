/*
 * w25q_spi.h
 *
 *  Created on: Jan 19, 2025
 *      Author: User1
 */

#ifndef INC_W25Q_SPI_H_
#define INC_W25Q_SPI_H_

#include "stm32f1xx_hal.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>

void SPI1_Send(uint8_t *dt, uint16_t cnt);
void SPI1_Recv(uint8_t *dt, uint16_t cnt);
void w25_Reset(void);
void w25_Read_Data(uint32_t addr, uint8_t *dat, uint32_t sz);
void w25_Read_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t size);
uint32_t w25_Read_ID(void);
void w25_Init(void);



#endif /* INC_W25Q_SPI_H_ */
