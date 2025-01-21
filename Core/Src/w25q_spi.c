/*
 * w25q_spi.c
 *
 *  Created on: Jan 19, 2025
 *      Author: User1
 */
#include "w25q_spi.h"

#define w25_ENABLE_RESET	0x66
#define w25_RESET			0x99
#define w25_READ			0x03
#define w25_GET_JDEC_ID		0x9f
//макросы для управления ножкой выбора NSS
#define cs_set()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET) //chip select, если ножка в низом уровне, отклик ведомого устройства разрешен, если в высоком, то запрещен
#define cs_reset()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

typedef struct
{
  uint16_t  PageSize;
  uint32_t  PageCount;
  uint32_t  SectorSize;
  uint32_t  SectorCount;
  uint32_t  BlockSize;
  uint32_t  BlockCount;
  uint32_t  NumKB;
  uint8_t   SR1;
  uint8_t   SR2;
  uint8_t   SR3;
}w25_info_t;

w25_info_t  w25_info;

char str[130];
uint8_t buf[10];

void SPI1_Send(uint8_t *dt, uint16_t cnt){
	HAL_SPI_Transmit(&hspi1, dt, cnt, 5000);
}
void SPI1_Recv(uint8_t *dt, uint16_t cnt){
	HAL_SPI_Receive(&hspi1, dt, cnt, 5000);
}
void w25_Reset(void){
	cs_set();
	buf[0] = w25_ENABLE_RESET;
	buf[1] = w25_RESET;
	SPI1_Send(buf, 2);
	cs_reset();
}
//func для начала чтения из flash-ки записываем команду 03 и 24 битный аддрес
void w25_Read_Data(uint32_t addr, uint8_t *dat, uint32_t sz){
	cs_set();
	buf[0] = w25_READ;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	SPI1_Send(buf, 4);
	SPI1_Recv(dat, sz);
	cs_reset();
}
uint32_t w25_Read_ID(void){
	uint8_t dt[3];
	buf[0] = w25_GET_JDEC_ID;
	cs_set();
	SPI1_Send(buf, 1);
	SPI1_Recv(dt, 3);
	cs_reset();
	return (dt[0] << 16 | dt[1] << 8) | dt[2];
}
void w25_Init(void){
	HAL_Delay(100);
	w25_Reset();
	HAL_Delay(100);
	unsigned int ID = w25_Read_ID();
	HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
	 sprintf(str,"ID:0x%X\r\n", ID);
	 HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	 ID &= 0x0000ffff; //маска отсечения manuf ID
	 switch(ID)
	  {
	    case 0x401A:
	      w25_info.BlockCount=1024;
	      sprintf(str,"w25qxx Chip: w25q512\r\n");
	      break;
	    case 0x4019:
	      w25_info.BlockCount=512;
	      sprintf(str,"w25qxx Chip: w25q256\r\n");
	      break;
	    case 0x4018:
	      w25_info.BlockCount=256;
	      sprintf(str,"w25qxx Chip: w25q128\r\n");
	      break;
	    case 0x4017:
	      w25_info.BlockCount=128;
	      sprintf(str,"w25qxx Chip: w25q64\r\n");
	      break;
	    case 0x4016:
	      w25_info.BlockCount=64;
	      sprintf(str,"w25qxx Chip: w25q32\r\n");
	      break;
	    case 0x4015:
	      w25_info.BlockCount=32;
	      sprintf(str,"w25qxx Chip: w25q16\r\n");
	      break;
	    case 0x4014:
	      w25_info.BlockCount=16;
	      sprintf(str,"w25qxx Chip: w25q80\r\n");
	      break;
	    case 0x4013:
	      w25_info.BlockCount=8;
	      sprintf(str,"w25qxx Chip: w25q40\r\n");
	      break;
	    case 0x4012:
	      w25_info.BlockCount=4;
	      sprintf(str,"w25qxx Chip: w25q20\r\n");
	      break;
	    case 0x4011:
	      w25_info.BlockCount=2;
	      sprintf(str,"w25qxx Chip: w25q10\r\n");
	      break;
	    default:
	      sprintf(str,"w25qxx Unknown ID\r\n");
	      HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	      return;
	  }
	  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	  HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   w25_info.PageSize=256;
	   w25_info.SectorSize=0x1000;
	   w25_info.SectorCount=w25_info.BlockCount*16;
	   w25_info.PageCount=(w25_info.SectorCount*w25_info.SectorSize)/w25_info.PageSize;
	   w25_info.BlockSize=w25_info.SectorSize*16;
	   w25_info.NumKB=(w25_info.SectorCount*w25_info.SectorSize)/1024;
	   sprintf(str,"Page Size: %d Bytes\r\n",(unsigned int)w25_info.PageSize);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Page Count: %u\r\n",(unsigned int)w25_info.PageCount);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Sector Size: %u Bytes\r\n",(unsigned int)w25_info.SectorSize);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Sector Count: %u\r\n",(unsigned int)w25_info.SectorCount);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Block Size: %u Bytes\r\n",(unsigned int)w25_info.BlockSize);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Block Count: %u\r\n",(unsigned int)w25_info.BlockCount);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	   sprintf(str,"Capacity: %u KB\r\n",(unsigned int)w25_info.NumKB);
	   HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);

}
