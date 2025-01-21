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
#define w25_FAST_READ		0x0b
#define w25_GET_JDEC_ID		0x9f
//макросы для управления ножкой выбора NSS
#define cs_set()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET) //chip select, если ножка в низом уровне, отклик ведомого устройства разрешен, если в высоком, то запрещен
#define cs_reset()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

static void delay_ms(uint16_t delay);

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
  uint8_t high_cap; //если обьем памяти 512 и больше, то аддресация 32 разрядная
}w25_info_t;

w25_info_t  w25_info;

uint32_t time = 0; //delay ms
uint8_t timeFlag = 0;
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
void w25_Read_Data(uint32_t addr, uint8_t *data, uint32_t size){
	cs_set();
	buf[0] = w25_READ;
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = addr & 0xFF;
	SPI1_Send(buf, 4);
	SPI1_Recv(data, size);
	cs_reset();
}
// func для чтения всей страницы
void w25_Read_Page(uint8_t* data, uint32_t page_addr, uint32_t offset, uint32_t size){
	if(size > w25_info.PageSize) // защита, смотрим чтобы размер не превышал страницу
		size = w25_info.PageSize;
	if((offset + size) > w25_info.PageSize) //второй уровень защиты, не превышает ли смещение максимальный addr
		size = w25_info.PageSize - offset;
	page_addr = page_addr * w25_info.PageSize + offset; //в байтах
	//fast read
	buf[0] = w25_FAST_READ;
	if(w25_info.high_cap){
		buf[1] = (page_addr >> 24) & 0xFF;
		buf[2] = (page_addr >> 16) & 0xFF;
		buf[3] = (page_addr >> 8) & 0xFF;
		buf[4] = page_addr & 0xFF;
		cs_set();
		SPI1_Send(buf, 5);
	}
	else{
		buf[1] = (page_addr >> 16) & 0xFF;
		buf[2] = (page_addr >> 8) & 0xFF;
		buf[3] = page_addr & 0xFF;
		cs_set();
		SPI1_Send(buf, 4);
	}
	SPI1_Recv(data, size);
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
	delay_ms(100);
	w25_Reset();
	delay_ms(100);
	unsigned int ID = w25_Read_ID();
	HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
	 sprintf(str,"ID:0x%X\r\n", ID);
	 HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),0x1000);
	 ID &= 0x0000ffff; //маска отсечения manuf ID
	 w25_info.high_cap = 0;
	 switch(ID)
	  {
	    case 0x401A:
	      w25_info.high_cap = 1;
	      w25_info.BlockCount=1024;
	      sprintf(str,"w25qxx Chip: w25q512\r\n");
	      break;
	    case 0x4019:
	      w25_info.high_cap = 1;
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


static void delay_ms(uint16_t delay){
	time = HAL_GetTick();
		while(timeFlag == 0){
			if((HAL_GetTick() - time) > delay){
				timeFlag = 1;
			}
		}
		timeFlag = 0;
}
