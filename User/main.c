/*******************************************************************/
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"
#include "stm32f30x_tim.h"
#include "stm32f30x.h"
 
 
 
/*******************************************************************/
#define DUMMY				0x00
#define TIMEOUT_TIME			0x1000
 
SPI_InitTypeDef spi;
TIM_TimeBaseInitTypeDef timer;
GPIO_InitTypeDef gpio;
uint8_t receiveData[8];;
uint16_t timeout;
uint8_t tempByte;
int16_t xResult;
float xPosition;
uint8_t xSign;
 
 
 
/*******************************************************************/
void initAll()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
 
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
 
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio);	
 
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    spi.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI1, &spi);
 
    SPI_Cmd(SPI1, ENABLE);
 
    SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
 
    SPI_DataSizeConfig(SPI1, ENABLE);
 
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 720;
    timer.TIM_Period = 2000;
    TIM_TimeBaseInit(TIM2, &timer);
}
 
 
 
/*******************************************************************/
uint8_t sendByte(uint8_t byteToSend)
{
    timeout = TIMEOUT_TIME;
    while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) & (timeout != 0))
    {
	timeout--;
    }		
 
    SPI_SendData8(SPI1, byteToSend);
 
    timeout = TIMEOUT_TIME;	
    while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) & (timeout != 0))
    {
    	timeout--;
    }	
 
    return (uint8_t)SPI_ReceiveData8(SPI1);
}
 

/******************************************************/

void writeData(uint8_t address, uint8_t dataToWrite)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);	
    sendByte(address);
    sendByte(dataToWrite);
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

/******************************************************/

uint8_t readData(uint8_t address)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);	
    sendByte(address);
    tempByte = sendByte(DUMMY);
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
    return tempByte;	
}

/******************************************************/
 
int main()
{
    __enable_irq();
    initAll();
    xPosition = 0;
 
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);
 
    writeData(0x20, 0x0F);
    writeData(0x23, 0x30);
 
    while(1)
    {			
    }
}
 
void TIM2_IRQHandler()
{
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
 
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);
    sendByte(0xE8);	
    receiveData[0] = sendByte(0x00);
    receiveData[1] = sendByte(0x00);
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
 
    xResult = receiveData[0] | (receiveData[1] << 8);
 
    if ((xResult & 0x8000) == 0)
    {
	xSign = 0;					
    }	
    else
    {
	xSign = 1;
    	xResult &= 0x7FFF;
	xResult = 0x7FFF - xResult;
    }
 
    if (xResult < 0x0A)
    {
	xResult = 0;
    }
 
    if (xSign == 0)
    {
	xPosition += 0.07 * xResult * 0.02;
    }
    else
    {
	xPosition -= 0.07 * xResult * 0.02;
    }
}
 
 
 
/*******************************************************************/