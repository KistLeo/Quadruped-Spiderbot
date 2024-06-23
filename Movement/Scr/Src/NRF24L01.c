/*
 * NRF24L01.c
 *
 *  Created on: Dec 30, 2023
 *      Author: caoph
 */

#include "stm32f4xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CE_PORT GPIOC
#define NRF24_CE_PIN GPIO_PIN_5

#define NRF24_CSN_PORT GPIOB
#define NRF24_CSN_PIN GPIO_PIN_1

void CS_Select(){
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

void CS_UnSelect(){
	HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}

void CE_Enable(){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_Disable(){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

void nrf24_WriteReg(uint8_t Reg, uint8_t Data){
	uint8_t buf[2];
	buf[0] = Reg | 1<<5;
	buf[1] = Data;

	//Select device
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

	//Release device
	CS_UnSelect();

}

void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *Data, int size){
	uint8_t buf[2];
	buf[0] = Reg | 1<<5;
	//buf[1] = Data;

	//Select device
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
	HAL_SPI_Transmit(NRF24_SPI, Data, size, 1000);

	//Release device
	CS_UnSelect();
}

uint8_t nrf24_ReadReg(uint8_t Reg){
	uint8_t data = 0;

	//Select device
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 1000);

	//Release device
	CS_UnSelect();

	return data;
}

void nrf24_ReadRegMulti(uint8_t Reg, uint8_t *data, int size){

	//Select device
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	//Release device
	CS_UnSelect();
}

void nrf24_SendCmd(uint8_t cmd){
	//Select device
	CS_Select();
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	//Release device
	CS_UnSelect();
}

void nrf24_reset(uint8_t REG)
{
	if (REG == STATUS)
	{
		nrf24_WriteReg(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		nrf24_WriteReg(FIFO_STATUS, 0x11);
	}

	else {
	nrf24_WriteReg(NRF_CONFIG, 0x08);
	nrf24_WriteReg(EN_AA, 0x3F);
	nrf24_WriteReg(EN_RXADDR, 0x03);
	nrf24_WriteReg(SETUP_AW, 0x03);
	nrf24_WriteReg(SETUP_RETR, 0x03);
	nrf24_WriteReg(RF_CH, 0x02);
	nrf24_WriteReg(RF_SETUP, 0x0E);
	nrf24_WriteReg(STATUS, 0x00);
	nrf24_WriteReg(OBSERVE_TX, 0x00);
	nrf24_WriteReg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
	nrf24_WriteReg(RX_PW_P0, 0);
	nrf24_WriteReg(RX_PW_P1, 0);
	nrf24_WriteReg(RX_PW_P2, 0);
	nrf24_WriteReg(RX_PW_P3, 0);
	nrf24_WriteReg(RX_PW_P4, 0);
	nrf24_WriteReg(RX_PW_P5, 0);
	nrf24_WriteReg(FIFO_STATUS, 0x11);
	nrf24_WriteReg(DYNPD, 0);
	nrf24_WriteReg(FEATURE, 0);
	}
}

void NRF24_Init(){
	//Disable chip
	CE_Disable();
	//CS_Unselect();

	//Reset all
	nrf24_reset(0);

	nrf24_WriteReg(NRF_CONFIG, 0);

	nrf24_WriteReg(EN_AA, 0); //No Auto Ack

	nrf24_WriteReg(EN_RXADDR, 0); //Not enable data pipe

	nrf24_WriteReg(SETUP_AW, 0x03); //5 bytes for Tx/Rx address

	nrf24_WriteReg(SETUP_RETR, 0); //No retransmission

	nrf24_WriteReg(RF_CH, 0); //Will be setup during Tx/Rx

	nrf24_WriteReg(RF_SETUP, 0x0E);//Power = 0db, data rate = 2Mbps

	//Enable chip
	CE_Enable();
	//CS_Select();
}

//Setup Tx mode
void NRF24_TxMode(uint8_t *Address, uint16_t channel){
	//Disable chip
	CE_Disable();
	//CS_Unselect();

	nrf24_WriteReg(RF_CH, channel); //Select channel

	nrf24_WriteRegMulti(TX_ADDR, Address, 5); //Write Tx address

	//Power up device
	uint8_t config = nrf24_ReadReg(NRF_CONFIG);
	config = config | (1<<1);
	nrf24_WriteReg(NRF_CONFIG, config);

	//Enable chip
	CE_Enable();
}

//Transmit data
uint8_t NRF24_Transmit(uint8_t *data){
	//Select device
	CS_Select();

	uint8_t cmd, fifostatus;

	//Payload cmd
	cmd = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	//Send payload
	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	//Unselect device
	CS_UnSelect();

	osDelay(1);
	fifostatus = nrf24_ReadReg(FIFO_STATUS);
	if((fifostatus & 1<<4) && (!(fifostatus & 1<<3))){
		cmd = FLUSH_TX;
		nrf24_SendCmd(cmd);
		return 1;
	}
	return 0;
}

void NRF24_RxMode(uint8_t *Address, uint16_t channel){
	//Disable chip
	CE_Disable();

	nrf24_reset(STATUS);

	nrf24_WriteReg(RF_CH, channel); //Select channel

	//Select data pipe 1
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr |= (1<<1);
	nrf24_WriteReg(EN_RXADDR, en_rxaddr);
	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5); //Write the pipe 1 address
	//nrf24_WriteReg(RX_ADDR_P2, Address); //Write the pipe 2 address

	nrf24_WriteReg(RX_PW_P1, 32); //32 bit payload size for pipe 1

	//Power up device
	uint8_t config = nrf24_ReadReg(NRF_CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_WriteReg(NRF_CONFIG, config);

	//Enable chip
	CE_Enable();
}

uint8_t isDataAvailable(int pipenum){
	uint8_t status = nrf24_ReadReg(STATUS);
	if((status & (1<<6)) && (status & (pipenum<<1))){
		nrf24_WriteReg(STATUS, (1<<6));
		return 1;
	}
	return 0;
}

uint8_t NRF24_Receive(uint8_t *data){
	//Select device
	CS_Select();

	uint8_t cmd;

	//Payload cmd
	cmd = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	//Send payload
	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	//Unselect device
	CS_UnSelect();

	osDelay(1);
	cmd = FLUSH_RX;
	nrf24_SendCmd(cmd);
	return 1;
}

// Read all the Register data
void NRF24_ReadAll (uint8_t *data)
{
	for (int i=0; i<10; i++)
	{
		*(data+i) = nrf24_ReadReg(i);
	}

	nrf24_ReadRegMulti(RX_ADDR_P0, (data+10), 5);

	nrf24_ReadRegMulti(RX_ADDR_P1, (data+15), 5);

	*(data+20) = nrf24_ReadReg(RX_ADDR_P2);
	*(data+21) = nrf24_ReadReg(RX_ADDR_P3);
	*(data+22) = nrf24_ReadReg(RX_ADDR_P4);
	*(data+23) = nrf24_ReadReg(RX_ADDR_P5);

	nrf24_ReadRegMulti(RX_ADDR_P0, (data+24), 5);

	for (int i=29; i<38; i++)
	{
		*(data+i) = nrf24_ReadReg(i-12);
	}

}



