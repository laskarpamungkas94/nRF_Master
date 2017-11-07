/*
 * nRF24L01P.c
 *
 *  Created on: 2014/9/29
 *      Author: Jone
 */
//	Pin :
//						GPIO20(O) -> 3(CE)
//						GPIO19(O) -> 4(CSN)
//						GPIO18(O) -> 5(SCK)
//						GPIO16(O) -> 6(MOSI)
//						GPIO17(I) -> 7(MISO)
//						GPIO21(I) -> 8(IRQ)
//
//--------------------------------------------------------------------------------

#include "PeripheralHeaderIncludes.h"
#include "Ctrl_A.h"
#include "nRF24L01P.h"

#define RX_ADDRESS		0x0EE502	// TX address
Uint32 Com = 0;
//Uint32 Bug_Count = 0;
Uint32 Command = 0;

Uint32 SPI_Read(Uint16 Command, Uint16 Bytes)
{
	Uint32 Stat, Temp;

	if(Bytes == 2)
	{
		Command <<= 8;
		SpiaRegs.SPITXBUF = Command;

		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
	}
	else if(Bytes == 4)
	{
		Command <<= 8;
		SpiaRegs.SPITXBUF = Command;
		SpiaRegs.SPITXBUF = 0x0;

		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
		Temp <<= 16;
		Com = Temp;
		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Stat = SpiaRegs.SPIRXBUF;
		Temp |= Stat;
		Com |= Stat;
	}

	return Temp;
}

Uint32 SPI_Rx_Read(void)		// 0xAA = nRF_VADC, 0xEA = nRF_IADC
{
	Uint16 Temp;

	SpiaRegs.SPITXBUF = 0x61FF;
	SpiaRegs.SPITXBUF = 0xFFFF;
	SpiaRegs.SPITXBUF = 0xFFFF;

	while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
	Temp = SpiaRegs.SPIRXBUF;
	Temp &= 0x00FF;
	/*if(ID == 0x00AA)
	{
		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
		Temp <<= 16;
		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp |= SpiaRegs.SPIRXBUF;
		Temp = 0xAAFFFFFF;
	}
	if(ID == 0x00EA)
	{
		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
		Temp <<= 16;
		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp |= SpiaRegs.SPIRXBUF;
		Temp = 0xEAFFFFFF;
	}
	if(ID == 0x00FF)
	{
		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
		Temp = 0xFF000000;
		Bug_Count++;
	}*/
	return Temp;
}

int SPI_Write(int Command, Uint32 Data, Uint16 Bytes)
{
	Uint32 Temp;

	if(Bytes == 2)
	{
		Command <<= 8;
		Command |= Data;

		SpiaRegs.SPITXBUF = Command;

		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
	}
	else if(Bytes == 4)
	{
		Command <<= 8;
		Temp = Data;
		Temp >>= 16;
		Command |= Temp;

		SpiaRegs.SPITXBUF = Command;
		Data &= 0xFFFF;
		SpiaRegs.SPITXBUF = Data;

		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;

		while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
		Temp = SpiaRegs.SPIRXBUF;
	}
	return Command;
}

void SPI_Rx_DR(void)
{
	SpiaRegs.SPITXBUF = 0x2740;

	while(SpiaRegs.SPIFFRX.bit.RXFFST == 0);
	Command = SpiaRegs.SPIRXBUF;
}

int RX_Mode(void)
{
	GpioDataRegs.GPADAT.bit.GPIO7 = 0;				//CE = 0		Chip disable

	//SPI_Write(W_REGISTER + CONFIG, 0x7B, 2);			// Enable CRC(1 bytes), PWR_UP, PRX
	//SPI_Write(W_REGISTER + CONFIG, 0x3B, 2);			// RX_DR, Enable CRC(1 bytes), PWR_UP, PRX Enable
	SPI_Write(W_REGISTER + CONFIG, 0x3F, 2);			// RX_DR, Enable CRC(2 bytes), PWR_UP, PRX Enable
	SPI_Write(W_REGISTER + EN_AA, 0x00, 2);				// Disable Auto.Ack pipe 0
	SPI_Write(W_REGISTER + EN_RXADDR, 0x01, 2);			// Enable RX_Pipe 0
	SPI_Write(W_REGISTER + SETUP_AW, 0x01, 2);			// Setup of address width (3 bytes)
	SPI_Write(W_REGISTER + SETUP_RETR, 0x00, 2);		// Disable Automatic Retransmission
	SPI_Write(W_REGISTER + RF_CH, 0x52, 2);				// Select RF channel 52
	SPI_Write(W_REGISTER + RF_SETUP, 0x0E, 2);			// RF_PWR : 0dBm, DataRate : 2Mbps
	//SPI_Write(W_REGISTER + RF_SETUP, 0x06, 2);			// RF_PWR : 0dBm, DataRate : 1Mbps
	SPI_Write(W_REGISTER + RX_ADDR_P0, RX_ADDRESS, 4);	// Define RX_Address_P0
	SPI_Write(W_REGISTER + RX_PW_P0, 0x05, 2);			// RX PayLoad 3 Bytes

	GpioDataRegs.GPADAT.bit.GPIO7 = 1;				//CE = 1		Chip enable

	return 0;
}

int nRF_Init(void)
{

	GpioDataRegs.GPADAT.bit.GPIO7 = 0;				//CE = 0		Chip disable

	RX_Mode();

	return 0;
}
