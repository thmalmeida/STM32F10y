/*
 * eeprom.cpp
 *
 *  Created on: 27 de fev de 2017
 *      Author: titi
 */

#include "stm32f10x.h"

/*
 * STM32F103R8T6 is a middle density devide wich has 64 kB of flash declared on datasheet.
 * But, it has 128 kB of flash. The other 64 kB flash memory part is hided and can be used
 * like as eeprom to store variables.
 *
 * The address goes from 0x8000-0000 to 0x8001-FFF0 (131056 bytes)
 */
/* Virtual address defined by the user: 0xFFFF value is prohibited */
//uint16_t VirtAddVarTab[NumbOfVar] = {0x0000, 0x0004, 0x0008};
//uint16_t addr = 0x000C;

class EEPROM {
public:
	//note that the starting address for flash is 0x8000000 for F103 and F4
	//you need to include flash.h in ST library
	//what I did here is write a 16 by 16 2D array to FLASH starting at address 0x8040000(256KB, in the middle of FLASH since my STM32 has 512K FLASH) and then read it back to memory afterwards.
	//notice you will lose the data in flash if you erase all flash when you download new code from computer
//	uint32_t startAddress = 0x804000;//starting from 256KB
	                                  //0x8020000 starting 128KB
//	uint32_t startAddress = 0x8000FA00;	//starting from 64KB
	// this page goes until 0x8000FFFF;

	static const uint32_t startAddress = 0x08000000;

	uint8_t pageSet = 64;

	static const uint16_t pageSize = 0x0400;
	uint16_t memVect[pageSize];

//	ErrorStatus  HSEStartUpStatus;
//	FLASH_Status FlashStatus;

	// Addresses
	const uint8_t addr_stateMode 			= 1;		// 1 bytes
	const uint8_t addr_LevelRef 			= 2;		// 4 bytes
	const uint8_t addr_standBy_min 			= 5;		// 2 bytes
	const uint8_t addr_PRessureRef 			= 7;		// 1 byte
	const uint8_t addr_PRessureRef_Valve	= 8;		// 1 byte
	const uint8_t addr_PRessureMax_Sensor	= 9;		// 1 byte
	const uint8_t addr_motorTimerE 			= 10;		// 2 byte
	const uint8_t addr_HourOnTM 			= 30;		// 9 bytes
	const uint8_t addr_MinOnTM 				= 39;		// nTM byte(s)
	const uint8_t addr_nTM 					= 48;		// 1 byte
	const uint8_t addr_motorTimerStart1 	= 65;		// 2 bytes
	const uint8_t addr_motorTimerStart2 	= 67;		// 2 bytes
	const uint8_t addr_PRessurePer 			= 69;		// 1 byte
	const uint8_t addr_rtc_PRL				= 80; 		// 2 bytes
	const uint8_t addr_rtc_clkSource		= 82;		// 1 byte

	void begin_eeprom(void);
	void end_eeprom(void);

	void setPage(uint8_t page);
	uint32_t pageAddress(uint8_t page);

	void erasePage(uint8_t page);
	void writePage(uint8_t page, uint16_t value);

	uint16_t read(uint8_t page, uint8_t addr);
	void write(uint8_t page, uint8_t addr, uint16_t var);

	void routine1(void);

private:
	void fillArrayFromPage(uint8_t page);
	void fillArrayToPage(uint8_t page);
};


void EEPROM::begin_eeprom(void)
{
	FLASH_Unlock();		// Unlock the Flash Program Erase controller
}
void EEPROM::end_eeprom(void)
{
	FLASH_Lock();
}
uint32_t EEPROM::pageAddress(uint8_t page)
{
	return (startAddress + page*0x0400);
}
void EEPROM::setPage(uint8_t page)
{
	pageSet = page;
}
void EEPROM::erasePage(uint8_t page)
{
	if(FLASH->CR & FLASH_CR_LOCK)				// 2- verify if its unlocked
	{
		FLASH_Unlock();									// 1- unlock FLASH_CR
	}

	// Erase page;
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(pageAddress(page));//erase the entire page before you can write as I //mentioned
}
uint16_t EEPROM::read(uint8_t page, uint8_t addr)
{
	fillArrayFromPage(page);

	return memVect[addr];
}
void EEPROM::fillArrayFromPage(uint8_t page)
{
	uint32_t k;
	for(k=0; k<pageSize; k++)
		memVect[k] = *(uint16_t *)(pageAddress(page) + (k)*2);
}
void EEPROM::fillArrayToPage(uint8_t page)
{
	uint16_t k;

	while(FLASH->SR & FLASH_SR_BSY);			// 3- Check if there is an existing process working;
	FLASH->CR |= FLASH_CR_PG;					// 3- Write ProGram bit FLASH_CR_PG to 1
	for(k=0; k<pageSize; k++)
	{
		FLASH_ProgramHalfWord((pageAddress(page) + (k)*2), memVect[k]);
	}
	while(FLASH->SR &= FLASH_SR_BSY);			// 5- Wait for the BSY bit to be reset.
}
void EEPROM::write(uint8_t page, uint8_t addr, uint16_t var)
{
	if(FLASH->CR & FLASH_CR_LOCK)				// 2- verify if its unlocked
	{
		FLASH_Unlock();									// 1- unlock FLASH_CR
	}

	// Restore process;
	fillArrayFromPage(page);
	memVect[addr] = var;

	erasePage(page);

	// Write page;
	fillArrayToPage(page);

	end_eeprom();
}
void EEPROM::writePage(uint8_t page, uint16_t value)
{
	if(FLASH->CR & FLASH_CR_LOCK)				// 2- verify if its unlocked
	{
		FLASH_Unlock();									// 1- unlock FLASH_CR
	}

	erasePage(page);

	uint16_t k;

	while(FLASH->SR & FLASH_SR_BSY);			// 3- Check if there is an existing process working;
	FLASH->CR |= FLASH_CR_PG;					// 3- Write ProGram bit FLASH_CR_PG to 1
	for(k=0; k<pageSize; k++)
	{
		FLASH_ProgramHalfWord((pageAddress(page) + (k)*2), value);
	}
	while(FLASH->SR &= FLASH_SR_BSY);			// 5- Wait for the BSY bit to be reset.

	end_eeprom();
}







//		fs = FLASH_ProgramHalfWord(addr1, data1);	// 4- Perfom half-word write at the desired address;
//		for(i=0; i<mSize; i++)
//			for(j=0; j<mSize; j++)
//				mazeWalls[i][j] = *(uint16_t *)(startAddress + (i*mSize+j)*2);

//		data2 = *(uint16_t *)(addr1);
//		data2 = (*(__IO uint16_t*)(addr1 - 2));	// 6- Check the programmed value by reading the programmed address

// Page Erase sequence
////		if((FLASH->CR & FLASH_CR_LOCK));	// First of all, its necessary to unlock the flash
//		while(FLASH->SR &= FLASH_SR_BSY);	// Check if there is some memory operation on BSY bit
//		FLASH->CR |= FLASH_CR_PER;			// Set the PER bit in the FLASH_CR register
//		FLASH->AR  = addr1;					// Program the Flash Address FLASH_AR register to select a page to erase
//		FLASH->CR |= FLASH_CR_STRT;			// Set the STRT bit in the FLASH_CR register
//		while(FLASH->SR &= FLASH_SR_BSY); 	// Wait for the BSY bit to be reset
									// Read the erased page and verify

//	for(i=0; i<mSize; i++)
//		for(j=0; j<mSize; j++)
//			FLASH_ProgramHalfWord((startAddress + (i*mSize+j)*2),mazeWalls[i][j]);
	//	for(i=0; i<mSize; i++)
	//		for(j=0; j<mSize; j++)
	//			mazeWalls[i][j] = *(uint16_t *)(startAddress + (i*mSize+j)*2);
		//	for(i=0; i<mSize; i++)
		//		for(j=0; j<mSize; j++)
		//		{
		//			mazeWalls[i][j] = *(uint16_t *)(startAddress + (i*mSize+j)*2);
		//			sprintf(Serial.buffer,"%ld", mazeWalls[i][j]);
		//			Serial.println(Serial.buffer);
		//		}

