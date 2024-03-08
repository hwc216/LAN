#ifndef __I2C_H
#define __I2C_H

#include "main.h"

void I2CStart(void);
void I2CStop(void);
unsigned char I2CWaitAck(void);
void I2CSendAck(void);
void I2CSendNotAck(void);
void I2CSendByte(unsigned char cSendByte);
unsigned char I2CReceiveByte(void);
void I2CInit(void);
unsigned char EEPROM_Read(unsigned char addr);            // ¶Áeeprom
void EEPROM_Write(unsigned char addr, unsigned char dat); // Ð´eeprom
void I2C_24c02_Write(unsigned char *pucBuf, unsigned char ucAddr, unsigned char ucNum);
void I2C_24c02_Read(unsigned char *pucBuf, unsigned char ucAddr, unsigned char ucNum);

#endif
