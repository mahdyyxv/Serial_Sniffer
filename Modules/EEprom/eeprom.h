#ifndef __EEPROM_H__
#define __EEPROM_H__

// this is the read address 
// when we need to write it should be 0xA0
// so we need to clear the first bit 
// Address &= ~(1<<0)
// then the address will be 0xA0
#define EEPROM_ADDRESS 0xA1 


eReturnStatus udtEEprom_Write(uint8_t Address, uint8_t ByteAddress, const uint8_t *Data, uint8_t size);
eReturnStatus udtEEprom_Read(uint8_t Address, uint8_t ByteAddress, uint8_t *ptrData, uint8_t size);

#endif /*__EEPROM_H__*/
