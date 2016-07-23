#ifndef _EEPROM_H_
#define _EEPROM_H_

unsigned char IapReadByte(unsigned int addr);
void IapProgramByte(unsigned int addr, unsigned char dat);
void IapEraseSector(unsigned int addr);
void IapIdle();
void IAPRead();
void IAP_Gyro();       //мсбщрг╫цуЩ
void IAP_Angle();
#endif  