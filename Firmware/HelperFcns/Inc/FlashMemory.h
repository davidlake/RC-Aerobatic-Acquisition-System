/*
 * FlashMemory.h
 *
 *  Created on: 23 may. 2021
 *      Author: dlago
 */

#ifndef INC_FLASHMEMORY_H_
#define INC_FLASHMEMORY_H_

#include "main.h"

HAL_StatusTypeDef eraseSector_InternalFlash(uint32_t sectorNumber);
HAL_StatusTypeDef writeStringData_InternalFlash(uint32_t flashAddress, char *pdata); //erases last sector of internal flash memory and writes char data
void readStringData_InternalFlash(uint32_t flashAddress, char *pdata); //reads all data from the address until finds non-written bit

//HAL_Delay(1500); //para evitar que se entre en la instruccion de borrado/escritura de flash antes de que se intente flashear el micro durante el debug
//char data2write[] = "Hola feo";
//char data2read[20] = {};
//writeStringData_InternalFlash(0x08060000, data2write);
//readStringData_InternalFlash(0x08060000, data2read);

#endif /* INC_FLASHMEMORY_H_ */
