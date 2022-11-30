#ifndef __LOG_H__
#define __LOG_H__

#include "main.h"

void log_init(void);

void PrintDat(uint8_t* datAddr, uint32_t num);
void PrintfDat(uint8_t* datAddr, uint32_t num, const char* prefix);
void PrintfDat16(uint16_t* datAddr, uint32_t num, const char* prefix);
void PrintfDat32(uint32_t* datAddr, uint32_t num, const char* prefix);

#endif /*__LOG_H__ */
