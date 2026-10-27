#ifndef INC_RC522_H_
#define INC_RC522_H_
#include "main.h"
void RC522_Init(void);
uint8_t RC522_ReadVersion(void);
uint8_t RC522_ReadUID(uint8_t uid[4]);
#endif
