#ifndef INC_RC522_H_
#define INC_RC522_H_
#include "main.h"

/* ---- DIAGNOSTIC BUILD: counters + register canaries (driver behavior unchanged) ---- */
typedef struct {
    uint32_t polls;        /* RC522_ReadUID calls                                   */
    uint32_t ok;           /* full UID returned                                     */
    uint32_t reqaOk;       /* a card answered REQA (ATQA, 2 bytes)  <-- key counter */
    uint32_t reqaTimeout;  /* nothing answered REQA (normal when no tag in field)   */
    uint32_t reqaError;    /* REQA stage: ErrorReg flagged / bad length / SPI error */
    uint32_t antiFail;     /* card answered REQA but anticollision failed           */
    uint32_t bccFail;      /* UID checksum mismatch                                 */
    uint32_t spiFail;      /* any HAL SPI call that did not return HAL_OK           */
    uint8_t  lastErr;      /* last non-zero ErrorReg value that caused a failure    */
} RC522_Diag_t;

typedef struct {
    uint8_t version, txControl, tMode, tReloadL, txAsk, mode;
    uint8_t spiOk;         /* 1 if every register read returned HAL_OK */
} RC522_Canary_t;

void RC522_Init(void);
uint8_t RC522_ReadVersion(void);
uint8_t RC522_ReadUID(uint8_t uid[4]);

void RC522_GetDiag(RC522_Diag_t *out);            /* copies counters, then clears them */
uint8_t RC522_ReadCanary(RC522_Canary_t *c);      /* reads version + config registers  */
uint8_t RC522_CanaryOk(const RC522_Canary_t *c);  /* 1 if chip still holds Init config */
void RC522_Reconfigure(void);                     /* rewrite config only (no HW reset) */
#endif


//---------------07-11-26-----------//



//#ifndef INC_RC522_H_
//#define INC_RC522_H_
//#include "main.h"
//void RC522_Init(void);
//uint8_t RC522_ReadVersion(void);
//uint8_t RC522_ReadUID(uint8_t uid[4]);
//#endif
