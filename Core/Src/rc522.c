/* rc522.c -- DIAGNOSTIC BUILD
 * Same driver logic as the uploaded rc522.c, plus:
 *  - failure-stage counters (RC522_Diag_t)
 *  - config factored into applyConfig() (same writes, same order as before)
 *  - register "canaries" to detect that the chip lost its configuration
 * No timing or protocol behavior has been changed in this build. */
#include "rc522.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;
#define CommandReg       0x01U
#define ComIrqReg        0x04U
#define ErrorReg         0x06U
#define FIFODataReg      0x09U
#define FIFOLevelReg     0x0AU
#define ControlReg       0x0CU
#define BitFramingReg    0x0DU
#define ModeReg          0x11U
#define TxControlReg     0x14U
#define TxASKReg         0x15U
#define RFCfgReg         0x26U
#define TModeReg         0x2AU
#define TPrescalerReg    0x2BU
#define TReloadRegH      0x2CU
#define TReloadRegL      0x2DU
#define VersionReg       0x37U
#define PCD_IDLE         0x00U
#define PCD_TRANSCEIVE   0x0CU
#define PCD_SOFTRESET    0x0FU
#define PICC_REQA        0x26U
#define PICC_ANTICOLL    0x93U
#define RC522_TIMEOUT_MS 25U

/* A/B test only, default off: 1 = receiver gain 33 dB -> 48 dB (more range, but also more noise). */
#ifndef RC522_RX_GAIN_MAX
#define RC522_RX_GAIN_MAX 0U
#endif

typedef enum { RC_OK=0, RC_TIMEOUT, RC_ERROR, RC_NO_ROOM } RC_Status;

static RC522_Diag_t diag;
static uint8_t expectedVersion = 0U;

static HAL_StatusTypeDef writeReg(uint8_t reg, uint8_t value) {
  uint8_t tx[2]={(uint8_t)((reg<<1U)&0x7EU),value};
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_RESET);
  HAL_StatusTypeDef st=HAL_SPI_Transmit(&hspi2,tx,2U,RC522_TIMEOUT_MS);
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_SET);
  if(st!=HAL_OK) diag.spiFail++;
  return st;
}
static HAL_StatusTypeDef readReg(uint8_t reg,uint8_t *value) {
  if(!value) return HAL_ERROR;
  uint8_t addr=(uint8_t)(((reg<<1U)&0x7EU)|0x80U),dummy=0U,discard=0U;
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_RESET);
  HAL_StatusTypeDef st=HAL_SPI_TransmitReceive(&hspi2,&addr,&discard,1U,RC522_TIMEOUT_MS);
  if(st==HAL_OK) st=HAL_SPI_TransmitReceive(&hspi2,&dummy,value,1U,RC522_TIMEOUT_MS);
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_SET);
  if(st!=HAL_OK) diag.spiFail++;
  return st;
}
static uint8_t readOrZero(uint8_t reg){uint8_t v=0U;(void)readReg(reg,&v);return v;}
static HAL_StatusTypeDef setBits(uint8_t reg,uint8_t mask){uint8_t v;if(readReg(reg,&v)!=HAL_OK)return HAL_ERROR;return writeReg(reg,(uint8_t)(v|mask));}
static HAL_StatusTypeDef clearBits(uint8_t reg,uint8_t mask){uint8_t v;if(readReg(reg,&v)!=HAL_OK)return HAL_ERROR;return writeReg(reg,(uint8_t)(v&(uint8_t)~mask));}

static RC_Status transceive(const uint8_t *tx,uint8_t txLen,uint8_t *rx,uint8_t *rxLen,uint8_t lastBits){
  uint8_t irq=0U,err=0U,count=0U,control=0U; uint32_t started;
  if(!tx||!txLen||!rxLen||(*rxLen&&!rx))return RC_ERROR;
  if(writeReg(CommandReg,PCD_IDLE)!=HAL_OK||writeReg(ComIrqReg,0x7FU)!=HAL_OK||writeReg(FIFOLevelReg,0x80U)!=HAL_OK||writeReg(BitFramingReg,lastBits&7U)!=HAL_OK)return RC_ERROR;
  for(uint8_t i=0;i<txLen;i++)if(writeReg(FIFODataReg,tx[i])!=HAL_OK)return RC_ERROR;
  if(writeReg(CommandReg,PCD_TRANSCEIVE)!=HAL_OK||setBits(BitFramingReg,0x80U)!=HAL_OK)return RC_ERROR;
  started=HAL_GetTick();
  do { if(readReg(ComIrqReg,&irq)!=HAL_OK){(void)clearBits(BitFramingReg,0x80U);return RC_ERROR;} if(irq&0x31U)break; } while((uint32_t)(HAL_GetTick()-started)<RC522_TIMEOUT_MS);
  (void)clearBits(BitFramingReg,0x80U); if(!(irq&0x30U))return RC_TIMEOUT;
  if(readReg(ErrorReg,&err)!=HAL_OK)return RC_ERROR;
  if(err&0x1BU){diag.lastErr=err;return RC_ERROR;}
  if (readReg(FIFOLevelReg, &count) != HAL_OK) {
      return RC_ERROR;
  }

  if (count > *rxLen) {
      return RC_NO_ROOM;
  }
  *rxLen=count; for(uint8_t i=0;i<count;i++)if(readReg(FIFODataReg,&rx[i])!=HAL_OK)return RC_ERROR;
  (void)readReg(ControlReg,&control); return RC_OK;
}

static void resetChip(void){HAL_GPIO_WritePin(RFID_RST_GPIO_Port,RFID_RST_Pin,GPIO_PIN_RESET);HAL_Delay(5U);HAL_GPIO_WritePin(RFID_RST_GPIO_Port,RFID_RST_Pin,GPIO_PIN_SET);HAL_Delay(50U);(void)writeReg(CommandReg,PCD_SOFTRESET);HAL_Delay(50U);}

static void applyConfig(void){
  (void)writeReg(TModeReg,0x8DU); (void)writeReg(TPrescalerReg,0x3EU); (void)writeReg(TReloadRegH,0x00U); (void)writeReg(TReloadRegL,30U); (void)writeReg(TxASKReg,0x40U); (void)writeReg(ModeReg,0x3DU);
#if RC522_RX_GAIN_MAX
  (void)writeReg(RFCfgReg,0x70U);
#endif
  (void)setBits(TxControlReg,0x03U);
}

void RC522_Init(void){
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(RFID_RST_GPIO_Port,RFID_RST_Pin,GPIO_PIN_SET);
  resetChip(); applyConfig();
  expectedVersion=readOrZero(VersionReg);
  memset(&diag,0,sizeof(diag));
}
void RC522_Reconfigure(void){ applyConfig(); }

uint8_t RC522_ReadVersion(void){return readOrZero(VersionReg);}

uint8_t RC522_ReadUID(uint8_t uid[4]){
  uint8_t req=PICC_REQA,response[5]={0U},len=sizeof(response);
  RC_Status s;
  diag.polls++;
  if(!uid||writeReg(BitFramingReg,0x07U)!=HAL_OK)return 0U;
  s=transceive(&req,1U,response,&len,7U);
  if(s==RC_TIMEOUT){diag.reqaTimeout++;return 0U;}
  if(s!=RC_OK||len!=2U){diag.reqaError++;return 0U;}
  diag.reqaOk++;
  if(writeReg(BitFramingReg,0U)!=HAL_OK)return 0U;
  uint8_t anti[2]={PICC_ANTICOLL,0x20U};len=sizeof(response);
  s=transceive(anti,2U,response,&len,0U);
  if(s!=RC_OK||len!=5U){diag.antiFail++;return 0U;}
  if((uint8_t)(response[0]^response[1]^response[2]^response[3])!=response[4]){diag.bccFail++;return 0U;}
  memcpy(uid,response,4U);(void)writeReg(CommandReg,PCD_IDLE);
  diag.ok++;
  return 1U;
}

void RC522_GetDiag(RC522_Diag_t *out){
  if(out) *out=diag;
  memset(&diag,0,sizeof(diag));
}

uint8_t RC522_ReadCanary(RC522_Canary_t *c){
  uint8_t ok=1U;
  if(!c) return 0U;
  ok &= (uint8_t)(readReg(VersionReg,&c->version)==HAL_OK);
  ok &= (uint8_t)(readReg(TxControlReg,&c->txControl)==HAL_OK);
  ok &= (uint8_t)(readReg(TModeReg,&c->tMode)==HAL_OK);
  ok &= (uint8_t)(readReg(TReloadRegL,&c->tReloadL)==HAL_OK);
  ok &= (uint8_t)(readReg(TxASKReg,&c->txAsk)==HAL_OK);
  ok &= (uint8_t)(readReg(ModeReg,&c->mode)==HAL_OK);
  c->spiOk=ok;
  return ok;
}

/* After a chip reset these revert to defaults (TxControl bits0/1 = 0, TMode = 0x00,
 * TReloadL = 0x00, TxASK = 0x00), so a mismatch means the RC522 lost its configuration. */
uint8_t RC522_CanaryOk(const RC522_Canary_t *c){
  if(!c||!c->spiOk) return 0U;
  return (c->version==expectedVersion && (c->txControl&0x03U)==0x03U &&
          c->tMode==0x8DU && c->tReloadL==30U && c->txAsk==0x40U) ? 1U : 0U;
}


//-----------------------07-11-26---------------------------------------------//



//#include "rc522.h"
//#include <string.h>
//
//extern SPI_HandleTypeDef hspi2;
//#define CommandReg       0x01U
//#define ComIrqReg        0x04U
//#define ErrorReg         0x06U
//#define FIFODataReg      0x09U
//#define FIFOLevelReg     0x0AU
//#define ControlReg       0x0CU
//#define BitFramingReg    0x0DU
//#define ModeReg          0x11U
//#define TxControlReg     0x14U
//#define TxASKReg         0x15U
//#define TModeReg         0x2AU
//#define TPrescalerReg    0x2BU
//#define TReloadRegH      0x2CU
//#define TReloadRegL      0x2DU
//#define VersionReg       0x37U
//#define PCD_IDLE         0x00U
//#define PCD_TRANSCEIVE   0x0CU
//#define PCD_SOFTRESET    0x0FU
//#define PICC_REQA        0x26U
//#define PICC_ANTICOLL    0x93U
//#define RC522_TIMEOUT_MS 25U
//
//typedef enum { RC_OK=0, RC_TIMEOUT, RC_ERROR, RC_NO_ROOM } RC_Status;
//static HAL_StatusTypeDef writeReg(uint8_t reg, uint8_t value) {
//  uint8_t tx[2]={(uint8_t)((reg<<1U)&0x7EU),value};
//  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_RESET);
//  HAL_StatusTypeDef st=HAL_SPI_Transmit(&hspi2,tx,2U,RC522_TIMEOUT_MS);
//  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_SET); return st;
//}
//static HAL_StatusTypeDef readReg(uint8_t reg,uint8_t *value) {
//  if(!value) return HAL_ERROR;
//  uint8_t addr=(uint8_t)(((reg<<1U)&0x7EU)|0x80U),dummy=0U,discard=0U;
//  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_RESET);
//  HAL_StatusTypeDef st=HAL_SPI_TransmitReceive(&hspi2,&addr,&discard,1U,RC522_TIMEOUT_MS);
//  if(st==HAL_OK) st=HAL_SPI_TransmitReceive(&hspi2,&dummy,value,1U,RC522_TIMEOUT_MS);
//  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_SET); return st;
//}
//static uint8_t readOrZero(uint8_t reg){uint8_t v=0U;(void)readReg(reg,&v);return v;}
//static HAL_StatusTypeDef setBits(uint8_t reg,uint8_t mask){uint8_t v;if(readReg(reg,&v)!=HAL_OK)return HAL_ERROR;return writeReg(reg,(uint8_t)(v|mask));}
//static HAL_StatusTypeDef clearBits(uint8_t reg,uint8_t mask){uint8_t v;if(readReg(reg,&v)!=HAL_OK)return HAL_ERROR;return writeReg(reg,(uint8_t)(v&(uint8_t)~mask));}
//static RC_Status transceive(const uint8_t *tx,uint8_t txLen,uint8_t *rx,uint8_t *rxLen,uint8_t lastBits){
//  uint8_t irq=0U,err=0U,count=0U,control=0U; uint32_t started;
//  if(!tx||!txLen||!rxLen||(*rxLen&&!rx))return RC_ERROR;
//  if(writeReg(CommandReg,PCD_IDLE)!=HAL_OK||writeReg(ComIrqReg,0x7FU)!=HAL_OK||writeReg(FIFOLevelReg,0x80U)!=HAL_OK||writeReg(BitFramingReg,lastBits&7U)!=HAL_OK)return RC_ERROR;
//  for(uint8_t i=0;i<txLen;i++)if(writeReg(FIFODataReg,tx[i])!=HAL_OK)return RC_ERROR;
//  if(writeReg(CommandReg,PCD_TRANSCEIVE)!=HAL_OK||setBits(BitFramingReg,0x80U)!=HAL_OK)return RC_ERROR;
//  started=HAL_GetTick();
//  do { if(readReg(ComIrqReg,&irq)!=HAL_OK){(void)clearBits(BitFramingReg,0x80U);return RC_ERROR;} if(irq&0x31U)break; } while((uint32_t)(HAL_GetTick()-started)<RC522_TIMEOUT_MS);
//  (void)clearBits(BitFramingReg,0x80U); if(!(irq&0x30U))return RC_TIMEOUT;
//  if(readReg(ErrorReg,&err)!=HAL_OK||(err&0x1BU))return RC_ERROR;
//  if (readReg(FIFOLevelReg, &count) != HAL_OK) {
//      return RC_ERROR;
//  }
//
//  if (count > *rxLen) {
//      return RC_NO_ROOM;
//  }
//  *rxLen=count; for(uint8_t i=0;i<count;i++)if(readReg(FIFODataReg,&rx[i])!=HAL_OK)return RC_ERROR;
//  (void)readReg(ControlReg,&control); return RC_OK;
//}
//static void resetChip(void){HAL_GPIO_WritePin(RFID_RST_GPIO_Port,RFID_RST_Pin,GPIO_PIN_RESET);HAL_Delay(5U);HAL_GPIO_WritePin(RFID_RST_GPIO_Port,RFID_RST_Pin,GPIO_PIN_SET);HAL_Delay(50U);(void)writeReg(CommandReg,PCD_SOFTRESET);HAL_Delay(50U);}
//void RC522_Init(void){
//  HAL_GPIO_WritePin(RFID_CS_GPIO_Port,RFID_CS_Pin,GPIO_PIN_SET);
//  HAL_GPIO_WritePin(RFID_RST_GPIO_Port,RFID_RST_Pin,GPIO_PIN_SET);
//  resetChip(); (void)writeReg(TModeReg,0x8DU); (void)writeReg(TPrescalerReg,0x3EU); (void)writeReg(TReloadRegH,0x00U); (void)writeReg(TReloadRegL,30U); (void)writeReg(TxASKReg,0x40U); (void)writeReg(ModeReg,0x3DU); (void)setBits(TxControlReg,0x03U);
//}
//uint8_t RC522_ReadVersion(void){return readOrZero(VersionReg);}
//uint8_t RC522_ReadUID(uint8_t uid[4]){
//  uint8_t req=PICC_REQA,response[5]={0U},len=sizeof(response);
//  if(!uid||writeReg(BitFramingReg,0x07U)!=HAL_OK)return 0U;
//  if(transceive(&req,1U,response,&len,7U)!=RC_OK||len!=2U)return 0U;
//  if(writeReg(BitFramingReg,0U)!=HAL_OK)return 0U;
//  uint8_t anti[2]={PICC_ANTICOLL,0x20U};len=sizeof(response);
//  if(transceive(anti,2U,response,&len,0U)!=RC_OK||len!=5U)return 0U;
//  if((uint8_t)(response[0]^response[1]^response[2]^response[3])!=response[4])return 0U;
//  memcpy(uid,response,4U);(void)writeReg(CommandReg,PCD_IDLE);return 1U;
//}
