//----------------------------07-11-26----------------------------//

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "line_follower.h"
#include "ultrasonic.h"
#include "rc522.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void USART1_ProcessRx(void);
static void USART1_SendLine(const char *line);
static void USART1_HandleCommand(char *line);
static void USART1_ReportEvents(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;

    /* Local STM32 debug console (USART2). */
    (void)HAL_UART_Transmit(&huart2, &c, 1U, 10U);

    /* Mirror STM32 debug text to ESP32 gateway (USART1). */
    (void)HAL_UART_Transmit(&huart1, &c, 1U, 10U);

    return ch;
}

#define USART1_RX_BUFFER_SIZE 96U
#define USART1_RX_QUEUE_SIZE 128U
static char usart1RxBuffer[USART1_RX_BUFFER_SIZE];
static uint8_t usart1RxIndex = 0U;
static uint8_t usart1RxByte = 0U;
static volatile uint8_t usart1RxQueue[USART1_RX_QUEUE_SIZE];
static volatile uint16_t usart1RxHead = 0U;
static volatile uint16_t usart1RxTail = 0U;
static volatile uint8_t usart1RxOverflow = 0U;
static char activeTurnJunction[8] = "J?";

/* Route graph: point index 0=START, 1..6=D1..D6.
 * Directions are taken from the user's verified origin-specific route sheets. */
typedef struct { const char *junction; const char *direction; } RouteStep;
#define MAX_ROUTE_STEPS 6U
#define POINT_COUNT 7U
#define STEP(j,d) { j, d }
#define EMPTY_STEP { NULL, NULL }
static const RouteStep routeTable[POINT_COUNT][POINT_COUNT][MAX_ROUTE_STEPS] = {
/* FROM START */
[0][1] = { STEP("J1","LEFT") },
[0][2] = { STEP("J1","STRAIGHT"), STEP("J6","LEFT"), STEP("J2","LEFT") },
[0][3] = { STEP("J1","STRAIGHT"), STEP("J6","LEFT"), STEP("J2","STRAIGHT"), STEP("J5","RIGHT"), STEP("J3","RIGHT") },
[0][4] = { STEP("J1","STRAIGHT"), STEP("J6","LEFT"), STEP("J2","STRAIGHT"), STEP("J5","RIGHT"), STEP("J3","STRAIGHT"), STEP("J4","LEFT") },
[0][5] = { STEP("J1","STRAIGHT"), STEP("J6","LEFT"), STEP("J2","STRAIGHT"), STEP("J5","STRAIGHT") },
[0][6] = { STEP("J1","STRAIGHT"), STEP("J6","STRAIGHT") },
/* FROM D1 */
[1][0] = { STEP("J1","RIGHT") },
[1][2] = { STEP("J1","LEFT"), STEP("J6","LEFT"), STEP("J2","LEFT") },
[1][3] = { STEP("J1","LEFT"), STEP("J6","LEFT"), STEP("J2","STRAIGHT"), STEP("J5","RIGHT"), STEP("J3","RIGHT") },
[1][4] = { STEP("J1","LEFT"), STEP("J6","LEFT"), STEP("J2","STRAIGHT"), STEP("J5","RIGHT"), STEP("J3","STRAIGHT"), STEP("J4","LEFT") },
[1][5] = { STEP("J1","LEFT"), STEP("J6","LEFT"), STEP("J2","STRAIGHT"), STEP("J5","STRAIGHT") },
[1][6] = { STEP("J1","LEFT"), STEP("J6","STRAIGHT") },
/* FROM D2 */
[2][0] = { STEP("J2","RIGHT"), STEP("J6","RIGHT"), STEP("J1","STRAIGHT") },
[2][1] = { STEP("J2","RIGHT"), STEP("J6","RIGHT"), STEP("J1","RIGHT") },
[2][3] = { STEP("J2","LEFT"), STEP("J5","RIGHT"), STEP("J3","RIGHT") },
[2][4] = { STEP("J2","LEFT"), STEP("J5","RIGHT"), STEP("J3","STRAIGHT"), STEP("J4","LEFT") },
[2][5] = { STEP("J2","LEFT"), STEP("J5","STRAIGHT") },
[2][6] = { STEP("J2","RIGHT"), STEP("J6","LEFT") },
/* FROM D3 */
[3][0] = { STEP("J3","LEFT"), STEP("J5","LEFT"), STEP("J2","STRAIGHT"), STEP("J6","RIGHT"), STEP("J1","STRAIGHT") },
[3][1] = { STEP("J3","LEFT"), STEP("J5","LEFT"), STEP("J2","STRAIGHT"), STEP("J6","RIGHT"), STEP("J1","RIGHT") },
[3][2] = { STEP("J3","LEFT"), STEP("J5","LEFT"), STEP("J2","RIGHT") },
[3][4] = { STEP("J3","RIGHT"), STEP("J4","LEFT") },
[3][5] = { STEP("J3","LEFT"), STEP("J5","RIGHT") },
[3][6] = { STEP("J3","LEFT"), STEP("J5","LEFT"), STEP("J2","STRAIGHT"), STEP("J6","LEFT") },
/* FROM D4 */
[4][0] = { STEP("J4","RIGHT"), STEP("J3","STRAIGHT"), STEP("J5","LEFT"), STEP("J2","STRAIGHT"), STEP("J6","RIGHT"), STEP("J1","STRAIGHT") },
[4][1] = { STEP("J4","RIGHT"), STEP("J3","STRAIGHT"), STEP("J5","LEFT"), STEP("J2","STRAIGHT"), STEP("J6","RIGHT"), STEP("J1","RIGHT") },
[4][2] = { STEP("J4","RIGHT"), STEP("J3","STRAIGHT"), STEP("J5","LEFT"), STEP("J2","RIGHT") },
[4][3] = { STEP("J4","RIGHT"), STEP("J3","LEFT") },
[4][5] = { STEP("J4","RIGHT"), STEP("J3","STRAIGHT"), STEP("J5","RIGHT") },
[4][6] = { STEP("J4","RIGHT"), STEP("J3","STRAIGHT"), STEP("J5","LEFT"), STEP("J2","STRAIGHT"), STEP("J6","LEFT") },
/* FROM D5 */
[5][0] = { STEP("J5","STRAIGHT"), STEP("J2","STRAIGHT"), STEP("J6","RIGHT"), STEP("J1","STRAIGHT") },
[5][1] = { STEP("J5","STRAIGHT"), STEP("J2","STRAIGHT"), STEP("J6","RIGHT"), STEP("J1","RIGHT") },
[5][2] = { STEP("J5","STRAIGHT"), STEP("J2","RIGHT") },
[5][3] = { STEP("J5","LEFT"), STEP("J3","RIGHT") },
[5][4] = { STEP("J5","LEFT"), STEP("J3","STRAIGHT"), STEP("J4","LEFT") },
[5][6] = { STEP("J5","STRAIGHT"), STEP("J2","STRAIGHT"), STEP("J6","LEFT") },
/* FROM D6 */
[6][0] = { STEP("J6","STRAIGHT"), STEP("J1","STRAIGHT") },
[6][1] = { STEP("J6","STRAIGHT"), STEP("J1","RIGHT") },
[6][2] = { STEP("J6","RIGHT"), STEP("J2","LEFT") },
[6][3] = { STEP("J6","RIGHT"), STEP("J2","STRAIGHT"), STEP("J5","RIGHT"), STEP("J3","RIGHT") },
[6][4] = { STEP("J6","RIGHT"), STEP("J2","STRAIGHT"), STEP("J5","RIGHT"), STEP("J3","STRAIGHT"), STEP("J4","LEFT") },
[6][5] = { STEP("J6","RIGHT"), STEP("J2","STRAIGHT"), STEP("J5","STRAIGHT") }
};
static const uint8_t routeLengths[POINT_COUNT][POINT_COUNT] = {
 {0,1,3,5,6,4,2},
 {1,0,3,5,6,4,2},
 {3,3,0,3,4,2,2},
 {5,5,3,0,2,2,4},
 {6,6,4,2,0,3,5},
 {4,4,2,2,3,0,3},
 {2,2,2,4,5,3,0}
};
static int8_t selectedDestination = -1;
static int8_t currentLocation = -1; /* 0=START, 1..6=D1..D6, -1=unknown */
static int8_t routeOrigin = -1;
static uint8_t routeIndex = 0U;
static uint8_t routeActive = 0U;
static uint8_t turnInProgress = 0U;
static uint8_t uTurnPending = 0U;
/* Arrival latch filters repeated events; RC522 polling never stops. */
static uint8_t destinationReachedLock = 0U;
static uint8_t rfidReady = 0U;
static uint8_t lastSeenUID[4] = {0U};
static uint8_t haveLastUID = 0U;
/* RFID anti-repeat: tolerate brief RC522 read dropouts without re-triggering. */
#define RFID_RELEASE_DEBOUNCE_MS  300U
#define RFID_DUPLICATE_GUARD_MS   1200U
static uint32_t lastSuccessfulUIDReadMs = 0U;
static uint32_t lastProcessedUIDMs = 0U;
static uint8_t haveProcessedUID = 0U;
static uint8_t lastProcessedUID[4] = {0U};

static const struct { const char *name; uint8_t uid[4]; } tagTable[] = {
    {"J1", {0x5A,0xA1,0xCB,0xF2}}, {"J2", {0x48,0xCF,0x69,0xFC}},
    {"J3", {0x2A,0xEA,0x45,0xF2}}, {"J4", {0x59,0xB7,0xEC,0xDA}},
    {"J5", {0x0A,0x6E,0xA0,0xF2}}, {"J6", {0xAD,0xD3,0x0F,0x05}},
    {"START", {0x44,0x30,0x19,0x07}},
    {"D1", {0x56,0x3E,0xC4,0xD3}}, {"D2", {0x49,0x4D,0x1D,0xDA}},
    {"D3", {0x1A,0x7B,0x49,0xF2}}, {"D4", {0x43,0x09,0x27,0x38}},
    {"D5", {0x0A,0xCF,0x77,0xF2}}, {"D6", {0xFA,0x44,0x1D,0xF1}}
};
#define TAG_COUNT ((uint8_t)(sizeof(tagTable)/sizeof(tagTable[0])))

static void USART1_SendLine(const char *line)
{
    if (line == NULL) return;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)strlen(line), 100U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2U, 100U);

    char debugLine[128];
    int n = snprintf(debugLine, sizeof(debugLine), "[USART1 TX] %s\r\n", line);
    if (n > 0) {
        size_t count = ((size_t)n < sizeof(debugLine)) ? (size_t)n : sizeof(debugLine) - 1U;
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)debugLine, (uint16_t)count, 50U);
    }
}

static void ResetRouteState(void)
{
    selectedDestination = -1; routeOrigin = -1; routeIndex = 0U;
    routeActive = 0U; turnInProgress = 0U; uTurnPending = 0U;
}

static int8_t PointIndexFromName(const char *name)
{
    if (name == NULL) return -1;
    if (strcmp(name, "START") == 0) return 0;
    if (name[0] == 'D' && name[1] >= '1' && name[1] <= '6' && name[2] == '\0') return (int8_t)(name[1] - '0');
    return -1;
}

static const char *PointName(int8_t point)
{
    if (point == 0) return "START";
    if (point >= 1 && point <= 6) { static const char *names[] = {"D1","D2","D3","D4","D5","D6"}; return names[point-1]; }
    return "UNKNOWN";
}

static void USART1_HandleCommand(char *line)
{
    char destination[8] = {0};
    if (strcmp(line, "STOP") == 0) { ResetRouteState(); LineFollower_LatchStop(); USART1_SendLine("ACK|STOP"); return; }
    if (strcmp(line, "RESET") == 0) {
        ResetRouteState();
        destinationReachedLock = 0U;
        haveLastUID = 0U;
        haveProcessedUID = 0U;
        uTurnPending = 0U;
        LineFollower_ResetNavigationFault();
        USART1_SendLine("ACK|RESET");
        printf("[RFID] Destination lock cleared by RESET.\r\n"); return;
    }
    const char *d = NULL;
    if (strcmp(line, "START") == 0 || (sscanf(line, "DEST|%7s", destination) == 1 && strcmp(destination, "START") == 0)) d = "START";
    else if ((strlen(line) == 2U && line[0] == 'D' && line[1] >= '1' && line[1] <= '6') ||
             (sscanf(line, "DEST|%7s", destination) == 1 && strlen(destination) == 2U && destination[0] == 'D' && destination[1] >= '1' && destination[1] <= '6'))
        d = (line[0] == 'D' && line[1] >= '1' && line[1] <= '6' && strlen(line) == 2U) ? line : destination;
    if (d != NULL) {
        int8_t target = PointIndexFromName(d);
        if (target < 0) { USART1_SendLine("ERR|DEST|INVALID"); return; }
        if (currentLocation < 0) {
            USART1_SendLine("ERR|LOCATION|UNKNOWN");
            printf("[LOCATION] Unknown current point. Read START or D1-D6 tag while idle before selecting a route.\r\n"); return;
        }
        if (currentLocation == target) {
            char message[64]; snprintf(message, sizeof(message), "ERR|DEST|ALREADY_THERE|%s", PointName(target));
            USART1_SendLine(message); printf("[DESTINATION] Already at %s; request rejected.\r\n", PointName(target)); return;
        }
        if (LineFollower_SetDestination()) {
            destinationReachedLock = 0U; haveLastUID = 0U; haveProcessedUID = 0U;
            routeOrigin = currentLocation; selectedDestination = target; routeIndex = 0U; routeActive = 1U; turnInProgress = 0U;
            uTurnPending = (routeOrigin != 0) ? 1U : 0U;
            char response[40]; snprintf(response, sizeof(response), "ACK|ROUTE|%s|%s", PointName(routeOrigin), PointName(target));
            USART1_SendLine(response);
            printf("[LOCATION] Current point: %s. Requested destination: %s.\r\n", PointName(routeOrigin), PointName(target));
            if (uTurnPending) printf("[NAV] A 180-degree alignment turn is required before the return route.\r\n");
            else printf("[ROUTE] Starting from START; route selected.\r\n");
        } else USART1_SendLine("ERR|DEST|LOCKED");
        return;
    }
    USART1_SendLine("ERR|UNKNOWN_COMMAND");
}

static void USART1_ProcessRx(void)
{
    while (usart1RxTail != usart1RxHead) {
        uint8_t ch = usart1RxQueue[usart1RxTail];
        usart1RxTail = (uint16_t)((usart1RxTail + 1U) % USART1_RX_QUEUE_SIZE);

        if (ch == '\r' || ch == '\n') {
            if (usart1RxIndex > 0U) {
                usart1RxBuffer[usart1RxIndex] = '\0';
                printf("[USART1 RX] %s\r\n", usart1RxBuffer);
                USART1_HandleCommand(usart1RxBuffer);
                usart1RxIndex = 0U;
            }
        } else if (ch >= 32U && ch <= 126U) {
            if (usart1RxIndex < USART1_RX_BUFFER_SIZE - 1U) {
                usart1RxBuffer[usart1RxIndex++] = (char)ch;
            } else {
                usart1RxIndex = 0U;
                printf("[USART1 ERROR] RX command too long; line discarded.\r\n");
                USART1_SendLine("ERR|RX_LINE_TOO_LONG");
            }
        }
    }

    if (usart1RxOverflow != 0U) {
        usart1RxOverflow = 0U;
        printf("[USART1 ERROR] RX queue overflow; some input bytes may have been lost.\r\n");
    }
}

static void HandleRFIDTag(const uint8_t uid[4])
{
    int found = -1;
    for (uint8_t i = 0U; i < TAG_COUNT; ++i) if (memcmp(uid, tagTable[i].uid, 4U) == 0) { found = (int)i; break; }
    if (found < 0) {
        printf("[RFID] Unknown UID %02X %02X %02X %02X\r\n", uid[0],uid[1],uid[2],uid[3]);
        if (routeActive) { ResetRouteState(); LineFollower_LatchStop(); USART1_SendLine("ERR|RFID|UNKNOWN"); } return;
    }
    const char *name = tagTable[found].name;
    printf("[RFID] Recognized %s\r\n", name);
    int8_t point = PointIndexFromName(name);
    /*
     * Keep polling the RC522 continuously, including after arrival. The
     * destination lock filters events; it never disables the RFID reader.
     */
    if (!routeActive || selectedDestination < 0) {
        if (destinationReachedLock) {
            /* Keep last confirmed arrival as currentLocation. A new destination
               command may depart from it, including START (point index 0). */
            return;
        }

        if (point >= 0) {
            currentLocation = point;
            printf("[LOCATION] Vehicle location confirmed: %s.\r\n", PointName(currentLocation));
            char msg[40];
            snprintf(msg, sizeof(msg), "LOCATION|%s", PointName(currentLocation));
            USART1_SendLine(msg);
        } else {
            printf("[RFID] No active route; junction tag logged only.\r\n");
        }
        return;
    }

    if (turnInProgress || uTurnPending) {
        printf("[RFID] Ignored while maneuver is in progress.\r\n");
        return;
    }

    /*
     * The source location tag is physically underneath the vehicle when the
     * route is selected. Ignore it throughout this route so it cannot be
     * mistaken for an unexpected/early destination.
     */
    if (point >= 0 && point == routeOrigin) {
        printf("[RFID] Ignored source-location tag %s while travelling to %s.\r\n",
               name, PointName(selectedDestination));
        return;
    }
    if (name[0] == 'J') {
        if (routeIndex >= routeLengths[(uint8_t)routeOrigin][(uint8_t)selectedDestination] ||
            routeTable[(uint8_t)routeOrigin][(uint8_t)selectedDestination][routeIndex].junction == NULL ||
            strcmp(name, routeTable[(uint8_t)routeOrigin][(uint8_t)selectedDestination][routeIndex].junction) != 0) {
            const char *expected = (routeIndex < routeLengths[(uint8_t)routeOrigin][(uint8_t)selectedDestination]) ? routeTable[(uint8_t)routeOrigin][(uint8_t)selectedDestination][routeIndex].junction : "no junction";
            printf("[NAV FAULT] Expected %s but detected %s.\r\n", expected, name);
            ResetRouteState(); LineFollower_LatchStop(); USART1_SendLine("ERR|RFID|WRONG_JUNCTION"); return;
        }
        const RouteStep *step = &routeTable[(uint8_t)routeOrigin][(uint8_t)selectedDestination][routeIndex];
        strncpy(activeTurnJunction, step->junction, sizeof(activeTurnJunction)-1U); activeTurnJunction[sizeof(activeTurnJunction)-1U] = '\0';
        if (LineFollower_RequestTurn(step->direction)) {
            turnInProgress = 1U; char response[64]; snprintf(response, sizeof(response), "NAV|%s|%s|%s", step->junction, step->direction, PointName(selectedDestination)); USART1_SendLine(response);
        } else { ResetRouteState(); LineFollower_LatchStop(); USART1_SendLine("ERR|NAV|LOCKED_OR_BUSY"); } return;
    }
    if (point >= 0) {
        if (point != selectedDestination || routeIndex != routeLengths[(uint8_t)routeOrigin][(uint8_t)selectedDestination]) {
            printf("[NAV FAULT] Wrong/early point %s.\r\n", name);
            ResetRouteState();
            LineFollower_LatchStop();
            USART1_SendLine("ERR|RFID|WRONG_OR_EARLY_DEST"); return;
        }
        LineFollower_Arrive();
        currentLocation = point;
        destinationReachedLock = 1U;
        ResetRouteState(); haveLastUID = 0U; uTurnPending = 0U;
        printf("[LOCATION] ARRIVED at %s.\r\n", PointName(currentLocation));
        printf("[RFID] Destination lock active until RESET or a new destination command.\r\n");
        char response[40]; snprintf(response, sizeof(response), "ARRIVE|%s", PointName(point)); USART1_SendLine(response);
    }
}

static void RFID_Process(void)
{
    uint8_t uid[4];
    uint32_t now = HAL_GetTick();
    static uint32_t lastDiagMs = 0U;

    /* Report reader-stage counters every 2 seconds for firmware debugging. */
    if ((uint32_t)(now - lastDiagMs) >= 2000U) {
        RC522_Diag_t d;
        RC522_GetDiag(&d);
//        printf("[RFID DIAG] polls=%lu uidOK=%lu REQA_OK=%lu REQA_timeout=%lu "
//               "REQA_error=%lu anticollision_fail=%lu BCC_fail=%lu SPI_fail=%lu lastErr=0x%02X\r\n",
//               (unsigned long)d.polls,
//               (unsigned long)d.ok,
//               (unsigned long)d.reqaOk,
//               (unsigned long)d.reqaTimeout,
//               (unsigned long)d.reqaError,
//               (unsigned long)d.antiFail,
//               (unsigned long)d.bccFail,
//               (unsigned long)d.spiFail,
//               d.lastErr);
        lastDiagMs = now;
    }

    /* Retry reader startup if a transient SPI/version read failed. */
    if (!rfidReady) {
        static uint32_t lastRetryMs = 0U;
        if ((uint32_t)(now - lastRetryMs) >= 2000U) {
            lastRetryMs = now;
            RC522_Init();
            uint8_t v = RC522_ReadVersion();
            rfidReady = (v != 0x00U && v != 0xFFU);
            printf("[RFID] Reader retry: version=0x%02X ready=%u\r\n", v, rfidReady);
        }
        if (!rfidReady) return;
    }

    if (!RC522_ReadUID(uid)) {
        /* Do not re-arm on a single missed read: RC522 reads can briefly drop
           while a card is still within range. Require a sustained absence. */
        if (haveLastUID &&
            (uint32_t)(now - lastSuccessfulUIDReadMs) >= RFID_RELEASE_DEBOUNCE_MS) {
            haveLastUID = 0U;
        }
        return;
    }

    lastSuccessfulUIDReadMs = now;

    /* Same card remains present: never dispatch the same event repeatedly. */
    if (haveLastUID && memcmp(uid, lastSeenUID, 4U) == 0) return;

    /* Also suppress rapid re-presentation/re-read of the same UID, even if
       the reader briefly reported no card between consecutive reads. */
    if (haveProcessedUID && memcmp(uid, lastProcessedUID, 4U) == 0 &&
        (uint32_t)(now - lastProcessedUIDMs) < RFID_DUPLICATE_GUARD_MS) {
        memcpy(lastSeenUID, uid, 4U);
        haveLastUID = 1U;
        return;
    }

    memcpy(lastSeenUID, uid, 4U);
    haveLastUID = 1U;
    memcpy(lastProcessedUID, uid, 4U);
    lastProcessedUIDMs = now;
    haveProcessedUID = 1U;
    HandleRFIDTag(uid);
}

static void USART1_ReportEvents(void)
{
    if (uTurnPending && routeActive && !turnInProgress) {
        if (LineFollower_RequestUTurn()) { turnInProgress = 1U; strncpy(activeTurnJunction, "UTURN", sizeof(activeTurnJunction)-1U); USART1_SendLine("NAV|UTURN|180|ALIGN"); }
        else { ResetRouteState(); LineFollower_LatchStop(); USART1_SendLine("ERR|NAV|UTURN_FAILED"); }
    }
    if (LineFollower_TakeUTurnDoneEvent()) {
        uTurnPending = 0U; turnInProgress = 0U;
        USART1_SendLine("DONE|UTURN|180");
        printf("[NAV] 180-degree turn complete; following route from %s to %s.\r\n", PointName(routeOrigin), PointName(selectedDestination));
    }
    if (LineFollower_TakeTurnDoneEvent()) {
        turnInProgress = 0U; routeIndex++;
        char response[48]; snprintf(response, sizeof(response), "DONE|%s|%u/%u", activeTurnJunction, routeIndex, (routeOrigin >= 0 && selectedDestination >= 0) ? routeLengths[(uint8_t)routeOrigin][(uint8_t)selectedDestination] : 0U); USART1_SendLine(response);
        printf("[ROUTE] %s -> %s progress %u/%u\r\n", PointName(routeOrigin), PointName(selectedDestination), routeIndex, (routeOrigin >= 0 && selectedDestination >= 0) ? routeLengths[(uint8_t)routeOrigin][(uint8_t)selectedDestination] : 0U);
    }
    if (LineFollower_TakeTurnFaultEvent()) {
        ResetRouteState(); USART1_SendLine("ERR|TURN_TIMEOUT");
    }
}

/* USART1 RX interrupt callback: queue bytes; parse commands in the main loop. */
static void USART1_StartRx(void)
{
    usart1RxHead = 0U;
    usart1RxTail = 0U;
    usart1RxIndex = 0U;
    usart1RxOverflow = 0U;

    if (HAL_UART_Receive_IT(&huart1, &usart1RxByte, 1U) != HAL_OK) {
        printf("[USART1 ERROR] Failed to start RX interrupt.\r\n");
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t next =
            (uint16_t)((usart1RxHead + 1U) % USART1_RX_QUEUE_SIZE);

        if (next != usart1RxTail)
        {
            usart1RxQueue[usart1RxHead] = usart1RxByte;
            usart1RxHead = next;
        }
        else
        {
            usart1RxOverflow = 1U;
        }

        HAL_StatusTypeDef status =
            HAL_UART_Receive_IT(&huart1, &usart1RxByte, 1U);

        if (status != HAL_OK)
        {
            /* Record the failure for reporting from the main loop. */
            usart1RxOverflow = 1U;
        }
    }
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1) {
//        uint16_t next = (uint16_t)((usart1RxHead + 1U) % USART1_RX_QUEUE_SIZE);
//        if (next != usart1RxTail) {
//            usart1RxQueue[usart1RxHead] = usart1RxByte;
//            usart1RxHead = next;
//        } else {
//            usart1RxOverflow = 1U;
//        }
//        (void)HAL_UART_Receive_IT(&huart1, &usart1RxByte, 1U);
//    }
//}

static volatile uint32_t usart1LastError = HAL_UART_ERROR_NONE;
static volatile uint8_t usart1ErrorEvent = 0U;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        usart1LastError = HAL_UART_GetError(huart);
        usart1ErrorEvent = 1U;

        (void)HAL_UART_AbortReceive(&huart1);
        (void)HAL_UART_Receive_IT(
            &huart1,
            &usart1RxByte,
            1U
        );
    }
}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART1) {
//        (void)HAL_UART_AbortReceive(&huart1);
//        (void)HAL_UART_Receive_IT(&huart1, &usart1RxByte, 1U);
//    }
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Start USART1 reception from the ESP32. */

    USART1_StartRx();

  /* Start both motor PWM channels (TIM3 ARR = 99). */
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

  Ultrasonic_Init();
  RC522_Init();
  uint8_t rc522Version = RC522_ReadVersion();
  printf("[RFID] RC522 version: 0x%02X\r\n", rc522Version);
  rfidReady = (rc522Version != 0x00U && rc522Version != 0xFFU);
  if (!rfidReady) { printf("[RFID ERROR] RC522 not responding at startup; automatic retry enabled.\r\n"); }
  else printf("[RFID] Reader ready; STM32 owns RFID and route logic.\r\n");

  printf("\r\n");
  printf("========================================\r\n");
  printf(" STM32H563 V2 ROBOT\r\n");
  printf(" RFID + IR + ULTRASONIC + MOTOR CONTROL\r\n");
  printf("========================================\r\n");
  printf("IR: PF11 PF12 PF13\r\n");
  printf("ULTRA TRIG: PC12\r\n");
  printf("ULTRA ECHO: PE5\r\n");
  printf("USART2: debug UART 115200\r\n");
  printf("USART1: ESP32 link 115200 8N1 (verify CubeMX AF pins)\r\n");
  printf("Startup state: navigation locked until RESET + DEST\r\n");
  printf("[LOCATION] Current point is UNKNOWN until START or D1-D6 tag is read while idle.\r\n");
  printf("[ROUTES] Point-to-point routing enabled; START UID = 44 30 19 07.\r\n");
  printf("========================================\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  RFID_Process();

	  if (usart1ErrorEvent != 0U)
	  {
	      uint32_t error = usart1LastError;
	      usart1ErrorEvent = 0U;

	      printf(
	          "[USART1 ERROR] HAL error code = 0x%08lX\r\n",
	          (unsigned long)error
	      );
	  }
	  USART1_ProcessRx();

	      LineFollower_Run();
	      USART1_ReportEvents();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_1);
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M_IN3_Pin|M_IN4_Pin|US_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M_IN1_Pin|M_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port, RFID_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFID_RST_GPIO_Port, RFID_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : US_ECHO_Pin */
  GPIO_InitStruct.Pin = US_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(US_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M_IN3_Pin M_IN4_Pin US_TRIG_Pin */
  GPIO_InitStruct.Pin = M_IN3_Pin|M_IN4_Pin|US_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M_IN1_Pin M_IN2_Pin RFID_CS_Pin */
  GPIO_InitStruct.Pin = M_IN1_Pin|M_IN2_Pin|RFID_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_IR_Pin Center_IR_Pin Right_IR_Pin */
  GPIO_InitStruct.Pin = Left_IR_Pin|Center_IR_Pin|Right_IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : RFID_RST_Pin */
  GPIO_InitStruct.Pin = RFID_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFID_RST_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
