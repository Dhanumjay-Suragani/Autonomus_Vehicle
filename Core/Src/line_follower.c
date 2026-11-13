/*
 * line_follower.c
 *
 *  Created on: Jun 16, 2026
 *      Author: dhanu
 */

/*
 * line_follower.c
 * STM32H563ZIT6: 3-IR line following + stop-and-wait obstacle protection
 *
 * IR: PF11=Left, PF12=Center, PF13=Right
 * Motor directions: Left PB0/PB1, Right PC0/PC1
 * PWM: TIM3_CH1=Left (PA6), TIM3_CH2=Right (PA7), ARR=99
 * Ultrasonic: TRIG=PC12, ECHO=PE5
 * Debug: printf -> USART2 (PD5 TX, 115200 baud)
 *
 * Behavior: when an obstacle is detected within the threshold, both motors
 * stop and remain stopped. No reverse, turning, bypassing, or line-search
 * movement is performed. Resume line following only after multiple valid
 * ultrasonic readings confirm that the obstacle has cleared.
 */

/*
 * STM32H563ZIT6 vehicle V2 controller.
 * Preserves original 3-IR patterns, PWM channels/speeds, ultrasonic threshold,
 * and the 3-valid-clear-reading obstacle latch. Adds USART1 route/turn control.
 */

#include "line_follower.h"
#include "ultrasonic.h"
#include <stdio.h>
#include <string.h>/* Added U-turn support for point-to-point navigation; existing sensor, motor,\n * line-following and ultrasonic safety behavior is otherwise preserved. */

extern TIM_HandleTypeDef htim3;

//#define BASE_SPEED                 70U
//#define TURN_SPEED                 85U
//#define SHARP_SPEED                95U
//#define MAX_PWM                    99U
//#define STOP_SPEED                 0U
//#define OBSTACLE_DISTANCE_CM       20.0f
//#define ULTRASONIC_PERIOD_MS       100U
//#define CLEAR_READINGS_REQUIRED    3U

/* ================= FEATURE CONFIGURATION ================= */

/* 0 = Disable ultrasonic obstacle protection temporarily.
 * 1 = Enable ultrasonic obstacle protection again.
 */
#define ENABLE_ULTRASONIC_OBSTACLE_PROTECTION    0U

/* ================= MOTOR CONFIGURATION =================== */

#define BASE_SPEED                 50U
#define TURN_SPEED                 75U
#define SHARP_SPEED                80U
#define MAX_PWM                    99U
#define STOP_SPEED                 0U

/* Retained for future ultrasonic reactivation. */
#define OBSTACLE_DISTANCE_CM       30.0f
#define ULTRASONIC_PERIOD_MS       100U
#define CLEAR_READINGS_REQUIRED    3U

#define TURN_MIN_MS                750U  /* Commit to the requested turn for at least 1.5 s. */
#define TURN_TIMEOUT_MS            1000U  /* Allow time to complete the committed turn and reacquire line. */
#define LINE_CONFIRM_READINGS      3U
#define UTURN_MIN_MS               1700U  /* Initial calibration value; tune on the actual vehicle. */
#define UTURN_TIMEOUT_MS           2000U

typedef enum { LAST_NONE = 0, LAST_LEFT, LAST_RIGHT } LastDirection_t;
typedef enum { TURN_NONE = 0, TURN_STRAIGHT, TURN_LEFT, TURN_RIGHT, TURN_AROUND } TurnMode_t;

static LastDirection_t lastDirection = LAST_NONE;
static uint8_t obstacleLatched = 0U;
static uint8_t navigationFaultLatched = 1U; /* fail-safe until ESP32 RESET */
static uint8_t routeEnabled = 0U;
static uint8_t arrived = 0U;
static uint8_t turnDoneEvent = 0U;
static uint8_t uTurnDoneEvent = 0U;
static uint8_t turnFaultEvent = 0U;
static uint8_t turnLineCount = 0U;
static TurnMode_t turnMode = TURN_NONE;
static uint32_t turnStartMs = 0U;
static uint32_t lastUltrasonicMs = 0U;
static uint32_t lastDebugMs = 0U;
static uint8_t clearReadings = 0U;
static float lastDistanceCm = -1.0f;

/* Status telemetry addition only: sensor interpretation and motor actions stay unchanged. */
static int8_t lastReportedLineStatus = -1;

static void ReportLineStatus(uint8_t state, const char *message)
{
    if (lastReportedLineStatus != (int8_t)state) {
        printf("[LINE STATUS] %s (sensor state=%u)\r\n", message, state);
        lastReportedLineStatus = (int8_t)state;
    }
}

static void SetLeftMotor(uint16_t pwm) {
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
}

static void SetRightMotor(uint16_t pwm) {
    if (pwm > MAX_PWM) pwm = MAX_PWM;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
}

static void StopMotors(void) {
    SetLeftMotor(0U); SetRightMotor(0U);
    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin | M_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin | M_IN4_Pin, GPIO_PIN_RESET);
}

static void MoveForward(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
    SetLeftMotor(speed); SetRightMotor(speed);
}

/* Pivot turns retain the original left/right motor direction convention. */
static void TurnLeft(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
    SetLeftMotor(speed); SetRightMotor(speed);
}

static void TurnRight(uint16_t speed) {
    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_SET);
    SetLeftMotor(speed); SetRightMotor(speed);
}

/* Keep the source's existing active-low sensor inversion and bit order. */
static uint8_t ReadSensors(void) {
    uint32_t idr = GPIOF->IDR;
    uint8_t L = (uint8_t)((idr >> 11) & 1U);
    uint8_t C = (uint8_t)((idr >> 12) & 1U);
    uint8_t R = (uint8_t)((idr >> 13) & 1U);
    return (uint8_t)(~((L << 2) | (C << 1) | R) & 0x07U);
}

uint8_t GetSensorState(void) { return ReadSensors(); }

static void CheckObstacle(void) {
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - lastUltrasonicMs) < ULTRASONIC_PERIOD_MS) {
        if (obstacleLatched) StopMotors();
        return;
    }
    lastUltrasonicMs = now;
    lastDistanceCm = Ultrasonic_ReadDistanceCm();
    if (lastDistanceCm < 0.0f) {
        if (obstacleLatched) { clearReadings = 0U; StopMotors(); }
        if ((uint32_t)(now - lastDebugMs) >= 500U) {
            printf("[ULTRA] ECHO timeout/no return\r\n"); lastDebugMs = now;
        }
        return; /* Timeout is never a clear-path confirmation. */
    }
    uint32_t d10 = (uint32_t)(lastDistanceCm * 10.0f + 0.5f);
    if ((uint32_t)(now - lastDebugMs) >= 250U) {
        printf("[ULTRA] %lu.%lu cm\r\n", (unsigned long)(d10 / 10U), (unsigned long)(d10 % 10U));
        lastDebugMs = now;
    }
    if (lastDistanceCm <= OBSTACLE_DISTANCE_CM) {
        clearReadings = 0U;
        if (!obstacleLatched) {
            obstacleLatched = 1U;
            printf("[OBSTACLE] Detected at %lu.%lu cm. STOPPED.\r\n", (unsigned long)(d10 / 10U), (unsigned long)(d10 % 10U));
        }
        StopMotors();
        return;
    }
    if (obstacleLatched) {
        if (clearReadings < CLEAR_READINGS_REQUIRED) clearReadings++;
        StopMotors();
        if (clearReadings >= CLEAR_READINGS_REQUIRED) {
            obstacleLatched = 0U; clearReadings = 0U;
            printf("[OBSTACLE] Clear confirmed after 3 valid readings.\r\n");
        } else {
            printf("[OBSTACLE] Waiting for clear path (%u/%u)\r\n", clearReadings, CLEAR_READINGS_REQUIRED);
        }
    }
}

uint8_t LineFollower_IsObstacleLatched(void) { return obstacleLatched; }

uint8_t LineFollower_IsNavigationFaultLatched(void) { return navigationFaultLatched; }

void LineFollower_ResetNavigationFault(void) {
    /* Deliberately does not clear obstacleLatched. */
    navigationFaultLatched = 0U;
    routeEnabled = 0U;
    arrived = 0U;
    turnMode = TURN_NONE;
    turnDoneEvent = 0U;
    uTurnDoneEvent = 0U;
    turnFaultEvent = 0U;
    StopMotors();
    printf("[NAV] Navigation fault cleared; waiting for DEST command.\r\n");
}

void LineFollower_LatchStop(void) {
    navigationFaultLatched = 1U;
    routeEnabled = 0U;
    arrived = 0U;
    turnMode = TURN_NONE;
    turnDoneEvent = 0U;
    uTurnDoneEvent = 0U;
    turnFaultEvent = 0U;
    StopMotors();
    printf("[NAV] STOP received; navigation fault latched.\r\n");
}

uint8_t LineFollower_SetDestination(void) {
    if (navigationFaultLatched || obstacleLatched) return 0U;
    arrived = 0U; routeEnabled = 1U; turnMode = TURN_NONE;
    turnDoneEvent = 0U; uTurnDoneEvent = 0U; turnFaultEvent = 0U;
    StopMotors();
    printf("[NAV] Destination accepted; line following enabled.\r\n");
    return 1U;
}

void LineFollower_Arrive(void) {
    routeEnabled = 0U; arrived = 1U; turnMode = TURN_NONE;
    StopMotors();
    printf("[NAV] Destination reached; motors stopped.\r\n");
}

uint8_t LineFollower_RequestTurn(const char *direction) {
    if (!direction || navigationFaultLatched || !routeEnabled || arrived || obstacleLatched || turnMode != TURN_NONE) return 0U;
    if (strcmp(direction, "LEFT") == 0) turnMode = TURN_LEFT;
    else if (strcmp(direction, "RIGHT") == 0) turnMode = TURN_RIGHT;
    else if (strcmp(direction, "STRAIGHT") == 0) turnMode = TURN_STRAIGHT;
    else return 0U;
    turnStartMs = HAL_GetTick(); turnLineCount = 0U;
    turnDoneEvent = 0U; uTurnDoneEvent = 0U; turnFaultEvent = 0U;
    printf("[NAV] Turn started: %s\r\n", direction);
    return 1U;
}

/* Request a 180-degree pivot after a destination has been reached.
 * This is accepted only when route control is enabled and safety locks are clear.
 * The caller should enable the new route before requesting the U-turn. */
uint8_t LineFollower_RequestUTurn(void) {
    if (navigationFaultLatched || !routeEnabled || arrived || obstacleLatched ||
        turnMode != TURN_NONE) return 0U;
    turnMode = TURN_AROUND;
    turnStartMs = HAL_GetTick();
    turnLineCount = 0U;
    turnDoneEvent = 0U;
    uTurnDoneEvent = 0U;
    turnFaultEvent = 0U;
    printf("[NAV] 180-degree alignment turn started.\r\n");
    return 1U;
}

uint8_t LineFollower_TakeTurnDoneEvent(void) {
    uint8_t v = turnDoneEvent; turnDoneEvent = 0U; return v;
}

uint8_t LineFollower_TakeTurnFaultEvent(void) {
    uint8_t v = turnFaultEvent; turnFaultEvent = 0U; return v;
}

uint8_t LineFollower_TakeUTurnDoneEvent(void)
{
    uint8_t v = uTurnDoneEvent;
    uTurnDoneEvent = 0U;
    return v;
}

static void RunTurn(void) {
    uint32_t now = HAL_GetTick();
    if (turnMode == TURN_LEFT) TurnLeft(SHARP_SPEED);
    else if (turnMode == TURN_RIGHT) TurnRight(SHARP_SPEED);
    else if (turnMode == TURN_STRAIGHT) MoveForward(BASE_SPEED);
    else if (turnMode == TURN_AROUND) TurnRight(SHARP_SPEED);
    uint32_t minimumTurnMs = (turnMode == TURN_AROUND) ? UTURN_MIN_MS : TURN_MIN_MS;
    uint32_t timeoutMs = (turnMode == TURN_AROUND) ? UTURN_TIMEOUT_MS : TURN_TIMEOUT_MS;
    if ((uint32_t)(now - turnStartMs) < minimumTurnMs) return;
    /* Sensor state 5 means the center sensor sees the line in the existing map. */
    if (ReadSensors() == 5U) {
        if (turnLineCount < LINE_CONFIRM_READINGS) turnLineCount++;
    } else {
        turnLineCount = 0U;
    }
    if (turnLineCount >= LINE_CONFIRM_READINGS) {
        const char *done = (turnMode == TURN_LEFT) ? "LEFT" :
                           (turnMode == TURN_RIGHT) ? "RIGHT" :
                           (turnMode == TURN_AROUND) ? "UTURN" : "STRAIGHT";
        if (turnMode == TURN_AROUND) {
            uTurnDoneEvent = 1U;
            turnDoneEvent = 0U;
        } else {
            turnDoneEvent = 1U;
            uTurnDoneEvent = 0U;
        }
        turnMode = TURN_NONE;
        lastDirection = LAST_NONE;
        /* Do not insert a stop pulse after reacquiring the line. On the next
           control iteration the normal line follower resumes immediately. */
        printf("[NAV] Turn %s complete; center line reacquired; resuming line follow.\r\n", done);
        return;
    }
    if ((uint32_t)(now - turnStartMs) >= timeoutMs) {
        turnMode = TURN_NONE; turnFaultEvent = 1U;
        turnDoneEvent = 0U; uTurnDoneEvent = 0U;
        navigationFaultLatched = 1U; routeEnabled = 0U;
        StopMotors();
        printf("[NAV FAULT] Line not reacquired before turn timeout.\r\n");
    }
}

void LineFollower_Run(void) {
    CheckObstacle();

    /* Safety priority is unchanged: obstacle/navigation/route locks stop motors. */
    if (obstacleLatched || navigationFaultLatched || !routeEnabled || arrived) {
        StopMotors();
        return;
    }

    if (turnMode != TURN_NONE) {
        RunTurn();
        return;
    }

    uint8_t state = ReadSensors();
    switch (state) {
        case 7U:
            ReportLineStatus(state, "LINE NOT DETECTED - all sensors white; vehicle stopped.");
            MoveForward(STOP_SPEED);
            break;

        case 6U:
            ReportLineStatus(state, "RIGHT - sharp right correction.");
            TurnRight(SHARP_SPEED);
            lastDirection = LAST_RIGHT;
            break;

        case 5U:
            ReportLineStatus(state, "CENTERED - moving straight.");
            MoveForward(BASE_SPEED);
            break;

        case 4U:
            ReportLineStatus(state, "RIGHT - correcting line position.");
            TurnRight(TURN_SPEED);
            lastDirection = LAST_RIGHT;
            break;

        case 3U:
            ReportLineStatus(state, "LEFT - sharp left correction.");
            TurnLeft(SHARP_SPEED);
            lastDirection = LAST_LEFT;
            break;

        case 2U:
            if (lastDirection == LAST_LEFT) {
                ReportLineStatus(state, "LINE LOST - recovery turning left.");
                TurnLeft(70U);
            } else if (lastDirection == LAST_RIGHT) {
                ReportLineStatus(state, "LINE LOST - recovery turning right.");
                TurnRight(70U);
            } else {
                ReportLineStatus(state, "LINE LOST - no recovery direction; stopped.");
                MoveForward(STOP_SPEED);
            }
            break;

        case 1U:
            ReportLineStatus(state, "LEFT - correcting line position.");
            TurnLeft(TURN_SPEED);
            lastDirection = LAST_LEFT;
            break;

        case 0U:
        default:
            ReportLineStatus(state, "ALL SENSORS BLACK - stopped; check line/junction pattern.");
            MoveForward(STOP_SPEED);
            break;
    }
}




//---------------------------------------------------------------------------------------------------------------

///*
// * line_follower.c
// *
// *  Created on: Jun 16, 2026
// *      Author: dhanu
// */
//
///*
// * line_follower.c
// * STM32H563ZIT6: 3-IR line following + stop-and-wait obstacle protection
// *
// * IR: PF11=Left, PF12=Center, PF13=Right
// * Motor directions: Left PB0/PB1, Right PC0/PC1
// * PWM: TIM3_CH1=Left (PA6), TIM3_CH2=Right (PA7), ARR=99
// * Ultrasonic: TRIG=PC12, ECHO=PE5
// * Debug: printf -> USART2 (PD5 TX, 115200 baud)
// *
// * Behavior: when an obstacle is detected within the threshold, both motors
// * stop and remain stopped. No reverse, turning, bypassing, or line-search
// * movement is performed. Resume line following only after multiple valid
// * ultrasonic readings confirm that the obstacle has cleared.
// */
//
///*
// * STM32H563ZIT6 vehicle V2 controller.
// * Preserves original 3-IR patterns, PWM channels/speeds, ultrasonic threshold,
// * and the 3-valid-clear-reading obstacle latch. Adds USART1 route/turn control.
// */
//
//#include "line_follower.h"
//#include "ultrasonic.h"
//#include <stdio.h>
//#include <string.h>
//
//extern TIM_HandleTypeDef htim3;
//
//#define BASE_SPEED                 70U
//#define TURN_SPEED                 85U
//#define SHARP_SPEED                95U
//#define MAX_PWM                    99U
//#define STOP_SPEED                 0U
//#define OBSTACLE_DISTANCE_CM       20.0f
//#define ULTRASONIC_PERIOD_MS       100U
//#define CLEAR_READINGS_REQUIRED    3U
//#define TURN_MIN_MS                1500U  /* Commit to the requested turn for at least 1.5 s. */
//#define TURN_TIMEOUT_MS            4000U  /* Allow time to complete the committed turn and reacquire line. */
//#define LINE_CONFIRM_READINGS      3U
//
//typedef enum { LAST_NONE = 0, LAST_LEFT, LAST_RIGHT } LastDirection_t;
//typedef enum { TURN_NONE = 0, TURN_STRAIGHT, TURN_LEFT, TURN_RIGHT } TurnMode_t;
//
//static LastDirection_t lastDirection = LAST_NONE;
//static uint8_t obstacleLatched = 0U;
//static uint8_t navigationFaultLatched = 1U; /* fail-safe until ESP32 RESET */
//static uint8_t routeEnabled = 0U;
//static uint8_t arrived = 0U;
//static uint8_t turnDoneEvent = 0U;
//static uint8_t turnFaultEvent = 0U;
//static uint8_t turnLineCount = 0U;
//static TurnMode_t turnMode = TURN_NONE;
//static uint32_t turnStartMs = 0U;
//static uint32_t lastUltrasonicMs = 0U;
//static uint32_t lastDebugMs = 0U;
//static uint8_t clearReadings = 0U;
//static float lastDistanceCm = -1.0f;
//
///* Status telemetry addition only: sensor interpretation and motor actions stay unchanged. */
//static int8_t lastReportedLineStatus = -1;
//
//static void ReportLineStatus(uint8_t state, const char *message)
//{
//    if (lastReportedLineStatus != (int8_t)state) {
//        printf("[LINE STATUS] %s (sensor state=%u)\r\n", message, state);
//        lastReportedLineStatus = (int8_t)state;
//    }
//}
//
//static void ResetLineStatusReport(void)
//{
//    lastReportedLineStatus = -1;
//}
//
//static void SetLeftMotor(uint16_t pwm) {
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
//}
//
//static void SetRightMotor(uint16_t pwm) {
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
//}
//
//static void StopMotors(void) {
//    SetLeftMotor(0U); SetRightMotor(0U);
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin | M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin | M_IN4_Pin, GPIO_PIN_RESET);
//}
//
//static void MoveForward(uint16_t speed) {
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed); SetRightMotor(speed);
//}
//
///* Pivot turns retain the original left/right motor direction convention. */
//static void TurnLeft(uint16_t speed) {
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed); SetRightMotor(speed);
//}
//
//static void TurnRight(uint16_t speed) {
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_SET);
//    SetLeftMotor(speed); SetRightMotor(speed);
//}
//
///* Keep the source's existing active-low sensor inversion and bit order. */
//static uint8_t ReadSensors(void) {
//    uint32_t idr = GPIOF->IDR;
//    uint8_t L = (uint8_t)((idr >> 11) & 1U);
//    uint8_t C = (uint8_t)((idr >> 12) & 1U);
//    uint8_t R = (uint8_t)((idr >> 13) & 1U);
//    return (uint8_t)(~((L << 2) | (C << 1) | R) & 0x07U);
//}
//
//uint8_t GetSensorState(void) { return ReadSensors(); }
//
//static void CheckObstacle(void) {
//    uint32_t now = HAL_GetTick();
//    if ((uint32_t)(now - lastUltrasonicMs) < ULTRASONIC_PERIOD_MS) {
//        if (obstacleLatched) StopMotors();
//        return;
//    }
//    lastUltrasonicMs = now;
//    lastDistanceCm = Ultrasonic_ReadDistanceCm();
//    if (lastDistanceCm < 0.0f) {
//        if (obstacleLatched) { clearReadings = 0U; StopMotors(); }
//        if ((uint32_t)(now - lastDebugMs) >= 500U) {
//            printf("[ULTRA] ECHO timeout/no return\r\n"); lastDebugMs = now;
//        }
//        return; /* Timeout is never a clear-path confirmation. */
//    }
//    uint32_t d10 = (uint32_t)(lastDistanceCm * 10.0f + 0.5f);
//    if ((uint32_t)(now - lastDebugMs) >= 250U) {
//        printf("[ULTRA] %lu.%lu cm\r\n", (unsigned long)(d10 / 10U), (unsigned long)(d10 % 10U));
//        lastDebugMs = now;
//    }
//    if (lastDistanceCm <= OBSTACLE_DISTANCE_CM) {
//        clearReadings = 0U;
//        if (!obstacleLatched) {
//            obstacleLatched = 1U;
//            printf("[OBSTACLE] Detected at %lu.%lu cm. STOPPED.\r\n", (unsigned long)(d10 / 10U), (unsigned long)(d10 % 10U));
//        }
//        StopMotors();
//        return;
//    }
//    if (obstacleLatched) {
//        if (clearReadings < CLEAR_READINGS_REQUIRED) clearReadings++;
//        StopMotors();
//        if (clearReadings >= CLEAR_READINGS_REQUIRED) {
//            obstacleLatched = 0U; clearReadings = 0U;
//            printf("[OBSTACLE] Clear confirmed after 3 valid readings.\r\n");
//        } else {
//            printf("[OBSTACLE] Waiting for clear path (%u/%u)\r\n", clearReadings, CLEAR_READINGS_REQUIRED);
//        }
//    }
//}
//
//uint8_t LineFollower_IsObstacleLatched(void) { return obstacleLatched; }
//
//uint8_t LineFollower_IsNavigationFaultLatched(void) { return navigationFaultLatched; }
//
//void LineFollower_ResetNavigationFault(void) {
//    /* Deliberately does not clear obstacleLatched. */
//    navigationFaultLatched = 0U;
//    routeEnabled = 0U;
//    arrived = 0U;
//    turnMode = TURN_NONE;
//    turnDoneEvent = 0U;
//    turnFaultEvent = 0U;
//    StopMotors();
//    printf("[NAV] Navigation fault cleared; waiting for DEST command.\r\n");
//}
//
//void LineFollower_LatchStop(void) {
//    navigationFaultLatched = 1U;
//    routeEnabled = 0U;
//    arrived = 0U;
//    turnMode = TURN_NONE;
//    turnDoneEvent = 0U;
//    StopMotors();
//    printf("[NAV] STOP received; navigation fault latched.\r\n");
//}
//
//uint8_t LineFollower_SetDestination(void) {
//    if (navigationFaultLatched || obstacleLatched) return 0U;
//    arrived = 0U; routeEnabled = 1U; turnMode = TURN_NONE;
//    StopMotors();
//    printf("[NAV] Destination accepted; line following enabled.\r\n");
//    return 1U;
//}
//
//void LineFollower_Arrive(void) {
//    routeEnabled = 0U; arrived = 1U; turnMode = TURN_NONE;
//    StopMotors();
//    printf("[NAV] Destination reached; motors stopped.\r\n");
//}
//
//uint8_t LineFollower_RequestTurn(const char *direction) {
//    if (!direction || navigationFaultLatched || !routeEnabled || arrived || obstacleLatched || turnMode != TURN_NONE) return 0U;
//    if (strcmp(direction, "LEFT") == 0) turnMode = TURN_LEFT;
//    else if (strcmp(direction, "RIGHT") == 0) turnMode = TURN_RIGHT;
//    else if (strcmp(direction, "STRAIGHT") == 0) turnMode = TURN_STRAIGHT;
//    else return 0U;
//    turnStartMs = HAL_GetTick(); turnLineCount = 0U; turnDoneEvent = 0U; turnFaultEvent = 0U;
//    printf("[NAV] Turn started: %s\r\n", direction);
//    return 1U;
//}
//
//uint8_t LineFollower_TakeTurnDoneEvent(void) {
//    uint8_t v = turnDoneEvent; turnDoneEvent = 0U; return v;
//}
//
//uint8_t LineFollower_TakeTurnFaultEvent(void) {
//    uint8_t v = turnFaultEvent; turnFaultEvent = 0U; return v;
//}
//
//static void RunTurn(void) {
//    uint32_t now = HAL_GetTick();
//    if (turnMode == TURN_LEFT) TurnLeft(SHARP_SPEED);
//    else if (turnMode == TURN_RIGHT) TurnRight(SHARP_SPEED);
//    else if (turnMode == TURN_STRAIGHT) MoveForward(BASE_SPEED);
//    if ((uint32_t)(now - turnStartMs) < TURN_MIN_MS) return;
//    /* Sensor state 5 means the center sensor sees the line in the existing map. */
//    if (ReadSensors() == 5U) {
//        if (turnLineCount < LINE_CONFIRM_READINGS) turnLineCount++;
//    } else {
//        turnLineCount = 0U;
//    }
//    if (turnLineCount >= LINE_CONFIRM_READINGS) {
//        const char *done = (turnMode == TURN_LEFT) ? "LEFT" : (turnMode == TURN_RIGHT) ? "RIGHT" : "STRAIGHT";
//        turnMode = TURN_NONE; turnDoneEvent = 1U; lastDirection = LAST_NONE;
//        /* Do not insert a stop pulse after reacquiring the line. On the next
//           control iteration the normal line follower resumes immediately. */
//        printf("[NAV] Turn %s complete; center line reacquired; resuming line follow.\r\n", done);
//        return;
//    }
//    if ((uint32_t)(now - turnStartMs) >= TURN_TIMEOUT_MS) {
//        turnMode = TURN_NONE; turnFaultEvent = 1U; navigationFaultLatched = 1U; routeEnabled = 0U;
//        StopMotors();
//        printf("[NAV FAULT] Line not reacquired before turn timeout.\r\n");
//    }
//}
//
//void LineFollower_Run(void) {
//    CheckObstacle();
//
//    /* Safety priority is unchanged: obstacle/navigation/route locks stop motors. */
//    if (obstacleLatched || navigationFaultLatched || !routeEnabled || arrived) {
//        StopMotors();
//        return;
//    }
//
//    if (turnMode != TURN_NONE) {
//        RunTurn();
//        return;
//    }
//
//    uint8_t state = ReadSensors();
//    switch (state) {
//        case 7U:
//            ReportLineStatus(state, "LINE NOT DETECTED - all sensors white; vehicle stopped.");
//            MoveForward(STOP_SPEED);
//            break;
//
//        case 6U:
//            ReportLineStatus(state, "RIGHT - sharp right correction.");
//            TurnRight(SHARP_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 5U:
//            ReportLineStatus(state, "CENTERED - moving straight.");
//            MoveForward(BASE_SPEED);
//            break;
//
//        case 4U:
//            ReportLineStatus(state, "RIGHT - correcting line position.");
//            TurnRight(TURN_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 3U:
//            ReportLineStatus(state, "LEFT - sharp left correction.");
//            TurnLeft(SHARP_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 2U:
//            if (lastDirection == LAST_LEFT) {
//                ReportLineStatus(state, "LINE LOST - recovery turning left.");
//                TurnLeft(70U);
//            } else if (lastDirection == LAST_RIGHT) {
//                ReportLineStatus(state, "LINE LOST - recovery turning right.");
//                TurnRight(70U);
//            } else {
//                ReportLineStatus(state, "LINE LOST - no recovery direction; stopped.");
//                MoveForward(STOP_SPEED);
//            }
//            break;
//
//        case 1U:
//            ReportLineStatus(state, "LEFT - correcting line position.");
//            TurnLeft(TURN_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 0U:
//        default:
//            ReportLineStatus(state, "ALL SENSORS BLACK - stopped; check line/junction pattern.");
//            MoveForward(STOP_SPEED);
//            break;
//    }
//}

//--------------------------------------------------------------------------------------------------------


//#include "line_follower.h"
//#include "ultrasonic.h"
//#include <stdio.h>
//#include <string.h>
//
//extern TIM_HandleTypeDef htim3;
//
//#define BASE_SPEED                 70U
//#define TURN_SPEED                 85U
//#define SHARP_SPEED                95U
//#define MAX_PWM                    99U
//#define STOP_SPEED                 0U
//#define OBSTACLE_DISTANCE_CM       20.0f
//#define ULTRASONIC_PERIOD_MS       100U
//#define CLEAR_READINGS_REQUIRED    3U
//#define TURN_MIN_MS                220U
//#define TURN_TIMEOUT_MS            2500U
//#define LINE_CONFIRM_READINGS      3U
//
//typedef enum { LAST_NONE = 0, LAST_LEFT, LAST_RIGHT } LastDirection_t;
//typedef enum { TURN_NONE = 0, TURN_STRAIGHT, TURN_LEFT, TURN_RIGHT } TurnMode_t;
//
//static LastDirection_t lastDirection = LAST_NONE;
//static uint8_t obstacleLatched = 0U;
//static uint8_t navigationFaultLatched = 1U; /* fail-safe until ESP32 RESET */
//static uint8_t routeEnabled = 0U;
//static uint8_t arrived = 0U;
//static uint8_t turnDoneEvent = 0U;
//static uint8_t turnFaultEvent = 0U;
//static uint8_t turnLineCount = 0U;
//static TurnMode_t turnMode = TURN_NONE;
//static uint32_t turnStartMs = 0U;
//static uint32_t lastUltrasonicMs = 0U;
//static uint32_t lastDebugMs = 0U;
//static uint8_t clearReadings = 0U;
//static float lastDistanceCm = -1.0f;
//
//static void SetLeftMotor(uint16_t pwm) {
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
//}
//static void SetRightMotor(uint16_t pwm) {
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
//}
//static void StopMotors(void) {
//    SetLeftMotor(0U); SetRightMotor(0U);
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin | M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin | M_IN4_Pin, GPIO_PIN_RESET);
//}
//static void MoveForward(uint16_t speed) {
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed); SetRightMotor(speed);
//}
///* Pivot turns retain the original left/right motor direction convention. */
//static void TurnLeft(uint16_t speed) {
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed); SetRightMotor(speed);
//}
//static void TurnRight(uint16_t speed) {
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_SET);
//    SetLeftMotor(speed); SetRightMotor(speed);
//}
//
///* Keep the source's existing active-low sensor inversion and bit order. */
//static uint8_t ReadSensors(void) {
//    uint32_t idr = GPIOF->IDR;
//    uint8_t L = (uint8_t)((idr >> 11) & 1U);
//    uint8_t C = (uint8_t)((idr >> 12) & 1U);
//    uint8_t R = (uint8_t)((idr >> 13) & 1U);
//    return (uint8_t)(~((L << 2) | (C << 1) | R) & 0x07U);
//}
//uint8_t GetSensorState(void) { return ReadSensors(); }
//
//static void CheckObstacle(void) {
//    uint32_t now = HAL_GetTick();
//    if ((uint32_t)(now - lastUltrasonicMs) < ULTRASONIC_PERIOD_MS) {
//        if (obstacleLatched) StopMotors();
//        return;
//    }
//    lastUltrasonicMs = now;
//    lastDistanceCm = Ultrasonic_ReadDistanceCm();
//    if (lastDistanceCm < 0.0f) {
//        if (obstacleLatched) { clearReadings = 0U; StopMotors(); }
//        if ((uint32_t)(now - lastDebugMs) >= 500U) {
//            printf("[ULTRA] ECHO timeout/no return\r\n"); lastDebugMs = now;
//        }
//        return; /* Timeout is never a clear-path confirmation. */
//    }
//    uint32_t d10 = (uint32_t)(lastDistanceCm * 10.0f + 0.5f);
//    if ((uint32_t)(now - lastDebugMs) >= 250U) {
//        printf("[ULTRA] %lu.%lu cm\r\n", (unsigned long)(d10 / 10U), (unsigned long)(d10 % 10U));
//        lastDebugMs = now;
//    }
//    if (lastDistanceCm <= OBSTACLE_DISTANCE_CM) {
//        clearReadings = 0U;
//        if (!obstacleLatched) {
//            obstacleLatched = 1U;
//            printf("[OBSTACLE] Detected at %lu.%lu cm. STOPPED.\r\n", (unsigned long)(d10 / 10U), (unsigned long)(d10 % 10U));
//        }
//        StopMotors();
//        return;
//    }
//    if (obstacleLatched) {
//        if (clearReadings < CLEAR_READINGS_REQUIRED) clearReadings++;
//        StopMotors();
//        if (clearReadings >= CLEAR_READINGS_REQUIRED) {
//            obstacleLatched = 0U; clearReadings = 0U;
//            printf("[OBSTACLE] Clear confirmed after 3 valid readings.\r\n");
//        } else {
//            printf("[OBSTACLE] Waiting for clear path (%u/%u)\r\n", clearReadings, CLEAR_READINGS_REQUIRED);
//        }
//    }
//}
//
//uint8_t LineFollower_IsObstacleLatched(void) { return obstacleLatched; }
//uint8_t LineFollower_IsNavigationFaultLatched(void) { return navigationFaultLatched; }
//
//void LineFollower_ResetNavigationFault(void) {
//    /* Deliberately does not clear obstacleLatched. */
//    navigationFaultLatched = 0U;
//    routeEnabled = 0U;
//    arrived = 0U;
//    turnMode = TURN_NONE;
//    turnDoneEvent = 0U;
//    turnFaultEvent = 0U;
//    StopMotors();
//    printf("[NAV] Navigation fault cleared; waiting for DEST command.\r\n");
//}
//void LineFollower_LatchStop(void) {
//    navigationFaultLatched = 1U;
//    routeEnabled = 0U;
//    arrived = 0U;
//    turnMode = TURN_NONE;
//    turnDoneEvent = 0U;
//    StopMotors();
//    printf("[NAV] STOP received; navigation fault latched.\r\n");
//}
//uint8_t LineFollower_SetDestination(void) {
//    if (navigationFaultLatched || obstacleLatched) return 0U;
//    arrived = 0U; routeEnabled = 1U; turnMode = TURN_NONE;
//    StopMotors();
//    printf("[NAV] Destination accepted; line following enabled.\r\n");
//    return 1U;
//}
//void LineFollower_Arrive(void) {
//    routeEnabled = 0U; arrived = 1U; turnMode = TURN_NONE;
//    StopMotors();
//    printf("[NAV] Destination reached; motors stopped.\r\n");
//}
//uint8_t LineFollower_RequestTurn(const char *direction) {
//    if (!direction || navigationFaultLatched || !routeEnabled || arrived || obstacleLatched || turnMode != TURN_NONE) return 0U;
//    if (strcmp(direction, "LEFT") == 0) turnMode = TURN_LEFT;
//    else if (strcmp(direction, "RIGHT") == 0) turnMode = TURN_RIGHT;
//    else if (strcmp(direction, "STRAIGHT") == 0) turnMode = TURN_STRAIGHT;
//    else return 0U;
//    turnStartMs = HAL_GetTick(); turnLineCount = 0U; turnDoneEvent = 0U; turnFaultEvent = 0U;
//    printf("[NAV] Turn started: %s\r\n", direction);
//    return 1U;
//}
//uint8_t LineFollower_TakeTurnDoneEvent(void) {
//    uint8_t v = turnDoneEvent; turnDoneEvent = 0U; return v;
//}
//uint8_t LineFollower_TakeTurnFaultEvent(void) {
//    uint8_t v = turnFaultEvent; turnFaultEvent = 0U; return v;
//}
//
//static void RunTurn(void) {
//    uint32_t now = HAL_GetTick();
//    if (turnMode == TURN_LEFT) TurnLeft(SHARP_SPEED);
//    else if (turnMode == TURN_RIGHT) TurnRight(SHARP_SPEED);
//    else if (turnMode == TURN_STRAIGHT) MoveForward(BASE_SPEED);
//    if ((uint32_t)(now - turnStartMs) < TURN_MIN_MS) return;
//
//    /* Sensor state 5 means the center sensor sees the line in the existing map. */
//    if (ReadSensors() == 5U) {
//        if (turnLineCount < LINE_CONFIRM_READINGS) turnLineCount++;
//    } else {
//        turnLineCount = 0U;
//    }
//    if (turnLineCount >= LINE_CONFIRM_READINGS) {
//        const char *done = (turnMode == TURN_LEFT) ? "LEFT" : (turnMode == TURN_RIGHT) ? "RIGHT" : "STRAIGHT";
//        turnMode = TURN_NONE; turnDoneEvent = 1U; lastDirection = LAST_NONE;
//        StopMotors();
//        printf("[NAV] Turn %s complete; center line reacquired.\r\n", done);
//        return;
//    }
//    if ((uint32_t)(now - turnStartMs) >= TURN_TIMEOUT_MS) {
//        turnMode = TURN_NONE; turnFaultEvent = 1U; navigationFaultLatched = 1U; routeEnabled = 0U;
//        StopMotors();
//        printf("[NAV FAULT] Line not reacquired before turn timeout.\r\n");
//    }
//}
//
//void LineFollower_Run(void) {
//    CheckObstacle();
//    /* Obstacle stop has highest priority and never clears a navigation fault. */
//    if (obstacleLatched || navigationFaultLatched || !routeEnabled || arrived) { StopMotors(); return; }
//    if (turnMode != TURN_NONE) { RunTurn(); return; }
//
//    uint8_t state = ReadSensors();
//    switch (state) {
//        case 7U: MoveForward(STOP_SPEED); break; /* all white */
//        case 6U: TurnRight(SHARP_SPEED); lastDirection = LAST_RIGHT; break;
//        case 5U: MoveForward(BASE_SPEED); break; /* center */
//        case 4U: TurnRight(TURN_SPEED); lastDirection = LAST_RIGHT; break;
//        case 3U: TurnLeft(SHARP_SPEED); lastDirection = LAST_LEFT; break;
//        case 2U:
//            if (lastDirection == LAST_LEFT) TurnLeft(70U);
//            else if (lastDirection == LAST_RIGHT) TurnRight(70U);
//            else MoveForward(STOP_SPEED);
//            break;
//        case 1U: TurnLeft(TURN_SPEED); lastDirection = LAST_LEFT; break;
//        case 0U: default: MoveForward(STOP_SPEED); break; /* all black */
//    }
//}



//----------------------------------------------------------------------------------------------------------------------------------------------------

//#include "line_follower.h"
//#include "ultrasonic.h"
//#include <stdio.h>
//
//extern TIM_HandleTypeDef htim3;
//
//#define BASE_SPEED                 70U
//#define TURN_SPEED                 85U
//#define SHARP_SPEED                95U
//#define MAX_PWM                    99U
//#define STOP_SPEED                 0U
//
//#define OBSTACLE_DISTANCE_CM       20.0f
//#define ULTRASONIC_PERIOD_MS       100U
//#define CLEAR_READINGS_REQUIRED    3U
//
///* Preserve the original line-following direction memory. */
//typedef enum
//{
//    LAST_NONE = 0,
//    LAST_LEFT,
//    LAST_RIGHT
//} LastDirection_t;
//
//static LastDirection_t lastDirection = LAST_NONE;
//static uint8_t obstacleLatched = 0U;
//static uint8_t clearReadings = 0U;
//static uint32_t lastUltrasonicMs = 0U;
//static uint32_t lastDebugMs = 0U;
//static float lastDistanceCm = -1.0f;
//
///* ---------------------------- Motor PWM ---------------------------- */
//static void SetLeftMotor(uint16_t pwm)
//{
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
//}
//
//static void SetRightMotor(uint16_t pwm)
//{
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
//}
//
//static void StopMotors(void)
//{
//    SetLeftMotor(STOP_SPEED);
//    SetRightMotor(STOP_SPEED);
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin | M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin | M_IN4_Pin, GPIO_PIN_RESET);
//}
//
//static void MoveForward(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
//static void TurnLeft(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
//static void TurnRight(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_SET);
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
///* ---------------------------- IR sensors ---------------------------- */
///* Preserve the original PF11/PF12/PF13 bit order and inversion. */
//static inline uint8_t ReadSensors(void)
//{
//    uint32_t idr = GPIOF->IDR;
//    uint8_t L = (uint8_t)((idr >> 11) & 1U);
//    uint8_t C = (uint8_t)((idr >> 12) & 1U);
//    uint8_t R = (uint8_t)((idr >> 13) & 1U);
//    return (uint8_t)(~((L << 2) | (C << 1) | R) & 0x07U);
//}
//
//uint8_t GetSensorState(void)
//{
//    return ReadSensors();
//}
//
///* ----------------------- Ultrasonic supervision --------------------- */
//static void CheckObstacle(void)
//{
//    uint32_t now = HAL_GetTick();
//    if ((uint32_t)(now - lastUltrasonicMs) < ULTRASONIC_PERIOD_MS)
//    {
//        if (obstacleLatched) StopMotors();
//        return;
//    }
//
//    lastUltrasonicMs = now;
//    lastDistanceCm = Ultrasonic_ReadDistanceCm();
//
//    if (lastDistanceCm < 0.0f)
//    {
//        /* Fail-safe: a timeout cannot confirm that the path is clear. */
//        if (obstacleLatched)
//        {
//            clearReadings = 0U;
//            StopMotors();
//        }
//        if ((uint32_t)(now - lastDebugMs) >= 500U)
//        {
//            printf("[ULTRA] ECHO timeout/no return\r\n");
//            lastDebugMs = now;
//        }
//        return;
//    }
//
//    uint32_t distanceTenths = (uint32_t)(lastDistanceCm * 10.0f + 0.5f);
//    printf("[ULTRA] %lu.%lu cm\r\n",
//           (unsigned long)(distanceTenths / 10U),
//           (unsigned long)(distanceTenths % 10U));
//
//    if (lastDistanceCm <= OBSTACLE_DISTANCE_CM)
//    {
//        clearReadings = 0U;
//        if (!obstacleLatched)
//        {
//            obstacleLatched = 1U;
//            printf("[OBSTACLE] Detected at %lu.%lu cm. STOPPED.\r\n",
//                   (unsigned long)(distanceTenths / 10U),
//                   (unsigned long)(distanceTenths % 10U));
//        }
//        StopMotors();
//        return;
//    }
//
//    /* A latched stop clears only after repeated valid readings beyond the
//     * threshold. An ECHO timeout never counts as a clear reading. */
//    if (obstacleLatched)
//    {
//        if (clearReadings < CLEAR_READINGS_REQUIRED)
//            clearReadings++;
//
//        StopMotors();
//
//        if (clearReadings >= CLEAR_READINGS_REQUIRED)
//        {
//            obstacleLatched = 0U;
//            clearReadings = 0U;
//            printf("[OBSTACLE] Clear confirmed. Resuming line following.\r\n");
//        }
//        else
//        {
//            printf("[OBSTACLE] Waiting for clear path (%u/%u)\r\n",
//                   clearReadings, CLEAR_READINGS_REQUIRED);
//        }
//    }
//}
//
///* ---------------------------- Main control -------------------------- */
//void LineFollower_Run(void)
//{
//    /* Check distance before issuing any movement command. */
//    CheckObstacle();
//
//    /* While an obstacle is latched, this is the only allowed motor action. */
//    if (obstacleLatched)
//    {
//        StopMotors();
//        return;
//    }
//
//    uint8_t state = ReadSensors();
//
//    switch (state)
//    {
//        case 7U: /* 111: all white */
//            MoveForward(STOP_SPEED);
//            break;
//
//        case 6U: /* Original sharp-right case */
//            TurnRight(SHARP_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 5U: /* Center line */
//            MoveForward(BASE_SPEED);
//            break;
//
//        case 4U: /* Original right correction */
//            TurnRight(TURN_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 3U: /* Original sharp-left case */
//            TurnLeft(SHARP_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 2U: /* Original line-lost recovery */
//            if (lastDirection == LAST_LEFT) TurnLeft(70U);
//            else if (lastDirection == LAST_RIGHT) TurnRight(70U);
//            else MoveForward(STOP_SPEED);
//            break;
//
//        case 1U: /* Original left correction */
//            TurnLeft(TURN_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 0U: /* All black */
//        default:
//            MoveForward(STOP_SPEED);
//            break;
//    }
//}
//


//-------------------------------------------------------------------------------------------------------------------------------------------

///*
// * line_follower.c
// * STM32H563ZIT6: 3-IR line following + HC-SR04 obstacle avoidance
// *
// * IR: PF11=Left, PF12=Center, PF13=Right
// * Motor directions: Left PB0/PB1, Right PC0/PC1
// * PWM: TIM3_CH1=Left (PA6), TIM3_CH2=Right (PA7), ARR=99
// * Ultrasonic: TRIG=PC12, ECHO=PE5
// * Debug: printf -> USART2 (PD5 TX, 115200 baud)
// */
//
//#include "line_follower.h"
//#include "ultrasonic.h"
//#include <stdio.h>
//
//extern TIM_HandleTypeDef htim3;
//
///* Preserve the known-working line follower PWM values. */
//#define BASE_SPEED                 70U
//#define TURN_SPEED                 85U
//#define SHARP_SPEED                95U
//#define MAX_PWM                    99U
//#define STOP_SPEED                 0U
//
///* Obstacle settings. Tune these on the raised-wheel/low-speed test rig. */
//#define OBSTACLE_DISTANCE_CM       20.0f
//#define ULTRASONIC_PERIOD_MS       70U
//#define OBSTACLE_STOP_MS           120U
//#define REVERSE_TIME_MS            300U
//#define AVOID_TURN_TIME_MS         420U
//#define BYPASS_FORWARD_TIME_MS     550U
//#define LINE_SEARCH_TIMEOUT_MS     1800U
//#define POST_REACQUIRE_PAUSE_MS    80U
//
//#define REVERSE_SPEED              60U
//#define AVOID_TURN_SPEED           75U
//#define BYPASS_SPEED               65U
//#define SEARCH_SPEED               48U
//
///* Original line-following memory. */
//typedef enum
//{
//    LAST_NONE = 0,
//    LAST_LEFT,
//    LAST_RIGHT
//} LastDirection_t;
//
//static LastDirection_t lastDirection = LAST_NONE;
//
///* Obstacle side is a heuristic: one forward ultrasonic sensor cannot see
// * which side of the obstacle is open. We choose the opposite of the most
// * recent line correction, or RIGHT when no direction is known. */
//typedef enum
//{
//    AVOID_LEFT = 0,
//    AVOID_RIGHT
//} AvoidDirection_t;
//
//static AvoidDirection_t avoidDirection = AVOID_RIGHT;
//
//typedef enum
//{
//    ROBOT_FOLLOW_LINE = 0,
//    ROBOT_OBSTACLE_STOP,
//    ROBOT_REVERSE,
//    ROBOT_TURN_AROUND_OBSTACLE,
//    ROBOT_BYPASS_FORWARD,
//    ROBOT_SEARCH_LINE,
//    ROBOT_REACQUIRE_PAUSE,
//    ROBOT_SEARCH_FAILED
//} RobotState_t;
//
//static RobotState_t robotState = ROBOT_FOLLOW_LINE;
//static uint32_t stateStartMs = 0U;
//static uint32_t lastUltrasonicMs = 0U;
//static uint32_t lastDebugMs = 0U;
//static float lastDistanceCm = -1.0f;
//
///* ---------------------------- Motor PWM ---------------------------- */
//static void SetLeftMotor(uint16_t pwm)
//{
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
//}
//
//static void SetRightMotor(uint16_t pwm)
//{
//    if (pwm > MAX_PWM) pwm = MAX_PWM;
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
//}
//
//static void StopMotors(void)
//{
//    SetLeftMotor(STOP_SPEED);
//    SetRightMotor(STOP_SPEED);
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin | M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin | M_IN4_Pin, GPIO_PIN_RESET);
//}
//
//static void MoveForward(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
//static void MoveReverse(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_SET);
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
///* Pivot turns: left motor backward/right forward, or the reverse. */
//static void TurnLeft(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
//static void TurnRight(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_SET);
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
//static void TurnForAvoidance(uint16_t speed)
//{
//    if (avoidDirection == AVOID_LEFT) TurnLeft(speed);
//    else TurnRight(speed);
//}
//
///* ---------------------------- IR sensors ---------------------------- */
///* Preserve original electrical convention and bit ordering:
// * read PF11/PF12/PF13, then invert so WHITE=1 and BLACK=0.
// * Returned bits are L:C:R. */
//static inline uint8_t ReadSensors(void)
//{
//    uint32_t idr = GPIOF->IDR;
//    uint8_t L = (uint8_t)((idr >> 11) & 1U);
//    uint8_t C = (uint8_t)((idr >> 12) & 1U);
//    uint8_t R = (uint8_t)((idr >> 13) & 1U);
//    return (uint8_t)(~((L << 2) | (C << 1) | R) & 0x07U);
//}
//
//uint8_t GetSensorState(void)
//{
//    return ReadSensors();
//}
//
///* A line is considered detected when at least one sensor gives one of the
// * line-following patterns already used by the original code. */
//static uint8_t IsLineDetected(uint8_t state)
//{
//    return (state == 1U || state == 3U || state == 4U ||
//            state == 5U || state == 6U) ? 1U : 0U;
//}
//
//static void EnterState(RobotState_t nextState)
//{
//    robotState = nextState;
//    stateStartMs = HAL_GetTick();
//}
//
///* ----------------------- Ultrasonic supervision --------------------- */
//static void UpdateUltrasonicAndMaybeAvoid(void)
//{
//    uint32_t now = HAL_GetTick();
//    if ((uint32_t)(now - lastUltrasonicMs) < ULTRASONIC_PERIOD_MS)
//        return;
//
//    lastUltrasonicMs = now;
//    lastDistanceCm = Ultrasonic_ReadDistanceCm();
//
//    if (lastDistanceCm < 0.0f)
//    {
//        /* A timeout is not treated as an obstacle: HC-SR04 commonly times
//         * out when no target is within its useful range. Check wiring if it
//         * times out continuously. */
//        if ((uint32_t)(now - lastDebugMs) >= 500U)
//        {
//            printf("[ULTRA] ECHO timeout/no return\r\n");
//            lastDebugMs = now;
//        }
//        return;
//    }
//
//    /* Avoid %f so this also works with embedded printf builds that omit
//     * floating-point formatting support. Print distance to 0.1 cm. */
//    uint32_t distanceTenths = (uint32_t)(lastDistanceCm * 10.0f + 0.5f);
//    printf("[ULTRA] %lu.%lu cm\r\n",
//           (unsigned long)(distanceTenths / 10U),
//           (unsigned long)(distanceTenths % 10U));
//
//    if (lastDistanceCm <= OBSTACLE_DISTANCE_CM)
//    {
//        /* Choose a consistent side based on recent line correction. This is
//         * a prototype heuristic, not true left/right obstacle sensing. */
//        if (lastDirection == LAST_LEFT) avoidDirection = AVOID_RIGHT;
//        else if (lastDirection == LAST_RIGHT) avoidDirection = AVOID_LEFT;
//        else avoidDirection = AVOID_RIGHT;
//
//        printf("[OBSTACLE] %lu.%lu cm: stopping; bypass %s\r\n",
//               (unsigned long)(distanceTenths / 10U),
//               (unsigned long)(distanceTenths % 10U),
//               (avoidDirection == AVOID_LEFT) ? "LEFT" : "RIGHT");
//        StopMotors();
//        EnterState(ROBOT_OBSTACLE_STOP);
//    }
//}
//
///* ---------------------- Obstacle state machine ---------------------- */
//static void RunObstacleStateMachine(void)
//{
//    uint32_t now = HAL_GetTick();
//    uint32_t elapsed = (uint32_t)(now - stateStartMs);
//
//    switch (robotState)
//    {
//        case ROBOT_OBSTACLE_STOP:
//            StopMotors();
//            if (elapsed >= OBSTACLE_STOP_MS)
//            {
//                printf("[AVOID] reversing\r\n");
//                MoveReverse(REVERSE_SPEED);
//                EnterState(ROBOT_REVERSE);
//            }
//            break;
//
//        case ROBOT_REVERSE:
//            MoveReverse(REVERSE_SPEED);
//            if (elapsed >= REVERSE_TIME_MS)
//            {
//                StopMotors();
//                if (avoidDirection == AVOID_LEFT)
//                    printf("[AVOID] pivot LEFT\r\n");
//                else
//                    printf("[AVOID] pivot RIGHT\r\n");
//                TurnForAvoidance(AVOID_TURN_SPEED);
//                EnterState(ROBOT_TURN_AROUND_OBSTACLE);
//            }
//            break;
//
//        case ROBOT_TURN_AROUND_OBSTACLE:
//            TurnForAvoidance(AVOID_TURN_SPEED);
//            if (elapsed >= AVOID_TURN_TIME_MS)
//            {
//                printf("[AVOID] moving past obstacle\r\n");
//                MoveForward(BYPASS_SPEED);
//                EnterState(ROBOT_BYPASS_FORWARD);
//            }
//            break;
//
//        case ROBOT_BYPASS_FORWARD:
//            MoveForward(BYPASS_SPEED);
//            if (elapsed >= BYPASS_FORWARD_TIME_MS)
//            {
//                StopMotors();
//                printf("[AVOID] searching for line\r\n");
//                EnterState(ROBOT_SEARCH_LINE);
//            }
//            break;
//
//        case ROBOT_SEARCH_LINE:
//        {
//            uint8_t sensorState = ReadSensors();
//            if (IsLineDetected(sensorState))
//            {
//                StopMotors();
//                printf("[AVOID] line reacquired; state=%u\r\n", sensorState);
//                EnterState(ROBOT_REACQUIRE_PAUSE);
//            }
//            else if (elapsed >= LINE_SEARCH_TIMEOUT_MS)
//            {
//                StopMotors();
//                printf("[FAULT] line not reacquired; stopped for safety\r\n");
//                EnterState(ROBOT_SEARCH_FAILED);
//            }
//            else
//            {
//                /* Sweep slowly in the selected bypass direction until an IR
//                 * sensor sees the line. */
//                TurnForAvoidance(SEARCH_SPEED);
//            }
//            break;
//        }
//
//        case ROBOT_REACQUIRE_PAUSE:
//            StopMotors();
//            if (elapsed >= POST_REACQUIRE_PAUSE_MS)
//            {
//                printf("[AVOID] resuming line following\r\n");
//                EnterState(ROBOT_FOLLOW_LINE);
//            }
//            break;
//
//        case ROBOT_SEARCH_FAILED:
//            /* Latch stopped; do not resume blindly. A reset/power cycle is
//             * required after manually placing the robot back on the line. */
//            StopMotors();
//            break;
//
//        case ROBOT_FOLLOW_LINE:
//        default:
//            break;
//    }
//}
//
///* ---------------------------- Main control -------------------------- */
//void LineFollower_Run(void)
//{
//    if (robotState != ROBOT_FOLLOW_LINE)
//    {
//        RunObstacleStateMachine();
//        return;
//    }
//
//    /* Ultrasonic obstacle detection has priority over normal line following. */
//    UpdateUltrasonicAndMaybeAvoid();
//    if (robotState != ROBOT_FOLLOW_LINE)
//    {
//        RunObstacleStateMachine();
//        return;
//    }
//
//    uint8_t state = ReadSensors();
//
//    switch (state)
//    {
//        case 7U: /* 111: all white */
//            MoveForward(STOP_SPEED);
//            break;
//
//        case 6U: /* Original sharp-right case */
//            TurnRight(SHARP_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 5U: /* Center line */
//            MoveForward(BASE_SPEED);
//            break;
//
//        case 4U: /* Original right correction */
//            TurnRight(TURN_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 3U: /* Original sharp-left case */
//            TurnLeft(SHARP_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 2U: /* Original line-lost recovery */
//            if (lastDirection == LAST_LEFT) TurnLeft(70U);
//            else if (lastDirection == LAST_RIGHT) TurnRight(70U);
//            else MoveForward(STOP_SPEED);
//            break;
//
//        case 1U: /* Original left correction */
//            TurnLeft(TURN_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 0U: /* All black */
//        default:
//            MoveForward(STOP_SPEED);
//            break;
//    }
//}

//-------------------------------------------------------------------------------------------------------------------------------------------------

//#include "line_follower.h"
//#include <stdio.h>
//
//extern TIM_HandleTypeDef htim3;
//
///* PWM Limits */
//#define BASE_SPEED     70
//#define TURN_SPEED     85
//#define SHARP_SPEED    95
//#define MAX_PWM        99
//#define STOP           0
//
//
//
//
//
//typedef enum
//{
//    LAST_NONE,
//    LAST_LEFT,
//    LAST_RIGHT
//} LastDirection_t;
//
//static LastDirection_t lastDirection = LAST_NONE;
//
///*---------------- Motor Functions ----------------*/
//
//static void SetLeftMotor(uint16_t pwm)
//{
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
//}
//
//static void SetRightMotor(uint16_t pwm)
//{
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
//}
//
//static void MoveForward(uint16_t speed)
//{
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
//static void TurnLeft(uint16_t speed)
//{
//    // Left motor backward
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_SET);
//
//    // Right motor forward
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_RESET);
//
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
//static void TurnRight(uint16_t speed)
//{
//    // Left motor forward
//    HAL_GPIO_WritePin(GPIOB, M_IN1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, M_IN2_Pin, GPIO_PIN_RESET);
//
//    // Right motor backward
//    HAL_GPIO_WritePin(GPIOC, M_IN3_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOC, M_IN4_Pin, GPIO_PIN_SET);
//
//    SetLeftMotor(speed);
//    SetRightMotor(speed);
//}
//
///*--------------- Sensor Reading ------------------*/
//
///*=========================================================
// * CHANGE 1
// * Direct Register Sensor Reading
// *
// * Faster than HAL_GPIO_ReadPin()
// * Reads all GPIO pins in one instruction.
// *=========================================================*/
//static inline uint8_t ReadSensors(void)
//{
//    uint32_t idr = GPIOF->IDR;
//
//    uint8_t L = (idr >> 11) & 0x01;
//    uint8_t C = (idr >> 12) & 0x01;
//    uint8_t R = (idr >> 13) & 0x01;
//
//    /*
//        Physical sensor:
//
//        White = 0
//        Black = 1
//
//        We invert so that
//
//        White = 1
//        Black = 0
//    */
//
//    return (~((L << 2) | (C << 1) | R)) & 0x07;
//}
//
///*-------------- Line Follower --------------------*/
//
//
//uint8_t GetSensorState(void)
//{
//    return ReadSensors();
//}
//
////void LineFollower_Run(void)
////{
////    uint8_t state = ReadSensors();
////
////	switch(state)
////	{
////	    case 5:   // Center
////	        MoveForward(BASE_SPEED);
////	        break;
////
////	    case 6:   // Left
////	        TurnLeft(TURN_SPEED);
////	        break;
////
////	    case 3:   // Right
////	        TurnRight(TURN_SPEED);
////	        break;
////
////	    case 4:   // Sharp Left
////	        TurnLeft(SHARP_SPEED);
////	        break;
////
////	    case 1:   // Sharp Right
////	        TurnRight(SHARP_SPEED);
////	        break;
////
////	    case 0:   // All Black
////	        MoveForward(STOP);
////	        break;
////
////	    case 7:   // Lost Line
////	        if(lastDirection == LAST_LEFT)
////	            TurnLeft(70);
////	        else if(lastDirection == LAST_RIGHT)
////	            TurnRight(70);
////	        else
////	            MoveForward(0);
////	        break;
////	}
////}
//
//
//void LineFollower_Run(void)
//{
//    uint8_t state = ReadSensors();
//
//    switch(state)
//    {
//        case 7:   // 111 - All WHITE - STOP
//            MoveForward(0);
//            break;
//
//        case 6:   // 110 - RIGHT on BLACK
//            TurnRight(SHARP_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 5:   // 101 - CENTER on BLACK - FORWARD
//            MoveForward(BASE_SPEED);
//            break;
//
//        case 4:   // 100 - CENTER + RIGHT on BLACK - RIGHT
//            TurnRight(TURN_SPEED);
//            lastDirection = LAST_RIGHT;
//            break;
//
//        case 3:   // 011 - LEFT on BLACK
//            TurnLeft(SHARP_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 2:   // 010 - LEFT + RIGHT on BLACK - LINE LOST
//            if(lastDirection == LAST_LEFT)
//            {
//                TurnLeft(70);
//            }
//            else if(lastDirection == LAST_RIGHT)
//            {
//                TurnRight(70);
//            }
//            else
//            {
//                MoveForward(0);
//            }
//            break;
//
//        case 1:   // 001 - LEFT + CENTER on BLACK - LEFT
//            TurnLeft(TURN_SPEED);
//            lastDirection = LAST_LEFT;
//            break;
//
//        case 0:   // 000 - ALL BLACK - STOP
//            MoveForward(0);
//            break;
//
//        default:
//            MoveForward(0);
//            break;
//    }
//}
