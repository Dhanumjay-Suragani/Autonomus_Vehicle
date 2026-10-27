/*
 * line_follower.h
 *
 *  Created on: Jun 16, 2026
 *      Author: dhanu
 */

#ifndef INC_LINE_FOLLOWER_H_
#define INC_LINE_FOLLOWER_H_
#include "main.h"
void LineFollower_Run(void);
uint8_t GetSensorState(void);
void LineFollower_ResetNavigationFault(void);
void LineFollower_LatchStop(void);
uint8_t LineFollower_SetDestination(void);
void LineFollower_Arrive(void);
uint8_t LineFollower_RequestTurn(const char *direction);
uint8_t LineFollower_TakeTurnDoneEvent(void);
uint8_t LineFollower_TakeTurnFaultEvent(void);
uint8_t LineFollower_IsObstacleLatched(void);
uint8_t LineFollower_IsNavigationFaultLatched(void);
uint8_t LineFollower_RequestUTurn(void);
uint8_t LineFollower_TakeUTurnDoneEvent(void);

#endif /* INC_LINE_FOLLOWER_H_ */
