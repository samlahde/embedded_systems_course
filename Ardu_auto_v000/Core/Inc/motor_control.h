/*
 * motor_control.h
 *
 *  Created on: Apr 3, 2022
 *      Author: samul
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"

#define BT_TURN_RIGHT  82
#define BT_TURN_LEFT   76
#define BT_FORWARD     70
#define BT_REVERSE     71
#define BT_X           88
#define BT_Y           89
#define BT_NO_COMMAND  0

#define CMD_STOP	   0
#define CMD_FAST_STOP  1
#define CMD_FORWARD    2
#define CMD_REVERSE    3
#define CMD_TURN_RIGHT 4
#define CMD_TURN_LEFT  5

#define LEFT_MT        0
#define RIGHT_MT       1

#define MT_STOP        0
#define MT_FAST_STOP   1
#define MT_FORWARD     2
#define MT_REVERSE     3

#define STOPPED        0
#define MOVING_FORWARD 1
#define MOVING_REVERSE 2

typedef struct cmd_holder {
	uint8_t *old_cmd;
	uint8_t *new_cmd;
	uint8_t *moving;
};

void motor_set(uint8_t motor, uint8_t command);
void handle_driving(cmd_holder cmd);

#endif /* INC_MOTOR_CONTROL_H_ */
