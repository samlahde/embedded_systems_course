/*
 * motor_control.h
 *
 *  Created on: Apr 3, 2022
 *      Author: samul
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#define CMD_NONE	   0
#define CMD_STOP	   1
#define CMD_FAST_STOP  2
#define CMD_FORWARD    3
#define CMD_REVERSE    4
#define CMD_TURN_RIGHT 5
#define CMD_TURN_LEFT  6

#define LEFT_MT        0
#define RIGHT_MT       1

#define MT_STOP        0
#define MT_FAST_STOP   1
#define MT_FORWARD     2
#define MT_REVERSE     3

#define STOPPED        0
#define MOVING_FORWARD 1
#define MOVING_REVERSE 2

typedef struct Cmd_holder_{
	uint8_t old_cmd;
	uint8_t new_cmd;
	uint8_t moving;
} *Cmd_holder;

void motor_set(uint8_t motor, uint8_t command);
void handle_driving(Cmd_holder cmd);
void print_driving_state();
uint8_t get_motor_state(uint8_t motor);

#endif /* INC_MOTOR_CONTROL_H_ */
