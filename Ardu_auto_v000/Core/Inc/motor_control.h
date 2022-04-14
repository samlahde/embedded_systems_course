/*
 * motor_control.h
 *
 *  Created on: Apr 3, 2022
 *      Author: samul
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "cmd.h"

#define LEFT_MT        0
#define RIGHT_MT       1

#define MT_STOP        0
#define MT_FAST_STOP   1
#define MT_FORWARD     2
#define MT_REVERSE     3

#define STOPPED        0
#define MOVING_FORWARD 1
#define MOVING_REVERSE 2

void motor_set(uint8_t motor, uint8_t command);
void handle_driving(Cmd_holder cmd);
void print_driving_state(Cmd_holder cmd);
uint8_t get_motor_state(uint8_t motor);

#endif /* INC_MOTOR_CONTROL_H_ */
