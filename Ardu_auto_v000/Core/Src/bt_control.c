/*
 * bt_control.c
 *
 *  Created on: 7 Apr 2022
 *      Author: samul
 */

#include "main.h"
#include <stdio.h>
#include <L3G4200D.h>
#include <string.h>
#include "bt_control.h"
#include "motor_control.h"

void handle_bt_msg(uint8_t msg, Cmd_holder cmd) {
	uint8_t new = CMD_NONE;
	uint8_t moving = cmd->moving;
	switch(msg) {
		case BT_TURN_LEFT:
			switch(moving){
				case MOVING_FORWARD:
					new = CMD_TURN_LEFT_FWD;
					break;
				case MOVING_REVERSE:
					new = CMD_TURN_LEFT_REV;
					break;
				case STOPPED:
					new = CMD_TURN_LEFT_STOPPED;
					break;
			}
			break;
		case BT_TURN_RIGHT:
			switch(moving){
				case MOVING_FORWARD:
					new = CMD_TURN_RIGHT_FWD;
					break;
				case MOVING_REVERSE:
					new = CMD_TURN_RIGHT_REV;
					break;
				case STOPPED:
					new = CMD_TURN_RIGHT_STOPPED;
					break;
			}
			break;
		case BT_FORWARD:
			new = CMD_FORWARD;
			break;
		case BT_REVERSE:
			new = CMD_REVERSE;
			break;
		case BT_X:
			break;
		case BT_Y:
			new = CMD_FAST_STOP;
			break;
		default:
			new = CMD_NONE;
			break;
	}
	cmd->new_cmd = new;
}
