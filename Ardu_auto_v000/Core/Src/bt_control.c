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
	switch(msg) {
		case BT_TURN_RIGHT:
			new = CMD_TURN_RIGHT;
			break;
		case BT_TURN_LEFT:
			new = CMD_TURN_LEFT;
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
			break;
		default:
			new = CMD_NONE;
	}
	cmd->new_cmd = new;
}
