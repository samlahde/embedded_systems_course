/*
 * bt_control.h
 *
 *  Created on: 7 Apr 2022
 *      Author: samul
 */

#ifndef INC_BT_CONTROL_H_
#define INC_BT_CONTROL_H_

#include "cmd.h"

#define BT_TURN_LEFT   76
#define BT_TURN_RIGHT  82
#define BT_FORWARD     70
#define BT_REVERSE     71
#define BT_X           88
#define BT_Y           89
#define BT_NO_COMMAND  0

void handle_bt_msg(uint8_t msg, Cmd_holder cmd);


#endif /* INC_BT_CONTROL_H_ */
