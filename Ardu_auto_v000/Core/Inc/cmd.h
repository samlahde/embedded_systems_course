/*
 * cmd.h
 *
 *  Created on: 7 Apr 2022
 *      Author: samul
 */

#ifndef INC_CMD_H_
#define INC_CMD_H_

#define CMD_NONE	   			0
#define CMD_STOP	   			1
#define CMD_FAST_STOP  			2
#define CMD_FORWARD    			3
#define CMD_REVERSE    			4
#define CMD_TURN_LEFT_FWD  		5
#define CMD_TURN_LEFT_REV 		6
#define CMD_TURN_LEFT_STOPPED	7
#define CMD_TURN_RIGHT_FWD  	8
#define CMD_TURN_RIGHT_REV  	9
#define CMD_TURN_RIGHT_STOPPED  10

typedef struct Cmd_holder_{
	uint8_t old_cmd;
	uint8_t new_cmd;
	uint8_t moving;
} *Cmd_holder;

#endif /* INC_CMD_H_ */
