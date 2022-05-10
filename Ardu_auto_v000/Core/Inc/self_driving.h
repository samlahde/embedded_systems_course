/*
 * self_driving.h
 *
 *  Created on: 14 Apr 2022
 *      Author: samul
 */

#ifndef INC_SELF_DRIVING_H_
#define INC_SELF_DRIVING_H_

#include "cmd.h"
typedef struct IR_data_type_{
	bool LEFT;
	bool RIGHT;
	bool FRONT;
} IR_data_type;

void self_driving(Cmd_holder cmd_holder, uint32_t * IR_data);
IR_data_type get_ir(uint32_t * IR_data);



#endif /* INC_SELF_DRIVING_H_ */
