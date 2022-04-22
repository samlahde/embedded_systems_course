/*
 * self_driving.c
 *
 *  Created on: 14 Apr 2022
 *      Author: samul
 */

#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <L3G4200D.h>
#include <string.h>
#include "self_driving.h"
#include "bt_control.h"
#include "motor_control.h"


void self_driving(Cmd_holder cmd_holder, uint32_t * IR_data){
    uint8_t command;
    IR_data_type IR_output_data = get_ir(IR_data);
    if(IR_output_data.LEFT && !IR_output_data.FRONT && !IR_output_data.RIGHT){
        command = CMD_TURN_RIGHT_FWD;
    }
    else if(!IR_output_data.LEFT && !IR_output_data.FRONT && IR_output_data.RIGHT){
        command = CMD_TURN_LEFT_FWD;
    }
    else if(IR_output_data.FRONT){
        if(!IR_output_data.LEFT){
            command = CMD_TURN_LEFT_STOPPED;
        }
        else {
            command = CMD_TURN_RIGHT_STOPPED;
        }
    }
    else{
        command = CMD_FORWARD;
    }
    cmd_holder->new_cmd = command;

}

IR_data_type get_ir(uint32_t * IR_data)
{
	IR_data_type IR_output_data;
	if(IR_data[2] < 4000) IR_output_data.LEFT = true;
	else IR_output_data.LEFT = false;
	if(IR_data[3] < 4000) IR_output_data.RIGHT = true;
	else IR_output_data.RIGHT = false;
	if(IR_data[4] < 4000) IR_output_data.FRONT = true;
	else IR_output_data.FRONT = false;
	return IR_output_data;
}

