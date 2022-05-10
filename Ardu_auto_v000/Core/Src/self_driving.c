/*
 * self_driving.c
 *
 *  Created on: 14 Apr 2022
 *      Author: samul
 */

#include "main.h"
#include <stdio.h>
#include <L3G4200D.h>
#include <string.h>
#include "self_driving.h"
#include "bt_control.h"
#include "motor_control.h"

/*
void self_driving(Cmd_holder cmd_holder){
    uint8_t command;
    bool IR_left = get_ir(IR_LEFT);
    bool IR_right = get_ir(IR_RIGHT);
    bool IR_front = get_ir(IR_FRONT);
    if(IR_left && !IR_front && !IR_right){
        command = CMD_TURN_RIGHT_FWD;
    }
    else if(!IR_left && !IR_front && IR_right){
        command = CMD_TURN_LEFT_FWD;
    }
    else if(IR_front){
        if(!IR_left){
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
*/
