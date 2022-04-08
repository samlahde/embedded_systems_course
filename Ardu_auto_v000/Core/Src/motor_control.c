/*
 * motor_control.c
 *
 *  Created on: Apr 3, 2022
 *      Author: samul
 */

#include "main.h"
#include <stdio.h>
#include <L3G4200D.h>
#include <string.h>
#include "bt_control.h"
#include "motor_control.h"

	  /*Motor control
	   * Outputs:
	   * GPIOF, ENA_Pin = Motor 1 enable signal
	   * GPIOF, IN1_Pin = Motor 1 A signal
	   * GPIOD, IN2_Pin = Motor 1 B signal
	   * ENB_GPIO_Port, ENB_Pin = Motor 2 enable signal
	   * GPIOE, IN3_Pin = Motor 2 A signal
	   * GPIOE, IN4_Pin = Motor 2 B signal
	   *
	   * GPIO_PIN_SET = High
	   * GPIO_PIN_RESET = LOW
	   * enable high:
	   * A high, B low -> forward
	   * A low, B high -> reverse
	   * A high, B high -> fast motor stop
	   *
	   * enable low -> motor stop
	   * */

void motor_set(uint8_t motor, uint8_t command){
	switch(motor){
		case LEFT_MT:
			switch(command){
				case MT_FORWARD:
					HAL_GPIO_WritePin(GPIOF, ENA_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF, IN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, IN2_Pin, GPIO_PIN_RESET);
				case MT_REVERSE:
					HAL_GPIO_WritePin(GPIOF, ENA_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF, IN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, IN2_Pin, GPIO_PIN_SET);
				case MT_STOP:
					HAL_GPIO_WritePin(GPIOF, ENA_Pin, GPIO_PIN_RESET);
				case MT_FAST_STOP:
					HAL_GPIO_WritePin(GPIOF, ENA_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOF, IN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, IN2_Pin, GPIO_PIN_SET);
			}
		case RIGHT_MT:
			switch(command){
				case MT_FORWARD:
					HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, IN3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, IN4_Pin, GPIO_PIN_RESET);
				case MT_REVERSE:
					HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, IN3_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE, IN4_Pin, GPIO_PIN_SET);
				case MT_STOP:
					HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_RESET);
				case MT_FAST_STOP:
					HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, IN3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, IN4_Pin, GPIO_PIN_SET);
		}
	}
}

void handle_driving(Cmd_holder cmd){
	uint8_t old = cmd->old_cmd;
	uint8_t new = cmd->new_cmd;
	uint8_t moving = cmd->moving;
	if (new != old && new != CMD_NONE){
		switch(new){
		    case CMD_FORWARD:
		    	motor_set(LEFT_MT, MT_FORWARD);
		    	motor_set(RIGHT_MT, MT_FORWARD);
		    	cmd->moving = MOVING_FORWARD;

		    case CMD_REVERSE:
				motor_set(LEFT_MT, MT_REVERSE);
				motor_set(RIGHT_MT, MT_REVERSE);
				cmd->moving = MOVING_REVERSE;

		    case CMD_STOP:
		    	motor_set(LEFT_MT, MT_STOP);
				motor_set(RIGHT_MT, MT_STOP);
				cmd->moving = STOPPED;

		    case CMD_FAST_STOP:
		    	motor_set(LEFT_MT, MT_FAST_STOP);
		    	motor_set(RIGHT_MT, MT_FAST_STOP);
		    	cmd->moving = STOPPED;

			case CMD_TURN_RIGHT:
				switch(moving){
					case MOVING_FORWARD:
						motor_set(LEFT_MT, MT_FORWARD);
						motor_set(RIGHT_MT, MT_STOP);

					case MOVING_REVERSE:
						motor_set(LEFT_MT, MT_REVERSE);
						motor_set(RIGHT_MT, MT_STOP);

					case STOPPED:
						motor_set(LEFT_MT, MT_FORWARD);
						motor_set(RIGHT_MT, MT_REVERSE);
				}
				cmd->moving = moving;

			case CMD_TURN_LEFT:
				switch(moving){
					case MOVING_FORWARD:
						motor_set(LEFT_MT, MT_REVERSE);
						motor_set(RIGHT_MT, MT_STOP);

					case MOVING_REVERSE:
						motor_set(LEFT_MT, MT_FORWARD);
						motor_set(RIGHT_MT, MT_STOP);

					case STOPPED:
						motor_set(LEFT_MT, MT_REVERSE);
						motor_set(RIGHT_MT, MT_FORWARD);
				}
				cmd->moving = moving;
			default:
				break;
		}
		cmd->old_cmd = new;
	}
}

