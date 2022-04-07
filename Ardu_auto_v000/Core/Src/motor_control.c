/*
 * motor_control.c
 *
 *  Created on: Apr 3, 2022
 *      Author: samul
 */

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
					HAL_GPIO_WritePin(ENB_GBIO_PORT, ENB_Pin, GPIO_PIN_RESET);
				case MT_FAST_STOP:
					HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, IN3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE, IN4_Pin, GPIO_PIN_SET);
		}
	}
}

void handle_driving(cmd_holder cmd){
	uint8_t old = cmd->old_cmd;
	uint8_t new = cmd->new_cmd;
	uint8_t moving = cmd->moving;
	if (new != old){
		switch(new){
		    case FORWARD:
		    	motor_set(LEFT_MT, MT_FORWARD);
		    	motor_set(RIGHT_MT, MT_FORWARD);
		    	cmd->moving = MOVING_FORWARD;

		    case REVERSE:
				motor_set(LEFT_MT, MT_REVERSE);
				motor_set(RIGHT_MT, MT_REVERSE);
				cmd->moving = MOVING_REVERSE;

		    case STOP:
		    	motor_set(LEFT_MT, MT_STOP);
				motor_set(RIGHT_MT, MT_STOP);
				cmd->moving = STOPPED;

		    case FAST_STOP:
		    	motor_set(LEFT_MT, MT_FAST_STOP);
		    	motor_set(RIGHT_MT, MT_FAST_STOP);
		    	cmd->moving = STOPPED;

			case TURN_RIGHT:
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

			case TURN_LEFT:
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

		}
	}
	else
		cmd->new_cmd = cmd->old_cmd;
}

