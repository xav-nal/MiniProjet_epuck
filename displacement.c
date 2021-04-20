#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <main.h>
#include<displacement.h>
#include <motors.h>

#define ANGLE_MIN         5 //degree
#define ON				  1
#define OFF				  0

int rotation_state = OFF;


void displacement_rotation (int angle_value){


	if(angle_value > ANGLE_MIN)
	{
		left_motor_set_speed(ROTATION_SPEED);
		right_motor_set_speed(-ROTATION_SPEED);
		rotation_state = ON;
	}
	else if ((angle_value <= ANGLE_MIN) && (rotation_state == ON))
	{
		left_motor_set_speed(OFF);
		right_motor_set_speed(OFF);
		rotation_state = OFF;
	}
	else{
		return;
	}
}

//code mauvaise facon de faire en dessous
/*
void displacement_process (distance_step){

	//  starting position
	int32_t left_motor_pos_start = left_motor_get_pos();
	int32_t right_motor_pos_start = right_motor_get_pos();
	int32_t left_motor_pos_inprogress = left_motor_pos_start ;
	int32_t right_motor_pos_inprogress = right_motor_pos_start;


	// setting the motors speed
	if((left_motor_pos_inprogress != (left_motor_pos_start + distance_step)) && (right_motor_pos_inprogress != (right_motor_pos_start + distance_step)) )
	{
		left_motor_set_speed(ROTATION_SPEED);
		right_motor_set_speed(-ROTATION_SPEED);
	}

	//end the rotation
	if((left_motor_pos_inprogress != (left_motor_pos_start + distance_step)) && (right_motor_pos_inprogress != (right_motor_pos_start + distance_step)) )
	{
		left_motor_pos_inprogress = left_motor_get_pos();
		right_motor_pos_inprogress = right_motor_get_pos();
	}



}
*/
