#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include<displacement.h>
#include <motors.h>

#define ANGLE_MIN           5 //degree
#define INTENSITY_LIM     200 //UNITY ?
#define ON				    1
#define OFF				    0
#define RIGHT				2
#define LEFT				3

int rotation_state = OFF;
int translation_state = OFF;

int phase = 0;
int intensity = 100;


// ********** thread function *********
/*static THD_WORKING_AREA(waDisplacement, 256);
static THD_FUNCTION(Displacement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    time = chVTGetSystemTime();

    //get_phase();
    //get_intensity


    displacement_rotation (phase);
    displacement_translation (intensity);

    //while waiting the functions get, we put:  (delete later)
    phase--;
    intensity += 25;

    //wake up in 200ms
    chThdSleepUntilWindowed(time, time + MS2ST(200));

}
*/

// ********** public function *********
/*void displacement_start(void)
{
	chThdCreateStatic(waDisplacement, sizeof(waDisplacement), NORMALPRIO, Displacement, NULL);

	//while waiting the functions get, we put:  (delete later)
	    phase = 180;
	    intensity = 100;
}

*/

// ********** intern function **********
void displacement_rotation (int angle_value){

	int angle_abs_value = abs(angle_value);

	if(angle_abs_value > ANGLE_MIN)
	{
		if(angle_value > 0)
		{
			rotation_movement(ON,RIGHT);
		}
		else
		{
			rotation_movement(ON,LEFT);
		}
		rotation_state = ON;
		translation_state = OFF;
	}
	else if ((angle_abs_value <= ANGLE_MIN) && (rotation_state == ON))
	{
		rotation_movement(OFF, OFF);

		rotation_state = OFF;
		translation_state = ON;
	}
	else return;

}

void displacement_translation (int intensity_value)
{

	if((intensity_value < INTENSITY_LIM ) && translation_state)
	{
		translation_movement(ON);
	}
	else if ((intensity_value >= INTENSITY_LIM) && translation_state)
	{
		translation_movement(OFF); // normalement pas besoins de plus d arguments � verifier experimentalement
		translation_state = OFF;
	}
	else return;

}

void rotation_movement(bool state,int direction)
{
	if(state == ON)
	{
		if(direction == RIGHT)
		{
			left_motor_set_speed(-ROTATION_SPEED);
			right_motor_set_speed(ROTATION_SPEED);
		}
		else
		{
			left_motor_set_speed(ROTATION_SPEED);
			right_motor_set_speed(-ROTATION_SPEED);
		}

	}else
	{
		left_motor_set_speed(OFF);
		right_motor_set_speed(OFF);
	}

}

void translation_movement(bool state)
{
	if(state == ON)
	{
		left_motor_set_speed(TRANSLATION_SPEED);
		right_motor_set_speed(TRANSLATION_SPEED);
	}
	else
	{
		left_motor_set_speed(OFF);
		right_motor_set_speed(OFF);
	}
}
