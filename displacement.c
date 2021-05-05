#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <audio_processing.h>

#include <main.h>
#include<displacement.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <leds.h>
#include<msgbus/messagebus.h>


#define ANGLE_MIN           0.1 //radian
#define DISTANCE_LIM        3 //cm
#define TRESHOLD_SENSOR     5 //---- a definir experimentalement? ----
#define ON				    1
#define OFF				    0
#define RIGHT				2
#define LEFT				3


//MUTEX_DECL(serialMtx);

enum { 	NORMAL_MODE, OBSTACLE_MODE};

int mode = NORMAL_MODE;

static int rotation_state = OFF;

static bool obstacle_detected = false;

float angle = 0;
int distance = 100;

unsigned int proximity_sensor[8];
unsigned int obstacle[8];
int obstacle_rotation[8] = {-73,-45, 0, 0, 0, 0, 45, 73};
int obstacle_translation[8] = {OFF, OFF, ON, OFF, OFF, ON, OFF, OFF};

int old_obstacle = false;

void obstacle_detection (void);
void obstacle_displacement(void);
void normal_displacement(void);
//void displacement_rotation (int angle_value);
void displacement_translation (int distance);
void rotation_movement(bool state,int direction);
void translation_movement(bool state);

//MUTEX_DECL(serialMtx);

// ********** thread function *********
static THD_WORKING_AREA(waDisplacement, 256);
static THD_FUNCTION(Displacement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //chMtxLock(&serialMtx);
    systime_t time;

    time = chVTGetSystemTime();



    //chMtxUnlock(&serialMtx);
    //chBSemSignal(&audio_ready_sem);

    while(1)
    {
    	time = chVTGetSystemTime();


		angle = get_angle();
		//chprintf((BaseSequentialStream *) &SDU1, " ANGLE %f ", angle);

	if(angle != 0){

		if(abs(angle) < ANGLE_MIN)
		{

			displacement_translation(10);
		}
		else
		{
			displacement_translation(0);
			displacement_rotation (angle);
		}

	}
	else {
		displacement_rotation(0);
		displacement_translation(0);
	}

		//wake up in 50ms
		//chThdSleepUntilWindowed(time, time + MS2ST(200));
	    chThdSleepMilliseconds(200);

   }

}

// ********** public function *********
void displacement_start(void)
{

	chThdCreateStatic(waDisplacement, sizeof(waDisplacement), NORMALPRIO, Displacement, NULL);
	//messagebus_init(&bus, &bus_lock, &bus_condvar);
	//proximity_start();


}
void displacement_test(void)
{
	chprintf((BaseSequentialStream *) &SDU1, " test deplacement");
	return;

}

// ********** intern function **********
void obstacle_displacement(void)
{
	int obstacle_num = 0;

	//find the nearest obstacle
	for(int i = 0; i < 8; i++)
	{
		if((obstacle[i] == true) && (proximity_sensor[i] > proximity_sensor[obstacle_num]))
		{
			obstacle_num = i;
		}
	}

	//doing the displacement corresponds to the obstacle

	//rotation
	angle = obstacle_rotation[obstacle_num];
	displacement_rotation(angle);

	//translation
	//displacement_translation()


}

void obstacle_detection (void)
{


	for(int i = 0; i < 8; i++)
	{
		proximity_sensor[i] = get_prox(i);

		if(proximity_sensor[i] < TRESHOLD_SENSOR)
		{
			obstacle[i] = true;

			if(obstacle_rotation[i] != 	false) obstacle_detected = true;
		}
		else
			obstacle[i] = false;
	}

	for(int i = 0; i < 8 ; i++)
	{
		if(obstacle[i] == true)
		{
			set_led(LED7, ON);
		}
	}


	if(obstacle_detected)
	{
		mode = OBSTACLE_MODE;
	}
	else mode = NORMAL_MODE;

}

void normal_displacement(void)
{
	//get_phase();
	//get_intensity


	displacement_rotation (angle);
	displacement_translation (distance);

	//---- while waiting the functions get, we put:  (delete later) ----
	angle--;
	distance -= 5;
}

void displacement_rotation (float angle_value){

	float angle_abs_value = abs(angle_value);

	if(angle_abs_value > ANGLE_MIN)
	{
		if(angle_value > 0)
		{
			rotation_movement(ON,RIGHT);
			//chprintf((BaseSequentialStream *) &SDU1, " Rotation ");
		}
		else
		{
			rotation_movement(ON,LEFT);
			//chprintf((BaseSequentialStream *) &SDU1, " Rotation ");
		}

	}
	else if ((angle_abs_value <= ANGLE_MIN) && (rotation_state == ON))
	{
		rotation_movement(OFF,OFF);
		//chprintf((BaseSequentialStream *) &SDU1, " Rotation OFF ");
	}
	else return;

}

void displacement_translation (int distance_value)
{
	if(distance_value != false)
	{
		//chprintf((BaseSequentialStream *) &SDU1, " Translation ");
		translation_movement(ON);
	}
	else{
		translation_movement(OFF);
		//chprintf((BaseSequentialStream *) &SDU1, " Translation OFF ");

	}
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



