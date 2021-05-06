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
#define TRESHOLD_SENSOR     100 //---- a definir experimentalement? ---- // plus on se rapproche plus la valeur du sensor augmente
#define ON				    1
#define OFF				    0
#define RIGHT				2
#define LEFT				    3
#define KP                  180.0f
#define KI                  1.8f
#define MAX_SUM_ERROR 	   (MOTOR_SPEED_LIMIT/10*KI)
#define ANGLE_MIN_PID       0.5

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


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
void normal_displacement(float angle);
//void displacement_rotation (int angle_value);
void displacement_translation (int distance);
void rotation_movement(bool state,int direction);
void translation_movement(bool state);
int16_t pid_regulator(float error);



// ********** thread function *********
static THD_WORKING_AREA(waDisplacement, 256);
static THD_FUNCTION(Displacement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    time = chVTGetSystemTime();

    while(1)
    {
    	time = chVTGetSystemTime();

    	obstacle_detection();

    	if(obstacle_detected == false){

    		angle = get_angle();
    		//chprintf((BaseSequentialStream *) &SDU1, " ANGLE %f ", angle);

    		normal_displacement(angle);

    	}else{

    		normal_displacement(OFF);

    	}

		//wake up in 50ms
		//chThdSleepUntilWindowed(time, time + MS2ST(200));
	    chThdSleepMilliseconds(50);

   }
}

// ********** public function *********
void displacement_start(void)
{

	chThdCreateStatic(waDisplacement, sizeof(waDisplacement), NORMALPRIO, Displacement, NULL);
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();


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

	int obst_det = 0;

	for(int i = 0; i < 8; i++)
	{
		proximity_sensor[i] = get_prox(i);

		if(proximity_sensor[i] > TRESHOLD_SENSOR)
		{
			obst_det = 1;
			//obstacle[i] = true;
			obstacle_detected = true;

			//if(obstacle_rotation[i] != 	false) obstacle_detected = true;
		}
		//else
			//obstacle[i] = false;
	}

	if(obst_det == 0) {
		obstacle_detected = false;
	}

	/*for(int i = 0; i < 8 ; i++)
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
	else mode = NORMAL_MODE;*/

}

void normal_displacement(float angle_value)
{
	if(angle_value != 0){

		if(abs(angle_value) < ANGLE_MIN)
		{
			//chprintf((BaseSequentialStream *) &SDU1, " rentré = ");
			displacement_translation(ON);
		}
		else
		{
			displacement_translation(OFF);
			displacement_rotation (angle_value);
		}

	}
	else {
		displacement_rotation(OFF);
		displacement_translation(OFF);
	}
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
	if(distance_value != OFF)
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
			//left_motor_set_speed(-ROTATION_SPEED);
			//right_motor_set_speed(ROTATION_SPEED);
			left_motor_set_speed(-pid_regulator(angle));
			right_motor_set_speed(pid_regulator(angle));
		}
		else
		{
			//left_motor_set_speed(ROTATION_SPEED);
			//right_motor_set_speed(-ROTATION_SPEED);
			left_motor_set_speed(pid_regulator(angle));
			right_motor_set_speed(-pid_regulator(angle));
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


int16_t pid_regulator(float error){

	error = fabs(error);

	float speed = 0;

	static float sum_error = 0;

	if(fabs(error) < ANGLE_MIN_PID){
		return 0;
	}

	sum_error += error;

	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP*error + KI*sum_error;

	return speed;

}

