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
#include <displacement.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <msgbus/messagebus.h>


#define ANGLE_MIN           0.1 //radian
#define DISTANCE_LIM        3   //cm
#define TRESHOLD_SENSOR     100 //defini experimentalement
#define TIME_LIM			5   //seconde
#define IDLE_ANGLE          2
#define ON				    1
#define OFF				    0
#define RIGHT				2
#define LEFT				    3
#define KP                  180.0f
#define KI                  1.8f
#define MAX_SUM_ERROR 	   (MOTOR_SPEED_LIMIT/10*KI)
#define ANGLE_MIN_PID       0.5
#define INTENSITY_LIM		10000

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


enum { 	NORMAL_MODE, OBSTACLE_MODE, IDLE_MODE, SUCCESS_MODE};

static  int mode = NORMAL_MODE;

static int rotation_state = OFF;
static float angle = 0;

static bool obstacle_detected = false;
static bool obst_lock = false;
static int obstacle_detected_time = 0;


static uint32_t last_sound_detected = 0 ;



unsigned int proximity_sensor[8];
unsigned int obstacle[8];
float obstacle_rotation[8] = {-1.273,-0.785, 0, 0, 0, 0, 0.785, 1.273};
int obstacle_translation[8] = {OFF, OFF, ON, OFF, OFF, ON, OFF, OFF};

int old_obstacle = false;

void obstacle_detection (void);
void obstacle_displacement(void);
void normal_displacement(float angle);
void displacement_rotation (float angle_value,int speed);
void displacement_translation (int distance);
void rotation_movement(bool state,int direction,int speed);
void translation_movement(bool state);
int16_t pid_regulator(float error);
int idle_displacement(int led1);
void success_displacement(void);



// ********** thread function *********
static THD_WORKING_AREA(waDisplacement, 256);
static THD_FUNCTION(Displacement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int led1 = false;

    systime_t time;

    time = chVTGetSystemTime();

    bool sound_detected = false;

    while(1)
    {
    	time = chVTGetSystemTime();
    	int intensity = 0;

    	//sound_detected = get_sound();


    	int time_nosound = time - last_sound_detected ;

    	obstacle_detection();

    	/*if(sound_detected == true)
		{
			last_sound_detected = time;
			mode = NORMAL_MODE;
		}*/

    	/*if(obstacle_detected == false){

    		if(time_nosound >= 5000)
			{
				mode = IDLE_MODE;
				//chprintf((BaseSequentialStream *) &SDU1, " no sound ");
			}

		}else{
			intensity =  get_intensity();

			if(intensity > INTENSITY_LIM)
			{
				mode = SUCCESS_MODE;
			}
			else
				mode = OBSTACLE_MODE;

		}*/
    	if(obst_lock == true)
		{
			obstacle_displacement();
		}


    	if(obstacle_detected == true)
    	{

    		if(obst_lock == false)
    		{
    			mode = OBSTACLE_MODE;
    			obst_lock = true;
    			obstacle_detected_time = time;
    			normal_displacement(OFF);

    		}
    	}

    	switch(mode)
    	{
    					case NORMAL_MODE:
							//angle = get_angle();
							//chprintf((BaseSequentialStream *) &SDU1, " normale mode angle %f ", angle);
							//normal_displacement(angle);
    						displacement_translation(TRANSLATION_SPEED);
    						//normal_displacement(OFF);
							break;

    					case OBSTACLE_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " obstacle mode ");
    						//normal_displacement(OFF);
    						break;



    					case IDLE_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " idle mode ");
							//led1 = idle_displacement(led1);
							break;

    					case SUCCESS_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " SUCCES MODE ");
    						success_displacement();
    						break;

    					default:
						normal_displacement(OFF);
						break;

    	}



		//wake up in 200ms
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


// ********** intern function **********
void success_displacement(void)
{
	//chprintf((BaseSequentialStream *) &SDU1, " success displacement ");
	//displacement_rotation (angle, ROTATION_SPEED);
	return;
}
int idle_displacement(int led1)
{
	//chprintf((BaseSequentialStream *) &SDU1, " idle displacement ");
	displacement_rotation (IDLE_ANGLE, ROTATION_SPEED);

	if(led1 == false)
	{
		led1 = true;
		set_led(LED1,ON);
	}
	else
	{
		set_led(LED1,OFF);
	}

	set_rgb_led(LED2, 0, 1, 0);
	set_rgb_led(LED4, 0, 0, 1);


	toggle_rgb_led(LED6, BLUE_LED, 0.5);
	toggle_rgb_led(LED8, GREEN_LED, 0.5);

	return led1;
}
void obstacle_displacement(void)
{
	systime_t time;
	time = chVTGetSystemTime();

	//find the nearest obstacle

	if((time - obstacle_detected_time) < 550)
	{
		displacement_rotation(100, ROTATION_SPEED);
	}
	else if(((time - obstacle_detected_time) < 800) && ((time - obstacle_detected_time) >= 800))
	{
		displacement_translation(100);
	}
	else
	{
		displacement_translation(OFF);
		obst_lock = false;
		mode = NORMAL_MODE;
	}




}

void obstacle_detection (void)
{
	//chprintf((BaseSequentialStream *) &SDU1, " obstacle detection mode = %d ", mode);

	int obst_det = 0;
	int nearest_sensor = 0;
	uint16_t nearest_sensor_index = 0;

	for(int i = 0; i < 2; i++)
	{
		proximity_sensor[i] = get_prox(i);

		if(abs(proximity_sensor[i]) > TRESHOLD_SENSOR)
		{
			obst_det = 1;
			obstacle_detected = true;

			if(abs(proximity_sensor[i]) > nearest_sensor)
			{
				nearest_sensor = proximity_sensor[i];
				nearest_sensor_index = i;
			}
		}
	}

	for(int i = 6; i < 8; i++)
	{
		proximity_sensor[i] = get_prox(i);

		if(abs(proximity_sensor[i]) > TRESHOLD_SENSOR)
		{
			obst_det = true;
			obstacle_detected = true;

			if(abs(proximity_sensor[i]) > nearest_sensor)
			{
				nearest_sensor = proximity_sensor[i];
				nearest_sensor_index = i;
			}
		}
	}

	if(obst_det == false) {
		obstacle_detected = false;
	}

	if(mode == OBSTACLE_MODE)
	{
		angle = obstacle_rotation[nearest_sensor_index];
		//chprintf((BaseSequentialStream *) &SDU1, " capteur %d  ",nearest_sensor_index+1);
		//chprintf((BaseSequentialStream *) &SDU1, " value %d  ",nearest_sensor);
	}

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
			displacement_rotation (angle_value,pid_regulator(angle));
		}

	}
	else {
		displacement_rotation(OFF,OFF);
		displacement_translation(OFF);
	}
}

void displacement_rotation (float angle_value,int speed){

	float angle_abs_value = abs(angle_value);

	if(angle_abs_value > ANGLE_MIN)
	{
		if(angle_value > 0)
		{
			rotation_movement(ON,RIGHT,speed);
			//chprintf((BaseSequentialStream *) &SDU1, " Rotation ");
		}
		else
		{
			rotation_movement(ON,LEFT,speed);
			//chprintf((BaseSequentialStream *) &SDU1, " Rotation ");
		}
	}
	else if ((angle_abs_value <= ANGLE_MIN) && (rotation_state == ON))
	{
		rotation_movement(OFF,OFF, OFF);
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

void rotation_movement(bool state,int direction,int speed)
{
	if(state == ON)
	{
		if(direction == RIGHT)
		{
			//left_motor_set_speed(-ROTATION_SPEED);
			//right_motor_set_speed(ROTATION_SPEED);
			left_motor_set_speed(-speed);
			right_motor_set_speed(speed);
		}
		else
		{
			//left_motor_set_speed(ROTATION_SPEED);
			//right_motor_set_speed(-ROTATION_SPEED);
			left_motor_set_speed(speed);
			right_motor_set_speed(-speed);
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

void initialisation_leds(void)
{
	clear_leds();
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

