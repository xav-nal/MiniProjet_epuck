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
#include <audio/play_melody.h>
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


//TP2

static int16_t counter_step_right = 0;          // in [step]
static int16_t counter_step_left = 0; 		    // in [step]
static int16_t position_to_reach_right = 0;	    // in [step]
static int16_t position_to_reach_left = 0;	    // in [step]
static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;
static uint8_t state_motor = 0;
#define POSITION_CONTROL    1
#define WHEEL_PERIMETER     13 // [cm]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define POSITION_NOT_REACHED    0
#define POSITION_REACHED        1


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


enum { 	NORMAL_MODE, OBSTACLE_MODE, IDLE_MODE, SUCCESS_MODE};

static  int mode = NORMAL_MODE;

static int rotation_state = OFF;
static float angle = 0;

static bool obstacle_detected = false;
static int obst_time = 0;
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

//TP2
uint8_t motor_position_reached(void);
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);



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

    	sound_detected = get_sound();


    	int time_nosound = time - last_sound_detected ;

    	obstacle_detection();

    	if(sound_detected == true)
		{
			last_sound_detected = time;
			mode = NORMAL_MODE;
		}
    	else {
    		mode = IDLE_MODE;
    	}

    	if(obstacle_detected == false){

    		if((time - obstacle_detected_time) < 5000)
			{
				mode = OBSTACLE_MODE;
			}
    		/*if(time_nosound >= 5000)
			{
				mode = IDLE_MODE;
				//chprintf((BaseSequentialStream *) &SDU1, " no sound ");
			}*/

		} else {
			intensity =  get_intensity();

			if(intensity > INTENSITY_LIM)
			{
				mode = SUCCESS_MODE;
			}
			else
			{
				mode = OBSTACLE_MODE;
				obstacle_detected_time = time;
				normal_displacement(OFF);

			}

		}


    	switch(mode)
    	{
    					case NORMAL_MODE:
    						    clear_leds();
							angle = get_angle();
							//chprintf((BaseSequentialStream *) &SDU1, " normale mode ");
							normal_displacement(angle);
    						//displacement_translation(TRANSLATION_SPEED);
    						//normal_displacement(OFF);
							break;

    					case OBSTACLE_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " obstacle mode ");
    						clear_leds();
    						obstacle_displacement();
    						break;

    					case IDLE_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " idle mode ");
						led1 = idle_displacement(led1);
							break;

    					case SUCCESS_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " SUCCES MODE ");
    						set_body_led(ON);
    						normal_displacement(OFF);
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
	playMelodyStart();


}


// ********** intern function **********

int idle_displacement(int led1)
{
	//chprintf((BaseSequentialStream *) &SDU1, " idle displacement ");
	displacement_rotation (IDLE_ANGLE, ROTATION_SPEED);

	//motor_set_position(10,10,200,200);
	//while(motor_position_reached() != POSITION_REACHED);

	playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);

	if(led1 == false)
	{
		led1 = true;
		set_led(LED1,ON);
		set_rgb_led(LED2,0,0,200);
		set_rgb_led(LED4,0,0,200);
		set_rgb_led(LED6,0,0,200);
		set_rgb_led(LED8,0,0,200);

	}
	else
	{
		led1 = false;
		set_led(LED1,OFF);

	}

	return led1;
}

void obstacle_displacement(void)
{
	systime_t time;
	time = chVTGetSystemTime();

	//find the nearest obstacle

	if((time - obstacle_detected_time) < 250)
	{
		displacement_rotation(100, ROTATION_SPEED);
	}
	else if(((time - obstacle_detected_time) < 1500) && ((time - obstacle_detected_time) >= 250))
	{
		displacement_translation(100);
	}
	else
	{
		displacement_translation(OFF);
		displacement_rotation(OFF,OFF);
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
			set_front_led(ON);

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
			set_front_led(ON);

			if(abs(proximity_sensor[i]) > nearest_sensor)
			{
				nearest_sensor = proximity_sensor[i];
				nearest_sensor_index = i;
			}
		}
	}

	if(obst_det == false) {
		obstacle_detected = false;
		set_front_led(OFF);
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

//from TP2

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	//reinit global variable
	counter_step_left = 0;
	counter_step_right = 0;

    position_right_reached = 0;
    position_left_reached = 0;

	//Set global variable with position to reach in step
	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = -position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	left_motor_set_speed(speed_r);
	right_motor_set_speed(speed_l);

	//flag for position control, will erase flag for speed control only
	state_motor = POSITION_CONTROL;
}

uint8_t motor_position_reached(void)
{
    if(state_motor == POSITION_CONTROL && position_right_reached && position_left_reached){
        return POSITION_REACHED;
    }else{
        return POSITION_NOT_REACHED;
}
}

