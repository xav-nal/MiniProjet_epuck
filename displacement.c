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
#include <leds.h>
#include <audio/play_melody.h>
#include <obstacle.h>
/*
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
*/


#define ANGLE_MIN           0.1 //radian
#define DISTANCE_LIM        3   //cm
#define ROTATION_SPEED      600 // speed robot in rotation [step/s]
#define TRANSLATION_SPEED   700 // speed robot in translation  [step/s]

#define TRESHOLD_SENSOR     100 //defini experimentalement
#define TIME_LIM			5   //seconde
#define IDLE_ANGLE          2
#define ON				    1
#define OFF				    0
#define RIGHT				2
#define LEFT			    3
#define KP                  180.0f
#define KI                  1.8f
#define MAX_SUM_ERROR 	   (MOTOR_SPEED_LIMIT/10*KI)
#define ANGLE_MIN_PID       0.5
#define INTENSITY_LIM		600000
#define OBSTACLE_ANGLE		100
#define OBSTACLE_ROT_RIGHT  100,ROTATION_SPEED
#define OBSTACLE_ROT_LEFT  -100,ROTATION_SPEED

#define DARK_BLUE           0,0,200

#define ROTATION_OFF		OFF,OFF
#define ROT_MVT_OFF 		OFF,OFF,OFF

#define TIME_MODE_OBST			1500
#define OBST_ROT_LIM			250


#define IDLE_FIRST_MVT_LIM		4000 //time ms
#define IDLE_SND_MVT_LIM		5790
#define IDLE_THD_MVT_LIM		9790
#define IDLE_FRTH_MVT_LIM		11580

#define IDLE_SPEED_ROT_LEFT 	624
#define IDLE_SPEED_ROT_RIGHT	300

#define TIME_NOSOUND_LIM		5000

#define IR_ONE					0
#define IR_TWO					1




/*messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);*/


enum { 	NORMAL_MODE, OBSTACLE_MODE, IDLE_MODE, SUCCESS_MODE};
static  int mode = NORMAL_MODE;

//static int rotation_state = OFF;
static float angle = 0;

static bool obstacle_detected = false;

static int obstacle_detected_time = 0;
static int obstacle_direction = 0;


static systime_t last_sound_detected = 0 ;

static systime_t time_idle;
static systime_t idle_time_loop = 0;

//intern function
int16_t obstacle_detection (void);
void mode_management(bool sound_detected_value, int time_nosound_value, int intensity_value, systime_t time, uint16_t nearest_sensor);
void obstacle_displacement(bool sound_detected);
void normal_displacement(float angle);
void displacement_rotation (float angle_value,int speed);
void displacement_translation (int distance);
void rotation_movement(bool state,int direction,int speed);
void translation_movement(bool state);
int16_t pid_regulator(float error);
int idle_displacement(int led1);
void idle_8_mouvement(systime_t time);
void idle_basic_mouvement(systime_t time);




// ********** thread function *********
static THD_WORKING_AREA(waDisplacement, 256);
static THD_FUNCTION(Displacement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int led1 = false;
    int intensity = 0;
    int time_nosound = 0;
    bool sound_detected = false;
    uint16_t nearest_sensor = 0;


    //time = chVTGetSystemTime();


    while(1)
    {
    	time = chVTGetSystemTime();

    	time_nosound = time - last_sound_detected ;

    	sound_detected = get_sound();

    	//nearest_sensor = obstacle_detection();

    	obstacle_detected = get_obstacle_detected();
    	nearest_sensor = get_nearest_sensor();


    	mode_management(sound_detected, time_nosound, intensity, time, nearest_sensor);


    	switch(mode)
    	{
    					case NORMAL_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " NORMAL MODE ");
    						clear_leds();
							angle = get_angle();
							normal_displacement(angle);
    						break;

    					case OBSTACLE_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " OBSTACLE MODE ");
    						clear_leds();
    						obstacle_displacement(sound_detected);
    						break;

    					case IDLE_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " IDLE MODE ");
    						led1 = idle_displacement(led1);
							break;

    					case SUCCESS_MODE:
    						//chprintf((BaseSequentialStream *) &SDU1, " SUCCES MODE ");
    						set_body_led(ON);
    						playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
    						normal_displacement(OFF);
    						break;

    					default:
							normal_displacement(OFF);
							break;

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
	//messagebus_init(&bus, &bus_lock, &bus_condvar);
	//proximity_start();
	playMelodyStart();
}


// ********** intern function **********
void mode_management(bool sound_detected_value, int time_nosound_value, int intensity_value, systime_t time, uint16_t nearest_sensor)
{
	time = chVTGetSystemTime();

	if(time_nosound_value >= TIME_NOSOUND_LIM)
	{
		if(mode != IDLE_MODE){
			idle_time_loop = time;
		}
		mode = IDLE_MODE;
	}

	if(sound_detected_value == true)
	{
		last_sound_detected = time;
		mode = NORMAL_MODE;
	}

	if(obstacle_detected == false){

		if((time - obstacle_detected_time) < TIME_MODE_OBST	)
		{
			mode = OBSTACLE_MODE;
		}

	} else {
		intensity_value =  get_intensity();

		if(intensity_value > INTENSITY_LIM)
		{
			mode = SUCCESS_MODE;
		}
		else
		{
			mode = OBSTACLE_MODE;
			obstacle_detected_time = time;
			normal_displacement(OFF);

			//set_front_led(ON);

			if((nearest_sensor == IR_ONE) || (nearest_sensor == IR_TWO))
				obstacle_direction = RIGHT;
			else
				obstacle_direction = LEFT;
		}
	}
}



void obstacle_displacement(bool sound_detected)
{
	if(sound_detected)
	{
		systime_t time;
		time = chVTGetSystemTime();

		if((time - obstacle_detected_time) < OBST_ROT_LIM)
		{

			if(obstacle_direction == RIGHT )
				displacement_rotation(OBSTACLE_ROT_RIGHT);
			else
				displacement_rotation(OBSTACLE_ROT_LEFT);

		}
		else
			displacement_translation(TRANSLATION_SPEED);
	}
	else
		normal_displacement(OFF);
}


void normal_displacement(float angle_value)
{
	if(angle_value != OFF)
	{

		if(abs(angle_value) < ANGLE_MIN)
		{
			displacement_translation(ON);
		}
		else
		{
			displacement_translation(OFF);
			displacement_rotation (angle_value,pid_regulator(angle));
		}

	}
	else {
		displacement_rotation(ROTATION_OFF);
		displacement_translation(OFF);
	}
}

void displacement_rotation (float angle_value,int speed){

	float angle_abs_value = abs(angle_value);

	if(angle_abs_value > ANGLE_MIN)
	{
		if(angle_value > ZERO)
			rotation_movement(ON,RIGHT,speed);
		else
			rotation_movement(ON,LEFT,speed);

	}
	else if (angle_abs_value <= ANGLE_MIN)
	{
		rotation_movement(ROT_MVT_OFF);
	}
	else
		return;
}

void displacement_translation (int distance_value)
{
	if(distance_value != OFF)
		translation_movement(ON);
	else
		translation_movement(OFF);
}

void rotation_movement(bool state, int direction,int speed)
{
	if(state == ON)
	{
		if(direction == RIGHT)
		{
			left_motor_set_speed(-speed);
			right_motor_set_speed(speed);
		}
		else
		{
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

int16_t pid_regulator(float error){

	error = fabs(error);

	float speed = 0;

	static float sum_error = 0;

	if(fabs(error) < ANGLE_MIN_PID)
		return 0;

	sum_error += error;

	if(sum_error > MAX_SUM_ERROR)
		sum_error = MAX_SUM_ERROR;
	else if(sum_error < -MAX_SUM_ERROR)
		sum_error = -MAX_SUM_ERROR;

	speed = KP*error + KI*sum_error;

	return speed;

}
int idle_displacement(int led1)
{

	time_idle = chVTGetSystemTime();

	idle_basic_mouvement(time_idle);

	if(led1 == false)
	{
		led1 = true;
		set_led(LED1,ON);
		set_rgb_led(LED2,DARK_BLUE);
		set_rgb_led(LED4,DARK_BLUE);
		set_rgb_led(LED6,DARK_BLUE);
		set_rgb_led(LED8,DARK_BLUE);
	}
	else
	{
		led1 = false;
		set_led(LED1,OFF);
	}

	return led1;
}

//trajet aller-retour du robot en continue avec demi_tour
void idle_basic_mouvement(systime_t time){
	if((time - idle_time_loop) < IDLE_FIRST_MVT_LIM)
	{
		displacement_translation(OFF);
	    left_motor_set_speed(IDLE_SPEED_ROT_LEFT);
	    right_motor_set_speed(IDLE_SPEED_ROT_RIGHT);
	}
	else if(((time - idle_time_loop) < IDLE_SND_MVT_LIM) && ((time - idle_time_loop) >= IDLE_FIRST_MVT_LIM))
	{
		displacement_translation(ON);
	}
	else if(((time - idle_time_loop) < IDLE_THD_MVT_LIM) && ((time - idle_time_loop) >= IDLE_SND_MVT_LIM))
	{
		displacement_translation(OFF);
		left_motor_set_speed(IDLE_SPEED_ROT_LEFT);
		right_motor_set_speed(IDLE_SPEED_ROT_RIGHT);
	}
	else if(((time - idle_time_loop) < IDLE_FRTH_MVT_LIM) && ((time - idle_time_loop) >= IDLE_THD_MVT_LIM))
	{
		displacement_translation(ON);
	}
	else
		idle_time_loop = time;

}

void initialisation_leds(void)
{
	clear_leds();
}
//trajet en forme de 8
/*
void idle_8_mouvement(systime_t time){

	if((time - idle_time) < 5960)
	{
		displacement_translation(OFF);
	    left_motor_set_speed(620);
	    right_motor_set_speed(300);
	}
	else if(((time - idle_time) < 7750) && ((time - idle_time) >= 5960))
	{
		displacement_translation(ON);
	}
	else if(((time - idle_time) < 13710) && ((time - idle_time) >= 7750))
	{
		displacement_translation(OFF);
		left_motor_set_speed(300);
		right_motor_set_speed(620);
	}
	else if(((time - idle_time) < 15500) && ((time - idle_time) >= 13710))
    {
		displacement_translation(ON);
	}
	else
		idle_time = time;
}
*/
