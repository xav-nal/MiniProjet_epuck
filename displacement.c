#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <audio_processing.h>
#include <displacement.h>
#include <motors.h>
#include <leds.h>
#include <audio/play_melody.h>
#include <obstacle.h>

#define SLEEP_TIME_THREAD	50//ms

#define ANGLE_MIN           0.1 //radian
#define ANGLE_START_TRANS	0.5 //radian
#define DISTANCE_LIM        3   //cm
#define ROTATION_SPEED      600 // speed robot in rotation [step/s]
#define TRANSLATION_SPEED   700 // speed robot in translation  [step/s]

#define TIME_LIM			5   //seconde
#define IDLE_ANGLE          2
#define ON				    1
#define OFF				    0
#define RIGHT				2
#define LEFT			    3
#define KP                  180.0f
#define KI                  1.8f
#define MAX_SUM_ERROR 	   (MOTOR_SPEED_LIMIT/10*KI)
#define ANGLE_MIN_PI        0.5
#define INTENSITY_LIM		250000

#define OBSTACLE_ANGLE		100
#define OBSTACLE_ROT_RIGHT  100,ROTATION_SPEED
#define OBSTACLE_ROT_LEFT  -100,ROTATION_SPEED
#define TIME_MODE_OBST			900
#define OBST_ROT_LIM			500

#define DARK_BLUE          0,0,200
#define DARK_GREEN         0,200,0
#define DARK_YELLOW		  255,255,0

#define ROTATION_OFF		OFF,OFF
#define ROT_MVT_OFF 		OFF,OFF,OFF

#define IDLE_FIRST_MVT_LIM		4000 //time ms
#define IDLE_SND_MVT_LIM		5790
#define IDLE_THD_MVT_LIM		9790
#define IDLE_FRTH_MVT_LIM		11580

#define IDLE_SPEED_ROT_LEFT 	624
#define IDLE_SPEED_ROT_RIGHT	300

#define TIME_NOSOUND_LIM		5000

#define IR_ONE					0
#define IR_TWO					1

enum { 	NORMAL_MODE, OBSTACLE_MODE, IDLE_MODE, SUCCESS_MODE};


//intern function
int  mode_management(int mode, bool sound_detected_value );
void obstacle_displacement(bool sound_detected);
void normal_displacement(float angle);
void displacement_rotation (float angle_value,int speed);
void displacement_translation (int distance);
void rotation_movement(bool state,int direction,int speed);
void translation_movement(bool state);
int16_t pi_regulator(float error);
void idle_displacement(void);
void idle_basic_mouvement(systime_t time);
void initialisation_leds(void);
void success_animation(void);
void normale_animation(void);

static int obstacle_direction = 0;
static bool obstacle_detected = false;

static systime_t obstacle_detected_time = 0;
static systime_t last_sound_detected = 0 ;
static systime_t idle_time_loop = 0;

// ********** thread function *********
static THD_WORKING_AREA(waDisplacement, 256);
static THD_FUNCTION(Displacement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    int mode = NORMAL_MODE;
    float angle = 0;
    bool sound_detected = false;


    while(1)
    {

    	sound_detected = get_sound();

    	mode = mode_management(mode, sound_detected);

    	switch(mode)
    	{
    					case NORMAL_MODE:
    						initialisation_leds();
    						normale_animation();
							angle = get_angle();
							normal_displacement(angle);
    						break;

    					case OBSTACLE_MODE:
    						initialisation_leds();
    						obstacle_displacement(sound_detected);
    						break;

    					case IDLE_MODE:
    						initialisation_leds();
    						idle_displacement();
							break;

    					case SUCCESS_MODE:
    						initialisation_leds();
    						set_body_led(ON);
    						success_animation();
    						playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
    						normal_displacement(OFF);
    						break;

    					default:
							normal_displacement(OFF);
							break;

    	}

		//wake up in 50ms
	    chThdSleepMilliseconds(SLEEP_TIME_THREAD);

   }
}

// ********** public function *********
void displacement_start(void)
{
	chThdCreateStatic(waDisplacement, sizeof(waDisplacement), NORMALPRIO, Displacement, NULL);
	playMelodyStart();
}


// ********** intern function **********
int mode_management(int mode, bool sound_detected_value )
{
	int intensity_value = 0;
	int old_mode = mode;
	uint16_t nearest_sensor = 0;
	systime_t time_nosound_value = 0;
	systime_t time = 0;

	time = chVTGetSystemTime();
	time_nosound_value = time - last_sound_detected ;

	obstacle_detected = get_obstacle_detected();
	nearest_sensor = get_nearest_sensor();

	if(obstacle_detected == false)
	{
		if(time_nosound_value >= TIME_NOSOUND_LIM)
		{
			if(mode != IDLE_MODE)
				idle_time_loop = time;

			mode = IDLE_MODE;
		}

		if(sound_detected_value == true)
		{
			last_sound_detected = time;
			mode = NORMAL_MODE;
		}

		if((time - obstacle_detected_time) < TIME_MODE_OBST	)//finish mvt in obst mode
		{
			mode = OBSTACLE_MODE;
		}

	}
	else
	{

		intensity_value =  get_intensity();

		if(intensity_value > INTENSITY_LIM)
		{
			mode = SUCCESS_MODE;
		}
		else
		{
			if(mode != OBSTACLE_MODE )// first time obst det
			{
				mode = OBSTACLE_MODE;
				obstacle_detected_time = time;
				normal_displacement(OFF);

				if((nearest_sensor == IR_ONE) || (nearest_sensor == IR_TWO))
					obstacle_direction = RIGHT;
				else
					obstacle_direction = LEFT;
			}
			else if(((time - obstacle_detected_time) > TIME_MODE_OBST	) && (mode == OBSTACLE_MODE))// finish obs mode but obs forever
			{
				obstacle_detected_time = time;
				normal_displacement(OFF);

				if((nearest_sensor == IR_ONE) || (nearest_sensor == IR_TWO))
					obstacle_direction = RIGHT;
				else
					obstacle_direction = LEFT;
			}
		}
	}
	if(old_mode == SUCCESS_MODE )
	{
		if(old_mode != mode)
			stopCurrentMelody();
	}
	return mode;
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
			displacement_rotation (angle_value,pi_regulator(angle_value));
		}

	}
	else
	{
		displacement_rotation(ROTATION_OFF);
		displacement_translation(OFF);
	}
}

void displacement_rotation (float angle_value,int speed)
{

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

	}
	else
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

int16_t pi_regulator(float error)
{

	error = fabs(error);

	float speed = 0;

	static float sum_error = 0;

	if(fabs(error) < ANGLE_MIN_PI)
		return 0;

	sum_error += error;

	if(sum_error > MAX_SUM_ERROR)
		sum_error = MAX_SUM_ERROR;
	else if(sum_error < -MAX_SUM_ERROR)
		sum_error = -MAX_SUM_ERROR;

	speed = KP*error + KI*sum_error;

	return speed;

}

void idle_displacement(void)
{
	systime_t time_idle;

	time_idle = chVTGetSystemTime();

	idle_basic_mouvement(time_idle);

	set_led(LED1,ON);
	set_rgb_led(LED2,DARK_BLUE);
	set_rgb_led(LED4,DARK_BLUE);
	set_rgb_led(LED6,DARK_BLUE);
	set_rgb_led(LED8,DARK_BLUE);

}

//trajet aller-retour du robot en continue avec demi_tour
void idle_basic_mouvement(systime_t time)
{
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
	set_body_led(OFF);
}

void success_animation(void)
{
	set_rgb_led(LED2,DARK_YELLOW);
	set_rgb_led(LED4,DARK_YELLOW);
	set_rgb_led(LED6,DARK_YELLOW);
	set_rgb_led(LED8,DARK_YELLOW);
}
void normale_animation(void)
{
	set_rgb_led(LED2,DARK_GREEN);
	set_rgb_led(LED4,DARK_GREEN);
	set_rgb_led(LED6,DARK_GREEN);
	set_rgb_led(LED8,DARK_GREEN);
}
