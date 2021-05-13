#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <leds.h>

#include <displacement.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>

#define TRESHOLD_SENSOR     100 //defini experimentalement
#define ON				    1
#define OFF				    0

void obstacle_detection (void);
static bool obstacle_detected = false;

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static uint16_t nearest_sensor_index = 0;
static unsigned int proximity_sensor[8];

// ********** thread function *********
static THD_WORKING_AREA(waObstacleDetection, 256);
static THD_FUNCTION(ObstacleDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;



    while(1)
        {
    		obstacle_detection();
    		 chThdSleepMilliseconds(45);
        }

}

void ObstacleDetection_start(void)
{
	chThdCreateStatic(waObstacleDetection, sizeof(waObstacleDetection), NORMALPRIO, ObstacleDetection, NULL);
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	proximity_start();

}

int16_t get_nearest_sensor(void)
{
	return nearest_sensor_index;
}

bool get_obstacle_detected(void)
{
	return obstacle_detected;
}


void obstacle_detection (void)
{

	int obst_det = 0;
	int nearest_sensor = 0;

	for(int i = 0; i < 2; i++)
	{
		proximity_sensor[i] = get_prox(i);

		if(abs(proximity_sensor[i]) > TRESHOLD_SENSOR)
		{
			obst_det = true;
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
		//set_front_led(OFF);
		nearest_sensor_index = -1;
	}
	else
	{
		obstacle_detected = true;
		//return nearest_sensor_index;
	}
}


