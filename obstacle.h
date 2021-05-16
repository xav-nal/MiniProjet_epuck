/**
 * \file		obstacle.h
 * \version		Final
 * \date		2021-05-16
 * \author		Xavier Nal et Estelle Richard
 * \brief		Header module in charge of obstacle detection
 *
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_


//Function that initializes and starts the ObstacleDetction thread
 void ObstacleDetection_start(void);


//Function that returns which sensor has been activated (sensor number)
int16_t get_nearest_sensor(void);


//Function that returns if an obstacle has been detected or not
bool get_obstacle_detected(void);


#endif /* OBSTACLE_H_ */
