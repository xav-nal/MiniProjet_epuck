#ifndef DISPLACEMENT_H
#define DISPLACEMENT_H


#define ROTATION_SPEED      600 // speed robot in rotation [step/s]
#define TRANSLATION_SPEED   700 // speed robot in translation  [step/s]
#define RAYON_ROBOT          53 // distance between the 2 wheels [mm], measurement not very precise

//#define MOTOR_SPEED_LIMIT     13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN      4 //number of steps to do 1 electrical turn
#define NB_OF_PHASES           4 //number of phases of the motors
#define WHEEL_PERIMETER      130 // [mm]

void displacement_start(void);

void displacement_rotation (int angle_value);
void displacement_translation (int intensity);
void rotation_movement(bool state,int direction);
void translation_movement(bool state);

#endif /* DIPLACEMENT_H */
