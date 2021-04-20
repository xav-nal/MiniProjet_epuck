#ifndef DISPLACEMENT_H
#define DISPLACEMENT_H


#define ROTATION_SPEED   600 // vitesse du robot en rotation [step/s]
#define RAYON_ROBOT       53 // distance entre les 2 roues [mm], mesure pas très précise

#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     130 // [mm]


void displacement_rotation (int angle_value);

#endif /* DIPLACEMENT_H */
