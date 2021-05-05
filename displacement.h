#ifndef DISPLACEMENT_H
#define DISPLACEMENT_H


#define ROTATION_SPEED      600 // speed robot in rotation [step/s]
#define TRANSLATION_SPEED   700 // speed robot in translation  [step/s]
#define RAYON_ROBOT          53 // distance between the 2 wheels [mm], measurement not very precise



void displacement_start(void);

void displacement_test(void);
void displacement_rotation (float angle_value);
void displacement_translation (int distance_value);


#endif /* DIPLACEMENT_H */
