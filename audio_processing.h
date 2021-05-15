/*
 * From TP5
 * We have kept and modified the processAudio
 * and sound_remote functions of the TP5.
 * Other functions have been added
 *
 */

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024


/*
 * The function calculates the angle of rotation
 * that the robot must perform and the intensity
 * of the sound. It uses the FFT, the amplitude
 * calculation and the phase difference calculation.
 */
void processAudio(int16_t *data, uint16_t num_samples);

//Function that returns the angle of rotation;
float get_angle(void);

//Function that returns if sound is detected or not;
bool get_sound(void);

//Function that returns the intensity value;
int get_intensity(void);



#endif /* AUDIO_PROCESSING_H */
