#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024


void audio_test(void);

void processAudio(int16_t *data, uint16_t num_samples);
float get_angle(void);
bool get_sound(void);



#endif /* AUDIO_PROCESSING_H */
