#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <math.h>
#include<displacement.h>



//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];


//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];


#define MIN_VALUE_THRESHOLD	10000 
#define WRONG_FREQ             -1

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_RESEARCH 	24 //370HZ
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_RESEARCH_L  (FREQ_RESEARCH-1)
#define FREQ_RESEARCH_G	 (FREQ_RESEARCH+1)

#define ANGLE_ADJUST  -3.98  // (pi/2)/(accumulate phase)


void calcul_angle(float im_r, float re_r, float im_l, float re_l);
int16_t sound_remote(float* data);
void regulateur(void);



static float angle_diff = 0;
static float angle_diff_old = 0;

float count_audio = 0;



/*
 * Fonction qui permet de calculer le pic le plus haut de la fréquence ;
 * elle retourne l'index correspondant à la position de cette fréquence
 * dans le buffer
 *
 */

int16_t sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//chprintf((BaseSequentialStream *) &SDU1, " maxnorme =    %d   ",max_norm_index);

	if((max_norm_index >= FREQ_RESEARCH_L) && (max_norm_index <= FREQ_RESEARCH_G) )
	{
		return max_norm_index;
	}else
		return WRONG_FREQ;


}


void processAudio(int16_t *data, uint16_t num_samples){

	/*
		*
		*	We get 160 samples per mic every 10ms
		*	So we fill the samples buffers to reach
		*	1024 samples, then we compute the FFTs.
		*
		*/

		static uint16_t nb_samples = 0;

		//loop to fill the buffers
		for(uint16_t i = 0 ; i < num_samples ; i+=4)
		{
			//construct an array of complex numbers. Put 0 to the imaginary part
			micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
			micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];


			nb_samples++;

			micRight_cmplx_input[nb_samples] = 0;
			micLeft_cmplx_input[nb_samples] = 0;


			nb_samples++;

			//stop when buffer is full
			if(nb_samples >= (2 * FFT_SIZE))
			{
				break;
			}
		}

		if(nb_samples >= (2 * FFT_SIZE))
		{
			/*	FFT proccessing
			*
			*	This FFT function stores the results in the input buffer given.
			*	This is an "In Place" function.
			*/

			doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);


			/*	Magnitude processing
			*
			*	Computes the magnitude of the complex numbers and
			*	stores them in a buffer of FFT_SIZE because it only contains
			*	real numbers.
			*
			*/
			arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

			nb_samples = 0;


			int16_t highest_pic_R = sound_remote(micRight_output);
			int16_t highest_pic_L = sound_remote(micLeft_output);




			if((highest_pic_R != WRONG_FREQ) && (highest_pic_L != WRONG_FREQ))
			{
				//chprintf((BaseSequentialStream *) &SDU1, " highest =    %d   ",highest_pic_L);
				calcul_angle(micRight_cmplx_input[2*highest_pic_R+1], micRight_cmplx_input[2*highest_pic_R],
						micLeft_cmplx_input[2*highest_pic_L+1], micLeft_cmplx_input[2*highest_pic_L]);

				angle_diff = angle_diff*ANGLE_ADJUST;

			}
			else
				angle_diff = false;

		}


		//chThdSleepMilliseconds(10);

}

void audio_test(void)
{
	chprintf((BaseSequentialStream *) &SDU1, " audio test ");
}

void calcul_angle(float im_r, float re_r, float im_l, float re_l)
{
	float angle_R = 0;
	float angle_L = 0;

	angle_R = atan2f(im_r, re_r);
	//chprintf((BaseSequentialStream *) &SDU1, " angle r =    %f rad  ",angle_R);

	angle_L = atan2f(im_l, re_l);

	angle_diff= angle_R - angle_L;
	//chprintf((BaseSequentialStream *) &SDU1, " angle =    %f rad  ",angle_diff);
}


void regulateur(void) {

	angle_diff = 0.9*angle_diff + 0.1*angle_diff_old;

	angle_diff_old = angle_diff;

}

float get_angle(void)
{
	return angle_diff;
}


