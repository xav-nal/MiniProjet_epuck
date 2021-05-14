#include "ch.h"
#include "hal.h"
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <math.h>
#include<displacement.h>



//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
//static float micFront_cmplx_input[2 * FFT_SIZE];
//static float micBack_cmplx_input[2 * FFT_SIZE];


//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
//static float micFront_output[FFT_SIZE];
//static float micBack_output[FFT_SIZE];


#define MIN_VALUE_THRESHOLD		10000
#define WRONG_FREQ             -1

#define MIN_FREQ				10	//we don't analyze before this index to not use resources for nothing
#define FREQ_RESEARCH 			24 //370HZ
#define MAX_FREQ				30	//we don't analyze after this index to not use resources for nothing

#define FREQ_RESEARCH_L  		(FREQ_RESEARCH-1)
#define FREQ_RESEARCH_G	 		(FREQ_RESEARCH+1)

#define ANGLE_ADJUST  			-3.98  // (pi/2)/(accumulate phase)
//#

#define ALPHA					0.9
#define BETA					0.1


void calcul_angle(float im_r, float re_r, float im_l, float re_l);//, float im_f, float re_f, float im_b, float re_b);
int16_t sound_remote(float* data);
void regulateur(void);


static float angle_diff = 0;
static float angle_diff_old = 0;
static int intensity = 0;

static bool sound_detected = 0;




/*
 * Fonction qui permet de calculer le pic le plus haut de la fréquence ;
 * elle retourne l'index correspondant à la position de cette fréquence
 * dans le buffer
 *
 */

int16_t sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = WRONG_FREQ;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
			//return max_norm_index;
		}
	}
	//return WRONG_FREQ;

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
			//micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
			//micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

			nb_samples++;

			micRight_cmplx_input[nb_samples] = 0;
			micLeft_cmplx_input[nb_samples] = 0;
			//micBack_cmplx_input[nb_samples] = 0;
			//micFront_cmplx_input[nb_samples] = 0;

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
			//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
			//doFFT_optimized(FFT_SIZE, micBack_cmplx_input);



			/*	Magnitude processing
			*
			*	Computes the magnitude of the complex numbers and
			*	stores them in a buffer of FFT_SIZE because it only contains
			*	real numbers.
			*
			*/
			arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
			//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
			//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

			nb_samples = 0;


			int16_t highest_pic_R = sound_remote(micRight_output);
			int16_t highest_pic_L = sound_remote(micLeft_output);
			//int16_t highest_pic_F = sound_remote(micFront_output);
			//int16_t highest_pic_B = sound_remote(micBack_output);


			if((highest_pic_R != WRONG_FREQ) && (highest_pic_L != WRONG_FREQ))// && (highest_pic_F != WRONG_FREQ) && (highest_pic_B != WRONG_FREQ))
			{

				sound_detected = true;


				calcul_angle(micRight_cmplx_input[2*highest_pic_R+1], micRight_cmplx_input[2*highest_pic_R],
						micLeft_cmplx_input[2*highest_pic_L+1], micLeft_cmplx_input[2*highest_pic_L]);//,
						//micFront_cmplx_input[2*highest_pic_F+1],micFront_cmplx_input[2*highest_pic_F],
						//micBack_cmplx_input[2*highest_pic_B+1], micBack_cmplx_input[2*highest_pic_B]);

				angle_diff = angle_diff*ANGLE_ADJUST;

				intensity = sqrt((micRight_cmplx_input[2*highest_pic_R+1])*(micRight_cmplx_input[2*highest_pic_R+1]) +
						micRight_cmplx_input[2*highest_pic_R]*micRight_cmplx_input[2*highest_pic_R] );


			}
			else
			{
				sound_detected = false;
				angle_diff = false;
			}

		}


		//chThdSleepMilliseconds(10);

}


void calcul_angle(float im_r, float re_r, float im_l, float re_l)//, float im_f, float re_f, float im_b, float re_b)
{
	float angle_R = 0;
	float angle_L = 0;
	//float angle_F = 0;
	//float angle_B = 0;

	//float angle_diff_two = 0;

	angle_R = atan2f(im_r, re_r);
	angle_L = atan2f(im_l, re_l);

	//angle_F = atan2f(im_f, re_f);
	//angle_B = atan2f(im_b, re_b);

	angle_diff = angle_R - angle_L;
	//angle_diff_two = angle_B - angle_F;




	//chprintf((BaseSequentialStream *) &SDU1, " ANGLE DIFF one %f  ",angle_diff);
	//chprintf((BaseSequentialStream *) &SDU1, " ANGLE DIFF TWO %f  ",angle_diff_two);

}


void regulateur(void)
{
	angle_diff = ALPHA*angle_diff + BETA*angle_diff_old;
	angle_diff_old = angle_diff;
}

float get_angle(void)
{
	return angle_diff;
}

bool get_sound(void)
{
	return sound_detected;
}

int get_intensity(void)
{
	return intensity;
}

