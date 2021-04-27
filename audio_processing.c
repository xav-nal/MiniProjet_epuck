#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include <math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		18	//we don't analyze before this index to not use resources for nothing
//#define FREQ_FORWARD	16	//250Hz
//#define FREQ_LEFT		19	//296Hz
//#define FREQ_RIGHT		23	//359HZ
//#define FREQ_BACKWARD	26	//406Hz
//#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing
#define FREQ_COUPURE   24 //375Hz

/*#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)
*/

static float angle_diff = 0;
static float angle_diff_old = 0;



void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}


/*
 * Fonction qui permet de calculer le pic le plus haut de la fréquence ;
 * elle retourne l'index correspondant à la position de cette fréquence
 * dans le buffer
 *
 */

uint16_t sound_remote_new(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= FREQ_COUPURE ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	return max_norm_index;


}



void Calcul_angle(int16_t *data, uint16_t num_samples){

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;
	angle_diff_old = angle_diff;
	float angle_R = 0;
	float angle_L =0;

		//loop to fill the buffers
		for(uint16_t i = 0 ; i < num_samples ; i+=4){
			//construct an array of complex numbers. Put 0 to the imaginary part
			micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
			micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

			nb_samples++;

			micRight_cmplx_input[nb_samples] = 0;
			micLeft_cmplx_input[nb_samples] = 0;

			nb_samples++;

			//stop when buffer is full
			if(nb_samples >= (2 * FFT_SIZE)){
				break;
			}
		}

		if(nb_samples >= (2 * FFT_SIZE)){
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



			//sends only one FFT result over 10 for 1 mic to not flood the computer
			//sends to UART3
			if(mustSend > 8){
				//signals to send the result to the computer
				chBSemSignal(&sendToComputer_sem);
				mustSend = 0;
			}
			nb_samples = 0;
			mustSend++;

			int16_t highest_pic_R = sound_remote_new(micRight_output);
			int16_t highest_pic_L = sound_remote_new(micLeft_output);

			//prendre float et non double parce que l'epuck convertit systématiquement les double en float de toute façon
			//arctan2f pour qu'on ait l'arc tangente d'un float en signé

	        float im_R = micRight_cmplx_input[2*highest_pic_R+1];
	        float re_R = micRight_cmplx_input[2*highest_pic_R];

			chprintf((BaseSequentialStream *) &SDU1, " im_R =    %f   ",micRight_cmplx_input[2*highest_pic_R+1]);
			chprintf((BaseSequentialStream *) &SDU1, " re_R =    %f   ",micRight_cmplx_input[2*highest_pic_R]);

			angle_R = atan2(im_R, re_R);
			chprintf((BaseSequentialStream *) &SDU1, " angle_R =    %f rad  ",angle_R);

			angle_L = atan2l(micLeft_cmplx_input[2*highest_pic_L+1], micLeft_cmplx_input[2*highest_pic_L]);
			//chprintf((BaseSequentialStream *) &SDU1, " angle_L =    %d rad  ",angle_L);

			angle_diff= angle_L - angle_R;
			//chprintf((BaseSequentialStream *) &SDU1, " angle =    %d rad  ",angle_diff);


					}

				}






double get_angle(void){
	return angle_diff;
}




/*Filtre passe-bas permettant d'éliminer les hautes fréquences
			 *
			 * Comme on a la relation : fréquence = position * 15,625
			 * On élimine toutes la valeurs dans le tableau dont l'index de position est supérieur
			 * à FREQ_COUPURE
			 *
			*/
