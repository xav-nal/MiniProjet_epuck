#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <displacement.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>





int main(void)
{

    halInit();
    chSysInit();
    mpu_init();


    //starts the USB communication
    usb_start();

    //inits the motors
    motors_init();

    //inits audio
    mic_start(&processAudio);

    //displacement init
    //displacement_start();

    /*
    while(1)
    {
    	audio_test();
    	displacement_test();
    	displacement_start();
    	chThdSleepMilliseconds(200);
    }*/

}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	while(1)
	{
    chSysHalt("Stack smashing detected");
	}
}
