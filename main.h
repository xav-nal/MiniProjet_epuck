/**
 * \file		main.c
 * \version		Final
 * \date		2021-05-16
 * \author		Xavier Nal et Estelle Richard
 * \brief		Header module of the main
 *
 *This module initializes the peripherals and the threads
 *
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define ZERO  0



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
