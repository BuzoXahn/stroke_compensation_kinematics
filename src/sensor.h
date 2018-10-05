/*
 *---------------------------------------------------------------------------
 *
 * Reaching Task Simulator
 * Computational Neuro-Rehabilitation Lab., BKN, USC
 * Copyright (c) 2005-2006 All rights reserved.
 *
 *---------------------------------------------------------------------------
 * $Id: sensor.h,v 1.4 2005/09/29 04:47:43 jylee Exp $
 *---------------------------------------------------------------------------
 * $Log: sensor.h,v $
 * Revision 1.4  2005/09/29 04:47:43  jylee
 * align codes
 *
 * Revision 1.3  2005/09/29 00:57:16  jylee
 * change to the single screen mode
 *
 * Revision 1.2  2005/09/24 00:38:13  jylee
 * *** empty log message ***
 *
 * Revision 1.1.1.1  2005/09/22 05:37:04  jylee
 * Reaching Task
 *
 * Revision 1.1  2005/09/17 02:41:59  jylee
 * rename serial.* to sensor.*
 *
 * Revision 1.1.1.1  2005/09/15 00:32:40  jylee
 * Reaching Task
 *
 * Revision 1.2  2005/09/11 03:17:01  jylee
 * move run() to main.c
 *
 * Revision 1.1  2005/09/08 03:36:10  jylee
 * move *.h files to a src directory
 *
 * Revision 1.1.1.1  2005/09/08 03:20:33  jylee
 * Reaching Task
 *
 *---------------------------------------------------------------------------
 */

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "asctech.h"
#include "debug.h"

#define	MAXBUFSIZE 		4096
#define	MAXSERIALERR	3
#define BAUDRATE		B115200
//#define MODEMDEVICE		"/dev/ttyS0"
//Nic
//define MODEMDEVICE to be where the RS323 serial adapter from USB is located
#define MODEMDEVICE "/dev/ttyUSB0"
// use: dmesg | grep tty    to confirm this is the location

#define _POSIX_SOURCE	1				/* POSIX compliant source */
        
/* global variable definition */
int serfd;
float posk;

/* function definition */
int config_serial();
void sensor_init(int numberofsensors);
void sensor_finalize(void);
int sensor_get_pos(void);
int sensor_clear_crap(void);

#endif /* __SENSOR_H__ */
