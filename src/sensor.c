/*
 *---------------------------------------------------------------------------
 *
 * Reaching Task Simulator
 * Computational Neuro-Rehabilitation Lab., BKN, USC
 * Copyright (c) 2005-2006 All rights reserved.
 *
 *---------------------------------------------------------------------------
 * $Id: sensor.c,v 1.13 2005/12/09 18:07:22 jylee Exp $
 *---------------------------------------------------------------------------
 * $Log: sensor.c,v $
 * Revision 1.13  2005/12/09 18:07:22  jylee
 * check-in the sources for reporting on Dec 09, 2005
 *
 * Revision 1.12  2005/12/08 05:46:22  jylee
 * .
 *
 * Revision 1.11  2005/12/05 23:59:56  jylee
 * .
 *
 * Revision 1.10  2005/10/24 23:18:18  jylee
 * modify test sessions
 *
 * Revision 1.9  2005/10/18 00:58:04  jylee
 * .
 *
 * Revision 1.8  2005/10/17 23:17:59  jylee
 * add codes for first presentation
 *
 * Revision 1.7  2005/09/29 04:47:43  jylee
 * align codes
 *
 * Revision 1.6  2005/09/29 00:57:16  jylee
 * change to the single screen mode
 *
 * Revision 1.5  2005/09/24 06:01:06  jylee
 * split a screen to show test results
 *
 * Revision 1.4  2005/09/24 01:26:02  jylee
 * add a simulation mode
 *
 * Revision 1.3  2005/09/24 00:38:13  jylee
 * *** empty log message ***
 *
 * Revision 1.2  2005/09/22 07:07:27  jylee
 * code arrangement
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
 * Revision 1.4  2005/09/11 03:16:46  jylee
 * move run() to main.c
 *
 * Revision 1.3  2005/09/10 23:54:22  jylee
 * fix the data received from bird to be correct
 *
 * Revision 1.2  2005/09/08 05:18:24  jylee
 * add hemishpere()
 *
 * Revision 1.1.1.1  2005/09/08 03:20:33  jylee
 * Reaching Task
 *
 *---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "command.h"
#include "sensor.h"

#include "nr.h"
#include "nrutil.h"
#include "global.h"

int g_nos = 2;						/* number of sensors */
int cnt = 0;
int t_wait = 2500;
float sensor[3][2];

int config_serial()
{
        struct termios oldtio,newtio;

/*Y*/ serfd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_SYNC );
	if (serfd <0) {
		dprintf(0, "no sensor detected : %s\n", strerror(errno));
		return -1;
	}
        tcgetattr(serfd,&oldtio); /* save current port settings */

        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
/*Y*/ 	newtio.c_iflag = IXOFF | IGNPAR | IXON ;
        newtio.c_oflag = 0;


        /* set input mode (non-canonical, no echo,...) */
/*Y*/	newtio.c_lflag = 1;
/*Y*/	newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
        newtio.c_cc[VMIN]     = 1 ;   /* blocking read until 5 chars received */

/*Y*/// 	tcflush(serfd, TCIFLUSH);
        tcsetattr(serfd,TCSANOW,&newtio);

	posk=POSK36;
	sensor_clear_crap();
	return serfd;
}

int sensor_clear_crap(void)
{
        char buffer[MAXBUFSIZE];
        int     retval;
				
	read(serfd,buffer,MAXBUFSIZE);
	int count_byte=0;
	while (count_byte<7)
	{
	while (1)
		{
		read(serfd,buffer,1);
		if ((buffer[0]&0x80)>0)
			{
			count_byte=1;
			break;
			}
		}
	retval=read(serfd,buffer,7-count_byte);
	count_byte+=retval;
	}
	return 1;
}


void sensor_init(int numberofsensors)
{
// deprecated for streaming mode.
/*	g_nos = numberofsensors;
	posk = POSK36;
	config_serial();
	dprintf(0,"the number of sensors : %d\n", g_nos);

	bird_hemisphere(0);
	bird_autoconfig(numberofsensors);
	setPosPoint();
*/
}

void sensor_finalize()
{
	close(serfd);
}


int sensor_get_pos()
{
        char buffer[MAXBUFSIZE];
	char buffer_actual[14];
	short	birddata[3];
	

	int retval, count_byte,i,j;

/* Contain crappy double reading
	count_byte=0;
	while (count_byte<14)
	{
	retval=read(serfd,buffer,14-count_byte);
	for (i=0; i<retval;i++)
		buffer_actual[i+count_byte]=buffer[i];
	count_byte+=retval;
	}
*/
	count_byte=0;
	retval=1;
	while ((count_byte<14) && (retval!=0))
	{
	retval=read(serfd,buffer,1);
	if ((count_byte==0) & (buffer[0]>0))
		continue;
	if ((count_byte==7) & (buffer[0]>0))
	    {count_byte=0;
		continue;}
	if ((count_byte!=0) & (count_byte!=7))
		if (buffer[0]<0)
			continue;
	if ((count_byte==6) | (count_byte==13))
		if ((buffer[0]!=1) & (buffer[0]!=2))
			{count_byte=0;
			continue;}
	buffer_actual[count_byte]=buffer[0];
	count_byte++;
	}	
	
	if (retval==0)
	{return 1;}
/*	
	for (i=0;i<count_byte;i++)
		printf("%d ",(unsigned int)buffer_actual[i]);
	printf("\n");
*/	
	for (i = 0, j = 0; i < 7; i+=2, j++) {
//Jeong's code to decoding
//		buffer_actual[i] &= 0x7F;
//		buffer_actual[i+1] |= ((buffer_actual[i+1] & 0x40) << 1);
//		birddata[j] = ((short)buffer_actual[i]) | (((short)buffer_actual[i+1]) << 8);
//my code to decoding
		buffer_actual[i] &= 0x7F;
		buffer_actual[i+1] &= 0x7f;
		birddata[j]=(signed int)((buffer_actual[i]<<2) | (buffer_actual[i+1]
					<< 9));
	}
	sensor[0][(signed int)buffer_actual[6]-1] = (float)(birddata[0+0] )* posk;
	sensor[1][(signed int)buffer_actual[6]-1] = (float)(birddata[1+0] )* posk;
	sensor[2][(signed int)buffer_actual[6]-1] = (float)(birddata[2+0] )* posk;

	for (i = 7, j = 0; i < 14; i+=2, j++) {
//Jeong's code to decoding
//		buffer_actual[i] &= 0x7F;
//		buffer_actual[i+1] |= ((buffer_actual[i+1] & 0x40) << 1);
//		birddata[j] = ((short)buffer_actual[i]) | (((short)buffer_actual[i+1]) << 8);
//my code to decoding
		buffer_actual[i] &= 0x7F;
		buffer_actual[i+1] &= 0x7f;
		birddata[j]=(signed int)((buffer_actual[i]<<2) | (buffer_actual[i+1]
					<< 9));
	}
	sensor[0][(signed int)buffer_actual[13]-1] = (float)(birddata[0+0] )* posk;
	sensor[1][(signed int)buffer_actual[13]-1] = (float)(birddata[1+0] )* posk;
	sensor[2][(signed int)buffer_actual[13]-1] = (float)(birddata[2+0] )* posk;

//printf("%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f\n",sensor[0][0],sensor[1][0],sensor[2][0],sensor[0][1],sensor[1][1],sensor[2][1]);

	g_pos_data_s[0] = sensor[0][0];
	g_pos_data_s[1] = sensor[1][0];
	g_pos_data_s[2] = sensor[2][0];

	g_pos_data_s2[0] = sensor[0][1];
	g_pos_data_s2[1] = sensor[1][1];
	g_pos_data_s2[2] = sensor[2][1];


	return 0;
}

