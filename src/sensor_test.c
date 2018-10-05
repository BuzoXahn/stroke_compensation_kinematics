#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>


#include <strings.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "global.h"

#include "debug.h"
#include "sensor.h"
#include "sensor_test.h"

#include "command.h"

#include "nr.h"
#include "nrutil.h"

#define MAXBUF	8192

float sensor[3][2];
fpos g_pos_data_t;		// the position of a target
fpos g_pos_data_s;		// the position of the first sensor
fpos g_pos_data_s2;		// the position of the first sensor

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
	while (count_byte<14 && retval!=0)
	{
	retval=read(serfd,buffer,1);
	if (count_byte==0 & buffer[0]>0)
		continue;
	if (count_byte==7 & buffer[0]>0)
	    {count_byte=0;
		continue;}
	if (count_byte!=0 & count_byte!=7)
		if (buffer[0]<0)
			continue;
	if (count_byte==6 | count_byte==13)
		if (buffer[0]!=1 & buffer[0]!=2)
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


int main(int argc, char **argv)
{
	int c, i = 0,j;
	float *pos_data_s;
	char s[256], fname[256];
	FILE *fp = NULL;
	time_t tm;
	struct tm *tmp;
	struct timeval t_current, t_start, t_samp_st, t_samp_end;
	struct result_data result_data[MAXBUF];
	float	wristTau,xForce,yForce,zForce;
	int count_byte;

        int bufsize = 1024*8;
        char *buffer = malloc(bufsize);
	char buffer_actual[MAXBUFSIZE];
	short	birddata[MAXBUFSIZE];

        fd_set  rfds;
        struct  timeval tv;
        int     retval;

config_serial();
pos_data_s = (float *)malloc(3*sizeof(float));

	printf("\nPress any keys to start to test:");
	c = getchar();

	tm = time(NULL);


	/* start a test */
	gettimeofday(&t_start,NULL);

	int	tSample;
	printf("Opened serial port:%d\n",serfd);
	
	// delete all previous datas//
	sensor_clear_crap();

	while (1) {
                /* Watch stdin (fd 0) to see when it has input. */
                FD_ZERO(&rfds);
                FD_SET(0, &rfds);
                FD_SET(serfd, &rfds);

                /* Wait up to five seconds. */
              	tv.tv_sec = 5;
              	tv.tv_usec = 0;

                if( select(20, &rfds, NULL, NULL, &tv ) < 0 )
                {
                        printf("Select error!!![%s]",strerror(errno));
                }

                if(FD_ISSET(serfd,&rfds))
                {
//					printf("selected %d\n",serfd);
                        //if( (retval=read( serfd, buffer, 14 ))<0 )
                        //{
                        //        printf("read from serial error:%s", strerror(errno));
                        //}
				

			sensor_get_pos();
   printf("1x:%f y%f z%f\n2x:%f y%f z%f\n",sensor[0][0],sensor[1][0],sensor[2][0],
		sensor[0][1],sensor[1][1],sensor[2][1]);
                }



		gettimeofday(&t_samp_st,NULL);
		gettimeofday(&t_samp_end,NULL);
		tSample = t_samp_end.tv_sec*1000000+t_samp_end.tv_usec; 
//		tSample = (t_samp_st.tv_sec - t_samp_end.tv_sec)*1000000+(t_samp_st.tv_usec-t_samp_end.tv_usec); 


#if 0 

		if( i==3 )
		{
			bird_anglealign(pos_data_s);
			sleep(1);	
			
		}
#endif

		

		gettimeofday(&t_current,NULL);
		if ((t_current.tv_sec - t_start.tv_sec) > 10) {
			printf("Now exiting..\n");
			break;
		}

		i++;
	}



	free(pos_data_s);

//	sensor_finalize();

	return(0);
}

