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


int main (void)
{
	char c[1000];
	fd_set rfds;
	struct  timeval tv;
	int udp_id2=CreateUnixUDP("./a.tmp");
	int	nret;
	while (1)
	{
                /* Watch stdin (fd 0) to see when it has input. */
                FD_ZERO(&rfds);
                FD_SET(0, &rfds);
                FD_SET(udp_id2, &rfds);

                /* Wait up to five seconds. */
              	tv.tv_sec = 5;
              	tv.tv_usec = 0;

                if( select(20, &rfds, NULL, NULL, &tv ) < 0 )
                {
                        printf("Select error!!![%s]",strerror(errno));
                }

                if(FD_ISSET(udp_id2,&rfds))
                {

		nret = ReadMsgFromUnixUDP(udp_id2,c,10);

		if( nret > 0 )
			{	
			printf("%s[%d]\n",c,nret);
			//getchar();
			}
		}
	}
	
}

