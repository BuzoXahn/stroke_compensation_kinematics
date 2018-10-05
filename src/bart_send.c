#include <stdio.h>
#include "unixudp.h"

int main (void)
{
	char c[1000];
	int udp_id=CreateUnixUDP("./bart_gui.tmp");

	int nret;
		sprintf(c,"1234aa7890");
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,10);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
}

