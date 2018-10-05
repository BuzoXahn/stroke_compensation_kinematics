#include <stdio.h>

int main (void)
{
	char c[1000];
	int udp_id=CreateUnixUDP("./b.tmp");
	int nret;
		sprintf(c,"1234aa7890");
		nret=WriteMsgToUnixUDP(udp_id,"./a.tmp",c,10);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
}

