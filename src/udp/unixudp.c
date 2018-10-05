#include "unixudp.h"


/* ------------------------------------------------------------------------ */
/* Function : Create UNIX Domain Server UDP                                 */
/* Arg      : cPath                                                          */
/* Ret      : -1 : socket open file, -2 : bind fail                         */
/* ------------------------------------------------------------------------ */
int
CreateUnixUDP(char *cPath)
{
	struct sockaddr_un unSock;
	int nSock;
	int nOptVal=64*1024;

	unlink(cPath);

	if( (nSock=socket(AF_UNIX,SOCK_DGRAM,0)) < 0 )	return -1;

	unSock.sun_family = AF_UNIX;
	strcpy(unSock.sun_path, cPath);

	if( bind(nSock,(struct sockaddr*)&unSock,sizeof(unSock)) < 0 ) return -2;

	setsockopt(nSock, SOL_SOCKET, SO_SNDBUF, (char *)&nOptVal, sizeof(nOptVal));
	setsockopt(nSock, SOL_SOCKET, SO_RCVBUF, (char *)&nOptVal, sizeof(nOptVal));

	fcntl(nSock,F_SETFL,O_NDELAY); 

	return nSock;
}


/* ------------------------------------------------------------------------ */
/* Function : Create UNIX Domain Server UDP                                 */
/* Arg      : cPath                                                          */
/* Ret      : -1 : socket open file, -2 : bind fail                         */
/* ------------------------------------------------------------------------ */
int
CreateUnixUDP_SendOnly()
{
	int nSock;
	int nOptVal=64*1024;

	if( (nSock=socket(AF_UNIX,SOCK_DGRAM,0)) < 0 )	return -1;

	setsockopt(nSock, SOL_SOCKET, SO_SNDBUF, (char *)&nOptVal, sizeof(nOptVal));

	fcntl(nSock,F_SETFL,O_NDELAY); 

	return nSock;
}


/*----------------------------------------------------------------- 
	Function : 	Send Data througth Unix UDP                               
	Arg      : 	cSock, cPath, cMsg, nLen
	Ret      : 	-1 : send fail 
			   	else : sent data length
------------------------------------------------------------------*/
int
WriteMsgToUnixUDP(int nSock, char *cPath, char *cMsg, int nLen)
{
	int		nRet;
	struct sockaddr_un 	addr;

	addr.sun_family = AF_UNIX;
	strcpy(addr.sun_path, cPath);

	nRet = sendto(nSock, (void *)cMsg, nLen, 0, 
			(struct sockaddr *) &addr, (int )sizeof(struct sockaddr_un));

	return nRet;
}


/*----------------------------------------------------------------- 
	Function : 	Receive Data througth Unix UDP                               
	Arg      : 	cSock, cBuf, 
			   	nLen : Maximum buffer size
	Ret      : 	-1 : read fail 
			   	else : received data length
	NOTE : Sender's path is not returned, because it is not possible.
------------------------------------------------------------------*/
int
ReadMsgFromUnixUDP(int nSock, char *cBuf, int nLen)
{
	int 		nLength;
	int		ret;
	struct sockaddr_un 	addr;

	ret = recvfrom(nSock, (void *)cBuf ,nLen, 0, (struct sockaddr *)&addr,	(int *)&nLength);

	return ret;
}


/*----------------------------------------------------------------- 
	Function : 	Send Data througth Unix UDP                               
	Arg      : 	cSock, cPath, cMsg, nLen
	Ret      : 	-1 : send fail 
			   	else : sent data length
------------------------------------------------------------------*/
int
WriteMsgToUDP(int nSock, char *cPath, char *cMsg, int nLen)
{
	int		nRet;
	struct sockaddr_un 	addr;

	addr.sun_family = AF_UNIX;
	strcpy(addr.sun_path, cPath);

	nRet = sendto(nSock, (void *)cMsg, nLen, 0, 
			(struct sockaddr *) &addr, (int )sizeof(struct sockaddr_un));

	return nRet;
}


/*----------------------------------------------------------------- 
	Function : 	Receive Data througth Unix UDP                               
	Arg      : 	cSock, cBuf, 
			   	nLen : Maximum buffer size
	Ret      : 	-1 : read fail 
			   	else : received data length
	NOTE : Sender's path is not returned, because it is not possible.
------------------------------------------------------------------*/
int
ReadMsgFromUDP(int nSock, char *cBuf, int nLen)
{
	int 		nLength;
	int		ret;
	struct sockaddr_un 	addr;

	ret = recvfrom(nSock, (void *)cBuf ,nLen, 0, (struct sockaddr *)&addr,	(int *)&nLength);

	return ret;
}


/* pkg: add */

/*----------------------------------------------------------------------------
 *	Name	:	UnixUDPServerOpen
 *	Desc	:	It open a socket for the server
 *	Input	:	char* udp_path
 *	Output	:	>0 == server socket
 * 				-1 == socket open error
 * 				-2 == bind error
 *	Func	:	
 *----------------------------------------------------------------------------
 */
int UnixUDPServerOpen(char* udp_path)
{
	struct sockaddr_un serv_addr;
	int		serv_len;
	int		sockfd;
	int		flag=1;
	int		schn;
	char	cPath[100];
	int		nOptVal=64*1024;

	if( (sockfd = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0 )
		return(-1);

	strcpy(cPath,udp_path);
	unlink(cPath);

	memset((char *)&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sun_family = AF_UNIX;
	strcpy(serv_addr.sun_path, cPath);
	serv_len = sizeof(serv_addr.sun_family) + strlen(serv_addr.sun_path);

	if( bind(sockfd,(struct sockaddr *)&serv_addr, serv_len) == -1 ){
		unlink(serv_addr.sun_path);
		close(sockfd);
		return(-2);
	}
	schn = sockfd;
	flag = 1;

	setsockopt(schn, SOL_SOCKET, SO_SNDBUF, (char *)&nOptVal, sizeof(nOptVal));
	setsockopt(schn, SOL_SOCKET, SO_RCVBUF, (char *)&nOptVal, sizeof(nOptVal));

#ifdef BSD
	fcntl(schn, F_SETFL, O_NDELAY);
#else
	ioctl(schn, FIONBIO, &flag);
#endif

	return(schn);
}


/*----------------------------------------------------------------------------
 *	Name	:	MakeUnixUDPSession
 *	Desc	:	A client bind local address by yourself
 *	Input	:
 *	Output	:	>0 == successfully connect to the server
 * 				-1 == socket open error
 * 				-2 == bind error
 *	Func	:	
 *----------------------------------------------------------------------------
 */
int MakeUnixUDPSession()
{
	int		sockfd;
	int		schn;
	int		nOptVal=64*1024;

	if( (sockfd = socket(AF_UNIX,SOCK_DGRAM,0)) < 0 )
		return(-1);

	schn = sockfd;

	setsockopt(schn, SOL_SOCKET, SO_SNDBUF, (char *)&nOptVal, sizeof(nOptVal));

#ifdef BSD
	fcntl(schn, F_SETFL, O_NDELAY);
#else
	ioctl(schn, FIONBIO, &flag);
#endif

	return(schn);
}


/*----------------------------------------------------------------------------
 *	Name	:	RecvFromUnixUDP
 *	Desc	:	It read data from the socket
 *	Input	:	int chn, char *buf, int len
 *	Output	:	>0 == successfully read data from the socket as len
 * 				-1 == read time out
 * 				-9 == closed the socket of the other side
 *	Func	:	
 *----------------------------------------------------------------------------
 */
int RecvFromUnixUDP(int chn, char* cPacket, int len)
{
	int		recv_len;

	recv_len = recvfrom(chn, cPacket, len, 0, (struct sockaddr *)NULL, NULL);

	if((recv_len > 0) && (recv_len <= len))
	{
		cPacket[recv_len] = 0x00;
		return(recv_len);
	}

	return(-1);
}


/*----------------------------------------------------------------------------
 *	Name	:	Write2UnixUDP
 *	Desc	:	It write data to the socket as len
 *	Input	:	int chn, char *buf, int len
 *	Output	:	>0 == successfully write data to the socket as len
 * 				-1 == write time out
 *				-9 == closed the socket of the other side
 *	Func	:	
 *----------------------------------------------------------------------------
 */
int Write2UnixUDP(int chn, char* buf, int len, char* cPath)
{
	struct sockaddr_un	send_addr;
	time_t  time0, time1;
	size_t	addr_len;
	char    *ptr;
	int     llen;

	memset((char *)&send_addr,0,sizeof(struct sockaddr_un));
	send_addr.sun_family = AF_UNIX;
	strcpy(send_addr.sun_path,cPath);
	addr_len = sizeof(send_addr.sun_family) + strlen(send_addr.sun_path);
	
	//Log(1,"WHAT0 send_addr.sun_path = %s\n",send_addr.sun_path);
	//Log(1,"WHAT1 cPath = %s\n",cPath);


	ptr = buf;
	time(&time0);

	while (len) {
		llen = sendto(chn,ptr,len,0,(struct sockaddr *)&send_addr,sizeof(struct sockaddr_un));
	//	Log(1,"llen = %d len %d\n",llen,len);
		if (llen > 0) {
			len -= llen;
			ptr += llen;
		}
		else if (llen == 0) {
			return(-9);
		}
		else if(llen == -1)
		{
			//Log(2,"WHAT2 error:%d %s\n",errno,strerror(errno));
			if(errno == ENOMEM || errno == ENOSR)
				return(-1);
			else if(errno == EWOULDBLOCK)
			{
#if 0
				/*
				time(&time1);
				if (difftime(time1, time0) >= WRITE_TIME)
				*/
					return(-2);
#else
				time(&time1);
				if (difftime(time1, time0) >= WRITE_TIME)
					return(-2);
#endif
			}
			else
			{
			//Log(2,"WHAT3 error:%d %s\n",errno,strerror(errno));
				return(-3);
			}
		}
	}
	return(1);
}
