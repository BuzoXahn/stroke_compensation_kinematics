

#ifndef __UNIXUDP_H__
#define __UNIXUDP_H__

/* ------------------------------------------------------------------------ */
/* Function : Create UNIX Domain Server UDP                                 */
/* Arg      : cPath                                                          */
/* Ret      : -1 : socket open file, -2 : bind fail                         */
/* ------------------------------------------------------------------------ */
int
CreateUnixUDP(char *cPath);


/* ------------------------------------------------------------------------ */
/* Function : Create UNIX Domain Server UDP                                 */
/* Arg      : cPath                                                          */
/* Ret      : -1 : socket open file, -2 : bind fail                         */
/* ------------------------------------------------------------------------ */
int
CreateUnixUDP_SendOnly();


/*----------------------------------------------------------------- 
	Function : 	Send Data througth Unix UDP                               
	Arg      : 	cSock, cPath, cMsg, nLen
	Ret      : 	-1 : send fail 
			   	else : sent data length
------------------------------------------------------------------*/
int
WriteMsgToUnixUDP(int nSock, char *cPath, char *cMsg, int nLen);

/*----------------------------------------------------------------- 
	Function : 	Receive Data througth Unix UDP                               
	Arg      : 	cSock, cBuf, 
			   	nLen : Maximum buffer size
	Ret      : 	-1 : read fail 
			   	else : received data length
	NOTE : Sender's path is not returned, because it is not possible.
------------------------------------------------------------------*/
int
ReadMsgFromUnixUDP(int nSock, char *cBuf, int nLen);

/*----------------------------------------------------------------- 
	Function : 	Send Data througth Unix UDP                               
	Arg      : 	cSock, cPath, cMsg, nLen
	Ret      : 	-1 : send fail 
			   	else : sent data length
------------------------------------------------------------------*/
int
WriteMsgToUDP(int nSock, char *cPath, char *cMsg, int nLen);

/*----------------------------------------------------------------- 
	Function : 	Receive Data througth Unix UDP                               
	Arg      : 	cSock, cBuf, 
			   	nLen : Maximum buffer size
	Ret      : 	-1 : read fail 
			   	else : received data length
	NOTE : Sender's path is not returned, because it is not possible.
------------------------------------------------------------------*/
int
ReadMsgFromUDP(int nSock, char *cBuf, int nLen);


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
int UnixUDPServerOpen(char* udp_path);


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
int MakeUnixUDPSession();


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
int RecvFromUnixUDP(int chn, char* cPacket, int len);


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
int Write2UnixUDP(int chn, char* buf, int len, char* cPath);


#endif /* __UNIXUDP_H__ */

