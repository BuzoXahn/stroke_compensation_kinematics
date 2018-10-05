CC=gcc
#CC=g++
INC=
DEF=-g -DLINUX -D__LINUX__ -DUNIX
LIBS=./libunixudp.a

udprecv: udpmisc.o 
	$(CC) $(DEF) -o udprecv udpmisc.o $(LIBS)

udpsend: udpmain.o 
	$(CC) $(DEF) -o udpsend udpmain.o $(LIBS)

udpmisc.o: 	udpmisc.c
	$(CC) $(INC) $(DEF) -c udpmisc.c

udpmain.o: 	udpmain.c
	$(CC) $(INC) $(DEF) -c udpmain.c
clean:	
	rm *.o
