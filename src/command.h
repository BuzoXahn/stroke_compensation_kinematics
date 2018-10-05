/*
 *---------------------------------------------------------------------------
 *
 * Reaching Task Simulator
 * Computational Neuro-Rehabilitation Lab., BKN, USC
 * Copyright (c) 2005-2006 All rights reserved.
 *
 *---------------------------------------------------------------------------
 * $Id: command.h,v 1.2 2005/09/29 00:57:16 jylee Exp $
 *---------------------------------------------------------------------------
 * $Log: command.h,v $
 * Revision 1.2  2005/09/29 00:57:16  jylee
 * change to the single screen mode
 *
 * Revision 1.1.1.1  2005/09/22 05:37:04  jylee
 * Reaching Task
 *
 * Revision 1.1.1.1  2005/09/15 00:32:40  jylee
 * Reaching Task
 *
 * Revision 1.3  2005/09/10 23:55:06  jylee
 * add a mode parameter to bird_hemisphere()
 *
 * Revision 1.2  2005/09/08 05:18:12  jylee
 * add hemisphere()
 *
 * Revision 1.1  2005/09/08 03:36:10  jylee
 * move *.h files to a src directory
 *
 * Revision 1.1.1.1  2005/09/08 03:20:33  jylee
 * Reaching Task
 *
 *---------------------------------------------------------------------------
 */

#ifndef __COMMAND_H__
#define __COMMAND_H__

int setPosPoint(void);
int getPosPoint(void);
int getSlavePosPoint(void);
int getPosStream(void);
int printposition(short *birddata, short buttonmode, unsigned char displayon,
				  FILE *datafilestream);
int bird_hemisphere(int mode);
int bird_autoconfig(int numberofsensors);

#endif /* __COMMAND_H__ */

