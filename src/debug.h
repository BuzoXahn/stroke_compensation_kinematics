/*
 *---------------------------------------------------------------------------
 *
 * Reaching Task Simulator
 * Computational Neuro-Rehabilitation Lab., BKN, USC
 * Copyright (c) 2005-2006 All rights reserved.
 *
 *---------------------------------------------------------------------------
 * $Id: debug.h,v 1.1.1.1 2005/09/22 05:37:04 jylee Exp $
 *---------------------------------------------------------------------------
 * $Log: debug.h,v $
 * Revision 1.1.1.1  2005/09/22 05:37:04  jylee
 * Reaching Task
 *
 * Revision 1.1.1.1  2005/09/15 00:32:40  jylee
 * Reaching Task
 *
 * Revision 1.1  2005/09/08 05:17:44  jylee
 * add debug files
 *
 *---------------------------------------------------------------------------
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#define DBG_LEVEL	0

//Nic
//changed return types of both functions to extern int from 'void' because of
// error with stdio.h return type conflict
extern int dprintf(int l, const char *a, ...) __attribute__ ((format (printf, 2, 3)));
extern int dprintf_cr(int l, const char *a, ...) __attribute__ ((format (printf, 2, 3)));
       
#endif /* __DEBUG_H__ */
