/*
 *---------------------------------------------------------------------------
 *
 * Reaching Task Simulator
 * Computational Neuro-Rehabilitation Lab., BKN, USC
 * Copyright (c) 2005-2006 All rights reserved.
 *
 *---------------------------------------------------------------------------
 * $Id: debug.c,v 1.1.1.1 2005/09/22 05:37:04 jylee Exp $
 *---------------------------------------------------------------------------
 * $Log: debug.c,v $
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

#include <stdio.h>
#include <stdarg.h>
#include "debug.h"

extern int dprintf(int l, const char *a, ...)
{
	va_list ap;
	int done;
	if (l <= DBG_LEVEL) {
		va_start (ap, a);
		printf("(%d) ", l);
		done = vprintf(a, ap);
		va_end (ap);
	}
	return done;
}

extern int dprintf_cr(int l, const char *a, ...)
{
	va_list ap;
	int done;
	if (l <= DBG_LEVEL) {
		va_start (ap, a);
		done = vprintf(a, ap);
		va_end (ap);
	}
	return done;
}
