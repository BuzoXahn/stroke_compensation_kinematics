#
#---------------------------------------------------------------------------
#
# Reaching Task Simulator
# Computational Neuro-Rehabilitation Lab., BKN, USC
# Copyright (c) 2005-2006 All rights reserved.
#
#---------------------------------------------------------------------------
# $Id: Makefile,v 1.1.1.1 2005/09/22 05:37:04 jylee Exp $
#---------------------------------------------------------------------------
# $Log: Makefile,v $
# Revision 1.1.1.1  2005/09/22 05:37:04  jylee
# Reaching Task
#
# Revision 1.1.1.1  2005/09/15 00:32:40  jylee
# Reaching Task
#
# Revision 1.1  2005/09/08 06:18:29  jylee
# add Makefile
#
#---------------------------------------------------------------------------
#

SUBDIRS= src

all:
	for i in $(SUBDIRS) ; do make -C $$i ; done

clean:
	for i in $(SUBDIRS) ; do make clean -C $$i ; done

.PHONY: all clean

