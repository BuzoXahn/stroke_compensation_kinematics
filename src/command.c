/*
 *---------------------------------------------------------------------------
 *
 * Reaching Task Simulator
 * Computational Neuro-Rehabilitation Lab., BKN, USC
 * Copyright (c) 2005-2006 All rights reserved.
 *
 *---------------------------------------------------------------------------
 * $Id: command.c,v 1.2 2005/09/29 00:57:16 jylee Exp $
 *---------------------------------------------------------------------------
 * $Log: command.c,v $
 * Revision 1.2  2005/09/29 00:57:16  jylee
 * change to the single screen mode
 *
 * Revision 1.1.1.1  2005/09/22 05:37:04  jylee
 * Reaching Task
 *
 * Revision 1.2  2005/09/17 02:42:35  jylee
 * rename serial.c to sensor.c
 *
 * Revision 1.1.1.1  2005/09/15 00:32:40  jylee
 * Reaching Task
 *
 * Revision 1.4  2005/09/13 17:18:55  jylee
 * include global.h instead of debug.h
 *
 * Revision 1.3  2005/09/10 23:55:06  jylee
 * add a mode parameter to bird_hemisphere()
 *
 * Revision 1.2  2005/09/08 05:18:12  jylee
 * add hemisphere()
 *
 * Revision 1.1.1.1  2005/09/08 03:20:33  jylee
 * Reaching Task
 *
 *---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "asctech.h"
#include "sensor.h"
#include "global.h"

int send_command( unsigned char *cmd, short cmdsize )
{
	int	cmdbytessent;

    dprintf(1, "COMMAND SENT: ");
    for (cmdbytessent = 0;cmdbytessent < cmdsize; cmdbytessent++)
        dprintf_cr(1, " 0x%X",cmd[cmdbytessent]);
    dprintf_cr(1, "\n\r");

	cmdbytessent = write(serfd, cmd, cmdsize );

    dprintf(1, "COMMAND SENT: %d\n",cmdbytessent);

	return cmdbytessent;
}

int setPosPoint()
{
	unsigned char cmd;	

	cmd = 'V';
	if( send_command( &cmd,1 ) != 1 )
		return(FALSE);
		
	return TRUE;
}

int getPosPoint()
{
	unsigned char cmd;	

	if (g_nos > 1) {
		cmd = 0xF1;
		if( send_command( &cmd,1 ) != 1 )
			return FALSE;
	}

	cmd = 'B';
	if( send_command( &cmd,1 ) != 1 )
		return FALSE;
		
	return TRUE;
}

int getSlavePosPoint()
{
	unsigned char cmd;	

	cmd = 0xF2;
	if( send_command( &cmd,1 ) != 1 )
		return FALSE;

	cmd = 'B';
	if( send_command( &cmd,1 ) != 1 )
		return FALSE;
		
	return TRUE;
}

int getPosStream()
{
	unsigned char cmd;	

	cmd = '@';
	if( send_command( &cmd,1 ) != 1 )
		return FALSE;
		
	return TRUE;
}

/*
    printposition       Print and File the Position Data
    Prototype in:       cmdutil.h
    Parameters Passed:  birddata - array of birddata
						buttonmode - ON/OFF
						displayon - DISPLAYON/OFF
						datafilestream - file to store data in..if any
    Return Value:       int 3
    Remarks:
*/
int printposition(short *birddata, short buttonmode, unsigned char displayon,
				  FILE *datafilestream)
{
    short i;
    float floatdata[3];
    char *printdataformat = "\t%7.2f\t%7.2f\t%7.2f\n";

    /* Only compute if display or file is enabled */
    if ( (displayon) || (datafilestream) ) {
		for (i=0;i<3;i++)
			floatdata[i] = (float)(birddata[i] * posk);
    }

    /* Display the Data and Button Value (if required) */
    if (displayon)
		printf(printdataformat,floatdata[0],floatdata[1],floatdata[2]);

    /* Save the Data to a File...only if one exists!  */
    if (datafilestream)
		/* print the data to the file */
		fprintf(datafilestream,printdataformat,
			floatdata[0],floatdata[1],floatdata[2]);

    return(3);
}

/*
    bird_hemisphere -	Set the Birds Hemisphere
    Prototype in:       birdcmds.h
    Parameters Passed:  void
    Return Value:       TRUE if successful
						FALSE if unsuccessful
						ESC_SEL if ESC selected
    Remarks:            prompt the user for the Bird hemisphere and send
						down coresponding hemisphere command to the Bird
*/
int bird_hemisphere(int mode)
{
    static unsigned char hemisphere_cdata[] = {'L',0,0};  /* command string 
							     to BIRD */

    /* Send the Menu to the User */
    switch (mode) {
	/* Setup the Command string to the Bird as a function of the
	   User menu selection.....
	   .....2 data bytes must be set for HEMI_AXIS and HEMI_SIGN */
	case 0: /* Forward */
	    hemisphere_cdata[1] = 0;       /* set XYZ character */
	    hemisphere_cdata[2] = 0;       /* set Sine character */
	    break;

	case 1: /* Aft */
	    hemisphere_cdata[1] = 0;       /* set XYZ character */
	    hemisphere_cdata[2] = 1;       /* set Sine character */
	    break;

	case 2: /* Upper */
	    hemisphere_cdata[1] = 0xc;     /* set XYZ character */
	    hemisphere_cdata[2] = 1;       /* set Sine character */
	    break;

	case 3: /* Lower */
	    hemisphere_cdata[1] = 0xc;     /* set XYZ character */
	    hemisphere_cdata[2] = 0;       /* set Sine character */
	    break;

	case 4: /* Left */
	    hemisphere_cdata[1] = 6;       /* set XYZ character */
	    hemisphere_cdata[2] = 1;       /* set Sine character */
	    break;

	case 5: /* Right */
	    hemisphere_cdata[1] = 6;       /* set XYZ character */
	    hemisphere_cdata[2] = 0;       /* set Sine character */
	    break;

	default:
	    return(FALSE);
    }

    /* Send the Command */
    if (send_command(hemisphere_cdata,3) != 3)
		return(FALSE);

    printf("Hemisphere Data Sent to the Bird\n\r");

    return(TRUE);
}

int bird_autoconfig(int numberofsensors)
{
	/*
	Set the Parameter number and the command size
	*/
	unsigned char autoconfig[3] = {'P', 50, 1};

	autoconfig[2] = numberofsensors;
	if (send_command(autoconfig,3) != 3)
		return (FALSE);

    printf("FBB Configured\n\r");

	return TRUE;
}
