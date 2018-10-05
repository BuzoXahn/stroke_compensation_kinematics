/*
 *---------------------------------------------------------------------------
 *
 * Reaching Task Simulator
 * Computational Neuro-Rehabilitation Lab., BKN, USC
 * Copyright (c) 2005-2011 All rights reserved.
 *
 *---------------------------------------------------------------------------
 * Revision 2.1  2011/07/05 Hyeshin Park
 *Hyeshin Park - 
 * version1 : June 22th 2011	HP
 * version2 : July 5th 2011		HP
 * version3 : Aug 5th 2011		HP
 * version4 : Sept 11th 2011	HP
 * version5 : Oct 11th 2011		HP
 * Reaching Task
 *---------------------------------------------------------------------------
 $Id: global.h,v 1.11 2005/12/09 18:07:22 jylee Exp $
 *---------------------------------------------------------------------------
 * $Log: global.h,v $
 * Revision 1.11  2005/12/09 18:07:22  jylee
 * check-in the sources for reporting on Dec 09, 2005
 *
 * Revision 1.10  2005/12/09 04:58:55  jylee
 * 1. save test results as a file
 * 2. add a final scene
 * 3. delete unused variables
 *
 * Revision 1.9  2005/12/08 05:46:22  jylee
 * 
 * Revision 1.8  2005/12/08 02:43:18  jylee
 * add additional scenes: screen preset, accuracy test
 *
 * Revision 1.7  2005/10/25 01:01:32  jylee
 * .
 *
 * Revision 1.6  2005/10/18 00:58:04  jylee
 * .
 *
 * Revision 1.5  2005/10/17 23:17:59  jylee
 * add codes for first presentation
 *
 * Revision 1.4  2005/09/29 00:57:16  jylee
 * change to the single screen mode
 *
 * Revision 1.3  2005/09/24 06:01:06  jylee
 * split a screen to show test results
 *
 * Revision 1.2  2005/09/23 23:29:38  jylee
 * add struct result_data
 *
 * Revision 1.1.1.1  2005/09/22 05:37:04  jylee
 * Reaching Task
 *
 * Revision 1.2  2005/09/17 02:43:08  jylee
 * remove files related with pthread, add g_pos_data_t
 *
 * Revision 1.1.1.1  2005/09/15 00:32:40  jylee
 * Reaching Task
 *
 * Revision 1.1  2005/09/11 03:05:28  jylee
 * gather global variables
 *
 * Revision 1.1  2005/09/08 03:36:10  jylee
 * move *.h files to a src directory
 *
 * Revision 1.1.1.1  2005/09/08 03:20:33  jylee
 * Reaching Task
 *
 *---------------------------------------------------------------------------
 */

#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "debug.h"

#define NO_SOUND		0
#define KEY_ENT			0x0d
#define	KEY_ESC			0x1b
#define	KEY_SPACE		0x20

#define MAXTARGET		10000		
#define MAXTRIAL		10000

#if (NO_SOUND == 0)
#define BEEP_SUCCESS()
#define BEEP_FAILURE()
#else
#define BEEP_SUCCESS()	system("aplay -q success.au")
#define BEEP_FAILURE()	system("aplay -q failure.au")

//HP
#define BEEP_CLAP()	    system("aplay -q clap.au")
#define BEEP_EXACTLY()		system("aplay -q exactly.au")
#define BEEP_BEST_SCORE()	system("aplay -q best_score.au")
#define BEEP_VERY_GOOD()	system("aplay -q verygood.au")
#define BEEP_GOOD()		system("aplay -q good.au")
#define BEEP_OK()		system("aplay -q ok.au")
#define BEEP_SLOWLY()	system("aplay -q slowly.au")
//NEW
#define BEEP_FASTEST()	system("aplay -q fastest.au")
#define BEEP_VERYFAST()	system("aplay -q veryfast.au")
#define BEEP_FAST()		system("aplay -q fast.au")
#define BEEP_SLOW()		system("aplay -q slow.au")
#define BEEP_VERYSLOW()	system("aplay -q veryslow.au")
#define BEEP_GO()	system("aplay -q go.au")

#endif

/* TODO divide a g_scene to two different stages, info and running. */
enum {

		// case name // g_scene = __
	   SC_CALIB_INFO, // 1
	   SC_CALIB_LU, // 2
	   SC_CALIB_RU, // 3
	   SC_CALIB_RB, // 4
	   SC_CALIB_LB, // 5
	   SC_CALIB_C,  // 6
	   SC_CALIB_T,  // 7
	   SC_CALIB_R,  // 8
	   SC_CALIB_B,  // 9
	   SC_CALIB_L,  // 10

	   SC_CALIB_INFO_LS, // 11
	   SC_CALIB_LU_LS, // 12
	   SC_CALIB_RU_LS, // 13
	   SC_CALIB_RB_LS, // 14
	   SC_CALIB_LB_LS, // 15
	   SC_CALIB_C_LS,  // 16
	   SC_CALIB_T_LS,  // 17
	   SC_CALIB_R_LS,  // 18
	   SC_CALIB_B_LS,  // 19
	   SC_CALIB_L_LS,  // 20

	   //first accuracy switch cases
	   accSC_TEST_INFO, //22
	   accSC_TEST_INFO_1, //23
	   accSC_TEST_READY1, // 24
	   accSC_TEST_READY2, // 25
	   accSC_TEST_GO, // 26
	   accSC_TEST_RETURN, // 27
	   accSC_TEST_INFO_2, // 28
	   // accSC_TEST_INFO_A, //
	   // accSC_TEST_INFO_3, //
	   // accSC_END, //

	   //first general switch cases
	   genSC_TEST_INFO_1, //29
	   genSC_TEST_READY1, //30
	   genSC_TEST_READY2, // 34
	   genSC_TEST_GO, // 35
	   genSC_TEST_RETURN, // 36
	   genSC_TEST_INFO_2, // 37
	   // genSC_TEST_INFO_A, //38
	   // genSC_TEST_INFO_3, // 39
	   // genSC_END, // 40

		//second accuracy switch cases
	   acc2SC_TEST_INFO_1, //41
	   acc2SC_TEST_READY1, // 42
	   acc2SC_TEST_READY2, // 43
	   acc2SC_TEST_GO, // 44
	   acc2SC_TEST_RETURN, // 45
	   acc2SC_TEST_INFO_2, // 46
	   // acc2SC_TEST_INFO_A, //47
	   // acc2SC_TEST_INFO_3, // 48
	   // acc2SC_END, // 49


	   //second general switch cases
	   gen2SC_TEST_INFO_1, //50
	   gen2SC_TEST_READY1, // 51
	   gen2SC_TEST_READY2, // 52
	   gen2SC_TEST_GO, // 53
	   gen2SC_TEST_RETURN, // 54
	   gen2SC_TEST_INFO_2, // 55
	   // gen2SC_TEST_INFO_A, //56
	   // gen2SC_TEST_INFO_3, // 57
	   gen2SC_END, // 58

	   SC_LOAD_MT, //59
	   SC_SHOW_MT, //60
	   SC_LOAD_MT1, //61
	   SC_SHOW_MT1 //62

}; 

typedef float fpos[3], ftarget[4], **fmatrix;

struct result_data
{
	fpos s;					/* the position of the 1st sensor */
	fpos s2;				/* the position of the 2nd sensor */
	fpos t;					/* the position of a target */
	long time;				/* time in miliseconds */
	//HP
	//int FB;					/* feedback */
	//////////////////////////////////////////////////////////////////////////
	int Hour;
	int Min;
	int Sec;
	int MSec;
	//////////////////////////////////////////////////////////////////////////
	double Vel1;
	double Vel2;
	
	int elbow;
	

	//////////////////////////////////////////////////////////////////////////
	// 0611 : Trj 파일에 Go Time과 Flag 저장을 위한 변수 선언
	int GoFlag;
	int GoTime;
};

extern fpos g_pos_data_t;
extern fpos g_pos_data_s;
extern fpos g_pos_data_s2;
extern int g_nos;
extern float sensor[3][2];


//HP
int argcCopy;
char **argvCopy;

#endif /* __GLOBAL_H__ */
