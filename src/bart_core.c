   //Hyeshin Park - 
// version1 : June 22th 2011	HP
// version2 : July 5th 2011		HP
// version3 : Aug 5th 2011		HP
// version4 : Sept 11th 2011	HP
// version5 : Oct 11th 2011		HP
// version6 : Dec 30th 2011		HP
//bart_core.c 
//make clean
//make bart_core
//make bart_gui

#include <math.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "global.h"
#include "sensor.h"
#include "unixudp.h"

#include "nr.h"
#include "nrutil.h"

//param
#define FCONFIG_PARAM	"../conf/Accuracy/config-param.txt"
#define FCONFIG_PARAM_T1	"../conf/Accuracy/config-param_T1.txt"
#define FCONFIG_PARAM_B	"../conf/Accuracy/config-param_b.txt"
//Pos
#define	FCONFIG_POS		"../conf/Accuracy/config-pos.txt"
//sched
#define	FCONFIG_SCHED	"../conf/Accuracy/config-sched.txt"
#define	FCONFIG_SCHED_T1	"../conf/Accuracy/config-sched1.txt"
#define	FCONFIG_SCHED_B	"../conf/Accuracy/config-sched_b.txt"

//param
#define genFCONFIG_PARAM	"../conf/General/config-param.txt"
#define genFCONFIG_PARAM_T1	"../conf/General/config-param_T1.txt"
#define genFCONFIG_PARAM_B	"../conf/General/config-param_b.txt"
//Pos
#define	genFCONFIG_POS		"../conf/General/config-pos.txt"
//sched
#define	genFCONFIG_SCHED	"../conf/General/config-sched.txt"
#define	genFCONFIG_SCHED_T1	"../conf/General/config-sched1.txt"
#define	genFCONFIG_SCHED_B	"../conf/General/config-sched_b.txt"

#define MAXBUFSZ		4096
#define	MAXLINESZ		32
#define	MAXPARAM		128

char LOGDIRparent[255]="../data/";

//directory paths for logfile
char LOGDIR[255]="";
char generalDIR[255]="";
char accuracyDIR[255]="";
char generalDIR2[255]="";
char accuracyDIR2[255]="";
//string additions for strncat
char addFirstHand[255]="/firstHand";
char addSecondHand[255]="/secondHand";
char addGeneral[255]="/General";
char addAccuracy[255]="/Accuracy";
//accuracy
char LOGDIR1[255]="../data1/Accuracy";
char LOGFILEacc[255]="";
char LOGFILEacc2[255]="";
char temp_characc[255]="/firstAccuracyLog.txt";
char temp_characc2[255]="/secondAccuracyLog.txt";
char log_tempacc[256];
char log_tempacc2[256];
//general
char LOGDIRgen[255]="";
char LOGDIR1gen[255]="../data1/General";
char LOGFILEgen[255]="";
char LOGFILEgen2[255]="";
char temp_chargen[255]="/firstGeneralLog.txt";
char temp_chargen2[255]="/secondGeneralLog.txt";
char log_tempgen[256];
char log_tempgen2[256];

char tempFirstHand[255]="";
char tempSecondHand[255]="";




fpos g_pos_data_t;		// the position of a target
fpos g_pos_data_s;		// the position of the first sensor
fpos g_pos_data_s2;		// the position of the first sensor

int setupCond; //need this so setup_condition from scanf can be carried across functions
float g_real_w = 28.8; // 9.9
float g_real_h = 22; // 7.5
float g_real_z = 2.138; //1.0; //0.964; // 1.424;	//1.334;

fmatrix g_calibs, g_calibt, g_matrix;
fpos g_s, g_s2, g_rms[7], g_s_init, g_s2_init;
struct result_data g_result_data[MAXTRIAL];

int cnt_trial = 1, cnt_target = 1, cnt_calib = 1, cnt_rms = 1;
int flag_direction=0,selected_hand=0;
int w = 640, h = 640, g_scene = SC_CALIB_INFO;
float g_ratio=1.318229;

FILE *g_fp = NULL, *g_logfp = NULL;
FILE *g_fpGen = NULL, *g_logfpGen = NULL;
FILE *g_fp2 = NULL, *g_logfp2 = NULL; //secondHand ones for accuracy
FILE *g_fpGen2 = NULL, *g_logfpGen2 = NULL; //secondHand ones for general

struct timeval t_end, t_start;
//HP
int mode =1;
int inmode=1;
int Feedback=0;
int	IsReached=0;

struct timeval tval;
struct timezone tzone;


//NEW
//double	MovTimeMeanStd[5][8];
double	MovTimeMeanStd[5][20];
double	MovTimeMeanStd1[5][12];
int FindAll=0;
float M1=0.0, M2=0.0, M3=0.0, M4=0.0, M5=0.0;
float Mean1=0.0, Mean2=0.0, Mean3=0.0, Mean4=0.0, Mean5=0.0;
float total1=0.0, total2=0.0, total3=0.0, total4=0.0, total5=0.0;
float sum1=0.0, sum2=0.0, sum3=0.0, sum4=0.0, sum5=0.0;
float stdev1=0.0, stdev2=0.0, stdev3=0.0, stdev4=0.0, stdev5=0.0;
float stdev21=0.0, stdev22=0.0, stdev23=0.0, stdev24=0.0, stdev25=0.0;
float stdev31=0.0, stdev32=0.0, stdev33=0.0, stdev34=0.0, stdev35=0.0;
float BestMT1=0.0, BestMT2=0.0, BestMT3=0.0, BestMT4=0.0, BestMT5=0.0;
int MeanStd=0;
//int MeanStd1=0;


// HP - target color change // target size, color [R, G, B]
ftarget t_calib = { 0.03, 0.0, 1.0, 0.0 }; // small & Green!!
ftarget t_ready = { 0.03, 1.0, 1.0, 1.0 }; // large & Green!!
ftarget t_test  = { 0.03, 1.0, 1.0, 1.0 }; // large & White!!

ftarget t_test0  = { 0.03, 1.0, 1.0, 1.0 }; // small & White!!
ftarget t_test2  = { 0.3, 1.0, 1.0, 1.0 }; // big & White!!

//HP - When they have best score!!!
ftarget t_best = { 0.03, 1.0, 1.0, 1.0 }; // big & green!! 
//HP - When they have best score!!!
ftarget t_very = { 0.03, 1.0, 1.0, 1.0 }; // big & green!! 
//HP - When they have best score!!!
ftarget t_good = { 0.03, 1.0, 1.0, 1.0 }; // big & Yellow!! 
//HP - When they have the ok score!!!
ftarget t_ok = { 0.03, 1.0, 1.0, 1.0 }; // big & Orange red!! 
//HP - When they have the harder score!!!
ftarget t_harder = { 0.03, 1.0, 1.0, 1.0 }; // big & Red!! 

/* A pseudo random sequence is calculated by pseudo.c */
int g_sched[MAXTRIAL];
/* During a pilot test, 50 positions are used to conduct reaching task. */
fpos g_pos_test[MAXTRIAL];


//HP
int maxtrial = MAXTRIAL, maxtarget = MAXTARGET, maxtime = 5000;
int ITT=1200;

//hp -2018/01/23
int session_score=0;

int wait_trial=0, return_trial=0, waitTime=1000000;
int no_desired_target=MAXTARGET,no_successful_target=0;
float ITTfloat=0.0;



/*  UDP  */
char c[1000];
fd_set rfds;
struct  timeval tv;
int udp_id;
int	nret;
/* serial for FDSET */
int serfd;
int target_displayed_gui=0;
int setup_condition=1;

int		MTstartIdx=0;
int		MTstopIdx=0;
















int shoulder = -20.8;
double watch1=0.0;
int is_elbow=0;
int elbowCnt=0;
double TimeElbow=0.0;
int initElbow=0;


//////////////////////////////////////////////////////////////////////////
// Bom 04/01/2013
int TargetReachCount=0;
int is_InitialTrargetReached=0;
int TargetRemain=0;
int pis_reached=0;
int SucTarReach=0;
double TargetReachTime=0.0;

//////////////////////////////////////////////////////////////////////////
// Log file
#define MAX_TRIAL_NUM			200
#define LOG_DATA_NUM			7
//find Max Movement
double	MaxMTvalue=0.0;
int		MaxMTidx=0;
double	MaxMTtargetPosX=0.0;
double	MaxMTtargetPosY=0.0;
int		MaxMTtargetNumber=0;
char	strLine[255]={0};
char	ch=0;
int		a_datacount=0;
int		a_headercount=0;
double	DataBuffer[MAX_TRIAL_NUM][LOG_DATA_NUM];
int		idxi, idxj;
int		trialnumber=1;
double	MovTimeFromLog[5][10];
int		NumOfTrainingSession=0;
int		NumOfTrialSession=0;


double	MovTimeBest[5];

#define	MTFILENAME	"./MT.txt"
int idxNumofTraining=0;

//new
double SXp1, SYp1, SXc1, SYc1, SZp1=0.0;
double SXp2, SYp2, SXc2, SYc2=0.0;
double SamplingTime=0.0;
double VelOfSen=0.0;

//////////////////////////////////////////////////////////////////////////
// Fixing
int T1X=1606;
int T1Y=1915;

int T2X=855;
int T2Y=2349;

int T3X=0;
int T3Y=2500;

int T4X=-855;
int T4Y=2349;

int T5X=-1606;
int T5Y=1915;
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// 0611
int IsGoSound=0;
int TimeForGo=0;
int AtTimeGo=0;
int IsReadySound=0;
//////////////////////////////////////////////////////////////////////////


//int is_elbow=0;


//////////////////////////////////////////////////////////////////////////
// New Calibration
//////////////////////////////////////////////////////////////////////////
// Sensor2 Calib
double Calibraton_Matrix2[12];
double m_CalibResult2[12];

//////////////////////////////////////////////////////////////////////////
double Calibraton_Matrix[12];
double m_CalibResult[12];
double R11=0.0;
double R12=0.0;
double R13=0.0;
double R21=0.0;
double R22=0.0;
double R23=0.0;
double R31=0.0;
double R32=0.0;
double R33=0.0;
double d1=0.0;
double d2=0.0;
double d3=0.0; 
//////////////////////////////////////////////////////////////////////////

double Baylor=0.0;

//here
void saveMTtoFile(char MTFILE[255]){
	FILE *g_MTfp = NULL;
	char s[256]="";
	if (g_MTfp == NULL) {
		if ((g_MTfp = fopen(MTFILE, "wb")) == NULL) {
			dprintf(0, "fopen error in saveMTtoFile: %s\n", strerror(errno));
		}
		else
		{
			for( idxNumofTraining=0 ; idxNumofTraining < 6 ; idxNumofTraining++){
				sprintf(s, "%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n", MovTimeFromLog[0][idxNumofTraining], MovTimeFromLog[1][idxNumofTraining], 
					MovTimeFromLog[2][idxNumofTraining], MovTimeFromLog[3][idxNumofTraining], MovTimeFromLog[4][idxNumofTraining]);
				fwrite(s, 1, strlen(s), g_MTfp);
				
				
			}
		}
	}
	fclose(g_MTfp);
}



void LoadLOGfileComputeMTbyTarget(char LOGFILEacc[255], int TrainingSession){
	FILE* m_FilePointer;
	m_FilePointer = fopen(LOGFILEacc, "r");
	
	for( idxi=0; idxi<LOG_DATA_NUM; idxi++){
		for( idxj=0; idxj<LOG_DATA_NUM; idxj++){
			DataBuffer[idxi][idxj]=0.0;
		}
	}
	//check the previous values
	printf("Check the previous values!!!\n");
	printf("check1-BestMT1:%4.2f,BestMT2:%4.2f,BestMT3:%4.2f,BestMT4:%4.2f,BestMT5:%4.2f\n",BestMT1, BestMT2, BestMT3, BestMT4, BestMT5);
	printf("check1-Mean1:%4.2f,stdev1:%4.2f,stdev21:%4.2f\n",Mean1,stdev1,stdev21);
	printf("check1-Mean2:%4.2f,stdev2:%4.2f,stdev22:%4.2f\n",Mean2,stdev2,stdev22);
	printf("check1-Mean3:%4.2f,stdev3:%4.2f,stdev23:%4.2f\n",Mean3,stdev3,stdev23);
	printf("check1-Mean4:%4.2f,stdev4:%4.2f,stdev24:%4.2f\n",Mean4,stdev4,stdev24);
	printf("check1-Mean5:%4.2f,stdev5:%4.2f,stdev25:%4.2f\n",Mean5,stdev5,stdev25);


	//update
	M1=0, M2=0, M3=0, M4=0, M5=0;
	Mean1=0, Mean2=0, Mean3=0, Mean4=0, Mean5=0;
	total1=0, total2=0, total3=0, total4=0, total5=0;
	sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
	stdev1=0, stdev2=0, stdev3=0, stdev4=0, stdev5=0;
	stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;
	MeanStd=0;
	//update

	for(idxi=0 ; idxi < 5 ; idxi++){
		MovTimeFromLog[idxi][TrainingSession]=0.0;
	}

	//////////////////////////////////////////////////////////////////////////
	a_headercount=0;  
	a_datacount=0;
	//////////////////////////////////////////////////////////////////////////

	while(!feof(m_FilePointer) && a_datacount <100){
		// Read the rest of the line so the file pointer returns to the next line.
		fgets(strLine, 1000, m_FilePointer);
		
		if(a_headercount >= 1){
			float trial, tposx, tposy, MT, err, hit, hand, MT1 , fb = 0.0; //in log.txt
			fscanf(m_FilePointer,"%f \t%f,%f \t%f \t%f \t%f \t%f \t%f \t%f",&trial,&tposx,&tposy,&MT,&err,&hit,&hand,&MT1, &fb);
			
			DataBuffer[a_datacount][0] = trial;
			DataBuffer[a_datacount][1] = tposx;
			DataBuffer[a_datacount][2] = tposy;
			DataBuffer[a_datacount][3] = MT;
			DataBuffer[a_datacount][4] = err;
			DataBuffer[a_datacount][5] = hit;
			DataBuffer[a_datacount][6] = hand;
			DataBuffer[a_datacount][7] = MT1;
			
// 			if(tposx==(float)T1X/100 && tposy==(float)T1Y/100){
// 				MovTimeFromLog[0][TrainingSession] += MT1;				
// 			}
// 			if(tposx==(float)T2X/100 && tposy==(float)T2Y/100){
// 				MovTimeFromLog[1][TrainingSession] += MT1;
// 			}
// 			if(tposx==(float)T3X/100 && tposy==(float)T3Y/100){
// 				MovTimeFromLog[2][TrainingSession] += MT1;
// 			}
// 			if(tposx==(float)T4X/100 && tposy==(float)T4Y/100){
// 				MovTimeFromLog[3][TrainingSession] += MT1;
// 			}
// 			if(tposx==(float)T5X/100 && tposy==(float)T5Y/100){
// 				MovTimeFromLog[4][TrainingSession] += MT1;
// 			}


			if((int)(tposx*100)>=T1X-5 && (int)(tposx*100)<=T1X+5 && (int)(tposy*100)>=T1Y-5 && (int)(tposy*100)<=T1Y+5){
				MovTimeFromLog[0][TrainingSession] += MT1;				
			}
			if((int)(tposx*100)>=T2X-5 && (int)(tposx*100)<=T2X+5 && (int)(tposy*100)>=T2Y-5 && (int)(tposy*100)<=T2Y+5){
				MovTimeFromLog[1][TrainingSession] += MT1;
			}
			if((int)(tposx*100)>=T3X-5 && (int)(tposx*100)<=T3X+5 && (int)(tposy*100)>=T3Y-5 && (int)(tposy*100)<=T3Y+5){
				MovTimeFromLog[2][TrainingSession] += MT1;
			}
			if((int)(tposx*100)>=T4X-5 && (int)(tposx*100)<=T4X+5 && (int)(tposy*100)>=T4Y-5 && (int)(tposy*100)<=T4Y+5){
				MovTimeFromLog[3][TrainingSession] += MT1;
			}
			if((int)(tposx*100)>=T5X-5 && (int)(tposx*100)<=T5X+5 && (int)(tposy*100)>=T5Y-5 && (int)(tposy*100)<=T5Y+5){
				MovTimeFromLog[4][TrainingSession] += MT1;
			}


			a_datacount++;
		}		
		a_headercount++;
		
	}
	fclose(m_FilePointer);
	
	for( idxi=0 ; idxi<5 ; idxi++){
		MovTimeFromLog[idxi][TrainingSession] = MovTimeFromLog[idxi][TrainingSession]/20;

		printf("Calculate the new average values!!!\n");
		printf("Check2 :TargetNum:%d, Average MT:[%4.2f]\n",idxi,MovTimeFromLog[idxi][TrainingSession]);

		if(idxi==0){
			Mean1=MovTimeFromLog[idxi][TrainingSession];
			printf("check3-new Mean1:%4.2f\n",Mean1);
		}
		if(idxi==1){
			Mean2=MovTimeFromLog[idxi][TrainingSession];
			printf("check3-new Mean2:%4.2f\n",Mean2);
		}
		if(idxi==2){
			Mean3=MovTimeFromLog[idxi][TrainingSession];
			printf("check3-new Mean3:%4.2f\n",Mean3);
		}
		if(idxi==3){
			Mean4=MovTimeFromLog[idxi][TrainingSession];
			printf("check3-new Mean4:%4.2f\n",Mean4);
		}
		if(idxi==4){
			Mean5=MovTimeFromLog[idxi][TrainingSession];
			printf("check3-new Mean5:%4.2f\n",Mean5);
		}

		printf("Compare check1 & this \n");

	}
	
	saveMTtoFile(MTFILENAME);

	a_datacount=0;
}
//////////////////////////////////////////////////////////////////////////





//NEW -------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////
// Log file  
#define MAX_TRIAL_NUM_t			200
#define LOG_DATA_NUM_t			7
//find Max Movement
double	MaxMTvalue_t=0.0;
int		MaxMTidx_t=0;
double	MaxMTtargetPosX_t=0.0;
double	MaxMTtargetPosY_t=0.0;
int		MaxMTtargetNumber_t=0;
char	strLine_t[255]={0};
char	ch_t=0;
int		a_datacount_t=0;
int		a_headercount_t=0;
double	DataBuffer_t[MAX_TRIAL_NUM_t][LOG_DATA_NUM_t];
int		idxi_t, idxj_t;
int		trialnumber_t=1;
double	MovTimeFromLog_t[5][20];
//double	MovTimeFromLogMT[5][20];
//int		NumOfTrainingSession=0;
int		TrialSession=0;

int i=0;
int Div=8;

#define	MTFILENAME_T	"./MT_t.txt"
int idxNumofTrial=0; //trial


float outlier1P=0.0; float outlier1N=0.0;
float outlier2P=0.0; float outlier2N=0.0;
float outlier3P=0.0; float outlier3N=0.0;
float outlier4P=0.0; float outlier4N=0.0;
float outlier5P=0.0; float outlier5N=0.0;
int jj1=0; 	int jj2=0; 	int jj3=0; 	int jj4=0; 	int jj5=0;
double NewNewMean[5];



int FindAllTarget=0;

void saveMTtoFile_t(char MTFILE_T[255]){
	FILE *g_MTfp_t = NULL;
	char s[256]="";
	if (g_MTfp_t == NULL) {
		if ((g_MTfp_t = fopen(MTFILE_T, "wb")) == NULL) {
			dprintf(0, "fopen error in save_t: %s\n", strerror(errno));
		}
		else
		{
			for( idxNumofTrial=0 ; idxNumofTrial < 20 ; idxNumofTrial++){
				sprintf(s, "%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n", MovTimeFromLog_t[0][idxNumofTrial], MovTimeFromLog_t[1][idxNumofTrial], 
					MovTimeFromLog_t[2][idxNumofTrial], MovTimeFromLog_t[3][idxNumofTrial], MovTimeFromLog_t[4][idxNumofTrial]);
				fwrite(s, 1, strlen(s), g_MTfp_t);			
				
			}
		}
	}
	fclose(g_MTfp_t);
}



int a_datacount_tmp;
int	ModifiedTime=0;

void LoadLOGfileComputeMTbyTarget_t(char LOGFILEacc[255], int TrialSession){

	FILE* m_FilePointer_t;
	m_FilePointer_t = fopen(LOGFILEacc, "r");
	
	for( idxi_t=0; idxi_t < LOG_DATA_NUM_t ; idxi_t++){
		for( idxj_t=0; idxj_t < LOG_DATA_NUM_t ; idxj_t++){
			DataBuffer_t[idxi_t][idxj_t]=0.0;
		}
	}


	//update
	//M1=0, M2=0, M3=0, M4=0, M5=0;
	Mean1=0, Mean2=0, Mean3=0, Mean4=0, Mean5=0;
	total1=0, total2=0, total3=0, total4=0, total5=0;
	sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
	stdev1=0, stdev2=0, stdev3=0, stdev4=0, stdev5=0;
	stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;
	stdev31=0, stdev32=0, stdev33=0, stdev34=0, stdev35=0;
		
	jj1=0, jj2=0, jj3=0, jj4=0, jj5=0;
	outlier1P=0, outlier2P=0, outlier3P=0, outlier4P=0, outlier5P=0;
	outlier1N=0, outlier2N=0, outlier3N=0, outlier4N=0, outlier5N=0;
	//MeanStd=0;
	//update

	//////////////////////////////////////////////////////////////////////////
	for( idxi_t=0 ; idxi_t < 5 ; idxi_t++){
		for( idxj_t=0 ; idxj_t < 20 ; idxj_t++){
			MovTimeFromLog_t[idxi_t][idxj_t]=0.0;
		}
	}	
	a_headercount_t=0;
	FindAllTarget =0;
	a_datacount_tmp=0;
	//////////////////////////////////////////////////////////////////////////

	while(!feof(m_FilePointer_t) && a_datacount_tmp < 100){
		// Read the rest of the line so the file pointer returns to the next line.
		fgets(strLine_t, 1000, m_FilePointer_t);
		
		if(a_headercount_t >= 1){
			float trial, tposx, tposy, MT, err, hit, hand, MT1, fb = 0.0;
			fscanf(m_FilePointer_t,"%f \t %f,%f \t %f \t %f \t %f \t %f \t %f  \t %f",&trial,&tposx,&tposy,&MT,&err,&hit,&hand,&MT1, &fb);
			
			DataBuffer_t[a_datacount_t][0] = trial;
			DataBuffer_t[a_datacount_t][1] = tposx;
			DataBuffer_t[a_datacount_t][2] = tposy;
			DataBuffer_t[a_datacount_t][3] = MT;
			DataBuffer_t[a_datacount_t][4] = err;
			DataBuffer_t[a_datacount_t][5] = hit;
			DataBuffer_t[a_datacount_t][6] = hand;
			DataBuffer_t[a_datacount_t][7] = MT1;
			
			if((int)(tposx*100)>=T1X-5 && (int)(tposx*100)<=T1X+5 && (int)(tposy*100)>=T1Y-5 && (int)(tposy*100)<=T1Y+5){
				MovTimeFromLog_t[0][a_datacount_t] = MT1;
				FindAllTarget++;
			}
			if((int)(tposx*100)>=T2X-5 && (int)(tposx*100)<=T2X+5 && (int)(tposy*100)>=T2Y-5 && (int)(tposy*100)<=T2Y+5){
				MovTimeFromLog_t[1][a_datacount_t] = MT1;
				FindAllTarget++;
			}
			if((int)(tposx*100)>=T3X-5 && (int)(tposx*100)<=T3X+5 && (int)(tposy*100)>=T3Y-5 && (int)(tposy*100)<=T3Y+5){
				MovTimeFromLog_t[2][a_datacount_t] = MT1;
				FindAllTarget++;
			}
			if((int)(tposx*100)>=T4X-5 && (int)(tposx*100)<=T4X+5 && (int)(tposy*100)>=T4Y-5 && (int)(tposy*100)<=T4Y+5){
				MovTimeFromLog_t[3][a_datacount_t] = MT1;
				FindAllTarget++;
			}
			if((int)(tposx*100)>=T5X-5 && (int)(tposx*100)<=T5X+5 && (int)(tposy*100)>=T5Y-5 && (int)(tposy*100)<=T5Y+5){
				MovTimeFromLog_t[4][a_datacount_t] = MT1;
				FindAllTarget++;
			}


			if(FindAllTarget ==5){
				a_datacount_t++;
				FindAllTarget =0;
			}	
			a_datacount_tmp++;
		}		
		a_headercount_t++;
		
	}


	fclose(m_FilePointer_t);
	
	for(idxi_t=0 ; idxi_t<5 ; idxi_t++){
		for (a_datacount_t=0 ; a_datacount_t<20 ; a_datacount_t++){
			MovTimeFromLog_t[idxi_t][a_datacount_t] = MovTimeFromLog_t[idxi_t][a_datacount_t];
			printf("TargetNum:%d, MT:[%f]\n",idxi_t,MovTimeFromLog_t[idxi_t][a_datacount_t]);
		}
	}



	//////////////////////////////////////////////////////////////////////////
	// NEW : Calculate each mean from the log file again.
	for (a_datacount_t=0 ; a_datacount_t<20 ; a_datacount_t++){
		Mean1 += MovTimeFromLog_t[0][a_datacount_t];			
		Mean2 += MovTimeFromLog_t[1][a_datacount_t];			
		Mean3 += MovTimeFromLog_t[2][a_datacount_t];			
		Mean4 += MovTimeFromLog_t[3][a_datacount_t];			
		Mean5 += MovTimeFromLog_t[4][a_datacount_t];			
	}
	Mean1 = Mean1/20;
	Mean2 = Mean2/20;
	Mean3 = Mean3/20;
	Mean4 = Mean4/20;
	Mean5 = Mean5/20;

	printf("check7-new Mean1:%4.2f\n",Mean1);
	printf("check7-new Mean2:%4.2f\n",Mean2);
	printf("check7-new Mean3:%4.2f\n",Mean3);
	printf("check7-new Mean4:%4.2f\n",Mean4);
	printf("check7-new Mean5:%4.2f\n",Mean5);
	//////////////////////////////////////////////////////////////////////////

	
	//HP-----------------------------
	for(i=0; i<20; i++){
		total1=(MovTimeFromLog_t[0][i]-Mean1)*(MovTimeFromLog_t[0][i]-Mean1);
		sum1 +=total1;
		total2=(MovTimeFromLog_t[1][i]-Mean2)*(MovTimeFromLog_t[1][i]-Mean2);
		sum2 +=total2;
		total3=(MovTimeFromLog_t[2][i]-Mean3)*(MovTimeFromLog_t[2][i]-Mean3);
		sum3 +=total3;
		total4=(MovTimeFromLog_t[3][i]-Mean4)*(MovTimeFromLog_t[3][i]-Mean4);
		sum4 +=total4;
		total5=(MovTimeFromLog_t[4][i]-Mean5)*(MovTimeFromLog_t[4][i]-Mean5);
		sum5 +=total5;
	}
	total1=sum1/20; total2=sum2/20; total3=sum3/20; total4=sum4/20; total5=sum5/20;
	//0.5 std
	stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
	//1 std
	stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);
	//2 std
	stdev31=2*sqrt(total1); stdev32=2*sqrt(total2); stdev33=2*sqrt(total3); stdev34=2*sqrt(total4); stdev35=2*sqrt(total5);

	printf("After 100 trials!\n");
	printf("This means should be same with Check3!!!\n");
	printf("check4-Mean1:%4.2f,stdev1:%4.2f,stdev21:%4.2f\n",Mean1,stdev1,stdev21);
	printf("check4-Mean2:%4.2f,stdev2:%4.2f,stdev22:%4.2f\n",Mean2,stdev2,stdev22);
	printf("check4-Mean3:%4.2f,stdev3:%4.2f,stdev23:%4.2f\n",Mean3,stdev3,stdev23);
	printf("check4-Mean4:%4.2f,stdev4:%4.2f,stdev24:%4.2f\n",Mean4,stdev4,stdev24);
	printf("check4-Mean5:%4.2f,stdev5:%4.2f,stdev25:%4.2f\n",Mean5,stdev5,stdev25);
	printf("Compared with check1: will be defferent!!!\n");
	//HP------------------------------


	//HP1 NEW------------------------------

	for( idxi_t=0 ; idxi_t < 5 ; idxi_t++){
		NewNewMean[idxi_t]=0.0;
	}	

	outlier1P=Mean1+stdev31; outlier1N=Mean1-stdev31;
	outlier2P=Mean2+stdev32; outlier2N=Mean2-stdev32;
	outlier3P=Mean3+stdev33; outlier3N=Mean3-stdev33;
	outlier4P=Mean4+stdev34; outlier4N=Mean4-stdev34;
	outlier5P=Mean5+stdev35; outlier5N=Mean5-stdev35;

	for(i=0; i<20; i++){
		if (MovTimeFromLog_t[0][i]> outlier1N && MovTimeFromLog_t[0][i]< outlier1P){
			NewNewMean[0]+=MovTimeFromLog_t[0][i];
			jj1=jj1+1;
		}
       if (MovTimeFromLog_t[1][i]> outlier2N && MovTimeFromLog_t[1][i]< outlier2P){
			NewNewMean[1]+=MovTimeFromLog_t[1][i];
			jj2=jj2+1;
		}
		if (MovTimeFromLog_t[2][i]> outlier3N && MovTimeFromLog_t[2][i]< outlier3P){
			NewNewMean[2]+=MovTimeFromLog_t[2][i];
			jj3=jj3+1;
		}
		if (MovTimeFromLog_t[3][i]> outlier4N && MovTimeFromLog_t[3][i]< outlier4P){
			NewNewMean[3]+=MovTimeFromLog_t[3][i];
			jj4=jj4+1;
		}
		if (MovTimeFromLog_t[4][i]> outlier5N && MovTimeFromLog_t[4][i]< outlier5P){
			NewNewMean[4]+=MovTimeFromLog_t[4][i];
			jj5=jj5+1;
		}
	}

	Mean1=0, Mean2=0, Mean3=0, Mean4=0, Mean5=0;
	total1=0, total2=0, total3=0, total4=0, total5=0;
	sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
	stdev1=0, stdev2=0, stdev3=0, stdev4=0, stdev5=0;
	stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;
	//stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;


	Mean1=NewNewMean[0]/jj1;
	Mean2=NewNewMean[1]/jj2;
	Mean3=NewNewMean[2]/jj3;
	Mean4=NewNewMean[3]/jj4;
	Mean5=NewNewMean[4]/jj5;


	for(i=0; i<20; i++){
		if (MovTimeFromLog_t[0][i]> outlier1N && MovTimeFromLog_t[0][i]< outlier1P){
			total1=(MovTimeFromLog_t[0][i]-Mean1)*(MovTimeFromLog_t[0][i]-Mean1);
			sum1 +=total1;
		}
		if (MovTimeFromLog_t[1][i]> outlier2N && MovTimeFromLog_t[1][i]< outlier2P){
			total2=(MovTimeFromLog_t[1][i]-Mean2)*(MovTimeFromLog_t[1][i]-Mean2);
			sum2 +=total2;
		}
		if (MovTimeFromLog_t[2][i]> outlier3N && MovTimeFromLog_t[2][i]< outlier3P){
			total3=(MovTimeFromLog_t[2][i]-Mean3)*(MovTimeFromLog_t[2][i]-Mean3);
			sum3 +=total3;
		}
		if (MovTimeFromLog_t[3][i]> outlier4N && MovTimeFromLog_t[3][i]< outlier4P){
			total4=(MovTimeFromLog_t[3][i]-Mean4)*(MovTimeFromLog_t[3][i]-Mean4);
			sum4 +=total4;
		}
		if (MovTimeFromLog_t[4][i]> outlier5N && MovTimeFromLog_t[4][i]< outlier5P){
			total5=(MovTimeFromLog_t[4][i]-Mean5)*(MovTimeFromLog_t[4][i]-Mean5);
			sum5 +=total5;
		}
	}

	total1=sum1/jj1; total2=sum2/jj2; total3=sum3/jj3; total4=sum4/jj4; total5=sum5/jj5;
	//0.5 std
	stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
	//1 std
	stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);
	//2 std
	//stdev31=2*sqrt(total1); stdev32=2*sqrt(total2); stdev33=2*sqrt(total3); stdev34=2*sqrt(total4); stdev35=2*sqrt(total5);

	//printf("After 100 trials!\n");
	printf("Remove Outliers based on 2 std\n");
	printf("check5-jj1:%d,jj2:%d,jj3:%d,jj4:%d,jj5:%d\n",jj1,jj2,jj3,jj4,jj5);
	printf("check5-Mean1:%4.2f,stdev1:%4.2f,stdev21:%4.2f\n",Mean1,stdev1,stdev21);
	printf("check5-Mean2:%4.2f,stdev2:%4.2f,stdev22:%4.2f\n",Mean2,stdev2,stdev22);
	printf("check5-Mean3:%4.2f,stdev3:%4.2f,stdev23:%4.2f\n",Mean3,stdev3,stdev23);
	printf("check5-Mean4:%4.2f,stdev4:%4.2f,stdev24:%4.2f\n",Mean4,stdev4,stdev24);
	printf("check5-Mean5:%4.2f,stdev5:%4.2f,stdev25:%4.2f\n",Mean5,stdev5,stdev25);
	printf("Compared with check1: will be defferent!!!\n");

	//HP1------------------------------

	saveMTtoFile_t(MTFILENAME_T);

	a_datacount_t=0;

}
//////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------



void InitializeVariables(){
	cnt_trial = 1;
	cnt_target = 1;
	cnt_calib = 1;
	no_successful_target=0;
	target_displayed_gui=0;
	setup_condition=1;
	session_score=0;
}
 
int ifileclean=0;


// #1
int loadparamGeneral(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	int mode=1;

	/* initialize */
	i = 0;
	memset(g_s_init, '\0', sizeof(fpos));
	memset(g_s2_init, '\0', sizeof(fpos));
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);

	/* read a configuration file */
	fp = fopen(genFCONFIG_PARAM, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		dprintf(1, "ptr = %s\n", ptr);
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	for (i = 0; strlen(bufline[i]) > 0; i++) {
		ptrtok = strtok(bufline[i], "\t");
		if (ptrtok != NULL) {
			dprintf(1, "ptrtok = %s\n", ptrtok);
			if (strcmp(ptrtok, "MAXTRIAL") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtrial = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FREE") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					if (setup_condition==1)
						no_desired_target = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FORCED") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						maxtarget = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "WAITTIME") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						waitTime = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_test[0] = atof(ptrtok);
			}
			// HP 
			else if (strcmp(ptrtok, "TARGET_COLOR_BEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_best[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_OK") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ok[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_HARDER") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_harder[0] = atof(ptrtok);
			}
		}
	}

	dprintf(1, "maxtrial  = %d\n", maxtrial);
	dprintf(1, "maxtarget = %d\n", maxtarget);
	dprintf(1, "t_calib = %f\n", t_calib[0]);
	dprintf(1, "t_ready = %f\n", t_ready[0]);
	dprintf(1, "t_test  = %f\n", t_test[0]);
	//HP
	dprintf(1, "t_best  = %f\n", t_best[0]);
	dprintf(1, "t_ok  = %f\n", t_ok[0]);
	dprintf(1, "t_harder  = %f\n", t_harder[0]);


	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	
	
	//HP
	/* read a test schedule */
	fp = fopen(genFCONFIG_SCHED, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);




	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		g_sched[i++] = atoi(ptr);
		ptr = strtok(NULL, "\n");
	}

	dprintf(1, "g_sched=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, " %d", g_sched[i]);
	}
	dprintf(1, "\n");


	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	/* read target pos_testitions */
	fp = fopen(genFCONFIG_POS, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	char Temp[255];

	for (i = 0; strlen(bufline[i]) > 0; i++) {
// 		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][0]);
// 		g_pos_test[i][0] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][1] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][1]);
// 		g_pos_test[i][1] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][2] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][2]);
// 		g_pos_test[i][2] = atof(Temp);
//		memset(Temp, '\0', 255);

		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
		g_pos_test[i][1] = atof(strtok(NULL, ","));
		g_pos_test[i][2] = atof(strtok(NULL, ","));

	}

	dprintf(1, "g_pos_test=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, "(%.2f,%.2f,%.2f)",
				g_pos_test[i][0], g_pos_test[i][1], g_pos_test[i][2]);
	}
	dprintf(1, "\n");

	// if (access(generalDIR, R_OK) < 0)
	// 	mkdir(LOGDIRgen, 0777);

	return 0;
}

// #1
int loadparam(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	int mode=1;

	/* initialize */
	i = 0;
	memset(g_s_init, '\0', sizeof(fpos));
	memset(g_s2_init, '\0', sizeof(fpos));
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);

	/* read a configuration file */
	fp = fopen(FCONFIG_PARAM, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		dprintf(1, "ptr = %s\n", ptr);
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	for (i = 0; strlen(bufline[i]) > 0; i++) {
		ptrtok = strtok(bufline[i], "\t");
		if (ptrtok != NULL) {
			dprintf(1, "ptrtok = %s\n", ptrtok);
			if (strcmp(ptrtok, "MAXTRIAL") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtrial = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FREE") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					if (setup_condition==1)
						no_desired_target = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FORCED") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						maxtarget = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "WAITTIME") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						waitTime = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_test[0] = atof(ptrtok);
			}
			// HP 
			else if (strcmp(ptrtok, "TARGET_COLOR_BEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_best[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_OK") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ok[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_HARDER") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_harder[0] = atof(ptrtok);
			}
		}
	}

	//add all params to file descriptor
	dprintf(1, "maxtrial  = %d\n", maxtrial);
	dprintf(1, "maxtarget = %d\n", maxtarget);
	dprintf(1, "t_calib = %f\n", t_calib[0]);
	dprintf(1, "t_ready = %f\n", t_ready[0]);
	dprintf(1, "t_test  = %f\n", t_test[0]);
	//HP
	dprintf(1, "t_best  = %f\n", t_best[0]);
	dprintf(1, "t_ok  = %f\n", t_ok[0]);
	dprintf(1, "t_harder  = %f\n", t_harder[0]);


	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	
	
	//HP
	/* read a test schedule */
	fp = fopen(FCONFIG_SCHED, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);




	/* extract each parameter 
		gets integers from file which have were psuedo-randomly generated once*/
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		g_sched[i++] = atoi(ptr);
		ptr = strtok(NULL, "\n");
	}

	//add to file descriptor
	dprintf(1, "g_sched=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, " %d", g_sched[i]);
	}
	dprintf(1, "\n");


	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	//get the coordinates of where the targets will be from config_pos.txt file
	//in accuracy these are simply along the x axis. in general they are in a cone shape with 20 degree seperations twice on both sides - 5 lines radially from origin point
	/* read target pos_testitions */
	fp = fopen(FCONFIG_POS, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	
	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	char Temp[255];

	for (i = 0; strlen(bufline[i]) > 0; i++) {
// 		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][0]);
// 		g_pos_test[i][0] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][1] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][1]);
// 		g_pos_test[i][1] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][2] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][2]);
// 		g_pos_test[i][2] = atof(Temp);
//		memset(Temp, '\0', 255);

		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
		g_pos_test[i][1] = atof(strtok(NULL, ","));
		g_pos_test[i][2] = atof(strtok(NULL, ","));
		g_pos_test[i][3] = atof(strtok(NULL, ",")); //not sure exactly what this fourth value is. as it is not XYZ and is either .07, .14 or .35. perhaps it is the size of the target?

	}

	//add to file descriptor
	dprintf(1, "g_pos_test=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, "(%.2f,%.2f,%.2f,%.2f)",
				g_pos_test[i][0], g_pos_test[i][1], g_pos_test[i][2], g_pos_test[i][3]);
	}
	dprintf(1, "\n");

	//make "../data/subjectcode123"
	if (access(LOGDIRparent, R_OK) < 0)
		mkdir(LOGDIRparent, 0777);

	//make "../data/subjectcode123/firstHand"
	if (access(tempFirstHand, R_OK) < 0)
		mkdir(tempFirstHand, 0777);
	//make "../data/subjectcode123/secondHand"
	if (access(tempSecondHand, R_OK) < 0)
		mkdir(tempSecondHand, 0777);


	//make "../data/subjectcode123/firstHand/Accuracy"
	if (access(accuracyDIR, R_OK) < 0)
		mkdir(accuracyDIR, 0777);
	//make "../data/subjectcode123/firstHand/General"
	if (access(generalDIR, R_OK) < 0)
		mkdir(generalDIR, 0777);

	//make "../data/subjectcode123/secondHand/Accuracy"
	if (access(accuracyDIR2, R_OK) < 0)
		mkdir(accuracyDIR2, 0777);
	//make "../data/subjectcode123/secondHand/General"
	if (access(generalDIR2, R_OK) < 0)
		mkdir(generalDIR2, 0777);
	return 0;
}


// #1.1 - HP added	
int loadparam_1(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	int mode=2;

	/* initialize */
	i = 0;
	memset(g_s_init, '\0', sizeof(fpos));
	memset(g_s2_init, '\0', sizeof(fpos));
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);

	/* read a configuration file */
	fp = fopen(FCONFIG_PARAM_T1, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		dprintf(1, "ptr = %s\n", ptr);
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	for (i = 0; strlen(bufline[i]) > 0; i++) {
		ptrtok = strtok(bufline[i], "\t");
		if (ptrtok != NULL) {
			dprintf(1, "ptrtok = %s\n", ptrtok);
			if (strcmp(ptrtok, "MAXTRIAL") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtrial = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FREE") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					if (setup_condition==1)
						no_desired_target = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FORCED") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						maxtarget = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "WAITTIME") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						waitTime = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_test[0] = atof(ptrtok);
			}
			// HP 
			else if (strcmp(ptrtok, "TARGET_COLOR_BEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_best[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_OK") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ok[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_HARDER") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_harder[0] = atof(ptrtok);
			}
		}
	}

	dprintf(1, "maxtrial  = %d\n", maxtrial);
	dprintf(1, "maxtarget = %d\n", maxtarget);
	dprintf(1, "t_calib = %f\n", t_calib[0]);
	dprintf(1, "t_ready = %f\n", t_ready[0]);
	dprintf(1, "t_test  = %f\n", t_test[0]);
	//HP
	dprintf(1, "t_best  = %f\n", t_best[0]);
	dprintf(1, "t_ok  = %f\n", t_ok[0]);
	dprintf(1, "t_harder  = %f\n", t_harder[0]);



	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	
	
	//HP
	/* read a test schedule */
	fp = fopen(FCONFIG_SCHED_T1, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);




	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		g_sched[i++] = atoi(ptr);
		ptr = strtok(NULL, "\n");
	}

	dprintf(1, "g_sched=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, " %d", g_sched[i]);
	}
	dprintf(1, "\n");


	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	/* read target pos_testitions */
	fp = fopen(FCONFIG_POS, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	char Temp[255];

	for (i = 0; strlen(bufline[i]) > 0; i++) {
// 		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][0]);
// 		g_pos_test[i][0] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][1] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][1]);
// 		g_pos_test[i][1] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][2] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][2]);
// 		g_pos_test[i][2] = atof(Temp);
//		memset(Temp, '\0', 255);

		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
		g_pos_test[i][1] = atof(strtok(NULL, ","));
		g_pos_test[i][2] = atof(strtok(NULL, ","));

	}

	dprintf(1, "g_pos_test=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, "(%.2f,%.2f,%.2f)",
				g_pos_test[i][0], g_pos_test[i][1], g_pos_test[i][2]);
	}
	dprintf(1, "\n");

	if (access(LOGDIR, R_OK) < 0)
		mkdir(LOGDIR, 0777);

	return 0;
}
// #1 -end (same)



// #1.1 - HP added	
int loadparam_2(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	int mode=3;

	/* initialize */
	i = 0;
	memset(g_s_init, '\0', sizeof(fpos));
	memset(g_s2_init, '\0', sizeof(fpos));
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);

	/* read a configuration file */
	fp = fopen(FCONFIG_PARAM_B, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		dprintf(1, "ptr = %s\n", ptr);
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	for (i = 0; strlen(bufline[i]) > 0; i++) {
		ptrtok = strtok(bufline[i], "\t");
		if (ptrtok != NULL) {
			dprintf(1, "ptrtok = %s\n", ptrtok);
			if (strcmp(ptrtok, "MAXTRIAL") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtrial = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FREE") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					if (setup_condition==1)
						no_desired_target = atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTARGET_FORCED") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						maxtarget = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "WAITTIME") == 0) {
				ptrtok = strtok(NULL,"\t");
				if (ptrtok != NULL)
					if (setup_condition==0)
						waitTime = atoi (ptrtok);
			} 
			else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} 
			else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_test[0] = atof(ptrtok);
			}
			// HP 
			else if (strcmp(ptrtok, "TARGET_COLOR_BEST") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_best[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_OK") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ok[0] = atof(ptrtok);
			}
			else if (strcmp(ptrtok, "TARGET_COLOR_HARDER") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_harder[0] = atof(ptrtok);
			}
		}
	}

	dprintf(1, "maxtrial  = %d\n", maxtrial);
	dprintf(1, "maxtarget = %d\n", maxtarget);
	dprintf(1, "t_calib = %f\n", t_calib[0]);
	dprintf(1, "t_ready = %f\n", t_ready[0]);
	dprintf(1, "t_test  = %f\n", t_test[0]);
	//HP
	dprintf(1, "t_best  = %f\n", t_best[0]);
	dprintf(1, "t_ok  = %f\n", t_ok[0]);
	dprintf(1, "t_harder  = %f\n", t_harder[0]);



	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	
	
	//HP
	/* read a test schedule */
	fp = fopen(FCONFIG_SCHED_B, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);




	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		g_sched[i++] = atoi(ptr);
		ptr = strtok(NULL, "\n");
	}

	dprintf(1, "g_sched=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, " %d", g_sched[i]);
	}
	dprintf(1, "\n");


	/* initialize */
	memset(buf, '\0', MAXBUFSZ);
	memset(bufline, '\0', MAXPARAM*MAXLINESZ);
	i = 0;

	/* read target pos_testitions */
	fp = fopen(FCONFIG_POS, "r");
	if ( (recv = fread(buf, 1, MAXBUFSZ, fp)) < 0)
		dprintf(0, "read failed: %s\n", strerror(errno));
	fclose(fp);

	/* extract each parameter */
	ptr = strtok(buf, "\n");
	while (ptr != NULL) {
		sprintf(bufline[i++], "%s\n", ptr);
		ptr = strtok(NULL, "\n");
	}

	/* extract each value */
	char Temp[255];

	for (i = 0; strlen(bufline[i]) > 0; i++) {
// 		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][0]);
// 		g_pos_test[i][0] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][1] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][1]);
// 		g_pos_test[i][1] = atof(Temp);
// 		memset(Temp, '\0', 255);
// 
// 		g_pos_test[i][2] = atof(strtok(NULL, ","));
// 		sprintf(Temp, "%.2f", g_pos_test[i][2]);
// 		g_pos_test[i][2] = atof(Temp);
//		memset(Temp, '\0', 255);

		g_pos_test[i][0] = atof(strtok(bufline[i], ","));
		g_pos_test[i][1] = atof(strtok(NULL, ","));
		g_pos_test[i][2] = atof(strtok(NULL, ","));
		//HP -2018/2018
		g_pos_test[i][3] = atof(strtok(NULL, ","));


	}

	dprintf(1, "g_pos_test=\n");
	for (i = 0; i < maxtarget; i++) {
		dprintf(1, "(%.2f,%.2f,%.2f)",
				g_pos_test[i][0], g_pos_test[i][1], g_pos_test[i][2]);
	}
	dprintf(1, "\n");

	if (access(LOGDIR, R_OK) < 0)
		mkdir(LOGDIR, 0777);

	return 0;
}
// #1 -end (same)


void CreateNewLogFile(){

	cnt_trial = 1;
	cnt_target = 1;


	// Directory name Initialize
	char LOGDIRtmp[255]="../data/";
	char LOGFILEtmp[255]="";
	// Directory name Initialize

	int temp; 
	char s[256]="";
	char fname[256]="";
	char sc[256]="";
	time_t tm=time(NULL);
	struct tm *tmp;
	tmp = localtime(&tm);
	strftime(s, 100, "-%m%d%Y-%H%M%S", tmp);
	//strftime(s, 100, "-%m%d%Y-%H%M%S", tmp);

	int Previouscode=0;
	if(Previouscode){
		printf("Subject Code? ");
		scanf("%255s",sc);
		//gets(sc); cant use gets because it is outdated and dangerous to overflow
		//255 characters so i discludes the /n or /0 that gets entered to a string 
			// after a scanf call


		printf("Forced(0)? or Free(1)? ");
		scanf("%d",&setup_condition);
	}
	else{
		printf("Subject Code? ");
		scanf("%s",sc);
		
		printf("Forced(0)? or Free(1)? ");
		scanf("%d",&setup_condition);
	}
	
	Previouscode=1;
	if(Previouscode){
		strncat(LOGDIRtmp,sc,strlen(sc));
	}
	else{
		if (argcCopy > 1)
		{
			strncat(LOGDIRtmp,argvCopy[1],strlen(argvCopy[1]));
		}
		else
		{
			if (strlen(sc)==0)
			{
				char temp2[255]="log";
				strncat(LOGDIRtmp,temp2,strlen(temp2));
			}
			else
			{
				strncat(LOGDIRtmp,sc,strlen(sc));
			}
		}
	}

    strncat(LOGDIRtmp,s,strlen(s));
	strncat(LOGFILEtmp,LOGDIRtmp,strlen(LOGDIRtmp));
	strncat(LOGFILEtmp,temp_characc,strlen(temp_characc));


	for (ifileclean=0; ifileclean <255 ; ifileclean++){
		LOGDIR[ifileclean]='\0';
		LOGFILEacc[ifileclean]='\0';
		
	}

		
	
	strncpy (LOGDIR,LOGDIRtmp,strlen(LOGDIRtmp));
	strncpy (LOGFILEacc,LOGFILEtmp,strlen(LOGFILEtmp));

	
	if (mode==2){
        loadparam_1(); // for training
	}
	else if (mode==3){
        loadparam_2(); // for BART
	}
	else {
		loadparam();
	}


	printf("The LOG directory is %s\n",LOGDIR);
	printf("The LOG FILE is %s\n",LOGFILEacc);
    printf("# of Targets = %d\n\n",maxtarget);
	
	
	//New log file is genereated in here
	if (g_logfp == NULL) {
		if ((g_logfp = fopen(LOGFILEacc, "a+")) == NULL) {
			dprintf(0, "fopen error in create: %s\n", strerror(errno));
		} else {
			sprintf(s, "#%s \t%s %s %s %s %s %s\n",
				"Trial", "Target Position", "Movement Time",
				"Error", "Hit", "Hand", "Feedback");
			fwrite(s, 1, strlen(s), g_logfp);
			sprintf(s, "%s\n",
				"#================================================================");
			fwrite(s, 1, strlen(s), g_logfp);
		}
	}
	
}








/* calculate the conversion matrix value */
void calib_screen()
{
	//new calibration
	// Iteration 1: 123
	double p1x=0.0,p1y=0.0, p1z=0.0, p2x=0.0, p2y=0.0, p2z=0.0, p3x=0.0, p3y=0.0, p3z=0.0, p4x=0.0, p4y=0.0, p4z=0.0;
	double p1_x=0.0, p1_y=0.0, p1_z=0.0, p2_x=0.0, p2_y=0.0, p2_z=0.0, p3_x=0.0, p3_y=0.0, p3_z=0.0, p4_x=0.0, p4_y=0.0, p4_z=0.0;
	int p1=0, p2=0, p3=0;
	int i=0;
	for (i=1 ; i <= 13 ; i++){

		if(i==1){
			p1=1, p2=2, p3=3;
		}
		else if(i==2){
			p1=1, p2=3, p3=4;
		}
		else if(i==3){
			p1=1, p2=2, p3=5;
		}
		else if(i==4){
			p1=2, p2=3, p3=5;
		}
		else if(i==5){
			p1=3, p2=4, p3=5;
		}
		else if(i==6){
			p1=1, p2=4, p3=5;
		}
		else if(i==7){
			p1=6, p2=7, p3=8;
		}
		else if(i==8){
			p1=7, p2=8, p3=9;
		}
		else if(i==9){
			p1=9, p2=6, p3=7;
		}
		else if(i==10){
			p1=5, p2=6, p3=7;
		}
		else if(i==11){
			p1=5, p2=7, p3=8;
		}
		else if(i==12){
			p1=5, p2=8, p3=9;
		}
		else if(i==13){
			p1=5, p2=6, p3=9;
		}


		p1x=g_calibt[1][p1];
		p1y=g_calibt[2][p1];
		p1z=g_calibt[3][p1];

		p2x=g_calibt[1][p2];
		p2y=g_calibt[2][p2];
		p2z=g_calibt[3][p2];

		p3x=g_calibt[1][p3];
		p3y=g_calibt[2][p3];
		p3z=g_calibt[3][p3];

		p1_x=g_calibs[1][p1];
		p1_y=g_calibs[2][p1];
		p1_z=g_calibs[3][p1];

		p2_x=g_calibs[1][p2];
		p2_y=g_calibs[2][p2];
		p2_z=g_calibs[3][p2];

		p3_x=g_calibs[1][p3];
		p3_y=g_calibs[2][p3];
		p3_z=g_calibs[3][p3];


		R11 = -(p2_y*p3x-p2_y*p1x-p3_y*p2x+p1x*p3_y+p1_y*p2x-p1_y*p3x)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		R12 = (p1_x*p2x-p1_x*p3x-p2_x*p1x-p2x*p3_x+p3_x*p1x+p2_x*p3x)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		R21 = -(p3y*p2_y-p1y*p2_y-p3_y*p2y+p3_y*p1y+p1_y*p2y-p1_y*p3y)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		R22 = (p2y*p1_x-p3y*p1_x-p2_x*p1y+p3_x*p1y-p2y*p3_x+p2_x*p3y)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		d1 = (p3x*p1_x*p2_y-p3x*p2_x*p1_y-p3_x*p1x*p2_y-p3_y*p1_x*p2x+p3_y*p2_x*p1x+p3_x*p1_y*p2x)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		d2 = (p3y*p1_x*p2_y-p3y*p2_x*p1_y-p3_x*p1y*p2_y-p3_y*p1_x*p2y+p3_y*p2_x*p1y+p3_x*p1_y*p2y)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);

			 
		Calibraton_Matrix[0] += R11;
		Calibraton_Matrix[1] += R12;
		Calibraton_Matrix[2] += R21;
		Calibraton_Matrix[3] += R22;
		Calibraton_Matrix[4] += d1;
		Calibraton_Matrix[5] += d2;
	}

	i=0;
	for (i=0 ; i < 6 ; i++){
		m_CalibResult[i] = Calibraton_Matrix[i]/13;
	}
}




//////////////////////////////////////////////////////////////////////////
// Sensor2 Calib
/* calculate the conversion matrix value */
void calib_screen2()
{

	//new calibration
	// Iteration 1: 123
	double p1x=0.0,p1y=0.0, p1z=0.0, p2x=0.0, p2y=0.0, p2z=0.0, p3x=0.0, p3y=0.0, p3z=0.0, p4x=0.0, p4y=0.0, p4z=0.0;
	double p1_x=0.0, p1_y=0.0, p1_z=0.0, p2_x=0.0, p2_y=0.0, p2_z=0.0, p3_x=0.0, p3_y=0.0, p3_z=0.0, p4_x=0.0, p4_y=0.0, p4_z=0.0;
	int p1=0, p2=0, p3=0;
	int i=0;
	for (i=1 ; i <= 13 ; i++){

		if(i==1){
			p1=1, p2=2, p3=3;
		}
		else if(i==2){
			p1=1, p2=3, p3=4;
		}
		else if(i==3){
			p1=1, p2=2, p3=5;
		}
		else if(i==4){
			p1=2, p2=3, p3=5;
		}
		else if(i==5){
			p1=3, p2=4, p3=5;
		}
		else if(i==6){
			p1=1, p2=4, p3=5;
		}
		else if(i==7){
			p1=6, p2=7, p3=8;
		}
		else if(i==8){
			p1=7, p2=8, p3=9;
		}
		else if(i==9){
			p1=9, p2=6, p3=7;
		}
		else if(i==10){
			p1=5, p2=6, p3=7;
		}
		else if(i==11){
			p1=5, p2=7, p3=8;
		}
		else if(i==12){
			p1=5, p2=8, p3=9;
		}
		else if(i==13){
			p1=5, p2=6, p3=9;
		}


		p1x=g_calibt[1][p1];
		p1y=g_calibt[2][p1];
		p1z=g_calibt[3][p1];

		p2x=g_calibt[1][p2];
		p2y=g_calibt[2][p2];
		p2z=g_calibt[3][p2];

		p3x=g_calibt[1][p3];
		p3y=g_calibt[2][p3];
		p3z=g_calibt[3][p3];

		p1_x=g_calibs[1][p1];
		p1_y=g_calibs[2][p1];
		p1_z=g_calibs[3][p1];

		p2_x=g_calibs[1][p2];
		p2_y=g_calibs[2][p2];
		p2_z=g_calibs[3][p2];

		p3_x=g_calibs[1][p3];
		p3_y=g_calibs[2][p3];
		p3_z=g_calibs[3][p3];


		R11 = -(p2_y*p3x-p2_y*p1x-p3_y*p2x+p1x*p3_y+p1_y*p2x-p1_y*p3x)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		R12 = (p1_x*p2x-p1_x*p3x-p2_x*p1x-p2x*p3_x+p3_x*p1x+p2_x*p3x)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		R21 = -(p3y*p2_y-p1y*p2_y-p3_y*p2y+p3_y*p1y+p1_y*p2y-p1_y*p3y)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		R22 = (p2y*p1_x-p3y*p1_x-p2_x*p1y+p3_x*p1y-p2y*p3_x+p2_x*p3y)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		d1 = (p3x*p1_x*p2_y-p3x*p2_x*p1_y-p3_x*p1x*p2_y-p3_y*p1_x*p2x+p3_y*p2_x*p1x+p3_x*p1_y*p2x)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);
		d2 = (p3y*p1_x*p2_y-p3y*p2_x*p1_y-p3_x*p1y*p2_y-p3_y*p1_x*p2y+p3_y*p2_x*p1y+p3_x*p1_y*p2y)/(p2_y*p1_x-p2_x*p1_y-p3_x*p2_y-p3_y*p1_x+p3_y*p2_x+p3_x*p1_y);

			 
		Calibraton_Matrix2[0] += R11;
		Calibraton_Matrix2[1] += R12;
		Calibraton_Matrix2[2] += R21;
		Calibraton_Matrix2[3] += R22;
		Calibraton_Matrix2[4] += d1;
		Calibraton_Matrix2[5] += d2;
	}

	i=0;
	for (i=0 ; i < 6 ; i++){
		m_CalibResult2[i] = Calibraton_Matrix2[i]/13;
	}
}
//////////////////////////////////////////////////////////////////////////


int check_sensor_pos()
{
	/*
	int temp_code=0;
	// Jeong's original convert the coordinate 
	g_s[0] = g_matrix[1][1]*g_pos_data_s[0] +
		   g_matrix[1][2]*g_pos_data_s[1] +
		   g_matrix[1][3]*g_pos_data_s[2];
	g_s[1] = g_matrix[2][1]*g_pos_data_s[0] +
		   g_matrix[2][2]*g_pos_data_s[1] +
		   g_matrix[2][3]*g_pos_data_s[2];
	g_s[2] = g_matrix[3][1]*g_pos_data_s[0] +
		   g_matrix[3][2]*g_pos_data_s[1] +
		   g_matrix[3][3]*g_pos_data_s[2];

	g_s2[0] = g_matrix[1][1]*g_pos_data_s2[0] +
		   g_matrix[1][2]*g_pos_data_s2[1] +
		   g_matrix[1][3]*g_pos_data_s2[2];
	g_s2[1] = g_matrix[2][1]*g_pos_data_s2[0] +
		   g_matrix[2][2]*g_pos_data_s2[1] +
		   g_matrix[2][3]*g_pos_data_s2[2];
	g_s2[2] = g_matrix[3][1]*g_pos_data_s2[0] +
		   g_matrix[3][2]*g_pos_data_s2[1] +
		   g_matrix[3][3]*g_pos_data_s2[2];



	if (mode == 3){

		// reached by the first sensor 
		// (fabsf(g_s[2] - g_pos_data_t[2]) < 2)
		if ( hypotf(g_s[0]-g_pos_data_t[0], g_s[1]-g_pos_data_t[1]) < t_test[0] )
			temp_code+=1;

		// reached by the second sensor
		if ( hypotf(g_s2[0]-g_pos_data_t[0], g_s2[1]-g_pos_data_t[1]) < t_test[0])
			temp_code+=10;
	}
	else{
		if ( hypotf(g_s[0]-g_pos_data_t[0], g_s[1]-g_pos_data_t[1]) < t_test[0] || hypotf(g_s2[0]-g_pos_data_t[0], g_s2[1]-g_pos_data_t[1]) < t_test[0]){
			temp_code+=11;
		}

	}

	return temp_code;
	*/


	//working Calibration for miniBIRD coordinates to our coordinates that we use

	int temp_code=0;
	// new calibration
	// g_pos_data_s[0] == x
	// g_pos_data_s{1] == y
	g_s[0] = ( m_CalibResult[0] * g_pos_data_s[0] + m_CalibResult[1] * g_pos_data_s[1] ) + m_CalibResult[4];
	g_s[1] = ( m_CalibResult[2] * g_pos_data_s[0] + m_CalibResult[3] * g_pos_data_s[1] ) + m_CalibResult[5];
	g_s[2] = g_pos_data_s[2];

	//////////////////////////////////////////////////////////////////////////
	// Sensor2 Calib
	g_s2[0] = ( m_CalibResult2[0] * g_pos_data_s2[0] + m_CalibResult2[1] * g_pos_data_s2[1]) + m_CalibResult2[4];
	g_s2[1] = ( m_CalibResult2[2] * g_pos_data_s2[0] + m_CalibResult2[3] * g_pos_data_s2[1]) + m_CalibResult2[5];
	g_s2[2] = g_pos_data_s2[2];
	//////////////////////////////////////////////////////////////////////////

// 	g_s[2]=g_pos_data_s[2];
//	g_s2[2]=g_pos_data_s2[2];
	

	printf("Calibrated S1: %f,%f,%f\n", g_s[0], g_s[1], g_s[2]);
	printf("Calibrated S2: %f,%f,%f\n\n", g_s2[0], g_s2[1], g_s2[2]);



// 	int temp_code=0;
// 	// new calibration
// 	g_s[0] = ( m_CalibResult[0] * g_pos_data_s[0] + m_CalibResult[1] * g_pos_data_s[1] + m_CalibResult[2] * g_pos_data_s[2] ) + m_CalibResult[9];
// 	g_s[1] = ( m_CalibResult[3] * g_pos_data_s[0] + m_CalibResult[4] * g_pos_data_s[1] + m_CalibResult[5] * g_pos_data_s[2] ) + m_CalibResult[10];
// 	g_s[2] = ( m_CalibResult[6] * g_pos_data_s[0] + m_CalibResult[7] * g_pos_data_s[1] + m_CalibResult[8] * g_pos_data_s[2] ) + m_CalibResult[11];
// 
// 	//////////////////////////////////////////////////////////////////////////
// 	// Sensor2 Calib
// 	g_s2[0] = ( m_CalibResult2[0] * g_pos_data_s2[0] + m_CalibResult2[1] * g_pos_data_s2[1] + m_CalibResult2[2] * g_pos_data_s2[2] ) + m_CalibResult2[9];
// 	g_s2[1] = ( m_CalibResult2[3] * g_pos_data_s2[0] + m_CalibResult2[4] * g_pos_data_s2[1] + m_CalibResult2[5] * g_pos_data_s2[2] ) + m_CalibResult2[10];
// 	g_s2[2] = ( m_CalibResult2[6] * g_pos_data_s2[0] + m_CalibResult2[7] * g_pos_data_s2[1] + m_CalibResult2[8] * g_pos_data_s2[2] ) + m_CalibResult2[11];
// 	//////////////////////////////////////////////////////////////////////////
// 
// 	g_s[2]=g_pos_data_s[2];
// 	g_s2[2]=g_pos_data_s2[2];
// 	
// 	//printf("%f,%f\n\n", g_s[2], g_s2[2]);



	/*
	float zDepth=0.25;
	if (mode == 3){

		// reached by the first sensor 
		if ( hypotf(g_s[0]-g_pos_data_t[0], g_s[1]-g_pos_data_t[1]) < t_test[0] )
			temp_code+=1;

		// reached by the second sensor 
		if ( hypotf(g_s2[0]-g_pos_data_t[0], g_s2[1]-g_pos_data_t[1]) < t_test[0] )
			temp_code+=10;
	}
	else{
		if ( (hypotf(g_s[0]-g_pos_data_t[0], g_s[1]-g_pos_data_t[1]) < t_test[0] )  || 
			 (hypotf(g_s2[0]-g_pos_data_t[0], g_s2[1]-g_pos_data_t[1]) < t_test[0]) ) {
			temp_code+=11;
		}

	}
*/


		

	float zDepth=-8.0;
	if (mode == 3){

		// reached by the first sensor 
		if ( hypotf(g_s[0]-g_pos_data_t[0], g_s[1]-g_pos_data_t[1]) < t_test[0] && g_s[2] >=  zDepth){
			temp_code+=1;
		}
		else{
			temp_code=0;
		}
		// reached by the second sensor 
		if ( hypotf(g_s2[0]-g_pos_data_t[0], g_s2[1]-g_pos_data_t[1]) < t_test[0] && g_s2[2] >=  zDepth){
			temp_code+=10;
		}
		else{
			temp_code=0;
		}
	}
	else{
		if ( (hypotf(g_s[0]-g_pos_data_t[0], g_s[1]-g_pos_data_t[1]) < t_test[0] && g_s[2] >= zDepth) 
				|| (hypotf(g_s2[0]-g_pos_data_t[0], g_s2[1]-g_pos_data_t[1]) < t_test[0] && g_s2[2] >=  zDepth)){
			temp_code+=11;
		}
		else{
			temp_code=0;
		}

	}
	


	return temp_code;
	

}



int check_hand_choice()
{
	float dist_right, dist_left; //don't care about the direction

	dist_right = hypotf(g_s[0]-g_pos_data_t[0], g_s[1]-g_pos_data_t[1]) ;
	dist_left = hypotf(g_s2[0]-g_pos_data_t[0], g_s2[1]-g_pos_data_t[1]) ;

	if (dist_right < dist_left)	//Cheol's selection (reverse to Jeong's)
		return 1;
	else if (dist_right == dist_left)
		return 0;
	else
		return -1; //2nd sensor is moving
}



void display_init()
{
	int i;
	struct timeval tv;
	struct timezone tz;

	gettimeofday(&tv, &tz);
	srandom((unsigned int)tv.tv_usec);
	g_pos_data_t[0] = 0;
	g_pos_data_t[1] = 0;
	g_pos_data_t[2] = -5;

	/* set the matrix to an identity matrix as default */
	g_matrix = matrix(1,3,1,3);
	for (i = 1; i <= 3; i++)
		g_matrix[i][i] = 1.0;

	// New Calib
	g_calibs = matrix(1,3,1,9);
	g_calibt = matrix(1,3,1,9);
	g_calibt[1][1]=-0.8;g_calibt[2][1]= 0.8;g_calibt[3][1]=-5.0;
	g_calibt[1][2]= 0.8;g_calibt[2][2]= 0.8;g_calibt[3][2]=-5.0;
	g_calibt[1][3]= 0.8;g_calibt[2][3]=-0.8;g_calibt[3][3]=-5.0;
	g_calibt[1][4]=-0.8;g_calibt[2][4]=-0.8;g_calibt[3][4]=-5.0;
	g_calibt[1][5]= 0.0;g_calibt[2][5]= 0.0;g_calibt[3][5]=-5.0;
	g_calibt[1][6]= 0.0;g_calibt[2][6]= 0.8;g_calibt[3][6]=-5.0;
	g_calibt[1][7]= 0.8;g_calibt[2][7]= 0.0;g_calibt[3][7]=-5.0;
	g_calibt[1][8]= 0.0;g_calibt[2][8]=-0.8;g_calibt[3][8]=-5.0;
	g_calibt[1][9]=-0.8;g_calibt[2][9]= 0.0;g_calibt[3][9]=-5.0;

	for (i = 1; i <= 9; i++)
		g_calibt[1][i] *= g_ratio;

}


int order =0;
int check =0;


/*
	######    ######      ##         ######## 
      ##      #     #     ##         ##
      ##      #     #     ##         ########
      ##      #     #     ##         ##      
    ######    ######      #######    ########

*/


//################################################################################################################################################################
//HP
void idledisplay()
{
	int temp,i, is_reached = 0;
	char s[256], fname[256];
	float error_s1, error_s2, final_error;



	/* display each scene */    //HP !!!
	switch (g_scene) {




	// case 1
	case SC_CALIB_INFO: //this is the base case, what it is set to in g_scene initialization
		display_init();
	break;

	// case 2
	case SC_CALIB_LU:
	
	// case 3
	case SC_CALIB_RU:
	
	// case 4
	case SC_CALIB_RB:
	
	// case 5
	case SC_CALIB_LB:
	
	// case 6
	case SC_CALIB_C:

	// New Calib
	case SC_CALIB_T:

	case SC_CALIB_R:

	case SC_CALIB_B:

	case SC_CALIB_L:
		
	break;
	//HP - end calibration




	//////////////////////////////////////////////////////////////////////////
	// Sensor2 Calib
	// case 1
	case SC_CALIB_INFO_LS:
	break;

	// case 2
	case SC_CALIB_LU_LS:
	
	// case 3
	case SC_CALIB_RU_LS:
	
	// case 4
	case SC_CALIB_RB_LS:
	
	// case 5
	case SC_CALIB_LB_LS:
	
	// case 6
	case SC_CALIB_C_LS:

	// New Calib
	case SC_CALIB_T_LS:

	case SC_CALIB_R_LS:

	case SC_CALIB_B_LS:

	case SC_CALIB_L_LS:

	break;
	//HP - end calibration
	//////////////////////////////////////////////////////////////////////////


	//accuracy test 1
	// case 7 : Pre-test
	/* start the test */
	case accSC_TEST_INFO:
	break;
	
	// case 8
	case accSC_TEST_INFO_1:
	break;
	
	
	// case 9	
	case accSC_TEST_READY1:
		printf("Finally ready_ 1 in bart_core.c\n");
		g_scene++;

		//HP !! important part!!! 
		memcpy(c,  &g_scene,  sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		target_displayed_gui=0;
		printf("2 Finally ready_ 1 in bart_core.c\n");
		
	break;



	// case 10
	case accSC_TEST_READY2:
	    flag_direction=0;
		selected_hand=0;
		//usleep(3000000);

	    // if(mode==1 || mode==3){
			usleep(ITT*1000); // int ITT=2000;
		// }
// 		else{	
// 			ITTfloat = 1+((float)(rand()%4+1))/2; // (1.5, 2, 2.5, 3 sec )
// 			ITT = ITTfloat*1000;
// 			usleep(ITT*1000); // int ITT=2000;
// 		                  // usleep unit = micro, usleep(2,000,000)=2sec
//         }
		//HP !! important part!!!
		g_scene++;		/* move to the next scene. */


		memcpy(c,  &g_scene,  sizeof(int));
		//sent
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		
		//Baylor
		//HP -2018 (only need)
		t_test[0]  =  0.07; // large & White!!
		//t_test0[0]  = 0.035; // small & White!!
		//t_test2[0]  = 0.14; // big & White!!

		/* record the initial sensor positions */
		check_sensor_pos();
		memcpy(g_s_init,  g_s,  sizeof(fpos));
		memcpy(g_s2_init, g_s2, sizeof(fpos));
		printf("[Init]\n%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n\n",
		g_pos_data_t[0],g_pos_data_t[1],
		g_s_init[0],g_s_init[1],g_s2_init[0],g_s2_init[1]);

		/* to measure the lap time of the first trial for every target */
		target_displayed_gui=1;
		gettimeofday(&t_start,NULL);
		sensor_clear_crap();

		//////////////////////////////////////////////////////////////////////////
		// 0611 : random time between 2 sec and 3sec 
		AtTimeGo = 1000;//(1+((float)(rand()%10+1))/10)*1000;
		IsGoSound=0;
		IsReadySound=0;
		is_elbow=0;

		//Stay
		TargetReachCount=0;
		is_InitialTrargetReached=0;
		TargetRemain=0;
		pis_reached=0;
		SucTarReach=0;
		TargetReachTime=0.0;

		//////////////////////////////////////////////////////////////////////////

	break;
	
	
	// case 11	
	case accSC_TEST_GO:
		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
			break;
		}

		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);

		pis_reached = is_reached;

		//t_test[0]  =  0.07; // large & White!!
		//t_test0[0]  = 0.035; // small & White!!
		//t_test2[0]  = 0.14; // big & White!!

		// Baylor=g_pos_test[g_sched[cnt_target-1]-1][3];
		if (session_score==0 || session_score==1 || session_score==4 || session_score==10 || session_score==14) {
			t_test[0]=t_test2[0]; // small & White!!
		}
		else if (session_score==17 || session_score==19 || session_score==20 || session_score==26 || session_score==29) {
			t_test[0]=t_test2[0]; // small & White!!
		}
		else if (session_score==31 || session_score==35 || session_score==38 || session_score==39 || session_score==42) {
			t_test[0]=t_test2[0]; // small & White!!
		}
		else if (session_score==3 || session_score==6 || session_score==7 || session_score==9 || session_score==12) {
			t_test[0]=t_test0[0]; // small & White!!
		}
		else if (session_score==16 || session_score==21 || session_score==22 || session_score==23 || session_score==30) {
			t_test[0]=t_test0[0]; // small & White!!
		}
		else if (session_score==33 || session_score==34 || session_score==41 || session_score==43 || session_score==44) {
			t_test[0]=t_test0[0]; // small & White!!
		}
		else {
			t_test[0]=t_test[0];
		}
		
		//target position
		//g_pos_data_t[0] =    0 ;
		//g_pos_data_t[1] = -0.8 ;

		is_reached = check_sensor_pos();
		
		dprintf_cr(1, "\n");
		//dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		//dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);


		/* record the result sensor 1xyz, sensor 2 xyz, and target xy */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;
		// HP-2018/01/18
		//g_result_data[cnt_trial-1].t[2] = g_pos_data_t[2];
		//g_result_data[cnt_trial-1].t[3] = g_pos_data_t[3];


		time_t tm=time(NULL);
		struct tm *tmp;
		tmp = localtime(&tm);
		
		gettimeofday(&tval,&tzone);

		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;	


		//get milliseconds into the trial at time of target success
		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = (1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = g_result_data[cnt_trial-2].time+(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		}

		//////////////////////////////////////////////////////////////////////////
		// 0611 : Go 소리 �
		if(IsReadySound==0){
			temp=107;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			IsReadySound=1;	
		}
		if(IsGoSound==0 && TimeForGo >= AtTimeGo){
			//system("aplay -q go.au");	
			temp=106;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			//printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			//printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);

			IsGoSound=1;
		}
		if(TimeForGo >= AtTimeGo){
			g_result_data[cnt_trial-1].GoTime = AtTimeGo;
		}
		else{
			g_result_data[cnt_trial-1].GoTime = 0;
		}

		g_result_data[cnt_trial-1].GoFlag = IsGoSound; // 데이터 저장할 때 같이 저장함 
		// IsGoSound가 0에서 1로 바뀌는 순간이 Go sound가 난 순간임 
		//////////////////////////////////////////////////////////////////////////
		
		//calculate velocity of target reach

		if(cnt_trial>=2){
			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];


			SZp1 = g_result_data[cnt_trial-1].s[2];

			
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;

			if(IsGoSound){

				if( (g_result_data[cnt_trial-1].Vel2 > 25.0) && check==0){
					MTstartIdx=cnt_trial-1;
					check=1;
				}
			}
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;
		}




		if(SZp1 < shoulder && initElbow==0){
			elbowCnt=cnt_trial-1;
			initElbow=1;
			temp=97;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			is_elbow=1;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(SZp1 > shoulder && initElbow==1){
			is_elbow=0;
			initElbow=0;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(initElbow==1){
			TimeElbow=g_result_data[cnt_trial-1].time-g_result_data[elbowCnt].time;
			if(TimeElbow>1000){
				temp=97;
				memcpy(c,  &temp,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				is_elbow=0;
				//initElbow=0;
				g_result_data[cnt_trial-1].elbow=is_elbow;
				elbowCnt=cnt_trial-1;
			}
		}
			

		//printf("z value= %4.2f\n", SZp1);


		// Convert from
		//stay
		if (is_InitialTrargetReached==0 && is_reached > 0){
			is_InitialTrargetReached=1;
			TargetReachCount=cnt_trial-1;
		}
		
		if(is_reached == 0){
			is_InitialTrargetReached=0;
			TargetReachCount=0;
			TargetReachTime=0.0;
		}
		
		if(is_InitialTrargetReached){
			TargetReachTime=g_result_data[cnt_trial-1].time-g_result_data[TargetReachCount].time;
			if(TargetReachTime >= 500){
				SucTarReach = 1;
				is_InitialTrargetReached=0;
				TargetReachCount=0;
				TargetReachTime=0.0;
			}
		}
		// Convert to

		// HP : here time is measured!!!
        if ( SucTarReach && IsGoSound && (is_reached>0   ||  ((check==1) && ( (g_result_data[cnt_trial-1].time - g_result_data[MTstartIdx].time) > maxtime ))) ) {	
			// note there is no "not reached in the backward direction within time constraint"
			//maxtime =10 sec
		
			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);
	
			check=0;
		
			if(mode==1 || mode ==3){
				if(is_reached>0){
					temp=99;
					IsReached = 1;
					session_score++;
				}
				else{
					temp=98;
					IsReached = 0;
				}

			}			

			ModifiedTime=g_result_data[cnt_trial-1].time-g_result_data[MTstartIdx].time;

			// NEW : START ------------------------------------------------
			
			// for calculation starting
			if(mode==2 && NumOfTrainingSession==0){ 
				//case 1
				if(cnt_target<41){ 

					//printf("Target:%f, %f\n",  g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
					//0.000000, 25.000000
					//-16.069700, 19.151100
					//printf("Target:%d, %d\n",  (int)(g_result_data[cnt_trial-1].t[0]),  (int)(g_result_data[cnt_trial-1].t[1]));
					// 0, 25
					//-16, 19

					// 1606, 1915
					// 1606, 1915


					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[0] && ModifiedTime >= 0){
								MovTimeBest[0]=ModifiedTime;
							}
							else{
								
							}
						}
						
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[1] && ModifiedTime >= 0){
								MovTimeBest[1]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[2] && ModifiedTime >= 0){
								MovTimeBest[2]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[3] && ModifiedTime >= 0){
								MovTimeBest[3]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){					
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[4] && ModifiedTime >= 0){
								MovTimeBest[4]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll ==5){
					MeanStd++;
					FindAll =0;
					}

					BestMT1=MovTimeBest[0];
					BestMT2=MovTimeBest[1];
					BestMT3=MovTimeBest[2];
					BestMT4=MovTimeBest[3];
					BestMT5=MovTimeBest[4];
				}

				//case 2
				if(cnt_target==40){
					
					for(MeanStd=0 ; MeanStd<8 ; MeanStd++){
						M1 += MovTimeMeanStd[0][MeanStd];
						M2 += MovTimeMeanStd[1][MeanStd];
						M3 += MovTimeMeanStd[2][MeanStd];
						M4 += MovTimeMeanStd[3][MeanStd];
						M5 += MovTimeMeanStd[4][MeanStd];
					}
					Mean1= M1/8; Mean2= M2/8; Mean3= M3/8; Mean4= M4/8; Mean5= M5/8;

					for(i=0; i<8; i++){
						total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
						sum1 +=total1;
						total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
						sum2 +=total2;
						total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
						sum3 +=total3;
						total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
						sum4 +=total4;
						total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
						sum5 +=total5;
					}
					total1=sum1/8; total2=sum2/8; total3=sum3/8; total4=sum4/8; total5=sum5/8;
					//1 std
					stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
					//2 std
					stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);


					printf("HP-check!!!\n");
					printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
					printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
					printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
					printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
					printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
				

				}


				//case 3				
				if(cnt_target>=41){
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){					
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1 && ModifiedTime >= 0){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2 && ModifiedTime >= 0){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3 && ModifiedTime >= 0){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4 && ModifiedTime >= 0){
								BestMT4=ModifiedTime;
							}
							else{
							
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5 && ModifiedTime >= 0){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						MeanStd++;
						FindAll =0;
						Div++;

						//-----------------------------
						M1=0, M2=0, M3=0, M4=0, M5=0;
						Mean1=0, Mean2=0, Mean3=0, Mean4=0, Mean5=0;
						total1=0, total2=0, total3=0, total4=0, total5=0;
						sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
						stdev1=0, stdev2=0, stdev3=0, stdev4=0, stdev5=0;
						stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;
						//MeanStd=0;
						

						for(MeanStd=0 ; MeanStd<Div ; MeanStd++){
							M1 += MovTimeMeanStd[0][MeanStd];
							M2 += MovTimeMeanStd[1][MeanStd];
							M3 += MovTimeMeanStd[2][MeanStd];
							M4 += MovTimeMeanStd[3][MeanStd];
							M5 += MovTimeMeanStd[4][MeanStd];
						}

						Mean1= M1/Div; Mean2= M2/Div; Mean3= M3/Div; Mean4= M4/Div; Mean5= M5/Div;

						for(i=0; i<Div; i++){
							total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
							sum1 +=total1;
							total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
							sum2 +=total2;
							total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
							sum3 +=total3;
							total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
							sum4 +=total4;
							total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
							sum5 +=total5;
						}
						total1=sum1/Div; total2=sum2/Div; total3=sum3/Div; total4=sum4/Div; total5=sum5/Div;
						//1 std
						stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
						//2 std
						stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);
				

						printf("HP-check!!!\n");
						printf("Div:%d\n", Div);
						printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
						printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
						printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
						printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
						printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
					//-----------------------------

					} //if(FindAll==5){
				} //if(cnt_target>=41){

			} //if(mode==2 && NumOfTrainingSession==0){ //Park
			// for calculation ending
			
			
			/* generate a beep sound */
			if (is_reached>0){
				if(mode==2 && NumOfTrainingSession==0){
					if(cnt_target<41){ 
						//case 1
						if(cnt_target <= 10){
							temp=99;
							IsReached = 1; // hit
							no_successful_target++;							
						}
						
						if(cnt_target>10){
							
							
							// 							printf("Target: %f, %f\n", g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
							// 							printf("ModifiedTime: %f\n",ModifiedTime);
							//							printf("BestMT: %f, %f, %f, %f, %f\n",BestMT1, BestMT2, BestMT3, BestMT4, BestMT5);
							
							
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
								if(BestMT1 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT1 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
								if(BestMT2 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT2 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
								if(BestMT3 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT3 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
								if(BestMT4 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT4 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
								if(BestMT5 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT5 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
						} //if(cnt_target>10){
					} //if(cnt_target<41){
					
					
					else{ //after 41 
						//same1   <=  
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
							//best score 
							if(BestMT1 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add
							//less than best score & better than 1std
							if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//less than 1td & better than 0.5std
							if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 0.5std
							if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean1+stdev21) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same2
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
							//best score
							if(BestMT2 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean2+stdev22) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same3
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
							//best score
							if(BestMT3 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean3+stdev23) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same4
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
							//best score
							if(BestMT4 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean4+stdev24) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same5
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
							//best score
							if(BestMT5 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev25) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean5-stdev25) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean5+stdev25) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						
					}//if(cnt_target<41){ 
					
					//feedback
					if (temp==98 || temp==99)
					{
						Feedback=0;
					}
					else if (temp==100)
					{
						Feedback = 1; //fastest
					}
					else if (temp==101)
					{
						Feedback=2;  //very fast
					}
					else if (temp==102)
					{
						Feedback=3; //fast
					}
					else if (temp==103)
					{
						Feedback=4; //fast
					}
					else if (temp==104)
					{
						Feedback=5; //slow
					}
					else{
						Feedback=6; //very slow
					}
					
				}//if(NumOfTrainingSession==1){ 
				
				
				
				//----------------------------------------------------------------------------------
				// after training >1       if(mode==2 && NumOfTrainingSession==0){
				else if(NumOfTrainingSession>=1) {
					//same1
					//----------------------------				
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4){
								BestMT4=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						FindAll =0;
					}
					//----------------------------
					
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						//best score
						if(BestMT1 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean1+stdev21) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same2
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						//best score
						if(BestMT2 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean2+stdev22) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same3
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						//best score
						if(BestMT3 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean3+stdev23) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same4
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						//best score
						if(BestMT4 >= ModifiedTime){
							//in main: "best score"+"clap"+"yellow target"
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){							
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean4+stdev24) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same5
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						//best score
						if(BestMT5 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add- better than 2std
						if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean5+stdev25) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					
					
				}//if(NumOfTrainingSession>1){
				else{
					temp=99;
					IsReached = 1; // hit
					no_successful_target++;
				}
				
			}//if (is_reached>0){
			else{
				temp=98;
				IsReached = 0; // hit
			}		
			//feedback
			if (temp==98 || temp==99)
			{
				Feedback=0;
			}
			if (temp==100)
			{
				Feedback = 1; //fastest
			}
			if (temp==101)
			{
				Feedback=2;  //very fast
			}
			if (temp==102)
			{
				Feedback=3; //fast
			}
			if (temp==103)
			{
				Feedback=4; //fast
			}
			if (temp==104)
			{
				Feedback=5; //slow
			}
			if (temp==105){
				Feedback=6; //very slow
			}
			
			// NEW : END ------------------------------------------------				
			
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);
			
			
			error_s1 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s[1]);
			error_s2 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s2[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s2[1]);
			final_error = (error_s1 < error_s2) ? error_s1 : error_s2;
			
			//printf("%f,%f,%f,%f,%f,%f,%f\n",trial,tposx,tposy,MT,err,hit,hand);
			sprintf(log_tempacc,
				"%-5d \t%3.2f, %3.2f \t%ld \t%3.2f \t%d \t%d \t%d \t%d \t%d\n",
				cnt_target,
				g_result_data[cnt_trial-1].t[0],
				g_result_data[cnt_trial-1].t[1],
				g_result_data[cnt_trial-1].time,
				final_error, IsReached, 
				check_hand_choice(),
				ModifiedTime,  
				Feedback,
				ITT);
			
			


			SucTarReach=0;
			is_reached = 0;
			pis_reached = 0;			
			g_scene=accSC_TEST_RETURN;
			target_displayed_gui=1;
			memcpy(c,  &g_scene,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;



	// case 12
	case accSC_TEST_RETURN: //time to make log file

		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
		break;
		}
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);



		//HP -2018 (only need)
		//t_test[0]  =  0.07; // large & White!!
		//t_test0[0]  = 0.035; // small & White!!
		//t_test2[0]  = 0.14; // big & White!!
		//Baylor
		t_test[0]=t_test[0];

		is_reached = check_sensor_pos();
		

		//HP
		tm=time(NULL);
		tmp = localtime(&tm);
		//HP


		dprintf_cr(1, "\n");
		dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);
		//dprintf(1, "tt:\t%d\t%d\t%d\n", tmp->tm_hour, tmp->tm_min, tmp->tm_sec);



		/* record the result */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;
		//HP

		gettimeofday(&tval,&tzone);

		//g_result_data[cnt_trial-1].FB = Feedback;
		//////////////////////////////////////////////////////////////////////////
		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;


		if(cnt_trial>=2){

			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];		
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;

		}


		tmp=NULL; 
		//delete (tmp); 
		//////////////////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////////////////


		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		}


		//HP - Time is measured, too
		if ( is_reached==11 ) {	
		// note there is no "not reached in the backward direction within time constraint"

			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);

			/* generate a beep sound */
			if (is_reached>0){
				temp=99;
				IsReached =1;
				//BEEP_SUCCESS();
			}
			else{
				temp=98;
				IsReached =0;
				//BEEP_FAILURE();
			}
			
			// send signal to the bart_gui, 99 success and 98 fail
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);

			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);



			/* create a file to save data */
			if (g_fp == NULL) {
				sprintf(fname, "%s/Traj-%d.txt", accuracyDIR, cnt_target);

				if ((g_fp = fopen(fname, "w+")) == NULL) {
					dprintf(0, "fopen error in testreturnacc1: %s\n", strerror(errno));
				} 
				else {
					sprintf(s, "#%s \t%-15s %-15s %-15s %-15s %-15s \n",
							   "Time", "Sensor1", "Sensor2", "Target",  "Time", "Vel");
					fwrite(s, 1, strlen(s), g_fp);
					sprintf(s, "%s\n",
		"#================================================================");
					fwrite(s, 1, strlen(s), g_fp);
				}
			} /* if (g_fp != NULL) */
			
			/* save data as a file. */
			sprintf(s, "\n");
			fwrite(s, 1, strlen(s), g_fp);

			
			//////////////////////////////////////////////////////
			//tmp->tm_hour; //  int : 0-24
			//tmp->tm_min; // int : 0-59 
			//tmp->tm_sec; // int : 0-59
			//////////////////////////////////////////////////////


			for (i = 0; i < cnt_trial; i++) {
				sprintf(s,
					"%-7ld \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f \t%d,%d,%d,%d	\t%3.2f,%3.2f\t%d,%d \t%d\n",
					 g_result_data[i].time,
					 g_result_data[i].s[0], g_result_data[i].s[1], g_result_data[i].s[2],
					 g_result_data[i].s2[0], g_result_data[i].s2[1], g_result_data[i].s2[2],
					 g_result_data[i].t[0], g_result_data[i].t[1],  
					 g_result_data[i].Hour, g_result_data[i].Min,g_result_data[i].Sec,
					 g_result_data[i].MSec/1000, // MSec/1000 -> millisecond unit
					 g_result_data[i].Vel1, g_result_data[i].Vel2,
					 g_result_data[i].GoTime, g_result_data[i].GoFlag,
					 g_result_data[i].elbow); // 0611 : Trj 파일에 Go Time과 Flag 저장
				fwrite(s, 1, strlen(s), g_fp);
			}

			fclose(g_fp);
			g_fp = NULL;

			/* Write log file */
			fwrite(log_tempacc, 1, strlen(log_tempacc), g_logfp);

			is_elbow = 0;
			is_reached = 0;
			cnt_target++;				/* move to the next target */
			cnt_trial = 1;
			
			order=0;
			check=0;
						
			/* if every target has tried, go to the next step. */
			if (cnt_target > maxtarget){
				g_scene = accSC_TEST_INFO_2; // case 13
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
			else{
				g_scene = accSC_TEST_READY1;
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("cnt<max, [udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
		} 
		else {
			cnt_trial++;				/* try again with the same target */
			//HP
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;


	// case 13 HP- added  : Pre-test
	case accSC_TEST_INFO_2:
		/* close the file */
		if (g_fp) {
			fclose(g_fp);
			g_fp = NULL;
		}

		if (g_logfp) {
			fclose(g_logfp);
			g_logfp = NULL;
		}		

	break;  



	// // case 14 : Training session
	// /* start the test */
	// case accSC_TEST_INFO_A:

	// break;

	

	// // case 15
	// case accSC_TEST_INFO_3:
		
	// break;


	// // case 16
	// case accSC_END:
	// 	/* close the file */
	// 	if (g_fp) {
	// 		fclose(g_fp);
	// 		g_fp = NULL;
	// 	}

	// 	if (g_logfp) {
	// 		fclose(g_logfp);
	// 		g_logfp = NULL;
	// 	}
	// 	// exit(0);
	// break;

	//start first general test

// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	// case 8, g_scene 22
	case genSC_TEST_INFO_1:
	
	//reset all global variables that were probably used for accuracy test
		cnt_trial = 1; cnt_target = 1; cnt_rms = 1;
		flag_direction=0;selected_hand=0;
		no_successful_target=0;
		target_displayed_gui=0;
		setup_condition=setupCond;

		session_score=0;
		
		mode =1;
		inmode=1;
		Feedback=0;
		IsReached=0;

		
		loadparamGeneral();
	 	
	 	//do again for general log file
			//HP - log file is genereated in here
			if (g_logfpGen == NULL) {
				if ((g_logfpGen = fopen(LOGFILEgen, "a+")) == NULL) {
					dprintf(0, "fopen error General: %s\n", strerror(errno));
				} else {
					sprintf(s, "#%s \t%s %s %s %s %s %s %s\n",
							   "Trial", "Target Position", "Movement Time",
							   "Error", "Hit", "Hand", "MT1", "Feedback");
					fwrite(s, 1, strlen(s), g_logfpGen);
					sprintf(s, "%s\n", "#================================================================");
					fwrite(s, 1, strlen(s), g_logfpGen);
				}
			}
	

	break;
	
	
	// case 9	g_scene 23
	case genSC_TEST_READY1:

		//need to create new log file and reset variables

		printf("Finally ready_ 1 in bart_core.c\n");
		g_scene++;

		//HP !! important part!!! 
		memcpy(c,  &g_scene,  sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		target_displayed_gui=0;
		printf("2 Finally ready_ 1 in bart_core.c\n");
		
	break;



	// case 10, g_scene 
	case genSC_TEST_READY2:
	    flag_direction=0;
		selected_hand=0;
		//usleep(3000000);

	    // if(mode==1 || mode==3){
			usleep(ITT*1000); // int ITT=2000;
		// }
// 		else{	
// 			ITTfloat = 1+((float)(rand()%4+1))/2; // (1.5, 2, 2.5, 3 sec )
// 			ITT = ITTfloat*1000;
// 			usleep(ITT*1000); // int ITT=2000;
// 		                  // usleep unit = micro, usleep(2,000,000)=2sec
//         }
		//HP !! important part!!!
		g_scene++;		/* move to the next scene. */


		memcpy(c,  &g_scene,  sizeof(int));
		//sent
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);

		/* record the initial sensor positions */
		check_sensor_pos();
		memcpy(g_s_init,  g_s,  sizeof(fpos));
		memcpy(g_s2_init, g_s2, sizeof(fpos));
		printf("[Init]\n%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n\n",
		g_pos_data_t[0],g_pos_data_t[1],
		g_s_init[0],g_s_init[1],g_s2_init[0],g_s2_init[1]);

		/* to measure the lap time of the first trial for every target */
		target_displayed_gui=1;
		gettimeofday(&t_start,NULL);
		sensor_clear_crap();

		//////////////////////////////////////////////////////////////////////////
		// 0611 : random time between 2 sec and 3sec 
		AtTimeGo = 1000;//(1+((float)(rand()%10+1))/10)*1000;
		IsGoSound=0;
		IsReadySound=0;
		is_elbow=0;

		//Stay
		TargetReachCount=0;
		is_InitialTrargetReached=0;
		TargetRemain=0;
		pis_reached=0;
		SucTarReach=0;
		TargetReachTime=0.0;

		//////////////////////////////////////////////////////////////////////////

	break;
	
	
	// case 11	g_scene 
	case genSC_TEST_GO:
		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
			break;
		}

		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);

		pis_reached = is_reached;

		is_reached = check_sensor_pos();
		
		dprintf_cr(1, "\n");
		//dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		//dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);


		/* record the result */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;


		tm=time(NULL);
		tmp = localtime(&tm);
		
		gettimeofday(&tval,&tzone);

		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;	


		
		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = (1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = g_result_data[cnt_trial-2].time+(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		}

		//////////////////////////////////////////////////////////////////////////
		// 0611 : Go 소리 �
		if(IsReadySound==0){
			temp=107;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			IsReadySound=1;	
		}
		if(IsGoSound==0 && TimeForGo >= AtTimeGo){
			//system("aplay -q go.au");	
			temp=106;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			//printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			//printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);

			IsGoSound=1;
		}
		if(TimeForGo >= AtTimeGo){
			g_result_data[cnt_trial-1].GoTime = AtTimeGo;
		}
		else{
			g_result_data[cnt_trial-1].GoTime = 0;
		}

		g_result_data[cnt_trial-1].GoFlag = IsGoSound; // 데이터 저장할 때 같이 저장함 
		// IsGoSound가 0에서 1로 바뀌는 순간이 Go sound가 난 순간임 
		//////////////////////////////////////////////////////////////////////////
		


		if(cnt_trial>=2){
			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];


			SZp1 = g_result_data[cnt_trial-1].s[2];

			
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;

			if(IsGoSound){

				if( (g_result_data[cnt_trial-1].Vel2 > 25.0) && check==0){
					MTstartIdx=cnt_trial-1;
					check=1;
				}
			}
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;
		}




		if(SZp1 < shoulder && initElbow==0){
			elbowCnt=cnt_trial-1;
			initElbow=1;
			temp=97;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			is_elbow=1;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(SZp1 > shoulder && initElbow==1){
			is_elbow=0;
			initElbow=0;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(initElbow==1){
			TimeElbow=g_result_data[cnt_trial-1].time-g_result_data[elbowCnt].time;
			if(TimeElbow>1000){
				temp=97;
				memcpy(c,  &temp,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				is_elbow=0;
				//initElbow=0;
				g_result_data[cnt_trial-1].elbow=is_elbow;
				elbowCnt=cnt_trial-1;
			}
		}
			

		//printf("z value= %4.2f\n", SZp1);


		// Convert from
		//stay
		if (is_InitialTrargetReached==0 && is_reached > 0){
			is_InitialTrargetReached=1;
			TargetReachCount=cnt_trial-1;
		}
		
		if(is_reached == 0){
			is_InitialTrargetReached=0;
			TargetReachCount=0;
			TargetReachTime=0.0;
		}
		
		if(is_InitialTrargetReached){
			TargetReachTime=g_result_data[cnt_trial-1].time-g_result_data[TargetReachCount].time;
			if(TargetReachTime >= 500){
				SucTarReach = 1;
				is_InitialTrargetReached=0;
				TargetReachCount=0;
				TargetReachTime=0.0;
			}
		}
		// Convert to

		// HP : here time is measured!!!
        if ( SucTarReach && IsGoSound && (is_reached>0   ||  ((check==1) && ( (g_result_data[cnt_trial-1].time - g_result_data[MTstartIdx].time) > maxtime ))) ) {	
			// note there is no "not reached in the backward direction within time constraint"
			//maxtime =10 sec
		
			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);
	
			check=0;
		
			if(mode==1 || mode ==3){
				if(is_reached>0){
					temp=99;
					IsReached = 1;
				}
				else{
					temp=98;
					IsReached = 0;
				}

			}			

			ModifiedTime=g_result_data[cnt_trial-1].time-g_result_data[MTstartIdx].time;

			// NEW : START ------------------------------------------------
			
			// for calculation starting
			if(mode==2 && NumOfTrainingSession==0){ 
				//case 1
				if(cnt_target<41){ 

					//printf("Target:%f, %f\n",  g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
					//0.000000, 25.000000
					//-16.069700, 19.151100
					//printf("Target:%d, %d\n",  (int)(g_result_data[cnt_trial-1].t[0]),  (int)(g_result_data[cnt_trial-1].t[1]));
					// 0, 25
					//-16, 19

					// 1606, 1915
					// 1606, 1915


					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[0] && ModifiedTime >= 0){
								MovTimeBest[0]=ModifiedTime;
							}
							else{
								
							}
						}
						
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[1] && ModifiedTime >= 0){
								MovTimeBest[1]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[2] && ModifiedTime >= 0){
								MovTimeBest[2]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[3] && ModifiedTime >= 0){
								MovTimeBest[3]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){					
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[4] && ModifiedTime >= 0){
								MovTimeBest[4]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll ==5){
					MeanStd++;
					FindAll =0;
					}

					BestMT1=MovTimeBest[0];
					BestMT2=MovTimeBest[1];
					BestMT3=MovTimeBest[2];
					BestMT4=MovTimeBest[3];
					BestMT5=MovTimeBest[4];
				}

				//case 2
				if(cnt_target==40){
					
					for(MeanStd=0 ; MeanStd<8 ; MeanStd++){
						M1 += MovTimeMeanStd[0][MeanStd];
						M2 += MovTimeMeanStd[1][MeanStd];
						M3 += MovTimeMeanStd[2][MeanStd];
						M4 += MovTimeMeanStd[3][MeanStd];
						M5 += MovTimeMeanStd[4][MeanStd];
					}
					Mean1= M1/8; Mean2= M2/8; Mean3= M3/8; Mean4= M4/8; Mean5= M5/8;

					for(i=0; i<8; i++){
						total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
						sum1 +=total1;
						total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
						sum2 +=total2;
						total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
						sum3 +=total3;
						total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
						sum4 +=total4;
						total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
						sum5 +=total5;
					}
					total1=sum1/8; total2=sum2/8; total3=sum3/8; total4=sum4/8; total5=sum5/8;
					//1 std
					stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
					//2 std
					stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);


					printf("HP-check!!!\n");
					printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
					printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
					printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
					printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
					printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
				

				}


				//case 3				
				if(cnt_target>=41){
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){					
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1 && ModifiedTime >= 0){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2 && ModifiedTime >= 0){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3 && ModifiedTime >= 0){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4 && ModifiedTime >= 0){
								BestMT4=ModifiedTime;
							}
							else{
							
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5 && ModifiedTime >= 0){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						MeanStd++;
						FindAll =0;
						Div++;

						//-----------------------------
						M1=0, M2=0, M3=0, M4=0, M5=0;
						Mean1=0, Mean2=0, Mean3=0, Mean4=0, Mean5=0;
						total1=0, total2=0, total3=0, total4=0, total5=0;
						sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
						stdev1=0, stdev2=0, stdev3=0, stdev4=0, stdev5=0;
						stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;
						//MeanStd=0;
						

						for(MeanStd=0 ; MeanStd<Div ; MeanStd++){
							M1 += MovTimeMeanStd[0][MeanStd];
							M2 += MovTimeMeanStd[1][MeanStd];
							M3 += MovTimeMeanStd[2][MeanStd];
							M4 += MovTimeMeanStd[3][MeanStd];
							M5 += MovTimeMeanStd[4][MeanStd];
						}

						Mean1= M1/Div; Mean2= M2/Div; Mean3= M3/Div; Mean4= M4/Div; Mean5= M5/Div;

						for(i=0; i<Div; i++){
							total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
							sum1 +=total1;
							total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
							sum2 +=total2;
							total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
							sum3 +=total3;
							total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
							sum4 +=total4;
							total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
							sum5 +=total5;
						}
						total1=sum1/Div; total2=sum2/Div; total3=sum3/Div; total4=sum4/Div; total5=sum5/Div;
						//1 std
						stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
						//2 std
						stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);
				

						printf("HP-check!!!\n");
						printf("Div:%d\n", Div);
						printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
						printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
						printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
						printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
						printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
					//-----------------------------

					} //if(FindAll==5){
				} //if(cnt_target>=41){

			} //if(mode==2 && NumOfTrainingSession==0){ //Park
			// for calculation ending
			
			
			/* generate a beep sound */
			if (is_reached>0){
				if(mode==2 && NumOfTrainingSession==0){
					if(cnt_target<41){ 
						//case 1
						if(cnt_target <= 10){
							temp=99;
							IsReached = 1; // hit
							no_successful_target++;							
						}
						
						if(cnt_target>10){
							
							
							// 							printf("Target: %f, %f\n", g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
							// 							printf("ModifiedTime: %f\n",ModifiedTime);
							//							printf("BestMT: %f, %f, %f, %f, %f\n",BestMT1, BestMT2, BestMT3, BestMT4, BestMT5);
							
							
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
								if(BestMT1 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT1 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
								if(BestMT2 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT2 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
								if(BestMT3 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT3 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
								if(BestMT4 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT4 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
								if(BestMT5 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT5 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
						} //if(cnt_target>10){
					} //if(cnt_target<41){
					
					
					else{ //after 41 
						//same1   <=  
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
							//best score 
							if(BestMT1 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add
							//less than best score & better than 1std
							if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//less than 1td & better than 0.5std
							if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 0.5std
							if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean1+stdev21) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same2
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
							//best score
							if(BestMT2 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean2+stdev22) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same3
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
							//best score
							if(BestMT3 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean3+stdev23) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same4
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
							//best score
							if(BestMT4 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean4+stdev24) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same5
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
							//best score
							if(BestMT5 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev25) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean5-stdev25) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean5+stdev25) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						
					}//if(cnt_target<41){ 
					
					//feedback
					if (temp==98 || temp==99)
					{
						Feedback=0;
					}
					else if (temp==100)
					{
						Feedback = 1; //fastest
					}
					else if (temp==101)
					{
						Feedback=2;  //very fast
					}
					else if (temp==102)
					{
						Feedback=3; //fast
					}
					else if (temp==103)
					{
						Feedback=4; //fast
					}
					else if (temp==104)
					{
						Feedback=5; //slow
					}
					else{
						Feedback=6; //very slow
					}
					
				}//if(NumOfTrainingSession==1){ 
				
				
				
				//----------------------------------------------------------------------------------
				// after training >1       if(mode==2 && NumOfTrainingSession==0){
				else if(NumOfTrainingSession>=1) {
					//same1
					//----------------------------				
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4){
								BestMT4=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						FindAll =0;
					}
					//----------------------------
					
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						//best score
						if(BestMT1 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean1+stdev21) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same2
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						//best score
						if(BestMT2 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean2+stdev22) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same3
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						//best score
						if(BestMT3 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean3+stdev23) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same4
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						//best score
						if(BestMT4 >= ModifiedTime){
							//in main: "best score"+"clap"+"yellow target"
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){							
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean4+stdev24) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same5
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						//best score
						if(BestMT5 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add- better than 2std
						if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean5+stdev25) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					
					
				}//if(NumOfTrainingSession>1){
				else{
					temp=99;
					IsReached = 1; // hit
					no_successful_target++;
				}
				
			}//if (is_reached>0){
			else{
				temp=98;
				IsReached = 0; // hit
			}		
			//feedback
			if (temp==98 || temp==99)
			{
				Feedback=0;
			}
			if (temp==100)
			{
				Feedback = 1; //fastest
			}
			if (temp==101)
			{
				Feedback=2;  //very fast
			}
			if (temp==102)
			{
				Feedback=3; //fast
			}
			if (temp==103)
			{
				Feedback=4; //fast
			}
			if (temp==104)
			{
				Feedback=5; //slow
			}
			if (temp==105){
				Feedback=6; //very slow
			}
			
			// NEW : END ------------------------------------------------				
			
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);
			
			
			error_s1 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s[1]);
			error_s2 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s2[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s2[1]);
			final_error = (error_s1 < error_s2) ? error_s1 : error_s2;
			
			//printf("%f,%f,%f,%f,%f,%f,%f\n",trial,tposx,tposy,MT,err,hit,hand);
			sprintf(log_tempgen,
				"%-5d \t%3.2f, %3.2f \t%ld \t%3.2f \t%d \t%d \t%d \t%d \t%d\n",
				cnt_target,
				g_result_data[cnt_trial-1].t[0],
				g_result_data[cnt_trial-1].t[1],
				g_result_data[cnt_trial-1].time,
				final_error, IsReached, 
				check_hand_choice(),
				ModifiedTime,  
				Feedback,
				ITT);
			
			


			SucTarReach=0;
			is_reached = 0;
			pis_reached = 0;			
			g_scene=genSC_TEST_RETURN;
			target_displayed_gui=1;
			memcpy(c,  &g_scene,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;



	// case 12, 
	case genSC_TEST_RETURN:
		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
		break;
		}
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);

		is_reached = check_sensor_pos();
		

		//HP
		tm=time(NULL);
		tmp = localtime(&tm);
		//HP


		dprintf_cr(1, "\n");
		dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);
		//dprintf(1, "tt:\t%d\t%d\t%d\n", tmp->tm_hour, tmp->tm_min, tmp->tm_sec);



		/* record the result */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;
		//HP

		gettimeofday(&tval,&tzone);

		//g_result_data[cnt_trial-1].FB = Feedback;
		//////////////////////////////////////////////////////////////////////////
		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;


		if(cnt_trial>=2){

			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];		
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;

		}


		tmp=NULL; 
		//delete (tmp); 
		//////////////////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////////////////


		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		}


		//HP - Time is measured, too
		if ( is_reached==11 ) {	
		// note there is no "not reached in the backward direction within time constraint"

			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);

			/* generate a beep sound */
			if (is_reached>0){
				temp=99;
				IsReached =1;
				//BEEP_SUCCESS();
			}
			else{
				temp=98;
				IsReached =0;
				//BEEP_FAILURE();
			}
			
			// send signal to the bart_gui, 99 success and 98 fail
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);

			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);



			/* create a file to save data */
			if (g_fpGen == NULL) {
				sprintf(fname, "%s/Traj-%d.txt", generalDIR, cnt_target);

				if ((g_fpGen = fopen(fname, "w+")) == NULL) {
					dprintf(0, "fopen error in genreturn1: %s\n", strerror(errno));
				} 
				else {
					sprintf(s, "#%s \t%-15s %-15s %-15s %-15s %-15s \n",
							   "Time", "Sensor1", "Sensor2", "Target",  "Time", "Vel");
					fwrite(s, 1, strlen(s), g_fpGen);
					sprintf(s, "%s\n",
		"#================================================================");
					fwrite(s, 1, strlen(s), g_fpGen);
				}
			} /* if (g_fpGen != NULL) */
			
			/* save data as a file. */
			sprintf(s, "\n");
			fwrite(s, 1, strlen(s), g_fpGen);

			
			//////////////////////////////////////////////////////
			//tmp->tm_hour; //  int : 0-24
			//tmp->tm_min; // int : 0-59 
			//tmp->tm_sec; // int : 0-59
			//////////////////////////////////////////////////////


			for (i = 0; i < cnt_trial; i++) {
				sprintf(s,
					"%-7ld \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f \t%d,%d,%d,%d	\t%3.2f,%3.2f\t%d,%d \t%d\n",
					 g_result_data[i].time,
					 g_result_data[i].s[0], g_result_data[i].s[1], g_result_data[i].s[2],
					 g_result_data[i].s2[0], g_result_data[i].s2[1], g_result_data[i].s2[2],
					 g_result_data[i].t[0], g_result_data[i].t[1],  
					 g_result_data[i].Hour, g_result_data[i].Min,g_result_data[i].Sec,
					 g_result_data[i].MSec/1000, // MSec/1000 -> millisecond unit
					 g_result_data[i].Vel1, g_result_data[i].Vel2,
					 g_result_data[i].GoTime, g_result_data[i].GoFlag,
					 g_result_data[i].elbow); // 0611 : Trj 파일에 Go Time과 Flag 저장
				fwrite(s, 1, strlen(s), g_fpGen);
			}

			fclose(g_fpGen);
			g_fpGen = NULL;

			/* Write log file */
			fwrite(log_tempgen, 1, strlen(log_tempgen), g_logfpGen);

			is_elbow = 0;
			is_reached = 0;
			cnt_target++;				/* move to the next target */
			cnt_trial = 1;
			
			order=0;
			check=0;
						
			/* if every target has tried, go to the next step. */
			if (cnt_target > maxtarget){
				g_scene = genSC_TEST_INFO_2; // case 13
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
			else{
				g_scene = genSC_TEST_READY1;
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
		} 
		else {
			cnt_trial++;				/* try again with the same target */
			//HP
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;


	// case 13 HP- added  : Pre-test
	case genSC_TEST_INFO_2:
		/* close the file */
		if (g_fpGen) {
			fclose(g_fpGen);
			g_fpGen = NULL;
		}

		if (g_logfpGen) {
			fclose(g_logfpGen);
			g_logfpGen = NULL;
		}		

	break;  



	// // case 14 : Training session
	// /* start the test */
	// case genSC_TEST_INFO_A:

	// break;
	

	// // case 15
	// case genSC_TEST_INFO_3:
		
	// break;


	// // case 16
	// case genSC_END:
	// 	/* close the file */
	// 	if (g_fpGen) {
	// 		fclose(g_fpGen);
	// 		g_fpGen = NULL;
	// 	}

	// 	if (g_logfpGen) {
	// 		fclose(g_logfpGen);
	// 		g_logfpGen = NULL;
	// 	}
		
	// break;

// FINISHED WITH GENERAL 1

// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	

//accuracy test 2
	// case 7 : Pre-test
	
	// case 8
	case acc2SC_TEST_INFO_1:

		//reset all global variables that were probably used for accuracy test
		cnt_trial = 1; cnt_target = 1; cnt_rms = 1;
		flag_direction=0;selected_hand=0;
		no_successful_target=0;
		target_displayed_gui=0;
		setup_condition=setupCond;

		session_score=0;
		
		mode =1;
		inmode=1;
		Feedback=0;
		IsReached=0;

		
		loadparam();
	 	
	 	//do again for second accuracy log file
			//HP - log file is genereated in here
			if (g_logfp2 == NULL) {
				if ((g_logfp2 = fopen(LOGFILEacc2, "a+")) == NULL) {
					dprintf(0, "fopen error acc2SC_TEST_INFO1: %s\n", strerror(errno));
				} else {
					sprintf(s, "#%s \t%s %s %s %s %s %s %s\n",
							   "Trial", "Target Position", "Movement Time",
							   "Error", "Hit", "Hand", "MT1", "Feedback");
					fwrite(s, 1, strlen(s), g_logfp2);
					sprintf(s, "%s\n", "#================================================================");
					fwrite(s, 1, strlen(s), g_logfp2);
				}
			}

	break;
	
	
	// case 9	
	case acc2SC_TEST_READY1:
		printf("Finally ready_ 1 in bart_core.c\n");
		g_scene++;

		//HP !! important part!!! 
		memcpy(c,  &g_scene,  sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		target_displayed_gui=0;
		printf("2 Finally ready_ 1 in bart_core.c\n");
		
	break;



	// case 10
	case acc2SC_TEST_READY2:
	    flag_direction=0;
		selected_hand=0;
		//usleep(3000000);

	    // if(mode==1 || mode==3){
			usleep(ITT*1000); // int ITT=2000;
		// }
// 		else{	
// 			ITTfloat = 1+((float)(rand()%4+1))/2; // (1.5, 2, 2.5, 3 sec )
// 			ITT = ITTfloat*1000;
// 			usleep(ITT*1000); // int ITT=2000;
// 		                  // usleep unit = micro, usleep(2,000,000)=2sec
//         }
		//HP !! important part!!!
		g_scene++;		/* move to the next scene. */


		memcpy(c,  &g_scene,  sizeof(int));
		//sent
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		
		//Baylor
		//HP -2018 (only need)
		t_test[0]  =  0.07; // large & White!!
		//t_test0[0]  = 0.035; // small & White!!
		//t_test2[0]  = 0.14; // big & White!!

		/* record the initial sensor positions */
		check_sensor_pos();
		memcpy(g_s_init,  g_s,  sizeof(fpos));
		memcpy(g_s2_init, g_s2, sizeof(fpos));
		printf("[Init]\n%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n\n",
		g_pos_data_t[0],g_pos_data_t[1],
		g_s_init[0],g_s_init[1],g_s2_init[0],g_s2_init[1]);

		/* to measure the lap time of the first trial for every target */
		target_displayed_gui=1;
		gettimeofday(&t_start,NULL);
		sensor_clear_crap();

		//////////////////////////////////////////////////////////////////////////
		// 0611 : random time between 2 sec and 3sec 
		AtTimeGo = 1000;//(1+((float)(rand()%10+1))/10)*1000;
		IsGoSound=0;
		IsReadySound=0;
		is_elbow=0;

		//Stay
		TargetReachCount=0;
		is_InitialTrargetReached=0;
		TargetRemain=0;
		pis_reached=0;
		SucTarReach=0;
		TargetReachTime=0.0;

		//////////////////////////////////////////////////////////////////////////

	break;
	
	
	// case 11	
	case acc2SC_TEST_GO:
		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
			break;
		}

		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);

		pis_reached = is_reached;

		//t_test[0]  =  0.07; // large & White!!
		//t_test0[0]  = 0.035; // small & White!!
		//t_test2[0]  = 0.14; // big & White!!

		// Baylor=g_pos_test[g_sched[cnt_target-1]-1][3];
		if (session_score==0 || session_score==1 || session_score==4 || session_score==10 || session_score==14) {
			t_test[0]=t_test2[0]; // small & White!!
		}
		else if (session_score==17 || session_score==19 || session_score==20 || session_score==26 || session_score==29) {
			t_test[0]=t_test2[0]; // small & White!!
		}
		else if (session_score==31 || session_score==35 || session_score==38 || session_score==39 || session_score==42) {
			t_test[0]=t_test2[0]; // small & White!!
		}
		else if (session_score==3 || session_score==6 || session_score==7 || session_score==9 || session_score==12) {
			t_test[0]=t_test0[0]; // small & White!!
		}
		else if (session_score==16 || session_score==21 || session_score==22 || session_score==23 || session_score==30) {
			t_test[0]=t_test0[0]; // small & White!!
		}
		else if (session_score==33 || session_score==34 || session_score==41 || session_score==43 || session_score==44) {
			t_test[0]=t_test0[0]; // small & White!!
		}
		else {
			t_test[0]=t_test[0];
		}
		
		//target position
		//g_pos_data_t[0] =    0 ;
		//g_pos_data_t[1] = -0.8 ;

		is_reached = check_sensor_pos();
		
		dprintf_cr(1, "\n");
		//dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		//dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);


		/* record the result sensor 1xyz, sensor 2 xyz, and target xy */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;
		// HP-2018/01/18
		//g_result_data[cnt_trial-1].t[2] = g_pos_data_t[2];
		//g_result_data[cnt_trial-1].t[3] = g_pos_data_t[3];


		tm=time(NULL);
		tmp = localtime(&tm);
		
		gettimeofday(&tval,&tzone);

		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;	


		//get milliseconds into the trial at time of target success
		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = (1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = g_result_data[cnt_trial-2].time+(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		}

		//////////////////////////////////////////////////////////////////////////
		// 0611 : Go 소리 �
		if(IsReadySound==0){
			temp=107;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			IsReadySound=1;	
		}
		if(IsGoSound==0 && TimeForGo >= AtTimeGo){
			//system("aplay -q go.au");	
			temp=106;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			//printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			//printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);

			IsGoSound=1;
		}
		if(TimeForGo >= AtTimeGo){
			g_result_data[cnt_trial-1].GoTime = AtTimeGo;
		}
		else{
			g_result_data[cnt_trial-1].GoTime = 0;
		}

		g_result_data[cnt_trial-1].GoFlag = IsGoSound; // 데이터 저장할 때 같이 저장함 
		// IsGoSound가 0에서 1로 바뀌는 순간이 Go sound가 난 순간임 
		//////////////////////////////////////////////////////////////////////////
		
		//calculate velocity of target reach

		if(cnt_trial>=2){
			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];


			SZp1 = g_result_data[cnt_trial-1].s[2];

			
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;

			if(IsGoSound){

				if( (g_result_data[cnt_trial-1].Vel2 > 25.0) && check==0){
					MTstartIdx=cnt_trial-1;
					check=1;
				}
			}
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;
		}




		if(SZp1 < shoulder && initElbow==0){
			elbowCnt=cnt_trial-1;
			initElbow=1;
			temp=97;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			is_elbow=1;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(SZp1 > shoulder && initElbow==1){
			is_elbow=0;
			initElbow=0;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(initElbow==1){
			TimeElbow=g_result_data[cnt_trial-1].time-g_result_data[elbowCnt].time;
			if(TimeElbow>1000){
				temp=97;
				memcpy(c,  &temp,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				is_elbow=0;
				//initElbow=0;
				g_result_data[cnt_trial-1].elbow=is_elbow;
				elbowCnt=cnt_trial-1;
			}
		}
			

		//printf("z value= %4.2f\n", SZp1);


		// Convert from
		//stay
		if (is_InitialTrargetReached==0 && is_reached > 0){
			is_InitialTrargetReached=1;
			TargetReachCount=cnt_trial-1;
		}
		
		if(is_reached == 0){
			is_InitialTrargetReached=0;
			TargetReachCount=0;
			TargetReachTime=0.0;
		}
		
		if(is_InitialTrargetReached){
			TargetReachTime=g_result_data[cnt_trial-1].time-g_result_data[TargetReachCount].time;
			if(TargetReachTime >= 500){
				SucTarReach = 1;
				is_InitialTrargetReached=0;
				TargetReachCount=0;
				TargetReachTime=0.0;
			}
		}
		// Convert to

		// HP : here time is measured!!!
        if ( SucTarReach && IsGoSound && (is_reached>0   ||  ((check==1) && ( (g_result_data[cnt_trial-1].time - g_result_data[MTstartIdx].time) > maxtime ))) ) {	
			// note there is no "not reached in the backward direction within time constraint"
			//maxtime =10 sec
		
			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);
	
			check=0;
		
			if(mode==1 || mode ==3){
				if(is_reached>0){
					temp=99;
					IsReached = 1;
					session_score++;
				}
				else{
					temp=98;
					IsReached = 0;
				}

			}			

			ModifiedTime=g_result_data[cnt_trial-1].time-g_result_data[MTstartIdx].time;

			// NEW : START ------------------------------------------------
			
			// for calculation starting
			if(mode==2 && NumOfTrainingSession==0){ 
				//case 1
				if(cnt_target<41){ 

					//printf("Target:%f, %f\n",  g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
					//0.000000, 25.000000
					//-16.069700, 19.151100
					//printf("Target:%d, %d\n",  (int)(g_result_data[cnt_trial-1].t[0]),  (int)(g_result_data[cnt_trial-1].t[1]));
					// 0, 25
					//-16, 19

					// 1606, 1915
					// 1606, 1915


					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[0] && ModifiedTime >= 0){
								MovTimeBest[0]=ModifiedTime;
							}
							else{
								
							}
						}
						
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[1] && ModifiedTime >= 0){
								MovTimeBest[1]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[2] && ModifiedTime >= 0){
								MovTimeBest[2]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[3] && ModifiedTime >= 0){
								MovTimeBest[3]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){					
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[4] && ModifiedTime >= 0){
								MovTimeBest[4]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll ==5){
					MeanStd++;
					FindAll =0;
					}

					BestMT1=MovTimeBest[0];
					BestMT2=MovTimeBest[1];
					BestMT3=MovTimeBest[2];
					BestMT4=MovTimeBest[3];
					BestMT5=MovTimeBest[4];
				}

				//case 2
				if(cnt_target==40){
					
					for(MeanStd=0 ; MeanStd<8 ; MeanStd++){
						M1 += MovTimeMeanStd[0][MeanStd];
						M2 += MovTimeMeanStd[1][MeanStd];
						M3 += MovTimeMeanStd[2][MeanStd];
						M4 += MovTimeMeanStd[3][MeanStd];
						M5 += MovTimeMeanStd[4][MeanStd];
					}
					Mean1= M1/8; Mean2= M2/8; Mean3= M3/8; Mean4= M4/8; Mean5= M5/8;

					for(i=0; i<8; i++){
						total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
						sum1 +=total1;
						total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
						sum2 +=total2;
						total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
						sum3 +=total3;
						total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
						sum4 +=total4;
						total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
						sum5 +=total5;
					}
					total1=sum1/8; total2=sum2/8; total3=sum3/8; total4=sum4/8; total5=sum5/8;
					//1 std
					stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
					//2 std
					stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);


					printf("HP-check!!!\n");
					printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
					printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
					printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
					printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
					printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
				

				}


				//case 3				
				if(cnt_target>=41){
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){					
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1 && ModifiedTime >= 0){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2 && ModifiedTime >= 0){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3 && ModifiedTime >= 0){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4 && ModifiedTime >= 0){
								BestMT4=ModifiedTime;
							}
							else{
							
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5 && ModifiedTime >= 0){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						MeanStd++;
						FindAll =0;
						Div++;

						//-----------------------------
						M1=0, M2=0, M3=0, M4=0, M5=0;
						Mean1=0, Mean2=0, Mean3=0, Mean4=0, Mean5=0;
						total1=0, total2=0, total3=0, total4=0, total5=0;
						sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
						stdev1=0, stdev2=0, stdev3=0, stdev4=0, stdev5=0;
						stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;
						//MeanStd=0;
						

						for(MeanStd=0 ; MeanStd<Div ; MeanStd++){
							M1 += MovTimeMeanStd[0][MeanStd];
							M2 += MovTimeMeanStd[1][MeanStd];
							M3 += MovTimeMeanStd[2][MeanStd];
							M4 += MovTimeMeanStd[3][MeanStd];
							M5 += MovTimeMeanStd[4][MeanStd];
						}

						Mean1= M1/Div; Mean2= M2/Div; Mean3= M3/Div; Mean4= M4/Div; Mean5= M5/Div;

						for(i=0; i<Div; i++){
							total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
							sum1 +=total1;
							total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
							sum2 +=total2;
							total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
							sum3 +=total3;
							total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
							sum4 +=total4;
							total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
							sum5 +=total5;
						}
						total1=sum1/Div; total2=sum2/Div; total3=sum3/Div; total4=sum4/Div; total5=sum5/Div;
						//1 std
						stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
						//2 std
						stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);
				

						printf("HP-check!!!\n");
						printf("Div:%d\n", Div);
						printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
						printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
						printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
						printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
						printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
					//-----------------------------

					} //if(FindAll==5){
				} //if(cnt_target>=41){

			} //if(mode==2 && NumOfTrainingSession==0){ //Park
			// for calculation ending
			
			
			/* generate a beep sound */
			if (is_reached>0){
				if(mode==2 && NumOfTrainingSession==0){
					if(cnt_target<41){ 
						//case 1
						if(cnt_target <= 10){
							temp=99;
							IsReached = 1; // hit
							no_successful_target++;							
						}
						
						if(cnt_target>10){
							
							
							// 							printf("Target: %f, %f\n", g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
							// 							printf("ModifiedTime: %f\n",ModifiedTime);
							//							printf("BestMT: %f, %f, %f, %f, %f\n",BestMT1, BestMT2, BestMT3, BestMT4, BestMT5);
							
							
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
								if(BestMT1 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT1 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
								if(BestMT2 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT2 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
								if(BestMT3 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT3 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
								if(BestMT4 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT4 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
								if(BestMT5 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT5 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
						} //if(cnt_target>10){
					} //if(cnt_target<41){
					
					
					else{ //after 41 
						//same1   <=  
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
							//best score 
							if(BestMT1 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add
							//less than best score & better than 1std
							if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//less than 1td & better than 0.5std
							if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 0.5std
							if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean1+stdev21) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same2
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
							//best score
							if(BestMT2 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean2+stdev22) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same3
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
							//best score
							if(BestMT3 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean3+stdev23) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same4
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
							//best score
							if(BestMT4 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean4+stdev24) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same5
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
							//best score
							if(BestMT5 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev25) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean5-stdev25) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean5+stdev25) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						
					}//if(cnt_target<41){ 
					
					//feedback
					if (temp==98 || temp==99)
					{
						Feedback=0;
					}
					else if (temp==100)
					{
						Feedback = 1; //fastest
					}
					else if (temp==101)
					{
						Feedback=2;  //very fast
					}
					else if (temp==102)
					{
						Feedback=3; //fast
					}
					else if (temp==103)
					{
						Feedback=4; //fast
					}
					else if (temp==104)
					{
						Feedback=5; //slow
					}
					else{
						Feedback=6; //very slow
					}
					
				}//if(NumOfTrainingSession==1){ 
				
				
				
				//----------------------------------------------------------------------------------
				// after training >1       if(mode==2 && NumOfTrainingSession==0){
				else if(NumOfTrainingSession>=1) {
					//same1
					//----------------------------				
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4){
								BestMT4=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						FindAll =0;
					}
					//----------------------------
					
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						//best score
						if(BestMT1 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean1+stdev21) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same2
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						//best score
						if(BestMT2 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean2+stdev22) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same3
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						//best score
						if(BestMT3 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean3+stdev23) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same4
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						//best score
						if(BestMT4 >= ModifiedTime){
							//in main: "best score"+"clap"+"yellow target"
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){							
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean4+stdev24) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same5
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						//best score
						if(BestMT5 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add- better than 2std
						if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean5+stdev25) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					
					
				}//if(NumOfTrainingSession>1){
				else{
					temp=99;
					IsReached = 1; // hit
					no_successful_target++;
				}
				
			}//if (is_reached>0){
			else{
				temp=98;
				IsReached = 0; // hit
			}		
			//feedback
			if (temp==98 || temp==99)
			{
				Feedback=0;
			}
			if (temp==100)
			{
				Feedback = 1; //fastest
			}
			if (temp==101)
			{
				Feedback=2;  //very fast
			}
			if (temp==102)
			{
				Feedback=3; //fast
			}
			if (temp==103)
			{
				Feedback=4; //fast
			}
			if (temp==104)
			{
				Feedback=5; //slow
			}
			if (temp==105){
				Feedback=6; //very slow
			}
			
			// NEW : END ------------------------------------------------				
			
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);
			
			
			error_s1 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s[1]);
			error_s2 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s2[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s2[1]);
			final_error = (error_s1 < error_s2) ? error_s1 : error_s2;
			
			//printf("%f,%f,%f,%f,%f,%f,%f\n",trial,tposx,tposy,MT,err,hit,hand);
			sprintf(log_tempacc2,
				"%-5d \t%3.2f, %3.2f \t%ld \t%3.2f \t%d \t%d \t%d \t%d \t%d\n",
				cnt_target,
				g_result_data[cnt_trial-1].t[0],
				g_result_data[cnt_trial-1].t[1],
				g_result_data[cnt_trial-1].time,
				final_error, IsReached, 
				check_hand_choice(),
				ModifiedTime,  
				Feedback,
				ITT);
			
			


			SucTarReach=0;
			is_reached = 0;
			pis_reached = 0;			
			g_scene=acc2SC_TEST_RETURN;
			target_displayed_gui=1;
			memcpy(c,  &g_scene,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;



	// case 12
	case acc2SC_TEST_RETURN: //time to make log file

		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
		break;
		}
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);



		//HP -2018 (only need)
		//t_test[0]  =  0.07; // large & White!!
		//t_test0[0]  = 0.035; // small & White!!
		//t_test2[0]  = 0.14; // big & White!!
		//Baylor
		t_test[0]=t_test[0];

		is_reached = check_sensor_pos();
		

		//HP
		tm=time(NULL);
		tmp = localtime(&tm);
		//HP


		dprintf_cr(1, "\n");
		dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);
		//dprintf(1, "tt:\t%d\t%d\t%d\n", tmp->tm_hour, tmp->tm_min, tmp->tm_sec);



		/* record the result */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;
		//HP

		gettimeofday(&tval,&tzone);

		//g_result_data[cnt_trial-1].FB = Feedback;
		//////////////////////////////////////////////////////////////////////////
		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;


		if(cnt_trial>=2){

			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];		
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;

		}


		tmp=NULL; 
		//delete (tmp); 
		//////////////////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////////////////


		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		}


		//HP - Time is measured, too
		if ( is_reached==11 ) {	
		// note there is no "not reached in the backward direction within time constraint"

			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);

			/* generate a beep sound */
			if (is_reached>0){
				temp=99;
				IsReached =1;
				//BEEP_SUCCESS();
			}
			else{
				temp=98;
				IsReached =0;
				//BEEP_FAILURE();
			}
			
			// send signal to the bart_gui, 99 success and 98 fail
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);

			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);



			/* create a file to save data */
			if (g_fp2 == NULL) {
				sprintf(fname, "%s/Traj-%d.txt", accuracyDIR2, cnt_target);

				if ((g_fp2 = fopen(fname, "w+")) == NULL) {
					dprintf(0, "fopen error in testreturnacc2: %s\n", strerror(errno));
				} 
				else {
					sprintf(s, "#%s \t%-15s %-15s %-15s %-15s %-15s \n",
							   "Time", "Sensor1", "Sensor2", "Target",  "Time", "Vel");
					fwrite(s, 1, strlen(s), g_fp2);
					sprintf(s, "%s\n",
		"#================================================================");
					fwrite(s, 1, strlen(s), g_fp2);
				}
			} /* if (g_fp2 != NULL) */
			
			/* save data as a file. */
			sprintf(s, "\n");
			fwrite(s, 1, strlen(s), g_fp2);

			
			//////////////////////////////////////////////////////
			//tmp->tm_hour; //  int : 0-24
			//tmp->tm_min; // int : 0-59 
			//tmp->tm_sec; // int : 0-59
			//////////////////////////////////////////////////////


			for (i = 0; i < cnt_trial; i++) {
				sprintf(s,
					"%-7ld \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f \t%d,%d,%d,%d	\t%3.2f,%3.2f\t%d,%d \t%d\n",
					 g_result_data[i].time,
					 g_result_data[i].s[0], g_result_data[i].s[1], g_result_data[i].s[2],
					 g_result_data[i].s2[0], g_result_data[i].s2[1], g_result_data[i].s2[2],
					 g_result_data[i].t[0], g_result_data[i].t[1],  
					 g_result_data[i].Hour, g_result_data[i].Min,g_result_data[i].Sec,
					 g_result_data[i].MSec/1000, // MSec/1000 -> millisecond unit
					 g_result_data[i].Vel1, g_result_data[i].Vel2,
					 g_result_data[i].GoTime, g_result_data[i].GoFlag,
					 g_result_data[i].elbow); // 0611 : Trj 파일에 Go Time과 Flag 저장
				fwrite(s, 1, strlen(s), g_fp2);
			}

			fclose(g_fp2);
			g_fp2 = NULL;

			/* Write log file */
			fwrite(log_tempacc2, 1, strlen(log_tempacc2), g_logfp2);

			is_elbow = 0;
			is_reached = 0;
			cnt_target++;				/* move to the next target */
			cnt_trial = 1;
			
			order=0;
			check=0;
						
			/* if every target has tried, go to the next step. */
			if (cnt_target > maxtarget){
				g_scene = acc2SC_TEST_INFO_2; // case 13
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
			else{
				g_scene = acc2SC_TEST_READY1;
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("cnt<max, [udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
		} 
		else {
			cnt_trial++;				/* try again with the same target */
			//HP
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;


	// case 13 HP- added  : Pre-test
	case acc2SC_TEST_INFO_2:
		/* close the file */
		if (g_fp2) {
			fclose(g_fp2);
			g_fp2 = NULL;
		}

		if (g_logfp2) {
			fclose(g_logfp2);
			g_logfp2 = NULL;
		}		

	break;  



	// // case 14 : Training session
	// /* start the test */
	// case acc2SC_TEST_INFO_A:

	// break;

	

	// // case 15
	// case acc2SC_TEST_INFO_3:
		
	// break;


	// // case 16
	// case acc2SC_END:
	// 	/* close the file */
	// 	if (g_fp2) {
	// 		fclose(g_fp2);
	// 		g_fp2 = NULL;
	// 	}

	// 	if (g_logfp2) {
	// 		fclose(g_logfp2);
	// 		g_logfp2 = NULL;
	// 	}
	// 	// exit(0);
	// break;

	//start second general test

// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	// case 8, g_scene 22
	case gen2SC_TEST_INFO_1:
	
	//reset all global variables that were probably used for accuracy test
		cnt_trial = 1; cnt_target = 1; cnt_rms = 1;
		flag_direction=0;selected_hand=0;
		no_successful_target=0;
		target_displayed_gui=0;
		setup_condition=setupCond;

		session_score=0;
		
		mode =1;
		inmode=1;
		Feedback=0;
		IsReached=0;

		
		loadparamGeneral();
	 	
	 	//do again for second general log file
			//HP - log file is genereated in here
			if (g_logfpGen2 == NULL) {
				if ((g_logfpGen2 = fopen(LOGFILEgen2, "a+")) == NULL) {
					dprintf(0, "fopen error General2sctestinfo2: %s\n", strerror(errno));
				} else {
					sprintf(s, "#%s \t%s %s %s %s %s %s %s\n",
							   "Trial", "Target Position", "Movement Time",
							   "Error", "Hit", "Hand", "MT1", "Feedback");
					fwrite(s, 1, strlen(s), g_logfpGen2);
					sprintf(s, "%s\n", "#================================================================");
					fwrite(s, 1, strlen(s), g_logfpGen2);
				}
			}
	

	break;
	
	
	// case 9	g_scene 23
	case gen2SC_TEST_READY1:

		//need to create new log file and reset variables

		printf("Finally ready_ 1 in bart_core.c\n");
		g_scene++;

		//HP !! important part!!! 
		memcpy(c,  &g_scene,  sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		target_displayed_gui=0;
		printf("2 Finally ready_ 1 in bart_core.c\n");
		
	break;



	// case 10, g_scene 
	case gen2SC_TEST_READY2:
	    flag_direction=0;
		selected_hand=0;
		//usleep(3000000);

	    // if(mode==1 || mode==3){
			usleep(ITT*1000); // int ITT=2000;
		// }
// 		else{	
// 			ITTfloat = 1+((float)(rand()%4+1))/2; // (1.5, 2, 2.5, 3 sec )
// 			ITT = ITTfloat*1000;
// 			usleep(ITT*1000); // int ITT=2000;
// 		                  // usleep unit = micro, usleep(2,000,000)=2sec
//         }
		//HP !! important part!!!
		g_scene++;		/* move to the next scene. */


		memcpy(c,  &g_scene,  sizeof(int));
		//sent
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);

		/* record the initial sensor positions */
		check_sensor_pos();
		memcpy(g_s_init,  g_s,  sizeof(fpos));
		memcpy(g_s2_init, g_s2, sizeof(fpos));
		printf("[Init]\n%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n\n",
		g_pos_data_t[0],g_pos_data_t[1],
		g_s_init[0],g_s_init[1],g_s2_init[0],g_s2_init[1]);

		/* to measure the lap time of the first trial for every target */
		target_displayed_gui=1;
		gettimeofday(&t_start,NULL);
		sensor_clear_crap();

		//////////////////////////////////////////////////////////////////////////
		// 0611 : random time between 2 sec and 3sec 
		AtTimeGo = 1000;//(1+((float)(rand()%10+1))/10)*1000;
		IsGoSound=0;
		IsReadySound=0;
		is_elbow=0;

		//Stay
		TargetReachCount=0;
		is_InitialTrargetReached=0;
		TargetRemain=0;
		pis_reached=0;
		SucTarReach=0;
		TargetReachTime=0.0;

		//////////////////////////////////////////////////////////////////////////

	break;
	
	
	// case 11	g_scene 
	case gen2SC_TEST_GO:
		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
			break;
		}

		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);

		pis_reached = is_reached;

		is_reached = check_sensor_pos();
		
		dprintf_cr(1, "\n");
		//dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		//dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);


		/* record the result */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;


		tm=time(NULL);
		tmp = localtime(&tm);
		
		gettimeofday(&tval,&tzone);

		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;	


		
		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = (1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			// 0611 : 시작 후 경과 시간
			TimeForGo = g_result_data[cnt_trial-2].time+(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
			//////////////////////////////////////////////////////////////////////////
			
		}

		//////////////////////////////////////////////////////////////////////////
		// 0611 : Go 소리 �
		if(IsReadySound==0){
			temp=107;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			IsReadySound=1;	
		}
		if(IsGoSound==0 && TimeForGo >= AtTimeGo){
			//system("aplay -q go.au");	
			temp=106;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			//printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			//printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);

			IsGoSound=1;
		}
		if(TimeForGo >= AtTimeGo){
			g_result_data[cnt_trial-1].GoTime = AtTimeGo;
		}
		else{
			g_result_data[cnt_trial-1].GoTime = 0;
		}

		g_result_data[cnt_trial-1].GoFlag = IsGoSound; // 데이터 저장할 때 같이 저장함 
		// IsGoSound가 0에서 1로 바뀌는 순간이 Go sound가 난 순간임 
		//////////////////////////////////////////////////////////////////////////
		


		if(cnt_trial>=2){
			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];


			SZp1 = g_result_data[cnt_trial-1].s[2];

			
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if (VelOfSen < 0.00001){
				VelOfSen=0.0;		
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;

			if(IsGoSound){

				if( (g_result_data[cnt_trial-1].Vel2 > 25.0) && check==0){
					MTstartIdx=cnt_trial-1;
					check=1;
				}
			}
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;
		}




		if(SZp1 < shoulder && initElbow==0){
			elbowCnt=cnt_trial-1;
			initElbow=1;
			temp=97;
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			is_elbow=1;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(SZp1 > shoulder && initElbow==1){
			is_elbow=0;
			initElbow=0;
			g_result_data[cnt_trial-1].elbow=is_elbow;
		}
		if(initElbow==1){
			TimeElbow=g_result_data[cnt_trial-1].time-g_result_data[elbowCnt].time;
			if(TimeElbow>1000){
				temp=97;
				memcpy(c,  &temp,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				is_elbow=0;
				//initElbow=0;
				g_result_data[cnt_trial-1].elbow=is_elbow;
				elbowCnt=cnt_trial-1;
			}
		}
			

		//printf("z value= %4.2f\n", SZp1);


		// Convert from
		//stay
		if (is_InitialTrargetReached==0 && is_reached > 0){
			is_InitialTrargetReached=1;
			TargetReachCount=cnt_trial-1;
		}
		
		if(is_reached == 0){
			is_InitialTrargetReached=0;
			TargetReachCount=0;
			TargetReachTime=0.0;
		}
		
		if(is_InitialTrargetReached){
			TargetReachTime=g_result_data[cnt_trial-1].time-g_result_data[TargetReachCount].time;
			if(TargetReachTime >= 500){
				SucTarReach = 1;
				is_InitialTrargetReached=0;
				TargetReachCount=0;
				TargetReachTime=0.0;
			}
		}
		// Convert to

		// HP : here time is measured!!!
        if ( SucTarReach && IsGoSound && (is_reached>0   ||  ((check==1) && ( (g_result_data[cnt_trial-1].time - g_result_data[MTstartIdx].time) > maxtime ))) ) {	
			// note there is no "not reached in the backward direction within time constraint"
			//maxtime =10 sec
		
			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);
	
			check=0;
		
			if(mode==1 || mode ==3){
				if(is_reached>0){
					temp=99;
					IsReached = 1;
				}
				else{
					temp=98;
					IsReached = 0;
				}

			}			

			ModifiedTime=g_result_data[cnt_trial-1].time-g_result_data[MTstartIdx].time;

			// NEW : START ------------------------------------------------
			
			// for calculation starting
			if(mode==2 && NumOfTrainingSession==0){ 
				//case 1
				if(cnt_target<41){ 

					//printf("Target:%f, %f\n",  g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
					//0.000000, 25.000000
					//-16.069700, 19.151100
					//printf("Target:%d, %d\n",  (int)(g_result_data[cnt_trial-1].t[0]),  (int)(g_result_data[cnt_trial-1].t[1]));
					// 0, 25
					//-16, 19

					// 1606, 1915
					// 1606, 1915


					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[0] && ModifiedTime >= 0){
								MovTimeBest[0]=ModifiedTime;
							}
							else{
								
							}
						}
						
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[1] && ModifiedTime >= 0){
								MovTimeBest[1]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[2] && ModifiedTime >= 0){
								MovTimeBest[2]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[3] && ModifiedTime >= 0){
								MovTimeBest[3]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){					
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < MovTimeBest[4] && ModifiedTime >= 0){
								MovTimeBest[4]=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll ==5){
					MeanStd++;
					FindAll =0;
					}

					BestMT1=MovTimeBest[0];
					BestMT2=MovTimeBest[1];
					BestMT3=MovTimeBest[2];
					BestMT4=MovTimeBest[3];
					BestMT5=MovTimeBest[4];
				}

				//case 2
				if(cnt_target==40){
					
					for(MeanStd=0 ; MeanStd<8 ; MeanStd++){
						M1 += MovTimeMeanStd[0][MeanStd];
						M2 += MovTimeMeanStd[1][MeanStd];
						M3 += MovTimeMeanStd[2][MeanStd];
						M4 += MovTimeMeanStd[3][MeanStd];
						M5 += MovTimeMeanStd[4][MeanStd];
					}
					Mean1= M1/8; Mean2= M2/8; Mean3= M3/8; Mean4= M4/8; Mean5= M5/8;

					for(i=0; i<8; i++){
						total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
						sum1 +=total1;
						total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
						sum2 +=total2;
						total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
						sum3 +=total3;
						total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
						sum4 +=total4;
						total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
						sum5 +=total5;
					}
					total1=sum1/8; total2=sum2/8; total3=sum3/8; total4=sum4/8; total5=sum5/8;
					//1 std
					stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
					//2 std
					stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);


					printf("HP-check!!!\n");
					printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
					printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
					printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
					printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
					printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
				

				}


				//case 3				
				if(cnt_target>=41){
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){					
						MovTimeMeanStd[0][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1 && ModifiedTime >= 0){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						MovTimeMeanStd[1][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2 && ModifiedTime >= 0){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						MovTimeMeanStd[2][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3 && ModifiedTime >= 0){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						MovTimeMeanStd[3][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4 && ModifiedTime >= 0){
								BestMT4=ModifiedTime;
							}
							else{
							
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						MovTimeMeanStd[4][MeanStd] = ModifiedTime;
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5 && ModifiedTime >= 0){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						MeanStd++;
						FindAll =0;
						Div++;

						//-----------------------------
						M1=0, M2=0, M3=0, M4=0, M5=0;
						Mean1=0, Mean2=0, Mean3=0, Mean4=0, Mean5=0;
						total1=0, total2=0, total3=0, total4=0, total5=0;
						sum1=0, sum2=0, sum3=0, sum4=0, sum5=0;
						stdev1=0, stdev2=0, stdev3=0, stdev4=0, stdev5=0;
						stdev21=0, stdev22=0, stdev23=0, stdev24=0, stdev25=0;
						//MeanStd=0;
						

						for(MeanStd=0 ; MeanStd<Div ; MeanStd++){
							M1 += MovTimeMeanStd[0][MeanStd];
							M2 += MovTimeMeanStd[1][MeanStd];
							M3 += MovTimeMeanStd[2][MeanStd];
							M4 += MovTimeMeanStd[3][MeanStd];
							M5 += MovTimeMeanStd[4][MeanStd];
						}

						Mean1= M1/Div; Mean2= M2/Div; Mean3= M3/Div; Mean4= M4/Div; Mean5= M5/Div;

						for(i=0; i<Div; i++){
							total1=(MovTimeMeanStd[0][i]-Mean1)*(MovTimeMeanStd[0][i]-Mean1);
							sum1 +=total1;
							total2=(MovTimeMeanStd[1][i]-Mean2)*(MovTimeMeanStd[1][i]-Mean2);
							sum2 +=total2;
							total3=(MovTimeMeanStd[2][i]-Mean3)*(MovTimeMeanStd[2][i]-Mean3);
							sum3 +=total3;
							total4=(MovTimeMeanStd[3][i]-Mean4)*(MovTimeMeanStd[3][i]-Mean4);
							sum4 +=total4;
							total5=(MovTimeMeanStd[4][i]-Mean5)*(MovTimeMeanStd[4][i]-Mean5);
							sum5 +=total5;
						}
						total1=sum1/Div; total2=sum2/Div; total3=sum3/Div; total4=sum4/Div; total5=sum5/Div;
						//1 std
						stdev1=0.5*sqrt(total1); stdev2=0.5*sqrt(total2); stdev3=0.5*sqrt(total3); stdev4=0.5*sqrt(total4); stdev5=0.5*sqrt(total5);
						//2 std
						stdev21=sqrt(total1); stdev22=sqrt(total2); stdev23=sqrt(total3); stdev24=sqrt(total4); stdev25=sqrt(total5);
				

						printf("HP-check!!!\n");
						printf("Div:%d\n", Div);
						printf("check1-Mean1:%f,stdev1:%f,stdev21:%f\n",Mean1,stdev1,stdev21);
						printf("check1-Mean2:%f,stdev2:%f,stdev22:%f\n",Mean2,stdev2,stdev22);
						printf("check1-Mean3:%f,stdev3:%f,stdev23:%f\n",Mean3,stdev3,stdev23);
						printf("check1-Mean4:%f,stdev4:%f,stdev24:%f\n",Mean4,stdev4,stdev24);
						printf("check1-Mean5:%f,stdev5:%f,stdev25:%f\n",Mean5,stdev5,stdev25);
					//-----------------------------

					} //if(FindAll==5){
				} //if(cnt_target>=41){

			} //if(mode==2 && NumOfTrainingSession==0){ //Park
			// for calculation ending
			
			
			/* generate a beep sound */
			if (is_reached>0){
				if(mode==2 && NumOfTrainingSession==0){
					if(cnt_target<41){ 
						//case 1
						if(cnt_target <= 10){
							temp=99;
							IsReached = 1; // hit
							no_successful_target++;							
						}
						
						if(cnt_target>10){
							
							
							// 							printf("Target: %f, %f\n", g_result_data[cnt_trial-1].t[0], g_result_data[cnt_trial-1].t[1]);
							// 							printf("ModifiedTime: %f\n",ModifiedTime);
							//							printf("BestMT: %f, %f, %f, %f, %f\n",BestMT1, BestMT2, BestMT3, BestMT4, BestMT5);
							
							
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
								if(BestMT1 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT1 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
								if(BestMT2 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT2 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
								if(BestMT3 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT3 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
								if(BestMT4 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT4 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
							if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
								if(BestMT5 >= ModifiedTime){
									temp=101;
									IsReached = 1; // hit
									no_successful_target++;
									BestMT5 = ModifiedTime;
								}
								else{
									temp=99;
									IsReached = 1; // hit
									no_successful_target++;
								}
							}
						} //if(cnt_target>10){
					} //if(cnt_target<41){
					
					
					else{ //after 41 
						//same1   <=  
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
							//best score 
							if(BestMT1 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add
							//less than best score & better than 1std
							if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//less than 1td & better than 0.5std
							if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 0.5std
							if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean1+stdev21) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same2
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
							//best score
							if(BestMT2 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean2+stdev22) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same3
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
							//best score
							if(BestMT3 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean3+stdev23) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same4
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
							//best score
							if(BestMT4 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean4+stdev24) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						//same5
						if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
							//best score
							if(BestMT5 >= ModifiedTime){
								temp=100;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//add - better than 2std
							if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev25) ){
								temp=101;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//better than 1std
							if((Mean5-stdev25) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
								temp=102;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//within 1std
							if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
								temp=103;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 1std && better than 2std
							if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
								temp=104;
								IsReached = 1; // hit
								no_successful_target++;
							}
							//worse than 2std 
							if(ModifiedTime > (Mean5+stdev25) ){
								temp=105;
								IsReached = 1; // hit
								no_successful_target++;
							}
						}
						
					}//if(cnt_target<41){ 
					
					//feedback
					if (temp==98 || temp==99)
					{
						Feedback=0;
					}
					else if (temp==100)
					{
						Feedback = 1; //fastest
					}
					else if (temp==101)
					{
						Feedback=2;  //very fast
					}
					else if (temp==102)
					{
						Feedback=3; //fast
					}
					else if (temp==103)
					{
						Feedback=4; //fast
					}
					else if (temp==104)
					{
						Feedback=5; //slow
					}
					else{
						Feedback=6; //very slow
					}
					
				}//if(NumOfTrainingSession==1){ 
				
				
				
				//----------------------------------------------------------------------------------
				// after training >1       if(mode==2 && NumOfTrainingSession==0){
				else if(NumOfTrainingSession>=1) {
					//same1
					//----------------------------				
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT1){
								BestMT1=ModifiedTime;
							}
							else{
								
							}
						} 							
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT2){
								BestMT2=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT3){
								BestMT3=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT4){
								BestMT4=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						FindAll++;
						if(FindAll>0){
							if(ModifiedTime < BestMT5){
								BestMT5=ModifiedTime;
							}
							else{
								
							}
						}
					}
					if(FindAll==5){
						FindAll =0;
					}
					//----------------------------
					
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T1X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T1Y){
						//best score
						if(BestMT1 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT1 < ModifiedTime && ModifiedTime <= (Mean1-stdev21) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean1-stdev21) < ModifiedTime && ModifiedTime <= (Mean1-stdev1) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean1-stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev1) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean1+stdev1) < ModifiedTime && ModifiedTime <= (Mean1+stdev21) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean1+stdev21) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same2
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T2X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T2Y){
						//best score
						if(BestMT2 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT2 < ModifiedTime && ModifiedTime <= (Mean2-stdev22) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean2-stdev22) < ModifiedTime && ModifiedTime <= (Mean2-stdev2) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean2-stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev2) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean2+stdev2) < ModifiedTime && ModifiedTime <= (Mean2+stdev22) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean2+stdev22) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same3
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T3X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T3Y){
						//best score
						if(BestMT3 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT3 < ModifiedTime && ModifiedTime <= (Mean3-stdev23) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean3-stdev23) < ModifiedTime && ModifiedTime <= (Mean3-stdev3) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean3-stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev3) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean3+stdev3) < ModifiedTime && ModifiedTime <= (Mean3+stdev23) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean3+stdev23) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same4
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T4X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T4Y){
						//best score
						if(BestMT4 >= ModifiedTime){
							//in main: "best score"+"clap"+"yellow target"
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add - better than 2std
						if(BestMT4 < ModifiedTime && ModifiedTime <= (Mean4-stdev24) ){							
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean4-stdev24) < ModifiedTime && ModifiedTime <= (Mean4-stdev4) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean4-stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev4) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean4+stdev4) < ModifiedTime && ModifiedTime <= (Mean4+stdev24) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean4+stdev24) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					//same5
					if( (int)(g_result_data[cnt_trial-1].t[0]*100)==T5X && (int)(g_result_data[cnt_trial-1].t[1]*100)==T5Y){
						//best score
						if(BestMT5 >= ModifiedTime){
							temp=100;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//add- better than 2std
						if(BestMT5 < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=101;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//better than 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5-stdev5) ){
							temp=102;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//within 1std
						if((Mean5-stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev5) ){
							temp=103;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 1std && better than 2std
						if((Mean5+stdev5) < ModifiedTime && ModifiedTime <= (Mean5+stdev25) ){
							temp=104;
							IsReached = 1; // hit
							no_successful_target++;
						}
						//worse than 2std 
						if(ModifiedTime > (Mean5+stdev25) ){
							temp=105;
							IsReached = 1; // hit
							no_successful_target++;
						}
					}
					
					
				}//if(NumOfTrainingSession>1){
				else{
					temp=99;
					IsReached = 1; // hit
					no_successful_target++;
				}
				
			}//if (is_reached>0){
			else{
				temp=98;
				IsReached = 0; // hit
			}		
			//feedback
			if (temp==98 || temp==99)
			{
				Feedback=0;
			}
			if (temp==100)
			{
				Feedback = 1; //fastest
			}
			if (temp==101)
			{
				Feedback=2;  //very fast
			}
			if (temp==102)
			{
				Feedback=3; //fast
			}
			if (temp==103)
			{
				Feedback=4; //fast
			}
			if (temp==104)
			{
				Feedback=5; //slow
			}
			if (temp==105){
				Feedback=6; //very slow
			}
			
			// NEW : END ------------------------------------------------				
			
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);
			
			
			error_s1 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s[1]);
			error_s2 =
				hypotf( g_result_data[cnt_trial-1].t[0]-g_result_data[cnt_trial-1].s2[0], g_result_data[cnt_trial-1].t[1]-g_result_data[cnt_trial-1].s2[1]);
			final_error = (error_s1 < error_s2) ? error_s1 : error_s2;
			
			//printf("%f,%f,%f,%f,%f,%f,%f\n",trial,tposx,tposy,MT,err,hit,hand);
			sprintf(log_tempgen2,
				"%-5d \t%3.2f, %3.2f \t%ld \t%3.2f \t%d \t%d \t%d \t%d \t%d\n",
				cnt_target,
				g_result_data[cnt_trial-1].t[0],
				g_result_data[cnt_trial-1].t[1],
				g_result_data[cnt_trial-1].time,
				final_error, IsReached, 
				check_hand_choice(),
				ModifiedTime,  
				Feedback,
				ITT);
			
			


			SucTarReach=0;
			is_reached = 0;
			pis_reached = 0;			
			g_scene=gen2SC_TEST_RETURN;
			target_displayed_gui=1;
			memcpy(c,  &g_scene,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;



	// case 12, 
	case gen2SC_TEST_RETURN:
		/* start a test for reaching task */
		if (target_displayed_gui==0) {
			gettimeofday(&t_start,NULL);
			sensor_clear_crap();
		break;
		}
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;

		/* check whether the target is reached or not */
		sensor_get_pos();

		gettimeofday(&t_end,NULL);

		is_reached = check_sensor_pos();
		

		//HP
		tm=time(NULL);
		tmp = localtime(&tm);
		//HP


		dprintf_cr(1, "\n");
		dprintf(1, "s:\t%7.2f\t%7.2f\n", g_s[0], g_s[1]);
		dprintf(1, "s2:\t%7.2f\t%7.2f\n", g_s2[0], g_s2[1]);
		dprintf(1, "t:\t%7.2f\t%7.2f\n", g_pos_data_t[0], g_pos_data_t[1]);
		//dprintf(1, "tt:\t%d\t%d\t%d\n", tmp->tm_hour, tmp->tm_min, tmp->tm_sec);



		/* record the result */
		g_result_data[cnt_trial-1].s[0] = g_s[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s[1] = (g_s[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s[2] = g_s[2];
		g_result_data[cnt_trial-1].s2[0] = g_s2[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].s2[1] = (g_s2[1]+0.8)*g_real_h;
		g_result_data[cnt_trial-1].s2[2] = g_s2[2];
		g_result_data[cnt_trial-1].t[0] = g_pos_data_t[0]*g_real_w/g_ratio;
		g_result_data[cnt_trial-1].t[1] = (g_pos_data_t[1]+0.8)*g_real_h;
		//HP

		gettimeofday(&tval,&tzone);

		//g_result_data[cnt_trial-1].FB = Feedback;
		//////////////////////////////////////////////////////////////////////////
		g_result_data[cnt_trial-1].Hour = tmp->tm_hour;
		g_result_data[cnt_trial-1].Min = tmp->tm_min;
		g_result_data[cnt_trial-1].Sec = tmp->tm_sec;
		g_result_data[cnt_trial-1].MSec = tval.tv_usec;


		if(cnt_trial>=2){

			SXp2 = g_result_data[cnt_trial-2].s2[0];
			SYp2 = g_result_data[cnt_trial-2].s2[1];
			SXc2 = g_result_data[cnt_trial-1].s2[0];
			SYc2 = g_result_data[cnt_trial-1].s2[1];
		
			SXp1 = g_result_data[cnt_trial-2].s[0];
			SYp1 = g_result_data[cnt_trial-2].s[1];
			SXc1 = g_result_data[cnt_trial-1].s[0];
			SYc1 = g_result_data[cnt_trial-1].s[1];		
			
			SamplingTime = (g_result_data[cnt_trial-1].time - g_result_data[cnt_trial-2].time)/1000.0;

			VelOfSen = sqrt((SXc1-SXp1)*(SXc1-SXp1) + (SYc1-SYp1)*(SYc1-SYp1)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel1 = VelOfSen;
			VelOfSen = sqrt((SXc2-SXp2)*(SXc2-SXp2) + (SYc2-SYp2)*(SYc2-SYp2)) / SamplingTime; // VelOfSen is cm/s
			if(VelOfSen < 0.00001){
				VelOfSen=0.0;
			}
			g_result_data[cnt_trial-1].Vel2 = VelOfSen;
		}
		else{
			g_result_data[cnt_trial-1].Vel1 = 0;
			g_result_data[cnt_trial-1].Vel2 = 0;

		}


		tmp=NULL; 
		//delete (tmp); 
		//////////////////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////////////////


		if (cnt_trial == 1) {
			g_result_data[cnt_trial-1].time =
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		} 
		else {
			g_result_data[cnt_trial-1].time = g_result_data[cnt_trial-2].time+
			(1000000*(t_end.tv_sec-t_start.tv_sec)+t_end.tv_usec-t_start.tv_usec)/1000;
		}


		//HP - Time is measured, too
		if ( is_reached==11 ) {	
		// note there is no "not reached in the backward direction within time constraint"

			dprintf(0, "%s (%d) to reach the %dth target by %d trials\n",
						is_reached?"succeed":"fail",
						is_reached, cnt_target, cnt_trial);

			/* generate a beep sound */
			if (is_reached>0){
				temp=99;
				IsReached =1;
				//BEEP_SUCCESS();
			}
			else{
				temp=98;
				IsReached =0;
				//BEEP_FAILURE();
			}
			
			// send signal to the bart_gui, 99 success and 98 fail
			memcpy(c,  &temp,  sizeof(int));
			nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
			printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);

			printf("[%d] isreached:%d, selected_hand:%d, direction:%d,[%2.2f,%2.2f]\n",cnt_trial,is_reached,selected_hand,flag_direction,g_pos_data_t[0],g_pos_data_t[1]);



			/* create a file to save data */
			if (g_fpGen2 == NULL) {
				sprintf(fname, "%s/Traj-%d.txt", generalDIR2, cnt_target);

				if ((g_fpGen2 = fopen(fname, "w+")) == NULL) {
					dprintf(0, "fopen error in genreturn2: %s\n", strerror(errno));
				} 
				else {
					sprintf(s, "#%s \t%-15s %-15s %-15s %-15s %-15s \n",
							   "Time", "Sensor1", "Sensor2", "Target",  "Time", "Vel");
					fwrite(s, 1, strlen(s), g_fpGen2);
					sprintf(s, "%s\n",
		"#================================================================");
					fwrite(s, 1, strlen(s), g_fpGen2);
				}
			} /* if (g_fpGen2 != NULL) */
			
			/* save data as a file. */
			sprintf(s, "\n");
			fwrite(s, 1, strlen(s), g_fpGen2);

			
			//////////////////////////////////////////////////////
			//tmp->tm_hour; //  int : 0-24
			//tmp->tm_min; // int : 0-59 
			//tmp->tm_sec; // int : 0-59
			//////////////////////////////////////////////////////


			for (i = 0; i < cnt_trial; i++) {
				sprintf(s,
					"%-7ld \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f, %3.2f \t%3.2f, %3.2f \t%d,%d,%d,%d	\t%3.2f,%3.2f\t%d,%d \t%d\n",
					 g_result_data[i].time,
					 g_result_data[i].s[0], g_result_data[i].s[1], g_result_data[i].s[2],
					 g_result_data[i].s2[0], g_result_data[i].s2[1], g_result_data[i].s2[2],
					 g_result_data[i].t[0], g_result_data[i].t[1],  
					 g_result_data[i].Hour, g_result_data[i].Min,g_result_data[i].Sec,
					 g_result_data[i].MSec/1000, // MSec/1000 -> millisecond unit
					 g_result_data[i].Vel1, g_result_data[i].Vel2,
					 g_result_data[i].GoTime, g_result_data[i].GoFlag,
					 g_result_data[i].elbow); // 0611 : Trj 파일에 Go Time과 Flag 저장
				fwrite(s, 1, strlen(s), g_fpGen2);
			}

			fclose(g_fpGen2);
			g_fpGen2 = NULL;

			/* Write log file */
			fwrite(log_tempgen2, 1, strlen(log_tempgen2), g_logfpGen2);

			is_elbow = 0;
			is_reached = 0;
			cnt_target++;				/* move to the next target */
			cnt_trial = 1;
			
			order=0;
			check=0;
						
			/* if every target has tried, go to the next step. */
			if (cnt_target > maxtarget){
				g_scene = gen2SC_TEST_INFO_2; // case 13
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
			else{
				g_scene = gen2SC_TEST_READY1;
				memcpy(c,  &g_scene,  sizeof(int));
				nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
				printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
			}
		} 
		else {
			cnt_trial++;				/* try again with the same target */
			//HP
		}	/* if (cnt_trial == maxtrial || is_reached) */

		gettimeofday(&t_start,NULL);

	break;


	// case 13 HP- added  : Pre-test
	case gen2SC_TEST_INFO_2:
		/* close the file */
		if (g_fpGen2) {
			fclose(g_fpGen2);
			g_fpGen2 = NULL;
		}

		if (g_logfpGen2) {
			fclose(g_logfpGen2);
			g_logfpGen2 = NULL;
		}		

	break;  



	// // case 14 : Training session
	// /* start the test */
	// case gen2SC_TEST_INFO_A:

	// break;
	

	// // case 15
	// case gen2SC_TEST_INFO_3:
		
	// break;


	// case 16
	case gen2SC_END:
		/* close the file */
		if (g_fpGen2) {
			fclose(g_fpGen2);
			g_fpGen2 = NULL;
		}

		if (g_logfpGen2) {
			fclose(g_logfpGen2);
			g_logfpGen2 = NULL;
		}
		exit(0);
	break;

// FINISHED WITH GENERAL 2

// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	

	// case 17
	case SC_LOAD_MT:
	break;
	
	// case 18
	case SC_SHOW_MT:		
	break;


	//NEW
	// case 19
	case SC_LOAD_MT1:
	break;
	
	// case 20
	case SC_SHOW_MT1:
		
	break;



	}	/* switch(g_scene) */
}


/*
	##  #  ##    ##
    ## #    ##  ## 
    ###      ####
    ## #      ##
    ##  #     ##

    ###############################################################################################################################
*/



void keyboard(int key)
{
	if (g_scene == gen2SC_END)
		exit(0);

	switch (key) {
	/* go to the next step or go to start */
	
	case 1:	//KEY_ENTER
	//case KEY_ENT:	//KEY_ENTER

		// New Calib
		if (SC_CALIB_LU <= g_scene && g_scene <= SC_CALIB_L) {
			/* screen calibration */
			printf("start calib\n");
			sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]); sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);sensor_clear_crap();
			
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s[0],g_pos_data_s[1],g_pos_data_s[2]);
			//NW: test slightly different variable to see if same
			//it IS the same if i print sensor
			//printf("%2.3f %2.3f %2.3f\n", sensor[0][0],sensor[1][0],sensor[2][0]);
			g_calibs[1][cnt_calib] = g_pos_data_s[0];
			g_calibs[2][cnt_calib] = g_pos_data_s[1];
			g_calibs[3][cnt_calib] = g_pos_data_s[2];
			if (g_scene == SC_CALIB_L) calib_screen();

			//BEEP_SUCCESS();
			system("aplay -q success.au");
			cnt_calib++;
		} 
		
		
		else if(SC_CALIB_INFO_LS == g_scene){
			InitializeVariables();
		}
		//////////////////////////////////////////////////////////////////////////
		else if (SC_CALIB_LU_LS <= g_scene && g_scene <= SC_CALIB_L_LS) {
			/* screen calibration */
			printf("start calib\n");
			sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]); sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();

			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();
			sensor_get_pos();
			printf("%2.3f %2.3f %2.3f\n",g_pos_data_s2[0],g_pos_data_s2[1],g_pos_data_s2[2]);sensor_clear_crap();

			g_calibs[1][cnt_calib] = g_pos_data_s2[0];
			g_calibs[2][cnt_calib] = g_pos_data_s2[1];
			g_calibs[3][cnt_calib] = g_pos_data_s2[2];
			if (g_scene == SC_CALIB_L_LS) calib_screen2();

			//BEEP_SUCCESS();
			system("aplay -q success.au");
			cnt_calib++;
		} 
		//////////////////////////////////////////////////////////////////////////


		
		//doesnt do anything eveb if true
		else if (accSC_TEST_READY1 <= g_scene && g_scene <= accSC_TEST_RETURN) {
			/* we don't have any more steps to skip. */
		break;
		}	
		
		g_scene++;
		memcpy(c,  &g_scene,  sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
	break;



	/* skip this step, not really sure how this matter, I think it skips a calibration by pressing space. DONT use */
	case 2: //SPACE
	//case KEY_SPACE: //SPACE
		if (g_scene == SC_CALIB_INFO)
			g_scene = accSC_TEST_INFO;
		else if (g_scene == accSC_TEST_INFO)
			g_scene = g_scene+7;
		memcpy(c,  &g_scene,  sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		
	break;

	/* end a program */
	//case KEY_ESC: //ESC
	case 3:
		if (g_fp)
			fclose(g_fp);
		if (g_logfp)
			fclose(g_logfp);
		if (g_fpGen)
			fclose(g_fpGen);
		if (g_logfpGen)
			fclose(g_logfpGen);
		if (g_fp2)
			fclose(g_fp2);
		if (g_logfp2)
			fclose(g_logfp2);
		if (g_fpGen2)
			fclose(g_fpGen2);
		if (g_logfpGen2)
			fclose(g_logfpGen2);
		exit(0);
	break;
	

	case 8: // BART GUI said, the target is displayed
		target_displayed_gui=1;
	break;

	case 9:	//BART GUI said, I am up.
		g_scene=0;
		memcpy(c,  &g_scene,  sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);

	break;

	//in main, keyboard 1
	case 15:
		inmode=1;
		mode=2;
		InitializeVariables();
		CreateNewLogFile(); 
		printf("Parameters are loaded at Bart Core\n");

		g_scene=accSC_TEST_INFO_1;  	
		memcpy(c,  &g_scene,  sizeof(int)); 
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		printf("Go to SC_LOAD_PARM_BARTCORE in main\n");	

	break;

	//in main, keyboard 2
	//MT display
	case 16:
		LoadLOGfileComputeMTbyTarget(LOGFILEacc, NumOfTrainingSession);
 		NumOfTrainingSession++;
		
		g_scene=SC_LOAD_MT;  	
		memcpy(c,  &g_scene,  sizeof(int)); 
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		printf("Go to SC_LOAD_PARM_BARTCORE in main\n");	
		
	break;


	//in main, keyboard 3
	//retention session
	case 17:
		inmode=1;
		mode=1;
		InitializeVariables();
		CreateNewLogFile(); 
		//inmode=1;
		//mode=1;
		printf("Parameters are loaded at Bart Core for retetion\n");

		g_scene=accSC_TEST_INFO_1;  	
		memcpy(c,  &g_scene,  sizeof(int)); 
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		printf("Go to SC_LOAD_PARM_BARTCORE in main\n");
	break;

	//in main, keyboard 4
	//BART session
	case 18:
		inmode=1;
		mode=3;
		InitializeVariables();
		CreateNewLogFile(); 

		printf("Parameters are loaded at Bart Core for BART\n");

		g_scene=accSC_TEST_INFO_1;  	
		memcpy(c,  &g_scene,  sizeof(int)); 
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		printf("Go to SC_LOAD_PARM_BARTCORE in main\n");
	break;

	//in main, keyboard 5 and 9
	//MT display
	case 19:
		LoadLOGfileComputeMTbyTarget_t(LOGFILEacc, NumOfTrialSession);
 		//NumOfTrainingSession++;
		NumOfTrialSession++;
		
		g_scene=SC_LOAD_MT1;  	
		memcpy(c,  &g_scene,  sizeof(int)); 
		nret=WriteMsgToUnixUDP(udp_id,"./bart_gui.tmp",c,4);
		printf("[udp:%d] [gscene:%d] %s [%d]\n",udp_id,g_scene,c,nret);
		printf("Go to SC_LOAD_PARM_BARTCORE in main\n");	
		
	break;


	}
	
}



int ii,jj;
//Here type in the black screen
int main(int argc, char **argv)
{

	// Movement time
	for( ii=0 ; ii < 5 ; ii++){
		MovTimeBest[ii]=10000.0;
		for( jj=0 ; jj < 10 ; jj++){
			MovTimeFromLog[ii][jj]=0.0;
		}
	}

	for( ii=0 ; ii < 5 ; ii++){
		for( jj=0 ; jj < 20 ; jj++){
			MovTimeFromLog_t[ii][jj]=0.0;
			MovTimeMeanStd[ii][jj]=0.0;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// For new calibration
	for (ii=0 ; ii < 12 ; ii++){
		Calibraton_Matrix[ii]=0.0;
		m_CalibResult[ii]=0.0;	
		Calibraton_Matrix2[ii]=0.0;
		m_CalibResult2[ii]=0.0;	
	}
	
	//////////////////////////////////////////////////////////////////////////

	check=0;

	int temp; 
	char s[256]="";
	char fname[256]="";
	char sc[256]="";
	time_t tm=time(NULL);
	struct tm *tmp;
	tmp = localtime(&tm);
	strftime(s, 100, "-%m%d%Y-%H%M%S", tmp);

	printf("Subject Code? ");
	//gets(sc);
	scanf("%s",sc);

	
	printf("Forced(0)? or Free(1)? ");
	scanf("%d",&setupCond);
	setup_condition=setupCond;
	

	//set LOGDIRparent to "../data/subjectcode"
	if (argc > 1)
	   {
       strncat(LOGDIRparent,argv[1],strlen(argv[1]));
	   }
	else
	   {
		   if (strlen(sc)==0)
		    {
	   		char temp2[255]="log";
       			strncat(LOGDIRparent,temp2,strlen(temp2));
	   		}
		   else
			{
			strncat(LOGDIRparent,sc,strlen(sc));
			}
       }

    //create logfilepaths
    strncat(LOGDIRparent,s,strlen(s)); //adds date and time to folder path = "../data/subjectcode123"
	strncat(LOGFILEacc,LOGDIRparent,strlen(LOGDIRparent)); //changes blank LOGFILEacc to "../data/subjectcode123"
	strncat(LOGFILEgen,LOGDIRparent,strlen(LOGDIRparent)); //changes blank LOGFILEgen to "../data/subjectcode123"
	strncat(LOGFILEacc2,LOGDIRparent,strlen(LOGDIRparent)); //changes blank LOGFILEacc2 to "../data/subjectcode123"
	strncat(LOGFILEgen2,LOGDIRparent,strlen(LOGDIRparent)); //changes blank LOGFILEgen2 to "../data/subjectcode123"
	
	//add firstHand
	strncat(LOGFILEacc,addFirstHand,strlen(addFirstHand)); //changes LOGFILEacc to "../data/subjectcode123/firstHand"
	strncat(LOGFILEgen,addFirstHand,strlen(addFirstHand)); //changes LOGFILEgen to "../data/subjectcode123/firstHand"
	//add secondHand
	strncat(LOGFILEacc2,addSecondHand,strlen(addSecondHand)); //changes LOGFILEacc2 to "../data/subjectcode123/secondHand"
	strncat(LOGFILEgen2,addSecondHand,strlen(addSecondHand)); //changes LOGFILEgen2 to "../data/subjectcode123/secondHand"

	//add Acc/Gen to firstHand
	strncat(LOGFILEacc,addAccuracy,strlen(addAccuracy)); //changes LOGFILEacc to "../data/subjectcode123/firstHand/Accuracy"
	strncat(LOGFILEgen,addGeneral,strlen(addGeneral)); //changes LOGFILEgen to "../data/subjectcode123/firstHand/General"
	//add Acc/Gen to secondHand
	strncat(LOGFILEacc2,addAccuracy,strlen(addAccuracy)); //changes LOGFILEacc2 to "../data/subjectcode123/secondHand/Accuracy"
	strncat(LOGFILEgen2,addGeneral,strlen(addGeneral)); //changes LOGFILEgen2 to "../data/subjectcode123/secondHand/General"

	//add logfile.txt to firstHand
	strncat(LOGFILEacc,temp_characc,strlen(temp_characc)); //changes LOGFILEacc to "../data/subjectcode123/firstHand/Accuracy/firstAccuracyLog.txt"
	strncat(LOGFILEgen,temp_chargen,strlen(temp_chargen)); //changes LOGFILEgen to "../data/subjectcode123/firstHand/General/firstGeneralLog.txt"
 	//add logfile.txt to secondHand
	strncat(LOGFILEacc2,temp_characc2,strlen(temp_characc2)); //changes LOGFILEacc2 to "../data/subjectcode123/secondHand/Accuracy/secondAccuracyLog.txt"
	strncat(LOGFILEgen2,temp_chargen2,strlen(temp_chargen2)); //changes LOGFILEgen2 to "../data/subjectcode123/secondHand/General/secondGeneralLog.txt"
 

 	// NOW make directory path for adding Trajectory files in switch loop
 	strncat(accuracyDIR,LOGDIRparent,strlen(LOGDIRparent)); //changes blank accuracyDIR to "../data/subjectcode123"
 	strncat(generalDIR,LOGDIRparent,strlen(LOGDIRparent)); //changes blank generalDIR to "../data/subjectcode123"
 	//add /firstHand into path
	strncat(accuracyDIR,addFirstHand,strlen(addFirstHand)); //changes accuracyDIR to "../data/subjectcode123/firstHand"
 	strncat(generalDIR,addFirstHand,strlen(addFirstHand)); //changes generalDIR to "../data/subjectcode123/firstHand"
 	strncat(tempFirstHand,accuracyDIR,strlen(accuracyDIR)); //add path to temp for mkdir command in loadparam()
 	//add accuracy and general to path 
	strncat(accuracyDIR,addAccuracy,strlen(addAccuracy)); //changes accuracyDIR to "../data/subjectcode123/firstHand/Accuracy"
 	strncat(generalDIR,addGeneral,strlen(addGeneral)); //changes generalDIR to "../data/subjectcode123/firstHand/General"
 	//DO IT AGAIN for secondHand
	strncat(accuracyDIR2,LOGDIRparent,strlen(LOGDIRparent)); //changes blank accuracyDIR2 to "../data/subjectcode123"
 	strncat(generalDIR2,LOGDIRparent,strlen(LOGDIRparent)); //changes blank generalDIR2 to "../data/subjectcode123"
 	//add /firstHand into path
	strncat(accuracyDIR2,addSecondHand,strlen(addSecondHand)); //changes accuracyDIR2 to "../data/subjectcode123/secondHand"
 	strncat(generalDIR2,addSecondHand,strlen(addSecondHand)); //changes generalDIR2 to "../data/subjectcode123/secondHand"
 	strncat(tempSecondHand,accuracyDIR2,strlen(accuracyDIR2)); //add path to temp for mkdir command in loadparam()
 	//add accuracy and general to path 
	strncat(accuracyDIR2,addAccuracy,strlen(addAccuracy)); //changes accuracyDIR2 to "../data/subjectcode123/secondHand/Accuracy"
 	strncat(generalDIR2,addGeneral,strlen(addGeneral)); //changes generalDIR2 to "../data/subjectcode123/secondHand/General"


	 
	argcCopy=argc; 
	argvCopy=argv;
 
	//HP 
	loadparam(); //adds all params to file descriptor fd

	printf("The LOG directory is %s\n\n",LOGDIRparent);
    printf("The LOG directory will be split into firstHand/secondHand,\n");
    printf("and then further into Accuracy/General for each hand.\n");
    printf("Up first, Accuracy test with %d targets.\n", maxtarget);
    printf("Then, General test with %d targets. Use the same arm!\n", maxtarget-15);
    printf("After completing Accuracy and General for 1 arm, switch sensors to second arm and repeat.\n");

			//HP - log file is genereated in here
			if (g_logfp == NULL) {
				if ((g_logfp = fopen(LOGFILEacc, "a+")) == NULL) {
					dprintf(0, "fopen error Accuracy: %s\n", strerror(errno));
				} else {
					sprintf(s, "#%s \t%s %s %s %s %s %s %s\n",
							   "Trial", "Target Position", "Movement Time",
							   "Error", "Hit", "Hand", "MT1", "Feedback");
					fwrite(s, 1, strlen(s), g_logfp);
					sprintf(s, "%s\n", "#================================================================");
					fwrite(s, 1, strlen(s), g_logfp);
				}
			}
			

	serfd=config_serial(); //initalize serial and clear it from sensor.cf
	
	udp_id=CreateUnixUDP("./bart_core.tmp");


	
	while(1)
	{
		FD_ZERO(&rfds);
		FD_SET(0, &rfds);
		FD_SET(udp_id, &rfds);
		FD_SET(serfd,&rfds);
		
		/* Wait up to five seconds. */
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		if( select(20, &rfds, NULL, NULL, &tv ) < 0 )
		{
			printf("Select error!!![%s]",strerror(errno));
		}
		
		if(FD_ISSET(udp_id,&rfds))
		{
			nret = ReadMsgFromUnixUDP(udp_id,c,4);
			if( nret > 0 )
			{
				//printf("%s[%d]\n",c,nret);
				temp=(int)(c[0] | c[1]<<8 | c[2] <<16 | c[3]<<24 );
				printf("receive %d[%d]\n",temp,nret);
				
				keyboard(temp);
			}
		}
        if(FD_ISSET(serfd,&rfds))
        {
			idledisplay();
			
		}
		
	}
	
	sensor_finalize();

	return(0);
}

