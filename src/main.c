//Hyeshin Park - 
// version1 : June 22th 2011	HP
// version2 : July 5th 2011		HP
// version3 : Aug 5th 2011		HP
// version4 : Sept 11th 2011	HP
// version5 : Oct 11th 2011		HP
// version6 : Jan 14th 2018		HP
//main.c 


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


fpos g_pos_data_t;		// the position of a target
fpos g_pos_data_s;		// the position of the first sensor
//HP
fpos g_pos_data_s2;		// the position of the first sensor

float g_real_w = 28.8; // 9.9
float g_real_h = 22; // 7.5
float g_real_z = 2.138; //1.0; //0.964; // 1.424;	//1.334;

fmatrix g_calibs, g_calibt, g_matrix;
fpos g_s, g_s2, g_rms[7], g_s_init, g_s2_init;
struct result_data g_result_data[MAXTRIAL];

int cnt_trial = 1, cnt_target = 1, cnt_calib = 1, cnt_rms = 1;
int flag_direction=0;

int w = 640, h = 640, g_scene = SC_CALIB_INFO;
float g_ratio=1.0;
FILE *g_fp = NULL, *g_logfp = NULL;
FILE *g_fpGen = NULL, *g_logfpGen = NULL;
struct timeval t_end, t_start;
//HP
int mode =1;
int inmode=1;

double Baylor=0.0;

// HP - target color change // target size, color [R, G, B]
ftarget t_calib = { 0.03, 0.0, 1.0, 0.0 }; // small & Green!!
ftarget t_ready = { 0.03, 0.0, 1.0, 0.0 }; // large & Green!!
ftarget t_test  = { 0.03, 1.0, 1.0, 1.0 }; // large & White!!

ftarget t_test0  = { 0.03, 1.0, 1.0, 1.0 }; // small & White!!
ftarget t_test2  = { 0.3, 1.0, 1.0, 1.0 }; // huge & White!!


// HP -2018/0118
//HP - When they have best score!!!t_test
//ftarget t_best = { 0.10, 0.0, 1.0, 0.0 }; // big & green!! 
ftarget t_best = { 0.03, 1.0, 1.0, 1.0 }; // big & green!! 
//HP - When they have best score!!!
ftarget t_very = { 0.03, 1.0, 1.0, 1.0 }; // big & Yellow!! 
//HP - When they have best score!!!
ftarget t_good = { 0.03, 1.0, 1.0, 1.0  }; // big & white!! 
//HP - When they have the ok score!!!
ftarget t_ok = { 0.03, 1.0, 1.0, 1.0 }; // big & Orange red!! 
//HP - When they have the harder score!!!
ftarget t_harder = { 0.03, 1.0, 1.0, 1.0 }; // big & Red!! 


////HP - When they have best score!!!
//ftarget t_best = { 0.10, 1.0, 1.0, 0.0 }; // large & Yellow!! 
////HP - When they have the ok score!!!
//ftarget t_ok = { 0.10, 1.0, 0.3, 0.0 }; // large & Orange red!! 
////HP - When they have the harder score!!!
//ftarget t_harder = { 0.10, 1.0, 0.0, 0.0 }; // large & Red!! 
//ftarget t_verygood = { 0.10, 1.0, 1.0, 1.0 }; // large & White!! 
//ftarget t_black = { 0.10, 0.0, 0.0, 0.0 }; // large & Yellow!! 



/* A pseudo random sequence is calculated by pseudo.c */
int g_sched[MAXTRIAL];
/* During a pilot test, 50 positions are used to conduct reaching task. */
fpos g_pos_test[MAXTRIAL];                       
//HP
int maxtrial = MAXTRIAL, maxtarget = MAXTARGET, maxtime = 10000;
int ITT=1200;
int session_score=0;
char str_session_score[20];
char str_score[20];



/*  UDP  */
char c[1000];
fd_set rfds;
struct  timeval tv;
int udp_id;
int	nret;

int flash=0;
int showTargetNum=0;	
int showMode=0;
int isSwapBuffer=1;
int isReadySound=0;

int isElbow=0;

void InitializeVariables(){
	cnt_trial = 1;
	cnt_target = 1;
	cnt_calib = 1;
	session_score=0;
}



//have loadparam and loadparamGeneral for accuracy and general parameter loading

// #1	
int loadparamGeneral(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	mode=1;
	inmode=1;

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
			} else if (strcmp(ptrtok, "MAXTARGET") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtarget = atoi(ptrtok);
			} else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
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
	for (i = 0; strlen(bufline[i]) > 0; i++) {
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

	//if (access(LOGDIR, R_OK) < 0)
		//mkdir(LOGDIR, 0777);

	return 0;
}



// #1	
int loadparam(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	mode=1;
	inmode=1;

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
			} else if (strcmp(ptrtok, "MAXTARGET") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtarget = atoi(ptrtok);
			} else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
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
	fp = fopen(FCONFIG_SCHED, "r");
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
	for (i = 0; strlen(bufline[i]) > 0; i++) {
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

	//if (access(LOGDIR, R_OK) < 0)
		//mkdir(LOGDIR, 0777);

	return 0;
}

int loadparam_1(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	mode=2;
	
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
			} else if (strcmp(ptrtok, "MAXTARGET") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtarget = atoi(ptrtok);
			} else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
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
	for (i = 0; strlen(bufline[i]) > 0; i++) {
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

	//if (access(LOGDIR, R_OK) < 0)
		//mkdir(LOGDIR, 0777);

	return 0;
}


int loadparam_2(void)
{
	FILE *fp = NULL;
	char *ptr = NULL, *ptrtok = NULL;
	char buf[MAXBUFSZ], bufline[MAXPARAM][MAXLINESZ]; 
	int	i, recv;
	mode=3;
	
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
			} else if (strcmp(ptrtok, "MAXTARGET") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtarget = atoi(ptrtok);
			} else if (strcmp(ptrtok, "MAXTIME") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					maxtime=atoi(ptrtok);
			} else if (strcmp(ptrtok, "ITT") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					ITT=atoi(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_CALIB") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_calib[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_READY") == 0) {
				ptrtok = strtok(NULL, "\t");
				if (ptrtok != NULL)
					t_ready[0] = atof(ptrtok);
			} else if (strcmp(ptrtok, "TARGET_RADIUS_TEST") == 0) {
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
	for (i = 0; strlen(bufline[i]) > 0; i++) {
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

	//if (access(LOGDIR, R_OK) < 0)
		//mkdir(LOGDIR, 0777);

	return 0;
}
// #1 -end (same)





// #2
void reshape(int w1,int h1)
{
	int i;

	if (h1 == 0)
		h1 = 1;

	w = w1;
	h = h1;
	g_ratio = (float)w/(float)h;
	

	
	glMatrixMode(GL_PROJECTION);


	glLoadIdentity();
	
	/* Set the viewport to be the entire window */
    glViewport(0, 0, w1, h1);
	glOrtho(-1.0*g_ratio, 1.0*g_ratio, -1.0, 1.0, 0.0, 20.0);


	// New Calib
	for (i = 1; i <= 9; i++)
		g_calibt[1][i] *= g_ratio;

	glMatrixMode(GL_MODELVIEW);
}



// #3
void setOrthographicProjection()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	/* set a 2D orthographic projection
	 * let origin be not buttom-left but upper-left */
	gluOrtho2D(0, w, 0, h);
	glScalef(1, -1, 1);
	glTranslatef(0, -h, 0);

	glMatrixMode(GL_MODELVIEW);
}



// #4
void resetPerspectiveProjection()
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}



// #5
void glprint(float x, float y, char *string)
{
	char *c;
	void *font=GLUT_BITMAP_8_BY_13;
//	void *font=GLUT_BITMAP_HELVETICA_12;
//	void *font=GLUT_BITMAP_TIMES_ROMAN_24;
	glRasterPos2f(x, y);
	for (c=string; *c != '\0'; c++)
		glutBitmapCharacter(font, *c);
}


// #6
void glprintbig(float x, float y, char *string)
{
	char *c;
//	void *font=GLUT_BITMAP_9_BY_15;
	void *font=GLUT_BITMAP_HELVETICA_18;
//	void *font=GLUT_BITMAP_TIMES_ROMAN_24;
	glRasterPos2f(x, y);
	for (c=string; *c != '\0'; c++)
		glutBitmapCharacter(font, *c);
}


// #7
void glprintbigbig(float x, float y, char *string)
{
	char *c;
//	void *font=GLUT_BITMAP_9_BY_15;
//	void *font=GLUT_BITMAP_HELVETICA_18;
	void *font=GLUT_BITMAP_TIMES_ROMAN_24; //original one
	//void *font=GLUT_BITMAP_TIMES_ROMAN_28; //make larger
	glRasterPos2f(x, y);
	for (c=string; *c != '\0'; c++)
		glutBitmapCharacter(font, *c);
}



// #8
// HP - target color & size!!!
void display_target(ftarget t)
{
	glPushMatrix();
		glLoadIdentity();
		glColor3f(t[1], t[2], t[3]);
		glTranslatef(g_pos_data_t[0], g_pos_data_t[1], g_pos_data_t[2]);
		
		// HP-2018/0118
		// t[0]=g_pos_data_t[3];

		glutSolidSphere(t[0],20,20);
			//glColor3f(1.0, 0.0, 0.0);
		//	sprintf(str_session_score, "%d",3);
		//	glprintbigbig(g_pos_data_t[0], g_pos_data_t[1], str_session_score);
	glPopMatrix();
}

void display_targetNew(ftarget t)
{
	glPushMatrix();
		glColor3f(t[1], t[2], t[3]);
		glTranslatef(g_pos_data_t[0], g_pos_data_t[1], g_pos_data_t[2]);
		glutSolidSphere(t[0],20,20);
	glPopMatrix();
}

// #9
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

	/*
	g_calibs = matrix(1,3,1,5);
	g_calibt = matrix(1,3,1,5);
	g_calibt[1][1]=-0.8;g_calibt[2][1]= 0.8;g_calibt[3][1]=-5.0;
	g_calibt[1][2]= 0.8;g_calibt[2][2]= 0.8;g_calibt[3][2]=-5.0;
	g_calibt[1][3]= 0.8;g_calibt[2][3]=-0.8;g_calibt[3][3]=-5.0;
	g_calibt[1][4]=-0.8;g_calibt[2][4]=-0.8;g_calibt[3][4]=-5.0;
	g_calibt[1][5]= 0.0;g_calibt[2][5]= 0.0;g_calibt[3][5]=-5.0;
	*/

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


	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}


// #10
void display()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glutSwapBuffers();
}





//////////////////////////////////////////////////////////////////////////
// Ʈ���̴� ������ ���� �Ŀ� 5���� Ÿ�ٿ� ���� MT�� �׷����� �׷��� �����ֱ� ���� 
// ���� �� �Լ� ���� 

double	MovTimeFromLog[5][10];
int		NumOfTrainingSession=0;

int		idx=1;
char	strMsg[222];
int		TraininingNumIdx=0;
int		TargetNumIdx=0;
int		IsTraining=0;


void renderMTResult()
{
	//////////////////////////////////////////////////////////////////////////
	// X-Y axix and tick
	// X�� ���� �׷��ִ� �κ� ���� 
	glColor3f(1, 1, 1);
	glLineWidth(1);
		glBegin (GL_LINE_LOOP );
		glVertex2f(-0.5,0);
		glVertex2f(1.1,0);
	glEnd();
	glprint(0,-0.13, "Number of training sessions");
	// X�� ���� �׷��ִ� �κ� ��

	
	
	// X�� ƽ�� �׷��ִ� �κ� ���� 
	for(idx=1 ; idx < 7 ; idx++){
		if(idx<7){
			glLineWidth(1);
			glBegin (GL_LINE_LOOP );
			glVertex2f(-0.5+idx*0.25,0.01);
			glVertex2f(-0.5+idx*0.25,-0.01);
			glEnd();
		}
		sprintf(strMsg, "%d", idx);
		glprint(-0.51+idx*0.25,-0.05, strMsg);
	}
	// X�� ƽ�� �׷��ִ� �κ� ��




	// Y�� ���� �׷��ִ� �κ� ���� 
	glBegin (GL_LINE_LOOP );
	glVertex2f(-0.5,0);
	glVertex2f(-0.5,1.3);
	glEnd();
	glprint(-0.75,1.4, "Movement Time");
	// Y�� ���� �׷��ִ� �κ� ��

	// Y�� ƽ�� �׷��ִ� �κ� ���� 
	for(idx=1 ; idx < 9 ; idx++){
		glLineWidth(1);
			glBegin (GL_LINE_LOOP );
			glVertex2f(-0.49,idx*0.16);
			glVertex2f(-0.51,idx*0.16);
		glEnd();

		sprintf(strMsg, "%1.1fsec", idx*0.5); // idx*0.1�� idx*0.5�� �ٲ��� �� 0.5sec�� y�� ���ڰ� ������
		glprint(-0.7,idx*0.16, strMsg);
	}
	// Y�� ƽ�� �׷��ִ� �κ� ��
	//////////////////////////////////////////////////////////////////////////

	//target 1
	glColor3f(1.0f, 0.0f, 0.0f);
	glprintbigbig(1.1,1.0,"[1]");

	//target 2
	glColor3f(0.0f, 1.0f, 0.0f);
	glprintbigbig(1.1,1.1,"[2]");
	
	//target 3
	glColor3f(1.0f, 0.0f, 1.0f);
	glprintbigbig(1.1,1.1,"[3]");
	
	//target 4
	glColor3f(1.0f, 1.0f, 0.0f);	
	glprintbigbig(1.1,1.1,"[4]");
	
	//target 5
	glColor3f(0.0f, 1.0f, 1.0f);
	glprintbigbig(1.1,1.1,"[5]");

	//////////////////////////////////////////////////////////////////////////
	// Movement time results as a function of the target.
	// MovTimeFromLog[5���� Ÿ�� index][10���� Ʈ���̴� ���� Index]
	// getMovTimeFromCore(MovTimeFromLog);
	if(NumOfTrainingSession==1){ // ó�� Ʈ���̴� ������ ������ ���ξ��� MT�� ǥ������ 

		for(TargetNumIdx=0 ; TargetNumIdx < 5 ; TargetNumIdx++){
			if(TargetNumIdx==0) //target 1
				glColor3f(1.0f, 0.0f, 0.0f);		
			else if(TargetNumIdx==1) //target 2
				glColor3f(0.0f, 1.0f, 0.0f);		
			else if(TargetNumIdx==2) //target 3
				glColor3f(1.0f, 0.0f, 1.0f);		
			else if(TargetNumIdx==3) //target 4
				glColor3f(1.0f, 1.0f, 0.0f);		
			else if(TargetNumIdx==4) //target 5
				glColor3f(0.0f, 1.0f, 1.0f);

			glPointSize(10.0f);//set point size to 10 pixels
			glBegin(GL_POINTS);
			glVertex2f(-0.5,(MovTimeFromLog[TargetNumIdx][0]/1000.0)*0.32);	//0.5�� x�ప, 1/1000=1sec, 1/10���Ѱ�, 0.1�� 1�ʷ� ǥ����.		
			glEnd();

			//show
			//sprintf(strMsg, "[%d]", TargetNumIdx+1);
			//glprint(-0.5,(MovTimeFromLog[TargetNumIdx][0]/1000.0)/5, strMsg);
			printf("TargetNum:%d, Average MT:[%f]\n",TargetNumIdx,MovTimeFromLog[TargetNumIdx][0]);	
		}		

	}
	else{ // �ι� �̻� Ʈ���̴� ������ ������ ���ΰ�  �Բ� MT ǥ������
		for(TraininingNumIdx=0 ; TraininingNumIdx < NumOfTrainingSession-1; TraininingNumIdx++){
			for(TargetNumIdx=0 ; TargetNumIdx < 5 ; TargetNumIdx++){
				if(TargetNumIdx==0) //target 1
					glColor3f(1.0f, 0.0f, 0.0f);		
				else if(TargetNumIdx==1) //target 2
					glColor3f(0.0f, 1.0f, 0.0f);		
				else if(TargetNumIdx==2) //target 3
					glColor3f(1.0f, 0.0f, 1.0f);		
				else if(TargetNumIdx==3) //target 4
					glColor3f(1.0f, 1.0f, 0.0f);		
				else if(TargetNumIdx==4) //target 5
					glColor3f(0.0f, 1.0f, 1.0f);
				
				glLineWidth(2);
				glBegin (GL_LINE_STRIP );		
				glVertex2f(-0.5+TraininingNumIdx*0.25, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx]/1000.0)*0.32);
				glVertex2f(-0.5+(TraininingNumIdx+1)*0.25, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx+1]/1000.0)*0.32);
				glEnd();


				glPointSize(10.0f);//set point size to 10 pixels
				glBegin(GL_POINTS);
				glVertex2f(-0.5+TraininingNumIdx*0.25, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx]/1000.0)*0.32);
				glVertex2f(-0.5+(TraininingNumIdx+1)*0.25, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx+1]/1000.0)*0.32);
				glEnd();

				//if(TraininingNumIdx==NumOfTrainingSession-2){ //���������� �� trainning session�� MT ��հ� �����ֱ�
				//	sprintf(strMsg, "[%d]", TargetNumIdx+1);
				//	glprint(-0.5+(TraininingNumIdx+1)*0.15,strMsg);
				//}
				printf("TargetNum:%d, Average MT:[%f]\n",TargetNumIdx,MovTimeFromLog[TargetNumIdx][TraininingNumIdx]);	
			}		
		}
	}
	//////////////////////////////////////////////////////////////////////////
}



// void renderMTResult()
// {
// 	//////////////////////////////////////////////////////////////////////////
// 	// X-Y axix and tick
// 	// X? ?? ???? ?? ?? 
// 	glColor3f(1, 1, 1);
// 	glLineWidth(1);
// 		glBegin (GL_LINE_LOOP );
// 		glVertex2f(-0.5,0);
// 		glVertex2f(1.1,0);
// 	glEnd();
// 	glprint(0,-0.13, "Number of training sessions");
// 	// X? ?? ???? ?? ?
// 
// 	
// 	
// 	// X? ?? ???? ?? ?? 
// 	for(idx=1 ; idx < 11 ; idx++){
// 		if(idx<10){
// 			glLineWidth(1);
// 			glBegin (GL_LINE_LOOP );
// 			glVertex2f(-0.5+idx*0.17,0.01);
// 			glVertex2f(-0.5+idx*0.17,-0.01);
// 			glEnd();
// 		}
// 		sprintf(strMsg, "%d", idx);
// 		glprint(-0.68+idx*0.17,-0.05, strMsg);
// 	}
// 	// X? ?? ???? ?? ?
// 
// 
// 
// 
// 	// Y? ?? ???? ?? ?? 
// 	glBegin (GL_LINE_LOOP );
// 	glVertex2f(-0.5,0);
// 	glVertex2f(-0.5,1.3);
// 	glEnd();
// 	glprint(-0.75,1.4, "Movement Time");
// 	// Y? ?? ???? ?? ?
// 
// 	// Y? ?? ???? ?? ?? 
// 	for(idx=1 ; idx < 9 ; idx++){
// 		glLineWidth(1);
// 			glBegin (GL_LINE_LOOP );
// 			glVertex2f(-0.49,idx*0.16);
// 			glVertex2f(-0.51,idx*0.16);
// 		glEnd();
// 
// 		sprintf(strMsg, "%1.1fsec", idx*0.5); // idx*0.1? idx*0.5? ??? ? 0.5sec? y? ??? ???
// 		glprint(-0.7,idx*0.16, strMsg);
// 	}
// 	// Y? ?? ???? ?? ?
// 	//////////////////////////////////////////////////////////////////////////
// 
// 	//target 1
// 	glColor3f(1.0f, 0.0f, 0.0f);
// 	glprintbigbig(1.1,1.0,"[1]");
// 
// 	//target 2
// 	glColor3f(0.0f, 1.0f, 0.0f);
// 	glprintbigbig(1.1,1.1,"[2]");
// 	
// 	//target 3
// 	glColor3f(1.0f, 0.0f, 1.0f);
// 	glprintbigbig(1.1,1.2,"[3]");
// 	
// 	//target 4
// 	glColor3f(1.0f, 1.0f, 0.0f);	
// 	glprintbigbig(1.1,1.3,"[4]");
// 	
// 	//target 5
// 	glColor3f(0.0f, 1.0f, 1.0f);
// 	glprintbigbig(1.1,1.4,"[5]");
// 
// 	//////////////////////////////////////////////////////////////////////////
// 	// Movement time results as a function of the target.
// 	// MovTimeFromLog[5?? ?? index][10?? ???? ?? Index]
// 	// getMovTimeFromCore(MovTimeFromLog);
// 	if(NumOfTrainingSession==1){ // ?? ???? ??? ??? ???? MT? ???? 
// 
// 		for(TargetNumIdx=0 ; TargetNumIdx < 5 ; TargetNumIdx++){
// 			if(TargetNumIdx==0) //target 1
// 				glColor3f(1.0f, 0.0f, 0.0f);		
// 			else if(TargetNumIdx==1) //target 2
// 				glColor3f(0.0f, 1.0f, 0.0f);		
// 			else if(TargetNumIdx==2) //target 3
// 				glColor3f(1.0f, 0.0f, 1.0f);		
// 			else if(TargetNumIdx==3) //target 4
// 				glColor3f(1.0f, 1.0f, 0.0f);		
// 			else if(TargetNumIdx==4) //target 5
// 				glColor3f(0.0f, 1.0f, 1.0f);
// 
// 			glPointSize(10.0f);//set point size to 10 pixels
// 			glBegin(GL_POINTS);
// 			glVertex2f(-0.5,(MovTimeFromLog[TargetNumIdx][0]/1000.0)*0.32);	//0.5? x??, 1/1000=1sec, 1/10???, 0.1? 1?? ???.		
// 			glEnd();
// 
// 			//show
// 			//sprintf(strMsg, "[%d]", TargetNumIdx+1);
// 			//glprint(-0.5,(MovTimeFromLog[TargetNumIdx][0]/1000.0)/5, strMsg);
// 			printf("TargetNum:%d, Average MT:[%f]\n",TargetNumIdx,MovTimeFromLog[TargetNumIdx][0]);	
// 		}		
// 
// 	}
// 	else{ // ?? ?? ???? ??? ??? ???  ?? MT ????
// 		for(TraininingNumIdx=0 ; TraininingNumIdx < NumOfTrainingSession-1; TraininingNumIdx++){
// 			for(TargetNumIdx=0 ; TargetNumIdx < 5 ; TargetNumIdx++){
// 				if(TargetNumIdx==0) //target 1
// 					glColor3f(1.0f, 0.0f, 0.0f);		
// 				else if(TargetNumIdx==1) //target 2
// 					glColor3f(0.0f, 1.0f, 0.0f);		
// 				else if(TargetNumIdx==2) //target 3
// 					glColor3f(1.0f, 0.0f, 1.0f);		
// 				else if(TargetNumIdx==3) //target 4
// 					glColor3f(1.0f, 1.0f, 0.0f);		
// 				else if(TargetNumIdx==4) //target 5
// 					glColor3f(0.0f, 1.0f, 1.0f);
// 				
// 				glLineWidth(2);
// 				glBegin (GL_LINE_STRIP );		
// 				glVertex2f(-0.5+TraininingNumIdx*0.17, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx]/1000.0)*0.32);
// 				glVertex2f(-0.5+(TraininingNumIdx+1)*0.17, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx+1]/1000.0)*0.32);
// 				glEnd();
// 
// 
// 				glPointSize(10.0f);//set point size to 10 pixels
// 				glBegin(GL_POINTS);
// 				glVertex2f(-0.5+TraininingNumIdx*0.17, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx]/1000.0)*0.32);
// 				glVertex2f(-0.5+(TraininingNumIdx+1)*0.17, (MovTimeFromLog[TargetNumIdx][TraininingNumIdx+1]/1000.0)*0.32);
// 				glEnd();
// 
// 				//if(TraininingNumIdx==NumOfTrainingSession-2){ //????? ? trainning session? MT ??? ????
// 				//	sprintf(strMsg, "[%d]", TargetNumIdx+1);
// 				//	glprint(-0.5+(TraininingNumIdx+1)*0.15,strMsg);
// 				//}
// 				printf("TargetNum:%d, Average MT:[%f]\n",TargetNumIdx,MovTimeFromLog[TargetNumIdx][TraininingNumIdx]);	
// 			}		
// 		}
// 	}
// 	//////////////////////////////////////////////////////////////////////////
//}




//////////////////////////////////////////////////////////////////////////
#define	MTFILENAME	"./MT.txt"
int		a_datacount=0;
char	strLine[255]={0};

void getMTfromFile(char MTFILE[255]){   //LoadLOGfileComputeMTbyTarget
	FILE* m_FilePointer;
	m_FilePointer = fopen(MTFILE, "r");
	
	while(!feof(m_FilePointer) && a_datacount<10){
		// Read the rest of the line so the file pointer returns to the next line.
		//fgets(strLine, 1000, m_FilePointer);
		
		//if(a_headercount >= 1){
		float mt1, mt2, mt3, mt4, mt5 = 0.0;
		fscanf(m_FilePointer,"%f,%f,%f,%f,%f",&mt1,&mt2,&mt3,&mt4,&mt5);
		//printf("%f,%f,%f,%f,%f,%f,%f\n",trial,tposx,tposy,MT,err,hit,hand);
		MovTimeFromLog[0][a_datacount] = mt1;
		MovTimeFromLog[1][a_datacount] = mt2;
		MovTimeFromLog[2][a_datacount] = mt3;
		MovTimeFromLog[3][a_datacount] = mt4;
		MovTimeFromLog[4][a_datacount] = mt5;
		a_datacount++;
		//}		
		//a_headercount++;
		fgets(strLine, 1000, m_FilePointer);
		printf("Training:%d, Average MT by target:[%4.2f,%4.2f,%4.2f,%4.2f,%4.2f]\n",a_datacount,mt1,mt2,mt3,mt4,mt5);
	}
	fclose(m_FilePointer);
	
	a_datacount=0;
}
//////////////////////////////////////////////////////////////////////////






//// NEW ------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////
// Ʈ���̴� ������ ���� �Ŀ� 5���� Ÿ�ٿ� ���� MT�� �׷����� �׷��� �����ֱ� ���� 
double	MovTimeFromLog_t[5][20];
//int		NumOfTrainingSession=0;
int		idx_t=1;
char	strMsg_t[222];
int		TrialNumIdx=0; //trial
int		TargetNumIdx_t=0;
int		IsTraining_t=1;


void renderMTResult_t()
{
	//////////////////////////////////////////////////////////////////////////
	// X-Y axix and tick
	// X�� ���� �׷��ִ� �κ� ���� 
	glColor3f(1, 1, 1);
	glLineWidth(1);
	glBegin (GL_LINE_LOOP );
	glVertex2f(-0.5,0);
	glVertex2f(1.1,0);
	glEnd();
	glprint(0,-0.13, "Number of trials");
	// X�� ���� �׷��ִ� �κ� ��
	
	// X�� ƽ�� �׷��ִ� �κ� ���� 
	for(idx=1 ; idx < 21 ; idx++){
		if(idx<20){
			glLineWidth(1);
			glBegin (GL_LINE_LOOP );
			glVertex2f(-0.5+idx*0.08,0.01);
			glVertex2f(-0.5+idx*0.08,-0.01);
			glEnd();
		}
		sprintf(strMsg, "%d", idx);
		glprint(-0.59+idx*0.08,-0.05, strMsg);
	}
	// X�� ƽ�� �׷��ִ� �κ� ��
	
	// Y�� ���� �׷��ִ� �κ� ���� 
	glBegin (GL_LINE_LOOP );
	glVertex2f(-0.5,0);
	glVertex2f(-0.5,1.3);
	glEnd();
	glprint(-0.75,1.4, "Movement Time");
	// Y�� ���� �׷��ִ� �κ� ��
	
	// Y�� ƽ�� �׷��ִ� �κ� ���� 
	for(idx=1 ; idx < 9 ; idx++){
		glLineWidth(1);
		glBegin (GL_LINE_LOOP );
		glVertex2f(-0.49,idx*0.16);
		glVertex2f(-0.51,idx*0.16);
		glEnd();
		
		sprintf(strMsg, "%1.1fsec", idx*0.5); // idx*0.1�� idx*0.5�� �ٲ��� �� 0.5sec�� y�� ���ڰ� ������
		glprint(-0.7,idx*0.16, strMsg);
	}
	// Y�� ƽ�� �׷��ִ� �κ� ��
	//////////////////////////////////////////////////////////////////////////

	//target 1
	glColor3f(1.0f, 0.0f, 0.0f);
	glprintbigbig(1.1,1.0,"[1]");

	//target 2
	glColor3f(0.0f, 1.0f, 0.0f);
	glprintbigbig(1.1,1.1,"[2]");
	
	//target 3
	glColor3f(1.0f, 0.0f, 1.0f);
	glprintbigbig(1.1,1.1,"[3]");
	
	//target 4
	glColor3f(1.0f, 1.0f, 0.0f);	
	glprintbigbig(1.1,1.1,"[4]");
	
	//target 5
	glColor3f(0.0f, 1.0f, 1.0f);
	glprintbigbig(1.1,1.1,"[5]");



	//////////////////////////////////////////////////////////////////////////
	// Movement time results as a function of the target.
	// MovTimeFromLog[5���� Ÿ�� index][10���� Ʈ���̴� ���� Index]
	// getMovTimeFromCore(MovTimeFromLog);
    for(TrialNumIdx=0 ; TrialNumIdx < 19 ; TrialNumIdx++){
		for(TargetNumIdx_t=0 ; TargetNumIdx_t < 5 ; TargetNumIdx_t++){
			if(TargetNumIdx_t==0) //target 1
				glColor3f(1.0f, 0.0f, 0.0f);		
			else if(TargetNumIdx_t==1) //target 2
				glColor3f(0.0f, 1.0f, 0.0f);		
			else if(TargetNumIdx_t==2) //target 3
				glColor3f(1.0f, 0.0f, 1.0f);		
			else if(TargetNumIdx_t==3) //target 4
				glColor3f(1.0f, 1.0f, 0.0f);		
			else if(TargetNumIdx_t==4) //target 5
				glColor3f(0.0f, 1.0f, 1.0f);
					
			glLineWidth(2);
			glBegin (GL_LINE_STRIP );		
			glVertex2f(-0.5+TrialNumIdx*0.08, (MovTimeFromLog_t[TargetNumIdx_t][TrialNumIdx]/1000.0)*0.32);
			glVertex2f(-0.5+(TrialNumIdx+1)*0.08, (MovTimeFromLog_t[TargetNumIdx_t][TrialNumIdx+1]/1000.0)*0.32);
			glEnd();


			glPointSize(10.0f);//set point size to 10 pixels
			glBegin(GL_POINTS);
			glVertex2f(-0.5+TrialNumIdx*0.08, (MovTimeFromLog_t[TargetNumIdx_t][TrialNumIdx]/1000.0)*0.32);
			glVertex2f(-0.5+(TrialNumIdx+1)*0.08, (MovTimeFromLog_t[TargetNumIdx_t][TrialNumIdx+1]/1000.0)*0.32);
			glEnd();

			//if(TrialNumIdx==18){ //���������� �� trainning session�� MT ��հ� �����ֱ�
			//	sprintf(strMsg, "[%d]", TargetNumIdx_t+1);
			//	glprint(-0.5+(TrialNumIdx)*0.08, MovTimeFromLogtmp[TargetNumIdx][19]/1000.0, strMsg);

			//glprint(-0.5+(TraininingNumIdx+1)*0.15,MovTimeFromLog[TargetNumIdx][TraininingNumIdx+1]/1000.0, strMsg);
			//}
			printf("TargetNum:%d, MT:[%f]\n",TargetNumIdx_t,MovTimeFromLog_t[TargetNumIdx][TraininingNumIdx]);	
		}	
	}
}


void renderMTResult_tt(int TNum)
{
	//////////////////////////////////////////////////////////////////////////
	// X-Y axix and tick
	// X�� ���� �׷��ִ� �κ� ���� 
	glColor3f(1, 1, 1);
	glLineWidth(1);
	glBegin (GL_LINE_LOOP );
	glVertex2f(-0.5,0);
	glVertex2f(1.1,0);
	glEnd();
	glprint(0,-0.13, "Number of trials");
	// X�� ���� �׷��ִ� �κ� ��
	
	// X�� ƽ�� �׷��ִ� �κ� ���� 
	for(idx=1 ; idx < 21 ; idx++){
		if(idx<20){
			glLineWidth(1);
			glBegin (GL_LINE_LOOP );
			glVertex2f(-0.5+idx*0.08,0.01);
			glVertex2f(-0.5+idx*0.08,-0.01);
			glEnd();
		}
		sprintf(strMsg, "%d", idx);
		glprint(-0.59+idx*0.08,-0.05, strMsg);
	}
	// X�� ƽ�� �׷��ִ� �κ� ��
	
	// Y�� ���� �׷��ִ� �κ� ���� 
	glBegin (GL_LINE_LOOP );
	glVertex2f(-0.5,0);
	glVertex2f(-0.5,1.3);
	glEnd();
	glprint(-0.75,1.4, "Movement Time");
	// Y�� ���� �׷��ִ� �κ� ��
	
	// Y�� ƽ�� �׷��ִ� �κ� ���� 
	for(idx=1 ; idx < 9 ; idx++){
		glLineWidth(1);
		glBegin (GL_LINE_LOOP );
		glVertex2f(-0.49,idx*0.16);
		glVertex2f(-0.51,idx*0.16);
		glEnd();
		
		sprintf(strMsg, "%1.1fsec", idx*0.5); // idx*0.1�� idx*0.5�� �ٲ��� �� 0.5sec�� y�� ���ڰ� ������
		glprint(-0.7,idx*0.16, strMsg);
	}
	// Y�� ƽ�� �׷��ִ� �κ� ��
	//////////////////////////////////////////////////////////////////////////

	//target 1
	glColor3f(1.0f, 0.0f, 0.0f);
	glprintbigbig(1.1,1.0,"[1]");

	//target 2
	glColor3f(0.0f, 1.0f, 0.0f);
	glprintbigbig(1.1,1.1,"[2]");
	
	//target 3
	glColor3f(1.0f, 0.0f, 1.0f);
	glprintbigbig(1.1,1.1,"[3]");
	
	//target 4
	glColor3f(1.0f, 1.0f, 0.0f);	
	glprintbigbig(1.1,1.1,"[4]");
	
	//target 5
	glColor3f(0.0f, 1.0f, 1.0f);
	glprintbigbig(1.1,1.1,"[5]");



	//////////////////////////////////////////////////////////////////////////
	// Movement time results as a function of the target.
	// MovTimeFromLog[5���� Ÿ�� index][10���� Ʈ���̴� ���� Index]
	// getMovTimeFromCore(MovTimeFromLog);
    for(TrialNumIdx=0 ; TrialNumIdx < 19 ; TrialNumIdx++){
		
		if(TNum==0) //target 1
			glColor3f(1.0f, 0.0f, 0.0f);		
		else if(TNum==1) //target 2
			glColor3f(0.0f, 1.0f, 0.0f);		
		else if(TNum==2) //target 3
			glColor3f(1.0f, 0.0f, 1.0f);		
		else if(TNum==3) //target 4
			glColor3f(1.0f, 1.0f, 0.0f);		
		else if(TNum==4) //target 5
			glColor3f(0.0f, 1.0f, 1.0f);
				
		glLineWidth(2);
		glBegin (GL_LINE_STRIP );		
		glVertex2f(-0.5+TrialNumIdx*0.08, (MovTimeFromLog_t[TNum][TrialNumIdx]/1000.0)*0.32);
		glVertex2f(-0.5+(TrialNumIdx+1)*0.08, (MovTimeFromLog_t[TNum][TrialNumIdx+1]/1000.0)*0.32);
		glEnd();


		glPointSize(10.0f);//set point size to 10 pixels
		glBegin(GL_POINTS);
		glVertex2f(-0.5+TrialNumIdx*0.08, (MovTimeFromLog_t[TNum][TrialNumIdx]/1000.0)*0.32);
		glVertex2f(-0.5+(TrialNumIdx+1)*0.08, (MovTimeFromLog_t[TNum][TrialNumIdx+1]/1000.0)*0.32);
		glEnd();

		//if(TrialNumIdx==18){ //??????????? trainning session??MT ??հ? ?????ֱ?
		//	sprintf(strMsg, "[%d]", TNum+1);
		//	glprint(-0.5+(TrialNumIdx)*0.08, MovTimeFromLogtmp[TargetNumIdx][19]/1000.0, strMsg);

		//glprint(-0.5+(TraininingNumIdx+1)*0.15,MovTimeFromLog[TargetNumIdx][TraininingNumIdx+1]/1000.0, strMsg);
		//}
		printf("TargetNum:%d, MT:[%f]\n",TNum,MovTimeFromLog_t[TargetNumIdx][TraininingNumIdx]);
	}
}

//////////////////////////////////////////////////////////////////////////
#define	MTFILENAME_T	"./MT_t.txt"
int		a_datacount_t=0;
char	strLine_t[255]={0};

void getMTfromFile_t(char MTFILE_T[255]){   //LoadLOGfileComputeMTbyTarget
	FILE* m_FilePointer_t;
	m_FilePointer_t = fopen(MTFILE_T, "r");
	
	while(!feof(m_FilePointer_t) && a_datacount_t <20){
		float mt11, mt22, mt33, mt44, mt55 = 0.0;
		fscanf(m_FilePointer_t,"%f,%f,%f,%f,%f",&mt11,&mt22,&mt33,&mt44,&mt55);
		//printf("%f,%f,%f,%f,%f,%f,%f\n",trial,tposx,tposy,MT,err,hit,hand);
		MovTimeFromLog_t[0][a_datacount_t] = mt11;
		MovTimeFromLog_t[1][a_datacount_t] = mt22;
		MovTimeFromLog_t[2][a_datacount_t] = mt33;
		MovTimeFromLog_t[3][a_datacount_t] = mt44;
		MovTimeFromLog_t[4][a_datacount_t] = mt55;
		a_datacount_t++;
		//}		
		//a_headercount++;
		fgets(strLine_t, 1000, m_FilePointer_t);
		printf("Trial:%d, Average MT by target:[%4.2f,%4.2f,%4.2f,%4.2f,%4.2f]\n",a_datacount_t,mt11,mt22,mt33,mt44,mt55);
	}
	fclose(m_FilePointer_t);
	
	a_datacount_t=0;
}

//////////////////////////////////////////////////////////////////////////
////------------------------------------------------------------------------



/*
	######    ######      ##         ######## 
      ##      #     #     ##         ##
      ##      #     #     ##         ########
      ##      #     #     ##         ##      
    ######    ######      #######    ########

*/


//################################################################################################################################################################



// #11   - 45-1000 targets' training session
void idledisplay()
{
	int temp_read;
	int temp, is_reached = 0;
	//int i // unsused var
	//char s[256], fname[256]; // unused vars

	//weren't using these variable so commented out
	// float error_s1, error_s2, final_error;
	// float dist_right, dist_left;

	int goSound=0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    FD_SET(udp_id, &rfds);

    /* Wait up to one seconds. */
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    if( select(20, &rfds, NULL, NULL, &tv ) < 0 )
        printf("Select error!!![%s]",strerror(errno));
		if(FD_ISSET(udp_id,&rfds))
		{
			nret = ReadMsgFromUnixUDP(udp_id,c,4); // Read "c"
			if( nret > 0 )
				{
				temp_read=(int)(c[0] | c[1]<<8 | c[2] <<16 | c[3]<<24 ); 
				if (temp_read<50){
					g_scene=temp_read;
				}
				else if(temp_read==106){
						goSound=1;
				
				}
				else if(temp_read==107){
						isReadySound=1;
				
				}
				else if(temp_read==97){
						isElbow=1;
				
				}
				else if(temp_read>50){
					is_reached=temp_read;
				}
				//HP
				printf("g_scene=%d is_reached=%d dierction=%d\n",g_scene,is_reached,flag_direction);
				}
		}


	/* initialization */
	glLoadIdentity();
	glTranslatef(0, 0, -5);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//HP -START!!!
 	/* display each scene */   //HP !!!
	switch (g_scene) {
	
	// case 1, g_scene 1
	case SC_CALIB_INFO:
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			glprintbig(w/2-140,h/2-60, "STEP 1.1 SCREEN CALIBRATION-Sensor1");
		
		glPopMatrix();
		resetPerspectiveProjection();
	break; 

	//HP - start calibration
	// case 2 , g_scene 2
	case SC_CALIB_LU:
	
	// case 3 , g_scene 3
	case SC_CALIB_RU:
	
	// case 4, g_scene 4
	case SC_CALIB_RB:
	
	// case 5, g_scene 5
	case SC_CALIB_LB:
	
	// case 6, g_scene 6
	case SC_CALIB_C:

	// New Calib, g_scene 7
	case SC_CALIB_T:

	case SC_CALIB_R://, g_scene 8

	case SC_CALIB_B://, g_scene 9

	case SC_CALIB_L://, g_scene 10
		g_pos_data_t[0] = g_calibt[1][cnt_calib];
		g_pos_data_t[1] = g_calibt[2][cnt_calib];
		display_target(t_calib);  // small & Green!!
	
	break;
	//HP - end calibration
	

	//////////////////////////////////////////////////////////////////////////
	// Sensor2 Calib
	// case 1, g_scene 11
	case SC_CALIB_INFO_LS:
		
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();

			glprintbig(w/2-140,h/2-60, "STEP 1.2 SCREEN CALIBRATION-Sensor2");
	
			
		glPopMatrix();
		resetPerspectiveProjection();
	break; 

	//HP - start calibration
	// case 2, g_scene 12
	case SC_CALIB_LU_LS:
	
	// case 3, g_scene 13
	case SC_CALIB_RU_LS:
	
	// case 4, g_scene 14
	case SC_CALIB_RB_LS:
	
	// case 5, g_scene 15
	case SC_CALIB_LB_LS:
	
	// case 6, g_scene 16
	case SC_CALIB_C_LS:

	case SC_CALIB_T_LS://, g_scene 17

	case SC_CALIB_R_LS://, g_scene 18

	case SC_CALIB_B_LS://, g_scene 19

	case SC_CALIB_L_LS://, g_scene 20
		g_pos_data_t[0] = g_calibt[1][cnt_calib];
		g_pos_data_t[1] = g_calibt[2][cnt_calib];
		display_target(t_calib);  // small & Green!!
	
	break;
	//////////////////////////////////////////////////////////////////////////
	//HP - end calibration



	//accuracy test 1
	// case 7 : Pre-test, g_scene 21
	/* start the test */
	case accSC_TEST_INFO:
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			glprintbig(w/2-120,h/2-60,"STEP 2. TEST SESSION");
			//glprint(w/2-80,h/2-20, "ENTER - Start");
		glPopMatrix();
		resetPerspectiveProjection();
	break; 


	// case 8 , g_scene 22
	/* start the test */
	case accSC_TEST_INFO_1:
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			glprintbig(w/2-115,h/2-60,"START OF Accuracy SESSION 1");
			//glprint(w/2-80,h/2-20, "ENTER - Start");
			//HP
			//system("aplay -q clap.au"); //HP -clap.au
		glPopMatrix();
		resetPerspectiveProjection();
	break;  // press 2

	
	// case 9 , g_scene 23
	case accSC_TEST_READY1:
		if (mode==1){  //HP - retention part
			
			if (inmode ==1) {
				if (cnt_target==1){
					sprintf(str_session_score,"0/0");
				}
				//cnt equals for specific test
				else if (cnt_target%45 ==0){
					sprintf(str_session_score,"%d/44",session_score);
				}
				else if (cnt_target%45 ==1){
					sprintf(str_session_score,"%d/45",session_score);
				}
				else{
					sprintf(str_session_score,"%d/%d", session_score,(cnt_target%45-1)); //case accSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				}

				//HP - here change the character size
				glprintbigbig(0.3,0.3,str_session_score);

 				if ((cnt_target%45)==1){  // ((cnt_target+1)%30==1 && session_score==30){
					//session_score=0;
					session_score=0;
					inmode=1;
					mode=2;
				}
			}
			///////////////////////////////////////////////////////////////
			
			if (inmode ==2){
				//HP
				//cnt_target_30= cnt_target-15;

				//if (cnt_target ==26)
					//sprintf(str_session_score,"0/0");
				//else if (cnt_target %60==0)
					//sprintf(str_session_score,"%d/34",session_score);
				//else if (cnt_target %60==1)
					//sprintf(str_session_score,"%d/35",session_score);
				//else{
					//sprintf(str_session_score,"%d/%d", session_score,( (cnt_target%60)-26)); //case accSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				//}

				//HP - here change the character size
				//glprintbigbig(0.5,0.7,str_session_score);

 				//if (((cnt_target)%30)==1)
				//{
				//	session_score=0;
				//	inmode=1;
				//	mode=2;
				//}
			}		

			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1


		}

		//////////////////////////////////////////////////////////////////
		if (mode==2){   //HP - training part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%100 ==0)
				sprintf(str_session_score,"%d/99",session_score);
			else if (cnt_target%100 ==1)
				sprintf(str_session_score,"%d/100",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%100-1)); //case accSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.3,0.3,str_session_score);

 			if ((cnt_target%100)==1)
			{
				session_score=0;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}


		//////////////////////////////////////////////////////////////////
		if (mode==3){   //HP - BART part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%30 ==0)
				sprintf(str_session_score,"%d/29",session_score);
			else if (cnt_target%30 ==1)
				sprintf(str_session_score,"%d/30",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%30-1)); //case accSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.3,0.3,str_session_score);

 			if ((cnt_target%30)==1)
			{
				session_score=0;
				inmode=1;
				mode=2;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}

	break;

		


	// case 10, g_scene 24
	case accSC_TEST_READY2:
		glprintbigbig(0.3,0.3,str_session_score); //case accSC_TEST_READY2:
       
		//HP 
		//showing target
		display_target(t_ready);  // large & Green!!


		//system("aplay -q go.au");	
	break;
	
	

	// case 11,  printed g_scene 25
	case accSC_TEST_GO:

		//setOrthographicProjection();
		//glLoadIdentity();
		glprintbigbig(0.5,0.7,str_session_score); //case accSC_TEST_GO:
		//resetPerspectiveProjection();
		//HP - "g_sched should be changed!!"

		//sets target screen calibrated X and Y position based on param load from file
		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;
		

		//HP -2018 (only need)
		//Baylor=g_pos_test[g_sched[cnt_target-1]-1][3];

		// size changes to large and small based on which iteration of reaching we are on
		if (session_score==0 || session_score==1 || session_score==4 || session_score==10 || session_score==14) {
			display_target(t_test2); // huge & White!!
		}
		else if (session_score==17 || session_score==19 || session_score==20 || session_score==26 || session_score==29) {
			display_target(t_test2); // huge & White!!
		}
		else if (session_score==31 || session_score==35 || session_score==38 || session_score==39 || session_score==42) {
			display_target(t_test2); // huge & White!!
		}
		else if (session_score==3 || session_score==6 || session_score==7 || session_score==9 || session_score==12) {
			display_target(t_test0); // small & White!!
		}
		else if (session_score==16 || session_score==21 || session_score==22 || session_score==23 || session_score==30) {
			display_target(t_test0); // small & White!!
		}
		else if (session_score==33 || session_score==34 || session_score==41 || session_score==43 || session_score==44) {
			display_target(t_test0); // small & White!!
		}
		else {
			display_target(t_test); //  medium & White!!
		}

		

		// HP-2018/0118
		// t[0]=g_pos_data_t[3];
		//target position
		//g_pos_data_t[0] =    0 ;
		//g_pos_data_t[1] = -0.8 ;
		


			//HP 
		//showing target
		// 2018/0118 - dis-visable
		// display_target(t_test); // larget & White!!
		

		//glPushMatrix();
		//	glColor3f(1.0, 0.0, 0.0);
		//	sprintf(str_session_score, "%d",3);
		//	glprintbigbig(g_pos_data_t[0], g_pos_data_t[1], str_session_score);
		//glPopMatrix();


		
		temp=8;		 
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		/* check whether the target is reached or not */
//		is_reached = check_sensor_pos();
		
		if(goSound>0){
			system("aplay -q go.au");	
			goSound=0;
		}
		if(isReadySound){
			system("aplay -q Ready.au");
			isReadySound=0;
		}
		if(isElbow==1){
			system("aplay -q elbow.au");
			isElbow=0;
		}

		if ( is_reached>0 ) { //network is connecting

				//if (is_reached==99) //Suceess
				if (is_reached>98) //Suceess
				{

					//HP - change a success song
					//system("aplay -q success.au");
					session_score++;
					//cnt_target++;

					
					////////////////////////////////////////////////////////////
					//HP

					if (mode==1 && inmode ==1){

						system("aplay -q success.au");

						if ((cnt_target+1)%45==1){
							sprintf(str_session_score,"%d/45", session_score); 
						}
						else if ((cnt_target+1)%45==0){
							sprintf(str_session_score,"%d/44", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%45-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);	
						
						if ((cnt_target+1)%45==1 && session_score==45){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case accSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
						
					}

					// pre-test & retension test
					if (mode==1 && inmode ==2){  
						//system("aplay -q success.au");

						//if ((cnt_target+1)%60==1){
							//sprintf(str_session_score,"%d/35", session_score); 
						//}
						//else if ((cnt_target+1)%60 ==0){
							//sprintf(str_session_score,"%d/34", session_score); 
						//}
						//else{
							//sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%60)-26); 
						//}

						//setOrthographicProjection();
						//glLoadIdentity();
						//glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						
						
						//if ((cnt_target+1)%30==1 && session_score==30){
						//	glLoadIdentity();
						//	glTranslatef(0, 0, -5);
						//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
						//	resetPerspectiveProjection();
						//	isSwapBuffer=0;							
						//	display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
						//}
					}

					// training test possibly delete all this for just accuracy and general
					if (mode==2){
						//system("aplay -q success.au");

						if ((cnt_target+1)%100==1){
							sprintf(str_session_score,"%d/100", session_score); 
						}
						else if ((cnt_target+1)%100==0){
							sprintf(str_session_score,"%d/99", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%100-1)); 
						}
						
						//setOrthographicProjection();
						//glLoadIdentity();
						glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						



						//NEW : START -----------------------------------------------
						//normal
						if (is_reached==99){
							system("aplay -q success.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au");
							//display_target(t_best); // larget & Yello!!
						}

						//best score
						if (is_reached==100){
							//resetPerspectiveProjection();		
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();		
							isSwapBuffer=0;
							display_target(t_best); // big & green!!
							glutSwapBuffers();
							
							//setOrthographicProjection();

							system("aplay -q verygood.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au"); //remove
						}

						//better than 2std
						if (is_reached==101){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_very); // big & Yellow!!
							glutSwapBuffers();
							system("aplay -q verygood.au");
							//Best=2;
							
						}

						//bettern than 1std
						if (is_reached==102){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_good); // big & white!! 
							glutSwapBuffers();
							system("aplay -q good.au");
						}

						//within 1std
						if (is_reached==103){
							system("aplay -q success.au");
							//system("aplay -q good.au");
							//display_target(t_test); // little bit smaller & White!!
						}

						//worse than 1std && better than 2std
						if (is_reached==104){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_ok); // larget & flashing Yello!!
							glutSwapBuffers();
							system("aplay -q ok.au");
							//Best=3;
							
						}

						//worse than 2std 
						if (is_reached==105){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_harder); // larget & flashing Yello!!
							glutSwapBuffers();
							// system("aplay -q harder.au");
							system("aplay -q faster.au");
							//Best=4;					

						}

						//NEW : END -----------------------------------------------


					}


					///////////////////////////////////////////////////////////////////////////
					// BART test
					if (mode==3){ 
						system("aplay -q success.au");

						if ((cnt_target+1)%30==1){
							sprintf(str_session_score,"%d/30", session_score); 
						}
						else if ((cnt_target+1)%30==0){
							sprintf(str_session_score,"%d/29", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%30-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);

						
						if ((cnt_target+1)%30==1 && session_score==30){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case accSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");

							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
					}		
				////////////////////////////////////////////////////////////
				}
				else{ // temp=98;
					system("aplay -q failure.au");
				}
				
				is_reached=0;


		} //if ( is_reached>0 ) { //network is connecting 
		else { 
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;


	
	// case 12, g_scene 26
	//writing log file in bart core
	case accSC_TEST_RETURN:
		glprintbigbig(0.5,0.7,str_session_score); //case accSC_TEST_RETURN:
			//if (cnt_target%10==0 && session_score==10){
			//	glprintbigbig(0.5,0.6,"PERFECT!");
			//	display_target(t_best); // larget & Yello!!
			//}
			
		//target position
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;
		
		//HP 
		//showing target
		display_target(t_test);

		temp=8;	 // 9 case accSC_TEST_READY1:	
		memcpy(c,&temp,sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		//HP
		if ( is_reached>0 ) {
				system("aplay -q success.au");
				is_reached=0;
				cnt_target++;
				cnt_trial = 1;
				is_reached = 0;
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;

	//-----------------------------------------------------
	
	
	// case 13 
	// g_scene= g_scene 27
	case accSC_TEST_INFO_2:
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
			glprintbig(w/2-110,h/2-60,"END OF Accuracy SESSION 1");
		glPopMatrix();
		resetPerspectiveProjection();
	break; 

	// //, g_scene 28
	// /* start the test */
	// case accSC_TEST_INFO_A:
	// 	setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		glprintbig(w/2-120,h/2-60,"STEP 2. General SESSION 1");
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;

	// // case 15, g_scene 29, accuracy ends here
	// case accSC_TEST_INFO_3:
	// 	setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
	// 		glprintbig(w/2-60,h/2-50, "press enter to continue");
	// 		//HP
	// 		//system("aplay -q clap.au"); //HP -clap.au
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;

	
	// // case 16
	// case accSC_END:
	// setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
	// 		glprintbig(w/2-60,h/2-50, "press enter AGAIN to continue");
	// 		//HP
	// 		//system("aplay -q clap.au"); //HP -clap.au
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;


// ##################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	
	//general test 1
	// case 8 g_scene 22
	/* start the test */
	case genSC_TEST_INFO_1:

		cnt_trial = 1; cnt_target = 1; cnt_rms = 1;
	
		session_score=0;
		
		mode =1;
		inmode=1;
		

		loadparamGeneral();

		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			glprintbig(w/2-115,h/2-60,"START OF General SESSION 1");
			//glprint(w/2-80,h/2-20, "ENTER - Start");
			//HP
			//system("aplay -q clap.au"); //HP -clap.au
		glPopMatrix();
		resetPerspectiveProjection();
	break;  // press 2

	
	// case 9  , gscene 23
	case genSC_TEST_READY1:
		if (mode==1){  //HP - retention part
			
			if (inmode ==1) {
				if (cnt_target==1){
					sprintf(str_session_score,"0/0");
				}
//cnt equals for specific test
				else if (cnt_target%30 ==0){
					sprintf(str_session_score,"%d/29",session_score);
				}
				else if (cnt_target%30 ==1){
					sprintf(str_session_score,"%d/30",session_score);
				}
				else{
					sprintf(str_session_score,"%d/%d", session_score,(cnt_target%30-1)); //case genSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				}

				//HP - here change the character size
				glprintbigbig(0.5,0.7,str_session_score);

 				if ((cnt_target%30)==1)
				{  // ((cnt_target+1)%30==1 && session_score==30){
					session_score=0;
					inmode=1;
					mode=2;
				}
			}
			///////////////////////////////////////////////////////////////
			
			if (inmode ==2){
				//HP
				//cnt_target_30= cnt_target-15;

				//if (cnt_target ==26)
					//sprintf(str_session_score,"0/0");
				//else if (cnt_target %60==0)
					//sprintf(str_session_score,"%d/34",session_score);
				//else if (cnt_target %60==1)
					//sprintf(str_session_score,"%d/35",session_score);
				//else{
					//sprintf(str_session_score,"%d/%d", session_score,( (cnt_target%60)-26)); //case genSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				//}

				//HP - here change the character size
				//glprintbigbig(0.5,0.7,str_session_score);

 				//if (((cnt_target)%30)==1)
				//{
				//	session_score=0;
				//	inmode=1;
				//	mode=2;
				//}
			}		

			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1


		}

		//////////////////////////////////////////////////////////////////
		if (mode==2){   //HP - training part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%100 ==0)
				sprintf(str_session_score,"%d/99",session_score);
			else if (cnt_target%100 ==1)
				sprintf(str_session_score,"%d/100",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%100-1)); //case genSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.5,0.7,str_session_score);

 			if ((cnt_target%100)==1)
			{
				session_score=0;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}


		//////////////////////////////////////////////////////////////////
		if (mode==3){   //HP - BART part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%30 ==0)
				sprintf(str_session_score,"%d/29",session_score);
			else if (cnt_target%30 ==1)
				sprintf(str_session_score,"%d/30",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%30-1)); //case genSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.5,0.7,str_session_score);

 			if ((cnt_target%30)==1)
			{
				session_score=0;
				inmode=1;
				mode=2;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}

	break;

		


	// case 10, g_scene 24
	case genSC_TEST_READY2:
		glprintbigbig(0.5,0.7,str_session_score); //case genSC_TEST_READY2:
       
		//HP 
		//showing target
		display_target(t_ready);  // large & Green!!


		//system("aplay -q go.au");	
	break;
	
	

	// case 11, g_scene 25
	case genSC_TEST_GO:

		//setOrthographicProjection();
		//glLoadIdentity();
		glprintbigbig(0.5,0.7,str_session_score); //case genSC_TEST_GO:
		//resetPerspectiveProjection();
		//HP - "g_sched should be changed!!"
		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;
		
		
		


			//HP 
		//showing target
		display_target(t_test); // larget & White!!
		

		//glPushMatrix();
		//	glColor3f(1.0, 0.0, 0.0);
		//	sprintf(str_session_score, "%d",3);
		//	glprintbigbig(g_pos_data_t[0], g_pos_data_t[1], str_session_score);
		//glPopMatrix();


		
		temp=8;		 
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		/* check whether the target is reached or not */
//		is_reached = check_sensor_pos();
		
		if(goSound>0){
			system("aplay -q go.au");	
			goSound=0;
		}
		if(isReadySound){
			system("aplay -q Ready.au");
			isReadySound=0;
		}
		if(isElbow==1){
			system("aplay -q elbow.au");
			isElbow=0;
		}

		if ( is_reached>0 ) { //network is connecting

				//if (is_reached==99) //Suceess
				if (is_reached>98) //Suceess
				{

					//HP - change a success song
					//system("aplay -q success.au");
					session_score++;
					//cnt_target++;

					
					////////////////////////////////////////////////////////////
					//HP

					if (mode==1 && inmode ==1){

						system("aplay -q success.au");

						if ((cnt_target+1)%30==1){
							sprintf(str_session_score,"%d/30", session_score); 
						}
						else if ((cnt_target+1)%30==0){
							sprintf(str_session_score,"%d/29", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%30-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);	
						
						if ((cnt_target+1)%30==1 && session_score==30){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case genSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
						
					}

					// pre-test & retension test
					if (mode==1 && inmode ==2){  
						//system("aplay -q success.au");

						//if ((cnt_target+1)%60==1){
							//sprintf(str_session_score,"%d/35", session_score); 
						//}
						//else if ((cnt_target+1)%60 ==0){
							//sprintf(str_session_score,"%d/34", session_score); 
						//}
						//else{
							//sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%60)-26); 
						//}

						//setOrthographicProjection();
						//glLoadIdentity();
						//glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						
						
						//if ((cnt_target+1)%30==1 && session_score==30){
						//	glLoadIdentity();
						//	glTranslatef(0, 0, -5);
						//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
						//	resetPerspectiveProjection();
						//	isSwapBuffer=0;							
						//	display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
						//}
					}

					// training test possibly delete all this for just accuracy and general
					if (mode==2){
						//system("aplay -q success.au");

						if ((cnt_target+1)%100==1){
							sprintf(str_session_score,"%d/100", session_score); 
						}
						else if ((cnt_target+1)%100==0){
							sprintf(str_session_score,"%d/99", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%100-1)); 
						}
						
						//setOrthographicProjection();
						//glLoadIdentity();
						glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						



						//NEW : START -----------------------------------------------
						//normal
						if (is_reached==99){
							system("aplay -q success.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au");
							//display_target(t_best); // larget & Yello!!
						}

						//best score
						if (is_reached==100){
							//resetPerspectiveProjection();		
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();		
							isSwapBuffer=0;
							display_target(t_best); // big & green!!
							glutSwapBuffers();
							
							//setOrthographicProjection();

							system("aplay -q verygood.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au"); //remove
						}

						//better than 2std
						if (is_reached==101){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_very); // big & Yellow!!
							glutSwapBuffers();
							system("aplay -q verygood.au");
							//Best=2;
							
						}

						//bettern than 1std
						if (is_reached==102){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_good); // big & white!! 
							glutSwapBuffers();
							system("aplay -q good.au");
						}

						//within 1std
						if (is_reached==103){
							system("aplay -q success.au");
							//system("aplay -q good.au");
							//display_target(t_test); // little bit smaller & White!!
						}

						//worse than 1std && better than 2std
						if (is_reached==104){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_ok); // larget & flashing Yello!!
							glutSwapBuffers();
							system("aplay -q ok.au");
							//Best=3;
							
						}

						//worse than 2std 
						if (is_reached==105){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_harder); // larget & flashing Yello!!
							glutSwapBuffers();
							// system("aplay -q harder.au");
							system("aplay -q faster.au");
							//Best=4;					

						}

						//NEW : END -----------------------------------------------


					}


					///////////////////////////////////////////////////////////////////////////
					// BART test
					if (mode==3){ 
						system("aplay -q success.au");

						if ((cnt_target+1)%30==1){
							sprintf(str_session_score,"%d/30", session_score); 
						}
						else if ((cnt_target+1)%30==0){
							sprintf(str_session_score,"%d/29", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%30-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);

						
						if ((cnt_target+1)%30==1 && session_score==30){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case genSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");

							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
					}		
				////////////////////////////////////////////////////////////
				}
				else{ // temp=98;
					system("aplay -q failure.au");
				}
				
				is_reached=0;


		} //if ( is_reached>0 ) { //network is connecting 
		else { 
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;


	
	// case 12, g_scene 26
	//writing log file in bart core
	case genSC_TEST_RETURN:
		glprintbigbig(0.5,0.7,str_session_score); //case genSC_TEST_RETURN:
			//if (cnt_target%10==0 && session_score==10){
			//	glprintbigbig(0.5,0.6,"PERFECT!");
			//	display_target(t_best); // larget & Yello!!
			//}
			
		//target position
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;
		
		//HP 
		//showing target
		display_target(t_test);

		temp=8;	 // 9 case genSC_TEST_READY1:	
		memcpy(c,&temp,sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		//HP
		if ( is_reached>0 ) {
				system("aplay -q success.au");
				is_reached=0;
				cnt_target++;
				cnt_trial = 1;
				is_reached = 0;
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;

	//-----------------------------------------------------
	
	
	// case 13 
	// g_scene= g_scene 27
	case genSC_TEST_INFO_2:
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
			glprintbig(w/2-110,h/2-60,"END OF General SESSION 1");
			glprintbig(w/2-110,h/2+20,"Switch Sensors to Other Arm");
		glPopMatrix();
		resetPerspectiveProjection();
	break; 

	// // case 14, g_scene 28
	// /* start the test */
	// case genSC_TEST_INFO_A:
	// 	setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		glprintbig(w/2-120,h/2-60,"hit enter to continue");
	// 		//glprint(w/2-110,h/2-20, "ENTER - Start");
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;

	// // case 15, g_scene 29, accuracy ends here
	// case genSC_TEST_INFO_3:
	// 	setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
	// 		glprintbig(w/2-60,h/2-50, "hit enter again");
	// 		//HP
	// 		//system("aplay -q clap.au"); //HP -clap.au
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;

	
	// // case 16
	// case genSC_END:
	// setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
	// 		glprintbig(w/2-60,h/2-50, "hit enter one more time");
	// 		//HP
	// 		//system("aplay -q clap.au"); //HP -clap.au
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;

// end of general test 1.
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	

//accuracy test 2

	// case 8 , g_scene 22
	/* start the test */
	case acc2SC_TEST_INFO_1:
	//reset some of the variables
		cnt_trial = 1; cnt_target = 1; cnt_rms = 1;
	
		session_score=0;
		
		mode =1;
		inmode=1;

		//get accuracy testing parameters
		loadparam();
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			glprintbig(w/2-115,h/2-60,"START OF Accuracy SESSION 2");
			//glprint(w/2-80,h/2-20, "ENTER - Start");
			//HP
			//system("aplay -q clap.au"); //HP -clap.au
		glPopMatrix();
		resetPerspectiveProjection();
	break;  // press 2

	
	// case 9 , g_scene 23
	case acc2SC_TEST_READY1:
		if (mode==1){  //HP - retention part
			
			if (inmode ==1) {
				if (cnt_target==1){
					sprintf(str_session_score,"0/0");
				}
				//cnt equals for specific test
				else if (cnt_target%45 ==0){
					sprintf(str_session_score,"%d/44",session_score);
				}
				else if (cnt_target%45 ==1){
					sprintf(str_session_score,"%d/45",session_score);
				}
				else{
					sprintf(str_session_score,"%d/%d", session_score,(cnt_target%45-1)); //case accSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				}

				//HP - here change the character size
				glprintbigbig(0.5,0.7,str_session_score);

 				if ((cnt_target%45)==1){  // ((cnt_target+1)%30==1 && session_score==30){
					//session_score=0;
					session_score=0;
					inmode=1;
					mode=2;
				}
			}
			///////////////////////////////////////////////////////////////
			
			if (inmode ==2){
				//HP
				//cnt_target_30= cnt_target-15;

				//if (cnt_target ==26)
					//sprintf(str_session_score,"0/0");
				//else if (cnt_target %60==0)
					//sprintf(str_session_score,"%d/34",session_score);
				//else if (cnt_target %60==1)
					//sprintf(str_session_score,"%d/35",session_score);
				//else{
					//sprintf(str_session_score,"%d/%d", session_score,( (cnt_target%60)-26)); //case accSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				//}

				//HP - here change the character size
				//glprintbigbig(0.5,0.7,str_session_score);

 				//if (((cnt_target)%30)==1)
				//{
				//	session_score=0;
				//	inmode=1;
				//	mode=2;
				//}
			}		

			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1


		}

		//////////////////////////////////////////////////////////////////
		if (mode==2){   //HP - training part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%100 ==0)
				sprintf(str_session_score,"%d/99",session_score);
			else if (cnt_target%100 ==1)
				sprintf(str_session_score,"%d/100",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%100-1)); //case accSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.5,0.7,str_session_score);

 			if ((cnt_target%100)==1)
			{
				session_score=0;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}


		//////////////////////////////////////////////////////////////////
		if (mode==3){   //HP - BART part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%30 ==0)
				sprintf(str_session_score,"%d/29",session_score);
			else if (cnt_target%30 ==1)
				sprintf(str_session_score,"%d/30",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%30-1)); //case accSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.5,0.7,str_session_score);

 			if ((cnt_target%30)==1)
			{
				session_score=0;
				inmode=1;
				mode=2;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}

	break;

		


	// case 10, g_scene 24
	case acc2SC_TEST_READY2:
		glprintbigbig(0.5,0.7,str_session_score); //case accSC_TEST_READY2:
       
		//HP 
		//showing target
		display_target(t_ready);  // large & Green!!


		//system("aplay -q go.au");	
	break;
	
	

	// case 11,  printed g_scene 25
	case acc2SC_TEST_GO:

		//setOrthographicProjection();
		//glLoadIdentity();
		glprintbigbig(0.5,0.7,str_session_score); //case accSC_TEST_GO:
		//resetPerspectiveProjection();
		//HP - "g_sched should be changed!!"

		//sets target screen calibrated X and Y position based on param load from file
		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;
		

		//HP -2018 (only need)
		//Baylor=g_pos_test[g_sched[cnt_target-1]-1][3];

		// size changes to large and small based on which iteration of reaching we are on
		if (session_score==0 || session_score==1 || session_score==4 || session_score==10 || session_score==14) {
			display_target(t_test2); // huge & White!!
		}
		else if (session_score==17 || session_score==19 || session_score==20 || session_score==26 || session_score==29) {
			display_target(t_test2); // huge & White!!
		}
		else if (session_score==31 || session_score==35 || session_score==38 || session_score==39 || session_score==42) {
			display_target(t_test2); // huge & White!!
		}
		else if (session_score==3 || session_score==6 || session_score==7 || session_score==9 || session_score==12) {
			display_target(t_test0); // small & White!!
		}
		else if (session_score==16 || session_score==21 || session_score==22 || session_score==23 || session_score==30) {
			display_target(t_test0); // small & White!!
		}
		else if (session_score==33 || session_score==34 || session_score==41 || session_score==43 || session_score==44) {
			display_target(t_test0); // small & White!!
		}
		else {
			display_target(t_test); //  medium & White!!
		}

		

		// HP-2018/0118
		// t[0]=g_pos_data_t[3];
		//target position
		//g_pos_data_t[0] =    0 ;
		//g_pos_data_t[1] = -0.8 ;
		


			//HP 
		//showing target
		// 2018/0118 - dis-visable
		// display_target(t_test); // larget & White!!
		

		//glPushMatrix();
		//	glColor3f(1.0, 0.0, 0.0);
		//	sprintf(str_session_score, "%d",3);
		//	glprintbigbig(g_pos_data_t[0], g_pos_data_t[1], str_session_score);
		//glPopMatrix();


		
		temp=8;		 
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		/* check whether the target is reached or not */
//		is_reached = check_sensor_pos();
		
		if(goSound>0){
			system("aplay -q go.au");	
			goSound=0;
		}
		if(isReadySound){
			system("aplay -q Ready.au");
			isReadySound=0;
		}
		if(isElbow==1){
			system("aplay -q elbow.au");
			isElbow=0;
		}

		if ( is_reached>0 ) { //network is connecting

				//if (is_reached==99) //Suceess
				if (is_reached>98) //Suceess
				{

					//HP - change a success song
					//system("aplay -q success.au");
					session_score++;
					//cnt_target++;

					
					////////////////////////////////////////////////////////////
					//HP

					if (mode==1 && inmode ==1){

						system("aplay -q success.au");

						if ((cnt_target+1)%45==1){
							sprintf(str_session_score,"%d/45", session_score); 
						}
						else if ((cnt_target+1)%45==0){
							sprintf(str_session_score,"%d/44", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%45-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);	
						
						if ((cnt_target+1)%45==1 && session_score==45){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case accSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
						
					}

					// pre-test & retension test
					if (mode==1 && inmode ==2){  
						//system("aplay -q success.au");

						//if ((cnt_target+1)%60==1){
							//sprintf(str_session_score,"%d/35", session_score); 
						//}
						//else if ((cnt_target+1)%60 ==0){
							//sprintf(str_session_score,"%d/34", session_score); 
						//}
						//else{
							//sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%60)-26); 
						//}

						//setOrthographicProjection();
						//glLoadIdentity();
						//glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						
						
						//if ((cnt_target+1)%30==1 && session_score==30){
						//	glLoadIdentity();
						//	glTranslatef(0, 0, -5);
						//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
						//	resetPerspectiveProjection();
						//	isSwapBuffer=0;							
						//	display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
						//}
					}

					// training test possibly delete all this for just accuracy and general
					if (mode==2){
						//system("aplay -q success.au");

						if ((cnt_target+1)%100==1){
							sprintf(str_session_score,"%d/100", session_score); 
						}
						else if ((cnt_target+1)%100==0){
							sprintf(str_session_score,"%d/99", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%100-1)); 
						}
						
						//setOrthographicProjection();
						//glLoadIdentity();
						glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						



						//NEW : START -----------------------------------------------
						//normal
						if (is_reached==99){
							system("aplay -q success.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au");
							//display_target(t_best); // larget & Yello!!
						}

						//best score
						if (is_reached==100){
							//resetPerspectiveProjection();		
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();		
							isSwapBuffer=0;
							display_target(t_best); // big & green!!
							glutSwapBuffers();
							
							//setOrthographicProjection();

							system("aplay -q verygood.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au"); //remove
						}

						//better than 2std
						if (is_reached==101){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_very); // big & Yellow!!
							glutSwapBuffers();
							system("aplay -q verygood.au");
							//Best=2;
							
						}

						//bettern than 1std
						if (is_reached==102){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_good); // big & white!! 
							glutSwapBuffers();
							system("aplay -q good.au");
						}

						//within 1std
						if (is_reached==103){
							system("aplay -q success.au");
							//system("aplay -q good.au");
							//display_target(t_test); // little bit smaller & White!!
						}

						//worse than 1std && better than 2std
						if (is_reached==104){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_ok); // larget & flashing Yello!!
							glutSwapBuffers();
							system("aplay -q ok.au");
							//Best=3;
							
						}

						//worse than 2std 
						if (is_reached==105){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_harder); // larget & flashing Yello!!
							glutSwapBuffers();
							// system("aplay -q harder.au");
							system("aplay -q faster.au");
							//Best=4;					

						}

						//NEW : END -----------------------------------------------


					}


					///////////////////////////////////////////////////////////////////////////
					// BART test
					if (mode==3){ 
						system("aplay -q success.au");

						if ((cnt_target+1)%30==1){
							sprintf(str_session_score,"%d/30", session_score); 
						}
						else if ((cnt_target+1)%30==0){
							sprintf(str_session_score,"%d/29", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%30-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);

						
						if ((cnt_target+1)%30==1 && session_score==30){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case accSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");

							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
					}		
				////////////////////////////////////////////////////////////
				}
				else{ // temp=98;
					system("aplay -q failure.au");
				}
				
				is_reached=0;


		} //if ( is_reached>0 ) { //network is connecting 
		else { 
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;


	
	// case 12, g_scene 26
	//writing log file in bart core
	case acc2SC_TEST_RETURN:
		glprintbigbig(0.5,0.7,str_session_score); //case accSC_TEST_RETURN:
			//if (cnt_target%10==0 && session_score==10){
			//	glprintbigbig(0.5,0.6,"PERFECT!");
			//	display_target(t_best); // larget & Yello!!
			//}
			
		//target position
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;
		
		//HP 
		//showing target
		display_target(t_test);

		temp=8;	 // 9 case accSC_TEST_READY1:	
		memcpy(c,&temp,sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		//HP
		if ( is_reached>0 ) {
				system("aplay -q success.au");
				is_reached=0;
				cnt_target++;
				cnt_trial = 1;
				is_reached = 0;
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;

	//-----------------------------------------------------
	
	
	// case 13 
	// g_scene= g_scene 27
	case acc2SC_TEST_INFO_2:
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
			glprintbig(w/2-110,h/2-60,"END OF Accuracy SESSION 2");
		glPopMatrix();
		resetPerspectiveProjection();
	break; 

	
	// // case 16
	// case acc2SC_END:
	// setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
	// 		glprintbig(w/2-60,h/2-50, "press enter to continue");
	// 		//HP
	// 		//system("aplay -q clap.au"); //HP -clap.au
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;
	
// ##################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	
	//general test 2
	// case 8 g_scene 22
	/* start the test */
	case gen2SC_TEST_INFO_1:

		cnt_trial = 1; cnt_target = 1; cnt_rms = 1;
	
		session_score=0;
		
		mode =1;
		inmode=1;
		

		loadparamGeneral();

		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			glprintbig(w/2-115,h/2-60,"START OF General SESSION 2");
			//glprint(w/2-80,h/2-20, "ENTER - Start");
			//HP
			//system("aplay -q clap.au"); //HP -clap.au
		glPopMatrix();
		resetPerspectiveProjection();
	break;  // press 2

	
	// case 9  , gscene 23
	case gen2SC_TEST_READY1:
		if (mode==1){  //HP - retention part
			
			if (inmode ==1) {
				if (cnt_target==1){
					sprintf(str_session_score,"0/0");
				}
//cnt equals for specific test
				else if (cnt_target%30 ==0){
					sprintf(str_session_score,"%d/29",session_score);
				}
				else if (cnt_target%30 ==1){
					sprintf(str_session_score,"%d/30",session_score);
				}
				else{
					sprintf(str_session_score,"%d/%d", session_score,(cnt_target%30-1)); //case genSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				}

				//HP - here change the character size
				glprintbigbig(0.5,0.7,str_session_score);

 				if ((cnt_target%30)==1)
				{  // ((cnt_target+1)%30==1 && session_score==30){
					session_score=0;
					inmode=1;
					mode=2;
				}
			}
			///////////////////////////////////////////////////////////////
			
			if (inmode ==2){
				//HP
				//cnt_target_30= cnt_target-15;

				//if (cnt_target ==26)
					//sprintf(str_session_score,"0/0");
				//else if (cnt_target %60==0)
					//sprintf(str_session_score,"%d/34",session_score);
				//else if (cnt_target %60==1)
					//sprintf(str_session_score,"%d/35",session_score);
				//else{
					//sprintf(str_session_score,"%d/%d", session_score,( (cnt_target%60)-26)); //case genSC_TEST_READY1:
					//system("aplay -q clap.au"); //HP -clap.au
				//}

				//HP - here change the character size
				//glprintbigbig(0.5,0.7,str_session_score);

 				//if (((cnt_target)%30)==1)
				//{
				//	session_score=0;
				//	inmode=1;
				//	mode=2;
				//}
			}		

			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1


		}

		//////////////////////////////////////////////////////////////////
		if (mode==2){   //HP - training part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%100 ==0)
				sprintf(str_session_score,"%d/99",session_score);
			else if (cnt_target%100 ==1)
				sprintf(str_session_score,"%d/100",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%100-1)); //case genSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.5,0.7,str_session_score);

 			if ((cnt_target%100)==1)
			{
				session_score=0;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}


		//////////////////////////////////////////////////////////////////
		if (mode==3){   //HP - BART part
			if (cnt_target==1)
				sprintf(str_session_score,"0/0");
			else if (cnt_target%30 ==0)
				sprintf(str_session_score,"%d/29",session_score);
			else if (cnt_target%30 ==1)
				sprintf(str_session_score,"%d/30",session_score);
			else{
				sprintf(str_session_score,"%d/%d", session_score,(cnt_target%30-1)); //case genSC_TEST_READY1:
				//system("aplay -q clap.au"); //HP -clap.au
			}

			//HP - here change the character size
			glprintbigbig(0.5,0.7,str_session_score);

 			if ((cnt_target%30)==1)
			{
				session_score=0;
				inmode=1;
				mode=2;
			}
			//initial position
			g_pos_data_t[0] = 0;
			g_pos_data_t[1] = -0.8;
			//HP -consideration!!!
			display_target(t_ready);  // large & Green!! #1
		}

	break;

		


	// case 10, g_scene 24
	case gen2SC_TEST_READY2:
		glprintbigbig(0.5,0.7,str_session_score); //case genSC_TEST_READY2:
       
		//HP 
		//showing target
		display_target(t_ready);  // large & Green!!


		//system("aplay -q go.au");	
	break;
	
	

	// case 11, g_scene 25
	case gen2SC_TEST_GO:

		//setOrthographicProjection();
		//glLoadIdentity();
		glprintbigbig(0.5,0.7,str_session_score); //case genSC_TEST_GO:
		//resetPerspectiveProjection();
		//HP - "g_sched should be changed!!"
		g_pos_data_t[0] =    0 + g_pos_test[g_sched[cnt_target-1]-1][0]/g_real_w*g_ratio;
		g_pos_data_t[1] = -0.8 + g_pos_test[g_sched[cnt_target-1]-1][1]/g_real_h;
		
		
		


			//HP 
		//showing target
		display_target(t_test); // larget & White!!
		

		//glPushMatrix();
		//	glColor3f(1.0, 0.0, 0.0);
		//	sprintf(str_session_score, "%d",3);
		//	glprintbigbig(g_pos_data_t[0], g_pos_data_t[1], str_session_score);
		//glPopMatrix();


		
		temp=8;		 
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		/* check whether the target is reached or not */
//		is_reached = check_sensor_pos();
		
		if(goSound>0){
			system("aplay -q go.au");	
			goSound=0;
		}
		if(isReadySound){
			system("aplay -q Ready.au");
			isReadySound=0;
		}
		if(isElbow==1){
			system("aplay -q elbow.au");
			isElbow=0;
		}

		if ( is_reached>0 ) { //network is connecting

				//if (is_reached==99) //Suceess
				if (is_reached>98) //Suceess
				{

					//HP - change a success song
					//system("aplay -q success.au");
					session_score++;
					//cnt_target++;

					
					////////////////////////////////////////////////////////////
					//HP

					if (mode==1 && inmode ==1){

						system("aplay -q success.au");

						if ((cnt_target+1)%30==1){
							sprintf(str_session_score,"%d/30", session_score); 
						}
						else if ((cnt_target+1)%30==0){
							sprintf(str_session_score,"%d/29", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%30-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);	
						
						if ((cnt_target+1)%30==1 && session_score==30){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case genSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
						
					}

					// pre-test & retension test
					if (mode==1 && inmode ==2){  
						//system("aplay -q success.au");

						//if ((cnt_target+1)%60==1){
							//sprintf(str_session_score,"%d/35", session_score); 
						//}
						//else if ((cnt_target+1)%60 ==0){
							//sprintf(str_session_score,"%d/34", session_score); 
						//}
						//else{
							//sprintf(str_session_score,"%d/%d", session_score, ((cnt_target+1)%60)-26); 
						//}

						//setOrthographicProjection();
						//glLoadIdentity();
						//glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						
						
						//if ((cnt_target+1)%30==1 && session_score==30){
						//	glLoadIdentity();
						//	glTranslatef(0, 0, -5);
						//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
						//	resetPerspectiveProjection();
						//	isSwapBuffer=0;							
						//	display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");
						//}
					}

					// training test possibly delete all this for just accuracy and general
					if (mode==2){
						//system("aplay -q success.au");

						if ((cnt_target+1)%100==1){
							sprintf(str_session_score,"%d/100", session_score); 
						}
						else if ((cnt_target+1)%100==0){
							sprintf(str_session_score,"%d/99", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%100-1)); 
						}
						
						//setOrthographicProjection();
						//glLoadIdentity();
						glprintbigbig(0.5,0.7,str_session_score);
						//resetPerspectiveProjection();						



						//NEW : START -----------------------------------------------
						//normal
						if (is_reached==99){
							system("aplay -q success.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au");
							//display_target(t_best); // larget & Yello!!
						}

						//best score
						if (is_reached==100){
							//resetPerspectiveProjection();		
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();		
							isSwapBuffer=0;
							display_target(t_best); // big & green!!
							glutSwapBuffers();
							
							//setOrthographicProjection();

							system("aplay -q verygood.au");
							//system("aplay -q best.au");
							//system("aplay -q clap.au"); //remove
						}

						//better than 2std
						if (is_reached==101){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_very); // big & Yellow!!
							glutSwapBuffers();
							system("aplay -q verygood.au");
							//Best=2;
							
						}

						//bettern than 1std
						if (is_reached==102){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_good); // big & white!! 
							glutSwapBuffers();
							system("aplay -q good.au");
						}

						//within 1std
						if (is_reached==103){
							system("aplay -q success.au");
							//system("aplay -q good.au");
							//display_target(t_test); // little bit smaller & White!!
						}

						//worse than 1std && better than 2std
						if (is_reached==104){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_ok); // larget & flashing Yello!!
							glutSwapBuffers();
							system("aplay -q ok.au");
							//Best=3;
							
						}

						//worse than 2std 
						if (is_reached==105){
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();	
							isSwapBuffer=0;
							display_target(t_harder); // larget & flashing Yello!!
							glutSwapBuffers();
							// system("aplay -q harder.au");
							system("aplay -q faster.au");
							//Best=4;					

						}

						//NEW : END -----------------------------------------------


					}


					///////////////////////////////////////////////////////////////////////////
					// BART test
					if (mode==3){ 
						system("aplay -q success.au");

						if ((cnt_target+1)%30==1){
							sprintf(str_session_score,"%d/30", session_score); 
						}
						else if ((cnt_target+1)%30==0){
							sprintf(str_session_score,"%d/29", session_score); 
						}
						else{
							sprintf(str_session_score,"%d/%d", session_score,((cnt_target+1)%30-1)); 
						}
						
						glprintbigbig(0.5,0.7,str_session_score);

						
						if ((cnt_target+1)%30==1 && session_score==30){
							//sprintf(str_session_score,"%d/%d", session_score, 100); 
							//glprintbigbig(0.5,0.5,str_session_score); //case genSC_TEST_RETURN:
							glLoadIdentity();
							glTranslatef(0, 0, -5);
							glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
							resetPerspectiveProjection();
							isSwapBuffer=0;							
							display_target(t_best); // larget & Yello!!
							//system("aplay -q exactly.au");

							//glprintbig(0.5,0.6,"PERFECT!!");
							//system("aplay -q clap.au"); //HP -works!!!
						}
					}		
				////////////////////////////////////////////////////////////
				}
				else{ // temp=98;
					system("aplay -q failure.au");
				}
				
				is_reached=0;


		} //if ( is_reached>0 ) { //network is connecting 
		else { 
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;


	
	// case 12, g_scene 26
	//writing log file in bart core
	case gen2SC_TEST_RETURN:
		glprintbigbig(0.5,0.7,str_session_score); //case genSC_TEST_RETURN:
			//if (cnt_target%10==0 && session_score==10){
			//	glprintbigbig(0.5,0.6,"PERFECT!");
			//	display_target(t_best); // larget & Yello!!
			//}
			
		//target position
		g_pos_data_t[0] =    0 ;
		g_pos_data_t[1] = -0.8 ;
		
		//HP 
		//showing target
		display_target(t_test);

		temp=8;	 // 9 case genSC_TEST_READY1:	
		memcpy(c,&temp,sizeof(int));

		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %d [%d]\n",udp_id,temp,nret);


		//HP
		if ( is_reached>0 ) {
				system("aplay -q success.au");
				is_reached=0;
				cnt_target++;
				cnt_trial = 1;
				is_reached = 0;
		} 
		else {
			cnt_trial++;				/* try again with the same target */
		}	/* if (cnt_trial == maxtrial || is_reached) */

	break;

	//-----------------------------------------------------
	
	
	// case 13 
	// g_scene= g_scene 27
	case gen2SC_TEST_INFO_2:
		setOrthographicProjection();
		glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
			glprintbig(w/2-110,h/2-60,"END OF General SESSION 2");
			glprintbig(w/2-110,h/2-160,"hit enter and you may exit this screen");
			
		glPopMatrix();
		resetPerspectiveProjection();

	break; 

	// // case 14, g_scene 28
	// /* start the test */
	// case gen2SC_TEST_INFO_A:
	// 	setOrthographicProjection();
	// 	glPushMatrix();
	// 		glColor3f(0.0,1.0,1.0);
	// 		glLoadIdentity();
	// 		glprintbig(w/2-120,h/2-60,"STEP 5. NO MORE SESSION");
	// 		//glprint(w/2-110,h/2-20, "ENTER - Start");
	// 	glPopMatrix();
	// 	resetPerspectiveProjection();
	// break;

	// // case 15, g_scene 29, accuracy ends here
	// case gen2SC_TEST_INFO_3:
		
	// break;

	
	// case 16
	case gen2SC_END:

	break;

// end of general test 2.
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
// #######################################################################################################################
	

	
	
	// case 17, the following cases DO NOT HAPPEN after accuracy test
	case SC_LOAD_MT:
		
		getMTfromFile(MTFILENAME);
		g_scene=SC_SHOW_MT;
	break;

	// case 18
	case SC_SHOW_MT:
		//setOrthographicProjection();
		if(IsTraining>=1){ // Pre-test??????IsTraining??0?̰?Ʈ???̴?? ??????IsTraining??????? ????. ??Ʈ???̴????????? MovTime?׷?? ?????ֱ?
			glPushMatrix();
			glTranslatef(-0.25, -0.7, 0);
			
			//HP
			renderMTResult();
			glPopMatrix();
			
			glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glTranslatef(-0.5, -0.9, 0);
			//glprintbig(w/2-110,h/2-60,"END OF TEST SESSION");
			//glprintbig(0,0,"END OF TEST SESSION");
			//glprint(0.2,-0.05, "#1 - move to the training session");
			//glprint(0.2,-0.1, "#2 - show training results if training completed");
			glPopMatrix();
			resetPerspectiveProjection();
		}
		else{
			setOrthographicProjection();
			glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
			glprintbig(w/2-110,h/2-60,"END OF TEST SESSION");
			//glprint(w/2-110,h/2-20, "#1 - move to the training session");
			//glprint(w/2-110,h/2, "#2 - show training results if training completed");
			//HP
			//system("aplay -q clap.au"); //HP -clap.au
			glPopMatrix();
			resetPerspectiveProjection();
		}

	break;


	// NEW --
	// case 19
	case SC_LOAD_MT1:
		getMTfromFile_t(MTFILENAME_T);
		g_scene=SC_SHOW_MT1;
	break;


	// case 20
	case SC_SHOW_MT1:
		//setOrthographicProjection();
		if(IsTraining_t >= 1){ // Pre-test??????IsTraining??0?̰?Ʈ???̴?? ??????IsTraining??????? ????. ??Ʈ???̴????????? MovTime?׷?? ?????ֱ?
			glPushMatrix();
			glTranslatef(-0.25, -0.7, 0);
			
			//HP
			if(showMode == 0){
				renderMTResult_t();
			}
			else{
				renderMTResult_tt(showTargetNum-1);
			}			
			glPopMatrix();
			
			glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glTranslatef(-0.5, -0.9, 0);	
			glprintbig(w/2-110,h/2-60,"END OF TEST SESSION");
// 			glprint(0.2,-0.05, "#1 - move to the training session");
// 			glprint(0.2,-0.1, "#2 - show training results if training completed");
			glPopMatrix();
			resetPerspectiveProjection();
		}
		else{
			setOrthographicProjection();
			glPushMatrix();
			glColor3f(0.0,1.0,1.0);
			glLoadIdentity();
			//glprintbig(w/2-100,h/2-60,"End of the pre-test session");
			glprintbig(w/2-110,h/2-60,"END OF TEST SESSION");
// 			glprint(w/2-110,h/2-20, "#1 - move to the training session");
// 			glprint(w/2-110,h/2, "#2 - show training results if training completed");
			//HP
			//system("aplay -q clap.au"); //HP -clap.au
			glPopMatrix();
			resetPerspectiveProjection();
		}

	break;


		
	}	/* switch(g_scene) */
	/* finalization */

	if(isSwapBuffer){
		glutSwapBuffers();
	}
	else{
		isSwapBuffer=1;
	}
}







// #12
void keyboard(unsigned char key, int x, int y)
{
	int nret;
	int temp;

// 	if (g_scene == gen2SC_END)
//		exit(0);
	switch (key) {

	/* go to the next step or go to start */
	case KEY_ENT:
	//case '5':
		// New Calib
		if (SC_CALIB_LU <= g_scene && g_scene <= SC_CALIB_L) {
			/* screen calibration */
			cnt_calib++;
			
		}
		//////////////////////////////////////////////////////////////////////////
		// Sensor2 Calib
		else if(SC_CALIB_INFO_LS == g_scene){
			InitializeVariables();
		}
		else if (SC_CALIB_LU_LS <= g_scene && g_scene <= SC_CALIB_L_LS) {
				/* screen calibration */
			cnt_calib++;
		
		}
		//////////////////////////////////////////////////////////////////////////


		temp=1; // HP: # 1					
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
	break;

	/* skip this step */
	case KEY_SPACE:
		temp=2;		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
	break;


	/* end a program */
	case KEY_ESC:
		temp=3;  // HP: # 3				
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
		exit(0);
	break;

	//training 1~10
	case '1':
		//cnt_target=1;
		InitializeVariables();
		loadparam_1();
		IsTraining++;
		temp=15;  
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
		printf("Parameters are loaded at Main\n");
	break;

	//MT 10 training session display
	case '2':
		NumOfTrainingSession++;
		temp=16;  
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
		printf("Parameters are loaded at Main\n");
	break;

	
	//retention session
	case '3':
		InitializeVariables();
		loadparam();
		temp=17;  
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
		printf("Parameters are loaded at Main_retention\n");

	break;

	//BART session
	case '4':
		InitializeVariables();
		loadparam_2();
		temp=18;  
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
		printf("Parameters are loaded at Main_BART\n");

	break;

		
	//NEW- 20 trial MT display
	case '5':
		//NumOfTrainingSession++;
		showMode=0;
		showTargetNum=0;
		temp=19;  
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
		printf("Parameters are loaded at Main\n");
	break;

	case '9':
		//NumOfTrainingSession++;
		showTargetNum++;
		if(showTargetNum > 5){
			showTargetNum=1;
		}
		showMode=1;  

		temp=19;  
		memcpy(c,&temp,sizeof(int));
		nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
		printf("[udp:%d] %s [%d]\n",udp_id,c,nret);
		printf("Parameters are loaded at Main\n");
	break;


	}
}


// #13
int ii,jj;
int main(int argc, char **argv)
{

	// Movement time
	for( ii=0 ; ii < 5 ; ii++){
		for( jj=0 ; jj < 10 ; jj++){
			MovTimeFromLog[ii][jj]=0.0;
		}
	}

	/* consider about using a configuration file */
//	sensor_init(2); //deprecated
	int temp;

	udp_id=CreateUnixUDP("./bart_gui.tmp");

	//TO BART_CORE: I AM UP! please send any initial data.
	temp=9;		
	memcpy(c,&temp,sizeof(int));
	
	nret=WriteMsgToUnixUDP(udp_id,"./bart_core.tmp",c,4);
	
	printf("[udp:%d] %s [%d] [sent:%d]\n",udp_id,c,(int)(c[0] | c[1]<<8 | c[2] <<16 | c[3]<<24 ),nret);	
	//printf("Oh dear, something went wrong with read()! %s\n", strerror(errno));
	if(nret<0){	
			printf("failure with %s\n", strerror(errno)); 
     			//exit(EXIT_FAILURE);
			
		}

	//again
	loadparam(); //load first params into file descriptor fd

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(w,h);
	glutCreateWindow("Reaching Task"); //extended one more window!

	glutKeyboardFunc(keyboard);

	glutReshapeFunc(reshape);
	glutDisplayFunc(display);

	glutIdleFunc(idledisplay); // process!!

	//usleep(3000000);  //3 sec
	//usleep(500,000);  //0.5 sec
	//usleep(10000000);  //10 sec

	display_init();
	glutMainLoop();

	return(0);
}

