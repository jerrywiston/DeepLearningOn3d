#ifndef CONFIG_H
#define CONFIG_H

#define RESOL					   256
#define RESOL_VOL 			 	   192
#define VOX_LEN 		 	  	   8.0f
#define TRUNCATE 		 	  	   8.0f
#define STEP 			 	  	   8.0f
#define MAX_LEN					   2048.0f
#define MAX_LEN_VOL 	   	       1536.0f
#define HALF_LEN_VOL			   768.0f
#define IMG_HEIGHT			 	   240
#define IMG_HEIGHT_OVER_TWO        120
#define IMG_WIDTH			 	   320
#define IMG_WIDTH_OVER_TWO         160
#define FOCAL_LEN		 		   284.36f
#define LOWER_BOUND				   4
#define RESOL_MACRO				   32
#define VOX_NUM_MACRO			   8
#define STEP_MACRO				   64.0f
#define SHIFT					   700.0f
#define MAX_DEPTH_VAL 			   255.0f
#define UPPERBOUND 				   230
#define LOWERBOUND 				   10
#define ALIGN_SAMPLE 			   50
#define MODEL_SAMPLE 			   2
#define WINDOW_SIZE 			   6
#define ITER_LOOPS 				   16
#define MAX_PC_COUNT               100000

#define INDEX(x, y) (x+IMG_WIDTH*(y))

#endif
