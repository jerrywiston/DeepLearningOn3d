#ifndef CONFIG_H
#define CONFIG_H

#define RESOL_X               512
#define RESOL_Y               256
#define RESOL_Z               512
#define RESOL_VOL 			 	    192
#define VOX_LEN 		 	  	    8.0f
#define TRUNCATE 		 	  	    8.0f
#define STEP 			 	  	      8.0f
#define MAX_LEN_X				      (RESOL_X * VOX_LEN)
#define MAX_LEN_Y             (RESOL_Y * VOX_LEN)
#define MAX_LEN_Z             (RESOL_Z * VOX_LEN)
#define MAX_LEN_VOL 	   	    (RESOL_VOL * VOX_LEN)
#define IMG_HEIGHT			 	    240
#define IMG_WIDTH			 	      320
#define FOCAL_LEN		 		      284.36f
#define LOWER_BOUND				    4
#define SHIFT					        1000.0f
#define MAX_DEPTH_VAL 			  255.0f
#define UPPERBOUND 				    230
#define LOWERBOUND 				    10
#define ALIGN_SAMPLE 			    50
#define MODEL_SAMPLE 			    2
#define WINDOW_SIZE 			    6
#define ITER_LOOPS 				    16
#define MAX_PC_COUNT          400000

#define INDEX(x, y)           (x+IMG_WIDTH*(y))

#endif
