#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>


#define TIME_LIMIT 10000 // 10 seconds
#define THRESH_X //2G for limit on motion detection?? 
#define THRESH_Y // maybe +/-1G away from starting. so 1.998-1 == ~ 1G?
#define THRESH_Z // z still is ~0.2G, could apply offset using OFF_X


void motionConfigMMA8451Q(void);
void motion_detection(void);
