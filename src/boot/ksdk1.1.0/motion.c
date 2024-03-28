#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devMMA8451Q.h"
#include "motion.h"
#include "devSSD1331.h"

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t			gWarpI2cBaudRateKbps;

//#define TIME_LIMIT 10000 // 10 seconds
#define THRESH_X // 
#define THRESH_Y //
#define THRESH_Z // Remember in front pos - z=1G ootb because the earth gravity 
int timer_count=0;

// Motion Detection
void motionConfigMMA8451Q(void){
	// set to standby
    standbySensorMAA8451Q();
	    // Enable ,Z and Enable LAtch -- See AN4070
        // X and Y work now yay
	    writeSensorRegisterMMA8451Q(MMA8451_REG_FF_MT_CFG, 0x7F);
	    // 32 Counts 0.063g LSB ==> 3g threshold=48 counts
	    writeSensorRegisterMMA8451Q(MMA8451_REG_FF_MT_THS, 0x30);
	    // 10 counts --> 200ms @ 50Hz, 5 counts --> 100ms
	    writeSensorRegisterMMA8451Q(MMA8451_REG_FF_MT_COUNT, 0x05);
	    // enable FF_MT_interrupt
	    writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG4, 0x04);
	    // FF_MT routes to INT2 for FRDMKL03Z
	    writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG5, 0x00);
	    // GO into active mode
    activateSensorMAA8451Q();
	return;
}

uint16_t x_counter=0;
uint16_t y_counter=0;
uint16_t z_counter=0;

void motion_detection(void){
    const char buf[]="\n MOTION DETECTION";
    SEGGER_RTT_WriteString(0,buf);
    OSA_TimeDelay(1000);
    bool still_x;
    bool still_y;
    bool still_z;
    for (int i=0; i<3; i++){
            devSSD_fill_blank();
            warpPrint("\n");
            SEGGER_RTT_WriteString(0,buf);
            SEGGER_RTT_WriteString(0,"Motion detection sequence intitiated. \nPlace device facing up."); 
            OSA_TimeDelay(1000);
            devSSD_fill_white();
            return;
    }
    warpPrint("\n");
    SEGGER_RTT_WriteString(0,"Motion detection sequence started. \nLift device upwards in next 10 seconds to trigger");
    OSA_TimeDelay(1000);
    while (timer_count<TIME_LIMIT){
        timer_count+=20;
        printSensorDataMMA8451Q_XYZ(false);
        OSA_TimeDelay(10);
        getSensorDataMMA8451Q_XYZ();
 
        // 3g in 8g mode is == 00x011000000000000 
        // Also want -3g detection
        // in 8b for MSB --> 00x011xx
        int16_t mythreshold = 2;
        //= ((1<<12)|(1<<11));  

        still_x = (((X>>10) & 0x07) < mythreshold) ? true : false;
        still_y = (((Y>>10) & 0x07) < mythreshold) ? true : false;
        still_z = (((Z>>10) & 0x07) < (mythreshold+1)) ? true : false;

        if (still_x && still_y && still_z){
            OSA_TimeDelay(20);
            continue;
        }
        else if (!still_x){
            devSSD_fill_red();
            warpPrint("Succesfully Lifted in x");
            //interrupt_service_handler(still);
            break;
        }
        else if (!still_y){
            devSSD_fill_green();
            warpPrint("Succesfully Lifted in y");
            break;
        }
        else if (!still_z){
            warpPrint("Succesfully Lifted in z");
            break;
        }
        else
        {
            devSSD_fill_white();
            warpPrint("Lifted in multi axis");
            break;
        }        
    }
    // function return when broken
    SEGGER_RTT_WriteString(0,"done");
    return;
}

// Set a threshold of 5g ??



char mychars[16];

// No Explanation on how to setup Interrupts on the FRDM-KL03Z 
// Just what pins map to what, not really useful, SDK only has one function for GPIO Interrrupts
void interrupt_service_handler(bool myBool){
	//clear interrupt flag
    myBool = false;
    // Read interrupt table register
	uint8_t IntSource=readSensorRegisterMMA8451Q(0x0C, 1);
    // Technically supposed to be a switch-case to enumerate on all possible interrupts
    // I have no other event mechanisms enabled right now, so until then, this is fine.
    if ((IntSource & 0x04)==0x04){
        uint8_t IntSource_MT_FF = readSensorRegisterMMA8451Q(MMA8451_REG_FF_MT_SRC,1);
        // handling routine
        uint8_t latched_src_data = deviceMMA8451QState.i2cBuffer[0];
        warpPrint("%s","\nPRINTING MM_SRC_REGISTER");
        warpPrint("\n %02x", latched_src_data);
        //for (int j=0;j<sizeof(latched_src_data);j++){
        //    mychars[j]=
        //}

        //switch(latched_motion_data){
        //    //+x
        //    case(0x00):
        //    break;
        //    case(0x01):
        //    x_counter++;
        //    break;
        //}
    }
}

/*
*	// declaring a check function from another file causes compiler error
*		//uint16_t myBuff[];
*
*
*		//readSensorRegisterMMA8451Q(MMA8451_REG_OUT_Z_MSB,2);
  *      //uint8_t zcheck = (deviceMMA8451QState.i2cBuffer[0]);
  *      //uint16_t zz = (deviceMMA8451QState.i2cBuffer[0]<<6)|(deviceMMA8451QState.i2cBuffer[0]>>2);
  *      //if ((zcheck << 3) > 0b10100000 ){
*		//	warpPrint("Succesfully Lifted");
*		//	getSensorDataMMA8451Q_XYZ();
*		//warpPrint("z, %01d.%04d", (sgn_z*int_z),frac_z);
*		//	warpPrint("\n accel in z is 0x%04x", zz);
*		//	for(int i=0;i<5;i++){
  *      //    	devSSD_fill_red();
*		//		OSA_TimeDelay(100);
*		//		devSSD_fill_blank();
*		//		OSA_TimeDelay(100);
*		//		devSSD_fill_red();
*		//	}
  *      //    warpPrint("\n Succesfully Lifted");
  *      //    
*		//	
*		//	break;
*		//}
*		//else{
*		//	OSA_TimeDelay(200);
*		//	continue;
*		//};
*/