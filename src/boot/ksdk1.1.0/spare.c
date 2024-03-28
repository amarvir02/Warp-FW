#include <stdlib.h>
#include <stdint.h>


//#define N 100;
//const int N = 100;

enum{N =10};

// At 12.5Hz, 100B should hopefully be enough to store 10s worth of data recordings  
uint16_t accel_x[N]={0}, vel_x[N]={0}, pos_x[N]={0};
uint16_t accel_y[N]={0}, vel_y[N]={0}, pos_y[N]={0};
uint16_t accel_z[N]={0}, vel_z[N]={0}, pos_z[N]={0};
uint16_t iter, t1, t2, time[N];


void recordDataMMA8451Q(){
	for(iter=0; iter<N; iter++){
		t2 = OSA_TimeGetMsec();
		readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6);
		//Needs repeating for all X, Y, Z
		X =((deviceMMA8451QState.i2cBuffer[0]<<6)|(deviceMMA8451QState.i2cBuffer[1]>>2));
		OSA_TimeDelay(80); //12.5Hz sample rate
		if (iter=0){
			accel_x[iter] = X;
			iter++;
			t1 = OSA_TimeGetMsec();
			continue;
		}
		if (iter=1){
			accel_x[iter] = X;
			iter++;
			t1 = OSA_TimeGetMsec();
			continue;
		}
		else{
			accel_x[iter] = X;
			time[iter]=t2-t1;
		}
		t1 = OSA_TimeGetMsec();

	}
}

void intergrate_(void){
	for(iter=0; iter<N; iter++)
	{
		vel_x[iter] = accel_x[iter] + 0.5*(t2-t1)*(accel_x[iter]- accel_x[iter-1]);
		pos_x[iter] = vel_x[iter] + 0.5*(t2-t1)*(vel_x[iter]- vel_x[iter-1]);
	
		vel_y[iter] = accel_y[iter] + 0.5*(t2-t1)*(accel_y[iter]- accel_y[iter-1]);
		pos_y[iter] = vel_y[iter] + 0.5*(t2-t1)*(vel_y[iter]- vel_y[iter-1]);
	
		vel_z[iter] = accel_z[iter] + 0.5*(t2-t1)*(accel_z[iter]- accel_z[iter-1]);
		pos_z[iter] = vel_z[iter] + 0.5*(t2-t1)*(vel_z[iter]- vel_z[iter-1]);
	
	}
}


void get_priors(void){

}

void Gaussian(void){
	for (iter=0;iter<N; iter++){
		if (iter < 2){
			continue;
		}
		else{
			sum[0]+=accel_x[iter];
			sum[1]+=accel_y[iter];
			sum[2]+=accel_z[iter];
		}
	}
	mean[0] = sum[0]/N;
	// clear sums
	//sum[0] = 0;
	// loop for variance
	for(iter=0; iter<100; iter++)
	{	
		if(iter<2){
		continue;
		}
		else{
			sumv[0]+=pow((accel_x[iter]-mean[0]),2);
			sumv[1]+=pow((accel_y[iter]-mean[1]),2);
			sumv[2]+=pow((accel_z[iter]-mean[2]),2);
		}
	}
	variance[0] = sumv[0]/N;
	variance[1] = sumv[1]/N;
	variance[2] = sumv[2]/N;
}


//void max_axis_selection(){
//	
//	for (iter=0; iter<100; iter++)
//		getSensorDataMMA8451Q_XYZ(false);
//		int_x = ((X & (0b001100000000000))>>13);
//		int_y = ((Y & (0b001100000000000))>>13);
//		int_z = ((Z & (0b001100000000000))>>13);
//
//		frac_x = (uint32_t) ((X & (0b0000011111111111))*10000)/divider;
//		frac_y = (uint32_t) ((Y & (0b0000011111111111))*10000)/divider;
//		frac_z = (uint32_t) ((Z & (0b0000011111111111))*10000)/divider;
//}


// See AN4076
int standbySensorMAA8451Q(void)
{
	readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 2);
	int n = deviceMMA8451QState.i2cBuffer[0];
	uint8_t activemask = 0b00000001;
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,( n & ~activemask ));
	return 0;
}

int activateSensorMAA8451Q(void)
{
	readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 2);
	int n = deviceMMA8451QState.i2cBuffer[0];
	uint8_t activemask = 0b00000001;
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,( n | activemask ));
	return 0;
}
