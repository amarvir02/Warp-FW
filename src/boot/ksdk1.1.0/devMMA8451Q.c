
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

// config.h needs to come first
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

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t			gWarpI2cBaudRateKbps;
extern volatile uint32_t			gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t			gWarpSupplySettlingDelayMilliseconds;

uint8_t X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB;
int16_t X, Y, Z;
uint16_t uint_x, uint_y, uint_z;
uint32_t frac_x, frac_y, frac_z;
double ffrac_x, ffrac_y, ffrac_z;
uint32_t x, y, z;
uint8_t r;

bool x_pos, y_pos, z_pos;

uint8_t	 range;
uint16_t divider;
uint16_t bitmask_int, bitmask_frac;
uint16_t bitmask_sign = (1<<13);
uint16_t r_shift;


void initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts){
	deviceMMA8451QState.i2cAddress					= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG2, 0x40); // reset
	// enable 8G range as per adafruit library
	MMA8451QsetRange(MMA8451_RANGE_8_G);
	// Set normal mode
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG2, 0x00);
	// Activate at ~~50Hz~~ 100Hz 
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 0x20);
	activateSensorMAA8451Q();

	return;
}

WarpStatus writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1){
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;
	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(MMA8451_REG_F_SETUP, 	payloadF_SETUP /* payload: Disable FIFO */);
	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,payloadCTRL_REG1 /* payload */);
	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;
	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		{break;}

		default:
		{return kWarpStatusBadDeviceCommand;}
	}

	i2c_device_t slave =
		{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceMMA8451QState.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{return kWarpStatusDeviceCommunicationFailed;}
	return kWarpStatusOK;
}


void printSensorDataMMA8451Q_arb(uint8_t deviceRegister, int numberOfBytes, bool hexModeFlag){
	// printing arbitrary registers
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;
	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	uint8_t storage_array[32];
	uint8_t register_array[32];
	
	// Should probably config registers before the read and print operations
	// Could do this elsewhe	re, but no
	/*
	 *	Prints out any selection of arbitrary registers from a given starting point
	 *
	 */
	
	i2cReadStatus = readSensorRegisterMMA8451Q(deviceRegister, numberOfBytes);
	for (int i; i<numberOfBytes; i++){
		register_array[i] = deviceRegister;
		storage_array[i]= deviceMMA8451QState.i2cBuffer[i];
	}
	//readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
	if (i2cReadStatus != kWarpStatusOK){
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag){
			for (int i; i<numberOfBytes; i++){
			warpPrint("\n %02x || %02x",register_array[i],storage_array[i]);
			}
		}
		else{
			for (int i; i<numberOfBytes; i++){
			warpPrint("\n %d",storage_array[i]);
			}
		}
	}
}

uint8_t appendSensorDataMMA8451Q(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	return index;
}


// From Adafruit Library

void MMA8451QsetDataRate(mma8451_dataRate_t dataRate) {
	uint8_t ctl1 = readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,1);
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 0x00); // deactivate
	ctl1 &= ~(MMA8451_DATARATE_MASK << 3);       // clears the datarate bits and sets to 1
	ctl1 |= (dataRate << 3);
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, ctl1); 
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, (ctl1 | 0x01));// activate
}

// GEts data rate aka ODR
mma8451_dataRate_t MMA8451QgetDataRate(void){
	readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,1);
	return (mma8451_dataRate_t)(((deviceMMA8451QState.i2cBuffer[0]) >> 3) & MMA8451_DATARATE_MASK);
}activateSensorMAA8451Q();

// Sets g mode - 2g, 4g, 8g
void MMA8451QsetRange(mma8451_range_t range){
	// fetch the control register
	uint8_t ctl1 = readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,1);
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 0x00);;// PUT INTO STANDBY
	writeSensorRegisterMMA8451Q(MMA8451_REG_XYZ_DATA_CFG, range & 0x3);
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, ctl1); // write the new setting before activation
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, (ctl1 | 0x01));// activate
}

mma8451_range_t getRangeMMA8451Q(void)
{
  /* Read the data format register to preserve bits */
	readSensorRegisterMMA8451Q((MMA8451_REG_XYZ_DATA_CFG), 1);
	range = deviceMMA8451QState.i2cBuffer[0];
	return (mma8451_range_t)(range);
}

void getSensorDataMMA8451Q_XYZ(void)
// formats sensor data into decimal fixed point
{	
	WarpStatus	i2cReadStatus;
	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterMMA8451Q(MMA8451_REG_OUT_X_MSB, 6);
	// the 6 LSB Bits are in the upper portion of the I2C register, with bottom 2 bits being 0 by default
	//sign extended, frac bits are preserved
	X = (((deviceMMA8451QState.i2cBuffer[0]<<6)|(deviceMMA8451QState.i2cBuffer[1]>>2))^(1<<13))-(1<<13);
	Y = (((deviceMMA8451QState.i2cBuffer[2]<<6)|(deviceMMA8451QState.i2cBuffer[3]>>2))^(1<<13))-(1<<13);
	Z = (((deviceMMA8451QState.i2cBuffer[4]<<6)|(deviceMMA8451QState.i2cBuffer[5]>>2))^(1<<13))-(1<<13);
	return;
}

int sign(myint){
	int c = (myint==0)? 1:-1;
	return c;
}
// what if, don't split XYZ, but present as a number to use in a whole function.
void convert2dec_XYZ(void){
	range = getRangeMMA8451Q();
	switch(range){
		case MMA8451_RANGE_8_G:
			divider = 1024;
			bitmask_int	=	0b0001110000000000;
			bitmask_frac=	0b0000001111111111;
			r_shift = 10;
			r = 10; //using floats will be slow
			break;
		case MMA8451_RANGE_4_G:
			divider = 2048;
			bitmask_int	=	0b0001100000000000;
			bitmask_frac=	0b0000011111111111;
			r_shift = 11;
			r = 5;
			break;
		case MMA8451_RANGE_2_G:
			divider = 4096;
			bitmask_int	=	0b0001000000000000;
			bitmask_frac=	0b0000111111111111;
			r_shift = 12;
			r = 2;
			break;
	}
	// if -ve
	//!! TODO - Getting correct Sign output - always stuck on +ve sign - fixed
	//!! Doesn't seem to handle the int portion correctly, either 0 to 4. at best - maybe realistic?
	//!! where's the third bit up to 7?--> unable to detect sharp-ish twists as high multi-axis acceleration
	x_pos = ((X>>15)==0) ? true : false;
	X = ((X>>13)!=0) ? (~X)+1 : X; 
	
	y_pos = ((Y>>15)==0) ? true : false;
	Y = ((Y>>15)!=0) ? (~Y)+1 : Y; 
	
	z_pos = ((Z>>15)==0) ? true : false;
	Z = ((Z>>15)!=0) ? (~Z)+1 : Z; 

	uint_x = ((X & bitmask_int)>>r_shift);
	uint_y = ((Y & bitmask_int)>>r_shift);
	uint_z = ((Z & bitmask_int)>>r_shift);

	frac_x = (X & bitmask_frac)*r;
	frac_y = (Y & bitmask_frac)*r;
	frac_z = (Z & bitmask_frac)*r;
}



void printSensorDataMMA8451Q_XYZ(bool hexModeFlag)
{

	WarpStatus	i2cReadStatus;
	//warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	//mma8451_range_t range = getRangeMMA8451Q();
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6);
	getSensorDataMMA8451Q_XYZ();

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint("0x%04x, 0x%04x, 0x%04x," , X, Y, Z);
			return;
		}
		else
		{	
			// Either this works with signs, or not 
			convert2dec_XYZ();
			warpPrint("\n x ");
			(x_pos) ? SEGGER_RTT_WriteString(0, "+") : SEGGER_RTT_WriteString(0, "-");
			warpPrint("%01d.%04d" , uint_x,frac_x);
			
			warpPrint(" y ");
			(y_pos) ? SEGGER_RTT_WriteString(0, "+") : SEGGER_RTT_WriteString(0, "-");
			warpPrint("%01d.%04d" , uint_y,frac_y);

			warpPrint(" z ");
			(z_pos) ? SEGGER_RTT_WriteString(0, "+") : SEGGER_RTT_WriteString(0, "-");
			warpPrint("%01d.%04d" , uint_z,frac_z);
			return;
		}
	}
}


int standbySensorMAA8451Q(void)
{
	// See AN4076
	uint8_t activemask = 0b00000001;
	readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 1);
	int n = deviceMMA8451QState.i2cBuffer[0];
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,( n & ~activemask ));
	return 0;
}

int activateSensorMAA8451Q(void)
{
	// See AN4076
	uint8_t activemask = 0b00000001;
	readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 1);
	int n = deviceMMA8451QState.i2cBuffer[0];
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,( n | activemask ));
	return 0;
}


// -----------------------------------------------------------------------------------------------
const uint8_t sample_size = 25;
//uint16_t x_arr[16];
//uint16_t y_arr[16];
uint16_t z_arr[26];
//char z_data_out[80];
double sgn_z;
#define sqrt2 = 1.414213562
bool still_x, still_y, still_z;
double float_z;

void printfloat(double myfloat){
	// Does "%f" since SEGGER can't
	// int typecasting always seems to round down
	//char sgn_myfloat[0] = (myfloat>0) ? "+":"-";
	int int_myfloat = (int)(myfloat);
    int frac_myfloat = abs((int)((myfloat - int_myfloat)*1000));
	//sign conversion  
    printf("%d.%d",int_myfloat,(int)frac_myfloat);	
}


void waiting_room(void){
	for (int i=0; i<3; i++){
			const char buf[]="\n MOTION DETECTION\n";
            //devSSD_fill_blank();
            warpPrint("\n");
            SEGGER_RTT_WriteString(0,buf);
            SEGGER_RTT_WriteString(0,"Motion detection sequence intitiated. \nPlace device facing up.\n"); 
            OSA_TimeDelay(1500);
            //devSSD_fill_white();
            return;
    }
}

					// 0b0000110000000000 - 0x0C00
//		((zcheck<<3) > 0b0110000000000000)	0X6000
	void liftcheck(bool stillx, bool stilly, bool stillz){
		readSensorRegisterMMA8451Q(MMA8451_REG_OUT_X_MSB,6);

		uint8_t Xcheck = deviceMMA8451QState.i2cBuffer[0];
		uint8_t Ycheck = deviceMMA8451QState.i2cBuffer[2];
		uint8_t Zcheck = deviceMMA8451QState.i2cBuffer[4];

		Xcheck = ((Xcheck&(1<<7))>0) ? (~Xcheck)+1 : Xcheck;
		Ycheck = ((Ycheck&(1<<7))>0) ? (~Ycheck)+1 : Ycheck;
		Zcheck = ((Zcheck&(1<<7))>0) ? (~Zcheck)+1 : Zcheck;
		//int16_t Xcheck = (((deviceMMA8451QState.i2cBuffer[0]<<6)|(deviceMMA8451QState.i2cBuffer[1]>>2))^(1<<13))-(1<<13);
		//int16_t Ycheck = (((deviceMMA8451QState.i2cBuffer[2]<<6)|(deviceMMA8451QState.i2cBuffer[3]>>2))^(1<<13))-(1<<13);
		//int16_t Zcheck = (((deviceMMA8451QState.i2cBuffer[4]<<6)|(deviceMMA8451QState.i2cBuffer[5]>>2))^(1<<13))-(1<<13);
		//Xcheck  = ((Xcheck >>13)!=0) ? (~Xcheck)+1 : Xcheck ; 
		//Ycheck  = ((Ycheck >>13)!=0) ? (~Ycheck)+1 : Ycheck ; 
		// 

		stillx = (((Xcheck>>4) & 0x07) >0x02) ? true : false;
		(stillx) ? SEGGER_RTT_WriteString(0, "\n x still") : SEGGER_RTT_WriteString(0, "\n x moving");

		stilly = (((Ycheck>>4) & 0x07) >0x02) ? true : false;
		(stilly) ? SEGGER_RTT_WriteString(0, "  y still") : SEGGER_RTT_WriteString(0, "  y moving");

		stillz = (((Zcheck>>4) & 0x07) >0x02) ? true : false;
		(stillz) ? SEGGER_RTT_WriteString(0, "  z still") : SEGGER_RTT_WriteString(0, "  z moving");
        return;
}

int zliftcheck(bool stillz){
	readSensorRegisterMMA8451Q(MMA8451_REG_OUT_Z_MSB,2);
	uint16_t Zcheck = (deviceMMA8451QState.i2cBuffer[0]<<8)|(deviceMMA8451QState.i2cBuffer[1]);
	// my threshold is 2g, 1g offset from gravity, then another 2g ontop
	Zcheck  = ((Zcheck >>13)!=0) ? (~Zcheck)+1 : Zcheck ;
	if ((Zcheck)>0b0011000000000000){
		SEGGER_RTT_WriteString(0, "\n  z moving");
		return 1;
	}
	else{
		//SEGGER_RTT_WriteString(0, "\n  z still");
		return 0;
	}
}

// OSAtimedelays are chosen to align with 50Hz
void mainloop(void){
	bool still_x, still_y,still_z;
	int liftflag=0;
	for (int l=0; l<5000; l++){
		if (l%50==0){
			printSensorDataMMA8451Q_XYZ(false);
			OSA_TimeDelay(20);
		}
		else{
			//liftcheck(still_x, still_y, still_z);
			liftflag = zliftcheck(still_z);
			if (liftflag!=0){
				for (int k=0; k<sample_size; k++){
					// grabs sample_size number of z readings after threshold is exceeded
					readSensorRegisterMMA8451Q(MMA8451_REG_OUT_Z_MSB,2);
					z_arr[k] = (deviceMMA8451QState.i2cBuffer[0]<<6)|(deviceMMA8451QState.i2cBuffer[1]>>2);
				}
					// Will corrupt X & Y data
					r=10;
				for (int k=0; k<sample_size; k++){
					r_shift=10;
					sgn_z = ((z_arr[k]>>15)==0) ? 1 : -1;
					//z_pos = ((z_arr[k]>>15)==0) ? true : false;
					Z = ((z_arr[k]>>15)!=0) ? (~Z)+1 : Z; 
					uint_z = ((z_arr[k] & bitmask_int)>>r_shift);
					frac_z = (z_arr[k] & bitmask_frac)*10;
					ffrac_z = (double)((z_arr[k] & bitmask_frac)/1024);
					float_z= uint_z+ffrac_z;
					uint16_t iffrac_z = (uint16_t)((ffrac_z)*1000);
					//double float_z = (double)uint_z + ffrac_z;
					(z_pos) ? SEGGER_RTT_WriteString(0, "\n+") : SEGGER_RTT_WriteString(0, "\n-");
					warpPrint("%d.%d",uint_z,iffrac_z);
					//printfloat(float_z);
					OSA_TimeDelay(40);
					uint_z=0;
					frac_z=0;
					float_z=0;
					ffrac_z=0;
				}
				
				for (int o; o<5; o++){
					//devSSD_fill_red();
					OSA_TimeDelay(1000);
					//devSSD_fill_blank();
				}
				//
				char alert[]="PING, YOU'VE TRIGGERED MOTION\n Dumped 1s of z values";
				SEGGER_RTT_WriteString(0, alert);
				break;
			}
		}
	}
	return;
}