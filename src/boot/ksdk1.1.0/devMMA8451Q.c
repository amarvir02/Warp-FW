#include <math.h>
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

const int TIME_LIMIT = 10000;
uint8_t X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB;
int16_t X, Y, Z;
uint16_t uint_x, uint_y, uint_z;
uint32_t frac_x, frac_y, frac_z;
float ffrac_x, ffrac_y, ffrac_z;
uint32_t x, y, z;
uint8_t r;

bool x_pos, y_pos, z_pos;

uint8_t	 range;
uint16_t divider;
uint16_t bitmask_int, bitmask_frac;
uint16_t bitmask_sign = (1<<13);
uint16_t r_shift;

// Determined by empirical experimentation
const float mean_stroke = 20.8333333f;
const float stddev_stroke = 1.9113254f;
const float stddev_stroke2 = 3.822650f;
float stroke_min = mean_stroke - stddev_stroke2;
float stroke_max = mean_stroke + stddev_stroke2;

const float mean_shake = 55.4545455f;
const float stddev_shake = 3.3402132f;
const float stddev_shake2 = 6.6804264f;
float shake_min = mean_shake - stddev_shake2;
float shake_max = mean_shake + stddev_shake2;

const float sqrt2 = 1.41421356237;

bool shake_bool;
bool stroke_bool;


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

void printSensorDataMMA8451Q(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	
	// Should probably config registers before the read and print operations
	// Could do this elsewhe	re, but no

	

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
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 * 	But they're not?
	 */
	
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
}

// unfinished


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
	switch(range){
	case MMA8451_RANGE_8_G:
		divider = 1024;
		bitmask_int	=	0b0001110000000000;
		bitmask_frac=	0b0000001111111111;
		r_shift = 10;
		r = 10; 
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

void convert2dec_XYZ(void){
	range = getRangeMMA8451Q();
	// if -ve
	x_pos = ((X>>15)==0) ? true : false;
	X = (x_pos) ? X:(~X)+1;
	uint_x = ((X & bitmask_int)>>r_shift);
	frac_x = (X & bitmask_frac)*r;

	y_pos = ((Y>>15)==0) ? true : false;
	Y = (y_pos) ? Y:(~Y)+1;  
	uint_y = ((Y & bitmask_int)>>r_shift);
	frac_y = (Y & bitmask_frac)*r;
	
	z_pos = ((Z>>15)==0) ? true : false;
	Z = (z_pos) ? Z:(~Z)+1; 
	uint_z = ((Z & bitmask_int)>>r_shift);
	frac_z = (Z & bitmask_frac)*r;
}


void printSensorDataMMA8451Q_XYZ(bool hexModeFlag)
{

	WarpStatus	i2cReadStatus;

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6);
	getSensorDataMMA8451Q_XYZ();

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint("%s"," ----,");
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
uint32_t total_time_ms=0;
uint32_t total_time_counter_ms=0;
uint32_t time_counter_loop=0;
uint32_t t2=0;
uint32_t t1=0;
uint32_t t0=0;
uint32_t t00=0;

int violent_counter=0;
int gentle_counter=0;
int high_counter=0;

const uint8_t sample_size = 200;
//uint16_t x_arr[16];
//uint16_t y_arr[16];
int16_t z_arr[200];
//char z_data_out[80];

#define sqrt2 = 1.414213562F
bool still_x, still_y, still_z;
float float_z;

//float z_farr[200];  -- allocating a float array is 2 much, 800B lol

// switching from double to normal float works fine, since doubles overflow the m_text region of memory
void printfloat(float myfloat){
	// Does "%f" since SEGGER can't
	// int typecasting always seems to round down
	int int_myfloat = (int)(myfloat);
	//char sgn_myfloat[0] = (int_myfloat>0) ? "+":"-";
    int frac_myfloat = abs((int)((myfloat - (float)int_myfloat)*10000));
	//sign conversion  
    warpPrint("%d.%d\n",int_myfloat,frac_myfloat);	
}


void waiting_room(void){
	for (int i=0; i<3; i++){
			const char buf[]="\n MOTION DETECTION\n";
            //devSSD_fill_blank();
            warpPrint("\n");
            SEGGER_RTT_WriteString(0,buf);
            SEGGER_RTT_WriteString(0,"Motion detection sequence intitiated. \nPlace device facing up.\n"); 
            OSA_TimeDelay(3000);
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

float convert2dec_Z(void){
	readSensorRegisterMMA8451Q(MMA8451_REG_OUT_Z_MSB,2);
	Z = (((deviceMMA8451QState.i2cBuffer[0]<<6)|(deviceMMA8451QState.i2cBuffer[1]>>2))^(1<<13))-(1<<13);	
	z_pos = ((Z>>15)==0) ? true : false;
	Z = (z_pos) ? Z:(~Z)+1; 
	uint_z = ((Z & bitmask_int)>>r_shift);
	frac_z = (Z & bitmask_frac)*r;
	//warpPrint("%d.%d",uint_z,frac_z);

	ffrac_z = (float)((Z & bitmask_frac)*(0.0009765625f));
	float_z = (float)uint_z+ffrac_z;
	float_z = (z_pos) ? float_z : float_z*-1;

	return float_z;
}



// OSAtimedelays are chosen to align with 50Hz
void mainloop(void){
	bool still_x, still_y,still_z;
	int liftflag=0;
	while(total_time_counter_ms<TIME_LIMIT){
		t0=OSA_TimeGetMsec();
		for(int l=0; l<10; l++){
		// prints out live data at lower than actual sampling rate
			if (l%10==0){
				printSensorDataMMA8451Q_XYZ(false);
				OSA_TimeDelay(20);
			}
			else{
			
			liftflag = zliftcheck(still_z);
			}
			if (liftflag!=0){
				bool z_pos2;
				getRangeMMA8451Q();
				for (int k=0; k<sample_size; k++){
					t1=OSA_TimeGetMsec();
					// grabs sample_size number of z readings after threshold is exceeded
					float_z = convert2dec_Z();
					z_arr[k] = (int16_t)float_z;
					//stores z as positive, unsigned integer only -- loses accuracy
					
					printfloat(float_z);
					OSA_TimeDelay(40); // calculating the math is currently at a +15ms overhead
					t2=OSA_TimeGetMsec();
					time_counter_loop+=(t2-t1);
				}
				char alert[]="\nPING, YOU'VE TRIGGERED MOTION\n Dumped some sec of z values";
				SEGGER_RTT_WriteString(0, alert);
				warpPrint("\nsampling time elapsed is %d ms\n --> Close to 10 sec",time_counter_loop);
				for (int o; o<5; o++){
					//devSSD_fill_red();
					OSA_TimeDelay(100);
					//devSSD_fill_blank();
				}
				t00=OSA_TimeGetMsec();
				total_time_counter_ms+= t00-t0;
				break;	
			}

			else{
				continue;
			}
		}
		t00=OSA_TimeGetMsec();
		total_time_counter_ms+= t00-t0;
	}
	int harlem_shake = count_loop();
	warpPrint("\n Total Time elapsed in detection loop = %d ms\n", total_time_counter_ms);
	//warpPrint("No of violent shakes = %d \n", violent_counter);;
	warpPrint("Est No of violent shakes = %d \n", harlem_shake);
	return;
}

// counts the array of z measurement points to 
int count_loop(void){
	int shake_count;
	for (int k=1; k<sample_size; k++){
		// decently accurate for large/hard movements
		// searches a quantized int array of motion for 3g crossssings
		// which will be near the peak
		if ((z_arr[k]>=3)&(z_arr[k-1]<3)){
			violent_counter++;
		}
		// not reliable
		//if ((z_arr[k]>4)&(z_arr[k-1]<1)){
		//	gentle_counter++;
		//}
		// not needed
		//if ((z_arr[k]>7)&(z_arr[k-1]<5)){
		//	high_counter++;
		//}
		else{
			continue;
		}
		// 
	}
	shake_count+= violent_counter;
	return shake_count;
}
// if activity mean count is within 2 sigma of that distribution
// it's considered to belong to that distribution

// mapping discrete step counts to a gaussian (continous) is not a good idea



// Testing 10 FMAs
double prod;
double sum;
//uint32_t ta;
//uint32_t tb;
//uint32_t dt;
void test_loop(double a1, double a2, double a3){
	uint32_t ta =OSA_TimeGetMsec();
	for (int i=0; i<1000; i++){
		prod = (a1*a2)+a3;
	}
	uint32_t tb=OSA_TimeGetMsec();
	uint32_t dt=tb-ta;
	warpPrint("time elapsed doing FMA: %d ms", dt);
} // better to do things as float
