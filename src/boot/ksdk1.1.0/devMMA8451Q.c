/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>
#include <stdint.h>

/*
 *	config.h needs to come first
 */
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

uint16_t X, Y, Z;
uint16_t int_x, int_y, int_z;
uint32_t frac_x, frac_y, frac_z;
uint16_t sgn_x, sgn_y, sgn_z;
uint32_t x, y, z;

uint8_t	 range;
uint16_t divider;
uint16_t bitmask_int, bitmask_frac;
uint16_t bitmask_sign = (1<<13);
uint16_t l_shift;

// 3-large arrays for x, y and z
uint64_t sum[2], mean[2];
uint64_t sumv[2], variance[2];


void
initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceMMA8451QState.i2cAddress					= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

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

WarpStatus
configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QF_SETUP /* register address F_SETUP */,
												  payloadF_SETUP /* payload: Disable FIFO */
	);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1 /* register address CTRL_REG1 */,
												  payloadCTRL_REG1 /* payload */
	);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
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
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataMMA8451Q(bool hexModeFlag)
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

uint8_t
appendSensorDataMMA8451Q(uint8_t* buf)
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

bool MMA8451Qconfig(const uint8_t i2cAddress){
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG2, 0x40); // reset
	while (deviceMMA8451QState.i2cBuffer[0] & 0x40);
		// enable 4G range
		readSensorRegisterMMA8451Q(MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_4_G);
		// High res
		writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG2, 0x02);
		// DRDY on INT1
		writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG4, 0x01);
		writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG5, 0x01);

		// Turn on orientation config
		writeSensorRegisterMMA8451Q(MMA8451_REG_PL_CFG, 0x40);
		// Activate at 12.5Hz with high resolution mode on
		writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 0x01|0x28);
	
	return true;
}

void MMA8451QsetDataRate(mma8451_dataRate_t dataRate) {
	uint8_t ctl1 = readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,1);
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 0x00); // deactivate
	ctl1 &= ~(MMA8451_DATARATE_MASK << 3);       // mask off bits
	ctl1 |= (dataRate << 3);
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, ctl1 | 0x01); // activate
}

// GEts data rate aka ODR
mma8451_dataRate_t MMA8451QgetDataRate(void){
	readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1,1);
	return (mma8451_dataRate_t)(((deviceMMA8451QState.i2cBuffer[0]) >> 3) & MMA8451_DATARATE_MASK);
}

// Sets g mode - 2g, 4g, 8g
void MMA8451QsetRange(mma8451_range_t range){
	// fetch the control register
	readSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 1);
	uint8_t ctrl_reg1 = deviceMMA8451QState.i2cBuffer[0];

	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, 0x00); // PUT INTO STANDBY
	writeSensorRegisterMMA8451Q(MMA8451_REG_XYZ_DATA_CFG, range & 0x3);
	writeSensorRegisterMMA8451Q(MMA8451_REG_CTRL_REG1, ctrl_reg1 | 0x01); // Acticatte
}


mma8451_range_t getRangeMMA8451Q(void) 
{
  /* Read the data format register to preserve bits */
	readSensorRegisterMMA8451Q((MMA8451_REG_XYZ_DATA_CFG) & 0x03, 1);
	range = deviceMMA8451QState.i2cBuffer[0];
	switch(range){
		case MMA8451_RANGE_8_G:
			divider = 1024;
			bitmask_int	=	0b0001110000000000;
			bitmask_frac=	0b0000011111111111;
			l_shift = 11;
			break;
		case MMA8451_RANGE_4_G:
			divider = 2048;
			bitmask_int	=	0b0001100000000000;
			bitmask_frac=	0b0000111111111111;
			l_shift = 12;
			break;
		case MMA8451_RANGE_2_G:
			divider = 4096;
			bitmask_int	=	0b0001000000000000;
			bitmask_frac=	0b0001111111111111;
			l_shift = 13;
			break;
	}
	return (mma8451_range_t)(range);
}

int getSensorDataMMA8451Q_XYZ(bool bits2integer)
{
	WarpStatus	i2cReadStatus;
	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6);

	X = (deviceMMA8451QState.i2cBuffer[0]<<6)|(deviceMMA8451QState.i2cBuffer[1]>>2);
	Y = (deviceMMA8451QState.i2cBuffer[2]<<6)|(deviceMMA8451QState.i2cBuffer[3]>>2);
	Z = (deviceMMA8451QState.i2cBuffer[4]<<6)|(deviceMMA8451QState.i2cBuffer[5]>>2);

	if (bits2integer)
	{
		mma8451_range_t range = getRangeMMA8451Q();
		/*
		* Should be multiplying by at least 10000 for maximal accuracy / value recovery,
		* but doing *100 works well enough, losing minimal accuracy of around 0.2%.
		* Could've probably done with 8b just fine
		* Multiply by 10000, then divide by 100 to have a total mul of 100
		* Ensures I have no out of range issues when dealing with powers of large numbers as ints 
		*/ 
		int_x = (((X & (bitmask_int))>>l_shift)*10000);
		int_y = (((Y & (bitmask_int))>>l_shift)*10000);
		int_z = (((Z & (bitmask_int))>>l_shift)*10000);

		frac_x = (uint32_t) ((X & (bitmask_frac))*10000)/divider;
		frac_y = (uint32_t) ((Y & (bitmask_frac))*10000)/divider;
		frac_z = (uint32_t) ((Z & (bitmask_frac))*10000)/divider;

		x = (int_x+frac_x)/100;
		y = (int_y+frac_y)/100;
		z = (int_z+frac_z)/100;

		if ((X & bitmask_sign) == 0x0000){
			x = x;			
		}
		if ((X & bitmask_sign) == 0x2000){/* (1<<13)== 0x2000*/
			x = (x*-1);
		}
		if ((Y & bitmask_sign) == 0x0000){
			y = y;			
		}
		if ((Y & bitmask_sign) == 0x2000){/* (1<<13)== 0x2000*/
			y = (y*-1);
		}
		if ((Z & bitmask_sign) == 0x0000){
			z = z;			
		}
		if ((Z & bitmask_sign) == 0x2000){/* (1<<13)== 0x2000*/
			z = (z*-1);
		}

		return 0;
	}
	else
	{
		return 0;
	}
}

void
printSensorDataMMA8451Q_XYZ(bool hexModeFlag)
{

	WarpStatus	i2cReadStatus;
	//warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	//mma8451_range_t range = getRangeMMA8451Q();
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6);
	getSensorDataMMA451Q_XYZ(true);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%04x,", X);
		}
		else
		{
			warpPrint("x %d y %d z %d", x,y,z);
		}
	}
}
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
