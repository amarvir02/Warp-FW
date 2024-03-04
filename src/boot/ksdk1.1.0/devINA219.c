// ac2362 - modified from HDC1000.c because it has 16B registers too.


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
#include <stdio.h>

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

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	returnValue;

	switch (deviceRegister)
	{
		case 0x00: case 0x05: //writeable registers on the INA219 are only these two
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
			.address       = deviceINA219State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	commandByte[0] = deviceRegister;
	payloadByte[0] = (payload >> 8) & 0xFF; /* MSB first */
	payloadByte[1] = payload & 0xFF;        /* LSB */
	returnValue    = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		2,
		gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

//WarpStatus
//readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
//{
//	uint8_t		cmdBuf[1] = {0xFF};
//	i2c_status_t	status;
//	// should I also add a case statement for the readable regs on the INA219 - they're all readable?
//		USED(numberOfBytes);
//	switch (deviceRegister)
//	{
//		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: 
//		{
//			/* OK */
//			break;
//		}
//
//		default:
//		{
//			return kWarpStatusBadDeviceCommand;
//		}
//	}
//
//	i2c_device_t slave =
//		{
//		.address = deviceINA219State.i2cAddress,
//		.baudRate_kbps = gWarpI2cBaudRateKbps
//	};
//
//	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
//	cmdBuf[0] = deviceRegister;
//	warpEnableI2Cpins();
//
//	status = I2C_DRV_MasterReceiveDataBlocking(
//		0 /* I2C peripheral instance */,
//		&slave,
//		cmdBuf,
//		1,
//		(uint8_t *)deviceINA219State.i2cBuffer,
//		numberOfBytes,
//		gWarpI2cTimeoutMilliseconds);
//		
//
//	if (status != kStatus_I2C_Success)
//	{
//		return kWarpStatusDeviceCommunicationFailed;
//	}
//
//	return kWarpStatusOK;
//}
//

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	
	uint8_t cmdBuf[1] = {0xFF};
	i2c_status_t status1, status2;

	i2c_device_t slave =
		{
			.address       = deviceINA219State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: 
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
		/*
		 *	Step 1: Trigger measurement 1
		 */
		cmdBuf[0] = deviceRegister;

		status1 = I2C_DRV_MasterSendDataBlocking(
			0 /* I2C peripheral instance */,
			&slave,
			cmdBuf,
			1,
			NULL,
			0,
			gWarpI2cTimeoutMilliseconds);

		/*
		 *	Step 2: Wait for max 6.5ms for conversion completion (see Table 7.5 of HDC1000 datasheet)
		 */
		OSA_TimeDelay(5);

		/*
		 *	Step 3: Read from the 5 registers
		 */
		status2 = I2C_DRV_MasterReceiveDataBlocking(
			0 /* I2C peripheral instance */,
			&slave,
			NULL,
			0,
			(uint8_t*)deviceINA219State.i2cBuffer,
			numberOfBytes,
			gWarpI2cTimeoutMilliseconds);

		if ((status1 != kStatus_I2C_Success) || (status2 != kStatus_I2C_Success))
		{
			return kWarpStatusDeviceCommunicationFailed;
		}
	else
	{
		cmdBuf[0] = deviceRegister;

		status1 = I2C_DRV_MasterReceiveDataBlocking(
			0 /* I2C peripheral instance */,
			&slave,
			cmdBuf,
			1,
			(uint8_t*)deviceState.i2cBuffer,
			numberOfBytes,
			gWarpI2cTimeoutMilliseconds);

		if (status1 != kStatus_I2C_Success)
		{
			return kWarpStatusDeviceCommunicationFailed;
		}
	}

	return kWarpStatusOK;
}

// Looks to take read values from over the I2C bus and manipulating them for printing.
void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;

	float 		myshuntvoltage;
	float 		mybusvoltage;
	float 		mypower;
	float 		mycurrent;
	
	WarpStatus	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	//kWarpSensorOutputRegisterINA219SHUNT_VOLTAGE			= 0x01,
	//kWarpSensorOutputRegisterINA219BUS_VOLTAGE			= 0x02,
	//kWarpSensorOutputRegisterINA219POWER					= 0x03,
	//kWarpSensorOutputRegisterINA219CURRENT				= 0x04,


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219SHUNT_VOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

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
			/*
			 *	So we do some conversion on this to give a number as output rather than some 0s and 1s
			 *	Reverse the process described in the manual
			 * 
			 * 	The leading bit is sign
			 * 	The number you have in the register is an actual value that's mul'ed by x100
			 * 	to give you 2dp precision.
			 * 	
			 * 	
			 */
			myshuntvoltage = readSensorRegisterValueCombined;
			warpPrint(" %f,", readSensorRegisterValueCombined );
			//warpPrint(" %f,", (myshuntvoltage/(100) ));
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219BUS_VOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

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
			/*
			 *	See Section 8.6.2 of the INA219 manual for the conversion to temperature.
			 */
			readSensorRegisterValueCombined = ((readSensorRegisterValueCombined & 0xFF)>>3);
			mybusvoltage = readSensorRegisterValueCombined;
			warpPrint(" %f,", mybusvoltage);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

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
			/*
			 *	See Section 8.6.2 of the INA219 manual for the conversion to temperature.
			 */
			mypower = readSensorRegisterValueCombined;
			warpPrint(" %f,", (mypower));
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

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
			/*
			 *	See Section 8.6.2 of the INA219 manual for the conversion to temperature.
			 */

			mycurrent = readSensorRegisterValueCombined;
			warpPrint(" %f,", (mycurrent));
		}
	}
}

uint8_t
appendSensorDataINA219(uint8_t* buf)
{
	uint8_t index = 0;

	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	i2cReadStatus                   = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219SHUNT_VOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{

		readSensorRegisterValueCombined = (readSensorRegisterValueCombined/100);
		

		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219BUS_VOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		readSensorRegisterValueCombined = ((readSensorRegisterValueCombined & 0xFF)>>3);
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{

		readSensorRegisterValueCombined = (readSensorRegisterValueCombined);
		

		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{

		readSensorRegisterValueCombined = (readSensorRegisterValueCombined);
		

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