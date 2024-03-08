// ac2362 - modified from HDC1000.c because it has 16B registers too.
// ALSO USED ADAFRUIT INA219 LIBRARY

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

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


int16_t		myshuntvoltage;
//float 		myshuntvoltage;
int16_t		mybusvoltage;
int32_t		mypower;
int32_t 	mycurrent;

int16_t value;

//kWarpSensorOutputRegisterINA219SHUNT_VOLTAGE			= 0x01,
//kWarpSensorOutputRegisterINA219BUS_VOLTAGE			= 0x02,
//kWarpSensorOutputRegisterINA219POWER					= 0x03,
//kWarpSensorOutputRegisterINA219CURRENT				= 0x04,

// copied from MMA8451Q, HDC1000 and adafruit library driver
#define INA219_REG_CONFIG (0X00)
#define INA219_REG_SHUNTVOLTAGE (0x01) /** shunt voltage register **/
#define INA219_REG_BUSVOLTAGE (0x02) /** bus voltage register **/
#define INA219_REG_POWER (0x03) /** power register **/
#define INA219_REG_CURRENT (0x04) /** current register **/
#define INA219_REG_CALIBRATION (0x05) /** calibration register **/
#define INA219_CONFIG_RESET (0x8000) // Reset Bit
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000),/** mask for bus voltage range **/

// I get compile errors if I don't include these here
// CAm

/** bus voltage range values **/
enum
{
    INA219_CONFIG_BVOLTAGERANGE_16V = (0x0000), // 0-16V Range
    INA219_CONFIG_BVOLTAGERANGE_32V = (0x2000), // 0-32V Range
};

/** mask for gain bits **/
#define INA219_CONFIG_GAIN_MASK (0x1800) // Gain Mask

/** values for gain bits **/
enum
{
    INA219_CONFIG_GAIN_1_40MV =     (0x0000),  // Gain 1, 40mV Range
    INA219_CONFIG_GAIN_2_80MV =     (0x0800),  // Gain 2, 80mV Range
    INA219_CONFIG_GAIN_4_160MV =    (0x1000), // Gain 4, 160mV Range
    INA219_CONFIG_GAIN_8_320MV =    (0x1800), // Gain 8, 320mV Range
};

/** mask for bus ADC resolution bits **/
#define INA219_CONFIG_BADCRES_MASK (0x0780)

/** values for bus ADC resolution **/
enum
{
    INA219_CONFIG_BADCRES_9BIT =                (0x0000),  // 9-bit bus res = 0..511
    INA219_CONFIG_BADCRES_10BIT =               (0x0080), // 10-bit bus res = 0..1023
    INA219_CONFIG_BADCRES_11BIT =               (0x0100), // 11-bit bus res = 0..2047
    INA219_CONFIG_BADCRES_12BIT =               (0x0180), // 12-bit bus res = 0..4097
    INA219_CONFIG_BADCRES_12BIT_2S_1060US =     (0x0480), // 2 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_4S_2130US =     (0x0500), // 4 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_8S_4260US =     (0x0580), // 8 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_16S_8510US =    (0x0600), // 16 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_32S_17MS =      (0x0680), // 32 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_64S_34MS =      (0x0700), // 64 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_128S_69MS =     (0x0780), // 128 x 12-bit bus samples averaged together

};

/** mask for shunt ADC resolution bits **/
#define INA219_CONFIG_SADCRES_MASK \
    (0x0078) // Shunt ADC Resolution and Averaging Mask

/** values for shunt ADC resolution **/
enum
{
    INA219_CONFIG_SADCRES_9BIT_1S_84US =    (0x0000),   // 1 x 9-bit shunt sample
    INA219_CONFIG_SADCRES_10BIT_1S_148US =  (0x0008), // 1 x 10-bit shunt sample
    INA219_CONFIG_SADCRES_11BIT_1S_276US =  (0x0010), // 1 x 11-bit shunt sample
    INA219_CONFIG_SADCRES_12BIT_1S_532US =  (0x0018), // 1 x 12-bit shunt sample
    INA219_CONFIG_SADCRES_12BIT_2S_1060US = (0x0048), // 2 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_4S_2130US = (0x0050), // 4 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_8S_4260US = (0x0058), // 8 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_16S_8510US =(0x0060), // 16 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_32S_17MS =  (0x0068), // 32 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_64S_34MS =  (0x0070), // 64 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_128S_69MS = (0x0078), // 128 x 12-bit shunt samples averaged together
};

/** mask for operating mode bits **/
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask

/** values for operating mode **/
enum
{
    INA219_CONFIG_MODE_POWERDOWN =              0x00,       /**< power down */
    INA219_CONFIG_MODE_SVOLT_TRIGGERED =        0x01, /**< shunt voltage triggered */
    INA219_CONFIG_MODE_BVOLT_TRIGGERED =        0x02, /**< bus voltage triggered */
    INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED =    0x03,                                   /**< shunt and bus voltage triggered */
    INA219_CONFIG_MODE_ADCOFF =                 0x04,           /**< ADC off */
    INA219_CONFIG_MODE_SVOLT_CONTINUOUS =       0x05, /**< shunt voltage continuous */
    INA219_CONFIG_MODE_BVOLT_CONTINUOUS =       0x06, /**< bus voltage continuous */
    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS =   0x07, /**< shunt and bus voltage continuous */
};

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
	uint8_t		payloadByte[2], cmdBuf[1];
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
	cmdBuf[0] = deviceRegister;
	payloadByte[0] = (payload >> 8) & 0xFF; /* MSB first */
	payloadByte[1] = payload & 0xFF;        /* LSB */
	returnValue    = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		cmdBuf,
		1,
		payloadByte,
		2,
		gWarpI2cTimeoutMilliseconds);
	if (returnValue != kStatus_I2C_Success)
	{
		warpPrint("\nI'm the INA219 read fucntion. I have failed.\n kWarpStatusDeviceCommunicationFailed\n");
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


/* do you even need device register, 
since read cmd checks the last register pointer*/
WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t cmdBuf[1];
	i2c_status_t status; //status2;
	//warpPrint("\n READING REGISTERS\n");

	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case INA219_REG_CONFIG: case INA219_REG_SHUNTVOLTAGE: 
		case INA219_REG_BUSVOLTAGE: case INA219_REG_POWER: 
		case INA219_REG_CURRENT: case INA219_REG_CALIBRATION: 
		{
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

	cmdBuf[0] = deviceRegister;
	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t*)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	//currently here, why...
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}
	//warpPrint("\n REGISTERS HAVE BEEN READ \n");
	return kWarpStatusOK;
}

// trying to modify the HDC1000 code taking way too long, getting nowhere
// Looks to take read values from over the I2C bus and manipulating them for printing.


// In order to calibrate, need
// claims on Google: 
// SSD1331 0.96" draws 25mA.
// SSD1331 0.95" draws 38mA all white
// So assume max exp current of 50mA, won't be dissapointed
// 2x margin on .96", +31.6% margin on .95"
// 50/(2^15) < I_LSB < 50/(2^12)  ==> Pick I_LSB = 5uA 
// say cuurent LSB = 5 micro-amp
// That's my resolution, good?
// cal = 0.04096/(I_LSB * R_SHUNT) --> 5uA and 0.1 Ohm
// cal = 81920 
// max possible I, with chosen values --> 5*2^15 == 0.16384A
// ~ >3x margin
//needed for power and current correct readings
// 5 uA !!!
//int16_t current_LSB = 5; // [Current is in every $current_LSB-micro-amps], If I Put it in as a correct float of 0.000005, I'd get o/p in amps
//int16_t power_LSB = 100;  // == 20 * current_LSB, gives power in micro-watts?
//uint16_t ina219_cal = 81920;
// Doesn't work as will overflow the 16b of 0x05 anway and get corrected to 16384. :(


// Current limited by resolution
// 2^16 = 65536, ==> current_LSB will be every 6.25uA, but I don't wanna deal with that, cos rounding. So
// Choose 10uA current_LSB
int16_t current_LSB = 10; 	/* uA [Current is in every $current_LSB-micro-amps], 
							If I Put it in as a correct float of 0.000005, I'd get o/p in amps*/
int16_t power_LSB = 200; 	// == 20 * current_LSB, gives power in micro-watts?
uint16_t ina219_cal = 40960;


void
config_and_cal()
{
	uint16_t config=INA219_CONFIG_BVOLTAGERANGE_32V |INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
					INA219_CONFIG_SADCRES_12BIT_1S_532US |INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
 	writeSensorRegisterINA219(INA219_REG_CONFIG, config);
	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_cal);
}

void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;
	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_CONFIG, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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
			warpPrint(" %x,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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
			// the value is internally multiplied by 100, so reverse this (optional)
			//// LSB is 10 uV
			//myshuntvoltage = (float)(readSensorRegisterValueCombined*0.01);

			warpPrint("%d,", (int16_t)readSensorRegisterValueCombined*10);
			
			//// Printing as floats - not necessary
			//// Takes value in 10s of uV, and converts to millivolts.
			//myshuntvoltage = (float)readSensorRegisterValueCombined*0.01;
			//const char buffer[20];
			//int sprintf(buffer, myshuntvoltage);
			//// Takes args BufferIndex and const char *str
			//SEGGER_RTT_WriteString(0, buffer);

		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB);

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
			// bit shift by 3, multiply out by VBUS LSB (4mV), then convert to V (optional)
			int16_t shifty = (readSensorRegisterValueCombined>>3);
			warpPrint("%d,", shifty*4);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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
			mypower = (float)readSensorRegisterValueCombined*power_LSB;
			mypower = (int32_t)readSensorRegisterValueCombined*power_LSB;
			//warpPrint("%d", (float)readSensorRegisterValueCombined*power_mul);
			warpPrint("%d,", mypower);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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
			mycurrent = (int32_t)readSensorRegisterValueCombined*current_LSB;
			//warpPrint(" %d,\n", (int)myshuntvoltage*50);
			warpPrint(" %d,\n", mycurrent);
			//warpPrint(" %d,", (mycurrent));
		}
	}
}
//idc about append yet
uint8_t
appendSensorDataINA219(uint8_t* buf)
{
	uint8_t index = 0;

	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
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

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
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
	// WHY IS readSensorRegisterValueMSB defined as uint16_t
	// It's a 8b value
	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
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

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
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


// Tried to copy the adafruit datasheet
// FLopped
int16_t getShuntVoltage_raw_INA219()
{
    uint16_t value;
    readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2);
	uint16_t LSB = deviceINA219State.i2cBuffer[0];
	uint16_t MSB = deviceINA219State.i2cBuffer[1];
	value = (MSB << 8) | (LSB);
    return value;
}
float getShuntVoltageINA219()
{
    int16_t value;
    value = getShuntVoltage_raw_INA219();
    return value * 10; // LSB is 10uV
}


int16_t getBusVoltageINA219_raw(void)
{
    int16_t value;
	// perform a read to get the reg pointer correct on the reg u wanna read
	// see datasheet about I2C reading and writing bite commands
    readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2);
    uint16_t LSB = deviceINA219State.i2cBuffer[0];
	uint16_t MSB = deviceINA219State.i2cBuffer[1];
	value = (MSB << 8) | (LSB);
	// Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	return value = ((value >> 3) * 4);
	// the number itself is a float but I can printf format it as such.
}
float getBusVoltageINA219(void)
{
	int16_t value = getBusVoltageINA219_raw();
	return value;
}
