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

// copied from MMA8451Q, HDC1000 and adafruit library driver
#define INA219_REG_CONFIG (0X00)
 /** shunt voltage register **/
#define INA219_REG_SHUNTVOLTAGE (0x01)
/** bus voltage register **/
#define INA219_REG_BUSVOLTAGE (0x02)
/** power register **/
#define INA219_REG_POWER (0x03)
/** current register **/
#define INA219_REG_CURRENT (0x04)
/** calibration register **/
#define INA219_REG_CALIBRATION (0x05)
/** reset bit **/
#define INA219_CONFIG_RESET (0x8000) // Reset Bit
/** mask for bus voltage range **/
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000),// Bus Voltage Range Mask

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
	
	int16_t value;

	//needed for power and current o/p
	int16_t current_div;
	int16_t power_mul = 20*current_div; 


	WarpStatus	i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	//kWarpSensorOutputRegisterINA219SHUNT_VOLTAGE			= 0x01,
	//kWarpSensorOutputRegisterINA219BUS_VOLTAGE			= 0x02,
	//kWarpSensorOutputRegisterINA219POWER					= 0x03,
	//kWarpSensorOutputRegisterINA219CURRENT				= 0x04,


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	// read and print probably incompatible due to calling different macros even if they have the same hex values
	// use same macros???
	i2cReadStatus = readSensorRegisterINA219(INA219_REG_CONFIG, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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
			//myshuntvoltage = readSensorRegisterValueCombined;
			warpPrint(" %x,", readSensorRegisterValueCombined );
			//warpPrint(" %f,", (myshuntvoltage/(100) ));
		}
	}


	i2cReadStatus = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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
			// the value is internally multiplied by 100, so reverse this (optional)
			// this is in mV
			myshuntvoltage = (readSensorRegisterValueCombined)*0.01;
			//warpPrint(" %d,", readSensorRegisterValueCombined );
			warpPrint(" %d,", (myshuntvoltage));
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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
			// bit shift by 3, multiply out by VBUS LSB (4mV), then convert to V (optional)
			mybusvoltage = ((readSensorRegisterValueCombined>>3)*4)/**0.001*/ ;
			//readSensorRegisterValueCombined;
			//warpPrint(" %d,", readSensorRegisterValueCombined );
			warpPrint(" %d,", (myshuntvoltage));
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);
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
			mypower = readSensorRegisterValueCombined*power_mul;
			warpPrint(" %d,", (mypower));
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB) << 8) | (readSensorRegisterValueLSB & 0xFF);

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

			mycurrent = (readSensorRegisterValueCombined)/current_div;
			warpPrint(" %f,", (readSensorRegisterValueCombined));
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

// Insert the adafruit library functions here, since the above doesn't seem to work and idk how2get it working
// manually calling printSensorDataINA219 doesn't work for w/e reason. broken


// when placing warpPrint inside here it doesn't want to print out for some reason.
// And yes, I still had it return the correct type.
int16_t getBusVoltageINA219(void)
{
    uint16_t value;
	// perform a read to get the reg pointer correct on the reg u wanna read
	// see datasheet about I2C reading and writing bite commands
    readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2);
    uint16_t LSB = deviceINA219State.i2cBuffer[0];
	uint16_t MSB = deviceINA219State.i2cBuffer[1];
	value = (MSB << 8) | (LSB);
	// Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	value = ((value >> 3) * 4);
	// the number itself is a float but I can printf format it as such.
	value = (value*0.001);
	return value;
}

//-----------------------------------------------------------------------------------------------------------------------------------
// Use the 32V_1A setting for config from adafruit library - voltage range on VBUS doesn't seem to matter as per datasheet.
// LSB of VBUS is 4mV
// Current 1A bc idk what coud current be. 

///*!
// *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
// *  @return the raw bus voltage reading
// */
//int16_t Adafruit_INA219::getBusVoltage_raw() {
//  uint16_t value;
//
//  Adafruit_BusIO_Register bus_voltage_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_BUSVOLTAGE, 2, MSBFIRST);
//  _success = bus_voltage_reg.read(&value);
//
//  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
//  return (int16_t)((value >> 3) * 4);
//}
//
///*!
// *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
// *  @return the raw shunt voltage reading
// */
//int16_t Adafruit_INA219::getShuntVoltage_raw() {
//  uint16_t value;
//  Adafruit_BusIO_Register shunt_voltage_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_SHUNTVOLTAGE, 2, MSBFIRST);
//  _success = shunt_voltage_reg.read(&value);
//  return value;
//}
//
///*!
// *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
// *  @return the raw current reading
// */
//int16_t Adafruit_INA219::getCurrent_raw() {
//  uint16_t value;
//
//  // Sometimes a sharp load will reset the INA219, which will
//  // reset the cal register, meaning CURRENT and POWER will
//  // not be available ... avoid this by always setting a cal
//  // value even if it's an unfortunate extra step
//  Adafruit_BusIO_Register calibration_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
//  calibration_reg.write(ina219_calValue, 2);
//
//  // Now we can safely read the CURRENT register!
//  Adafruit_BusIO_Register current_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CURRENT, 2, MSBFIRST);
//  _success = current_reg.read(&value);
//  return value;
//}
//
///*!
// *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
// *  @return raw power reading
// */
//int16_t Adafruit_INA219::getPower_raw() {
//  uint16_t value;
//
//  // Sometimes a sharp load will reset the INA219, which will
//  // reset the cal register, meaning CURRENT and POWER will
//  // not be available ... avoid this by always setting a cal
//  // value even if it's an unfortunate extra step
//  Adafruit_BusIO_Register calibration_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
//  calibration_reg.write(ina219_calValue, 2);
//
//  // Now we can safely read the POWER register!
//  Adafruit_BusIO_Register power_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_POWER, 2, MSBFIRST);
//  _success = power_reg.read(&value);
//  return value;
//}
//
///*!
// *  @brief  Gets the shunt voltage in mV (so +-327mV)
// *  @return the shunt voltage converted to millivolts
// */
//float Adafruit_INA219::getShuntVoltage_mV() {
//  int16_t value;
//  value = getShuntVoltage_raw();
//  return value * 0.01;
//}
//
///*!
// *  @brief  Gets the bus voltage in volts
// *  @return the bus voltage converted to volts
// */
//float Adafruit_INA219::getBusVoltage_V() {
//  int16_t value = getBusVoltage_raw();
//  return value * 0.001;
//}
//
///*!
// *  @brief  Gets the current value in mA, taking into account the
// *          config settings and current LSB
// *  @return the current reading convereted to milliamps
// */
//float Adafruit_INA219::getCurrent_mA() {
//  float valueDec = getCurrent_raw();
//  valueDec /= ina219_currentDivider_mA;
//  return valueDec;
//}
//
///*!
// *  @brief  Gets the power value in mW, taking into account the
// *          config settings and current LSB
// *  @return power reading converted to milliwatts
// */
//float Adafruit_INA219::getPower_mW() {
//  float valueDec = getPower_raw();
//  valueDec *= ina219_powerMultiplier_mW;
//  return valueDec;
//}
//
///*!
// *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
// *          of current.  Each unit of current corresponds to 100uA, and
// *          each unit of power corresponds to 2mW. Counter overflow
// *          occurs at 3.2A.
// *  @note   These calculations assume a 0.1 ohm resistor is present
// */
//void Adafruit_INA219::setCalibration_32V_2A() {
//  // By default we use a pretty huge range for the input voltage,
//  // which probably isn't the most appropriate choice for system
//  // that don't use a lot of power.  But all of the calculations
//  // are shown below if you want to change the settings.  You will
//  // also need to change any relevant register settings, such as
//  // setting the VBUS_MAX to 16V instead of 32V, etc.
//
//  // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
//  // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08,
//  // 0.04) RSHUNT = 0.1               (Resistor value in ohms)
//
//  // 1. Determine max possible current
//  // MaxPossible_I = VSHUNT_MAX / RSHUNT
//  // MaxPossible_I = 3.2A
//
//  // 2. Determine max expected current
//  // MaxExpected_I = 2.0A
//
//  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
//  // MinimumLSB = MaxExpected_I/32767
//  // MinimumLSB = 0.000061              (61uA per bit)
//  // MaximumLSB = MaxExpected_I/4096
//  // MaximumLSB = 0,000488              (488uA per bit)
//
//  // 4. Choose an LSB between the min and max values
//  //    (Preferrably a roundish number close to MinLSB)
//  // CurrentLSB = 0.0001 (100uA per bit)
//
//  // 5. Compute the calibration register
//  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
//  // Cal = 4096 (0x1000)
//
//  ina219_calValue = 4096;