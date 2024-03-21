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

void		initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payloadBtye);
WarpStatus 	configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1);
void		printSensorDataMMA8451Q(bool hexModeFlag);
uint8_t		appendSensorDataMMA8451Q(uint8_t* buf);

const uint8_t bytesPerMeasurementMMA8451Q            = 6;
const uint8_t bytesPerReadingMMA8451Q                = 2;
const uint8_t numberOfReadingsPerMeasurementMMA8451Q = 3;

// Default adress already defined

#define MMA8451_REG_F_STATUS 0x00

// Define most on-device registers
#define MMA8451_REG_OUT_X_MSB 0x01
#define MMA8451_REG_OUT_X_LSB 0x02
#define MMA8451_REG_OUT_Y_MSB 0x03
#define MMA8451_REG_OUT_Y_LSB 0x04
#define MMA8451_REG_OUT_Z_MSB 0x05
#define MMA8451_REG_OUT_Z_LSB 0x06

#define MMA8451_REG_F_SETUP			0x09
#define MMA8451_REG_TRIG_CONFIG		0x0A
#define MMA8451_REG_SYSMOD			0x08
#define MMA8451_REG_INT_SOURCE		0x0C
#define MMA8451_REG_WHO_AM_I		0x0D
#define MMA8451_REG_XYZ_DATA_CFG	0xE
#define MMA8451_REG_HP_FILTER_CUTOFF 0x0F
#define MMA8451_REG_PL_STATUS		0x10
#define MMA8451_REG_PL_CFG			0x11
#define MMA8451_REG_PL_COUNT		0x12
#define MMA8451_REG_PL_BF_ZCOMP		0x13
#define MMA8451_REG_P_L_THS_REG		0x14

#define MMA8451_REG_FF_MT_CFG		0x16
#define MMA8451_REG_FF_MT_COUNT		0x17
#define MMA8451_REG_FF_MT_THS		0x18

#define MMA8451_REG_TRANSIENT_CFG	0x1D
#define MMA8451_REG_TRANSIENT_SCR	0x1E
#define MMA8451_REG_TRANSIENT_THS	0x1F
#define MMA8451_REG_TRANSIENT_COUNT 0x20

#define MMA8451_REG_PULSE_CFG		0x21
#define MMA8451_REG_PULSE_SRC		0x22
#define MMA8451_REG_PULSE_THSX		0x23
#define MMA8451_REG_PULSE_THSY		0x24
#define MMA8451_REG_PULSE_THSZ		0x25
#define MMA8451_REG_PULSE_TMLT		0x26
#define MMA8451_REG_PULSE_LTCY		0x27
#define MMA8451_REG_PULSE_WIND		0x28

#define MMA8451_REG_CTRL_REG1 0x2A
#define MMA8451_REG_CTRL_REG2 0x2B
#define MMA8451_REG_CTRL_REG3 0x2C
#define MMA8451_REG_CTRL_REG4 0x2D
#define MMA8451_REG_CTRL_REG5 0x2E


///@{
//* Different portrait and landscape settings */
#define MMA8451_PL_PUF 0
#define MMA8451_PL_PUB 1
#define MMA8451_PL_PDF 2
#define MMA8451_PL_PDB 3
#define MMA8451_PL_LRF 4
#define MMA8451_PL_LRB 5
#define MMA8451_PL_LLF 6
#define MMA8451_PL_LLB 7
///@}


typedef enum {
  MMA8451_RANGE_8_G = 0b10, // +/- 8g
  MMA8451_RANGE_4_G = 0b01, // +/- 4g
  MMA8451_RANGE_2_G = 0b00  // +/- 2g (default value)
} mma8451_range_t;

/*! Used with register 0x2A (MMA8451_REG_CTRL_REG1) to set bandwidth */
typedef enum {
  MMA8451_DATARATE_800_HZ = 0b000,  //  800Hz
  MMA8451_DATARATE_400_HZ = 0b001,  //  400Hz
  MMA8451_DATARATE_200_HZ = 0b010,  //  200Hz
  MMA8451_DATARATE_100_HZ = 0b011,  //  100Hz
  MMA8451_DATARATE_50_HZ = 0b100,   //   50Hz
  MMA8451_DATARATE_12_5_HZ = 0b101, // 12.5Hz
  MMA8451_DATARATE_6_25HZ = 0b110,  // 6.25Hz
  MMA8451_DATARATE_1_56_HZ = 0b111, // 1.56Hz

  MMA8451_DATARATE_MASK = 0b111
} mma8451_dataRate_t;

#define FRAC_2d1 5000
#define FRAC_2d2 2500
#define FRAC_2d3 1250
#define FRAC_2d4 625
#define FRAC_2d5 313
#define FRAC_2d6 156
#define FRAC_2d7 78
#define FRAC_2d8 39
#define FRAC_2d9 20
#define FRAC_2d10 10
#define FRAC_2d11 5
#define FRAC_2d12 2

