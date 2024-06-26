#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	
 */
// enum
// {
// 	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
// 	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
// 	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
// 	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
// 	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
// };

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

/*!
 * @brief Initialises the SSD1331 OLED driver
 */
int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins(); // was previously enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
    SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
    SEGGER_RTT_WriteString(0, "\r\n\tMeow Meow Meow Green\n");

    /*
     *	Read the manual for the SSD1331 (SSD1331_1.2.pdf) to figure
     *	out how to fill the entire screen with the brightest shade
     *	of green.
     */

    // set channel contrasts
    writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0xFF);
    
    // Set highest 1st phase precharge
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3E);	

    // Don't need to actually set current cos it already defaults to max
	// See manual
	//writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	//writeCommand(0x0F);

    writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(0x00); //start col
    writeCommand(0x00); // start row
    writeCommand(0x5F); // end col
    writeCommand(0x3F); // end row
   
    writeCommand(0x00); 
    writeCommand(0x3F); // setting border
    writeCommand(0x00);
    
    writeCommand(0x00);
    writeCommand(0x3F); // setting fill
    writeCommand(0x00);

    SEGGER_RTT_WriteString(0, "\r\n\tRectangle should be shaded by now\n");

	return 0;
}

// COlours ABC (BGR) are 5,6,5 bit respectively

int
devSSD_fill_red(void)
{
	writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(0x00); //start col
    writeCommand(0x00); // start row
    writeCommand(0x5F); // end col
    writeCommand(0x3F); // end row
   
    writeCommand(0b11111); 
    writeCommand(0x00); // setting border
    writeCommand(0x00);
    
    writeCommand(0b11111);
    writeCommand(0x00); // setting fill
    writeCommand(0x00);
	return 0;
}

int
devSSD_fill_green(void)
{
	writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(0x00); //start col
    writeCommand(0x00); // start row
    writeCommand(0x5F); // end col
    writeCommand(0x3F); // end row
   
    writeCommand(0x00); 
    writeCommand(0b111111); // setting border
    writeCommand(0x00);
    
    writeCommand(0x00);
    writeCommand(0b111111); // setting fill
    writeCommand(0x00);
	return 0;
}

int
devSSD_fill_blue(void)
{
	writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(0x00); //start col
    writeCommand(0x00); // start row
    writeCommand(0x5F); // end col
    writeCommand(0x3F); // end row
   
    writeCommand(0x00); 
    writeCommand(0x00); // setting border
    writeCommand(0b11111);

    writeCommand(0x00);
    writeCommand(0x00); // setting fill
    writeCommand(0b11111);
	return 0;
	
}

int
devSSD_fill_white(void)
{
	writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(0x00); //start col
    writeCommand(0x00); // start row
    writeCommand(0x5F); // end col
    writeCommand(0x3F); // end row
   
    writeCommand(0b11111); 
    writeCommand(0b011111); // setting border
    writeCommand(0b11111);

    writeCommand(0b11111); 
    writeCommand(0b011111); // setting fill
    writeCommand(0b11111);
	return 0;	
}

int
devSSD_fill_blank(void)
{
	writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(0x00); //start col
    writeCommand(0x00); // start row
    writeCommand(0x5F); // end col
    writeCommand(0x3F); // end row
   
    writeCommand(0x00); 
    writeCommand(0x00); // setting border
    writeCommand(0x00);

    writeCommand(0x00); 
    writeCommand(0x00); // setting fill
    writeCommand(0x00);
	return 0;	
}