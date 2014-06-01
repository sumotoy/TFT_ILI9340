/*
	ILI9340C - A fast SPI driver for TFT that use Ilitek ILI9340C.
	
	Features:
	- Very FAST!, expecially with Teensy 3.x where uses DMA SPI.
	- It uses just 4 or 5 wires.
	- Compatible at command level with Adafruit display series so it's easy to adapt existing code.
	- It uses the standard Adafruit_GFX Library (you need to install). 
	
	Background:
	Adafruit it's a great company, always very nice with developers and released a library for
	all his products completely free downloadable. However I don't like some of their libraries
	since are slow and not optimized for all popular MCU. I got a display from Ebay
	http://www.ebay.com/itm/281304733556
	and I'm not using the Adafruit one since was not available when I ordered but it looks
	exact the same and works in the same way so I've tried the related library first that
	was slow and not optimized, it works but can be much better.
	Paul Stoffregen and Adafruit maded together a great work for the ST7735 that it's a popular
	display and works fast with almost all popular MCU so I grab some of the routines and applied
	to this one, optimized some of the code and result it was really nice.
	
	
	Code Optimizations:
	The purpose of this library it's SPEED. I have tried to use hardware optimized calls
	where was possible and results are quite good for most applications, actually nly filled circles
    are still a bit slow. Many SPI call has been optimized by reduce un-needed triggers to RS and CS
	lines. Of course it can be improved so feel free to add suggestions.
	-------------------------------------------------------------------------------
    Copyright (c) 2014, .S.U.M.O.T.O.Y., coded by Max MC Costa.    

    ILI9340C Library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ILI9340C Library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    This file needs the following Libraries:
 
    Adafruit_GFX by Adafruit:
    https://github.com/adafruit/Adafruit-GFX-Library
	Remember to update GFX library often to have more features with this library!
	From this version I'm using my version of Adafruit_GFX library:
	https://github.com/sumotoy/Adafruit-GFX-Library
	It has faster char rendering and some small little optimizations but you can
	choose one of the two freely since are both fully compatible.
	''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
	Special Thanks:
	Thanks Adafruit for his Adafruit_GFX!
	Thanks to Paul Stoffregen for his beautiful Teensy3 and DMA SPI.
	
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Version:
	0.5b1: First one and working.
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	BugList of the current version:
	
	- Actually no scroll commands (only in release will be included).
*/
#ifndef _TFT_ILI9340H_
#define _TFT_ILI9340H_

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif


#if defined(__MK20DX128__) || defined(__MK20DX256__)
	//Teensy3.x has 2 SPI interfaces, choose one
	#define _SPIINTERFACE_A   //mosi=11, sclk=13 (default)
	//#define _SPIINTERFACE_B //mosi=7, sclk=14
	#if defined (_SPIINTERFACE_A) && defined(_SPIINTERFACE_B)
		#error you must choose only one SPI interface
	#endif
#endif

#if defined(__SAM3X8E__)
#include <include/pio.h>
  #define PROGMEM
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
  #define pgm_read_word(addr) (*(const unsigned short *)(addr))
typedef unsigned char prog_uchar;
#endif

#if defined(__MK20DX128__) || defined(__MK20DX256__)
	//do not touch here
	#define CTAR_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
	#define CTAR_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
	#define CTAR_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
	#define CTAR_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
	#define CTAR_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
	#define CTAR_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#endif

#ifdef __AVR__
  #include <avr/pgmspace.h>
#endif



#define TFT_WIDTH  				240//240
#define TFT_HEIGHT 				320//320
#define TFT_MW					TFT_WIDTH -1
#define TFT_MH					TFT_HEIGHT -1

#define _CMD_NOP     			0x00//No Operation
#define _CMD_SWRESET 			0x01//Software Reset
#define _CMD_SLPIN   			0x10//Enter Sleep Mode 
#define _CMD_SLPOUT  			0x11//*Sleep OUT
#define _CMD_PTLON   			0x12//Partial Mode ON 
#define _CMD_NORON   			0x13//Normal Display Mode ON 
#define _CMD_INVOFF  			0x20//Display Inversion OFF 
#define _CMD_INVON   			0x21//Display Inversion ON 
#define _CMD_GAMMASET 			0x26//*Gamma Set
#define _CMD_DISPOFF 			0x28//Display OFF
#define _CMD_DISPON  			0x29//*Display ON
#define _CMD_CASET   			0x2A//*Column Address Set 
#define _CMD_PASET   			0x2B//*Page Address Set
#define _CMD_RAMWR   			0x2C//*Memory Write
#define _CMD_PTLAR   			0x30//Partial Area
#define _CMD_VSCRLLD   			0x33//Vertical Scrolling Definition
#define _CMD_TEFXOFF   			0x34//Tearing Effect Line OFF
#define _CMD_TEFXON   			0x35//Tearing Effect Line ON
#define _CMD_MADCTL  			0x36//*Memory Access Control
#define _CMD_VSCLLSA  			0x37//Vertical Scrolling Start Address
#define _CMD_IDLEOFF  			0x38//Idle Mode OFF
#define _CMD_IDLEON  			0x39//Idle Mode ON
#define _CMD_PIXFMT  			0x3A//*Pixel Format Set
#define _CMD_WRMEMCON  			0x3C//Write Memory Continue
#define _CMD_TEARSCN  			0x44//Set Tear Scanline
#define _CMD_WDBRIGHT  			0x51//Write Display Brightness
#define _CMD_WCTRLDIS  			0x53//Write CTRL Display 
#define _CMD_WCABRGCTRL  		0x55//Write Content Adaptive Brightness Control 
#define _CMD_WCABCMBRG  		0x5E//Write CABC Minimum Brightness
#define _CMD_RGBINTSC  			0xB0//RGB Interface Signal Control
#define _CMD_FRMCTR1 			0xB1//*Frame Control (In Normal Mode) 
#define _CMD_FRMCTR2 			0xB2//Frame Control (In Idle Mode) 
#define _CMD_FRMCTR3 			0xB3//Frame Control (In Partial Mode) 
#define _CMD_INVCTR  			0xB4//Display Inversion Control
#define _CMD_BLKPC  			0xB5//Blanking Porch Control
#define _CMD_DFUNCTR 			0xB6//*Display Function Control
#define _CMD_ENTMSET 			0xB7//Entry Mode Set
#define _CMD_BKLGCTRL1 			0xB8//Backlight Control 1
#define _CMD_BKLGCTRL2 			0xB9//Backlight Control 2
#define _CMD_BKLGCTRL3 			0xBA//Backlight Control 3
#define _CMD_BKLGCTRL4 			0xBB//Backlight Control 4
#define _CMD_BKLGCTRL5 			0xBC//Backlight Control 5
#define _CMD_BKLGCTRL6 			0xBD//Backlight Control 6
#define _CMD_BKLGCTRL7 			0xBE//Backlight Control 7
#define _CMD_BKLGCTRL8 			0xBF//Backlight Control 8
#define _CMD_PWCTR1  			0xC0//*Power Control 1
#define _CMD_PWCTR2  			0xC1//*Power Control 2
#define _CMD_PWCTR3  			0xC2//Power Control 3 (normal)
#define _CMD_PWCTR4  			0xC3//Power Control 4 (idle)
#define _CMD_PWCTR5  			0xC4//Power Control 5 (partial)
#define _CMD_VMCTR1  			0xC5//VCOM Control 1
#define _CMD_VMCTR2  			0xC7//VCOM Control 2
#define _CMD_MVMEMWR  			0xD0//NV Memory Write 
#define _CMD_MVMEMPK  			0xD0//NV Memory Protection Key
#define _CMD_POSGAMUT  			0xE0//Positive Gamma Correction
#define _CMD_NEGGAMUT  			0xE1//Negative Gamma Correction
#define _CMD_DGAMMCTRL1  		0xE2//Digital Gamma Control 1
#define _CMD_DGAMMCTRL2  		0xE3//Digital Gamma Control 2
#define _CMD_INTFCTRL  			0xF6//Interface Control


#define BIT_MADCTL_MY 			0x80//*
#define BIT_MADCTL_MX  			0x40//*
#define BIT_MADCTL_MV 			0x20//*
#define BIT_MADCTL_ML  			0x10
#define BIT_MADCTL_RGB 			0x00
#define BIT_MADCTL_BGR 			0x08//*
#define BIT_MADCTL_MH  			0x04


#define _CMD_RDID1   			0xDA
#define _CMD_RDID2   			0xDB
#define _CMD_RDID3   			0xDC
#define _CMD_RDID4   			0xDD

#define _CMD_GMCTRP1 			0xE0
#define _CMD_GMCTRN1 			0xE1
/*
#define ILI9340_PWCTR6  0xFC

*/




class TFT_ILI9340 : public Adafruit_GFX {

 public:

  TFT_ILI9340(uint8_t CS, uint8_t RS, uint8_t RST);
  TFT_ILI9340(uint8_t CS, uint8_t RS);

  void     			begin(void),
					setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1),
					pushColor(uint16_t color),
					fillScreen(uint16_t color),
					drawPixel(int16_t x, int16_t y, uint16_t color),
					drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
					drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
					fillRect(int16_t x, int16_t y, int16_t w, int16_t h,uint16_t color),
					setRotation(uint8_t r),
					invertDisplay(boolean i);
	void 			clearScreen(int16_t color = 0x0000);				
	uint16_t 		Color565(uint8_t r, uint8_t g, uint8_t b);
	//void 			setBrightness(byte val);
	void 			writeData(uint8_t data);
	void 			setBitrate(uint32_t n);//speed for the SPI interface

 private:
	bool			_inited;
	void	 		commonInit();
	void 			chipInit();
	void 			writedata16(uint16_t d);
	void 			writeCommand(uint8_t c);
	void  			writeCommands(uint8_t *cmd, uint8_t length);
	void 			setRegister(const uint8_t reg,uint8_t val);
	bool 			boundaryCheck(int16_t x,int16_t y);

#if defined(__AVR__)
	void	 		spiwrite(uint8_t);
	volatile uint8_t *dataport, *clkport, *csport, *rsport;
	uint8_t 		_cs,_rs,_sid,_sclk,_rst;
	uint8_t  		datapinmask, clkpinmask, cspinmask, rspinmask;
#endif
#if defined(__SAM3X8E__)
	void	 		spiwrite(uint8_t);
	Pio *dataport, *clkport, *csport, *rsport;
	uint8_t 		_cs,_rs,_sid,_sclk,_rst;
	uint32_t  		datapinmask, clkpinmask, cspinmask, rspinmask;
#endif 

#if defined(__MK20DX128__) || defined(__MK20DX256__)
	uint8_t 		_cs,_rs,_rst;
	uint8_t 		pcs_data, pcs_command;
	uint32_t 		ctar;
	volatile uint8_t *datapin, *clkpin, *cspin, *rspin;
#endif
};

#endif
