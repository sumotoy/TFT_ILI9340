
#include "Adafruit_GFX.h"
#include "TFT_ILI9340.h"
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

#if defined(__SAM3X8E__)
#include <include/pio.h>
  #define SET_BIT(port, bitMask) (port)->PIO_SODR |= (bitMask)
  #define CLEAR_BIT(port, bitMask) (port)->PIO_CODR |= (bitMask)
  #define USE_SPI_LIBRARY
#endif
#ifdef __AVR__
  #define SET_BIT(port, bitMask) *(port) |= (bitMask)
  #define CLEAR_BIT(port, bitMask) *(port) &= ~(bitMask)
#endif
#if defined(__arm__) && defined(CORE_TEENSY)
  #define USE_SPI_LIBRARY
  #define SET_BIT(port, bitMask) digitalWrite(*(port), HIGH);
  #define CLEAR_BIT(port, bitMask) digitalWrite(*(port), LOW);
#endif


TFT_ILI9340::TFT_ILI9340(uint8_t cs, uint8_t dc, uint8_t rst) : Adafruit_GFX(TFT_WIDTH, TFT_HEIGHT) {
	_cs   = cs;
	_rs   = dc;
	_rst  = rst;
	_inited = false;
}

TFT_ILI9340::TFT_ILI9340(uint8_t cs, uint8_t dc) : Adafruit_GFX(TFT_WIDTH, TFT_HEIGHT) {
	_cs   = cs;
	_rs   = dc;
	_rst  = 0;
	_inited = false;
}

//helper
void TFT_ILI9340::setRegister(const uint8_t reg,uint8_t val){
	uint8_t cmd[2] = {reg,val};
	writeCommands(cmd,2);
}

/********************************** low level pin and SPI transfer based on MCU */
#ifdef __AVR__

	inline void TFT_ILI9340::spiwrite(uint8_t c){
		SPDR = c;
		while(!(SPSR & _BV(SPIF)));
	}

	void TFT_ILI9340::writeCommand(uint8_t c){
		*rsport &= ~rspinmask;
		*csport &= ~cspinmask;
		spiwrite(c);
		*csport |= cspinmask;
	}

	void TFT_ILI9340::writeCommands(uint8_t *cmd, uint8_t length){
		*rsport &= ~rspinmask;
		*csport &= ~cspinmask;
		for (uint8_t i = 0; i < length; i++) {
			spiwrite(*cmd++);
		}
		*csport |= cspinmask;
	}
	
	void TFT_ILI9340::writeData(uint8_t c){
		*rsport |=  rspinmask;
		*csport &= ~cspinmask;
		spiwrite(c);
		*csport |= cspinmask;
	} 

	void TFT_ILI9340::writedata16(uint16_t d){
		*rsport |=  rspinmask;
		*csport &= ~cspinmask;
		spiwrite(d >> 8);
		spiwrite(d);
		*csport |= cspinmask;
	} 

	void TFT_ILI9340::setBitrate(uint32_t n){
		if (n >= 8000000) {
			SPI.setClockDivider(SPI_CLOCK_DIV2);
		} else if (n >= 4000000) {
			SPI.setClockDivider(SPI_CLOCK_DIV4);
		} else if (n >= 2000000) {
			SPI.setClockDivider(SPI_CLOCK_DIV8);
		} else {
			SPI.setClockDivider(SPI_CLOCK_DIV16);
		}
	}
#elif defined(__SAM3X8E__)

	inline void TFT_ILI9340::spiwrite(uint8_t c){
		SPI.transfer(c);
	}
	
	void TFT_ILI9340::writeCommand(uint8_t c){
		rsport->PIO_CODR |=  rspinmask;
		csport->PIO_CODR  |=  cspinmask;
		spiwrite(c);
		csport->PIO_SODR  |=  cspinmask;
	}
	
	void TFT_ILI9340::writeCommands(uint8_t *cmd, uint8_t length){
		rsport->PIO_CODR |=  rspinmask;
		csport->PIO_CODR  |=  cspinmask;
		for (uint8_t i = 0; i < length; i++) {
			spiwrite(*cmd++);
		}
		csport->PIO_SODR  |=  cspinmask;
	}
	
	void TFT_ILI9340::writeData(uint8_t c){
		rsport->PIO_SODR |=  rspinmask;
		csport->PIO_CODR  |=  cspinmask;
		spiwrite(c);
		csport->PIO_SODR  |=  cspinmask;
	} 
	
	void TFT_ILI9340::writedata16(uint16_t d){
		rsport->PIO_SODR |=  rspinmask;//HI
		csport->PIO_CODR  |=  cspinmask;//LO
		spiwrite(d >> 8);
		spiwrite(d);
		csport->PIO_SODR  |=  cspinmask;//HI
	}

	void TFT_ILI9340::setBitrate(uint32_t n){
		uint32_t divider=1;
		while (divider < 255) {
			if (n >= 84000000 / divider) break;
			divider = divider - 1;
		}
		SPI.setClockDivider(divider);
	}
#elif defined(__MK20DX128__) || defined(__MK20DX256__)

	void TFT_ILI9340::writeCommand(uint8_t c){
		SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	}

	void TFT_ILI9340::writeCommands(uint8_t *cmd, uint8_t length){
		for (uint8_t i = 0; i < length; i++) {
			SPI0.PUSHR = *cmd++ | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
			while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
		}
	}
	
	void TFT_ILI9340::writeData(uint8_t c){
		SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	}

	void TFT_ILI9340::writedata16(uint16_t d){
		SPI0.PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	}
	/*
	Helper:
	This function return true only if the choosed pin can be used for CS or RS
	*/
	static bool spi_pin_is_cs(uint8_t pin){
		if (pin == 2 || pin == 6 || pin == 9) return true;
		if (pin == 10 || pin == 15) return true;
		if (pin >= 20 && pin <= 23) return true;
		return false;
	}
	
	/*
	Helper:
	This function configure register in relation to pin
	*/
	static uint8_t spi_configure_cs_pin(uint8_t pin){
		switch (pin) {
			case 10: CORE_PIN10_CONFIG = PORT_PCR_MUX(2); return 0x01; // PTC4
			case 2:  CORE_PIN2_CONFIG  = PORT_PCR_MUX(2); return 0x01; // PTD0
			case 9:  CORE_PIN9_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTC3
			case 6:  CORE_PIN6_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTD4
			case 20: CORE_PIN20_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTD5
			case 23: CORE_PIN23_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTC2
			case 21: CORE_PIN21_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTD6
			case 22: CORE_PIN22_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTC1
			case 15: CORE_PIN15_CONFIG = PORT_PCR_MUX(2); return 0x10; // PTC0
		}
		return 0;
	}

	/*
	Helper:
	This function set the speed of the SPI interface
	*/
	void TFT_ILI9340::setBitrate(uint32_t n){
		if (n >= 24000000) {
			ctar = CTAR_24MHz;
		} else if (n >= 16000000) {
			ctar = CTAR_16MHz;
		} else if (n >= 12000000) {
			ctar = CTAR_12MHz;
		} else if (n >= 8000000) {
			ctar = CTAR_8MHz;
		} else if (n >= 6000000) {
			ctar = CTAR_6MHz;
		} else {
			ctar = CTAR_4MHz;
		}
		SIM_SCGC6 |= SIM_SCGC6_SPI0;
		SPI0.MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
		SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
		SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
		SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
	}
#endif

void TFT_ILI9340::commonInit(){
#if defined(__AVR__) 
	pinMode(_rs, OUTPUT);
	pinMode(_cs, OUTPUT);
	csport    = portOutputRegister(digitalPinToPort(_cs));
	rsport    = portOutputRegister(digitalPinToPort(_rs));
	cspinmask = digitalPinToBitMask(_cs);
	rspinmask = digitalPinToBitMask(_rs);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
    //Due defaults to 4mHz (clock divider setting of 21)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
	*csport &= ~cspinmask;
	_inited = true;
#elif defined(__SAM3X8E__) 
	pinMode(_rs, OUTPUT);
	pinMode(_cs, OUTPUT);
	csport    = digitalPinToPort(_cs);
	rsport    = digitalPinToPort(_rs);
	cspinmask = digitalPinToBitMask(_cs);
	rspinmask = digitalPinToBitMask(_rs);
    SPI.begin();
    SPI.setClockDivider(21); // 4 MHz
    //Due defaults to 4mHz (clock divider setting of 21), but we'll set it anyway 
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
	// toggle RST low to reset; CS low so it'll listen to us
	csport ->PIO_CODR  |=  cspinmask; // Set control bits to LOW (idle)
	_inited = true;
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
	if (spi_pin_is_cs(_cs) && spi_pin_is_cs(_rs)
	 && !(_cs ==  2 && _rs == 10) && !(_rs ==  2 && _cs == 10)
	 && !(_cs ==  6 && _rs ==  9) && !(_rs ==  6 && _cs ==  9)
	 && !(_cs == 20 && _rs == 23) && !(_rs == 20 && _cs == 23)
	 && !(_cs == 21 && _rs == 22) && !(_rs == 21 && _cs == 22)) {
		#if defined (_SPIINTERFACE_A)
			CORE_PIN13_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE;
			SPCR.setSCK(13);
			CORE_PIN11_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMOSI(11);
		#else 
			CORE_PIN14_CONFIG = PORT_PCR_MUX(2);
			SPCR.setSCK(14);
			CORE_PIN7_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMOSI(7);
		#endif
		ctar = CTAR_12MHz;
		pcs_data = spi_configure_cs_pin(_cs);
		pcs_command = pcs_data | spi_configure_cs_pin(_rs);
		SIM_SCGC6 |= SIM_SCGC6_SPI0;
		SPI0.MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
		SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
		SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
		SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
		_inited = true;
	} else {
		_inited = false;
		//error! cannot continue
		// TODO!  Escape code to stop all
	}	
#endif
	if (_inited && _rst) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(500);
		digitalWrite(_rst, LOW);
		delay(500);
		digitalWrite(_rst, HIGH);
		delay(500);
	}
}

void TFT_ILI9340::begin(void) {
	commonInit();
	if (_inited) chipInit();
}


void TFT_ILI9340::chipInit() {
	writeCommand(0xEF);
	writeData(0x03);
	writeData(0x80);
	writeData(0x02);
  
	writeCommand(0xCF);
	writeData(0x00);
	writeData(0XC1);
	writeData(0X30);

	writeCommand(0xED);
	writeData(0x64);
	writeData(0x03);
	writeData(0X12);
	writeData(0X81);

	writeCommand(0xE8);
	writeData(0x85);
	writeData(0x00);
	writeData(0x78);

	writeCommand(0xCB);
	writeData(0x39);
	writeData(0x2C);
	writeData(0x00);
	writeData(0x34);
	writeData(0x02);
  
	writeCommand(0xF7);  
	writeData(0x20);

	writeCommand(0xEA);
	writeData(0x00);
	writeData(0x00);
 
	writeCommand(_CMD_PWCTR1);
	writeData(0x23);
	writeCommand(_CMD_PWCTR2);
	writeData(0x10);

	writeCommand(_CMD_VMCTR1);
	writeData(0x3e);
	writeData(0x28);
	writeCommand(_CMD_VMCTR2);
	writeData(0x86);
	writeCommand(_CMD_MADCTL);
	writeData(BIT_MADCTL_MX | BIT_MADCTL_BGR);
	writeCommand(_CMD_PIXFMT);
	writeData(0x55);

	writeCommand(_CMD_FRMCTR1);
	writeData(0x00);
	writeData(0x18);

	writeCommand(_CMD_DFUNCTR);
	writeData(0x08);
	writeData(0x82);
	writeData(0x27);

	writeCommand(0xF2);
	writeData(0x00);
	writeCommand(_CMD_GAMMASET);
	writeData(0x01);

	writeCommand(_CMD_GMCTRP1);
	writeData(0x0F);
	writeData(0x31);
	writeData(0x2B);
	writeData(0x0C);
	writeData(0x0E);
	writeData(0x08);
	writeData(0x4E);
	writeData(0xF1);
	writeData(0x37);
	writeData(0x07);
	writeData(0x10);
	writeData(0x03);
	writeData(0x0E);
	writeData(0x09);
	writeData(0x00);

	writeCommand(_CMD_GMCTRN1);
	writeData(0x00);
	writeData(0x0E);
	writeData(0x14);
	writeData(0x03);
	writeData(0x11);
	writeData(0x07);
	writeData(0x31);
	writeData(0xC1);
	writeData(0x48);
	writeData(0x08);
	writeData(0x0F);
	writeData(0x0C);
	writeData(0x31);
	writeData(0x36);
	writeData(0x0F);

	writeCommand(_CMD_SLPOUT); 
	delay(120); 		
	writeCommand(_CMD_DISPON); 
}


void TFT_ILI9340::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	writeCommand(_CMD_CASET);
	writedata16(x0);
	writedata16(x1);
	writeCommand(_CMD_PASET);
	writedata16(y0);
	writedata16(y1);
	writeCommand(_CMD_RAMWR);
}

bool TFT_ILI9340::boundaryCheck(int16_t x,int16_t y){
	if ((x >= _width) || (y >= _height)) return true;
	return false;
}

void TFT_ILI9340::pushColor(uint16_t color) {
	writedata16(color);
}

void TFT_ILI9340::drawPixel(int16_t x, int16_t y, uint16_t color) {
	if (boundaryCheck(x,y)) return;
	if ((x < 0) || (y < 0)) return;
	setAddrWindow(x,y,x+1,y+1);
	writedata16(color);
}


void TFT_ILI9340::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
	if (boundaryCheck(x,y)) return;
	if (((y + h) - 1) >= _height) h = _height - y;
	setAddrWindow(x,y,x,(y+h)-1);
	while (h--) {
		writedata16(color);
	}
}


void TFT_ILI9340::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
	if (boundaryCheck(x,y)) return;
	if (((x+w) - 1) >= _width)  w = _width-x;
	setAddrWindow(x,y,(x+w)-1,y);
	while (w--) {
		writedata16(color);
	}
}

void TFT_ILI9340::fillScreen(uint16_t color) {
	fillRect(0,0,_width,_height,color);
}

// fill a rectangle
void TFT_ILI9340::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
	if (boundaryCheck(x,y)) return;
	if (((x + w) - 1) >= _width)  w = _width - x;
	if (((y + h) - 1) >= _height) h = _height - y;
	setAddrWindow(x,y,(x+w)-1,(y+h)-1);
	for (y = h;y > 0;y--) {
		for (x = w;x > 0;x--) {
			writedata16(color);
		}
	}
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t TFT_ILI9340::Color565(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void TFT_ILI9340::setRotation(uint8_t m) {
	uint8_t _dta;
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
		case 0:
			_dta = BIT_MADCTL_MX | BIT_MADCTL_BGR;
			_width  = TFT_WIDTH;
			_height = TFT_HEIGHT;
			break;
		case 1:
			_dta = BIT_MADCTL_MV | BIT_MADCTL_BGR;
			_width  = TFT_HEIGHT;
			_height = TFT_WIDTH;
		break;
		case 2:
			_dta = BIT_MADCTL_MY | BIT_MADCTL_BGR;
			_width  = TFT_WIDTH;
			_height = TFT_HEIGHT;
		break;
		case 3:
			_dta = BIT_MADCTL_MV | BIT_MADCTL_MY | BIT_MADCTL_MX | BIT_MADCTL_BGR;
			_width  = TFT_HEIGHT;
			_height = TFT_WIDTH;
		break;
	}
	writeCommand(_CMD_MADCTL); 
	writeData(_dta);
}


void TFT_ILI9340::invertDisplay(boolean i) {
	writeCommand(i ? _CMD_INVON : _CMD_INVOFF); 
}



