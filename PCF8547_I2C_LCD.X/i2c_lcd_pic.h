/* 
 * File:   
 * Author: Alameen v
 * Comments: Interfacing Character LCD by using I2C for PIC MCU
 * Revision history: REV_00
 */
 
#ifndef I2C_LCD_PIC_H
#define	I2C_LCD_PIC_H

#include <xc.h> 

#define _XTAL_FREQ 10e6

//#define lcdType20x4
#define lcdType16x2

#define RS		0b00000001	//PCF8574T I2C IC pin -> P0		// RS -> 0 cmd , 1 data
#define RW		0b00000010  //PCF8574T I2C IC pin -> P1// RW -> 0 write , 1 read  ==> Not used and set to 0 -> write
#define EN		0b00000100	// PCF8574T I2C IC pin -> P2

#define PCF8574T_addr	0x4E		//write address - PCF8574T

#define _backlight  0x08

#define CMD		0
#define DATA	1

#define DISPLAY_OFF		0b00001000
#define DISPLAY_ON		0b00001100
#define CURSOR_SHOW		0b00000010
#define CURSOR_BLINK	0b00000001
#define CURSOR_NONE		0b00000000


#define I2C_Master_Wait() {\
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));\
}

#define I2C_Master_Start() {\
    I2C_Master_Wait();\
    SSPCON2bits.SEN = 1;\
}

#define I2C_Master_Stop() {\
    I2C_Master_Wait();\
    SSPCON2bits.PEN = 1;\
}

#define I2C_Master_Write( d) { \
    I2C_Master_Wait();  \
    SSPBUF = d;         \
}

typedef signed char     int8_t;
typedef unsigned char   uint8_t;
typedef unsigned short  uint16_t;

// Function prototyping
void I2C_Master_Init(const unsigned long clk);

//unsigned short I2C_Master_Read(unsigned short a);
void i2c_lcd_Write(unsigned char cmd);
void expanderWrite(unsigned char cmdData);
void delay_loop( uint16_t nCount );
void pulseWrite( uint8_t portPulse );
void sendByte( uint8_t byte , uint8_t mode );
void i2c_lcd_Init(void);
void i2c_lcd_Print( char *str);
void gotoXY( uint8_t X,  uint8_t Y );
void createChar(  uint8_t location, uint8_t charMap[] );
void getChar( uint8_t X, uint8_t Y, uint8_t location );
void i2c_lcd_Print_Int( uint8_t X, uint8_t Y, int val, uint8_t numOfDigits );



// Function definition
void I2C_Master_Init(const unsigned long clk) {
    
    /* SSPADD = (unsigned char)( (_XTAL_FREQ / (4 * clock)) - 1 );
     * 
     * _XTAL_FREQ = 10MHz
     * clock = 100kHz
     */
    
    SSPCON1 = 0b00101000;
    SSPCON2 = 0;
    SSPADD = 0x18;  //24 in decimal
    SSPSTAT = 0;    //0b11000000;
    TRISCbits.RC3 = 1; // SCL as INPUT given in data sheet
    TRISCbits.RC4 = 1; // SDA as INPUT given in data sheet
}

void i2c_lcd_Write(unsigned char cmd){	
	
	I2C_Master_Start();
	I2C_Master_Write( ( uint8_t )(PCF8574T_addr ) );
	I2C_Master_Write( cmd );
	I2C_Master_Stop();
   
}

void expanderWrite(unsigned char cmdData){
    i2c_lcd_Write( (uint8_t )( cmdData | _backlight ) );
}

/*
unsigned short I2C_Master_Read(unsigned short a) {
    unsigned short temp;
    I2C_Master_Wait();
    RCEN = 1;
    I2C_Master_Wait();
    temp = SSPBUF;
    I2C_Master_Wait();
    ACKDT = (a) ? 0 : 1;
    ACKEN = 1;
    return temp;
}
 */

void delay_loop( uint16_t nCount  ){
	while (nCount != 0){
      nCount--;
    }
}

void pulseWrite( uint8_t portPulse ){
	
	portPulse |=  EN ;
    expanderWrite(  portPulse  );
	
	delay_loop( 0x0010 );
	
	portPulse &= ~EN;
	expanderWrite(  portPulse  );
	
}

void sendByte( uint8_t byte , uint8_t mode ){
	
	uint8_t mask_lower_nibble , swap_mask_Lower_nibble , _data;
	
	mask_lower_nibble = byte;
	
	// Shift 4 place to the left and Mask lower byte -> SWAP and MASK
	swap_mask_Lower_nibble =   (uint8_t)( ( byte << 4 ) & 0xF0 );		
	
	mask_lower_nibble		&=	0xF0;			// Mask lower nibble
	
	_data		=	0b00000101;
	_data		|=	mask_lower_nibble;
    expanderWrite(  _data  );

	if( mode == CMD ){
		_data &= ~( RS ) ;		// RS = 0 -> CMD
        expanderWrite(  _data  );
	}
	if( mode == DATA ){
		_data |= (  RS ) ;		// RS = 1 -> DATA
        expanderWrite(  _data  );
	}
	pulseWrite( _data );
    
	delay_loop( 0x0100 );
    
	// SECOND HALF
	_data		=	0b00000101;
	_data		|=	swap_mask_Lower_nibble;
    expanderWrite(  _data  );
	
	if( mode == CMD ){
		_data &= ~(  RS ) ;		// RS = 0 -> CMD
        expanderWrite(  _data  );
	}
	if( mode == DATA ){
		_data |= (  RS ) ;		// RS = 1 -> DATA
        expanderWrite(  _data  );
	}
	pulseWrite( _data );
	
	delay_loop( 0x0100 );
    
}

void i2c_lcd_Init(void){
	I2C_Master_Init(100000);
    
	delay_loop( 0x000F );  
	sendByte( 0x28 , CMD );
	sendByte( 0x06 , CMD );
	sendByte( 0x02 , CMD );
	sendByte( 0x01 , CMD );
	sendByte( DISPLAY_OFF | CURSOR_SHOW , CMD );
	sendByte( DISPLAY_ON | CURSOR_SHOW , CMD );
	
}

void i2c_lcd_Print( char *str){
	while(*str ){
		sendByte( *str++ , DATA );
	}
}

void gotoXY( uint8_t X,  uint8_t Y ){
	
	uint8_t LcdType[ 4 ] = { 0 , 64 , 20 , 84 }; // 20 x 4 LCD
	
	#ifdef	lcdType20x4
		if( X >= 21) return;
		sendByte( 0x80 + LcdType[ Y - 1 ] + ( X - 1 ) , CMD );
	#endif

	#ifdef  lcdType16x2
		if( X >= 17 ) return;
		sendByte(  (uint8_t)( 0x80 + LcdType[ (uint8_t)( Y - 1) ] + ( X - 1 ) ) , CMD );
	#endif
}

void createChar(  uint8_t location, uint8_t charMap[] ) {
    
	uint8_t i ;
	location &= 0x07; // we only have 8 locations 0-7
	sendByte( (uint8_t )( 0x40 | ( location << 3 ) ), CMD );
	
	delay_loop( 0x0100 );
	
	for ( i = 0 ; i < 8 ; i++ ) {
		sendByte( charMap[i] , DATA );
	}
}
void getChar( uint8_t X, uint8_t Y, uint8_t location ){
	gotoXY( X, Y );
	sendByte( location , DATA);
}
void i2c_lcd_Print_Int( uint8_t X, uint8_t Y, int val, uint8_t numOfDigits ){
	
	char str[5] = {0,0,0,0,0};
	uint8_t i = 4 , j = 0;
	
	gotoXY( X , Y );
	//Handle negative integers
	if( val < 0 )
	{
		sendByte( '-' , DATA );     //Write Negative sign
		val *=  ( - 1 );            //convert to positive
	}
	else
	{
		sendByte( ' ', DATA );
	}

	while( val )
	{
		str[i] = (uint8_t)( val % 10);
		val = val / 10;
		i--;
	}

	if( numOfDigits ==  0 )    //-1
        while( str[j] == 0 ) j++;
	else
        j = (uint8_t)( sizeof(str) - numOfDigits );

	for( i = j; i < 5; i++ )        
	{
		sendByte( '0' + str[i]  , DATA );   // Adding '0', Converting to ASCII
	}
}
#endif	/* I2C_LCD_PIC_H */

