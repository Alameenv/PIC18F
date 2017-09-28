/*
 * File:   main.c
 * Author: amn
 *
 * Created on August 27, 2017, 11:23 PM
 */


#include <xc.h>
#include "config.h"
#include "i2c_lcd_pic.h"

#define _XTAL_FREQ 10e6

uint16_t res1,res2;

void ADC_Init(void);
uint16_t readADC(uint8_t channel);
void ADC_ISR();

void interrupt high_priority high_isr(void) {
    if (PIR1bits.ADIF == 1)     // Is ADC interrupt ?
        ADC_ISR();              // YES
}

int main(void){
    TRISBbits.TRISB4 = 0;
    PORTBbits.RB4 = 0;
    
    i2c_lcd_Init();
    
    gotoXY(1,1);
    i2c_lcd_Print("ADC1:");
    gotoXY(1,2);
    i2c_lcd_Print("ADC2:");
    
    /*########################## ADC ##########################*/
    ADC_Init();
    
    /*########################## INTERRUPT ##########################*/
    PIR1bits.ADIF = 0;      // clear ADC flag
    PIE1bits.ADIE = 1;      // enable ADC interrupt
    INTCONbits.PEIE = 1;    // enable peripheral interrupt
    INTCONbits.GIE = 1;     // enable global interrupt
    ADCON0bits.GO = 1;      // start ADC
    
    while(1){
        PORTBbits.RB4 ^= 1;
        __delay_ms(500);
    }
    return 0;
}

void ADC_Init(void){
    TRISAbits.TRISA0 = 1;   //AN0
    TRISAbits.TRISA1 = 1;   //AN1
    TRISAbits.TRISA3 = 1;   //Vref+
    
    //Fosc/64, channel 0 , channel 1, A/D is ON
    ADCON0 =  0x81;     // ADCS1 , ADON 
    // right adjusted, Fosc / 64 , AN0 , AN1 = analog
    ADCON1 = 0xC5;      // ADFM , ADCS2 , PCFG2, PCFG0
}

//uint16_t and uint8_t are defined in i2c_lcd_pic.h library
uint16_t readADC(uint8_t channel){
    if(channel > 7)
        return 0;
    
    ADCON0 = (uint8_t)( ( 0b11000101 & ADCON0 ) | ( channel << 3 ) );
    __delay_ms(5);
    //start conversion
    ADCON0bits.GO = 1; 
    return (uint16_t)( (ADRESH << 8 ) + ADRESL );
}

void ADC_ISR() {
    __delay_ms(1);

    res1 = readADC( 0 );
    i2c_lcd_Print_Int(6,1, res1 ,4);
    __delay_ms(100);
    
    res2 = readADC( 1 );
    i2c_lcd_Print_Int(6,2, res2 ,4);
     __delay_ms(100);

    PIR1bits.ADIF = 0;
    ADCON0bits.GO = 1; //re-start the ADC
}