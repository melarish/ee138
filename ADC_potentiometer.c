#include "msp.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

uint16_t conversion_result;
int d =0;
int sseg_table[11]=
{
 0b11000000,0b11111001,0b10100100,0b10110000, //0,1,2,3
     0B10011001,0b10010010,0b010000010,0b11111000, //4,5,6,7
    0b10000000,0b10010000,0b11111111 //8,9,blank
};
int p8_table[4]=
{
 0b00011100,0b00101100,0b00110100,0b00111000
};
int display[4]= {10,10,10,10};

void delay(int millis)
{
    int i;
    for(i=0; i<millis;i++) {}
}
void show()
{
    int output=sseg_table[display[d]];
    if(d==0)
    {
        output &= ~(1UL<<7); //Set decimal point after 1st digit
    }
    P8->OUT=p8_table[d];
    P4->OUT=output;

}

void simple_clock_init(void)
{
           //Set power level for the desired clock frequency
    while (PCM->CTL1 & 0x00000100) {;} //wait for PCM to become not busy
    uint32_t key_bits = 0x695A0000; //0x695A is the key code
    uint32_t AM_LDO_VCORE1_bits = 0x00000100; //AMR Active Mode Request - 01b = AM_LDO_VCORE1
    PCM->CTL0 = key_bits | AM_LDO_VCORE1_bits; //unlock PCM register and set power mode
    while (PCM->CTL1 & 0x00000100); //wait for PCM to become not busy
    PCM->CTL0 &= 0x0000FFFF; //lock PCM register again

          //Flash read wait state number change
    FLCTL->BANK0_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15)); //reset bits
    FLCTL->BANK0_RDCTL |= BIT(12); //bit 12~15: wait state selection. 0001b=1 wait states
    FLCTL->BANK1_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15)); //reset bits
    FLCTL->BANK1_RDCTL |= BIT(12); //bit 12~15: wait state selection. 0001b = 1 wait states

     //Clock source: DCO, nominal DCO frequency: 48MHz
    CS->KEY = 0x695A; //unlock CS registers
    CS->CTL0 |= BIT(18) | BIT(16); //bit 16~18 DCORSEL frequency range select 101b = 48Mhz
    CS->CTL0 |= BIT(23); //bit 23 - DCOEN, enables DCO oscillator

     //clock module that uses DCO: MCLK
    CS->CTL1 &= ~(BIT(16) | BIT(17) | BIT(18)); //source divider = 1
    CS->CTL1 &= ~(BIT0 | BIT1 | BIT2 ); //reset all bits
    CS->CTL1 |= 0x3; //bits 0 to 2 - SELM, selects MCLK source. 011b = DCOCLK (by default)
    CS->CLKEN |= BIT1; //bit 1 - MCLK_EN, enable MCLK (by default)
    CS->KEY = 0x0; //lock CS registers

    while (!(CS->STAT & BIT(25))){} //while MCLK isn't steady
}

void init_adc()
{
    ADC14->CTL0 &= ~(1 << 1);  //ADCENC =0
    ADC14->CTL0 &= ~(1 << 29); // ADC14SHSx to ADC14SC bit (000b) -29 -27th bit
    ADC14->CTL0 &= ~(1 << 28); // ADC14SHSx to ADC14SC bit (000b) -29 -27th bit
    ADC14->CTL0 &= ~(1 << 27); // ADC14SHSx to ADC14SC bit (000b) -29 -27th bit
    ADC14->CTL0 |= (1 << 26);//  ADC14SHP to SAMPCON signal is sourced from the sampling timer (1b)
    ADC14->CTL0 &= ~(1 << 24); // ADC14DIVx to 0b011 clock divider = 4 (011b) - 24- 22 bit
    ADC14->CTL0 |= (1 << 23); // ADC14DIVx to 0b011 clock divider = 4 (011b) - 24- 22 bit
    ADC14->CTL0 |= (1 << 22);// ADC14DIVx to 0b011 clock divider = 4 (011b) - 24- 22 bit
    ADC14->CTL0 &= ~(1 << 21); //ADC14SSELx to 011b clock source select = MCLK -21-19 (011b)
    ADC14->CTL0 |= (1 << 20); //ADC14SSELx to 011b clock source select = MCLK -21-19 (011b)
    ADC14->CTL0 |= (1 << 19); //ADC14SSELx to 011b clock source select = MCLK -21-19 (011b)
    ADC14->CTL0 &= ~(1 << 11); // ADC14SHT0x to sample and hold time 0011b = 32 (11-8)
    ADC14->CTL0 &= ~(1 << 10); // ADC14SHT0x to sample and hold time 0011b = 32 (11-8)
    ADC14->CTL0 |= (1 << 9); // ADC14SHT0x to sample and hold time 0011b = 32 (11-8)
    ADC14->CTL0 |= (1 << 8); // ADC14SHT0x to sample and hold time 0011b = 32 (11-8)
    ADC14->CTL0 |= (1 << 4); //ADC14ON to 1b - 4th
    P5->SEL1 |= BIT1;
    P5->SEL0 |= BIT1;
    ADC14->MCTL[0] &= ~(0x1F); //reset to all zeros
    ADC14->MCTL[0] |= 1 << 2;
    ADC14->CTL0 |= (1 << 1); //ADCENC =1
}

void read_adc()
{
    ADC14->CTL0 |= (1 << 0); //ADC start Conversion=1 - 0th bit
    while (ADC14->CTL0 & (1<<16)){;} // ADCBUSY==1
    conversion_result = ADC14->MEM[0];
}
void convert_to_array()
{
    int result = (conversion_result*3.3*1000)/16384;
    display[0] = result/1000;
    result = result %1000;
    display[1] = result/100;
    result = result %100;
    display[2] = result/10;
    result = result %10;
    display[3] = result/1;

}

void main(void)
{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // disable watchdog timer
    P8->DIR|=0b00111100; // select digit/keypad row
    P4->DIR|=0b11111111; // 7 segment display
    P9->DIR|=BIT4;

    simple_clock_init();
    init_adc();
    while (1) //infinite loop
    {
           read_adc();    // Start conversion, polling, and read result.
           convert_to_array();
           show();
            d++;
            if(d==4) {d=0;}
            delay(300);

    }

}
