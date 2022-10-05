#include "msp.h"

int counter,d,neg_flag=0;

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

void init_interrupt()
{
    P1->IES |= BIT1;//High to low transition for bit1
    P1->IES |= BIT4;//High to low transition for bit4
    P1->IFG=0;//clear all the interrupt flags
    P1->IE |= BIT1;// Enable interrupt for P1.1
    P1->IE |= BIT4;// Enable interrupt for P1.4
    NVIC->ISER[1] = 1<<((PORT1_IRQn) & 0x1F); //Enable NVIC interrupt
    NVIC->IP[35] = 0x0; // set Port1 at highest priority
    P1->REN |=BIT1;
    P1->REN |=BIT4;
}

void PORT1_IRQHandler(void)
{
    int temp = P1->IV;     // read the interrupt v. reg of Port1 and reset the flag
    if (temp == 0x04 ) // check if P1.1 is the interrupt source (see Fig. 6.5)
    {
       counter++; //increment counter
    }
    if (temp == 0x0A) // check if P1.4 is the interrupt source (see Fig. 6.5)
    {
       counter--; //decrement counter
    }
}

void delay(int millis)
{
    int i;
    for(i=0; i<millis;i++) {}
}

void show()
{
    int output=sseg_table[display[d]];
    P8->OUT=p8_table[d];
    P4->OUT=output;
    if(neg_flag==1)
    {
        P5->OUT=~BIT0;
    }
    if(neg_flag==0)
    {
        P5->OUT=BIT0;
    }
}

void init_GPIO()
{
    P8->DIR|=0b00111100; // select digit/keypad row
    P4->DIR|=0b11111111; // 7 segment display
    P5->DIR|=BIT0;
}

void convert_to_array()
{
    int result = counter;
    if(counter<0)
    {
       neg_flag=1;
       result=-counter;
    }
    if (counter>=0)
    {
        neg_flag=0;
    }
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
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	simple_clock_init();
	init_GPIO();
	init_interrupt();
	while (1) //infinite loop
	    {
            convert_to_array();
            show();
            d++;
            if(d==4) {d=0;}
            delay(300);
	    }
}

