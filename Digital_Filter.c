#include "msp.h"

int neg_flag=1;

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

     //clock module that uses DCO: SMCLK
//    CS->CTL1 &= ~(BIT(28) | BIT(30)); //SMCLK source divider = 4, 010b = f(SMCLK)/4
    CS->CTL1 |= BIT(29);
    CS->CTL1 |= BIT(28) | BIT(30); //SMCLK source divider = 128
    CS->CTL1 &= ~(BIT4 | BIT5 | BIT6 ); //reset all bits
    CS->CTL1 |= 0x3; //bits 4 to 6 - SELS, selects SMCLK source. 011b = DCOCLK (by default)
    CS->CLKEN |= BIT3; //bit 3 - SMCLK_EN, enable SMCLK (by default)
    CS->KEY = 0x0; //lock CS registers

    while (!(CS->STAT & BIT(25))){} //while MCLK isn't steady
}

void init_timer()
{
    //Timer_A1 uses SMCLK (low-speed subsystem master clock)
    TIMER_A1->CTL &= ~(BIT8 | BIT9 ); //reset bits
    TIMER_A1->CTL |= BIT9; //bit 8 to 9: timer clock source select. 10b = SMCLK
    TIMER_A1->CTL &= ~(BIT7 | BIT6 ); //bit 6 to 7: input divider. 00b = /1
    TIMER_A1->CTL &= ~(BIT4 | BIT5 ); //reset bits
    TIMER_A1->CTL |= BIT4; //bit 4 to 5: mode control. 01b = up mode: counter counts up to TAxCCR0
    TIMER_A1->CCTL[0] |= BIT4; // bit 4: CCR0 interrupt enable. 1b = interrupt enabled

    //Setting up capture/compare registers
//    TIMER_A1->CCR[0] = 23437; //TA1CCR0 is the period of the timer
    TIMER_A1->CCR[0] = 234; //TA1CCR0 is the period of the timer
}

void init_interrupt()
{
    NVIC->ISER[0] = 1<<((TA1_0_IRQn) & 31); //TA1_0_IRQn is the interrupt number for Timer A1
//    NVIC->IP[10] = 0x0; // set Port1 at highest priority
    NVIC->IP[10] |= BIT5; // set priority
}

void TA1_0_IRQHandler(void)
{
    TIMER_A1->CCTL[0] &= ~BIT0;     // reset the interrupt flag of Timer A1
    if(neg_flag==1)
    {
        P5->OUT=~BIT0;
    }
    if(neg_flag==-1)
    {
        P5->OUT=BIT0;
    }
    neg_flag*=-1;
}

void delay(int millis)
{
    int i;
    for(i=0; i<millis;i++) {}
}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    simple_clock_init();
    init_timer();
    init_interrupt();
    P5->DIR|=BIT0; // init GPIO led for testing
    P5->OUT=BIT0;
    while (1) //infinite loop
        {
            delay(300);
        }
}
