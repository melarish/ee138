#include "msp.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

int rowx,colx,ptr, d,state,debounce_counter,k,key_pressed;
int *display;
int inNum[4]={0,0,0,0};
int Tx_length = 4; // we can only see 4 numbers on the display
int Rx_length=4;
int Tx_cnt,Rx_cnt=0;
char inmsg;
int sseg_table[11]=
{
 0b11000000,0b11111001,0b10100100,0b10110000, //0,1,2,3
     0B10011001,0b10010010,0b010000010,0b11111000, //4,5,6,7
    0b10000000,0b10010000,0b11111111 //8,9,blank
};// which digit we're displaying
int p8_table[4]=
{
 0b00011100,0b00101100,0b00110100,0b00111000
};
int bits[4]=
{
 BIT0,BIT1,BIT2,BIT3
};
int keypad_table[4][4]=
{
    {10,3,2,1},
    {11,6,5,4},
    {12,9,8,7},
    {13,15,0,14}
};

void delay(int millis)
{
    int i;
    for(i=0; i<millis;i++) {}
}
void input_key()
{
    P4->OUT=sseg_table[10];
    int key;
    for (k=0;k<4;k++)
    {
        P8->OUT=p8_table[k];
        for(key=0;key<4;key++)
        {
            if(P9->IN&bits[key])
            {
                rowx=k;
                colx=key;
                if(state==0)
                {
                    state=1;
                }
            }
        }
    }
}
void show()
{
    int output=sseg_table[display[d]];
    P8->OUT=p8_table[d];
    P4->OUT=output;
}

void keypad_function()
{
    switch(state)
        {
           case(0): //idle
            {
               input_key();
               break;
            }

           case(1): //keypress debounce
            {
                input_key();
                debounce_counter++;
                if(debounce_counter>400)
                {
                    state=2;
                }
                break;
            }

           case(2): //process input
            {
               key_pressed=keypad_table[rowx][colx];
               if(0<=key_pressed&&key_pressed<10) // save only if numeric
               {
                   inNum[ptr]=key_pressed;
                   ptr++;
                   if(ptr==4) {ptr=0;}
               }
               if(key_pressed==10) // A key pressed
               {
                   EUSCI_A2->IE |= EUSCI_A_IE_TXIE; // enable transmit interrupt from UART
                   EUSCI_A2->IFG |= 0b0010; // set Transmit interrupt pending flag.
                   //send the message
               }
               state=3;
               debounce_counter=0;
               break;
            }

           case(3): //key release debounce
            {
               input_key();
               debounce_counter++;
               if(debounce_counter>400)
               {
                   state=0;
                   key_pressed=9999;
               }
               break;
            }
        }
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

     //Clock source: DCO, nominal DCO frequency: 12MHz
    CS->KEY = 0x695A; //unlock CS registers
    CS->CTL0 |= BIT(17) | BIT(16); //bit 16~18 DCORSEL frequency range select 011b = Nominal DCO Frequency (MHz): 12;
    CS->CTL0 |= BIT(23); //bit 23 - DCOEN, enables DCO oscillator

     //clock module that uses DCO: MCLK
    CS->CTL1 &= ~(BIT(16) | BIT(17) | BIT(18)); //source divider = 1
    CS->CTL1 &= ~(BIT0 | BIT1 | BIT2 ); //reset all bits
    CS->CTL1 |= 0x3; //bits 0 to 2 - SELM, selects MCLK source. 011b = DCOCLK (by default)
    CS->CLKEN |= BIT1; //bit 1 - MCLK_EN, enable MCLK (by default)

    //clock module that uses DCO: SMCLK
    CS->CTL1 |= BIT(29);
    CS->CTL1 &= ~(BIT(28) | BIT(29) | BIT(30)); //SMCLK source divider = 1, 000b = f(SMCLK)/1
    CS->CTL1 &= ~(BIT4 | BIT5 | BIT6 ); //reset all bits
    CS->CTL1 |= CS_CTL1_SELS__DCOCLK; //bits 4 to 6 - SELS, selects SMCLK source. 011b = DCOCLK (by default)
    CS->CLKEN |= BIT3; //bit 3 - SMCLK_EN, enable SMCLK (by default)

    CS->KEY = 0x0; //lock CS registers

    while (!(CS->STAT & BIT(25))){} //while MCLK isn't steady

}

void init_gpio()
{
    P9->DIR&=0b00111100; // input (keypad column)
    P8->DIR|=0b00111100; // select digit/keypad row
    P4->DIR|=0b11111111; // 7 segment display
    P3->SEL0 |= (BIT2|BIT3);
    P3->SEL1 &= ~(BIT2|BIT3);
}

void init_interrupt()
{
    NVIC->ISER[0] = 1<<((EUSCIA2_IRQn) & 31); // EUSCIA2_IRQn is the interrupt number for UART
    NVIC->IP[18] = 0xF; // set priority
    EUSCI_A2->IE = 0x0000; // disable interrupt
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE; // enable receive interrupt from UART
    EUSCI_A2->IFG = 0;
}
void EUSCIA2_IRQHandler()
{
    uint8_t iv_val = EUSCI_A2->IV;
    if (iv_val & 0x4) //if transmit buffer empty
     {
        if(Tx_cnt < Tx_length) // if more to send.
         {
            EUSCI_A2->TXBUF = inNum[Tx_cnt];
            Tx_cnt++;
         }
        else // if done with sending
         {
            EUSCI_A2->IE &= ~0x0002; // disable TX interrupt
            Tx_cnt = 0;
         }
     }

//        Whenever a character is received from the PC,
//        it is displayed on the 7-segment display sequentially.  Only the last 4 characters will be shown on the 7-
//        segment display.   The transmit and receive functions can happen at the same time.
    if (iv_val & 0x2) // if receive buffer full
    {
         if(Rx_cnt < Rx_length) // if more to receive.
         {
             inmsg = EUSCI_A2->RXBUF; // save to inmsg char variable
             int msg = inmsg - '0'; // convert char to int
             // if numeric, add to inNum array
             if(msg>=0 && msg <10)
             {
                 inNum[ptr]=msg;
                 ptr++;
                 if(ptr==4) {ptr=0;}
             }
             Rx_cnt++;
         }
        else // if done with receiving
         {
             Rx_cnt = 0;
         }
     }
}

void UART_init()
{
   EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // enable reset
   EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK; // use smclk for this peripheral
   EUSCI_A2->MCTLW |= EUSCI_A_MCTLW_OS16; // over sampling mode (n = freq/Baud rate = 12Meg/9600 = 1250 > 16)
   EUSCI_A2->BRW =78;  // 0-15 bit n/16=78
   EUSCI_A2->MCTLW|=BIT4|BIT5; // UCBRFx=2 (remainder from the above calculation)
   EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;// disable reset
}
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // disable watchdog timer

    simple_clock_init();
    init_gpio();
    UART_init();
    init_interrupt();
    ptr=state=d=debounce_counter=0;
    key_pressed=9999;
    while (1) //infinite loop
    {
        display=inNum;
        keypad_function();
        show();
        d++;
        if(d==4) {d=0;}
        delay(300);
    }
}
