#include "msp.h"
#include "string.h"


int rowx,colx,ptr,state,counter,k,d,lock_state,lock_counter,key_pressed,passcode_complete;
int pass_code[4];
int *display;
int inNum[4]={16,16,16,16};
int blank[4]={16,16,16,16};
int idle[4]={0,0,0,0};
int lock[4]={16,15,0,12};
int open[4]={0,17,14,18};
int ld[4]={19,15,13,19};
int sseg_table[20]=
{
 0b11000000,0b11111001,0b10100100,0b10110000, //0,1,2,3
     0B10011001,0b10010010,0b010000010,0b11111000, //4,5,6,7
    0b10000000,0b10010000,0b10001000,0b10000011,//8,9,A,B
    0b11000110,0b10100001,0b10000110,0b11000111, //C,d,E,L
    0b11111111,0b10001100,0b10101011,0b10111111 //BLANK,P,n, '-'
}; // 15- 'L' ,17 - P, 18 -'n', 19-'-'

// which digit we're displaying
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
    P4->OUT=sseg_table[16];
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
    P8->OUT=p8_table[d];
    P4->OUT=sseg_table[display[d]];
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
                counter++;
                if(counter>200)
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
                   if(ptr==4)
                   {
                       passcode_complete=1;
                       ptr=0;
                   }
               }
               state=3;
               counter=0;
               break;
            }

           case(3): //key release debounce
            {
               input_key();
               counter++;
               if(counter>200)
               {
                   state=0;
                   key_pressed=9999;
               }
               break;
            }
        }
}

void lockbox_fn()
{
    switch(lock_state)
    {
       case(0): //Normal/Idle
       {
           display=idle;
           P5->OUT=BIT0;
           passcode_complete=0;
           keypad_function();
           if(key_pressed==10)
           {
               lock_state=1;
               lock_counter=0;
               P5->OUT=~BIT0;
           }
           if(0<=key_pressed&&key_pressed<10)
           {
               lock_state=2;
           }
           break;
       }

       case(1): //Solenoid on
       {
           display=open;
           if(lock_counter>20)
           {
               lock_state=0;
           }
           lock_counter++;
           break;
       }

       case(2): //Pre Lock
       {
           display=inNum;
           keypad_function();
           if(key_pressed==11&&passcode_complete==1)
           {
               lock_state=3;
               memcpy(pass_code, inNum, sizeof pass_code);
               memcpy(inNum, blank, sizeof inNum);
               passcode_complete=0;
               display=lock;
           }
           break;
       }

       case(3): //Lock
       {
           keypad_function();
           if(0<=key_pressed&&key_pressed<10)
           {
               display=inNum;
           }
           if(key_pressed==10&&passcode_complete==1&&memcmp(inNum,pass_code,4)==0)
           {
               lock_state=1;
               memcpy(pass_code, blank, sizeof pass_code);
               memcpy(inNum, blank, sizeof inNum);
               passcode_complete=0;
               display=open;
           }
           break;
       }
       case(4): //Lockdown
       {
           break;
       }
    }
}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |
            WDT_A_CTL_HOLD; // stop watchdog timer
    P9->DIR&=0b00111100; // input (keypad column)
    P8->DIR|=0b00111100; // select digit/keypad row
    P4->DIR|=0b11111111; // 7 segment display
    P5->DIR|=BIT0;
    ptr=state=d=lock_state=counter=0;
    key_pressed=9999;

   while(1)
   {
       lockbox_fn();
       show();
       d++;
       if(d==4) {d=0;}
       delay(300);
   }
}
