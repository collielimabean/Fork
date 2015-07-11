/***************************************twodrive.c******************************/

/*
2 WHEEL PWM MOTOR DRIVER
Sourya Roy/Nick Renold/Edward Jen - 05/2010 */

#include <18f4520.h> 
#include "fork.h"

#fuses HS,NOLVP,NOWDT,NOPROTECT 
#use delay(clock=20000000)

//motor polarities
#define M1p 1
#define M2p 1 //polarity for motor 2 corrected in H/W

//for motor PWM
#define M1PWM(num) set_pwm1_duty(num) //c2
#define M1REV(bit) OUTPUT_BIT(PIN_C5,bit) // c5
#define M2PWM(num) set_pwm2_duty(num) //c1
#define M2REV(bit) OUTPUT_BIT(PIN_C4,bit) // c4

void setup_motors() //initialize PWM
{
    //setup_timer_2(T2_DIV_BY_1, 255, 16);
    setup_timer_2(T2_DIV_BY_16, SPEED, 16);
    //setup_timer_2(T2_DIV_BY_4, 63, 16); 
    
    // 5Mz instruction cycle, pwm cycle every 4*64 cycles (19.5khz)
    // interrupt every 16th of that (not used)
    setup_ccp1(CCP_PWM);
    // PWM output on CCP1/RC2, pin 17

    setup_ccp2(CCP_PWM);
    // PWM output on CCP2/RC1, pin 16.  PWM2 can be moved to pin 36; see ServoSkeleton
}

void PWMSET(signed int pwm,int motor)
{ 
    int cycle;
    int16 rev_bit; 
    if(pwm<0)
    {
        rev_bit=1;
        cycle=pwm+SPEED;
        //cycle=pwm+64;
    }
    else
    {
        rev_bit=0; 
        cycle=pwm;
    }
    
    if(motor==1)
    { 
        M1PWM(cycle); 
        M1REV(rev_bit);
    }
    else if(motor==2)
    { 
        M2PWM(cycle);
        M2REV(rev_bit);
    }
}

void rotate(int i) 
{ 
    //rotate; i -> speed (-64 to +64)
    //rotates leftwise
    PWMSET(M1p*i,1);	//motor 1 moves in (+)ve direction
    PWMSET(-M2p*i,2);	//motor 2 moves in (-)ve direction
    //output_high(PIN_D1);
}

void forward(int i) 
{
    //move straight forward; i -> speed (-64 to +64)
    PWMSET(M1p*i,1);	//both motors move straight in (+)ve direction
    PWMSET(M2p*i,2);
    //output_high(PIN_D0);
}

void stop () 
{
    //stops all motion
    PWMSET(0,1);
    PWMSET(0,2); 
    //output_low(PIN_D0); 
    //output_low(PIN_D1);
}