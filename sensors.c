/***************************************sensors.c******************************/
/*
SENSOR INPUT DRIVER
Sourya Roy/Nick Renold/Edward Jen - 05/2010 */

#include <18f4520.h>
#include "fork.h"

#fuses HS,NOLVP,NOWDT,NOPROTECT 
#use delay(clock=20000000)

//#DEVICE ADC=8
// set ADC to 10 bit accuracy, or it could be just 8

// Setup ADC sensor input


void setup_sensors() 
{
    setup_adc_ports(AN0_TO_AN7);
    // Enable analog inputs; choices run from just
    AN0, up to AN0_TO_AN11
    // the range selected has to start with AN0
    setup_adc(ADC_CLOCK_INTERNAL);
    //setup_adc_ports(AN0_TO_AN6|VSS_VDD); //setup_adc(ADC_CLOCK_DIV_2|ADC_TAD_MUL_0);
}

// Reads ADC and outputs value 
int read_sensor(int s) 
{
    int value;
    set_adc_channel(s);
    // there's only one ADC so select which input to connect to it; here pin AN0
    // wait 10uS for ADC to settle to a newly selected input
    delay_us(10);
    value = read_adc();
    //now you can read ADC as frequently as you like
    return value;
}


void calib_sensors() 
{ 
    //sensor auto calibration 
    int r3=0,r2=0,r1=0,c=0,l1=0,l2=0,l3=0,temp=0; 
    int16 counter;
    output_high(PIN_D6);

    for (counter=0; counter<20000; counter ++)
    {
        //rotate 
        rotate(25);
        
        //find max sensor readings 
        temp=read_sensor (RIGHT3); 
        if (temp>r3)
            r3=temp; 
        
        temp=read_sensor (RIGHT2);
        
        if (temp>r2) 
            r2=temp;
        
        temp=read_sensor (RIGHT1); 
        
        if (temp>r1)
            r1=temp; 
        
        temp=read_sensor (CENTER);
        if (temp>c)
            c=temp;
        
        temp=read_sensor (LEFT1);

        if (temp>l1)
            l1=temp;
        temp=read_sensor (LEFT2); 
        
        if (temp>l2)
            l2=temp; 
        
        temp=read_sensor (LEFT3); 
        
        if (temp>l3)
            l3=temp;
    }

    stop(); //stop motors

    //decrement max values by subtraction constant (SUB) r3-=R3_SUB;
    r2-=R2_SUB;
    r1-=R1_SUB; 
    c-=C_SUB; 
    l1-=L1_SUB;
    l2-=L2_SUB; 
    l3-=L3_SUB;

    //write to EEPROM
    write_eeprom(0,r3); //keep eeprom address numbers same as adc ports (0,1,2 etc) 
    write_eeprom(1,r2);
    write_eeprom(2,r1); 
    write_eeprom(3,c); 
    write_eeprom(4,l1); 
    write_eeprom(5,l2);
    write_eeprom(6,l3);

    output_low(PIN_D6);
}