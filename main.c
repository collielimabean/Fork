/***************************************main.c******************************/

/*
 * MAIN CONTROL PROGRAM
 * Sourya Roy/Nick Renold/Edward Jen - 05/2010 
 */
#include <18f4520.h> 
#include "fork.h"

#fuses HS,NOLVP,NOWDT,NOPROTECT,NOWRTD 
#use delay(clock=40000000)

//sensors
int r3_in,r2_in,r1_in,c_in,l1_in,l2_in,l3_in; int1 r3,r2,r1,c,l1,l2,l3;

//PID
float mPos = 0;  //current position

//bunch of random variables needed to carry out PID calculation LINE SENSOR PART float Prop = 0;

float Temp = 0; 
float Integral = 0;
float Diff = 0; 
float Difflast = 0; 
float Control = 0;
float Control_prev = 0; 
float Rate = 0;
float Deriv = 0;

float Kp = 8.0; //Proportionality Constant (P) 
float Kd = 1.5; //Integration Constant (I) 
float Ki = 0.0; //Derivative Constant (D)

int isFirstLoop = 1;

int RIGHT3_TH=0,RIGHT2_TH=0,RIGHT1_TH=0,CENTER_TH=0,LEFT1_TH=0,LEFT2_TH=0,LEFT3_TH=0;

int flag=0; //flag for bump-rotate

void main()
{
    //initialize sensors 
    setup_sensors();
    
    //initialize motors 
    setup_motors(); 
    stop();

    //switch on sensors 
    output_high(SENSORS);

    //[OPTION] calibrate sensors 
    if (input(PIN_B0)==1) 
    {
        output_high(PIN_D7);
        delay_ms(1000); 
        calib_sensors();
        output_d(255); //success
        while(1);
    }

    output_d(255);

    //load sensor data 
    RIGHT3_TH=read_eeprom(0);
    RIGHT2_TH=read_eeprom(1); 
    RIGHT1_TH=read_eeprom(2); 
    CENTER_TH=read_eeprom(3); 
    LEFT1_TH=read_eeprom(4); 
    LEFT2_TH=read_eeprom(5); 
    LEFT3_TH=read_eeprom(6);

    PWMSET(SPEED-4,1); 
    PWMSET(SPEED,2); 
    delay_ms(2000); 
    output_d(0); 
    output_high(PIN_D7);

    //[OPTION] initialize slo-mo 
    if (input(PIN_B1)==1) {
        output_low(PIN_D7);
    }

    while(1)
    {
        r3_in=read_sensor(RIGHT3); 
        r2_in=read_sensor(RIGHT2);
        r1_in=read_sensor(RIGHT1); 
        c_in=read_sensor(CENTER); 
        l1_in=read_sensor(LEFT1);
        l2_in=read_sensor(LEFT2); 
        l3_in=read_sensor(LEFT3);

        //DEBUG
        if (r3_in>RIGHT3_TH) { 
            output_high(PIN_D0); r3=1;
        }
        else { 
            output_low(PIN_D0); r3=0;
        }

        if (r2_in>RIGHT2_TH) { 
            output_high(PIN_D1);
            r2=1;
        }
        else 
        { 
            output_low(PIN_D1); r2=0;
        }

        if (r1_in>RIGHT1_TH) { 
            output_high(PIN_D2); r1=1;
        }
        else { 
            output_low(PIN_D2);
            r1=0;
        }

        if (c_in>CENTER_TH) {
            output_high(PIN_D3); 
            c=1;
        }
        else { 
            output_low(PIN_D3);
            c=0;
        }

        if (l1_in>LEFT1_TH) { 
            output_high(PIN_D4);
            l1=1;
        }
        else { 
            output_low(PIN_D4); 
            l1=0;
        }

        if (l2_in>LEFT2_TH) {
            output_high(PIN_D5); l2=1;
        }
        else { 
            output_low(PIN_D5);
            l2=0;
        }

        if (l3_in>LEFT3_TH) { 
            output_high(PIN_D6);
            l3=1;
        }
        else { 
            output_low(PIN_D6);
            l3=0;
        }

        //NEW PID //0000001 = 6 //0000011 = 5

        //0000010 = 4 T U R N //0000110 = 3 //0000100 = 2 L E F T //0001100 = 1

        //0001000 = 0 ----------
        //0011000 = -1 //0010000 = -2 T U R N //0110000 = -3
        //0100000 = -4 R I G H T //1100000 = -5 //1000000 = -6
        //0000000 = 7 or -7 depending on previous values

        if (r1==0 && r2==0 && r3==0 && c==0 && l1==0 && l2==0 && l3==1)
            mPos=7;
        else if (r1==0 && r2==0 && r3==0 && c==0 && l1==0 && l2==1 && l3==1)
            mPos=6;
        else if (r1==0 && r2==0 && r3==0 && c==0 && l1==0 && l2==1 && l3==0) 
            mPos=5;
        else if (r1==0 && r2==0 && r3==0 && c==0 && l1==1 && l2==1 && l3==0) 
            mPos=3;
        else if (r1==0 && r2==0 && r3==0 && c==0 && l1==1 && l2==0 && l3==0)
            mPos=2;
        else if (r1==0 && r2==0 && r3==0 && c==1 && l1==1 && l2==0 && l3==0)
            mPos=1;
        else if (r1==0 && r2==0 && r3==0 && c==1 && l1==0 && l2==0 && l3==0) 
            mPos=0;
        else if (r1==0 && r2==0 && r3==1 && c==1 && l1==0 && l2==0 && l3==0) 
            mPos=-1;
        else if (r1==0 && r2==0 && r3==1 && c==0 && l1==0 && l2==0 && l3==0) 
            mPos=-2;
        else if (r1==0 && r2==1 && r3==1 && c==0 && l1==0 && l2==0 && l3==0) 
            mPos=-3;
        else if (r1==0 && r2==1 && r3==0 && c==0 && l1==0 && l2==0 && l3==0)
            mPos=-4;
        else if (r1==1 && r2==1 && r3==0 && c==0 && l1==0 && l2==0 && l3==0)
            mPos=-5;
        else if (r1==1 && r2==0 && r3==0 && c==0 && l1==0 && l2==0 && l3==0) 
            mPos=-6;
        else if (mPos >= 0.0) 
            mPos=8;
        else if (mPos <= 0.0) 
            mPos=-8;

        //[OPTION] Slo-Mo BANG BANG Approach
        if (input(PIN_B1) == 1) 
        {
            if(c==1) //go straight 
                forward(LOSPEED);
            else if ((r1+r2+r3)>(l1+l2+l3)) { //turn right 
                PWMSET(-SPEED,1);
            }
            else { //turn left 
                PWMSET(SPEED,1);
            }
        }

        Diff = tPos - mPos; //P 
        Prop = Diff * Kp;

        Integral += Diff ; // I 
        Integral *= Ki;

        Rate = Diff - Difflast; // D 
        Deriv = Rate * Kd;

        Difflast = Diff;

        Control = Prop + Deriv + Integral;

        if	(isFirstLoop == 1){ 
            isFirstLoop = 0;
            Control_prev = Control + 1; //dummy value so that Control isn't equal to Control_prev for FIRST RUN
        }

        if (input(PIN_B5) == 1){ //bump sensor is activated; both sensors are depressed -> wait three seconds and back up
            PWMSET(SPEED,1);
            PWMSET(SPEED,2); 
            delay_ms(1500);
            if (input(PIN_B5) == 1){ 
                PWMSET(-SPEED,1); 
                PWMSET(-SPEED,2);
                delay_ms(1500); 
                if (flag==0) {
                    rotate(SPEED); 
                    delay_ms (1500); 
                    stop(); 
                    forward(SPEED); 
                    delay_ms(1000); 
                    mPos = 0; 
                    flag=1;
                }
                else { 
                    rotate(SPEED); 
                    delay_ms (1000); 
                    stop();
                    forward(SPEED); 
                    delay_ms(1500);
                    mPos = 0;
                    flag=0;
                }
            }
        }

        if (Control != Control_prev){ //change PWM ONLY if needed, else skip
            if (input(PIN_B1) != 1) { //follow PID
                if (Control < 0){ //Turn left 
                    PWMSET(motor2PWMdefault + Control,2); 
                    PWMSET(motor1PWMdefault,1);
                }
                else {//Turn right 
                    PWMSET(motor1PWMdefault - Control,1); 
                    PWMSET(motor2PWMdefault,2);
                }
            }

            Control = Control_prev;
            if (input(PIN_B1) == 1) {
                if (Control < 0){ //Turn left 
                    PWMSET(30 + Control,2); 
                    PWMSET(30,1);
                }
                else {//Turn right
                    PWMSET(30 - Control,1);
                    PWMSET(30,2);
                }
            }   
        }
    //delay_ms(DELAYTIME);
    }
}
