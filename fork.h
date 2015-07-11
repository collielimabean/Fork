/***************************************fork.h******************************/

/*
ROBOT CONFIGURATION HEADER FILE
Sourya Roy/Nick Renold/Edward Jen - 05/2010 */
#ifndef FORK_H
#define FORK_H

//GLOBAL SETTINGS
#define SPEED 64
#define LOSPEED 35

//ADC Sensor Subtractions #define R3_SUB 25 #define R2_SUB 25 #define R1_SUB 25 #define C_SUB 25
#define L1_SUB 20
#define L2_SUB 25
#define L3_SUB 40

//ADC Pin List
#define RIGHT3 0
#define RIGHT2 1

#define RIGHT1 2 //changed 
#define CENTER 3 //changed 
#define LEFT1 4

#define LEFT2 5
#define LEFT3 7

//Sensor LED Enable Pin 
#define SENSORS PIN_C0

//PWM

#define motor1PWMdefault 40 //default PWM value for motor 1 (CCP1 - Right) 
#define motor2PWMdefault 40 //default PWM value for motor 2 (CCP2 - Left) 
#define Upperlimit SPEED

#define Lowerlimit 0
#define tPos 0  //target position (DO NOT CHANGE)
#define DELAYTIME 5  //delay time for sensor measurements

//FUNCTIONS

//twodrive.c - PWM Motor Control
void setup_motors(); //initialize PWM
void PWMSET(signed int,int); //PWM sentup function (internal) 
void rotate(int); //rotate
void forward(int); //go forward or backward 
void stop (); //stop motors

//sensors.c - Sensor ADC Output

void setup_sensors(); // Setup ADC sensor input
int read_sensor(int); // Reads ADC and outputs value void calib_sensors(); //caliberate sensors
void calib_sensors_flash(); //calibrate pulsed sensors
void read_sensors(int *); // Reads sensors into array of 7 ints 
void read_pulsed_sensors(int *);

#endif