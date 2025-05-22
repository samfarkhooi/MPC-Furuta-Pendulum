//This file handles the SPI communication with the QFLEX interface to give the commands to the pendulum.
//Author: Sam Farkhooi

#include <Arduino.h>
#include <project.hpp>
#include <SPI.h>
#include <cmath>

byte write_mask = 0b00011111;
byte encoder0[3] = {0, 0, 0}; // arm encoder
byte encoder1[3] = {0, 0, 0}; // pendulum encoder

byte mode = 1;

byte red_MSB = 0x03;
byte red_LSB = 0xE7;
byte green_MSB = 0;
byte green_LSB = 0;
byte blue_MSB = 0;
byte blue_LSB = 0;

byte input_MSB = 0;
byte input_LSB = 0;

float theta_1 = 0;
float theta_2 = -PI;
float old_theta_1 = 0;
float old_theta_2 = -PI+0.01;
float theta_1_dot = 0;
float theta_2_dot = 0;

float derivative = 0;
float alpha = 0.85;

bool motor_on = false;
/// @brief 
float volt = 0;

void light_switch(){    //toggles between Red and Green LED.
    blue_MSB = 0;
    blue_LSB = 0;
    red_MSB ^= 0x03;
    red_LSB ^= 0xE7;
    green_MSB ^= 0x03;
    green_LSB ^= 0xE7;
}

void compute_voltage(){
    if (fabs(theta_2) > (30*(2*PI)/360)){
        volt = swing_up_voltage(theta_2, theta_2_dot);
        blue_MSB = 0;
        blue_LSB = 0;
    }
    else{
        //unsigned long start = micros();
        volt = MPC(theta_1, theta_1_dot, theta_2, theta_2_dot);
        //unsigned long end = micros();
        //Serial.println((end-start));                //Uncomment these lines to take time
        blue_MSB ^= 0x03;
        blue_LSB ^= 0xE7;
    }
}

void compute_derivatives(){    //alpha acts as a smoothing factor
    derivative = (theta_1 - old_theta_1) * 1000.0 / delta_time;
    theta_1_dot = alpha*theta_1_dot + (1-alpha)*derivative;
    old_theta_1 = theta_1;

    derivative = (theta_2 - old_theta_2) * 1000.0 / delta_time;
        if (abs(theta_2 - old_theta_2) < PI){
            theta_2_dot = alpha*theta_2_dot + (1.0-alpha)*derivative;
            }    //Avoid situations when the pendulum jumps between theta and -theta   
    old_theta_2 = theta_2;
}


void receive_thetas(){
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    digitalWrite(ss, LOW);

    //QUBE expects a 16-byte transfer everytime
    //The order is roughly: mode, mask, color scheme (RGB), u (control input)
    SPI.transfer(mode);
    SPI.transfer(0);
    encoder0[2] = SPI.transfer(write_mask);          
    encoder0[1] = SPI.transfer(red_MSB);             
    encoder0[0] = SPI.transfer(red_LSB);            
    encoder1[2] = SPI.transfer(green_MSB);          
    encoder1[1] = SPI.transfer(green_LSB);          
    encoder1[0] = SPI.transfer(blue_MSB);           //Transfer zeros to fill out the 16 bit transfer       
    SPI.transfer(blue_LSB);            
    SPI.transfer(0);     
    SPI.transfer(0);    
    SPI.transfer(0);    
    SPI.transfer(0);    
    SPI.transfer(0);    
    SPI.transfer(0); 

    long Encoder0 = ((long)encoder0[2] << 16) | ((long)encoder0[1] << 8) | encoder0[0];

    if (Encoder0 & 0x00800000){
        Encoder0 = Encoder0 | 0xFF000000;}

    Encoder0 = Encoder0 % 2048;
    if (Encoder0 < 0){
        Encoder0 = 2048 + Encoder0;}
    
    theta_1 = (float)Encoder0 * (2.0 * M_PI / 2048);
    if (theta_1 > M_PI){
        theta_1 = theta_1 - 2*M_PI;}
    theta_1 = -theta_1;
    
    long Encoder1 = ((long)encoder1[2] << 16) | ((long)encoder1[1] << 8) | encoder1[0];

    if (Encoder1 & 0x00800000){
        Encoder1 = Encoder1 | 0xFF000000;}

    Encoder1 = Encoder1 % 2048;
    if (Encoder1 < 0){
        Encoder1 = 2048 + Encoder1;}

    //Convert to radians
    theta_2 = ((float)Encoder1 * (2.0 * M_PI / 2048) - M_PI);
}

void send_bytes(){

    volt = std::min(15.0f, std::max(-15.0f, volt));

    int input = (int)(volt * (1000.0 / 24.0));
    input = 0x8000 | input;
    input_MSB = (byte)(input >> 8);
    input_LSB = (byte)(input & 0x00FF);
    SPI.transfer(input_MSB); //control input u MSB and LSB comes last in the 16 byte transfer 
    SPI.transfer(input_LSB);

    digitalWrite(ss, HIGH);
    SPI.endTransaction();
}
