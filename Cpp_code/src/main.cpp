//This file runs the process of reading angles, computing derivatives, computing control input, and sending the input to the pendulum.
//Author: Sam Farkhooi

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <project.hpp>
#include <cmath>

unsigned long lastPressTime = 0;
int joystick = 1;
volatile float joystick_input = 0;
float joystick_offset = 0;

int old_time = 0;
volatile unsigned long delta_time = 0;
volatile unsigned long current_time = 0;
int print_states = 0;

int time_step = 4;

void setup() {
  pinMode(ss, OUTPUT);
  SPI.begin();
  Serial.begin(115200);
  reset();
  
  if (print_states){setup_print();}    //Printing is for making the plots by tracking the states in the serial monitor.
  pinMode(mode_button, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  joystick_offset = (analogRead(YPIN) >> 4);
}

void loop() {
  current_time = millis();
  bool pressed = (digitalRead(mode_button) == LOW);
  if (pressed && (current_time - lastPressTime) > 1000) {   //debouncing
    joystick = 1 - joystick;
    light_switch();
    lastPressTime = current_time;  //Update last press time
  }

  if (current_time - old_time > time_step){
    delta_time = (current_time - old_time);
    old_time = old_time + time_step;
    
    joystick_input = ((analogRead(YPIN) >> 4)-joystick_offset)/128.0;
    joystick_input = std::min(0.8f, std::max(-0.8f, static_cast<float>(joystick_input)));     //Clamp it becuase it is not symmetrical.
    
    receive_thetas();      //Get data (angles)

    if (joystick){
      volt=-2.0*joystick_input;  //Control by joystick
  }
    else{
      //Compute derivatives (angular velocities)
      compute_derivatives();

      if (print_states){print_all();}

      //Compute swing up or MPC action  
      compute_voltage();  
    }   

    //Send input to motor
    send_bytes();
  }
}

