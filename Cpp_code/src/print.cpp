//Printing the states with a header making it compatible with a csv file, the serial monitor can be read by a Python script that creates a csv file
//Author: Sam Farkhooi

#include <Arduino.h>
#include <project.hpp>

volatile int print_variable = 0;

void setup_print(){
  receive_thetas();
  Serial.print("theta_1");
  Serial.print(",");
  Serial.print("theta_1_dot");
  Serial.print(",");
  Serial.print("theta_2");
  Serial.print(",");
  Serial.println("theta_2_dot");

  Serial.print(theta_1);
  Serial.print(",");
  Serial.print(theta_1_dot);
  Serial.print(",");
  Serial.print(theta_2);
  Serial.print(",");
  Serial.println(theta_2_dot);
  send_bytes();
}

void print_all(){
  if (print_variable % 6 == 0){
  Serial.print(theta_1);
  Serial.print(",");
  Serial.print(theta_1_dot);
  Serial.print(",");
  Serial.print(theta_2);
  Serial.print(",");
  Serial.println(theta_2_dot);}
  
  print_variable++;
}