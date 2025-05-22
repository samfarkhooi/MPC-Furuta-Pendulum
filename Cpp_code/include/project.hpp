//#include <Arduino.h>

const int mode_button = 22;
const int led = 2; // ESP32 LED

const int ss = 5;
const int YPIN = 34;

extern int joystick;
extern volatile float joystick_input;

extern float volt;

extern float theta_1;
extern float theta_2;

extern float theta_1_dot;
extern float theta_2_dot;

extern volatile unsigned long delta_time;

//Functions
void light_switch();
void send_bytes();
float swing_up_voltage(float theta, float theta_dot);
float MPC(float theta_1, float theta_1_dot, float theta_2, float theta_2_dot);
void receive_thetas();
void compute_derivatives();
void compute_voltage();
void reset();
void setup_print();
void print_all();
