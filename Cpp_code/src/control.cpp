//This file contains the energy-based swing up and the stabilization with both MPC and LQR. MPC is run by default (see communication.cpp)
//Author: Sam Farkhooi

#include <math.h>
#include <Arduino.h>
#include <project.hpp>
#include "matrices.hpp"
#include <time.h>

extern "C" {
    #include <daqp.h>
    #include <api.h>
    #include <auxiliary.h>
    #include <types.h>
}

// Arm
float m1 = 0.095; // Mass (kg)
float l1 = 0.085; // Total length (m)

float Rm = 8.4;   // Resistance
float kt = 0.042; // Current-torque (N-m/A)

// Pendulum
float m2 = 0.024;     // Mass (kg)
float l2 = 0.129;     // Total length (m)
float J2 = 3.3282e-5; // Moment of inertia about pivot (kg-m^2)
float g = 9.82;       // Gravity Constant

float u_to_volt = m1 * l1 * Rm / kt;

int n = 6;           // Prediction horizon
int m = (n+1)*6;    // Number of constraints
int ms = 0;         // Number of simple bounds

float energy(float theta, float theta_dot){
    return (l2*(theta_dot*theta_dot)/(6*g) + cos(theta)-1);
}

float swing_up_voltage(float theta, float theta_dot){
    float E = energy(theta, theta_dot);

    float sign = ((cos(theta)*theta_dot > 0) - (cos(theta)*theta_dot < 0));   
    float u = 1.0 * E * sign;

    float voltage = u * u_to_volt; 
    return (voltage);
}

void matmul(const double* A, const double* B, double* C, int m, int n, int p) {
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < p; ++j) {
            C[i * p + j] = 0;
            for (int k = 0; k < n; ++k) {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
  }
  
void subtract_arrays(const double* b, const double* W, double* T, int n) {
    for (int i = 0; i < n; ++i) {
        T[i] = b[i] - W[i];
    }
  }

float LQR(float theta_1, float theta_1_dot, float theta_2, float theta_2_dot){
    float voltage = -1.39509209*(theta_1-joystick_input*3) - 1.2058144*theta_1_dot + 31.23476634*theta_2 + 2.73290132*theta_2_dot;
    return (voltage);
}

//The main control function of interest. 
float MPC(float theta_1, float theta_1_dot, float theta_2, float theta_2_dot){
    double state[4] = {theta_1, theta_1_dot, theta_2, theta_2_dot};
    //Joystick changes the reference point for the arm
    double state_w_reference[4] = {theta_1 - joystick_input*3, theta_1_dot, theta_2, theta_2_dot};

    double f[n];
    matmul(state_w_reference, f_theta, f, 1, 4, n);
    double W_theta[m];
    matmul(W, state, W_theta, m, 4, 1);
    double bupper[m];
    subtract_arrays(b, W_theta, bupper, m);

    int sense[m];
    for (int i = 0; i < m; ++i) {
    if (i % 6 == 4 || i % 6 == 5){
        sense[i] = 0;   //Hard constraint for u
    } else {
        sense[i] = 8;   //Soft constraints for the rest
    } }

    double blower[m];
    std::fill(blower, blower + m, -DAQP_INF);       //Only upper limits

    DAQPProblem qp = {n, m, ms, H, f, A, bupper, blower, sense};

    // Define and initialize result structure
    DAQPResult result;
    double u[n], lam[m];

    result.x = u;   // Primal variable
    result.lam = lam; // Dual variable

    // Solve the QP
    daqp_quadprog(&result, &qp, NULL);       //The NULL indicates default settings

    //Serial.println(result.exitflag);     //1 means solution is optimal, 2 means optimal with soft constraint violated, negative means it failed.

    float voltage = u[0];
    return (voltage);
}