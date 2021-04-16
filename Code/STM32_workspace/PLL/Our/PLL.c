// this file contains all the functions of PLL.
#include <math.h>
#include "CONSTANTS.h"

// Constants



void abc_to_alphabeta(float a, float b, float c, float *alpha, float *beta)
{
	// constants
	const float two_thirds = 2.0f/3.0f;
	const float one_third = 1.0f/3.0f;
	const float sqrt3_div_2 = sqrt(3.0f)/2.0f;

	// calculation
    *alpha =  two_thirds*a - one_third*b - one_third*c;
    *beta = two_thirds * (sqrt3_div_2*b - sqrt3_div_2*c);
}

//float abc_to_alpha(float a, float b, float c)
//{
//    float alpha;
//    // alpha = (float)2 * (float)a / ((float) 3); // - 0.3333333 * b - 0.3333333 * c;
//    alpha =  0.66666667 * a - 0.3333333 * b - 0.3333333 * c;
//    // alpha = 2.0/3.0 * a - 1.0/3.0 * b - 1.0/3.0 * c;
//
//    return alpha;
//    // return a;
//}

//float abc_to_beta(float a, float b, float c)
//{
//    float beta;
//
//
//    // beta = 0.5773503*b - 0.5773503*c;
//    beta = 2.0/3.0*((sqrt(3.0)/2.0)*b - (sqrt(3.0)/2.0)*c);
//
//    return beta;
//}


void alphabeta_to_dq(float alpha, float beta, float angle, float *d, float *q)
{
	*d = cosf(angle)*alpha + sinf(angle)*beta;
	*q = -sinf(angle)*alpha + cosf(angle)*beta;
}

//float alphabeta_to_d(float alpha, float beta, float angle)
//{
//    float d;
//
//
//    d = cosf(angle)*alpha + sinf(angle)*beta;
//
//    return d;
//}
//
//float alphabeta_to_q(float alpha, float beta, float angle)
//{
//    float q;
//
//
//    q = -sinf(angle)*alpha + cosf(angle)*beta;
//
//    return q;
//}


void dq_to_alphabeta(float d, float q, float angle, float *alpha, float *beta)
{
	*alpha = cosf(angle)*d - sinf(angle)*q;
	*beta = sinf(angle)*d + cosf(angle)*q;
}

//float dq_to_alpha(float d, float q, float angle)
//{
//    float alpha;
//
//    alpha = cosf(angle)*d - sinf(angle)*q;
//
//    return alpha;
//}
//
//float dq_to_beta(float d, float q, float angle)
//{
//    float beta;
//
//
//    beta = sinf(angle)*d + cosf(angle)*q;
//
//    return beta;
//}


void cos_sin_grid(float alpha, float beta, float *cos_grid, float *sin_grid)
{
	static float sqrtCalc;

	sqrtCalc = pow((alpha*alpha + beta*beta), -0.5f);
	*cos_grid = alpha * sqrtCalc;
	*sin_grid = beta * sqrtCalc;
}

//float cos_grid(float alpha, float beta)
//{
//    return alpha / (sqrt(alpha*alpha + beta*beta));
//}
//
//float sin_grid(float alpha, float beta)
//{
//    return beta/ (sqrt(alpha*alpha + beta*beta));
//}


void pi_regulator(float phaseError, float feedForward, float ki, float kp, float kPhi, float Ts, float *anglePll, float *anglePllComp)
{
    static float phaseError_old, integral_old, angle_old, omega_old, integral, omega;;

    integral = ki*(phaseError*0.5 + phaseError_old*0.5)*Ts + integral_old;
    omega = phaseError*kp + integral + feedForward; 

    *anglePll = (omega*0.5 + omega_old*0.5)*Ts + angle_old;

    if (*anglePll > TWO_PI) {
    		*anglePll = *anglePll - TWO_PI;
    }

    *anglePllComp = *anglePll - (omega - feedForward - phaseError*kp)*kPhi;

    phaseError_old = phaseError;
    integral_old = integral;
    angle_old = *anglePll;
    omega_old = omega;
}

//float pi_regulator(float phaseError, float feedForward, float ki, float kp, float kPhi, float Ts)
//{
//    float anglePll, integral, omega;
//
//    static float phaseError_old, integral_old, angle_old, omega_old;
//
//    integral = ki*(phaseError*0.5 + phaseError_old*0.5)*Ts + integral_old;
//    omega = phaseError*kp + integral + feedForward;
//
//    anglePll = (omega*0.5 + omega_old*0.5)*Ts + angle_old;
//    // anglePllComp = anglePll - (omega - feedForward - phaseError*kp)*kPhi;
//
//    if (anglePll > TWO_PI) {
//    		anglePll = anglePll - TWO_PI;
//    }
//
//    phaseError_old = phaseError;
//    integral_old = integral;
//    angle_old = anglePll;
//    omega_old = omega;
//
//    return anglePll;
//}

//float pi_regulator_comp(float phaseError, float feedForward, float ki, float kp, float kPhi, float Ts)
//{
//    float anglePll, integral, omega, anglePllComp;
//    static float phaseError_old, integral_old, angle_old, omega_old;
//
//    integral = ki*(phaseError*0.5 + phaseError_old*0.5)*Ts + integral_old;
//    omega = phaseError*kp + integral + feedForward;
//
//    anglePll = (omega*0.5 + omega_old*0.5)*Ts + angle_old;
//
//    if (anglePll > TWO_PI) {
//        	anglePll = anglePll - TWO_PI;
//    }
//
//    anglePllComp = anglePll - (omega - feedForward - phaseError*kp)*kPhi;
//
//    phaseError_old = phaseError;
//    integral_old = integral;
//    angle_old = anglePll;
//    omega_old = omega;
//
//
//    return anglePllComp;
//}

float phase_detector(float cosGrid, float sinGrid, float anglePllComp)
{
    return sinGrid*cosf(anglePllComp) - cosGrid*sinf(anglePllComp);
}

//float phase_detector(float cosGrid, float sinGrid, float anglePllComp)
//{
//    float phaseError;
//
//    phaseError = sinGrid*cosf(anglePllComp) - cosGrid*sinf(anglePllComp);
//
//    return phaseError;
//}

