// this file contains all the functions of PLL.
#include <math.h>

float abc_to_alpha(float a, float b, float c)
{
    float alpha;
    // alpha = (float)2 * (float)a / ((float) 3); // - 0.3333333 * b - 0.3333333 * c;
    alpha =  0.66666667 * a - 0.3333333 * b - 0.3333333 * c;
    // alpha = 2.0/3.0 * a - 1.0/3.0 * b - 1.0/3.0 * c;

    return alpha;
    // return a;
}

float abc_to_beta(float a, float b, float c)
{
    float beta;


    // beta = 0.5773503*b - 0.5773503*c;
    beta = 2.0/3.0*((sqrt(3.0)/2.0)*b - (sqrt(3.0)/2.0)*c);

    return beta;
}

float alphabeta_to_d(float alpha, float beta, float angle)
{
    float d;


    d = cosf(angle)*alpha + sinf(angle)*beta;

    return d;
}

float alphabeta_to_q(float alpha, float beta, float angle)
{
    float q;


    q = -sinf(angle)*alpha + cosf(angle)*beta;

    return q;
}

float dq_to_alpha(float d, float q, float angle)
{
    float alpha;

    alpha = cosf(angle)*d - sinf(angle)*q;

    return alpha;
}

float dq_to_beta(float d, float q, float angle)
{
    float beta;


    beta = sinf(angle)*d + cosf(angle)*q;

    return beta;
}

float cos_grid(float alpha, float beta)
{
    float cosGrid;

    cosGrid = alpha/ (sqrt(alpha*alpha + beta*beta));
    

    return cosGrid;
}

float sin_grid(float alpha, float beta)
{
    float sinGrid;


    sinGrid = beta/ (sqrt(alpha*alpha + beta*beta));

    return sinGrid;
}



float pi_regulator(float phaseError, float feedForward, float ki, float kp, float kPhi, float t, float Ts)
{
    // float anglePll, integral, omega, anglePllComp, angle;
    float anglePll, integral, omega;

    static float phaseError_old, integral_old, angle_old, omega_old;

    if (t < 0.002)
    {
        phaseError_old = 0;
        integral_old = 0;
        angle_old = 0;
        omega_old = 0;
    }
    integral = ki*(phaseError*0.5 + phaseError_old*0.5)*Ts + integral_old;
    omega = phaseError*kp + integral + feedForward; 

    anglePll = (omega*0.5 + omega_old*0.5)*Ts + angle_old; 
    // anglePllComp = anglePll - (omega - feedForward - phaseError*kp)*kPhi;

    phaseError_old = phaseError;
    integral_old = integral;
    angle_old = anglePll;
    omega_old = omega;
    

    return anglePll;
}

float pi_regulator_comp(float phaseError, float feedForward, float ki, float kp, float kPhi, float t, float Ts)
{
    // float anglePll, integral, omega, anglePllComp, angle;
    float anglePll, integral, omega, anglePllComp;
    static float phaseError_old, integral_old, angle_old, omega_old;

    if (t < 0.002)
    {
        phaseError_old = 0;
        integral_old = 0;
        angle_old = 0;
        omega_old = 0;
    }
    integral = ki*(phaseError*0.5 + phaseError_old*0.5)*Ts + integral_old;
    omega = phaseError*kp + integral + feedForward; 

    anglePll = (omega*0.5 + omega_old*0.5)*Ts + angle_old; 
    anglePllComp = anglePll - (omega - feedForward - phaseError*kp)*kPhi;

    phaseError_old = phaseError;
    integral_old = integral;
    angle_old = anglePll;
    omega_old = omega;

    

    return anglePllComp;
}


float phase_detector(float cosGrid, float sinGrid, float anglePllComp)
{
    float phaseError;

    phaseError = sinGrid*cosf(anglePllComp) - cosGrid*sinf(anglePllComp);
    

    return phaseError;
}
