// this file contains all the functions of PLL.
#include <math.h>

float abc_to_alpha(float a, float b, float c)
{
    float alpha;

    alpha = 0.6666667 * a - 0.3333333 * b - 0.3333333 * c;

    return alpha;
}

float abc_to_beta(float a, float b, float c)
{
    float beta;


    beta = 0.5773503*b - 0.5773503*c;

    return beta;
}

float alphabeta_to_d(float alpha, float beta, float angle)
{
    float d;


    d = cos(angle)*alpha + sin(angle)*beta;

    return d;
}

float alphabeta_to_q(float alpha, float beta, float angle)
{
    float q;


    q = -sin(angle)*alpha + cos(angle)*beta;

    return q;
}

float dq_to_alpha(float d, float q, float angle)
{
    float alpha;

    alpha = cos(angle)*d - sin(angle)*q;

    return alpha;
}

float dq_to_beta(float d, float q, float angle)
{
    float beta;


    beta = sin(angle)*d + cos(angle)*q;

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
    float anglePll, integral, omega, anglePllComp, angle;
    static float phaseError_old, integral_old, angle_old, omega_old;

    if (t == 0)
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
    float anglePll, integral, omega, anglePllComp, angle;
    static float phaseError_old, integral_old, angle_old, omega_old;

    if (t == 0)
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

    phaseError = sinGrid*cos(anglePllComp) - cosGrid*sin(anglePllComp);
    

    return phaseError;
}


// checking whether phase_detector works
// #include <stdio.h>
// void main()
// {
//     float cosGrid, sinGrid, angle, diff, intermediate;

//     for(int i = 0; i< 360; i++)
//         {
//             // 
//             intermediate = 1.0f*i* 3.1416/180;
//             printf("intermediate at loop %i is %.5f \n", i, intermediate);

//             cosGrid = cos(intermediate);
//             printf("cosGrid      at loop %i is %.5f \n", i, cosGrid);

//             sinGrid = sin(intermediate);
//             printf("sinGrid      at loop %i is %.5f \n", i, sinGrid);       
            
//             angle = 0.0175 + 1.0f*i* 3.1416/180;    
//             printf("angle        at loop %i is %.5f \n", i, angle);

//             diff = phase_detector(cosGrid, sinGrid, angle);

//             printf("diff         at loop %i is %.5f \n \n", i, diff);
            
//         }
    
// }
