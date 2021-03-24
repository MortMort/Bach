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

    // static float phaseError_old, integral_old, angle_old, omega_old;
    static float phaseError_old=0;
    static float integral_old=0;
    static float angle_old=0;
    static float omega_old=0;
    if (t < 0.0002)
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
    // static float phaseError_old, integral_old, angle_old, omega_old;
    static float phaseError_old=0;
    static float integral_old=0;
    static float angle_old=0;
    static float omega_old=0;

    if (t < 0.0002)
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

// // // '-------------------------------------------------------------------------------------------------------------------------------------- to be deleted later..
// float maf1(float nextSample, float t) //float t is not supposed to be included in stm
// {
//   static float sum, out_old;
//   static int pos;
//   int len = 200;
//   static float sampleArr[200] = {0}; // tried parsing len instead of 200 but didnt work
//   if (t == 0)
//   {
//         sum = 0;
//         out_old = 0;
//         pos = 0;
//         for(int i = 0; i < len; i++)
//         {
//             sampleArr[i] = 0;
//         }	
//   }

//   sum = sum - sampleArr[pos] + nextSample;
//   sampleArr[pos] = nextSample;
//   pos++;
//   if(pos>=len)
//   {
//       pos = 0; 
//   }
// //   avg = sum / len;
// //   return avg;
// //   return (float) sum / len;
//   return sum / (float) len; 
// }

// float maf2(float nextSample, float t) //float t is not supposed to be included in stm
// {
//   static float sum, out_old;
//   static int pos;
//   int len = 200;
//   static float sampleArr[200] = {0}; // tried parsing len instead of 200 but didnt work
  

//   if (t == 0)
//   {
//         sum = 0;
//         out_old = 0;
//         pos = 0;
//         for(int i = 0; i < len; i++)
//         {
//             sampleArr[i] = 0;
//         }	
//   }

//   sum = sum - sampleArr[pos] + nextSample;
//   sampleArr[pos] = nextSample;
//   pos++;
//   if(pos>=len)
//   {
//       pos = 0; 
//   }
// //   return (float) sum / len;
//   return sum / (float) len;
// }
// // '-------------------------------------------------------------------------------------------------------------------------------------- to be deleted later end


// float pll(float a, float b, float c, float feedForward, float ki, float kp, float kPhi, float t, float Ts, float angle)
// {
//     float alpha, beta, d, q, d_maf, q_maf, alpha1, beta1, cosGrid, sinGrid, phaseError, anglePllComp, anglePll;
//     // static float phaseError_old, anglePllComp_old;
//     static float anglePllComp_old;
    
//     if (t == 0)
//     {
//             anglePllComp_old = 0;
//     }

//     alpha = abc_to_alpha(a, b, c);
//     beta = abc_to_beta(a, b, c);
//     d = alphabeta_to_d(alpha, beta, angle);
//     q = alphabeta_to_q(alpha, beta, angle);
//     d_maf = maf1(d, t); 
//     q_maf = maf1(q, t); 
//     alpha1 = dq_to_alpha(d_maf, q_maf, angle);
//     beta1 = dq_to_beta(d_maf, q_maf, angle);
//     cosGrid = cos_grid(alpha1, beta1);
//     sinGrid = sin_grid(alpha1, beta1);
//     phaseError = phase_detector(cosGrid, sinGrid, anglePllComp_old);
//     anglePll = pi_regulator(phaseError, feedForward, ki, kp, kPhi, t, Ts);
//     anglePllComp = pi_regulator_comp(phaseError, feedForward, ki, kp, kPhi, t, Ts);



//     anglePllComp_old = anglePllComp;

//     return anglePll;
// }

// making it work with floats
// #include <stdio.h>
// #include <math.h>
// void main()
// {
//     float a, b, c, phi, alpha;

//     for(int i = 0; i< 360; i++)
//         {
//             // 
//             phi = 1.0*i* 3.1416/180;
//             a = sin(phi);
//             b = sin(phi-2*3.1416/3);
//             c = sin(phi+2*3.1416/3);

//             alpha = abc_to_alpha(a, b, c);
//             printf("alpha at loop %i is %.5f \n", i, alpha);

//         }
    
// }


// // checking whether phase_detector works
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
