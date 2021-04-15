// this file contains prototypes of functions of PLL.

// OLD
//float abc_to_alpha(float a, float b, float c);
//float abc_to_beta(float a, float b, float c);
//float alphabeta_to_d(float alpha, float beta, float angle);
//float alphabeta_to_q(float alpha, float beta, float angle);
//float dq_to_alpha(float d, float q, float angle);
//float dq_to_beta(float d, float q, float angle);
//float pi_regulator(float phaseError, float feedForward, float ki, float kp, float kPhi, float Ts);
//float pi_regulator_comp(float phaseError, float feedForward, float ki, float kp, float kPhi, float Ts);

float cos_grid(float alpha, float beta);
float sin_grid(float alpha, float beta);
float phase_detector(float cosGrid, float sinGrid, float anglePllComp);


// NEW
void abc_to_alphabeta(float a, float b, float c, float *alpha, float *beta);
void alphabeta_to_dq(float alpha, float beta, float angle, float *d, float *q);
void dq_to_alphabeta(float d, float q, float angle, float *alpha, float *beta);
void pi_regulator(float phaseError, float feedForward, float ki, float kp, float kPhi, float Ts, float *anglePll, float *anglePllComp);
