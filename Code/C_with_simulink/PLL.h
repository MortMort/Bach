// this file contains prototypes of functions of PLL.


float abc_to_alpha(float a, float b, float c);
float abc_to_beta(float a, float b, float c);
float alphabeta_to_d(float alpha, float beta, float angle);
float alphabeta_to_q(float alpha, float beta, float angle);
float dq_to_alpha(float d, float q, float angle);
float dq_to_beta(float d, float q, float angle);
float cos_grid(float alpha, float beta);
float sin_grid(float alpha, float beta);
float pi_regulator(float phaseError, float feedForward, float ki, float kp, float kPhi, float t, float Ts);
float pi_regulator_comp(float phaseError, float feedForward, float ki, float kp, float kPhi, float t, float Ts);
float phase_detector(float cosGrid, float sinGrid, float anglePllComp);
