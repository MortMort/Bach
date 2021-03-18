

float abc_to_alphabeta(float *a, float *b, float *c)
{
    // float *alpha = 0;
    // float *beta = 0;
    float arr[2];

    alpha =  0.6666667 * a - 0.3333333 * b - 0.3333333 * c;
    beta = 0.5773503*b - 0.5773503*c;

    arr[0] = alpha;
    arr[1] = beta;

    return arr;
}