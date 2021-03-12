#include <stdio.h>
#include <stdlib.h>

/*
float MAFilter(int N, float x)
{
    float xmem, ymem, y;
    int counter;
    if (counter = N-1)
    {
        xmem = x;
        counter = 0;
    }
    counter++;

    y = 1/N*x-xmem+ymem;
    ymem = y;
    return y;
}
*/

void main()
{
    float x_array[10] = {200,200,200,200,200,200,200,200,200,200};

    float x = 0, xmem = 0, ymem = 0, y = 0;
    int counter=0;
    float N = 5;
    int i = 0;
    for(int i = 0; i < 10; i++)
    {
        x = x_array[i];
        printf("loop no: %i\n", i);

        y = 1/N*(x - xmem) + ymem;
        printf("y: %.2f\n", y);
        /*
        printf("x - xmem + ymem:%.2f\n", (x - xmem + ymem));
        printf("x:%.2f\n", x);
        printf("xmem:%.2f\n", xmem);
        printf("ymem:%.2f\n", ymem);
        printf("1/N = %f", (1/N));
        */
        ymem = y;
        if (counter >= N-1)
        {
            xmem = x;
        }
        counter++;
        printf("%.2f\n\n", y);
    }

}
