
// float movingAvg(float u)
// {
    
//     return u/4;
// }
// #include <stdio.h>
// #include <math.h>
// float movingAvg(float u); // delete later 
// float y;
// void main()
// {

//     // float x_array[10] = {200,200,200,200,200,200,200,200,200,200};
//     float x;
//     for(int k=0; k<2000; k++)
//     {
//         x = sin(0.001*k);
//         // y = movingAvg(x_array[k]);
//         y = movingAvg(x);
//         printf("loop no: %i, avg: %f \n", k, y);

//     }

// }


float movingAvg(float u)
{
    // static float dataArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // try this out tomorrow (18/3/2021)
    // static float dataArray[10] = {};
    static float dataArray[10000];
    static int i = 0;
    int j = 0;
    float sum = 0;

    dataArray[i] = u;
    for(int j = 0; j < 2000; j++)
    {
        // sum = sum + dataArray[i+j];
        sum = sum + dataArray[j];
    }
    
    
    i = i + 1;
    return sum/2000;
}

// #include <stdio.h>
// #include <stdlib.h>


// void main()
// {
//     float x_array[10] = {200,200,200,200,200,200,200,200,200,200};

//     float x = 0, xmem = 0, ymem = 0, y = 0;
//     int counter=0;
//     float N = 5;
//     int i = 0;
//     for(int i = 0; i < 10; i++)
//     {
//         x = x_array[i];
//         printf("loop no: %i\n", i);

//         y = 1/N*(x - xmem) + ymem;
//         printf("y: %.2f\n", y);
//         /*
//         printf("x - xmem + ymem:%.2f\n", (x - xmem + ymem));
//         printf("x:%.2f\n", x);
//         printf("xmem:%.2f\n", xmem);
//         printf("ymem:%.2f\n", ymem);
//         printf("1/N = %f", (1/N));
//         */
//         ymem = y;
//         if (counter >= N-1)
//         {
//             xmem = x;
//         }
//         counter++;
//         printf("%.2f\n\n", y);
//     }

// }