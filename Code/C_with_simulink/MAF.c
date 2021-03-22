// This file contains functions that are associated with moving average filter

float moving_avg(float newSample, float t)
{
    // static float dataArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // try this out tomorrow (18/3/2021)
    // static float dataArray[10] = {};
    static float dataArray[200];
    static int i = 0;
    int j = 0;
    float sum = 0;

    if (t == 0)
	{
        for(int ii = 0; ii< 200;ii++)
        {
            dataArray[ii] = 0;
        }

		// dataArray[] = {0};
        i = 0;
		// integrator = 0;
		// old_error = 0;
		// last_t = 0;
	}

    dataArray[i] = newSample;
    for(int j = 0; j < 200; j++)
    {
        // sum = sum + dataArray[i+j];
        sum = sum + dataArray[j];
    }
    
    
    i = i + 1;
    return sum/200;
}


int maf(int len, int nextSample) //float t
{
  static float sum, out_old;
  static int pos;
//   static float sampleArr[len] = {0};
  static float sampleArr[5] = {0, 0, 0, 0, 0};
  
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


    // if statement to set pos to zero when pos > len

  //Subtract the oldest number from the prev sum, add the new number
  sum = sum - sampleArr[pos] + nextSample;
  //Assign the nextNum to the position in the array
  sampleArr[pos] = nextSample;
  pos++;
  if(pos>=len)
  {
      pos = 0; 
  }
  //return the average
  return sum / len;
}

#include <stdio.h>

int main(int argc, char *argv[])
{
  // a sample array of numbers. The represent "readings" from a sensor over time
  int sample[] = {50, 10, 20, 18, 20, 100, 18, 10, 13, 500, 50, 40, 10};
  // the size of this array represents how many numbers will be used
  // to calculate the average
  int arrNumbers[5] = {0};

  int pos = 0;
  int newAvg = 0;
  long sum = 0;
//   int len = sizeof(arrNumbers) / sizeof(int);
  int len = 5;
  int count = sizeof(sample) / sizeof(int);

  for(int i = 0; i < count; i++){
    // newAvg = movingAvg(arrNumbers, &sum, pos, len, sample[i]);
    newAvg = maf(len, sample[i]);
    printf("The new average is %d\n", newAvg);
    // pos++;
    // if (pos >= len){
    //   pos = 0;
    // }
  }

  return 0;
}