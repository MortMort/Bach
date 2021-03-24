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


// float maf(float len, float nextSample) //float t is not supposed to be included in stm
// {
//   static float sum, out_old;
//   static int pos;
//   static float sampleArr[200] = {0}; // tried parsing len instead of 200 but didnt work
  
// //   if (t == 0)
// //   {
// //         sum = 0;
// //         out_old = 0;
// //         pos = 0;
// //         for(int i = 0; i < len; i++)
// //         {
// //             sampleArr[i] = 0;
// //         }	
// //   }

//   sum = sum - sampleArr[pos] + nextSample;
//   sampleArr[pos] = nextSample;
//   pos++;
//   if(pos>=len)
//   {
//       pos = 0; 
//   }

//   return sum / len;
// }

// float maf1(float len, float nextSample) //float t is not supposed to be included in stm
float maf1(float nextSample, float t) //float t is not supposed to be included in stm
{
  static float sum, out_old;
  static int pos;
  int len = 200;
  static float sampleArr[200] = {0}; // tried parsing len instead of 200 but didnt work
  if (t == 0)
  {
        sum = 0;
        out_old = 0;
        pos = 0;
        for(int i = 0; i < len; i++)
        {
            sampleArr[i] = 0;
        }	
  }

  sum = sum - sampleArr[pos] + nextSample;
  sampleArr[pos] = nextSample;
  pos++;
  if(pos>=len)
  {
      pos = 0; 
  }
//   avg = sum / len;
//   return avg;
//   return (float) sum / len;
  return sum / (float) len; // 200.0f;
}

float maf2(float nextSample, float t) //float t is not supposed to be included in stm
{
  static float sum, out_old;
  static int pos;
  int len = 200;
  static float sampleArr[200] = {0}; // tried parsing len instead of 200 but didnt work
  

  if (t == 0)
  {
        sum = 0;
        out_old = 0;
        pos = 0;
        for(int i = 0; i < len; i++)
        {
            sampleArr[i] = 0;
        }	
  }

  sum = sum - sampleArr[pos] + nextSample;
  sampleArr[pos] = nextSample;
  pos++;
  if(pos>=len)
  {
      pos = 0; 
  }
//   return (float) sum / len;
  return sum / (float) len;
}


// #include <stdio.h>
// // int main(int argc, char *argv[])
// void main()
// {
//   // a sample array of numbers. The represent "readings" from a sensor over time
//   float sample[] = {50.1, 10.01, 20, 18, 20, 100, 18, 10, 13, 500, 50, 40, 10};

// //   int newAvg = 0;
//   float newAvg = 0;

// //   int len = 5;
//   float len = 5;
// //   int count = sizeof(sample) / sizeof(int);

//   for(int i = 0; i < 13; i++){
//     newAvg = maf(len, sample[i]);
//     printf("The new average is %.5f\n", newAvg);
//     // pos++;
//     // if (pos >= len){
//     //   pos = 0;
//     // }
//   }

// //   return 0;
// }

// // these functions are with length input
// float maf1(float len, float nextSample, float t) //float t is not supposed to be included in stm
// {
//   static float sum, out_old;
//   static int pos;
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
//   return sum / len;
// }

// float maf2(float len, float nextSample, float t) //float t is not supposed to be included in stm
// {
//   static float sum, out_old;
//   static int pos;
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
//   return sum / len;
// }