// This file contains functions that are associated with moving average filter


float maf1(float nextSample, float t) //float t is not supposed to be included in stm
{
  static float sum, out_old;
  static int pos;
  int len = 2000;
  static float sampleArr[2000] = {0}; // tried parsing len instead of 200 but didnt work
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

  return sum / (float) len; 
}

float maf2(float nextSample, float t) //float t is not supposed to be included in stm
{
  static float sum, out_old;
  static int pos;
  int len = 2000;
  static float sampleArr[2000] = {0}; // tried parsing len instead of 200 but didnt work
  

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
  return sum / (float) len;
}


