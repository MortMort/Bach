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


int maf(int len, float nextSample, float t) //float t is not supposed to be included in stm
{
  static float sum, out_old;
  static int pos;
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

  return sum / len;
}

int maf1(int len, float nextSample, float t) //float t is not supposed to be included in stm
{
  static float sum, out_old;
  static int pos;
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

  return sum / len;
}

int maf2(int len, float nextSample, float t) //float t is not supposed to be included in stm
{
  static float sum, out_old;
  static int pos;
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

  return sum / len;
}
