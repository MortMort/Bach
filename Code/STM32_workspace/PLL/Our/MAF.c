// This file contains functions that are associated with moving average filter
#include "CONSTANTS.h"


//  Function    :   circular_buffer
//  Description :   This function creates a third symmetrical phase from two phases
//  Parameters  :   float *nextSampleD		-
//					float *nextSampleQ		-
//  Returns     :   indirect : float *mafD	-
//							   float *mafQ	-
void maf(float *nextSampleD, float *nextSampleQ, float *mafD, float *mafQ)
{
	static int pos;
	static float sampleArrD[MAF_LEN], sampleArrQ[MAF_LEN], sumD, sumQ;

	sumD = sumD - sampleArrD[pos] + *nextSampleD;
	sumQ = sumQ - sampleArrQ[pos] + *nextSampleQ;

	sampleArrD[pos] = *nextSampleD;
	sampleArrQ[pos] = *nextSampleQ;

	pos++;
	if (pos >= MAF_LEN) {
		pos = 0;
	}

	*mafD = sumD/(float)MAF_LEN;
	*mafQ = sumQ/(float)MAF_LEN;
}
//
//float maf1(float nextSample) //float t is not supposed to be included in stm
//{
//  static float sum;
//  static int pos;
//  const int len = 20;
//  static float sampleArr[20] = {0}; // tried parsing len instead of 200 but didnt work
//
//  sum = sum - sampleArr[pos] + nextSample;
//  sampleArr[pos] = nextSample;
//  pos++;
//  if(pos>=len)
//  {
//      pos = 0;
//  }
//
//  return sum / (float) len;
//}
//
//float maf2(float nextSample) //float t is not supposed to be included in stm
//{
//  static float sum;
//  static int pos;
//  int len = 20;
//  static float sampleArr[20] = {0}; // tried parsing len instead of 200 but didnt work
//
//  sum = sum - sampleArr[pos] + nextSample;
//  sampleArr[pos] = nextSample;
//  pos++;
//  if(pos>=len)
//  {
//      pos = 0;
//  }
//  return sum / (float) len;
//}


