// This file contains prototypes of functions that are associated with moving average filter


// OLD
//float maf1(float nextSample);
//float maf2(float nextSample);
// float moving_avg(float newSample, float t);
// float maf(float len, float nextSample);

// NEW
void maf(float *nextSampleD, float *nextSampleQ, float *mafD, float *mafQ);
