#include <math.h>
#include <stdio.h>
#include <string.h>

float *sine_creator(uint32_t fSample, float tDuration)
{
	uint32_t nSample = round(fSample*tDuration);
	static float sine[10000];
	uint32_t var;
	for (var = 0; var < nSample; ++var)
	{
		sine[var] = sinf((float)var*1/fSample*2*3.14159265359);
	}
	return sine;
}


//float *sine_creator()
//{
//	uint32_t f_sample;
//	float t_duration;
//	f_sample = 10000;
//	t_duration = 1;
//	static float sine[10000];
//	uint32_t var;
//	for (var = 0; var < 10000; ++var)
//	{
//		sine[var] = sinf(var*f_sample);
//	}
//	return sine;
//}
