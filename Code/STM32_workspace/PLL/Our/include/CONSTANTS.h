#define adcBuf_LEN 			10									// Size of ADC buffer (unused)
#define RING_BUF_LEN 		2000								// Size of ring buffer
#define PI 					(3.1415926535897)
#define TWO_PI 				(2.0*PI)
#define F_RAD 				(50.0f*3.1415926535897f*2.0f)
#define F_SAMPLE 			1000
#define T_SAMPLE 			0.001f
#define T_SINE 				1.0f								// [s] sine time
#define RAD_120				120.0f*3.1415926535897f/180.0f
//#define RING_BUF_SCALING	0xFFFF/(2*TWO_PI)	// 5215.2
#define RING_BUF_SCALING	5000				// Lower than above for protection against noise
