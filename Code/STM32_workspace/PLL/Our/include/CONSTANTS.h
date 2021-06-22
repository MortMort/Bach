// All constants in the system should be defined here
//--------------------------------------

// Ring buffer
#define RING_BUF_LEN 		35000								// Size of ring buffer
//#define RING_BUF_SCALING	0xFFFF/(2*TWO_PI)	// 5215.2
#define RING_BUF_SCALING	5000								// Lower than above for protection against noise?
#define RING_BUF_SIZE		1									// Amount of signals in the ring buffer
#define RING_BUF_SPLIT		0.0f								// Percentage of the ring buffer allocation to before trigger event

#define ADC_RING_BUF_SIZE	10									// Size of adc ring buffers

// Math
#define PI 					(3.1415926535897)					// PI
#define TWO_PI 				(2.0*PI)							// 2*PI
#define RAD_120				120.0f*3.1415926535897f/180.0f		// [rad] 120 deg phase in radians
#define F_RAD 				(50.0f*3.1415926535897f*2.0f)		// [rad/s] 50 Hz in radians
#define DAC_SCALING			4096.0f/3.3f						// Voltage -> 12 bit

// Sampling
#define F_SAMPLE 			10000								// [Hz] The sample rate of the system (the interrupt timer
																// should be changed along with this value!!!!)
#define T_SAMPLE 			0.0001f								// [s] The sample time (should always be 1/F_SAMPLE!!)

// MAF
#define T_MAF				0.02f
#define MAF_LEN 			(int)(200)							// MAF length REMEMBER TO SET THIS WHEN CHANGING SAMPLE FREQUENCY

// Tuning

// T_st = 0.02 * 6:
//float ki = 2938.8889;
//float kp = 106.0408611;
//float kPhi = 0.010;

// T_st = 0.02 * 1:
#define KI 105800
#define KP 1465.1f
#define KPHI 0.0095f
// Other
