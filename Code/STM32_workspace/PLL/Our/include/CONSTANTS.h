// All constants in the system should be defined here
//--------------------------------------

// Ring buffer
#define RING_BUF_LEN 		7000								// Size of ring buffer
//#define RING_BUF_SCALING	0xFFFF/(2*TWO_PI)	// 5215.2
#define RING_BUF_SCALING	5000								// Lower than above for protection against noise?
#define RING_BUF_SIZE		8									// Amount of signals in the ring buffer
#define RING_BUF_SPLIT		0.0f								// Percentage of the ring buffer allocation to before trigger event

#define ADC_RING_BUF_SIZE	10									// Size of adc ring buffers

// Math
#define PI 					(3.1415926535897)					// PI
#define TWO_PI 				(2.0*PI)							// 2*PI
#define RAD_120				120.0f*3.1415926535897f/180.0f		// [rad] 120 deg phase in radians
#define F_RAD 				(50.0f*3.1415926535897f*2.0f)		// [rad/s] 50 Hz in radians

// Sampling
#define F_SAMPLE 			2000								// [Hz] The sample rate of the system (the interrupt timer
																// should be changed along with this value!!!!)
#define T_SAMPLE 			0.0005f								// [s] The sample time (should always be 1/F_SAMPLE!!)

// MAF
#define T_MAF				0.02f
#define MAF_LEN 			(int)(40)							// MAF length REMEMBER TO SET THIS WHEN CHANGING SAMPLE FREQUENCY

// Other
