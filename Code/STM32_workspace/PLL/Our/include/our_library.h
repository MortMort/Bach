#include "CONSTANTS.h"
float two_to_three_phase(float *phaseA, float *phaseB);
// void circular_buffer(uint16_t bufferSize, uint16_t *circularBuffer, uint16_t *dataInput, uint8_t event, float bufSplit);
uint8_t circular_buffer(uint16_t bufferSize, int16_t circularBuffer[][RING_BUF_SIZE], int16_t *dataInput, uint8_t event, uint8_t reset, float bufSplit, uint16_t *readStart);
float dac_offset(float var, float b, float a);
//void printRingBuf(uint16_t bufferSize, uint16_t *circularBuffer, uint16_t readStart);
