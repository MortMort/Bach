#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"


//  Function    :   circular_buffer
//  Description :   This function creates a third symmetrical phase from two phases
//  Parameters  :   float *phaseA, float *phaseB
//  Returns     :   float phaseC
float two_to_three_phase(float *phaseA, float *phaseB)
{
	return -*phaseA - *phaseB;
}



/*
//  Function    :   circular_buffer
//  Description :   This function implements a circular buffer with trigger event. After trigger event a percentage
//                  of the buffer is filled after which the circular buffer stops.
//  Parameters  :   uint16_t bufferSize: pointer to an int to store the number
//                  uint16_t *circularBuffer: Pointer to circular buffer array
//                  uint16_t *dataInput: Pointer to data to be inserted into circular buffer
//                  uint8_t event: A flag which, when triggered, sets the ring buffer to fill the last 1-bufSplit% of the buffer
//                  float bufSplit: The precentage of the buffer which is kept in the ring buffer after event trigger.
//  Returns     :   none
void circular_buffer(uint16_t bufferSize, uint16_t *circularBuffer, uint16_t *dataInput, uint8_t event, float bufSplit)
{
    static uint16_t writeIndex	    =	0;	// Index of the write pointer
    static uint16_t bufferLength	=	0;	// Number of values in circular buffer
    static uint8_t  eventEntry      =   0;
    static uint8_t  bufferFullEntry =   0;



    if (event) {
        if (!eventEntry) {
            // printf("Event triggered!\n");
            bufferLength = bufSplit * bufferSize;
            eventEntry = 1;
        }

        if (bufferLength == bufferSize) {
            if (!bufferFullEntry) {
            	// Do something
                bufferFullEntry = 1;
            }


            ;// FUCKING STOP
        }
        else {
            circularBuffer[writeIndex] = *dataInput;
            bufferLength++;
            writeIndex++;
        }
    }
    else {
        circularBuffer[writeIndex] = *dataInput;
        bufferLength++;
        writeIndex++;
    }


    // Reset bufferlength
    if (bufferLength > bufferSize) {
        bufferLength = bufferSize;
    }
    // Reset writeindex
    if (writeIndex == bufferSize) {
            writeIndex = 0;
    }

}
*/



// Functions


//  Function    :   circular_buffer
//  Description :   This function implements a circular buffer with trigger event. After trigger event a percentage
//                  of the buffer is filled after which the circular buffer stops.
//  Parameters  :   uint16_t bufferSize: pointer to an int to store the number
//                  uint16_t *circularBuffer: Pointer to circular buffer array
//                  uint16_t *dataInput: Pointer to data to be inserted into circular buffer
//                  uint8_t event: A flag which, when triggered, sets the ring buffer to fill the last 1-bufSplit% of the buffer
//                  float bufSplit: The precentage of the buffer which is kept in the ring buffer after event trigger.
//  Returns     :   *readStart: Where to starte reading the ring buffer
uint8_t circular_buffer(uint16_t bufferSize, int16_t *circularBuffer, int16_t *dataInput, uint8_t event, float bufSplit, uint16_t *readStart)
{
    static uint16_t writeIndex	        =	0;	// Index of the write pointer
    static uint16_t bufferLength	    =	0;	// Number of values in circular buffer
    static uint16_t bufferSplitLength   =   0;
    static uint8_t  eventEntry          =   0;
    static uint8_t  bufferFullEntry     =   0;
    static uint8_t	bufferDoneFlag		=	0;

    // If the buffer has been filled, the oldest sample is always in the NEXT index
    // else it is equal to 0;
    /* I no understand why readIndex != writeIndex + 1
    if (bufferLength == bufferSize) {
        if (writeIndex != bufferSize) {
            *readStart = writeIndex;
        }
        else {
            *readStart = 0;
        }
    } */

    if (bufferLength == bufferSize) {
        *readStart = writeIndex;
    }

    if (event) {
        if (!eventEntry) {
            // Event has triggered
            bufferSplitLength = bufSplit * bufferSize;
            eventEntry = 1;
        }

        if (bufferSplitLength == bufferSize) {
            if (!bufferFullEntry) {
            	bufferDoneFlag = 1;
                // Do something?
                bufferFullEntry = 1;
            }
            // STOP
        }
        else {
            circularBuffer[writeIndex] = *dataInput;
            bufferSplitLength++;
            writeIndex++;
            bufferLength++;
        }
    }
    else {
        circularBuffer[writeIndex] = *dataInput;
        writeIndex++;
        bufferLength++;
    }



    // Reset bufferlength
    if (bufferLength > bufferSize) {
        bufferLength = bufferSize;
    }
    // Reset writeindex
    if (writeIndex == bufferSize) {
        writeIndex = 0;
    }

    return bufferDoneFlag;
}


/*
//  Function    :   printRingBuf
//  Description :   prints the ring buffer values
//  Parameters  :   uint16_t bufferSize: pointer to an int to store the number
//                  uint16_t *circularBuffer: Pointer to circular buffer array
//                  uint16_t readStart: starting index of the circular buffer
//  Returns     :	none
void printRingBuf(uint16_t bufferSize, uint16_t *circularBuffer, uint16_t readStart) {
    static uint16_t readIndex   =   0;
    static uint8_t init         =   0;

    static char msg[50];

    // Initialize readIndex to readStart
    if (!init) {
        readIndex = readStart;
        init = 1;
    }

    for (int i = 0; i < bufferSize; i++)
    {
        //printf("Buffervalue at index [%d] = %d\n", readIndex, circularBuffer[readIndex]);

    	huart2.Instance->CR3 |= USART_CR3_DMAT;
    	sprintf(msg, "circularBuffer index [%d] = %d\r\n", readIndex, circularBuffer[readIndex]);	// Update message for usart print
    	HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)msg,
    							(uint32_t)&huart2.Instance->DR, strlen(msg));

        readIndex++;
        if (readIndex > bufferSize) {
            readIndex = 0;
        }
    }

}
*/