/*
 * gesture.c
 *
 */
/* Standard Include Files. */

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <control/mmwave/mmwave.h>

#include "pose.h"
#include "tvmgen_default.h"

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern uint32_t Cycleprofiler_getTimeStamp(void);
extern uint32_t gpioBaseAddrLed, pinNumLed;
extern uint32_t gDataSet[];

int32_t inputVector[ML_TYPE3_FEATURE_COUNT * NUM_FRAMES_OF_DATA];
int32_t inputVectorTemp[ML_TYPE3_FEATURE_COUNT * NUM_FRAMES_OF_DATA];

float input[NUM_FRAMES_OF_DATA * ML_TYPE3_FEATURE_COUNT] = {0.0};
float output[ML_TYPE_HUMANPOSE_CLASSES] = {-1.0};

struct tvmgen_default_inputs inputs = { (void *) &input };
struct tvmgen_default_outputs outputs = { (void *) &output };

#define FRAME_REF_TIMER_CLOCK_MHZ  40

/* ML attributes */
ML_Attrs gMLAttrs =
{
    .mlType               = ML_TYPE_HUMANPOSE,
    .mlClassNumber        = ML_TYPE_HUMANPOSE_CLASSES,
    .mlClass1             = "Stood",
    .mlClass2             = "Sat",
    .mlClass3             = "Lying",
    .mlClass4             = "Falling",
    .mlClass5             = "Walking",
};

/**
 *  @b Description
 *  @n
 *      Build feature vector used for nn model inference.
 */
void CreateFeatureVector()
{
    uint32_t i = 0;

    // Open up the head of the circular buffer for the new frame of data.
    for (i = 0; i < NUM_FRAMES_OF_DATA * (ML_TYPE3_FEATURE_COUNT - 1); i++) {
        inputVectorTemp[i] = inputVectorTemp[i + ML_TYPE3_FEATURE_COUNT];
    }

    // Now add in the latest frame of data
    for (i = 0; i < ML_TYPE3_FEATURE_COUNT; i++) {
        inputVectorTemp[NUM_FRAMES_OF_DATA * ML_TYPE3_FEATURE_COUNT - ML_TYPE3_FEATURE_COUNT + i] = gDataSet[i];
    }

    // Now interleave the frames
    uint32_t j = 0;
    uint32_t k = 0;
    for (i = 0; i < NUM_FRAMES_OF_DATA * ML_TYPE3_FEATURE_COUNT; i++) {
        inputVector[i] = inputVectorTemp[k * ML_TYPE3_FEATURE_COUNT + j];
        if (k == NUM_FRAMES_OF_DATA-1) {
            k = 0;
            j++;
        }
        else
            k++;
    }
    // copy to pointer for input into model
    memcpy(inputs.input_1, inputVector, sizeof(inputVector));
}

/**
 *  @b Description
 *  @n
 *      Create the feature vector 
 *		Call the inference model.
 *		return 0:success, -1: error
 */

int32_t Inference(float *outputBuf)
{
    int32_t retVal = 0;

#ifdef benchTime
    int end_time, start_time;
    start_time = Cycleprofiler_getTimeStamp();
#endif

    CreateFeatureVector();       // create the formatted input for the model

#ifdef benchTime
    end_time = Cycleprofiler_getTimeStamp();
    // benchmark the model calculation.
    DebugP_log("CreateFeatureVector(us): %.2f \n", (float)(end_time-start_time)/FRAME_REF_TIMER_CLOCK_MHZ);
#endif
    // call the model for inference output.
#ifdef benchTime
    GPIO_pinWriteHigh(gpioBaseAddrLed, pinNumLed);
    start_time = Cycleprofiler_getTimeStamp();
#endif

    retVal = tvmgen_default_run(&inputs, &outputs);      //run the model
    memcpy(outputBuf, (float *)outputs.output, ML_NUMBER_OF_CLASSES*ML_TYPE3_FEATURE_DATA_TYPE);


#ifdef benchTime
    end_time = Cycleprofiler_getTimeStamp();
    GPIO_pinWriteLow(gpioBaseAddrLed, pinNumLed);
    // benchmark the model calculation. approx 600uS for model execution
    DebugP_log("Model time(us): %.2f \n\n", (float)(end_time-start_time)/FRAME_REF_TIMER_CLOCK_MHZ);
#endif

    return retVal;          // return 0:success, -1: error
}
