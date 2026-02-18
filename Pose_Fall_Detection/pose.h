/*
 * surface.h
 *
 */

#ifndef POSE_H_
#define POSE_H_

/* MMWAVE library Include Files */
#include <control/mmwave/mmwave.h>
#include <common/sys_common_xwrLx4xx.h>
#include <drivers/uart/v0/uart_sci.h>
#include "motion_detect.h"

#include "tvmgen_default.h"
#define NO_NN_RESULT       -1
#define CLASS_THRESHOLD     0.5

// define the format of the range profile data that we are feeding to the model (used in CreateFeatureVector())
#define NUM_FRAMES_OF_DATA 8

// define some program control conditions
//#define benchTime   // benchmark the code.

// Define the TLV header index. 
#define MMWDEMO_OUTPUT_MSG_ML_CLASSIFICATION_PROBABILITY 1031

// Wet/Dry detection
#define ML_TYPE_WET_DRY 		        1       // classification identifier (TYPE)
#define ML_TYPE_WET_DRY_CLASSES         2       // number of classes
#define ML_TYPE1_FEATURE_COUNT          25		// The number of range bins to include as input to the neural network model
#define ML_TYPE1_FEATURE_START          6		// Starting Range bin
#define ML_TYPE1_FEATURE_END          	(ML_TYPE2_FEATURE_START + ML_TYPE2_FEATURE_COUNT - 1)		// End Range bin
#define ML_TYPE1_FEATURE_SIZE           4       //4B = FLOAT32

// Grass/notGrass detection
#define ML_TYPE_GRASS_NOTGRASS          2       // classification identifier (TYPE)
#define ML_TYPE_GRASS_NOTGRASS_CLASSES  2       // number of classes
#define ML_TYPE2_CLASS1_DATA            "D:/Workspaces/workspace_v12/ML/pose_and_fall_demo_xwrL6432/dataSets/CCS_grass_data.bin"
#define ML_TYPE2_CLASS2_DATA            "D:/Workspaces/workspace_v12/ML/pose_and_fall_demo_xwrL6432/dataSets/CCS_not_data.bin"
#define ML_TYPE2_FEATURE_COUNT          25		// The number of range bins to include as input to the neural network model
#define ML_TYPE2_FEATURE_START          6		// Starting Range bin
#define ML_TYPE2_FEATURE_END          	(ML_TYPE2_FEATURE_START + ML_TYPE2_FEATURE_COUNT - 1)		// End Range bin
#define ML_TYPE2_FEATURE_SIZE           4       //4B = FLOAT32

// Human pose detection
// posz, velx, vely, velz, accx, accy, accz, y0, z0, snr0, y1, z1, snr1, y2, z2, snr2, y3, z3, snr3, y4, z4, snr4
#define ML_TYPE_HUMANPOSE               3       // classification identifier (TYPE)
#define ML_TYPE_HUMANPOSE_CLASSES       5       // number of classes
#define ML_TYPE3_CLASS1_DATA            "D:/Workspaces/workspace_v12/ML/pose_and_fall_demo_xwrL6432/dataSets/CCS_STANDING_data.bin"
#define ML_TYPE3_CLASS2_DATA            "D:/Workspaces/workspace_v12/ML/pose_and_fall_demo_xwrL6432/dataSets/CCS_SITTING_data.bin"
#define ML_TYPE3_CLASS3_DATA            "D:/Workspaces/workspace_v12/ML/pose_and_fall_demo_xwrL6432/dataSets/CCS_LYING_data.bin"
#define ML_TYPE3_FEATURE_COUNT          22
#define ML_TYPE3_FEATURE_DATA_TYPE      4       //4B = FLOAT32
#define ML_TYPE3_CLASS_DATA_TYPE        4       //4B = FLOAT32
#define ML_TYPE3_FEATURE_MIN_COUNT      5		//Minimum of 5 points needed for 5 Max height points


// Define the type of classification
#define ML_NUMBER_OF_CLASSES            ML_TYPE_HUMANPOSE_CLASSES

// Structure holds classification result to be sent
typedef struct MmwDemo_output_message_ml_t
{
    uint32_t mlClassType;                   // Classification ID
    uint32_t mlClassCount;                 // # of classes
    float mlResult[ML_NUMBER_OF_CLASSES];   // Classification results
                                            // (Closer to 1 indicates higher confidence in that class)
} MmwDemo_output_message_ml;

/** \brief ML attributes */
typedef struct
{
    uint32_t   mlType;              // Classification ID
    uint32_t   mlClassNumber;       // # of classes
    char       mlClass1[10];        // Class1 type
    char       mlClass2[10];        // Class2 type
    char       mlClass3[10];        // Class3 type
    char       mlClass4[10];        // Class4 type
    char       mlClass5[10];        // Class5 type
} ML_Attrs;


// Prototypes
void CreateFeatureVector();	        // formulating the model input.
int32_t Inference(float *);			// main model function

#endif /* POSE_H_ */
