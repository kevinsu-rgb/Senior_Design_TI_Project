/* dpu_manager.h */
#ifndef DPU_MANAGER_H
#define DPU_MANAGER_H

#include <stdint.h>


typedef struct Radar_Params_t {
    uint8_t  numTxAntennas;
    uint8_t  numRxAntennas;
    uint8_t  numVirtualAntennas;
    uint16_t numRangeBins;
    uint16_t rangeFftSize;
    uint16_t numDopplerChirps;
    uint16_t numDopplerBins;
} Radar_Params_t;



#endif /* DPU_MANAGER_H */