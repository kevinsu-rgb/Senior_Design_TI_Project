/*
TI Senior Design Project
----------------------------

dpc.c 
--------

This file is the data path control logic. Uses rangeproc(rangefft), dopplerproc(range-doppler heatmap),
and doaproc(range-azimuth heatmap). 
*/


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <datapath/dpu/rangeproc/v0/rangeprochwa.h>
#include <datapath/dpu/dopplerproc/v0/dopplerprochwa.h>
#include <datapath/dpu/doaproc/v0/doaproc.h>

// L3 = 512kB
#define L3_MEM_SIZE (0x80000)
uint8_t MmwL3[L3_MEM_SIZE]  __attribute((section(".l3")));

// memory buffers
cmplx16ImRe_t *radarCube;
uint32_t *rangeDopplerHeatmap;
uint32_t *rangeAzimuthHeatmap;

// Global Handles
DPU_RangeProcHWA_Handle rangeProcHandle;
DPU_DopplerProcHWA_Handle dopplerProcHandle;
DPU_DoaProc_Handle doaProcHandle;

// Configs
DPU_RangeProcHWA_Config rangeProcCfg;
DPU_DopplerProcHWA_Config dopplerProcCfg;
DPU_DoaProc_Config doaProcCfg;

typedef struct Radar_Params_t {
    uint8_t  numTxAntennas;
    uint8_t  numRxAntennas;
    uint16_t numAdcSamples;
    uint8_t  numVirtualAntennas;
    uint16_t numRangeBins;
    uint16_t rangeFftSize;
    uint16_t numDopplerChirps;
    uint16_t numDopplerBins;
} Radar_Params_t;


static Radar_Params_t radarParams = {
    .numTxAntennas      = 2,
    .numRxAntennas      = 3,
    .numVirtualAntennas = 6,
    .numAdcSamples      = 256,
    .rangeFftSize       = 256,
    .numRangeBins       = 128,
    .numDopplerChirps   = 64,
    .numDopplerBins     = 64
};


#define EDMA_CH_RANGE_IN        0
#define EDMA_CH_RANGE_OUT_PING  1
#define EDMA_CH_RANGE_OUT_PONG  2


// Range Proc (1D fft) config parser
int32_t rangeProcCfgParser(){
    int32_t retVal = 0;
    DPU_RangeProcHWA_HW_Resources *HwConfig = &rangeProcCfg.hwRes;
    DPU_RangeProcHWA_StaticConfig  *params = &rangeProcCfg.staticCfg;

    memset((void *)&rangeProcCfg, 0, sizeof(DPU_RangeProcHWA_Config));

    params->numTxAntennas = radarParams.numTxAntennas;
    params->numVirtualAntennas = radarParams.numVirtualAntennas;
    params->numRangeBins = radarParams.numRangeBins;
    params->numDopplerChirpsPerProc = radarParams.numDopplerChirps;
    params->numChirpsPerFrame = params->numDopplerChirpsPerProc * params->numTxAntennas;
    params->rangeFftSize = radarParams.rangeFftSize;
    params->windowSize = sizeof(uint32_t) * ((radarParams.numAdcSamples + 1) / 2);

    params->ADCBufData.dataProperty.numAdcSamples = radarParams.numAdcSamples;
    params->ADCBufData.dataProperty.numRxAntennas = radarParams.numRxAntennas;
    params->ADCBufData.dataProperty.dataFmt       = DPIF_DATAFORMAT_REAL16; 
    params->ADCBufData.dataProperty.interleave    = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
    params->ADCBufData.data                       = (void *)CSL_APP_HWA_ADCBUF_RD_U_BASE;
    params->ADCBufData.dataSize                   = radarParams.numAdcSamples * radarParams.numRxAntennas * 4; 

    // Calculate RX offsets 
    uint16_t bytesPerRxChan = (radarParams.numAdcSamples * sizeof(uint16_t) + 15) / 16 * 16; // 16-byte aligned
    for (int i = 0; i < radarParams.numRxAntennas; i++) {
        params->ADCBufData.dataProperty.rxChanOffset[i] = i * bytesPerRxChan;
    }

        
    /* Radar Cube Output (L3) */
    HwConfig->radarCube.data      = radarCube;
    HwConfig->radarCube.dataSize  = radarParams.numRangeBins * radarParams.numVirtualAntennas * radarParams.numDopplerChirps * sizeof(cmplx16ImRe_t);;
    HwConfig->radarCube.datafmt   = DPIF_RADARCUBE_FORMAT_6; 

    /* EDMA Configuration (Input Path) */
    HwConfig->edmaInCfg.dataIn.channel        = EDMA_CH_RANGE_IN;
    HwConfig->edmaInCfg.dataIn.channelShadow[0] = EDMA_CH_RANGE_IN + 10; 
    HwConfig->edmaInCfg.dataIn.channelShadow[1] = EDMA_CH_RANGE_IN + 11;

    /* EDMA Configuration (Output Path - Ping/Pong) */
    HwConfig->edmaOutCfg.path[0].dataOutMajor.channel = EDMA_CH_RANGE_OUT_PING;
    HwConfig->edmaOutCfg.path[1].dataOutMajor.channel = EDMA_CH_RANGE_OUT_PONG;
}



void dpu_main(void *args)
{
    // Open drivers
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Debug Message\r\n");

   

    Board_driversClose();
    Drivers_close();
}
