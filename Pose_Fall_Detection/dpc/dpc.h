/*
 * Copyright (C) 2022-24 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL 0
#define DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL 1

/*DPC Object Detection functions*/
uint8_t      DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(HwaDmaTrigChanPoolObj *pool);
void         DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(HwaDmaTrigChanPoolObj *pool);
void         DPC_ObjDet_MemPoolReset(MemPoolObj *pool);
static void  DPC_ObjDet_MemPoolSet(MemPoolObj *pool, void *addr);
static void *DPC_ObjDet_MemPoolGet(MemPoolObj *pool);
uint32_t     DPC_ObjDet_MemPoolGetMaxUsage(MemPoolObj *pool);
void        *DPC_ObjDet_MemPoolAlloc(MemPoolObj *pool, uint32_t size, uint8_t align);

/*Utility functions*/
void    rangeBiasRxChPhaseMeasure_quadfit(float *x, float *y, float *xv, float *yv);
void    MmwDemo_calcActiveAntennaGeometry();
int32_t MmwDemo_cfgDopplerParamMapping(DPU_DoaProc_HWA_Option_Cfg *dopplerParamCfg, uint32_t mappingOption);
void    MmwDemo_compressPointCloudList(MmwDemo_output_message_UARTpointCloud *pointCloudOut,
                                       MmwDemo_output_message_point_unit     *pointCloudUintRecip,
                                       DPIF_PointCloudCartesianExt           *pointCloudIn,
                                       uint32_t                               numPoints);

/*Feature Extraction functions*/
void     featExtract_heapConstruct();
void    *featExtract_malloc(uint32_t sizeInBytes);
void     featExtract_free(void *pFree, uint32_t sizeInBytes);
uint32_t featExtract_memUsage();

/*DPU init functions*/
void rangeProc_dpuInit();
void doaProc_dpuInit();
void cfarProc_dpuInit();
void mpdProc_dpuInit();
void udopProc_dpuInit();
void trackerProc_dpuInit();

/*DPU config functions*/
void mmwDemo_rangeProcConfig();
void mmwDemo_doaProcConfig();
void mmwDemo_cfarProcConfig();
void mmwDemo_mpdProcConfig();
void mmwDemo_uDopProcConfig();
void mmwDemo_trackerConfig(void);

/*DPU config parser functions*/
int32_t RangeProc_configParser();
int32_t DoaProc_configParser();
int32_t CfarProc_configParser();
int32_t MpdProc_configParser();
int32_t uDopProc_configParser();

/*Task for configuring and executing DPC*/
void mmwDemo_dpcTask();
/*Function initiliazing all indvidual DPUs*/
void DPC_Init();
/*Function configuring all DPUs*/
void DPC_Config();
/*DPC processing chain execute function.*/
void DPC_Execute();
