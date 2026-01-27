/*
TI Senior Design Project
----------------------------

sensor.c 
--------

This file is what controls the radar config, init, and start.

*/



/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* Sensor SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <utils/testlogger/logger.h>
#include <control/mmwave/mmwave.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>

MCPI_LOGBUF_INIT(9216);

// GLOBALS
MMWave_Handle sensorHandle;
MMWave_InitCfg initCfg;
MMWave_CtrlCfg ctrlCfg;
MMWave_OpenCfg openCfg;
MMWave_StrtCfg strtCfg;
int32_t errCode;
MMWave_CalibrationCfg calibrationCfg;
int32_t retVal;
MMWave_ErrorLevel errorLevel;
int16_t mmWaveErrorCode;
int16_t subsysErrorCode;
int32_t status;

typedef struct
{
    uint32_t StartFreqHighRes[4]; /* LUT address 0 */
    uint32_t StartFreqLowRes[4]; /* LUT address 16 */
    int16_t ChirpSlope[4]; /* LUT address 32 */
    uint16_t ChirpIdleTime[4]; /* LUT address 40 */
    uint16_t ChirpAdcStartTime[4]; /* LUT address 48 */
    int16_t ChirpTxStartTime[4]; /* LUT address 56 */
    uint8_t ChirpTxEn[4]; /* LUT address 64 */
    uint8_t ChirpBpmEn[4]; /* LUT address 68 */
} T_SensPerChirpLut;

T_SensPerChirpLut* sensPerChirpLuTable = (T_SensPerChirpLut*)(0x21880000U);

static void populateDefaultProfileCfg (T_RL_API_SENS_CHIRP_PROF_COMN_CFG* ptrProfileCfg, T_RL_API_SENS_CHIRP_PROF_TIME_CFG* ptrProfileTimeCfg)
{
    /* Initialize the profile configuration: */
    memset ((void*)ptrProfileCfg, 0, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG));

    /* Populate the *default* profile configuration: */
    ptrProfileCfg->c_DigOutputSampRate = M_RL_SENS_DIG_OUT_SAMP_RATE_MAX_12P5M;
    ptrProfileCfg->c_DigOutputBitsSel = M_RL_SENS_DIG_OUT_12BITS_4LSB_ROUND;
    ptrProfileCfg->c_DfeFirSel = M_RL_SENS_DFE_FIR_LONG_FILT;
    ptrProfileCfg->c_VcoMultiChipMode = M_RL_SENS_VCO_MULT_CHIP_SINGLE;
    ptrProfileCfg->h_NumOfAdcSamples = 256U; /* 2.56us */
    ptrProfileCfg->c_ChirpTxMimoPatSel = M_RL_SENS_TX_MIMO_PATRN_DIS;

    ptrProfileCfg->c_MiscSettings = 0U; /* HPF FINIT, CRD ena, PA blank dis */
    ptrProfileCfg->c_HpfFastInitDuration = 15U; /* 1.5us */
    ptrProfileCfg->h_CrdNSlopeMag = 0x800U; /* default slope */

    //ptrProfileCfg->h_ChirpRampEndTime = 200U; /* 4us high res */
    ptrProfileCfg->h_ChirpRampEndTime = 250U; /* 25us low res */
    ptrProfileCfg->c_ChirpRxHpfSel = M_RL_SENS_RX_HPF_SEL_350KHZ;

    /*ptrProfileCfg->c_ChirpRxGainSel = (36U | \
                    (M_RL_SENS_RF_GAIN_TARG_1 << M_RL_SENS_RF_GAIN_OFFSET));
    ptrProfileCfg->c_ChirpTxBackOffSel[0] = 0U;
    ptrProfileCfg->c_ChirpTxBackOffSel[1] = 0U;*/

    /* Initialize the profile time configuration: */
    memset ((void*)ptrProfileTimeCfg, 0, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG));

    //ptrProfileTimeCfg->h_ChirpIdleTime = 325U; /* 6.5us high res */
    ptrProfileTimeCfg->h_ChirpIdleTime = 65U; /* 6.5us low res */
    ptrProfileTimeCfg->h_ChirpAdcStartTime = ((UINT16)25U | \
            ((UINT16)5U << M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET)); /* 0.5us Fract + 0.4us skip */
    ptrProfileTimeCfg->xh_ChirpTxStartTime = 0;
    ptrProfileTimeCfg->xh_ChirpRfFreqSlope = 3495; /* 100MHz/us , 77G - 2621 */
    //ptrProfileTimeCfg->w_ChirpRfFreqStart  = M_RL_SENS_CHIRP_RFFREQ_HR_57G; /* 57GHz / 76GHz High */
    #ifdef SOC_XWRL64XX
        ptrProfileTimeCfg->w_ChirpRfFreqStart  = M_RL_SENS_CHIRP_RFFREQ_LR_57G; /* 57GHz*/
    #else
        ptrProfileTimeCfg->w_ChirpRfFreqStart  = M_RL_SENS_CHIRP_RFFREQ_LR_76G; /* 76GHz low */
    #endif
    ptrProfileTimeCfg->h_ChirpTxEnSel = 0x3U; /* 2 TX enable in chirp */
    ptrProfileTimeCfg->h_ChirpTxBpmEnSel = 0x2U; /* TX1 BPM enable in chirp */
}



static void populateDefaultChirpCfg (T_RL_API_SENS_PER_CHIRP_CFG* ptrChirpCfg, T_RL_API_SENS_PER_CHIRP_CTRL* ptrChirpCtrl)
{
    /* Initialize the chirp configuration: */
    memset ((void*)ptrChirpCfg, 0, sizeof(T_RL_API_SENS_PER_CHIRP_CFG));

    /* Populate the chirp configuration: */
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_START] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_IDLE_TIME] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_START_TIME] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_ENABLE] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = 4;

    /* repeat count is not applicable for acc chirps, so new chirp param is picked after 2*2 chirps */
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_START] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_IDLE_TIME] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_START_TIME] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_ENABLE] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = 1;

    /* Initialize the chirp control configuration: */
    memset ((void*)ptrChirpCtrl, 0, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL));

    /* Sensor per chirp control api */
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_START] = \
        ((UINT32)(&sensPerChirpLuTable->StartFreqLowRes[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpSlope[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_IDLE_TIME] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpIdleTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpAdcStartTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_START_TIME] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpTxStartTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_ENABLE] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpTxEn[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpBpmEn[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);

    ptrChirpCtrl->h_PerChirpParamCtrl = M_RL_SENS_PER_CHIRP_CTRL_MAX;
}



void populateDefaultOpenCfg (MMWave_OpenCfg* ptrOpenCfg)
{
    ptrOpenCfg->useRunTimeCalib = false;
    ptrOpenCfg->useCustomCalibration = false;
    ptrOpenCfg->customCalibrationEnableMask = 0U;
}



void populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg)
{
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG      profileCfg;
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG      profileTimeCfg;
    T_RL_API_SENS_PER_CHIRP_CFG        chirpCfg;
    T_RL_API_SENS_PER_CHIRP_CTRL       chirpCtrl;
    int32_t             errCode;
    MMWave_ChirpHandle  chirpHandle;

    /* Initialize the control configuration: */
    memset ((void*)ptrCtrlCfg, 0, sizeof(MMWave_CtrlCfg));

    /* Populate the profile configuration: */
    populateDefaultProfileCfg (&profileCfg, &profileTimeCfg);

    /* Create the profile: */
    ptrCtrlCfg->frameCfg[0].profileHandle[0] = MMWave_addProfile (sensorHandle, &profileCfg, &profileTimeCfg, &errCode);
    if (ptrCtrlCfg->frameCfg[0].profileHandle[0] == NULL)
    {
        DebugP_logError ("Error: Unable to add the profile [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("MMWave Add Profile", MCPI_TestResult_PASS);

    /**************************************************************************************************
     * Unit Test: Verify the Full Configuration Profile API
     **************************************************************************************************/
    {
        T_RL_API_SENS_CHIRP_PROF_COMN_CFG          profileCfgTmp;
        T_RL_API_SENS_CHIRP_PROF_TIME_CFG          profileTimeCfgTmp;
        uint32_t                numProfiles;
        MMWave_ProfileHandle    tmpProfileHandle;

        /* Verify the number of profiles */
        if (MMWave_getNumProfiles (sensorHandle, &numProfiles, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the number of profiles [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Get Number Profile", MCPI_TestResult_FAIL);
            return;
        }
        if (numProfiles != 1U)
        {
            DebugP_logError ("Error: Invalid number of profiles detected [%d]\n", numProfiles);
            MCPI_setFeatureTestResult ("MMWave Get Number Profile", MCPI_TestResult_FAIL);
        }
        MCPI_setFeatureTestResult ("MMWave Get Number Profile", MCPI_TestResult_PASS);

        /* Get the profile handle: */
        if (MMWave_getProfileHandle (sensorHandle, 0U, &tmpProfileHandle, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the profile handle [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Get Profile Handle", MCPI_TestResult_FAIL);
        }
        if (tmpProfileHandle != ptrCtrlCfg->frameCfg[0].profileHandle[0])
        {
            DebugP_logError ("Error: Invalid profile handle detected\n");
            MCPI_setFeatureTestResult ("MMWave Get Profile Handle", MCPI_TestResult_FAIL);
        }
        MCPI_setFeatureTestResult ("MMWave Get Profile Handle", MCPI_TestResult_PASS);

        /* Get the profile configuration */
        if (MMWave_getProfileCfg (ptrCtrlCfg->frameCfg[0].profileHandle[0], &profileCfgTmp, &profileTimeCfgTmp, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the profile configuration [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Get Profile", MCPI_TestResult_FAIL);
            return;
        }
        if (memcmp ((void*)&profileCfg, (void*)&profileCfgTmp, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG)) != 0)
        {
            DebugP_logError ("Error: Invalid profile configuration detected\n");
            MCPI_setFeatureTestResult ("MMWave Get Profile", MCPI_TestResult_FAIL);
            return;
        }

        if (memcmp ((void*)&profileTimeCfg, (void*)&profileTimeCfgTmp, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG)) != 0)
        {
            DebugP_logError ("Error: Invalid profile time configuration detected\n");
            MCPI_setFeatureTestResult ("MMWave Get Profile", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Get Profile", MCPI_TestResult_PASS);
    }

    /* Populate the default chirp configuration */
    populateDefaultChirpCfg (&chirpCfg, &chirpCtrl);

    /* Add the chirp to the profile: */
    chirpHandle = MMWave_addChirp (ptrCtrlCfg->frameCfg[0].profileHandle[0], &chirpCfg, &chirpCtrl, &errCode);
    if (chirpHandle == NULL)
    {
        DebugP_logError ("Error: Unable to add the chirp [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave Add Chirp", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("MMWave Add Chirp", MCPI_TestResult_PASS);

    /**************************************************************************************************
     * Unit Test: Verify the Full Configuration Chirp API
     **************************************************************************************************/
    {
        T_RL_API_SENS_PER_CHIRP_CFG        chirpCfgTmp;
        T_RL_API_SENS_PER_CHIRP_CTRL       chirpCtrlTmp;
        uint32_t            numChirps;
        MMWave_ChirpHandle  chirpHandleTmp;

        /* Get the number of chirps attached to the profile */
        if (MMWave_getNumChirps (ptrCtrlCfg->frameCfg[0].profileHandle[0], &numChirps, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the number of chirps [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Get Number of Chirps", MCPI_TestResult_FAIL);
            return;
        }
        if (numChirps != 1U)
        {
            DebugP_logError ("Error: Invalid number of chirps detected [%d]\n", numChirps);
            MCPI_setFeatureTestResult ("MMWave Get Number of Chirps", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Get Number of Chirps", MCPI_TestResult_PASS);

        /* Get the Chirp Handle */
        if (MMWave_getChirpHandle (ptrCtrlCfg->frameCfg[0].profileHandle[0], 1U, &chirpHandleTmp, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the chirp handle [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Get Chirp Handle", MCPI_TestResult_FAIL);
            return;
        }
        if (chirpHandleTmp != chirpHandle)
        {
            DebugP_logError ("Error: Chirp handle validation failed [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Get Chirp Handle", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Get Chirp Handle", MCPI_TestResult_PASS);

        /* Get the chirp configuration */
        if (MMWave_getChirpCfg (chirpHandle, &chirpCfgTmp, &chirpCtrlTmp, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the profile configuration [Error code %d]\n", errCode);
            MCPI_setFeatureTestResult ("MMWave Get Chirp", MCPI_TestResult_FAIL);
            return;
        }
        if (memcmp ((void*)&chirpCfg, (void*)&chirpCfgTmp, sizeof(T_RL_API_SENS_PER_CHIRP_CFG)) != 0)
        {
            DebugP_logError ("Error: Invalid chirp configuration detected\n");
            MCPI_setFeatureTestResult ("MMWave Get Chirp Configuration", MCPI_TestResult_FAIL);
            return;
        }

        if (memcmp ((void*)&chirpCtrl, (void*)&chirpCtrlTmp, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL)) != 0)
        {
            DebugP_logError ("Error: Invalid chirp configuration detected\n");
            MCPI_setFeatureTestResult ("MMWave Get Chirp Configuration", MCPI_TestResult_FAIL);
            return;
        }
        MCPI_setFeatureTestResult ("MMWave Get Chirp Configuration", MCPI_TestResult_PASS);
    }

    /* Populate the frame configuration: */
    ptrCtrlCfg->frameCfg[0].frameCfg.h_NumOfChirpsInBurst = 2U;
    ptrCtrlCfg->frameCfg[0].frameCfg.c_NumOfChirpsAccum = 0U;
   //ptrCtrlCfg->frameCfg[0].frameCfg.w_BurstPeriodicity = 2500U; /* 8 chirps = 40us + 10us idle */
    ptrCtrlCfg->frameCfg[0].frameCfg.w_BurstPeriodicity = 3480U; /* 4 chirps = 148us + 200us idle , (12 + 25) chirp*/
    ptrCtrlCfg->frameCfg[0].frameCfg.h_NumOfBurstsInFrame = 2U;
    ptrCtrlCfg->frameCfg[0].frameCfg.w_FramePeriodicity = 29440U; /* 2 bursts = 696us + 40us idle, 40M XTAL */
    ptrCtrlCfg->frameCfg[0].frameCfg.h_NumOfFrames = 2U;
    ptrCtrlCfg->frameCfg[0].frameCfg.w_FrameEvent0TimeCfg = \
        (ptrCtrlCfg->frameCfg[0].frameCfg.w_FramePeriodicity - 240U); /* frame period - 6us */
    ptrCtrlCfg->frameCfg[0].frameCfg.w_FrameEvent1TimeCfg = \
        (ptrCtrlCfg->frameCfg[0].frameCfg.w_FramePeriodicity - 120U); /* frame period - 3us */

    return;
}


void populateDefaultCalibrationCfg (MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    ptrCalibrationCfg->chirpCalibrationCfg.enableCalibration    = false;
    ptrCalibrationCfg->chirpCalibrationCfg.enablePeriodicity    = false;
    ptrCalibrationCfg->chirpCalibrationCfg.periodicTimeInFrames = 10U;
    return;
}

void openSensor(){
    populateDefaultOpenCfg (&openCfg);
    
    if (MMWave_open (sensorHandle, &openCfg, &errCode) < 0)
    {
        // Error
        DebugP_logError ("Error: Sensor open failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("Sensor Open", MCPI_TestResult_FAIL);
        return;
    }
   DebugP_logInfo("Sensor Open.\r\n");
}

void configSensor(){
    populateDefaultChirpControlCfg (&ctrlCfg);

    if (MMWave_config (sensorHandle, &ctrlCfg, &errCode) < 0)
    {
        // Error
        DebugP_logError ("Error: Sensor configuration failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("Sensor Configuration", MCPI_TestResult_FAIL);
        return;
    }
    DebugP_logInfo("Sensor Config Success.\r\n");

    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    populateDefaultCalibrationCfg (&calibrationCfg);

    // Configuring the Start Parameter
    strtCfg.frameTrigMode = M_RL_SENS_FRAME_SW_TRIG; // SW trigger
    strtCfg.chirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS; // Disable LoopBack
    strtCfg.frameLivMonEn = 0; //Disable all Live Monitors
    strtCfg.frameTrigTimerVal = 0;
}


void startSensor(){
    if (MMWave_start (sensorHandle, &calibrationCfg,&strtCfg, &errCode) < 0)
    {
        // Error
        DebugP_logError ("Error: Sensor start failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("Sensor Start", MCPI_TestResult_FAIL);
        return;
    }
   
    DebugP_logInfo  ("Sensor started.\r\n");
}

void stopSensor(){
    retVal = MMWave_stop (sensorHandle, &errCode);
    if (retVal < 0)
    {
        // Error
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        DebugP_logError ("Error Level: %s Sensor: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            MCPI_setFeatureTestResult ("Sensor Stop", MCPI_TestResult_FAIL);
            return;
        }
    }
    DebugP_logInfo ("Sensor Stop done.\r\n");
}

void closeSensor(){
    if (MMWave_close (sensorHandle, &errCode) < 0)
    {
        DebugP_logError ("Error: Sensor failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("Sensor Close", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("Sensor Close", MCPI_TestResult_PASS);
    DebugP_logInfo ("Sensor close done.\r\n");
    
}

void deInitSensor(){
    if (MMWave_deinit(sensorHandle, &errCode) < 0)
    {
        DebugP_logError ("Error: Sensor Deinitialization failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("Sensor Deinitialized", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("Sensor Deinitialized", MCPI_TestResult_PASS);
    return;
    DebugP_logInfo ("Sensor deinit done.\r\n");
}

void executeSensor()
{

    if (MMWave_execute (sensorHandle, &errCode) < 0)
        DebugP_logError ("Error: mmWaSensor control execution failed [Error code %d]\n", errCode);
    
}

void testSensor(){
    
    MCPI_Initialize();

    openSensor();
    configSensor();
    startSensor();
    uint32_t start = ClockP_getTimeUsec();
    while(1){
        executeSensor();
        if ((ClockP_getTimeUsec() - start) > 1000*1000) break;
    }

    stopSensor();
    closeSensor();
    deInitSensor();

}