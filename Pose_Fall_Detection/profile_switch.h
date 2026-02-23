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

#ifndef PROFILE_SWITCH_H_
#define PROFILE_SWITCH_H_

#include "motion_detect.h"

/**
 * @brief
 *  MmwDemo_PresenceSwitch_Config
 *
 * @details
 *  This structure is used to store all presence detect
 *  specific configurations, to support reconfig for
 * switching based on the Profile Switch state machine.
 */
typedef struct MmwDemo_PresenceSwitch_Config_t
{
    /**
     * @brief RF power ON/OFF config command data structure in presence mode
     *
     */
    T_RL_API_FECSS_RF_PWR_CFG_CMD channelCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG profileComCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG profileTimeCfg;

    /**
     * @brief MMWave configuration tracked by the module in presence mode
     */
    T_RL_API_SENS_FRAME_CFG frameCfg;


    /**
     * @brief Signal Chain CFG
     *
     */
    CLI_sigProcChainCfg sigProcChainCfg;

    /**
     * @brief Cfar Cfg
     *
     */
    DPU_CFARProc_CfarCfg cfarCfg;

    /**
     * @brief Fov Cfg
     *
     */
    DPU_CFARProc_AoaFovCfg fovCfg;

    /**
     * @brief Gui Monitor Sel
     *
     */
    CLI_GuiMonSel guiMonSel;

    /**
     * @brief Range Select Cfg
     *
     */
    DPU_CFARProc_RangeFovCfg rangeSelCfg;

    /**
     * @brief DPC Object detection: enabled flag: 1-enabled 0-disabled
     *
     */
    uint32_t staticClutterRemovalEnable;

    /**
     * @brief Motion mode configuration based state parameters - major and minor
     *
     */
    mpdProc_MotionModeStateParamCfg majorStateParamCfg, minorStateParamCfg;

    /**
     * @brief Clustering configuration parameters
     *
     */
    mpdProc_ClusterParamCfg clusterParamCfg;

    /**
     * @brief Scenary params - sensor position, boundary of interest
     *
     */
    mpdProc_SceneryParams sceneryParams;

    /*! @brief Flag set to 1 if Major Motion Detection mode is enabled  */
    uint8_t enableMajorMotion;

    /*! @brief Start frequency in GHz */
    float startFreq;

    /*! @brief Chirp slope in MHz/usec */
    float chirpSlope;

    /*! @brief Burst period in msec */
    float burstPeriod;
} MmwDemo_PresenceSwitch_Config;


/**
 * @brief
 *  MmwDemo_TrackerSwitch_Config
 *
 * @details
 *  This structure is used to store all tracker processing
 *  specific configurations, to support reconfig for switching
 *  based on the Profile Switch state machine.
 */
typedef struct MmwDemo_TrackerSwitch_Config_t
{
    /**
     * @brief RF power ON/OFF config command data structure in presence mode
     *
     */
    T_RL_API_FECSS_RF_PWR_CFG_CMD channelCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG profileComCfg;

    /**
     * @brief Sensor chirp profile common config command data structure in presence mode
     *
     */
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG profileTimeCfg;

    /**
     * @brief MMWave configuration tracked by the module in presence mode
     */
    T_RL_API_SENS_FRAME_CFG frameCfg;

    /**
     * @brief Signal Chain CFG
     *
     */
    CLI_sigProcChainCfg sigProcChainCfg;

    /**
     * @brief Cfar Cfg
     *
     */
    DPU_CFARProc_CfarCfg cfarCfg;

    /**
     * @brief Fov Cfg
     *
     */
    DPU_CFARProc_AoaFovCfg fovCfg;

    /*! @brief   Tracker DPU Static Configuration */
    DPU_TrackerProc_Config trackerCfg;

    /**
     * @brief Micro Doppler DPU configuration
     *
     */
    DPU_uDopProcCliCfg microDopplerCliCfg;

    /**
     * @brief Gui Monitor Sel
     *
     */
    CLI_GuiMonSel guiMonSel;

    /**
     * @brief Micro Doppler Classifier configuration
     *
     */
    DPU_uDopClassifierCliCfg microDopplerClassifierCliCfg;

    /**
     * @brief Range Select Cfg
     *
     */
    DPU_CFARProc_RangeFovCfg rangeSelCfg;

    /**
     * @brief DPC Object detection: enabled flag: 1-enabled 0-disabled
     *
     */
    uint32_t staticClutterRemovalEnable;

    /*! @brief Flag set to 1 if Major Motion Detection mode is enabled  */
    uint8_t enableMajorMotion;

    /*! @brief Start frequency in GHz */
    float startFreq;

    /*! @brief Chirp slope in MHz/usec */
    float chirpSlope;

    /*! @brief Burst period in msec */
    float burstPeriod;

} MmwDemo_TrackerSwitch_Config;


#endif /* PROFILE_SWITCH_H_ */
