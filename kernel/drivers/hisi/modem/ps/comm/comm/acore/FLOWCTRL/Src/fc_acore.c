/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
 * foss@huawei.com
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 and
 * * only version 2 as published by the Free Software Foundation.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) Neither the name of Huawei nor the names of its contributors may
 * *    be used to endorse or promote products derived from this software
 * *    without specific prior written permission.
 *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include "product_config.h"
#if (FEATURE_ACPU_FC_POINT_REG == FEATURE_ON)
#include "fc.h"
#include "fc_interface.h"
#include "fc_cds_interface.h"
#include "fc_cst_interface.h"
#include "at_fc_interface.h"
#include "ads_fc_interface.h"
#include "acpuload_interface.h"
#include "imm_interface.h"
#include "nfext_interface.h"
#include "fc_intra_msg.h"
#include "PsTypeDef.h"
#include "ps_common_def.h"
#include "ps_lib.h"
#include "mdrv_nvim.h"
#include "fc_acore_c_reset.h"
#include "ttf_util.h"
#include "fc_acore.h"
#include "ttf_comm.h"
#include "gucttf_tag.h"
#include "securec.h"
#define THIS_FILE_ID PS_FILE_ID_ACORE_FLOW_CTRL_C
#define THIS_MODU mod_fc

/* A?????????????? */
FC_AcoreCfgNv g_fcAcoreCfg;

/* ?????????????????????? */
FC_BridgeRate g_fcBridgeRate;

/* CDS????????????????????FC IdRAB???????? */
FC_RabMappingInfoSet g_fcRabMappingInfoSet[MODEM_ID_BUTT];

/* CPU?????????????????????????????? */
FC_CpuCtrl g_fcCpuaCtrl;

FC_CpuDrvAssemConfigPara g_cpuDriverAssePara =
    {
        6,
        10,
        5,
        0,
        {
            /* cpuload, PC??????????????UE????TX????????, UE????TX??????UE????RX??????????UE????RX???????????????? */
            {
                85, { 20, 40, 10, 10, 10, 56, { 0, 0 }}
            },
            {
                70, { 15, 20, 10, 10, 10, 20, { 0, 0 }}
            },
            {
                60, { 10, 10, 10, 10, 10, 5, { 0, 0 }}
            },
            {
                40, { 5, 5, 10, 10, 10, 1, { 0, 0 }}
            },
            {
                0, { 0, 1, 1, 1, 1, 1, { 0, 0 }}
            }
        }
    };

/* ???????????????????? */
FC_CpuDrvAssemParaEntity g_drvAssemParaEntity = { 0, 0, 0, 0, 0, { 0, 0, 0, 0 }, { 100, { 0, 0, 0, 0, 0, 0, { 0, 0 }}},
    VOS_NULL_PTR, VOS_NULL_PTR };

VOS_SPINLOCK g_fcMemSpinLock;

STATIC VOS_VOID FC_BridgeResetRate(VOS_VOID);
#if (FEATURE_NFEXT == FEATURE_ON)
STATIC VOS_UINT32 FC_BridgeGetRate(VOS_VOID);
STATIC VOS_UINT32 FC_RmRateJudge(VOS_VOID);
#endif
STATIC VOS_VOID FC_DrvAssemInit(VOS_VOID);
STATIC VOS_VOID FC_JudgeAssemSmoothFactor(FC_CpuDrvAssemPara *drvAssemPara);
STATIC FC_CpuDrvAssemPara* FC_GetCpuDrvAssemPara(VOS_UINT32 lev);
STATIC FC_CpuDrvAssemPara* FC_GetCurrentAssemPara(VOS_UINT32 assemLev);
STATIC VOS_VOID FC_JudgeDrvAssemAction(VOS_UINT32 assemLev);

STATIC FC_CpuDrvAssemPara* FC_SelectDrvAssemParaRule(VOS_UINT32 cpuLoad, VOS_UINT32 *assemLev);
STATIC VOS_VOID FC_JudgeDrvAssemPara(VOS_UINT32 cpuLoad);

STATIC VOS_UINT32 FC_PsRateJudge(VOS_VOID);
STATIC VOS_VOID FC_GetPsRate(VOS_UINT32 *ulRate, VOS_UINT32 *dlRate);
STATIC VOS_UINT32 FC_CpuaDownJudge(VOS_UINT32 cpuLoad, FC_CFG_CPU_STRU *fcCfgCpu, FC_Policy *fcPolicy);

VOS_UINT32 FC_CpuaStopFcAttempt(VOS_UINT32 param1, VOS_UINT32 param2);

STATIC VOS_UINT32 FC_CpuaUpProcess(VOS_VOID);
STATIC VOS_UINT32 FC_CpuaDownProcess(VOS_VOID);
STATIC VOS_UINT32 FC_CpuaStopFlowCtrl(VOS_VOID);

STATIC VOS_UINT32 FC_CpuaInit(VOS_VOID);

STATIC VOS_UINT32 FC_MEM_AdjustPriForDown(FC_PriTypeUint8 pointPri, FC_IdUint8 fcId);

STATIC VOS_UINT32 FC_MEM_SndUpToTargetPriIndMsg(FC_PriTypeUint8 targetPri, VOS_UINT16 memFreeCnt);
STATIC VOS_UINT32 FC_MEM_SndDownToTargetPriIndMsg(FC_PriTypeUint8 targetPri, VOS_UINT16 memFreeCnt);

STATIC VOS_UINT32 FC_CstUpProcess(VOS_UINT8 rabId);
STATIC VOS_UINT32 FC_CstDownProcess(VOS_UINT8 rabId);
STATIC FC_RabMappingInfo* FC_CdsGetFcInfo(VOS_UINT8 rabId, ModemIdUint16 modemId);

STATIC VOS_UINT32 FC_CdsInit(VOS_VOID);

STATIC VOS_VOID FC_UpProcess(VOS_RatModeUint32 rateMode);
STATIC VOS_VOID FC_DownProcess(VOS_RatModeUint32 rateMode);

STATIC VOS_UINT32 FC_RcvCstMsg(MsgBlock *msg);
STATIC VOS_UINT32 FC_RcvCdsMsg(MsgBlock *msg);
STATIC VOS_UINT32 FC_AcoreRcvTimerMsg(REL_TimerMsgBlock *timerMsg);
#if (FEATURE_UE_MODE_CDMA == FEATURE_ON)
STATIC VOS_UINT32 FC_CdmaDownProcess(VOS_VOID);
STATIC VOS_UINT32 FC_CdmaUpProcess(VOS_VOID);
#endif
STATIC VOS_UINT32 FC_GprsDownProcess(VOS_VOID);

STATIC VOS_UINT32 FC_GprsUpProcess(VOS_VOID);

#if (FEATURE_NFEXT == FEATURE_ON)
#if (FEATURE_ACPU_STAT == FEATURE_ON)
STATIC VOS_VOID FC_BridgeCalcRate(VOS_UINT32 period)
{
    VOS_UINT32 currentByteCnt;
    VOS_UINT32 lastByteCnt;
    VOS_UINT32 rate;
    VOS_UINT32 deltaPacketCnt;

    if (period == 0) {
        g_fcBridgeRate.rate = 0;
        return;
    }

    lastByteCnt    = g_fcBridgeRate.lastByteCnt;
    currentByteCnt = NFEXT_GetBrBytesCnt();
    deltaPacketCnt = (currentByteCnt - lastByteCnt);

    /* ??????bps,???????????????? */
    if (deltaPacketCnt < period) {
        rate = (deltaPacketCnt * 1000 * 8) / period;
    } else {
        rate = ((deltaPacketCnt * 8) / period) * 1000;
    }

    g_fcBridgeRate.lastByteCnt = currentByteCnt;
    g_fcBridgeRate.rate        = rate;

    return;
}
#endif

/* ???????????????? */
STATIC VOS_VOID FC_BridgeResetRate(VOS_VOID)
{
    g_fcBridgeRate.lastByteCnt = NFEXT_GetBrBytesCnt();
    g_fcBridgeRate.rate        = 0;
}

STATIC VOS_UINT32 FC_BridgeGetRate(VOS_VOID)
{
    return (g_fcBridgeRate.rate);
}

/* Rm????????????????CPU?????????? */
STATIC VOS_UINT32 FC_RmRateJudge(VOS_VOID)
{
    /* ????????????????????????????????????????CPU?? */
    if (g_fcBridgeRate.rate > g_fcAcoreCfg.fcCfgCpuA.rmRateThreshold) {
        return VOS_TRUE;
    }

    return VOS_FALSE;
}
#else

/* ???????????????? */
STATIC VOS_VOID FC_BridgeResetRate(VOS_VOID)
{
    return;
}

/* ????????????????????CPU?????????? */
STATIC VOS_UINT32 FC_UmRateOverThreshold(VOS_VOID)
{
    VOS_UINT32 ulRate = 0;
    VOS_UINT32 dlRate = 0;

    /* ????UM??????, ADS?????????????????????????????????????????????? */

    /* ??????????????????????????????????????????????CPU?? */
    if ((ulRate > g_fcAcoreCfg.fcCfgCpuA.umUlRateThreshold) || (dlRate > g_fcAcoreCfg.fcCfgCpuA.umDlRateThreshold)) {
        return VOS_TRUE;
    }

    return VOS_FALSE;
}

#endif

/* ???????????????????????????? */
VOS_UINT32 FC_AcoreRegDrvAssemFunc(FC_AcoreDrvAssembleParaFunc fcDrvSetAssemParaFuncUe,
    FC_AcoreDrvAssembleParaFunc fcDrvSetAssemParaFuncPc)
{
    /* ADS???????????????????????????????????????????? */
    return VOS_OK;
}


/* ??????????????????????NV?????????? */
STATIC VOS_VOID FC_DrvAssemInit(VOS_VOID)
{
    FC_CpuDrvAssemParaNv cpuDrvAssemPara = {0};
    VOS_UINT32           rst;

    rst = GUCTTF_NV_Read(
        MODEM_ID_0, NV_ITEM_FC_ACPU_DRV_ASSEMBLE_PARA, &cpuDrvAssemPara, (VOS_UINT32)sizeof(FC_CpuDrvAssemParaNv));

    /* NV?????????????????????? */
    if (rst != NV_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_DrvAssemInit Read NV fail!\n");

        return;
    }

    g_drvAssemParaEntity.curAssemPara.cpuLoad = 100;
    g_drvAssemParaEntity.curLev               = FC_ACPU_DRV_ASSEM_LEV_1;
    g_drvAssemParaEntity.setDrvFailCnt        = 0;

    /* NV??????????????????????????????????NV?????? */
    TTF_ACORE_SF_CHK(
        memcpy_s(&g_cpuDriverAssePara, sizeof(g_cpuDriverAssePara), &cpuDrvAssemPara, sizeof(cpuDrvAssemPara)));
}

/* ???????????? */
STATIC VOS_VOID FC_JudgeAssemSmoothFactor(FC_CpuDrvAssemPara *drvAssemPara)
{
    /* ???????????????????????????????????? */
    if (g_drvAssemParaEntity.curAssemPara.cpuLoad > drvAssemPara->cpuLoad) {
        /* ???????????? */
        g_drvAssemParaEntity.smoothCntDown++;
        g_drvAssemParaEntity.smoothCntUp = 0;
    } else if (g_drvAssemParaEntity.curAssemPara.cpuLoad < drvAssemPara->cpuLoad) {
        /* ???????????? */
        g_drvAssemParaEntity.smoothCntUp++;
        g_drvAssemParaEntity.smoothCntDown = 0;
    } else {
        /* ?????????????????????????????????????????? */
        g_drvAssemParaEntity.smoothCntUp   = 0;
        g_drvAssemParaEntity.smoothCntDown = 0;
    }
}

/* ???????????????????? */
STATIC FC_CpuDrvAssemPara* FC_GetCpuDrvAssemPara(VOS_UINT32 lev)
{
    if (lev >= FC_ACPU_DRV_ASSEM_LEV_BUTT) {
        return VOS_NULL_PTR;
    }

    return &g_cpuDriverAssePara.cpuDrvAssemPara[lev];
}

/* ????????ASSEM???? */
STATIC FC_CpuDrvAssemPara* FC_GetCurrentAssemPara(VOS_UINT32 assemLev)
{
    FC_CpuDrvAssemPara *drvAssemPara = VOS_NULL_PTR;

    /* ?????????????????? */
    if (g_drvAssemParaEntity.smoothCntUp >= g_cpuDriverAssePara.smoothCntUpLev) {
        g_drvAssemParaEntity.curLev = assemLev;
    } else if (g_drvAssemParaEntity.smoothCntDown >= g_cpuDriverAssePara.smoothCntDownLev) { /* ???????????????????????????? */
        if (g_drvAssemParaEntity.curLev < (FC_ACPU_DRV_ASSEM_LEV_BUTT - 1)) {
            g_drvAssemParaEntity.curLev++;
        }
    } else {
        return VOS_NULL_PTR;
    }

    drvAssemPara = FC_GetCpuDrvAssemPara(g_drvAssemParaEntity.curLev);

    return drvAssemPara;
}

/* ???????????????????? */
VOS_UINT32 FC_MNTN_TraceDrvAssemPara(FC_DrvAssemPara *drvAssenPara)
{
    FC_MntnDrvAssemPara fcMntnDrvAssemPara;

    if (drvAssenPara == VOS_NULL_PTR) {
        return VOS_ERR;
    }

    (VOS_VOID)memset_s(&fcMntnDrvAssemPara, sizeof(FC_MntnDrvAssemPara), 0, sizeof(FC_MntnDrvAssemPara));
    TTF_SET_MSG_SENDER_ID(&fcMntnDrvAssemPara, UEPS_PID_FLOWCTRL);
    TTF_SET_MSG_RECEIVER_ID(&fcMntnDrvAssemPara, UEPS_PID_FLOWCTRL);
    TTF_SET_MSG_LEN(&fcMntnDrvAssemPara, sizeof(FC_MntnDrvAssemPara) - VOS_MSG_HEAD_LENGTH);

    fcMntnDrvAssemPara.msgName = ID_FC_MNTN_DRV_ASSEM_PARA;
    (VOS_VOID)memcpy_s(
        &fcMntnDrvAssemPara.drvAssemPara, sizeof(FC_DrvAssemPara), drvAssenPara, sizeof(FC_DrvAssemPara));

    FC_MNTN_TraceEvent(&fcMntnDrvAssemPara);

    return VOS_OK;
}

/* ?????????????????? */
STATIC VOS_VOID FC_DoJudgeDrvAssem(FC_CpuDrvAssemPara *drvAssemPara)
{
    /* ???????????? */
    if ((FC_ACPU_DRV_ASSEM_PC_ON_MASK & g_cpuDriverAssePara.enableMask) == FC_ACPU_DRV_ASSEM_PC_ON_MASK) {
        if (g_drvAssemParaEntity.drvSetAssemParaFuncPc != VOS_NULL_PTR) {
            /* ??????????????????????????????????????PC?????????? */
            if (g_drvAssemParaEntity.drvSetAssemParaFuncPc(&(drvAssemPara->drvAssemPara)) != VOS_OK) {
                FC_LOG(PS_PRINT_ERROR, "Set Driver Assemble parameter fail!\n");
                /* ???????????? */
                g_drvAssemParaEntity.setDrvFailCnt++;

                return;
            }
        }
    }

    /* ???????????? */
    g_drvAssemParaEntity.smoothCntUp   = 0;
    g_drvAssemParaEntity.smoothCntDown = 0;

    /* ???????????????? */
    g_drvAssemParaEntity.setDrvFailCnt = 0;

    /* ???????????????? */
    (VOS_VOID)memcpy_s(&g_drvAssemParaEntity.curAssemPara, sizeof(g_drvAssemParaEntity.curAssemPara), drvAssemPara,
        sizeof(FC_CpuDrvAssemPara));

    /* ???????? */
    FC_MNTN_TraceDrvAssemPara(&(drvAssemPara->drvAssemPara));
}

/* ?????????????? */
STATIC VOS_VOID FC_JudgeDrvAssemAction(VOS_UINT32 assemLev)
{
    FC_CpuDrvAssemPara *drvAssemPara = VOS_NULL_PTR;

    drvAssemPara = FC_GetCurrentAssemPara(assemLev);

    /* ???????????????????????????????? */
    if (drvAssemPara != VOS_NULL_PTR) {
        FC_DoJudgeDrvAssem(drvAssemPara);
    }
}

/* ???????????????????? */
VOS_VOID FC_JudgeDrvToMaxPara(VOS_VOID)
{
    FC_CpuDrvAssemPara *drvAssemPara = VOS_NULL_PTR;

    /* ?????????????????? */
    g_drvAssemParaEntity.curLev = FC_ACPU_DRV_ASSEM_LEV_1;

    drvAssemPara = FC_GetCpuDrvAssemPara(FC_ACPU_DRV_ASSEM_LEV_1);

    FC_DoJudgeDrvAssem(drvAssemPara);
}

/* ????CPULOAD?????????????????? */
STATIC FC_CpuDrvAssemPara* FC_SelectDrvAssemParaRule(VOS_UINT32 cpuLoad, VOS_UINT32 *assemLev)
{
    FC_CpuDrvAssemPara *cpuDrvAssemParaRst = VOS_NULL_PTR;
    FC_CpuDrvAssemPara *cpuDrvAssemPara    = VOS_NULL_PTR;
    VOS_INT             i;

    /* ???????????????????? */
    for (i = 0; i < FC_ACPU_DRV_ASSEM_LEV_BUTT; i++) {
        cpuDrvAssemPara = &g_cpuDriverAssePara.cpuDrvAssemPara[i];
        if (cpuLoad >= cpuDrvAssemPara->cpuLoad) {
            cpuDrvAssemParaRst = cpuDrvAssemPara;
            *assemLev          = (VOS_UINT32)i;

            break;
        }
    }

    return cpuDrvAssemParaRst;
}

/* ????CPULOAD???????????????????????? */
STATIC VOS_VOID FC_JudgeDrvAssemPara(VOS_UINT32 cpuLoad)
{
    FC_CpuDrvAssemPara *drvAssemPara = VOS_NULL_PTR;
    /* ?????????? */
    VOS_UINT32 assemLev = FC_ACPU_DRV_ASSEM_LEV_1;

    /* ?????????????????????????????????? */
    if (g_cpuDriverAssePara.enableMask == 0) {
        return;
    }

    if ((g_drvAssemParaEntity.setDrvFailCnt % CPU_MAX_SET_DRV_FAIL_SMOOTH_CNT) != 0) {
        g_drvAssemParaEntity.setDrvFailCnt++;

        return;
    }

    /* ????CPU???????????????????? */
    drvAssemPara = FC_SelectDrvAssemParaRule(cpuLoad, &assemLev);

    if (drvAssemPara == VOS_NULL_PTR) {
        FC_LOG(PS_PRINT_ERROR, "FC_JudgeDrvAssemPara pstDrvAssemPara is NULL!\n");

        return;
    }

    /* ???????????????????????? */
    FC_JudgeAssemSmoothFactor(drvAssemPara);

    /* ?????????????? */
    FC_JudgeDrvAssemAction(assemLev);
}

/* ?????????????? */
STATIC VOS_UINT32 FC_PsRateJudge(VOS_VOID)
{
    /* STICK????????????????E5???????????????? */
#if (FEATURE_NFEXT == FEATURE_ON)
    return (FC_RmRateJudge());
#else
    return (FC_UmRateOverThreshold());
#endif
}

/* ???????????? */
STATIC VOS_VOID FC_GetPsRate(VOS_UINT32 *ulRate, VOS_UINT32 *dlRate)
{
    /* E5???????????????????? */
#if (FEATURE_NFEXT == FEATURE_ON)
    /* ?????????????????????????????????? */
    *ulRate = FC_BridgeGetRate();
    *dlRate = *ulRate;
#else
    /* STICK???????????????????? ADS?????????????????????????????????????????????? */
    *ulRate = 0;
    *dlRate = 0;
#endif
}

/* ????????CPU???????????????????????? */
VOS_UINT32 FC_CpuaUpJudge(VOS_UINT32 cpuLoad, FC_CFG_CPU_STRU *fcCfgCpu, FC_Policy *fcPolicy)
{
    VOS_UINT32 result;

    if (cpuLoad < fcCfgCpu->cpuOverLoadVal) {
        g_fcCpuaCtrl.smoothTimerLen = 0;
        return VOS_FALSE;
    }

    /* ????????CPU???????????????????????????????? */
    if (fcPolicy->highestPri <= fcPolicy->donePri) {
        return VOS_FALSE;
    }

    /* ????????????????CPU?????????????????????????????????????????????? */
    if (fcPolicy->donePri != FC_PRI_NULL) {
        return VOS_TRUE;
    }

    if (g_fcCpuaCtrl.smoothTimerLen == 0) {
        FC_BridgeResetRate();
    }

    g_fcCpuaCtrl.smoothTimerLen++;

    if (g_fcCpuaCtrl.smoothTimerLen < fcCfgCpu->smoothTimerLen) {
        return VOS_FALSE;
    }

#if (FEATURE_ACPU_STAT == FEATURE_ON)
    /* A??ulSmoothTimerLen >= 2??????FC_CFG_CheckParam?????????? */
    FC_BridgeCalcRate((fcCfgCpu->ulSmoothTimerLen - 1) * CPULOAD_GetRegularTimerLen());
#endif

    g_fcCpuaCtrl.smoothTimerLen = 0;

    /* ??????????????CPU???????? */
    result = FC_PsRateJudge();
    if (result == VOS_FALSE) {
        return VOS_FALSE;
    }

    return VOS_TRUE;
}

/* ????????CPU???????? */
STATIC VOS_UINT32 FC_CpuaDownJudge(VOS_UINT32 cpuLoad, FC_CFG_CPU_STRU *fcCfgCpu, FC_Policy *fcPolicy)
{
    if ((cpuLoad < fcCfgCpu->cpuUnderLoadVal) && (fcPolicy->donePri > FC_PRI_NULL)) {
        return VOS_TRUE;
    }

    return VOS_FALSE;
}

/* A????????????????CPU???????????? */
VOS_VOID FC_CpuaRcvCpuLoad(VOS_UINT32 cpuLoad)
{
    FC_CFG_CPU_STRU *fcCfgCpu = VOS_NULL_PTR;
    FC_Policy       *fcPolicy = VOS_NULL_PTR;
    VOS_UINT32       startCtrl;
    VOS_UINT32       ulRate;
    VOS_UINT32       dlRate;

    /* ????CPU LOAD??????????????????????CPU???????????? */
    FC_JudgeDrvAssemPara(cpuLoad);

    /* ???????? */
    if ((FC_POLICY_MASK_CPU_A & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_CPU_A) {
        /* CPU?????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_CPU_ProcessLoad, INFO, CPU FlowCtrl is disabled %d\n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return;
    }

    /* ???????????? */
    FC_MNTN_TraceCpuLoad(ID_FC_MNTN_CPU_A_CPULOAD, cpuLoad);

    if (cpuLoad > 100) {
        /* ???????? */
        FC_LOG1(PS_PRINT_WARNING, "FC_CPU_ProcessLoad, WARNING, Invalid Cpu Load %d\n", (VOS_INT32)cpuLoad);
        return;
    }

    fcCfgCpu = &(g_fcAcoreCfg.fcCfgCpuA);
    fcPolicy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CPU_A_MODEM_0);

    /* ??????????CPU???? */
    startCtrl = FC_CpuaUpJudge(cpuLoad, fcCfgCpu, fcPolicy);
    if (startCtrl == VOS_TRUE) {
        FC_GetPsRate(&ulRate, &dlRate);
        FC_SndCpuMsg(ID_FC_CPU_A_OVERLOAD_IND, cpuLoad, ulRate, dlRate);
        return;
    }

    /* ????CPU?????????? */
    startCtrl = FC_CpuaDownJudge(cpuLoad, fcCfgCpu, fcPolicy);
    if (startCtrl == VOS_TRUE) {
        FC_GetPsRate(&ulRate, &dlRate);
        FC_SndCpuMsg(ID_FC_CPU_A_UNDERLOAD_IND, cpuLoad, ulRate, dlRate);
        return;
    }

    return;
}

/* ????????CPU???? */
VOS_UINT32 FC_CpuaStopFcAttempt(VOS_UINT32 param1, VOS_UINT32 param2)
{
    FC_CFG_CPU_STRU *cfgCpu = VOS_NULL_PTR;

    cfgCpu = &(g_fcAcoreCfg.fcCfgCpuA);
    if (cfgCpu->stopAttemptTimerLen == 0) {
        return VOS_OK;
    }

#if (FEATURE_ACPU_STAT == FEATURE_ON)
    CPULOAD_ResetUserDefLoad();
#endif

    /* ???????????????????????????????????? */
    if (g_fcCpuaCtrl.stopAttemptTHandle == VOS_NULL_PTR) {
        if (VOS_StartRelTimer(&g_fcCpuaCtrl.stopAttemptTHandle, UEPS_PID_FLOWCTRL, cfgCpu->stopAttemptTimerLen,
                FC_TIMER_STOP_CPU_ATTEMPT, 0, VOS_RELTIMER_NOLOOP, VOS_TIMER_PRECISION_0) != VOS_OK) {
            FC_LOG(PS_PRINT_WARNING, "FC_CpuaStopFcAttempt, VOS_StartRelTimer Fail!\n");
            return VOS_ERR;
        }
    }

    /* for lint 715 */
    (VOS_VOID)param1;
    (VOS_VOID)param2;

    return VOS_OK;
}

/* ACPU?????????????? */
STATIC VOS_UINT32 FC_CpuaUpProcess(VOS_VOID)
{
    FC_Policy *fcPolicy = VOS_NULL_PTR;

    /* ????CPU???????????????????????????????? */
    fcPolicy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CPU_A_MODEM_0);
    FC_PolicyUp(fcPolicy);

    /* ???????????????????????????????????????? */
    if (fcPolicy->donePri == fcPolicy->highestPri) {
        if (fcPolicy->postFunc != VOS_NULL_PTR) {
            (VOS_VOID)fcPolicy->postFunc(FC_POLICY_ID_CPU_A, 0);
        }
    }

    return VOS_OK;
}

/* ACPU?????????????????? */
STATIC VOS_UINT32 FC_CpuaDownProcess(VOS_VOID)
{
    FC_Policy *policy = VOS_NULL_PTR;

    /* ????CPU???????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CPU_A_MODEM_0);
    FC_PolicyDown(policy);

    return VOS_OK;
}

/* ????CPU??????????????????????????CPU???? */
STATIC VOS_UINT32 FC_CpuaStopFlowCtrl(VOS_VOID)
{
    FC_CFG_CPU_STRU *fcCfgCpu = VOS_NULL_PTR;
    FC_Policy       *fcPolicy = VOS_NULL_PTR;
    VOS_UINT32       cpuLoad  = 0;

    fcCfgCpu = &(g_fcAcoreCfg.fcCfgCpuA);
    fcPolicy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CPU_A_MODEM_0);

#if (FEATURE_ACPU_STAT == FEATURE_ON)
    cpuLoad = CPULOAD_GetUserDefLoad();
#endif

    /* ????CPU????????????????,??????,FEATURE_ACPU_STAT??????ulCpuLoad??0???????????????????? */
    /*lint --e(685) */
    if ((cpuLoad <= fcCfgCpu->cpuUnderLoadVal) && (fcPolicy->donePri > FC_PRI_NULL)) {
        FC_CpuaDownProcess();
    }

    return VOS_OK;
}

/* ACPU???????????????? */
STATIC VOS_UINT32 FC_CpuaInit(VOS_VOID)
{
    /* ?????????????????????????????????? */
#if (FEATURE_ACPU_STAT == FEATURE_ON)
    /* ??CPU???????????????????? */
    if (CPULOAD_RegRptHook((CPULOAD_RptHookFunc)FC_CpuaRcvCpuLoad) != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_CpuaInit, ERROR, CPULOAD_RegRptHook return error!\n");
        return VOS_ERR;
    }

    /* A??CPU?????????????? */
    g_fcPolicy[FC_POLICY_ID_CPU_A].postFunc = FC_CpuaStopFcAttempt;
#endif

    (VOS_VOID)memset_s(&g_fcBridgeRate, (VOS_UINT32)sizeof(FC_BridgeRate), 0, (VOS_UINT32)sizeof(FC_BridgeRate));
    (VOS_VOID)memset_s(&g_fcCpuaCtrl, (VOS_UINT32)sizeof(g_fcCpuaCtrl), 0, (VOS_UINT32)sizeof(g_fcCpuaCtrl));

    return VOS_OK;
}

/*
 * ??????????????????????????????????????????????????????????
 * ??????????????????????????????Lev3????
 */
FC_PriTypeUint8 FC_MEM_CalcUpTargetFcPri(FC_Policy *policy, VOS_UINT32 memValue)
{
    FC_PriTypeUint8 targetPri;
    FC_CfgMem      *memCfg = VOS_NULL_PTR;

    memCfg    = &(g_fcAcoreCfg.fcCfgMem);
    targetPri = policy->donePri;

    /* ????????????????????????????????Lev3???? */
    if (policy->priCnt == 1) {
        /* ???????????????????? */
        if (memValue <= memCfg->threshold[FC_MEM_THRESHOLD_LEV_3].setThreshold) {
            targetPri = policy->highestPri;
        }
    } else { /* ?????????????????????????? */
        /* ???????????????????? */
        if (memValue <= memCfg->threshold[FC_MEM_THRESHOLD_LEV_4].setThreshold) {
            targetPri = FC_PRI_FOR_MEM_LEV_4;
        } else if (memValue <= memCfg->threshold[FC_MEM_THRESHOLD_LEV_3].setThreshold) {
            targetPri = FC_PRI_FOR_MEM_LEV_3;
        } else if (memValue <= memCfg->threshold[FC_MEM_THRESHOLD_LEV_2].setThreshold) {
            targetPri = FC_PRI_FOR_MEM_LEV_2;
        } else if (memValue <= memCfg->threshold[FC_MEM_THRESHOLD_LEV_1].setThreshold) {
            targetPri = FC_PRI_FOR_MEM_LEV_1;
        } else {
            ;
        }
    }

    return targetPri;
}

/*
 * ????????????????????????????????????????????????????????????
 * ??????????????????????????????Lev3????
 */
FC_PriTypeUint8 FC_MEM_CalcDownTargetFcPri(FC_Policy *policy, VOS_UINT32 memValue)
{
    FC_PriTypeUint8 targetPri;
    FC_CfgMem      *memCfg = VOS_NULL_PTR;

    memCfg    = &(g_fcAcoreCfg.fcCfgMem);
    targetPri = policy->donePri;

    /* ?????????????????????????? */
    if (policy->priCnt == 1) {
        /* ???????????????????? */
        if (memValue > memCfg->threshold[FC_MEM_THRESHOLD_LEV_3].stopThreshold) {
            targetPri = FC_PRI_NULL;
        }
    } else { /* ?????????????????????????? */
        /* ???????????????????? */
        if (memValue > memCfg->threshold[FC_MEM_THRESHOLD_LEV_1].stopThreshold) {
            targetPri = FC_PRI_NULL;
        } else if (memValue > memCfg->threshold[FC_MEM_THRESHOLD_LEV_2].stopThreshold) {
            targetPri = FC_PRI_FOR_MEM_LEV_1;
        } else if (memValue > memCfg->threshold[FC_MEM_THRESHOLD_LEV_3].stopThreshold) {
            targetPri = FC_PRI_FOR_MEM_LEV_2;
        } else if (memValue > memCfg->threshold[FC_MEM_THRESHOLD_LEV_4].stopThreshold) {
            targetPri = FC_PRI_FOR_MEM_LEV_3;
        } else {
            ;
        }
    }

    return targetPri;
}

STATIC VOS_UINT32 FC_MEM_AdjustPriForUp(FC_PriTypeUint8 pointPri, FC_IdUint8 fcId)
{
    FC_Policy      *policy = VOS_NULL_PTR;
    FC_PriTypeUint8 targetPri;

    /* ?????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_MEM_MODEM_0);

    /* ???????????????????????????????????????? */
    targetPri = FC_MEM_CalcUpTargetFcPri(policy, 0);

    /* ?????????????????????????????????????????????????????? */
    if (policy->donePri < targetPri) {
        FC_PolicyUpToTargetPri(policy, targetPri);
    } else if (policy->donePri > targetPri) {
        FC_PolicyDownToTargetPri(policy, targetPri);
    } else {
        ;
    }

    /* for lint 715 */
    (VOS_VOID)pointPri;
    (VOS_VOID)fcId;

    return VOS_OK;
}

STATIC VOS_UINT32 FC_MEM_AdjustPriForDown(FC_PriTypeUint8 pointPri, FC_IdUint8 fcId)
{
    FC_Policy      *policy = VOS_NULL_PTR;
    FC_PriTypeUint8 targetPri;

    /* ?????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_MEM_MODEM_0);

    /* ???????????????????????????????????????? */
    targetPri = FC_MEM_CalcDownTargetFcPri(policy, 0);

    /* ?????????????????????????????????????????????????????? */
    if (policy->donePri < targetPri) {
        FC_PolicyUpToTargetPri(policy, targetPri);
    } else if (policy->donePri > targetPri) {
        FC_PolicyDownToTargetPri(policy, targetPri);
    } else {
        ;
    }

    /* for lint 715 */
    (VOS_VOID)pointPri;
    (VOS_VOID)fcId;

    return VOS_OK;
}

/* ?????????????????????????????? */
STATIC VOS_UINT32 FC_MEM_SndUpToTargetPriIndMsg(FC_PriTypeUint8 targetPri, VOS_UINT16 memFreeCnt)
{
    FC_MemUpToTargetPriInd *msg = VOS_NULL_PTR;
    VOS_UINT32              result;

    msg = (FC_MemUpToTargetPriInd*)(VOS_UINT_PTR)VOS_AllocMsg(
        UEPS_PID_FLOWCTRL, (VOS_UINT32)sizeof(FC_MemUpToTargetPriInd) - VOS_MSG_HEAD_LENGTH);

    if (msg == VOS_NULL_PTR) {
        FC_LOG(PS_PRINT_ERROR, "FC_SndMemUpToTargetPriIndMsg, Alloc Msg Fail\n");
        return VOS_ERR;
    }

    TTF_SET_MSG_RECEIVER_ID(msg, UEPS_PID_FLOWCTRL);
    msg->msgName    = ID_FC_MEM_UP_TO_TARGET_PRI_IND;
    msg->targetPri  = targetPri;
    msg->memFreeCnt = memFreeCnt;

    result = VOS_SendMsg(UEPS_PID_FLOWCTRL, msg);

    if (result != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_SndMemUpToTargetPriIndMsg, Send Msg Fail\n");
        return VOS_ERR;
    }

    return VOS_OK;
}

/* ?????????????????????????????? */
STATIC VOS_UINT32 FC_MEM_SndDownToTargetPriIndMsg(FC_PriTypeUint8 targetPri, VOS_UINT16 memFreeCnt)
{
    FC_MemDownToTargetPriInd *msg = VOS_NULL_PTR;
    VOS_UINT32                result;

    msg = (FC_MemDownToTargetPriInd*)(VOS_UINT_PTR)VOS_AllocMsg(
        UEPS_PID_FLOWCTRL, (VOS_UINT32)sizeof(FC_MemDownToTargetPriInd) - VOS_MSG_HEAD_LENGTH);

    if (msg == VOS_NULL_PTR) {
        FC_LOG(PS_PRINT_ERROR, "FC_SndMemDownToTargetPriIndMsg, Alloc Msg Fail\n");
        return VOS_ERR;
    }

    TTF_SET_MSG_RECEIVER_ID(msg, UEPS_PID_FLOWCTRL);
    msg->msgName    = ID_FC_MEM_DOWN_TO_TARGET_PRI_IND;
    msg->targetPri  = targetPri;
    msg->memFreeCnt = memFreeCnt;

    result = VOS_SendMsg(UEPS_PID_FLOWCTRL, msg);

    if (result != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_SndMemDownToTargetPriIndMsg, Send Msg Fail\n");
        return VOS_ERR;
    }

    return VOS_OK;
}

/*
 * ????????????????????
 * ????????:ulMemValue:????????
 */
VOS_VOID FC_MEM_UpProcess(VOS_UINT32 memValue)
{
    FC_Policy      *policy = VOS_NULL_PTR;
    FC_PriTypeUint8 targetPri;
    VOS_ULONG       flags = 0UL;

    /* ???????? */
    if ((FC_POLICY_MASK_MEM & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_MEM) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_MEM_UpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return;
    }

    policy    = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_MEM_MODEM_0);
    targetPri = FC_MEM_CalcUpTargetFcPri(policy, memValue);

    if ((policy->donePri < targetPri) && (policy->toPri < targetPri)) {
        /*lint --e(506,774,550,730) */
        /*lint --e{550,1072} */
        if (likely(preemptible())) { /* ???????????????????????????????????????????????? */
            VOS_SpinLockIntLock(&g_fcMemSpinLock, flags);
            policy->toPri = targetPri;
            VOS_SpinUnlockIntUnlock(&g_fcMemSpinLock, flags);

            /* ????????????????????????????????enToPri?????? */
            FC_MEM_SndUpToTargetPriIndMsg(targetPri, (VOS_UINT16)memValue);
        }
    }

    return;
}

/* ???????????????????????????????? */
VOS_VOID FC_MEM_DownProcess(VOS_UINT32 memValue)
{
    FC_Policy      *policy = VOS_NULL_PTR;
    FC_PriTypeUint8 targetPri;
    VOS_ULONG       flags = 0UL;

    /* ???????? */
    if ((FC_POLICY_MASK_MEM & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_MEM) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_MEM_UpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return;
    }

    policy    = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_MEM_MODEM_0);
    targetPri = FC_MEM_CalcDownTargetFcPri(policy, memValue);

    if ((policy->donePri > targetPri) && (policy->toPri > targetPri)) {
        /*lint --e(506,774,550,730) */
        /*lint --e{550,1072} */
        if (likely(preemptible())) { /* ???????????????????????????????????????????????? */
            VOS_SpinLockIntLock(&g_fcMemSpinLock, flags);
            policy->toPri = targetPri;
            VOS_SpinUnlockIntUnlock(&g_fcMemSpinLock, flags);

            /* ????????????????????????????????enToPri?????? */
            FC_MEM_SndDownToTargetPriIndMsg(targetPri, (VOS_UINT16)memValue);
        }
    }

    return;
}

/* ?????????????? */
STATIC VOS_UINT32 FC_MEM_Init(VOS_VOID)
{
    VOS_SpinLockInit(&g_fcMemSpinLock);

    /* ???????????????? */
    if ((FC_POLICY_MASK(FC_POLICY_ID_MEM) == (FC_POLICY_MASK(FC_POLICY_ID_MEM) & g_fcAcoreCfg.fcEnableMask))) {
        /* A???????????????????? */
        g_fcPolicy[FC_POLICY_ID_MEM].adjustForUpFunc   = FC_MEM_AdjustPriForUp;
        g_fcPolicy[FC_POLICY_ID_MEM].adjustForDownFunc = FC_MEM_AdjustPriForDown;
    } else {
        /* A???????????????????? */
        g_fcPolicy[FC_POLICY_ID_MEM].adjustForUpFunc   = VOS_NULL_PTR;
        g_fcPolicy[FC_POLICY_ID_MEM].adjustForDownFunc = VOS_NULL_PTR;
    }

    return VOS_OK;
}

/* CST???????? */
STATIC VOS_UINT32 FC_CstUpProcess(VOS_UINT8 rabId)
{
    FC_Policy *policy = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_CST & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_CST) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_CstUpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    /* ????CPU???????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CST_MODEM_0);
    FC_PolicyUpToTargetPri(policy, policy->highestPri);

    /* for lint 715 */
    (VOS_VOID)rabId;

    return VOS_OK;
}

/* CST???????????? */
STATIC VOS_UINT32 FC_CstDownProcess(VOS_UINT8 rabId)
{
    FC_Policy *policy = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_CST & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_CST) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_CstDownProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    /* ????CPU???????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CST_MODEM_0);
    FC_PolicyDownToTargetPri(policy, FC_PRI_NULL);

    /* for lint 715 */
    (VOS_VOID)rabId;

    return VOS_OK;
}

/* ????Fc???????? */
STATIC FC_RabMappingInfo* FC_CdsGetFcInfo(VOS_UINT8 rabId, ModemIdUint16 modemId)
{
    FC_RabMappingInfoSet *fcRabMappingInfoSet = VOS_NULL_PTR;
    FC_RabMappingInfo    *fcRabMappingInfo    = VOS_NULL_PTR;
    FC_IdUint8            fcId;
    VOS_UINT32            rabMask;

    fcRabMappingInfoSet = &g_fcRabMappingInfoSet[modemId];
    rabMask             = (VOS_UINT32)(1UL << rabId);

    /* ??????????????????????????????????RAB_ID?????????????????? */
    for (fcId = 0; fcId < fcRabMappingInfoSet->fcIdCnt; fcId++) {
        fcRabMappingInfo = &(fcRabMappingInfoSet->fcRabMappingInfo[fcId]);
        if ((rabMask & fcRabMappingInfo->includeRabMask) != 0) {
            return fcRabMappingInfo;
        }
    }

    return VOS_NULL_PTR;
}

/* ????FCId??????FC???? */
VOS_UINT32 FC_CdsDelFcId(FC_IdUint8 fcId, ModemIdUint16 modemId)
{
    FC_RabMappingInfoSet *fcRabMappingInfoSet = VOS_NULL_PTR;
    FC_IdUint8            fcIdNum;
    FC_IdUint8            shiftFcId;

    fcRabMappingInfoSet = &g_fcRabMappingInfoSet[modemId];

    for (fcIdNum = 0; fcIdNum < fcRabMappingInfoSet->fcIdCnt; fcIdNum++) {
        if (fcId == fcRabMappingInfoSet->fcRabMappingInfo[fcIdNum].fcId) {
            break;
        }
    }

    if (fcIdNum >= fcRabMappingInfoSet->fcIdCnt) {
        FC_LOG1(PS_PRINT_WARNING, "FC_CdsDelFcId, can not find the Fc Id <1>!\n", (VOS_INT32)fcId);
        return VOS_ERR;
    }

    /* ???????????????????????????????????????? */
    for (shiftFcId = (FC_PriTypeUint8)(fcIdNum + 1); shiftFcId < fcRabMappingInfoSet->fcIdCnt; shiftFcId++) {
        fcRabMappingInfoSet->fcRabMappingInfo[shiftFcId - 1] = fcRabMappingInfoSet->fcRabMappingInfo[shiftFcId];
    }

    /* ???????????????????????????????????????????????? */
    (VOS_VOID)memset_s(&(fcRabMappingInfoSet->fcRabMappingInfo[shiftFcId - 1]), (VOS_UINT32)sizeof(FC_RabMappingInfo),
        0, (VOS_UINT32)sizeof(FC_RabMappingInfo));
    fcRabMappingInfoSet->fcIdCnt--;

    return VOS_OK;
}

/* ????ClentId??RAB???? */
VOS_UINT32 FC_CdsAddRab(FC_IdUint8 fcId, VOS_UINT8 rabId, ModemIdUint16 modemId)
{
    FC_RabMappingInfoSet *fcRabMappingInfoSet = VOS_NULL_PTR;
    FC_RabMappingInfo    *fcRabMappingInfo    = VOS_NULL_PTR;
    FC_IdUint8            fcIdNum;
    VOS_UINT32            rabMask;

    fcRabMappingInfoSet = &g_fcRabMappingInfoSet[modemId];
    rabMask             = (VOS_UINT32)(1UL << rabId);

    if (fcRabMappingInfoSet->fcIdCnt > FC_MAX_NUM) {
        FC_LOG1(PS_PRINT_ERROR, "FC_CdsAddRab, g_fcRabMappingInfoSet is exceed the ranger!\n",
            (VOS_INT32)fcRabMappingInfoSet->fcIdCnt);
        return VOS_ERR;
    }

    /* ??????FC????????????????????????????RAB_ID??Fc Id???????? */
    for (fcIdNum = 0; fcIdNum < fcRabMappingInfoSet->fcIdCnt; fcIdNum++) {
        fcRabMappingInfo = &(fcRabMappingInfoSet->fcRabMappingInfo[fcIdNum]);
        if (fcId == fcRabMappingInfo->fcId) {
            if ((fcRabMappingInfo->includeRabMask & rabMask) == 0) {
                fcRabMappingInfo->includeRabMask |= rabMask;
                fcRabMappingInfo->noFcRabMask |= rabMask;
                (VOS_VOID)FC_PointClrFc(FC_POLICY_MASK_CDS, fcId);
            }

            return VOS_OK;
        }
    }

    if (fcRabMappingInfoSet->fcIdCnt == FC_MAX_NUM) {
        FC_LOG(PS_PRINT_ERROR, "FC_CdsAddRab, AtClientCnt reaches the max num!\n");
        return VOS_ERR;
    }

    /* ??????????FC Id??????????Fc Id??RAB ID */
    fcRabMappingInfo                 = &(fcRabMappingInfoSet->fcRabMappingInfo[fcRabMappingInfoSet->fcIdCnt]);
    fcRabMappingInfo->fcId           = fcId;
    fcRabMappingInfo->includeRabMask = rabMask;
    fcRabMappingInfo->noFcRabMask    = rabMask;

    fcRabMappingInfoSet->fcIdCnt++;

    return VOS_OK;
}

/* ??????RABID?????????????? */
VOS_UINT32 FC_CdsDelRab(VOS_UINT8 rabId, ModemIdUint16 modemId)
{
    FC_RabMappingInfo *fcRabMappingInfo = VOS_NULL_PTR;
    VOS_UINT32         rabMask;

    /* ????Rab Id????????Rab Id??????FC???? */
    fcRabMappingInfo = FC_CdsGetFcInfo(rabId, modemId);
    if (fcRabMappingInfo == VOS_NULL_PTR) {
        return VOS_ERR;
    }

    /* ??????RabId??????????????Fc????????????????????????RabId???????????? */
    rabMask = (VOS_UINT32)(1UL << rabId);

    fcRabMappingInfo->includeRabMask &= (~rabMask);

    if (fcRabMappingInfo->noFcRabMask != 0) {
        fcRabMappingInfo->noFcRabMask &= (~rabMask);
        if ((fcRabMappingInfo->noFcRabMask == 0) && (fcRabMappingInfo->includeRabMask != 0)) {
            (VOS_VOID)FC_PointSetFc(FC_POLICY_MASK_CDS, fcRabMappingInfo->fcId);
        }
    }

    /* ??????????RabId??????FC????????????????????????FC ID?????????? */
    if (fcRabMappingInfo->includeRabMask == 0) {
        FC_CdsDelFcId(fcRabMappingInfo->fcId, modemId);
    }

    return VOS_OK;
}

/* ATClient?????????? */
STATIC VOS_UINT32 FC_CdsInit(VOS_VOID)
{
    (VOS_VOID)memset_s(
        g_fcRabMappingInfoSet, (VOS_UINT32)sizeof(g_fcRabMappingInfoSet), 0, (VOS_UINT32)sizeof(g_fcRabMappingInfoSet));

    return VOS_OK;
}

/* CDS???????? */
VOS_UINT32 FC_CdsUpProcess(VOS_UINT8 rabId, ModemIdUint16 modemId)
{
    FC_RabMappingInfo *fcRabMappingInfo = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_CDS & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_CDS) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_CstUpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    if (modemId >= MODEM_ID_BUTT) {
        FC_LOG1(PS_PRINT_WARNING, "FC_CdsUpProcess ModemId Is Invalid %d\n", modemId);
        return VOS_ERR;
    }

    /* ????RAB id?????? Client???????? */
    fcRabMappingInfo = FC_CdsGetFcInfo(rabId, modemId);
    if (fcRabMappingInfo == VOS_NULL_PTR) {
        return VOS_ERR;
    }

    /* ??????Fc?????????????????????????????????????? */
    if (fcRabMappingInfo->noFcRabMask == 0) {
        return VOS_OK;
    }

    /* ??????FC????????????????RAB????????????0??????????RAB??????????????FC???????? */
    fcRabMappingInfo->noFcRabMask &= (~((VOS_UINT32)1 << rabId));
    if (fcRabMappingInfo->noFcRabMask == 0) {
        (VOS_VOID)FC_PointSetFc(FC_POLICY_MASK_CDS, fcRabMappingInfo->fcId);
    }

    return VOS_OK;
}

/* CDS???????????? */
VOS_UINT32 FC_CdsDownProcess(VOS_UINT8 rabId, ModemIdUint16 modemId)
{
    FC_RabMappingInfo *fcRabMappingInfo = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_CDS & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_CDS) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_CdsDownProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    if (modemId >= MODEM_ID_BUTT) {
        FC_LOG1(PS_PRINT_WARNING, "FC_CdsDownProcess ModemId Is Invalid %d\n", modemId);
        return VOS_ERR;
    }

    /* ????RAB id?????? FC???? */
    fcRabMappingInfo = FC_CdsGetFcInfo(rabId, modemId);
    if (fcRabMappingInfo == VOS_NULL_PTR) {
        return VOS_ERR;
    }

    /* ??????FC?????????????????????????????????????? */
    if (fcRabMappingInfo->noFcRabMask != 0) {
        fcRabMappingInfo->noFcRabMask |= ((VOS_UINT32)1 << rabId);
        return VOS_OK;
    }

    /* ????v????????????????RAB????????????0??????FC????s?????????????? */
    fcRabMappingInfo->noFcRabMask |= ((VOS_UINT32)1 << rabId);
    (VOS_VOID)FC_PointClrFc(FC_POLICY_MASK_CDS, fcRabMappingInfo->fcId);

    return VOS_OK;
}

/* ???????????? */
STATIC VOS_VOID FC_UpProcess(VOS_RatModeUint32 rateMode)
{
    switch (rateMode) {
        case VOS_RATMODE_GSM:

            FC_GprsUpProcess();

            break;
#if (FEATURE_UE_MODE_CDMA == FEATURE_ON)
        case VOS_RATMODE_1X:
        case VOS_RATMODE_HRPD:

            FC_CdmaUpProcess();

            break;
#endif
        default:

            FC_LOG1(PS_PRINT_WARNING, "FC_UpProcess RateMode Is Invalid %d\n", (VOS_INT32)rateMode);

            break;
    }

    return;
}

/* ????GPRS???????? */
STATIC VOS_UINT32 FC_GprsUpProcess(VOS_VOID)
{
    FC_Policy *policy = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_GPRS & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_GPRS) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_GprsUpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    /* ????CPU???????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_GPRS_MODEM_0);
    FC_PolicyUpToTargetPri(policy, policy->highestPri);

    return VOS_OK;
}

/* ???????????? */
STATIC VOS_VOID FC_DownProcess(VOS_RatModeUint32 rateMode)
{
    switch (rateMode) {
        case VOS_RATMODE_GSM:

            FC_GprsDownProcess();

            break;
#if (FEATURE_UE_MODE_CDMA == FEATURE_ON)
        case VOS_RATMODE_1X:
        case VOS_RATMODE_HRPD:

            FC_CdmaDownProcess();

            break;
#endif
        default:

            FC_LOG1(PS_PRINT_WARNING, "FC_DownProcess RateMode Is Invalid %d\n", (VOS_INT32)rateMode);

            break;
    }

    return;
}

/* ????GPRS???????????? */
STATIC VOS_UINT32 FC_GprsDownProcess(VOS_VOID)
{
    FC_Policy *policy = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_GPRS & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_GPRS) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_GprsUpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    /* ????CPU???????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_GPRS_MODEM_0);
    FC_PolicyDownToTargetPri(policy, FC_PRI_NULL);

    return VOS_OK;
}

#if (FEATURE_UE_MODE_CDMA == FEATURE_ON)
/* ????CDMA???????? */
STATIC VOS_UINT32 FC_CdmaUpProcess(VOS_VOID)
{
    FC_Policy *policy = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_CDMA & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_CDMA) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_CdmaUpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    /* ????CPU???????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CDMA_MODEM_0);
    FC_PolicyUpToTargetPri(policy, policy->highestPri);

    return VOS_OK;
}

/* ????CDMA???????????? */
STATIC VOS_UINT32 FC_CdmaDownProcess(VOS_VOID)
{
    FC_Policy *policy = VOS_NULL_PTR;

    /* ???????? */
    if ((FC_POLICY_MASK_CDMA & g_fcAcoreCfg.fcEnableMask) != FC_POLICY_MASK_CDMA) {
        /* ?????????????? */
        FC_LOG1(PS_PRINT_INFO, "FC_CdmaUpProcess, INFO, MEM FlowCtrl is disabled %d \n",
            (VOS_INT32)g_fcAcoreCfg.fcEnableMask);
        return VOS_OK;
    }

    /* ????CPU???????????????????????????????? */
    policy = FC_POLICY_GET(FC_PRIVATE_POLICY_ID_CDMA_MODEM_0);
    FC_PolicyDownToTargetPri(policy, FC_PRI_NULL);

    return VOS_OK;
}
#endif
/* A????????????????????AT??????????RABID???????? */
VOS_VOID FC_ChannelMapCreate(FC_IdUint8 fcId, VOS_UINT8 rabId, ModemIdUint16 modemId)
{
    FC_AddRabFcIdMapInd *msg = VOS_NULL_PTR;
    VOS_UINT32           result;

    /* ??????????RabId??????[5,15] */
    if ((rabId < FC_UE_MIN_RAB_ID) || (rabId > FC_UE_MAX_RAB_ID)) {
        FC_LOG1(PS_PRINT_WARNING, "FC_ChannelMapCreate RabId Is Invalid %d\n", rabId);
        return;
    }

    if (modemId >= MODEM_ID_BUTT) {
        FC_LOG1(PS_PRINT_WARNING, "FC_ChannelMapCreate ModemId Is Invalid %d\n", modemId);
        return;
    }

    /* ???????? */
    msg = (FC_AddRabFcIdMapInd*)(VOS_UINT_PTR)VOS_AllocMsg(
        UEPS_PID_FLOWCTRL_A, (VOS_UINT32)sizeof(FC_AddRabFcIdMapInd) - VOS_MSG_HEAD_LENGTH);

    if (msg == VOS_NULL_PTR) {
        FC_LOG(PS_PRINT_ERROR, "FC_ChannelMapCreate, Alloc Msg Fail\n");
        return;
    }

    TTF_SET_MSG_RECEIVER_ID(msg, UEPS_PID_FLOWCTRL);
    msg->msgName = ID_FC_ADD_RAB_FCID_MAP_IND;
    msg->fcId    = fcId;
    msg->rabId   = rabId;
    msg->modemId = modemId;

    result = VOS_SendMsg(UEPS_PID_FLOWCTRL_A, msg);

    if (result != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_ChannelMapCreate, Send Msg Fail\n");
        return;
    }
}

/* ????????RABID????????AT?????????????????????????????? */
VOS_VOID FC_ChannelMapDelete(VOS_UINT8 rabId, ModemIdUint16 modemId)
{
    FC_DelRabFcIdMapInd *msg = VOS_NULL_PTR;
    VOS_UINT32           result;

    /* ??????????RabId??????[5,15] */
    if ((rabId < FC_UE_MIN_RAB_ID) || (rabId > FC_UE_MAX_RAB_ID)) {
        FC_LOG1(PS_PRINT_WARNING, "FC_ChannelMapDelete RabId Is Invalid %d\n", rabId);
        return;
    }

    if (modemId >= MODEM_ID_BUTT) {
        FC_LOG1(PS_PRINT_WARNING, "FC_ChannelMapDelete ModemId Is Invalid %d\n", modemId);
        return;
    }

    /* ???????? */
    msg = (FC_DelRabFcIdMapInd*)(VOS_UINT_PTR)VOS_AllocMsg(
        UEPS_PID_FLOWCTRL_A, (VOS_UINT32)sizeof(FC_DelRabFcIdMapInd) - VOS_MSG_HEAD_LENGTH);

    if (msg == VOS_NULL_PTR) {
        FC_LOG(PS_PRINT_ERROR, "FC_ChannelMapDelete, Alloc Msg Fail\n");
        return;
    }

    TTF_SET_MSG_RECEIVER_ID(msg, UEPS_PID_FLOWCTRL);
    msg->msgName = ID_FC_DEL_RAB_FCID_MAP_IND;
    msg->rabId   = rabId;
    msg->modemId = modemId;

    result = VOS_SendMsg(UEPS_PID_FLOWCTRL_A, msg);

    if (result != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_ChannelMapDelete, Send Msg Fail\n");
        return;
    }
}

/* ????CST???? */
STATIC VOS_UINT32 FC_RcvCstMsg(MsgBlock *msg)
{
    /*lint --e(826) ????????????UINT8????????UINT32???????????????? */
    switch (FC_GET_MSG_NAME32(msg)) {
        case ID_CST_FC_SET_FLOWCTRL_REQ:
            FC_CstUpProcess(((CST_FC_SetFlowctrlReq*)(VOS_UINT_PTR)msg)->rabId);
            break;

        case ID_CST_FC_STOP_FLOWCTRL_REQ:
            FC_CstDownProcess(((CST_FC_StopFlowctrlReq*)(VOS_UINT_PTR)msg)->rabId);
            break;

        default:
            break;
    }

    return VOS_OK;
}

/* ????????CDS???? */
STATIC VOS_UINT32 FC_RcvCdsMsg(MsgBlock *msg)
{
    /*lint --e(826) ????????????UINT8????????UINT32???????????????? */
    switch (FC_GET_MSG_NAME32(msg)) {
        case ID_CDS_FC_STOP_CHANNEL_IND:
            FC_CdsUpProcess(((CDS_FC_StopChannelInd*)(VOS_UINT_PTR)msg)->rabId,
                ((CDS_FC_StopChannelInd*)(VOS_UINT_PTR)msg)->modemId);
            break;

        case ID_CDS_FC_START_CHANNEL_IND:
            FC_CdsDownProcess(((CDS_FC_StartChannelInd*)(VOS_UINT_PTR)msg)->rabId,
                ((CDS_FC_StartChannelInd*)(VOS_UINT_PTR)msg)->modemId);
            break;

        default:
            break;
    }

    return VOS_OK;
}

/* ?????????????????????????????? */
STATIC VOS_UINT32 FC_AcoreRegPoint(VOS_VOID)
{
#if (FEATURE_UE_MODE_CDMA == FEATURE_OFF)
/* V9R1????????????????????????FC?????????????????????????? */
#if (FEATURE_NFEXT == FEATURE_ON)
    FC_RegPointPara fcRegPoint = {0};

    /* CPU?????????????? */
    fcRegPoint.policyId = FC_POLICY_ID_CPU_A;
    fcRegPoint.fcPri    = FC_PRI_FOR_BRIDGE_FORWARD_DISCARD;
    fcRegPoint.fcId     = FC_ID_BRIDGE_FORWARD_DISACRD;
    fcRegPoint.param1   = 0;
    fcRegPoint.param2   = 0;
    fcRegPoint.setFunc  = NFEXT_BrSetFlowCtrl;
    fcRegPoint.clrFunc  = NFEXT_BrStopFlowCtrl;
    fcRegPoint.rstFunc  = VOS_NULL_PTR;
    FC_PointReg(&fcRegPoint);

    fcRegPoint.policyId = FC_POLICY_ID_CPU_A;
    fcRegPoint.fcPri    = FC_PRI_HIGHEST;
    fcRegPoint.fcId     = FC_ID_WIFI_ETH;
    fcRegPoint.param1   = 0;
    fcRegPoint.param2   = 0;
    fcRegPoint.setFunc  = DRV_WIFI_SET_RX_FCTL;
    fcRegPoint.clrFunc  = DRV_WIFI_CLR_RX_FCTL;
    fcRegPoint.rstFunc  = VOS_NULL_PTR;
    FC_PointReg(&fcRegPoint);

    /* ?????????????? */
    fcRegPoint.policyId = FC_POLICY_ID_MEM;
    fcRegPoint.fcPri    = FC_PRI_FOR_BRIDGE_FORWARD_DISCARD;
    fcRegPoint.fcId     = FC_ID_BRIDGE_FORWARD_DISACRD;
    fcRegPoint.param1   = 0;
    fcRegPoint.param2   = 0;
    fcRegPoint.setFunc  = NFEXT_BrSetFlowCtrl;
    fcRegPoint.clrFunc  = NFEXT_BrStopFlowCtrl;
    fcRegPoint.rstFunc  = VOS_NULL_PTR;
    FC_PointReg(&fcRegPoint);

    fcRegPoint.policyId = FC_POLICY_ID_MEM;
    fcRegPoint.fcPri    = FC_PRI_FOR_MEM_LEV_4;
    fcRegPoint.fcId     = FC_ID_WIFI_ETH;
    fcRegPoint.param1   = 0;
    fcRegPoint.param2   = 0;
    fcRegPoint.setFunc  = DRV_WIFI_SET_RX_FCTL;
    fcRegPoint.clrFunc  = DRV_WIFI_CLR_RX_FCTL;
    fcRegPoint.rstFunc  = VOS_NULL_PTR;
    FC_PointReg(&fcRegPoint);

#endif
#endif

    return VOS_OK;
}

/* ?????????????????? */
STATIC VOS_UINT32 FC_AcoreRcvTimerMsg(REL_TimerMsgBlock *timerMsg)
{
    switch (timerMsg->name) {
        case FC_TIMER_STOP_CPU_ATTEMPT:
            FC_CpuaStopFlowCtrl();
            break;

        default:
            break;
    }

    return VOS_OK;
}

/* ???????????????????? */
VOS_UINT32 FC_AcoreRcvIntraMsg(MsgBlock *msg)
{
    /*lint --e(826) ????????????UINT8????????UINT16???????????????? */
    switch (FC_GET_MSG_NAME16(msg)) {
        case ID_FC_REG_POINT_IND:
            FC_PointReg(&(((FC_RegPointInd*)(VOS_UINT_PTR)msg)->fcPoint));
            break;

        case ID_FC_DEREG_POINT_IND:
            FC_PointDeReg(
                ((FC_DeregPointInd*)(VOS_UINT_PTR)msg)->fcId, ((FC_DeregPointInd*)(VOS_UINT_PTR)msg)->modemId);
            break;

        case ID_FC_CHANGE_POINT_IND:
            FC_PointChange(((FC_ChangePointInd*)(VOS_UINT_PTR)msg)->fcId,
                ((FC_ChangePointInd*)(VOS_UINT_PTR)msg)->policyId, ((FC_ChangePointInd*)(VOS_UINT_PTR)msg)->pri,
                ((FC_ChangePointInd*)(VOS_UINT_PTR)msg)->modemId);
            break;

        case ID_FC_CPU_A_OVERLOAD_IND:
            FC_CpuaUpProcess();
            break;

        case ID_FC_CPU_A_UNDERLOAD_IND:
            FC_CpuaDownProcess();
            break;

        case ID_FC_SET_FLOWCTRL_IND:
            FC_UpProcess(((FC_SetFlowCtrlInd*)(VOS_UINT_PTR)msg)->rateMode);
            break;

        case ID_FC_STOP_FLOWCTRL_IND:
            FC_DownProcess(((FC_SetFlowCtrlInd*)(VOS_UINT_PTR)msg)->rateMode);
            break;

        case ID_FC_ADD_RAB_FCID_MAP_IND:
            FC_CdsAddRab(((FC_AddRabFcIdMapInd*)(VOS_UINT_PTR)msg)->fcId,
                ((FC_AddRabFcIdMapInd*)(VOS_UINT_PTR)msg)->rabId, ((FC_AddRabFcIdMapInd*)(VOS_UINT_PTR)msg)->modemId);
            break;

        case ID_FC_DEL_RAB_FCID_MAP_IND:
            FC_CdsDelRab(
                ((FC_DelRabFcIdMapInd*)(VOS_UINT_PTR)msg)->rabId, ((FC_DelRabFcIdMapInd*)(VOS_UINT_PTR)msg)->modemId);
            break;

        case ID_FC_MEM_UP_TO_TARGET_PRI_IND:
            FC_PolicyUpToTargetPri(
                &(g_fcPolicy[FC_POLICY_ID_MEM]), ((FC_MemUpToTargetPriInd*)(VOS_UINT_PTR)msg)->targetPri);
            break;

        case ID_FC_MEM_DOWN_TO_TARGET_PRI_IND:
            FC_PolicyDownToTargetPri(
                &(g_fcPolicy[FC_POLICY_ID_MEM]), ((FC_MemDownToTargetPriInd*)(VOS_UINT_PTR)msg)->targetPri);
            break;

        case ID_FC_ACORE_CRESET_START_IND:
            FC_AcoreCResetProc(FC_ACORE_CRESET_BEFORE_RESET);
            break;

        case ID_FC_ACORE_CRESET_END_IND:
            FC_AcoreCResetProc(FC_ACORE_CRESET_AFTER_RESET);
            break;

        case ID_FC_ACORE_CRESET_START_RSP:
            FC_AcoreCResetRcvStartRsp();
            break;

        default:
            break;
    }

    return VOS_OK;
}

/* ???????????????? */
VOS_VOID FC_AcoreMsgProc(MsgBlock *msg)
{
    switch (TTF_GET_MSG_SENDER_ID(msg)) {
        case UEPS_PID_FLOWCTRL_A:
        case UEPS_PID_FLOWCTRL_C:
            FC_AcoreRcvIntraMsg(msg);
            break;

        case UEPS_PID_CST:
            FC_RcvCstMsg(msg);
            break;

        case UEPS_PID_CDS:
            FC_RcvCdsMsg(msg);
            break;

        case VOS_PID_TIMER:
            FC_AcoreRcvTimerMsg((REL_TimerMsgBlock*)(VOS_UINT_PTR)msg);
            break;

        default:
            break;
    }
}

/* ????A??NV?????? */
STATIC VOS_UINT32 FC_AcoreCfgSetDefaultValue(FC_AcoreCfgNv *acoreFcCfg)
{
    FC_LOG(PS_PRINT_WARNING, "FC_CFG_SetDefaultValue, Set Default NV Value.\n");

    (VOS_VOID)memset_s(acoreFcCfg, sizeof(FC_AcoreCfgNv), 0x0, sizeof(FC_AcoreCfgNv));

    acoreFcCfg->fcEnableMask = 0xFF;
    FC_SetFcEnableMask(acoreFcCfg->fcEnableMask);

    acoreFcCfg->fcCfgCpuA.cpuOverLoadVal      = 95;
    acoreFcCfg->fcCfgCpuA.cpuUnderLoadVal     = 70;
    acoreFcCfg->fcCfgCpuA.smoothTimerLen      = 8;
    acoreFcCfg->fcCfgCpuA.stopAttemptTimerLen = 100;
    acoreFcCfg->fcCfgCpuA.umUlRateThreshold   = 1048576;
    acoreFcCfg->fcCfgCpuA.umDlRateThreshold   = 10485760;
    acoreFcCfg->fcCfgCpuA.rmRateThreshold     = 10485760;

    acoreFcCfg->fcCfgMem.thresholdCnt               = 4;
    acoreFcCfg->fcCfgMem.threshold[0].setThreshold  = 300;
    acoreFcCfg->fcCfgMem.threshold[0].stopThreshold = 350;
    acoreFcCfg->fcCfgMem.threshold[1].setThreshold  = 250;
    acoreFcCfg->fcCfgMem.threshold[1].stopThreshold = 300;
    acoreFcCfg->fcCfgMem.threshold[2].setThreshold  = 150;
    acoreFcCfg->fcCfgMem.threshold[2].stopThreshold = 200;
    acoreFcCfg->fcCfgMem.threshold[3].setThreshold  = 0;
    acoreFcCfg->fcCfgMem.threshold[3].stopThreshold = 20;

    acoreFcCfg->fcCfgCst.threshold.setThreshold  = 3072;
    acoreFcCfg->fcCfgCst.threshold.stopThreshold = 1024;

    return VOS_OK;
}

/* A??????????NV?????????????? */
STATIC VOS_UINT32 FC_AcoreCfgCheckMemParam(FC_CfgMem *fcCfgMem)
{
    VOS_UINT32 thresholdLoop;

    if (fcCfgMem->thresholdCnt > FC_MEM_THRESHOLD_LEV_BUTT) {
        FC_LOG1(PS_PRINT_WARNING, "FC, FC_AcoreCfgCheckMemParam, WARNING, ulThresholdCnt %d\n",
            (VOS_INT32)fcCfgMem->thresholdCnt);

        return VOS_ERR;
    }

    for (thresholdLoop = 0; thresholdLoop < fcCfgMem->thresholdCnt; thresholdLoop++) {
        /* ?????????????????????? */
        if (fcCfgMem->threshold[thresholdLoop].setThreshold > fcCfgMem->threshold[thresholdLoop].stopThreshold) {
            FC_LOG3(PS_PRINT_WARNING,
                "FC, FC_AcoreCfgCheckMemParam, WARNING, ulThresholdLoop %d ulSetThreshold %D less than ulStopThreshold %d\n",
                (VOS_INT32)thresholdLoop, (VOS_INT32)fcCfgMem->threshold[thresholdLoop].setThreshold,
                (VOS_INT32)fcCfgMem->threshold[thresholdLoop].stopThreshold);
            return VOS_ERR;
        }
    }

    return VOS_OK;
}

/* A??NV?????????????? */
VOS_UINT32 FC_AcoreCfgCheckParam(FC_AcoreCfgNv *acoreFcCfg)
{
    /* ??????????A??????????????????????????CPU????????????????CPU????????????????????????????????????????????2?? */
    if (acoreFcCfg->fcCfgCpuA.smoothTimerLen < 2) {
        FC_LOG1(PS_PRINT_WARNING, "FC_CFG_CheckParam, WARNING, CPUA flow ctrl ulSmoothTimerLen is %d!\n",
            (VOS_INT32)acoreFcCfg->fcCfgCpuA.smoothTimerLen);
        return VOS_ERR;
    }

    if (FC_CfgCheckCpuParam(&(acoreFcCfg->fcCfgCpuA)) != VOS_OK) {
        FC_LOG(PS_PRINT_WARNING, "FC_CFG_CheckParam, WARNING, Check CPUA flow ctrl param failed!\n");
        return VOS_ERR;
    }

    if (FC_AcoreCfgCheckMemParam(&(acoreFcCfg->fcCfgMem)) != VOS_OK) {
        FC_LOG(PS_PRINT_WARNING, "FC_CFG_CheckParam, WARNING, Check mem flow ctrl param failed!\n");
        return VOS_ERR;
    }

    return VOS_OK;
}

/* A???????????????? */
VOS_VOID FC_AcoreCfgInit(VOS_VOID)
{
    VOS_UINT32 result;
    VOS_UINT32 fcEnableMask;

    (VOS_VOID)memset_s(&g_fcAcoreCfg, sizeof(FC_AcoreCfgNv), 0, sizeof(FC_AcoreCfgNv));

    result = GUCTTF_NV_Read(MODEM_ID_0, NV_ITEM_ACORE_FLOW_CRTL_CONFIG, &g_fcAcoreCfg, sizeof(FC_AcoreCfgNv));

    /* ??A??FC NV??????????????????FC???????????? */
    fcEnableMask              = g_fcAcoreCfg.fcEnableMask;
    g_fcAcoreCfg.fcEnableMask = 0;
    g_fcAcoreCfg.fcEnableMask |= ((1 == FC_ACORE_GetEnableMask(fcEnableMask, FC_ACORE_MEM_ENABLE_MASK)) ?
                                      (FC_POLICY_MASK(FC_POLICY_ID_MEM)) :
                                      (0));
    g_fcAcoreCfg.fcEnableMask |= ((1 == FC_ACORE_GetEnableMask(fcEnableMask, FC_ACORE_CPUA_ENABLE_MASK)) ?
                                      (FC_POLICY_MASK(FC_POLICY_ID_CPU_A)) :
                                      (0));
    g_fcAcoreCfg.fcEnableMask |= ((1 == FC_ACORE_GetEnableMask(fcEnableMask, FC_ACORE_CDS_ENABLE_MASK)) ?
                                      (FC_POLICY_MASK(FC_POLICY_ID_CDS)) :
                                      (0));
    g_fcAcoreCfg.fcEnableMask |= ((1 == FC_ACORE_GetEnableMask(fcEnableMask, FC_ACORE_CST_ENABLE_MASK)) ?
                                      (FC_POLICY_MASK(FC_POLICY_ID_CST)) :
                                      (0));

    /* C???????????????????????????? */
    g_fcAcoreCfg.fcEnableMask |= FC_POLICY_MASK_GPRS;
    g_fcAcoreCfg.fcEnableMask |= FC_POLICY_MASK_TMP;
    g_fcAcoreCfg.fcEnableMask |= FC_POLICY_MASK_CPU_C;
#if (FEATURE_UE_MODE_CDMA == FEATURE_ON)
    g_fcAcoreCfg.fcEnableMask |= FC_POLICY_MASK_CDMA;
#endif
    FC_SetFcEnableMask(g_fcAcoreCfg.fcEnableMask);

    if (result != NV_OK) {
        FC_LOG1(PS_PRINT_WARNING, "FC_AcoreCfgInit, WARNING, Fail to read NV, result %d\n", (VOS_INT32)result);

        (VOS_VOID)FC_AcoreCfgSetDefaultValue(&g_fcAcoreCfg);

        return;
    }

    result = FC_AcoreCfgCheckParam(&g_fcAcoreCfg);

    if (result != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_CommInit, ERROR, Check NV parameter fail!\n");

        /* ??????NV?????????????????????????? */
        (VOS_VOID)FC_AcoreCfgSetDefaultValue(&g_fcAcoreCfg);
    }

    return;
}

/* ?????????????? */
VOS_UINT32 FC_AcoreInit(VOS_VOID)
{
    VOS_UINT32 result;
    VOS_INT    iRet;
    VOS_CHAR   smName[] = "FcACoreCResetDoneSem";

    result = FC_CommInit();

    if (result != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_Init, ERROR, FC_CommInit return error!\n");
        return VOS_ERR;
    }

    /* ???????????????? */
    FC_AcoreCfgInit();

    result = FC_CpuaInit();
    if (result != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_AcoreInit, ERROR, FC_CpuaInit return error!\n");
        return VOS_ERR;
    }

    FC_CdsInit();

    FC_MEM_Init();

    FC_AcoreRegPoint();

    FC_DrvAssemInit();

    /* ????????????????C??????????????????????FcACore???????????????? */
    if (VOS_SmBCreate(smName, 0, VOS_SEMA4_FIFO, (VOS_SEM*)&g_fcaCorecResetDoneSem) != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_AcoreInit, Create Sem Fail\n");
        return VOS_ERR;
    }

    /* ??????????????????C???????????? */
    iRet = mdrv_sysboot_register_reset_notify("TTF_FcACore", FC_AcoreCResetCallback, 0, FC_ACORE_CRESET_CALLBACK_PRIOR);

    if (iRet != VOS_OK) {
        FC_LOG(PS_PRINT_ERROR, "FC_AcoreInit, ERROR, DRV_CCORERESET_REGCBFUNC fail!\n");
        return VOS_ERR;
    }

    return VOS_OK;
}

/* ????FID?????????? */
VOS_UINT32 FC_AcoreFidInit(enum VOS_InitPhaseDefine phase)
{
    VOS_UINT32 result = VOS_ERR;

    switch (phase) {
        case VOS_IP_LOAD_CONFIG:
            result = VOS_RegisterPIDInfo(UEPS_PID_FLOWCTRL, (InitFunType)VOS_NULL_PTR, (MsgFunType)FC_AcoreMsgProc);
            if (result != VOS_OK) {
                FC_LOG(PS_PRINT_ERROR, "FC_FidInit, VOS_RegisterPIDInfo Fail\n");
                return VOS_ERR;
            }
            result = VOS_RegisterTaskPrio(UEPS_FID_FLOWCTRL, TTF_FLOW_CTRL_TASK_PRIO);
            if (result != VOS_OK) {
                FC_LOG(PS_PRINT_ERROR, "FC_FidInit, OS_RegisterTaskPrio Fail\n");
                return VOS_ERR;
            }

            result = FC_AcoreInit();

            if (result != VOS_OK) {
                FC_LOG(PS_PRINT_ERROR, "FC_FidInit, Call FC_Init return error!\n");
                return VOS_ERR;
            }

            break;
        case VOS_IP_FARMALLOC:
        case VOS_IP_INITIAL:
        case VOS_IP_ENROLLMENT:
        case VOS_IP_LOAD_DATA:
        case VOS_IP_FETCH_DATA:
        case VOS_IP_STARTUP:
        case VOS_IP_RIVAL:
        case VOS_IP_KICKOFF:
        case VOS_IP_STANDBY:
        case VOS_IP_BROADCAST_STATE:
        case VOS_IP_RESTART:
            break;
        default:
            break;
    }

    return VOS_OK;
}


#endif

