
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

/*
 * 1 Include HeadFile
 */
#include <securec.h>
#include "diag_api.h"
#include <vos.h>
#include <mdrv.h>
#include <msp.h>
#include <nv_stru_lps.h>
#include <nv_id_tlas.h>
#include <nv_id_drv.h>
#include <nv_stru_drv.h>
#include <soc_socp_adapter.h>
#include "diag_common.h"
#include "diag_cfg.h"
#include "diag_debug.h"
#include "diag_message.h"


#define THIS_FILE_ID MSP_FILE_ID_DIAG_API_C
/*
 * 2 Declare the Global Variable
 */

extern VOS_UINT8 g_EventModuleCfg[DIAG_CFG_PID_NUM];

VOS_UINT32 g_ulDiagProfile = DIAGLOG_POWER_LOG_PROFILE_OFF;

mdrv_diag_layer_msg_match_func g_pLayerMatchFunc = VOS_NULL;
mdrv_diag_layer_msg_notify_func g_pLayerMatchNotifyFunc = VOS_NULL;

/*
 * 3 Function
 */


unsigned int mdrv_diag_get_conn_state(void)
{
    return (VOS_UINT32)((DIAG_IS_CONN_ON) ? 1 : 0);
}

/*
 * Function Name: mdrv_diag_log_port_switch
 * Description: ??????????????????(?????????NAS??????at^logport?????????)
 * Input: VOS_UINT32 ulPhyPort: ??????????????????????????????(USB or VCOM)
 *        VOS_BOOL ulEffect: ??????????????????
 * Return: VOS_OK:success
 *         VOS_ERR: error
 *         ERR_MSP_AT_CHANNEL_BUSY:diag???????????????????????????USB?????????VCOM
 */
unsigned int mdrv_diag_log_port_switch(unsigned int phy_port, unsigned int effect)
{
    VOS_UINT32 enLogPort = 0;  // ???????????????logport
    VOS_UINT32 DiagStatus;     // ??????????????????
    VOS_UINT32 ulRet;

    (VOS_VOID)mdrv_ppm_query_log_port(&enLogPort); /* ????????????????????????????????? */

    DiagStatus = mdrv_diag_get_conn_state(); /* ??????diag??????????????? */

    /* diag??????????????????????????????USB?????????VCOM */
    if ((1 == DiagStatus) && (DIAG_CPM_OM_PORT_TYPE_USB == enLogPort) && (DIAG_CPM_OM_PORT_TYPE_VCOM == phy_port)) {
        diag_crit("diag connected, USB does not allowed to change to vcom\n");
        return ERR_MSP_AT_CHANNEL_BUSY;
    } else {
        ulRet = mdrv_ppm_log_port_switch(phy_port, effect);
    }
    return ulRet;
}


VOS_UINT32 diag_GetLayerMsgCfg(VOS_UINT32 ulCatId, VOS_UINT32 ulMsgId)
{
    DIAG_CFG_LOG_CAT_MSG_CFG_STRU *pstItemCfg = NULL;
    VOS_UINT32 i;

    for (i = 0; i < g_stMsgCfg.ulCfgCnt; i++) {
        pstItemCfg = (g_stMsgCfg.astMsgCfgList + i);

        if (ulMsgId == pstItemCfg->ulId) {
            if (DIAG_CFG_SWT_CLOSE == pstItemCfg->ulSwt) {
                return ERR_MSP_CFG_LOG_NOT_ALLOW;
            } else if (DIAG_CFG_SWT_OPEN == pstItemCfg->ulSwt) {
                return ERR_MSP_SUCCESS;
            } else {
                return ERR_MSP_DIAG_MSG_CFG_NOT_SET;
            }
        }
    }

    return ERR_MSP_DIAG_MSG_CFG_NOT_SET;
}

VOS_UINT32 diag_GetLayerSrcCfg(VOS_UINT32 ulModuleId)
{
    VOS_UINT32 ulOffset = 0;

    if (DOPRA_PID_TIMER == ulModuleId) {
        return ERR_MSP_SUCCESS;
    }

    if (DIAG_CFG_LAYER_MODULE_IS_ACORE(ulModuleId)) {
        ulOffset = DIAG_CFG_LAYER_MODULE_ACORE_OFFSET(ulModuleId);

        if (DIAG_CFG_SWT_OPEN == g_ALayerSrcModuleCfg[ulOffset]) {
            return ERR_MSP_SUCCESS;
        }
    } else if (DIAG_CFG_LAYER_MODULE_IS_CCORE(ulModuleId)) {
        ulOffset = DIAG_CFG_LAYER_MODULE_CCORE_OFFSET(ulModuleId);

        if (DIAG_CFG_SWT_OPEN == g_CLayerSrcModuleCfg[ulOffset]) {
            return ERR_MSP_SUCCESS;
        }
    } else if (DIAG_CFG_LAYER_MODULE_IS_NRM(ulModuleId)) {
        ulOffset = DIAG_CFG_LAYER_MODULE_NRM_OFFSET(ulModuleId);

        if (DIAG_CFG_SWT_OPEN == g_NrmLayerSrcModuleCfg[ulOffset]) {
            return ERR_MSP_SUCCESS;
        }
    }

    return ERR_MSP_CFG_LOG_NOT_ALLOW;
}

VOS_UINT32 diag_GetLayerDstCfg(VOS_UINT32 ulModuleId)
{
    VOS_UINT32 ulOffset = 0;

    if (DIAG_CFG_LAYER_MODULE_IS_ACORE(ulModuleId)) {
        ulOffset = DIAG_CFG_LAYER_MODULE_ACORE_OFFSET(ulModuleId);

        if (DIAG_CFG_SWT_OPEN == g_ALayerDstModuleCfg[ulOffset]) {
            return ERR_MSP_SUCCESS;
        }
    } else if (DIAG_CFG_LAYER_MODULE_IS_CCORE(ulModuleId)) {
        ulOffset = DIAG_CFG_LAYER_MODULE_CCORE_OFFSET(ulModuleId);

        if (DIAG_CFG_SWT_OPEN == g_CLayerDstModuleCfg[ulOffset]) {
            return ERR_MSP_SUCCESS;
        }
    } else if (DIAG_CFG_LAYER_MODULE_IS_NRM(ulModuleId)) {
        ulOffset = DIAG_CFG_LAYER_MODULE_NRM_OFFSET(ulModuleId);

        if (DIAG_CFG_SWT_OPEN == g_NrmLayerDstModuleCfg[ulOffset]) {
            return ERR_MSP_SUCCESS;
        }
    }

    return ERR_MSP_CFG_LOG_NOT_ALLOW;
}


VOS_UINT32 diag_GetLayerCfg(VOS_UINT32 ulSrcModuleId, VOS_UINT32 ulDstModuleId, VOS_UINT32 ulMsgId)
{
    VOS_UINT32 ret = ERR_MSP_CFG_LOG_NOT_ALLOW;
    VOS_UINT32 ret2 = ERR_MSP_CFG_LOG_NOT_ALLOW;
    VOS_UINT32 ulMsgCfg;

    /* ???????????????ID??????????????????????????????????????????????????????ID?????? */
    ulMsgCfg = diag_GetLayerMsgCfg(DIAG_CMD_LOG_CATETORY_LAYER_ID, ulMsgId);
    if (ERR_MSP_DIAG_MSG_CFG_NOT_SET == ulMsgCfg) {
        ret = diag_GetLayerSrcCfg(ulSrcModuleId);
        ret2 = diag_GetLayerDstCfg(ulDstModuleId);
        if ((ERR_MSP_SUCCESS != ret) && (ERR_MSP_SUCCESS != ret2)) {
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        } else {
            return ERR_MSP_SUCCESS;
        }
    } else if (ERR_MSP_SUCCESS == ulMsgCfg) {
        return ERR_MSP_SUCCESS;
    } else {
        return ERR_MSP_CFG_LOG_NOT_ALLOW;
    }
}


VOS_UINT32 diag_GetPrintCfg(VOS_UINT32 ulModuleId, VOS_UINT32 ulLevel)
{
    VOS_UINT32 ulLevelFilter = 0;
    VOS_UINT32 ulTmp = 0;

    if ((PS_LOG_LEVEL_OFF == ulLevel) || (ulLevel >= PS_LOG_LEVEL_BUTT)) {
        diag_LNR(EN_DIAG_LNR_LOG_LOST, ulLevel, 0xAAAAAAAA);
        return ERR_MSP_CFG_LOG_NOT_ALLOW;
    }

    /* ???????????????LEVEL????????????MSP???HSO?????????LEVEL??? */
    /*
     * TOOL...............MSP   ...... PS
     * 0x40000000??????0x40  ?????? 1 (ERROR);
     * 0x20000000??????0x20  ?????? 2 (WARNING);
     * 0x10000000??????0x10  ?????? 3 (NORMAL);
     * 0x08000000??????0x08  ?????? 4 (INFO)
     */

    ulLevelFilter = ((VOS_UINT32)1 << (7 - ulLevel));

    ulTmp = (ulLevelFilter << 16) | g_PrintTotalCfg;

    /* ????????????????????????????????? */
    if (DIAG_CFG_PRINT_TOTAL_MODULE_SWT_NOT_USE != g_PrintTotalCfg) {
        if (ulLevelFilter & g_PrintTotalCfg) {
            return ERR_MSP_SUCCESS;
        } else {
            diag_LNR(EN_DIAG_LNR_LOG_LOST, ulTmp, ulModuleId);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }
    } else {
        /* ?????????????????????????????? */
        if (DIAG_CFG_MODULE_IS_INVALID((VOS_INT32)ulModuleId)) {
            diag_LNR(EN_DIAG_LNR_LOG_LOST, ulModuleId, 0xBBBBBBBB);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }

        ulTmp = (ulLevelFilter << 16) | g_PrintModuleCfg[ulModuleId - VOS_PID_DOPRAEND];

        if (ulLevelFilter & g_PrintModuleCfg[ulModuleId - VOS_PID_DOPRAEND]) {
            return ERR_MSP_SUCCESS;
        } else {
            diag_LNR(EN_DIAG_LNR_LOG_LOST, ulTmp, ulModuleId);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }
    }
}
#ifdef DIAG_SYSTEM_5G
VOS_UINT32 diag_GetPrintPowerOnCfg(VOS_UINT32 ulLevel)
{
    if (((DIAGLOG_POWER_LOG_PROFILE_SIMPLE == g_ulDiagProfile) && (ulLevel <= PS_LOG_LEVEL_ERROR)) ||
        ((DIAGLOG_POWER_LOG_PROFILE_NORAML == g_ulDiagProfile) && (ulLevel <= PS_LOG_LEVEL_NORMAL)) ||
        ((DIAGLOG_POWER_LOG_PROFILE_WHOLE == g_ulDiagProfile) && (ulLevel <= PS_LOG_LEVEL_INFO))) {
        return ERR_MSP_SUCCESS;
    } else {
        return ERR_MSP_CFG_LOG_NOT_ALLOW;
    }
}

#endif

/*
 *  ??? ??? ???: mdrv_diag_log_report
 *  ????????????: ??????????????????
 *  ????????????: ulModuleId( 31-24:modemid,23-16:modeid,15-12:level )
 *            pcFileName(??????????????????????????????????????????????????????)
 *            ulLineNum(??????)
 *            pszFmt(????????????)
 * ????????????: ?????????????????????????????????????????????????????????????????????????????????????????????1K????????????????????????????????????????????????
 *           ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????1K?????????
 *                                         ??????????????????????????????log????????????1K?????????????????????????????????
 */
VOS_UINT32 mdrv_diag_log_report(VOS_UINT32 ulModuleId, VOS_UINT32 ulPid, VOS_CHAR *cFileName, VOS_UINT32 ulLineNum,
                          VOS_CHAR *pszFmt, ...)
{
    VOS_UINT32 ret = ERR_MSP_SUCCESS;
    VOS_UINT32 module = 0;
    VOS_UINT32 ulLevel = 0;
    va_list arg;

    /* ?????????????????? */
    module = ulPid;

    /* ???????????????????????? */
    ulLevel = DIAG_GET_PRINTF_LEVEL(ulModuleId);

    if (!DIAG_IS_POLOG_ON) {
        /* ??????DIAG??????????????????HSO??????????????? */
        if (!DIAG_IS_CONN_ON) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_PRINTFV_ERR, ERR_MSP_NO_INITILIZATION, ulModuleId, 1);
            return ERR_MSP_NO_INITILIZATION;
        }

        ret = diag_GetPrintCfg(module, ulLevel);
        if (ERR_MSP_SUCCESS != ret) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_PRINTFV_ERR, ERR_MSP_UNAVAILABLE, ulModuleId, 2);
            return ERR_MSP_UNAVAILABLE;
        }
    }
#ifdef DIAG_SYSTEM_5G
    else {
        ret = diag_GetPrintPowerOnCfg(ulLevel);
        if (ERR_MSP_SUCCESS != ret) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_PRINTFV_ERR, ret, ulModuleId, 5);
            return ret;
        }
    }
#endif

    va_start(arg, pszFmt);

    ret = mdrv_diag_report_log(ulModuleId, module, cFileName, ulLineNum, pszFmt, arg);
    if (ret) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_PRINTFV_ERR, ret, 0, 3);
    } else {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_PRINTFV_OK, 0, 0, 4);
    }

    va_end(arg);

    return ret;
}

/*
 * ??? ??? ???: mdrv_diag_trans_report_ex
 * ????????????: ?????????????????????????????????(???DIAG_TransReport????????????DIAG_MESSAGE_TYPE)
 * ????????????: mdrv_diag_trans_ind_s->module( 31-24:modemid,23-16:modeid,15-12:level,11-8:groupid )
 *           mdrv_diag_trans_ind_s->ulMsgId(????????????ID)
 *           mdrv_diag_trans_ind_s->ulLength(?????????????????????)
 *           mdrv_diag_trans_ind_s->pData(????????????)
 */
VOS_UINT32 mdrv_diag_trans_report_ex(mdrv_diag_trans_ind_s *pstData)
{
    VOS_UINT32 ret = ERR_MSP_SUCCESS;

    if (!DIAG_IS_POLOG_ON) {
        if (!DIAG_IS_CONN_ON) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_ERR, ERR_MSP_NO_INITILIZATION, 0, 0);
            return ERR_MSP_NO_INITILIZATION;
        }
#ifdef DIAG_SYSTEM_5G
        if (DIAG_CFG_SWT_CLOSE == g_ulTransCfg) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_ERR, ERR_MSP_CFG_LOG_NOT_ALLOW, 0, 1);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }
#endif
    }
#ifdef DIAG_SYSTEM_5G
    else {
        if (g_ulDiagProfile <= DIAGLOG_POWER_LOG_PROFILE_SIMPLE) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_ERR, ERR_MSP_CFG_LOG_NOT_ALLOW, 0, 2);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }
    }
#endif

    /* ????????????????????? */
    if ((VOS_NULL_PTR == pstData) || (NULL == pstData->pData) || (0 == pstData->ulLength)) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_ERR, ERR_MSP_NO_INITILIZATION, 0, 3);
        return ERR_MSP_INVALID_PARAMETER;
    }

    diag_LNR(EN_DIAG_LNR_TRANS_IND, pstData->ulMsgId, VOS_GetSlice());

    ret = mdrv_diag_report_trans((diag_trans_ind_s *)pstData);
    if (ret) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_ERR, ret, 0, 4);
    } else {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_OK, pstData->ulMsgId, VOS_GetSlice(), 5);
    }

    return ret;
}


VOS_UINT32 mdrv_diag_trans_report(mdrv_diag_trans_ind_s *pstData)
{
    /* ??????DIAG??????????????????HSO??????????????? */
    if ((!DIAG_IS_CONN_ON) && (!DIAG_IS_POLOG_ON)) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_ERR, ERR_MSP_NO_INITILIZATION, 0, 1);
        return ERR_MSP_NO_INITILIZATION;
    }

    /* ????????????????????? */
    if ((VOS_NULL_PTR == pstData) || (NULL == pstData->pData) || (0 == pstData->ulLength)) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRANS_ERR, ERR_MSP_NO_INITILIZATION, 0, 2);
        return ERR_MSP_INVALID_PARAMETER;
    }

    pstData->ulModule = MDRV_DIAG_GEN_MODULE_EX( DIAG_GET_MODEM_ID(pstData->ulModule), \
                                            DIAG_GET_MODE_ID(pstData->ulModule),  \
                                            DIAG_MSG_TYPE_PS);

    return mdrv_diag_trans_report_ex(pstData);
}


VOS_UINT32 mdrv_diag_event_report(mdrv_diag_event_ind_s *pstEvent)
{
    VOS_UINT32 ret = ERR_MSP_SUCCESS;

    if (VOS_NULL_PTR == pstEvent) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_EVENT_ERR, ERR_MSP_INVALID_PARAMETER, 0, 2);
        return ERR_MSP_INVALID_PARAMETER;
    }

    if (!DIAG_IS_POLOG_ON) {
        /* ?????????????????????????????? */
        if (!DIAG_IS_EVENT_ON) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_EVENT_ERR, ERR_MSP_NO_INITILIZATION, 0, 1);
            return ERR_MSP_NO_INITILIZATION;
        }

        /* ??????GU???event????????????????????????????????? */
        if ((DIAG_CFG_MODULE_IS_VALID(pstEvent->ulPid)) &&
            (0 == g_EventModuleCfg[pstEvent->ulPid - VOS_PID_DOPRAEND])) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_EVENT_ERR, ERR_MSP_INVALID_PARAMETER, 0, 3);
            return ERR_MSP_INVALID_PARAMETER;
        }
    }

    ret = mdrv_diag_report_event((diag_event_ind_s *)pstEvent);
    if (ret) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_EVENT_ERR, ret, 0, 4);
    } else {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_EVENT_OK, 0, 0, 5);
    }
    return ret;
}

/*
 * ??? ??? ???: mdrv_diag_allow_air_msg_report
 * ????????????: ??????????????????,????????????????????????????????????AirMsg
 * ????????????: VOS_VOID
 * ?????????: ERR_MSP_SUCCESS = 0 ??????????????????????????????
 *         ERR_MSP_CFG_LOG_NOT_ALLOW = 203 ?????????????????????????????????
 */
unsigned int mdrv_diag_allow_air_msg_report(void)
{
    /* ??????????????????LT ???????????? */
    if ((!DIAG_IS_LT_AIR_ON) && (!DIAG_IS_POLOG_ON)) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_AIR_ERR, ERR_MSP_NO_INITILIZATION, 0, 1);
        return ERR_MSP_CFG_LOG_NOT_ALLOW;
    }
    return ERR_MSP_SUCCESS;
}


VOS_UINT32 mdrv_diag_air_report(mdrv_diag_air_ind_s *pstAir)
{
    VOS_UINT32 ret = ERR_MSP_SUCCESS;

    /* ??????????????????LT ???????????? */
    if ((!DIAG_IS_LT_AIR_ON) && (!DIAG_IS_POLOG_ON)) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_AIR_ERR, ERR_MSP_NO_INITILIZATION, 0, 1);
        return ERR_MSP_NO_INITILIZATION;
    }

    /* ????????????????????? */
    if ((VOS_NULL == pstAir) || (VOS_NULL == pstAir->pData) || (0 == pstAir->ulLength)) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_AIR_ERR, ERR_MSP_INVALID_PARAMETER, 0, 2);
        return ERR_MSP_INVALID_PARAMETER;
    }

    ret = mdrv_diag_report_air((diag_air_ind_s *)pstAir);
    if (ret) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_AIR_ERR, ret, 0, 3);
    } else {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_AIR_OK, 0, 0, 4);
    }

    return ret;
}


VOS_VOID mdrv_diag_trace_report(VOS_VOID *pMsg)
{
    diag_osa_msg_head_s *pDiagMsg = (diag_osa_msg_head_s *)pMsg;
    VOS_UINT32 ret, ulSrcModule, ulDstModule, ulMsgId;

    if (VOS_NULL == pDiagMsg) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRACE_ERR, ERR_MSP_INVALID_PARAMETER, 0, 1);
        return;
    }

    if (pDiagMsg->ulLength < LAYER_MSG_MIN_LEN) {
        return;
    }

    ulSrcModule = pDiagMsg->ulSenderPid;
    ulDstModule = pDiagMsg->ulReceiverPid;
    ulMsgId = pDiagMsg->ulMsgId;

    if (!DIAG_IS_POLOG_ON) {
        /* ??????DIAG??????????????????HSO??????????????? */
        if (!DIAG_IS_CONN_ON) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRACE_ERR, ERR_MSP_NO_INITILIZATION, 0, 1);
            return;
        }

        /* ???????????????????????????????????? */
        ret = diag_GetLayerCfg(ulSrcModule, ulDstModule, ulMsgId);
        if (ret) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRACE_ERR, ERR_MSP_CFG_LOG_NOT_ALLOW, 0, 1);
            return;
        }
    }
#ifdef DIAG_SYSTEM_5G
    else {
        if (g_ulDiagProfile <= DIAGLOG_POWER_LOG_PROFILE_SIMPLE) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRACE_ERR, ERR_MSP_CFG_LOG_NOT_ALLOW, 0, 6);
            return;
        }
    }
#endif

    ret = mdrv_diag_report_trace(pMsg, DIAG_MODEM_0);
    if (ret) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRACE_ERR, ret, 0, 3);
    } else {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_CBT_API_TRACE_OK, 0, 0, 5);
    }
    return;
}


VOS_UINT32 mdrv_diag_layer_msg_report(VOS_VOID *pMsg)
{
    diag_osa_msg_head_s *pDiagMsg = (diag_osa_msg_head_s *)pMsg;
    diag_osa_msg_head_s *pNewMsg = NULL;
    VOS_UINT32 ret, ulSrcModule, ulDstModule, ulMsgId;

    if (VOS_NULL == pDiagMsg) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_API_MSG_LAYER_ERR, ERR_MSP_INVALID_PARAMETER, 0, 1);
        return ERR_MSP_INVALID_PARAMETER;
    }

    if (pDiagMsg->ulLength < LAYER_MSG_MIN_LEN) {
        return ERR_MSP_INVALID_PARAMETER;
    }

    ulSrcModule = pDiagMsg->ulSenderPid;
    ulDstModule = pDiagMsg->ulReceiverPid;
    ulMsgId = pDiagMsg->ulMsgId;

    if (!DIAG_IS_POLOG_ON) {
        /* ??????DIAG??????????????????HSO??????????????? */
        if (!DIAG_IS_CONN_ON) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_API_MSG_LAYER_ERR, ERR_MSP_NO_INITILIZATION, 0, 2);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }

        /* ???????????????????????????????????? */
        ret = diag_GetLayerCfg(ulSrcModule, ulDstModule, ulMsgId);
        if (ret) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_API_MSG_LAYER_ERR, ERR_MSP_CFG_LOG_NOT_ALLOW, 0, 3);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }
    }

    pNewMsg = pDiagMsg;

    /* ??????????????????????????????????????????????????????????????? */
    /* ????????????????????????????????????????????????????????????????????????????????????????????????notify????????????????????????????????? */
    if (VOS_NULL != g_pLayerMatchFunc) {
        pNewMsg = g_pLayerMatchFunc(pDiagMsg);
        if (VOS_NULL == pNewMsg) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_API_MSG_LAYER_MATCH, ulSrcModule, ulDstModule, ulMsgId);
            return ERR_MSP_CFG_LOG_NOT_ALLOW;
        }
    }

    ret = mdrv_diag_report_trace(pNewMsg, DIAG_MODEM_0);
    if (ret) {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_API_MSG_LAYER_ERR, ret, 0, 4);
    } else {
        DIAG_DEBUG_SDM_FUN(EN_DIAG_API_MSG_LAYER_OK, 0, 0, 5);
    }

    if ((pNewMsg != pDiagMsg) && (g_pLayerMatchNotifyFunc != NULL)) {
        ret = g_pLayerMatchNotifyFunc(ulSrcModule, pNewMsg);
        if (ret) {
            DIAG_DEBUG_SDM_FUN(EN_DIAG_API_MSG_LAYER_NOTIFY, 0, 0, 6);
        }
    }
    return ERR_MSP_SUCCESS;
}


unsigned int mdrv_diag_layer_msg_match_func_reg(mdrv_diag_layer_msg_match_func pfunc)
{
    if (VOS_NULL == g_pLayerMatchFunc) {
        g_pLayerMatchFunc = pfunc;
        return ERR_MSP_SUCCESS;
    } else {
        return ERR_MSP_FAILURE;
    }
}
/*
 * Function Name: mdrv_diag_layer_msg_notify_func_reg
 * Description: ????????????????????????????????????notify??????(?????????????????????????????????????????????????????????)
 *              ????????????logfilter???????????????????????????
 * Input:
 * Output: None
 * Return: ????????????????????????: 0-?????????????????????-????????????
 */
unsigned int mdrv_diag_layer_msg_notify_func_reg(mdrv_diag_layer_msg_notify_func pfunc)
{
    if (VOS_NULL == g_pLayerMatchNotifyFunc) {
        g_pLayerMatchNotifyFunc = pfunc;
        return ERR_MSP_SUCCESS;
    } else {
        return ERR_MSP_FAILURE;
    }
}

/* ???MBB?????????errorlog??????????????????MBB???phone?????? */
VOS_UINT32 mdrv_diag_error_log(VOS_CHAR * cFileName,VOS_UINT32 ulFileId, VOS_UINT32 ulLine,
                VOS_UINT32 ulErrNo, VOS_VOID * pBuf, VOS_UINT32 ulLen)
{
    return 0;
}


VOS_UINT32 diag_SendMsg(VOS_UINT32 ulSenderId, VOS_UINT32 ulRecverId, VOS_UINT32 ulMsgId, VOS_UINT8 *pDta,
                        VOS_UINT32 dtaSize)
{
    VOS_UINT32 ret = ERR_MSP_UNKNOWN;
    DIAG_DATA_MSG_STRU *pDataMsg = NULL;
    errno_t err = EOK;

    pDataMsg = (DIAG_DATA_MSG_STRU *)VOS_AllocMsg(ulSenderId,
                                                  (sizeof(DIAG_DATA_MSG_STRU) + dtaSize - VOS_MSG_HEAD_LENGTH));

    if (pDataMsg != NULL) {
        pDataMsg->ulReceiverPid = ulRecverId;
        pDataMsg->ulSenderPid = ulSenderId;
        pDataMsg->ulLength = sizeof(DIAG_DATA_MSG_STRU) + dtaSize - 20;

        pDataMsg->ulMsgId = ulMsgId;
        pDataMsg->ulLen = dtaSize;
        if (0 != dtaSize) {
            err = memcpy_s(pDataMsg->pContext, pDataMsg->ulLen, pDta, dtaSize);
            if (err != EOK) {
                diag_error("memcpy fail:%x\n", err);
            }
        }

        ret = VOS_SendMsg(ulSenderId, pDataMsg);
        if (ret != VOS_OK) {
            diag_error("ulRecverId=0x%x,ulMsgId=0x%x,dtaSize=0x%x,ret=0x%x\n", ulRecverId, ulMsgId, dtaSize, ret);
        } else {
            ret = ERR_MSP_SUCCESS;
        }
    }

    return ret;
}

unsigned int mdrv_diag_get_log_level(unsigned int pid)
{
    VOS_UINT8 level;

    /* ????????????ID?????????CCPU?????????PS????????? */
    if ((VOS_PID_DOPRAEND <= pid) && (VOS_PID_BUTT > pid)) {
        if (g_PrintTotalCfg == DIAG_CFG_PRINT_TOTAL_MODULE_SWT_NOT_USE) {
            level = g_PrintModuleCfg[pid - VOS_PID_DOPRAEND];
        } else {
            level = (VOS_UINT8)g_PrintTotalCfg;
        }

        /* level???????????????(0|ERROR|WARNING|NORMAL|INFO|0|0|0) bit 6-3 ????????????ERROR-INFO */

        if (level & 0x08) {
            return PS_LOG_LEVEL_INFO;
        } else if (level & 0x10) {
            return PS_LOG_LEVEL_NORMAL;
        } else if (level & 0x20) {
            return PS_LOG_LEVEL_WARNING;
        } else if (level & 0x40) {
            return PS_LOG_LEVEL_ERROR;
        } else {
            return PS_LOG_LEVEL_OFF;
        }
    }

    return PS_LOG_LEVEL_OFF;
}

/*
 * Function Name: diag_FailedCmdCnf
 * Description: MSP???????????????????????????????????????
 * Input: pData     ???????????????????????????
 *        ulErrcode ???????????????????????????
 * Output: None
 * Return: VOS_UINT32
 * History:
 *    1.c64416      2016-02-15  Draft Enact
 */
VOS_UINT32 diag_FailedCmdCnf(msp_diag_frame_info_s *pData, VOS_UINT32 ulErrcode)
{
    DIAG_COMM_CNF_STRU stCnfCmd = {0};
    MSP_DIAG_CNF_INFO_STRU stDiagInfo = {0};

    mdrv_diag_ptr(EN_DIAG_PTR_FAIL_CMD_CNF, 1, pData->ulCmdId, ulErrcode);

    DIAG_MSG_COMMON_PROC(stDiagInfo, stCnfCmd, pData);

    stDiagInfo.ulMsgType = pData->stID.pri4b;

    stCnfCmd.ulRet = ulErrcode;

    return DIAG_MsgReport(&stDiagInfo, &stCnfCmd, sizeof(stCnfCmd));
}

/*
 * Function Name: diag_IsPowerOnLogOpen
 * Description: ????????????????????????log????????????????????????
 * Input: None
 * Output: None
 * Return: VOS_TRUE:yes ; VOS_FALSE:no
 */
VOS_BOOL diag_IsPowerOnLogOpen(VOS_VOID)
{
#ifdef DIAG_SYSTEM_5G
    DRV_NV_POWER_ON_LOG_SWITCH_STRU stNVPowerOnLog = {};
    VOS_UINT32 ulRet;
    VOS_UINT32 cBufUsable;

    /* read buff use able */
    cBufUsable = mdrv_diag_shared_mem_read(POWER_ON_LOG_BUFF);
    if ((ERR_MSP_FAILURE == cBufUsable) || (!cBufUsable)) {
        diag_crit("open log buff is invalid, ret:0x%x\n", cBufUsable);
        g_ulDiagProfile = DIAGLOG_POWER_LOG_PROFILE_OFF;
        return VOS_FALSE;
    }

    /* read power on log swtich */
    ulRet = mdrv_nv_read(NV_ID_DRV_DIAG_POWER_LOG, &stNVPowerOnLog, sizeof(stNVPowerOnLog));
    if (ulRet) {
        diag_error("read power log nv fail, ret:0x%x\n", ulRet);
        g_ulDiagProfile = DIAGLOG_POWER_LOG_PROFILE_OFF;
        return VOS_FALSE;
    }

    if (!stNVPowerOnLog.cMasterSwitch) {
        diag_error("power on log is closed\n");
        g_ulDiagProfile = DIAGLOG_POWER_LOG_PROFILE_OFF;
        return VOS_FALSE;
    }

    /* set profile */
    g_ulDiagProfile = stNVPowerOnLog.cswACPUDiag;
    return VOS_TRUE;
#else
    VOS_UINT32 ulRet;
    VOS_INT32 stPowerOnLogA;
    NV_POWER_ON_LOG_SWITCH_STRU stNVPowerOnLog = {};

    stPowerOnLogA = mdrv_diag_shared_mem_read(POWER_ON_LOG_A);
    if (stPowerOnLogA) {
        ulRet = mdrv_nv_read(EN_NV_ID_POWER_ON_LOG_SWITCH, &stNVPowerOnLog, sizeof(stNVPowerOnLog));
        if ((ERR_MSP_SUCCESS == ulRet) && (stNVPowerOnLog.cPowerOnlogC)) {
            return VOS_TRUE;
        }
        return VOS_FALSE;
    }

    return VOS_FALSE;
#endif
}

EXPORT_SYMBOL(mdrv_diag_log_port_switch);

