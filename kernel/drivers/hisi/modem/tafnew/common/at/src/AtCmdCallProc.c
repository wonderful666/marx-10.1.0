/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
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

#include "AtCmdCallProc.h"

#include "AtSndMsg.h"
#include "ATCmdProc.h"
#include "taf_ccm_api.h"
#include "AtTafAgentInterface.h"
#include "app_vc_api.h"
#include "securec.h"
#include "AtEventReport.h"


/*
 * ??????????????????????.C??????????
 */
#define THIS_FILE_ID PS_FILE_ID_AT_CMD_CALL_PROC_C

#define AT_ECLSTART_PARA_MAX_NUM 4
#define AT_ECLSTART_ACTIVATION_TYPE 0
#define AT_ECLSTART_TYPE_OF_CALL 1
#define AT_ECLSTART_DIAL_NUM 2
#define AT_ECLSTART_OPRT_MADE 3

#define AT_ECLCFG_PARA_MAX_NUM 3
#define AT_ECLCFG_MODE 0
#define AT_ECLCFG_VOC_CONFIG 1
#define AT_ECLCFG_REDIAL_CONFIG 2

#define AT_ECLMSD_ECALL_MSD 0

#define AT_CBURSTDTMF_PARA_NUM 4
#define AT_CBURSTDTMF_CALL_ID 0
#define AT_CBURSTDTMF_DTMF_KEY 1
#define AT_CBURSTDTMF_ON_LENGTH 2
#define AT_CBURSTDTMF_OFF_LENGTH 3

#define AT_CCONTDTMF_DTMF_KEY_VALID_LEN 1
#define AT_CCONTDTMF_DTMF_KEY_SWITCH_STOP 0
#define AT_CCONTDTMF_DTMF_KEY 2

#define AT_ECLMODE_ECALLFORCE_MODE 0
#define AT_ECLMODE_ECALLFORCE_MODE_MAX_VALUE 2

#define AT_REJCALL_PARA_NUM 2

#define AT_OUTPUT_MAX_LENGTH 2
#define AT_DIGIT_MAX_LENGTH 2

#if (FEATURE_ECALL == FEATURE_ON)

VOS_UINT32 AT_SetCecallPara(VOS_UINT8 indexNum)
{
    TAF_Ctrl          ctrl;
    MN_CALL_OrigParam callOrigPara;
    VOS_UINT32        rst;
    ModemIdUint16     modemId;

    (VOS_VOID)memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    (VOS_VOID)memset_s(&callOrigPara, sizeof(callOrigPara), 0x00, sizeof(callOrigPara));

    /*
     * ??????????1??
     * ??????????1????????????????????0??
     * ??"AT+CECALL="????????g_atParaIndex??0
     */
    if (g_atParaIndex != 1) {
        return AT_ERROR;
    }

    switch (g_atParaList[0].paraValue) {
        case AT_ECALL_TYPE_TEST:
            callOrigPara.callType = MN_CALL_TYPE_TEST;
            break;
        case AT_ECALL_TYPE_RECFGURATION:
            callOrigPara.callType = MN_CALL_TYPE_RECFGURATION;
            break;
        case AT_ECALL_TYPE_MIEC:
            callOrigPara.callType = MN_CALL_TYPE_MIEC;
            break;
        case AT_ECALL_TYPE_AIEC:
            callOrigPara.callType = MN_CALL_TYPE_AIEC;
            break;
        default:
            return AT_ERROR;
    }

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* AT??CCM???????????? */
    rst = TAF_CCM_CallCommonReq(&ctrl, &callOrigPara, ID_TAF_CCM_CALL_ORIG_REQ, sizeof(callOrigPara), modemId);

    if (rst == VOS_OK) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_CECALL_SET;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}

VOS_UINT32 AT_SetEclstartPara(VOS_UINT8 indexNum)
{
    TAF_Ctrl              ctrl;
    MN_CALL_OrigParam     callOrigPara;
    MN_CALL_CalledNum     dialNumber;
    VOS_UINT32            rst;
    ModemIdUint16         modemId;
    APP_VC_SetOprtmodeReq eclOprtModepara;

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    memset_s(&callOrigPara, sizeof(callOrigPara), 0x00, sizeof(callOrigPara));

    /*  ????1: ????????????????????0 */
    if ((g_atParaList[AT_ECLSTART_ACTIVATION_TYPE].paraLen == 0) ||
        (g_atParaList[AT_ECLSTART_TYPE_OF_CALL].paraLen == 0)) {
        return AT_ERROR;
    }

    /* ????2: ????????????, ????1??????????????>=2 */
    if (g_atParaIndex > AT_ECLSTART_PARA_MAX_NUM) {
        return AT_ERROR;
    }

    /* ????3:  AT^ECLSTART=1,1, ?????????????? */
    if ((g_atParaIndex == 3) && (g_atParaList[AT_ECLSTART_DIAL_NUM].paraLen == 0)) {
        return AT_ERROR;
    }

    /* ????4:  AT^ECLSTART=1,1,, ?????????????? */
    if ((g_atParaIndex == 4) && (g_atParaList[AT_ECLSTART_OPRT_MADE].paraLen == 0)) {
        return AT_ERROR;
    }

    /* ????5:  ????ecall?????? */
    if (AT_HaveEcallActive(indexNum, VOS_FALSE) == VOS_TRUE) {
        return AT_ERROR;
    }
    /* ?????? */
    memset_s(&eclOprtModepara, sizeof(eclOprtModepara), 0x00, sizeof(eclOprtModepara));

    /* oprt_mode ??????PUSH???? */
    if ((g_atParaList[AT_ECLSTART_OPRT_MADE].paraLen == 0) ||
        (g_atParaList[AT_ECLSTART_OPRT_MADE].paraValue == APP_VC_ECALL_OPRT_PUSH)) {
        eclOprtModepara.ecallOpMode = APP_VC_ECALL_OPRT_PUSH;
    } else {
        eclOprtModepara.ecallOpMode = APP_VC_ECALL_OPRT_PULL;
    }

    /* ????VC???????????????????? */
    rst = AT_FillAndSndAppReqMsg(g_atClientTab[indexNum].clientId, g_atClientTab[indexNum].opId,
                                 APP_VC_MSG_SET_ECALL_OPRTMODE_REQ, (VOS_UINT8 *)&eclOprtModepara,
                                 sizeof(eclOprtModepara), I0_WUEPS_PID_VC);

    if (rst == TAF_FAILURE) {
        return AT_ERROR;
    }

    /* ?????? */
    memset_s(&dialNumber, sizeof(dialNumber), 0x00, sizeof(dialNumber));

    /* ?????????? */
    if (g_atParaList[AT_ECLSTART_DIAL_NUM].paraLen > 0) {
        /* ?????????????????? */
        if (AT_FillCalledNumPara(g_atParaList[AT_ECLSTART_DIAL_NUM].para,
                                 g_atParaList[AT_ECLSTART_DIAL_NUM].paraLen, &dialNumber) != VOS_OK) {
            AT_UpdateCallErrInfo(indexNum, TAF_CS_CAUSE_INVALID_PARAMETER, VOS_NULL_PTR);
            return AT_ERROR;
        }
    }

    /* ^ECLSTART=x,0  ????????call */
    if (g_atParaList[AT_ECLSTART_TYPE_OF_CALL].paraValue == 0) {
        callOrigPara.callType = MN_CALL_TYPE_TEST;

        /* ?????????????????????????????????????????????????? */
        memcpy_s(&callOrigPara.dialNumber, sizeof(callOrigPara.dialNumber), &dialNumber, sizeof(dialNumber));
    } else if (g_atParaList[AT_ECLSTART_TYPE_OF_CALL].paraValue == 1) {
        /* ^ECLSTART=0,1  ????????????call */
        if (g_atParaList[AT_ECLSTART_ACTIVATION_TYPE].paraValue == 0) {
            callOrigPara.callType = MN_CALL_TYPE_MIEC;
        }
        /* ^ECLSTART=1,1  ????????????call */
        else {
            callOrigPara.callType = MN_CALL_TYPE_AIEC;
        }
    } else {
        callOrigPara.callType = MN_CALL_TYPE_RECFGURATION;

        /* ?????????????????????????????????????????????????? */
        memcpy_s(&callOrigPara.dialNumber, sizeof(callOrigPara.dialNumber), &dialNumber, sizeof(dialNumber));
    }

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* AT??CCM???????????? */
    rst = TAF_CCM_CallCommonReq(&ctrl, &callOrigPara, ID_TAF_CCM_CALL_ORIG_REQ, sizeof(callOrigPara), modemId);

    if (rst == VOS_OK) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ECLSTART_SET;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_SetEclstopPara(VOS_UINT8 indexNum)
{
    VOS_UINT32 rst;

    TAF_Ctrl          ctrl;
    MN_CALL_SupsParam callSupsPara;
    ModemIdUint16     modemId;

    /* ?????????????????? */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_CMD_NO_PARA) {
        return AT_ERROR;
    }

    /* ?????? */
    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    memset_s(&callSupsPara, sizeof(callSupsPara), 0x00, sizeof(callSupsPara));

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    callSupsPara.callSupsCmd = MN_CALL_SUPS_CMD_REL_ECALL;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* AT??CCM???????????? */
    rst = TAF_CCM_CallCommonReq(&ctrl, &callSupsPara, ID_TAF_CCM_CALL_SUPS_CMD_REQ, sizeof(callSupsPara), modemId);

    if (rst == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ECLSTOP_SET;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_SetEclcfgPara(VOS_UINT8 indexNum)
{
    VOS_UINT32               rst;
    APP_VC_MsgSetEcallCfgReq eclcfgSetPara;

    /* ??????: ????????????????????0 */
    if (g_atParaList[AT_ECLCFG_MODE].paraLen == 0) {
        return AT_ERROR;
    }

    /* ??????: ????????????, ??????????????????????>=1 */
    if (g_atParaIndex > AT_ECLCFG_PARA_MAX_NUM) {
        return AT_ERROR;
    }

    /* ????3:  AT^ECLCFG=0, ???? AT^ECLCFG=0,1, ?????????????? */
    if (((g_atParaIndex == AT_ECLCFG_PARA_MAX_NUM - 1) && (g_atParaList[AT_ECLCFG_VOC_CONFIG].paraLen == 0)) ||
        ((g_atParaIndex == AT_ECLCFG_PARA_MAX_NUM) && (g_atParaList[AT_ECLCFG_REDIAL_CONFIG].paraLen == 0))) {
        return AT_ERROR;
    }

    /* ?????? */
    memset_s(&eclcfgSetPara, sizeof(eclcfgSetPara), 0x00, sizeof(eclcfgSetPara));

    eclcfgSetPara.mode = (APP_VC_EcallMsdModeUint16)g_atParaList[AT_ECLCFG_MODE].paraValue;

    if (g_atParaList[AT_ECLCFG_VOC_CONFIG].paraLen == 0) {
        eclcfgSetPara.vocConfig = VOC_CONFIG_NO_CHANGE;
    } else {
        eclcfgSetPara.vocConfig = (APP_VC_EcallVocConfigUint16)g_atParaList[AT_ECLCFG_VOC_CONFIG].paraValue;
    }

    /* ????ECALL????????????????????????, ???????????????????? */
    if ((g_atParaList[AT_ECLCFG_REDIAL_CONFIG].paraLen != 0) &&
        (g_atParaList[AT_ECLCFG_REDIAL_CONFIG].paraValue == VOS_TRUE)) {
        return AT_ERROR;
    }

    rst = AT_FillAndSndAppReqMsg(g_atClientTab[indexNum].clientId, g_atClientTab[indexNum].opId,
                                 APP_VC_MSG_SET_ECALL_CFG_REQ, (VOS_UINT8 *)&eclcfgSetPara, sizeof(eclcfgSetPara),
                                 I0_WUEPS_PID_VC);

    if (rst == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ECLCFG_SET;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_SetEclmsdPara(VOS_UINT8 indexNum)
{
    VOS_UINT32          rst;
    APP_VC_MsgSetMsdReq eclmsdPara;

    /*
     * ??????????1??
     * ??????????1????????????????????0??
     * ??"AT+CELMSD="????????g_atParaIndex??0
     */
    if (g_atParaIndex != 1) {
        return AT_ERROR;
    }

    /*
     * ????????????
     * ??????????????????????????????????????????????????????????280????????????????MSD??????????140??????????????
     */
    if (((APP_VC_MSD_DATA_LEN * 2) != g_atParaList[AT_ECLMSD_ECALL_MSD].paraLen)) {
        return AT_ERROR;
    }

    if (At_AsciiString2HexSimple(eclmsdPara.msdData, g_atParaList[AT_ECLMSD_ECALL_MSD].para,
                                 APP_VC_MSD_DATA_LEN * 2) == AT_FAILURE) {
        return AT_ERROR;
    }

    rst = AT_FillAndSndAppReqMsg(g_atClientTab[indexNum].clientId, g_atClientTab[indexNum].opId, APP_VC_MSG_SET_MSD_REQ,
                                 (VOS_UINT8 *)&eclmsdPara, sizeof(eclmsdPara), I0_WUEPS_PID_VC);
    if (rst == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ECLMSD_SET;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_QryCecallPara(VOS_UINT8 indexNum)
{
    VOS_UINT32    rst;
    TAF_Ctrl      ctrl;
    ModemIdUint16 modemId;

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* ????????C?????????????????????? */
    rst = TAF_CCM_CallCommonReq(&ctrl, VOS_NULL_PTR, ID_TAF_CCM_QRY_ECALL_INFO_REQ, 0, modemId);

    if (rst == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_CECALL_QRY;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_QryEclcfgPara(VOS_UINT8 indexNum)
{
    VOS_UINT32 rst;

    rst = AT_FillAndSndAppReqMsg(g_atClientTab[indexNum].clientId, g_atClientTab[indexNum].opId,
                                 APP_VC_MSG_QRY_ECALL_CFG_REQ, VOS_NULL, 0, I0_WUEPS_PID_VC);

    if (rst == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ECLCFG_QRY;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_QryEclmsdPara(VOS_UINT8 indexNum)
{
    VOS_UINT32 rst;

    rst = AT_FillAndSndAppReqMsg(g_atClientTab[indexNum].clientId, g_atClientTab[indexNum].opId, APP_VC_MSG_QRY_MSD_REQ,
                                 VOS_NULL, 0, I0_WUEPS_PID_VC);

    if (rst == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ECLMSD_QRY;
        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_TestEclstartPara(VOS_UINT8 indexNum)
{
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s: (0,1),(0,1,2),(0,1)",
                                                     g_parseContext[indexNum].cmdElement->cmdName);
    return AT_OK;
}


VOS_UINT32 AT_TestEclmsdPara(VOS_UINT8 indexNum)
{
    /*
     * ^eclmsd????????????????
     * ????????????eclmsd??????????????ERROR????????"(MSD)"
     */
    return AT_ERROR;
}

VOS_UINT8 AT_HaveEcallActive(VOS_UINT8 indexNum, VOS_UINT8 checkFlag)
{
    VOS_UINT8 numOfCalls = 0;
    VOS_UINT8 i          = 0;
    VOS_UINT32 rst;
    TAFAGENT_CALL_InfoParam callInfos[MN_CALL_MAX_NUM];

    if (indexNum >= AT_MAX_CLIENT_NUM) {
        AT_ERR_LOG("AT_HaveEcallActive : ucIndex.");

        return VOS_FALSE;
    }
    memset_s(callInfos, sizeof(callInfos), 0x00, sizeof(callInfos));

    /* ??????API???????????? */
    rst = TAF_AGENT_GetCallInfoReq(g_atClientTab[indexNum].clientId, &numOfCalls, callInfos);

    if (rst != VOS_OK) {
        AT_ERR_LOG("AT_HaveEcallActive : TAF_AGENT_GetCallInfoReq.");

        return VOS_FALSE;
    }

    AT_NORM_LOG1("AT_HaveEcallActive : [ucCheckFlag]", checkFlag);
    numOfCalls = TAF_MIN(numOfCalls, MN_CALL_MAX_NUM);
    /* ??????ECALL ????????TRUE */
    for (i = 0; i < numOfCalls; i++) {
        if ((callInfos[i].callType == MN_CALL_TYPE_MIEC) || (callInfos[i].callType == MN_CALL_TYPE_AIEC) ||
            (callInfos[i].callType == MN_CALL_TYPE_PSAP_ECALL) ||
            (callInfos[i].callType == MN_CALL_TYPE_RECFGURATION) || (callInfos[i].callType == MN_CALL_TYPE_EMERGENCY) ||
            (callInfos[i].callType == MN_CALL_TYPE_TEST)) {
            if (checkFlag == VOS_TRUE) {
                if (callInfos[i].callState == MN_CALL_S_ACTIVE) {
                    return VOS_TRUE;
                }
            } else {
                return VOS_TRUE;
            }
        }
    }
    return VOS_FALSE;
}


VOS_UINT32 AT_SetEclpushPara(VOS_UINT8 indexNum)
{
    VOS_UINT32          rst;
    APP_VC_MsgSetMsdReq eclmsdPara;

    /* ?????????????????? */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_CMD_NO_PARA) {
        return AT_ERROR;
    }

    /* ??????Ecall????????????ERROR */
    if (AT_HaveEcallActive(indexNum, VOS_TRUE) == VOS_FALSE) {
        return AT_ERROR;
    }

    memset_s(&eclmsdPara, sizeof(APP_VC_MsgSetMsdReq), 0x00, sizeof(APP_VC_MsgSetMsdReq));
    /* ??VC????APP_VC_MSG_ECALL_PUSH_REQ???? */
    rst = AT_FillAndSndAppReqMsg(g_atClientTab[indexNum].clientId, g_atClientTab[indexNum].opId,
                                 APP_VC_MSG_SET_ECALL_PUSH_REQ, (VOS_UINT8 *)&eclmsdPara, sizeof(eclmsdPara),
                                 I0_WUEPS_PID_VC);

    if (rst == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ECLPUSH_SET;

        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}


VOS_UINT32 AT_SetEclAbortPara(VOS_UINT8 indexNum)
{
    VOS_UINT32 ret;
    VOS_UINT32 abortReason = 0;

    /* ?????????????????? */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_CMD_NO_PARA) {
        return AT_ERROR;
    }

    /* ??????Ecall????????????ERROR */
    if (AT_HaveEcallActive(indexNum, VOS_TRUE) == VOS_FALSE) {
        return AT_ERROR;
    }

    /* ??VC????APP_VC_MSG_ECALL_ABORT_REQ???? */
    ret = AT_FillAndSndAppReqMsg(g_atClientTab[indexNum].clientId, g_atClientTab[indexNum].opId,
                                 APP_VC_MSG_SET_ECALL_ABORT_REQ, (VOS_UINT8 *)&abortReason, sizeof(abortReason),
                                 I0_WUEPS_PID_VC);

    if (ret == TAF_SUCCESS) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_ABORT_SET;

        return AT_WAIT_ASYNC_RETURN;
    } else {
        return AT_ERROR;
    }
}

VOS_UINT32 AT_SetEclModePara(VOS_UINT8 indexNum)
{
    TAF_NVIM_CustomEcallCfg ecallCfg;
    VOS_UINT32              result;

    memset_s(&ecallCfg, sizeof(ecallCfg), 0x00, sizeof(ecallCfg));

    /* ???????? */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_PARA_CMD) {
        AT_ERR_LOG("AT_SetEclModePara: ucCmdOptType Error!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* ???????????? */
    if (g_atParaIndex != 1) {
        AT_ERR_LOG("AT_SetEclModePara: num Error!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* <mode>????????????????0-2 */
    if (g_atParaList[AT_ECLMODE_ECALLFORCE_MODE].paraValue > AT_ECLMODE_ECALLFORCE_MODE_MAX_VALUE) {
        AT_ERR_LOG("AT_SetEclModePara: value Error!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* ??????NV???? */
    result = TAF_ACORE_NV_READ(MODEM_ID_0, NV_ITEM_CUSTOM_ECALL_CFG, &ecallCfg, sizeof(ecallCfg));

    /* NV??????????NV??????????????error */
    if (result != NV_OK) {
        AT_ERR_LOG("AT_SetEclModePara: NV read!");
        return AT_ERROR;
    }

    if (ecallCfg.ecallForceMode != g_atParaList[AT_ECLMODE_ECALLFORCE_MODE].paraValue) {
        ecallCfg.ecallForceMode = (VOS_UINT8)g_atParaList[AT_ECLMODE_ECALLFORCE_MODE].paraValue;
        /* ????NV???? */
        result = TAF_ACORE_NV_WRITE(MODEM_ID_0, NV_ITEM_CUSTOM_ECALL_CFG, (VOS_UINT8 *)&ecallCfg, sizeof(ecallCfg));

        if (result != NV_OK) {
            return AT_ERROR;
        }

        AT_SetEclModeValue(ecallCfg.ecallForceMode);
    }

    return AT_OK;
}

VOS_UINT32 AT_QryEclModePara(VOS_UINT8 indexNum)
{
    VOS_UINT16              length;
    TAF_NVIM_CustomEcallCfg ecallCfg;
    VOS_UINT32              result;

    memset_s(&ecallCfg, sizeof(ecallCfg), 0x00, sizeof(ecallCfg));
    /* ??????NV???? */
    result = TAF_ACORE_NV_READ(MODEM_ID_0, NV_ITEM_CUSTOM_ECALL_CFG, &ecallCfg, sizeof(ecallCfg));

    /* NV??????????NV???????????????????? */
    if (result != NV_OK) {
        AT_ERR_LOG("AT_QryEclModePara: NV read error!");
    } else {
        AT_SetEclModeValue(ecallCfg.ecallForceMode);
    }

    length = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress, (VOS_CHAR *)g_atSndCodeAddress,
                                    "%s: %d", g_parseContext[indexNum].cmdElement->cmdName, AT_GetEclModeValue());
    g_atSendDataBuff.bufLen = length;
    return AT_OK;
}

VOS_UINT32 AT_TestEclModePara(VOS_UINT8 indexNum)
{
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s: %s",
                                                     g_parseContext[indexNum].cmdElement->cmdName,
                                                     g_parseContext[indexNum].cmdElement->param);
    return AT_OK;
}


VOS_VOID AT_EcallAlackDisplay(AT_ECALL_AlackValue ecallAlackInfo, VOS_UINT16 *length)
{
    TAF_INT8        timeZone = 0;
    TIME_ZONE_Time *timeInfo = VOS_NULL_PTR;

    timeInfo = &ecallAlackInfo.ecallAlackTimeInfo.universalTimeandLocalTimeZone;
    /* YYYY */
    *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                      (VOS_CHAR *)g_atSndCodeAddress + *length, "\"%4d/",
                                      timeInfo->year + 2000); /* year */

    /* MM */
    *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                      (VOS_CHAR *)g_atSndCodeAddress + *length, "%d%d/",
                                      timeInfo->month / 10,  /* month high */
                                      timeInfo->month % 10); /* month low */
    /* dd */
    *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                      (VOS_CHAR *)g_atSndCodeAddress + *length, "%d%d,",
                                      timeInfo->day / 10,  /* day high */
                                      timeInfo->day % 10); /* day high */

    /* hh */
    *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                      (VOS_CHAR *)g_atSndCodeAddress + *length, "%d%d:",
                                      timeInfo->hour / 10,  /* hour high */
                                      timeInfo->hour % 10); /* hour high */

    /* mm */
    *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                      (VOS_CHAR *)g_atSndCodeAddress + *length,
                                      "%d%d:", timeInfo->minute / 10, /* minutes high */
                                      timeInfo->minute % 10);         /* minutes high */

    /* ss */
    *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                      (VOS_CHAR *)g_atSndCodeAddress + *length, "%d%d",
                                      timeInfo->second / 10,  /* sec high */
                                      timeInfo->second % 10); /* sec high */

    /* ???????? */
    if ((ecallAlackInfo.ecallAlackTimeInfo.ieFlg & NAS_MM_INFO_IE_LTZ) == NAS_MM_INFO_IE_LTZ) {
        timeZone = ecallAlackInfo.ecallAlackTimeInfo.localTimeZone;
    } else {
        timeZone = timeInfo->timeZone;
    }

    if (timeZone == AT_INVALID_TZ_VALUE) {
        timeZone = 0;
    }

    if (timeZone != AT_INVALID_TZ_VALUE) {
        *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                          (VOS_CHAR *)g_atSndCodeAddress + *length, "%s%02d\"",
                                          (timeZone < 0) ? "-" : "+", (timeZone < 0) ? (-timeZone) : timeZone);
    }

    /* AlackValue */
    *length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                      (VOS_CHAR *)g_atSndCodeAddress + *length, ",%d", ecallAlackInfo.ecallAlackValue);
    return;
}


VOS_VOID AT_EcallAlAckListDisplay(VOS_VOID)
{
    VOS_UINT32          i                  = 0;
    VOS_UINT16          length             = 0;
    VOS_UINT32          readNum            = 0;
    AT_ECALL_AlackInfo *ecallAlackInfoAddr = VOS_NULL_PTR;

    ecallAlackInfoAddr = AT_EcallAlAckInfoAddr();

    /* ????????ALACK ???????????? */
    for (i = 0; i < (VOS_UINT32)AT_MIN(ecallAlackInfoAddr->ecallAlackNum, AT_ECALL_ALACK_NUM); i++) {
        length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (TAF_CHAR *)g_atSndCodeAddress,
                                         (TAF_CHAR *)g_atSndCodeAddress + length, "^ECLLIST: ");

        readNum = (ecallAlackInfoAddr->ecallAlackBeginNum + i) % AT_ECALL_ALACK_NUM;
        AT_EcallAlackDisplay(ecallAlackInfoAddr->ecallAlackInfo[readNum], &length);

        if (i + 1 < ecallAlackInfoAddr->ecallAlackNum) {
            /* ???????? */
            length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                             (VOS_CHAR *)g_atSndCodeAddress + length, "%s", g_atCrLf);
        }
    }

    g_atSendDataBuff.bufLen = length;
    return;
}


VOS_UINT32 AT_QryEclListPara(VOS_UINT8 indexNum)
{
    AT_EcallAlAckListDisplay();

    return AT_OK;
}


VOS_UINT32 AT_RcvVcMsgEcallPushCnfProc(MN_AT_IndEvt *data)
{
    VOS_UINT8         indexNum = 0;
    VOS_UINT32        ret      = VOS_ERR;
    APP_VC_SetMsdCnf *rslt     = VOS_NULL_PTR;

    if (data == VOS_NULL_PTR) {
        return VOS_ERR;
    }

    /* ????clientid????index */
    if (At_ClientIdToUserId(data->clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvVcMsgEcallPushCnfProc:WARNING:AT INDEX NOT FOUND!");

        return VOS_ERR;
    }

    if (AT_IS_BROADCAST_CLIENT_INDEX(indexNum)) {
        AT_WARN_LOG("AT_RcvVcMsgEcallPushCnfProc : AT_BROADCAST_INDEX.");

        return VOS_ERR;
    }

    /* ????AT???????????????????? */
    if (g_atClientTab[indexNum].cmdCurrentOpt != AT_CMD_ECLPUSH_SET) {
        AT_WARN_LOG("AT_RcvVcMsgEcallPushCnfProc:WARNING:AT ARE WAITING ANOTHER CMD!");

        return VOS_ERR;
    }

    rslt = (APP_VC_SetMsdCnf *)data->content;

    if (rslt->rslt == VOS_OK) {
        ret = AT_OK;
    } else {
        ret = AT_ERROR;
    }

    g_atSendDataBuff.bufLen = 0;

    /* ????AT???? */
    AT_STOP_TIMER_CMD_READY(indexNum);

    At_FormatResultData(indexNum, ret);

    return VOS_OK;
}


VOS_UINT32 AT_ProcVcEcallAbortCnf(VOS_UINT8 indexNum, APP_VC_EventInfo *vcEvtInfo)
{
    VOS_UINT32 rslt = AT_ERROR;

    /* ?????????????? */
    if (vcEvtInfo == VOS_NULL) {
        return VOS_ERR;
    }

    if (AT_IS_BROADCAST_CLIENT_INDEX(indexNum)) {
        AT_WARN_LOG("AT_ProcVcEcallAbortCnf : AT_BROADCAST_INDEX.");

        return VOS_ERR;
    }

    /* ????AT???????????????????? */
    if (g_atClientTab[indexNum].cmdCurrentOpt != AT_CMD_ABORT_SET) {
        return VOS_ERR;
    }

    if (vcEvtInfo->success != VOS_TRUE) {
        rslt = AT_ERROR;
    } else {
        rslt = AT_OK;
    }

    AT_STOP_TIMER_CMD_READY(indexNum);

    At_FormatResultData(indexNum, rslt);

    return VOS_OK;
}


VOS_UINT32 AT_ProcVcReportEcallAlackEvent(VOS_UINT8 indexNum, APP_VC_EventInfo *vcEvtInfo)
{
    NAS_MM_InfoInd      localAtTimeInfo;
    ModemIdUint16       modemId            = MODEM_ID_0;
    AT_ECALL_AlackInfo *ecallAlackInfoAddr = VOS_NULL_PTR;
    VOS_UINT32          rslt;
    VOS_UINT32          writeNum = 0;
    VOS_UINT16          length   = 0;
    AT_ModemNetCtx     *netCtx   = VOS_NULL_PTR;
    NAS_MM_InfoInd      cCLKTimeInfo;

    if (vcEvtInfo == VOS_NULL_PTR) {
        return VOS_ERR;
    }

    rslt = AT_GetModemIdFromClient(indexNum, &modemId);

    if (rslt != VOS_OK) {
        AT_ERR_LOG("AT_ProcVcReportEcallAlackEvent: Get modem id fail.");

        return VOS_ERR;
    }

    netCtx = AT_GetModemNetCtxAddrFromModemId(modemId);

    memset_s(&cCLKTimeInfo, sizeof(cCLKTimeInfo), 0x00, sizeof(cCLKTimeInfo));
    memset_s(&localAtTimeInfo, sizeof(localAtTimeInfo), 0x00, sizeof(localAtTimeInfo));

    if ((netCtx->timeInfo.ieFlg & NAS_MM_INFO_IE_UTLTZ) == NAS_MM_INFO_IE_UTLTZ) {
        /* ????????????????CCLK????????????localAT???? */
        AT_GetLiveTime(&netCtx->timeInfo, &localAtTimeInfo, netCtx->nwSecond);
    } else {
        memcpy_s(&cCLKTimeInfo, sizeof(cCLKTimeInfo), &netCtx->timeInfo, sizeof(NAS_MM_InfoInd));
        cCLKTimeInfo.ieFlg = NAS_MM_INFO_IE_UTLTZ;
        AT_GetLiveTime(&cCLKTimeInfo, &localAtTimeInfo, 1);
    }

    /* ??????????ALACK ?? */
    ecallAlackInfoAddr = AT_EcallAlAckInfoAddr();

    if (ecallAlackInfoAddr->ecallAlackNum < AT_ECALL_ALACK_NUM) {
        ecallAlackInfoAddr->ecallAlackNum++;
        writeNum = ecallAlackInfoAddr->ecallAlackNum - 1;
    } else {
        ecallAlackInfoAddr->ecallAlackBeginNum += 1;
        ecallAlackInfoAddr->ecallAlackBeginNum %= AT_ECALL_ALACK_NUM;

        /*
         * EcallAlAckList??BeginNum????0????writeNum????????????EcallAlAckList??????????????
         * ????????AT_ECALL_ALACK_NUM - 1
         */
        if (ecallAlackInfoAddr->ecallAlackBeginNum == 0) {
            writeNum = AT_ECALL_ALACK_NUM - 1;
        } else {
            writeNum = ecallAlackInfoAddr->ecallAlackBeginNum - 1;
        }
    }

    memcpy_s(&ecallAlackInfoAddr->ecallAlackInfo[writeNum].ecallAlackTimeInfo, sizeof(NAS_MM_InfoInd), &localAtTimeInfo,
             sizeof(localAtTimeInfo));

    ecallAlackInfoAddr->ecallAlackInfo[writeNum].ecallAlackValue = vcEvtInfo->ecallReportAlackValue;

    /* ??????????ALACK ?? */
    length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress, (VOS_CHAR *)g_atSndCodeAddress,
                                     "%s^ECLREC: ", g_atCrLf);

    AT_EcallAlackDisplay(ecallAlackInfoAddr->ecallAlackInfo[writeNum], &length);

    /* ???????? */
    length += (TAF_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                     (VOS_CHAR *)g_atSndCodeAddress + length, "%s", g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, length);

    At_SendResultData(AT_CLIENT_ID_CTRL, g_atSndCodeAddress, length);

    return VOS_OK;
}


VOS_VOID AT_RcvTafEcallStatusErrorInd(VOS_VOID)
{
    VOS_UINT16 length;

    length = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress, (VOS_CHAR *)g_atSndCodeAddress,
                                    "%s^ECLSTAT: 2,3%s", g_atCrLf, g_atCrLf);

    At_SendResultData(AT_BROADCAST_CLIENT_INDEX_MODEM_0, g_atSndCodeAddress, length);

    return;
}
#endif


VOS_UINT32 At_RcvVcMsgDtmfDecoderIndProc(MN_AT_IndEvt *data)
{
    APP_VC_DtmfDecoderInd *dtmfInd  = VOS_NULL_PTR;
    VOS_UINT8              indexNum = 0;
    VOS_CHAR               output[AT_OUTPUT_MAX_LENGTH];

    /* ????clientid????index */
    if (At_ClientIdToUserId(data->clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("At_RcvVcMsgDtmfDecoderIndProc:WARNING:AT INDEX NOT FOUND!");
        return VOS_ERR;
    }

    /* ?????? */
    dtmfInd   = (APP_VC_DtmfDecoderInd *)data->content;
    output[0] = dtmfInd->ucDtmfCode;
    output[1] = '\0';

    /* ???????????? */
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s^DDTMF: %s%s", g_atCrLf, output,
                                                     g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}
#if (FEATURE_UE_MODE_CDMA == FEATURE_ON)

VOS_UINT32 AT_CheckCfshNumber(VOS_UINT8 *atPara, VOS_UINT16 len)
{
    VOS_UINT16 loop;

    /* ??????????????????:+??????????????????????????????????33??????????????32 */
    if (atPara[0] == '+') {
        if (len > (TAF_CALL_MAX_FLASH_DIGIT_LEN + 1)) {
            return VOS_ERR;
        }

        atPara++;
        len--;
    } else {
        if (len > TAF_CALL_MAX_FLASH_DIGIT_LEN) {
            return VOS_ERR;
        }
    }

    /* ??????????????????(??????????????????????'+') */
    for (loop = 0; loop < len; loop++) {
        if (((atPara[loop] >= '0') && (atPara[loop] <= '9')) || (atPara[loop] == '*') || (atPara[loop] == '#')) {
            continue;
        } else {
            return VOS_ERR;
        }
    }

    return VOS_OK;
}


VOS_UINT32 AT_SetCfshPara(VOS_UINT8 indexNum)
{
    errno_t            memResult;
    VOS_UINT32         rst;
    TAF_Ctrl           ctrl;
    TAF_CALL_FlashPara flashPara;
    ModemIdUint16      modemId;

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    memset_s(&flashPara, sizeof(flashPara), 0x00, sizeof(flashPara));

    /* ???????? */
    if (g_atParaIndex > 1) {
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* ????????????<number>?????????????? */
    if (g_atParaIndex == 1) {
        if (AT_CheckCfshNumber(g_atParaList[0].para, g_atParaList[0].paraLen) != VOS_OK) {
            return AT_CME_INCORRECT_PARAMETERS;
        }
    } else {
        /* ????AT????AT^CFSH= ???????????? */
        if (g_atParseCmd.cmdOptType == AT_CMD_OPT_SET_PARA_CMD) {
            return AT_CME_INCORRECT_PARAMETERS;
        }
    }

    memset_s(&flashPara, sizeof(flashPara), 0x00, sizeof(TAF_CALL_FlashPara));

    flashPara.digitNum = (VOS_UINT8)g_atParaList[0].paraLen;
    if (g_atParaList[0].paraLen > 0) {
        memResult = memcpy_s(flashPara.digit, sizeof(flashPara.digit), g_atParaList[0].para, g_atParaList[0].paraLen);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(flashPara.digit), g_atParaList[0].paraLen);
    }

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* ????ID_TAF_CCM_SEND_FLASH_REQ???? */
    rst = TAF_CCM_CallCommonReq(&ctrl, &flashPara, ID_TAF_CCM_SEND_FLASH_REQ, sizeof(flashPara), modemId);

    if (rst == VOS_OK) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_CFSH_SET;
        return AT_WAIT_ASYNC_RETURN;
    }

    return AT_ERROR;
}


VOS_UINT32 AT_RcvTafCcmSndFlashRslt(struct MsgCB *msg)
{
    VOS_UINT8             indexNum     = 0;
    TAF_CCM_SendFlashCnf *sndFlashRslt = VOS_NULL_PTR;

    sndFlashRslt = (TAF_CCM_SendFlashCnf *)msg;

    /* ????ClientID???????????? */
    if (At_ClientIdToUserId(sndFlashRslt->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallSndFlashRslt: Get Index Fail!");
        return VOS_ERR;
    }

    /* AT??????????^CFSH?????????????????????? */
    if (g_atClientTab[indexNum].cmdCurrentOpt != AT_CMD_CFSH_SET) {
        AT_WARN_LOG("AT_RcvTafCallSndFlashRslt: Error Option!");
        return VOS_ERR;
    }

    /* ????AT_STOP_TIMER_CMD_READY????AT??????????????READY???? */
    AT_STOP_TIMER_CMD_READY(indexNum);

    /* ?????????????????????????????????? */
    if (sndFlashRslt->result.result == VOS_OK) {
        At_FormatResultData(indexNum, AT_OK);
    } else {
        At_FormatResultData(indexNum, AT_ERROR);
    }

    return VOS_OK;
}


VOS_UINT32 At_TestCBurstDTMFPara(VOS_UINT8 indexNum)
{
    VOS_UINT16 length;

    length = 0;

    length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                     (VOS_CHAR *)g_atSndCodeAddress + length,
                                     "^CBURSTDTMF: (1-7),(0-9,*,#),(95,150,200,250,300,350),(60,100,150,200)");
    g_atSendDataBuff.bufLen = length;

    return AT_OK;
}


VOS_UINT32 AT_SetCBurstDTMFPara(VOS_UINT8 indexNum)
{
    VOS_UINT32 rst;
    errno_t    memResult;
    VOS_UINT32 loop;
    VOS_UINT32 paraInvalidFlg;

    TAF_Ctrl               ctrl;
    TAF_CALL_BurstDtmfPara burstDTMFPara;
    ModemIdUint16          modemId;

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    memset_s(&burstDTMFPara, sizeof(burstDTMFPara), 0x00, sizeof(burstDTMFPara));

    /* ?????????????? */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_PARA_CMD) {
        return AT_CME_INCORRECT_PARAMETERS;
    }

    paraInvalidFlg = (g_atParaIndex != AT_CBURSTDTMF_PARA_NUM) ||
                     (g_atParaList[AT_CBURSTDTMF_CALL_ID].paraLen == 0) ||
                     (g_atParaList[AT_CBURSTDTMF_DTMF_KEY].paraLen == 0) ||
                     (g_atParaList[AT_CBURSTDTMF_ON_LENGTH].paraLen == 0) ||
                     (g_atParaList[AT_CBURSTDTMF_OFF_LENGTH].paraLen == 0);

    if (paraInvalidFlg == VOS_TRUE) {
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* DTMF Key?????????????? */
    if (g_atParaList[AT_CBURSTDTMF_DTMF_KEY].paraLen > TAF_CALL_MAX_BURST_DTMF_NUM) {
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* DTMF Key?????????? */
    for (loop = 0; loop < g_atParaList[AT_CBURSTDTMF_DTMF_KEY].paraLen; loop++) {
        if (((g_atParaList[AT_CBURSTDTMF_DTMF_KEY].para[loop] >= '0') &&
            (g_atParaList[AT_CBURSTDTMF_DTMF_KEY].para[loop] <= '9')) ||
            (g_atParaList[AT_CBURSTDTMF_DTMF_KEY].para[loop] == '*') ||
            (g_atParaList[AT_CBURSTDTMF_DTMF_KEY].para[loop] == '#')) {
            continue;
        } else {
            return AT_CME_INCORRECT_PARAMETERS;
        }
    }

    burstDTMFPara.callId   = (VOS_UINT8)g_atParaList[AT_CBURSTDTMF_CALL_ID].paraValue;
    burstDTMFPara.digitNum = (VOS_UINT8)g_atParaList[AT_CBURSTDTMF_DTMF_KEY].paraLen;

    memResult = memcpy_s(burstDTMFPara.digit, sizeof(burstDTMFPara.digit), g_atParaList[AT_CBURSTDTMF_DTMF_KEY].para,
                         burstDTMFPara.digitNum);
    TAF_MEM_CHK_RTN_VAL(memResult, sizeof(burstDTMFPara.digit), burstDTMFPara.digitNum);

    burstDTMFPara.onLength  = g_atParaList[AT_CBURSTDTMF_ON_LENGTH].paraValue;
    burstDTMFPara.offLength = g_atParaList[AT_CBURSTDTMF_OFF_LENGTH].paraValue;

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* ????ID_TAF_CCM_SEND_BURST_DTMF_REQ???? */
    rst = TAF_CCM_CallCommonReq(&ctrl, &burstDTMFPara, ID_TAF_CCM_SEND_BURST_DTMF_REQ, sizeof(burstDTMFPara), modemId);

    if (rst == VOS_OK) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_CBURSTDTMF_SET;

        /* ???????????????????? */
        return AT_WAIT_ASYNC_RETURN;
    }

    return AT_ERROR;
}

VOS_UINT32 AT_RcvTafCcmSndBurstDTMFCnf(struct MsgCB *msg)
{
    VOS_UINT8                 indexNum     = 0;
    TAF_CCM_SendBurstDtmfCnf *burstDtmfCnf = VOS_NULL_PTR;

    burstDtmfCnf = (TAF_CCM_SendBurstDtmfCnf *)msg;

    /* ????ClientID???????????? */
    if (At_ClientIdToUserId(burstDtmfCnf->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallSndBurstDTMFCnf: Get Index Fail!");
        return VOS_ERR;
    }

    /* AT??????????^CBURSTDTMF?????????????????????????? */
    if (g_atClientTab[indexNum].cmdCurrentOpt != AT_CMD_CBURSTDTMF_SET) {
        AT_WARN_LOG("AT_RcvTafCallSndBurstDTMFCnf: Error Option!");
        return VOS_ERR;
    }

    /* ????AT_STOP_TIMER_CMD_READY????AT??????????????READY???? */
    AT_STOP_TIMER_CMD_READY(indexNum);

    /* ?????????????????????????????????? */
    if (burstDtmfCnf->burstDtmfCnfPara.result != TAF_CALL_SEND_BURST_DTMF_CNF_RESULT_SUCCESS) {
        At_FormatResultData(indexNum, AT_ERROR);
    } else {
        At_FormatResultData(indexNum, AT_OK);
    }

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmSndBurstDTMFRslt(struct MsgCB *msg)
{
    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmCalledNumInfoInd(struct MsgCB *msg)
{
    TAF_CCM_CalledNumInfoInd *calledNum = VOS_NULL_PTR;
    VOS_UINT8                 indexNum  = 0;
    VOS_UINT8                 digit[TAF_CALL_MAX_CALLED_NUMBER_CHARI_OCTET_NUM + 1];
    errno_t                   memResult;

    calledNum = (TAF_CCM_CalledNumInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(calledNum->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallCalledNumInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    calledNum->calledNumInfoPara.digitNum = AT_MIN(calledNum->calledNumInfoPara.digitNum,
                                                   TAF_CALL_MAX_CALLED_NUMBER_CHARI_OCTET_NUM);
    /* ?????? */
    memset_s(digit, sizeof(digit), 0x00, sizeof(digit));
    if (calledNum->calledNumInfoPara.digitNum > 0) {
        memResult = memcpy_s(digit, sizeof(digit), calledNum->calledNumInfoPara.digit,
                             calledNum->calledNumInfoPara.digitNum);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), calledNum->calledNumInfoPara.digitNum);
    }

    /* ??pstCalledNum->aucDigit????????????'\0',??????pstCalledNum->aucDigit??????????????AT?????? */
    digit[calledNum->calledNumInfoPara.digitNum] = '\0';

    /* ???????????? */
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s%s%d,%d,%s%s", g_atCrLf,
                                                     g_atStringTab[AT_STRING_CCALLEDNUM].text,
                                                     calledNum->calledNumInfoPara.numType,
                                                     calledNum->calledNumInfoPara.numPlan, digit, g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmCallingNumInfoInd(struct MsgCB *msg)
{
    TAF_CCM_CallingNumInfoInd *callingNum = VOS_NULL_PTR;
    VOS_UINT8                  indexNum   = 0;
    VOS_UINT8                  digit[TAF_CALL_MAX_CALLING_NUMBER_CHARI_OCTET_NUM + 1];
    errno_t                    memResult;

    callingNum = (TAF_CCM_CallingNumInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(callingNum->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallCallingNumInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    callingNum->callIngNumInfoPara.digitNum = AT_MIN(callingNum->callIngNumInfoPara.digitNum,
                                                     TAF_CALL_MAX_CALLING_NUMBER_CHARI_OCTET_NUM);
    /* ?????? */
    memset_s(digit, sizeof(digit), 0x00, sizeof(digit));

    if (callingNum->callIngNumInfoPara.digitNum > 0) {
        memResult = memcpy_s(digit, sizeof(digit), callingNum->callIngNumInfoPara.digit,
                             callingNum->callIngNumInfoPara.digitNum);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), callingNum->callIngNumInfoPara.digitNum);
    }

    /* ??pstCallingNum->aucDigit????????????'\0',??????pstCallingNum->aucDigit??????????????AT?????? */
    digit[callingNum->callIngNumInfoPara.digitNum] = '\0';

    /* ???????????? */
    g_atSendDataBuff.bufLen =
        (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress, (VOS_CHAR *)g_atSndCodeAddress,
                               "%s%s%d,%d,%d,%d,%s%s", g_atCrLf, g_atStringTab[AT_STRING_CCALLINGNUM].text,
                               callingNum->callIngNumInfoPara.numType, callingNum->callIngNumInfoPara.numPlan,
                               callingNum->callIngNumInfoPara.pi, callingNum->callIngNumInfoPara.si, digit, g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmDispInfoInd(struct MsgCB *msg)
{
    TAF_CCM_DisplayInfoInd *displayInfo = VOS_NULL_PTR;
    VOS_UINT8               indexNum    = 0;
    VOS_UINT8               digit[TAF_CALL_MAX_DISPALY_CHARI_OCTET_NUM + 1];
    errno_t                 memResult;

    displayInfo = (TAF_CCM_DisplayInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(displayInfo->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCcmDispInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    displayInfo->disPlayInfoIndPara.digitNum = AT_MIN(displayInfo->disPlayInfoIndPara.digitNum,
                                                      TAF_CALL_MAX_DISPALY_CHARI_OCTET_NUM);
    /* ?????? */
    memset_s(digit, sizeof(digit), 0x00, sizeof(digit));
    if (displayInfo->disPlayInfoIndPara.digitNum > 0) {
        memResult = memcpy_s(digit, sizeof(digit), displayInfo->disPlayInfoIndPara.digit,
                             displayInfo->disPlayInfoIndPara.digitNum);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), displayInfo->disPlayInfoIndPara.digitNum);
    }

    /* ??pstDisplayInfo->aucDigit????????????'\0',??????pstDisplayInfo->aucDigit??????????????AT?????? */
    digit[displayInfo->disPlayInfoIndPara.digitNum] = '\0';

    /* ???????????? */
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s%s%s,,,%s", g_atCrLf,
                                                     g_atStringTab[AT_STRING_CDISP].text, digit, g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmExtDispInfoInd(struct MsgCB *msg)
{
    TAF_CCM_ExtDisplayInfoInd *extDispInfo = VOS_NULL_PTR;
    VOS_UINT8                  digit[TAF_CALL_MAX_EXTENDED_DISPALY_CHARI_OCTET_NUM + 1];
    VOS_UINT32                 loop;
    VOS_UINT32                 digitNum;
    errno_t                    memResult;
    VOS_UINT8                  indexNum = 0;

    extDispInfo = (TAF_CCM_ExtDisplayInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(extDispInfo->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallExtDispInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    /* ?????? */
    extDispInfo->disPlayInfoIndPara.infoRecsDataNum = AT_MIN(extDispInfo->disPlayInfoIndPara.infoRecsDataNum,
        TAF_CALL_MAX_EXT_DISPLAY_DATA_NUM);
    for (loop = 0; loop < extDispInfo->disPlayInfoIndPara.infoRecsDataNum; loop++) {
        /*
         * ??pstExtDispInfo->aucInfoRecsData[ulLoop].aucDigit????????????'\0',
         * ??????pstExtDispInfo->aucInfoRecsData[ulLoop].aucDigit??????????????AT??????
         */
        digitNum = AT_MIN(extDispInfo->disPlayInfoIndPara.infoRecsData[loop].digitNum,
                          TAF_CALL_MAX_EXTENDED_DISPALY_CHARI_OCTET_NUM + 1);
        memset_s(digit, sizeof(digit), 0x00, sizeof(digit));
        if (digitNum > 0) {
            memResult = memcpy_s(digit, sizeof(digit), extDispInfo->disPlayInfoIndPara.infoRecsData[loop].digit,
                                 digitNum);
            TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), digitNum);
        }
        digit[digitNum] = '\0';

        /* ???????????? */
        g_atSendDataBuff.bufLen =
            (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress, (VOS_CHAR *)g_atSndCodeAddress,
                                   "%s%s%s,%d,%d,%d%s", g_atCrLf, g_atStringTab[AT_STRING_CDISP].text, digit,
                                   extDispInfo->disPlayInfoIndPara.extDispInd,
                                   extDispInfo->disPlayInfoIndPara.displayType,
                                   extDispInfo->disPlayInfoIndPara.infoRecsData[loop].dispalyTag, g_atCrLf);

        At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);
    }

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmConnNumInfoInd(struct MsgCB *msg)
{
    TAF_CCM_ConnNumInfoInd *connNumInfo = VOS_NULL_PTR;
    VOS_UINT8               digit[TAF_CALL_MAX_CONNECTED_NUMBER_CHARI_OCTET_NUM + 1];
    errno_t                 memResult;
    VOS_UINT8               indexNum = 0;

    connNumInfo = (TAF_CCM_ConnNumInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(connNumInfo->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCcmConnNumInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    connNumInfo->connNumInfoIndPara.digitNum = AT_MIN(connNumInfo->connNumInfoIndPara.digitNum,
                                                      TAF_CALL_MAX_CONNECTED_NUMBER_CHARI_OCTET_NUM);
    /* ?????? */
    memset_s(digit, sizeof(digit), 0x00, sizeof(digit));
    if (connNumInfo->connNumInfoIndPara.digitNum > 0) {
        memResult = memcpy_s(digit, sizeof(digit), connNumInfo->connNumInfoIndPara.digit,
                             connNumInfo->connNumInfoIndPara.digitNum);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), connNumInfo->connNumInfoIndPara.digitNum);
    }

    /* ??pstConnNumInfo->aucDigit????????????'\0',??????pstConnNumInfo->aucDigit??????????????AT?????? */
    digit[connNumInfo->connNumInfoIndPara.digitNum] = '\0';

    /* ???????????? */
    g_atSendDataBuff.bufLen =
        (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress, (VOS_CHAR *)g_atSndCodeAddress,
                               "%s%s%d,%d,%d,%d,%s%s", g_atCrLf, g_atStringTab[AT_STRING_CCONNNUM].text,
                               connNumInfo->connNumInfoIndPara.numType, connNumInfo->connNumInfoIndPara.numPlan,
                               connNumInfo->connNumInfoIndPara.pi, connNumInfo->connNumInfoIndPara.si, digit, g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmRedirNumInfoInd(struct MsgCB *msg)
{
    TAF_CCM_RedirNumInfoInd *redirNumInfo = VOS_NULL_PTR;
    VOS_UINT8                digit[TAF_CALL_MAX_REDIRECTING_NUMBER_CHARI_OCTET_NUM + 1];
    errno_t                  memResult;
    VOS_UINT16               length;
    VOS_UINT8                indexNum = 0;

    length       = 0;
    redirNumInfo = (TAF_CCM_RedirNumInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(redirNumInfo->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCcmRedirNumInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    redirNumInfo->redirNumInfoIndPara.digitNum = AT_MIN(redirNumInfo->redirNumInfoIndPara.digitNum,
                                                        TAF_CALL_MAX_REDIRECTING_NUMBER_CHARI_OCTET_NUM);
    /* ?????? */
    memset_s(digit, sizeof(digit), 0x00, sizeof(digit));
    if (redirNumInfo->redirNumInfoIndPara.digitNum > 0) {
        memResult = memcpy_s(digit, sizeof(digit), redirNumInfo->redirNumInfoIndPara.digitNumArray,
                             redirNumInfo->redirNumInfoIndPara.digitNum);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), redirNumInfo->redirNumInfoIndPara.digitNum);
    }

    /* ??pstRedirNumInfo->aucDigitNum????????????'\0',??????pstRedirNumInfo->aucDigitNum??????????????AT?????? */
    digit[redirNumInfo->redirNumInfoIndPara.digitNum] = '\0';

    /* ??????????????????EXTENSIONBIT1??EXTENSIONBIT2?????????? */
    length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                     (VOS_CHAR *)g_atSndCodeAddress + length, "%s%s%d,%d,%s", g_atCrLf,
                                     g_atStringTab[AT_STRING_CREDIRNUM].text, redirNumInfo->redirNumInfoIndPara.numType,
                                     redirNumInfo->redirNumInfoIndPara.numPlan, digit);

    if (redirNumInfo->redirNumInfoIndPara.opPi == VOS_TRUE) {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",%d",
                                         redirNumInfo->redirNumInfoIndPara.pi);
    } else {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",");
    }

    if (redirNumInfo->redirNumInfoIndPara.opSi == VOS_TRUE) {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",%d",
                                         redirNumInfo->redirNumInfoIndPara.si);
    } else {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",");
    }

    if (redirNumInfo->redirNumInfoIndPara.opRedirReason == VOS_TRUE) {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",%d",
                                         redirNumInfo->redirNumInfoIndPara.redirReason);
    } else {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",");
    }

    length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                     (VOS_CHAR *)g_atSndCodeAddress + length, "%s", g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, length);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmSignalInfoInd(struct MsgCB *msg)
{
    TAF_CCM_SignalInfoInd *pstsignalInfo = VOS_NULL_PTR;
    VOS_UINT8              indexNum      = 0;

    pstsignalInfo = (TAF_CCM_SignalInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(pstsignalInfo->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCcmSignalInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    /* ???????????? */
    g_atSendDataBuff.bufLen =
        (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress, (VOS_CHAR *)g_atSndCodeAddress,
                               "%s%s%d,%d,%d%s", g_atCrLf, g_atStringTab[AT_STRING_CSIGTONE].text,
                               pstsignalInfo->signalInfoIndPara.signalType, pstsignalInfo->signalInfoIndPara.alertPitch,
                               pstsignalInfo->signalInfoIndPara.signal, g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmLineCtrlInfoInd(struct MsgCB *msg)
{
    TAF_CCM_LineCtrlInfoInd *lineCtrlInfo = VOS_NULL_PTR;
    VOS_UINT16               length;
    VOS_UINT8                indexNum = 0;

    length       = 0;
    lineCtrlInfo = (TAF_CCM_LineCtrlInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(lineCtrlInfo->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCcmLineCtrlInfoInd: Get Index Fail!");
        return VOS_ERR;
    }

    /* ???????????? */
    length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                     (VOS_CHAR *)g_atSndCodeAddress + length, "%s%s%d", g_atCrLf,
                                     g_atStringTab[AT_STRING_CLCTR].text,
                                     lineCtrlInfo->lineCtrlInfoIndPara.polarityIncluded);

    if (lineCtrlInfo->lineCtrlInfoIndPara.toggleModePresent == VOS_TRUE) {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",%d",
                                         lineCtrlInfo->lineCtrlInfoIndPara.toggleMode);
    } else {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",");
    }

    if (lineCtrlInfo->lineCtrlInfoIndPara.reversePolarityPresent == VOS_TRUE) {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",%d",
                                         lineCtrlInfo->lineCtrlInfoIndPara.reversePolarity);
    } else {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, ",");
    }

    length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                     (VOS_CHAR *)g_atSndCodeAddress + length, ",%d%s",
                                     lineCtrlInfo->lineCtrlInfoIndPara.powerDenialTime, g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, length);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmCCWACInd(struct MsgCB *msg)
{
    TAF_CCM_CcwacInfoInd *ccwac = VOS_NULL_PTR;
    VOS_UINT8             digit[TAF_CALL_MAX_CALLING_NUMBER_CHARI_OCTET_NUM + 1];
    errno_t               memResult;
    VOS_UINT16            length;
    VOS_UINT8             indexNum = 0;

    length = 0;
    ccwac  = (TAF_CCM_CcwacInfoInd *)msg;

    /* ????clientId???????????? */
    if (At_ClientIdToUserId(ccwac->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallCCWACInd: Get Index Fail!");
        return VOS_ERR;
    }

    ccwac->ccwacInfoPara.digitNum = AT_MIN(ccwac->ccwacInfoPara.digitNum,
                                           TAF_CALL_MAX_CALLING_NUMBER_CHARI_OCTET_NUM + 1);
    /* ?????? */
    memset_s(digit, sizeof(digit), 0x00, sizeof(digit));
    if (ccwac->ccwacInfoPara.digitNum > 0) {
        memResult = memcpy_s(digit, sizeof(digit), ccwac->ccwacInfoPara.digit, ccwac->ccwacInfoPara.digitNum);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), ccwac->ccwacInfoPara.digitNum);
    }

    /* ??pstCCWAC->aucDigit????????????'\0',??????pstCCWAC->aucDigit??????????????AT?????? */
    if (ccwac->ccwacInfoPara.digitNum < (TAF_CALL_MAX_CALLING_NUMBER_CHARI_OCTET_NUM + 1)) {
        digit[ccwac->ccwacInfoPara.digitNum] = '\0';
    }

    /* ???????????? */
    if (ccwac->ccwacInfoPara.signalIsPresent == VOS_TRUE) {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, "%s%s%s,%d,%d,%d,%d,%d,%d,%d,%d%s",
                                         g_atCrLf, g_atStringTab[AT_STRING_CCWAC].text, digit, ccwac->ccwacInfoPara.pi,
                                         ccwac->ccwacInfoPara.si, ccwac->ccwacInfoPara.numType,
                                         ccwac->ccwacInfoPara.numPlan, ccwac->ccwacInfoPara.signalIsPresent,
                                         ccwac->ccwacInfoPara.signalType, ccwac->ccwacInfoPara.alertPitch,
                                         ccwac->ccwacInfoPara.signal, g_atCrLf);

    } else {
        length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                         (VOS_CHAR *)g_atSndCodeAddress + length, "%s%s%s,%d,%d,%d,%d,%d,,,%s",
                                         g_atCrLf, g_atStringTab[AT_STRING_CCWAC].text, digit,
                                         ccwac->ccwacInfoPara.pi, ccwac->ccwacInfoPara.si,
                                         ccwac->ccwacInfoPara.numType, ccwac->ccwacInfoPara.numPlan,
                                         ccwac->ccwacInfoPara.signalIsPresent, g_atCrLf);
    }

    At_SendResultData(indexNum, g_atSndCodeAddress, length);

    return VOS_OK;
}


VOS_UINT32 At_TestCContinuousDTMFPara(VOS_UINT8 indexNum)
{
    VOS_UINT16 length;

    length = 0;

    length += (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                     (VOS_CHAR *)g_atSndCodeAddress + length, "^CCONTDTMF: (1-7),(0,1),(0-9,*,#)");
    g_atSendDataBuff.bufLen = length;

    return AT_OK;
}


VOS_UINT32 AT_SetCContinuousDTMFPara(VOS_UINT8 indexNum)
{
    VOS_UINT32            rst;
    TAF_Ctrl              ctrl;
    TAF_CALL_ContDtmfPara contDTMFPara;
    ModemIdUint16         modemId;

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    memset_s(&contDTMFPara, sizeof(contDTMFPara), 0x00, sizeof(contDTMFPara));

    /* Check the validity of parameter */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_PARA_CMD) {
        AT_WARN_LOG("AT_SetCContinuousDTMFPara: Non set command!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /*  Check the validity of <Call_ID> and <Switch> */
    if ((g_atParaList[0].paraLen == 0) || (g_atParaList[1].paraLen == 0)) {
        AT_WARN_LOG("AT_SetCContinuousDTMFPara: Invalid <Call_ID> or <Switch>!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /*
     * If the <Switch> is Start and the number of parameter isn't equal to 3.
     * Or if the <Switch> is Stop and the number of parameter isn't equal to 2??both invalid
     */
    if (((g_atParaList[1].paraValue == TAF_CALL_CONT_DTMF_STOP) && (g_atParaIndex != AT_CCONTDTMF_PARA_NUM_MIN)) ||
        ((g_atParaList[1].paraValue == TAF_CALL_CONT_DTMF_START) && (g_atParaIndex != AT_CCONTDTMF_PARA_NUM_MAX))) {
        AT_WARN_LOG("AT_SetCContinuousDTMFPara: The number of parameters mismatch!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* If the <Switch> is Start,the <Dtmf_Key> should be setted and check its validity */
    if (g_atParaList[1].paraValue == TAF_CALL_CONT_DTMF_START) {
        if (AT_CheckCContDtmfKeyPara() == VOS_ERR) {
            AT_WARN_LOG("AT_SetCContinuousDTMFPara: Invalid <Dtmf_Key>!");
            return AT_CME_INCORRECT_PARAMETERS;
        }
    }

    contDTMFPara.callId     = (VOS_UINT8)g_atParaList[0].paraValue;
    contDTMFPara.dtmfSwitch = (VOS_UINT8)g_atParaList[1].paraValue;
    contDTMFPara.digit      = (VOS_UINT8)g_atParaList[AT_CCONTDTMF_DTMF_KEY].para[AT_CCONTDTMF_DTMF_KEY_SWITCH_STOP];

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* ????ID_TAF_CCM_SEND_CONT_DTMF_REQ???? */
    rst = TAF_CCM_CallCommonReq(&ctrl, &contDTMFPara, ID_TAF_CCM_SEND_CONT_DTMF_REQ, sizeof(contDTMFPara), modemId);

    if (rst == VOS_OK) {
        g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_CCONTDTMF_SET;

        /* Return hang-up state */
        return AT_WAIT_ASYNC_RETURN;
    }

    return AT_ERROR;
}


VOS_UINT32 AT_CheckCContDtmfKeyPara(VOS_VOID)
{
    if (g_atParaList[AT_CCONTDTMF_DTMF_KEY].paraLen != AT_CCONTDTMF_DTMF_KEY_VALID_LEN) {
        return VOS_ERR;
    }

    if (((g_atParaList[AT_CCONTDTMF_DTMF_KEY].para[AT_CCONTDTMF_DTMF_KEY_SWITCH_STOP] >= '0') &&
         (g_atParaList[AT_CCONTDTMF_DTMF_KEY].para[AT_CCONTDTMF_DTMF_KEY_SWITCH_STOP] <= '9')) ||
         (g_atParaList[AT_CCONTDTMF_DTMF_KEY].para[AT_CCONTDTMF_DTMF_KEY_SWITCH_STOP] == '*') ||
         (g_atParaList[AT_CCONTDTMF_DTMF_KEY].para[AT_CCONTDTMF_DTMF_KEY_SWITCH_STOP] == '#')) {
        return VOS_OK;
    } else {
        return VOS_ERR;
    }
}


VOS_UINT32 AT_RcvTafCcmSndContinuousDTMFCnf(struct MsgCB *msg)
{
    VOS_UINT8                indexNum    = 0;
    TAF_CCM_SendContDtmfCnf *contDtmfCnf = VOS_NULL_PTR;

    contDtmfCnf = (TAF_CCM_SendContDtmfCnf *)msg;

    /* According to ClientID to get the index */
    if (At_ClientIdToUserId(contDtmfCnf->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallSndContinuousDTMFCnf: Get Index Fail!");
        return VOS_ERR;
    }

    /* AT module is waiting for report the result of ^CCONTDTMF command */
    if (g_atClientTab[indexNum].cmdCurrentOpt != AT_CMD_CCONTDTMF_SET) {
        AT_WARN_LOG("AT_RcvTafCallSndContinuousDTMFCnf: Error Option!");
        return VOS_ERR;
    }

    /* Use AT_STOP_TIMER_CMD_READY to recover the AT command state to READY state */
    AT_STOP_TIMER_CMD_READY(indexNum);

    /* According to the error code of temporary respond, printf the result of command */
    if (contDtmfCnf->contDtmfCnfPara.result != TAF_CALL_SEND_CONT_DTMF_CNF_RESULT_SUCCESS) {
        At_FormatResultData(indexNum, AT_ERROR);
    } else {
        At_FormatResultData(indexNum, AT_OK);
    }

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmSndContinuousDTMFRslt(struct MsgCB *msg)
{
    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmRcvContinuousDtmfInd(struct MsgCB *msg)
{
    TAF_CCM_ContDtmfInd *rcvContDtmf = VOS_NULL_PTR;
    VOS_UINT8            digit[AT_DIGIT_MAX_LENGTH];
    VOS_UINT8            indexNum = 0;

    rcvContDtmf = (TAF_CCM_ContDtmfInd *)msg;

    /* According to ClientID to get the index */
    if (At_ClientIdToUserId(rcvContDtmf->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallRcvContinuousDtmfInd: Get Index Fail!");
        return VOS_ERR;
    }

    /*
     * Initialize aucDigit[0] with pstRcvContDtmf->ucDigit and  aucDigit[1] = '\0'
     * Because At_sprintf does not allow to print pstRcvContDtmf->ucDigit with %c
     * Hence, need to convert digit into string and print as string
     */
    digit[0] = rcvContDtmf->contDtmfIndPara.digit;
    digit[1] = '\0';

    /* Output the inquire result */
    if (rcvContDtmf->contDtmfIndPara.dtmfSwitch == TAF_CALL_CONT_DTMF_START) {
        g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                         (VOS_CHAR *)g_atSndCodeAddress, "%s^CCONTDTMF: %d,%d,\"%s\"%s",
                                                         g_atCrLf, rcvContDtmf->ctrl.callId,
                                                         rcvContDtmf->contDtmfIndPara.dtmfSwitch, digit, g_atCrLf);
    } else {
        g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                         (VOS_CHAR *)g_atSndCodeAddress, "%s^CCONTDTMF: %d,%d%s",
                                                         g_atCrLf, rcvContDtmf->ctrl.callId,
                                                         rcvContDtmf->contDtmfIndPara.dtmfSwitch, g_atCrLf);
    }

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}


VOS_UINT32 AT_RcvTafCcmRcvBurstDtmfInd(struct MsgCB *msg)
{
    TAF_CCM_BurstDtmfInd *rcvBurstDtmf = VOS_NULL_PTR;
    VOS_UINT8             digit[TAF_CALL_MAX_BURST_DTMF_NUM + 1];
    errno_t               memResult;
    VOS_UINT8             indexNum = 0;

    rcvBurstDtmf = (TAF_CCM_BurstDtmfInd *)msg;

    /* According to ClientID to get the index */
    if (At_ClientIdToUserId(rcvBurstDtmf->ctrl.clientId, &indexNum) == AT_FAILURE) {
        AT_WARN_LOG("AT_RcvTafCallRcvBurstDtmfInd: Get Index Fail!");
        return VOS_ERR;
    }

    rcvBurstDtmf->burstDtmfIndPara.digitNum = (VOS_UINT8)AT_MIN(rcvBurstDtmf->burstDtmfIndPara.digitNum,
                                                     TAF_CALL_MAX_BURST_DTMF_NUM);
    /* initialization */
    memset_s(digit, sizeof(digit), 0x00, sizeof(digit));
    if (rcvBurstDtmf->burstDtmfIndPara.digitNum > 0) {
        memResult = memcpy_s(digit, sizeof(digit), rcvBurstDtmf->burstDtmfIndPara.digit,
                             rcvBurstDtmf->burstDtmfIndPara.digitNum);
        TAF_MEM_CHK_RTN_VAL(memResult, sizeof(digit), rcvBurstDtmf->burstDtmfIndPara.digitNum);
    }

    /* Add the '\0' to the last byte of pstRcvBurstDtmf->aucDigit */
    digit[rcvBurstDtmf->burstDtmfIndPara.digitNum] = '\0';

    /* Output the inquire result */
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s^CBURSTDTMF: %d,\"%s\",%d,%d%s",
                                                     g_atCrLf, rcvBurstDtmf->ctrl.callId, digit,
                                                     rcvBurstDtmf->burstDtmfIndPara.onLength,
                                                     rcvBurstDtmf->burstDtmfIndPara.offLength, g_atCrLf);

    At_SendResultData(indexNum, g_atSndCodeAddress, g_atSendDataBuff.bufLen);

    return VOS_OK;
}

VOS_UINT32 AT_TestCclprPara(VOS_UINT8 indexNum)
{
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s: (1-7)",
                                                     g_parseContext[indexNum].cmdElement->cmdName);

    return AT_OK;
}


VOS_UINT32 AT_SetCclprPara(VOS_UINT8 indexNum)
{
    VOS_UINT32           result;
    TAF_Ctrl             ctrl;
    TAF_CALL_QryClprPara qryClprPara;
    ModemIdUint16        modemId;

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    memset_s(&qryClprPara, sizeof(qryClprPara), 0x00, sizeof(qryClprPara));

    /* ???????? */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_PARA_CMD) {
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* ???????????? */
    if (g_atParaIndex != 1) {
        return AT_CME_INCORRECT_PARAMETERS;
    }

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    qryClprPara.callId          = (MN_CALL_ID_T)g_atParaList[0].paraValue;
    qryClprPara.qryClprModeType = TAF_CALL_QRY_CLPR_MODE_C;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* ????????????ID_TAF_CCM_QRY_CLPR_REQ??C?? */
    result = TAF_CCM_CallCommonReq(&ctrl, (void *)&qryClprPara, ID_TAF_CCM_QRY_CLPR_REQ, sizeof(qryClprPara), modemId);

    if (result != VOS_OK) {
        AT_WARN_LOG("AT_SetCclprPara: TAF_XCALL_SendCclpr fail.");
        return AT_ERROR;
    }

    /* ????AT???????????????????????????? */
    g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_CCLPR_SET;

    return AT_WAIT_ASYNC_RETURN;
}
#endif

VOS_UINT32 AT_SetRejCallPara(VOS_UINT8 indexNum)
{
    AT_ModemCcCtx *ccCtx = VOS_NULL_PTR;

    TAF_Ctrl          ctrl;
    MN_CALL_SupsParam callSupsPara;
    ModemIdUint16     modemId;

    /* ???????????? */
    if (g_atParseCmd.cmdOptType != AT_CMD_OPT_SET_PARA_CMD) {
        AT_WARN_LOG("AT_SetRejCallPara : Current Option is not AT_CMD_REJCALL!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* ???????? */
    if (g_atParaIndex != AT_REJCALL_PARA_NUM) {
        AT_WARN_LOG("AT_SetRejCallPara : The number of input parameters is error!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    /* ???????? */
    if ((g_atParaList[0].paraLen == 0) || (g_atParaList[1].paraLen == 0)) {
        AT_WARN_LOG("AT_SetRejCallPara : Input parameters is error!");
        return AT_CME_INCORRECT_PARAMETERS;
    }

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));
    memset_s(&callSupsPara, sizeof(callSupsPara), 0x00, sizeof(callSupsPara));

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    callSupsPara.callSupsCmd  = MN_CALL_SUPS_CMD_REL_INCOMING_OR_WAITING;
    callSupsPara.callId       = (VOS_UINT8)g_atParaList[0].paraValue;
    callSupsPara.callRejCause = (VOS_UINT8)g_atParaList[1].paraValue;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    if (TAF_CCM_CallCommonReq(&ctrl, &callSupsPara, ID_TAF_CCM_CALL_SUPS_CMD_REQ, sizeof(callSupsPara), modemId) !=
        VOS_OK) {
        AT_WARN_LOG("AT_SetRejCallPara : Send Msg fail!");
        return AT_ERROR;
    }

    /* ???????????? */
    ccCtx = AT_GetModemCcCtxAddrFromClientId(indexNum);

    if (ccCtx->s0TimeInfo.timerStart == VOS_TRUE) {
        AT_StopRelTimer(ccCtx->s0TimeInfo.timerName, &(ccCtx->s0TimeInfo.s0Timer));
        ccCtx->s0TimeInfo.timerStart = TAF_FALSE;
        ccCtx->s0TimeInfo.timerName  = 0;
    }

    g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_REJCALL_SET;

    return AT_WAIT_ASYNC_RETURN; /* ???????????????????? */
}


VOS_UINT32 AT_TestRejCallPara(VOS_UINT8 indexNum)
{
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s: (0,1)",
                                                     g_parseContext[indexNum].cmdElement->cmdName);

    return AT_OK;
}

#if (FEATURE_IMS == FEATURE_ON)

VOS_UINT32 AT_QryCimsErrPara(VOS_UINT8 indexNum)
{
    TAF_CALL_ErrorInfoText *callErrInfo = VOS_NULL_PTR;

    callErrInfo = AT_GetCallErrInfoText(indexNum);

    /* ???????????? */
    g_atSendDataBuff.bufLen = (VOS_UINT16)At_sprintf(AT_CMD_MAX_LEN, (VOS_CHAR *)g_atSndCodeAddress,
                                                     (VOS_CHAR *)g_atSndCodeAddress, "%s: %d,\"%s\"",
                                                     g_parseContext[indexNum].cmdElement->cmdName,
                                                     AT_GetCsCallErrCause(indexNum), callErrInfo->errInfoText);

    return AT_OK;
}
#endif


VOS_UINT32 AT_QryCsChannelInfoPara(VOS_UINT8 indexNum)
{
    TAF_Ctrl      ctrl;
    VOS_UINT32    rst;
    ModemIdUint16 modemId;

    memset_s(&ctrl, sizeof(ctrl), 0x00, sizeof(ctrl));

    ctrl.moduleId = WUEPS_PID_AT;
    ctrl.clientId = g_atClientTab[indexNum].clientId;
    ctrl.opId     = g_atClientTab[indexNum].opId;

    if (AT_GetModemIdFromClient(g_atClientTab[indexNum].clientId, &modemId) != VOS_OK) {
        return AT_ERROR;
    }

    /* AT??CCM???????????? */
    rst = TAF_CCM_CallCommonReq(&ctrl, VOS_NULL_PTR, ID_TAF_CCM_QRY_CHANNEL_INFO_REQ, 0, modemId);

    if (rst != VOS_OK) {
        AT_WARN_LOG("AT_QryCsChannelInfoPara: Send Msg fail!");
        return AT_ERROR;
    }

    /* ????AT???????????????????????????? */
    g_atClientTab[indexNum].cmdCurrentOpt = AT_CMD_CSCHANNELINFO_QRY;

    return AT_WAIT_ASYNC_RETURN;
}

