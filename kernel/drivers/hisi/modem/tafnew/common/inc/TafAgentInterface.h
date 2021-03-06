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

#ifndef _TAFAGENTAPSINTERFACE_H_
#define _TAFAGENTAPSINTERFACE_H_

#include "vos.h"
#include "taf_type_def.h"
#include "at_mn_interface.h"
#include "taf_ps_api.h"
#include "mn_call_api.h"
#include "taf_app_mma.h"

#pragma pack(4)

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#if (VOS_OS_VER == VOS_WIN32)
#pragma pack()
#else
#pragma pack(0)
#endif


enum TAFAGENT_MsgId {
    /* TAFAGENT->DSM ????CID?????????????? */
    /* _H2ASN_MsgChoice TAFAGENT_PS_GetCidParaReq */
    ID_TAFAGENT_PS_GET_CID_PARA_REQ = 0x1002,

    /* _H2ASN_MsgChoice TAFAGENT_MTA_GetSysmodeReq */
    ID_TAFAGENT_MTA_GET_SYSMODE_REQ = 0x1005,

    /* _H2ASN_MsgChoice TAFAGENT_MTA_GetAntStateReq */
    ID_TAFAGENT_MTA_GET_ANT_STATE_REQ = 0x1006,

    /* _H2ASN_MsgChoice TAFAGENT_MTA_WriteAcoreNvReq */
    ID_TAFAGENT_MTA_WRITE_ACORE_NV_REQ = 0x100C,

    /* _H2ASN_MsgChoice TAFAGENT_MTA_WriteAcoreNvReq */
    ID_TAFAGENT_MTA_PROC_ACORE_NV_REQ            = 0x100D,
    ID_TAFAGENT_MTA_READ_ACORE_ENCRYPTION_NV_REQ = 0x100E,
    /* DSM->TAFAGENT ????CID?????????????? */
    /* _H2ASN_MsgChoice TAFAGENT_PS_GetCidParaCnf */
    ID_TAFAGENT_PS_GET_CID_PARA_CNF = 0x2002,

    /* TAFAGENT->MTA  ???????????????????????? */
    /* _H2ASN_MsgChoice TAFAGENT_MTA_GetSysmodeCnf */
    ID_TAFAGENT_MTA_GET_SYSMODE_CNF = 0x2005,

    /* TAFAGENT->MN  ???????????????????? */
    /* _H2ASN_MsgChoice TAFAGERNT_MN_GetCallInfoCnf */
    ID_TAFAGENT_MN_GET_CALL_INFO_CNF = 0x2006,

    /* MTA->TAFAGENT  ???????????????????? */
    /* _H2ASN_MsgChoice TAFAGENT_MTA_GetAntStateCnf */
    ID_TAFAGENT_MTA_GET_ANT_STATE_CNF = 0x2007,

    /* _H2ASN_MsgChoice TAFAGENT_MTA_WriteAcoreNvCnf */
    ID_TAFAGENT_MTA_WRITE_ACORE_NV_CNF = 0x200D,

    /* _H2ASN_MsgChoice TAFAGENT_MTA_WriteAcoreNvCnf */
    ID_TAFAGENT_MTA_PROC_ACORE_NV_CNF            = 0x200E,
    ID_TAFAGENT_MTA_READ_ACORE_ENCRYPTION_NV_CNF = 0x200F,

    ID_TAFAGENT_MSG_ID_ENUM_BUTT
};
typedef VOS_UINT32 TAFAGENT_MsgIdUint32;


typedef struct {
    VOS_MSG_HEADER              /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId; /* _H2ASN_Skip */

    VOS_UINT8            cid; /* ????????????CID */

    VOS_UINT8            reserved[3];
} TAFAGENT_PS_GetCidParaReq;


typedef struct {
    VOS_MSG_HEADER              /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId; /* _H2ASN_Skip */
} TAFAGENT_MTA_GetSysmodeReq;


typedef struct {
    VOS_MSG_HEADER                /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId;   /* _H2ASN_Skip */

    VOS_UINT32           ret;     /* ???????? */
    TAF_PDP_PrimContext  cidInfo; /* ????????????CID?????? */
} TAFAGENT_PS_GetCidParaCnf;



typedef struct {
    MN_CALL_ID_T       callId;      /* ?????????????????? */
    MN_CALL_TypeUint8  callType;    /* ???????? */
    MN_CALL_StateUint8 callState;   /* ???????? */
    VOS_UINT8          reserved[1]; /* ?????? */
} TAFAGERNT_MN_CallInfo;



typedef struct {
    VOS_MSG_HEADER               /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32  msgId; /* _H2ASN_Skip */

    VOS_UINT8             numOfCalls;                /* ?????????????????? */
    VOS_UINT8             reserved[3];               /* ?????? */
    TAFAGERNT_MN_CallInfo callInfo[MN_CALL_MAX_NUM]; /* ???????????? */
} TAFAGERNT_MN_GetCallInfoCnf;

/*
 * ????????: ????????????????, ????:
 *           (1) RAT Type ????????
 *           (2) Sub Mode ??????????
 */
typedef struct {
    TAF_SysModeUint8    ratType;    /* ????????   */
    TAF_SysSubmodeUint8 sysSubMode; /* ?????????? */
    VOS_UINT8           reserve[2];
} TAF_AGENT_SysMode;



typedef struct {
    VOS_MSG_HEADER              /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId; /* _H2ASN_Skip */
    VOS_UINT32           ret;   /* ???????? */
    /* ?????????????????????????????????? */
    TAF_AGENT_SysMode    sysMode;
} TAFAGENT_MTA_GetSysmodeCnf;


typedef struct {
    VOS_MSG_HEADER              /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId; /* _H2ASN_Skip */
} TAFAGENT_MTA_GetAntStateReq;


typedef struct {
    VOS_MSG_HEADER              /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId; /* _H2ASN_Skip */

    VOS_UINT32           rslt;
    VOS_UINT16           antState; /* ???????? */
    VOS_UINT8            reserved1[2];

} TAFAGENT_MTA_GetAntStateCnf;


typedef struct {
    VOS_MSG_HEADER
    TAFAGENT_MsgIdUint32 msgId; /* _H2ASN_Skip */
    VOS_UINT32           nvItemId;
    VOS_UINT32           nvLength;
    VOS_UINT32           isNeedCheck;
    VOS_UINT32           offset;
    VOS_UINT8            data[4];
} TAFAGENT_MTA_WriteAcoreNvReq;


typedef struct {
    VOS_MSG_HEADER               /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId;  /* _H2ASN_Skip */
    VOS_UINT32           result; /* ???????? */
} TAFAGENT_MTA_WriteAcoreNvCnf;


typedef struct {
    VOS_MSG_HEADER
    TAFAGENT_MsgIdUint32               msgId; /* _H2ASN_Skip */
    TAFAGENT_MTA_ProcAcoreNvTpyeUint32 procACoreNvType;
} TAFAGENT_MTA_ProcAcoreNvReq;


typedef struct {
    VOS_MSG_HEADER               /* _H2ASN_Skip */
    TAFAGENT_MsgIdUint32 msgId;  /* _H2ASN_Skip */
    VOS_UINT32           result; /* ???????? */
} TAFAGENT_MTA_ProcAcoreNvCnf;

typedef struct {
    VOS_MSG_HEADER
    TAFAGENT_MsgIdUint32 msgId;
    VOS_UINT32           nvItemId;
} TAFAGENT_MTA_ReadAcoreEncryptionNvReq;

typedef struct {
    VOS_MSG_HEADER
    TAFAGENT_MsgIdUint32 msgId;
    VOS_UINT32           result; /* ???????? */
    VOS_UINT32           nvLength;
    VOS_UINT8            dataArray[4];
} TAFAGENT_MTA_ReadAcoreEncryptionNvCnf;
/*
 * H2ASN????????????????
 */
typedef struct {
    /* _H2ASN_MsgChoice_Export TAFAGENT_MsgIdUint32 */
    TAFAGENT_MsgIdUint32 msgId;
    VOS_UINT8            msgBlock[4];
    /*
     * _H2ASN_MsgChoice_When_Comment          TAFAGENT_MsgIdUint32
     */
} TAF_AGENT_MSG_DATA;
/* _H2ASN_Length UINT32 */

typedef struct {
    VOS_MSG_HEADER
    TAF_AGENT_MSG_DATA msgData;
} TafAgentInterface_MSG;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif
