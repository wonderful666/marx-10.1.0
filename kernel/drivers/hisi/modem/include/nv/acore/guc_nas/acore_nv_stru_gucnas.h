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

#ifndef __ACORE_NV_STRU_GUCNAS_H__
#define __ACORE_NV_STRU_GUCNAS_H__

#include "vos.h"
#include "acore_nv_id_gucnas.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#pragma pack(push, 4)

#define AT_MDATE_STRING_LENGTH 20
#define TAF_AT_NVIM_CLIENT_CFG_LEN 128
#define AT_DISSD_PWD_LEN 16
#define AT_OPWORD_PWD_LEN 16
#define AT_FACINFO_INFO1_LENGTH 128
#define AT_FACINFO_INFO2_LENGTH 128
#define AT_FACINFO_STRING_LENGTH ((AT_FACINFO_INFO1_LENGTH + 1) + (AT_FACINFO_INFO2_LENGTH + 1))
#define AT_FACINFO_INFO1_STR_LENGTH (AT_FACINFO_INFO1_LENGTH + 1)
#define AT_FACINFO_INFO2_STR_LENGTH (AT_FACINFO_INFO2_LENGTH + 1)
#define AT_NVIM_SETZ_LEN 16
#define AT_NOTSUPPORT_STR_LEN 16
#define AT_NVIM_RIGHT_PWD_LEN 16
#define ADS_UL_QUEUE_SCHEDULER_PRI_MAX 9 /* ???????????????? */
#define AT_WIFI_SSID_LEN_MAX 33          /* WIFI SSID KEY???????? */
#define AT_WIFI_KEY_LEN_MAX 27
#define AT_WIFI_WLAUTHMODE_LEN 16     /* ?????????????????? */
#define AT_WIFI_ENCRYPTIONMODES_LEN 5 /* ?????????????????? */
#define AT_WIFI_WLWPAPSK_LEN 65       /* WPA???????????????? */
#define AT_WIFI_MAX_SSID_NUM 4        /* ????????4??SSID */
#define AT_WIFI_KEY_NUM (AT_WIFI_MAX_SSID_NUM)
#define AT_MAX_ABORT_CMD_STR_LEN 16
#define AT_MAX_ABORT_RSP_STR_LEN 16
#define AT_PRODUCT_NAME_MAX_NUM 29
#define AT_PRODUCT_NAME_LENGHT (AT_PRODUCT_NAME_MAX_NUM + 1)
#define AT_WEBUI_PWD_MAX 16 /* WEB UI ???????????? */
#define AT_AP_XML_RPT_SRV_URL_LEN 127
#define AT_AP_XML_RPT_SRV_URL_STR_LEN (AT_AP_XML_RPT_SRV_URL_LEN + 1)
#define AT_AP_NVIM_XML_RPT_INFO_TYPE_LEN 127
#define AT_AP_NVIM_XML_RPT_INFO_TYPE_STR_LEN (AT_AP_NVIM_XML_RPT_INFO_TYPE_LEN + 1)
#define RNIC_NVIM_NAPI_LB_MAX_LEVEL 16 /* ?????????????????????? */
#define RNIC_NVIM_NAPI_LB_MAX_CPUS 16  /* ??????????????????CPU???? */
#define RNIC_NVIM_MAX_CLUSTER 3        /* CPU????CLUSTER???? */
#define RNIC_NVIM_RHC_MAX_LEVEL 16     /* ???????????????? */

enum AT_UART_LinkType {
    AT_UART_LINK_TYPE_OM = 1, /* OM???? */
    AT_UART_LINK_TYPE_AT = 2, /* AT???? */
    AT_UART_LINK_TYPE_BUTT    /* ?????? */
};
typedef VOS_UINT16 AT_UART_LinkTypeUint16;


enum NAPI_WEIGHT_AdjMode {
    NAPI_WEIGHT_ADJ_STATIC_MODE  = 0x00, /* ???????????? */
    NAPI_WEIGHT_ADJ_DYNAMIC_MODE = 0x01, /* ???????????? */

    NAPI_WEIGHT_ADJ_MODE_BUTT            /* ?????? */
};
typedef VOS_UINT8 NAPI_WEIGHT_AdjModeUint8;

/*
 * ??    ??: NV_ITEM_USB_ENUM_STATUS(21)
 * ????????: ??????USB??????????????????????????at^u2diag=??????????
 */
typedef struct {
    /*
     * 0??????????
     * 1????????
     */
    VOS_UINT32 status;
    VOS_UINT32 value;    /* ?????????? */
    VOS_UINT32 reserve1; /* ???? */
    VOS_UINT32 reserve2; /* ???? */
} USB_ENUM_Status;


typedef struct {
    /*
     * Bit[0]??DRX Debug Flag??
     * Bit[1]??USIMM Debug Flag??
     * Bit[2] - Bit[31]????????
     */
    VOS_UINT32 commDebugFlag;
} TAF_AT_NvimCommdegbugCfg;

/*
 * ??    ??: NV_ITEM_PID_ENABLE_TYPE(2601)
 * ????????: ??????????
 */
typedef struct {
    VOS_UINT32 pidEnabled; /* pid??????????0??????????1?????? */
} NV_PID_EnableType;


typedef struct {
    /*
     * 0????????????
     * 1????????????
     */
    VOS_UINT32 nvSwVerFlag;
} OM_SW_VerFlag;


typedef struct {
    VOS_INT8 ate5DissdPwd[AT_DISSD_PWD_LEN]; /* ???????? */
} TAF_AT_NvimDissdPwd;


typedef struct {
    VOS_INT8 atOpwordPwd[AT_OPWORD_PWD_LEN]; /* ???????????????????????? */
} TAF_AT_NvimDislogPwdNew;


typedef struct {
    /*
     * UART??????????????????
     * 1??OM??
     * 2??AT??
     */
    AT_UART_LinkTypeUint16  uartLinkType;
    VOS_UINT8                     reserve1; /* ???? */
    VOS_UINT8                     reserve2; /* ???? */
} TAF_AT_NvimDefaultLinkOfUart;

/*
 * ??    ??: NV_ITEM_BATTERY_ADC(90)
 * ????????: ????????????????????????????
 */
typedef struct {
    VOS_UINT16 minValue; /* ?????? */
    VOS_UINT16 maxValue; /* ?????? */
} VBAT_CALIBART_Type;


typedef struct {
    /* ????0??????????????????????128??????????????????????????????129???????????????? */
    VOS_UINT8 factInfo1[AT_FACINFO_INFO1_STR_LENGTH];
    /* ????1??????????????????????128??????????????????????????????258???????????????? */
    VOS_UINT8 factInfo2[AT_FACINFO_INFO2_STR_LENGTH];
    VOS_UINT8 reserve1; /* ???? */
    VOS_UINT8 reserve2; /* ???? */
} TAF_AT_NvimFactoryInfo;


typedef struct {
    /*
     * Yyyy-mm-dd hh-mm-ss????????????????????
     * ??????????4??????????????????????????????????2????????2????????????0??
     */
    VOS_UINT8 mDate[AT_MDATE_STRING_LENGTH];
} TAF_AT_NvimManufactureDate;


typedef struct {
    VOS_UINT32 smsRiOnInterval;    /* ????RI??????????????????????????(ms)?? */
    VOS_UINT32 smsRiOffInterval;   /* ????RI??????????????????????????(ms)?? */
    VOS_UINT32 voiceRiOnInterval;  /* ????RI??????????????????????????(ms)?? */
    VOS_UINT32 voiceRiOffInterval; /* ????RI??????????????????????????(ms)?? */
    VOS_UINT8  voiceRiCycleTimes;  /* ????RI?????????????? */
    VOS_UINT8  reserved[3];        /* ?????? */
} TAF_NV_UartRi;


typedef struct {
    /*
     * ????????????????????1-6??
     * 1??8????????2????????
     * 2??8????????1????????1????????
     * 3??8????????1????????
     * 4??7????????2????????
     * 5??7????????1????????1????????
     * 6??7????????1????????
     */
    VOS_UINT8 format;
    /*
     * ??????????????????????0-3??
     * 0????????
     * 1????????
     * 2??????????
     * 3????????
     */
    VOS_UINT8 parity;
    VOS_UINT8 reserved[2]; /* ???? */

} TAF_NV_UartFrame;


typedef struct {
    /*
     * ??????????????????
     * 0, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800. 921600, 2764800, 4000000
     */
    VOS_UINT32             baudRate;
    TAF_NV_UartFrame frame;    /* UART???????? */
    TAF_NV_UartRi    riConfig; /* UART Ring???????? */
} TAF_NV_UartCfg;


typedef struct {
    /*
     * ??????????????????
     * 0????????
     * 1????????
     */
    VOS_UINT8 enableFlg;
    VOS_UINT8 reserved[3]; /* ???? */
} TAF_NV_PortBuffCfg;


typedef struct {
    /*
     * CDMAMODEMSWITCH??????Modem??????????????
     * 0??????????????????????Modem??
     * 1??????????????????????Modem??
     * ??????
     * 1. ??????????????????
     * 2. ????????????????????????????????CdmaModemSwitch????????????????????????CdmaModemSwitch??????
     *    ????????CdmaModemSwitch????????????????AP????????????Modem????????????????????????????????????
     * 3. ??????????????????????????Modem??
     */
    VOS_UINT8 enableFlg;
    VOS_UINT8 reversed[7]; /* ?????????????????? */
} TAF_NVIM_CdmamodemswitchNotResetCfg;


typedef struct {
    VOS_UINT32 ipv6AddrTestModeCfg; /* ????0x55AA55AA?????????????????????????????????? */
} TAF_NVIM_Ipv6AddrTestModeCfg;


typedef struct {
    /*
     * IPF????ADS????????????????
     * 0??????????????
     * 1??????????????
     */
    VOS_UINT8 ipfMode;
    VOS_UINT8 reserved0; /* ?????? */
    VOS_UINT8 reserved1; /* ?????? */
    VOS_UINT8 reserved2; /* ?????? */
} TAF_NV_AdsIpfModeCfg;


typedef struct {
    VOS_UINT8 info[128]; /* ????????????????16bytes????????SN?????????????????? */
} OM_ManufactureInfo;


typedef struct {
    /*
     * ??????????????????
     * 1??????
     * 0??????
     */
    VOS_UINT32 enable;
    VOS_UINT32 txWakeTimeout; /* ?????????????????????????? */
    VOS_UINT32 rxWakeTimeout; /* ?????????????????????????? */
    VOS_UINT32 reserved;      /* ?????????? */
} TAF_NV_AdsWakeLockCfg;


typedef struct {
    /*
     * ????????????netlink socket????modem log??
     * 0????????, default:0??
     * 1????????
     */
    VOS_UINT8 printModemLogType;
    VOS_UINT8 reserved0; /* ???????? */
    VOS_UINT8 reserved1; /* ???????? */
    VOS_UINT8 reserved2; /* ???????? */
} TAF_NV_PrintModemLogType;


typedef struct {
    /*
     * ????????????
     * ????BlkSize??448 bytes????????????????512??BlkSize??1540 bytes??????????????768????
     */
    VOS_UINT16 blkNum;
    VOS_UINT16 reserved0; /* ?????????? */
    VOS_UINT16 reserved1; /* ?????????? */
    VOS_UINT16 reserved2; /* ?????????? */

} TAF_NV_AdsMemCfg;


typedef struct {
    /*
     * ??CAHCE??????????????
     * 1????????
     * 0????????
     */
    VOS_UINT32 enable;
    /*
     * memCfg[0]??BlkSize??448 bytes????????????
     * memCfg[1]??BlkSize??1540 bytes????????????
     */
    TAF_NV_AdsMemCfg memCfg[2];
} TAF_NV_AdsMemPoolCfg;


typedef struct {
    /*
     * ??????????????????
     * 1??????
     * 0??????
     */
    VOS_UINT32 enabled;
    /* ????????????????????[50,100]???????????????? */
    VOS_UINT32 errorRateThreshold;  /* ????????????????????[50,100]???????????????? */
    VOS_UINT32 detectDuration;      /* ??????????????????????????[1000,10000]??????????????*/
    VOS_UINT32 reserved;            /* ???????? */

} TAF_NV_AdsErrorFeedbackCfg;


typedef struct {
    /*
     * ????????????????AT+CFUN=1,1??????????????????
     * 0xEF??????
     * ????????????
     */
    VOS_UINT8 userRebootConfig;
    VOS_UINT8 reserved1; /* ???????? */
    VOS_UINT8 reserved2; /* ???????? */
    VOS_UINT8 reserved3; /* ???????? */
} TAF_NVIM_UserRebootSupport;


typedef struct {
    VOS_UINT32 dlPktNumPerSecLevel1; /* RNIC??????????????????????1 */
    VOS_UINT32 dlPktNumPerSecLevel2; /* RNIC??????????????????????2 */
    VOS_UINT32 dlPktNumPerSecLevel3; /* RNIC??????????????????????3 */
    VOS_UINT32 dlPktNumPerSecLevel4; /* RNIC??????????????????????4 */
    VOS_UINT32 dlPktNumPerSecLevel5; /* RNIC??????????????????????5 */
    VOS_UINT32 reserved0;            /* ?????? */
    VOS_UINT32 reserved1;            /* ?????? */
    VOS_UINT32 reserved2;            /* ?????? */

} RNIC_NAPI_DlPktNumPerSecLevel;


typedef struct {
    VOS_UINT8 napiWeightLevel1;  // RNIC??????????????????????NAPI Weight??????1
    VOS_UINT8 napiWeightLevel2;  // RNIC??????????????????????NAPI Weight??????2
    VOS_UINT8 napiWeightLevel3;  // RNIC??????????????????????NAPI Weight??????3
    VOS_UINT8 napiWeightLevel4;  // RNIC??????????????????????NAPI Weight??????4
    VOS_UINT8 napiWeightLevel5;  // RNIC??????????????????????NAPI Weight??????5
    VOS_UINT8 reserved0;         // ??????
    VOS_UINT8 reserved1;         // ??????
    VOS_UINT8 reserved2;         // ??????

} RNIC_NAPI_WeightLevel;


typedef struct {
    RNIC_NAPI_DlPktNumPerSecLevel dlPktNumPerSecLevel; /* RNIC?????????????????????? */
    RNIC_NAPI_WeightLevel             napiWeightLevel;     /* RNIC??????????????????????NAPI Weight?????? */

} RNIC_NAPI_WeightDynamicAdjCfg;


typedef struct {
    /*
     * RNIC??????????????Linux??????????????????????
     * 0??Net_rx????
     * 1??NAPI????
     */
    VOS_UINT8 napiEnable;
    /*
     * NAPI Weight??????????
     * 0??????????
     * 1??????????????
     */
    NAPI_WEIGHT_AdjModeUint8  napiWeightAdjMode;
    /* RNIC????NAPI????????poll???????????? */
    VOS_UINT8                       napiPollWeight;
    /*
     * GRO??????
     * 0????????
     * 1??????
     */
    VOS_UINT8                             groEnable;
    VOS_UINT16                            napiPollQueMaxLen;       /* RNIC Poll?????????????????? */
    VOS_UINT8                             reserved1;               /* ?????? */
    VOS_UINT8                             reserved2;               /* ?????? */
    RNIC_NAPI_WeightDynamicAdjCfg napiWeightDynamicAdjCfg; /* Napi Weight???????????? */

} TAF_NV_RnicNapiCfg;


typedef struct {
    VOS_UINT32 pps;                                      /* RNIC?????????????????????????? */
    VOS_UINT8  cpusWeight[RNIC_NVIM_NAPI_LB_MAX_CPUS];  /* ????NAPI??????????CPU Weight?????? */

} RNIC_NAPI_LbLevelCfg;


typedef struct {
    VOS_UINT8                   napiLbEnable;                                 /* NAPI???????????????? */
    VOS_UINT8                   napiLbValidLevel;                             /* Napi???????????????? */
    VOS_UINT16                  napiLbCpumask;                                /* ????Napi??????????CPU???? */
    VOS_UINT8                   reserved0;                                    /* ?????? */
    VOS_UINT8                   reserved1;                                    /* ?????? */
    VOS_UINT8                   reserved2;                                    /* ?????? */
    VOS_UINT8                   reserved3;                                    /* ?????? */
    VOS_UINT32                  reserved4;                                    /* ?????? */
    VOS_UINT32                  reserved5;                                    /* ?????? */
    RNIC_NAPI_LbLevelCfg napiLbLevelCfg[RNIC_NVIM_NAPI_LB_MAX_LEVEL]; /* Napi???????????????? */

} TAF_NV_RnicNapiLbCfg;


typedef struct {
    VOS_UINT32 totalPps;                       /* RNIC?????????????????????????? */
    VOS_UINT32 nonTcpPps;                      /* RNIC????????????????tcp???????????? */
    VOS_UINT32 backlogQueLimitLen;             /* CPU backlog que???? */
    VOS_UINT8  congestionCtrl;                 /* ???????????? */
    VOS_UINT8  rpsBitMask;                     /* RPS CPU???? */
    VOS_UINT8  reserved1;                      /* ?????? */
    VOS_UINT8  isolationDisable;               /* ????isolation???? */
    VOS_UINT32 freqReq[RNIC_NVIM_MAX_CLUSTER]; /* ????????CLUSTER?????? */

} RNIC_RhcLevel;


typedef struct {
    VOS_UINT8           rhcFeature;                        /* ???????????????????? */
    VOS_UINT8           reserved1;                         /* ?????? */
    VOS_UINT8           reserved2;                         /* ?????? */
    VOS_UINT8           reserved3;                         /* ?????? */
    VOS_UINT32          reserved4;                         /* ?????? */
    RNIC_RhcLevel levelCfg[RNIC_NVIM_RHC_MAX_LEVEL]; /* ???????????????????? */

} TAF_NV_RnicRhcCfg;


typedef struct {
    /*
     * ??????????????BIT0-BIT1??????client??????????????Modem Id??
     * 00:????Modem0
     * 01:????Modem1
     * 10:????Modem2??
     * BIT2????????client????????????:
     * 0:??????????
     * 1:??????????
     * ????????96??????
     */
    VOS_UINT8 atClientConfig[TAF_AT_NVIM_CLIENT_CFG_LEN];
} TAF_AT_NvimAtClientCfg;


typedef struct {
    VOS_UINT8 gmmInfo[2]; /* Gmm???? */
    VOS_UINT8 reserve1;    /* ???? */
    VOS_UINT8 reserve2;    /* ???? */
} TAF_AT_NvGmmInfo;


typedef struct {
    /*
     * 0????????????????????????
     * 1??????????????????????
     */
    VOS_UINT8 vaild;
    VOS_UINT8 reserved1; /* ?????? */
    VOS_UINT8 reserved2; /* ?????? */
    VOS_UINT8 reserved3; /* ?????? */
} TAF_AT_NvimRxdivConfig;


typedef struct {
    /*
     * 0????????????????????????????????
     * 1????????????????????????????
     */
    VOS_UINT16 reportRegActFlg;
    VOS_UINT8  reserve1; /* ???? */
    VOS_UINT8  reserve2; /* ???? */
} TAF_AT_NvimReportRegActFlg;

/*
 * ??    ??: NV_ITEM_NDIS_DHCP_DEF_LEASE_TIME(2635)
 * ????????: ??NV????????
 */
typedef struct {
    VOS_UINT32 dhcpLeaseHour; /* Range:[0x1,0x2250] */
} NDIS_NV_DhcpLeaseHour;


typedef struct {
    /*
     * ??NV????????????
     * 0??????????
     * 1????????
     */
    VOS_UINT8 status;
    /*
     * CREG/CGREG??<CI>????????????
     * 0??<CI>????2??????????????
     * 1??<CI>????4??????????????
     */
    VOS_UINT8 ciBytesRpt;
    VOS_UINT8 reserve1;                  /* ???? */
    VOS_UINT8 reserve2;                  /* ???? */
} NAS_NV_CregCgregCiFourByteRpt; /* VDF????: CREG/CGREG????<CI>????????4??????????NV???????????? */


typedef struct {
    VOS_INT8 tz[AT_NVIM_SETZ_LEN]; /* ???????? */
} TAF_AT_Tz;


typedef struct {
    VOS_INT8 errorText[AT_NOTSUPPORT_STR_LEN]; /* ???????? */
} TAF_AT_NotSupportCmdErrorText;


typedef struct {
    /*
     * 0??PCUI????????
     * 1??PCUI????????
     */
    VOS_UINT32 rightOpenFlg;
    VOS_INT8   password[AT_NVIM_RIGHT_PWD_LEN]; /* ????PCUI?????????????????? */
} TAF_AT_NvimRightOpenFlag;


typedef struct {
    /* IPV6 MTU ????, ??????????  */
    VOS_UINT32 ipv6RouterMtu;
} TAF_NDIS_NvIpv6RouterMtu;

/*
 * ??    ??: NV_ITEM_IPV6_CAPABILITY(8514)
 * ????????: ????????????IPV6????????????, ????IPV6??????????????IPV4V6_OVER_ONE_PDP
 */
typedef struct {
    /*
     * ??NV??????????????
     * 0????????
     * 1????????
     */
    VOS_UINT8 status;
    /*
     * 1??IPv4 only??
     * 2??IPv6 only??
     * 4??IPV4V6 support enabled over one IPV4V6 context (fallbacking on 2 single address PDP contexts if necessary)??
     * 8??IPV4V6 support enabled over 2 single address PDP contexts??????????????
     */
    VOS_UINT8 ipv6Capablity;
    VOS_UINT8 reversed[2]; /* ?????? */
} AT_NV_Ipv6Capability;


typedef struct {
    VOS_UINT8 gsmConnectRate;   /* GSM????????????????MB/s?? */
    VOS_UINT8 gprsConnectRate;  /* Gprs????????????????MB/s?? */
    VOS_UINT8 edgeConnectRate;  /* Edge????????????????MB/s?? */
    VOS_UINT8 wcdmaConnectRate; /* Wcdma????????????????MB/s?? */
    VOS_UINT8 dpaConnectRate;   /* DPA????????????????MB/s?? */
    VOS_UINT8 reserve1;         /* ???? */
    VOS_UINT8 reserve2;         /* ???? */
    VOS_UINT8 reserve3;         /* ???? */
} AT_NVIM_DialConnectDisplayRate;


typedef struct {
    /*
     * ????????ADS_Queue_Scheduler_Pri??????
     * 0??????????????????????????,??????????????
     * 1??????????????????
     */
    VOS_UINT32 status;
    VOS_UINT16 priWeightedNum[ADS_UL_QUEUE_SCHEDULER_PRI_MAX]; /* ADS???????????????????????? */
    VOS_UINT8  rsv[2];                                         /* ?????? */
} ADS_UL_QueueSchedulerPriNv;


typedef struct {
    VOS_UINT32 opbSupport : 1;
    VOS_UINT32 opgSupport : 1;
    VOS_UINT32 opnSupport : 1;
    VOS_UINT32 opSpare : 29;
    VOS_UINT8  bHighChannel; /* 802.11b????????????????????????????0~14 */
    VOS_UINT8  bLowChannel;  /* 802.11b????????????????????????????0~14 */
    VOS_UINT8  rsv1[2];     /* ?????????????? */
    /*
     * 802.11b??????????????????????????0~65535??
     * bPower[0]??PA??????????????
     * bPower[1]??NO PA??????????????
     */
    VOS_UINT16 bPower[2];
    VOS_UINT8  gHighChannel; /* 802.11g????????????????????????????0~14 */
    VOS_UINT8  gLowChannel;  /* 802.11g????????????????????????????0~14 */
    VOS_UINT8  rsv2[2];     /* ?????????????? */
    /*
     * 802.11g??????????????????????????0~65535??
     * gPower[0]??PA??????????????
     * gPower[1]??NO PA??????????????
     */
    VOS_UINT16 gPower[2];   /* ?????????????? */
    VOS_UINT8  nHighChannel; /* 802.11n????????????????????????????0~14 */
    VOS_UINT8  nLowChannel;  /* 802.11n????????????????????????????0~14 */
    VOS_UINT8  rsv3[2];     /* ?????????????? */
    /*
     * 802.11n??????????????????????????0~65535??
     * nPower[0]??PA??????????????
     * nPower[1]??NO PA??????????????
     */
    VOS_UINT16 nPower[2];
} AT_WIFI_Info;

/*
 * ??    ??: NV_ITEM_WIFI_INFO(2656)
 * ????????: ????????WIFI??????????????????????????????????
 */
typedef struct {
    /*
     * BIT 0????????????802.11b??????
     * 0??????????
     * 1????????
     * BIT 1????????????802.11g??????
     * 0??????????
     * 1????????
     * BIT 2????????????802.11n??????
     * 0??????????
     * 1????????
     * BIT 4~BIT 31????????????????
     */
    VOS_UINT32 opSupport;
    VOS_UINT8  bHighChannel; /* 802.11b????????????????????????????0~14?? */
    VOS_UINT8  bLowChannel;  /* 802.11b????????????????????????????0~14?? */
    VOS_UINT8  rsv1[2];     /* ???????????????? */
    /*
     * 802.11b??????????????????????????0~65535??
     * bPower[0]??PA??????????????
     * bPower[1]??NO PA??????????????
     */
    VOS_UINT16 bPower[2];
    VOS_UINT8  gHighsChannel; /* 802.11g????????????????????????????0~14?? */
    VOS_UINT8  gLowChannel;   /* 802.11g????????????????????????????0~14?? */
    VOS_UINT8  rsv2[2];      /* ???????????????? */
    /*
     * 802.11g??????????????????????????0~65535??
     * gPower[0]??PA??????????????
     * gPower[1]??NO PA??????????????
     */
    VOS_UINT16 gPower[2];
    VOS_UINT8  nHighsChannel; /* 802.11n????????????????????????????0~14?? */
    VOS_UINT8  nLowChannel;   /* 802.11n????????????????????????????0~14?? */
    VOS_UINT8  rsv3[2];     /* ???????????????? */
    /*
     * 802.11n??????????????????????????0~65535??
     * nPower[0]??PA??????????????
     * nPower[1]??NO PA??????????????
     */
    VOS_UINT16 nPower[2];
} NV_WIFI_Info;


typedef struct {
    VOS_UINT16 platform; /* ????????????????????0~65535?? */
    VOS_UINT8  reserve1; /* ???? */
    VOS_UINT8  reserve2; /* ???? */
} NAS_NVIM_Platform;


typedef struct {
    VOS_UINT16 eqver;    /* ??????????AT??????????????????????0~65535?? */
    VOS_UINT8  reserve1; /* ???? */
    VOS_UINT8  reserve2; /* ???? */
} TAF_AT_EqVer;


typedef struct {
    /*
     * ??????????????????????????0~65535??
     * ????????????????????
     */
    VOS_UINT16 csver;
    VOS_UINT8  reserve1; /* ?????? */
    VOS_UINT8  reserve2; /* ?????? */
} TAF_NVIM_CsVer;


typedef struct {
    /*
     * ????????MUX??????
     * 0??????????HILINK??????????
     * 1????????
     */
    VOS_UINT8 muxSupportFlg;
    VOS_UINT8 reserved1; /* ???????? */
    VOS_UINT8 reserved2; /* ???????? */
    VOS_UINT8 reserved3; /* ???????? */
} TAF_AT_NvimMuxSupportFlg;


typedef struct {
    VOS_UINT8 wifiAuthmode[AT_WIFI_WLAUTHMODE_LEN];                   /* ?????????? */
    VOS_UINT8 wifiBasicencryptionmodes[AT_WIFI_ENCRYPTIONMODES_LEN];  /* ?????????????? */
    VOS_UINT8 wifiWpaencryptionmodes[AT_WIFI_ENCRYPTIONMODES_LEN];    /* WPA?????????? */
    VOS_UINT8 wifiWepKey1[AT_WIFI_MAX_SSID_NUM][AT_WIFI_KEY_LEN_MAX]; /* WIFI KEY1?? */
    VOS_UINT8 wifiWepKey2[AT_WIFI_MAX_SSID_NUM][AT_WIFI_KEY_LEN_MAX]; /* WIFI KEY2?? */
    VOS_UINT8 wifiWepKey3[AT_WIFI_MAX_SSID_NUM][AT_WIFI_KEY_LEN_MAX]; /* WIFI KEY3?? */
    VOS_UINT8 wifiWepKey4[AT_WIFI_MAX_SSID_NUM][AT_WIFI_KEY_LEN_MAX]; /* WIFI KEY4?? */
    VOS_UINT8 wifiWepKeyIndex[AT_WIFI_MAX_SSID_NUM]; /* ??????WIFI KEY INDEX????????1????????wlKeys1 */
    VOS_UINT8 wifiWpapsk[AT_WIFI_MAX_SSID_NUM][AT_WIFI_WLWPAPSK_LEN]; /* WPA???????? */
    VOS_UINT8 wifiWpsenbl; /* WPS??????????0??????????1?????? */
    VOS_UINT8 wifiWpscfg;  /* ??????????????????enrollee????????0: ??????(????); 1:???? */
} TAF_AT_MultiWifiSec;


typedef struct {
    /* ??????????????WIFI SSID????????????????33?? */
    VOS_UINT8 wifiSsid[AT_WIFI_MAX_SSID_NUM][AT_WIFI_SSID_LEN_MAX];
    VOS_UINT8 reserved[84]; /* ???? */
} TAF_AT_MultiWifiSsid;


typedef struct {
    /*
     * ??????????????????
     * 1??????
     * 0??????
     */
    VOS_UINT8 abortEnableFlg;
    VOS_UINT8 reserve1; /* ??????????0?? */
    VOS_UINT8 reserve2; /* ??????????0?? */
    VOS_UINT8 reserve3; /* ??????????0?? */
    /* ??????????????????AT???????????????????????????????????????????????????????????? */
    VOS_UINT8 abortAtCmdStr[AT_MAX_ABORT_CMD_STR_LEN];
    /* ????????????????????AT?????????????????????????????????????????????? */
    VOS_UINT8 abortAtRspStr[AT_MAX_ABORT_RSP_STR_LEN];
} AT_NVIM_AbortCmdPara;


typedef struct {
    VOS_UINT32 muxReportCfg; /* MUX ????????????0????????MUX??????????????????AT?????? */
} TAF_AT_NvimMuxReportCfg;


typedef struct {
    /*
     * ??????????????????????AT????????AT+CIMI????????IMSI??
     * Bit????0????????
     * Bit????1??????????
     */
    VOS_UINT32 cimiPortCfg;
} TAF_AT_NvimCimiPortCfg;


typedef struct {
    /*
     * NV??????????
     * 0??????????
     * 1????????
     */
    VOS_UINT8 status;
    /*
     * SS????????????????
     * BIT0??????+CCWA??????????????????????????????
     * 0??????????
     * 1????????
     * BIT1??????+CLCK??????????????????????????
     * 0??????????
     * 1????????
     * ????BIT????????
     */
    VOS_UINT8 ssCmdCustomize;
    VOS_UINT8 reserved1[2]; /* ???? */
} AT_SS_CustomizePara;


typedef struct {
    /*
     * ????????Share PDP??????
     * 1????????
     * 0????????
     */
    VOS_UINT8 enableFlag;
    VOS_UINT8 reserved; /* ?????? */
    /*
     * ??????????????????(s)??
     * ??Share PDP????????????????????????0????????????????????0??????????????????
     */
    VOS_UINT16 agingTimeLen;
} TAF_NVIM_SharePdpInfo;


typedef struct {
    VOS_UINT32 waterLevel1; /* ????????1 */
    VOS_UINT32 waterLevel2; /* ????????2 */
    VOS_UINT32 waterLevel3; /* ????????3 */
    VOS_UINT32 waterLevel4; /* ????????4,???? */
} ADS_UL_WaterMarkLevel;


typedef struct {
    VOS_UINT32 threshold1; /* ????????1 */
    VOS_UINT32 threshold2; /* ????????2 */
    VOS_UINT32 threshold3; /* ????????3 */
    VOS_UINT32 threshold4; /* ????????4 */
} ADS_UL_ThresholdLevel;


typedef struct {
    VOS_UINT32                   activeFlag;       /* ??????????0??????????1?????? */
    VOS_UINT32                   protectTmrExpCnt; /* ?????????????????????????????????? */
    ADS_UL_WaterMarkLevel waterMarkLevel;   /* ???????????????? */
    ADS_UL_ThresholdLevel  thresholdLevel;   /* ?????????????? */
    VOS_UINT32                   reserved[6];     /* ???????? */
} ADS_NV_DynamicThreshold;


typedef struct {
    /*
     * VCOM log????????????????INFO??NORM??ERR??DEBUG,??????
     * INFO: 1
     * NORMAL:2
     * ERR:4
     * DEBUG:7
     */
    VOS_UINT32 appVcomDebugLevel;
    /*
     * APP VCOM ????????????1bit????????????????????????
     * 0: ??????
     * 1: ????
     */
    VOS_UINT32 appVcomPortIdMask1; /* VCOM????ID???? */
    VOS_UINT32 appVcomPortIdMask2; /* VCOM????ID???? */

    /*
     * DMS log????????????????INFO??WARNING??ERROR??DEBUG,??????
     * INFO:    1
     * WARNING: 2
     * ERROR:   4
     * DEBUG:   7
     */
    VOS_UINT32 dmsDebugLevel;
    /*
     * DMS ????????????1bit????????????????????????
     * 0: ??????
     * 1: ????
     */
    VOS_UINT32 dmsPortIdMask1; /* ??0??31??bit????????DMS_PortId??????0????31?????? */
    VOS_UINT32 dmsPortIdMask2; /* ??0??31??bit????????DMS_PortId??????32????63?????? */
} TAF_NV_PortDebugCfg;


typedef struct {
    VOS_UINT8 macAddr[32]; /* MAC?????? */
} OM_MAC_Addr;


typedef struct {
    /*
     * NV????????????
     * *0??NV??????????
     *  1??NV??????????
     */
    VOS_UINT32 nvStatus;
    VOS_UINT8  productId[AT_PRODUCT_NAME_LENGHT]; /* ??????????ASIC????????????\0???????? */
    VOS_UINT8  reserve1;                           /* ???? */
    VOS_UINT8  reserve2;                           /* ???? */
} TAF_AT_ProductId;


typedef struct {
    /*
     * NV????????????
     * 0????????
     * 1????????
     */
    VOS_UINT8 status;
    /*
     * ????????????????????????
     * 0??????????????????????????????????????????AT^GLASTERR=1????????ERROR??
     * 1????????????AT^GLASTERR=1??????????????????????????
     */
    VOS_UINT8 errCodeRpt;
} NAS_NV_PppDialErrCode; /* ????TIM??????????NV???????? */

/*
 * ??    ??: NV_ITEM_HUAWEI_DYNAMIC_PID_TYPE(50091)
 * ????????: ????????????????????????????
 */
typedef struct {
    /*
     * ??NV????????????
     * 0??????????
     * 1????????
     */
    VOS_UINT32 nvStatus;
    VOS_UINT8  firstPortStyle[17];  /* ??????????????????, ????????0x00~0xFF?? */
    VOS_UINT8  rewindPortStyle[17]; /* ??????????????????, ????????0x00~0xFF?? */
    VOS_UINT8  reserved[22];        /* ?????? */
} AT_DynamicPidType;


typedef struct {
    VOS_UINT8 webPwd[AT_WEBUI_PWD_MAX]; /* ????????????WEBUI?????????? */
} TAF_AT_NvimWebAdminPassword;


typedef struct {
    /* ????XML reporting????????URL????????????????????????\0???? */
    VOS_UINT8 apRptSrvUrl[AT_AP_XML_RPT_SRV_URL_STR_LEN];
} TAF_AT_NvimApRptSrvUrl;


typedef struct {
    /* ????XML reporting??????????????????????????????????\0???? */
    VOS_UINT8 apXmlInfoType[AT_AP_NVIM_XML_RPT_INFO_TYPE_STR_LEN];
} TAF_AT_NvimApXmlInfoType;


typedef struct {
    /*
     * 0??????XML reporting??????
     * 1??????XML reporting??????
     */
    VOS_UINT8 apXmlRptFlg;
    VOS_UINT8 reserve[3]; /* ?????????????? */
} TAF_AT_NvimApXmlRptFlg;


typedef struct {
    /*
     * ??????????????^IMSSWITCH??????????<utran_enable>??????????????????????<lte_enable>??????????????0~1??
     * 0????
     * 1????
     */
    VOS_UINT8 utranRelationLteFlg;
    /*
     * ??????????????^IMSSWITCH??????????<gsm_enable>??????????????????????<lte_enable>??????????????0~1??
     * 0????
     * 1????
     */
    VOS_UINT8 gsmRelationLteFlg;
    VOS_UINT8 reserved1; /* ???????? */
    VOS_UINT8 reserved2; /* ???????? */
} TAF_NV_ImsSwitchRatRelationCfg;

#pragma pack(pop)

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* __ACORE_NV_STRU_GUCNAS_H__ */
