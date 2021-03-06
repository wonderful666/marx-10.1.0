

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#ifdef _PRE_WLAN_FEATURE_EDCA_OPT_AP

/* 1 头文件包含 */
#include "hmac_edca_opt.h"
#include "hmac_vap.h"
#include "oam_wdk.h"
#ifdef _PRE_WLAN_FEATURE_DUAL_BAND_PERF_OPT
#include "hmac_resource.h"
#endif
#include "securec.h"
#include "securectype.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_EDCA_OPT_C
/* 2 结构体定义 */
/* 3 宏定义 */
#define HMAC_EDCA_OPT_ADJ_STEP 2

/* (3-a)/3*X + a/3*Y */
#define WLAN_EDCA_OPT_MOD(X, Y, a) \
    (((X) * (WLAN_EDCA_OPT_MAX_WEIGHT_STA - a) + (Y) * (a)) / WLAN_EDCA_OPT_MAX_WEIGHT_STA);

/* 4 全局变量定义 */
#ifdef _PRE_WLAN_FEATURE_DUAL_BAND_PERF_OPT
/* 是否已绑定cpu0 */
oal_uint8 g_uc_has_bind_cpu0 = OAL_FALSE;
oal_bool_enum_uint8 g_en_2g_tx_amsdu = OAL_TRUE;
#endif

#if defined(_PRE_WLAN_FEATURE_RX_AGGR_EXTEND) || defined(_PRE_FEATURE_WAVEAPP_CLASSIFY)
extern oal_uint8 g_en_wave_bind_cpu0_ctrl;
#endif

/* 5 内部静态函数声明 */
OAL_STATIC oal_bool_enum_uint8 hmac_edca_opt_check_is_tcp_data(mac_ip_header_stru *pst_ip);
OAL_STATIC oal_uint32 hmac_edca_opt_stat_traffic_num(hmac_vap_stru *pst_hmac_vap,
                                                     oal_uint8 (*ppuc_traffic_num)[WLAN_TXRX_DATA_BUTT]);

/* 6 函数实现 */

OAL_STATIC oal_bool_enum_uint8 hmac_edca_opt_check_is_tcp_data(mac_ip_header_stru *pst_ip)
{
    oal_uint8 *puc_ip = (oal_uint8 *)pst_ip;
    oal_uint16 us_ip_len;
    oal_uint8 uc_ip_header_len = ((*puc_ip) & 0x0F) << 2; /* IP_HDR_LEN */
    oal_uint8 uc_tcp_header_len;

    /* 获取ip报文长度 */
    us_ip_len = (*(puc_ip + 2 /* length in ip header */)) << 8;
    us_ip_len |= *(puc_ip + 2 /* length in ip header */ + 1);

    /* 获取tcp header长度 */
    uc_tcp_header_len = *(puc_ip + uc_ip_header_len + 12 /* length in tcp header */);
    uc_tcp_header_len = (uc_tcp_header_len >> 4) << 2;

    if ((us_ip_len - uc_ip_header_len) == uc_tcp_header_len) {
        OAM_INFO_LOG3(0, OAM_SF_TX,
            "{hmac_edca_opt_check_is_tcp_data:is tcp ack, us_ip_len:%d, uc_ip_header_len:%d, uc_tcp_header_len:%d",
            us_ip_len, uc_ip_header_len, uc_tcp_header_len);
        return OAL_FALSE;
    } else {
        OAM_INFO_LOG3(0, OAM_SF_TX,
            "{hmac_edca_opt_check_is_tcp_data:is tcp data, us_ip_len:%d, uc_ip_header_len:%d, uc_tcp_header_len:%d",
            us_ip_len, uc_ip_header_len, uc_tcp_header_len);
        return OAL_TRUE;
    }
}

#ifdef _PRE_WLAN_FEATURE_DUAL_BAND_PERF_OPT

OAL_STATIC oal_void hmac_dyn_subcore_amsdu_switch(hmac_vap_stru *pst_hmac_vap, oal_uint8 uc_traffic_flag_num)
{
    mac_device_stru *pst_device;
    hmac_vap_stru *pst_hmac_vap_tmp;
    mac_chip_stru *pst_mac_chip;
    oal_uint8 uc_device_num;
    oal_uint8 uc_chip_num;
    oal_uint8 uc_vap_num;
    oal_bool_enum_uint8 en_all_vap_idle = OAL_TRUE;
    oal_bool_enum_uint8 en_5g_vap_idle = OAL_TRUE;

    if (uc_traffic_flag_num > 0) {
        pst_hmac_vap->uc_idle_cycle_num = 0;

        /* 有业务,关闭分核 */
        if (g_uc_has_bind_cpu0 == OAL_FALSE) {
            oal_hi_kernel_change_hw_rps_enable(0);
            g_uc_has_bind_cpu0 = OAL_TRUE;

            OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_TX,
                             "hmac_dyn_subcore_amsdu_switch: bind to CPU0. ul_traffic_flag_num = %d.",
                             uc_traffic_flag_num);
        }

        if ((pst_hmac_vap->st_vap_base_info.st_channel.en_band == WLAN_BAND_5G) && (g_en_2g_tx_amsdu == OAL_TRUE)) {
            /* 5G vap 有业务关2g tx amsdu */
            g_en_2g_tx_amsdu = OAL_FALSE;
            OAM_WARNING_LOG0(0, OAM_SF_TX, "hmac_dyn_subcore_amsdu_switch:g_en_2g_tx_amsdu set flase");
        }

        return;
    }

    if (pst_hmac_vap->uc_idle_cycle_num < HMAC_IDLE_CYCLE_TH) {
        pst_hmac_vap->uc_idle_cycle_num++;
        return;
    }

    /* 已是双核,返回,避免每个vap都做遍历 */
    if (g_uc_has_bind_cpu0 == OAL_FALSE) {
        return;
    }

    /* 遍历board下的所有vap是否都空闲 */
    for (uc_chip_num = 0; uc_chip_num < WLAN_CHIP_MAX_NUM_PER_BOARD; uc_chip_num++) {
        pst_mac_chip = hmac_res_get_mac_chip(uc_chip_num);

        for (uc_device_num = 0; uc_device_num < WLAN_DEVICE_MAX_NUM_PER_CHIP; uc_device_num++) {
            pst_device = mac_res_get_dev_etc(pst_mac_chip->auc_device_id[uc_device_num]);

            if (pst_device == OAL_PTR_NULL) {
                continue;
            }

            /* vap遍历 */
            for (uc_vap_num = 0; uc_vap_num < pst_device->uc_vap_num; uc_vap_num++) {
                pst_hmac_vap_tmp = mac_res_get_hmac_vap(pst_device->auc_vap_id[uc_vap_num]);
                if (pst_hmac_vap_tmp == OAL_PTR_NULL) {
                    continue;
                }

                if ((pst_hmac_vap_tmp->st_vap_base_info.en_vap_mode == WLAN_VAP_MODE_BSS_AP)
                    && (pst_hmac_vap_tmp->uc_edca_opt_flag_ap == OAL_TRUE)) {
                    if (pst_hmac_vap_tmp->uc_idle_cycle_num < HMAC_IDLE_CYCLE_TH) {
                        if ((WLAN_BAND_5G == pst_hmac_vap_tmp->st_vap_base_info.st_channel.en_band)
                            && (pst_hmac_vap_tmp->st_vap_base_info.en_vap_state == MAC_VAP_STATE_UP)) {
                            en_5g_vap_idle = OAL_FALSE;
                            en_all_vap_idle = OAL_FALSE;
                            return;
                        }
                        en_all_vap_idle = OAL_FALSE;
                    }
                }
            }
        }
    }

    if (g_en_2g_tx_amsdu == OAL_FALSE) {
        /* 5G vap 都没有业务 开2g tx amsdu */
        g_en_2g_tx_amsdu = OAL_TRUE;
        OAM_WARNING_LOG0(0, OAM_SF_TX, "hmac_dyn_subcore_amsdu_switch:g_en_2g_tx_amsdu set true");
    }

    if (en_all_vap_idle == OAL_TRUE) {
        /* 开启分核 */
        oal_hi_kernel_change_hw_rps_enable(1);
        g_uc_has_bind_cpu0 = OAL_FALSE;

        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_TX,
            "hmac_dyn_subcore_amsdu_switch: open 2CPU! uc_idle_cycle_num = %d",
            pst_hmac_vap->uc_idle_cycle_num);
    }
}
#endif


OAL_STATIC oal_uint32 hmac_edca_opt_stat_traffic_num(hmac_vap_stru *pst_hmac_vap,
                                                     oal_uint8 (*ppuc_traffic_num)[WLAN_TXRX_DATA_BUTT])
{
    mac_user_stru *pst_user;
    hmac_user_stru *pst_hmac_user;
    oal_uint8 uc_ac_idx;
    oal_uint8 uc_data_idx;
    mac_vap_stru *pst_vap = &(pst_hmac_vap->st_vap_base_info);
    oal_dlist_head_stru *pst_list_pos = OAL_PTR_NULL;

#ifdef _PRE_WLAN_FEATURE_DUAL_BAND_PERF_OPT
    oal_uint8 uc_traffic_flag_num = 0;
#endif

    pst_list_pos = pst_vap->st_mac_user_list_head.pst_next;

    for (; pst_list_pos != &(pst_vap->st_mac_user_list_head); pst_list_pos = pst_list_pos->pst_next) {
        pst_user = OAL_DLIST_GET_ENTRY(pst_list_pos, mac_user_stru, st_user_dlist);
        pst_hmac_user = mac_res_get_hmac_user_etc(pst_user->us_assoc_id);
        if (pst_hmac_user == OAL_PTR_NULL) {
            OAM_ERROR_LOG1(pst_vap->uc_vap_id, OAM_SF_CFG,
                           "hmac_edca_opt_stat_traffic_num: pst_hmac_user[%d] is null ptr!", pst_user->us_assoc_id);
            continue;
        }

        for (uc_ac_idx = 0; uc_ac_idx < WLAN_WME_AC_BUTT; uc_ac_idx++) {
            for (uc_data_idx = 0; uc_data_idx < WLAN_TXRX_DATA_BUTT; uc_data_idx++) {
                OAM_INFO_LOG4(0, OAM_SF_TX,
                              "mac_edca_opt_stat_traffic_num, assoc_id = %d, uc_ac = %d, uc_idx = %d, pkt_num = %d",
                              pst_user->us_assoc_id, uc_ac_idx, uc_data_idx,
                              pst_hmac_user->aaul_txrx_data_stat[uc_ac_idx][uc_data_idx]);

                if (pst_hmac_user->aaul_txrx_data_stat[uc_ac_idx][uc_data_idx] > HMAC_EDCA_OPT_PKT_NUM) {
                    ppuc_traffic_num[uc_ac_idx][uc_data_idx]++;

#ifdef _PRE_WLAN_FEATURE_DUAL_BAND_PERF_OPT
                    /* 避免溢出 */
                    if (uc_traffic_flag_num < HMAC_IDLE_CYCLE_TH) {
                        uc_traffic_flag_num++;
                        OAM_INFO_LOG3(pst_vap->uc_vap_id, OAM_SF_TX,
                            "hmac_edca_opt_stat_traffic_num: uc_ac_idx=%d, uc_data_idx=%d, txrx_data_stat=%d.",
                            uc_ac_idx, uc_data_idx, pst_hmac_user->aaul_txrx_data_stat[uc_ac_idx][uc_data_idx]);
                    }
#endif
                }

                /* 统计完毕置0 */
                pst_hmac_user->aaul_txrx_data_stat[uc_ac_idx][uc_data_idx] = 0;
            }
        }
    }

#ifdef _PRE_WLAN_FEATURE_DUAL_BAND_PERF_OPT
#if defined(_PRE_WLAN_FEATURE_RX_AGGR_EXTEND) || defined(_PRE_FEATURE_WAVEAPP_CLASSIFY)
    if (g_en_wave_bind_cpu0_ctrl == OAL_FALSE) {
        hmac_dyn_subcore_amsdu_switch(pst_hmac_vap, uc_traffic_flag_num);
    }
#else
    hmac_dyn_subcore_amsdu_switch(pst_hmac_vap, uc_traffic_flag_num);
#endif
#endif

    return OAL_SUCC;
}


oal_uint32 hmac_edca_opt_timeout_fn_etc(oal_void *p_arg)
{
    oal_uint8 aast_uc_traffic_num[WLAN_WME_AC_BUTT][WLAN_TXRX_DATA_BUTT];
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;

    frw_event_mem_stru *pst_event_mem;
    frw_event_stru *pst_event;

    if (OAL_UNLIKELY(p_arg == OAL_PTR_NULL)) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{hmac_edca_opt_timeout_fn_etc::p_arg is null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_hmac_vap = (hmac_vap_stru *)p_arg;

    /* 计数初始化 */
    memset_s(aast_uc_traffic_num, OAL_SIZEOF(aast_uc_traffic_num), 0, OAL_SIZEOF(aast_uc_traffic_num));

    /* 统计device下所有用户上/下行 TPC/UDP条数目 */
    hmac_edca_opt_stat_traffic_num(pst_hmac_vap, aast_uc_traffic_num);

    /***************************************************************************
        抛事件到dmac模块,将统计信息报给dmac
    ***************************************************************************/

    pst_event_mem = FRW_EVENT_ALLOC(OAL_SIZEOF(aast_uc_traffic_num));
    if (OAL_UNLIKELY(pst_event_mem == OAL_PTR_NULL)) {
        OAM_ERROR_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ANTI_INTF,
                       "{hmac_edca_opt_timeout_fn_etc::pst_event_mem null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_event = frw_get_event_stru(pst_event_mem);

    /* 填写事件头 */
    FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                       FRW_EVENT_TYPE_WLAN_CTX,
                       DMAC_WLAN_CTX_EVENT_SUB_TYPR_EDCA_OPT,
                       OAL_SIZEOF(aast_uc_traffic_num),
                       FRW_EVENT_PIPELINE_STAGE_1,
                       pst_hmac_vap->st_vap_base_info.uc_chip_id,
                       pst_hmac_vap->st_vap_base_info.uc_device_id,
                       pst_hmac_vap->st_vap_base_info.uc_vap_id);

    /* 拷贝参数 */
    if (EOK != memcpy_s(frw_get_event_payload(pst_event_mem), OAL_SIZEOF(aast_uc_traffic_num),
                        (oal_uint8 *)aast_uc_traffic_num, OAL_SIZEOF(aast_uc_traffic_num))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "hwifi_config_init_fcc_ce_txpwr_nvram::memcpy fail!");
        FRW_EVENT_FREE(pst_event_mem);
        return OAL_FAIL;
    }

    /* 分发事件 */
    frw_event_dispatch_event_etc(pst_event_mem);
    FRW_EVENT_FREE(pst_event_mem);

    return OAL_SUCC;
}


oal_void hmac_edca_opt_rx_pkts_stat_etc(oal_uint16 us_assoc_id, oal_uint8 uc_tidno, mac_ip_header_stru *pst_ip)
{
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    pst_hmac_user = (hmac_user_stru *)mac_res_get_hmac_user_etc(us_assoc_id);
    if (OAL_UNLIKELY(pst_hmac_user == OAL_PTR_NULL)) {
        OAM_ERROR_LOG1(0, OAM_SF_RX, "{hmac_edca_opt_rx_pkts_stat_etc::null param,pst_hmac_user[%d].}", us_assoc_id);
        return;
    }
    OAM_INFO_LOG0(0, OAM_SF_RX, "{hmac_edca_opt_rx_pkts_stat_etc}");

    /* 过滤IP_LEN 小于 HMAC_EDCA_OPT_MIN_PKT_LEN的报文 */
    if (OAL_NET2HOST_SHORT(pst_ip->us_tot_len) < HMAC_EDCA_OPT_MIN_PKT_LEN) {
        return;
    }

    if (pst_ip->uc_protocol == MAC_UDP_PROTOCAL) {
        pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_RX_UDP_DATA]++;
        OAM_INFO_LOG4(0, OAM_SF_RX,
                      "{hmac_edca_opt_rx_pkts_stat_etc:is udp_data, assoc_id = %d, tidno = %d, type = %d, num = %d",
                      pst_hmac_user->st_user_base_info.us_assoc_id, uc_tidno, WLAN_RX_UDP_DATA,
                      pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_RX_UDP_DATA]);
    } else if (pst_ip->uc_protocol == MAC_TCP_PROTOCAL) {
        if (hmac_edca_opt_check_is_tcp_data(pst_ip)) {
            pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_RX_TCP_DATA]++;
            OAM_INFO_LOG4(0, OAM_SF_RX,
                          "{hmac_edca_opt_rx_pkts_stat_etc:is tcp_data, assoc_id = %d, tidno = %d, type = %d, num = %d",
                          pst_hmac_user->st_user_base_info.us_assoc_id, uc_tidno, WLAN_RX_TCP_DATA,
                          pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_RX_TCP_DATA]);
        }
    } else {
        OAM_INFO_LOG0(0, OAM_SF_RX, "{hmac_tx_pkts_stat_for_edca_opt: neither UDP nor TCP ");
    }
}


oal_void hmac_edca_opt_tx_pkts_stat_etc(mac_tx_ctl_stru *pst_tx_ctl, oal_uint8 uc_tidno, mac_ip_header_stru *pst_ip)
{
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;

    pst_hmac_user = (hmac_user_stru *)mac_res_get_hmac_user_etc(MAC_GET_CB_TX_USER_IDX(pst_tx_ctl));
    if (OAL_UNLIKELY(pst_hmac_user == OAL_PTR_NULL)) {
        OAM_ERROR_LOG1(0, OAM_SF_CFG, "{hmac_edca_opt_rx_pkts_stat_etc::null param,pst_hmac_user[%d].}",
                       MAC_GET_CB_TX_USER_IDX(pst_tx_ctl));
        return;
    }
    OAM_INFO_LOG0(0, OAM_SF_RX, "{hmac_edca_opt_tx_pkts_stat_etc}");

    /* 过滤IP_LEN 小于 HMAC_EDCA_OPT_MIN_PKT_LEN的报文 */
    if (OAL_NET2HOST_SHORT(pst_ip->us_tot_len) < HMAC_EDCA_OPT_MIN_PKT_LEN) {
        return;
    }

    if (pst_ip->uc_protocol == MAC_UDP_PROTOCAL) {
        pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_TX_UDP_DATA]++;
        OAM_INFO_LOG4(0, OAM_SF_TX,
                      "{hmac_edca_opt_tx_pkts_stat_etc:is udp_data, assoc_id = %d, tidno = %d, type = %d, num = %d",
                      pst_hmac_user->st_user_base_info.us_assoc_id, uc_tidno, WLAN_TX_UDP_DATA,
                      pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_TX_UDP_DATA]);
    } else if (pst_ip->uc_protocol == MAC_TCP_PROTOCAL) {
        if (hmac_edca_opt_check_is_tcp_data(pst_ip)) {
            pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_TX_TCP_DATA]++;
            OAM_INFO_LOG4(0, OAM_SF_TX,
                          "{hmac_edca_opt_tx_pkts_stat_etc:is tcp_data, assoc_id = %d, tidno = %d, type = %d, num = %d",
                          pst_hmac_user->st_user_base_info.us_assoc_id, uc_tidno, WLAN_TX_TCP_DATA,
                          pst_hmac_user->aaul_txrx_data_stat[WLAN_WME_TID_TO_AC(uc_tidno)][WLAN_TX_TCP_DATA]);
        }
    } else {
        OAM_INFO_LOG0(0, OAM_SF_TX, "{hmac_edca_opt_tx_pkts_stat_etc: neither UDP nor TCP");
    }
}

#endif /* end of _PRE_WLAN_FEATURE_EDCA_OPT_AP */

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

