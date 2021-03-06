

/* 1 头文件包含 */
#include "oal_cfg80211.h"
#include "oam_ext_if.h"
#include "wlan_spec.h"
#include "hmac_mgmt_ap.h"
#include "hmac_encap_frame.h"
#include "hmac_encap_frame_ap.h"
#include "hmac_mgmt_bss_comm.h"
#include "mac_frame.h"
#include "hmac_rx_data.h"
#include "hmac_uapsd.h"
#include "hmac_tx_amsdu.h"
#include "mac_ie.h"
#include "mac_user.h"
#include "hmac_user.h"
#include "hmac_11i.h"
#include "hmac_protection.h"
#include "hmac_chan_mgmt.h"
#include "hmac_smps.h"
#include "hmac_fsm.h"
#include "hmac_ext_if.h"
#include "hmac_config.h"
#if (_PRE_WLAN_FEATURE_BLACKLIST_LEVEL != _PRE_WLAN_FEATURE_BLACKLIST_NONE)
#include "hmac_blacklist.h"
#endif
#include "hmac_dfs.h"
#include "hmac_p2p.h"
#include "hmac_blockack.h"
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#include "hmac_ext_if.h"
#endif
#ifdef _PRE_WLAN_FEATURE_OPMODE_NOTIFY
#include "hmac_opmode.h"
#endif
#ifdef _PRE_WLAN_FEATURE_SMPS
#include "hmac_smps.h"
#endif
#ifdef _PRE_WLAN_FEATURE_WMMAC
#include "hmac_wmmac.h"
#endif

#ifdef _PRE_WLAN_FEATURE_VIRTUAL_MULTI_STA
#include "hmac_wds.h"
#endif

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE) && (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#include "plat_pm_wlan.h"
#endif

#ifdef _PRE_WLAN_FEATURE_SNIFFER
#include <hwnet/ipv4/sysctl_sniffer.h>
#endif

#ifdef _PRE_WLAN_FEATURE_HIEX
#include "hmac_hiex.h"
#endif

#include "securec.h"
#include "securectype.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_MGMT_AP_C

/* 2 全局变量定义 */
/* 3 函数实现 */

oal_void hmac_handle_disconnect_rsp_ap_etc(hmac_vap_stru *pst_hmac_vap, hmac_user_stru *pst_hmac_user)
{
    mac_device_stru *pst_mac_device;
    frw_event_mem_stru *pst_event_mem = OAL_PTR_NULL;
    frw_event_stru *pst_event = OAL_PTR_NULL;

    pst_mac_device = mac_res_get_dev_etc(pst_hmac_vap->st_vap_base_info.uc_device_id);
    if (pst_mac_device == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_handle_disconnect_rsp_ap_etc::pst_mac_device null.}");
        return;
    }

    if (pst_hmac_user->en_report_kernel == OAL_FALSE) {
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "hmac_handle_disconnect_rsp_ap_etc::do not report kernel");
        return;
    }

    /* 抛扫描完成事件到WAL */
    pst_event_mem = FRW_EVENT_ALLOC(WLAN_MAC_ADDR_LEN);
    if (pst_event_mem == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                       "{hmac_handle_disconnect_rsp_ap_etc::pst_event_mem null.}");
        return;
    }

    /* 填写事件 */
    pst_event = frw_get_event_stru(pst_event_mem);

    FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                       FRW_EVENT_TYPE_HOST_CTX,
                       HMAC_HOST_CTX_EVENT_SUB_TYPE_STA_DISCONNECT_AP,
                       WLAN_MAC_ADDR_LEN,
                       FRW_EVENT_PIPELINE_STAGE_0,
                       pst_hmac_vap->st_vap_base_info.uc_chip_id,
                       pst_hmac_vap->st_vap_base_info.uc_device_id,
                       pst_hmac_vap->st_vap_base_info.uc_vap_id);

    /* 去关联的STA mac地址 */
    if (EOK != memcpy_s(frw_get_event_payload(pst_event_mem), WLAN_MAC_ADDR_LEN,
                        (oal_uint8 *)pst_hmac_user->st_user_base_info.auc_user_mac_addr, WLAN_MAC_ADDR_LEN)) {
        OAM_ERROR_LOG0(0, OAM_SF_ASSOC, "hmac_handle_disconnect_rsp_ap_etc::memcpy fail!");
        FRW_EVENT_FREE(pst_event_mem);
        return;
    }

    /* 分发事件 */
    frw_event_dispatch_event_etc(pst_event_mem);
    FRW_EVENT_FREE(pst_event_mem);
}


oal_void hmac_handle_connect_rsp_ap(hmac_vap_stru *pst_hmac_vap, hmac_user_stru *pst_hmac_user)
{
    mac_device_stru *pst_mac_device;
    frw_event_mem_stru *pst_event_mem = OAL_PTR_NULL;
    frw_event_stru *pst_event = OAL_PTR_NULL;
    hmac_asoc_user_req_ie_stru *pst_asoc_user_req_info = OAL_PTR_NULL;

    pst_mac_device = mac_res_get_dev_etc(pst_hmac_vap->st_vap_base_info.uc_device_id);
    if (pst_mac_device == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_handle_connect_rsp_ap::pst_mac_device null.}");
        return;
    }

    /* 抛关联一个新的sta完成事件到WAL */
    pst_event_mem = FRW_EVENT_ALLOC(WLAN_MAC_ADDR_LEN);
    if (pst_event_mem == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                       "{hmac_handle_connect_rsp_ap::pst_event_mem null.}");
        return;
    }

    /* 标记该user关联成功，并且上报内核 */
    pst_hmac_user->en_report_kernel = OAL_TRUE;

    /* 填写事件 */
    pst_event = frw_get_event_stru(pst_event_mem);

    FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                       FRW_EVENT_TYPE_HOST_CTX,
                       HMAC_HOST_CTX_EVENT_SUB_TYPE_STA_CONNECT_AP,
                       WLAN_MAC_ADDR_LEN,
                       FRW_EVENT_PIPELINE_STAGE_0,
                       pst_hmac_vap->st_vap_base_info.uc_chip_id,
                       pst_hmac_vap->st_vap_base_info.uc_device_id,
                       pst_hmac_vap->st_vap_base_info.uc_vap_id);

    pst_asoc_user_req_info = (hmac_asoc_user_req_ie_stru *)(pst_event->auc_event_data);

    /* 上报内核的关联sta发送的关联请求帧ie信息 */
    pst_asoc_user_req_info->puc_assoc_req_ie_buff = pst_hmac_user->puc_assoc_req_ie_buff;
    pst_asoc_user_req_info->ul_assoc_req_ie_len = pst_hmac_user->ul_assoc_req_ie_len;

    /* 关联的STA mac地址 */
    memcpy_s((oal_uint8 *)pst_asoc_user_req_info->auc_user_mac_addr, WLAN_MAC_ADDR_LEN,
             pst_hmac_user->st_user_base_info.auc_user_mac_addr, WLAN_MAC_ADDR_LEN);

    /* 分发事件 */
    frw_event_dispatch_event_etc(pst_event_mem);
    FRW_EVENT_FREE(pst_event_mem);
}


oal_void hmac_mgmt_update_auth_mib(hmac_vap_stru *pst_hmac_vap, oal_netbuf_stru *pst_auth_rsp)
{
    oal_uint16 us_status;
    oal_uint8 auc_addr1[6] = { 0 };
#ifdef _PRE_DEBUG_MODE
    oal_uint8 uc_auth_type = 0;
    oal_uint8 uc_auth_seq_num = 0;
#endif
    oal_uint8 *puc_mac_header = oal_netbuf_header(pst_auth_rsp);

    us_status = mac_get_auth_status(puc_mac_header);

    mac_get_addr1(puc_mac_header, auc_addr1, OAL_SIZEOF(auc_addr1));

    if (us_status != MAC_SUCCESSFUL_STATUSCODE) {
        /* DEBUG */
        OAM_INFO_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                      "{hmac_mgmt_update_auth_mib::Authentication of STA Failed.Status Code=%d.}", us_status);
    } else {
#ifdef _PRE_DEBUG_MODE
        /* DEBUG */
        uc_auth_type = (oal_uint8)mac_get_auth_algo_num(pst_auth_rsp);
        uc_auth_seq_num = (oal_uint8)mac_get_auth_seq_num(puc_mac_header);
        if (((uc_auth_type == WLAN_WITP_AUTH_OPEN_SYSTEM) && (uc_auth_seq_num == 2)) ||
            ((uc_auth_type == WLAN_WITP_AUTH_SHARED_KEY) && (uc_auth_seq_num == 4))) {
            OAM_INFO_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                          "{hmac_mgmt_update_auth_mib::Successfully Authenticated STA.}");
        }
#endif /* DEBUG_MODE */
    }
}
#ifdef _PRE_WLAN_FEATURE_SAE
OAL_STATIC oal_void hmac_ap_set_user_auth_alg_num(oal_uint16 us_user_index ,oal_uint16 us_auth_alg_num)
{
    mac_user_stru *pst_mac_user;

    pst_mac_user = mac_res_get_mac_user_etc(us_user_index);
    if(pst_mac_user == OAL_PTR_NULL) {
        return;
    }
    pst_mac_user->us_auth_alg_num = us_auth_alg_num;

    OAM_WARNING_LOG1(0, OAM_SF_AUTH,"{hmac_ap_set_user_auth_alg_num::\
                us_auth_alg_num=[%d]!}",pst_mac_user->us_auth_alg_num);
}


oal_void hmac_ap_up_rx_auth_req_to_host(hmac_vap_stru *pst_hmac_vap,
        oal_netbuf_stru *pst_auth_req, oal_uint8 us_auth_alg_num)
{
    oal_uint8 auc_addr2[ETHER_ADDR_LEN] = { 0 };
    oal_uint8 uc_is_seq1;
    oal_uint16 us_auth_seq;
    oal_uint16 us_user_index = 0xffff;
    oal_uint8 uc_auth_resend = 0;
    oal_uint32 ul_ret;

    hmac_rx_mgmt_send_to_host_etc(pst_hmac_vap, pst_auth_req);
    /* 获取STA的地址 */
    mac_get_address2(oal_netbuf_header(pst_auth_req), auc_addr2, ETHER_ADDR_LEN);
    if (mac_addr_is_zero_etc(auc_addr2)) {
        OAM_WARNING_LOG4(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                         "{hmac_ap_rx_auth_req::user mac:%02X:XX:XX:%02X:%02X:%02X is all 0 and invaild!}",
                         auc_addr2[0], auc_addr2[3], auc_addr2[4], auc_addr2[5]);
        return;
    }

    /* 解析auth transaction number */
    us_auth_seq = mac_get_auth_seq_num(oal_netbuf_header(pst_auth_req));
    if (us_auth_seq > HMAC_AP_AUTH_SEQ3_WEP_COMPLETE) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                         "{hmac_ap_rx_auth_req::auth recieve invalid seq, auth seq [%d]}",
                         us_auth_seq);
        return;
    }
    /* 获取用户idx */
    uc_is_seq1 = (us_auth_seq == WLAN_AUTH_TRASACTION_NUM_ONE);
    ul_ret = hmac_encap_auth_rsp_get_user_idx_etc(&(pst_hmac_vap->st_vap_base_info),
                                                  auc_addr2,
                                                  sizeof(auc_addr2),
                                                  uc_is_seq1,
                                                  &uc_auth_resend,
                                                  &us_user_index);
    if (ul_ret != OAL_SUCC) {
        if (ul_ret == OAL_ERR_CODE_CONFIG_EXCEED_SPEC) {
            OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                             "{hmac_ft_ap_up_rx_auth_req::hmac_ap_get_user_idx fail, users exceed config spec!}");
        } else {
            OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                             "{hmac_ft_ap_up_rx_auth_req::hmac_ap_get_user_idx Err!}");
        }
    }

    hmac_ap_set_user_auth_alg_num(us_user_index, us_auth_alg_num);

    return;
}
#endif /* _PRE_WLAN_FEATURE_SAE */

OAL_STATIC oal_uint32 hmac_ap_rx_auth_sae_handle(hmac_vap_stru *pst_hmac_vap,
    oal_netbuf_stru *pst_auth_req, oal_uint16 us_auth_alg_num)
{
#if defined(_PRE_WLAN_FEATURE_SAE)
    /* 如果是FT/SAE认证算法，上报hostapd */
    if (us_auth_alg_num == WLAN_MIB_AUTH_ALG_FT || us_auth_alg_num == WLAN_MIB_AUTH_ALG_SAE) {
            hmac_ap_up_rx_auth_req_to_host(pst_hmac_vap, pst_auth_req, us_auth_alg_num);
        return OAL_SUCC;
    } else if (us_auth_alg_num == WLAN_MIB_AUTH_ALG_OPEN_SYS &&
            g_st_mac_device_custom_cfg.bit_forbit_open_auth) {
        OAM_WARNING_LOG0(0, OAM_SF_CONN, "{hmac_ap_rx_auth_sae_handle::forbid open user to auth}");
        return OAL_SUCC;
    }
#endif
    return OAL_FAIL;
}


OAL_STATIC oal_void hmac_ap_rx_auth_req(hmac_vap_stru *pst_hmac_vap, oal_netbuf_stru *pst_auth_req)
{
    oal_netbuf_stru *pst_auth_rsp = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    oal_uint8 auc_chtxt[WLAN_CHTXT_SIZE] = { 0 };
    oal_uint16 us_auth_rsp_len;
    mac_tx_ctl_stru *pst_tx_ctl = OAL_PTR_NULL;
    oal_uint32 ul_ret;
    oal_uint32 ul_pedding_data = 0;
    oal_uint16 us_auth_alg_num;
    oal_uint8 uc_vap_id;

    if (OAL_ANY_NULL_PTR2(pst_hmac_vap, pst_auth_req)) {
        OAM_ERROR_LOG2(0, OAM_SF_AUTH, "{hmac_ap_rx_auth_req::param null, %x %x.}",
                       (uintptr_t)pst_hmac_vap, (uintptr_t)pst_auth_req);
        return;
    }

    uc_vap_id = pst_hmac_vap->st_vap_base_info.uc_vap_id;
    us_auth_alg_num = mac_get_auth_algo_num(pst_auth_req);

    OAM_WARNING_LOG1(uc_vap_id, OAM_SF_CONN, "{hmac_ap_rx_auth_req::rx_auth_alg_num=[%d]}",
        us_auth_alg_num);
    if (OAL_SUCC == hmac_ap_rx_auth_sae_handle(pst_hmac_vap, pst_auth_req, us_auth_alg_num)) {
        return;
    }


    if (us_auth_alg_num == WLAN_WITP_AUTH_SHARED_KEY) {
        /* 获取challenge text */
        /* 硬件寄存器获取随即byte,用于WEP SHARED加密 */
        oal_get_random_bytes((oal_int8 *)&auc_chtxt, WLAN_CHTXT_SIZE);
    }

    /* AP接收到STA发来的认证请求帧组相应的认证响应帧 */
    pst_auth_rsp = (oal_netbuf_stru *)OAL_MEM_NETBUF_ALLOC(OAL_NORMAL_NETBUF,
                                                           WLAN_MEM_NETBUF_SIZE2,
                                                           OAL_NETBUF_PRIORITY_MID);
    if (pst_auth_rsp == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(uc_vap_id, OAM_SF_AUTH, "{hmac_ap_rx_auth_req::pst_auth_rsp null.}");
        return;
    }

    OAL_MEM_NETBUF_TRACE(pst_auth_rsp, OAL_TRUE);

    memset_s(oal_netbuf_cb(pst_auth_rsp), OAL_NETBUF_CB_SIZE(), 0, OAL_NETBUF_CB_SIZE());

    us_auth_rsp_len = hmac_encap_auth_rsp_etc(&pst_hmac_vap->st_vap_base_info,
                                              pst_auth_rsp,
                                              pst_auth_req,
                                              auc_chtxt,
                                              sizeof(auc_chtxt));
    if (us_auth_rsp_len == 0) {
        oal_netbuf_free(pst_auth_rsp);
        OAM_ERROR_LOG0(uc_vap_id, OAM_SF_AUTH,"{hmac_ap_rx_auth_req::us_auth_rsp_len is 0.}");
        return;
    }

    oal_netbuf_put(pst_auth_rsp, us_auth_rsp_len);

    hmac_mgmt_update_auth_mib(pst_hmac_vap, pst_auth_rsp);

    /* 获取cb字段 */
    pst_tx_ctl = (mac_tx_ctl_stru *)oal_netbuf_cb(pst_auth_rsp);

    /* 发送认证响应帧之前，将用户的节能状态复位 */
    /* hmac_encap_auth_rsp中user id的字段值非法，表明组帧失败, 直接回复失败的认证响应帧 */
    if (MAC_GET_CB_TX_USER_IDX(pst_tx_ctl) == MAC_INVALID_USER_ID) {
        OAM_WARNING_LOG1(uc_vap_id, OAM_SF_AUTH,"{hmac_ap_rx_auth_req::user[%d] auth fail.}",
                         MAC_GET_CB_TX_USER_IDX(pst_tx_ctl));
    } else {
        pst_hmac_user = mac_res_get_hmac_user_etc(MAC_GET_CB_TX_USER_IDX(pst_tx_ctl));
        if (pst_hmac_user == OAL_PTR_NULL) {
            oal_netbuf_free(pst_auth_rsp);
            OAM_ERROR_LOG0(uc_vap_id, OAM_SF_AUTH,"{hmac_ap_rx_auth_req::hmac_user null.}");
            return;
        }

        hmac_mgmt_reset_psm_etc(&pst_hmac_vap->st_vap_base_info, MAC_GET_CB_TX_USER_IDX(pst_tx_ctl));
    }

    hmac_config_scan_abort_etc(&pst_hmac_vap->st_vap_base_info, OAL_SIZEOF(oal_uint32), (oal_uint8 *)&ul_pedding_data);

    /* 无论认证成功或者失败，都抛事件给dmac发送认证帧 */
    ul_ret = hmac_tx_mgmt_send_event_etc(&pst_hmac_vap->st_vap_base_info, pst_auth_rsp, us_auth_rsp_len);
    if (ul_ret != OAL_SUCC) {
        oal_netbuf_free(pst_auth_rsp);
        OAM_WARNING_LOG1(uc_vap_id, OAM_SF_AUTH,"{hmac_ap_rx_auth_req::ul_ret=[%d].}", ul_ret);
    }
}


OAL_STATIC oal_uint32 hmac_ap_rx_deauth_req(hmac_vap_stru *pst_hmac_vap,
                                            oal_uint8 *puc_mac_hdr,
                                            oal_bool_enum_uint8 en_is_protected)
{
    oal_uint8 *puc_sa = OAL_PTR_NULL;
    oal_uint8 *puc_da = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    oal_uint16 us_err_code;
    oal_uint32 ul_ret;

    if (OAL_ANY_NULL_PTR2(pst_hmac_vap, puc_mac_hdr)) {
        OAM_ERROR_LOG2(0, OAM_SF_AUTH, "{hmac_ap_rx_deauth_req::param null, %p %p.}",
                       (uintptr_t)pst_hmac_vap, (uintptr_t)puc_mac_hdr);
        return OAL_ERR_CODE_PTR_NULL;
    }

    mac_rx_get_sa((mac_ieee80211_frame_stru *)puc_mac_hdr, &puc_sa);

    us_err_code = *((oal_uint16 *)(puc_mac_hdr + MAC_80211_FRAME_LEN));

    /* 增加接收到去认证帧时的维测信息 */
    OAM_WARNING_LOG4(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_CONN,
                     "{hmac_ap_rx_deauth_req:: DEAUTH rx, reason code = %d, sa[XX:XX:XX:%2X:%2X:%2X]}",
                     us_err_code, puc_sa[3], puc_sa[4], puc_sa[5]);

    pst_hmac_user = mac_vap_get_hmac_user_by_addr_etc(&pst_hmac_vap->st_vap_base_info, puc_sa);
    if (pst_hmac_user == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                         "{aput rx deauth frame, pst_hmac_user null.}");
        return OAL_FAIL;
    }

#if (_PRE_WLAN_FEATURE_PMF != _PRE_PMF_NOT_SUPPORT)
    /* 检查是否需要发送SA query request */
    if ((pst_hmac_user->st_user_base_info.en_user_asoc_state == MAC_USER_STATE_ASSOC) &&
        (OAL_SUCC == hmac_pmf_check_err_code_etc(&pst_hmac_user->st_user_base_info, en_is_protected, puc_mac_hdr))) {
        /* 在关联状态下收到未加密的ReasonCode 6/7需要开启SA Query流程 */
        ul_ret = hmac_start_sa_query_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user,
                                         pst_hmac_user->st_user_base_info.st_cap_info.bit_pmf_active);
        if (ul_ret != OAL_SUCC) {
            return OAL_ERR_CODE_PMF_SA_QUERY_START_FAIL;
        }
        return OAL_SUCC;
    }
#endif

    /* 如果该用户的管理帧加密属性不一致，丢弃该报文 */
    mac_rx_get_da((mac_ieee80211_frame_stru *)puc_mac_hdr, &puc_da);
    if ((OAL_TRUE != ETHER_IS_MULTICAST(puc_da)) &&
        (en_is_protected != pst_hmac_user->st_user_base_info.st_cap_info.bit_pmf_active)) {
        OAM_ERROR_LOG2(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_AUTH,
                       "{hmac_ap_rx_deauth_req::PMF check failed %d %d.}",
                       en_is_protected, pst_hmac_user->st_user_base_info.st_cap_info.bit_pmf_active);
        return OAL_FAIL;
    }
#ifdef _PRE_WLAN_1103_CHR
    CHR_EXCEPTION_REPORT(CHR_PLATFORM_EXCEPTION_EVENTID, CHR_SYSTEM_WIFI,
                         CHR_LAYER_DRV, CHR_WIFI_DRV_EVENT_SOFTAP_PASSIVE_DISCONNECT, us_err_code);
#endif
    /* 抛事件上报内核，已经去关联某个STA */
    hmac_handle_disconnect_rsp_ap_etc(pst_hmac_vap, pst_hmac_user);

    hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);
    return OAL_SUCC;
}

OAL_STATIC oal_void hmac_user_sort_op_rates(hmac_user_stru *pst_hmac_user)
{
    oal_uint8 uc_loop;
    oal_uint8 uc_num_rates;
    oal_uint8 uc_min_rate;
    oal_uint8 uc_temp_rate; /* 临时速率，用于数据交换 */
    oal_uint8 uc_index;
    oal_uint8 uc_temp_index; /* 临时索引，用于数据交换 */

    uc_num_rates = pst_hmac_user->st_op_rates.uc_rs_nrates;

    for (uc_loop = 0; uc_loop < uc_num_rates; uc_loop++) {
        /* 记录当前速率为最小速率 */
        uc_min_rate = (pst_hmac_user->st_op_rates.auc_rs_rates[uc_loop] & 0x7F);
        uc_temp_index = uc_loop;

        /* 依次查找最小速率 */
        for (uc_index = uc_loop + 1; uc_index < uc_num_rates; uc_index++) {
            /* 记录的最小速率大于如果当前速率 */
            if (uc_min_rate > (pst_hmac_user->st_op_rates.auc_rs_rates[uc_index] & 0x7F)) {
                /* 更新最小速率 */
                uc_min_rate = (pst_hmac_user->st_op_rates.auc_rs_rates[uc_index] & 0x7F);
                uc_temp_index = uc_index;
            }
        }

        uc_temp_rate = pst_hmac_user->st_op_rates.auc_rs_rates[uc_loop];
        pst_hmac_user->st_op_rates.auc_rs_rates[uc_loop] = pst_hmac_user->st_op_rates.auc_rs_rates[uc_temp_index];
        pst_hmac_user->st_op_rates.auc_rs_rates[uc_temp_index] = uc_temp_rate;
    }

    /*******************************************************************
      重排11g模式的可操作速率，使11b速率都聚集在11a之前
      802.11a 速率:6、9、12、18、24、36、48、54Mbps
      802.11b 速率:1、2、5.5、11Mbps
      由于按由小到大排序后802.11b中的速率11Mbps在802.11a中，下标为5
      所以从第五位进行检验并排序。
    *******************************************************************/
    if (pst_hmac_user->st_op_rates.uc_rs_nrates == MAC_DATARATES_PHY_80211G_NUM) { /* 11g_compatibility mode */
        if ((pst_hmac_user->st_op_rates.auc_rs_rates[5] & 0x7F) == 0x16) { /* 11Mbps */
            uc_temp_rate = pst_hmac_user->st_op_rates.auc_rs_rates[5];
            pst_hmac_user->st_op_rates.auc_rs_rates[5] = pst_hmac_user->st_op_rates.auc_rs_rates[4];
            pst_hmac_user->st_op_rates.auc_rs_rates[4] = pst_hmac_user->st_op_rates.auc_rs_rates[3];
            pst_hmac_user->st_op_rates.auc_rs_rates[3] = uc_temp_rate;
        }
    }
}


OAL_STATIC oal_bool_enum_uint8 hmac_ap_up_update_sta_cap_info(hmac_vap_stru *pst_hmac_vap,
                                                              oal_uint16 us_cap_info,
                                                              hmac_user_stru *pst_hmac_user,
                                                              mac_status_code_enum_uint16 *pen_status_code)
{
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oal_uint32 ul_ret;
    mac_cap_info_stru *pst_cap_info = (mac_cap_info_stru *)(&us_cap_info);

    if (OAL_ANY_NULL_PTR2(pst_hmac_vap, pst_hmac_user)) {
        OAM_ERROR_LOG3(0, OAM_SF_ANY, "{hmac_ap_up_update_sta_cap_info::param null, %x %x %x.}",
                       (uintptr_t)pst_hmac_vap, (uintptr_t)pst_hmac_user, (uintptr_t)pen_status_code);
        *pen_status_code = MAC_UNSPEC_FAIL;
        return OAL_FALSE;
    }
    pst_mac_vap = &(pst_hmac_vap->st_vap_base_info);

    /* check bss capability info MAC,忽略MAC能力不匹配的STA */
    ul_ret = hmac_check_bss_cap_info_etc(us_cap_info, pst_mac_vap);
    if (ul_ret != OAL_TRUE) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ANY,
                       "{hmac_ap_up_update_sta_cap_info::hmac_check_bss_cap_info_etc failed[%d].}", ul_ret);
        *pen_status_code = MAC_UNSUP_CAP;
        return OAL_FALSE;
    }

    /* 如果以上各能力信息均满足关联要求，则继续处理其他能力信息 */
    mac_vap_check_bss_cap_info_phy_ap_etc(us_cap_info, pst_mac_vap);

    if ((0 == pst_cap_info->bit_privacy) &&
        (WLAN_80211_CIPHER_SUITE_NO_ENCRYP != pst_hmac_user->st_user_base_info.st_key_info.en_cipher_type)) {
        *pen_status_code = MAC_UNSPEC_FAIL;
        return OAL_FALSE;
    }

    return OAL_TRUE;
}


OAL_STATIC oal_uint16 hmac_check_wpa_cipher_ap(mac_vap_stru *pst_mac_vap, oal_uint8 *puc_ie)
{
    mac_crypto_settings_stru st_crypto;

    if (OAL_SUCC != mac_ie_get_wpa_cipher(puc_ie, &st_crypto)) {
        return MAC_INVALID_INFO_ELMNT;
    }

    if (mac_mib_get_wpa_group_suite(pst_mac_vap) != st_crypto.ul_group_suite) {
        return MAC_INVALID_GRP_CIPHER;
    }

    if (0 == mac_mib_wpa_pair_match_suites_s(pst_mac_vap, st_crypto.aul_pair_suite, sizeof(st_crypto.aul_pair_suite))) {
        return MAC_INVALID_PW_CIPHER;
    }

    if (0 == mac_mib_wpa_akm_match_suites_s(pst_mac_vap, st_crypto.aul_akm_suite, sizeof(st_crypto.aul_akm_suite))) {
        return MAC_INVALID_AKMP_CIPHER;
    }

    return MAC_SUCCESSFUL_STATUSCODE;
}

OAL_STATIC oal_uint16 hmac_check_rsn_cipher_ap(mac_vap_stru *pst_mac_vap,
                                               mac_user_stru *pst_mac_user,
                                               oal_uint8 *puc_ie)
{
    mac_crypto_settings_stru st_crypto;
    oal_uint16 us_rsn_cap;

    if (OAL_SUCC != mac_ie_get_rsn_cipher(puc_ie, &st_crypto)) {
        return MAC_INVALID_INFO_ELMNT;
    }

    if (mac_mib_get_rsn_group_suite(pst_mac_vap) != st_crypto.ul_group_suite) {
        return MAC_INVALID_GRP_CIPHER;
    }

    if (0 == mac_mib_rsn_pair_match_suites_s(pst_mac_vap, st_crypto.aul_pair_suite, sizeof(st_crypto.aul_pair_suite))) {
        return MAC_INVALID_PW_CIPHER;
    }

    if (0 == mac_mib_rsn_akm_match_suites_s(pst_mac_vap, st_crypto.aul_akm_suite, sizeof(st_crypto.aul_akm_suite))) {
        return MAC_INVALID_AKMP_CIPHER;
    }

    us_rsn_cap = mac_get_rsn_capability_etc(puc_ie);

    /* 预认证能力检查 */
    if ((us_rsn_cap & BIT0) != 0) {
        return MAC_INVALID_RSN_INFO_CAP;
    }

    /* sae auth 成功但是user 不支持pmf,不允许入网,待认证结束删除维测 */
    OAM_WARNING_LOG2(0, OAM_SF_ASSOC,
                     "{hmac_check_rsn_cipher_ap::us_auth_alg_num=[%d],us_rsn_cap=[%X].}",
                     pst_mac_user->us_auth_alg_num, us_rsn_cap);
    if (pst_mac_user->us_auth_alg_num == WLAN_MIB_AUTH_ALG_SAE && (!(us_rsn_cap & BIT7))) {
        OAM_WARNING_LOG0(0, OAM_SF_ASSOC,
                     "{hmac_check_rsn_cipher_ap::WLAN_MIB_AUTH_ALG_SAE MFP disable.}");
        return MAC_R0KH_UNREACHABLE;
    }

    /* 本地强制，对端没有MFP能力 */
    if ((OAL_TRUE == mac_mib_get_dot11RSNAMFPR(pst_mac_vap)) && (!(us_rsn_cap & BIT7))) {
        return MAC_MFP_VIOLATION;
    }
    /* 对端强制，本地没有MFP能力 */
    if ((OAL_FALSE == mac_mib_get_dot11RSNAMFPC(pst_mac_vap)) && (us_rsn_cap & BIT6)) {
        return MAC_MFP_VIOLATION;
    }

#if (_PRE_WLAN_FEATURE_PMF != _PRE_PMF_NOT_SUPPORT)
    if ((OAL_TRUE == mac_mib_get_dot11RSNAMFPC(pst_mac_vap)) && (us_rsn_cap & BIT7)) {
        mac_user_set_pmf_active_etc(pst_mac_user, OAL_TRUE);
    }
#endif

    return MAC_SUCCESSFUL_STATUSCODE;
}


OAL_STATIC oal_uint16 hmac_check_rsn_ap(mac_vap_stru *pst_mac_vap, mac_user_stru *pst_mac_user,
                                        oal_uint8 *puc_payload, oal_uint32 ul_msg_len)
{
    oal_uint8 *puc_rsn_ie = OAL_PTR_NULL;
    oal_uint8 *puc_wpa_ie = OAL_PTR_NULL;

    /* 若本地没有rsn能力,忽略检查ie。以增加兼容性 */
    if (OAL_FALSE == mac_mib_get_rsnaactivated(pst_mac_vap)) {
        return MAC_SUCCESSFUL_STATUSCODE;
    }

    /* 获取RSN和WPA IE信息 */
    puc_rsn_ie = mac_find_ie_etc(MAC_EID_RSN, puc_payload, (oal_int32)ul_msg_len);
    puc_wpa_ie = mac_find_vendor_ie_etc(MAC_WLAN_OUI_MICROSOFT, MAC_OUITYPE_WPA, puc_payload, (oal_int32)ul_msg_len);
    if ((puc_rsn_ie == OAL_PTR_NULL) && (puc_wpa_ie == OAL_PTR_NULL)) {
        if (OAL_TRUE == mac_mib_get_WPSActive(pst_mac_vap)) {
            return MAC_SUCCESSFUL_STATUSCODE;
        } else {
            return MAC_INVALID_INFO_ELMNT;
        }
    }

    if ((pst_mac_vap->st_cap_flag.bit_wpa2 == OAL_TRUE) && (puc_rsn_ie != OAL_PTR_NULL)) {
        return hmac_check_rsn_cipher_ap(pst_mac_vap, pst_mac_user, puc_rsn_ie);
    }

    if ((pst_mac_vap->st_cap_flag.bit_wpa == OAL_TRUE) && (puc_wpa_ie != OAL_PTR_NULL)) {
        return hmac_check_wpa_cipher_ap(pst_mac_vap, puc_wpa_ie);
    }

    return MAC_CIPHER_REJ;
}


OAL_STATIC OAL_INLINE oal_bool_enum_uint8 hmac_is_erp_sta(hmac_user_stru *pst_hmac_user)
{
    oal_uint32 ul_loop = 0;
    oal_bool_enum_uint8 en_is_erp_sta;

    /* 确认是否是erp 站点 */
    if (pst_hmac_user->st_op_rates.uc_rs_nrates <= MAC_NUM_DR_802_11B) {
        en_is_erp_sta = OAL_FALSE;
        for (ul_loop = 0; ul_loop < pst_hmac_user->st_op_rates.uc_rs_nrates; ul_loop++) {
            /* 如果支持速率不在11b的1M, 2M, 5.5M, 11M范围内，则说明站点为支持ERP的站点 */
            if (((pst_hmac_user->st_op_rates.auc_rs_rates[ul_loop] & 0x7F) != 0x2) &&
                ((pst_hmac_user->st_op_rates.auc_rs_rates[ul_loop] & 0x7F) != 0x4) &&
                ((pst_hmac_user->st_op_rates.auc_rs_rates[ul_loop] & 0x7F) != 0xb) &&
                ((pst_hmac_user->st_op_rates.auc_rs_rates[ul_loop] & 0x7F) != 0x16)) {
                en_is_erp_sta = OAL_TRUE;
                break;
            }
        }
    } else {
        en_is_erp_sta = OAL_TRUE;
        ;
    }

    return en_is_erp_sta;
}


OAL_STATIC oal_uint32 hmac_ap_up_update_legacy_capability(hmac_vap_stru *pst_hmac_vap,
                                                          hmac_user_stru *pst_hmac_user,
                                                          oal_uint16 us_cap_info)
{
    mac_protection_stru *pst_protection = &(pst_hmac_vap->st_vap_base_info.st_protection);
    mac_user_stru *pst_mac_user = &(pst_hmac_user->st_user_base_info);
    oal_bool_enum_uint8 en_is_erp_sta;

    /* 如果STA不支持short slot */
    if ((us_cap_info & MAC_CAP_SHORT_SLOT) != MAC_CAP_SHORT_SLOT) {
        /* 如果STA之前没有关联， 或者之前以支持short slot站点身份关联，需要update处理 */
        if ((pst_mac_user->en_user_asoc_state != MAC_USER_STATE_ASSOC)
            || (pst_hmac_user->st_hmac_cap_info.bit_short_slot_time == OAL_TRUE)) {
            pst_protection->uc_sta_no_short_slot_num++;
        }

        pst_hmac_user->st_hmac_cap_info.bit_short_slot_time = OAL_FALSE;
    } else { /* 如果STA支持short slot */
        /* 如果STA以不支持short slot站点身份关联，需要update处理 */
        if ((pst_mac_user->en_user_asoc_state == MAC_USER_STATE_ASSOC)
            && (pst_hmac_user->st_hmac_cap_info.bit_short_slot_time == OAL_FALSE)
            && (0 != pst_protection->uc_sta_no_short_slot_num)) {
            pst_protection->uc_sta_no_short_slot_num--;
        }

        pst_hmac_user->st_hmac_cap_info.bit_short_slot_time = OAL_TRUE;
    }

    pst_hmac_user->st_user_stats_flag.bit_no_short_slot_stats_flag = OAL_TRUE;

    /* 如果STA不支持short preamble */
    if ((us_cap_info & MAC_CAP_SHORT_PREAMBLE) != MAC_CAP_SHORT_PREAMBLE) {
        /* 如果STA之前没有关联， 或者之前以支持short preamble站点身份关联，需要update处理 */
        if ((pst_mac_user->en_user_asoc_state != MAC_USER_STATE_ASSOC)
            || (pst_hmac_user->st_hmac_cap_info.bit_short_preamble == OAL_TRUE)) {
            pst_protection->uc_sta_no_short_preamble_num++;
        }

        pst_hmac_user->st_hmac_cap_info.bit_short_preamble = OAL_FALSE;
    } else { /* 如果STA支持short preamble */
        /* 如果STA之前以不支持short preamble站点身份关联，需要update处理 */
        if ((pst_mac_user->en_user_asoc_state == MAC_USER_STATE_ASSOC)
            && (pst_hmac_user->st_hmac_cap_info.bit_short_preamble == OAL_FALSE)
            && (0 != pst_protection->uc_sta_no_short_preamble_num)) {
            pst_protection->uc_sta_no_short_preamble_num--;
        }

        pst_hmac_user->st_hmac_cap_info.bit_short_preamble = OAL_TRUE;
    }

    pst_hmac_user->st_user_stats_flag.bit_no_short_preamble_stats_flag = OAL_TRUE;

    /* 确定user是否是erp站点 */
    en_is_erp_sta = hmac_is_erp_sta(pst_hmac_user);
    /* 如果STA不支持ERP */
    if (en_is_erp_sta == OAL_FALSE) {
        /* 如果STA之前没有关联， 或者之前以支持ERP站点身份关联，需要update处理 */
        if ((pst_mac_user->en_user_asoc_state != MAC_USER_STATE_ASSOC)
            || (pst_hmac_user->st_hmac_cap_info.bit_erp == OAL_TRUE)) {
            pst_protection->uc_sta_non_erp_num++;
        }

        pst_hmac_user->st_hmac_cap_info.bit_erp = OAL_FALSE;
    } else { /* 如果STA支持ERP */
        /* 如果STA之前以不支持ERP身份站点关联，需要update处理 */
        if ((pst_mac_user->en_user_asoc_state == MAC_USER_STATE_ASSOC)
            && (pst_hmac_user->st_hmac_cap_info.bit_erp == OAL_FALSE)
            && (0 != pst_protection->uc_sta_non_erp_num)) {
            pst_protection->uc_sta_non_erp_num--;
        }

        pst_hmac_user->st_hmac_cap_info.bit_erp = OAL_TRUE;
    }

    pst_hmac_user->st_user_stats_flag.bit_no_erp_stats_flag = OAL_TRUE;

    if ((us_cap_info & MAC_CAP_SPECTRUM_MGMT) != MAC_CAP_SPECTRUM_MGMT) {
        mac_user_set_spectrum_mgmt_etc(&pst_hmac_user->st_user_base_info, OAL_FALSE);
    } else {
        mac_user_set_spectrum_mgmt_etc(&pst_hmac_user->st_user_base_info, OAL_TRUE);
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_ap_up_update_asoc_entry_prot(oal_uint8 *puc_payload,
                                                        oal_uint8 uc_sub_type,
                                                        oal_uint32 ul_msg_len,
                                                        hmac_user_stru *pst_hmac_user)
{
    /* WMM */
    hmac_uapsd_update_user_para_etc(puc_payload, uc_sub_type, ul_msg_len, pst_hmac_user);

    return OAL_SUCC;
}



oal_bool_enum hmac_go_is_auth(mac_vap_stru *pst_mac_vap)
{
    oal_dlist_head_stru *pst_entry = OAL_PTR_NULL;
    oal_dlist_head_stru *pst_dlist_tmp = OAL_PTR_NULL;
    mac_user_stru *pst_user_tmp = OAL_PTR_NULL;

    if (WLAN_P2P_GO_MODE != pst_mac_vap->en_p2p_mode) {
        return OAL_FALSE;
    }

    OAL_DLIST_SEARCH_FOR_EACH_SAFE(pst_entry, pst_dlist_tmp, &(pst_mac_vap->st_mac_user_list_head))
    {
        pst_user_tmp = OAL_DLIST_GET_ENTRY(pst_entry, mac_user_stru, st_user_dlist);
        if (pst_user_tmp == OAL_PTR_NULL) {
            OAM_ERROR_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{hmac_ap_is_auth::pst_user_tmp null.}");
            continue;
        }
        if ((pst_user_tmp->en_user_asoc_state == MAC_USER_STATE_AUTH_COMPLETE)
            || (MAC_USER_STATE_AUTH_KEY_SEQ1 == pst_user_tmp->en_user_asoc_state)) {
            return OAL_TRUE;
        }
    }
    return OAL_FALSE;
}


void hmac_ap_check_owe_capa(hmac_vap_stru *pst_hmac_vap, hmac_user_stru *pst_hmac_user,
                            oal_uint8 *puc_payload, oal_uint32 ul_payload_len,
                            mac_status_code_enum_uint16 *pen_status_code)
{
    oal_uint32 ul_ret;
    oal_uint8 *puc_rsn_ie = OAL_PTR_NULL;
    oal_uint8 *puc_owe_dh = OAL_PTR_NULL;
    mac_crypto_settings_stru st_crypto = { 0 };
    oal_uint16 us_suite_idx = 0;
    oal_uint16 owe_group;

    pst_hmac_user->st_hmac_cap_info.bit_owe = OAL_FALSE;

    /* 优先查找 RSN 信息元素 */
    puc_rsn_ie = mac_find_ie_etc(MAC_EID_RSN, (oal_uint8 *)puc_payload, (oal_int32)ul_payload_len);
    if (puc_rsn_ie == OAL_PTR_NULL) {
        return;
    }

    /* 1. check OWE AKM */
    ul_ret = mac_ie_get_rsn_cipher(puc_rsn_ie, &st_crypto);
    if (ul_ret != OAL_SUCC) {
        return;
    }

    /* AKM Suite */
    for (us_suite_idx = 0; us_suite_idx < WLAN_AUTHENTICATION_SUITES; us_suite_idx++) {
        if (st_crypto.aul_akm_suite[us_suite_idx] == MAC_RSN_AKM_OWE) {
            pst_hmac_user->st_hmac_cap_info.bit_owe = OAL_TRUE;
            break;
        }
    }
    if (pst_hmac_user->st_hmac_cap_info.bit_owe == OAL_FALSE) {
        return;
    }

    OAM_WARNING_LOG3(0, OAM_SF_WPA,
                     "hmac_ap_check_owe_capa::get_rsn_cipher owe:%d, akm_suite[0]:0x%x akm_suite[1]:0x%x",
                     pst_hmac_user->st_hmac_cap_info.bit_owe,
                     st_crypto.aul_akm_suite[0], st_crypto.aul_akm_suite[1]);

    /* 2. check OWE Diffie-Hellman Parameter Element */
    /*************************************************************************/
    /*             OWE Diffie-Hellman Parameter element                      */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /* |Element ID | Length | Element ID |  element-specific data            */
    /* |           |        | Extension  |  group        | public key        */
    /* --------------------------------------------------------------------- */
    /* |     1     |    1   |    1       |      2        |                   */
    /* --------------------------------------------------------------------- */
    /* |    255    |variable|    32      |   19/20/21    |                   */
    /* --------------------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/
    puc_owe_dh = mac_find_ie_ext_ie(MAC_EID_EXTENSION, MAC_EID_EXT_OWE_DH_PARAM,
                                    puc_payload, (oal_int32)ul_payload_len);
    /* owe dh ie len不足3字节不处理 */
    if ((puc_owe_dh == OAL_PTR_NULL) || (puc_owe_dh[1] < MAC_EXT_OWE_DH_PARAM_LEN)) {
        pst_hmac_user->st_hmac_cap_info.bit_owe = OAL_FALSE;
        return;
    }

    puc_owe_dh += MAC_IE_HDR_LEN; /* skip EID and length */
    owe_group = *(puc_owe_dh + 1);
    owe_group += (*(puc_owe_dh + 2)) << 8; /*  字节2左移8位 */
    if ((owe_group == 0) || ((pst_hmac_vap->owe_group[0] != owe_group) &&
        (pst_hmac_vap->owe_group[1] != owe_group) && (pst_hmac_vap->owe_group[2] != owe_group))) { /* group 1 2 */
        *pen_status_code = MAC_FINITE_CYCLIC_GROUP_NOT_SUPPORTED; /* OWE group not matched */
    }

    OAM_WARNING_LOG4(0, OAM_SF_WPA,
                     "hmac_ap_check_owe_capa::vap owe_group_cap=%d, associated STA bit_owe=%d owe_group:%d, status:%d",
                     pst_hmac_vap->owe_group_cap, pst_hmac_user->st_hmac_cap_info.bit_owe,
                     owe_group, *pen_status_code);

    return;
}

OAL_STATIC OAL_INLINE oal_void hmac_ap_2g_11ac_user_asoc(hmac_vap_stru *pst_hmac_vap,
                                                         oal_uint8 *puc_payload,
                                                         oal_uint32 ul_msg_len,
                                                         hmac_user_stru *pst_hmac_user)
{
    oal_uint8 *puc_vendor_vht_ie;
    oal_uint32 ul_vendor_vht_ie_offset = MAC_WLAN_OUI_VENDOR_VHT_HEADER + MAC_IE_HDR_LEN;
    oal_uint8 *puc_ie_tmp = OAL_PTR_NULL;

    puc_vendor_vht_ie = mac_find_vendor_ie_etc(MAC_WLAN_OUI_BROADCOM_EPIGRAM,
                                               MAC_WLAN_OUI_VENDOR_VHT_TYPE,
                                               puc_payload, ul_msg_len);

    if ((puc_vendor_vht_ie != OAL_PTR_NULL) && (puc_vendor_vht_ie[1] >= MAC_WLAN_OUI_VENDOR_VHT_HEADER)) {
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_update_sta_user::find broadcom/epigram vendor ie, enable hidden bit_11ac2g}");

        /* 进入此函数代表user支持2G 11ac */
        puc_ie_tmp = mac_find_ie_etc(MAC_EID_VHT_CAP,
                                     puc_vendor_vht_ie + ul_vendor_vht_ie_offset,
                                     (oal_int32)(puc_vendor_vht_ie[1] - MAC_WLAN_OUI_VENDOR_VHT_HEADER));
        if (puc_ie_tmp != OAL_PTR_NULL) {
            pst_hmac_user->en_user_vendor_vht_capable = OAL_TRUE;
            hmac_proc_vht_cap_ie_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user, puc_ie_tmp);
        }
        /* 表示支持5G 20M mcs9 */
        else {
            pst_hmac_user->en_user_vendor_novht_capable = OAL_TRUE;
        }
    }
}

OAL_STATIC oal_bool_enum hmac_ap_frame_len_check(oal_uint8 *puc_mac_hdr, oal_uint32 ul_msg_len)
{
    oal_uint32 ul_len_min;

    ul_len_min = ((WLAN_FC0_SUBTYPE_REASSOC_REQ | WLAN_FC0_TYPE_MGT) == mac_get_frame_type_and_subtype(puc_mac_hdr)) ?
        (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN + WLAN_MAC_ADDR_LEN) : (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN);

    return ((ul_msg_len > ul_len_min) ? OAL_TRUE : OAL_FALSE);
}

#ifdef _PRE_WLAN_FEATURE_11AC2G
OAL_STATIC oal_void hmac_ap_update_avail_protocol_mode(mac_vap_stru *pst_mac_vap,
    mac_user_stru *pst_mac_user, oal_uint8 *puc_avail_mode)
{
    if ((pst_mac_vap->en_protocol == WLAN_HT_MODE) && (pst_mac_user->en_protocol_mode == WLAN_VHT_MODE) &&
        (pst_mac_vap->st_cap_flag.bit_11ac2g == OAL_TRUE) && (pst_mac_vap->st_channel.en_band == WLAN_BAND_2G)) {
        *puc_avail_mode = WLAN_VHT_MODE;
    }
}
#endif


OAL_STATIC oal_uint32 hmac_ap_up_update_sta_user(hmac_vap_stru *pst_hmac_vap, oal_uint8 *puc_mac_hdr,
    oal_uint8 *puc_payload, oal_uint32 ul_msg_len, hmac_user_stru *pst_hmac_user,
    mac_status_code_enum_uint16 *pen_status_code)
{
    oal_uint16 us_cap_info;
    mac_status_code_enum_uint16 us_ret_val;
    oal_uint8 *puc_ie_tmp = OAL_PTR_NULL;
    wlan_bw_cap_enum_uint8 en_bandwidth_cap = WLAN_BW_CAP_BUTT;
    wlan_bw_cap_enum_uint8 en_bwcap_ap; /* ap自身带宽能力 */
    oal_uint32 ul_ret;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    mac_user_stru *pst_mac_user = OAL_PTR_NULL;
    wlan_bw_cap_enum_uint8 en_bwcap_vap;
    oal_uint8 uc_avail_mode;
    oal_uint32 ul_remain_len = 0;

    *pen_status_code = MAC_SUCCESSFUL_STATUSCODE;

    pst_mac_vap = &(pst_hmac_vap->st_vap_base_info);
    pst_mac_user = &(pst_hmac_user->st_user_base_info);

    if (OAL_TRUE != hmac_ap_frame_len_check(puc_mac_hdr, ul_msg_len)) {
        OAM_ERROR_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC, "{hmac_ap_up_update_sta_user::len[%d] err}", ul_msg_len);
        return OAL_FAIL;
    }

    /***************************************************************************
        检查AP是否支持当前正在关联的STA的所有能力
        |ESS|IBSS|CFPollable|CFPReq|Privacy|Preamble|PBCC|Agility|Reserved|
    ***************************************************************************/
    us_cap_info = OAL_MAKE_WORD16(puc_payload[0], puc_payload[1]);

    ul_ret = hmac_ap_up_update_sta_cap_info(pst_hmac_vap, us_cap_info, pst_hmac_user, pen_status_code);

    if (ul_ret != OAL_TRUE) {
        OAM_WARNING_LOG2(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC, "{hmac_ap_up_update_sta_user::\
            hmac_ap_up_update_sta_cap_info failed[%d], status_code=%d.}", ul_ret, *pen_status_code);
        return ul_ret;
    }

    puc_payload += (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN);
    ul_msg_len -= (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN);

    if ((WLAN_FC0_SUBTYPE_REASSOC_REQ | WLAN_FC0_TYPE_MGT) == mac_get_frame_type_and_subtype(puc_mac_hdr)) {
        puc_payload += WLAN_MAC_ADDR_LEN;
        ul_msg_len -= WLAN_MAC_ADDR_LEN;
    }

    /* 判断SSID,长度或内容不一致时,认为是SSID不一致，考虑兼容性找不到ie时不处理 */
    puc_ie_tmp = mac_find_ie_sec_etc(MAC_EID_SSID, puc_payload, (oal_int32)ul_msg_len, &ul_remain_len);
    if ((puc_ie_tmp != OAL_PTR_NULL) && (ul_remain_len > 2)) {
        if ((puc_ie_tmp[1] != (oal_uint8)OAL_STRLEN((oal_int8 *)mac_mib_get_DesiredSSID(pst_mac_vap))) ||
            (0 != oal_memcmp(&puc_ie_tmp[2], mac_mib_get_DesiredSSID(pst_mac_vap), puc_ie_tmp[1]))) {
            *pen_status_code = MAC_UNSPEC_FAIL;
            OAM_WARNING_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC, "{hmac_ap_up_update_sta_user::ssid mismatch.}");
            return OAL_FAIL;
        }
    }

    /* 根据wmm ie是否存在获取sta的wmm开关 */
    puc_ie_tmp = mac_find_ie_etc(MAC_EID_WMM, puc_payload, (oal_int32)ul_msg_len);
    pst_hmac_user->en_wmm_switch = (puc_ie_tmp != OAL_PTR_NULL) ? OAL_FALSE : OAL_TRUE;

    /* 当前用户已关联 */
    ul_ret = hmac_ie_proc_assoc_user_legacy_rate(puc_payload, ul_msg_len, pst_hmac_user);
    if (ul_ret != OAL_SUCC) {
        *pen_status_code = MAC_UNSUP_RATE;
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_update_sta_user::rates mismatch ret[%d].}", ul_ret);
        return ul_ret;
    }
    /* 按一定顺序重新排列速率 */
    hmac_user_sort_op_rates(pst_hmac_user);

    /* 更新对应STA的legacy协议能力 */
    hmac_ap_up_update_legacy_capability(pst_hmac_vap, pst_hmac_user, us_cap_info);

    /* 检查HT capability以及Extend capability是否匹配，并进行处理  */
    us_ret_val = hmac_vap_check_ht_capabilities_ap_etc(pst_hmac_vap, puc_payload, ul_msg_len, pst_hmac_user);
    if (us_ret_val != MAC_SUCCESSFUL_STATUSCODE) {
        *pen_status_code = us_ret_val;
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_update_sta_user::Reject STA because of ht_capability[%d].}", us_ret_val);
        return us_ret_val;
    }

    /* 更新AP中保护相关mib量 */
    ul_ret = hmac_user_protection_sync_data(pst_mac_vap);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_update_sta_user::protection update mib failed, ret=%d.}", ul_ret);
    }

    /* 更新对应STA的协议能力 update_asoc_entry_prot(ae, msa, rx_len, cap_info, is_p2p); */
    hmac_ap_up_update_asoc_entry_prot(puc_payload, mac_get_frame_type_and_subtype(puc_mac_hdr),
                                      ul_msg_len, pst_hmac_user);

    /* 更新QoS能力 */
    hmac_mgmt_update_assoc_user_qos_table_etc(puc_payload, (oal_uint16)ul_msg_len, pst_hmac_user);

#ifdef _PRE_WLAN_FEATURE_TXBF
    /* 更新11n txbf能力 */
    puc_ie_tmp = mac_find_vendor_ie_etc(MAC_HUAWEI_VENDER_IE, MAC_EID_11NTXBF, puc_payload, (oal_int32)ul_msg_len);
    hmac_mgmt_update_11ntxbf_cap_etc(puc_ie_tmp, pst_hmac_user);
#endif

#ifdef _PRE_WLAN_FEATURE_HIEX
    hmac_hiex_rx_assoc_req(pst_hmac_vap, pst_hmac_user, puc_payload, ul_msg_len);
#endif

    /* 更新11ac VHT capabilities ie */
    puc_ie_tmp = mac_find_ie_etc(MAC_EID_VHT_CAP, puc_payload, (oal_int32)ul_msg_len);
    if (puc_ie_tmp != OAL_PTR_NULL) {
        hmac_proc_vht_cap_ie_etc(pst_mac_vap, pst_hmac_user, puc_ie_tmp);
#ifdef _PRE_WLAN_FEATURE_1024QAM
        puc_ie_tmp = mac_find_vendor_ie_etc(MAC_HUAWEI_VENDER_IE, MAC_HISI_1024QAM_IE, puc_payload, ul_msg_len);
        if (puc_ie_tmp != OAL_PTR_NULL) {
            pst_hmac_user->st_user_base_info.st_cap_info.bit_1024qam_cap = OAL_TRUE;
            pst_hmac_user->st_user_base_info.st_cap_info.bit_20mmcs11_compatible_1103 = OAL_TRUE;
        }
#endif
    } else if (pst_mac_vap->en_protocol == WLAN_VHT_ONLY_MODE) {
        /* 不允许不支持11ac STA关联11aconly 模式的AP */
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_update_sta_user:AP 11ac only, but STA not support 11ac}");
        *pen_status_code = MAC_MISMATCH_VHTCAP;
        return OAL_FAIL;
    }
    if (pst_hmac_user->st_user_base_info.st_vht_hdl.en_vht_capable == OAL_FALSE) {
        hmac_ap_2g_11ac_user_asoc(pst_hmac_vap, puc_payload, ul_msg_len, pst_hmac_user);
    }
    /* 检查接收到的ASOC REQ消息中的SECURITY参数.如出错,则返回对应的错误码 */
    mac_user_init_key_etc(&pst_hmac_user->st_user_base_info);
    us_ret_val = hmac_check_rsn_ap(pst_mac_vap, &pst_hmac_user->st_user_base_info, puc_payload, ul_msg_len);
    if (us_ret_val != MAC_SUCCESSFUL_STATUSCODE) {
        *pen_status_code = us_ret_val;
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC, "{hmac_ap_up_update_sta_user::hmac_check_rsn_ap fail[%d].}",
                         us_ret_val);
        return OAL_FAIL;
    }

#ifdef _PRE_WLAN_FEATURE_11AX
    /* 更新11ax HE capabilities ie */
    puc_ie_tmp = mac_find_ie_ext_ie(MAC_EID_HE, MAC_EID_EXT_HE_CAP, puc_payload, (oal_int32)ul_msg_len);
    hmac_proc_he_cap_ie(pst_mac_vap, pst_hmac_user, puc_ie_tmp);
#endif
    hmac_user_cap_update_by_hisi_cap_ie(pst_hmac_user, puc_payload, ul_msg_len);

    /* 获取用户的协议模式 */
    hmac_set_user_protocol_mode_etc(pst_mac_vap, pst_hmac_user);

    uc_avail_mode = g_auc_avail_protocol_mode_etc[pst_mac_vap->en_protocol][pst_mac_user->en_protocol_mode];
    if (uc_avail_mode == WLAN_PROTOCOL_BUTT) {
        OAM_WARNING_LOG3(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC,
            "{hmac_ap_up_update_sta_user::user not allowed:no valid protocol:vap mode=%d,user mode=%d,user avail mode=%d.}",
            pst_mac_vap->en_protocol, pst_mac_user->en_protocol_mode, pst_mac_user->en_avail_protocol_mode);
        *pen_status_code = MAC_UNSUP_CAP;
        return OAL_FAIL;
    }

#ifdef _PRE_WLAN_FEATURE_11AC2G
    hmac_ap_update_avail_protocol_mode(pst_mac_vap, pst_mac_user, &uc_avail_mode);
#endif

    /* 获取用户与VAP协议模式交集 */
    mac_user_set_avail_protocol_mode_etc(pst_mac_user, uc_avail_mode);
    mac_user_set_cur_protocol_mode_etc(pst_mac_user, pst_mac_user->en_avail_protocol_mode);

    OAM_WARNING_LOG3(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC,
        "{hmac_ap_up_update_sta_user::mac_vap->en_protocol:%d,mac_user->en_protocol_mode:%d,en_avail_protocol_mode:%d}",
        pst_mac_vap->en_protocol, pst_mac_user->en_protocol_mode, pst_mac_user->en_avail_protocol_mode);

#ifdef _PRE_WLAN_FEATURE_11AX
    if (pst_hmac_user->st_user_base_info.en_cur_protocol_mode == WLAN_HE_MODE) {
        pst_hmac_user->uc_tx_ba_limit = DMAC_UCAST_FRAME_TX_COMP_TIMES_HE;
    }
#endif

    /* 获取用户和VAP 可支持的11a/b/g 速率交集 */
    hmac_vap_set_user_avail_rates_etc(pst_mac_vap, pst_hmac_user);

    ul_ret = hmac_config_user_cap_syn_etc(pst_mac_vap, pst_mac_user);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_mac_user->uc_vap_id, 0, "{hmac_ap_up_update_sta_user::usr_cap_syn fail:%d}", ul_ret);
    }

    /* 获取用户与VAP空间流交集 */
    ul_ret = hmac_user_set_avail_num_space_stream_etc(pst_mac_user, pst_mac_vap->en_vap_rx_nss);
    if (ul_ret != OAL_SUCC) {
        *pen_status_code = MAC_UNSPEC_FAIL;
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC, "{hmac_ap_up_update_sta_user::set nss fail:%d}", ul_ret);
    }

    /* 获取用户的带宽能力 */
    mac_user_get_sta_cap_bandwidth_etc(pst_mac_user, &en_bandwidth_cap);

    /* 获取vap带宽能力与用户带宽能力的交集 */
    mac_vap_get_bandwidth_cap_etc(&pst_hmac_vap->st_vap_base_info, &en_bwcap_ap);
    en_bwcap_vap = OAL_MIN(en_bwcap_ap, en_bandwidth_cap);
    mac_user_set_bandwidth_info_etc(pst_mac_user, en_bwcap_vap, en_bwcap_vap);

    OAM_WARNING_LOG3(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC, "{hmac_ap_up_update_sta_user::mac_vap->bandwidth:%d, \
        mac_user->bandwidth:%d, cur_bandwidth:%d.}", en_bwcap_ap, en_bandwidth_cap, pst_mac_user->en_cur_bandwidth);

    /* 获取用户160M带宽下的空间流 */
    hmac_user_set_num_spatial_stream_160M(pst_mac_user);

    /* 根据用户支持带宽能力，协商出当前带宽，dmac offload架构下，同步带宽信息到device */
    ul_ret = hmac_config_user_info_syn_etc(pst_mac_vap, pst_mac_user);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_mac_user->uc_vap_id, 0, "{hmac_ap_up_update_sta_user::usr_info_syn failed[%d].}", ul_ret);
    }

#ifdef _PRE_WLAN_FEATURE_SMPS
    /* 根据smps更新空间流能力 */
    if (!IS_VAP_SINGLE_NSS(pst_mac_vap) && !IS_USER_SINGLE_NSS(pst_mac_user)) {
        hmac_smps_update_user_status(pst_mac_vap, pst_mac_user);
    }
#endif

#ifdef _PRE_WLAN_FEATURE_OPMODE_NOTIFY
    /* 处理Operating Mode Notification 信息元素 */
    ul_ret = hmac_check_opmode_notify_etc(pst_hmac_vap, puc_mac_hdr, puc_payload, ul_msg_len, pst_hmac_user);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_update_sta_user::hmac_check_opmode_notify_etc failed[%d].}", ul_ret);
    }
#endif

    /* 同步空间流信息 */
    ul_ret = hmac_config_user_num_spatial_stream_cap_syn(pst_mac_vap, pst_mac_user);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ASSOC,
                       "{hmac_ap_up_update_sta_user::hmac_config_user_num_spatial_stream_cap_syn failed[%d].}", ul_ret);
    }

    return OAL_SUCC;
}


oal_uint32 hmac_ap_save_user_assoc_req(hmac_user_stru *pst_hmac_user, oal_uint8 *puc_payload,
                                       oal_uint32 ul_payload_len, oal_uint8 uc_mgmt_frm_type)
{
    oal_uint32 ul_ret;

    /* AP 保存STA 的关联请求帧信息，以备上报内核 */
    ul_ret = hmac_user_free_asoc_req_ie(pst_hmac_user->st_user_base_info.us_assoc_id);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG0(0, OAM_SF_ASSOC, "{hmac_ap_save_user_assoc_req :: hmac_user_free_asoc_req_ie fail.}");
        return OAL_FAIL;
    }

    /* 目前11r没有实现，所以处理重关联帧的流程和关联帧一样，11r实现后此处需要修改 */
    return hmac_user_set_asoc_req_ie(pst_hmac_user,
                                     puc_payload + MAC_CAP_INFO_LEN + MAC_LIS_INTERVAL_IE_LEN,
                                     ul_payload_len - MAC_CAP_INFO_LEN - MAC_LIS_INTERVAL_IE_LEN,
                                     (oal_uint8)(uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_REASSOC_REQ));
}


oal_uint32 hmac_ap_send_assoc_rsp(hmac_vap_stru *pst_hmac_vap,
                                  hmac_user_stru *pst_hmac_user,
                                  const unsigned char *puc_sta_addr,
                                  oal_uint8 uc_mgmt_frm_type,
                                  mac_status_code_enum_uint16 *pen_status_code)
{
    oal_uint32 ul_rslt;
    oal_netbuf_stru *pst_asoc_rsp;
    oal_uint32 ul_asoc_rsp_len;
    mac_tx_ctl_stru *pst_tx_ctl = OAL_PTR_NULL;
    oal_net_device_stru *pst_net_device = OAL_PTR_NULL;
    oal_uint8 uc_tx_mgmt_frm_type = WLAN_FC0_SUBTYPE_ASSOC_RSP;

    pst_asoc_rsp = (oal_netbuf_stru *)OAL_MEM_NETBUF_ALLOC(OAL_NORMAL_NETBUF,
                                                           WLAN_MEM_NETBUF_SIZE2,
                                                           OAL_NETBUF_PRIORITY_MID);
    if (pst_asoc_rsp == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                       "{hmac_ap_up_rx_asoc_req::pst_asoc_rsp null.}");
        /* 异常返回之前删除user */
        hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);

        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }
    pst_tx_ctl = (mac_tx_ctl_stru *)oal_netbuf_cb(pst_asoc_rsp);
    memset_s(pst_tx_ctl, OAL_NETBUF_CB_SIZE(), 0, OAL_NETBUF_CB_SIZE());

    OAL_MEM_NETBUF_TRACE(pst_asoc_rsp, OAL_TRUE);

    /* update tx mgmt frame type according to assoc req type */
    if (uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_ASSOC_REQ) {
        uc_tx_mgmt_frm_type = WLAN_FC0_SUBTYPE_ASSOC_RSP;
    } else if (uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_REASSOC_REQ) {
        uc_tx_mgmt_frm_type = WLAN_FC0_SUBTYPE_REASSOC_RSP;
    }
    ul_asoc_rsp_len = hmac_mgmt_encap_asoc_rsp_ap_etc(&(pst_hmac_vap->st_vap_base_info),
                                                      puc_sta_addr,
                                                      pst_hmac_user->st_user_base_info.us_assoc_id,
                                                      *pen_status_code,
                                                      OAL_NETBUF_HEADER(pst_asoc_rsp),
                                                      uc_tx_mgmt_frm_type);
    if (ul_asoc_rsp_len == 0) {
        OAM_WARNING_LOG0(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_rx_asoc_req::hmac_mgmt_encap_asoc_rsp_ap_etc encap msg fail.}");
        oal_netbuf_free(pst_asoc_rsp);

        /* 异常返回之前删除user */
        hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);

        return OAL_FAIL;
    }

    oal_netbuf_put(pst_asoc_rsp, ul_asoc_rsp_len);

    MAC_GET_CB_TX_USER_IDX(pst_tx_ctl) = pst_hmac_user->st_user_base_info.us_assoc_id;
    MAC_GET_CB_MPDU_LEN(pst_tx_ctl) = (oal_uint16)ul_asoc_rsp_len;

    /* 发送关联响应帧之前，将用户的节能状态复位 */
    hmac_mgmt_reset_psm_etc(&pst_hmac_vap->st_vap_base_info, MAC_GET_CB_TX_USER_IDX(pst_tx_ctl));

    /* 判断当前状态，如果用户已经关联成功则向上报用户离开信息 */
    if (pst_hmac_user->st_user_base_info.en_user_asoc_state == MAC_USER_STATE_ASSOC) {
        pst_net_device = hmac_vap_get_net_device_etc(pst_hmac_vap->st_vap_base_info.uc_vap_id);
        if (pst_net_device != OAL_PTR_NULL) {
#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#if defined(_PRE_PRODUCT_ID_HI110X_HOST)
            oal_kobject_uevent_env_sta_leave_etc(pst_net_device, puc_sta_addr);
#endif
#endif
        }
    }

    if ((*pen_status_code != MAC_SUCCESSFUL_STATUSCODE) &&
        (*pen_status_code != MAC_REJECT_TEMP)) {
        hmac_user_set_asoc_state_etc(&(pst_hmac_vap->st_vap_base_info),
                                     &pst_hmac_user->st_user_base_info,
                                     MAC_USER_STATE_AUTH_COMPLETE);
    }

    ul_rslt = hmac_tx_mgmt_send_event_etc(&(pst_hmac_vap->st_vap_base_info), pst_asoc_rsp, (oal_uint16)ul_asoc_rsp_len);
    if (ul_rslt != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_rx_asoc_req::hmac_tx_mgmt_send_event_etc failed[%d].}", ul_rslt);
        oal_netbuf_free(pst_asoc_rsp);

        /* 异常返回之前删除user */
        hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);

        return ul_rslt;
    }

    if (*pen_status_code == MAC_SUCCESSFUL_STATUSCODE) {
        /* AP检测STA成功，允许其关联成功 */
#ifdef _PRE_DEBUG_MODE_USER_TRACK
        mac_user_change_info_event(pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                                   pst_hmac_vap->st_vap_base_info.uc_vap_id,
                                   pst_hmac_user->st_user_base_info.en_user_asoc_state,
                                   MAC_USER_STATE_ASSOC, OAM_MODULE_ID_HMAC,
                                   OAM_USER_INFO_CHANGE_TYPE_ASSOC_STATE);
#endif

        ul_rslt = hmac_config_user_rate_info_syn_etc(&(pst_hmac_vap->st_vap_base_info),
                                                     &pst_hmac_user->st_user_base_info);
        if (ul_rslt != OAL_SUCC) {
            OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                           "{hmac_sta_wait_asoc_rx_etc::hmac_config_user_rate_info_syn_etc failed[%d].}", ul_rslt);
        }

        /*  user已经关联上，抛事件给DMAC，在DMAC层挂用户算法钩子 */
        hmac_user_add_notify_alg_etc((&pst_hmac_vap->st_vap_base_info), pst_hmac_user->st_user_base_info.us_assoc_id);
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_ap_up_rx_asoc_req(hmac_vap_stru *pst_hmac_vap,
                                             oal_uint8 uc_mgmt_frm_type,
                                             oal_uint8 *puc_mac_hdr,
                                             oal_uint32 ul_mac_hdr_len,
                                             oal_uint8 *puc_payload,
                                             oal_uint32 ul_payload_len)
{
    oal_uint32 ul_rslt;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    oal_uint16 us_user_idx = 0;
    mac_status_code_enum_uint16 en_status_code;
    oal_uint8 auc_sta_addr[WLAN_MAC_ADDR_LEN];
    mac_cfg_user_info_param_stru st_hmac_user_info_event;

#ifdef _PRE_WLAN_FEATURE_P2P
    oal_int32 l_len;
#endif

#ifdef _PRE_WLAN_FEATURE_VIRTUAL_MULTI_STA
    oal_int32 l_not_ie_len;
#endif

    oal_uint8 uc_frm_least_len;

    uc_frm_least_len = MAC_CAP_INFO_LEN + MAC_LIS_INTERVAL_IE_LEN;
    uc_frm_least_len += (WLAN_FC0_SUBTYPE_REASSOC_REQ == uc_mgmt_frm_type) ? WLAN_MAC_ADDR_LEN : 0;

    if (ul_payload_len <= uc_frm_least_len) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "hmac_ap_up_rx_asoc_req:assoc req len[%d] too short!",
                         ul_payload_len);
        return OAL_FAIL;
    }

    mac_get_address2(puc_mac_hdr, auc_sta_addr, WLAN_MAC_ADDR_LEN);

    ul_rslt = mac_vap_find_user_by_macaddr_etc(&(pst_hmac_vap->st_vap_base_info), auc_sta_addr, &us_user_idx);
    if (ul_rslt != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_rx_asoc_req::mac_vap_find_user_by_macaddr_etc failed[%d].}", ul_rslt);
        OAM_WARNING_LOG4(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_rx_asoc_req::user mac:%02X:XX:XX:%02X:%02X:%02X.}",
                         auc_sta_addr[0], auc_sta_addr[3], auc_sta_addr[4], auc_sta_addr[5]);
        hmac_mgmt_send_deauth_frame_etc(&(pst_hmac_vap->st_vap_base_info), auc_sta_addr, MAC_ASOC_NOT_AUTH, OAL_FALSE);

        return ul_rslt;
    }

    pst_hmac_user = (hmac_user_stru *)mac_res_get_hmac_user_etc(us_user_idx);

    if (pst_hmac_user == OAL_PTR_NULL) {
        OAM_ERROR_LOG1(0, OAM_SF_ASSOC,
                       "{hmac_ap_up_rx_asoc_req::pst_hmac_user[%d] null.}", us_user_idx);

        /* 没有查到对应的USER,发送去认证消息 */
        hmac_mgmt_send_deauth_frame_etc(&(pst_hmac_vap->st_vap_base_info), auc_sta_addr, MAC_ASOC_NOT_AUTH, OAL_FALSE);

        return OAL_ERR_CODE_PTR_NULL;
    }

    if (pst_hmac_user->st_mgmt_timer.en_is_registerd == OAL_TRUE) {
        FRW_TIMER_IMMEDIATE_DESTROY_TIMER(&pst_hmac_user->st_mgmt_timer);
    }

    en_status_code = MAC_SUCCESSFUL_STATUSCODE;

    /* 是否符合触发SA query流程的条件 */
#if (_PRE_WLAN_FEATURE_PMF != _PRE_PMF_NOT_SUPPORT)
    if ((pst_hmac_user->st_user_base_info.en_user_asoc_state == MAC_USER_STATE_ASSOC) &&
        (pst_hmac_user->st_user_base_info.st_cap_info.bit_pmf_active == OAL_TRUE)) {
        ul_rslt = hmac_start_sa_query_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user,
                                          pst_hmac_user->st_user_base_info.st_cap_info.bit_pmf_active);
        if (ul_rslt != OAL_SUCC) {
            OAM_ERROR_LOG1(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                           "{hmac_ap_up_rx_asoc_req::hmac_start_sa_query_etc failed[%d].}", ul_rslt);
            return ul_rslt;
        }
        OAM_INFO_LOG0(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                      "{hmac_ap_up_rx_asoc_req::set en_status_code is MAC_REJECT_TEMP.}");
        en_status_code = MAC_REJECT_TEMP;
    }
#endif

    if (en_status_code != MAC_REJECT_TEMP) {
        /* 当可以查找到用户时,说明当前USER的状态为已关联或已认证完成 处理用户相关信息以及能力交互 */
        OAM_WARNING_LOG4(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_CONN,
                         "{hmac_ap_up_rx_asoc_req:: ASSOC_REQ rx : user mac:%02X:XX:XX:%02X:%02X:%02X.}",
                         auc_sta_addr[0], auc_sta_addr[3], auc_sta_addr[4], auc_sta_addr[5]);

        ul_rslt = hmac_ap_up_update_sta_user(pst_hmac_vap, puc_mac_hdr,
                                             puc_payload, ul_payload_len,
                                             pst_hmac_user, &en_status_code);
        if (en_status_code != MAC_SUCCESSFUL_STATUSCODE) {
            OAM_WARNING_LOG1(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                             "{hmac_ap_up_rx_asoc_req::hmac_ap_up_update_sta_user failed[%d].}", en_status_code);
#ifdef _PRE_DEBUG_MODE_USER_TRACK
            mac_user_change_info_event(pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                                       pst_hmac_vap->st_vap_base_info.uc_vap_id,
                                       pst_hmac_user->st_user_base_info.en_user_asoc_state,
                                       MAC_USER_STATE_AUTH_COMPLETE, OAM_MODULE_ID_HMAC,
                                       OAM_USER_INFO_CHANGE_TYPE_ASSOC_STATE);
#endif
            hmac_user_set_asoc_state_etc(&(pst_hmac_vap->st_vap_base_info),
                                         &pst_hmac_user->st_user_base_info,
                                         MAC_USER_STATE_AUTH_COMPLETE);
        }

        /* check OWE capability in assoc req frame */
        l_len = ((uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_REASSOC_REQ) ?
            (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN + WLAN_MAC_ADDR_LEN) : (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN));
        hmac_ap_check_owe_capa(pst_hmac_vap, pst_hmac_user,
                               (puc_payload + l_len), (ul_payload_len - l_len), &en_status_code);

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
        /* 同步ap带宽，能力等信息到dmac */
        hmac_chan_sync_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_vap->st_vap_base_info.st_channel.uc_chan_number,
                           MAC_VAP_GET_CAP_BW(&pst_hmac_vap->st_vap_base_info), OAL_FALSE);

#endif

        /* 根据用户支持带宽能力，协商出当前带宽，dmac offload架构下，同步带宽信息到device */
        ul_rslt = hmac_config_user_info_syn_etc(&(pst_hmac_vap->st_vap_base_info), &pst_hmac_user->st_user_base_info);
        if (ul_rslt != OAL_SUCC) {
            OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                           "{hmac_ap_up_rx_asoc_req::usr_info_syn failed[%d].}", ul_rslt);
        }

        if (en_status_code == MAC_SUCCESSFUL_STATUSCODE) {
            ul_rslt = hmac_init_security_etc(&(pst_hmac_vap->st_vap_base_info), auc_sta_addr, sizeof(auc_sta_addr));
            if (ul_rslt != OAL_SUCC) {
                OAM_ERROR_LOG2(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                               "{hmac_ap_up_rx_asoc_req::hmac_init_security_etc failed[%d] status_code[%d].}",
                               ul_rslt, MAC_UNSPEC_FAIL);
                en_status_code = MAC_UNSPEC_FAIL;
            }

            ul_rslt = hmac_init_user_security_port_etc(&(pst_hmac_vap->st_vap_base_info),
                                                       &(pst_hmac_user->st_user_base_info));
            if (ul_rslt != OAL_SUCC) {
                OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                               "{hmac_ap_up_rx_asoc_req::hmac_init_user_security_port_etc failed[%d].}", ul_rslt);
            }
        }

        if ((ul_rslt != OAL_SUCC)
            || (en_status_code != MAC_SUCCESSFUL_STATUSCODE)) {
            OAM_WARNING_LOG2(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_CFG,
                             "{hmac_ap_up_rx_asoc_req::hmac_ap_up_update_sta_user fail rslt[%d] status_code[%d].",
                             ul_rslt, en_status_code);
#ifdef _PRE_DEBUG_MODE_USER_TRACK
            mac_user_change_info_event(pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                                       pst_hmac_vap->st_vap_base_info.uc_vap_id,
                                       pst_hmac_user->st_user_base_info.en_user_asoc_state,
                                       MAC_USER_STATE_AUTH_COMPLETE, OAM_MODULE_ID_HMAC,
                                       OAM_USER_INFO_CHANGE_TYPE_ASSOC_STATE);
#endif
            hmac_user_set_asoc_state_etc(&(pst_hmac_vap->st_vap_base_info),
                                         &pst_hmac_user->st_user_base_info,
                                         MAC_USER_STATE_AUTH_COMPLETE);
        }
#ifdef _PRE_WLAN_FEATURE_P2P
        l_len = ((uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_REASSOC_REQ) ?
            (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN + WLAN_MAC_ADDR_LEN) : (MAC_CAP_INFO_LEN + MAC_LISTEN_INT_LEN));
        if (IS_P2P_GO(&pst_hmac_vap->st_vap_base_info) &&
            (OAL_PTR_NULL == mac_find_vendor_ie_etc(MAC_WLAN_OUI_WFA, MAC_WLAN_OUI_TYPE_WFA_P2P,
                                                    puc_payload + l_len, (oal_int32)ul_payload_len - l_len))) {
            OAM_INFO_LOG1(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                          "{hmac_ap_up_rx_asoc_req::GO got assoc request from legacy device, length = [%d]}",
                          ul_payload_len);
            hmac_disable_p2p_pm_etc(pst_hmac_vap);
        }
#endif

#ifdef _PRE_WLAN_FEATURE_VIRTUAL_MULTI_STA
        /* 调用check接口校验用户是否支持4地址 是则将用户的4地址传输能力位置1 */
        if (pst_hmac_vap->st_wds_table.en_wds_vap_mode == WDS_MODE_ROOTAP) {
            if (hmac_vmsta_check_user_a4_support(puc_mac_hdr, ul_mac_hdr_len + ul_payload_len)) {
                pst_hmac_user->uc_is_wds = OAL_TRUE;
                OAM_WARNING_LOG0(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_ASSOC,
                                 "{hmac_ap_up_rx_asoc_req::user surpport 4 address.}");
            }
        }
#endif  // _PRE_WLAN_FEATURE_VIRTUAL_MULTI_STA
    }

    if (en_status_code == MAC_SUCCESSFUL_STATUSCODE) {
        hmac_user_set_asoc_state_etc(&(pst_hmac_vap->st_vap_base_info),
                                     &pst_hmac_user->st_user_base_info,
                                     MAC_USER_STATE_ASSOC);
    }

    if (pst_hmac_user->st_hmac_cap_info.bit_owe == OAL_TRUE) {
        /* APUT received OWE assoc req && check valid OWE DH Parameter Element, report assoc req to hostapd;
         * otherwise, APUT send assoc rsp frame with reject status code */
        if (en_status_code == MAC_SUCCESSFUL_STATUSCODE) {
            /* AP 保存STA 的关联请求帧信息，以备上报内核 */
            hmac_ap_save_user_assoc_req(pst_hmac_user, puc_payload, ul_payload_len, uc_mgmt_frm_type);

            /* 上报WAL层(WAL上报内核) AP关联上了一个新的STA */
            hmac_handle_connect_rsp_ap(pst_hmac_vap, pst_hmac_user);
            OAM_WARNING_LOG4(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_CONN,
                "{hmac_ap_up_rx_asoc_req:: new OWE STA assoc AP! STA_indx=%d. user mac:XX:XX:XX:%02X:%02X:%02X.}",
                us_user_idx, auc_sta_addr[3], auc_sta_addr[4], auc_sta_addr[5]);
        } else {
            hmac_ap_send_assoc_rsp(pst_hmac_vap, pst_hmac_user, auc_sta_addr, uc_mgmt_frm_type, &en_status_code);

            hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);
        }
    } else {
        hmac_ap_send_assoc_rsp(pst_hmac_vap, pst_hmac_user, auc_sta_addr, uc_mgmt_frm_type, &en_status_code);

        if (en_status_code == MAC_SUCCESSFUL_STATUSCODE) {
            /* AP 保存STA 的关联请求帧信息，以备上报内核 */
            hmac_ap_save_user_assoc_req(pst_hmac_user, puc_payload, ul_payload_len, uc_mgmt_frm_type);

            /* 上报WAL层(WAL上报内核) AP关联上了一个新的STA */
            hmac_handle_connect_rsp_ap(pst_hmac_vap, pst_hmac_user);
            OAM_WARNING_LOG4(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_CONN,
                "{hmac_ap_up_rx_asoc_req::ASSOC_RSP tx,STA assoc AP SUCC! STA_idx=%d.user mac:XX:XX:XX:%02X:%02X:%02X}",
                us_user_idx, auc_sta_addr[3], auc_sta_addr[4], auc_sta_addr[5]);
        } else {
            /* AP检测STA失败，将其删除 */
            if (en_status_code != MAC_REJECT_TEMP) {
                if (pst_hmac_vap->st_vap_base_info.pst_mib_info->st_wlan_mib_privacy.en_dot11RSNAMFPC == OAL_FALSE) {
                    if (en_status_code == MAC_INVALID_AKMP_CIPHER) {
                        hmac_mgmt_send_deauth_frame_etc(&pst_hmac_vap->st_vap_base_info,
                                                        pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                                                        MAC_AUTH_NOT_VALID, OAL_FALSE);
                    }
                    hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);
                } else {
                    hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);
                }
            }
        }
    }

    /* STA 入网后，上报VAP 信息和用户信息 */
    st_hmac_user_info_event.us_user_idx = us_user_idx;

    hmac_config_vap_info_etc(&(pst_hmac_vap->st_vap_base_info), OAL_SIZEOF(oal_uint32), (oal_uint8 *)&ul_rslt);
    hmac_config_user_info_etc(&(pst_hmac_vap->st_vap_base_info),
                              OAL_SIZEOF(mac_cfg_user_info_param_stru),
                              (oal_uint8 *)&st_hmac_user_info_event);

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_ap_up_rx_disasoc(hmac_vap_stru *pst_hmac_vap,
                                            oal_uint8 *puc_mac_hdr,
                                            oal_uint32 ul_mac_hdr_len,
                                            oal_uint8 *puc_payload,
                                            oal_uint32 ul_payload_len,
                                            oal_bool_enum_uint8 en_is_protected)
{
    oal_uint32 ul_ret;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    oal_uint8 *puc_da = OAL_PTR_NULL;
    oal_uint8 *puc_sa = OAL_PTR_NULL;
    oal_uint8 auc_sta_addr[WLAN_MAC_ADDR_LEN];

    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_AUTH, "{hmac_ap_up_rx_disasoc::pst_hmac_vap null}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    if (ul_payload_len < MAC_80211_REASON_CODE_LEN) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_CONN,
                         "{hmac_ap_up_rx_disasoc::invalid ul_payload_len len [%d]}", ul_payload_len);
        return OAL_FAIL;
    }

    mac_get_address2(puc_mac_hdr, auc_sta_addr, WLAN_MAC_ADDR_LEN);

    /* 增加接收到去关联帧时的维测信息 */
    mac_rx_get_sa((mac_ieee80211_frame_stru *)puc_mac_hdr, &puc_sa);

    OAM_WARNING_LOG4(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_CONN,
        "{hmac_ap_up_rx_disasoc::DISASSOC rx, Because of err_code[%d], received disassoc frame from source addr %02X:XX:XX:XX:%02X:%02X.}",
        *((oal_uint16 *)(puc_mac_hdr + MAC_80211_FRAME_LEN)), puc_sa[0], puc_sa[4], puc_sa[5]);

    pst_hmac_user = mac_vap_get_hmac_user_by_addr_etc(&(pst_hmac_vap->st_vap_base_info), &auc_sta_addr[0]);
    if (pst_hmac_user == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                         "{hmac_ap_up_rx_disasoc::pst_hmac_user null.}");
        /* 没有查到对应的USER,发送去认证消息 */
        hmac_mgmt_send_deauth_frame_etc(&(pst_hmac_vap->st_vap_base_info), auc_sta_addr, MAC_NOT_ASSOCED, OAL_FALSE);

        return OAL_ERR_CODE_PTR_NULL;
    }

    OAM_INFO_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                  "{hmac_ap_up_rx_disasoc::ap rx a disaasoc req.}");
    if (pst_hmac_user->st_user_base_info.en_user_asoc_state == MAC_USER_STATE_ASSOC) {
#if (_PRE_WLAN_FEATURE_PMF != _PRE_PMF_NOT_SUPPORT)
        /* 检查是否需要发送SA query request */
        if (OAL_SUCC == hmac_pmf_check_err_code_etc(&pst_hmac_user->st_user_base_info, en_is_protected, puc_mac_hdr)) {
            /* 在关联状态下收到未加密的ReasonCode 6/7需要启动SA Query流程 */
            ul_ret = hmac_start_sa_query_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user,
                pst_hmac_user->st_user_base_info.st_cap_info.bit_pmf_active);
            if (ul_ret != OAL_SUCC) {
                OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                               "{hmac_ap_up_rx_disasoc::hmac_start_sa_query_etc failed[%d].}", ul_ret);
                return OAL_ERR_CODE_PMF_SA_QUERY_START_FAIL;
            }
            return OAL_SUCC;
        }
#endif

        /* 如果该用户的管理帧加密属性不一致，丢弃该报文 */
        mac_rx_get_da((mac_ieee80211_frame_stru *)puc_mac_hdr, &puc_da);
        if ((OAL_TRUE != ETHER_IS_MULTICAST(puc_da)) &&
            (en_is_protected != pst_hmac_user->st_user_base_info.st_cap_info.bit_pmf_active)) {
            OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                           "{hmac_ap_up_rx_disasoc::PMF check failed,en_is_protected=%d.}", en_is_protected);
            return OAL_FAIL;
        }

#ifdef _PRE_DEBUG_MODE_USER_TRACK
        mac_user_change_info_event(pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                                   pst_hmac_vap->st_vap_base_info.uc_vap_id,
                                   pst_hmac_user->st_user_base_info.en_user_asoc_state,
                                   MAC_USER_STATE_AUTH_COMPLETE, OAM_MODULE_ID_HMAC,
                                   OAM_USER_INFO_CHANGE_TYPE_ASSOC_STATE);
#endif
        mac_user_set_asoc_state_etc(&pst_hmac_user->st_user_base_info, MAC_USER_STATE_AUTH_COMPLETE);

#ifdef _PRE_WLAN_1103_CHR
        CHR_EXCEPTION_REPORT(CHR_PLATFORM_EXCEPTION_EVENTID, CHR_SYSTEM_WIFI,
                             CHR_LAYER_DRV, CHR_WIFI_DRV_EVENT_SOFTAP_PASSIVE_DISCONNECT,
                             *((oal_uint16 *)(puc_mac_hdr + MAC_80211_FRAME_LEN)));
#endif

        /* 抛事件上报内核，已经去关联某个STA */
        hmac_handle_disconnect_rsp_ap_etc(pst_hmac_vap, pst_hmac_user);

        /* 有些网卡去关联时只发送DISASOC,也将删除其在AP内部的数据结构 */
        ul_ret = hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                             "{hmac_ap_up_rx_disasoc::hmac_user_del_etc failed[%d].}", ul_ret);
        }
    }

    return OAL_SUCC;
}


OAL_STATIC oal_void hmac_ap_up_rx_action_nonuser(hmac_vap_stru *pst_hmac_vap, oal_netbuf_stru *pst_netbuf)
{
    dmac_rx_ctl_stru *pst_rx_ctrl;
    oal_uint8 *puc_data;
    mac_ieee80211_frame_stru *pst_frame_hdr; /* 保存mac帧的指针 */

    pst_rx_ctrl = (dmac_rx_ctl_stru *)oal_netbuf_cb(pst_netbuf);

    /* 获取帧头信息 */
    pst_frame_hdr = (mac_ieee80211_frame_stru *)MAC_GET_RX_CB_MAC_HEADER_ADDR(&pst_rx_ctrl->st_rx_info);

    /* 获取帧体指针 */
    puc_data = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(&pst_rx_ctrl->st_rx_info) +
               pst_rx_ctrl->st_rx_info.uc_mac_header_len;

    /* Category */
    switch (puc_data[MAC_ACTION_OFFSET_CATEGORY]) {
        case MAC_ACTION_CATEGORY_PUBLIC: {
            /* Action */
            switch (puc_data[MAC_ACTION_OFFSET_ACTION]) {
                case MAC_PUB_VENDOR_SPECIFIC: {
#if defined(_PRE_WLAN_FEATURE_LOCATION) || defined(_PRE_WLAN_FEATURE_PSD_ANALYSIS)
                    if (0 == oal_memcmp(puc_data + MAC_ACTION_CATEGORY_AND_CODE_LEN, g_auc_huawei_oui, MAC_OUI_LEN)) {
                        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                                         "{hmac_ap_up_rx_action::hmac location get.}");
                        hmac_huawei_action_process(pst_hmac_vap, pst_netbuf,
                                                   puc_data[MAC_ACTION_CATEGORY_AND_CODE_LEN + MAC_OUI_LEN]);
                    }
#endif
                    break;
                }
                default:
                    break;
            }
        }
        break;

        default:
            break;
    }
    return;
}


OAL_STATIC oal_void hmac_ap_up_rx_action(hmac_vap_stru *pst_hmac_vap,
                                         oal_netbuf_stru *pst_netbuf,
                                         oal_bool_enum_uint8 en_is_protected)
{
    dmac_rx_ctl_stru *pst_rx_ctrl = OAL_PTR_NULL;
    oal_uint8 *puc_data = OAL_PTR_NULL;
    mac_ieee80211_frame_stru *pst_frame_hdr = OAL_PTR_NULL; /* 保存mac帧的指针 */
    hmac_user_stru *pst_hmac_user;

    pst_rx_ctrl = (dmac_rx_ctl_stru *)oal_netbuf_cb(pst_netbuf);

    /* 获取帧头信息 */
    pst_frame_hdr = (mac_ieee80211_frame_stru *)MAC_GET_RX_CB_MAC_HEADER_ADDR(&pst_rx_ctrl->st_rx_info);

    /* 获取发送端的用户指针 */
    pst_hmac_user = mac_vap_get_hmac_user_by_addr_etc(&pst_hmac_vap->st_vap_base_info, pst_frame_hdr->auc_address2);
    if (pst_hmac_user == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                         "{hmac_ap_up_rx_action::mac_vap_find_user_by_macaddr_etc failed.}");
        hmac_ap_up_rx_action_nonuser(pst_hmac_vap, pst_netbuf);
        return;
    }

    /* 获取帧体指针 */
    puc_data = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(&pst_rx_ctrl->st_rx_info) +
               pst_rx_ctrl->st_rx_info.uc_mac_header_len;

    /* Category */
    switch (puc_data[MAC_ACTION_OFFSET_CATEGORY]) {
        case MAC_ACTION_CATEGORY_BA: {
            switch (puc_data[MAC_ACTION_OFFSET_ACTION]) {
                case MAC_BA_ACTION_ADDBA_REQ:
                    hmac_mgmt_rx_addba_req_etc(pst_hmac_vap, pst_hmac_user, puc_data);
                    break;

                case MAC_BA_ACTION_ADDBA_RSP:
                    hmac_mgmt_rx_addba_rsp_etc(pst_hmac_vap, pst_hmac_user, puc_data);
                    break;

                case MAC_BA_ACTION_DELBA:
                    hmac_mgmt_rx_delba_etc(pst_hmac_vap, pst_hmac_user, puc_data);
                    break;

                default:
                    break;
            }
        }
        break;

        case MAC_ACTION_CATEGORY_PUBLIC: {
            /* Action */
            switch (puc_data[MAC_ACTION_OFFSET_ACTION]) {
                case MAC_PUB_VENDOR_SPECIFIC: {
#ifdef _PRE_WLAN_FEATURE_P2P
                    /* 查找OUI-OUI type值为 50 6F 9A - 09 (WFA P2P v1.0)  */
                    /* 并用hmac_rx_mgmt_send_to_host接口上报 */
                    if (OAL_TRUE == mac_ie_check_p2p_action_etc(puc_data + MAC_ACTION_CATEGORY_AND_CODE_LEN)) {
                        hmac_rx_mgmt_send_to_host_etc(pst_hmac_vap, pst_netbuf);
                    }
#endif /* _PRE_WLAN_FEATURE_P2P */
#if defined(_PRE_WLAN_FEATURE_LOCATION) || defined(_PRE_WLAN_FEATURE_PSD_ANALYSIS)
                    if (0 == oal_memcmp(puc_data + MAC_ACTION_CATEGORY_AND_CODE_LEN, g_auc_huawei_oui, MAC_OUI_LEN)) {
                        OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                                         "{hmac_ap_up_rx_action::hmac location get.}");
                        hmac_huawei_action_process(pst_hmac_vap, pst_netbuf,
                                                   puc_data[MAC_ACTION_CATEGORY_AND_CODE_LEN + MAC_OUI_LEN]);
                    }
#endif
                    break;
                }
                default:
                    break;
            }
        }
        break;

        case MAC_ACTION_CATEGORY_HT: {
            /* Action */
            switch (puc_data[MAC_ACTION_OFFSET_ACTION]) {
                case MAC_HT_ACTION_NOTIFY_CHANNEL_WIDTH:
                    break;

                case MAC_HT_ACTION_BUTT:
                default:
                    break;
            }
        }
        break;
#if (_PRE_WLAN_FEATURE_PMF != _PRE_PMF_NOT_SUPPORT)
        case MAC_ACTION_CATEGORY_SA_QUERY: {
            /* Action */
            switch (puc_data[MAC_ACTION_OFFSET_ACTION]) {
                case MAC_SA_QUERY_ACTION_REQUEST:
                    hmac_rx_sa_query_req_etc(pst_hmac_vap, pst_netbuf, en_is_protected);
                    break;
                case MAC_SA_QUERY_ACTION_RESPONSE:
                    hmac_rx_sa_query_rsp_etc(pst_hmac_vap, pst_netbuf, en_is_protected);
                    break;
                default:
                    break;
            }
        }
        break;
#endif
        case MAC_ACTION_CATEGORY_VHT: {
            switch (puc_data[MAC_ACTION_OFFSET_ACTION]) {
                case MAC_VHT_ACTION_BUTT:
                default:
                    break;
            }
        }
        break;

        case MAC_ACTION_CATEGORY_VENDOR: {
#ifdef _PRE_WLAN_FEATURE_P2P
            /* 查找OUI-OUI type值为 50 6F 9A - 09 (WFA P2P v1.0)  */
            /* 并用hmac_rx_mgmt_send_to_host接口上报 */
            if (OAL_TRUE == mac_ie_check_p2p_action_etc(puc_data + MAC_ACTION_CATEGORY_AND_CODE_LEN)) {
                hmac_rx_mgmt_send_to_host_etc(pst_hmac_vap, pst_netbuf);
            }
#endif
        }
        break;

#ifdef _PRE_WLAN_FEATURE_WMMAC
        case MAC_ACTION_CATEGORY_WMMAC_QOS: {
            if (g_en_wmmac_switch_etc == OAL_TRUE) {
                switch (puc_data[MAC_ACTION_OFFSET_ACTION]) {
                    case MAC_WMMAC_ACTION_ADDTS_REQ:
                        hmac_mgmt_rx_addts_req_frame_etc(pst_hmac_vap, pst_netbuf);
                        break;

                    default:
                        break;
                }
            }
        }
        break;
#endif  // _PRE_WLAN_FEATURE_WMMAC

        default:
            break;
    }
    return;
}


OAL_STATIC oal_void hmac_ap_up_rx_probe_req(hmac_vap_stru *pst_hmac_vap, oal_netbuf_stru *pst_netbuf)
{
    dmac_rx_ctl_stru *pst_rx_ctrl = OAL_PTR_NULL;
    mac_rx_ctl_stru *pst_rx_info = OAL_PTR_NULL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
    enum nl80211_band en_band;
#else
    enum ieee80211_band en_band;
#endif
    oal_int l_freq;

    pst_rx_ctrl = (dmac_rx_ctl_stru *)oal_netbuf_cb(pst_netbuf);
    pst_rx_info = (mac_rx_ctl_stru *)(&(pst_rx_ctrl->st_rx_info));

    /* 获取AP 当前信道 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
    if (WLAN_BAND_2G == pst_hmac_vap->st_vap_base_info.st_channel.en_band) {
        en_band = NL80211_BAND_2GHZ;
    } else if (WLAN_BAND_5G == pst_hmac_vap->st_vap_base_info.st_channel.en_band) {
        en_band = NL80211_BAND_5GHZ;
    } else {
        en_band = NUM_NL80211_BANDS;
    }
#else
    if (WLAN_BAND_2G == pst_hmac_vap->st_vap_base_info.st_channel.en_band) {
        en_band = IEEE80211_BAND_2GHZ;
    } else if (WLAN_BAND_5G == pst_hmac_vap->st_vap_base_info.st_channel.en_band) {
        en_band = IEEE80211_BAND_5GHZ;
    } else {
        en_band = IEEE80211_NUM_BANDS;
    }
#endif
    l_freq = oal_ieee80211_channel_to_frequency(pst_hmac_vap->st_vap_base_info.st_channel.uc_chan_number,
                                                en_band);

    /* 上报接收到的probe req 管理帧 */
    hmac_send_mgmt_to_host_etc(pst_hmac_vap, pst_netbuf, pst_rx_info->us_frame_len, l_freq);
}


oal_uint32 hmac_ap_up_rx_mgmt_etc(hmac_vap_stru *pst_hmac_vap, oal_void *p_param)
{
    dmac_wlan_crx_event_stru *pst_mgmt_rx_event = OAL_PTR_NULL;
    dmac_rx_ctl_stru *pst_rx_ctrl = OAL_PTR_NULL;
    mac_rx_ctl_stru *pst_rx_info = OAL_PTR_NULL;
    oal_uint8 *puc_mac_hdr = OAL_PTR_NULL;
    oal_uint8 *puc_payload = OAL_PTR_NULL;
    oal_uint32 ul_msg_len;     /* 消息总长度,不包括FCS */
    oal_uint32 ul_mac_hdr_len; /* MAC头长度 */
    oal_uint8 uc_mgmt_frm_type;
    oal_bool_enum_uint8 en_is_protected;

#if (_PRE_WLAN_FEATURE_BLACKLIST_LEVEL != _PRE_WLAN_FEATURE_BLACKLIST_NONE)
    oal_uint8 *puc_sa;
    oal_bool_enum_uint8 en_blacklist_result = OAL_FALSE;
#endif

    if (OAL_ANY_NULL_PTR2(pst_hmac_vap, p_param)) {
        OAM_ERROR_LOG2(0, OAM_SF_RX, "{hmac_ap_up_rx_mgmt_etc::param null, %x %x.}",
                       (uintptr_t)pst_hmac_vap, (uintptr_t)p_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_mgmt_rx_event = (dmac_wlan_crx_event_stru *)p_param;
    pst_rx_ctrl = (dmac_rx_ctl_stru *)oal_netbuf_cb(pst_mgmt_rx_event->pst_netbuf);
    pst_rx_info = (mac_rx_ctl_stru *)(&(pst_rx_ctrl->st_rx_info));
    puc_mac_hdr = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(pst_rx_info);
    ul_mac_hdr_len = pst_rx_info->uc_mac_header_len; /* MAC头长度 */
    puc_payload = (oal_uint8 *)(puc_mac_hdr) + ul_mac_hdr_len;
    ul_msg_len = pst_rx_info->us_frame_len; /* 消息总长度,不包括FCS */
    en_is_protected = (oal_uint8)mac_is_protectedframe(puc_mac_hdr);

    /* AP在UP状态下 接收到的各种管理帧处理 */
    uc_mgmt_frm_type = mac_get_frame_sub_type(puc_mac_hdr);

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#ifdef _PRE_WLAN_WAKEUP_SRC_PARSE
    if (OAL_TRUE == wlan_pm_wkup_src_debug_get()) {
        wlan_pm_wkup_src_debug_set(OAL_FALSE);
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                         "{wifi_wake_src:chmac_ap_up_rx_mgmt_etc::wakeup mgmt type[0x%x]}",
                         uc_mgmt_frm_type);
    }
#endif
#endif

    /* Bar frame proc here */
    if (WLAN_FC0_TYPE_CTL == mac_get_frame_type(puc_mac_hdr)) {
        uc_mgmt_frm_type = mac_get_frame_sub_type(puc_mac_hdr);
        if (WLAN_FC0_SUBTYPE_BAR == uc_mgmt_frm_type) {
            hmac_up_rx_bar_etc(pst_hmac_vap, pst_rx_ctrl, pst_mgmt_rx_event->pst_netbuf);
        }
    } else if (WLAN_FC0_TYPE_MGT == mac_get_frame_type(puc_mac_hdr)) {
#if (_PRE_WLAN_FEATURE_BLACKLIST_LEVEL != _PRE_WLAN_FEATURE_BLACKLIST_NONE)
        mac_rx_get_sa((mac_ieee80211_frame_stru *)puc_mac_hdr, &puc_sa);

        /* 自动加入黑名单检查 */
        if (OAL_VALUE_EQ_ANY2(uc_mgmt_frm_type, WLAN_FC0_SUBTYPE_ASSOC_REQ, WLAN_FC0_SUBTYPE_REASSOC_REQ)) {
            hmac_autoblacklist_filter_etc(&pst_hmac_vap->st_vap_base_info, puc_sa);
        }

        /* 黑名单过滤检查 */
        en_blacklist_result = hmac_blacklist_filter_etc(&pst_hmac_vap->st_vap_base_info, puc_sa);
        if ((en_blacklist_result == OAL_TRUE) && (WLAN_FC0_SUBTYPE_AUTH != uc_mgmt_frm_type)) {
            return OAL_SUCC;
        }
#endif

#ifdef _PRE_WLAN_FEATURE_SNIFFER
        proc_sniffer_write_file(NULL, 0, puc_mac_hdr, pst_rx_info->us_frame_len, 0);
#endif

        switch (uc_mgmt_frm_type) {
            case WLAN_FC0_SUBTYPE_AUTH:
                hmac_ap_rx_auth_req(pst_hmac_vap, pst_mgmt_rx_event->pst_netbuf);
                break;

            case WLAN_FC0_SUBTYPE_DEAUTH:
                if (ul_msg_len < ul_mac_hdr_len + MAC_80211_REASON_CODE_LEN) {
                    OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                                     "{hmac_ap_up_rx_mgmt_etc::invalid deauth_req length[%d]}", ul_msg_len);
                } else {
                    hmac_ap_rx_deauth_req(pst_hmac_vap, puc_mac_hdr, en_is_protected);
                }
                break;

            case WLAN_FC0_SUBTYPE_ASSOC_REQ:
            case WLAN_FC0_SUBTYPE_REASSOC_REQ:
                hmac_ap_up_rx_asoc_req(pst_hmac_vap, uc_mgmt_frm_type,
                                       puc_mac_hdr, ul_mac_hdr_len,
                                       puc_payload, (ul_msg_len - ul_mac_hdr_len));
                break;

            case WLAN_FC0_SUBTYPE_DISASSOC:
                hmac_ap_up_rx_disasoc(pst_hmac_vap, puc_mac_hdr,
                                      ul_mac_hdr_len, puc_payload,
                                      (ul_msg_len - ul_mac_hdr_len), en_is_protected);
                break;

            case WLAN_FC0_SUBTYPE_ACTION:
                hmac_ap_up_rx_action(pst_hmac_vap, pst_mgmt_rx_event->pst_netbuf, en_is_protected);
                break;

            case WLAN_FC0_SUBTYPE_PROBE_REQ:
                hmac_ap_up_rx_probe_req(pst_hmac_vap, pst_mgmt_rx_event->pst_netbuf);
                break;

            default:
                break;
        }
    }

    return OAL_SUCC;
}


oal_uint32 hmac_mgmt_timeout_ap_etc(oal_void *p_param)
{
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user;
    oal_uint32 ul_ret;

    pst_hmac_user = (hmac_user_stru *)p_param;
    if (pst_hmac_user == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_AUTH, "{hmac_mgmt_timeout_ap_etc::pst_hmac_user null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_hmac_vap = (hmac_vap_stru *)mac_res_get_hmac_vap(pst_hmac_user->st_user_base_info.uc_vap_id);

    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_AUTH,
                       "{hmac_mgmt_timeout_ap_etc::pst_hmac_vap null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    OAM_WARNING_LOG1(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_AUTH,
                     "{hmac_mgmt_timeout_ap_etc::Wait AUTH timeout!! After %d ms.}",
                     WLAN_AUTH_AP_TIMEOUT);
#ifdef _PRE_WLAN_1103_CHR
    CHR_EXCEPTION_REPORT(CHR_PLATFORM_EXCEPTION_EVENTID, CHR_SYSTEM_WIFI,
                         CHR_LAYER_DRV, CHR_WIFI_DRV_EVENT_SOFTAP_CONNECT,
                         MAC_AP_AUTH_RSP_TIMEOUT);
#endif
    /* 发送去关联帧消息给STA */
    hmac_mgmt_send_deauth_frame_etc(&pst_hmac_vap->st_vap_base_info,
                                    pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                                    MAC_AUTH_NOT_VALID, OAL_FALSE);

    /* 抛事件上报内核，已经去关联某个STA */
    hmac_handle_disconnect_rsp_ap_etc(pst_hmac_vap, pst_hmac_user);

    ul_ret = hmac_user_del_etc(&pst_hmac_vap->st_vap_base_info, pst_hmac_user);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_hmac_user->st_user_base_info.uc_vap_id, OAM_SF_AUTH,
                         "{hmac_mgmt_timeout_ap_etc::hmac_user_del_etc failed[%d].}", ul_ret);
    }

    return OAL_SUCC;
}


oal_uint32 hmac_ap_wait_start_misc_etc(hmac_vap_stru *pst_hmac_vap, oal_void *p_param)
{
    hmac_misc_input_stru *pst_misc_input = OAL_PTR_NULL;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_hmac_vap, p_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_RX, "{hmac_ap_wait_start_misc_etc::param null, %x %x.}",
                       (uintptr_t)pst_hmac_vap, (uintptr_t)p_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_misc_input = (hmac_misc_input_stru *)p_param;

    switch (pst_misc_input->en_type) {
        case HMAC_MISC_RADAR:
#ifdef _PRE_WLAN_FEATURE_DFS
            hmac_dfs_ap_wait_start_radar_handler_etc(pst_hmac_vap);
#endif
            break;

        default:
            break;
    }

    return OAL_SUCC;
}


oal_uint32 hmac_ap_up_misc_etc(hmac_vap_stru *pst_hmac_vap, oal_void *p_param)
{
    hmac_misc_input_stru *pst_misc_input = OAL_PTR_NULL;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_hmac_vap, p_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_RX, "{hmac_ap_up_misc_etc::param null, %x %x.}",
                       (uintptr_t)pst_hmac_vap, (uintptr_t)p_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_misc_input = (hmac_misc_input_stru *)p_param;

    switch (pst_misc_input->en_type) {
        case HMAC_MISC_RADAR:
#ifdef _PRE_WLAN_FEATURE_DFS
            hmac_dfs_ap_up_radar_handler_etc(pst_hmac_vap);
#endif
            break;

        default:
            break;
    }

    return OAL_SUCC;
}

oal_uint32 hmac_ap_clean_bss_etc(hmac_vap_stru *pst_hmac_vap)
{
    oal_dlist_head_stru *pst_entry = OAL_PTR_NULL;
    oal_dlist_head_stru *pst_next_entry = OAL_PTR_NULL;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    mac_user_stru *pst_user_tmp = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user_tmp = OAL_PTR_NULL;
    oal_bool_enum_uint8 en_is_protected;

    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_RX, "{hmac_ap_clean_bss_etc::hmac vap is null}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* 删除vap下所有已关联用户，并通知内核 */
    pst_mac_vap = &pst_hmac_vap->st_vap_base_info;
    OAL_DLIST_SEARCH_FOR_EACH_SAFE(pst_entry, pst_next_entry, &(pst_mac_vap->st_mac_user_list_head))
    {
        pst_user_tmp = OAL_DLIST_GET_ENTRY(pst_entry, mac_user_stru, st_user_dlist);
        if (pst_user_tmp == OAL_PTR_NULL) {
            continue;
        }

        pst_hmac_user_tmp = mac_res_get_hmac_user_etc(pst_user_tmp->us_assoc_id);
        if (pst_hmac_user_tmp == OAL_PTR_NULL) {
            continue;
        }

        /* 管理帧加密是否开启 */
        en_is_protected = pst_user_tmp->st_cap_info.bit_pmf_active;

        /* 发去关联帧 */
        hmac_mgmt_send_disassoc_frame_etc(pst_mac_vap, pst_user_tmp->auc_user_mac_addr,
                                          MAC_DISAS_LV_SS, en_is_protected);

        /* 通知内核 */
        hmac_handle_disconnect_rsp_ap_etc(pst_hmac_vap, pst_hmac_user_tmp);

        /* 删除用户 */
        hmac_user_del_etc(pst_mac_vap, pst_hmac_user_tmp);
    }

    return OAL_SUCC;
}
