

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#ifdef _PRE_WLAN_FEATURE_ROAM

/* 1 头文件包含 */
#include "oam_ext_if.h"
#include "mac_ie.h"
#include "mac_device.h"
#include "mac_resource.h"
#include "dmac_ext_if.h"
#include "hmac_fsm.h"
#include "hmac_sme_sta.h"
#include "hmac_resource.h"
#include "hmac_device.h"
#include "hmac_mgmt_sta.h"
#include "hmac_mgmt_bss_comm.h"
#include "hmac_encap_frame_sta.h"
#include "hmac_tx_amsdu.h"
#include "hmac_rx_data.h"
#include "hmac_chan_mgmt.h"
#include "hmac_11i.h"
#include "hmac_roam_main.h"
#include "hmac_roam_connect.h"
#include "hmac_roam_alg.h"
#include "hmac_user.h"
#include "hmac_dfx.h"
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE) && (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#include "plat_pm_wlan.h"
#endif
#include "securec.h"
#include "securectype.h"
#include "hmac_ht_self_cure.h"

#ifdef _PRE_WLAN_FEATURE_11K
#include "wal_config.h"
#include "wal_linux_ioctl.h"
#endif


#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_ROAM_CONNECT_C

/* 2 全局变量定义 */
OAL_STATIC hmac_roam_fsm_func g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_BUTT][ROAM_CONNECT_FSM_EVENT_TYPE_BUTT];

OAL_STATIC oal_uint32 hmac_roam_connect_null_fn(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_start_join(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_send_auth_seq1(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
#ifdef _PRE_WLAN_FEATURE_11R
OAL_STATIC oal_uint32 hmac_roam_send_ft_req(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_process_ft_rsp(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_process_ft_preauth_rsp(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
#endif  // _PRE_WLAN_FEATURE_11R
OAL_STATIC oal_uint32 hmac_roam_process_auth_seq2(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_process_assoc_rsp(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_process_action(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_connect_succ(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_connect_fail(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_auth_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_assoc_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_handshaking_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
#ifdef _PRE_WLAN_FEATURE_11R
OAL_STATIC oal_uint32 hmac_roam_ft_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
OAL_STATIC oal_uint32 hmac_roam_ft_preauth_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param);
#endif  // _PRE_WLAN_FEATURE_11R
OAL_STATIC oal_uint32 hmac_roam_send_reassoc_req(hmac_roam_info_stru *pst_roam_info);
#ifdef _PRE_WLAN_FEATURE_11K
OAL_STATIC oal_uint32  hmac_sta_up_send_neighbor_req(hmac_roam_info_stru *pst_roam_info);
#endif

/* 3 函数实现 */

oal_void hmac_roam_connect_fsm_init_etc(oal_void)
{
    oal_uint32 ul_state;
    oal_uint32 ul_event;

    for (ul_state = 0; ul_state < ROAM_CONNECT_STATE_BUTT; ul_state++) {
        for (ul_event = 0; ul_event < ROAM_CONNECT_FSM_EVENT_TYPE_BUTT; ul_event++) {
            g_hmac_roam_connect_fsm_func[ul_state][ul_event] = hmac_roam_connect_null_fn;
        }
    }
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_INIT][ROAM_CONNECT_FSM_EVENT_START] = hmac_roam_start_join;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_AUTH_COMP][ROAM_CONNECT_FSM_EVENT_MGMT_RX] =
        hmac_roam_process_auth_seq2;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_AUTH_COMP][ROAM_CONNECT_FSM_EVENT_TIMEOUT] =
        hmac_roam_auth_timeout;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_ASSOC_COMP][ROAM_CONNECT_FSM_EVENT_MGMT_RX] =
        hmac_roam_process_assoc_rsp;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_ASSOC_COMP][ROAM_CONNECT_FSM_EVENT_TIMEOUT] =
        hmac_roam_assoc_timeout;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_HANDSHAKING][ROAM_CONNECT_FSM_EVENT_MGMT_RX] =
        hmac_roam_process_action;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_HANDSHAKING][ROAM_CONNECT_FSM_EVENT_KEY_DONE] =
        hmac_roam_connect_succ;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_HANDSHAKING][ROAM_CONNECT_FSM_EVENT_TIMEOUT] =
        hmac_roam_handshaking_timeout;
#ifdef _PRE_WLAN_FEATURE_11R
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_INIT][ROAM_CONNECT_FSM_EVENT_FT_OVER_DS] = hmac_roam_send_ft_req;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_FT_COMP][ROAM_CONNECT_FSM_EVENT_MGMT_RX] =
        hmac_roam_process_ft_rsp;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_FT_COMP][ROAM_CONNECT_FSM_EVENT_TIMEOUT] =
        hmac_roam_ft_timeout;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_PREAUTH_COMP][ROAM_CONNECT_FSM_EVENT_MGMT_RX] =
        hmac_roam_process_ft_preauth_rsp;
    g_hmac_roam_connect_fsm_func[ROAM_CONNECT_STATE_WAIT_PREAUTH_COMP][ROAM_CONNECT_FSM_EVENT_TIMEOUT] =
        hmac_roam_ft_preauth_timeout;
#endif  // _PRE_WLAN_FEATURE_11R
}


oal_uint32 hmac_roam_connect_fsm_action_etc(hmac_roam_info_stru *pst_roam_info,
                                            roam_connect_fsm_event_type_enum en_event, oal_void *p_param)
{
    if (pst_roam_info == OAL_PTR_NULL) {
        return OAL_ERR_CODE_PTR_NULL;
    }

    if (pst_roam_info->st_connect.en_state >= ROAM_CONNECT_STATE_BUTT) {
        return OAL_ERR_CODE_ROAM_STATE_UNEXPECT;
    }

    if (en_event >= ROAM_CONNECT_FSM_EVENT_TYPE_BUTT) {
        return OAL_ERR_CODE_ROAM_EVENT_UXEXPECT;
    }

    return g_hmac_roam_connect_fsm_func[pst_roam_info->st_connect.en_state][en_event](pst_roam_info, p_param);
}


OAL_STATIC oal_void hmac_roam_connect_change_state(hmac_roam_info_stru *pst_roam_info,
                                                   roam_connect_state_enum_uint8 en_state)
{
    if (pst_roam_info != OAL_PTR_NULL) {
        OAM_WARNING_LOG2(0, OAM_SF_ROAM,
                         "{hmac_roam_connect_change_state::[%d]->[%d]}", pst_roam_info->st_connect.en_state, en_state);
        pst_roam_info->st_connect.en_state = en_state;
    }
}


OAL_STATIC oal_uint32 hmac_roam_connect_check_state(hmac_roam_info_stru *pst_roam_info,
                                                    mac_vap_state_enum_uint8 en_vap_state,
                                                    roam_main_state_enum_uint8 en_main_state,
                                                    roam_connect_state_enum_uint8 en_connect_state)
{
    if (pst_roam_info == OAL_PTR_NULL) {
        return OAL_ERR_CODE_PTR_NULL;
    }

    if (pst_roam_info->pst_hmac_vap == OAL_PTR_NULL) {
        return OAL_ERR_CODE_ROAM_INVALID_VAP;
    }

    if (pst_roam_info->pst_hmac_user == OAL_PTR_NULL) {
        return OAL_ERR_CODE_ROAM_INVALID_USER;
    }

    if (pst_roam_info->uc_enable == 0) {
        return OAL_ERR_CODE_ROAM_DISABLED;
    }

    if ((pst_roam_info->pst_hmac_vap->st_vap_base_info.en_vap_state != en_vap_state) ||
        (pst_roam_info->en_main_state != en_main_state) ||
        (pst_roam_info->st_connect.en_state != en_connect_state)) {
        OAM_WARNING_LOG3(0, OAM_SF_ROAM,
                         "{hmac_roam_connect_check_state::unexpect vap_state[%d] main_state[%d] connect_state[%d]!}",
                         pst_roam_info->pst_hmac_vap->st_vap_base_info.en_vap_state,
                         pst_roam_info->en_main_state, pst_roam_info->st_connect.en_state);
        return OAL_ERR_CODE_ROAM_INVALID_VAP_STATUS;
    }

    return OAL_SUCC;
}


oal_uint32 hmac_roam_connect_timeout_etc(oal_void *p_arg)
{
    hmac_roam_info_stru *pst_roam_info;

    pst_roam_info = (hmac_roam_info_stru *)p_arg;
    if (pst_roam_info == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_timeout_etc::p_arg is null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    OAM_WARNING_LOG2(0, OAM_SF_ROAM, "{hmac_roam_connect_timeout_etc::MAIN_STATE[%d] CONNECT_STATE[%d].}",
                     pst_roam_info->en_main_state, pst_roam_info->st_connect.en_state);

    return hmac_roam_connect_fsm_action_etc(pst_roam_info, ROAM_CONNECT_FSM_EVENT_TIMEOUT, OAL_PTR_NULL);
}

OAL_STATIC oal_uint32 hmac_roam_connect_null_fn(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_null_fn .}");

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#ifdef _PRE_WLAN_WAKEUP_SRC_PARSE
    if (OAL_TRUE == wlan_pm_wkup_src_debug_get()) {
        wlan_pm_wkup_src_debug_set(OAL_FALSE);
        OAM_WARNING_LOG0(0, OAM_SF_RX, "{wifi_wake_src:hmac_roam_connect_null_fn::rcv mgmt frame,drop it}");
    }
#endif
#endif

    return OAL_SUCC;
}

OAL_STATIC oal_void hmac_roam_connect_pm_wkup(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#ifdef _PRE_WLAN_WAKEUP_SRC_PARSE
    if (OAL_TRUE == wlan_pm_wkup_src_debug_get()) {
        wlan_pm_wkup_src_debug_set(OAL_FALSE);
        OAM_WARNING_LOG0(0, OAM_SF_RX, "{wifi_wake_src:hmac_roam_connect_pm_wkup::rcv mgmt frame}");
    }
#endif
#endif
}


OAL_STATIC oal_void hmac_roam_connect_start_timer(hmac_roam_info_stru *pst_roam_info, oal_uint32 ul_timeout)
{
    frw_timeout_stru *pst_timer = &(pst_roam_info->st_connect.st_timer);

    OAM_INFO_LOG1(0, OAM_SF_ROAM, "{hmac_roam_connect_start_timer [%d].}", ul_timeout);

    /* 启动认证超时定时器 */
    FRW_TIMER_CREATE_TIMER(pst_timer,
                           hmac_roam_connect_timeout_etc,
                           ul_timeout,
                           pst_roam_info,
                           OAL_FALSE,
                           OAM_MODULE_ID_HMAC,
                           pst_roam_info->pst_hmac_vap->st_vap_base_info.ul_core_id);
}


OAL_STATIC oal_uint32 hmac_roam_connect_del_timer(hmac_roam_info_stru *pst_roam_info)
{
    FRW_TIMER_IMMEDIATE_DESTROY_TIMER(&(pst_roam_info->st_connect.st_timer));
    return OAL_SUCC;
}


oal_uint32 hmac_roam_connect_set_join_reg_etc(mac_vap_stru *pst_mac_vap, hmac_user_stru *pst_hmac_user)
{
    frw_event_mem_stru *pst_event_mem = OAL_PTR_NULL;
    frw_event_stru *pst_event = OAL_PTR_NULL;
    dmac_ctx_join_req_set_reg_stru *pst_reg_params = OAL_PTR_NULL;
    hmac_join_req_stru st_join_req;

    memset_s(&st_join_req, OAL_SIZEOF(hmac_join_req_stru), 0, OAL_SIZEOF(hmac_join_req_stru));

    /* 抛事件DMAC_WLAN_CTX_EVENT_SUB_TYPE_JOIN_SET_REG到DMAC */
    pst_event_mem = FRW_EVENT_ALLOC(OAL_SIZEOF(dmac_ctx_join_req_set_reg_stru));
    if (pst_event_mem == OAL_PTR_NULL) {
        OAM_ERROR_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_connect_set_join_reg_etc::pst_event_mem ALLOC FAIL, size = %d.}",
                       OAL_SIZEOF(dmac_ctx_join_req_set_reg_stru));
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }
    /* 填写事件 */
    pst_event = frw_get_event_stru(pst_event_mem);
    FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                       FRW_EVENT_TYPE_WLAN_CTX,
                       DMAC_WLAN_CTX_EVENT_SUB_TYPE_JOIN_SET_REG,
                       OAL_SIZEOF(dmac_ctx_join_req_set_reg_stru),
                       FRW_EVENT_PIPELINE_STAGE_1,
                       pst_mac_vap->uc_chip_id,
                       pst_mac_vap->uc_device_id,
                       pst_mac_vap->uc_vap_id);

    pst_reg_params = (dmac_ctx_join_req_set_reg_stru *)pst_event->auc_event_data;

    /* 设置需要写入寄存器的BSSID信息 */
    oal_set_mac_addr(pst_reg_params->auc_bssid, pst_mac_vap->auc_bssid);

    /* 填写信道相关信息 */
    pst_reg_params->st_current_channel.uc_chan_number = pst_mac_vap->st_channel.uc_chan_number;
    pst_reg_params->st_current_channel.en_band = pst_mac_vap->st_channel.en_band;
    pst_reg_params->st_current_channel.en_bandwidth = pst_mac_vap->st_channel.en_bandwidth;
    pst_reg_params->st_current_channel.uc_chan_idx = pst_mac_vap->st_channel.uc_chan_idx;

    /* 以old user信息塑造虚假的入网结构体，调用函数 */
    /* 填写速率相关信息 */
    st_join_req.st_bss_dscr.uc_num_supp_rates = pst_hmac_user->st_op_rates.uc_rs_nrates;
    if (EOK != memcpy_s(st_join_req.st_bss_dscr.auc_supp_rates,
                        OAL_SIZEOF(oal_uint8) * WLAN_USER_MAX_SUPP_RATES,
                        pst_hmac_user->st_op_rates.auc_rs_rates,
                        OAL_SIZEOF(oal_uint8) * WLAN_MAX_SUPP_RATES)) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "hmac_roam_connect_set_join_reg_etc::memcpy fail!");
        FRW_EVENT_FREE(pst_event_mem);
        return OAL_FAIL;
    }
    if (hmac_ht_self_cure_in_blacklist(pst_hmac_user->st_user_base_info.auc_user_mac_addr)) {
        st_join_req.st_bss_dscr.en_ht_capable = pst_hmac_user->st_user_base_info.st_ht_hdl.en_ht_capable;
    }
    st_join_req.st_bss_dscr.en_vht_capable = pst_hmac_user->st_user_base_info.st_vht_hdl.en_vht_capable;
    hmac_sta_get_min_rate(&pst_reg_params->st_min_rate, &st_join_req);

    /* 设置dtim period信息 */
    pst_reg_params->us_beacon_period = (oal_uint16)mac_mib_get_BeaconPeriod(pst_mac_vap);

    /* 同步FortyMHzOperationImplemented */
    pst_reg_params->en_dot11FortyMHzOperationImplemented = mac_mib_get_FortyMHzOperationImplemented(pst_mac_vap);

    /* 设置beacon filter关闭 */
    pst_reg_params->ul_beacon_filter = OAL_FALSE;

    /* 设置no frame filter打开 */
    pst_reg_params->ul_non_frame_filter = OAL_TRUE;

    pst_reg_params->en_ap_type = pst_hmac_user->en_user_ap_type;

    /* 分发事件 */
    frw_event_dispatch_event_etc(pst_event_mem);
    FRW_EVENT_FREE(pst_event_mem);

    return OAL_SUCC;
}


oal_uint32 hmac_roam_connect_set_dtim_param_etc(mac_vap_stru *pst_mac_vap,
                                                oal_uint8 uc_dtim_cnt,
                                                oal_uint8 uc_dtim_period)
{
    frw_event_mem_stru *pst_event_mem = OAL_PTR_NULL;
    frw_event_stru *pst_event = OAL_PTR_NULL;
    dmac_ctx_set_dtim_tsf_reg_stru *pst_set_dtim_tsf_reg_params = OAL_PTR_NULL;

    if (pst_mac_vap == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_set_dtim_param_etc, pst_mac_vap = NULL!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* 抛事件 DMAC_WLAN_CTX_EVENT_SUB_TYPE_JOIN_DTIM_TSF_REG 到DMAC, 申请事件内存 */
    pst_event_mem = FRW_EVENT_ALLOC(OAL_SIZEOF(dmac_ctx_set_dtim_tsf_reg_stru));
    if (pst_event_mem == OAL_PTR_NULL) {
        OAM_ERROR_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_connect_set_dtim_param_etc::pst_event_mem ALLOC FAIL, size = %d.}",
                       OAL_SIZEOF(dmac_ctx_set_dtim_tsf_reg_stru));
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }

    /* 填写事件 */
    pst_event = frw_get_event_stru(pst_event_mem);

    FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                       FRW_EVENT_TYPE_WLAN_CTX,
                       DMAC_WLAN_CTX_EVENT_SUB_TYPE_JOIN_DTIM_TSF_REG,
                       OAL_SIZEOF(dmac_ctx_set_dtim_tsf_reg_stru),
                       FRW_EVENT_PIPELINE_STAGE_1,
                       pst_mac_vap->uc_chip_id,
                       pst_mac_vap->uc_device_id,
                       pst_mac_vap->uc_vap_id);

    pst_set_dtim_tsf_reg_params = (dmac_ctx_set_dtim_tsf_reg_stru *)pst_event->auc_event_data;

    /* 将dtim相关参数抛到dmac */
    pst_set_dtim_tsf_reg_params->ul_dtim_cnt = uc_dtim_cnt;
    pst_set_dtim_tsf_reg_params->ul_dtim_period = uc_dtim_period;
    memcpy_s(pst_set_dtim_tsf_reg_params->auc_bssid, WLAN_MAC_ADDR_LEN, pst_mac_vap->auc_bssid, WLAN_MAC_ADDR_LEN);
    pst_set_dtim_tsf_reg_params->us_tsf_bit0 = BIT0;

    /* 分发事件 */
    frw_event_dispatch_event_etc(pst_event_mem);
    FRW_EVENT_FREE(pst_event_mem);

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 hmac_roam_connect_notify_wpas(hmac_roam_info_stru *pst_roam_info,
                                                    oal_uint8 *puc_mac_hdr,
                                                    oal_uint16 us_msg_len)
{
    hmac_asoc_rsp_stru st_asoc_rsp;
    hmac_roam_rsp_stru st_roam_rsp;
    frw_event_mem_stru *pst_event_mem = OAL_PTR_NULL;
    frw_event_stru *pst_event = OAL_PTR_NULL;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    oal_uint8 *puc_mgmt_data = OAL_PTR_NULL;
    oal_uint32 ul_ret;
    oal_int32 l_ret;

    memset_s(&st_asoc_rsp, OAL_SIZEOF(hmac_asoc_rsp_stru), 0, OAL_SIZEOF(hmac_asoc_rsp_stru));
    memset_s(&st_roam_rsp, OAL_SIZEOF(hmac_roam_rsp_stru), 0, OAL_SIZEOF(hmac_roam_rsp_stru));
    /* 获取AP的mac地址 */
    mac_get_address3(puc_mac_hdr, st_roam_rsp.auc_bssid, WLAN_MAC_ADDR_LEN);
    mac_get_address3(puc_mac_hdr, st_asoc_rsp.auc_addr_ap, WLAN_MAC_ADDR_LEN);

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    pst_hmac_user = (hmac_user_stru *)mac_res_get_hmac_user_etc(pst_hmac_vap->st_vap_base_info.us_assoc_vap_id);
    if (pst_hmac_user == OAL_PTR_NULL) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_connect_notify_wpas, pst_hmac_user[%d] = NULL!}",
                         pst_hmac_vap->st_vap_base_info.us_assoc_vap_id);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* 获取关联请求帧信息 */
    st_roam_rsp.puc_asoc_req_ie_buff = pst_hmac_user->puc_assoc_req_ie_buff;
    st_roam_rsp.ul_asoc_req_ie_len = pst_hmac_user->ul_assoc_req_ie_len;
    st_asoc_rsp.puc_asoc_req_ie_buff = pst_hmac_user->puc_assoc_req_ie_buff;
    st_asoc_rsp.ul_asoc_req_ie_len = pst_hmac_user->ul_assoc_req_ie_len;

    pst_event_mem = FRW_EVENT_ALLOC(OAL_SIZEOF(hmac_roam_rsp_stru));
    if (pst_event_mem == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_handle_asoc_rsp_sta_etc::FRW_EVENT_ALLOC fail!}");
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }

    /* 记录关联响应帧的部分内容，用于上报给内核 */
    if (us_msg_len < OAL_ASSOC_RSP_IE_OFFSET) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                       "{hmac_handle_asoc_rsp_sta_etc::us_msg_len is too short, %d.}", us_msg_len);
        FRW_EVENT_FREE(pst_event_mem);
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }

    puc_mgmt_data = (oal_uint8 *)oal_memalloc(us_msg_len - OAL_ASSOC_RSP_IE_OFFSET);
    if (puc_mgmt_data == OAL_PTR_NULL) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                       "{hmac_handle_asoc_rsp_sta_etc::pst_mgmt_data alloc null,size %d.}",
                       (us_msg_len - OAL_ASSOC_RSP_IE_OFFSET));
        FRW_EVENT_FREE(pst_event_mem);
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }
    st_roam_rsp.ul_asoc_rsp_ie_len = us_msg_len - OAL_ASSOC_RSP_IE_OFFSET;
    l_ret = memcpy_s(puc_mgmt_data, st_roam_rsp.ul_asoc_rsp_ie_len,
                     (oal_uint8 *)(puc_mac_hdr + OAL_ASSOC_RSP_IE_OFFSET), st_roam_rsp.ul_asoc_rsp_ie_len);
    st_roam_rsp.puc_asoc_rsp_ie_buff = puc_mgmt_data;
    st_asoc_rsp.ul_asoc_rsp_ie_len = st_roam_rsp.ul_asoc_rsp_ie_len;
    st_asoc_rsp.puc_asoc_rsp_ie_buff = st_roam_rsp.puc_asoc_rsp_ie_buff;

    pst_event = frw_get_event_stru(pst_event_mem);

    if (!oal_memcmp(st_roam_rsp.auc_bssid, pst_roam_info->st_old_bss.auc_bssid, WLAN_MAC_ADDR_LEN)) {
        /* Reassociation to the same BSSID: report NL80211_CMD_CONNECT event to supplicant instead of NL80211_CMD_ROAM event
         * in case supplicant ignore roam event to the same bssid which will cause 4-way handshake failure */
        /* wpa_supplicant: wlan0: WPA: EAPOL-Key Replay Counter did not increase - dropping packet */
        OAM_WARNING_LOG4(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                         "{hmac_roam_connect_notify_wpas::roam_to the same bssid [%02X:XX:XX:%02X:%02X:%02X]}",
                         st_roam_rsp.auc_bssid[0], st_roam_rsp.auc_bssid[3],
                         st_roam_rsp.auc_bssid[4], st_roam_rsp.auc_bssid[5]);
        FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                           FRW_EVENT_TYPE_HOST_CTX,
                           HMAC_HOST_CTX_EVENT_SUB_TYPE_ASOC_COMP_STA,
                           OAL_SIZEOF(hmac_asoc_rsp_stru),
                           FRW_EVENT_PIPELINE_STAGE_0,
                           pst_hmac_vap->st_vap_base_info.uc_chip_id,
                           pst_hmac_vap->st_vap_base_info.uc_device_id,
                           pst_hmac_vap->st_vap_base_info.uc_vap_id);
        l_ret += memcpy_s((oal_uint8 *)frw_get_event_payload(pst_event_mem),
                          OAL_SIZEOF(hmac_roam_rsp_stru),
                          (oal_uint8 *)&st_asoc_rsp,
                          OAL_SIZEOF(hmac_asoc_rsp_stru));
    } else {
        FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                           FRW_EVENT_TYPE_HOST_CTX,
                           HMAC_HOST_CTX_EVENT_SUB_TYPE_ROAM_COMP_STA,
                           OAL_SIZEOF(hmac_roam_rsp_stru),
                           FRW_EVENT_PIPELINE_STAGE_0,
                           pst_hmac_vap->st_vap_base_info.uc_chip_id,
                           pst_hmac_vap->st_vap_base_info.uc_device_id,
                           pst_hmac_vap->st_vap_base_info.uc_vap_id);
        l_ret += memcpy_s((oal_uint8 *)frw_get_event_payload(pst_event_mem),
                          OAL_SIZEOF(hmac_roam_rsp_stru),
                          (oal_uint8 *)&st_roam_rsp,
                          OAL_SIZEOF(hmac_roam_rsp_stru));
    }
    if (l_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "hmac_roam_connect_notify_wpas::memcpy fail!");
        oal_free(puc_mgmt_data);
        puc_mgmt_data = OAL_PTR_NULL;
        FRW_EVENT_FREE(pst_event_mem);
        return OAL_FAIL;
    }

    /* 分发事件 */
    ul_ret = frw_event_dispatch_event_etc(pst_event_mem);
    if (OAL_UNLIKELY(ul_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY,
                         "{hmac_roam_connect_notify_wpas::frw_event_dispatch_event_etc failed[%d].}", ul_ret);
        oal_free(puc_mgmt_data);
        puc_mgmt_data = OAL_PTR_NULL;
    }

    FRW_EVENT_FREE(pst_event_mem);
    return ul_ret;
}
#ifdef _PRE_WLAN_FEATURE_11R

OAL_STATIC oal_uint32 hmac_roam_ft_notify_wpas(hmac_vap_stru *pst_hmac_vap,
                                               oal_uint8 *puc_mac_hdr,
                                               oal_uint16 us_msg_len)
{
    hmac_roam_ft_stru *pst_ft_event = OAL_PTR_NULL;
    frw_event_mem_stru *pst_event_mem;
    frw_event_stru *pst_event = OAL_PTR_NULL;
    oal_uint16 us_ie_offset;
    oal_uint8 *puc_ft_ie_buff = OAL_PTR_NULL;
    oal_uint32 ul_ret;
    oal_uint8 *puc_target_ap_addr = OAL_PTR_NULL;
    oal_int32 l_ret = EOK;

    pst_event_mem = FRW_EVENT_ALLOC(OAL_SIZEOF(hmac_roam_rsp_stru));
    if (pst_event_mem == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_ft_notify_wpas::FRW_EVENT_ALLOC fail!}");
        return OAL_ERR_CODE_PTR_NULL;
    }
    pst_event = frw_get_event_stru(pst_event_mem);
    FRW_EVENT_HDR_INIT(&(pst_event->st_event_hdr),
                       FRW_EVENT_TYPE_HOST_CTX,
                       HMAC_HOST_CTX_EVENT_SUB_TYPE_FT_EVENT_STA,
                       OAL_SIZEOF(hmac_roam_ft_stru),
                       FRW_EVENT_PIPELINE_STAGE_0,
                       pst_hmac_vap->st_vap_base_info.uc_chip_id,
                       pst_hmac_vap->st_vap_base_info.uc_device_id,
                       pst_hmac_vap->st_vap_base_info.uc_vap_id);

    pst_ft_event = (hmac_roam_ft_stru *)pst_event->auc_event_data;

    if ((WLAN_FC0_SUBTYPE_AUTH | WLAN_FC0_TYPE_MGT) == mac_get_frame_type_and_subtype(puc_mac_hdr)) {
        us_ie_offset = OAL_AUTH_IE_OFFSET;
        mac_get_address3(puc_mac_hdr, pst_ft_event->auc_bssid, WLAN_MAC_ADDR_LEN);
    } else {
        us_ie_offset = OAL_FT_ACTION_IE_OFFSET;
        /* 在ft response中，header结尾到target ap addr的偏移量为8字节 */
        puc_target_ap_addr = puc_mac_hdr + MAC_80211_FRAME_LEN + 8;

        l_ret += memcpy_s(pst_ft_event->auc_bssid, OAL_MAC_ADDR_LEN, puc_target_ap_addr, OAL_MAC_ADDR_LEN);
    }

    pst_ft_event->us_ft_ie_len = us_msg_len - us_ie_offset;
    /* 修改为hmac申请内存，wal释放
     * 避免hmac抛事件后netbuffer被释放，wal使用已经释放的内存
     */
    puc_ft_ie_buff = oal_memalloc(pst_ft_event->us_ft_ie_len);
    if (puc_ft_ie_buff == OAL_PTR_NULL) {
        FRW_EVENT_FREE(pst_event_mem);
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ASSOC,
                       "{hmac_roam_ft_notify_wpas::alloc ft_ie_buff fail.len [%d].}",
                       pst_ft_event->us_ft_ie_len);
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }
    l_ret += memcpy_s(puc_ft_ie_buff, pst_ft_event->us_ft_ie_len,
                      puc_mac_hdr + us_ie_offset, pst_ft_event->us_ft_ie_len);
    if (l_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "hmac_roam_ft_notify_wpas::memcpy fail!");
        oal_free(puc_ft_ie_buff);
        puc_ft_ie_buff = OAL_PTR_NULL;
        FRW_EVENT_FREE(pst_event_mem);
        return OAL_FAIL;
    }

    pst_ft_event->puc_ft_ie_buff = puc_ft_ie_buff;

    /* 分发事件 */
    ul_ret = frw_event_dispatch_event_etc(pst_event_mem);
    if (OAL_UNLIKELY(ul_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{hmac_roam_ft_notify_wpas::frw_event_dispatch_event_etc failed[%d].}", ul_ret);
        oal_free(puc_ft_ie_buff);
        puc_ft_ie_buff = OAL_PTR_NULL;
    }

    FRW_EVENT_FREE(pst_event_mem);
    return ul_ret;
}


OAL_STATIC oal_uint32 hmac_roam_send_ft_req(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    oal_netbuf_stru *pst_ft_frame = OAL_PTR_NULL;
    oal_uint8 *puc_ft_buff = OAL_PTR_NULL;
    mac_tx_ctl_stru *pst_tx_ctl = OAL_PTR_NULL;
    oal_uint8 *puc_my_mac_addr = OAL_PTR_NULL;
    oal_uint8 *puc_current_bssid = OAL_PTR_NULL;
    oal_uint16 us_ft_len;
    oal_uint16 us_app_ie_len;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_UP,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_INIT);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_send_ft_req::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    pst_hmac_user = pst_roam_info->pst_hmac_user;

    if (pst_hmac_vap->bit_11r_enable != OAL_TRUE) {
        return OAL_SUCC;
    }

    pst_ft_frame = OAL_MEM_NETBUF_ALLOC(OAL_NORMAL_NETBUF, WLAN_MEM_NETBUF_SIZE2, OAL_NETBUF_PRIORITY_MID);
    if (pst_ft_frame == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_send_ft_req::OAL_MEM_NETBUF_ALLOC fail.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    puc_ft_buff = (oal_uint8 *)OAL_NETBUF_HEADER(pst_ft_frame);
    memset_s(oal_netbuf_cb(pst_ft_frame), OAL_NETBUF_CB_SIZE(), 0, OAL_NETBUF_CB_SIZE());
    memset_s(puc_ft_buff, MAC_80211_FRAME_LEN, 0, MAC_80211_FRAME_LEN);

    puc_my_mac_addr = mac_mib_get_StationID(&pst_hmac_vap->st_vap_base_info);
    puc_current_bssid = pst_hmac_vap->st_vap_base_info.auc_bssid;
    /*************************************************************************/
    /*                        Management Frame Format                        */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BSSID|Sequence Control|Frame Body|FCS|  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |6    |2               |0 - 2312  |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/
    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/
    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    mac_hdr_set_frame_control(puc_ft_buff, WLAN_FC0_SUBTYPE_ACTION);
    /*  Set DA  */
    oal_set_mac_addr(((mac_ieee80211_frame_stru *)puc_ft_buff)->auc_address1, puc_current_bssid);
    /*  Set SA  */
    oal_set_mac_addr(((mac_ieee80211_frame_stru *)puc_ft_buff)->auc_address2, puc_my_mac_addr);
    /*  Set SSID  */
    oal_set_mac_addr(((mac_ieee80211_frame_stru *)puc_ft_buff)->auc_address3, puc_current_bssid);

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/
    /*************************************************************************/
    /*                  FT Request Frame - Frame Body                          */
    /* --------------------------------------------------------------------- */
    /* | Category | Action | STA Addr |Target AP Addr | FT Req frame body  | */
    /* --------------------------------------------------------------------- */
    /* |     1    |   1    |     6    |       6       |       varibal      | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/
    puc_ft_buff += MAC_80211_FRAME_LEN;
    us_ft_len = MAC_80211_FRAME_LEN;

    puc_ft_buff[0] = MAC_ACTION_CATEGORY_FAST_BSS_TRANSITION;
    puc_ft_buff[1] = MAC_FT_ACTION_REQUEST;
    puc_ft_buff += 2;
    us_ft_len += 2;

    oal_set_mac_addr(puc_ft_buff, puc_my_mac_addr);
    puc_ft_buff += OAL_MAC_ADDR_LEN;
    us_ft_len += OAL_MAC_ADDR_LEN;

    oal_set_mac_addr(puc_ft_buff, pst_roam_info->st_connect.pst_bss_dscr->auc_bssid);
    puc_ft_buff += OAL_MAC_ADDR_LEN;
    us_ft_len += OAL_MAC_ADDR_LEN;

    mac_add_app_ie_etc((oal_void *)&pst_hmac_vap->st_vap_base_info, puc_ft_buff, &us_app_ie_len, OAL_APP_FT_IE);
    us_ft_len += us_app_ie_len;
    puc_ft_buff += us_app_ie_len;

    oal_netbuf_put(pst_ft_frame, us_ft_len);

    /* 为填写发送描述符准备参数 */
    pst_tx_ctl = (mac_tx_ctl_stru *)oal_netbuf_cb(pst_ft_frame);
    MAC_GET_CB_MPDU_LEN(pst_tx_ctl) = us_ft_len;
    MAC_GET_CB_TX_USER_IDX(pst_tx_ctl) = pst_hmac_user->st_user_base_info.us_assoc_id;
    MAC_GET_CB_NETBUF_NUM(pst_tx_ctl) = 1;

    /* 抛事件让dmac将该帧发送 */
    ul_ret = hmac_tx_mgmt_send_event_etc(&pst_hmac_vap->st_vap_base_info, pst_ft_frame, us_ft_len);
    if (ul_ret != OAL_SUCC) {
        oal_netbuf_free(pst_ft_frame);
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_send_ft_req::hmac_tx_mgmt_send_event_etc failed[%d].}", ul_ret);
        return ul_ret;
    }

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_FT_COMP);

    /* 启动认证超时定时器 */
    hmac_roam_connect_start_timer(pst_roam_info, ROAM_AUTH_TIME_MAX);

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 hmac_roam_process_ft_rsp(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    dmac_wlan_crx_event_stru *pst_crx_event = OAL_PTR_NULL;
    mac_rx_ctl_stru *pst_rx_ctrl = OAL_PTR_NULL;
    oal_uint8 *puc_mac_hdr = OAL_PTR_NULL;
    oal_uint8 *puc_ft_frame_body = OAL_PTR_NULL;
    oal_uint16 us_auth_status;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_UP,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_FT_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_process_ft_rsp::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;

    if (pst_hmac_vap->bit_11r_enable != OAL_TRUE) {
        return OAL_SUCC;
    }

    pst_crx_event = (dmac_wlan_crx_event_stru *)p_param;
    pst_rx_ctrl = (mac_rx_ctl_stru *)oal_netbuf_cb(pst_crx_event->pst_netbuf);
    puc_mac_hdr = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(pst_rx_ctrl);
    hmac_roam_connect_pm_wkup(pst_roam_info, p_param);

    /* 只处理action帧 */
    if ((WLAN_FC0_SUBTYPE_ACTION | WLAN_FC0_TYPE_MGT) != mac_get_frame_type_and_subtype(puc_mac_hdr)) {
        return OAL_SUCC;
    }

    puc_ft_frame_body = puc_mac_hdr + MAC_80211_FRAME_LEN;

    if ((puc_ft_frame_body[0] != MAC_ACTION_CATEGORY_FAST_BSS_TRANSITION) ||
        (puc_ft_frame_body[1] != MAC_FT_ACTION_RESPONSE)) {
        return OAL_SUCC;
    }

    us_auth_status = mac_get_ft_status(puc_mac_hdr);
    if (us_auth_status != MAC_SUCCESSFUL_STATUSCODE) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                         "{hmac_roam_process_ft_rsp:: status code[%d], change to ft over the air!}", us_auth_status);
        pst_roam_info->st_connect.uc_ft_force_air = OAL_TRUE;
        pst_roam_info->st_connect.uc_ft_failed = OAL_TRUE;
        ul_ret = hmac_roam_connect_ft_ds_change_to_air_etc(pst_hmac_vap, pst_roam_info->st_connect.pst_bss_dscr);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                "{hmac_roam_process_ft_rsp::hmac_roam_connect_ft_ds_change_to_air_etc failed[%d].}", ul_ret);
            return hmac_roam_connect_fail(pst_roam_info, pst_roam_info->st_connect.pst_bss_dscr);
        }
        return OAL_SUCC;
    }

#ifdef _PRE_WLAN_1103_CHR
    pst_roam_info->st_static.uc_roam_mode = HMAC_CHR_OVER_DS;
#endif

    /* 上报FT成功消息给APP，以便APP下发新的FT_IE用于发送reassociation */
    ul_ret = hmac_roam_ft_notify_wpas(pst_hmac_vap, puc_mac_hdr, pst_rx_ctrl->us_frame_len);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_process_ft_rsp::hmac_roam_ft_notify_wpas failed[%d].}", ul_ret);
        return ul_ret;
    }

    if (pst_hmac_vap->bit_11r_private_preauth == OAL_TRUE) {
        hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_PREAUTH_COMP);
    } else {
        hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_ASSOC_COMP);
    }

    /* 启动关联超时定时器 */
    hmac_roam_connect_start_timer(pst_roam_info, ROAM_ASSOC_TIME_MAX);
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_roam_encap_ft_preauth_req(hmac_roam_info_stru *pst_roam_info,
                                                              oal_netbuf_stru *pst_ft_frame,
                                                              oal_uint16 *pus_ft_len)
{
    oal_uint8 *puc_ft_buff = OAL_PTR_NULL;
    oal_uint8 *puc_my_mac_addr = OAL_PTR_NULL;
    oal_uint8 *puc_current_bssid = OAL_PTR_NULL;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    oal_uint16 us_app_ie_len;

    if (pst_roam_info == OAL_PTR_NULL || pst_ft_frame== OAL_PTR_NULL || pus_ft_len == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{hmac_roam_encap_ft_preauth_req::NULL pointer}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    puc_ft_buff = (oal_uint8 *)OAL_NETBUF_HEADER(pst_ft_frame);
    memset_s(oal_netbuf_cb(pst_ft_frame), OAL_NETBUF_CB_SIZE(), 0, OAL_NETBUF_CB_SIZE());
    memset_s(puc_ft_buff, MAC_80211_FRAME_LEN, 0, MAC_80211_FRAME_LEN);

    puc_my_mac_addr     = mac_mib_get_StationID(&pst_hmac_vap->st_vap_base_info);
    puc_current_bssid   = pst_hmac_vap->st_vap_base_info.auc_bssid;
    /*************************************************************************/
    /*                        Management Frame Format                        */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BSSID|Sequence Control|Frame Body|FCS|  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |6    |2               |0 - 2312  |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/
    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/
    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    mac_hdr_set_frame_control(puc_ft_buff, WLAN_FC0_SUBTYPE_ACTION);
    /*  Set DA  */
    oal_set_mac_addr(((mac_ieee80211_frame_stru *)puc_ft_buff)->auc_address1, puc_current_bssid);
    /*  Set SA  */
    oal_set_mac_addr(((mac_ieee80211_frame_stru *)puc_ft_buff)->auc_address2, puc_my_mac_addr);
    /*  Set SSID  */
    oal_set_mac_addr(((mac_ieee80211_frame_stru *)puc_ft_buff)->auc_address3, puc_current_bssid);

    /*************************************************************************/
    /*                Set the contents of the frame body                     */
    /*************************************************************************/
    /*************************************************************************/
    /*                  FT Request Frame - Frame Body                          */
    /* --------------------------------------------------------------------- */
    /* | Category | Action | STA Addr |Target AP Addr | FT Req frame body  |*/
    /* --------------------------------------------------------------------- */
    /* |     1    |   1    |     6    |       6       |       varibal      | */
    /* --------------------------------------------------------------------- */
    /*                                                                       */
    /*************************************************************************/
    puc_ft_buff += MAC_80211_FRAME_LEN;
    *pus_ft_len = MAC_80211_FRAME_LEN;

    puc_ft_buff[0] = MAC_ACTION_CATEGORY_FAST_BSS_TRANSITION;
    puc_ft_buff[1] = MAC_FT_ACTION_PREAUTH_REQUEST;
    puc_ft_buff += 2;
    *pus_ft_len += 2;

    oal_set_mac_addr(puc_ft_buff, puc_my_mac_addr);
    puc_ft_buff += OAL_MAC_ADDR_LEN;
    *pus_ft_len += OAL_MAC_ADDR_LEN;

    oal_set_mac_addr(puc_ft_buff, pst_roam_info->st_connect.pst_bss_dscr->auc_bssid);
    puc_ft_buff += OAL_MAC_ADDR_LEN;
    *pus_ft_len += OAL_MAC_ADDR_LEN;

    mac_add_app_ie_etc((oal_void *)&pst_hmac_vap->st_vap_base_info, puc_ft_buff, &us_app_ie_len, OAL_APP_FT_IE);
    *pus_ft_len += us_app_ie_len;
    puc_ft_buff += us_app_ie_len;

    return OAL_SUCC;
}



OAL_STATIC oal_uint32 hmac_roam_send_ft_preauth_req(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32              ul_ret;
    hmac_vap_stru          *pst_hmac_vap = OAL_PTR_NULL;
    hmac_user_stru         *pst_hmac_user = OAL_PTR_NULL;
    oal_netbuf_stru        *pst_ft_frame = OAL_PTR_NULL;
    mac_tx_ctl_stru        *pst_tx_ctl = OAL_PTR_NULL;
    oal_uint16              us_ft_len = 0;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_UP, ROAM_MAIN_STATE_CONNECTING,
                                           ROAM_CONNECT_STATE_WAIT_PREAUTH_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_send_ft_preauth_req::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap  = pst_roam_info->pst_hmac_vap;
    pst_hmac_user = pst_roam_info->pst_hmac_user;

    if (pst_hmac_vap->bit_11r_enable != OAL_TRUE || pst_hmac_vap->bit_11r_private_preauth != OAL_TRUE) {
        return OAL_SUCC;
    }

    pst_ft_frame = OAL_MEM_NETBUF_ALLOC(OAL_NORMAL_NETBUF, WLAN_MEM_NETBUF_SIZE2, OAL_NETBUF_PRIORITY_MID);
    if (pst_ft_frame == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_send_ft_preauth_req::OAL_MEM_NETBUF_ALLOC fail.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    ul_ret = hmac_roam_encap_ft_preauth_req(pst_roam_info, pst_ft_frame, &us_ft_len);
    if (ul_ret != OAL_SUCC) {
        oal_netbuf_free(pst_ft_frame);
        OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{hmac_roam_send_ft_preauth_req::encap ft preauth FAILED}");
        return ul_ret;
    }

    oal_netbuf_put(pst_ft_frame, us_ft_len);

    /* 为填写发送描述符准备参数 */
    pst_tx_ctl = (mac_tx_ctl_stru *)oal_netbuf_cb(pst_ft_frame);
    MAC_GET_CB_MPDU_LEN(pst_tx_ctl) = us_ft_len;
    MAC_GET_CB_TX_USER_IDX(pst_tx_ctl) = pst_hmac_user->st_user_base_info.us_assoc_id;
    MAC_GET_CB_NETBUF_NUM(pst_tx_ctl) = 1;

    OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{hmac_roam_send_ft_preauth_req::sending preauth request}");

    /* 抛事件让dmac将该帧发送 */
    if (hmac_tx_mgmt_send_event_etc(&pst_hmac_vap->st_vap_base_info, pst_ft_frame, us_ft_len) != OAL_SUCC) {
        oal_netbuf_free(pst_ft_frame);
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_send_ft_preauth_req::send event failed}");
        return ul_ret;
    }

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_PREAUTH_COMP);

    /* 启动认证超时定时器 */
    hmac_roam_connect_start_timer(pst_roam_info, ROAM_AUTH_TIME_MAX);

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_roam_process_ft_preauth_rsp(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32                   ul_ret;
    hmac_vap_stru               *pst_hmac_vap = OAL_PTR_NULL;
    mac_rx_ctl_stru             *pst_rx_ctrl = OAL_PTR_NULL;
    oal_uint8                   *puc_mac_hdr = OAL_PTR_NULL;
    oal_uint8                   *puc_ft_frame_body = OAL_PTR_NULL;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_UP,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_PREAUTH_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ROAM, "{hmac_roam_process_ft_preauth_rsp::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    if (pst_hmac_vap->bit_11r_enable != OAL_TRUE || pst_hmac_vap->bit_11r_private_preauth != OAL_TRUE) {
        return OAL_SUCC;
    }

    if (p_param == OAL_PTR_NULL) {
        return OAL_FAIL;
    }
    pst_rx_ctrl = (mac_rx_ctl_stru *)oal_netbuf_cb(((dmac_wlan_crx_event_stru *)p_param)->pst_netbuf);
    puc_mac_hdr = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(pst_rx_ctrl);
    hmac_roam_connect_pm_wkup(pst_roam_info, p_param);

    /* 只处理action帧 */
    if ((WLAN_FC0_SUBTYPE_ACTION | WLAN_FC0_TYPE_MGT) != mac_get_frame_type_and_subtype(puc_mac_hdr)) {
        return OAL_SUCC;
    }

    puc_ft_frame_body = puc_mac_hdr + MAC_80211_FRAME_LEN;
    if ((puc_ft_frame_body[0] != MAC_ACTION_CATEGORY_FAST_BSS_TRANSITION) ||
        (puc_ft_frame_body[1] != MAC_FT_ACTION_PREAUTH_RESPONSE)) {
        return OAL_SUCC;
    }

    if (mac_get_ft_status(puc_mac_hdr) != MAC_SUCCESSFUL_STATUSCODE) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_process_ft_preauth_rsp::bad status,change to air}");
        pst_roam_info->st_connect.uc_ft_force_air = OAL_TRUE;
        pst_roam_info->st_connect.uc_ft_failed = OAL_TRUE;
        ul_ret = hmac_roam_connect_ft_ds_change_to_air_etc(pst_hmac_vap, pst_roam_info->st_connect.pst_bss_dscr);
        if (ul_ret != OAL_SUCC) {
            return hmac_roam_connect_fail(pst_roam_info, pst_roam_info->st_connect.pst_bss_dscr);
        }
        return OAL_SUCC;
    }

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_ASSOC_COMP);
    hmac_roam_connect_start_timer(pst_roam_info, ROAM_ASSOC_TIME_MAX);
    if (hmac_roam_reassoc_etc(pst_hmac_vap) != OAL_SUCC) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_process_ft_preauth_rsp::reassoc failed}");
        return OAL_FAIL;
    }
    return OAL_SUCC;
}
#endif  // _PRE_WLAN_FEATURE_11R


OAL_STATIC oal_uint32 hmac_roam_set_owe_dh_ie(mac_vap_stru *pst_mac_vap,
                                              oal_uint8 *puc_buffer,
                                              oal_uint8 *puc_buffer_len,
                                              oal_uint32 uc_buffer_max_len)
{
    oal_uint8 *puc_app_ie = OAL_PTR_NULL;
    oal_uint32 ul_app_ie_len;
    oal_uint8 *puc_ie = OAL_PTR_NULL;

    puc_app_ie = pst_mac_vap->ast_app_ie[OAL_APP_ASSOC_REQ_IE].puc_ie;
    ul_app_ie_len = pst_mac_vap->ast_app_ie[OAL_APP_ASSOC_REQ_IE].ul_ie_len;
    puc_ie = mac_find_ie_ext_ie(MAC_EID_EXTENSION, MAC_EID_EXT_OWE_DH_PARAM, puc_app_ie, ul_app_ie_len);

    if (puc_ie == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_ROAM, "hmac_roam_set_owe_dh_ie::not find owe dh param");
        return OAL_FAIL;
    }

    if (*puc_buffer_len + puc_ie[1] + MAC_IE_HDR_LEN > uc_buffer_max_len) {
        OAM_WARNING_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_ROAM, "hmac_roam_set_owe_dh_ie::app ie len exceed limit");
        return OAL_FAIL;
    }

    if (EOK != memcpy_s(puc_buffer + *puc_buffer_len, WLAN_WPS_IE_MAX_SIZE - *puc_buffer_len,
                        puc_ie, puc_ie[1] + MAC_IE_HDR_LEN)) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "hmac_roam_set_owe_dh_ie::memcpy fail!");
        return OAL_FAIL;
    }
    *puc_buffer_len += puc_ie[1] + MAC_IE_HDR_LEN;

    return OAL_SUCC;
}

OAL_STATIC oal_void hmac_roam_join_ht_blacklist_check(hmac_vap_stru *pst_hmac_vap,
    mac_bss_dscr_stru *pst_bss_dscr)
{
    mac_vap_stru *p_mac_vap = OAL_PTR_NULL;
    hmac_user_stru *p_hmac_user = OAL_PTR_NULL;

    p_mac_vap = &pst_hmac_vap->st_vap_base_info;

    if (p_mac_vap->en_protocol <  WLAN_HT_MODE) {
        p_hmac_user = mac_res_get_hmac_user_etc(p_mac_vap->us_assoc_vap_id);
        if (p_hmac_user == OAL_PTR_NULL) {
            return;
        }

        if (p_hmac_user->closeHtFalg) {
            pst_bss_dscr->en_ht_capable = OAL_FALSE;
            OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_join_ht_blacklist_check::en_ht_capable = [%d]!}",
                pst_bss_dscr->en_ht_capable);
        }
    }
}


OAL_STATIC oal_uint32 hmac_roam_start_join(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    mac_bss_dscr_stru *pst_bss_dscr = OAL_PTR_NULL;
    hmac_join_req_stru st_join_req;
    oal_app_ie_stru st_app_ie;
    oal_uint8 uc_ie_len = 0;
    oal_uint8 *puc_pmkid = OAL_PTR_NULL;
    oal_uint8 uc_rate_num;
    oal_uint32 aul_rsn_akm_suites[WLAN_PAIRWISE_CIPHER_SUITES] = { 0 };

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_INIT);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_start_join::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;

    pst_bss_dscr = (mac_bss_dscr_stru *)p_param;

    uc_rate_num = (pst_bss_dscr->uc_num_supp_rates < WLAN_MAX_SUPP_RATES) ?
        pst_bss_dscr->uc_num_supp_rates : WLAN_MAX_SUPP_RATES;
    if (EOK != memcpy_s(pst_hmac_vap->auc_supp_rates, WLAN_MAX_SUPP_RATES, pst_bss_dscr->auc_supp_rates, uc_rate_num)) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "hmac_roam_start_join::memcpy fail!");
        return OAL_FAIL;
    }
    mac_mib_set_SupportRateSetNums(&pst_hmac_vap->st_vap_base_info, pst_bss_dscr->uc_num_supp_rates);
    hmac_roam_join_ht_blacklist_check(pst_hmac_vap, pst_bss_dscr);
    /* 配置join参数 */
    hmac_prepare_join_req_etc(&st_join_req, pst_bss_dscr);

    ul_ret = hmac_sta_update_join_req_params_etc(pst_hmac_vap, &st_join_req);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_start_join::hmac_sta_update_join_req_params_etc fail[%d].}", ul_ret);
        return ul_ret;
    }

    if (pst_hmac_vap->st_vap_base_info.st_cap_flag.bit_wpa == OAL_TRUE) {
        /* 设置 WPA Capability IE */
        mac_set_wpa_ie_etc((oal_void *)&pst_hmac_vap->st_vap_base_info, st_app_ie.auc_ie, &uc_ie_len);
    }

    if (pst_hmac_vap->st_vap_base_info.st_cap_flag.bit_wpa2 == OAL_TRUE) {
        /* 设置 RSN Capability IE */
        puc_pmkid = hmac_vap_get_pmksa_etc(pst_hmac_vap, pst_bss_dscr->auc_bssid);
        mac_set_rsn_ie_etc((oal_void *)&pst_hmac_vap->st_vap_base_info, puc_pmkid, st_app_ie.auc_ie, &uc_ie_len);

        /* 设置OWE DH参数     */
        mac_mib_get_rsn_akm_suites_s(&pst_hmac_vap->st_vap_base_info, aul_rsn_akm_suites, sizeof(aul_rsn_akm_suites));
        if (IS_SUPPORT_OWE(aul_rsn_akm_suites)) {
            hmac_roam_set_owe_dh_ie(&pst_hmac_vap->st_vap_base_info,
                                    st_app_ie.auc_ie, &uc_ie_len,
                                    WLAN_WPS_IE_MAX_SIZE);
        }
    }

    if (0 != uc_ie_len) {
        st_app_ie.en_app_ie_type = OAL_APP_REASSOC_REQ_IE;
        st_app_ie.ul_ie_len = uc_ie_len;
        ul_ret = hmac_config_set_app_ie_to_vap_etc(&pst_hmac_vap->st_vap_base_info, &st_app_ie, OAL_APP_REASSOC_REQ_IE);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                             "{hmac_roam_start_join::hmac_config_set_app_ie_to_vap_etc fail, ul_ret=[%d].}", ul_ret);
        }
    } else {
        mac_vap_clear_app_ie_etc(&pst_hmac_vap->st_vap_base_info, OAL_APP_REASSOC_REQ_IE);
    }

    hmac_roam_connect_set_dtim_param_etc(&pst_hmac_vap->st_vap_base_info,
                                         pst_bss_dscr->uc_dtim_cnt, pst_bss_dscr->uc_dtim_cnt);

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_JOIN);

    ul_ret = hmac_roam_send_auth_seq1(pst_roam_info, p_param);
    if (ul_ret != OAL_SUCC) {
        hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_FAIL);

        /* 通知ROAM主状态机 */
        hmac_roam_connect_complete_etc(pst_hmac_vap, OAL_FAIL);
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_SCAN,
                         "{hmac_roam_process_beacon::hmac_roam_send_auth_seq1 fail[%d].}", ul_ret);
        return ul_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_SAE

OAL_STATIC oal_uint32 hmac_roam_triger_sae_auth(hmac_roam_info_stru *pst_roam_info)
{
    hmac_vap_stru *pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    hmac_user_stru *pst_hmac_user = pst_roam_info->pst_hmac_user;
    oal_uint8 uc_vap_id;

    uc_vap_id = pst_hmac_vap->st_vap_base_info.uc_vap_id;

    OAM_WARNING_LOG0(uc_vap_id, OAM_SF_ROAM, "hmac_roam_triger_sae_auth:: triger sae auth");

    oal_set_mac_addr(pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                     pst_roam_info->st_connect.pst_bss_dscr->auc_bssid);

    hmac_report_external_auth_req_etc(pst_hmac_vap, NL80211_EXTERNAL_AUTH_START);

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_AUTH_COMP);

    hmac_roam_connect_start_timer(pst_roam_info, ROAM_AUTH_TIME_MAX * 2);

    return OAL_SUCC;
}


oal_uint32 hmac_roam_sae_config_reassoc_req(hmac_vap_stru *pst_hmac_vap)
{
    hmac_roam_info_stru *pst_roam_info;
    oal_uint8 uc_vap_id;

    uc_vap_id = pst_hmac_vap->st_vap_base_info.uc_vap_id;
    pst_roam_info = (hmac_roam_info_stru *)pst_hmac_vap->pul_roam_info;

    if (pst_roam_info == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(uc_vap_id, OAM_SF_ROAM, "{hmac_roam_sae_config_reassoc_req::roam_info null!}");
        return OAL_ERR_CODE_ROAM_INVALID_VAP;
    }

    hmac_roam_send_reassoc_req(pst_roam_info);

    return OAL_SUCC;
}

#endif


OAL_STATIC oal_uint32 hmac_roam_send_auth_seq1(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    hmac_user_stru *pst_hmac_user = pst_roam_info->pst_hmac_user;
    oal_netbuf_stru *pst_auth_frame;
    mac_tx_ctl_stru *pst_tx_ctl = OAL_PTR_NULL;
    oal_uint16 us_auth_len;
    oal_uint8 uc_vap_id;

#ifdef _PRE_WLAN_FEATURE_SAE
    if (mac_mib_get_AuthenticationMode(&(pst_hmac_vap->st_vap_base_info)) == WLAN_WITP_AUTH_SAE) {
        ul_ret = hmac_roam_triger_sae_auth(pst_roam_info);
        return ul_ret;
    }
#endif

    uc_vap_id = pst_hmac_vap->st_vap_base_info.uc_vap_id;

    pst_auth_frame = OAL_MEM_NETBUF_ALLOC(OAL_NORMAL_NETBUF, WLAN_MEM_NETBUF_SIZE2, OAL_NETBUF_PRIORITY_MID);
    if (pst_auth_frame == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(uc_vap_id, OAM_SF_ROAM, "{hmac_roam_send_auth_seq1::OAL_MEM_NETBUF_ALLOC fail.}");
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }

    OAL_MEM_NETBUF_TRACE(pst_auth_frame, OAL_TRUE);

    memset_s(oal_netbuf_cb(pst_auth_frame), OAL_NETBUF_CB_SIZE(), 0, OAL_NETBUF_CB_SIZE());

    memset_s((oal_uint8 *)oal_netbuf_header(pst_auth_frame), MAC_80211_FRAME_LEN, 0, MAC_80211_FRAME_LEN);

    /* 更新用户mac */
    oal_set_mac_addr(pst_hmac_user->st_user_base_info.auc_user_mac_addr,
                     pst_roam_info->st_connect.pst_bss_dscr->auc_bssid);

    us_auth_len = hmac_mgmt_encap_auth_req_etc(pst_hmac_vap, (oal_uint8 *)(OAL_NETBUF_HEADER(pst_auth_frame)));
    if (us_auth_len < OAL_AUTH_IE_OFFSET) {
        OAM_WARNING_LOG0(uc_vap_id, OAM_SF_ROAM, "{hmac_roam_send_auth_seq1::hmac_mgmt_encap_auth_req_etc failed.}");
        oal_netbuf_free(pst_auth_frame);
        return OAL_ERR_CODE_ROAM_FRAMER_LEN;
    }

    oal_netbuf_put(pst_auth_frame, us_auth_len);

    /* 为填写发送描述符准备参数 */
    pst_tx_ctl = (mac_tx_ctl_stru *)oal_netbuf_cb(pst_auth_frame);
    MAC_GET_CB_MPDU_LEN(pst_tx_ctl) = us_auth_len;
    MAC_GET_CB_TX_USER_IDX(pst_tx_ctl) = pst_hmac_user->st_user_base_info.us_assoc_id;
    MAC_GET_CB_NETBUF_NUM(pst_tx_ctl) = 1;

    /* 抛事件让dmac将该帧发送 */
    ul_ret = hmac_tx_mgmt_send_event_etc(&pst_hmac_vap->st_vap_base_info, pst_auth_frame, us_auth_len);
    if (ul_ret != OAL_SUCC) {
        oal_netbuf_free(pst_auth_frame);
        OAM_ERROR_LOG1(uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_send_auth_seq1::hmac_tx_mgmt_send_event_etc failed[%d].}", ul_ret);
        return ul_ret;
    }

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_AUTH_COMP);

    /* 启动认证超时定时器 */
    hmac_roam_connect_start_timer(pst_roam_info, ROAM_AUTH_TIME_MAX);

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_roam_send_reassoc_req(hmac_roam_info_stru *pst_roam_info)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    hmac_user_stru *pst_hmac_user = pst_roam_info->pst_hmac_user;
    mac_vap_stru *pst_mac_vap;
    oal_netbuf_stru *pst_assoc_req_frame;
    mac_tx_ctl_stru *pst_tx_ctl = OAL_PTR_NULL;
    oal_uint32 ul_assoc_len;

    pst_mac_vap = &pst_hmac_vap->st_vap_base_info;

    pst_assoc_req_frame = OAL_MEM_NETBUF_ALLOC(OAL_NORMAL_NETBUF, WLAN_MEM_NETBUF_SIZE2, OAL_NETBUF_PRIORITY_MID);
    if (OAL_ANY_NULL_PTR2(pst_hmac_user, pst_assoc_req_frame)) {
        OAM_ERROR_LOG2(pst_mac_vap->uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_send_reassoc_req::pst_hmac_user[%d] or pst_assoc_req_frame[%d] is NULL.}",
                       (uintptr_t)pst_hmac_user, (uintptr_t)pst_assoc_req_frame);
        return OAL_ERR_CODE_PTR_NULL;
    }
    OAL_MEM_NETBUF_TRACE(pst_assoc_req_frame, OAL_TRUE);

    memset_s(oal_netbuf_cb(pst_assoc_req_frame), OAL_NETBUF_CB_SIZE(), 0, OAL_NETBUF_CB_SIZE());

    /* 将mac header清零 */
    memset_s((oal_uint8 *)oal_netbuf_header(pst_assoc_req_frame), MAC_80211_FRAME_LEN, 0, MAC_80211_FRAME_LEN);

    pst_hmac_vap->bit_reassoc_flag = OAL_TRUE;

    ul_assoc_len = hmac_mgmt_encap_asoc_req_sta_etc(pst_hmac_vap,
        (oal_uint8 *)(OAL_NETBUF_HEADER(pst_assoc_req_frame)),
        pst_roam_info->st_old_bss.auc_bssid);

    oal_netbuf_put(pst_assoc_req_frame, ul_assoc_len);

    /* 帧长异常 */
    if (ul_assoc_len <= OAL_ASSOC_REQ_IE_OFFSET) {
        OAM_ERROR_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_send_reassoc_req::unexpected assoc len[%d].}",
                       ul_assoc_len);
        oal_netbuf_free(pst_assoc_req_frame);
        return OAL_FAIL;
    }
    hmac_user_free_asoc_req_ie(pst_hmac_user->st_user_base_info.us_assoc_id);

    /* 记录关联请求帧的部分内容，用于上报给内核 */
    ul_ret = hmac_user_set_asoc_req_ie(pst_hmac_user,
                                       OAL_NETBUF_HEADER(pst_assoc_req_frame) + OAL_ASSOC_REQ_IE_OFFSET,
                                       ul_assoc_len - OAL_ASSOC_REQ_IE_OFFSET,
                                       OAL_TRUE);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_send_reassoc_req::hmac_user_set_asoc_req_ie failed}");
        oal_netbuf_free(pst_assoc_req_frame);
        return OAL_FAIL;
    }

    /* 为填写发送描述符准备参数 */
    pst_tx_ctl = (mac_tx_ctl_stru *)oal_netbuf_cb(pst_assoc_req_frame);
    MAC_GET_CB_MPDU_LEN(pst_tx_ctl) = (oal_uint16)ul_assoc_len;
    MAC_GET_CB_TX_USER_IDX(pst_tx_ctl) = pst_hmac_user->st_user_base_info.us_assoc_id;
    MAC_GET_CB_NETBUF_NUM(pst_tx_ctl) = 1;

    /* 抛事件让dmac将该帧发送 */
    ul_ret = hmac_tx_mgmt_send_event_etc(&pst_hmac_vap->st_vap_base_info,
                                         pst_assoc_req_frame,
                                         (oal_uint16)ul_assoc_len);
    if (ul_ret != OAL_SUCC) {
        oal_netbuf_free(pst_assoc_req_frame);
        hmac_user_free_asoc_req_ie(pst_hmac_user->st_user_base_info.us_assoc_id);
        OAM_ERROR_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_send_reassoc_req::hmac_tx_mgmt_send_event_etc failed[%d].}", ul_ret);
        return ul_ret;
    }

    /* 启动关联超时定时器 */
    hmac_roam_connect_start_timer(pst_roam_info, ROAM_ASSOC_TIME_MAX);

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_ASSOC_COMP);

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 hmac_roam_process_auth_seq2(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    dmac_wlan_crx_event_stru *pst_crx_event = OAL_PTR_NULL;
    mac_rx_ctl_stru *pst_rx_ctrl = OAL_PTR_NULL;
    oal_uint8 *puc_mac_hdr = OAL_PTR_NULL;
    oal_uint8 auc_bssid[WLAN_MAC_ADDR_LEN] = { 0 };
    oal_uint16 us_auth_status;
    oal_uint8 uc_frame_sub_type;
    oal_uint16 us_auth_seq_num;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_AUTH_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_process_auth_seq2::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    pst_hmac_user = pst_roam_info->pst_hmac_user;

    pst_crx_event = (dmac_wlan_crx_event_stru *)p_param;
    pst_rx_ctrl = (mac_rx_ctl_stru *)oal_netbuf_cb(pst_crx_event->pst_netbuf);
    puc_mac_hdr = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(pst_rx_ctrl);

    mac_get_address3(puc_mac_hdr, auc_bssid, WLAN_MAC_ADDR_LEN);
    if (oal_compare_mac_addr(pst_hmac_user->st_user_base_info.auc_user_mac_addr, auc_bssid)) {
        return OAL_SUCC;
    }

    uc_frame_sub_type = mac_get_frame_type_and_subtype(puc_mac_hdr);
    us_auth_seq_num = mac_get_auth_seq_num(puc_mac_hdr);
    us_auth_status = mac_get_auth_status(puc_mac_hdr);

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#ifdef _PRE_WLAN_WAKEUP_SRC_PARSE
    if (OAL_TRUE == wlan_pm_wkup_src_debug_get()) {
        wlan_pm_wkup_src_debug_set(OAL_FALSE);
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                         "{wifi_wake_src:hmac_roam_process_auth_seq2::wakeup mgmt type[0x%x]}",
                         uc_frame_sub_type);
    }
#endif
#endif

    /* auth_seq2帧校验，错误帧不处理，在超时中统一处理 */
    if ((WLAN_FC0_SUBTYPE_AUTH | WLAN_FC0_TYPE_MGT) != uc_frame_sub_type) {
        return OAL_SUCC;
    }

#ifdef _PRE_WLAN_FEATURE_SAE
    if (mac_mib_get_AuthenticationMode(&(pst_hmac_vap->st_vap_base_info)) == WLAN_WITP_AUTH_SAE) {
        OAM_WARNING_LOG2(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                         "{hmac_roam_process_auth_seq2::rx sae auth frame, status_code %d, seq_num %d.}",
                         mac_get_auth_status(puc_mac_hdr),
                         mac_get_auth_seq_num(puc_mac_hdr));

        hmac_rx_mgmt_send_to_host_etc(pst_hmac_vap, pst_crx_event->pst_netbuf);

        return OAL_SUCC;
    }
#endif

    if ((us_auth_seq_num != WLAN_AUTH_TRASACTION_NUM_TWO) || (us_auth_status != MAC_SUCCESSFUL_STATUSCODE)) {
        return OAL_SUCC;
    }

#ifdef _PRE_WLAN_FEATURE_11R
    if (pst_hmac_vap->bit_11r_enable == OAL_TRUE) {
        if (WLAN_WITP_AUTH_FT == mac_get_auth_alg(puc_mac_hdr)) {
#ifdef _PRE_WLAN_1103_CHR
            pst_roam_info->st_static.uc_roam_mode = HMAC_CHR_OVER_THE_AIR;
#endif
            /* 上报FT成功消息给APP，以便APP下发新的FT_IE用于发送reassociation */
            ul_ret = hmac_roam_ft_notify_wpas(pst_hmac_vap, puc_mac_hdr, pst_rx_ctrl->us_frame_len);
            if (ul_ret != OAL_SUCC) {
                OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                               "{hmac_roam_process_auth_seq2::hmac_roam_ft_notify_wpas failed[%d].}", ul_ret);
                return ul_ret;
            }

            hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_WAIT_ASSOC_COMP);
            /* 启动关联超时定时器 */
            hmac_roam_connect_start_timer(pst_roam_info, ROAM_ASSOC_TIME_MAX);
            return OAL_SUCC;
        }
    }
#endif  // _PRE_WLAN_FEATURE_11R

    if (WLAN_WITP_AUTH_OPEN_SYSTEM != mac_get_auth_alg(puc_mac_hdr)) {
        return OAL_SUCC;
    }

    /* 发送关联请求 */
#ifdef _PRE_WLAN_1103_CHR
    pst_roam_info->st_static.uc_roam_mode = HMAC_CHR_ROAM_NORMAL;
#endif
    ul_ret = hmac_roam_send_reassoc_req(pst_roam_info);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_process_auth_seq2::hmac_roam_send_assoc_req failed[%d].}", ul_ret);
        return ul_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_11K

OAL_STATIC oal_uint32  hmac_sta_up_send_neighbor_req(hmac_roam_info_stru *pst_roam_info)
{
    wal_msg_write_stru             st_write_msg;
    oal_int32                      l_ret;
    mac_cfg_ssid_param_stru       *pst_ssid = OAL_PTR_NULL;
    oal_net_device_stru           *pst_net_dev = OAL_PTR_NULL;
    hmac_vap_stru                 *pst_hmac_vap = OAL_PTR_NULL;

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_sta_up_send_neighbor_req::pst_hmac_vap null!}");
        return OAL_ERR_CODE_ROAM_INVALID_VAP;
    }
    pst_net_dev = pst_hmac_vap->pst_net_device;

    if (pst_roam_info->pst_hmac_vap->bit_nb_rpt_11k == OAL_FALSE ||
        pst_roam_info->pst_hmac_vap->bit_11k_enable == OAL_FALSE ||
        hmac_roam_is_neighbor_report_allowed(pst_roam_info->pst_hmac_vap) == OAL_FALSE) {
        OAM_WARNING_LOG2(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_sta_up_send_neighbor_req::not allowed,nb_rpt_11k=[%d],11k_enable=[%d]!}",
                       pst_roam_info->pst_hmac_vap->bit_nb_rpt_11k,
                       pst_roam_info->pst_hmac_vap->bit_11k_enable);
        return OAL_FAIL;
    }

    /***************************************************************************
                                抛事件到wal层处理
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SEND_NEIGHBOR_REQ, OAL_SIZEOF(mac_cfg_ssid_param_stru));
    pst_ssid = (mac_cfg_ssid_param_stru *)st_write_msg.auc_value;
    pst_ssid->uc_ssid_len = 0;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_ssid_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{hmac_sta_up_send_neighbor_req::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 hmac_roam_process_assoc_rsp(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    dmac_wlan_crx_event_stru *pst_crx_event = OAL_PTR_NULL;
    mac_rx_ctl_stru *pst_rx_ctrl = OAL_PTR_NULL;
    oal_uint8 *puc_mac_hdr = OAL_PTR_NULL;
    oal_uint8 *puc_payload = OAL_PTR_NULL;
    oal_uint16 us_msg_len;
    oal_uint16 us_hdr_len;
    mac_status_code_enum_uint16 en_asoc_status;
    oal_uint8 auc_bss_addr[WLAN_MAC_ADDR_LEN];
    oal_uint8 uc_frame_sub_type;

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    pst_hmac_user = pst_roam_info->pst_hmac_user;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_ASSOC_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_process_assoc_rsp::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_crx_event = (dmac_wlan_crx_event_stru *)p_param;
    pst_rx_ctrl = (mac_rx_ctl_stru *)oal_netbuf_cb(pst_crx_event->pst_netbuf);
    puc_mac_hdr = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(pst_rx_ctrl);
    puc_payload = puc_mac_hdr + pst_rx_ctrl->uc_mac_header_len;
    us_msg_len = pst_rx_ctrl->us_frame_len - pst_rx_ctrl->uc_mac_header_len;
    us_hdr_len = pst_rx_ctrl->uc_mac_header_len;

    /* mac地址校验 */
    mac_get_address3(puc_mac_hdr, auc_bss_addr, WLAN_MAC_ADDR_LEN);
    if (oal_compare_mac_addr(pst_hmac_user->st_user_base_info.auc_user_mac_addr, auc_bss_addr)) {
        return OAL_SUCC;
    }

    /* assoc帧校验，错误帧处理 */
    uc_frame_sub_type = mac_get_frame_type_and_subtype(puc_mac_hdr);
    en_asoc_status = mac_get_asoc_status(puc_payload);
    hmac_roam_connect_pm_wkup(pst_roam_info, p_param);

    if ((uc_frame_sub_type != (WLAN_FC0_SUBTYPE_REASSOC_RSP | WLAN_FC0_TYPE_MGT)) &&
        (uc_frame_sub_type != (WLAN_FC0_SUBTYPE_ASSOC_RSP | WLAN_FC0_TYPE_MGT))) {
        return OAL_SUCC;
    }

    /* 关联响应帧长度校验 */
    if (pst_rx_ctrl->us_frame_len <= OAL_ASSOC_RSP_IE_OFFSET) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                         "{hmac_roam_process_assoc_rsp::rsp ie length error, us_frame_len[%d].}",
                         pst_rx_ctrl->us_frame_len);
        return OAL_ERR_CODE_ROAM_FRAMER_LEN;
    }

    pst_roam_info->st_connect.en_status_code = MAC_SUCCESSFUL_STATUSCODE;
    if (en_asoc_status != MAC_SUCCESSFUL_STATUSCODE) {
        pst_roam_info->st_connect.en_status_code = en_asoc_status;
        return OAL_SUCC;
    }

    ul_ret = hmac_process_assoc_rsp_etc(pst_hmac_vap, pst_hmac_user, puc_mac_hdr, us_hdr_len, puc_payload, us_msg_len);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                         "{hmac_roam_process_assoc_rsp::hmac_process_assoc_rsp_etc failed[%d].}", ul_ret);
        return ul_ret;
    }

    /* user已经关联上，抛事件给DMAC，在DMAC层挂用户算法钩子 */
    hmac_user_add_notify_alg_etc(&(pst_hmac_vap->st_vap_base_info), pst_hmac_user->st_user_base_info.us_assoc_id);

    /* 上报关联成功消息给APP */
    ul_ret = hmac_roam_connect_notify_wpas(pst_roam_info, puc_mac_hdr, pst_rx_ctrl->us_frame_len);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_process_assoc_rsp::hmac_roam_connect_notify_wpas failed[%d].}", ul_ret);
        return ul_ret;
    }

/* 在STA上线后立即发送NR Req以获取邻居AP channel信息 */
#ifdef _PRE_WLAN_FEATURE_11K
    ul_ret = hmac_sta_up_send_neighbor_req(pst_roam_info);
#endif

    if (OAL_TRUE != mac_mib_get_privacyinvoked(&pst_hmac_vap->st_vap_base_info)) {
        /* 非加密情况下，漫游成功 */
        hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_HANDSHAKING);
        hmac_roam_connect_succ(pst_roam_info, OAL_PTR_NULL);
    } else {
#ifdef _PRE_WLAN_FEATURE_11R
        if (pst_hmac_vap->bit_11r_enable == OAL_TRUE) {
            if (OAL_TRUE == mac_mib_get_ft_trainsistion(&pst_hmac_vap->st_vap_base_info)) {
                /* FT情况下，漫游成功 */
                hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_HANDSHAKING);
                hmac_roam_connect_succ(pst_roam_info, OAL_PTR_NULL);
                return OAL_SUCC;
            }
        }
#endif  // _PRE_WLAN_FEATURE_11R
        if (OAL_TRUE == mac_mib_get_rsnaactivated(&pst_hmac_vap->st_vap_base_info)) {
            hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_HANDSHAKING);
            /* 启动握手超时定时器 */
            hmac_roam_connect_start_timer(pst_roam_info, ROAM_HANDSHAKE_TIME_MAX);
        } else {
            /* 非 WPA 或者 WPA2 加密情况下(WEP_OPEN/WEP_SHARED)，漫游成功 */
            hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_HANDSHAKING);
            hmac_roam_connect_succ(pst_roam_info, OAL_PTR_NULL);
        }
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_roam_process_action(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    hmac_user_stru *pst_hmac_user = OAL_PTR_NULL;
    dmac_wlan_crx_event_stru *pst_crx_event = OAL_PTR_NULL;
    mac_rx_ctl_stru *pst_rx_ctrl = OAL_PTR_NULL;
    oal_uint8 *puc_mac_hdr = OAL_PTR_NULL;
    oal_uint8 *puc_payload = OAL_PTR_NULL;
    oal_uint8 auc_bss_addr[WLAN_MAC_ADDR_LEN];
    oal_uint8 uc_frame_sub_type;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_HANDSHAKING);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_process_action::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    pst_hmac_user = pst_roam_info->pst_hmac_user;

    pst_crx_event = (dmac_wlan_crx_event_stru *)p_param;
    pst_rx_ctrl = (mac_rx_ctl_stru *)oal_netbuf_cb(pst_crx_event->pst_netbuf);
    puc_mac_hdr = (oal_uint8 *)MAC_GET_RX_CB_MAC_HEADER_ADDR(pst_rx_ctrl);
    puc_payload = puc_mac_hdr + pst_rx_ctrl->uc_mac_header_len;

    /* mac地址校验 */
    mac_get_address3(puc_mac_hdr, auc_bss_addr, WLAN_MAC_ADDR_LEN);
    if (oal_compare_mac_addr(pst_hmac_user->st_user_base_info.auc_user_mac_addr, auc_bss_addr)) {
        return OAL_SUCC;
    }

    uc_frame_sub_type = mac_get_frame_type_and_subtype(puc_mac_hdr);

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#ifdef _PRE_WLAN_WAKEUP_SRC_PARSE
    if (OAL_TRUE == wlan_pm_wkup_src_debug_get()) {
        wlan_pm_wkup_src_debug_set(OAL_FALSE);
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_RX,
                         "{wifi_wake_src:hmac_roam_process_auth_seq2::wakeup mgmt type[0x%x]}",
                         uc_frame_sub_type);
    }
#endif
#endif

    if ((WLAN_FC0_SUBTYPE_ACTION | WLAN_FC0_TYPE_MGT) != uc_frame_sub_type) {
        return OAL_SUCC;
    }

    if (puc_payload[MAC_ACTION_OFFSET_CATEGORY] == MAC_ACTION_CATEGORY_BA) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                         "{hmac_roam_process_action::BA_ACTION_TYPE [%d].}", puc_payload[MAC_ACTION_OFFSET_ACTION]);
        switch (puc_payload[MAC_ACTION_OFFSET_ACTION]) {
            case MAC_BA_ACTION_ADDBA_REQ:
                ul_ret = hmac_mgmt_rx_addba_req_etc(pst_hmac_vap, pst_hmac_user, puc_payload);
                break;

            case MAC_BA_ACTION_ADDBA_RSP:
                ul_ret = hmac_mgmt_rx_addba_rsp_etc(pst_hmac_vap, pst_hmac_user, puc_payload);
                break;

            case MAC_BA_ACTION_DELBA:
                ul_ret = hmac_mgmt_rx_delba_etc(pst_hmac_vap, pst_hmac_user, puc_payload);
                break;

            default:
                break;
        }
    }

    return ul_ret;
}


OAL_STATIC oal_uint32 hmac_roam_connect_succ(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_HANDSHAKING);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_connect_succ::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_UP);

    /* 删除定时器 */
    hmac_roam_connect_del_timer(pst_roam_info);

    /* 通知ROAM主状态机 */
    hmac_roam_connect_complete_etc(pst_roam_info->pst_hmac_vap, OAL_SUCC);

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 hmac_roam_auth_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_AUTH_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_auth_timeout::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;

    if (++pst_roam_info->st_connect.uc_auth_num >= MAX_AUTH_CNT) {
        return hmac_roam_connect_fail(pst_roam_info, p_param);
    }

#ifdef _PRE_WLAN_FEATURE_SAE
    if (mac_mib_get_AuthenticationMode(&(pst_hmac_vap->st_vap_base_info)) == WLAN_WITP_AUTH_SAE) {
        ul_ret = hmac_roam_triger_sae_auth(pst_roam_info);
        return ul_ret;
    }
#endif

    ul_ret = hmac_roam_send_auth_seq1(pst_roam_info, p_param);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_auth_timeout::hmac_roam_send_auth_seq1 failed[%d].}", ul_ret);
    }

    return ul_ret;
}


OAL_STATIC oal_uint32 hmac_roam_assoc_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_ASSOC_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_assoc_timeout::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    if ((++pst_roam_info->st_connect.uc_assoc_num >= MAX_ASOC_CNT) ||
        (pst_roam_info->st_connect.en_status_code == MAC_REJECT_TEMP)) {
        return hmac_roam_connect_fail(pst_roam_info, p_param);
    }

    ul_ret = hmac_roam_send_reassoc_req(pst_roam_info);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ROAM, "{hmac_roam_assoc_timeout::hmac_roam_send_reassoc_req failed[%d].}", ul_ret);
    }
    return ul_ret;
}


OAL_STATIC oal_uint32 hmac_roam_handshaking_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_ROAMING,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_HANDSHAKING);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_handshaking_timeout::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    return hmac_roam_connect_fail(pst_roam_info, p_param);
}

#ifdef _PRE_WLAN_FEATURE_11R

OAL_STATIC oal_uint32 hmac_roam_ft_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32 ul_ret;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_UP,
                                           ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_FT_COMP);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_ft_timeout::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;

    if (pst_hmac_vap->bit_11r_enable != OAL_TRUE) {
        return OAL_SUCC;
    }

    if (++pst_roam_info->st_connect.uc_ft_num >= MAX_AUTH_CNT) {
        return hmac_roam_connect_fail(pst_roam_info, pst_roam_info->st_connect.pst_bss_dscr);
    }

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_INIT);

    OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                     "{hmac_roam_ft_timeout::change to ft over the air!}");
    pst_roam_info->st_connect.uc_ft_force_air = OAL_TRUE;
    pst_roam_info->st_connect.uc_ft_failed = OAL_TRUE;
    ul_ret = hmac_roam_connect_ft_ds_change_to_air_etc(pst_hmac_vap, pst_roam_info->st_connect.pst_bss_dscr);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                         "{hmac_roam_ft_timeout::hmac_roam_connect_ft_ds_change_to_air_etc failed[%d].}", ul_ret);
        return hmac_roam_connect_fail(pst_roam_info, pst_roam_info->st_connect.pst_bss_dscr);
    }
    return ul_ret;
}


OAL_STATIC oal_uint32 hmac_roam_ft_preauth_timeout(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    oal_uint32              ul_ret;
    hmac_vap_stru          *pst_hmac_vap = OAL_PTR_NULL;

    ul_ret = hmac_roam_connect_check_state(pst_roam_info, MAC_VAP_STATE_UP,
        ROAM_MAIN_STATE_CONNECTING, ROAM_CONNECT_STATE_WAIT_PREAUTH_COMP);

    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{hmac_roam_ft_preauth_timeout::check_state fail[%d]!}", ul_ret);
        return ul_ret;
    }

    pst_hmac_vap = pst_roam_info->pst_hmac_vap;

    if (pst_hmac_vap->bit_11r_enable != OAL_TRUE) {
        return OAL_SUCC;
    }

    if (++pst_roam_info->st_connect.uc_ft_num >= MAX_PREAUTH_CNT) {
        return hmac_roam_connect_fail(pst_roam_info, pst_roam_info->st_connect.pst_bss_dscr);
    }

    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_INIT);

    OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
        "{hmac_roam_ft_preauth_timeout::change to ft over the air!}");
    pst_roam_info->st_connect.uc_ft_force_air = OAL_TRUE;
    pst_roam_info->st_connect.uc_ft_failed = OAL_TRUE;
    ul_ret = hmac_roam_connect_ft_ds_change_to_air_etc(pst_hmac_vap, pst_roam_info->st_connect.pst_bss_dscr);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
            "{hmac_roam_ft_preauth_timeout::hmac_roam_connect_ft_ds_change_to_air_etc failed[%d].}",
            ul_ret);
        return hmac_roam_connect_fail(pst_roam_info, pst_roam_info->st_connect.pst_bss_dscr);
    }
    return ul_ret;
}

#endif  // _PRE_WLAN_FEATURE_11R

OAL_STATIC oal_uint32 hmac_roam_connect_fail(hmac_roam_info_stru *pst_roam_info, oal_void *p_param)
{
    hmac_vap_stru *pst_hmac_vap = pst_roam_info->pst_hmac_vap;
    roam_connect_state_enum_uint8 connect_state = pst_roam_info->st_connect.en_state;
    /* connect状态切换 */
    hmac_roam_connect_change_state(pst_roam_info, ROAM_CONNECT_STATE_FAIL);

    /* connect失败时，需要添加到黑名单 */
    hmac_roam_alg_add_blacklist_etc(pst_roam_info,
                                    pst_roam_info->st_connect.pst_bss_dscr->auc_bssid,
                                    ROAM_BLACKLIST_TYPE_REJECT_AP);
    OAM_WARNING_LOG0(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                     "{hmac_roam_connect_fail::hmac_roam_alg_add_blacklist_etc!}");

    /* 通知ROAM主状态机，BSS回退由主状态机完成 */

    if (connect_state == ROAM_CONNECT_STATE_HANDSHAKING) {
        hmac_roam_connect_complete_etc(pst_hmac_vap, OAL_ERR_CODE_ROAM_HANDSHAKE_FAIL);
    } else if (connect_state == ROAM_CONNECT_STATE_WAIT_ASSOC_COMP ||
               connect_state == ROAM_CONNECT_STATE_WAIT_AUTH_COMP) {
        hmac_roam_connect_complete_etc(pst_hmac_vap, OAL_ERR_CODE_ROAM_NO_RESPONSE);
    } else {
        hmac_roam_connect_complete_etc(pst_hmac_vap, OAL_FAIL);
    }

    return OAL_SUCC;
}

oal_uint32 hmac_roam_connect_start_etc(hmac_vap_stru *pst_hmac_vap, mac_bss_dscr_stru *pst_bss_dscr)
{
    hmac_roam_info_stru *pst_roam_info = OAL_PTR_NULL;

    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_start_etc::vap null!}");
        return OAL_ERR_CODE_ROAM_INVALID_VAP;
    }

    pst_roam_info = (hmac_roam_info_stru *)pst_hmac_vap->pul_roam_info;
    if (pst_roam_info == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_start_etc::roam_info null!}");
        return OAL_ERR_CODE_ROAM_INVALID_VAP;
    }

    /* 漫游开关没有开时，不处理tbtt中断 */
    if (pst_roam_info->uc_enable == 0) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_start_etc::roam disabled!}");
        return OAL_ERR_CODE_ROAM_DISABLED;
    }

    pst_roam_info->st_connect.pst_bss_dscr = pst_bss_dscr;
    pst_roam_info->st_connect.uc_auth_num = 0;
    pst_roam_info->st_connect.uc_assoc_num = 0;
    pst_roam_info->st_connect.uc_ft_num = 0;

#ifdef _PRE_WLAN_FEATURE_11R
    if (pst_hmac_vap->bit_11r_enable == OAL_TRUE) {
        if (OAL_TRUE == (mac_mib_get_ft_trainsistion(&pst_hmac_vap->st_vap_base_info) &&
            mac_mib_get_ft_over_ds(&pst_hmac_vap->st_vap_base_info)) &&
            (pst_hmac_vap->bit_11r_over_ds == OAL_TRUE) &&
            (pst_roam_info->st_connect.uc_ft_force_air != OAL_TRUE)) {
            return hmac_roam_connect_fsm_action_etc(pst_roam_info,
                                                    ROAM_CONNECT_FSM_EVENT_FT_OVER_DS,
                                                    (oal_void *)pst_bss_dscr);
        }
    } else {
        OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_start_etc::11r NOT Enabled}");
    }
#endif  // _PRE_WLAN_FEATURE_11R

    return hmac_roam_connect_fsm_action_etc(pst_roam_info, ROAM_CONNECT_FSM_EVENT_START, (oal_void *)pst_bss_dscr);
}


oal_uint32 hmac_roam_connect_stop_etc(hmac_vap_stru *pst_hmac_vap)
{
    hmac_roam_info_stru *pst_roam_info = OAL_PTR_NULL;

    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_start_etc::vap null!}");
        return OAL_ERR_CODE_ROAM_INVALID_VAP;
    }

    pst_roam_info = (hmac_roam_info_stru *)pst_hmac_vap->pul_roam_info;
    if (pst_roam_info == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_start_etc::roam_info null!}");
        return OAL_ERR_CODE_ROAM_INVALID_VAP;
    }
    pst_roam_info->st_connect.en_state = ROAM_CONNECT_STATE_INIT;
    return OAL_SUCC;
}


oal_void hmac_roam_connect_rx_mgmt_etc(hmac_vap_stru *pst_hmac_vap, dmac_wlan_crx_event_stru *pst_crx_event)
{
    hmac_roam_info_stru *pst_roam_info = OAL_PTR_NULL;
    oal_uint32 ul_ret;

    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_rx_mgmt_etc::vap null!}");
        return;
    }

    pst_roam_info = (hmac_roam_info_stru *)pst_hmac_vap->pul_roam_info;
    if (pst_roam_info == OAL_PTR_NULL) {
        return;
    }

    /* 漫游开关没有开时，不处理管理帧接收 */
    if (pst_roam_info->uc_enable == 0) {
        return;
    }

    ul_ret = hmac_roam_connect_fsm_action_etc(pst_roam_info, ROAM_CONNECT_FSM_EVENT_MGMT_RX, (oal_void *)pst_crx_event);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                         "{hmac_roam_connect_rx_mgmt_etc::MGMT_RX FAIL[%d]!}", ul_ret);
    }

    return;
}


oal_void hmac_roam_connect_key_done_etc(hmac_vap_stru *pst_hmac_vap)
{
    hmac_roam_info_stru *pst_roam_info = OAL_PTR_NULL;
    oal_uint32 ul_ret;

    if (pst_hmac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_key_done_etc::vap null!}");
        return;
    }

    pst_roam_info = (hmac_roam_info_stru *)pst_hmac_vap->pul_roam_info;
    if (pst_roam_info == OAL_PTR_NULL) {
        return;
    }

    /* 漫游开关没有开时，不处理管理帧接收 */
    if (pst_roam_info->uc_enable == 0) {
        return;
    }

    /* 主状态机为非CONNECTING状态/CONNECT状态机为非UP状态，失败 */
    if (pst_roam_info->en_main_state != ROAM_MAIN_STATE_CONNECTING) {
        return;
    }

    ul_ret = hmac_roam_connect_fsm_action_etc(pst_roam_info, ROAM_CONNECT_FSM_EVENT_KEY_DONE, OAL_PTR_NULL);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_connect_key_done_etc::KEY_DONE FAIL[%d]!}", ul_ret);
    }
    OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{hmac_roam_connect_key_done_etc::KEY_DONE !}");

    return;
}

#endif  // _PRE_WLAN_FEATURE_ROAM
#ifdef _PRE_WLAN_FEATURE_11R

oal_uint32 hmac_roam_connect_ft_reassoc_etc(hmac_vap_stru *pst_hmac_vap)
{
    hmac_roam_info_stru *pst_roam_info = OAL_PTR_NULL;
    hmac_join_req_stru st_join_req;
    oal_uint32 ul_ret;
    mac_bss_dscr_stru *pst_bss_dscr;
    oal_uint8 uc_rate_num;

    pst_roam_info = (hmac_roam_info_stru *)pst_hmac_vap->pul_roam_info;
    pst_bss_dscr = pst_roam_info->st_connect.pst_bss_dscr;

    if (pst_roam_info->pst_hmac_user != OAL_PTR_NULL) {
        oal_set_mac_addr(pst_roam_info->pst_hmac_user->st_user_base_info.auc_user_mac_addr, pst_bss_dscr->auc_bssid);
    }

    uc_rate_num = (pst_bss_dscr->uc_num_supp_rates < WLAN_MAX_SUPP_RATES) ?
        pst_bss_dscr->uc_num_supp_rates : WLAN_MAX_SUPP_RATES;
    if (EOK != memcpy_s(pst_hmac_vap->auc_supp_rates, WLAN_MAX_SUPP_RATES,
                        pst_bss_dscr->auc_supp_rates, uc_rate_num)) {
        OAM_ERROR_LOG0(0, OAM_SF_SCAN, "hmac_roam_connect_ft_reassoc_etc::memcpy fail!");
        return OAL_FAIL;
    }
    mac_mib_set_SupportRateSetNums(&pst_hmac_vap->st_vap_base_info, pst_bss_dscr->uc_num_supp_rates);

    if (mac_mib_get_ft_over_ds(&pst_hmac_vap->st_vap_base_info) && (pst_hmac_vap->bit_11r_over_ds == OAL_TRUE)) {
        /* 配置join参数 */
        hmac_prepare_join_req_etc(&st_join_req, pst_bss_dscr);

        ul_ret = hmac_sta_update_join_req_params_etc(pst_hmac_vap, &st_join_req);
        if (ul_ret != OAL_SUCC) {
            OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_SCAN,
                           "{hmac_roam_connect_ft_reassoc_etc::hmac_sta_update_join_req_params_etc fail[%d].}", ul_ret);
            return ul_ret;
        }
    }
    /* 发送关联请求 */
    ul_ret = hmac_roam_send_reassoc_req(pst_roam_info);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
                       "{hmac_roam_connect_ft_reassoc_etc::hmac_roam_send_assoc_req failed[%d].}", ul_ret);
        return ul_ret;
    }

    return OAL_SUCC;
}


oal_uint32 hmac_roam_connect_ft_preauth_etc(hmac_vap_stru *pst_hmac_vap)
{
    hmac_roam_info_stru                             *pst_roam_info;
    oal_uint32                                       ul_ret;
    mac_bss_dscr_stru                               *pst_bss_dscr;

    pst_roam_info = (hmac_roam_info_stru *)pst_hmac_vap->pul_roam_info;
    pst_bss_dscr = pst_roam_info->st_connect.pst_bss_dscr;

    /* 发送关联请求 */
    ul_ret = hmac_roam_send_ft_preauth_req(pst_roam_info, pst_roam_info->st_connect.pst_bss_dscr);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(pst_hmac_vap->st_vap_base_info.uc_vap_id, OAM_SF_ROAM,
            "{hmac_roam_connect_ft_preauth_etc::hmac_roam_send_preauth_req failed[%d].}", ul_ret);
        return ul_ret;
    }

    return OAL_SUCC;
}

#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

