

#ifdef _PRE_WLAN_CFGID_DEBUG

/* 1 ͷ�ļ����� */
#include "oal_ext_if.h"
#include "oal_profiling.h"
#include "oal_kernel_file.h"
#include "oal_cfg80211.h"

#include "oam_ext_if.h"
#include "frw_ext_if.h"

#include "wlan_spec.h"
#include "wlan_types.h"

#include "mac_vap.h"
#include "mac_resource.h"
#include "mac_regdomain.h"
#include "mac_ie.h"
#include "hmac_ext_if.h"
#include "hmac_chan_mgmt.h"
#include "wal_main.h"
#include "wal_ext_if.h"
#include "wal_config.h"
#include "wal_regdb.h"
#include "wal_linux_scan.h"
#include "wal_linux_ioctl.h"
#include "wal_linux_bridge.h"
#include "wal_linux_flowctl.h"
#include "wal_linux_atcmdsrv.h"
#include "wal_linux_event.h"
#include "hmac_resource.h"
#include "hmac_p2p.h"

#ifdef _PRE_WLAN_FEATURE_P2P
#include "wal_linux_cfg80211.h"
#endif

#include "wal_dfx.h"

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#include "oal_hcc_host_if.h"
#include "plat_cali.h"
#endif

#ifdef _PRE_WLAN_FEATURE_ARP_OFFLOAD
#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#include <linux/notifier.h>
#include <linux/inetdevice.h>
#include <net/addrconf.h>
#endif
#include "hmac_arp_offload.h"
#endif

#include "hmac_auto_adjust_freq.h"

#ifdef _PRE_WLAN_FEATURE_ROAM
#include "hmac_roam_main.h"
#endif  // _PRE_WLAN_FEATURE_ROAM
#ifdef _PRE_PLAT_FEATURE_CUSTOMIZE
#include "hisi_customize_wifi.h"
#endif /* #ifdef _PRE_PLAT_FEATURE_CUSTOMIZE */

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE) && (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#include "plat_pm_wlan.h"
#endif

#ifdef _PRE_MEM_TRACE
#include "mem_trace.h"
#endif

#if defined(WIN32) && (_PRE_PRODUCT_ID == _PRE_PRODUCT_ID_HI1103_HOST)
#include "hisi_customize_wifi_hi110x.h"
#endif

#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#include "board.h"
#endif
#include "securec.h"
#include "securectype.h"
#ifdef _PRE_WLAN_FEATURE_11AX
#include "hmac_wifi6_self_cure.h"
#endif

#undef THIS_FILE_ID
#define THIS_FILE_ID      OAM_FILE_ID_WAL_LINUX_IOCTL_DEBUG_C
#define MAX_PRIV_CMD_SIZE 4096

/* 2 �ṹ�嶨�� */
/* 3 ȫ�ֱ������� */
#ifdef _PRE_WLAN_FEATURE_DFR
extern hmac_dfr_info_stru g_st_dfr_info_etc;
#endif  // _PRE_WLAN_FEATURE_DFR
extern OAL_CONST oal_int8 *pauc_tx_dscr_param_name_etc[];
extern OAL_CONST oal_int8 *pauc_tx_pow_param_name[];
extern OAL_CONST wal_ioctl_alg_cfg_stru g_ast_alg_cfg_map_etc[];
extern OAL_CONST wal_ioctl_dyn_cali_stru g_ast_dyn_cali_cfg_map[];
extern OAL_CONST wal_ioctl_tlv_stru g_ast_set_tlv_table[];

/* 4 ����ʵ�� */

OAL_STATIC oal_uint32 wal_hipriv_global_log_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_int32 l_switch_val;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;

    /* ��ȡ����״ֵ̬ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_global_log_switch::error code[%d]}\r\n", ul_ret);
        return ul_ret;
    }

    if ((0 != oal_strcmp("0", ac_name)) && (0 != oal_strcmp("1", ac_name))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_global_log_switch::invalid switch value}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    l_switch_val = oal_atoi(ac_name);

    return oam_log_set_global_switch_etc((oal_switch_enum_uint8)l_switch_val);
}


OAL_STATIC oal_uint32 wal_hipriv_vap_log_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oal_int32 l_switch_val;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;

    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (OAL_UNLIKELY(pst_mac_vap == OAL_PTR_NULL)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_vap_log_switch::null pointer.}\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡ����״ֵ̬ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_vap_log_switch::error code[%d]}\r\n", ul_ret);
        return ul_ret;
    }

    if ((0 != oal_strcmp("0", ac_name)) && (0 != oal_strcmp("1", ac_name))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_vap_log_switch::invalid switch value}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    l_switch_val = oal_atoi(ac_name);

    return oam_log_set_vap_switch_etc(pst_mac_vap->uc_vap_id, (oal_switch_enum_uint8)l_switch_val);
}


OAL_STATIC oal_uint32 wal_hipriv_feature_log_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oam_feature_enum_uint8 en_feature_id;
    oal_uint8 uc_switch_vl;
    oal_uint32 ul_off_set;
    oal_int8 ac_param[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oam_log_level_enum_uint8 en_log_lvl;
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
    wal_msg_write_stru st_write_msg;
#endif

    /* OAM logģ��Ŀ��ص�����: hipriv "Hisilicon0[vapx] feature_log_switch {feature_name} {0/1}"
       1-2(error��warning)������־��vap����Ϊά�ȣ�
    */

    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (OAL_UNLIKELY(pst_mac_vap == OAL_PTR_NULL)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_feature_log_switch::null pointer.}\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡ�������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* �ṩ�������İ�����Ϣ */
    if ('?' == ac_param[0]) {
        OAL_IO_PRINT("please input abbr feature name. \r\n");
        oam_show_feature_list_etc();
        return OAL_SUCC;
    }

    /* ��ȡ����ID */
    ul_ret = oam_get_feature_id_etc((oal_uint8 *)ac_param, &en_feature_id);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_feature_log_switch::invalid feature name}\r\n");
        return ul_ret;
    }

    /* ��ȡ����ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ��ȡINFO���𿪹�״̬ */
    if ((0 != oal_strcmp("0", ac_param)) && (0 != oal_strcmp("1", ac_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_feature_log_switch::invalid switch value}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }
    uc_switch_vl = (oal_uint8)oal_atoi(ac_param);

    /* �ر�INFO��־����ʱ���ָ���Ĭ�ϵ���־���� */
    en_log_lvl = (uc_switch_vl == OAL_SWITCH_ON) ? OAM_LOG_LEVEL_INFO : OAM_LOG_DEFAULT_LEVEL;
    ul_ret = oam_log_set_feature_level_etc(pst_mac_vap->uc_vap_id, en_feature_id, en_log_lvl);

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_FEATURE_LOG, OAL_SIZEOF(oal_int32));
    *((oal_uint16 *)(st_write_msg.auc_value)) = ((en_feature_id << 8) | en_log_lvl);
    ul_ret |= (oal_uint32)wal_send_cfg_event_etc(pst_net_dev,
                                                 WAL_MSG_TYPE_WRITE,
                                                 WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                                 (oal_uint8 *)&st_write_msg,
                                                 OAL_FALSE,
                                                 OAL_PTR_NULL);

    if (OAL_UNLIKELY(ul_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_feature_log_switch::return err code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
#endif

    return ul_ret;
}


OAL_STATIC oal_uint32 wal_hipriv_log_ratelimit(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oam_ratelimit_stru st_ratelimit;
    oam_ratelimit_type_enum_uint8 en_ratelimit_type;
    oal_uint32 ul_off_set;
    oal_int8 ac_param[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;

    /* OAM log printk������������: hipriv "Hisilicon0[vapx] {log_ratelimit} {printk(0)/sdt(1)}{switch(0/1)} {interval} {burst}" */

    st_ratelimit.en_ratelimit_switch = OAL_SWITCH_OFF;
    st_ratelimit.ul_interval = OAM_RATELIMIT_DEFAULT_INTERVAL;
    st_ratelimit.ul_burst = OAM_RATELIMIT_DEFAULT_BURST;

    /* ��ȡ�������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }
    pc_param += ul_off_set;

    en_ratelimit_type = (oam_ratelimit_type_enum_uint8)oal_atoi(ac_param);

    /* ��ȡ����״̬ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }
    pc_param += ul_off_set;

    st_ratelimit.en_ratelimit_switch = (oal_switch_enum_uint8)oal_atoi(ac_param);

    if (st_ratelimit.en_ratelimit_switch == OAL_SWITCH_ON) {
        /* ��ȡintervalֵ */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            return ul_ret;
        }
        pc_param += ul_off_set;

        st_ratelimit.ul_interval = (oal_uint32)oal_atoi(ac_param);

        /* ��ȡburstֵ */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            return ul_ret;
        }
        pc_param += ul_off_set;

        st_ratelimit.ul_burst = (oal_uint32)oal_atoi(ac_param);
    }

    return oam_log_set_ratelimit_param_etc(en_ratelimit_type, &st_ratelimit);
}

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)

OAL_STATIC oal_uint32 wal_hipriv_log_lowpower(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* OAM eventģ��Ŀ��ص�����: hipriv "Hisilicon0 log_pm 0 | 1"
        �˴���������"1"��"0"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_log_lowpower::wal_get_cmd_one_arg_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����eventģ����в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else if (0 == (oal_strcmp("2", ac_name))) {
        l_tmp = 2;
    } else if (0 == (oal_strcmp("3", ac_name))) {
        l_tmp = 3;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_log_lowpower::the log switch command is error [%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_LOG_PM, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_event_switch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_pm_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* OAM eventģ��Ŀ��ص�����: hipriv "Hisilicon0 wal_hipriv_pm_switch 0 | 1"
        �˴���������"1"��"0"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pm_switch::wal_get_cmd_one_arg_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����eventģ����в�ͬ������ */
    l_tmp = (oal_uint8)oal_atoi(ac_name);

    if (l_tmp < 0 || l_tmp > 5) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pm_switch::the log switch command is error [%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_PM_SWITCH, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pm_switch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_power_test(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* OAM eventģ��Ŀ��ص�����: hipriv "Hisilicon0 wal_hipriv_power_test 0 | 1"
        �˴���������"1"��"0"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pm_switch::wal_get_cmd_one_arg_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����eventģ����в�ͬ������ */
    l_tmp = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_POWER_TEST, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pm_switch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif


OAL_STATIC oal_uint8 wal_get_tx_pow_param(oal_int8 *pc_param, oal_uint8 uc_param_len)
{
    wal_tx_pow_param_enum_uint8 en_param_index;

    /* ������������һ���ֶ� */
    for (en_param_index = 0; en_param_index < WAL_TX_POW_PARAM_BUTT; en_param_index++) {
        if (uc_param_len <= OAL_STRLEN(pauc_tx_pow_param_name[en_param_index])) {
            return WAL_TX_POW_PARAM_BUTT;
        }

        if (!oal_strcmp(pauc_tx_pow_param_name[en_param_index], pc_param)) {
            break;
        }
    }

    return en_param_index;
}


OAL_STATIC oal_void wal_get_tx_pow_log_param(mac_cfg_set_tx_pow_param_stru *pst_set_tx_pow_param, oal_int8 *pc_param)
{
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;

    if (pst_set_tx_pow_param->auc_value[0] == 0) {
        // pow
        if (pst_set_tx_pow_param->auc_value[1] >= HAL_POW_RATE_CODE_TBL_LEN) {
            // rate_idx
            OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::err rate idx [%d]!}", pst_set_tx_pow_param->auc_value[1]);
            pst_set_tx_pow_param->auc_value[1] = 0;
        }
    } else {
        // evm
        if (OAL_SUCC != wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set)) {
            OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::evm arg err input!}");
            return;
        }
        // chn_idx
        pst_set_tx_pow_param->auc_value[2] = (oal_uint8)oal_atoi(ac_arg);
    }

    return;
}


extern oal_bool_enum_uint8 aen_tas_switch_en[];
OAL_STATIC oal_uint32 wal_hipriv_set_tx_pow_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    mac_cfg_set_tx_pow_param_stru *pst_set_tx_pow_param = OAL_PTR_NULL;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret = OAL_SUCC;
    oal_int32 l_sec_ret = EOK;
    wal_tx_pow_param_enum_uint8 en_param_index;
    oal_uint8 uc_value = 0;
#ifdef _PRE_WLAN_FEATURE_TPC_OPT
    oal_uint8 auc_sar_ctrl_params[CUS_NUM_OF_SAR_PARAMS][WLAN_RF_CHANNEL_NUMS];
    oal_uint8 *puc_sar_ctrl_params;
#ifdef _PRE_WLAN_FEATURE_TAS_ANT_SWITCH
    mac_device_stru *pst_mac_device;
    mac_cfg_tas_pwr_ctrl_stru st_tas_pow_ctrl_params;
#endif
#endif

    /* �����ʽ: hipriv "vap0 set_tx_pow rf_reg_ctl 0/1" ,   0:��ʹ��, 1:ʹ��            */
    /* �����ʽ: hipriv "vap0 set_tx_pow fix_level 0/1/2/3"  ��������֡���ʵȼ�, ��data0 */
    /* �����ʽ: hipriv "vap0 set_tx_pow mag_level 0/1/2/3"  ���ù���֡���ʵȼ�          */
    /* �����ʽ: hipriv "vap0 set_tx_pow ctl_level 0/1/2/3"  ���ÿ���֡���ʵȼ�          */
    /* �����ʽ: hipriv "vap0 set_tx_pow amend <value>"      ����upc code                */
    /* �����ʽ: hipriv "vap0 set_tx_pow no_margin"          ���ʲ�����������, ��51��    */
    /* �����ʽ: hipriv "vap0 set_tx_pow show_log type (bw chn)" ��ʾ����ά����־
       type: 0/1 pow/evm
       bw:   rate_idx(pow) 0/1/2 20M/80M/160M(evm)
       chn:  0~6(20M/80M) 0/1(160M)  */
    /* �����ʽ: hipriv "vap0 set_tx_pow sar_level 0/1/2/3"  ���ý�sar�ȼ�               */
    /* �����ʽ: hipriv "vap0 set_tx_pow tas_pwr_ctrl 0/1 0/1" tas���ʿ���               */
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_TX_POW, OAL_SIZEOF(mac_cfg_set_tx_pow_param_stru));

    /* ��������������������� */
    pst_set_tx_pow_param = (mac_cfg_set_tx_pow_param_stru *)(st_write_msg.auc_value);
    memset_s(pst_set_tx_pow_param, OAL_SIZEOF(mac_cfg_set_tx_pow_param_stru), 0, OAL_SIZEOF(mac_cfg_set_tx_pow_param_stru));

    /* ��ȡ�������ֶ����������ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ������������һ���ֶ� */
    en_param_index = wal_get_tx_pow_param(ac_arg, sizeof(ac_arg));

    /* ��������Ƿ��� */
    if (en_param_index == WAL_TX_POW_PARAM_BUTT) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::no such param for tx pow!}\r\n");
        return OAL_FAIL;
    }

    pst_set_tx_pow_param->en_type = en_param_index;

    /* ��ȡ��һ������ */
    pc_param += ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
    } else {
        uc_value = (oal_uint8)oal_atoi(ac_arg);
    }
    pst_set_tx_pow_param->auc_value[0] = uc_value;

    /*lint -e571 */
    /* ����У�� */
    switch (en_param_index) {
        case WAL_TX_POW_PARAM_SET_RF_REG_CTL:
        case WAL_TX_POW_PARAM_SET_NO_MARGIN:
            if (uc_value >= 2) {
                return OAL_ERR_CODE_INVALID_CONFIG;
            }
            break;
        case WAL_TX_POW_PARAM_SET_FIX_LEVEL:
        case WAL_TX_POW_PARAM_SET_MAG_LEVEL:
        case WAL_TX_POW_PARAM_SET_CTL_LEVEL:
            if (uc_value > 4) {
                return OAL_ERR_CODE_INVALID_CONFIG;
            }
            break;

        case WAL_TX_POW_PARAM_SET_SHOW_LOG:
            /* ��ȡ��һ������ */
            pc_param += ul_off_set;
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::show log arg1 missing!}");
                break;
            }
            pst_set_tx_pow_param->auc_value[1] = (oal_uint8)oal_atoi(ac_arg);
            /* to reduce cycle complexity */
            pc_param += ul_off_set;
            wal_get_tx_pow_log_param(pst_set_tx_pow_param, pc_param);
            break;

        case WAL_TX_POW_PARAM_SET_AMEND:
        case WAL_TX_POW_PARAM_SHOW_TPC_TABLE_GAIN:
        case WAL_TX_POW_POW_SAVE:
        case WAL_TX_POW_SET_TPC_IDX:
            break;

        case WAL_TX_POW_PARAM_SET_SAR_LEVEL:
            OAM_WARNING_LOG1(0, OAM_SF_TPC, "{wal_hipriv_set_tx_pow_param::input reduce SAR level [%d]!}\r\n", uc_value);
#ifdef _PRE_WLAN_FEATURE_TPC_OPT
            if ((uc_value > CUS_NUM_OF_SAR_LVL) || (uc_value == 0)) {
                OAM_ERROR_LOG0(0, OAM_SF_TPC, "{wal_hipriv_set_tx_pow_param::reduce SAR STOP!}\r\n");
                memset_s(auc_sar_ctrl_params, OAL_SIZEOF(auc_sar_ctrl_params), 0xFF, OAL_SIZEOF(auc_sar_ctrl_params));
                puc_sar_ctrl_params = &auc_sar_ctrl_params[0][0];
            } else {
                puc_sar_ctrl_params = wal_get_reduce_sar_ctrl_params(uc_value);
            }

            if (puc_sar_ctrl_params) {
                l_sec_ret = memcpy_s(pst_set_tx_pow_param->auc_value, sizeof(pst_set_tx_pow_param->auc_value), puc_sar_ctrl_params, sizeof(auc_sar_ctrl_params));
            }
#endif
            break;

#ifdef _PRE_WLAN_FEATURE_TAS_ANT_SWITCH
        case WAL_TX_POW_PARAM_TAS_POW_CTRL:
            pst_mac_device = mac_res_get_dev_etc(0);
            /* ����ǵ�VAP,�򲻴��� */
            if (mac_device_calc_up_vap_num_etc(pst_mac_device) > 1) {
                OAM_ERROR_LOG0(0, OAM_SF_ANY, "wal_hipriv_set_tx_pow_param::WAL_TX_POW_PARAM_TAS_POW_CTRL more than 1 vap");
                return OAL_FAIL;
            }

            memset_s(&st_tas_pow_ctrl_params, OAL_SIZEOF(mac_cfg_tas_pwr_ctrl_stru),
                     0, OAL_SIZEOF(mac_cfg_tas_pwr_ctrl_stru));
            st_tas_pow_ctrl_params.uc_core_idx = !!uc_value;

            if (aen_tas_switch_en[!!uc_value] == OAL_FALSE) {
                return OAL_FAIL;
            }

            /* ��ȡ��һ������ */
            pc_param += ul_off_set;
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::TAS pwr ctrl core or needimproved lost return err_code [%d]!}\r\n");
                return OAL_ERR_CODE_INVALID_CONFIG;
            }
            st_tas_pow_ctrl_params.en_need_improved = (oal_uint8) !!oal_atoi(ac_arg);
            l_sec_ret = memcpy_s(pst_set_tx_pow_param->auc_value, sizeof(pst_set_tx_pow_param->auc_value),
                                 &st_tas_pow_ctrl_params, OAL_SIZEOF(mac_cfg_tas_pwr_ctrl_stru));

            OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::WAL_TX_POW_PARAM_TAS_POW_CTRL core[%d] improved_flag[%d]!}\r\n",
                             st_tas_pow_ctrl_params.uc_core_idx, st_tas_pow_ctrl_params.en_need_improved);
            break;

        case WAL_TX_POW_PARAM_TAS_RSSI_MEASURE:
            if (aen_tas_switch_en[!!uc_value] == OAL_FALSE) {
                OAM_ERROR_LOG1(0, OAM_SF_ANY, "wal_hipriv_set_tx_pow_param::WAL_TX_POW_PARAM_TAS_RSSI_MEASURE core[%d]!", uc_value);
                return OAL_FAIL;
            }
            break;

        case WAL_TX_POW_PARAM_TAS_ANT_SWITCH:
            if (OAL_ANY_TRUE_VALUE2(aen_tas_switch_en[WLAN_RF_CHANNEL_ZERO], aen_tas_switch_en[WLAN_RF_CHANNEL_ONE])) {
                /* 0:Ĭ��̬ 1:tas̬ */
                OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::CMD_SET_MEMO_CHANGE antIndex[%d].}", uc_value);
                l_ret = board_wifi_tas_set(uc_value);
            }
            return (oal_uint32)l_ret;
#endif
        case WAL_TX_POW_GET_PD_INFO:
#ifdef _PRE_WLAN_FEATURE_FULL_QUAN_PROD_CAL
            /* ��ʾ��ǰ��ʼ����ȫ��У׼�����ݲɼ�,��ȡ��һ������״̬ */
            if (pst_set_tx_pow_param->auc_value[0] == 1) {
                pc_param += ul_off_set;
                if (OAL_SUCC == wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set)) {
                    pst_set_tx_pow_param->auc_value[1] = (oal_uint8)oal_atoi(ac_arg);
                }
            }
            OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::pdet info param[%d %d]!}",
                             pst_set_tx_pow_param->auc_value[0], pst_set_tx_pow_param->auc_value[1]);
#endif
            break;

        default:
            OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::error input type!}\r\n");
            return OAL_FAIL;
    }
    /*lint +e571 */
    if (l_sec_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::memcpy fail!}");
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_set_tx_pow_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tx_pow_param::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_ucast_data_dscr_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_set_dscr_param_stru *pst_set_dscr_param = OAL_PTR_NULL;
    wal_dscr_param_enum_uint8 en_param_index;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* ��������������������� */
    pst_set_dscr_param = (mac_cfg_set_dscr_param_stru *)(st_write_msg.auc_value);

    /* ��ȡ�������ֶ����������ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_data_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ������������һ���ֶ� */
    for (en_param_index = 0; en_param_index < WAL_DSCR_PARAM_BUTT; en_param_index++) {
        if (!oal_strcmp(pauc_tx_dscr_param_name_etc[en_param_index], ac_arg)) {
            break;
        }
    }

    /* ��������Ƿ��� */
    if (en_param_index == WAL_DSCR_PARAM_BUTT) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_data_dscr_param::CMD ERR!cmd type::wlan0 set_ucast_data CMD VALUE!}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->ta: set tx rts antenna![0] or [1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->ra: set rx cts/ack/ba antenna![0] or [1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->cc: set channel code![0][BCC] or [1][LDPC]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->power: set lpf/pa/upc/dac!}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->shortgi: set tx dscr short gi or long gi![0] or [1] }");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->preamble: set tx dscr preamble mode![0] or [1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->rtscts: set tx dscr enable rts or not![0] or [1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->lsigtxop: set tx dscr enable lsigtxop or not![0] or [1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->smooth: set rx channel matrix with smooth or not![0] or [1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->snding: set sounding mode![0][NON]-[1][NDP]-[2][STAGGERD]-[3][LEGACY]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->txbf: set txbf mode![0][NON]-[1][EXPLICIT]-[2][LEGACY]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->stbc: set STBC or not![0] or [1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->rd_ess: expand spatial stream}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->dyn_bw: set rts/cts dynamic signaling or not![0][STATIC]-[1][DYNAMIC]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->dyn_bw_exist: set tx dscr dynamic when no ht exist!set[1]] }");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->ch_bw_exist: set tx dscr dynamic when no ht exist!set[1]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->rate: set 11a/b/g rate as rate table}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->mcs: set 11n rate as rate table mcs index}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->mcsac: set 11ac rate as rate table mcs index}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->mcsax: set 11ax rate as rate table mcs index}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->mcsax_er: set 11ax_er rate as rate table mcs index}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->nss: set 11ac nss mode [1][SINGLE]-[2][DOUBLE]-[3][TRIBLE]-[4][QUAD]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->bw: set tx dscr channel bw![20][20M]-[40][40M]-[d40][40MDUP]-[80][80M]-[d80][80MDUP]-[160][160M]-[d160][160MDUP]-[80_80][80+80M]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->ltf: set tx dscr ltf [0][1*]-[1][2*]-[2][4*]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->gi: set tx dscr gi [0][lgi]-[1][sgi]-[2][mgi]}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->txchain: set tx dscr txchain![1]CH0-[2]CH1-[3]DOUBLE}");
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD-->dcm: set tx dscr enable dcm![0] or [1] }");
        return OAL_FAIL;
    }

    pst_set_dscr_param->uc_function_index = en_param_index;

    /* �������ʡ��ռ����������� */
    if (en_param_index >= WAL_DSCR_PARAM_RATE && en_param_index <= WAL_DSCR_PARAM_BW) {
        ul_ret = wal_hipriv_process_rate_params(pst_net_dev, pc_param, pst_set_dscr_param);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_data_dscr_param::wal_hipriv_process_ucast_params return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
    } else {
        /* ����Ҫ����Ϊ�������� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_data_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;
        pst_set_dscr_param->l_value = oal_strtol(ac_arg, OAL_PTR_NULL, 0);
    }

    /* ��������֡���������� tpye = MAC_VAP_CONFIG_UCAST_DATA */
    pst_set_dscr_param->en_type = MAC_VAP_CONFIG_UCAST_DATA;
    OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_data_dscr_param::en_param_index [%d]!,value[%d]}\r\n", pst_set_dscr_param->uc_function_index, pst_set_dscr_param->l_value);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_DSCR, OAL_SIZEOF(mac_cfg_set_dscr_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_set_dscr_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_data_dscr_param::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_bcast_data_dscr_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_set_dscr_param_stru *pst_set_dscr_param = OAL_PTR_NULL;
    wal_dscr_param_enum_uint8 en_param_index;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_DSCR, OAL_SIZEOF(mac_cfg_set_dscr_param_stru));

    /* ��������������������� */
    pst_set_dscr_param = (mac_cfg_set_dscr_param_stru *)(st_write_msg.auc_value);

    /* ��ȡ�������ֶ����������ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_bcast_data_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ������������һ���ֶ� */
    for (en_param_index = 0; en_param_index < WAL_DSCR_PARAM_BUTT; en_param_index++) {
        if (!oal_strcmp(pauc_tx_dscr_param_name_etc[en_param_index], ac_arg)) {
            break;
        }
    }

    /* ��������Ƿ��� */
    if (en_param_index == WAL_DSCR_PARAM_BUTT) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_bcast_data_dscr_param::no such param for tx dscr!}\r\n");
        return OAL_FAIL;
    }
    pst_set_dscr_param->uc_function_index = en_param_index;

    /* �������ʡ��ռ����������� */
    if (en_param_index >= WAL_DSCR_PARAM_RATE && en_param_index <= WAL_DSCR_PARAM_BW) {
        ul_ret = wal_hipriv_process_rate_params(pst_net_dev, pc_param, pst_set_dscr_param);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_bcast_data_dscr_param::wal_hipriv_process_ucast_params return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
    } else {
        /* ����Ҫ����Ϊ�������� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_bcast_data_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;
        pst_set_dscr_param->l_value = oal_strtol(ac_arg, OAL_PTR_NULL, 0);
    }

    /* �㲥����֡���������� tpye = MAC_VAP_CONFIG_BCAST_DATA */
    pst_set_dscr_param->en_type = MAC_VAP_CONFIG_BCAST_DATA;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_set_dscr_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_bcast_data_dscr_param::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_ucast_mgmt_dscr_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_set_dscr_param_stru *pst_set_dscr_param = OAL_PTR_NULL;
    wal_dscr_param_enum_uint8 en_param_index;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 uc_band;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_DSCR, OAL_SIZEOF(mac_cfg_set_dscr_param_stru));

    /* ��������������������� */
    pst_set_dscr_param = (mac_cfg_set_dscr_param_stru *)(st_write_msg.auc_value);

    /***************************************************************************
             sh hipriv.sh "vap0 set_ucast_mgmt data0 2 8389137"
    ***************************************************************************/
    /* ����data0 */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_mgmt_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ������������һ���ֶ� */
    for (en_param_index = 0; en_param_index < WAL_DSCR_PARAM_BUTT; en_param_index++) {
        if (!oal_strcmp(pauc_tx_dscr_param_name_etc[en_param_index], ac_arg)) {
            break;
        }
    }

    /* ��������Ƿ��� */
    if (en_param_index == WAL_DSCR_PARAM_BUTT) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_mgmt_dscr_param::no such param for tx dscr!}\r\n");
        return OAL_FAIL;
    }

    pst_set_dscr_param->uc_function_index = en_param_index;

    /* ����Ҫ����Ϊ�ĸ�Ƶ�εĵ�������֡ 2G or 5G */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_mgmt_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    uc_band = (oal_uint8)oal_atoi(ac_arg);

    /* ��������֡���������� tpye = MAC_VAP_CONFIG_UCAST_MGMT 2Ϊ2G,����Ϊ5G  */
    if (WLAN_BAND_2G == uc_band) {
        pst_set_dscr_param->en_type = MAC_VAP_CONFIG_UCAST_MGMT_2G;
    } else {
        pst_set_dscr_param->en_type = MAC_VAP_CONFIG_UCAST_MGMT_5G;
    }

    /* �������ʡ��ռ����������� */
    if (en_param_index >= WAL_DSCR_PARAM_RATE && en_param_index <= WAL_DSCR_PARAM_BW) {
        ul_ret = wal_hipriv_process_rate_params(pst_net_dev, pc_param, pst_set_dscr_param);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_mgmt_dscr_param::wal_hipriv_process_ucast_params return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
    } else {
        /* ����Ҫ����Ϊ�������� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_mgmt_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;
        pst_set_dscr_param->l_value = oal_strtol(ac_arg, OAL_PTR_NULL, 0);
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_set_dscr_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ucast_mgmt_dscr_param::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_mbcast_mgmt_dscr_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_set_dscr_param_stru *pst_set_dscr_param = OAL_PTR_NULL;
    wal_dscr_param_enum_uint8 en_param_index;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 uc_band;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_DSCR, OAL_SIZEOF(mac_cfg_set_dscr_param_stru));

    /* ��������������������� */
    pst_set_dscr_param = (mac_cfg_set_dscr_param_stru *)(st_write_msg.auc_value);

    /***************************************************************************
             sh hipriv.sh "vap0 set_mcast_mgmt data0 5 8389137"
    ***************************************************************************/
    /* ����data0 */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_mbcast_mgmt_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ������������һ���ֶ� */
    for (en_param_index = 0; en_param_index < WAL_DSCR_PARAM_BUTT; en_param_index++) {
        if (!oal_strcmp(pauc_tx_dscr_param_name_etc[en_param_index], ac_arg)) {
            break;
        }
    }

    /* ��������Ƿ��� */
    if (en_param_index == WAL_DSCR_PARAM_BUTT) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_mbcast_mgmt_dscr_param::no such param for tx dscr!}\r\n");
        return OAL_FAIL;
    }

    pst_set_dscr_param->uc_function_index = en_param_index;

    /* ����Ҫ����Ϊ�ĸ�Ƶ�εĵ�������֡ 2G or 5G */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_mbcast_mgmt_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    uc_band = (oal_uint8)oal_atoi(ac_arg);

    /* ��������֡���������� tpye = MAC_VAP_CONFIG_UCAST_MGMT 2Ϊ2G,����Ϊ5G  */
    if (WLAN_BAND_2G == uc_band) {
        pst_set_dscr_param->en_type = MAC_VAP_CONFIG_MBCAST_MGMT_2G;
    } else {
        pst_set_dscr_param->en_type = MAC_VAP_CONFIG_MBCAST_MGMT_5G;
    }

    /* �������ʡ��ռ����������� */
    if (en_param_index >= WAL_DSCR_PARAM_RATE && en_param_index <= WAL_DSCR_PARAM_BW) {
        ul_ret = wal_hipriv_process_rate_params(pst_net_dev, pc_param, pst_set_dscr_param);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_mbcast_mgmt_dscr_param::wal_hipriv_process_ucast_params return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
    } else {
        /* ����Ҫ����Ϊ�������� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_mbcast_mgmt_dscr_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;

        pst_set_dscr_param->l_value = oal_strtol(ac_arg, OAL_PTR_NULL, 0);
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_set_dscr_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_mbcast_mgmt_dscr_param::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_11D

oal_uint32 wal_hipriv_set_rd_by_ie_switch_etc(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_switch_enum_uint8 *pst_set_rd_by_ie_switch;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_switch_enum_uint8 en_rd_by_ie_switch;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_RD_IE_SWITCH, OAL_SIZEOF(oal_switch_enum_uint8));

    /* ��������������������� */
    pst_set_rd_by_ie_switch = (oal_switch_enum_uint8 *)(st_write_msg.auc_value);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_rd_by_ie_switch_etc::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    en_rd_by_ie_switch = (oal_uint8)oal_atoi(ac_name);
    *pst_set_rd_by_ie_switch = en_rd_by_ie_switch;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_switch_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_rd_by_ie_switch_etc::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


oal_uint32 wal_hipriv_set_nss(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_tx_comp_stru *pst_set_nss_param = OAL_PTR_NULL;
    oal_int32 l_nss;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_idx = 0;
    wal_msg_stru *pst_rsp_msg = OAL_PTR_NULL;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_NSS, OAL_SIZEOF(mac_cfg_tx_comp_stru));

    /* ��������������������� */
    pst_set_nss_param = (mac_cfg_tx_comp_stru *)(st_write_msg.auc_value);

    /* ��ȡ����ֵ�ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_nss::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��������Ϸ��Լ�� */
    while ('\0' != ac_arg[l_idx]) {
        if (isdigit(ac_arg[l_idx])) {
            l_idx++;
            continue;
        } else {
            l_idx++;
            OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_nss::input illegal!}\r\n");
            return OAL_ERR_CODE_INVALID_CONFIG;
        }
    }

    /* ����Ҫ����Ϊ����ֵ */
    l_nss = oal_atoi(ac_arg);

    if (l_nss < WAL_HIPRIV_NSS_MIN || l_nss > WAL_HIPRIV_NSS_MAX) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_nss::input val out of range [%d]!}\r\n", l_nss);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pst_set_nss_param->uc_param = (oal_uint8)(l_nss - 1);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_tx_comp_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_TRUE,
                                   &pst_rsp_msg);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_nss::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }
    /* ��ȡ���صĴ����� */
    ul_ret = wal_check_and_release_msg_resp_etc(pst_rsp_msg);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_DFR, "{wal_hipriv_set_freq fail, err code[%u]!}\r\n", ul_ret);
        return ul_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_set_rfch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_tx_comp_stru *pst_set_rfch_param = OAL_PTR_NULL;
    oal_uint8 uc_ch;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 c_ch_idx;
    wal_msg_stru *pst_rsp_msg = OAL_PTR_NULL;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_RFCH, OAL_SIZEOF(mac_cfg_tx_comp_stru));

    /* ��������������������� */
    pst_set_rfch_param = (mac_cfg_tx_comp_stru *)(st_write_msg.auc_value);

    /* ��ȡ����ֵ�ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_rfch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ����Ҫ����Ϊ����ֵ */
    uc_ch = 0;
    for (c_ch_idx = 0; c_ch_idx < WAL_HIPRIV_CH_NUM; c_ch_idx++) {
        if ('0' == ac_arg[c_ch_idx]) {
            continue;
        } else if ('1' == ac_arg[c_ch_idx]) {
            uc_ch += (oal_uint8)(1 << (WAL_HIPRIV_CH_NUM - (oal_uint8)c_ch_idx - 1));
        }
        /* ���������з�01���֣�����������4λ���쳣 */
        else {
            OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_rfch::input err!}\r\n");
            return OAL_ERR_CODE_INVALID_CONFIG;
        }
    }

    /* �������������λ���쳣 */
    if ('\0' != ac_arg[c_ch_idx]) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_rfch::input err!}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pst_set_rfch_param->uc_param = uc_ch;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_tx_comp_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_TRUE,
                                   &pst_rsp_msg);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_rfch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    /* ��ȡ���صĴ����� */
    ul_ret = wal_check_and_release_msg_resp_etc(pst_rsp_msg);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_DFR, "{wal_hipriv_set_rfch fail, err code[%u]!}\r\n", ul_ret);
        return ul_ret;
    }
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_get_thruput(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 uc_stage;
    oal_int32 l_idx = 0;

    /* ��ȡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_get_thruput::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��������Ϸ��Լ�� */
    while ('\0' != ac_arg[l_idx]) {
        if (isdigit(ac_arg[l_idx])) {
            l_idx++;
            continue;
        } else {
            l_idx++;
            OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_get_thruput::input illegal!}\r\n");
            return OAL_ERR_CODE_INVALID_CONFIG;
        }
    }

    /* ���������ֵ�ַ���ת��Ϊ���� */
    uc_stage = (oal_uint8)oal_atoi(ac_arg);

    *(oal_uint8 *)(st_write_msg.auc_value) = uc_stage;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_GET_THRUPUT, OAL_SIZEOF(oal_uint8));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_get_thruput::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_freq_skew(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint16 us_len;
    oal_uint16 i;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    mac_cfg_freq_skew_stru *pst_freq_skew;

    /*             �����ʽ: hipriv "Hisilicon0 set_freq_skew <>"
     * <idx chn T0Int20M T0Frac20M T1Int20M T1Frac20M T0Int40M T0Frac40M T1Int40M T1Frac40M>
     */
    pst_freq_skew = (mac_cfg_freq_skew_stru *)st_write_msg.auc_value;

    /* ����ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_freq_skew::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_freq_skew->us_idx = (oal_uint16)oal_atoi(ac_arg);

    /* �ŵ� */
    pc_param += ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_freq_skew::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_freq_skew->us_chn = (oal_uint16)oal_atoi(ac_arg);

    /* ��ȡ8��У������ */
    for (i = 0; i < WAL_HIPRIV_FREQ_SKEW_ARG_NUM; i++) {
        pc_param += ul_off_set;
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_freq_skew::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pst_freq_skew->as_corr_data[i] = (oal_int16)oal_atoi(ac_arg);
    }

    us_len = OAL_SIZEOF(mac_cfg_freq_skew_stru);
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_FREQ_SKEW, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_freq_skew::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_adjust_ppm(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint16 us_len;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    mac_cfg_adjust_ppm_stru *pst_adjust_ppm;

    /* �����ʽ: hipriv "Hisilicon0 adjust_ppm ppm band clock" */
    pst_adjust_ppm = (mac_cfg_adjust_ppm_stru *)st_write_msg.auc_value;

    /* ppm */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_adjust_ppm::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    if ('-' == ac_arg[0]) {
        pst_adjust_ppm->c_ppm_val = -(oal_int8)oal_atoi(ac_arg + 1);
    } else {
        pst_adjust_ppm->c_ppm_val = (oal_int8)oal_atoi(ac_arg);
    }

    /* clock */
    pc_param += ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        /* �˲��������ã�����Ĭ��ʱ�����ã�5G 26M 2G 40M */
        pst_adjust_ppm->uc_clock_freq = 0;
    } else {
        pst_adjust_ppm->uc_clock_freq = (oal_uint8)oal_atoi(ac_arg);
    }

    us_len = OAL_SIZEOF(mac_cfg_adjust_ppm_stru);
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ADJUST_PPM, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_adjust_ppm::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_event_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* OAM eventģ��Ŀ��ص�����: hipriv "Hisilicon0 event_switch 0 | 1"
        �˴���������"1"��"0"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_event_switch::wal_get_cmd_one_arg_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����eventģ����в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_event_switch::the log switch command is error [%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_EVENT_SWITCH, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_event_switch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_NARROW_BAND

OAL_STATIC oal_uint32 wal_hipriv_narrow_bw(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_narrow_bw_stru *pst_nrw_bw;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_NARROW_BW, OAL_SIZEOF(mac_cfg_narrow_bw_stru));

    /* ��������������������� */
    pst_nrw_bw = (mac_cfg_narrow_bw_stru *)(st_write_msg.auc_value);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_narrow_bw::get switch  [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;

    pst_nrw_bw->en_open = (oal_uint8)oal_atoi(ac_arg);

    /* խ����ģʽ��ǿ�ƹر�ampdu amsdu�ۺϺ������������㷨 */
    if (pst_nrw_bw->en_open == OAL_TRUE) {
        wal_hipriv_alg_cfg_etc(pst_net_dev, "anti_inf_unlock_en 0");
        wal_hipriv_ampdu_tx_on(pst_net_dev, "0");
        wal_hipriv_amsdu_tx_on(pst_net_dev, "0");
    } else {
        wal_hipriv_alg_cfg_etc(pst_net_dev, "anti_inf_unlock_en 1");
        wal_hipriv_ampdu_tx_on(pst_net_dev, "1");
        wal_hipriv_amsdu_tx_on(pst_net_dev, "1");
    }

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_narrow_bw::get switch  [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;

    if (0 == (oal_strcmp("1m", ac_arg))) {
        pst_nrw_bw->en_bw = NARROW_BW_1M;
    } else if (0 == (oal_strcmp("5m", ac_arg))) {
        pst_nrw_bw->en_bw = NARROW_BW_5M;
    } else if (0 == (oal_strcmp("10m", ac_arg))) {
        pst_nrw_bw->en_bw = NARROW_BW_10M;
    } else {
        pst_nrw_bw->en_bw = NARROW_BW_BUTT;
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_narrow_bw::bw should be 1/5/10 m");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_narrow_bw_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif


OAL_STATIC oal_uint32 wal_hipriv_ota_beacon_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_param;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* OAM otaģ��Ŀ��ص�����: hipriv "Hisilicon0 ota_beacon_switch 0 | 1"
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ota_beacon_switch::wal_get_cmd_one_arg_etc fails!}\r\n");
        return ul_ret;
    }
    l_param = oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_OTA_BEACON_SWITCH, OAL_SIZEOF(oal_uint32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_param;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ota_beacon_switch::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_ota_rx_dscr_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_param;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* OAM otaģ��Ŀ��ص�����: hipriv "Hisilicon0 ota_rx_dscr_switch 0 | 1"
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ota_rx_dscr_switch::wal_get_cmd_one_arg_etc fails!}\r\n");
        return ul_ret;
    }

    /* �������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param + ul_off_set, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ota_rx_dscr_switch::wal_get_cmd_one_arg_etc fails!}\r\n");
        return ul_ret;
    }
    l_param = oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_OTA_RX_DSCR_SWITCH, OAL_SIZEOF(oal_uint32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_param;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ota_rx_dscr_switch::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_ether_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_cfg_eth_switch_param_stru st_eth_switch_param;

    /* "vap0 ether_switch user_macaddr oam_ota_frame_direction_type_enum(֡����) 0|1(����)" */
    memset_s(&st_eth_switch_param, OAL_SIZEOF(mac_cfg_eth_switch_param_stru),
             0, OAL_SIZEOF(mac_cfg_eth_switch_param_stru));

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, st_eth_switch_param.auc_user_macaddr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ether_switch::wal_hipriv_get_mac_addr_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ��ȡ��̫��֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ether_switch::wal_get_cmd_one_arg_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_eth_switch_param.en_frame_direction = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ether_switch::wal_get_cmd_one_arg_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    st_eth_switch_param.en_switch = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ETH_SWITCH, OAL_SIZEOF(st_eth_switch_param));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_eth_switch_param,
                        sizeof(st_eth_switch_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_ether_switch::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_eth_switch_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ether_switch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_80211_ucast_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_cfg_80211_ucast_switch_stru st_80211_ucast_switch;

    /* sh hipriv.sh "vap0 80211_uc_switch user_macaddr 0|1(֡����tx|rx) 0|1(֡����:����֡|����֡)
                                                       0|1(֡���ݿ���) 0|1(CB����) 0|1(����������)"
    */
    memset_s(&st_80211_ucast_switch, OAL_SIZEOF(mac_cfg_80211_ucast_switch_stru),
             0, OAL_SIZEOF(mac_cfg_80211_ucast_switch_stru));

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, st_80211_ucast_switch.auc_user_macaddr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::wal_hipriv_get_mac_addr_etc return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ��ȡ80211֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::get 80211 ucast frame direction return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_frame_direction = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::get ucast frame type return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_frame_type = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���ݴ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::get ucast frame content switch  return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_frame_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡CB�ֶδ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::get ucast frame cb switch return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_cb_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ��������ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::get ucast frame dscr switch return err_code[%d]}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_dscr_switch = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_80211_UCAST_SWITCH, OAL_SIZEOF(st_80211_ucast_switch));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_80211_ucast_switch,
                        OAL_SIZEOF(st_80211_ucast_switch))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_80211_ucast_switch),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_TXOPPS

OAL_STATIC oal_uint32 wal_hipriv_set_txop_ps_machw(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_txopps_machw_param_stru st_txopps_machw_param = { 0 };

    /* sh hipriv.sh "stavap_name txopps_hw_en 0|1(txop_ps_en) 0|1(condition1) 0|1(condition2)" */
    /* ��ȡtxop psʹ�ܿ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_TXOP, "{wal_hipriv_set_txop_ps_machw::get machw txop_ps en return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_txopps_machw_param.en_machw_txopps_en = (oal_switch_enum_uint8)oal_atoi(ac_name);

    /* ��ȡtxop ps condition1ʹ�ܿ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_TXOP, "{wal_hipriv_set_txop_ps_machw::get machw txop_ps condition1 return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_txopps_machw_param.en_machw_txopps_condition1 = (oal_switch_enum_uint8)oal_atoi(ac_name);

    /* ��ȡtxop ps condition2ʹ�ܿ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_TXOP, "{wal_hipriv_set_txop_ps_machw::get machw txop_ps condition2 return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_txopps_machw_param.en_machw_txopps_condition2 = (oal_switch_enum_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TXOP_PS_MACHW, OAL_SIZEOF(st_txopps_machw_param));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_txopps_machw_param,
                        OAL_SIZEOF(st_txopps_machw_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_txop_ps_machw::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_txopps_machw_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_TXOP, "{wal_hipriv_set_txop_ps_machw::return err code[%d]!}\r\n", ul_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_set_80211_mcast_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_cfg_80211_mcast_switch_stru st_80211_mcast_switch = { 0 };

    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));
    /*
     * sh hipriv.sh "Hisilicon0 80211_mc_switch 0|1(֡����tx|rx) 0|1(֡����:����֡|����֡)
     * 0|1(֡���ݿ���) 0|1(CB����) 0|1(����������)"
     */
    /* ��ȡ80211֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_mcast_switch::get 80211 mcast frame direction return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_mcast_switch.en_frame_direction = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_mcast_switch::get mcast frame type return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_mcast_switch.en_frame_type = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���ݴ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_mcast_switch::get mcast frame content switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_mcast_switch.en_frame_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡CB�ֶδ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_mcast_switch::get mcast frame cb switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_mcast_switch.en_cb_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ��������ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_mcast_switch::get mcast frame dscr switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_mcast_switch.en_dscr_switch = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_80211_MCAST_SWITCH, OAL_SIZEOF(st_80211_mcast_switch));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_80211_mcast_switch,
                        OAL_SIZEOF(st_80211_mcast_switch))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_80211_mcast_switch::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_80211_mcast_switch),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_mcast_switch::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_all_80211_ucast(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    mac_cfg_80211_ucast_switch_stru st_80211_ucast_switch;
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /* sh hipriv.sh "Hisilicon0 80211_uc_all 0|1(֡����tx|rx) 0|1(֡����:����֡|����֡)
                                             0|1(֡���ݿ���) 0|1(CB����) 0|1(����������)"
    */

    memset_s(&st_80211_ucast_switch, OAL_SIZEOF(mac_cfg_80211_ucast_switch_stru),
             0, OAL_SIZEOF(mac_cfg_80211_ucast_switch_stru));

    /* ��ȡ80211֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_80211_ucast::get 80211 ucast frame direction return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_frame_direction = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_80211_ucast::get ucast frame type return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_frame_type = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���ݴ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_80211_ucast::get ucast frame content switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_frame_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡CB�ֶδ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_80211_ucast::get ucast frame cb switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_cb_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ��������ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_80211_ucast::get ucast frame dscr switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_80211_ucast_switch.en_dscr_switch = (oal_uint8)oal_atoi(ac_name);

    /* ���ù㲥mac��ַ */
    memcpy_s(st_80211_ucast_switch.auc_user_macaddr, WLAN_MAC_ADDR_LEN, BROADCAST_MACADDR, WLAN_MAC_ADDR_LEN);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_80211_UCAST_SWITCH, OAL_SIZEOF(st_80211_ucast_switch));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_80211_ucast_switch,
                        OAL_SIZEOF(st_80211_ucast_switch))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_all_80211_ucast::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_80211_ucast_switch),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_all_ucast_switch::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_all_ether_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_uint16 us_user_num;
    oal_uint8 uc_frame_direction;
    oal_uint8 uc_switch;

    /* sh hipriv.sh "Hisilicon0 ether_all 0|1(֡����tx|rx) 0|1(����)" */
    /* ��ȡ��̫��֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_ether_switch::get eth frame direction return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    uc_frame_direction = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_ether_switch::get eth type return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    uc_switch = (oal_uint8)oal_atoi(ac_name);

    /* ���ÿ��� */
    for (us_user_num = 0; us_user_num < WLAN_USER_MAX_USER_LIMIT; us_user_num++) {
        oam_report_eth_frame_set_switch_etc(us_user_num, uc_switch, uc_frame_direction);
    }

    /* ͬʱ���ù㲥arp dhcp֡���ϱ����� */
    oam_report_dhcp_arp_set_switch_etc(uc_switch);

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_dhcp_arp_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_switch;

    /* sh hipriv.sh "Hisilicon0 dhcp_arp_switch 0|1(����)" */
    /* ��ȡ֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_dhcp_arp_switch::get switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    uc_switch = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_DHCP_ARP, OAL_SIZEOF(oal_uint32));
    *((oal_int32 *)(st_write_msg.auc_value)) = (oal_uint32)uc_switch;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_dhcp_arp_switch::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)

OAL_STATIC oal_uint32 wal_hipriv_report_vap_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_uint32 ul_flag_value;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;

    /* sh hipriv.sh "wlan0 report_vap_info  flags_value" */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_vap_info::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    ul_flag_value = (oal_uint32)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_REPORT_VAP_INFO, OAL_SIZEOF(ul_flag_value));

    /* ��д��Ϣ�壬���� */
    *(oal_uint32 *)(st_write_msg.auc_value) = ul_flag_value;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(ul_flag_value),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_vap_info::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_set_trlr_info(mac_phy_debug_switch_stru *pst_phy_debug_switch, oal_int8 c_value)
{
    if (isdigit(c_value)) {
        pst_phy_debug_switch->auc_trlr_sel_info[pst_phy_debug_switch->uc_trlr_sel_num] = (oal_uint8)(c_value - '0');
    } else if ((c_value >= 'a') && (c_value <= 'f')) {
        pst_phy_debug_switch->auc_trlr_sel_info[pst_phy_debug_switch->uc_trlr_sel_num] = (oal_uint8)(c_value - 'a' + 10);
    } else if ((c_value >= 'A') || (c_value <= 'F')) {
        pst_phy_debug_switch->auc_trlr_sel_info[pst_phy_debug_switch->uc_trlr_sel_num] = (oal_uint8)(c_value - 'A' + 10);
    } else {
        OAM_ERROR_LOG0(0, OAM_SF_CFG, "{wal_hipriv_set_trlr_info:: param input illegal, should be [0-f/F].!!}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_get_tone_tran_para(oal_int8 **pc_param, oal_int8 *ac_value, oal_uint32 ul_value_len, mac_phy_debug_switch_stru *pst_phy_debug_switch)
{
    oal_uint32 ul_ret = 0;
    oal_uint32 ul_off_set = 0;

    if (0 == oal_strcmp("help", ac_value)) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_get_tone_tran_para:tone tansmit command.!!}\r\n");
        return OAL_SUCC;
    }

    pst_phy_debug_switch->st_tone_tran.uc_tone_tran_switch = (oal_uint8)oal_atoi(ac_value);
    if (pst_phy_debug_switch->st_tone_tran.uc_tone_tran_switch == 1) {
        ul_ret = wal_get_cmd_one_arg_etc(*pc_param, ac_value, ul_value_len, &ul_off_set);
        if ((ul_ret != OAL_SUCC) || (ul_off_set == 0)) {
            OAM_ERROR_LOG0(0, OAM_SF_CFG, "{wal_get_tone_tran_para:chain index is illegal'!!}\r\n");
            return ul_ret;
        }
        *pc_param += ul_off_set;
        pst_phy_debug_switch->st_tone_tran.uc_chain_idx = (oal_uint16)oal_atoi(ac_value);

        if ((pst_phy_debug_switch->st_tone_tran.uc_chain_idx != 0) && (pst_phy_debug_switch->st_tone_tran.uc_chain_idx != 1)) {
            OAM_ERROR_LOG1(0, OAM_SF_CFG, "{wal_get_tone_tran_para:chain index[%d] is invalid'!!}\r\n", pst_phy_debug_switch->st_tone_tran.uc_chain_idx);
            return OAL_FAIL;
        }

        /*  ��ȡ���ݳ���  */
        ul_ret = wal_get_cmd_one_arg_etc(*pc_param, ac_value, ul_value_len, &ul_off_set);
        if ((ul_ret != OAL_SUCC) || (ul_off_set == 0) || (0 == oal_atoi(ac_value))) {
            OAM_ERROR_LOG0(0, OAM_SF_CFG, "{wal_get_tone_tran_para:tone data len is illegal'!!}\r\n");
            return ul_ret;
        }
        *pc_param += ul_off_set;
        pst_phy_debug_switch->st_tone_tran.us_data_len = (oal_uint16)oal_atoi(ac_value);
    }
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_print_phy_debug_help_info(oal_int8 *pc_param, oal_uint8 uc_para_len)
{
    if (uc_para_len < OAL_STRLEN("help")) {
        return OAL_FAIL;
    }

    if (0 == oal_strcmp("help", pc_param)) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD[rssi]Print the rssi of rx packets, reported from rx dscr of MAC, range [-128 ~ +127] dBm.!!}\r\n");
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD[tsensor]Print the code of T-sensor.!!}\r\n");
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD[snr]Print snr values of two rx ants, 11B not included, reported from rx dscr of MAC, range [-10 ~ +55] dBm.!!}\r\n");
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD[evm]Print evm values of two rx ants, 11B not included, reported from rx dscr of MAC, range [-10 ~ +55] dBm.!!}\r\n");
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD[spatial_reuse]Print Sptial Reuse of MAC srg and non-srg frame time and rssi!!}\r\n");
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD[trlr/vect]Sel range::trailer [0~f/F], vector [0~7], eg1: trlr 01234 eg2: vec123 trlr12, sum of both is less than 5.!!}\r\n");
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD[count]Set the interval of print (packets), range [0 ~ 2^32].!!}\r\n");

        return OAL_SUCC;
    }

    return OAL_FAIL;
}


OAL_STATIC oal_uint32 wal_hipriv_set_phy_debug_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    mac_phy_debug_switch_stru st_phy_debug_switch;
    oal_uint32 ul_ret = 0;
    oal_int32 l_ret;
    oal_bool_enum_uint8 en_cmd_updata = OAL_FALSE;
    oal_uint8 uc_data_cnt;  // ���������������

    /* sh hipriv.sh "wlan0 phy_debug snr 0|1(�ر�|��) rssi 0|1(�ر�|��) trlr 1234a count N(ÿ��N�����Ĵ�ӡһ��)" */

    memset_s(&st_phy_debug_switch, OAL_SIZEOF(st_phy_debug_switch), 0, OAL_SIZEOF(st_phy_debug_switch));

    st_phy_debug_switch.ul_rx_comp_isr_interval = 10;  // ���û�����ã���Ĭ��10������ӡһ�Σ���������Ը���
#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
    st_phy_debug_switch.uc_trlr_sel_num = (BOARD_VERSION_HI1105 == get_hi110x_subchip_type()) ? 0 : 1;  // 1103mpw2��rx hderռ��һ������������1��ʼ����, 1105��0��ʼ
#endif
    st_phy_debug_switch.uc_force_work_switch = 0xff;
    st_phy_debug_switch.st_tone_tran.uc_tone_tran_switch = 0xF; /*  Ĭ�ϵ������Ͳ����ڷ���/�ر�״̬  */
    st_phy_debug_switch.uc_dfr_reset_switch = 0xff;
    st_phy_debug_switch.uc_extlna_chg_bypass_switch = 0xff;
    st_phy_debug_switch.uc_edca_param_switch = 0x0;

    do {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if ((ul_ret != OAL_SUCC) && (ul_off_set != 0)) {
            OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_phy_debug_switch::cmd format err, ret:%d;!!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;

        if (en_cmd_updata == OAL_FALSE) {
            en_cmd_updata = OAL_TRUE;
        } else if (ul_off_set == 0) {
            break;
        }

        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAL_IO_PRINT("CMD format::sh hipriv.sh 'wlan0 phy_debug [rssi 0|1] [snr 0|1] [trlr xxxxx] [vect yyyyy] [count N] [edca 0-3(tid_no)] [aifsn N] [cwmin N] [cwmax N] [txoplimit N]'");
            OAM_ERROR_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 phy_debug [rssi 0|1] [snr 0|1] [trlr xxxxx] [vect yyyyy] [count N] [edca 0-3(tid_no)] [aifsn N] [cwmin N] [cwmax N] [txoplimit N]'!!}\r\n");
            return ul_ret;
        }
        pc_param += ul_off_set;
        ul_off_set = 0;

        /* ��ӡhelp��Ϣ */
        ul_ret = wal_hipriv_print_phy_debug_help_info(ac_value, sizeof(ac_value));
        if (ul_ret == OAL_SUCC) {
            return ul_ret;
        }

        /* ��ѯ�������� */
        if (0 == oal_strcmp("rssi", ac_name)) {
            st_phy_debug_switch.en_rssi_debug_switch = ((oal_uint8)oal_atoi(ac_value)) & OAL_TRUE;
        } else if (0 == oal_strcmp("tsensor", ac_name)) {
            st_phy_debug_switch.en_tsensor_debug_switch = ((oal_uint8)oal_atoi(ac_value)) & OAL_TRUE;
        } else if (0 == oal_strcmp("snr", ac_name)) {
            st_phy_debug_switch.en_snr_debug_switch = ((oal_uint8)oal_atoi(ac_value)) & OAL_TRUE;
        } else if (0 == oal_strcmp("evm", ac_name)) {
            st_phy_debug_switch.en_evm_debug_switch = ((oal_uint8)oal_atoi(ac_value)) & OAL_TRUE;
        } else if (0 == oal_strcmp("spatial_reuse", ac_name)) {
            st_phy_debug_switch.en_sp_reuse_debug_switch = ((oal_uint8)oal_atoi(ac_value)) & OAL_TRUE;
        } else if (OAL_ANY_ZERO_VALUE2(oal_strcmp("trlr", ac_name), oal_strcmp("vect", ac_name))) {
            uc_data_cnt = 0;
            while (ac_value[uc_data_cnt] != '\0') {
                /* ��������Ϸ��Լ�� */
                if (st_phy_debug_switch.uc_trlr_sel_num >= WAL_PHY_DEBUG_TEST_WORD_CNT) {
                    OAM_ERROR_LOG1(0, OAM_SF_CFG, "{Param input illegal, cnt [%d] reached 5!!}\r\n", st_phy_debug_switch.uc_trlr_sel_num);
                    return OAL_ERR_CODE_ARRAY_OVERFLOW;
                }

                ul_ret = wal_hipriv_set_trlr_info(&st_phy_debug_switch, ac_value[uc_data_cnt]);
                if (ul_ret != OAL_SUCC) {
                    return ul_ret;
                }

                if (0 == oal_strcmp("vect", ac_name)) {
                    if (ac_value[uc_data_cnt] > '7') {
                        OAM_ERROR_LOG0(0, OAM_SF_CFG, "{param input illegal, vect should be [0-7].!!}\r\n");
                        return OAL_ERR_CODE_INVALID_CONFIG;
                    }
                } else {
                    /* vector bit4Ϊ0��trailer��bit4��1, ���üĴ�������һ������ */
                    st_phy_debug_switch.auc_trlr_sel_info[st_phy_debug_switch.uc_trlr_sel_num] |= 0x10;
                }

                uc_data_cnt++;
                st_phy_debug_switch.uc_trlr_sel_num++;
            }

            /* ���������������trailer���� */
            if (uc_data_cnt != 0) {
                st_phy_debug_switch.en_trlr_debug_switch = OAL_TRUE;
            }
        } else if (0 == oal_strcmp("count", ac_name)) {
            st_phy_debug_switch.ul_rx_comp_isr_interval = (oal_uint32)oal_atoi(ac_value);
        } else if (0 == oal_strcmp("iq_cali", ac_name)) {
            st_phy_debug_switch.uc_iq_cali_switch = (oal_uint8)oal_atoi(ac_value);
        } else if (0 == oal_strcmp("tone_tran", ac_name)) {
            ul_ret = wal_get_tone_tran_para(&pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &st_phy_debug_switch);
            if (ul_ret != OAL_SUCC) {
                return ul_ret;
            }
        } else if (0 == oal_strcmp("pdet", ac_name)) {
            st_phy_debug_switch.en_pdet_debug_switch = ((oal_uint8)oal_atoi(ac_value)) & OAL_TRUE;
        } else if (0 == oal_strcmp("force_work", ac_name)) {
            st_phy_debug_switch.uc_force_work_switch = ((oal_uint8)oal_atoi(ac_value));
        } else if (0 == oal_strcmp("dfr_reset", ac_name)) {
            st_phy_debug_switch.uc_dfr_reset_switch = ((oal_uint8)oal_atoi(ac_value));
        } else if (0 == oal_strcmp("fsm_info", ac_name)) {
            st_phy_debug_switch.uc_fsm_info_switch = ((oal_uint8)oal_atoi(ac_value)) & OAL_TRUE;
        } else if (0 == oal_strcmp("report_radar", ac_name)) {
            st_phy_debug_switch.uc_report_radar_switch = OAL_TRUE;
        } else if (0 == oal_strcmp("extlna_bypass", ac_name)) {
            st_phy_debug_switch.uc_extlna_chg_bypass_switch = ((oal_uint8)oal_atoi(ac_value));
        } else if (0 == oal_strcmp("edca", ac_name)) {
            st_phy_debug_switch.uc_edca_param_switch |= ((oal_uint8)oal_atoi(ac_value)) << 4;
        } else if (0 == oal_strcmp("aifsn", ac_name)) {
            st_phy_debug_switch.uc_edca_param_switch |= (oal_uint8)BIT3;
            st_phy_debug_switch.uc_edca_aifsn = (oal_uint8)oal_atoi(ac_value);
        } else if (0 == oal_strcmp("cwmin", ac_name)) {
            st_phy_debug_switch.uc_edca_param_switch |= (oal_uint8)BIT2;
            st_phy_debug_switch.uc_edca_cwmin = (oal_uint8)oal_atoi(ac_value);
        } else if (0 == oal_strcmp("cwmax", ac_name)) {
            st_phy_debug_switch.uc_edca_param_switch |= (oal_uint8)BIT1;
            st_phy_debug_switch.uc_edca_cwmax = (oal_uint8)oal_atoi(ac_value);
        } else if (0 == oal_strcmp("txoplimit", ac_name)) {
            st_phy_debug_switch.uc_edca_param_switch |= (oal_uint8)BIT0;
            st_phy_debug_switch.us_edca_txoplimit = (oal_uint16)oal_atoi(ac_value);
        } else {
            OAL_IO_PRINT("CMD format::sh hipriv.sh 'wlan0 phy_debug [rssi 0|1] [snr 0|1] [trlr xxxxx] [vect yyyyy] [count N] [edca 0-3(tid_no)] [aifsn N] [cwmin N] [cwmax N] [txoplimit N]'");
            OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 phy_debug [rssi 0|1] [snr 0|1] [trlr xxxx] [vect yyyy] [count N] [edca 0-3(tid_no)] [aifsn N] [cwmin N] [cwmax N] [txoplimit N]'!!}\r\n");
            return OAL_FAIL;
        }
    } while (*pc_param != '\0');

    /* ����ӡ�ܿ��ر��浽 */
    st_phy_debug_switch.en_debug_switch = st_phy_debug_switch.en_rssi_debug_switch | st_phy_debug_switch.en_snr_debug_switch | st_phy_debug_switch.en_trlr_debug_switch \
                                          | st_phy_debug_switch.uc_iq_cali_switch | st_phy_debug_switch.en_tsensor_debug_switch | st_phy_debug_switch.en_evm_debug_switch;

    OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_phy_debug_switch:: phy_debug switch [%d].}\r\n", st_phy_debug_switch.en_debug_switch);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PHY_DEBUG_SWITCH, OAL_SIZEOF(st_phy_debug_switch));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_phy_debug_switch,
                        OAL_SIZEOF(st_phy_debug_switch))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_phy_debug_switch::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_phy_debug_switch),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_phy_debug_switch::return err code[%d]!}", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC OAL_INLINE oal_void protocol_debug_cmd_format_info(void)
{
    OAM_WARNING_LOG0(0, OAM_SF_ANY, "{CMD format::sh hipriv.sh 'wlan0 protocol_debug\
    [band_force_switch 0|1|2(20M|40M+|40M-)]\
    [2040_ch_swt_prohi 0|1]\
    [40_intol 0|1]'!!}\r\n");
    OAM_WARNING_LOG0(0, OAM_SF_ANY,
                     "{[csa 0(csa mode) 1(csa channel) 10(csa cnt) 1(debug  flag,0:normal channel channel,1:only include csa ie 2:cannel debug)]\
    [2040_user_switch 0|1]'!!}\r\n");
    OAM_WARNING_LOG0(0, OAM_SF_ANY,
                     "[lsig 0|1]\
    '!!}\r\n");
}


OAL_STATIC oal_uint32 wal_protocol_debug_parase_csa_cmd(oal_int8 *pc_param, mac_protocol_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_value;

    *pul_offset = 0;
    /* ����csa mode */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::get csa mode error,return.}");
        return ul_ret;
    }
    uc_value = (oal_uint8)oal_atoi(ac_value);
    if (uc_value > 1) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::csa mode = [%d] invalid,return.}", uc_value);
        return OAL_FAIL;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_csa_debug_bit3.en_mode = uc_value;
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����csa channel */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::get csa channel error,return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_csa_debug_bit3.uc_channel = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����bandwidth */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::get bandwidth error,return.}");
        return ul_ret;
    }
    uc_value = (oal_uint8)oal_atoi(ac_value);
    if (uc_value >= WLAN_BAND_WIDTH_BUTT) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::invalid bandwidth = %d,return.}", uc_value);
        return OAL_FAIL;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_csa_debug_bit3.en_bandwidth = uc_value;
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����csa cnt */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::get csa cnt error,return.}");
        return ul_ret;
    }
    uc_value = (oal_uint8)oal_atoi(ac_value);
    if (uc_value >= 255) {
        uc_value = 255;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_csa_debug_bit3.uc_cnt = uc_value;
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����debug flag */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::get debug flag error,return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    uc_value = (oal_uint8)oal_atoi(ac_value);
    if (uc_value >= MAC_CSA_FLAG_BUTT) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_protocol_debug_parase_csa_cmd::invalid debug flag = %d,return.}", uc_value);
        return OAL_FAIL;
    }
    pst_debug_info->st_csa_debug_bit3.en_debug_flag = (mac_csa_flag_enum_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_protocol_debug_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    mac_protocol_debug_switch_stru st_protocol_debug;
    oal_uint32 ul_ret = 0;
    oal_int32 l_ret;
    oal_bool_enum_uint8 en_cmd_updata = OAL_FALSE;

    /* sh hipriv.sh "wlan0 protocol_debug band_force_switch 0|1|2(20|40-|40+) 2040_ch_swt_prohi 0|1(�ر�|��) 2040_intolerant 0|1(�ر�|��)" */
    memset_s(&st_protocol_debug, OAL_SIZEOF(st_protocol_debug), 0, OAL_SIZEOF(st_protocol_debug));

    do {
        /* ��ȡ����ؼ��� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if ((ul_ret != OAL_SUCC) && (ul_off_set != 0)) {
            protocol_debug_cmd_format_info();
            return ul_ret;
        }
        pc_param += ul_off_set;

        if (en_cmd_updata == OAL_FALSE) {
            en_cmd_updata = OAL_TRUE;
        } else if (ul_off_set == 0) {
            break;
        }

        /* ������� */
        if (0 == oal_strcmp("band_force_switch", ac_name)) {
            /* ȡ��������ֵ */
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
            if ((ul_ret != OAL_SUCC) || (ac_value[0]) < '0' || (ac_value[0] > '9')) {
                protocol_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            /* ��д�ṹ�� */
            st_protocol_debug.en_band_force_switch_bit0 = ((oal_uint8)oal_atoi(ac_value));
            st_protocol_debug.ul_cmd_bit_map |= BIT0;
        } else if (0 == oal_strcmp("2040_ch_swt_prohi", ac_name)) {
            /* ȡ��������ֵ */
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
            if ((ul_ret != OAL_SUCC) || OAL_VALUE_NOT_IN_VALID_RANGE(ac_value[0], '0', '9')) {
                protocol_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            /* ��д�ṹ�� */
            st_protocol_debug.en_2040_ch_swt_prohi_bit1 = ((oal_uint8)oal_atoi(ac_value)) & BIT0;
            st_protocol_debug.ul_cmd_bit_map |= BIT1;
        } else if (0 == oal_strcmp("40_intol", ac_name)) {
            /* ȡ��������ֵ */
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
            if ((ul_ret != OAL_SUCC) || OAL_VALUE_NOT_IN_VALID_RANGE(ac_value[0], '0', '9')) {
                protocol_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            /* ��д�ṹ�� */
            st_protocol_debug.en_40_intolerant_bit2 = ((oal_uint8)oal_atoi(ac_value)) & BIT0;
            st_protocol_debug.ul_cmd_bit_map |= BIT2;
        } else if (0 == oal_strcmp("csa", ac_name)) {
            ul_ret = wal_protocol_debug_parase_csa_cmd(pc_param, &st_protocol_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                protocol_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_protocol_debug.ul_cmd_bit_map |= BIT3;
        }
#ifdef _PRE_WLAN_FEATURE_HWBW_20_40
        else if (0 == oal_strcmp("2040_user_switch", ac_name)) {
            /* ȡ��������ֵ */
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
            if ((ul_ret != OAL_SUCC) || (ac_value[0]) < '0' || (ac_value[0]) > '1' || (ac_value[1] != '\0')) {
                protocol_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            /* ��д�ṹ�� */
            st_protocol_debug.en_2040_user_switch_bit4 = ((oal_uint8)oal_atoi(ac_value)) & BIT0;
            st_protocol_debug.ul_cmd_bit_map |= BIT4;
        }
#endif
        else if (0 == oal_strcmp("lsig", ac_name)) {
            /* ȡ��������ֵ */
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
            if ((ul_ret != OAL_SUCC) || (ac_value[0]) < '0' || (ac_value[0] > '9')) {
                protocol_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            /* ��д�ṹ�� */
            st_protocol_debug.en_lsigtxop_bit5 = ((oal_uint8)oal_atoi(ac_value)) & BIT0;
            st_protocol_debug.ul_cmd_bit_map |= BIT5;
        }
#ifdef _PRE_WLAN_FEATURE_11AX
        else if (0 == oal_strcmp("11ax", ac_name)) {
            st_protocol_debug.ul_cmd_bit_map |= BIT6;
        }
#endif
        else {
            protocol_debug_cmd_format_info();
            return OAL_FAIL;
        }
    } while (*pc_param != '\0');

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_protocol_debug_info::ul_cmd_bit_map: 0x%08x.}", st_protocol_debug.ul_cmd_bit_map);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PROTOCOL_DBG, OAL_SIZEOF(st_protocol_debug));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_protocol_debug,
                        OAL_SIZEOF(st_protocol_debug))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_show_protocol_debug_info::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_protocol_debug),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_protocol_debug_info::return err code[%d]!}", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)

OAL_STATIC oal_uint32 wal_hipriv_set_pm_debug_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oal_uint32 ul_ret = 0;
    oal_int32 l_ret;
    oal_uint32 ul_offset;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    mac_pm_debug_cfg_stru *pst_pm_debug_cfg = OAL_PTR_NULL;
    oal_uint8 uc_switch;
    oal_bool_enum_uint8 en_cmd_updata = OAL_FALSE;

    /* sh hipriv.sh "wlan0 pm_debug srb " */

    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (pst_mac_vap == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_pm_debug_switch::pst_mac_vap is null!}");
        return OAL_FAIL;
    }

    pst_pm_debug_cfg = (mac_pm_debug_cfg_stru *)(st_write_msg.auc_value);

    memset_s(pst_pm_debug_cfg, OAL_SIZEOF(mac_pm_debug_cfg_stru), 0, OAL_SIZEOF(mac_pm_debug_cfg_stru));

    do {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_offset);
        if ((ul_ret != OAL_SUCC) && (0 != ul_offset)) {
            OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_pm_debug_switch::cmd format err, ret:%d;!!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_offset;

        if (en_cmd_updata == OAL_FALSE) {
            en_cmd_updata = OAL_TRUE;
        } else if (0 == ul_offset) {
            break;
        }

        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_offset);
        if ((ul_ret != OAL_SUCC) || ((!isdigit(ac_value[0])) && (0 != oal_strcmp("help", ac_value)))) {
            OAL_IO_PRINT("CMD format::sh hipriv.sh 'wlan0 pm_debug [srb 0|1]'\r\n");
            OAM_ERROR_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug [srb 0|1]'!!}\r\n");
            return ul_ret;
        }
        pc_param += ul_offset;

        if (0 == oal_strcmp("srb", ac_name)) {
            if (0 == oal_strcmp("help", ac_value)) {
                OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug srb [0|1]'}");
                return OAL_SUCC;
            }

            uc_switch = (oal_uint8)oal_atoi(ac_value);
            if (uc_switch > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug srb [0|1]', input[%d]!!}", uc_switch);
                return OAL_FAIL;
            }
            pst_pm_debug_cfg->ul_cmd_bit_map |= BIT(MAC_PM_DEBUG_SISO_RECV_BCN);
            pst_pm_debug_cfg->uc_srb_switch = uc_switch;

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_pm_debug_switch::siso recv beacon switch[%d].}", uc_switch);
        } else if (0 == oal_strcmp("dto", ac_name)) {
            if (0 == oal_strcmp("help", ac_value)) {
                OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug dto [0|1]'}");
                return OAL_SUCC;
            }

            uc_switch = (oal_uint8)oal_atoi(ac_value);
            if (uc_switch > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug dto [0|1]', input[%d]!!}", uc_switch);
                return OAL_FAIL;
            }
            pst_pm_debug_cfg->ul_cmd_bit_map |= BIT(MAC_PM_DEBUG_DYN_TBTT_OFFSET);
            pst_pm_debug_cfg->uc_dto_switch = uc_switch;

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_pm_debug_switch::dyn tbtt offset switch[%d].}", uc_switch);
        } else if (0 == oal_strcmp("nfi", ac_name)) {
            if (0 == oal_strcmp("help", ac_value)) {
                OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug nfi [0|1]'}");
                return OAL_SUCC;
            }

            uc_switch = (oal_uint8)oal_atoi(ac_value);
            if (uc_switch > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug nfi [0|1]', input[%d]!!}", uc_switch);
                return OAL_FAIL;
            }
            pst_pm_debug_cfg->ul_cmd_bit_map |= BIT(MAC_PM_DEBUG_NO_PS_FRM_INT);
            pst_pm_debug_cfg->uc_nfi_switch = uc_switch;

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_pm_debug_switch::no ps frm int switch[%d].}", uc_switch);
        } else if (0 == oal_strcmp("apf", ac_name)) {
            if (0 == oal_strcmp("help", ac_value)) {
                OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug apf [0|1]'}");
                return OAL_SUCC;
            }

            uc_switch = (oal_uint8)oal_atoi(ac_value);
            if (uc_switch > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug apf [0|1]', input[%d]!!}", uc_switch);
                return OAL_FAIL;
            }
            pst_pm_debug_cfg->ul_cmd_bit_map |= BIT(MAC_PM_DEBUG_APF);
            pst_pm_debug_cfg->uc_apf_switch = uc_switch;

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_pm_debug_switch::apf switch[%d].}", uc_switch);
        } else if (0 == oal_strcmp("ao", ac_name)) {
            if (0 == oal_strcmp("help", ac_value)) {
                OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug ao [0|1]'}");
                return OAL_SUCC;
            }

            uc_switch = (oal_uint8)oal_atoi(ac_value);
            if (uc_switch > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug ao [0|1]', input[%d]!!}", uc_switch);
                return OAL_FAIL;
            }
            pst_pm_debug_cfg->ul_cmd_bit_map |= BIT(MAC_PM_DEBUG_AO);
            pst_pm_debug_cfg->uc_ao_switch = uc_switch;

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_pm_debug_switch::arp offload switch[%d].}", uc_switch);
        } else {
            OAL_IO_PRINT("CMD format::sh hipriv.sh 'wlan0 pm_debug [srb 0|1] [dto 0|1] [nfi 0|1] [apf 0|1] [ao 0|1]'");
            OAM_ERROR_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 pm_debug [srb 0|1] [dto 0|1] [nfi 0|1] [apf 0|1]'!!}");
            return OAL_FAIL;
        }
    } while (*pc_param != '\0');

    /***************************************************************************
                               ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PM_DEBUG_SWITCH, OAL_SIZEOF(mac_pm_debug_cfg_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_pm_debug_cfg_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_dbdc_debug_switch::return err code[%d]!}", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_DBDC

OAL_STATIC oal_uint32 wal_hipriv_set_dbdc_debug_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    mac_dbdc_debug_switch_stru st_dbdc_debug_switch;
    mac_vap_stru *pst_mac_vap;
    oal_uint32 ul_ret = 0;
    oal_int32 l_ret;
    oal_uint8 uc_dst_hal_dev_id;
    oal_uint8 en_dbdc_enable;
    oal_uint8 en_fast_scan_enable;
    oal_bool_enum_uint8 en_cmd_updata = OAL_FALSE;

    /* sh hipriv.sh "wlan0 dbdc_debug change_hal_dev 0|1(hal 0|hal 1)" */

    memset_s(&st_dbdc_debug_switch, OAL_SIZEOF(st_dbdc_debug_switch), 0, OAL_SIZEOF(st_dbdc_debug_switch));

    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (pst_mac_vap == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_dbdc_debug_switch::pst_mac_vap is null!}");
        return OAL_FAIL;
    }

    do {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if ((ul_ret != OAL_SUCC) && (ul_off_set != 0)) {
            OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_dbdc_debug_switch::cmd format err, ret:%d;!!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;

        if (en_cmd_updata == OAL_FALSE) {
            en_cmd_updata = OAL_TRUE;
        } else if (ul_off_set == 0) {
            break;
        }

        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
        if ((ul_ret != OAL_SUCC) || ((!isdigit(ac_value[0])) && (0 != oal_strcmp("help", ac_value)))) {
            OAL_IO_PRINT("CMD format::sh hipriv.sh 'wlan0 dbdc_debug [change_hal_dev 0|1]'\r\n");
            OAM_ERROR_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug [change_hal_dev 0|1]'!!}\r\n");
            return ul_ret;
        }
        pc_param += ul_off_set;
        ul_off_set = 0;

        if (0 == oal_strcmp("help", ac_value)) {
            OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug [change_hal_dev 0|1] .!!}");
            OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug dbdc_enable 0|1].}");
            OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug fast_scan 0|1].}");
            OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug dbdc_status 1' !}");
            return OAL_SUCC;
        }

        if (0 == oal_strcmp("change_hal_dev", ac_name)) {
            uc_dst_hal_dev_id = (oal_uint8)oal_atoi(ac_value);
            if (uc_dst_hal_dev_id > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug [change_hal_dev 0|1],input[%d]'!!}", uc_dst_hal_dev_id);
                return OAL_FAIL;
            }
            st_dbdc_debug_switch.ul_cmd_bit_map |= BIT(MAC_DBDC_CHANGE_HAL_DEV);
            st_dbdc_debug_switch.uc_dst_hal_dev_id = ((oal_uint8)oal_atoi(ac_value));

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_dbdc_debug_switch::change to hal device[%d].}", st_dbdc_debug_switch.uc_dst_hal_dev_id);
        } else if (0 == oal_strcmp("dbdc_enable", ac_name)) {
            en_dbdc_enable = (oal_uint8)oal_atoi(ac_value);
            if (en_dbdc_enable > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug [change_hal_dev 0|1],input[%d]'!!}", en_dbdc_enable);
                return OAL_FAIL;
            }
            st_dbdc_debug_switch.ul_cmd_bit_map |= BIT(MAC_DBDC_SWITCH);
            st_dbdc_debug_switch.uc_dbdc_enable = ((oal_uint8)oal_atoi(ac_value));

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_dbdc_debug_switch::dbdc enable[%d].}", st_dbdc_debug_switch.uc_dbdc_enable);
        }

        else if (0 == oal_strcmp("fast_scan", ac_name)) {
            en_fast_scan_enable = (oal_uint8)oal_atoi(ac_value);
            if (en_fast_scan_enable > 1) {
                OAM_ERROR_LOG1(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug [fast_scan 0|1],input[%d]'!!}", en_fast_scan_enable);
                return OAL_FAIL;
            }
            st_dbdc_debug_switch.ul_cmd_bit_map |= BIT(MAC_FAST_SCAN_SWITCH);
            st_dbdc_debug_switch.en_fast_scan_enable = ((oal_uint8)oal_atoi(ac_value));

            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_dbdc_debug_switch::fast_scan enable[%d].}", st_dbdc_debug_switch.en_fast_scan_enable);
        } else if (0 == oal_strcmp("dbdc_status", ac_name)) {
            st_dbdc_debug_switch.ul_cmd_bit_map |= BIT(MAC_DBDC_STATUS);
            st_dbdc_debug_switch.uc_dbdc_status = ((oal_uint8)oal_atoi(ac_value));

            OAM_WARNING_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{wal_hipriv_set_dbdc_debug_switch::dbdc_status enquiry.}");
        } else {
            OAL_IO_PRINT("CMD format::sh hipriv.sh 'wlan0 dbdc_debug [change_hal_dev 0|1]'");
            OAM_ERROR_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 dbdc_debug [change_hal_dev 0|1]'!!}");
            return OAL_FAIL;
        }
    } while (*pc_param != '\0');

    /***************************************************************************
                            ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DBDC_DEBUG_SWITCH, OAL_SIZEOF(st_dbdc_debug_switch));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_dbdc_debug_switch,
                        OAL_SIZEOF(st_dbdc_debug_switch))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_dbdc_debug_switch::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_dbdc_debug_switch),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_dbdc_debug_switch::return err code[%d]!}", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_MEM_TRACE

OAL_STATIC oal_uint32 wal_hipriv_mem_trace_info_show(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_mode;
    oal_uint32 ul_fileid = ~0;
    oal_uint32 ul_line = 0;

    /* ��ȡ���� sh hipriv.sh "Hisilicon0 mem_trace_show 0 (file_id,line) | 1 | 2" */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }

    ul_mode = (oal_uint32)oal_atoi((const oal_int8 *)ac_name);
    if (ul_mode == 0) {
        pc_param += ul_off_set;
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret == OAL_SUCC) {
            ul_fileid = (oal_uint32)oal_atoi((const oal_int8 *)ac_name);

            pc_param += ul_off_set;
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret == OAL_SUCC) {
                ul_line = (oal_uint32)oal_atoi((const oal_int8 *)ac_name);
            }
        }
    }

    mem_trace_info_show(ul_mode, ul_fileid, ul_line);
    return OAL_SUCC;
}

#endif


OAL_STATIC oal_uint32 wal_hipriv_set_probe_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_cfg_probe_switch_stru st_probe_switch;

    /*
     * sh hipriv.sh "Hisilicon0 probe_switch 0|1(֡����tx|rx) 0|1(֡���ݿ���)
     * 0|1(CB����) 0|1(����������)"
     */
    /* ��ȡ֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_probe_switch::get probe direction return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_probe_switch.en_frame_direction = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡���ݴ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_probe_switch::get probe frame content switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_probe_switch.en_frame_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ֡CB�ֶδ�ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_probe_switch::get probe frame cb switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_probe_switch.en_cb_switch = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ��������ӡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_probe_switch::get probe frame dscr switch return err_code[%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    st_probe_switch.en_dscr_switch = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PROBE_SWITCH, OAL_SIZEOF(st_probe_switch));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_probe_switch,
                        OAL_SIZEOF(st_probe_switch))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_probe_switch),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_80211_ucast_switch::return err code[%d]!}\r\n", ul_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_get_mpdu_num(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_cfg_get_mpdu_num_stru st_param;

    /* sh hipriv.sh "vap_name mpdu_num user_macaddr" */

    memset_s(&st_param, OAL_SIZEOF(mac_cfg_get_mpdu_num_stru), 0, OAL_SIZEOF(mac_cfg_get_mpdu_num_stru));

    /* ��ȡ�û�mac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, st_param.auc_user_macaddr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_GET_MPDU_NUM, OAL_SIZEOF(st_param));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_param, OAL_SIZEOF(st_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_dpd_cfg::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_all_ota(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_param;
    wal_msg_write_stru st_write_msg;

    /* ��ȡ���� sh hipriv.sh "Hisilicon0 set_all_ota 0|1" */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }

    l_param = oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_ALL_OTA, OAL_SIZEOF(oal_uint32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_param;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_all_ota::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_oam_output(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;

    /* OAM logģ��Ŀ��ص�����: hipriv "Hisilicon0 log_level 0~3"
        �˴���������"1"��"0"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_oam_output::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����logģ����в�ͬ������ ȡֵ:oam_output_type_enum_uint8 */
    l_tmp = oal_atoi(ac_name);
    if (l_tmp >= OAM_OUTPUT_TYPE_BUTT) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_oam_output::output type invalid [%d]!}\r\n", l_tmp);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_OAM_OUTPUT_TYPE, OAL_SIZEOF(oal_int32));
    /* ��������������� */
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_oam_output::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_auto_ba_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* �����Զ���ʼBA�Ự�Ŀ���:hipriv "vap0  auto_ba 0 | 1" ���������ĳһ��VAP */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_auto_ba_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����AUTO BA���в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_auto_ba_switch::the auto ba switch command is error[%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_AUTO_BA_SWITCH, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_auto_ba_switch::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_profiling_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* �����Զ���ʼBA�Ự�Ŀ���:hipriv "vap0  profiling 0 | 1" ���������ĳһ��VAP */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_profiling_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����AUTO BA���в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_profiling_switch::the profiling switch command is error[%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PROFILING_SWITCH, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_profiling_switch::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_addba_req(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_addba_req_param_stru *pst_addba_req_param = OAL_PTR_NULL;
    mac_cfg_addba_req_param_stru st_addba_req_param; /* ��ʱ�����ȡ��addba req����Ϣ */
    oal_uint32 ul_get_addr_idx;

    /*
     * ����AMPDU�رյ���������:
     * hipriv "Hisilicon0 addba_req xx xx xx xx xx xx(mac��ַ) tidno ba_policy buffsize timeout"
     */
    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    memset_s((oal_uint8 *)&st_addba_req_param, OAL_SIZEOF(st_addba_req_param), 0, OAL_SIZEOF(st_addba_req_param));
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_addba_req_param.auc_mac_addr, WLAN_MAC_ADDR_LEN);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡtid */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    if (OAL_STRLEN(ac_name) > 2) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::the addba req command is error[%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    st_addba_req_param.uc_tidno = (oal_uint8)oal_atoi(ac_name);
    if (st_addba_req_param.uc_tidno >= WLAN_TID_MAX_NUM) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::the addba req command is error!uc_tidno is [%d]!}\r\n", st_addba_req_param.uc_tidno);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pc_param = pc_param + ul_off_set;

    /* ��ȡba_policy */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_addba_req_param.en_ba_policy = (oal_uint8)oal_atoi(ac_name);
    if (st_addba_req_param.en_ba_policy != MAC_BA_POLICY_IMMEDIATE) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::the ba policy is not correct! ba_policy is[%d]!}\r\n", st_addba_req_param.en_ba_policy);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pc_param = pc_param + ul_off_set;

    /* ��ȡbuffsize */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_addba_req_param.us_buff_size = (oal_uint16)oal_atoi(ac_name);

    pc_param = pc_param + ul_off_set;

    /* ��ȡtimeoutʱ�� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_addba_req_param.us_timeout = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ADDBA_REQ, OAL_SIZEOF(mac_cfg_addba_req_param_stru));

    /* ��������������� */
    pst_addba_req_param = (mac_cfg_addba_req_param_stru *)(st_write_msg.auc_value);
    for (ul_get_addr_idx = 0; ul_get_addr_idx < WLAN_MAC_ADDR_LEN; ul_get_addr_idx++) {
        pst_addba_req_param->auc_mac_addr[ul_get_addr_idx] = st_addba_req_param.auc_mac_addr[ul_get_addr_idx];
    }

    pst_addba_req_param->uc_tidno = st_addba_req_param.uc_tidno;
    pst_addba_req_param->en_ba_policy = st_addba_req_param.en_ba_policy;
    pst_addba_req_param->us_buff_size = st_addba_req_param.us_buff_size;
    pst_addba_req_param->us_timeout = st_addba_req_param.us_timeout;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_addba_req_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addba_req::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_delba_req(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_delba_req_param_stru *pst_delba_req_param = OAL_PTR_NULL;
    mac_cfg_delba_req_param_stru st_delba_req_param; /* ��ʱ�����ȡ��addba req����Ϣ */
    oal_uint32 ul_get_addr_idx;

    /*
     * ����AMPDU�رյ���������:
     * hipriv "Hisilicon0 delba_req xx xx xx xx xx xx(mac��ַ) tidno direction reason_code"
     */
    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    memset_s((oal_uint8 *)&st_delba_req_param, OAL_SIZEOF(st_delba_req_param), 0, OAL_SIZEOF(st_delba_req_param));
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_delba_req_param.auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡtid */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    if (OAL_STRLEN(ac_name) > 2) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delba_req::the delba_req req command is error[%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    st_delba_req_param.uc_tidno = (oal_uint8)oal_atoi(ac_name);
    if (st_delba_req_param.uc_tidno >= WLAN_TID_MAX_NUM) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delba_req::the delba_req req command is error! uc_tidno is[%d]!}\r\n", st_delba_req_param.uc_tidno);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pc_param = pc_param + ul_off_set;

    /* ��ȡdirection */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delba_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_delba_req_param.en_direction = (oal_uint8)oal_atoi(ac_name);
    if (st_delba_req_param.en_direction >= MAC_BUTT_DELBA) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delba_req::the direction is not correct! direction is[%d]!}\r\n", st_delba_req_param.en_direction);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DELBA_REQ, OAL_SIZEOF(mac_cfg_delba_req_param_stru));

    /* ��������������� */
    pst_delba_req_param = (mac_cfg_delba_req_param_stru *)(st_write_msg.auc_value);
    for (ul_get_addr_idx = 0; ul_get_addr_idx < WLAN_MAC_ADDR_LEN; ul_get_addr_idx++) {
        pst_delba_req_param->auc_mac_addr[ul_get_addr_idx] = st_delba_req_param.auc_mac_addr[ul_get_addr_idx];
    }

    pst_delba_req_param->uc_tidno = st_delba_req_param.uc_tidno;
    pst_delba_req_param->en_direction = st_delba_req_param.en_direction;
    pst_delba_req_param->en_trigger = MAC_DELBA_TRIGGER_COMM;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_delba_req_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delba_req::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_WMMAC

OAL_STATIC oal_uint32 wal_hipriv_addts_req(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_wmm_tspec_stru_param_stru *pst_addts_req_param;
    mac_cfg_wmm_tspec_stru_param_stru st_addts_req_param; /* ��ʱ�����ȡ��addts req����Ϣ */

    /*
     * ���÷���ADDTS REQ��������:
     * hipriv "vap0 addts_req tid direction psb up nominal_msdu_size maximum_data_rate
     * minimum_data_rate mean_data_rate peak_data_rate minimum_phy_rate surplus_bandwidth_allowance"
     */
    /***********************************************************************************************
    TSPEC�ֶ�:
          --------------------------------------------------------------------------------------
          |TS Info|Nominal MSDU Size|Max MSDU Size|Min Serv Itvl|Max Serv Itvl|
          ---------------------------------------------------------------------------------------
    Octets:  | 3     |  2              |   2         |4            |4            |
          ---------------------------------------------------------------------------------------
          | Inactivity Itvl | Suspension Itvl | Serv Start Time |Min Data Rate | Mean Data Rate |
          ---------------------------------------------------------------------------------------
    Octets:  |4                | 4               | 4               |4             |  4             |
          ---------------------------------------------------------------------------------------
          |Peak Data Rate|Burst Size|Delay Bound|Min PHY Rate|Surplus BW Allowance  |Medium Time|
          ---------------------------------------------------------------------------------------
    Octets:  |4             |4         | 4         | 4          |  2                   |2          |
          ---------------------------------------------------------------------------------------

    TS info�ֶ�:
          ---------------------------------------------------------------------------------------
          |Reserved |TSID |Direction |1 |0 |Reserved |PSB |UP |Reserved |Reserved |Reserved |
          ---------------------------------------------------------------------------------------
    Bits:  |1        |4    |2         |  2  |1        |1   |3  |2        |1        |7        |
          ----------------------------------------------------------------------------------------
    ***********************************************************************************************/
    /* ��ȡtid��ȡֵ��Χ0~7 */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    if (OAL_STRLEN(ac_name) > 2) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::the addba req command is error[%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    st_addts_req_param.ts_info.bit_tsid = (oal_uint16)oal_atoi(ac_name);
    if (st_addts_req_param.ts_info.bit_tsid >= WLAN_TID_MAX_NUM) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::the addts req command is error!uc_tidno is [%d]!}\r\n", st_addts_req_param.ts_info.bit_tsid);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pc_param = pc_param + ul_off_set;

    /* ��ȡdirection 00:uplink 01:downlink 10:reserved 11:Bi-directional */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_addts_req_param.ts_info.bit_direction = (oal_uint16)oal_atoi(ac_name);
    if (st_addts_req_param.ts_info.bit_direction == MAC_WMMAC_DIRECTION_RESERVED) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::the direction is not correct! direction is[%d]!}\r\n", st_addts_req_param.ts_info.bit_direction);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pc_param = pc_param + ul_off_set;

    /* ��ȡPSB��1��ʾU-APSD��0��ʾlegacy */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_addts_req_param.ts_info.bit_apsd = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡUP */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.ts_info.bit_user_prio = (oal_uint16)oal_atoi(ac_name);

    pc_param = pc_param + ul_off_set;

    /* ��ȡNominal MSDU Size ,��һλΪ1 */
    /*
        ------------
        |fixed|size|
        ------------
    bits:  |1    |15  |
        ------------
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.us_norminal_msdu_size = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡmaximum MSDU size */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.us_max_msdu_size = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡminimum data rate */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.ul_min_data_rate = (oal_uint32)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡmean data rate */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.ul_mean_data_rate = (oal_uint32)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡpeak data rate */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.ul_peak_data_rate = (oal_uint32)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡminimum PHY Rate */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.ul_min_phy_rate = (oal_uint32)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡsurplus bandwidth allowance */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_addts_req_param.us_surplus_bw = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ADDTS_REQ, OAL_SIZEOF(mac_cfg_wmm_tspec_stru_param_stru));

    /* ��������������� */
    pst_addts_req_param = (mac_cfg_wmm_tspec_stru_param_stru *)(st_write_msg.auc_value);

    pst_addts_req_param->ts_info.bit_tsid = st_addts_req_param.ts_info.bit_tsid;
    pst_addts_req_param->ts_info.bit_direction = st_addts_req_param.ts_info.bit_direction;
    pst_addts_req_param->ts_info.bit_apsd = st_addts_req_param.ts_info.bit_apsd;
    pst_addts_req_param->ts_info.bit_user_prio = st_addts_req_param.ts_info.bit_user_prio;
    pst_addts_req_param->us_norminal_msdu_size = st_addts_req_param.us_norminal_msdu_size;
    pst_addts_req_param->us_max_msdu_size = st_addts_req_param.us_max_msdu_size;
    pst_addts_req_param->ul_min_data_rate = st_addts_req_param.ul_min_data_rate;
    pst_addts_req_param->ul_mean_data_rate = st_addts_req_param.ul_mean_data_rate;
    pst_addts_req_param->ul_peak_data_rate = st_addts_req_param.ul_peak_data_rate;
    pst_addts_req_param->ul_min_phy_rate = st_addts_req_param.ul_min_phy_rate;
    pst_addts_req_param->us_surplus_bw = st_addts_req_param.us_surplus_bw;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_wmm_tspec_stru_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_addts_req::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_delts(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_wmm_tspec_stru_param_stru *pst_delts_param;
    mac_cfg_wmm_tspec_stru_param_stru st_delts_param;

    /* ����ɾ��TS����������: hipriv "Hisilicon0 delts tidno" */
    /* ��ȡtsid */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delts::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_delts_param.ts_info.bit_tsid = (oal_uint8)oal_atoi(ac_name);
    if (st_delts_param.ts_info.bit_tsid >= WLAN_TID_MAX_NUM) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delts::the delts command is error! tsid is[%d]!}\r\n", st_delts_param.ts_info.bit_tsid);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DELTS, OAL_SIZEOF(mac_cfg_wmm_tspec_stru_param_stru));

    /* ��������������� */
    pst_delts_param = (mac_cfg_wmm_tspec_stru_param_stru *)(st_write_msg.auc_value);
    memset_s(pst_delts_param, OAL_SIZEOF(mac_cfg_wmm_tspec_stru_param_stru), 0, OAL_SIZEOF(mac_cfg_wmm_tspec_stru_param_stru));

    pst_delts_param->ts_info.bit_tsid = st_delts_param.ts_info.bit_tsid;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_wmm_tspec_stru_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delts::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_wmmac_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_wmmac_switch;
    mac_cfg_wmm_ac_param_stru st_wmm_ac_param;

    /* ����ɾ��TS����������: hipriv "vap0 wmmac_switch 1/0(ʹ��) 0|1(WMM_AC��֤ʹ��) AC xxx(limit_medium_time)" */

    memset_s(&st_wmm_ac_param, OAL_SIZEOF(mac_cfg_wmm_ac_param_stru), 0, OAL_SIZEOF(mac_cfg_wmm_ac_param_stru));
    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wmmac_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    uc_wmmac_switch = (oal_uint8)oal_atoi(ac_name);
    if (uc_wmmac_switch != OAL_FALSE) {
        uc_wmmac_switch = OAL_TRUE;
    }
    st_wmm_ac_param.en_wmm_ac_switch = uc_wmmac_switch;

    /* ��ȡauth flag */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wmmac_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_wmm_ac_param.en_auth_flag = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /* timeout period ms */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wmmac_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_wmm_ac_param.us_timeout_period = (oal_uint16)oal_atoi(ac_name);
    pc_param += ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wmmac_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_wmm_ac_param.uc_factor = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    /* ��������������� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_WMMAC_SWITCH, OAL_SIZEOF(st_wmm_ac_param));
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_wmm_ac_param,
                        OAL_SIZEOF(st_wmm_ac_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_delts::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_wmm_ac_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_delts::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_reassoc_req(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
        ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_REASSOC_REQ, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_reassoc_req::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif  // #ifdef _PRE_WLAN_FEATURE_WMMAC


OAL_STATIC oal_uint32 wal_hipriv_mem_info(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    oal_int8 auc_token[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_mem_pool_id_enum_uint8 en_pool_id;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;

    /* ��μ�� */
    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_cfg_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_mem_info::pst_net_dev or pc_param null ptr error [%x] [%x]!}\r\n", (uintptr_t)pst_cfg_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡ�ڴ��ID */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, auc_token, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_mem_info::wal_get_cmd_one_arg_etc return error code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    en_pool_id = (oal_mem_pool_id_enum_uint8)oal_atoi(auc_token);

    /* ��ӡ�ڴ����Ϣ */
    oal_mem_info_etc(en_pool_id);

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_mem_leak(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    oal_int8 auc_token[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_mem_pool_id_enum_uint8 en_pool_id;
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;

    /* ��μ�� */
    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_cfg_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_mem_leak::pst_net_dev or pc_param null ptr error [%x] [%x]!}\r\n", (uintptr_t)pst_cfg_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡ�ڴ��ID */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, auc_token, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_mem_leak::wal_get_cmd_one_arg_etc return error code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    en_pool_id = (oal_mem_pool_id_enum_uint8)oal_atoi(auc_token);
    if (en_pool_id > OAL_MEM_POOL_ID_SDT_NETBUF) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_mem_leak::mem pool id exceeds,en_pool_id[%d]!}\r\n", en_pool_id);
        return OAL_SUCC;
    }

    /* ����ڴ��й©�ڴ�� */
    oal_mem_leak_etc(en_pool_id);

    return OAL_SUCC;
}

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)

OAL_STATIC oal_uint32 wal_hipriv_device_mem_leak(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_pool_id;
    mac_device_pool_id_stru *pst_pool_id_param = OAL_PTR_NULL;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_device_mem_leak::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_pool_id = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DEVICE_MEM_LEAK, OAL_SIZEOF(mac_device_pool_id_stru));

    /* ��������������� */
    pst_pool_id_param = (mac_device_pool_id_stru *)(st_write_msg.auc_value);
    pst_pool_id_param->uc_pool_id = uc_pool_id;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_device_pool_id_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_PWR, "{wal_hipriv_device_mem_leak::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_memory_info(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_meminfo_type = MAC_MEMINFO_BUTT;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru st_write_msg;
    mac_cfg_meminfo_stru *pst_meminfo_param = OAL_PTR_NULL;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{CMD format::sh hipriv.sh 'wlan0 memoryinfo [all|dscr|netbuff|user|vap|sdio_sch_q] [pool_usage/pool_debug 0|1,..7<pool_id>]'!}\r\n");
        return ul_ret;
    }

    if (0 == (oal_strcmp("host", ac_name))) {
        oal_mem_print_pool_info_etc();
        return OAL_SUCC;
    } else if (0 == (oal_strcmp("device", ac_name))) {
        hcc_print_device_mem_info_etc();
        return OAL_SUCC;
    } else if (0 == (oal_strcmp("all", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_ALL;
    } else if (0 == (oal_strcmp("dscr", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_DSCR;
    } else if (0 == (oal_strcmp("netbuff", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_NETBUFF;
    } else if (0 == (oal_strcmp("user", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_USER;
    } else if (0 == (oal_strcmp("vap", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_VAP;
    } else if (0 == (oal_strcmp("sdio_sch_q", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_SDIO_TRX;
    } else if (0 == (oal_strcmp("pool_usage", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_POOL_INFO;
    } else if (0 == (oal_strcmp("pool_debug", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_POOL_DBG;
    } else if (0 == (oal_strcmp("sample_alloc", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_SAMPLE_ALLOC;
    } else if (0 == (oal_strcmp("sample_free", ac_name))) {
        uc_meminfo_type = MAC_MEMINFO_SAMPLE_FREE;
    } else {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_memory_info::wal_get_cmd_one_arg_etc::second arg:: please check input!}\r\n");
        return OAL_FAIL;
    }

    /***************************************************************************
        ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DEVICE_MEM_INFO, OAL_SIZEOF(mac_cfg_meminfo_stru));
    pst_meminfo_param = (mac_cfg_meminfo_stru *)(st_write_msg.auc_value);
    pst_meminfo_param->uc_meminfo_type = uc_meminfo_type;
    /* host��device mempool������һ�� ��������0xff����defaultֵ */
    pst_meminfo_param->uc_object_index = 0xff;

    if ((uc_meminfo_type == MAC_MEMINFO_POOL_INFO) || (uc_meminfo_type == MAC_MEMINFO_POOL_DBG) || (uc_meminfo_type == MAC_MEMINFO_SAMPLE_ALLOC)) {
        pc_param = pc_param + ul_off_set;

        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        /* û�к����������˳� */
        if (ul_ret == OAL_SUCC) {
            pst_meminfo_param->uc_object_index = (oal_uint8)oal_atoi(ac_name);
        }
    }

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_meminfo_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_memory_info::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif


OAL_STATIC oal_uint32 wal_hipriv_beacon_chain_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_beacon_chain_switch::pst_net_dev or pc_param null ptr error %x, %x!}\r\n", (uintptr_t)pst_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* beaconͨ��(0/1/2)�л����ص�����: hipriv "vap0 beacon_chain_switch 0 | 1 | 2"
        �˴���������"0"��"1"��"2"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_beacon_chain_switch::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ������ò�ͬ��ͨ�� */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else if (0 == (oal_strcmp("2", ac_name))) {
        l_tmp = 2;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_beacon_chain_switch::the beacon chain switch command is error %x!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
        ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_BEACON_CHAIN_SWITCH, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_beacon_chain_switch::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_BTCOEX

OAL_STATIC oal_uint32 wal_hipriv_btcoex_status_print(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));

    /* sh hipriv.sh "vap_name coex_print" */
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_BTCOEX_STATUS_PRINT, OAL_SIZEOF(oal_uint32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_status_print::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_btcoex_preempt_type(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    mac_btcoex_preempt_mgr_stru *pst_btcoex_preempt_mgr;
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));

    /* ��������������� */
    pst_btcoex_preempt_mgr = (mac_btcoex_preempt_mgr_stru *)st_write_msg.auc_value;

    /* 1.��ȡ��һ������: mode */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_preempt_type::wal_get_cmd_one_arg1 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_btcoex_preempt_mgr->uc_cfg_preempt_mode = (oal_uint8)oal_atoi(ac_name);
    if (pst_btcoex_preempt_mgr->uc_cfg_preempt_mode > 6) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_preempt_type:: pst_btcoex_preempt_mgr->uc_cfg_preempt_mode [%d]!}\r\n",
                         pst_btcoex_preempt_mgr->uc_cfg_preempt_mode);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* 2.��ȡ��2������ */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_preempt_type::wal_get_cmd_one_arg2 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* �����ſ����ã�Լ��������Ϊ */
    pst_btcoex_preempt_mgr->uc_cfg_preempt_type = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_BTCOEX_PREEMPT_TYPE, OAL_SIZEOF(mac_btcoex_preempt_mgr_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_btcoex_preempt_mgr_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_preempt_type::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_btcoex_set_perf_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_btcoex_mgr_stru *pst_btcoex_mgr;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };

    /* st_write_msg��������� */
    memset_s(&st_write_msg, OAL_SIZEOF(wal_msg_write_stru), 0, OAL_SIZEOF(wal_msg_write_stru));

    /* ��������������� */
    pst_btcoex_mgr = (mac_btcoex_mgr_stru *)st_write_msg.auc_value;

    /* 1.��ȡ��һ������: mode */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg1 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_btcoex_mgr->uc_cfg_btcoex_mode = (oal_uint8)oal_atoi(ac_name);

    if (pst_btcoex_mgr->uc_cfg_btcoex_mode == 0) {
    } else if (pst_btcoex_mgr->uc_cfg_btcoex_mode == 1) {
        /* ƫ�ƣ�ȡ��һ������ */
        pc_param = pc_param + ul_off_set;

        /* 2.��ȡ�ڶ������� */
        ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg2 return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        pst_btcoex_mgr->uc_cfg_btcoex_type = (oal_uint8)oal_atoi(ac_name);
        if (pst_btcoex_mgr->uc_cfg_btcoex_type > 2) {
            OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param:: pst_btcoex_mgr->uc_cfg_btcoex_type error [%d], [0/1/2]!}\r\n",
                             pst_btcoex_mgr->uc_cfg_btcoex_type);
            return OAL_ERR_CODE_INVALID_CONFIG;
        }

        /* ƫ�ƣ�ȡ��һ������ */
        pc_param = pc_param + ul_off_set;

        if (pst_btcoex_mgr->uc_cfg_btcoex_type == 0) {
            /* 3.��ȡ���������� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg3 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rx_size.en_btcoex_nss = (wlan_nss_enum_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ���ĸ����� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg4 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.threhold.uc_20m_low = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ��������� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg5 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.threhold.uc_20m_high = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ����������: */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg6 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.threhold.uc_40m_low = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ���߸�����: */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg7 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.threhold.us_40m_high = (oal_uint16)oal_atoi(ac_name);
        } else if (pst_btcoex_mgr->uc_cfg_btcoex_type == 1) {
            /* 3.��ȡ���������� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg3 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rx_size.en_btcoex_nss = (wlan_nss_enum_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ���ĸ����� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg3 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rx_size.uc_grade = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ��������� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg4 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rx_size.uc_rx_size0 = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ����������: */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg5 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rx_size.uc_rx_size1 = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ���߸�����: */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg6 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rx_size.uc_rx_size2 = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ�ڰ˸�����: */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg7 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rx_size.uc_rx_size3 = (oal_uint8)oal_atoi(ac_name);
        } else {
            /* 3.��ȡ���������� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg3 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rssi_param.en_rssi_limit_on = (oal_bool_enum_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ���ĸ�����: */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg6 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rssi_param.en_rssi_log_on = (oal_bool_enum_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ��������� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg3 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rssi_param.uc_cfg_rssi_detect_cnt = (oal_uint8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ���������� */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg4 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rssi_param.c_cfg_rssi_detect_m2s_th = (oal_int8)oal_atoi(ac_name);

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* ��ȡ���߸�����: */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::wal_get_cmd_one_arg5 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_btcoex_mgr->pri_data.rssi_param.c_cfg_rssi_detect_s2m_th = (oal_int8)oal_atoi(ac_name);
        }
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param::pst_btcoex_mgr->uc_cfg_btcoex_mode err_code [%d]!}\r\n",
                         pst_btcoex_mgr->uc_cfg_btcoex_mode);
        return OAL_FAIL;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_BTCOEX_SET_PERF_PARAM, OAL_SIZEOF(mac_btcoex_mgr_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_btcoex_mgr_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_btcoex_set_perf_param:: return err code = [%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_PLAT_FEATURE_CUSTOMIZE

OAL_STATIC oal_uint32 wal_hipriv_dev_customize_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_tmp = 0;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    hwifi_get_cfg_params();

    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_COEX, "{wal_hipriv_dev_customize_info::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ���� */
    if (0 == (oal_strcmp("1", ac_name))) {
        ul_tmp = 1;
    }

    /***************************************************************************
        ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SHOW_DEV_CUSTOMIZE_INFOS, OAL_SIZEOF(oal_int32));
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dev_customize_info::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_ONLINE_DPD

OAL_STATIC oal_uint32 wal_hipriv_dpd_cfg(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_dpd_cfg::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DPD, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_reg_write::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dpd_start(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_dpd_start::memcpy fail !}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DPD_START, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_reg_write::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif

#ifdef _PRE_WLAN_FEATURE_NRCOEX
/*
 * �� �� ��  : wal_hipriv_nrcoex_set_prior
 * ��������  : ����NR����WiFi���ȼ�
 */
OAL_STATIC oal_uint32 wal_hipriv_nrcoex_set_prior(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_val;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_nrcoex_set_prior::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    uc_val = (oal_uint8)oal_atoi((const oal_int8 *)ac_name);

    /* ���¼���wal�㴦�� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_NRCOEX_PRIORITY_SET, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint8 *)(st_write_msg.auc_value)) = uc_val;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_nrcoex_set_prior::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_nrcoex_cfg_test(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_nrcoex_cfg_test::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_NRCOEX_TEST, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_reg_write::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


oal_uint32 wal_hipriv_set_txpower(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_int32 l_pwer;
    oal_uint32 ul_off_set;
    oal_int8 ac_val[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_idx = 0;
    wal_msg_stru *pst_rsp_msg = OAL_PTR_NULL;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_val, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_txpower::wal_get_cmd_one_arg_etc vap name return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    if (ac_val[0] == '-') {
        l_idx++;
    }

    /* ��������Ϸ��Լ�� */
    while ('\0' != ac_val[l_idx]) {
        if (isdigit(ac_val[l_idx])) {
            l_idx++;
            continue;
        } else {
            l_idx++;
            OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_txpower::input illegal!}\r\n");
            return OAL_ERR_CODE_INVALID_CONFIG;
        }
    }

    l_pwer = oal_atoi(ac_val);

    if (l_pwer >= WLAN_MAX_TXPOWER * 10 || l_pwer <= WLAN_MIN_TXPOWER * 10) { /* �����쳣: �������ƴ���1W */
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_txpower::invalid argument!}");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_txpower::val[%d]!}", l_pwer);
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TX_POWER, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_pwer;
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg, OAL_TRUE, &pst_rsp_msg);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_txpower::return err code %d!}", l_ret);
        return (oal_uint32)l_ret;
    }
    /* ��ȡ���صĴ����� */
    ul_ret = wal_check_and_release_msg_resp_etc(pst_rsp_msg);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_DFR, "{wal_hipriv_set_bw fail, err code[%u]!}\r\n", ul_ret);
    }

    return ul_ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 44))

OAL_STATIC oal_uint32 wal_ioctl_set_beacon_interval(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_beacon_interval;
    oal_uint32 ul_off_set;
    oal_int8 ac_beacon_interval[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;

    /* �豸��up״̬���������ã�������down */
    if (0 != (OAL_IFF_RUNNING & OAL_NETDEVICE_FLAGS(pst_net_dev))) {
        OAM_ERROR_LOG1(0, OAM_SF_CFG, "{wal_ioctl_set_beacon_interval::device is busy, please down it firs %d!}\r\n", OAL_NETDEVICE_FLAGS(pst_net_dev));
        return OAL_ERR_CODE_CONFIG_BUSY;
    }

    /* pc_paramָ���´�����net_device��name, ����ȡ����ŵ�ac_name�� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_beacon_interval, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_ioctl_set_beacon_interval::wal_get_cmd_one_arg_etc vap name return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    l_beacon_interval = oal_atoi(ac_beacon_interval);
    OAM_INFO_LOG1(0, OAM_SF_ANY, "{wal_ioctl_set_beacon_interval::l_beacon_interval = %d!}\r\n", l_beacon_interval);

    /***************************************************************************
        ���¼���wal�㴦��
    ***************************************************************************/
    /* ��д��Ϣ */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_BEACON_INTERVAL, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_beacon_interval;

    /* ������Ϣ */
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_ioctl_set_beacon_interval::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_start_vap(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    OAM_ERROR_LOG0(0, OAM_SF_CFG, "DEBUG:: priv start enter.");
    wal_netdev_open_etc(pst_net_dev, OAL_FALSE);
    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_amsdu_start(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_amsdu_start_param_stru *pst_amsdu_start_param = OAL_PTR_NULL;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    /* ��д��Ϣ */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_AMSDU_START, OAL_SIZEOF(mac_cfg_amsdu_start_param_stru));

    /* ��������������������� */
    pst_amsdu_start_param = (mac_cfg_amsdu_start_param_stru *)(st_write_msg.auc_value);

    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_amsdu_start::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), pst_amsdu_start_param->auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_amsdu_start::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    pst_amsdu_start_param->uc_amsdu_max_num = (oal_uint8)oal_atoi(ac_name);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_amsdu_start::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    pst_amsdu_start_param->us_amsdu_max_size = (oal_uint16)oal_atoi(ac_name);

    /* ������Ϣ */
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_amsdu_start_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_amsdu_start::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_list_ap(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LIST_AP, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_list_ap::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_list_sta(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LIST_STA, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_list_sta::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_list_channel(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LIST_CHAN, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_list_channel::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_regdomain_pwr_priv(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_pwr;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "wal_hipriv_set_regdomain_pwr, get arg return err %d", ul_ret);
        return ul_ret;
    }

    ul_pwr = (oal_uint32)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_REGDOMAIN_PWR, OAL_SIZEOF(oal_int32));

    ((mac_cfg_regdomain_max_pwr_stru *)st_write_msg.auc_value)->uc_pwr = (oal_uint8)ul_pwr;
    ((mac_cfg_regdomain_max_pwr_stru *)st_write_msg.auc_value)->en_exceed_reg = OAL_TRUE;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_regdomain_pwr::wal_send_cfg_event_etc fail.return err code %d}", l_ret);
    }

    return (oal_uint32)l_ret;
}


OAL_STATIC oal_uint32 wal_hipriv_start_scan(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
#ifdef _PRE_WLAN_FEATURE_P2P
    oal_uint8 uc_is_p2p0_scan;
#endif /* _PRE_WLAN_FEATURE_P2P */

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_START_SCAN, OAL_SIZEOF(oal_int32));

#ifdef _PRE_WLAN_FEATURE_P2P
    uc_is_p2p0_scan = (oal_memcmp(pst_net_dev->name, "p2p0", OAL_STRLEN("p2p0")) == 0) ? 1 : 0;
    // uc_is_p2p0_scan = (pst_net_dev->ieee80211_ptr->iftype == NL80211_IFTYPE_P2P_DEVICE)?1:0;
    st_write_msg.auc_value[0] = uc_is_p2p0_scan;
#endif /* _PRE_WLAN_FEATURE_P2P */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_start_scan::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_start_join(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_START_JOIN, OAL_SIZEOF(oal_int32));

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_start_join::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Ҫ����AP�ı�Ÿ��Ƶ��¼�msg�У�AP��������ֵ�ASSCI�룬������4���ֽ� */
    if (EOK != memcpy_s((oal_int8 *)st_write_msg.auc_value, sizeof(st_write_msg.auc_value), (oal_int8 *)ac_name, OAL_SIZEOF(oal_int32))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_start_join::memcpy fail}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_start_join::return err codereturn err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_start_deauth(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_START_DEAUTH, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_start_deauth::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dump_timer(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DUMP_TIEMR, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_timer::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_kick_user(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_kick_user_param_stru *pst_kick_user_param = OAL_PTR_NULL;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };

    /* ȥ����1���û������� hipriv "vap0 kick_user xx:xx:xx:xx:xx:xx" */
    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_kick_user::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_KICK_USER, OAL_SIZEOF(mac_cfg_kick_user_param_stru));

    /* ��������������� */
    pst_kick_user_param = (mac_cfg_kick_user_param_stru *)(st_write_msg.auc_value);
    oal_set_mac_addr(pst_kick_user_param->auc_mac_addr, auc_mac_addr);

    /* ��дȥ����reason code */
    pst_kick_user_param->us_reason_code = MAC_UNSPEC_REASON;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_kick_user_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_kick_user::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_pause_tid(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_pause_tid_param_stru *pst_pause_tid_param = OAL_PTR_NULL;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
    oal_uint8 uc_tid;
    /* ȥ����1���û������� hipriv "vap0 kick_user xx xx xx xx xx xx" */
    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pause_tid::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pause_tid::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_tid = (oal_uint8)oal_atoi(ac_name);

    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PAUSE_TID, OAL_SIZEOF(mac_cfg_pause_tid_param_stru));

    /* ��������������� */
    pst_pause_tid_param = (mac_cfg_pause_tid_param_stru *)(st_write_msg.auc_value);
    oal_set_mac_addr(pst_pause_tid_param->auc_mac_addr, auc_mac_addr);
    pst_pause_tid_param->uc_tid = uc_tid;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pause_tid::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_pause_tid_param->uc_is_paused = (oal_uint8)oal_atoi(ac_name);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_pause_tid_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pause_tid::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_OFFLOAD_FLOWCTL

OAL_STATIC oal_uint32 wal_hipriv_get_hipkt_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    // sh hipriv.sh "wlan0 get_hipkt_stat"
    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_GET_HIPKT_STAT, OAL_SIZEOF(oal_uint8));

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_get_hipkt_stat:: return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_flowctl_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set = 0;
    oal_int32 l_ret;
    oal_int8 ac_param[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    mac_cfg_flowctl_param_stru st_flowctl_param;
    mac_cfg_flowctl_param_stru *pst_param;

    // sh hipriv.sh "Hisilicon0 set_flowctl_param 0/1/2/3 20 20 40"
    // 0/1/2/3 �ֱ����be,bk,vi,vo
    /* ��ȡ�������Ͳ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_flowctl_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_flowctl_param.uc_queue_type = (oal_uint8)oal_atoi(ac_param);

    /* ���ö��ж�Ӧ��ÿ�ε��ȱ��ĸ��� */
    pc_param += ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_flowctl_param::wal_get_cmd_one_arg_etc burst_limit return err_code %d!}\r\n", ul_ret);
        return (oal_uint32)ul_ret;
    }
    st_flowctl_param.us_burst_limit = (oal_uint16)oal_atoi(ac_param);

    /* ���ö��ж�Ӧ������low_waterline */
    pc_param += ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_flowctl_param::wal_get_cmd_one_arg_etc low_waterline return err_code %d!}\r\n", ul_ret);
        return (oal_uint32)ul_ret;
    }
    st_flowctl_param.us_low_waterline = (oal_uint16)oal_atoi(ac_param);

    /* ���ö��ж�Ӧ������high_waterline */
    pc_param += ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_param, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_flowctl_param::wal_get_cmd_one_arg_etc high_waterline return err_code %d!}\r\n", ul_ret);
        return (oal_uint32)ul_ret;
    }
    st_flowctl_param.us_high_waterline = (oal_uint16)oal_atoi(ac_param);

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_FLOWCTL_PARAM, OAL_SIZEOF(mac_cfg_flowctl_param_stru));
    pst_param = (mac_cfg_flowctl_param_stru *)(st_write_msg.auc_value);

    pst_param->uc_queue_type = st_flowctl_param.uc_queue_type;
    pst_param->us_burst_limit = st_flowctl_param.us_burst_limit;
    pst_param->us_low_waterline = st_flowctl_param.us_low_waterline;
    pst_param->us_high_waterline = st_flowctl_param.us_high_waterline;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_flowctl_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_flowctl_param:: return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_get_flowctl_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    // sh hipriv.sh "Hisilicon0 get_flowctl_stat"
    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_GET_FLOWCTL_STAT, OAL_SIZEOF(oal_uint8));

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_get_flowctl_stat:: return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_event_queue_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return frw_event_queue_info_etc();
}


OAL_STATIC oal_uint32 wal_hipriv_set_user_vip(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_user_vip_param_stru *pst_user_vip_param = OAL_PTR_NULL;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0 };
    oal_uint8 uc_vip_flag;

    /*
     * �����û�Ϊvip�û�: 0 ������VIP�û���1����VIP�û�
     * sh hipriv.sh "vap0 set_user_vip xx xx xx xx xx xx 0|1"
     */
    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_user_vip::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_user_vip::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_vip_flag = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_USER_VIP, OAL_SIZEOF(mac_cfg_pause_tid_param_stru));

    /* ��������������� */
    pst_user_vip_param = (mac_cfg_user_vip_param_stru *)(st_write_msg.auc_value);
    oal_set_mac_addr(pst_user_vip_param->auc_mac_addr, auc_mac_addr);
    pst_user_vip_param->uc_vip_flag = uc_vip_flag;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_user_vip_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_user_vip::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_vap_host(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_host_flag;

    /* ����vap��host flag: 0 ����guest vap, 1����host vap
       sh hipriv.sh "vap0 set_host 0|1" */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_vap_host::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_host_flag = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_VAP_HOST, OAL_SIZEOF(oal_uint8));

    /* ��������������� */
    *((oal_uint8 *)(st_write_msg.auc_value)) = uc_host_flag;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_vap_host::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_send_bar(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_pause_tid_param_stru *pst_pause_tid_param = OAL_PTR_NULL;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
    oal_uint8 uc_tid;

    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_bar::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_bar::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_tid = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SEND_BAR, OAL_SIZEOF(mac_cfg_pause_tid_param_stru));

    /* ��������������� */
    pst_pause_tid_param = (mac_cfg_pause_tid_param_stru *)(st_write_msg.auc_value);
    oal_set_mac_addr(pst_pause_tid_param->auc_mac_addr, auc_mac_addr);
    pst_pause_tid_param->uc_tid = uc_tid;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_pause_tid_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_bar::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_amsdu_tx_on(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_ampdu_tx_on_param_stru *pst_aggr_tx_on_param = OAL_PTR_NULL;
    oal_uint8 uc_aggr_tx_on;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_amsdu_tx_on::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_aggr_tx_on = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_AMSDU_TX_ON, OAL_SIZEOF(mac_cfg_ampdu_tx_on_param_stru));

    /* ��������������� */
    pst_aggr_tx_on_param = (mac_cfg_ampdu_tx_on_param_stru *)(st_write_msg.auc_value);
    pst_aggr_tx_on_param->uc_aggr_tx_on = uc_aggr_tx_on;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_ampdu_tx_on_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_amsdu_tx_on::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_frag_threshold(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    mac_cfg_frag_threshold_stru *pst_threshold = OAL_PTR_NULL;
    oal_uint32 ul_threshold;

    /* ��ȡ��Ƭ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_frag_threshold::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    ul_threshold = (oal_uint16)oal_atoi(ac_name);
    pc_param += ul_off_set;

    if ((ul_threshold < WLAN_FRAG_THRESHOLD_MIN) ||
        (ul_threshold > WLAN_FRAG_THRESHOLD_MAX)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_frag_threshold::ul_threshold value error [%d]!}\r\n", ul_threshold);
        return OAL_FAIL;
    }

    pst_threshold = (mac_cfg_frag_threshold_stru *)(st_write_msg.auc_value);
    pst_threshold->ul_frag_threshold = ul_threshold;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(mac_cfg_frag_threshold_stru);
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_FRAG_THRESHOLD_REG, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_frag_threshold::return err code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_wmm_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_open_wmm;

    /* ��ȡ�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wmm_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_open_wmm = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(oal_uint8);
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_open_wmm;
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_WMM_SWITCH, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wmm_switch::return err code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_hide_ssid(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_hide_ssid;

    /* ��ȡ�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_hide_ssid::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_hide_ssid = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(oal_uint8);
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_hide_ssid;
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_HIDE_SSID, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_hide_ssid::return err code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_ampdu_tx_on(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_ampdu_tx_on_param_stru *pst_aggr_tx_on_param = OAL_PTR_NULL;
    oal_uint8 uc_aggr_tx_on;
    oal_uint8 uc_snd_type = 0;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ampdu_tx_on::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    uc_aggr_tx_on = (oal_uint8)oal_atoi(ac_name);

    /* ֻ��Ӳ���ۺ���Ҫ���õڶ����� */
    if ((uc_aggr_tx_on & BIT3) || (uc_aggr_tx_on & BIT2) || (uc_aggr_tx_on & BIT1)) {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ampdu_tx_on::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        uc_snd_type = (oal_uint8)oal_atoi(ac_name);
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_AMPDU_TX_ON, OAL_SIZEOF(mac_cfg_ampdu_tx_on_param_stru));

    /* ��������������� */
    pst_aggr_tx_on_param = (mac_cfg_ampdu_tx_on_param_stru *)(st_write_msg.auc_value);
    pst_aggr_tx_on_param->uc_aggr_tx_on = uc_aggr_tx_on;
    if (uc_snd_type > 1) {
        pst_aggr_tx_on_param->uc_snd_type = 1;
        pst_aggr_tx_on_param->en_aggr_switch_mode = AMPDU_SWITCH_BY_BA_LUT;
    } else {
        pst_aggr_tx_on_param->uc_snd_type = uc_snd_type;
        pst_aggr_tx_on_param->en_aggr_switch_mode = AMPDU_SWITCH_BY_DEL_BA;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_ampdu_tx_on_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ampdu_tx_on::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_HIEX

OAL_STATIC oal_uint32 wal_hipriv_set_user_hiex_enable(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru             st_write_msg;
    oal_uint32                     ul_off_set;
    oal_int8                       ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32                     ul_ret;
    oal_int32                      l_ret;
    mac_cfg_user_hiex_param_stru  *pst_user_hiex_param = OAL_PTR_NULL;
    oal_uint8                      auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0 };
    oal_uint32                     ul_hiex_flag;

    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY,
                         "{wal_hipriv_set_user_hiex_enable::wal_get_cmd_one_arg return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY,
                         "{wal_hipriv_set_user_hiex_enable::wal_get_cmd_one_arg return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    ul_hiex_flag = oal_strtol(ac_name, NULL, NUM_16_BITS);

    /* ���¼���wal�㴦�� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_USER_HIEX_ENABLE, OAL_SIZEOF(mac_cfg_user_hiex_param_stru));

    /* ��������������� */
    pst_user_hiex_param = (mac_cfg_user_hiex_param_stru *)(st_write_msg.auc_value);
    oal_set_mac_addr(pst_user_hiex_param->auc_mac_addr, auc_mac_addr);
    *(oal_uint32 *)&pst_user_hiex_param->st_hiex_cap = ul_hiex_flag;

    l_ret = wal_send_cfg_event_etc(pst_net_dev, WAL_MSG_TYPE_WRITE,
        WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_user_hiex_param_stru), (oal_uint8 *)&st_write_msg,
        OAL_FALSE, OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_user_hiex_enable::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATUER_PCIE_TEST

OAL_STATIC oal_uint32 wal_hipriv_pcie_test(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_pcie_test_stru *pst_pcie_test_param;
    oal_int32 ul_num = 3;
    oal_uint32 uc_tmp[3];

    for (l_ret = 0; l_ret < ul_num; l_ret++) {
        pc_param = pc_param + ul_off_set;
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pcie_test::wal_hipriv_pcie_test return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        uc_tmp[l_ret] = (oal_uint32)oal_atoi(ac_name);
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PCIE_TEST, OAL_SIZEOF(mac_cfg_pcie_test_stru));

    /* ��������������� */
    pst_pcie_test_param = (mac_cfg_pcie_test_stru *)(st_write_msg.auc_value);
    pst_pcie_test_param->us_burst = (oal_uint16)uc_tmp[0];
    if (uc_tmp[1] > 1 || uc_tmp[2] > 1) {
        uc_tmp[1] = 0;
        uc_tmp[2] = 0;
    }
    pst_pcie_test_param->uc_wdata = uc_tmp[1];
    pst_pcie_test_param->uc_rdata = uc_tmp[2];
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_pcie_test_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pcie_test::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_reset_device(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_reset_device::memcpy fail}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_RESET_HW, us_len);

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_reset_device::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_reset_operate(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    if (OAL_UNLIKELY(WAL_MSG_WRITE_MAX_LEN <= OAL_STRLEN(pc_param))) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_reset_operate:: pc_param overlength is %d}\n", OAL_STRLEN(pc_param));
        return OAL_FAIL;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_reset_operate::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_RESET_HW_OPERATE, us_len);

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_reset_operate::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_UAPSD

OAL_STATIC oal_uint32 wal_hipriv_uapsd_debug(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    if (OAL_UNLIKELY(WAL_MSG_WRITE_MAX_LEN <= OAL_STRLEN(pc_param))) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_uapsd_debug:: pc_param overlength is %d}\n", OAL_STRLEN(pc_param));
        return OAL_FAIL;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_uapsd_debug::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_UAPSD_DEBUG, us_len);

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_uapsd_debug::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_DFT_STAT

OAL_STATIC oal_uint32 wal_hipriv_set_phy_stat_en(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oam_stats_phy_node_idx_stru st_phy_stat_node_idx;
    oal_uint8 uc_loop;

    /* sh hipriv.sh "Hisilicon0 phy_stat_en idx1 idx2 idx3 idx4" */
    for (uc_loop = 0; uc_loop < OAM_PHY_STAT_NODE_ENABLED_MAX_NUM; uc_loop++) {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_phy_stat_en::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        st_phy_stat_node_idx.auc_node_idx[uc_loop] = (oal_uint8)oal_atoi(ac_name);

        /* �������Ƿ�Ϸ���������Χ��1~16 */
        if (st_phy_stat_node_idx.auc_node_idx[uc_loop] < OAM_PHY_STAT_ITEM_MIN_IDX ||
            st_phy_stat_node_idx.auc_node_idx[uc_loop] > OAM_PHY_STAT_ITEM_MAX_IDX) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY,
                             "{wal_hipriv_set_phy_stat_en::stat_item_idx invalid! should between 1 and 16!}.",
                             st_phy_stat_node_idx.auc_node_idx[uc_loop]);
            return OAL_ERR_CODE_INVALID_CONFIG;
        }

        pc_param += ul_off_set;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PHY_STAT_EN, OAL_SIZEOF(st_phy_stat_node_idx));

    /* ��д��Ϣ�壬���� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        &st_phy_stat_node_idx, OAL_SIZEOF(st_phy_stat_node_idx))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_phy_stat_en::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_phy_stat_node_idx),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_phy_stat_en::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dbb_env_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_param;

    /* sh hipriv.sh "Hisilicon0 dbb_env_param 0|1" */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dbb_env_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_param = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DBB_ENV_PARAM, OAL_SIZEOF(uc_param));

    /* ��д��Ϣ�壬���� */
    st_write_msg.auc_value[0] = uc_param;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(uc_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dbb_env_param::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_usr_queue_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_usr_queue_param_stru st_usr_queue_param;

    /* sh hipriv.sh "vap_name usr_queue_stat XX:XX:XX:XX:XX:XX 0|1" */
    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));
    memset_s((oal_uint8 *)&st_usr_queue_param, OAL_SIZEOF(st_usr_queue_param), 0, OAL_SIZEOF(st_usr_queue_param));

    /* ��ȡ�û�mac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, st_usr_queue_param.auc_user_macaddr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_usr_queue_stat::wal_hipriv_get_mac_addr_etc return [%d].}", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_usr_queue_stat::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_usr_queue_param.uc_param = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_USR_QUEUE_STAT, OAL_SIZEOF(st_usr_queue_param));

    /* ��д��Ϣ�壬���� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        &st_usr_queue_param, OAL_SIZEOF(st_usr_queue_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_usr_queue_stat::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_usr_queue_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_usr_queue_stat::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_report_vap_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_param;

    /* sh hipriv.sh "vap_name vap _stat  0|1" */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_vap_stat::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_param = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_VAP_STAT, OAL_SIZEOF(uc_param));

    /* ��д��Ϣ�壬���� */
    st_write_msg.auc_value[0] = uc_param;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(uc_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_vap_stat::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_report_all_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /* sh hipriv.sh "Hisilicon0 reprt_all_stat type(phy/machw/mgmt/irq/all)  0|1" */
    /* ��ȡrepot���� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_report_all_stat::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';
    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);
    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ALL_STAT, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_all_stat::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_set_ampdu_aggr_num(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    mac_cfg_aggr_num_stru st_aggr_num_ctl = { 0 };
    oal_uint32 ul_ret;
    oal_int32 l_ret;

    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ampdu_aggr_num::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    st_aggr_num_ctl.uc_aggr_num_switch = (oal_uint8)oal_atoi(ac_name);
    if (st_aggr_num_ctl.uc_aggr_num_switch == 0) {
        /* ��ָ���ۺϸ���ʱ���ۺϸ����ָ�Ϊ0 */
        st_aggr_num_ctl.uc_aggr_num = 0;
    } else {
        /* ��ȡ�ۺϸ��� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ampdu_aggr_num::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        st_aggr_num_ctl.uc_aggr_num = (oal_uint8)oal_atoi(ac_name);

        /* �����ۺ���������ж� */
        if (st_aggr_num_ctl.uc_aggr_num > WLAN_AMPDU_TX_MAX_NUM) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ampdu_aggr_num::exceed max aggr num [%d]!}\r\n", st_aggr_num_ctl.uc_aggr_num);
            return ul_ret;
        }
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_AGGR_NUM, OAL_SIZEOF(st_aggr_num_ctl));

    /* ��д��Ϣ�壬���� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        &st_aggr_num_ctl, OAL_SIZEOF(st_aggr_num_ctl))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_ampdu_aggr_num::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_aggr_num_ctl),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ampdu_aggr_num::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_stbc_cap(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_value;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_cfg_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_stbc_cap::pst_cfg_net_dev or pc_param null ptr error %x, %x!}\r\n", (uintptr_t)pst_cfg_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* STBC���ÿ��ص�����: hipriv "vap0 set_stbc_cap 0 | 1"
            �˴���������"1"��"0"����ac_name
    */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_stbc_cap::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�������TDLS���ÿ��� */
    if (0 == (oal_strcmp("0", ac_name))) {
        ul_value = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        ul_value = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_stbc_cap::the set stbc command is error %x!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_STBC_CAP, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_value;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_stbc_cap::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_ldpc_cap(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_value;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_cfg_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_ldpc_cap::pst_cfg_net_dev or pc_param null ptr error %x, %x!}\r\n", (uintptr_t)pst_cfg_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* LDPC���ÿ��ص�����: hipriv "vap0 set_ldpc_cap 0 | 1"
            �˴���������"1"��"0"����ac_name
    */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ldpc_cap::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�������TDLS���ÿ��� */
    if (0 == (oal_strcmp("0", ac_name))) {
        ul_value = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        ul_value = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ldpc_cap::the set ldpc command is error %x!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_LDPC_CAP, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_value;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_ldpc_cap::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_txbf_cap(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_value;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_txbf_cap::v or pc_param null ptr error %p, %p!}\r\n", (uintptr_t)pst_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /************************************************************
      TXBF���ÿ��ص�����: sh hipriv "vap0 set_txbf_cap 0 | 1 | 2 |3"
             bit0��ʾRX(bfee)����     bit1��ʾTX(bfer)����
    *************************************************************/

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if ((ul_ret != OAL_SUCC) || (0 == (oal_strcmp("help", ac_name)))) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_txbf_cap::set txbf cap [bit0 indicates RX(bfee), bit1 TX(bfer)], ret[%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    ul_value = (oal_uint32)oal_atoi(ac_name);

    if (ul_value > 3) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_txbf_cap::set txbf cap [0 ~ 3]!}\r\n", ul_value);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TXBF_SWITCH, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_value;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_txbf_cap::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_dump_rx_dscr_etc(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_value;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_rx_dscr_etc::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    ul_value = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DUMP_RX_DSCR, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_value;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_rx_dscr_etc::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dump_tx_dscr(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_value;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_tx_dscr::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    ul_value = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DUMP_TX_DSCR, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_value;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_tx_dscr::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dump_memory(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_addr[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 ac_len[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_len;
    oal_uint32 ul_addr;
    mac_cfg_dump_memory_stru *pst_cfg = OAL_PTR_NULL;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_addr, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_memory::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pc_param += ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_len, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_memory::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��ַ�ַ���ת��16λ��ַ */
    ul_addr = (oal_uint32)oal_strtol(ac_addr, 0, 16);
    ul_len = (oal_uint32)oal_atoi(ac_len);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DUMP_MEMORY, OAL_SIZEOF(mac_cfg_dump_memory_stru));

    /* ��������������� */

    pst_cfg = (mac_cfg_dump_memory_stru *)(st_write_msg.auc_value);

    pst_cfg->ul_addr = ul_addr;
    pst_cfg->ul_len = ul_len;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_dump_memory_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_memory::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_show_tx_dscr_addr(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dump_ba_bitmap(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_mpdu_ampdu_tx_param_stru *pst_aggr_tx_on_param = OAL_PTR_NULL;
    oal_uint8 uc_tid;
    oal_uint8 auc_ra_addr[WLAN_MAC_ADDR_LEN] = { 0 };

    /* ��ȡtid */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_ba_bitmap::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_tid = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡMAC��ַ�ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_ba_bitmap::get mac err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    /* ��ַ�ַ���ת��ַ���� */
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_ra_addr, WLAN_MAC_ADDR_LEN);
    pc_param += ul_off_set;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DUMP_BA_BITMAP, OAL_SIZEOF(mac_cfg_mpdu_ampdu_tx_param_stru));

    /* ��������������� */
    pst_aggr_tx_on_param = (mac_cfg_mpdu_ampdu_tx_param_stru *)(st_write_msg.auc_value);
    pst_aggr_tx_on_param->uc_tid = uc_tid;
    oal_set_mac_addr(pst_aggr_tx_on_param->auc_ra_mac, auc_ra_addr);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_mpdu_ampdu_tx_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_ba_bitmap::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_alg(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;  // FIXME : st_write_msg can only carry bytes less than 48
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 *pc_tmp = (oal_int8 *)pc_param;
    oal_uint16 us_config_len;
    oal_uint16 us_param_len;

    mac_ioctl_alg_config_stru st_alg_config;

    st_alg_config.uc_argc = 0;
    while (OAL_SUCC == wal_get_cmd_one_arg_etc(pc_tmp, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set)) {
        st_alg_config.auc_argv_offset[st_alg_config.uc_argc] = (oal_uint8)((oal_uint8)(pc_tmp - pc_param) + (oal_uint8)ul_off_set - (oal_uint8)OAL_STRLEN(ac_arg));
        pc_tmp += ul_off_set;
        st_alg_config.uc_argc++;

        if (st_alg_config.uc_argc > DMAC_ALG_CONFIG_MAX_ARG) {
            OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_alg::wal_hipriv_alg error, argc too big [%d]!}\r\n", st_alg_config.uc_argc);
            return OAL_FAIL;
        }
    }

    if (st_alg_config.uc_argc == 0) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_alg::argc=0!}\r\n");
        return OAL_FAIL;
    }

    us_param_len = (oal_uint16)OAL_STRLEN(pc_param);
    if (us_param_len > WAL_MSG_WRITE_MAX_LEN - 1 - OAL_SIZEOF(mac_ioctl_alg_config_stru)) {
        return OAL_FAIL;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    us_config_len = OAL_SIZEOF(mac_ioctl_alg_config_stru) + us_param_len + 1;
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ALG, us_config_len);
    l_ret = memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), &st_alg_config, OAL_SIZEOF(mac_ioctl_alg_config_stru));
    l_ret += memcpy_s(st_write_msg.auc_value + OAL_SIZEOF(mac_ioctl_alg_config_stru),
                      sizeof(st_write_msg.auc_value) - OAL_SIZEOF(mac_ioctl_alg_config_stru), pc_param, us_param_len + 1);
    if (l_ret != EOK) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_alg::memcpy fail! [%d]}", l_ret);
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_config_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_alg::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_RX_AGGR_EXTEND

OAL_STATIC oal_uint32 wal_hipriv_waveapp_32plus_user_enable(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 uc_enable_flag;
    oal_uint8 *puc_value;

    /* hipriv "Hisilicon0 waveapp_enable 1" */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_waveapp_32plus_user_enable::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_enable_flag = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_WAVEAPP_32PLUS_USER_ENABLE, OAL_SIZEOF(oal_uint8));

    puc_value = (oal_uint8 *)(st_write_msg.auc_value);
    *puc_value = uc_enable_flag;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_waveapp_32plus_user_enable::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_show_stat_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
#ifdef _PRE_PRODUCT_ID_HI110X_HOST
    oam_stats_report_info_to_sdt_etc(OAM_OTA_TYPE_DEV_STAT_INFO);
    oam_stats_report_info_to_sdt_etc(OAM_OTA_TYPE_VAP_STAT_INFO);
#endif
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_show_vap_pkt_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                 ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_VAP_PKT_STAT, OAL_SIZEOF(oal_uint32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_vap_pkt_stat::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_intf_det_log(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    mac_ioctl_alg_intfdet_log_param_stru *pst_alg_intfdet_log_param = OAL_PTR_NULL;
    wal_ioctl_alg_cfg_stru st_alg_cfg;
    oal_uint8 uc_map_index = 0;
    oal_int32 l_ret;

    pst_alg_intfdet_log_param = (mac_ioctl_alg_intfdet_log_param_stru *)(st_write_msg.auc_value);

    /* ��ȡ���ò������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_intf_det_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;

    /* Ѱ��ƥ������� */
    st_alg_cfg = g_ast_alg_cfg_map_etc[0];
    while (st_alg_cfg.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_alg_cfg.pc_name, ac_name)) {
            break;
        }
        st_alg_cfg = g_ast_alg_cfg_map_etc[++uc_map_index];
    }

    /* û���ҵ���Ӧ������򱨴� */
    if (st_alg_cfg.pc_name == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_intf_det_log::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }

    /* ��¼�����Ӧ��ö��ֵ */
    pst_alg_intfdet_log_param->en_alg_cfg = g_ast_alg_cfg_map_etc[uc_map_index].en_alg_cfg;

    /* ���ֻ�ȡ�ض�֡���ʺ�ͳ����־�����:��ȡ����ֻ���ȡ֡���� */
    if (pst_alg_intfdet_log_param->en_alg_cfg == MAC_ALG_CFG_INTF_DET_STAT_LOG_START) {
        /* ��ȡ���ò������� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_intf_det_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        /* ��¼���� */
        pst_alg_intfdet_log_param->us_value = (oal_uint16)oal_atoi(ac_name);
        // en_stop_flag = OAL_TRUE;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ALG_PARAM, OAL_SIZEOF(mac_ioctl_alg_intfdet_log_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_ioctl_alg_intfdet_log_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_clear_stat_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
#if defined(_PRE_PRODUCT_ID_HI110X_HOST)
    oam_stats_clear_stat_info_etc();
#endif
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_user_stat_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
#if defined(_PRE_PRODUCT_ID_HI110X_HOST)

    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;

    /* sh hipriv.sh "Hisilicon0 usr_stat_info usr_id" */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_user_stat_info::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    l_tmp = oal_atoi(ac_name);
    if ((l_tmp < 0) || (l_tmp >= WLAN_ACTIVE_USER_MAX_NUM)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_user_stat_info::user id invalid [%d]!}\r\n", l_tmp);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    oam_stats_report_usr_info_etc((oal_uint16)l_tmp);
#endif
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_timer_start(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_timer_switch;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_timer_start::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);

        return ul_ret;
    }

    uc_timer_switch = (oal_uint8)oal_atoi(ac_name);
    if (uc_timer_switch >= 2) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_timer_start::invalid choicee [%d]!}\r\n", uc_timer_switch);
        return OAL_ERR_CODE_ARRAY_OVERFLOW;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TIMER_START, OAL_SIZEOF(oal_uint8));

    /* ��������������� */
    *((oal_uint8 *)(st_write_msg.auc_value)) = uc_timer_switch;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_timer_start::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_show_profiling(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
#ifdef _PRE_PROFILING_MODE

    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_show_profiling_param_stru *pst_show_profiling_param = { 0 };
    oal_int8 uc_show_profiling_type;
    oal_int8 uc_show_level;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_profiling::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_show_profiling_type = (oal_uint8)oal_atoi(ac_name);

    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_profiling::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_show_level = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SHOW_PROFILING, OAL_SIZEOF(mac_cfg_show_profiling_param_stru));

    /* ��������������� */
    pst_show_profiling_param = (mac_cfg_show_profiling_param_stru *)(st_write_msg.auc_value);
    pst_show_profiling_param->uc_show_profiling_type = uc_show_profiling_type;
    pst_show_profiling_param->uc_show_level = uc_show_level;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_show_profiling_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_profiling::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

#endif

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_profiling_packet_add(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
#ifdef _PRE_PROFILING_MODE
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint8 ul_value;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;

    l_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_profiling::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", l_ret);
        return l_ret;
    }
    ul_value = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                           ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PROFILING_PACKET_ADD, OAL_SIZEOF(oal_int8));

    /* ��������������� */
    *((oal_uint8 *)(st_write_msg.auc_value)) = ul_value;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_profiling_packet_add::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

#endif

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_DFT_STAT

OAL_STATIC oal_uint32 wal_hipriv_clear_vap_stat_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint8 uc_vap_id;

    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_clear_vap_stat_info::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;
    oam_stats_clear_vap_stat_info_etc(uc_vap_id);

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_DFR

OAL_STATIC oal_uint32 wal_hipriv_test_dfr_start(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_cfg_rst;
    oal_wireless_dev_stru *pst_wdev;
    mac_wiphy_priv_stru *pst_wiphy_priv = OAL_PTR_NULL;
    mac_device_stru *pst_mac_device = OAL_PTR_NULL;

    pst_wdev = OAL_NETDEVICE_WDEV(pst_net_dev);
    if (pst_wdev == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_test_dfr_start::pst_wdev is null!}\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_wiphy_priv = (mac_wiphy_priv_stru *)oal_wiphy_priv(pst_wdev->wiphy);
    if (pst_wiphy_priv == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_test_dfr_start::pst_wiphy_priv is null!}\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }
    pst_mac_device = pst_wiphy_priv->pst_mac_device;
    if (pst_mac_device == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_test_dfr_start::pst_mac_device is null!}\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }
    if ((!g_st_dfr_info_etc.bit_device_reset_enable) || g_st_dfr_info_etc.bit_device_reset_process_flag) {
        OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_test_dfr_start::now DFR disabled or in DFR process, enable=%d, reset_flag=%d}",
                         g_st_dfr_info_etc.bit_device_reset_enable, g_st_dfr_info_etc.bit_device_reset_process_flag);
        return OAL_ERR_CODE_RESET_INPROGRESS;
    }

    g_st_dfr_info_etc.bit_device_reset_enable = OAL_TRUE;
    g_st_dfr_info_etc.bit_device_reset_process_flag = OAL_FALSE;
    g_st_dfr_info_etc.ul_netdev_num = 0;
    memset_s((oal_uint8 *)(g_st_dfr_info_etc.past_netdev),
             OAL_SIZEOF(g_st_dfr_info_etc.past_netdev[0]) * WLAN_VAP_SUPPORT_MAX_NUM_LIMIT, 0,
             OAL_SIZEOF(g_st_dfr_info_etc.past_netdev[0]) * WLAN_VAP_SUPPORT_MAX_NUM_LIMIT);

    ul_cfg_rst = wal_dfr_excp_rx_etc(pst_mac_device->uc_device_id, 0);
    if (OAL_UNLIKELY(ul_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_dfr_start::wal_send_cfg_event_etc return err_code [%d]!}\r\n", ul_cfg_rst);
        return ul_cfg_rst;
    }

    return OAL_SUCC;
}

#endif  // _PRE_WLAN_FEATURE_DFR


OAL_STATIC oal_uint32 wal_hipriv_ar_log(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    mac_ioctl_alg_ar_log_param_stru *pst_alg_ar_log_param = OAL_PTR_NULL;
    wal_ioctl_alg_cfg_stru st_alg_cfg;
    oal_uint8 uc_map_index = 0;
    oal_int32 l_ret;
    oal_bool_enum_uint8 en_stop_flag = OAL_FALSE;

    pst_alg_ar_log_param = (mac_ioctl_alg_ar_log_param_stru *)(st_write_msg.auc_value);

    /* ��ȡ���ò������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ar_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;

    /* Ѱ��ƥ������� */
    st_alg_cfg = g_ast_alg_cfg_map_etc[0];
    while (st_alg_cfg.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_alg_cfg.pc_name, ac_name)) {
            break;
        }
        st_alg_cfg = g_ast_alg_cfg_map_etc[++uc_map_index];
    }

    /* û���ҵ���Ӧ������򱨴� */
    if (st_alg_cfg.pc_name == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ar_log::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }

    /* ��¼�����Ӧ��ö��ֵ */
    pst_alg_ar_log_param->en_alg_cfg = g_ast_alg_cfg_map_etc[uc_map_index].en_alg_cfg;

    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, pst_alg_ar_log_param->auc_mac_addr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ar_log::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    pc_param += ul_off_set;

    while ((*pc_param == ' ') || (*pc_param == '\0')) {
        if (*pc_param == '\0') {
            en_stop_flag = OAL_TRUE;
            break;
        }
        ++pc_param;
    }

    /* ��ȡҵ������ֵ */
    if (en_stop_flag != OAL_TRUE) {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ar_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        pst_alg_ar_log_param->uc_ac_no = (oal_uint8)oal_atoi(ac_name);
        pc_param = pc_param + ul_off_set;

        en_stop_flag = OAL_FALSE;
        while ((*pc_param == ' ') || (*pc_param == '\0')) {
            if (*pc_param == '\0') {
                en_stop_flag = OAL_TRUE;
                break;
            }
            ++pc_param;
        }

        if (en_stop_flag != OAL_TRUE) {
            /* ��ȡ��������ֵ */
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ar_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            /* ��¼��������ֵ */
            pst_alg_ar_log_param->us_value = (oal_uint16)oal_atoi(ac_name);
        }
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ALG_PARAM, OAL_SIZEOF(mac_ioctl_alg_ar_log_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_ioctl_alg_ar_log_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_txbf_log(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    mac_ioctl_alg_txbf_log_param_stru *pst_alg_txbf_log_param = OAL_PTR_NULL;
    wal_ioctl_alg_cfg_stru st_alg_cfg;
    oal_uint8 uc_map_index = 0;
    oal_int32 l_ret;
    oal_bool_enum_uint8 en_stop_flag = OAL_FALSE;
    pst_alg_txbf_log_param = (mac_ioctl_alg_txbf_log_param_stru *)(st_write_msg.auc_value);
    /* ��ȡ���ò������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_txbf_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;
    /* Ѱ��ƥ������� */
    st_alg_cfg = g_ast_alg_cfg_map_etc[0];
    while (st_alg_cfg.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_alg_cfg.pc_name, ac_name)) {
            break;
        }
        st_alg_cfg = g_ast_alg_cfg_map_etc[++uc_map_index];
    }
    /* û���ҵ���Ӧ������򱨴� */
    if (st_alg_cfg.pc_name == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ar_log::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }
    /* ��¼�����Ӧ��ö��ֵ */
    pst_alg_txbf_log_param->en_alg_cfg = g_ast_alg_cfg_map_etc[uc_map_index].en_alg_cfg;
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, pst_alg_txbf_log_param->auc_mac_addr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_txbf_log::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    pc_param += ul_off_set;
    while (*pc_param == ' ') {
        ++pc_param;
        if (*pc_param == '\0') {
            en_stop_flag = OAL_TRUE;
            break;
        }
    }
    /* ��ȡ��������ֵ */
    if (en_stop_flag != OAL_TRUE) {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_txbf_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        pst_alg_txbf_log_param->uc_ac_no = (oal_uint8)oal_atoi(ac_name);
        pc_param = pc_param + ul_off_set;

        en_stop_flag = OAL_FALSE;
        while ((*pc_param == ' ') || (*pc_param == '\0')) {
            if (*pc_param == '\0') {
                en_stop_flag = OAL_TRUE;
                break;
            }
            ++pc_param;
        }

        if (en_stop_flag != OAL_TRUE) {
            /* ��ȡ��������ֵ */
            ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_txbf_log::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            /* ��¼��������ֵ */
            pst_alg_txbf_log_param->us_value = (oal_uint16)oal_atoi(ac_name);
        }
    }
    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ALG_PARAM, OAL_SIZEOF(mac_ioctl_alg_txbf_log_param_stru));
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_ioctl_alg_txbf_log_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        return (oal_uint32)l_ret;
    }
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_ar_test(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_offset;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    mac_ioctl_alg_ar_test_param_stru *pst_alg_ar_test_param = OAL_PTR_NULL;
    wal_ioctl_alg_cfg_stru st_alg_cfg;
    oal_uint8 uc_map_index = 0;
    oal_int32 l_ret;

    pst_alg_ar_test_param = (mac_ioctl_alg_ar_test_param_stru *)(st_write_msg.auc_value);

    /* ��ȡ���ò������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ar_test::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_offset;

    /* Ѱ��ƥ������� */
    st_alg_cfg = g_ast_alg_cfg_map_etc[0];
    while (st_alg_cfg.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_alg_cfg.pc_name, ac_name)) {
            break;
        }
        st_alg_cfg = g_ast_alg_cfg_map_etc[++uc_map_index];
    }

    /* û���ҵ���Ӧ������򱨴� */
    if (st_alg_cfg.pc_name == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ar_test::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }

    /* ��¼�����Ӧ��ö��ֵ */
    pst_alg_ar_test_param->en_alg_cfg = g_ast_alg_cfg_map_etc[uc_map_index].en_alg_cfg;

    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, pst_alg_ar_test_param->auc_mac_addr, &ul_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ar_test::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    pc_param += ul_offset;

    /* ��ȡ��������ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ar_test::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��¼��������ֵ */
    pst_alg_ar_test_param->us_value = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ALG_PARAM, OAL_SIZEOF(mac_ioctl_alg_ar_test_param_stru));
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_ioctl_alg_ar_test_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_EDCA_OPT_AP

OAL_STATIC oal_uint32 wal_hipriv_set_edca_opt_weight_sta(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_weight;
    oal_uint8 *puc_value = 0;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set = 0;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    // sh hipriv.sh "vap0 set_edca_weight_sta 1"
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_edca_opt_weight_sta::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡmac_vap */
    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (pst_mac_vap->en_vap_mode != WLAN_VAP_MODE_BSS_STA) {
        OAM_WARNING_LOG0(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_cycle_ap:: only AP_MODE support}");
        return OAL_FAIL;
    }

    /* ��ȡ����ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_cycle_ap::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /*lint -e734*/
    uc_weight = (oal_uint32)oal_atoi(ac_name);
    /*lint +e734*/
    /* ���Ȩ��Ϊ3 */
    if (uc_weight > 3) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "wal_hipriv_set_edca_opt_weight_sta: valid value is between 0 and %d", 3);
        return OAL_FAIL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_EDCA_OPT_WEIGHT_STA, OAL_SIZEOF(oal_uint8));
    puc_value = (oal_uint8 *)(st_write_msg.auc_value);
    *puc_value = uc_weight;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_weight_sta:: return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_edca_opt_switch_ap(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_flag;
    oal_uint8 *puc_value = 0;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set = 0;
    oal_int32 l_ret;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    // sh hipriv.sh "vap0 set_edca_switch_ap 1/0"
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_edca_opt_switch_ap::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡmac_vap */
    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (pst_mac_vap->en_vap_mode != WLAN_VAP_MODE_BSS_AP) {
        OAM_WARNING_LOG0(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_cycle_ap:: only AP_MODE support}");
        return OAL_FAIL;
    }

    /* ��ȡ���ò��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_cycle_ap::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_flag = (oal_uint8)oal_atoi(ac_name);

    /* �Ƿ����ò��� */
    if (uc_flag > 1) {
        OAM_WARNING_LOG0(0, OAM_SF_EDCA, "wal_hipriv_set_edca_opt_cycle_ap, invalid config, should be 0 or 1");
        return OAL_SUCC;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_EDCA_OPT_SWITCH_AP, OAL_SIZEOF(oal_uint8));
    puc_value = (oal_uint8 *)(st_write_msg.auc_value);
    *puc_value = uc_flag;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_switch_ap:: return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_edca_opt_cycle_ap(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_cycle_ms;
    oal_uint32 *pul_value = 0;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set = 0;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    // sh hipriv.sh "vap0 set_edca_cycle_ap 200"
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_edca_opt_cycle_ap::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡmac_vap */
    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (pst_mac_vap->en_vap_mode != WLAN_VAP_MODE_BSS_AP) {
        OAM_WARNING_LOG0(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_cycle_ap:: only AP_MODE support}");
        return OAL_FAIL;
    }

    /* ��ȡ����ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_cycle_ap::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    ul_cycle_ms = (oal_uint32)oal_atoi(ac_name);

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_EDCA_OPT_CYCLE_AP, OAL_SIZEOF(oal_uint32));
    pul_value = (oal_uint32 *)(st_write_msg.auc_value);
    *pul_value = ul_cycle_ms;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_edca_opt_cycle_ap:: return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_set_default_key(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    mac_setdefaultkey_param_stru st_payload_params = { 0 };
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    wal_msg_write_stru st_write_msg;

    /* 1.1 ��μ�� */
    if (OAL_ANY_NULL_PTR2(pst_net_dev, pc_param)) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_default_key::Param Check ERROR,pst_netdev, pst_params %x, %x!}\r\n", (uintptr_t)pst_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ��ȡkey_index */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    st_payload_params.uc_key_index = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡen_unicast */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    st_payload_params.en_unicast = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡmulticast */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    st_payload_params.en_multicast = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    /* 3.2 ��д msg ��Ϣ�� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        &st_payload_params, OAL_SIZEOF(mac_setdefaultkey_param_stru))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::memcpy fail!}");
        return OAL_FAIL;
    }

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DEFAULT_KEY, OAL_SIZEOF(mac_setdefaultkey_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_setdefaultkey_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_default_key::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_test_add_key(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    mac_addkey_param_stru st_payload_params;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_int32 l_sec_ret;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_uint8 auc_key[OAL_WPA_KEY_LEN] = { 0 };
    oal_int8 *pc_key = OAL_PTR_NULL;
    oal_uint32 ul_char_index;
    oal_uint16 us_len;
    wal_msg_stru *pst_rsp_msg = OAL_PTR_NULL;

    /* 1.1 ��μ�� */
    if (OAL_ANY_NULL_PTR2(pst_net_dev, pc_param)) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::Param Check ERROR,pst_netdev, pst_params %x, %x!}\r\n", (uintptr_t)pst_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }
    /*
     * xxx(cipher) xx(en_pairwise) xx(key_len) xxx(key_index) xxxx:xx:xx:xx:xx:xx...
     *  (key С��32�ֽ�) xx:xx:xx:xx:xx:xx(Ŀ�ĵ�ַ)
     */
    memset_s(&st_payload_params, OAL_SIZEOF(st_payload_params), 0, OAL_SIZEOF(st_payload_params));
    memset_s(&st_payload_params.st_key, OAL_SIZEOF(mac_key_params_stru), 0, OAL_SIZEOF(mac_key_params_stru));
    st_payload_params.st_key.seq_len = 6;
    memset_s(st_payload_params.auc_mac_addr, WLAN_MAC_ADDR_LEN, 0, WLAN_MAC_ADDR_LEN);

    /* ��ȡcipher */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    st_payload_params.st_key.cipher = (oal_uint32)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡen_pairwise */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    st_payload_params.en_pairwise = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡkey_len */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }

    pc_param = pc_param + ul_off_set;
    st_payload_params.st_key.key_len = (oal_uint8)oal_atoi(ac_name);
    if (OAL_VALUE_NOT_IN_VALID_RANGE(st_payload_params.st_key.key_len, 0, OAL_WPA_KEY_LEN)) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::Param Check ERROR! key_len[%x]  }\r\n",
                       (oal_int32)st_payload_params.st_key.key_len);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /* ��ȡkey_index */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    st_payload_params.uc_key_index = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡkey */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;
    pc_key = ac_name;
    /* 16����ת�� */
    for (ul_char_index = 0; ul_char_index < ul_off_set; ul_char_index++) {
        if (*pc_key == '-') {
            pc_key++;
            if (ul_char_index != 0) {
                ul_char_index--;
            }

            continue;
        }

        auc_key[ul_char_index / 2] = (oal_uint8)(auc_key[ul_char_index / 2] * 16 * (ul_char_index % 2) + oal_strtohex(pc_key));
        pc_key++;
    }

    l_sec_ret = memcpy_s(st_payload_params.st_key.auc_key, sizeof(st_payload_params.st_key.auc_key),
                         auc_key, (oal_uint32)st_payload_params.st_key.key_len);

    /* ��ȡĿ�ĵ�ַ */
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::wal_get_cmd_one_arg_etc fail.err code[%u]}", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_payload_params.auc_mac_addr, WLAN_MAC_ADDR_LEN);

    OAM_INFO_LOG3(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::key_len:%d, seq_len:%d, cipher:0x%08x!}\r\n",
                  st_payload_params.st_key.key_len, st_payload_params.st_key.seq_len, st_payload_params.st_key.cipher);

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    /* 3.2 ��д msg ��Ϣ�� */
    us_len = (oal_uint32)OAL_SIZEOF(mac_addkey_param_stru);

    l_sec_ret += memcpy_s((oal_int8 *)st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                          (oal_int8 *)&st_payload_params, (oal_uint32)us_len);

    if (l_sec_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_test_add_key::memcpy fail!}");
        return OAL_FAIL;
    }

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ADD_KEY, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_TRUE,
                                   &pst_rsp_msg);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_enable_pmf::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    if (OAL_SUCC != wal_check_and_release_msg_resp_etc(pst_rsp_msg)) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "wal_hipriv_test_add_key::wal_check_and_release_msg_resp_etc fail");
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_thruput_bypass(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_bypass_type;
    oal_uint8 uc_value;
    mac_cfg_set_thruput_bypass_stru *pst_set_bypass = OAL_PTR_NULL;

    /* ��ȡ�趨mib���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_thruput_bypass::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    uc_bypass_type = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ�趨�� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_thruput_bypass::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return (oal_uint32)ul_ret;
    }
    pc_param += ul_off_set;
    uc_value = (oal_uint8)oal_atoi(ac_name);

    pst_set_bypass = (mac_cfg_set_thruput_bypass_stru *)(st_write_msg.auc_value);
    pst_set_bypass->uc_bypass_type = uc_bypass_type;
    pst_set_bypass->uc_value = uc_value;
    us_len = OAL_SIZEOF(mac_cfg_set_thruput_bypass_stru);
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_THRUPUT_BYPASS, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_thruput_bypass::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_str_cmd(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    wal_ioctl_str_stru st_cmd_table;
    wal_ioctl_tlv_stru *pst_sub_cmd_table = OAL_PTR_NULL;
    mac_cfg_set_str_stru *pst_set_cmd = OAL_PTR_NULL;
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_len;
    oal_uint8 uc_cmd_cnt;
    oal_uint16 us_cfg_id;
    oal_uint8 uc_map_index = 0;
    oal_uint8 uc_value_base;
    oal_uint8 *pc_value = OAL_PTR_NULL;

    /* ��ȡ���ò������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* Ѱ��ƥ������� */
    st_cmd_table = g_ast_set_str_table[0];
    while (st_cmd_table.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_cmd_table.pc_name, ac_name)) {
            break;
        }
        st_cmd_table = g_ast_set_str_table[++uc_map_index];
    }

    /* û���ҵ���Ӧ������򱨴� */
    if (st_cmd_table.pc_name == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }

    /* ��¼�������Ӧ��ö��ֵ */
    us_cfg_id = g_ast_set_str_table[uc_map_index].en_tlv_cfg_id;
    pst_sub_cmd_table = g_ast_set_str_table[uc_map_index].pst_cfg_table;

    /* ��ȡ�趨mib���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ��ǰ����Ĳ������� */
    uc_cmd_cnt = (oal_uint8)oal_atoi(ac_name);
    if (uc_cmd_cnt > MAC_STR_CMD_MAX_SIZE) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd::too many para [%d]!}\r\n", uc_cmd_cnt);
        return OAL_FAIL;
    }

    uc_len = OAL_SIZEOF(mac_cfg_set_str_stru);
    pst_set_cmd = (mac_cfg_set_str_stru *)(st_write_msg.auc_value);
    pst_set_cmd->us_cfg_id = us_cfg_id;
    pst_set_cmd->uc_cmd_cnt = uc_cmd_cnt;
    pst_set_cmd->uc_len = uc_len;
    OAM_WARNING_LOG3(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd:: cfg id[%d] cfg len[%d] cmd cnt[%d]!}",
                     us_cfg_id, uc_len, uc_cmd_cnt);

    /* ������ȡÿ���ַ����� */
    for (uc_map_index = 0; uc_map_index < uc_cmd_cnt; uc_map_index++) {
        /* ��ȡ������ */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;

        /* ����������Ҷ�Ӧ�������� */
        while (pst_sub_cmd_table->pc_name != OAL_PTR_NULL) {
            if (0 == oal_strcmp(pst_sub_cmd_table->pc_name, ac_name)) {
                break;
            }
            pst_sub_cmd_table++;
        }

        /* û���ҵ���Ӧ������򱨴� */
        if (pst_sub_cmd_table->pc_name == OAL_PTR_NULL) {
            OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd::invalid alg_cfg command!}\r\n");
            return OAL_FAIL;
        }

        /* ��ȡ�趨�� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_value = ac_name;
        uc_value_base = 10;
        if (BASIC_VALUE_IS_HEX(ac_name)) {
            uc_value_base = 16;
            pc_value = ac_name + 2;
        }
        pc_param += ul_off_set;

        /* ��¼ÿ���������id�ź;��������ֵ */
        pst_set_cmd->us_set_id[uc_map_index] = pst_sub_cmd_table->en_tlv_cfg_id;
        pst_set_cmd->ul_value[uc_map_index] = (oal_uint32)oal_strtol(pc_value, OAL_PTR_NULL, uc_value_base);
        OAM_WARNING_LOG4(0, OAM_SF_ANY, "{wal_hipriv_set_str_cmd:: index[%d] sub cfg id[%d] ,uc_value_base=[%s] ,value[%d]!}",
                         uc_map_index, pst_set_cmd->us_set_id[uc_map_index], uc_value_base, pst_set_cmd->ul_value[uc_map_index]);
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, us_cfg_id, uc_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + uc_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tlv_cmd::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_tlv_cmd(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    wal_ioctl_tlv_stru st_tlv_cfg;
    mac_cfg_set_tlv_stru *pst_set_tlv = OAL_PTR_NULL;
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_len;
    oal_uint8 uc_cmd_type;
    oal_uint16 us_cfg_id;
    oal_uint32 ul_value;
    oal_uint8 uc_map_index = 0;

    /* ��ȡ���ò������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tlv_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* Ѱ��ƥ������� */
    st_tlv_cfg = g_ast_set_tlv_table[0];
    while (st_tlv_cfg.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_tlv_cfg.pc_name, ac_name)) {
            break;
        }
        st_tlv_cfg = g_ast_set_tlv_table[++uc_map_index];
    }

    /* û���ҵ���Ӧ������򱨴� */
    if (st_tlv_cfg.pc_name == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_tlv_cmd::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }

    /* ��¼�����Ӧ��ö��ֵ */
    us_cfg_id = g_ast_set_tlv_table[uc_map_index].en_tlv_cfg_id;

    /* ��ȡ�趨mib���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tlv_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pc_param += ul_off_set;
    uc_cmd_type = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ�趨�� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tlv_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    ul_value = (oal_uint32)oal_atoi(ac_name);

    uc_len = OAL_SIZEOF(mac_cfg_set_tlv_stru);

    pst_set_tlv = (mac_cfg_set_tlv_stru *)(st_write_msg.auc_value);
    pst_set_tlv->uc_cmd_type = uc_cmd_type;
    pst_set_tlv->us_cfg_id = us_cfg_id;
    pst_set_tlv->uc_len = uc_len;
    pst_set_tlv->ul_value = ul_value;

    OAM_WARNING_LOG4(0, OAM_SF_ANY, "{wal_hipriv_set_tlv_cmd:: cfg id[%d] cfg len[%d] cmd type[%d], set val[%d]!}",
                     us_cfg_id, uc_len, uc_cmd_type, ul_value);
    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, us_cfg_id, uc_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + uc_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_tlv_cmd::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_val_cmd(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    wal_ioctl_tlv_stru st_tlv_cfg;
    mac_cfg_set_tlv_stru *pst_set_tlv = OAL_PTR_NULL;
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_len;
    oal_uint16 us_cfg_id;
    oal_uint32 ul_value;
    oal_uint8 uc_map_index = 0;

    /* ��ȡ���ò������� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_val_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* Ѱ��ƥ������� */
    st_tlv_cfg = g_ast_set_tlv_table[0];
    while (st_tlv_cfg.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_tlv_cfg.pc_name, ac_name)) {
            break;
        }
        st_tlv_cfg = g_ast_set_tlv_table[++uc_map_index];
    }

    /* û���ҵ���Ӧ������򱨴� */
    if (st_tlv_cfg.pc_name == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_val_cmd::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }

    /* ��¼�����Ӧ��ö��ֵ */
    us_cfg_id = g_ast_set_tlv_table[uc_map_index].en_tlv_cfg_id;

    /* ��ȡ�趨ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_val_cmd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pc_param += ul_off_set;
    ul_value = (oal_uint32)oal_atoi(ac_name);

    uc_len = OAL_SIZEOF(mac_cfg_set_tlv_stru);

    pst_set_tlv = (mac_cfg_set_tlv_stru *)(st_write_msg.auc_value);
    pst_set_tlv->uc_cmd_type = 0xFF;
    pst_set_tlv->us_cfg_id = us_cfg_id;
    pst_set_tlv->uc_len = uc_len;
    pst_set_tlv->ul_value = ul_value;

    OAM_WARNING_LOG3(0, OAM_SF_ANY, "{wal_hipriv_set_val_cmd:: cfg id[%d], cfg len[%d],set_val[%d]!}", us_cfg_id, uc_len, ul_value);
    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, us_cfg_id, uc_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + uc_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_val_cmd::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_DFT_STAT

OAL_STATIC oal_uint32 wal_hipriv_performance_log_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_performance_switch_type;
    oal_uint8 uc_value;
    mac_cfg_set_performance_log_switch_stru *pst_set_performance_log_switch;

    /* ��ȡ�趨mib���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_performance_log_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    uc_performance_switch_type = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡ�趨�� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_performance_log_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return (oal_uint32)ul_ret;
    }
    pc_param += ul_off_set;
    uc_value = (oal_uint8)oal_atoi(ac_name);

    pst_set_performance_log_switch = (mac_cfg_set_performance_log_switch_stru *)(st_write_msg.auc_value);
    pst_set_performance_log_switch->uc_performance_log_switch_type = uc_performance_switch_type;
    pst_set_performance_log_switch->uc_value = uc_value;
    us_len = OAL_SIZEOF(mac_cfg_set_performance_log_switch_stru);
    OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_performance_log_switch::uc_performance_switch_type = %d, uc_value = %d!}\r\n", uc_performance_switch_type, uc_value);
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_PERFORMANCE_LOG_SWITCH, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_performance_log_switch::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_set_auto_protection(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint32 ul_auto_protection_flag;

    /* ��ȡmib���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_auto_protection::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    ul_auto_protection_flag = (oal_uint32)oal_atoi(ac_name);

    us_len = OAL_SIZEOF(oal_uint32);
    *(oal_uint32 *)(st_write_msg.auc_value) = ul_auto_protection_flag;
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_AUTO_PROTECTION, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_auto_protection::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_WAPI

#ifdef _PRE_WAPI_DEBUG
OAL_STATIC oal_uint32 wal_hipriv_show_wapi_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    mac_vap_stru *pst_mac_vap;
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_user_info_param_stru *pst_user_info_param;
    oal_uint8 auc_mac_addr[6] = { 0 }; /* ��ʱ�����ȡ��use��mac��ַ��Ϣ */
    oal_uint8 uc_char_index;
    oal_uint16 us_user_idx;
    /* ȥ���ַ����Ŀո� */
    pc_param++;

    /* ��ȡmac��ַ,16����ת�� */
    for (uc_char_index = 0; uc_char_index < 12; uc_char_index++) {
        if (*pc_param == ':') {
            pc_param++;
            if (uc_char_index != 0) {
                uc_char_index--;
            }

            continue;
        }

        auc_mac_addr[uc_char_index / 2] =
            (oal_uint8)(auc_mac_addr[uc_char_index / 2] * 16 * (uc_char_index % 2) +
                        oal_strtohex(pc_param));
        pc_param++;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_WAPI_INFO, OAL_SIZEOF(mac_cfg_user_info_param_stru));

    /* ����mac��ַ���û� */
    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);

    l_ret = (oal_int32)mac_vap_find_user_by_macaddr_etc(pst_mac_vap, auc_mac_addr, &us_user_idx);
    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_user_info::no such user!}\r\n");
        return OAL_FAIL;
    }

    /* ��������������� */
    pst_user_info_param = (mac_cfg_user_info_param_stru *)(st_write_msg.auc_value);
    pst_user_info_param->us_user_idx = us_user_idx;

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "wal_hipriv_show_wapi_info::us_user_idx %u", us_user_idx);
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_user_info_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ANY, "{wal_hipriv_user_info::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif /* #ifdef WAPI_DEBUG_MODE */

#endif /* #ifdef _PRE_WLAN_FEATURE_WAPI */

#ifdef _PRE_WLAN_FEATURE_FTM
OAL_STATIC wal_ftm_cmd_entry_stru g_ast_ftm_common_cmd[] = {
    { "enable_ftm_initiator", OAL_TRUE, BIT0 },
    { "enable",               OAL_TRUE, BIT2 },
    { "cali",                 OAL_TRUE, BIT3 },
    { "enable_ftm_responder", OAL_TRUE, BIT5 },
    { "enable_ftm_range",     OAL_TRUE, BIT8 },
    { "en_multipath",         OAL_TRUE, BIT12 },
};

OAL_STATIC OAL_INLINE oal_void ftm_debug_cmd_format_info(void)
{
    OAM_WARNING_LOG0(0, OAM_SF_FTM,
        "{CMD format::hipriv.sh wlan0 ftm_debug \
        enable_ftm_initiator[0|1] \
        enable[0|1] \
        cali[0|1] \
        enable_ftm_responder[0|1] \
        set_ftm_time t1[] t2[] t3[] t4[] \
        set_ftm_timeout ul_bursts_timeout ul_ftms_timeout \
        enable_ftm_responder[0|1] \
        enable_ftm_range[0|1] \
        get_cali \
        !!}\r\n");
    OAM_WARNING_LOG0(0, OAM_SF_FTM,
        "{CMD format::hipriv.sh wlan0 ftm_debug \
        send_initial_ftm_request channel[] ftms_per_burst[n] burst_num[n] en_lci_civic_request[0|1] asap[0|1] \
        bssid[xx:xx:xx:xx:xx:xx] format_bw[9~13] send_ftm bssid[xx:xx:xx:xx:xx:xx] \
        send_range_req mac[] num_rpt[0-65535] delay[] ap_cnt[] bssid[] channel[] bssid[] channel[] ...\
        set_location type[] mac[] mac[] mac[] \
        white_list index mac[xx:xx:xx:xx:xx:xx] \
        !!}\r\n");
}


OAL_STATIC oal_uint32 ftm_debug_parase_iftmr_cmd(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };

    *pul_offset = 0;

    /* �����ʽ:send_initial_ftm_request channel[] ftms_per_burst[n] burst_num[n]---����(2^n)��burst
                                    en_lci_civic_request[0|1] asap[0|1] bssid[xx:xx:xx:xx:xx:xx]
                                    format_bw[9~13] */
    /* ����channel */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_iftmr_cmd::get iftmr mode error, return.}");
        return ul_ret;
    }

    *pul_offset += ul_off_set;
    pst_debug_info->st_send_iftmr_bit1.uc_channel_num = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����ftms_per_burst */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_iftmr_cmd::get uc_ftms_per_burst error, return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_send_iftmr_bit1.uc_ftms_per_burst = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����burst */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_iftmr_cmd::get burst error, return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_send_iftmr_bit1.uc_burst_num = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����LCI and Location Civic Measurement requestʹ�ܱ�־ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_iftmr_cmd::get en_lci_civic_request error, return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_send_iftmr_bit1.en_lci_civic_request = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����asap,as soon as possibleʹ�ܱ�־ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_iftmr_cmd::get asap error, return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_send_iftmr_bit1.en_is_asap_on = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ��ȡBSSID */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_iftmr_cmd::No bssid, set the associated bssid.}");
        *pul_offset += ul_off_set;
        memset_s(pst_debug_info->st_send_iftmr_bit1.auc_bssid, OAL_SIZEOF(pst_debug_info->st_send_iftmr_bit1.auc_bssid),
                 0, OAL_SIZEOF(pst_debug_info->st_send_iftmr_bit1.auc_bssid));
        pc_param += ul_off_set;
        ul_off_set = 0;

        return OAL_SUCC;
    }
    *pul_offset += ul_off_set;
    oal_set_mac_addr(pst_debug_info->st_send_iftmr_bit1.auc_bssid, auc_mac_addr);
    pc_param += ul_off_set;
    ul_off_set = 0;

    OAM_WARNING_LOG3(0, OAM_SF_ANY, "{ftm_debug_parase_iftmr_cmd::bssid=*:*:*:%x:%x:%x}",
                     auc_mac_addr[3],
                     auc_mac_addr[4],
                     auc_mac_addr[5]);

    /* ����format bw */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_iftmr_cmd::get format bw error, return.}");
        return OAL_SUCC;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_send_iftmr_bit1.uc_format_bw = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_parase_ftm_cmd(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    oal_uint32 ul_off_set = 0;

    *pul_offset = 0;
    /* ����mac */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_ftm_cmd::ger mac error.}");
        return OAL_FAIL;
    }
    *pul_offset += ul_off_set;
    oal_strtoaddr(ac_value, OAL_SIZEOF(ac_value), pst_debug_info->st_send_ftm_bit4.auc_mac, WLAN_MAC_ADDR_LEN);
    pc_param += ul_off_set;
    ul_off_set = 0;

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_parase_ftm_set_white_list(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN] = { 0 };
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
    oal_uint32 ul_off_set = 0;
    oal_int32 l_ret;

    *pul_offset = 0;

    /* ����������index */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_ftm_set_white_list::ger index error.}");
        return OAL_FAIL;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_set_white_list_bit17.uc_index = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;
    OAM_WARNING_LOG1(0, OAM_SF_FTM, "{ftm_debug_parase_ftm_set_white_list::index=[%d].}",
        pst_debug_info->st_set_white_list_bit17.uc_index);

    /* ����������mac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_ftm_set_white_list::No mac addr,set the associated bssid.}");
        *pul_offset += ul_off_set;
        l_ret = memset_s(pst_debug_info->st_set_white_list_bit17.auc_mac,
                         OAL_SIZEOF(pst_debug_info->st_set_white_list_bit17.auc_mac),
                         0, OAL_SIZEOF(pst_debug_info->st_set_white_list_bit17.auc_mac));
        if (l_ret != EOK) {
            OAM_WARNING_LOG1(0, OAM_SF_FTM, "ftm_debug_parase_ftm_set_white_list::memcpy fail![%d]", l_ret);
        }
        pc_param += ul_off_set;
        ul_off_set = 0;

        return OAL_SUCC;
    }
    *pul_offset += ul_off_set;
    oal_set_mac_addr(pst_debug_info->st_set_white_list_bit17.auc_mac, auc_mac_addr);
    pc_param += ul_off_set;
    ul_off_set = 0;

    OAM_WARNING_LOG3(0, OAM_SF_ANY, "{ftm_debug_parase_ftm_set_white_list::bssid=*:*:*:%x:%x:%x}",
                     auc_mac_addr[3],
                     auc_mac_addr[4],
                     auc_mac_addr[5]);
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_parase_ftm_timeout_cmd(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    oal_uint32 ul_off_set = 0;

    *pul_offset = 0;
    /* ����ftm bursts timeout */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_ftm_time_cmd::get correct time1 error,return.}");
        return ul_ret;
    }

    *pul_offset += ul_off_set;
    pst_debug_info->st_ftm_timeout_bit14.ul_bursts_timeout = (oal_uint32)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����ftms timeout */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_ftm_time_cmd::get correct time2 error,return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_ftm_timeout_bit14.ul_ftms_timeout = (oal_uint32)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_set_cali(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;

    *pul_offset = 0;

    /* ����t1 */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_ftm_time_cmd::get correct time1 error, return.}");
        return ul_ret;
    }

    *pul_offset += ul_off_set;
    pst_debug_info->ul_ftm_cali = (oal_uint32)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_parase_range_req_cmd(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_ap_cnt;

    *pul_offset = 0;

    /* ����mac */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_range_req_cmd::mac error!.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    oal_strtoaddr(ac_value, OAL_SIZEOF(ac_value), pst_debug_info->st_send_range_req_bit7.auc_mac, WLAN_MAC_ADDR_LEN);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����num_rpt */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_range_req_cmd::get num_rpt error,return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_send_range_req_bit7.us_num_rpt = (oal_uint16)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����ap_cnt */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_range_req_cmd::get ap_cnt error,return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_send_range_req_bit7.uc_minimum_ap_count = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    for (uc_ap_cnt = 0; uc_ap_cnt < pst_debug_info->st_send_range_req_bit7.uc_minimum_ap_count; uc_ap_cnt++) {
        /* ����bssid */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_range_req_cmd::bssid error!.}");
            return ul_ret;
        }
        *pul_offset += ul_off_set;
        oal_strtoaddr(ac_value, OAL_SIZEOF(ac_value), pst_debug_info->st_send_range_req_bit7.aauc_bssid[uc_ap_cnt], WLAN_MAC_ADDR_LEN);
        pc_param += ul_off_set;
        ul_off_set = 0;

        /* ����channel */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_range_req_cmd::get channel error,return.}");
            return ul_ret;
        }
        *pul_offset += ul_off_set;
        pst_debug_info->st_send_range_req_bit7.auc_channel[uc_ap_cnt] = (oal_uint8)oal_atoi(ac_value);
        pc_param += ul_off_set;
        ul_off_set = 0;
    }
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_parase_m2s_cmd(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    ftm_m2s_stru *pst_m2s = &(pst_debug_info->st_m2s_bit11);

    *pul_offset = 0;

    /* ����m2s */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_m2s_cmd::get m2s error,return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_m2s->en_is_mimo = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����tx_chain */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_m2s_cmd::get tx_chain error,return.}");
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_m2s->uc_tx_chain_selection = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_parase_neighbor_report_req_cmd(oal_int8 *pc_param, mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set = 0;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };

    *pul_offset = 0;

    /* ����bssid */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM,
            "{ftm_debug_parase_neighbor_report_req_cmd::No bssid,set the associated bssid.}");
        *pul_offset += ul_off_set;
        memset_s(pst_debug_info->st_neighbor_report_req_bit15.auc_bssid, OAL_SIZEOF(pst_debug_info->st_neighbor_report_req_bit15.auc_bssid),
                 0, OAL_SIZEOF(pst_debug_info->st_neighbor_report_req_bit15.auc_bssid));
        pc_param += ul_off_set;
        ul_off_set = 0;
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    oal_set_mac_addr(pst_debug_info->st_neighbor_report_req_bit15.auc_bssid, auc_mac_addr);
    pc_param += ul_off_set;
    ul_off_set = 0;

    OAM_WARNING_LOG3(0, OAM_SF_ANY,
        "{ftm_debug_parase_neighbor_report_req_cmd::send neighbor request frame to ap[*:*:*:%x:%x:%x]}",
        auc_mac_addr[3], auc_mac_addr[4], auc_mac_addr[5]);

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 ftm_debug_parase_gas_init_req_cmd(oal_int8 *pc_param,
    mac_ftm_debug_switch_stru *pst_debug_info, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };

    /* �����ʽ:send_gas_init_req lci_enable[0|1] bssid[xx:xx:xx:xx:xx:xx] */

    *pul_offset = 0;

    /* ����lci_enable */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM,
            "{ftm_debug_parase_gas_init_req_cmd::get lci_enable error, use default value [true].}");
        ac_value[0] = OAL_TRUE;
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    pst_debug_info->st_gas_init_req_bit16.en_lci_enable = (oal_uint8)oal_atoi(ac_value);
    pc_param += ul_off_set;
    ul_off_set = 0;

    /* ����bssid */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_FTM, "{ftm_debug_parase_gas_init_req_cmd::No bssid, set the associated bssid.}");
        *pul_offset += ul_off_set;
        memset_s(pst_debug_info->st_gas_init_req_bit16.auc_bssid, OAL_SIZEOF(pst_debug_info->st_gas_init_req_bit16.auc_bssid),
                 0, OAL_SIZEOF(pst_debug_info->st_gas_init_req_bit16.auc_bssid));
        pc_param += ul_off_set;
        ul_off_set = 0;
        return ul_ret;
    }
    *pul_offset += ul_off_set;
    oal_set_mac_addr(pst_debug_info->st_gas_init_req_bit16.auc_bssid, auc_mac_addr);
    pc_param += ul_off_set;
    ul_off_set = 0;

    OAM_WARNING_LOG3(0, OAM_SF_ANY,
        "{ftm_debug_parase_gas_init_req_cmd::send gas initial request frame to ap[*:*:*:%x:%x:%x]}",
        auc_mac_addr[3], auc_mac_addr[4], auc_mac_addr[5]);
    return OAL_SUCC;
}



OAL_STATIC oal_uint32 wal_hipriv_set_white_list(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    mac_ftm_debug_switch_stru st_ftm_debug;
    oal_uint32 ul_ret = 0;
    oal_int32 l_ret;

    memset_s(&st_ftm_debug, OAL_SIZEOF(st_ftm_debug), 0, OAL_SIZEOF(st_ftm_debug));

    do {
        /* ��ȡ����ؼ��� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if ((ul_ret != OAL_SUCC) && (ul_off_set != 0)) {
            ftm_debug_cmd_format_info();
            return ul_ret;
        }
        pc_param += ul_off_set;

        if (ul_off_set == 0) {
            break;
        }

        if (0 == oal_strcmp("white_list", ac_name)) {
            ul_ret = ftm_debug_parase_ftm_set_white_list(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT17;

        } else {
            ftm_debug_cmd_format_info();
            return OAL_FAIL;
        }

    } while (*pc_param != '\0');

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ftm::ul_cmd_bit_map: 0x%08x.}", st_ftm_debug.ul_cmd_bit_map);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_FTM_DBG, OAL_SIZEOF(st_ftm_debug));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_ftm_debug,
                        OAL_SIZEOF(st_ftm_debug))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ftm::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_ftm_debug),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_FTM, "{wal_hipriv_ftm::return err code[%d]!}", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint8 wal_hipriv_ftm_is_common_cmd(oal_int8 ac_name[], oal_uint8 *puc_cmd_index)
{
    oal_uint8 uc_cmd_index;

    for (uc_cmd_index = 0; uc_cmd_index < OAL_SIZEOF(g_ast_ftm_common_cmd) / OAL_SIZEOF(wal_ftm_cmd_entry_stru); uc_cmd_index++) {
        if (0 == oal_strcmp(g_ast_ftm_common_cmd[uc_cmd_index].pc_cmd_name, ac_name)) {
            break;
        }
    }

    *puc_cmd_index = uc_cmd_index;
    if (uc_cmd_index == OAL_SIZEOF(g_ast_ftm_common_cmd) / OAL_SIZEOF(wal_ftm_cmd_entry_stru)) {
        return OAL_FALSE;
    } else {
        return OAL_TRUE;
    }
}


OAL_STATIC oal_uint32 wal_hipriv_ftm_set_common_cmd(oal_int8 *pc_param,
    oal_uint8 uc_cmd_index, mac_ftm_debug_switch_stru *pst_ftm_debug, oal_uint32 *pul_offset)
{
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_value[WAL_HIPRIV_CMD_VALUE_MAX_LEN];
    *pul_offset = 0;

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ftm_set_common_cmd:: ftm common cmd [%d].}", uc_cmd_index);

    /* ȡ��������ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_value, WAL_HIPRIV_CMD_VALUE_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        ftm_debug_cmd_format_info();
        return ul_ret;
    }

    /* ������������ֵ */
    if ((g_ast_ftm_common_cmd[uc_cmd_index].uc_is_check_para) &&
        OAL_VALUE_NOT_IN_VALID_RANGE(ac_value[0], '0', '9')) {
        ftm_debug_cmd_format_info();
        return OAL_FAIL;
    }

    *pul_offset += ul_off_set;
    ul_off_set = 0;

    /* ��д����ṹ�� */
    switch (g_ast_ftm_common_cmd[uc_cmd_index].ul_bit) {
        case BIT0:
            pst_ftm_debug->en_ftm_initiator_bit0 = ((oal_uint8)oal_atoi(ac_value));
            pst_ftm_debug->ul_cmd_bit_map |= BIT0;
            break;
        case BIT2:
            pst_ftm_debug->en_enable_bit2 = ((oal_uint8)oal_atoi(ac_value));
            pst_ftm_debug->ul_cmd_bit_map |= BIT2;
            break;
        case BIT3:
            pst_ftm_debug->en_cali_bit3 = ((oal_uint8)oal_atoi(ac_value));
            pst_ftm_debug->ul_cmd_bit_map |= BIT3;
            break;
        case BIT5:
            pst_ftm_debug->en_ftm_resp_bit5 = ((oal_uint8)oal_atoi(ac_value));
            pst_ftm_debug->ul_cmd_bit_map |= BIT5;
            break;
        case BIT8:
            pst_ftm_debug->en_ftm_range_bit8 = ((oal_uint8)oal_atoi(ac_value));
            pst_ftm_debug->ul_cmd_bit_map |= BIT8;
            break;
        case BIT12:
            pst_ftm_debug->en_multipath_bit12 = ((oal_uint8)oal_atoi(ac_value));
            pst_ftm_debug->ul_cmd_bit_map |= BIT12;
            break;
        default:
            OAM_WARNING_LOG1(0, OAM_SF_FTM, "{wal_hipriv_ftm_set_common_cmd::invalid cmd bit[0x%x]!}", g_ast_ftm_common_cmd[uc_cmd_index].ul_bit);
            break;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_ftm(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    mac_ftm_debug_switch_stru st_ftm_debug;
    oal_uint32 ul_ret = 0;
    oal_int32 l_ret;
    oal_uint8 uc_cmd_index = 0;

    memset_s(&st_ftm_debug, OAL_SIZEOF(st_ftm_debug), 0, OAL_SIZEOF(st_ftm_debug));

    do {
        /* ��ȡ����ؼ��� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if ((ul_ret != OAL_SUCC) && (ul_off_set != 0)) {
            ftm_debug_cmd_format_info();
            return ul_ret;
        }
        pc_param += ul_off_set;

        if (ul_off_set == 0) {
            break;
        }

        /* ������� */
        if (wal_hipriv_ftm_is_common_cmd(ac_name, &uc_cmd_index)) {
            ul_ret = wal_hipriv_ftm_set_common_cmd(pc_param, uc_cmd_index, &st_ftm_debug, &ul_off_set);
            pc_param += ul_off_set;
            ul_off_set = 0;
            if (ul_ret != OAL_SUCC) {
                return ul_ret;
            }
        } else if (0 == oal_strcmp("send_initial_ftm_request", ac_name)) {
            ul_ret = ftm_debug_parase_iftmr_cmd(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT1;
        } else if (0 == oal_strcmp("send_ftm", ac_name)) {
            ul_ret = ftm_debug_parase_ftm_cmd(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT4;
        } else if (0 == oal_strcmp("set_ftm_timeout", ac_name)) {
            ul_ret = ftm_debug_parase_ftm_timeout_cmd(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT14;
        } else if (0 == oal_strcmp("set_cali", ac_name)) {
            ul_ret = ftm_debug_set_cali(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT13;
        } else if (0 == oal_strcmp("send_range_req", ac_name)) {
            ul_ret = ftm_debug_parase_range_req_cmd(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT7;
        } else if (0 == oal_strcmp("get_cali", ac_name)) {
            st_ftm_debug.ul_cmd_bit_map |= BIT9;
        } else if (0 == oal_strcmp("set_ftm_m2s", ac_name)) {
            ul_ret = ftm_debug_parase_m2s_cmd(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT11;
        } else if (0 == oal_strcmp("send_nbr_rpt_req", ac_name)) {
            ul_ret = ftm_debug_parase_neighbor_report_req_cmd(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT15;
        } else if (0 == oal_strcmp("send_gas_init_req", ac_name)) {
            ul_ret = ftm_debug_parase_gas_init_req_cmd(pc_param, &st_ftm_debug, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                ftm_debug_cmd_format_info();
                return ul_ret;
            }
            pc_param += ul_off_set;
            ul_off_set = 0;
            st_ftm_debug.ul_cmd_bit_map |= BIT16;
        } else {
            ftm_debug_cmd_format_info();
            return OAL_FAIL;
        }
    } while (*pc_param != '\0');

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_ftm::ul_cmd_bit_map: 0x%08x.}", st_ftm_debug.ul_cmd_bit_map);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_FTM_DBG, OAL_SIZEOF(st_ftm_debug));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)&st_ftm_debug, OAL_SIZEOF(st_ftm_debug))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_ftm::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_ftm_debug),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_FTM, "{wal_hipriv_ftm::return err code[%d]!}", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_send_2040_coext(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set = 0;
    mac_cfg_set_2040_coexist_stru *pst_2040_coexist;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    pst_2040_coexist = (mac_cfg_set_2040_coexist_stru *)st_write_msg.auc_value;
    /* ��ȡmib���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_2040_coext::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    pst_2040_coexist->ul_coext_info = (oal_uint32)oal_atoi(ac_name);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_2040_coext::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    pst_2040_coexist->ul_channel_report = (oal_uint32)oal_atoi(ac_name);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SEND_2040_COEXT,
                           OAL_SIZEOF(mac_cfg_set_2040_coexist_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_set_2040_coexist_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_2040_coext::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_2040_coext_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    if (OAL_UNLIKELY(WAL_MSG_WRITE_MAX_LEN <= OAL_STRLEN(pc_param))) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_2040_coext_info:: pc_param overlength is %d}\n", OAL_STRLEN(pc_param));
        oal_print_hex_dump((oal_uint8 *)pc_param, WAL_MSG_WRITE_MAX_LEN, 32, "wal_hipriv_2040_coext_info: param is overlong:");
        return OAL_FAIL;
    }

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_2040_coext_info::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_2040_COEXT_INFO, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_2040_coext_info::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_get_version(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_get_version::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_GET_VERSION, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_get_version::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_OPMODE_NOTIFY

OAL_STATIC oal_uint32 wal_hipriv_set_opmode_notify(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_value = 0;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_OPMODE, "{wal_hipriv_set_opmode_notify::pst_net_dev or pc_param null ptr error %x, %x!}\r\n", (uintptr_t)pst_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* ���ù���ģʽ֪ͨ����: 0�ر� | 1��ʼ  �˴���������"1"��"0"����ac_name */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_OPMODE, "{wal_hipriv_set_opmode_notify::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ������ò�ͬ��ͨ�� */
    if (0 == (oal_strcmp("0", ac_name))) {
        uc_value = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        uc_value = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_OPMODE, "{wal_hipriv_set_opmode_notify::opmode notify command is error [%x]!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_value;

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_OPMODE_NOTIFY, OAL_SIZEOF(oal_uint8));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_OPMODE, "{wal_hipriv_set_opmode_notify::wal_hipriv_reset_device return err code = [%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_get_user_nssbw(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_add_user_param_stru *pst_add_user_param;
    mac_cfg_add_user_param_stru st_add_user_param; /* ��ʱ�����ȡ��use����Ϣ */
    oal_uint32 ul_get_addr_idx;

    /* ��ȡ�û������Ϳռ�����Ϣ: hipriv "vap0 add_user xx xx xx xx xx xx(mac��ַ)" */
    memset_s((oal_void *)&st_add_user_param, OAL_SIZEOF(mac_cfg_add_user_param_stru),
             0, OAL_SIZEOF(mac_cfg_add_user_param_stru));
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_add_user::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_add_user_param.auc_mac_addr, WLAN_MAC_ADDR_LEN);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    /* ��������������� */
    pst_add_user_param = (mac_cfg_add_user_param_stru *)(st_write_msg.auc_value);
    for (ul_get_addr_idx = 0; ul_get_addr_idx < WLAN_MAC_ADDR_LEN; ul_get_addr_idx++) {
        pst_add_user_param->auc_mac_addr[ul_get_addr_idx] = st_add_user_param.auc_mac_addr[ul_get_addr_idx];
    }

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_GET_USER_RSSBW, OAL_SIZEOF(mac_cfg_add_user_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_add_user_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_add_user::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_M2S


oal_uint32 wal_hipriv_mss_param_handle_etc(mac_m2s_mgr_stru *pst_m2s_mgr, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;

    /* 1.��ȡ�ڶ�����: �����л�����״̬ */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_mss_param_handle::wal_get_cmd_one_arg2 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_m2s_mgr->pri_data.mss_mode.en_mss_on = (oal_uint8)oal_atoi(ac_name);
    if (pst_m2s_mgr->pri_data.mss_mode.en_mss_on >= 2) {
        OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_mss_param_handle::pst_m2s_mgr->pri_data.mss_mode.en_mss_on error [%d] (0 or 1)!}\r\n",
                         pst_m2s_mgr->pri_data.mss_mode.en_mss_on);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_M2S_MODEM

oal_uint32 wal_hipriv_modem_param_handle_etc(mac_m2s_mgr_stru *pst_m2s_mgr, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;

    /* 1.��ȡ�ڶ�����: �����л�����״̬ */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_mss_param_handle::wal_get_cmd_one_arg2 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_m2s_mgr->pri_data.modem_mode.en_m2s_state = (oal_uint8)oal_atoi(ac_name);

    return OAL_SUCC;
}
#endif


oal_uint32 wal_hipriv_rssi_param_handle_etc(mac_m2s_mgr_stru *pst_m2s_mgr, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;

    /* 1.��ȡ�ڶ������� */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch::wal_get_cmd_one_arg2 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_m2s_mgr->pri_data.rssi_mode.uc_opt = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch::wal_get_cmd_one_arg2 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_m2s_mgr->pri_data.rssi_mode.c_value = (oal_int8)oal_atoi(ac_name);

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_set_m2s_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    mac_m2s_mgr_stru *pst_m2s_mgr;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };

    /* st_write_msg��������� */
    memset_s(&st_write_msg, OAL_SIZEOF(wal_msg_write_stru),
             0, OAL_SIZEOF(wal_msg_write_stru));

    /* ��������������� */
    pst_m2s_mgr = (mac_m2s_mgr_stru *)st_write_msg.auc_value;

    /* 1.��ȡ��һ������: mode */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch::wal_get_cmd_one_arg1 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_m2s_mgr->en_cfg_m2s_mode = (mac_m2s_mode_enum_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    switch (pst_m2s_mgr->en_cfg_m2s_mode) {
        /* mimo-siso�л�������ѯ */
        case MAC_M2S_MODE_QUERY:
            /* ���¼�dmac��ӡȫ�ֹ����������� */
            break;

        case MAC_M2S_MODE_MSS:
            ul_ret = wal_hipriv_mss_param_handle_etc(pst_m2s_mgr, pc_param);
            if (ul_ret != OAL_SUCC) {
                return ul_ret;
            }
            break;

        case MAC_M2S_MODE_DELAY_SWITCH:
        case MAC_M2S_MODE_SW_TEST:
        case MAC_M2S_MODE_HW_TEST:
            /* 1.��ȡ�ڶ�����: �����л�����״̬ */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch::wal_get_cmd_one_arg2 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_m2s_mgr->pri_data.test_mode.uc_m2s_state = (oal_uint8)oal_atoi(ac_name);
            if (pst_m2s_mgr->pri_data.test_mode.uc_m2s_state >= MAC_M2S_COMMAND_STATE_BUTT) {
                OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch::pst_m2s_mgr->pri_data.test_mode.uc_m2s_state error [%d] (<=4)!}\r\n",
                                 pst_m2s_mgr->pri_data.test_mode.uc_m2s_state);
                return OAL_ERR_CODE_INVALID_CONFIG;
            }

            /* ƫ�ƣ�ȡ��һ������ */
            pc_param = pc_param + ul_off_set;

            /* 2.��ȡ����������:��·���Ǹ�· 0Ϊ��· 1Ϊ��· ��ʱ��ʹ�� ��ȱTBD */
            ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
            if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch::wal_get_cmd_one_arg3 return err_code [%d]!}\r\n", ul_ret);
                return ul_ret;
            }

            pst_m2s_mgr->pri_data.test_mode.uc_master_id = (oal_uint8)oal_atoi(ac_name);
            if (pst_m2s_mgr->pri_data.test_mode.uc_master_id >= HAL_DEVICE_ID_BUTT) {
                OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch:: pst_m2s_mgr->pri_data.test_mode.uc_master_id error [%d] 0/1!}\r\n",
                                 pst_m2s_mgr->pri_data.test_mode.uc_master_id);
                return OAL_ERR_CODE_INVALID_CONFIG;
            }

            if (MAC_M2S_MODE_HW_TEST == pst_m2s_mgr->en_cfg_m2s_mode) {
                /* Ӳ�л�����ģʽ����Ĭ�����л����� */
                pst_m2s_mgr->pri_data.test_mode.en_m2s_type = WLAN_M2S_TYPE_HW;
            } else {
                /* ҵ���л������л�����ģʽ����Ĭ�����л����� */
                pst_m2s_mgr->pri_data.test_mode.en_m2s_type = WLAN_M2S_TYPE_SW;
            }

            /* ��ʶҵ������ */
            if (MAC_M2S_MODE_DELAY_SWITCH == pst_m2s_mgr->en_cfg_m2s_mode) {
                pst_m2s_mgr->pri_data.test_mode.uc_trigger_mode = WLAN_M2S_TRIGGER_MODE_COMMAND;
            } else {
                pst_m2s_mgr->pri_data.test_mode.uc_trigger_mode = WLAN_M2S_TRIGGER_MODE_TEST;
            }
            break;

        case MAC_M2S_MODE_RSSI:
            ul_ret = wal_hipriv_rssi_param_handle_etc(pst_m2s_mgr, pc_param);
            if (ul_ret != OAL_SUCC) {
                return ul_ret;
            }
            break;

#ifdef _PRE_WLAN_FEATURE_M2S_MODEM
        case MAC_M2S_MODE_MODEM:
            ul_ret = wal_hipriv_modem_param_handle_etc(pst_m2s_mgr, pc_param);
            if (ul_ret != OAL_SUCC) {
                return ul_ret;
            }
            break;
#endif

        default:
            OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_switch: en_cfg_m2s_mode[%d] error!}", pst_m2s_mgr->en_cfg_m2s_mode);
            return OAL_FAIL;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_M2S_SWITCH, OAL_SIZEOF(mac_m2s_mgr_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_m2s_mgr_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_M2S, "{wal_hipriv_set_m2s_stitch::wal_hipriv_reset_device return err code = [%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_radar_set(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    memset_s(&st_write_msg, OAL_SIZEOF(wal_msg_write_stru), 0, OAL_SIZEOF(wal_msg_write_stru));

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_RADAR, 0);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_radar_set::wal_send_cfg_event_etc return err code = [%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_vap_nss(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_vap_nss;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_vap_nss::wal_get_cmd_one_arg_etc fails!}\r\n");
        return ul_ret;
    }

    pc_param += ul_off_set;
    uc_vap_nss = (oal_uint8)oal_atoi((const oal_int8 *)ac_name);

    us_len = OAL_SIZEOF(oal_uint8);
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_vap_nss;

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_VAP_NSS, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_vap_nss::wal_hipriv_reset_device return err code = [%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#if (_PRE_WLAN_FEATURE_BLACKLIST_LEVEL != _PRE_WLAN_FEATURE_BLACKLIST_NONE)


OAL_STATIC oal_uint32 wal_hipriv_blacklist_add(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;
    mac_blacklist_stru *pst_blklst;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_blacklist_add:wal_get_cmd_one_arg_etc fail!}\r\n");
        return ul_ret;
    }
    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));
    pst_blklst = (mac_blacklist_stru *)(st_write_msg.auc_value);

    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), pst_blklst->auc_mac_addr, WLAN_MAC_ADDR_LEN); /* ���ַ� ac_name ת�������� mac_add[6] */

    us_len = OAL_SIZEOF(mac_blacklist_stru);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ADD_BLACK_LIST, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_blacklist_add:wal_send_cfg_event_etc return[%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_blacklist_del(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_blacklist_add:wal_get_cmd_one_arg_etc fail!}\r\n");
        return ul_ret;
    }
    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));

    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_write_msg.auc_value, WLAN_MAC_ADDR_LEN); /* ���ַ� ac_name ת�������� mac_add[6] */

    us_len = OAL_MAC_ADDR_LEN; /* OAL_SIZEOF(oal_uint8); */

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DEL_BLACK_LIST, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_blacklist_add:wal_send_cfg_event_etc return[%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_blacklist_mode(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_BLACKLIST_MODE);
}

OAL_STATIC oal_uint32 wal_hipriv_blacklist_show(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    memset_s(&st_write_msg, OAL_SIZEOF(wal_msg_write_stru), 0, OAL_SIZEOF(wal_msg_write_stru));

    us_len = 4; /* OAL_SIZEOF(oal_uint32) */
    *(oal_uint32 *)(st_write_msg.auc_value) = 1;

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_BLACKLIST_SHOW, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_blacklist_show:wal_send_cfg_event_etc return [%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_set_abl_on(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_AUTOBLACKLIST_ON);
}

OAL_STATIC oal_uint32 wal_hipriv_set_abl_aging(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_AUTOBLACKLIST_AGING);
}

OAL_STATIC oal_uint32 wal_hipriv_set_abl_threshold(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_AUTOBLACKLIST_THRESHOLD);
}

OAL_STATIC oal_uint32 wal_hipriv_set_abl_reset(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_AUTOBLACKLIST_RESET);
}
#endif
#ifdef _PRE_WLAN_FEATURE_ISOLATION

OAL_STATIC oal_uint32 wal_hipriv_set_isolation_mode(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_ISOLATION_MODE);
}

OAL_STATIC oal_uint32 wal_hipriv_set_isolation_type(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_bss;
    oal_uint8 uc_isolation;
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_isolation_type:wal_get_cmd_one_arg_etc fail!}\r\n");
        return ul_ret;
    }

    uc_bss = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_isolation_type::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_isolation = (oal_uint8)oal_atoi(ac_name);

    memset_s((oal_uint8 *)&st_write_msg, OAL_SIZEOF(st_write_msg), 0, OAL_SIZEOF(st_write_msg));

    us_len = 2; /* OAL_SIZEOF(oal_uint8); */
    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ISOLATION_TYPE, us_len);

    /* ��������������� */
    st_write_msg.auc_value[0] = uc_bss;
    st_write_msg.auc_value[1] = uc_isolation;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_isolation_type:wal_send_cfg_event_etc return[%d].}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_set_isolation_fwd(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_ISOLATION_FORWARD);
}

OAL_STATIC oal_uint32 wal_hipriv_set_isolation_clear(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_ISOLATION_CLEAR);
}

OAL_STATIC oal_uint32 wal_hipriv_set_isolation_show(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    return wal_hipriv_send_cfg_uint32_data_etc(pst_net_dev, pc_param, WLAN_CFGID_ISOLATION_SHOW);
}
#endif /* _PRE_WLAN_FEATURE_CUSTOM_SECURITY */


OAL_STATIC oal_uint32 wal_hipriv_vap_classify_en(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_val = 0xff;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret == OAL_SUCC) {
        ul_val = (oal_uint32)oal_atoi(ac_name);
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_VAP_CLASSIFY_EN, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_val;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_packet_xmit::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_vap_classify_tid(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_val = 0xff;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret == OAL_SUCC) {
        ul_val = (oal_uint32)oal_atoi(ac_name);
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_VAP_CLASSIFY_TID, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_val;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_packet_xmit::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_STA_PM

OAL_STATIC oal_uint32 wal_hipriv_sta_psm_param(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint16 us_beacon_timeout;
    oal_uint16 us_tbtt_offset;
    oal_uint16 us_ext_tbtt_offset;
    oal_uint16 us_dtim3_on;
    mac_cfg_ps_param_stru *pst_ps_para;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_sta_psm_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    us_beacon_timeout = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_sta_psm_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    us_tbtt_offset = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_sta_psm_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    us_ext_tbtt_offset = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_sta_psm_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    us_dtim3_on = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_PSM_PARAM, OAL_SIZEOF(mac_cfg_ps_param_stru));

    /* ��������������� */
    pst_ps_para = (mac_cfg_ps_param_stru *)(st_write_msg.auc_value);
    pst_ps_para->us_beacon_timeout = us_beacon_timeout;
    pst_ps_para->us_tbtt_offset = us_tbtt_offset;
    pst_ps_para->us_ext_tbtt_offset = us_ext_tbtt_offset;
    pst_ps_para->us_dtim3_on = us_dtim3_on;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_ps_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_PWR, "{wal_hipriv_sta_psm_param::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_sta_pm_on_etc(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_sta_pm_open;
    mac_cfg_ps_open_stru *pst_sta_pm_open;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_sta_pm_open::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_sta_pm_open = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_STA_PM_ON, OAL_SIZEOF(mac_cfg_ps_open_stru));

    /* ��������������� */
    pst_sta_pm_open = (mac_cfg_ps_open_stru *)(st_write_msg.auc_value);
    /* MAC_STA_PM_SWITCH_ON / MAC_STA_PM_SWITCH_OFF */
    pst_sta_pm_open->uc_pm_enable = uc_sta_pm_open;
    pst_sta_pm_open->uc_pm_ctrl_type = MAC_STA_PM_CTRL_TYPE_CMD;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_ps_open_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_PWR, "{wal_hipriv_sta_pm_open::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_TCP_OPT

OAL_STATIC oal_uint32 wal_hipriv_get_tcp_ack_stream_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_GET_TCP_ACK_STREAM_INFO, OAL_SIZEOF(oal_uint32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_arpoffload_info::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_tcp_tx_ack_opt_enable(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_val;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_tcp_tx_ack_opt_enable::wal_get_cmd_one_arg_etc vap name return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    pc_param += ul_off_set;

    ul_val = (oal_uint32)oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TX_TCP_ACK_OPT_ENALBE, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_val;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_arpoffload_info::wal_send_cfg_event_etc fail.return err code [%ud]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_tcp_rx_ack_opt_enable(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_val;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_tcp_rx_ack_opt_enable::wal_get_cmd_one_arg_etc vap name return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    ul_val = (oal_uint32)oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_RX_TCP_ACK_OPT_ENALBE, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_val;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_arpoffload_info::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_tcp_tx_ack_limit(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_val;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_tcp_tx_ack_limit::wal_get_cmd_one_arg_etc vap name return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    ul_val = (oal_uint32)oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TX_TCP_ACK_OPT_LIMIT, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_val;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_arpoffload_info::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_tcp_rx_ack_limit(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_val;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_tcp_tx_ack_limit::wal_get_cmd_one_arg_etc vap name return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    ul_val = (oal_uint32)oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_RX_TCP_ACK_OPT_LIMIT, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_val;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_arpoffload_info::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_enable_2040bss(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_uint8 uc_2040bss_switch;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_enable_2040bss::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    if ((0 != oal_strcmp("0", ac_name)) && (0 != oal_strcmp("1", ac_name))) {
        OAM_ERROR_LOG0(0, OAM_SF_CFG, "{wal_hipriv_enable_2040bss::invalid parameter.}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    uc_2040bss_switch = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_2040BSS_ENABLE, OAL_SIZEOF(oal_uint8));
    *((oal_uint8 *)(st_write_msg.auc_value)) = uc_2040bss_switch; /* ��������������� */

    ul_ret = (oal_uint32)wal_send_cfg_event_etc(pst_net_dev,
                                                WAL_MSG_TYPE_WRITE,
                                                WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                                (oal_uint8 *)&st_write_msg,
                                                OAL_FALSE,
                                                OAL_PTR_NULL);

    if (OAL_UNLIKELY(ul_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_enable_2040bss::return err code %d!}\r\n", ul_ret);
    }

    return ul_ret;
}

#ifdef _PRE_WLAN_FEATURE_TX_CLASSIFY_LAN_TO_WLAN

OAL_STATIC oal_uint32 wal_hipriv_set_tx_classify_switch(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_flag;
    oal_uint8 *puc_value = 0;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set = 0;
    oal_int32 l_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    // sh hipriv.sh "p2p-p2p0-0 set_tx_classify_switch 1/0"
    /* ��ȡ���ò��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_tx_classify_switch::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_flag = (oal_uint8)oal_atoi(ac_name);
    /* �Ƿ����ò��� */
    if (uc_flag > 1) {
        OAM_WARNING_LOG0(0, OAM_SF_EDCA, "wal_hipriv_set_tx_classify_switch::invalid config, should be 0 or 1");
        return OAL_SUCC;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TX_CLASSIFY_LAN_TO_WLAN_SWITCH, OAL_SIZEOF(oal_uint8));
    puc_value = (oal_uint8 *)(st_write_msg.auc_value);
    *puc_value = uc_flag;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_EDCA, "{wal_hipriv_set_tx_classify_switch:: return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif /* _PRE_WLAN_FEATURE_TX_CLASSIFY_LAN_TO_WLAN */


#ifdef _PRE_WLAN_FEATURE_AP_PM
OAL_STATIC oal_uint32 wal_hipriv_wifi_enable(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /* OAM logģ��Ŀ��ص�����: hipriv "Hisilicon0 enable 0 | 1"
        �˴���������"1"��"0"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wifi_enable::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����logģ����в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_wifi_enable::command param is error!}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_WIFI_EN, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wifi_enable::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}



OAL_STATIC oal_uint32 wal_hipriv_pm_info(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
        ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PM_INFO, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pm_info::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_pm_enable(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wifi_enable::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����logģ����в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_wifi_enable::command param is error!}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PM_EN, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_pm_enable::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_CHIP_TEST

OAL_STATIC oal_uint32 wal_hipriv_beacon_offload_test(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    wal_msg_write_stru st_write_msg;
    oal_uint8 *uc_param;
    oal_uint32 ul_off_set = 0;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    oal_uint8 i;

    uc_param = (oal_uint8 *)st_write_msg.auc_value;

    /* hipriv "Hisilicon0 beacon_offload_test param0 param1 param2 param3", */
    for (i = 0; i < 4; i++) {
        pc_param += ul_off_set;
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_beacon_offload_test::wal_get_cmd_one_arg_etc param[%d] fails!}", i);
            return ul_ret;
        }
        *uc_param = (oal_uint8)oal_atoi((const oal_int8 *)ac_name);
        uc_param++;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_BEACON_OFFLOAD_TEST, OAL_SIZEOF(wal_specific_event_type_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif

#ifdef _PRE_DEBUG_MODE_USER_TRACK

OAL_STATIC oal_uint32 wal_hipriv_report_thrput_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_usr_thrput_stru st_usr_thrput;

    /* sh hipriv.sh "vap_name thrput_stat  XX:XX:XX:XX:XX;XX 0|1" */
    /* ��ȡ�û�mac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, st_usr_thrput.auc_user_macaddr, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_thrput_stat::wal_hipriv_get_mac_addr_etc return [%d].}", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_thrput_stat::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_usr_thrput.uc_param = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_USR_THRPUT_STAT, OAL_SIZEOF(st_usr_thrput));

    /* ��д��Ϣ�壬���� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        &st_usr_thrput, OAL_SIZEOF(st_usr_thrput))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_report_thrput_stat::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_usr_thrput),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_report_thrput_stat::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif

#ifdef _PRE_SUPPORT_ACS

OAL_STATIC oal_uint32 wal_hipriv_acs(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint16 us_len;
    oal_int32 l_ret;

    if (OAL_UNLIKELY(WAL_MSG_WRITE_MAX_LEN <= OAL_STRLEN(pc_param))) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_acs:: pc_param overlength is %d}\n", OAL_STRLEN(pc_param));
        oal_print_hex_dump((oal_uint8 *)pc_param, WAL_MSG_WRITE_MAX_LEN, 32, "wal_hipriv_acs: param is overlong:");
        return OAL_FAIL;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_acs::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ACS_CONFIG, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_acs::return err code [%d]!}\r\n", l_ret);

        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_chan_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_int8 *pc_token;
    oal_int8 *pc_end;
    oal_int8 *pc_ctx;
    oal_int8 *pc_sep = " ";
    oal_uint8 uc_idx;
    oal_int32 l_scan_mode;
    oal_uint8 uc_chan_num = 0;

    mac_chan_stat_param_stru st_param;

    pc_token = oal_strtok(pc_param, pc_sep, &pc_ctx);
    if (pc_token == NULL) {
        return OAL_FAIL;
    }

    st_param.us_duration_ms = (oal_uint16)oal_strtol(pc_token, &pc_end, 10);

    pc_token = oal_strtok(OAL_PTR_NULL, pc_sep, &pc_ctx);
    if (pc_token == NULL) {
        return OAL_FAIL;
    }

    st_param.en_probe = oal_strtol(pc_token, &pc_end, 10) ? OAL_TRUE : OAL_FALSE;
    st_param.uc_chan_cnt = 0;

    pc_token = oal_strtok(OAL_PTR_NULL, pc_sep, &pc_ctx);
    if (pc_token == NULL) {
        return OAL_FAIL;
    }

    l_scan_mode = oal_strtol(pc_token, &pc_end, 10);
    ;

    if (OAL_VALUE_EQ_ANY2(l_scan_mode, 0, 2)) {
        for (uc_idx = 0; uc_idx < MAC_CHANNEL_FREQ_2_BUTT; uc_idx++) {
            if (OAL_SUCC == mac_is_channel_idx_valid_etc(WLAN_BAND_2G, uc_idx)) {
                mac_get_channel_num_from_idx_etc(WLAN_BAND_2G, uc_idx, &uc_chan_num);
                st_param.auc_channels[st_param.uc_chan_cnt++] = uc_chan_num;
            }
        }
    }

    if (OAL_VALUE_EQ_ANY2(l_scan_mode, 1, 2)) {
        for (uc_idx = 0; uc_idx < MAC_CHANNEL_FREQ_5_BUTT; uc_idx++) {
            if (OAL_SUCC == mac_is_channel_idx_valid_etc(WLAN_BAND_5G, uc_idx)) {
                mac_get_channel_num_from_idx_etc(WLAN_BAND_5G, uc_idx, &uc_chan_num);
                st_param.auc_channels[st_param.uc_chan_cnt++] = uc_chan_num;
            }
        }
    } else {
        for (uc_idx = 0; uc_idx < WLAN_MAX_CHANNEL_NUM; uc_idx++) {
            pc_token = oal_strtok(OAL_PTR_NULL, pc_sep, &pc_ctx);
            if (pc_token == NULL) {
                break;
            }
            st_param.auc_channels[st_param.uc_chan_cnt++] = (oal_uint8)oal_strtol(pc_token, &pc_end, 10);
        }
    }

    if (st_param.uc_chan_cnt == 0) {
        return OAL_FAIL;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        &st_param, OAL_SIZEOF(st_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_chan_stat::memcpy fail!}");
        return OAL_FAIL;
    }

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_CHAN_STAT, OAL_SIZEOF(st_param));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(st_param),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_chan_stat::return err code [%d]!}\r\n", l_ret);

        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_11V

OAL_STATIC oal_uint32 wal_hipriv_11v_cfg_wl_mgmt(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_debug_switch;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_BSSTRANSITION, "{wal_hipriv_11v_cfg_wl_mgmt::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }
    /* ����0Ϊ�ر�����λ 1λ������λ������������֧�� */
    uc_debug_switch = (oal_uint8)oal_atoi((const oal_int8 *)ac_name);
    switch (uc_debug_switch) {
        case 0:
        case 1:
            break;
        default:
            OAM_WARNING_LOG0(0, OAM_SF_BSSTRANSITION, "{wal_hipriv_11v_cfg_wl_mgmt::command param is error!}\r\n");
            return OAL_ERR_CODE_INVALID_CONFIG;
    }
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_11V_WL_MGMT_SWITCH, OAL_SIZEOF(oal_uint8));
    st_write_msg.auc_value[0] = uc_debug_switch; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_BSSTRANSITION, "{wal_hipriv_11v_cfg_wl_mgmt::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif  // _PRE_WLAN_FEATURE_11V

#if (defined(_PRE_WLAN_FEATURE_11V) && defined(_PRE_DEBUG_MODE)) || defined(_PRE_WLAN_FEATURE_11V_ENABLE)

OAL_STATIC oal_uint32 wal_hipriv_11v_cfg_bsst(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint8 uc_debug_switch;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_11v_cfg_bsst::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }
    /* ����0Ϊ�ر�����λ 1λ������λ������������֧�� */
    uc_debug_switch = (oal_uint8)oal_atoi((const oal_int8 *)ac_name);
    switch (uc_debug_switch) {
        case 0:
        case 1:
            break;
        default:
            OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_hipriv_11v_cfg_wl_mgmt::command param is error!}\r\n");
            return OAL_ERR_CODE_INVALID_CONFIG;
    }
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_11V_BSST_SWITCH, OAL_SIZEOF(oal_uint8));
    st_write_msg.auc_value[0] = uc_debug_switch; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_11v_cfg_bsst::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_11v_tx_query(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0 };

    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_11v_tx_query::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_11V_TX_QUERY, OAL_SIZEOF(auc_mac_addr));

    /* ��������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), auc_mac_addr, WLAN_MAC_ADDR_LEN)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_dpd_cfg::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(auc_mac_addr),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_11v_tx_query::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif


OAL_STATIC oal_uint32 wal_hipriv_set_priv_flag(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_flag[2];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    wal_msg_write_stru st_write_msg;
    oal_bool_enum_uint8 *pen_priv_flag = OAL_PTR_NULL;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_flag, OAL_SIZEOF(ac_flag), &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "wal_hipriv_set_priv_flag: get first arg fail.");
        return OAL_FAIL;
    }

    /***************************************************************************
                            ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_PRIV_FLAG, OAL_SIZEOF(oal_bool_enum_uint8));

    /* ��������������� */
    pen_priv_flag = (oal_bool_enum_uint8 *)(st_write_msg.auc_value);
    *pen_priv_flag = (oal_bool_enum_uint8)oal_atoi(ac_flag);

    if ((*pen_priv_flag != 0) && (*pen_priv_flag != 1)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "wal_hipriv_set_priv_flag: Inavlid flag,should be 0 or 1!");
        return OAL_FAIL;
    }

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "wal_hipriv_set_priv_flag:: priv_flag= %d.", *pen_priv_flag);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_bool_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_priv_flag::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_bw_fixed(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_flag[2];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    wal_msg_write_stru st_write_msg;
    oal_bool_enum_uint8 *puc_bw_fixed_flag = OAL_PTR_NULL;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_net_dev, pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_bw_fixed::pst_net_dev/p_param null ptr error!}\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (OAL_UNLIKELY(pst_mac_vap == NULL)) {
        OAM_ERROR_LOG0(0, OAM_SF_CFG, "{wal_hipriv_set_bw_fixed::can't get mac vap from netdevice priv data!}");
        return OAL_FAIL;
    }

    /* �豸��up״̬���������ã�������down */
    if (pst_mac_vap->en_vap_state != MAC_VAP_STATE_INIT) {
        OAM_ERROR_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ANY, "{wal_hipriv_set_bw_fixed::device is busy, please down it first %d!}\r\n", pst_mac_vap->en_vap_state);
        return OAL_FAIL;
    }

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_flag, OAL_SIZEOF(ac_flag), &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "wal_hipriv_set_bw_fixed: get first arg fail.");
        return OAL_FAIL;
    }

    /***************************************************************************
                            ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_BW_FIXED, OAL_SIZEOF(oal_uint8));

    /* ��������������� */
    puc_bw_fixed_flag = (oal_uint8 *)(st_write_msg.auc_value);
    *puc_bw_fixed_flag = (oal_uint8)oal_atoi(ac_flag);

    *puc_bw_fixed_flag = !!(*puc_bw_fixed_flag);

    OAM_WARNING_LOG1(0, OAM_SF_ANY, "wal_hipriv_set_bw_fixed:: bw_fixed = %d.", *puc_bw_fixed_flag);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_bool_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);
    if (l_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_bw_fixed::wal_send_cfg_event_etc fail.return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_PERFORM_STAT

OAL_STATIC oal_uint32 wal_hipriv_stat_tid_thrpt(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0 };
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_total_offset = 0;
    mac_cfg_stat_param_stru *pst_stat_param;

    /* vap0 stat_tid_thrpt xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_thrpt::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_STAT, OAL_SIZEOF(mac_cfg_stat_param_stru));
    pst_stat_param = (mac_cfg_stat_param_stru *)(st_write_msg.auc_value);

    pst_stat_param->en_stat_type = MAC_STAT_TYPE_TID_THRPT;
    pst_stat_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_thrpt::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_stat_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡtidno */
    pc_param = pc_param + ul_total_offset;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_stat_param->uc_tidno = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡͳ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_period = (oal_uint16)oal_atoi(ac_name);

    /* ��ȡͳ�ƴ��� */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_num = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_stat_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_thrpt::wal_hipriv_stat_tid_thrpt return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_stat_user_thrpt(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0 };
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_total_offset = 0;
    mac_cfg_stat_param_stru *pst_stat_param;

    /* vap0 stat_user_thrpt xx xx xx xx xx xx(mac��ַ) stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_user_thrpt::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_STAT, OAL_SIZEOF(mac_cfg_stat_param_stru));
    pst_stat_param = (mac_cfg_stat_param_stru *)(st_write_msg.auc_value);

    pst_stat_param->en_stat_type = MAC_STAT_TYPE_USER_THRPT;
    pst_stat_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_user_thrpt::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_stat_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡͳ������ */
    pc_param = pc_param + ul_total_offset;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_user_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_period = (oal_uint16)oal_atoi(ac_name);

    /* ��ȡͳ�ƴ��� */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_user_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_num = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_stat_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_user_thrpt::wal_hipriv_stat_tid_thrpt return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_stat_user_bsd(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_total_offset = 0;
    mac_cfg_stat_param_stru *pst_stat_param;

    /* vap0 stat_user_thrpt xx xx xx xx xx xx(mac��ַ) stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_STAT, OAL_SIZEOF(mac_cfg_stat_param_stru));
    pst_stat_param = (mac_cfg_stat_param_stru *)(st_write_msg.auc_value);

    pst_stat_param->en_stat_type = MAC_STAT_TYPE_USER_BSD;
    pst_stat_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_user_bsd::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_stat_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡͳ������ */
    pc_param = pc_param + ul_total_offset;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_user_bsd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_period = (oal_uint16)oal_atoi(ac_name);

    /* ��ȡͳ�ƴ��� */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_user_bsd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_num = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_stat_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_user_thrpt::wal_hipriv_stat_tid_thrpt return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_stat_vap_thrpt(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_stat_param_stru *pst_stat_param;

    /* vap0 stat_vap_thrpt stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_vap_thrpt::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_STAT, OAL_SIZEOF(mac_cfg_stat_param_stru));
    pst_stat_param = (mac_cfg_stat_param_stru *)(st_write_msg.auc_value);

    pst_stat_param->en_stat_type = MAC_STAT_TYPE_VAP_THRPT;
    pst_stat_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡͳ������ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_vap_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_period = (oal_uint16)oal_atoi(ac_name);

    /* ��ȡͳ�ƴ��� */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_vap_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_num = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_stat_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_vap_thrpt::wal_hipriv_stat_tid_thrpt return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_stat_tid_per(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_stat_param_stru *pst_stat_param;
    oal_uint32 ul_total_offset = 0;

    /* vap0 stat_tid_per xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_per::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_STAT, OAL_SIZEOF(mac_cfg_stat_param_stru));
    pst_stat_param = (mac_cfg_stat_param_stru *)(st_write_msg.auc_value);

    pst_stat_param->en_stat_type = MAC_STAT_TYPE_TID_PER;
    pst_stat_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_per::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_stat_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡtidno */
    pc_param = pc_param + ul_total_offset;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_per::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_stat_param->uc_tidno = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡͳ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_per::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_period = (oal_uint16)oal_atoi(ac_name);

    /* ��ȡͳ�ƴ��� */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_per::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_num = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_stat_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_per::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_stat_tid_delay(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_stat_param_stru *pst_stat_param;
    oal_uint32 ul_total_offset = 0;

    /* vap0 stat_tid_delay xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_delay::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_STAT, OAL_SIZEOF(mac_cfg_stat_param_stru));
    pst_stat_param = (mac_cfg_stat_param_stru *)(st_write_msg.auc_value);

    pst_stat_param->en_stat_type = MAC_STAT_TYPE_TID_DELAY;
    pst_stat_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_delay::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_stat_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡtidno */
    pc_param = pc_param + ul_total_offset;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_delay::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_stat_param->uc_tidno = (oal_uint8)oal_atoi(ac_name);

    /* ��ȡͳ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_delay::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_period = (oal_uint16)oal_atoi(ac_name);

    /* ��ȡͳ�ƴ��� */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_delay::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_stat_param->us_stat_num = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_stat_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_stat_tid_delay::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_display_tid_thrpt(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_display_param_stru *pst_display_param;
    oal_uint32 ul_total_offset = 0;

    /* vap0 stat_tid_thrpt xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_tid_thrpt::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_DISPLAY, OAL_SIZEOF(mac_cfg_display_param_stru));
    pst_display_param = (mac_cfg_display_param_stru *)(st_write_msg.auc_value);

    pst_display_param->en_stat_type = MAC_STAT_TYPE_TID_THRPT;
    pst_display_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_tid_thrpt::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_display_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡtidno */
    pc_param = pc_param + ul_total_offset;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_tid_thrpt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_display_param->uc_tidno = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_display_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_tid_thrpt::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_display_user_thrpt(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_display_param_stru *pst_display_param;
    oal_uint32 ul_total_offset = 0;

    /* vap0 stat_user_thrpt xx xx xx xx xx xx(mac��ַ) stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_user_thrpt::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_DISPLAY, OAL_SIZEOF(mac_cfg_display_param_stru));
    pst_display_param = (mac_cfg_display_param_stru *)(st_write_msg.auc_value);

    pst_display_param->en_stat_type = MAC_STAT_TYPE_USER_THRPT;
    pst_display_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_user_thrpt::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_display_param->auc_mac_addr, auc_mac_addr);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_display_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_user_thrpt::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_display_vap_thrpt(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_display_param_stru *pst_display_param;

    /* vap0 stat_vap_thrpt stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_vap_thrpt::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_DISPLAY, OAL_SIZEOF(mac_cfg_display_param_stru));
    pst_display_param = (mac_cfg_display_param_stru *)(st_write_msg.auc_value);

    pst_display_param->en_stat_type = MAC_STAT_TYPE_VAP_THRPT;
    pst_display_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_display_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_vap_thrpt::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_display_tid_per(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_display_param_stru *pst_display_param;
    oal_uint32 ul_total_offset = 0;

    /* vap0 stat_tid_per xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_tid_per::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_DISPLAY, OAL_SIZEOF(mac_cfg_display_param_stru));
    pst_display_param = (mac_cfg_display_param_stru *)(st_write_msg.auc_value);

    pst_display_param->en_stat_type = MAC_STAT_TYPE_TID_PER;
    pst_display_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_tid_per::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_display_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡtidno */
    pc_param = pc_param + ul_total_offset;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_tid_per::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_display_param->uc_tidno = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_display_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_tid_per::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_display_tid_delay(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0 };
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_display_param_stru *pst_display_param;
    oal_uint32 ul_total_offset = 0;

    /* vap0 stat_tid_delay xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���) */
    if (OAL_PTR_NULL == OAL_NET_DEV_PRIV(pst_net_dev)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_tid_delay::OAL_NET_DEV_PRIV(pst_net_dev) is null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* �����¼��ڴ� */
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PFM_DISPLAY, OAL_SIZEOF(mac_cfg_display_param_stru));
    pst_display_param = (mac_cfg_display_param_stru *)(st_write_msg.auc_value);

    pst_display_param->en_stat_type = MAC_STAT_TYPE_TID_DELAY;
    pst_display_param->uc_vap_id = ((mac_vap_stru *)OAL_NET_DEV_PRIV(pst_net_dev))->uc_vap_id;

    /* ��ȡmac��ַ */
    ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_mac_addr, &ul_total_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_display_tid_delay::wal_hipriv_get_mac_addr_etc failed!}\r\n");
        return ul_ret;
    }
    oal_set_mac_addr(pst_display_param->auc_mac_addr, auc_mac_addr);

    /* ��ȡtidno */
    pc_param = pc_param + ul_total_offset;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_tid_delay::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_display_param->uc_tidno = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_display_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_display_tid_delay::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_DAQ

OAL_STATIC oal_uint32 wal_hipriv_data_acq(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;
    if (OAL_UNLIKELY(WAL_MSG_WRITE_MAX_LEN <= OAL_STRLEN(pc_param))) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_data_acq:: pc_param overlength is %d}\n", OAL_STRLEN(pc_param));
        return OAL_FAIL;
    }

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    while (*pc_param == ' ') {
        ++pc_param;
    }
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_data_acq::memcpy fail!}");
        return OAL_FAIL;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';

    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DATA_ACQ, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_data_acq::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_PSD_ANALYSIS

OAL_STATIC oal_uint32 wal_hipriv_set_psd(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_value;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_cfg_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_psd::pst_cfg_net_dev or pc_param null ptr error %x, %x!}\r\n", (uintptr_t)pst_cfg_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* PSD���ÿ��ص�����: hipriv "vap0 set_psd 0 | 1"
            �˴���������"1"��"0"����ac_name
    */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_psd::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    if (0 == (oal_strcmp("0", ac_name))) {
        ul_value = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        ul_value = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_psd::the set psd command is error %x!}\r\n", (uintptr_t)ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_PSD, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_value;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_psd::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_cfg_psd(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    oal_int32 l_ret;
    oal_bool_enum_uint8 en_enable;
    wal_msg_write_stru st_write_msg;

    /* PSD���ÿ��ص�����: hipriv "vap0 cfg_psd */

    en_enable = 1;
    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_CFG_PSD, OAL_SIZEOF(oal_uint32));

    /* ��������������� */
    *((oal_bool_enum_uint8 *)(st_write_msg.auc_value)) = en_enable;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_bool_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_cfg_psd::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif
#ifdef _PRE_WLAN_FEATURE_CSI

OAL_STATIC oal_uint32 wal_hipriv_set_csi(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_csi_param_stru *pst_cfg_csi_param;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
    oal_uint8 uc_csi_en;
    oal_uint8 uc_csi_bw;
    oal_uint8 uc_csi_frame_type;
    oal_uint8 uc_csi_sample_period;
    oal_uint8 uc_csi_phy_report_mode;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_cfg_net_dev, pc_param))) {
        OAM_ERROR_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_csi::pst_cfg_net_dev or pc_param null ptr error %x, %x!}\r\n", (uintptr_t)pst_cfg_net_dev, (uintptr_t)pc_param);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /* PSD���ÿ��ص�����: hipriv "vap0 set_csi ta ta_check csi_en"
       TA:������CSI��mac��ַ��Ϊ0��ʾ��ʹ��
       TA_check: Ϊ1ʱ��TA��Ч����ʾÿ�βɼ�CSI��Ϣʱ��ȶ�ta��
       csi_en:   Ϊ1ʱ����ʾʹ��CSI�ɼ�
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, 0, "{wal_hipriv_set_csi::wal_get_cmd_one_arg_etc get mac addr return err[%d]!}", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_csi::wal_get_cmd_one_arg_etc get csi en return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_csi_en = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_csi::wal_get_cmd_one_arg_etc get csi bw return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_csi_bw = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_csi::wal_get_cmd_one_arg_etc get csi frame type return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_csi_frame_type = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_csi::wal_get_cmd_one_arg_etc get csi sample period return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_csi_sample_period = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_csi::wal_get_cmd_one_arg_etc get csi phy report mode return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_csi_phy_report_mode = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_CSI, OAL_SIZEOF(mac_cfg_csi_param_stru));

    /* ��������������� */
    pst_cfg_csi_param = (mac_cfg_csi_param_stru *)(st_write_msg.auc_value);
    oal_set_mac_addr(pst_cfg_csi_param->auc_mac_addr, auc_mac_addr);
    pst_cfg_csi_param->en_cfg_csi_on = uc_csi_en;
    pst_cfg_csi_param->uc_csi_bw = uc_csi_bw;
    pst_cfg_csi_param->uc_csi_frame_type = uc_csi_frame_type;
    pst_cfg_csi_param->uc_csi_sample_period = uc_csi_sample_period;
    pst_cfg_csi_param->uc_csi_phy_report_mode = uc_csi_phy_report_mode;

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_csi_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_csi::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif
#ifdef _PRE_WLAN_CHIP_TEST

OAL_STATIC oal_uint32 wal_hipriv_lpm_soc_mode(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_lpm_soc_set_stru *pst_set_para;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;

    /* SOC���ܲ���ģʽ����, hipriv "Hisilicon0 lpm_soc_mode 0|1|2|3|4(����gating|PCIE RD BY PASS|mem precharge|PCIE L0-S|PCIE L1-0)
     * 0|1(disable|enable) pcie_idle(PCIE�͹��Ŀ���ʱ��1~7us) "
     */

    pst_set_para = (mac_cfg_lpm_soc_set_stru *)(st_write_msg.auc_value);
    /* ��������������� */
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ����ģʽ */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_soc_mode::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_set_para->en_mode = (mac_lpm_soc_set_enum_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡ�������ǹر� */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_soc_mode::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_set_para->uc_on_off = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡPCIE����ʱ������ */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_soc_mode::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pst_set_para->uc_pcie_idle = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_SOC_MODE, OAL_SIZEOF(mac_cfg_lpm_soc_set_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_soc_set_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_soc_mode::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_chip_state(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;
    mac_cfg_lpm_sleep_para_stru *pst_set_para;

    pst_set_para = (mac_cfg_lpm_sleep_para_stru *)(st_write_msg.auc_value);
    /* ��������������� */
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_set_para->uc_pm_switch = (mac_lpm_state_enum_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡ��ʱ˯�߲��� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_set_para->us_sleep_ms = (oal_uint16)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_CHIP_STATE, OAL_SIZEOF(mac_cfg_lpm_sleep_para_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_sleep_para_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_chip_state::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_psm_param(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_lpm_psm_param_stru *pst_psm_para;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* psm���ܼĴ�������, hipriv "Hisilicon0 lpm_psm_param 0|1(ps off|ps on) 0|1(DTIM|listen intval) xxx(listen intervalֵ) xxx(TBTT offset)" */

    pst_psm_para = (mac_cfg_lpm_psm_param_stru *)(st_write_msg.auc_value);
    /* ��������������� */
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ�����Ƿ��� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_psm_para->uc_psm_on = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡ��DTIM���ѻ���listen interval���� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_psm_para->uc_psm_wakeup_mode = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡlisten interval��ֵ */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_psm_para->us_psm_listen_interval = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡTBTT�ж���ǰ����ֵ */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_psm_para->us_psm_tbtt_offset = (oal_uint8)oal_atoi(ac_name);
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_PSM_PARAM, OAL_SIZEOF(mac_cfg_lpm_psm_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_psm_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_psm_param::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_smps_mode(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;
    oal_uint8 uc_smps_mode;

    wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);

    uc_smps_mode = (oal_uint8)oal_atoi(ac_name);
    if (uc_smps_mode >= 3) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_smps_mode::invalid choice [%d]!}\r\n", uc_smps_mode);

        return OAL_ERR_CODE_ARRAY_OVERFLOW;
    }
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    /* ��������������� */
    *((oal_uint8 *)(st_write_msg.auc_value)) = uc_smps_mode;

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_SMPS_MODE, OAL_SIZEOF(oal_uint8));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_smps_mode::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_smps_stub(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_lpm_smps_stub_stru *pst_smps_stub;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* smps ap������׮, hipriv "vap0 lpm_smps_stub 0|1|2(off|����|˫��) 0|1(�Ƿ�RTS) */
    /* ��������������� */
    pst_smps_stub = (mac_cfg_lpm_smps_stub_stru *)(st_write_msg.auc_value);
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ׮���� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_smps_stub->uc_stub_type = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* RTS */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_smps_stub->uc_rts_en = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_SMPS_STUB, OAL_SIZEOF(mac_cfg_lpm_smps_stub_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_smps_stub_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_smps_stub::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_txopps_set(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_lpm_txopps_set_stru *pst_txopps_set;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* txop ps���ܼĴ�������, hipriv "Hisilicon0 lpm_txopps_set 0|1(off|on|debug) 0|1(contion1 off|on) 0|1(condition2 off|on)" */
    /* ��������������� */
    pst_txopps_set = (mac_cfg_lpm_txopps_set_stru *)(st_write_msg.auc_value);
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ�����Ƿ��� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_txopps_set->uc_txop_ps_on = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡcondition1 */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_txopps_set->uc_conditon1 = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡcondition2 */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_txopps_set->uc_conditon2 = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_TXOP_PS_SET, OAL_SIZEOF(mac_cfg_lpm_txopps_set_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_txopps_set_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_txopps_set::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_txopps_tx_stub(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_lpm_txopps_tx_stub_stru *pst_txopps_tx_stub;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* txop ps�������Դ�׮����, hipriv "vap0 lpm_txopps_tx_stub 0|1(off|on) xxx(�ڼ�������׮)" */
    /* ��������������� */
    pst_txopps_tx_stub = (mac_cfg_lpm_txopps_tx_stub_stru *)(st_write_msg.auc_value);
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ׮���� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_txopps_tx_stub->uc_stub_on = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡ�ڼ������Ĵ�׮ */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_txopps_tx_stub->us_begin_num = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_TXOP_TX_STUB, OAL_SIZEOF(mac_cfg_lpm_txopps_tx_stub_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_txopps_tx_stub_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_txopps_tx_stub::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_tx_data(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_lpm_tx_data_stru *pst_lpm_tx_data;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* ���Է���, hipriv "vap0 lpm_tx_data xxx(����) xxx(����) xx:xx:xx:xx:xx:xx(Ŀ��mac) xxx(AC����)" */
    pst_lpm_tx_data = (mac_cfg_lpm_tx_data_stru *)(st_write_msg.auc_value);
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ�������� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_lpm_tx_data->us_num = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡ�������� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_lpm_tx_data->us_len = (oal_uint16)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡĿ�ĵ�ַ */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), pst_lpm_tx_data->auc_da, WLAN_MAC_ADDR_LEN);
    pc_param = pc_param + ul_off_set;

    /* ��ȡ����AC���� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_lpm_tx_data->uc_ac = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_TX_DATA, OAL_SIZEOF(mac_cfg_lpm_tx_data_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_tx_data_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_tx_data::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_lpm_tx_probe_request(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_lpm_tx_data_stru *pst_lpm_tx_data;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* ���Է���, hipriv "vap0 lpm_tx_probe_request 0|1(����|����) xx:xx:xx:xx:xx:xx(����ģʽ��BSSID)" */
    pst_lpm_tx_data = (mac_cfg_lpm_tx_data_stru *)(st_write_msg.auc_value);
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ����or����probe request */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_lpm_tx_data->uc_positive = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /* ��ȡbssid */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), pst_lpm_tx_data->auc_da, WLAN_MAC_ADDR_LEN);
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_LPM_TX_PROBE_REQUEST, OAL_SIZEOF(mac_cfg_lpm_tx_data_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_lpm_tx_data_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_tx_probe_request::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_remove_user_lut(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_remove_lut_stru *pst_param; /* ���︴��ɾ���û���������Ľṹ�� */
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
    mac_vap_stru *pst_mac_vap;
    oal_uint16 us_user_idx;

    /* ɾ���ָ��û�lut��, hipriv "vap0 remove_lut xx:xx:xx:xx:xx:xx(mac��ַ) 0|1(�ָ�/ɾ��)" */
    pst_param = (mac_cfg_remove_lut_stru *)(st_write_msg.auc_value);

    /* ��ȡMAC��ַ�ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_remove_user_lut::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
    }

    /* ��ַ�ַ���ת��ַ���� */
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);

    /* ��ȡ �ָ�/ɾ�� ��ʶ */
    pc_param += ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_remove_user_lut::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    pst_param->uc_is_remove = (oal_uint8)oal_atoi(ac_name);

    /* ����mac��ַ���û� */
    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);

    ul_ret = mac_vap_find_user_by_macaddr_etc(pst_mac_vap, auc_mac_addr, &us_user_idx);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_remove_user_lut::no such user!}\r\n");

        return ul_ret;
    }

    pst_param->us_user_idx = us_user_idx;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_REMOVE_LUT, OAL_SIZEOF(mac_cfg_kick_user_param_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_kick_user_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ANY, "{wal_hipriv_remove_user_lut::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

OAL_STATIC oal_uint32 wal_hipriv_parase_send_frame_body(oal_int8 *p_str_body,
                                                        oal_uint8 uc_str_body_len, oal_uint8 *p_char_body, oal_uint8 uc_char_body_lenth, oal_uint8 *p_char_body_len)
{
    oal_uint8 uc_index;
    oal_uint8 uc_value = 0;
    oal_uint8 uc_hvalue = 0;
    oal_uint8 uc_char_body_len = 0;

    if (OAL_ANY_NULL_PTR3(p_str_body, p_char_body, p_char_body_len)) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_parase_send_frame_body::param is NULL,return!}\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }

    if ((uc_str_body_len % 2) != 0) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_parase_send_frame_body::uc_str_body_len=%d invalid,return!}\r\n", uc_str_body_len);
        return OAL_FAIL;
    }

    for (uc_index = 0; uc_index < uc_str_body_len; uc_index++) {
        if (p_str_body[uc_index] >= '0' && p_str_body[uc_index] <= '9') {
            uc_value = (oal_uint8)(p_str_body[uc_index] - '0');
        } else if (p_str_body[uc_index] >= 'A' && p_str_body[uc_index] <= 'F') {
            uc_value = (oal_uint8)(p_str_body[uc_index] - 'A' + 10);
        } else {
            OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_parase_send_frame_body::p_str_body[%d]=%c invalid,return!}\r\n", uc_index, (oal_uint8)p_str_body[uc_index]);
            return OAL_FAIL;
        }

        if ((uc_index % 2) == 0) {
            uc_hvalue = ((uc_value << 4) & 0XF0);
        } else if ((uc_index % 2) == 1) {
            p_char_body[uc_char_body_len] = uc_value + uc_hvalue;
            uc_value = 0;
            uc_char_body_len++;
            if (uc_char_body_len >= uc_char_body_lenth) {
                OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_parase_send_frame_body::uc_char_body_len =%d over_max = %d!}\r\n",
                                 uc_char_body_len, MAC_TEST_INCLUDE_FRAME_BODY_LEN);
                return OAL_FAIL;
            }
        } else {
            uc_value = 0;
        }
    }

    *p_char_body_len = uc_char_body_len;
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_send_frame(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_offset;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    mac_cfg_send_frame_param_stru *pst_test_send_frame = OAL_PTR_NULL;
    oal_uint8 auc_mac_addr[WLAN_MAC_ADDR_LEN] = { 0 };

    mac_test_frame_type_enum_uint8 en_frame_type;
    oal_uint8 uc_pkt_num;
    oal_int8 ac_str_frame_body[MAC_TEST_INCLUDE_FRAME_BODY_LEN * 2] = { 0 };
    oal_uint8 uc_str_frame_body_len;
    oal_uint8 ac_frame_body[MAC_TEST_INCLUDE_FRAME_BODY_LEN] = { 0 };
    oal_uint8 uc_frame_body_len = 0;

    /* ��ȡ֡���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_frame::get frame type err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    en_frame_type = (mac_test_frame_type_enum_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_offset;

    /* ��ȡ֡��Ŀ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_frame::get frame num err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_pkt_num = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_offset;

    /* ��ȡMAC��ַ�ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_offset);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_frame::get mac err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    /* ��ַ�ַ���ת��ַ���� */
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);
    pc_param += ul_offset;

    if (en_frame_type == MAC_TEST_MGMT_ACTION || en_frame_type == MAC_TEST_MGMT_BEACON_INCLUDE_IE) {
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_str_frame_body, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_offset);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_frame::get frame body err_code = [%d]!}\r\n", ul_ret);
            return ul_ret;
        }

        pc_param += ul_offset;
        uc_str_frame_body_len = (oal_uint8)(OAL_STRLEN(ac_str_frame_body));
        ul_ret = wal_hipriv_parase_send_frame_body(ac_str_frame_body, uc_str_frame_body_len, ac_frame_body, sizeof(ac_frame_body), &uc_frame_body_len);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_frame::parase_send_frame_body err_code = [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
    }

    /***************************************************************************
                                 ���¼���dmac�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SEND_FRAME, OAL_SIZEOF(mac_cfg_send_frame_param_stru));

    /* ��������������� */
    pst_test_send_frame = (mac_cfg_send_frame_param_stru *)(st_write_msg.auc_value);
    memset_s(pst_test_send_frame, OAL_SIZEOF(mac_cfg_send_frame_param_stru),
             0, OAL_SIZEOF(mac_cfg_send_frame_param_stru));
    oal_set_mac_addr(pst_test_send_frame->auc_mac_ra, auc_mac_addr);
    pst_test_send_frame->en_frame_type = en_frame_type;
    pst_test_send_frame->uc_pkt_num = uc_pkt_num;
    pst_test_send_frame->uc_frame_body_length = uc_frame_body_len;
    if (EOK != memcpy_s(pst_test_send_frame->uc_frame_body, sizeof(pst_test_send_frame->uc_frame_body),
                        ac_frame_body, uc_frame_body_len)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_send_frame::memcpy fail!}");
        return OAL_FAIL;
    }

    OAM_WARNING_LOG3(0, OAM_SF_ANY, "{wal_hipriv_send_frame:: frame_type = [%d] send_times = %d body_len=%d!}\r\n",
                     pst_test_send_frame->en_frame_type, pst_test_send_frame->uc_pkt_num, pst_test_send_frame->uc_frame_body_length);
    ul_ret = (oal_uint32)wal_send_cfg_event_etc(pst_net_dev,
                                                WAL_MSG_TYPE_WRITE,
                                                WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_send_frame_param_stru),
                                                (oal_uint8 *)&st_write_msg,
                                                OAL_FALSE,
                                                OAL_PTR_NULL);

    if (OAL_UNLIKELY(ul_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_frame::wal_send_cfg_event_etc return err_code [%d]!}\r\n", ul_ret);
        return (oal_uint32)ul_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_CHIP_TEST

OAL_STATIC oal_uint32 wal_hipriv_set_rx_pn(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 auc_mac_addr[OAL_MAC_ADDR_LEN];
    mac_vap_stru *pst_mac_vap;
    oal_uint16 us_user_idx;
    mac_cfg_set_rx_pn_stru *pst_rx_pn;
    oal_uint16 us_pn;
    /* ��ȡMAC��ַ�ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_rx_pn::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��ַ�ַ���ת��ַ���� */
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), auc_mac_addr, WLAN_MAC_ADDR_LEN);
    pc_param += ul_off_set;

    /* ��ȡpn�� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_rx_pn::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    us_pn = (oal_uint16)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /* ����mac��ַ���û� */
    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    ul_ret = mac_vap_find_user_by_macaddr_etc(pst_mac_vap, auc_mac_addr, &us_user_idx);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_rx_pn::no such user!}\r\n");

        return ul_ret;
    }

    pst_rx_pn = (mac_cfg_set_rx_pn_stru *)(st_write_msg.auc_value);
    pst_rx_pn->us_rx_pn = us_pn;
    pst_rx_pn->us_user_idx = us_user_idx;
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(mac_cfg_set_rx_pn_stru);
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_RX_PN_REG, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_ANY, "{wal_hipriv_set_rx_pn::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_soft_retry(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_software_retry;
    oal_uint8 uc_retry_test;
    mac_cfg_set_soft_retry_stru *pst_soft_retry;
    /* �Ƿ�Ϊtest�����ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_soft_retry::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_retry_test = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /* ��ȡ�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_soft_retry::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_software_retry = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    pst_soft_retry = (mac_cfg_set_soft_retry_stru *)(st_write_msg.auc_value);
    pst_soft_retry->uc_retry_test = uc_retry_test;
    pst_soft_retry->uc_software_retry = uc_software_retry;
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(mac_cfg_set_soft_retry_stru);

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_SOFT_RETRY, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_soft_retry::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_open_addr4(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_open_addr4;

    /* ��ȡ�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_open_addr4::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_open_addr4 = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(oal_uint8);
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_open_addr4;
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_OPEN_ADDR4, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_open_addr4::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_open_wmm_test(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_open_wmm;

    /* ��ȡ�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_open_wmm_test::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_open_wmm = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(oal_uint8);
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_open_wmm;
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_OPEN_WMM_TEST, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_open_wmm_test::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_chip_test_open(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_chip_test_open;

    /* ��ȡ�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_chip_test_open::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_chip_test_open = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(oal_uint8);
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_chip_test_open;
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_CHIP_TEST_OPEN, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_chip_test_open::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_coex(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint32 ul_mac_ctrl;
    oal_uint32 ul_rf_ctrl;
    mac_cfg_coex_ctrl_param_stru *pst_coex_ctrl;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_coex::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    ul_mac_ctrl = (oal_uint32)oal_atoi(ac_name);
    pc_param += ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_coex::wal_get_cmd_2nd_arg return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    ul_rf_ctrl = (oal_uint32)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/

    pst_coex_ctrl = (mac_cfg_coex_ctrl_param_stru *)(st_write_msg.auc_value);
    pst_coex_ctrl->ul_mac_ctrl = ul_mac_ctrl;
    pst_coex_ctrl->ul_rf_ctrl = ul_rf_ctrl;

    us_len = OAL_SIZEOF(mac_cfg_coex_ctrl_param_stru);
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_COEX, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_coex::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_dfx(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_tmp;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_dfx::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����logģ����в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        l_tmp = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        l_tmp = 1;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_dfx::the log switch command is error [%d]!}\r\n", ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DFX_SWITCH, OAL_SIZEOF(oal_int32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_tmp; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_dfx::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_test_send_action(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_cfg_rst;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    mac_cfg_send_action_param_stru st_action_param;

    memset_s(&st_action_param, OAL_SIZEOF(mac_cfg_send_action_param_stru),
             0, OAL_SIZEOF(mac_cfg_send_action_param_stru));

    /* ��ȡuc_category�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_send_action::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    st_action_param.uc_category = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /* ��ȡĿ�ĵ�ַ */
    ul_ret = wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_send_action::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_action_param.auc_mac_da, WLAN_MAC_ADDR_LEN);
    pc_param = pc_param + ul_off_set;
    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        &st_action_param, OAL_SIZEOF(mac_cfg_send_action_param_stru))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_test_send_action::memcpy fail!}");
        return OAL_FAIL;
    }

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SEND_ACTION, OAL_SIZEOF(mac_cfg_send_action_param_stru));

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_send_action_param_stru),
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_test_send_action::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_send_pspoll(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SEND_PSPOLL, OAL_SIZEOF(oal_int32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_pspoll::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_send_nulldata(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_tx_nulldata_stru *pst_tx_nulldata;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN] = { 0 };
    oal_uint32 ul_ret;

    pst_tx_nulldata = (mac_cfg_tx_nulldata_stru *)st_write_msg.auc_value;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_nulldata::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    pst_tx_nulldata->l_is_psm = oal_atoi((const oal_int8 *)ac_name);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_nulldata::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    pst_tx_nulldata->l_is_qos = oal_atoi((const oal_int8 *)ac_name);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_nulldata::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;
    pst_tx_nulldata->l_tidno = oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SEND_NULLDATA, OAL_SIZEOF(mac_cfg_tx_nulldata_stru));
    if (EOK != memcpy_s((oal_void *)st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                        (const oal_void *)pst_tx_nulldata,
                        OAL_SIZEOF(mac_cfg_tx_nulldata_stru))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_send_nulldata::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_tx_nulldata_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_send_nulldata::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_clear_all_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;

    /***************************************************************************
                                 ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_CLEAR_ALL_STAT, OAL_SIZEOF(oal_uint32));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_clear_all_stat::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_P2P

OAL_STATIC oal_uint32 wal_parse_ops_param(oal_int8 *pc_param, mac_cfg_p2p_ops_param_stru *pst_p2p_ops_param)
{
    oal_uint32 ul_ret;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;
    oal_int32 l_ct_window;

    /* ������һ���������Ƿ�ʹ��OPS ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_ops_param::wal_get_cmd_one_arg_etc 1 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    if (0 == (oal_strcmp("0", ac_name))) {
        pst_p2p_ops_param->en_ops_ctrl = OAL_FALSE;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        pst_p2p_ops_param->en_ops_ctrl = OAL_TRUE;
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_ops_param::the log switch command[%c] is error!}", (oal_uint8)ac_name[0]);
        OAL_IO_PRINT("{wal_parse_ops_param::the log switch command is error [%6s....]!}\r\n", ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /* �����ڶ���������OPS ����CT Window */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    OAL_IO_PRINT("wal_parse_ops_param:ct window %s\r\n", ac_name);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_ops_param::wal_get_cmd_one_arg_etc 2 return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    l_ct_window = oal_atoi(ac_name);
    if (l_ct_window < 0 || l_ct_window > 255) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_ops_param::ct window out off range [%d]!}\r\n", l_ct_window);
        return OAL_FAIL;
    } else {
        pst_p2p_ops_param->uc_ct_window = (oal_uint8)l_ct_window;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_parse_noa_param(oal_int8 *pc_param, mac_cfg_p2p_noa_param_stru *pst_p2p_noa_param)
{
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;
    oal_int32 l_count;
    oal_uint32 ul_ret;

    /* ������һ��������start_time */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_parse_noa_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    pst_p2p_noa_param->ul_start_time = (oal_uint32)oal_atoi(ac_name);

    /* �����ڶ���������dulration */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_noa_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    pst_p2p_noa_param->ul_duration = (oal_uint32)oal_atoi(ac_name);

    /* ����������������interval */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_noa_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    pst_p2p_noa_param->ul_interval = (oal_uint32)oal_atoi(ac_name);

    /* �������ĸ�������count */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_noa_param::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    l_count = oal_atoi(ac_name);

    if (l_count < 0 || l_count > 255) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_ops_param::ct window out off range [%d]!}\r\n", l_count);
        return OAL_FAIL;
    } else {
        pst_p2p_noa_param->uc_count = (oal_uint8)l_count;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_p2p_ps(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    mac_cfg_p2p_ops_param_stru st_p2p_ops_param;
    mac_cfg_p2p_noa_param_stru st_p2p_noa_param;
    mac_cfg_p2p_stat_param_stru st_p2p_stat_param;
    oal_int32 l_ret = EOK;
    oal_uint32 ul_ret;
    oal_uint16 us_len;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_set_p2p_ps::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param += ul_off_set;

    /* ��Խ������Ĳ�ͬ�����logģ����в�ͬ������ */
    if (0 == (oal_strcmp("ops", ac_name))) {
        /* ����P2P OPS ���ܲ��� */
        ul_ret = wal_parse_ops_param(pc_param, &st_p2p_ops_param);
        if (ul_ret != OAL_SUCC) {
            return ul_ret;
        }
        OAM_INFO_LOG2(0, OAM_SF_CFG, "{wal_hipriv_set_p2p_ps ops::ctrl[%d], ct_window[%d]!}\r\n",
                      st_p2p_ops_param.en_ops_ctrl,
                      st_p2p_ops_param.uc_ct_window);
        us_len = OAL_SIZEOF(mac_cfg_p2p_ops_param_stru);
        WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_P2P_PS_OPS, OAL_SIZEOF(mac_cfg_p2p_ops_param_stru));
        l_ret = memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value),
                         &st_p2p_ops_param, OAL_SIZEOF(mac_cfg_p2p_ops_param_stru));

    } else if (0 == (oal_strcmp("noa", ac_name))) {
        /* ����P2P NOA ���ܲ��� */
        ul_ret = wal_parse_noa_param(pc_param, &st_p2p_noa_param);
        if (ul_ret != OAL_SUCC) {
            return ul_ret;
        }
        OAM_INFO_LOG4(0, OAM_SF_CFG, "{wal_hipriv_set_p2p_ps noa::start_time[%d], duration[%d], interval[%d], count[%d]!}\r\n",
                      st_p2p_noa_param.ul_start_time,
                      st_p2p_noa_param.ul_duration,
                      st_p2p_noa_param.ul_interval,
                      st_p2p_noa_param.uc_count);
        us_len = OAL_SIZEOF(mac_cfg_p2p_noa_param_stru);
        WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_P2P_PS_NOA, OAL_SIZEOF(mac_cfg_p2p_noa_param_stru));
        l_ret = memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), &st_p2p_noa_param, OAL_SIZEOF(mac_cfg_p2p_noa_param_stru));

    } else if (0 == (oal_strcmp("statistics", ac_name))) {
        /* ��ȡP2P����ͳ�� */
        /* �����������鿴����ͳ�� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret != OAL_SUCC) {
            OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_parse_ops_param::wal_get_cmd_one_arg_etc 1 return err_code [%d]!}\r\n", ul_ret);
            return ul_ret;
        }
        pc_param += ul_off_set;
        if (0 == (oal_strcmp("0", ac_name))) {
            st_p2p_stat_param.uc_p2p_statistics_ctrl = 0;
        } else if (0 == (oal_strcmp("1", ac_name))) {
            st_p2p_stat_param.uc_p2p_statistics_ctrl = 1;
        } else {
            OAM_WARNING_LOG0(0, OAM_SF_CFG, "{wal_hipriv_set_p2p_ps statistics::wrong parm\r\n}");
            return OAL_FAIL;
        }

        us_len = OAL_SIZEOF(mac_cfg_p2p_stat_param_stru);
        WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_P2P_PS_STAT, us_len);
        l_ret = memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), &st_p2p_stat_param, us_len);

        OAM_INFO_LOG2(0, OAM_SF_CFG, "{wal_hipriv_set_p2p_ps statistics::ctrl[%d], len:%d!}\r\n",
                      st_p2p_stat_param.uc_p2p_statistics_ctrl, us_len);
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_p2p_ps::the log switch command is error [%d]!}\r\n", ac_name);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    if (l_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_p2p_ps::memcpy fail!}");
        return OAL_FAIL;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_p2p_ps::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif
#endif
#ifdef _PRE_WLAN_FEATURE_GREEN_AP

OAL_STATIC oal_uint32 wal_hipriv_set_gap_free_ratio(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    mac_cfg_free_ratio_set_stru *pst_set_para;
    oal_uint32 ul_off_set = 0;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    /* hipriv "Hisilicon0 set_free_ratio 0|1 val" */

    pst_set_para = (mac_cfg_free_ratio_set_stru *)(st_write_msg.auc_value);
    /* ��������������� */
    memset_s(ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, 0, WAL_HIPRIV_CMD_NAME_MAX_LEN);

    /* ��ȡ����ģʽ */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_set_para->en_mode = (mac_lpm_soc_set_enum_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡ�������ǹر� */
    wal_get_cmd_one_arg_etc((oal_int8 *)pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    pst_set_para->uc_th_value = (oal_uint8)oal_atoi(ac_name);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_FREE_RATIO_SET, OAL_SIZEOF(mac_cfg_free_ratio_set_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_free_ratio_set_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_lpm_soc_mode::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#if (_PRE_WLAN_FEATURE_PMF != _PRE_PMF_NOT_SUPPORT)

OAL_STATIC oal_uint32 wal_hipriv_enable_pmf(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;

    oal_int32 l_cfg_rst;
    oal_uint16 us_len;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set = 0;
    oal_uint8 uc_chip_test_open;

    /* ��ȡ�趨��ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_enable_pmf::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_chip_test_open = (oal_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    us_len = OAL_SIZEOF(oal_uint8);
    *(oal_uint8 *)(st_write_msg.auc_value) = uc_chip_test_open;
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PMF_ENABLE, us_len);

    l_cfg_rst = wal_send_cfg_event_etc(pst_net_dev,
                                       WAL_MSG_TYPE_WRITE,
                                       WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                       (oal_uint8 *)&st_write_msg,
                                       OAL_FALSE,
                                       OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_cfg_rst != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_enable_pmf::wal_send_cfg_event_etc return err_code [%d]!}\r\n", l_cfg_rst);
        return (oal_uint32)l_cfg_rst;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_DFT_REG

OAL_STATIC oal_uint32 wal_hipriv_dump_reg(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oam_reg_type_enum_uint8 en_reg_type = 0;
    oal_uint32 ul_reg_subtype;
    oal_uint8 uc_flag;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    if ((0 != oal_strcmp(ac_name, "soc")) &&
        (0 != oal_strcmp(ac_name, "mac")) &&
        (0 != oal_strcmp(ac_name, "phy")) &&
        (0 != oal_strcmp(ac_name, "abb")) &&
        (0 != oal_strcmp(ac_name, "rf"))) {
        return OAL_FAIL;
    }

    if (0 == oal_strcmp(ac_name, "soc")) {
        en_reg_type = OAM_REG_SOC;
    }
    if (0 == oal_strcmp(ac_name, "mac")) {
        en_reg_type = OAM_REG_MAC;
    }
    if (0 == oal_strcmp(ac_name, "phy")) {
        en_reg_type = OAM_REG_PHY;
    }
    if (0 == oal_strcmp(ac_name, "abb")) {
        en_reg_type = OAM_REG_ABB;
    }
    if (0 == oal_strcmp(ac_name, "rf")) {
        en_reg_type = OAM_REG_RF;
    }

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;
    ul_reg_subtype = (oal_uint32)oal_atoi(ac_name);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_flag = (oal_uint8)oal_atoi(ac_name);

    oam_reg_set_flag_etc(en_reg_type, ul_reg_subtype, uc_flag);
    OAL_IO_PRINT("dump reg: regtype %u, subtype %u,uc flag %u\n",
                 en_reg_type,
                 ul_reg_subtype,
                 uc_flag);
    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_dump_reg_evt(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oam_reg_evt_enum_uint32 en_evt_type = 0;
    oal_uint32 ul_tick;
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    /* ��ȡ�¼����� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_evt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    if ((0 != oal_strcmp(ac_name, "tbtt"))
        && (0 != oal_strcmp(ac_name, "rx"))
        && (0 != oal_strcmp(ac_name, "tx"))
        && (0 != oal_strcmp(ac_name, "prd"))
        && (0 != oal_strcmp(ac_name, "usr"))
        && (0 != oal_strcmp(ac_name, "all"))
        && (0 != oal_strcmp(ac_name, "err"))) {
        return OAL_FAIL;
    }

    if (0 == oal_strcmp(ac_name, "tbtt")) {
        en_evt_type = OAM_REG_EVT_TBTT;
    }
    if (0 == oal_strcmp(ac_name, "rx")) {
        en_evt_type = OAM_REG_EVT_RX;
    }
    if (0 == oal_strcmp(ac_name, "tx")) {
        en_evt_type = OAM_REG_EVT_TX;
    }
    if (0 == oal_strcmp(ac_name, "prd")) {
        en_evt_type = OAM_REG_EVT_PRD;
    }
    if (0 == oal_strcmp(ac_name, "usr")) {
        en_evt_type = OAM_REG_EVT_USR;
    }
    if (0 == oal_strcmp(ac_name, "err")) {
        en_evt_type = OAM_REG_EVT_ERR;
    }
    if (0 == oal_strcmp(ac_name, "all")) {
        en_evt_type = OAM_REG_EVT_BUTT;
    }

    /* ��ȡtick */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_evt::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    ul_tick = (oal_uint32)oal_atoi(ac_name);

    oam_reg_set_evt_etc(en_evt_type, ul_tick);

    if (en_evt_type != OAM_REG_EVT_USR) {
        return OAL_SUCC;
    }

    /***************************************************************************
                             ���¼���wal�㴥������ˢ�º��ϱ�
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DUMP_REG, 0);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_evt::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dump_reg_addr(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oam_reg_type_enum_uint8 en_reg_type = 0;
    oal_uint8 uc_flag;
    oal_uint32 ul_addr_start;
    oal_uint32 ul_addr_end;
    oal_int8 *pc_end;
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_addr::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    if ((0 != oal_strcmp(ac_name, "soc"))
        && (0 != oal_strcmp(ac_name, "mac"))
        && (0 != oal_strcmp(ac_name, "phy"))
        && (0 != oal_strcmp(ac_name, "abb"))
        && (0 != oal_strcmp(ac_name, "rf"))) {
        return OAL_FAIL;
    }

    if (0 == oal_strcmp(ac_name, "soc")) {
        en_reg_type = OAM_REG_SOC;
    }
    if (0 == oal_strcmp(ac_name, "mac")) {
        en_reg_type = OAM_REG_MAC;
    }
    if (0 == oal_strcmp(ac_name, "phy")) {
        en_reg_type = OAM_REG_PHY;
    }
    if (0 == oal_strcmp(ac_name, "abb")) {
        en_reg_type = OAM_REG_ABB;
    }
    if (0 == oal_strcmp(ac_name, "rf")) {
        en_reg_type = OAM_REG_RF;
    }

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_addr::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;
    ul_addr_start = (oal_uint32)oal_strtol(ac_name, &pc_end, 16);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_addr::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    pc_param = pc_param + ul_off_set;
    ul_addr_end = (oal_uint32)oal_strtol(ac_name, &pc_end, 16);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_addr::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    uc_flag = (oal_uint8)oal_atoi(ac_name);

    if ((ul_addr_start % 4)
        || (ul_addr_end % 4)
        || (ul_addr_start > ul_addr_end)) {
        OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_dump_reg_addr::start %u, end %u Err [%d] [%d]!}\r\n", ul_addr_start, ul_addr_end);
        return OAL_FAIL;
    }
    oam_reg_set_flag_addr_etc(en_reg_type, ul_addr_start, ul_addr_end, uc_flag);
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_dump_reg_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oam_reg_info_etc();
    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_QOS_ENHANCE
/*
 * �� �� ��  : wal_hipriv_qos_enhance_on
 * ��������  : ������ر�qos enhance����
 */
OAL_STATIC oal_uint32 wal_hipriv_qos_enhance_on(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_qos_enhance_on;
    oal_int32 l_idx = 0;
    mac_cfg_qos_enhance_on_param_stru *pst_qos_enhance_on_param;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_qos_enhance_on::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��������Ϸ��Լ�� */
    while (ac_name[l_idx] != '\0') {
        if (isdigit(ac_name[l_idx])) {
            l_idx++;
            continue;
        } else {
            l_idx++;
            OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_qos_enhance_on::input illegal!}");
            return OAL_ERR_CODE_INVALID_CONFIG;
        }
    }

    uc_qos_enhance_on = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_QOS_ENHANCE_ON, OAL_SIZEOF(mac_cfg_qos_enhance_on_param_stru));

    /* ��������������� */
    pst_qos_enhance_on_param = (mac_cfg_qos_enhance_on_param_stru *)(st_write_msg.auc_value);
    pst_qos_enhance_on_param->uc_qos_enhance_on = uc_qos_enhance_on;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_qos_enhance_on_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_QOS, "{wal_hipriv_qos_enhance_on::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_PROFLING_MIPS

OAL_STATIC oal_uint32 wal_hipriv_set_mips(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_mips_type;
    oal_int32 l_switch;
    wal_msg_write_stru st_write_msg;
    oal_mips_type_param_stru *pst_mips_type_param;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }
    l_mips_type = oal_atoi((const oal_int8 *)ac_name);

    ul_ret = wal_get_cmd_one_arg_etc(pc_param + ul_off_set, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }
    l_switch = oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_MIPS, OAL_SIZEOF(oal_mips_type_param_stru));
    pst_mips_type_param = (oal_mips_type_param_stru *)st_write_msg.auc_value;
    pst_mips_type_param->l_mips_type = l_mips_type;
    pst_mips_type_param->l_switch = l_switch;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_mips_type_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_mips::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_show_mips(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_mips_type;
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }
    l_mips_type = oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SHOW_MIPS, OAL_SIZEOF(oal_uint32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_mips_type;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_mips::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_RR_PERFORMENCE_DEBUG

oal_uint32 wal_hipriv_show_rr_time_info(oal_net_device_stru *net_dev, oal_int8 *param)
{
    oal_uint32 ret;
    oal_int32 lret;
    oal_uint32 off_set;
    oal_switch_enum_uint8 rr_switch;
    oal_int8 name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru write_msg;

    ret = wal_get_cmd_one_arg_etc(param, name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &off_set);
    if (ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_CFG, "wal_hipriv_show_rr_time_info return err_code: %d", ret);
        return ret;
    }
    rr_switch = (oal_switch_enum_uint8)oal_atoi(name);
    if (rr_switch != OAL_SWITCH_OFF && rr_switch != OAL_SWITCH_ON) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_show_rr_time_info::command param is error!}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&write_msg, WLAN_CFGID_ENABLE_RR_TIMEINFO, OAL_SIZEOF(oal_switch_enum_uint8));
    *(oal_switch_enum_uint8 *)(write_msg.auc_value) = rr_switch;

    lret = wal_send_cfg_event_etc(net_dev,
                                  WAL_MSG_TYPE_WRITE,
                                  WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_switch_enum_uint8),
                                  (oal_uint8 *)&write_msg,
                                  OAL_FALSE,
                                  OAL_PTR_NULL);

    if (OAL_UNLIKELY(lret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_show_rr_time_info::return err code[%d]!}\r\n", lret);
        return (oal_uint32)lret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_ARP_OFFLOAD

oal_uint32 wal_hipriv_arp_offload_enable(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_switch_enum_uint8 en_switch;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_ERROR_LOG1(0, OAM_SF_CFG, "wal_hipriv_arp_offload_enable return err_code: %d", ul_ret);
        return ul_ret;
    }
    en_switch = (oal_switch_enum_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ENABLE_ARP_OFFLOAD, OAL_SIZEOF(oal_switch_enum_uint8));
    *(oal_switch_enum_uint8 *)(st_write_msg.auc_value) = en_switch;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_switch_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_CFG, "{wal_hipriv_arp_offload_enable::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


oal_uint32 wal_hipriv_show_arpoffload_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_show_ip_addr;
    oal_uint8 uc_show_arpoffload_info;
    mac_cfg_arpoffload_info_stru *pst_arpoffload_info;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_arpoffload_info::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_show_ip_addr = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_show_arpoffload_info::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_show_arpoffload_info = (oal_uint8)oal_atoi(ac_name);
    pc_param = pc_param + ul_off_set;

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SHOW_ARPOFFLOAD_INFO, OAL_SIZEOF(mac_cfg_arpoffload_info_stru));

    /* ��������������� */
    pst_arpoffload_info = (mac_cfg_arpoffload_info_stru *)(st_write_msg.auc_value);
    pst_arpoffload_info->uc_show_ip_addr = uc_show_ip_addr;
    pst_arpoffload_info->uc_show_arpoffload_info = uc_show_arpoffload_info;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_arpoffload_info_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_PWR, "{wal_hipriv_show_arpoffload_info::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }
    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_ROAM
OAL_STATIC oal_uint32 wal_hipriv_roam_homenetwork_thr(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int8 c_trigger_rssi; /* ��ͥ������ѡ����rssi */
    oal_int8 c_5g_rssi_th;   /* ��ͥ����5g�������� */
    oal_int8 c_5g_hold_th;   /* ��ͥ����5g������������ */

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "roam_homenetwork_thr return err_code [%d]", ul_ret);
        return ul_ret;
    }
    c_trigger_rssi = (oal_bool_enum_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "roam_homenetwork_thr return err_code [%d]", ul_ret);
        return ul_ret;
    }
    c_5g_rssi_th = (oal_bool_enum_uint8)oal_atoi(ac_name);
    pc_param += ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "roam_homenetwork_thr return err_code [%d]", ul_ret);
        return ul_ret;
    }
    c_5g_hold_th = (oal_bool_enum_uint8)oal_atoi(ac_name);
    hmac_roam_home_network_set_thr_etc(c_trigger_rssi, c_5g_rssi_th, c_5g_hold_th);
    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_roam_enable(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_bool_enum_uint8 en_enable;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "roam enable type return err_code [%d]", ul_ret);
        return ul_ret;
    }
    en_enable = (oal_bool_enum_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ROAM_ENABLE, OAL_SIZEOF(oal_uint32));
    *((oal_bool_enum_uint8 *)(st_write_msg.auc_value)) = en_enable;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_bool_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_roam_enable::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_roam_org(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_uint8 uc_org;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "roam org type return err_code[%d]", ul_ret);
        return ul_ret;
    }
    uc_org = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ROAM_ORG, OAL_SIZEOF(oal_uint32));
    *((oal_uint8 *)(st_write_msg.auc_value)) = uc_org;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{wal_hipriv_roam_org::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_roam_band(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_uint8 uc_band;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru st_write_msg;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "roam band type return err_code[%d]", ul_ret);
        return ul_ret;
    }
    uc_band = (oal_uint8)oal_atoi(ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ROAM_BAND, OAL_SIZEOF(oal_uint32));
    *((oal_uint8 *)(st_write_msg.auc_value)) = uc_band;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_bool_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{wal_hipriv_roam_band::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

OAL_STATIC oal_uint32 wal_hipriv_roam_start(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_int32 l_ret;
    oal_uint32 ul_ret;
    oal_uint8 uc_scan_type = ROAM_SCAN_CHANNEL_ORG_BUTT;
    oal_bool_enum_uint8 en_current_bss_ignore = OAL_TRUE;
    oal_uint8 auc_bssid[OAL_MAC_ADDR_LEN] = { 0 };
    wal_msg_write_stru st_write_msg;
    mac_cfg_set_roam_start_stru *pst_roam_start;

    oal_uint32 ul_off_set;
    oal_uint8 uc_param;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret == OAL_ERR_CODE_PTR_NULL) {
        /* Ĭ��ɨ��+���� */
        // uc_scan_type = ROAM_SCAN_CHANNEL_ORG_BUTT;
        // en_current_bss_ignore = OAL_TRUE;
        memset_s(auc_bssid, sizeof(auc_bssid), 0, sizeof(auc_bssid));
    } else if (ul_ret == OAL_SUCC) {
        /* ָ������ʱ���Ƿ����ɨ����� */
        uc_scan_type = (oal_uint8)oal_atoi(ac_name);

        pc_param += ul_off_set;
        /* ��ȡ�Ƿ���Թ������Լ��Ĳ��� */
        ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
        if (ul_ret == OAL_ERR_CODE_PTR_NULL) {
            /* Ĭ�ϲ��ܹ������Լ� */
            // en_current_bss_ignore = OAL_TRUE;
            memset_s(auc_bssid, sizeof(auc_bssid), 0, sizeof(auc_bssid));
        } else if (ul_ret == OAL_SUCC) {
            uc_param = (oal_uint8)oal_atoi(ac_name);
            en_current_bss_ignore = (uc_param == 0) ? OAL_TRUE : OAL_FALSE; /* 0/TRUE���߲���ȱʧ��ʾ���ε�����AP, 1/FALSE��ʾ���ε��Լ� */

            /* ��ȡBSSID */
            pc_param = pc_param + ul_off_set;
            ul_ret = wal_hipriv_get_mac_addr_etc(pc_param, auc_bssid, &ul_off_set);
            if (ul_ret == OAL_ERR_CODE_PTR_NULL) {
                /* Ĭ�ϲ�ָ��BSSID */
                memset_s(auc_bssid, sizeof(auc_bssid), 0, sizeof(auc_bssid));
            } else if (ul_ret != OAL_SUCC) {
                OAM_WARNING_LOG0(0, OAM_SF_ROAM, "{wal_hipriv_roam_start::bssid failed!}\r\n");
                return ul_ret;
            }
        } else {
            OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{wal_hipriv_roam_start::roam_start return err_code [%d]!}\r\n", ul_ret);
            return (oal_uint32)ul_ret;
        }
    } else {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{wal_hipriv_roam_start::roam_start cfg_cmd error[%d]}", ul_ret);
        return ul_ret;
    }

    OAM_WARNING_LOG4(0, OAM_SF_ROAM, "{wal_hipriv_roam_start::roam_start uc_scan_type[%d], en_current_bss_ignore[%d], target bssid[XXXX:%02X:%02X]}",
                     uc_scan_type, en_current_bss_ignore, auc_bssid[4], auc_bssid[5]);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    memset_s(&st_write_msg, OAL_SIZEOF(wal_msg_write_stru), 0, OAL_SIZEOF(wal_msg_write_stru));
    pst_roam_start = (mac_cfg_set_roam_start_stru *)(st_write_msg.auc_value);
    pst_roam_start->uc_scan_type = uc_scan_type;
    pst_roam_start->en_current_bss_ignore = en_current_bss_ignore;
    if (EOK != memcpy_s(pst_roam_start->auc_bssid, sizeof(pst_roam_start->auc_bssid),
                        auc_bssid, sizeof(auc_bssid))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_roam_start::memcpy fail!}");
        return OAL_FAIL;
    }

    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ROAM_START, OAL_SIZEOF(mac_cfg_set_roam_start_stru));

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_set_roam_start_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{wal_hipriv_roam_start::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_roam_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_int32 l_ret;
    oal_bool_enum_uint8 en_enable;
    wal_msg_write_stru st_write_msg;

    en_enable = 1;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ROAM_INFO, OAL_SIZEOF(oal_uint32));
    *((oal_bool_enum_uint8 *)(st_write_msg.auc_value)) = en_enable;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_bool_enum_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ROAM, "{wal_hipriv_roam_enable::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif  // _PRE_WLAN_FEATURE_ROAM

#ifdef _PRE_WLAN_FIT_BASED_REALTIME_CALI

oal_uint32 wal_hipriv_dyn_cali_cfg(oal_net_device_stru *pst_net_dev, oal_int8 *puc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    mac_ioctl_dyn_cali_param_stru *pst_dyn_cali_param = OAL_PTR_NULL;
    wal_ioctl_dyn_cali_stru st_cyn_cali_cfg;
    oal_uint32 ul_ret;
    oal_uint8 uc_map_index = 0;
    oal_int32 l_send_event_ret;

    pst_dyn_cali_param = (mac_ioctl_dyn_cali_param_stru *)(st_write_msg.auc_value);
    memset_s(pst_dyn_cali_param, OAL_SIZEOF(mac_ioctl_dyn_cali_param_stru),
             0, OAL_SIZEOF(mac_ioctl_dyn_cali_param_stru));

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_net_dev, puc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_dyn_cali_cfg::pst_cfg_net_dev or puc_param null ptr error }\r\n");
        return OAL_ERR_CODE_PTR_NULL;
    }

    ul_ret = wal_get_cmd_one_arg_etc(puc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dyn_cali_cfg::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }
    /* Ѱ��ƥ������� */
    st_cyn_cali_cfg = g_ast_dyn_cali_cfg_map[0];
    while (st_cyn_cali_cfg.pc_name != OAL_PTR_NULL) {
        if (0 == oal_strcmp(st_cyn_cali_cfg.pc_name, ac_name)) {
            break;
        }
        st_cyn_cali_cfg = g_ast_dyn_cali_cfg_map[++uc_map_index];
    }

    /* û���ҵ���Ӧ������򱨴� */
    if (st_cyn_cali_cfg.pc_name == OAL_PTR_NULL) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_dyn_cali_cfg::invalid alg_cfg command!}\r\n");
        return OAL_FAIL;
    }

    /* ��¼�����Ӧ��ö��ֵ */
    pst_dyn_cali_param->en_dyn_cali_cfg = g_ast_dyn_cali_cfg_map[uc_map_index].en_dyn_cali_cfg;
    /* ��ȡ��������ֵ */
    ul_ret = wal_get_cmd_one_arg_etc(puc_param + ul_off_set, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret == OAL_SUCC) {
        /* ��¼��������ֵ */
        pst_dyn_cali_param->us_value = (oal_uint16)oal_atoi(ac_name);
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_dyn_cali_cfg::wal_get_cmd_one_arg_etc [%d]!}\r\n", pst_dyn_cali_param->us_value);
    }

    /***************************************************************************
                             ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_DYN_CALI_CFG, OAL_SIZEOF(oal_uint32));

    l_send_event_ret = wal_send_cfg_event_etc(pst_net_dev,
                                              WAL_MSG_TYPE_WRITE,
                                              WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                              (oal_uint8 *)&st_write_msg,
                                              OAL_FALSE,
                                              OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_send_event_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_cyn_cali_set_dscr_interval::wal_send_cfg_event_etc return err code [%d]!}\r\n", l_send_event_ret);
        return (oal_uint32)l_send_event_ret;
    }
    return OAL_SUCC;
}

#endif

#ifdef _PRE_WLAN_FEATURE_PACKET_CAPTURE

OAL_STATIC oal_uint32 wal_hipriv_packet_capture(oal_net_device_stru *pst_cfg_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_capture_switch;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_ret;
    oal_uint32 ul_ret;

    /*
       ץ�����ܵĿ��ص�����: hipriv "Hisilicon0 packet_capture 0 | 1 | 2 | 3 | 4"
       �˴���������"0 ~ 4"����ac_name
    */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_packet_capture::wal_get_cmd_one_arg_etc return err_code %d!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ��Խ������Ĳ�ͬ�����ץ��ģ����в�ͬ������ */
    if (0 == (oal_strcmp("0", ac_name))) {
        ul_capture_switch = 0;
    } else if (0 == (oal_strcmp("1", ac_name))) {
        ul_capture_switch = 1;
    } else if (0 == (oal_strcmp("2", ac_name))) {
        ul_capture_switch = 2;
    } else if (0 == (oal_strcmp("3", ac_name))) {
        ul_capture_switch = 3;
    } else if (0 == (oal_strcmp("4", ac_name))) {
        ul_capture_switch = 4;
    } else {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_packet_capture::command param is error!}\r\n");
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                              ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_PACKET_CAPTURE_SWITCH, OAL_SIZEOF(oal_int32));
    *((oal_uint32 *)(st_write_msg.auc_value)) = ul_capture_switch; /* ��������������� */

    l_ret = wal_send_cfg_event_etc(pst_cfg_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_int32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_wifi_enable::return err code %d!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_11K_STAT

OAL_STATIC oal_uint32 wal_hipriv_query_stat_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_int32 l_ret;
    oal_uint16 us_len;

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), pc_param, OAL_STRLEN(pc_param))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_query_stat_info::memcpy fail!}");
        return;
    }

    st_write_msg.auc_value[OAL_STRLEN(pc_param)] = '\0';
    us_len = (oal_uint16)(OAL_STRLEN(pc_param) + 1);
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_QUERY_STAT_INFO, us_len);

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_query_stat_info::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }
    return OAL_SUCC;
}

#endif



#define WAL_HIPRIV_OWE_MAX 8
oal_uint32 wal_hipriv_set_owe(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint8 uc_owe_group;
    oal_int8 ac_arg[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;
    wal_msg_stru *pst_rsp_msg = OAL_PTR_NULL;

    /* ��ȡ�ַ��� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_arg, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_owe::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    /* ����Ҫ���õ�ֵ */
    uc_owe_group = (oal_uint8)oal_atoi(ac_arg);
    if (uc_owe_group >= WAL_HIPRIV_OWE_MAX) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_owe::input value out of range [%d]!}\r\n", uc_owe_group);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SET_OWE, OAL_SIZEOF(oal_uint8));

    /* ��������������������� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), &uc_owe_group, OAL_SIZEOF(oal_uint8))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_owe::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint8),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_TRUE,
                                   &pst_rsp_msg);
    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_owe::return err code [%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    /* ��ȡ���صĴ����� */
    ul_ret = wal_check_and_release_msg_resp_etc(pst_rsp_msg);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_DFR, "{wal_hipriv_set_owe fail, err code[%u]!}\r\n", ul_ret);
        return ul_ret;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_TWT

OAL_STATIC oal_uint32 wal_hipriv_twt_setup_req(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_twt_setup_req_param_stru *pst_twt_setup_req_param;
    mac_cfg_twt_setup_req_param_stru st_twt_setup_req_param; /* ��ʱ�����ȡ��twt setup req����Ϣ */
    oal_uint32 ul_get_addr_idx;

    /*
        ����AMPDU�رյ���������:
        hipriv "Hisilicon0 twt_setup_req xx xx xx xx xx xx(mac��ַ)  setup_cmd flow_type flow_ID start_time_offset interval_exponent wake_duration interval_mantissa"
    */
    OAM_WARNING_LOG0(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req\r\n");

    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    memset_s((oal_uint8 *)&st_twt_setup_req_param, OAL_SIZEOF(st_twt_setup_req_param),
             0, OAL_SIZEOF(st_twt_setup_req_param));
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_twt_setup_req_param.auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_setup_req_param.uc_twt_setup_cmd = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_setup_req_param.uc_twt_flow_type = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_setup_req_param.uc_twt_flow_id = (oal_uint8)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_setup_req_param.ul_twt_start_time_offset = (oal_uint32)oal_atoi(ac_name);

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡinterval_exponent */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_setup_req_param.ul_twt_exponent = (oal_uint32)oal_atoi(ac_name);
    if (st_twt_setup_req_param.ul_twt_exponent > 0x1F) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::the ul_twt_exponent is not correct! ul_twt_exponent is[%d]!}\r\n", st_twt_setup_req_param.ul_twt_exponent);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pc_param = pc_param + ul_off_set;

    /* ��ȡwake_duration */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_setup_req_param.ul_twt_duration = (oal_uint32)oal_atoi(ac_name);
    if (st_twt_setup_req_param.ul_twt_duration > 0xFF) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::the ul_twt_duration is not correct! ul_twt_duration is[%d]!}\r\n", st_twt_setup_req_param.ul_twt_duration);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    pc_param = pc_param + ul_off_set;

    /* ��ȡinterval_mantissa */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_setup_req_param.ul_intrval_mantissa = (oal_uint16)oal_atoi(ac_name);
    if (st_twt_setup_req_param.ul_intrval_mantissa > 0xFFFF) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::the ul_intrval_mantissa is not correct! ul_intrval_mantissa is[%d]!}\r\n", st_twt_setup_req_param.ul_intrval_mantissa);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }
    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TWT_SETUP_REQ, OAL_SIZEOF(mac_cfg_twt_setup_req_param_stru));

    /* ��������������� */
    pst_twt_setup_req_param = (mac_cfg_twt_setup_req_param_stru *)(st_write_msg.auc_value);
    for (ul_get_addr_idx = 0; ul_get_addr_idx < WLAN_MAC_ADDR_LEN; ul_get_addr_idx++) {
        pst_twt_setup_req_param->auc_mac_addr[ul_get_addr_idx] = st_twt_setup_req_param.auc_mac_addr[ul_get_addr_idx];
    }

    pst_twt_setup_req_param->uc_twt_setup_cmd = st_twt_setup_req_param.uc_twt_setup_cmd;
    pst_twt_setup_req_param->uc_twt_flow_type = st_twt_setup_req_param.uc_twt_flow_type;
    pst_twt_setup_req_param->uc_twt_flow_id = st_twt_setup_req_param.uc_twt_flow_id;
    pst_twt_setup_req_param->ul_twt_start_time_offset = st_twt_setup_req_param.ul_twt_start_time_offset;
    pst_twt_setup_req_param->ul_twt_exponent = st_twt_setup_req_param.ul_twt_exponent;
    pst_twt_setup_req_param->ul_twt_duration = st_twt_setup_req_param.ul_twt_duration;
    pst_twt_setup_req_param->ul_intrval_mantissa = st_twt_setup_req_param.ul_intrval_mantissa;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_twt_setup_req_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_setup_req::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_twt_teardown_req(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    wal_msg_write_stru st_write_msg;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    mac_cfg_twt_teardown_req_param_stru *pst_twt_teardown_req_param;
    mac_cfg_twt_teardown_req_param_stru st_twt_teardown_req_param; /* ��ʱ�����ȡ��twt teardown req����Ϣ */
    oal_uint32 ul_get_addr_idx;

    /*
     * ����AMPDU�رյ���������:
     * hipriv "Hisilicon0 twt_teardown_req xx xx xx xx xx xx(mac��ַ) flwo_id"
     */
    /* ��ȡmac��ַ */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_teardown_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    memset_s((oal_uint8 *)&st_twt_teardown_req_param, OAL_SIZEOF(st_twt_teardown_req_param),
             0, OAL_SIZEOF(st_twt_teardown_req_param));
    oal_strtoaddr(ac_name, OAL_SIZEOF(ac_name), st_twt_teardown_req_param.auc_mac_addr, WLAN_MAC_ADDR_LEN);
    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡflwo_id */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_teardown_req::wal_get_cmd_one_arg_etc return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_twt_teardown_req_param.uc_twt_flow_id = (oal_uint8)oal_atoi(ac_name);
    if (st_twt_teardown_req_param.uc_twt_flow_id > 0x07) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_teardown_req::the twt teardown req command is error!uc_twt_flow_id is [%d]!}\r\n", st_twt_teardown_req_param.uc_twt_flow_id);
        return OAL_ERR_CODE_INVALID_CONFIG;
    }

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_TWT_TEARDOWN_REQ, OAL_SIZEOF(mac_cfg_twt_teardown_req_param_stru));

    /* ��������������� */
    pst_twt_teardown_req_param = (mac_cfg_twt_teardown_req_param_stru *)(st_write_msg.auc_value);
    for (ul_get_addr_idx = 0; ul_get_addr_idx < WLAN_MAC_ADDR_LEN; ul_get_addr_idx++) {
        pst_twt_teardown_req_param->auc_mac_addr[ul_get_addr_idx] = st_twt_teardown_req_param.auc_mac_addr[ul_get_addr_idx];
    }

    pst_twt_teardown_req_param->uc_twt_flow_id = st_twt_teardown_req_param.uc_twt_flow_id;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(mac_cfg_twt_teardown_req_param_stru),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{wal_hipriv_twt_teardown_req::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}

#endif


oal_uint32 wal_hipriv_fem_lowpower(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_int32 l_param;
    wal_msg_write_stru st_write_msg;

    /* ��ȡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        return ul_ret;
    }

    l_param = oal_atoi((const oal_int8 *)ac_name);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_SPEC_FEMLP_EN, OAL_SIZEOF(oal_uint32));
    *((oal_int32 *)(st_write_msg.auc_value)) = l_param;

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(oal_uint32),
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_fem_lowpower::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

    return OAL_SUCC;
}


OAL_STATIC oal_uint32 wal_hipriv_set_icmp_filter(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
#ifndef WIN32
    oal_uint32 ul_ret;
    oal_int32 l_ret;
    oal_uint16 us_len;
    oal_uint32 ul_off_set;
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    wal_msg_write_stru st_write_msg;
    mac_assigned_filter_cmd_stru st_assigned_filter_cmd;

    /* ׼���������� */
    us_len = OAL_SIZEOF(st_assigned_filter_cmd);
    memset_s((oal_uint8 *)&st_assigned_filter_cmd, us_len, 0, us_len);

    /* ��ȡfilter id */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_icmp_filter::first para return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    if (MAC_ICMP_FILTER != (oal_uint8)oal_atoi(ac_name)) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_icmp_filter::icmp filter id not match.}");
        return OAL_FAIL;
    }

    st_assigned_filter_cmd.en_filter_id = MAC_ICMP_FILTER;

    /* ƫ�ƣ�ȡ��һ������ */
    pc_param = pc_param + ul_off_set;

    /* ��ȡ���� */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_icmp_filter::second para return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    st_assigned_filter_cmd.en_enable = ((oal_uint8)oal_atoi(ac_name) > 0) ? OAL_TRUE : OAL_FALSE;

    OAM_WARNING_LOG2(0, OAM_SF_ANY, "{wal_hipriv_set_icmp_filter::filter id [%d] on/off[%d].}",
                     st_assigned_filter_cmd.en_filter_id, st_assigned_filter_cmd.en_enable);

    /***************************************************************************
                                ���¼���wal�㴦��
    ***************************************************************************/
    WAL_WRITE_MSG_HDR_INIT(&st_write_msg, WLAN_CFGID_ASSIGNED_FILTER, us_len);
    /* �������netbuf�׵�ַ��д��msg��Ϣ���� */
    if (EOK != memcpy_s(st_write_msg.auc_value, sizeof(st_write_msg.auc_value), (oal_uint8 *)&st_assigned_filter_cmd, us_len)) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{wal_hipriv_set_icmp_filter::memcpy fail!}");
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev,
                                   WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + us_len,
                                   (oal_uint8 *)&st_write_msg,
                                   OAL_FALSE,
                                   OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_set_icmp_filter::return err code[%d]!}\r\n", l_ret);
        return (oal_uint32)l_ret;
    }

#endif
    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_PSM_FLT_STAT
OAL_STATIC oal_uint32 wal_hipriv_psm_flt_stat(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    oal_int8 ac_name[WAL_HIPRIV_CMD_NAME_MAX_LEN];
    oal_uint32 ul_off_set;
    oal_uint32 ul_ret;
    oal_uint8 uc_query_type;
    /* ��ȡquery type */
    ul_ret = wal_get_cmd_one_arg_etc(pc_param, ac_name, WAL_HIPRIV_CMD_NAME_MAX_LEN, &ul_off_set);
    if (ul_ret != OAL_SUCC) {
        OAM_WARNING_LOG1(0, OAM_SF_ANY, "{wal_hipriv_psm_flt_stat::first para return err_code [%d]!}\r\n", ul_ret);
        return ul_ret;
    }

    uc_query_type = (oal_uint8)oal_atoi((const oal_int8 *)ac_name);
    wal_ioctl_get_psm_stat(pst_net_dev, uc_query_type, OAL_PTR_NULL);
    return OAL_SUCC;
}
#endif

#ifdef _PRE_WLAN_FEATURE_GET_STATION_INFO_EXT
OAL_STATIC oal_void wal_hipriv_printf_station_info(hmac_vap_stru *pst_hmac_vap)
{
#ifdef CONFIG_HW_GET_EXT_SIG_ULDELAY
    OAM_WARNING_LOG4(0, OAM_SF_ANY,
        "{wal_hipriv_get_station_info::tid delay[%d]ms, tx drop[%d], tx pkts[%d], tx bytes[%d]}",
        pst_hmac_vap->station_info.ul_delay,
        pst_hmac_vap->station_info.tx_failed,
        pst_hmac_vap->station_info.tx_packets,
        pst_hmac_vap->station_info.tx_bytes);
#else
    OAM_WARNING_LOG3(0, OAM_SF_ANY,
        "{wal_hipriv_get_station_info::tx drop[%d], tx pkts[%d], tx bytes[%d]}",
        pst_hmac_vap->station_info.tx_failed,
        pst_hmac_vap->station_info.tx_packets,
        pst_hmac_vap->station_info.tx_bytes);
#endif
}

OAL_STATIC oal_uint32 wal_hipriv_get_station_info(oal_net_device_stru *pst_net_dev, oal_int8 *pc_param)
{
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    hmac_vap_stru *pst_hmac_vap = OAL_PTR_NULL;
    dmac_query_request_event st_dmac_query_request_event;
    dmac_query_station_info_request_event *pst_query_station_info = OAL_PTR_NULL;
    wal_msg_write_stru st_write_msg;
    oal_int i_leftime;
    oal_int32 l_ret;
    oal_uint8 uc_vap_id;
    if (OAL_ANY_NULL_PTR2(pst_net_dev, pc_param)) {
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_mac_vap = OAL_NET_DEV_PRIV(pst_net_dev);
    if (pst_mac_vap == OAL_PTR_NULL) {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{wal_hipriv_get_station_info::OAL_NET_DEV_PRIV, return null!}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    uc_vap_id = pst_mac_vap->uc_vap_id;

    pst_query_station_info = (dmac_query_station_info_request_event *)&st_dmac_query_request_event;
    pst_query_station_info->query_event = OAL_QUERY_STATION_INFO_EVENT;

    pst_hmac_vap = (hmac_vap_stru *)mac_res_get_hmac_vap(uc_vap_id);
    if (pst_hmac_vap == OAL_PTR_NULL) {
        return OAL_ERR_CODE_PTR_NULL;
    }


    pst_hmac_vap->station_info_query_completed_flag = OAL_FALSE;

    /********************************************************************************
        ���¼���wal�㴦�� �����ڵ͹�����Ҫ�����⴦�����������²����¼���ֱ����ʱ��
        �͹��Ļ��ڽ���beacon֡��ʱ�������ϱ���Ϣ��
    ********************************************************************************/
    /* 3.1 ��д msg ��Ϣͷ */
    st_write_msg.en_wid = WLAN_CFGID_QUERY_STATION_STATS;
    st_write_msg.us_len = OAL_SIZEOF(st_dmac_query_request_event);

    /* 3.2 ��д msg ��Ϣ�� */
    if (EOK != memcpy_s(st_write_msg.auc_value, OAL_SIZEOF(dmac_query_station_info_request_event),
                        pst_query_station_info, OAL_SIZEOF(dmac_query_station_info_request_event))) {
        return OAL_FAIL;
    }

    l_ret = wal_send_cfg_event_etc(pst_net_dev, WAL_MSG_TYPE_WRITE,
                                   WAL_MSG_WRITE_MSG_HDR_LENGTH + OAL_SIZEOF(dmac_query_station_info_request_event),
                                   (oal_uint8 *)&st_write_msg, OAL_FALSE, OAL_PTR_NULL);

    if (OAL_UNLIKELY(l_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(uc_vap_id, 0, "{wal_hipriv_get_station_info::wal_send_cfg_event_etc return err code %d!}", l_ret);
        return OAL_FAIL;
    }
    /*lint -e730*/ /* info, boolean argument to function */
    i_leftime = OAL_WAIT_EVENT_INTERRUPTIBLE_TIMEOUT(pst_hmac_vap->query_wait_q,
        (OAL_TRUE == pst_hmac_vap->station_info_query_completed_flag), 5 * OAL_TIME_HZ);
    /*lint +e730*/
    if (i_leftime <= 0) {
        OAM_WARNING_LOG0(uc_vap_id, OAM_SF_ANY, "{wal_hipriv_get_station_info::query info wait for 5 s timeout!}");
        return OAL_FAIL;
    } else {
        /* ��������  */
        wal_hipriv_printf_station_info(pst_hmac_vap);
        return OAL_SUCC;
    }
}
#endif

OAL_CONST wal_hipriv_cmd_entry_stru g_ast_hipriv_cmd_debug_etc[] = {
    /***********************��������***********************/
    /* ����TLV����ͨ������: hipriv "wlan0 set_tlv xxx 0 0"  */
    { "set_tlv", wal_hipriv_set_tlv_cmd },
    /* ����Value����ͨ������: hipriv "wlan0 set_val xxx 0"  */
    { "set_val", wal_hipriv_set_val_cmd },
    /* ����String����ͨ������: hipriv "wlan0 set_str 11ax_debug 3 tid 0 val 1 cnt 2"  */
    { "set_str", wal_hipriv_set_str_cmd },
#ifdef _PRE_WLAN_FEATURE_TWT
    { "twt_setup_req", wal_hipriv_twt_setup_req },
    { "twt_teardown_req", wal_hipriv_twt_teardown_req },
#endif

#ifdef _PRE_WLAN_FEATURE_AP_PM
    { "pm_info",   wal_hipriv_pm_info },     /* ����͹���PM��Ϣ hipriv "Hisilicon0 pm_info" */
    { "pm_enable", wal_hipriv_pm_enable }, /* ����͹���PM��Ϣ hipriv "Hisilicon0 pm_enable 0|1" */
    { "enable",    wal_hipriv_wifi_enable },  /* ������ر�wifi: hipriv "Hisilicon0 enable 0|1" */
#endif
    { "destroy", wal_hipriv_del_vap_etc }, /* ɾ��vap˽������Ϊ: hipriv "vap0 destroy" */

    { "global_log_switch",  wal_hipriv_global_log_switch },   /* ȫ����־����:  hipriv "Hisilicon0 global_log_switch 0 | 1 */
    /* VAP�������־����: hipriv "Hisilicon0{VAPx} log_switch 0 | 1"��������������е�VAP */
    { "log_switch",         wal_hipriv_vap_log_switch },
    { "feature_log_switch", wal_hipriv_feature_log_switch }, /* ���Ե�INFO������־���� hipriv "VAPX feature_name {0/1}" */
    /* ���Ե�INFO������־���� hipriv "Hisilicon0 log_ratelimit {type} {switch} {interval} {burst}"   */
    { "log_ratelimit",      wal_hipriv_log_ratelimit },
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
    { "log_pm",     wal_hipriv_log_lowpower },   /* log�͹���ģʽ: hipriv "Hisilicon0 log_pm 0 | 1"��log pmģʽ���� */
    { "pm_switch",  wal_hipriv_pm_switch },   /* log�͹���ģʽ: hipriv "Hisilicon0 pm_switch 0 | 1"��log pmģʽ���� */
    { "power_test", wal_hipriv_power_test }, /* log�͹���ģʽ: hipriv "Hisilicon0 power_test 0 | 1"�����Ĳ���ģʽ�� */
#endif
    { "event_switch", wal_hipriv_event_switch }, /* OAM eventģ��Ŀ��ص�����: hipriv "Hisilicon0 event_switch 0 | 1"��������������е�VAP */

#ifdef _PRE_WLAN_NARROW_BAND
    { "narrow_bw", wal_hipriv_narrow_bw }, /* Start DPD Calibration */
#endif

#ifdef _PRE_WLAN_CHIP_TEST
    { "beacon_offload_test", wal_hipriv_beacon_offload_test }, /* �ֶ�����host sleep״̬�������ڲ���: hipriv "Hisilicon0 host_sleep 0 | 1" */
#endif
    { "ota_beacon_on",    wal_hipriv_ota_beacon_switch },   /* �����Ƿ��ϱ�beacon֡����: hipriv "Hisilicon0 ota_beacon_switch 0 | 1"��������������е�VAP */
    { "ota_switch",       wal_hipriv_ota_rx_dscr_switch },     /* �����Ƿ��ϱ�����������֡����: hipriv "Hisilicon0 ota_rx_dscr_switch 0 | 1"��������������е�VAP */
    { "oam_output",       wal_hipriv_oam_output },             /* ����oamģ�����Ϣ��ӡλ������:hipriv "Hisilicon0 oam_output 0~4 (oam_output_type_enum_uint8)"��������������е�VAP */
    { "auto_ba",          wal_hipriv_auto_ba_switch },            /* �����Զ���ʼBA�Ự�Ŀ���:hipriv "vap0  auto_ba 0 | 1" ���������ĳһ��VAP */
    { "profiling_switch", wal_hipriv_profiling_switch }, /* �������ܲ��ԵĿ���:hipriv "vap0  profiling_switch 0 | 1"  */
    { "addba_req",        wal_hipriv_addba_req },               /* ���ý���BA�Ự����������:hipriv "vap0 addba_req xx xx xx xx xx xx(mac��ַ) tidno ba_policy buffsize timeout" ���������ĳһ��VAP */
    { "delba_req",        wal_hipriv_delba_req },               /* ����ɾ��BA�Ự����������: hipriv "vap0 delba_req xx xx xx xx xx xx(mac��ַ) tidno direction" ���������ĳһ��VAP */
#ifdef _PRE_WLAN_FEATURE_WMMAC
    { "addts_req",    wal_hipriv_addts_req },       /* ���ý���TS��������ADDTS REQ����������:hipriv "vap0 addts_req tid direction apsd up nominal_msdu_size max_msdu_size                                                                       minimum_data_rate mean_data_rate peak_data_rate minimum_phy_rate surplus_bandwidth_allowance" ���������ĳһ��VAP */
    { "delts",        wal_hipriv_delts },               /* ����ɾ��TS��������DELTS����������: hipriv "vap0 tidno" ���������ĳһ��VAP */
    { "reassoc_req",  wal_hipriv_reassoc_req },   /*  �����ع�������֡: hipriv "vap0 reassoc_req" */
    { "wmmac_switch", wal_hipriv_wmmac_switch }, /* ����WMMAC���أ���������: hipriv "vap0 wmmac_switch 1/0(ʹ��) 0|1(WMM_AC��֤ʹ��) xxx(timeout_period) factor" ����Device */
#endif
    { "meminfo", wal_hipriv_mem_info }, /* ��ӡ�ڴ����Ϣ: hipriv "Hisilicon0 meminfo poolid" */
    { "memleak", wal_hipriv_mem_leak }, /* ��ӡ�ڴ����Ϣ: hipriv "Hisilicon0 memleak poolid" */
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
    { "devicememleak", wal_hipriv_device_mem_leak }, /* ��ӡ�ڴ����Ϣ: hipriv "Hisilicon0 devicememleak poolid" */
    { "memoryinfo", wal_hipriv_memory_info },        /* ��ӡ�ڴ����Ϣ: hipriv "Hisilicon0 memoryinfo host/device" */
#endif
    { "beacon_chain_switch", wal_hipriv_beacon_chain_switch }, /* ����beacon֡���Ͳ�����������: hipriv "vap0 beacon_chain_switch 0/1" Ŀǰ��ȡ���ǵ�ͨ��ģʽ(ʹ��ͨ��0)��0��ʾ�ر�˫·�������ͣ�1��ʾ���������������ĳһ��VAP */

#ifdef _PRE_WLAN_FEATURE_BTCOEX
    { "coex_print",          wal_hipriv_btcoex_status_print },            /* ��ӡ����ά����Ϣ��sh hipriv.sh "coex_print" */
    { "coex_preempt_type",   wal_hipriv_btcoex_preempt_type },     /* ����preempt_type��sh hipriv.sh "vap0 coex_preempt_type  0/1(Ӳ����������) 0-3/0-1"  0 noframe 1 self-cts 2 nulldata 3 qosnull  ����ps�򿪻��߹ر� 2 slot��ǰ�� */
    { "set_coex_perf_param", wal_hipriv_btcoex_set_perf_param }, /* ����BAɾ�����ޣ�sh hipriv.sh "vap0 set_coex_perf_param  0/1/2/3" */
#endif
#ifdef _PRE_WLAN_FEATURE_NRCOEX
    { "nrcoex_set_prior", wal_hipriv_nrcoex_set_prior }, /* ����NR���������sh hipriv.sh "nrcoex_set_prior 0/1/2/3" */
    { "nrcoex_test", wal_hipriv_nrcoex_cfg_test },       /* ����NR���������sh hipriv.sh "nrcoex_test 0~20" */
#endif

    { "protocol_debug", wal_hipriv_set_protocol_debug_info }, /* ���ô�ӡphyά��������Ϣ��sh hipriv.sh "wlan0 protocol_debug [band_force_switch 0|1|2] [2040_ch_swt_prohi 0|1] [40_intol 0|1]" */

    { "set_tx_pow", wal_hipriv_set_tx_pow_param }, /* ���÷��͹��ʲ���: hipriv "vap0 set_tx_pow <param name> <value>" */

    { "set_ucast_data",  wal_hipriv_set_ucast_data_dscr_param },   /* ��ӡ��������Ϣ: hipriv "vap0 set_ucast_data <param name> <value>" */
    { "set_bcast_data",  wal_hipriv_set_bcast_data_dscr_param },   /* ��ӡ��������Ϣ: hipriv "vap0 set_bcast_data <param name> <value>" */
    { "set_ucast_mgmt",  wal_hipriv_set_ucast_mgmt_dscr_param },   /* ��ӡ��������Ϣ: hipriv "vap0 set_ucast_mgmt <param name> <value>" */
    { "set_mbcast_mgmt", wal_hipriv_set_mbcast_mgmt_dscr_param }, /* ��ӡ��������Ϣ: hipriv "vap0 set_mbcast_mgmt <param name> <value>" */
#ifdef _PRE_WLAN_FEATURE_WEB_CFG_FIXED_RATE
    { "set_mode_ucast_data", wal_hipriv_set_mode_ucast_data_dscr_param }, /* ����ָ��ģʽ��������֡������: hipriv "vap0 set_mode_ucast_data <protocol_mode> <param name> <value>" */
#endif
    { "nss",           wal_hipriv_set_nss },                 /* ����HTģʽ�µĿռ�������:   hipriv "vap0 nss   <value>" */
    { "txch",          wal_hipriv_set_rfch },               /* ���÷���ͨ��:               hipriv "vap0 rfch  <value>" */
    { "get_thruput",   wal_hipriv_get_thruput },     /* ��ȡоƬ������������        hipriv "vap0 get_thruput >" */
    { "set_freq_skew", wal_hipriv_set_freq_skew }, /* ����Ƶƫ����                 hipriv "Hisilicon0 set_freq_skew
                                                                     * <idx chn T0Int20M T0Frac20M T1Int20M T1Frac20M T0Int40M T0Frac40M T1Int40M T1Frac40M>" */
    { "adjust_ppm",   wal_hipriv_adjust_ppm },       /* ����ppm         hipriv "Hisilicon0 adjust_ppm ppm clock" */
    { "amsdu_start",  wal_hipriv_amsdu_start },     /* ��ӡ�Ĵ�����Ϣ: hipriv "vap0 amsdu_start xx xx xx xx xx xx(mac��ַ10����oal_atoi) <max num> <max size> " */
    { "list_ap",      wal_hipriv_list_ap },             /* ��ӡSTAɨ�赽��AP�б�: hipriv "sta0 list_ap" */
    { "list_sta",     wal_hipriv_list_sta },           /* ��ӡAP������STA�б�: hipriv "sta0 list_sta" */
    { "start_scan",   wal_hipriv_start_scan },       /* ����staɨ��: hipriv "sta0 start_scan" */
    { "start_join",   wal_hipriv_start_join },       /* ����sta���벢��֤����: hipriv "sta0 start_join 1" 1��ʾɨ�赽��AP��deviceд�����±�� */
    { "start_deauth", wal_hipriv_start_deauth },   /* ����staȥ��֤: hipriv "vap0 start_deauth" */
    { "dump_timer",   wal_hipriv_dump_timer },       /* ��ӡ����timer��ά����Ϣ hipriv "vap0 dump_timer" */
    { "kick_user",    wal_hipriv_kick_user },         /* ɾ��1���û� hipriv "vap0 kick_user xx xx xx xx xx xx(mac��ַ)" */
    { "pause_tid",    wal_hipriv_pause_tid },         /* ��ָͣ���û���ָ��tid hipriv "vap0 pause_tid xx xx xx xx xx xx(mac��ַ) tid_num 0\1" */
    { "set_user_vip", wal_hipriv_set_user_vip },   /* ����ĳ���û�ΪVIP���߷�VIP��sh hipriv.sh "vap0 set_user_vip xx xx xx xx xx xx(mac��ַ) 0\1" */
    { "set_vap_host", wal_hipriv_set_vap_host },   /* ����ĳ��vapΪhost����guest vap: sh hipriv.sh "vap0 st_vap_host 0\1" */
    { "ampdu_tx_on",  wal_hipriv_ampdu_tx_on },     /* ������ر�ampdu���͹��� hipriv "vap0 ampdu_tx_on 0\1" */
    { "amsdu_tx_on",  wal_hipriv_amsdu_tx_on },     /* ������ر�ampdu���͹��� hipriv "vap0 amsdu_tx_on 0\1" */
    { "send_bar",     wal_hipriv_send_bar },           /* ָ��tid����bar hipriv "vap0 send_bar A6C758662817(mac��ַ) tid_num" */
#ifdef _PRE_WLAN_FEATUER_PCIE_TEST
    { "pcie_test", wal_hipriv_pcie_test }, /* PCIE test: 0-15(���߲�������0-15) 0/1(��) 0/1(д): hipriv "vap0 pcie_test 0-15 0/1 0/1" */
#endif
    { "dump_ba_bitmap",       wal_hipriv_dump_ba_bitmap },              /* ��ӡ����ba��bitmap hipriv "vap0 dump_ba_bitmap (tid_no) (RA)" */
    { "wifi_stat_info",       wal_hipriv_show_stat_info },              /* ��ȡ����ά��ͳ����Ϣ: hipriv "Hisilicon0 wifi_stat_info" */
    { "vap_pkt_stat",         wal_hipriv_show_vap_pkt_stat },             /* ��ȡĳһ��vap�µ��շ���ͳ����Ϣ: sh hipriv.sh "vap_name vap_pkt_stat" */
    { "clear_stat_info",      wal_hipriv_clear_stat_info },            /* ��������ά��ͳ����Ϣ: hipriv "Hisilicon0 clear_stat_info" */
    { "usr_stat_info",        wal_hipriv_user_stat_info },               /* �ϱ�ĳ��user�µ�ά��ͳ����Ϣ: sh hipriv.sh "Hisilicon0 usr_stat_info usr_id" */
    { "timer_start",          wal_hipriv_timer_start },                    /* ����5115Ӳ����ʱ��: hipriv "Hisilicon0 timer_start 0/1" */
    { "show_profiling",       wal_hipriv_show_profiling },              /* ����5115Ӳ����ʱ��: hipriv "Hisilicon0 show_profiling 0/1/2(0��rx 1��tx 2��chipstart) 0/1/2 (0�Ƿֶ����� 1�Ƿֶ����ݺ�ÿ���ڵ����� 2�Ǵ�ӡ�ֶ����ݺ�ÿ���ڵ����ݣ����������)" */
    { "profiling_packet_add", wal_hipriv_profiling_packet_add },  /* ����5115Ӳ����ʱ��: hipriv "Hisilicon0 profiling_packet_add" һ�η���������������һ��ͳ�ƣ���packet num ��1 */
    { "reset_hw",             wal_hipriv_reset_device },                      /* ��λӲ��phy&mac: hipriv "Hisilicon0 reset_hw 0|1|2|3|4|5|6|8|9|10|11
                                                                                                                    (all|phy|mac|debug|mac_tsf|mac_cripto|mac_non_cripto|phy_AGC|phy_HT_optional|phy_VHT_optional|phy_dadar )
                                                                                                                    0|1(reset phy reg) 0|1(reset mac reg) */
    { "reset_operate",       wal_hipriv_reset_operate },                /* ��λӲ��phy&mac: hipriv "Hisilicon0 reset_hw 0|1|2|3(all|phy|mac|debug) 0|1(reset phy reg) 0|1(reset mac reg) */
    { "dump_rx_dscr",        wal_hipriv_dump_rx_dscr_etc },              /* dump�����������������У�hipriv "Hisilicon0 dump_rx_dscr 0|1", 0:�����ȼ����� 1:��ͨ���ȼ�����  */
    { "dump_tx_dscr",        wal_hipriv_dump_tx_dscr },                  /* dump�����������������У�hipriv "Hisilicon0 dump_tx_dscr value", valueȡֵ0~3����AC���Ͷ��У�4��������֡ */
    { "dump_memory",         wal_hipriv_dump_memory },                    /* dump�ڴ棬 hipriv "Hisilicon0 dump_memory 0xabcd len" */
    { "show_tx_dscr_addr",   wal_hipriv_show_tx_dscr_addr },        /* ��ӡ�ڴ�������з�����������ַ hipriv "Hisilicon0 show_tx_dscr_addr" */
    { "list_channel",        wal_hipriv_list_channel },                  /* ֧���ŵ��б��� hipriv "Hisilicon0 list_channel" */
    { "set_regdomain_pwr_p", wal_hipriv_set_regdomain_pwr_priv }, /* ���ù���������͹���(����ͻ�ƹ����������)��hipriv "Hisilicon0 set_regdomain_pwr_priv 20",��λdBm */
    { "event_queue",         wal_hipriv_event_queue_info },               /* ��ӡ�¼�������Ϣ������ӡ��ÿһ���ǿ��¼��������¼��ĸ������Լ�ÿһ���¼�ͷ��Ϣ, hipriv "Hisilicon0 event_queue" */
    { "frag_threshold",      wal_hipriv_frag_threshold },              /* ���÷�Ƭ���޵���������: hipriv "vap0 frag_threshold (len)" ���������ĳһ��VAP */
    { "wmm_switch",          wal_hipriv_wmm_switch },                      /* ��̬�������߹ر�wmm hipriv "vap0 wmm_switch 0|1"(0��ʹ�ܣ�1ʹ��)  */
    { "hide_ssid",           wal_hipriv_hide_ssid },                        /*  ����ssid���ܿ������߹ر� wmm hipriv "Hisilicon0 hide_ssid 0|1"(0��ʹ�ܣ�1ʹ��)  */
    { "ether_switch",        wal_hipriv_set_ether_switch },              /* ������̫��֡�ϱ��Ŀ��أ�sh hipriv.sh "vap0 ether_switch user_macaddr oam_ota_frame_direction_type_enum(֡����) 0|1(����)" */
    { "80211_uc_switch",     wal_hipriv_set_80211_ucast_switch },     /* ����80211����֡�ϱ��Ŀ��أ�sh hipriv.sh "vap0 80211_uc_switch user_macaddr 0|1(֡����tx|rx) 0|1(֡����:����֡|����֡) 0|1(֡���ݿ���) 0|1(CB����) 0|1(����������)" */
    { "80211_mc_switch",     wal_hipriv_set_80211_mcast_switch },     /* ����80211�鲥\�㲥֡�ϱ��Ŀ��أ�sh hipriv.sh "Hisilicon0 80211_mc_switch 0|1(֡����tx|rx) 0|1(֡����:����֡|����֡) 0|1(֡���ݿ���) 0|1(CB����) 0|1(����������)" */
    { "probe_switch",        wal_hipriv_set_probe_switch },              /* ����probe req��rsp�ϱ��Ŀ��أ�sh hipriv.sh "Hisilicon0 probe_switch 0|1(֡����tx|rx) 0|1(֡���ݿ���) 0|1(CB����) 0|1(����������)" */
    { "phy_debug",           wal_hipriv_set_phy_debug_switch },             /* ���ô�ӡphyά��������Ϣ��sh hipriv.sh "Hisilicon0 phy_debug [snr 0|1] [rssi 0|1] [trlr 0|1] [count N]" */
#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
    { "report_vap_info", wal_hipriv_report_vap_info }, /* ���ݱ��λ�ϱ�vap�Ķ�Ӧ��Ϣ sh hipriv.sh "wlan0 report_vap_info 1" */
#endif
    { "mpdu_num",        wal_hipriv_get_mpdu_num },               /* ��ȡdevice�º�ÿһ��tid�µ�ǰmpdu������sh hipriv.sh "vap_name mpdu_num user_macaddr" */
    { "set_all_ota",     wal_hipriv_set_all_ota },             /* ��������ota�ϱ������Ϊ1������������֡��cb���������������Ϊ0��ʲô��������sh hipriv.sh "Hisilicon0 set_all_ota 0|1" */
    { "80211_uc_all",    wal_hipriv_set_all_80211_ucast },    /* ���������û��ĵ������أ�sh hipriv.sh "Hisilicon0 80211_uc_all 0|1(֡����tx|rx) 0|1(֡����:����֡|����֡) 0|1(֡���ݿ���) 0|1(CB����) 0|1(����������)" */
    { "ether_all",       wal_hipriv_set_all_ether_switch },      /* ���������û�����̫�����أ�sh hipriv.sh "Hisilicon0 ether_all 0|1(֡����tx|rx) 0|1(����)" */
    { "dhcp_arp_switch", wal_hipriv_set_dhcp_arp_switch }, /* ���÷��͹㲥arp��dhcp���أ�sh hipriv.sh "Hisilicon0 dhcp_arp_switch 0|1(����)" */
#ifdef _PRE_DEBUG_MODE_USER_TRACK
    { "thrput_stat", wal_hipriv_report_thrput_stat }, /* �ϱ�����ֹͣ�ϱ���Ӧuserʵʱ����ͳ����Ϣ: sh hipriv.sh "vap_name thrput_stat  XX:XX:XX:XX:XX;XX 0|1" */
#endif
#ifdef _PRE_WLAN_DFT_STAT
    { "clear_vap_stat_info", wal_hipriv_clear_vap_stat_info }, /* ����ָ��VAP��ͳ����Ϣ: hipriv "vap_name clear_vap_stat_info" */
#endif
#ifdef _PRE_WLAN_FEATURE_TXOPPS
    { "txopps_hw_en", wal_hipriv_set_txop_ps_machw }, /* ����mac txop psʹ�ܼĴ�����sh hipriv.sh "stavap_name txopps_hw_en 0|1(txop_ps_en) 0|1(condition1) 0|1(condition2)" */
#endif
#ifdef _PRE_WLAN_FEATURE_UAPSD
    { "uapsd_debug", wal_hipriv_uapsd_debug }, /* uapsdά����Ϣ��sh hipriv "vap0 uapsd_debug 0|1|2(���û�|all user|���ͳ�Ƽ�����) xx:xx:xx:xx:xx:xx(mac��ַ)" */
#endif
#ifdef _PRE_WLAN_DFT_STAT
    { "phy_stat_en", wal_hipriv_set_phy_stat_en }, /* ����phyͳ��ʹ�ܽڵ��ţ�һ�ο�������4����������Χ1~16��sh hipriv.sh "Hisilicon0 phy_stat_en idx1 idx2 idx3 idx4" */

    { "dbb_env_param",  wal_hipriv_dbb_env_param },    /* �ϱ�����ֹͣ�ϱ��տڻ����������Ϣ: sh hipriv.sh "Hisilicon0 dbb_env_param 0|1" */
    { "usr_queue_stat", wal_hipriv_usr_queue_stat },  /* �ϱ����������û�����ͳ����Ϣ: sh hipriv.sh "vap_name usr_queue_stat XX:XX:XX:XX:XX:XX 0|1" */
    { "vap_stat",       wal_hipriv_report_vap_stat },       /* �ϱ�����ֹͣ�ϱ�vap����ͳ����Ϣ: sh hipriv.sh "vap_name vap _stat  0|1" */
    { "reprt_all_stat", wal_hipriv_report_all_stat }, /* �ϱ�������������ά��ͳ����Ϣ: sh hipriv.sh "Hisilicon0 reprt_all_stat type(phy/machw/mgmt/irq/all)  0|1" */
#endif

    { "ampdu_aggr_num", wal_hipriv_set_ampdu_aggr_num }, /* ����AMPDU�ۺϸ���: sh hipriv.sh "Hisilicon0 ampdu_aggr_num aggr_num_switch aggr_num" ,aggr_num_switch��0ʱ��aggr_num��Ч */

    { "set_stbc_cap", wal_hipriv_set_stbc_cap }, /* ����STBC���� */
    { "set_ldpc_cap", wal_hipriv_set_ldpc_cap }, /* ����LDPC���� */
    { "set_txbf_cap", wal_hipriv_set_txbf_cap }, /* ������ر�txbf�� ����bit0/����bit1 ���� hipriv "vap0 alg_txbf_switch 0|1|2|3" */
#ifdef _PRE_WLAN_FEATURE_STA_PM
    { "set_psm_para", wal_hipriv_sta_psm_param },  /* sh hipriv.sh 'wlan0 set_psm_para 100 40 */
    { "set_sta_pm_on", wal_hipriv_sta_pm_on_etc }, /* sh hipriv.sh 'wlan0 set_sta_pm_on xx xx xx xx */
#endif
#ifdef _PRE_WLAN_CHIP_TEST
    { "lpm_soc_mode", wal_hipriv_lpm_soc_mode }, /* ˯�߻���оƬ, hipriv "Hisilicon0 lpm_soc_mode 0|1|2|3|4(����gating|PCIE RD BY PASS|mem precharge|PCIE L0-S|PCIE L1-0) 0|1(disable|enable)" */
    { "lpm_chip_state",     wal_hipriv_lpm_chip_state },         /* ˯�߻���оƬ, hipriv "Hisilicon0 lpm_chip_state 0|1|2(0:soft sleep��1:gpio sleep,2:work)" */
    { "lpm_psm_param",      wal_hipriv_lpm_psm_param },           /* psm���ܼĴ�������, hipriv "Hisilicon0 lpm_psm_param 0|1|2(ps off|ps on|debug) 0|1(DTIM|listen intval) xxx(listen intervalֵ) xxx(TBTT offset)" */
    { "lpm_smps_mode",      wal_hipriv_lpm_smps_mode },           /* smps����ģʽ����, hipriv "Hisilicon0 lpm_smps_mode 0|1|2(off|static|dynamic)" */
    { "lpm_smps_stub",      wal_hipriv_lpm_smps_stub },           /* smps ap������׮, hipriv "vap0 lpm_smps_stub 0|1|2(off|����|˫��) 0|1(�Ƿ�RTS)" */
    { "lpm_txopps_set",     wal_hipriv_lpm_txopps_set },         /* txop ps����ģʽ����, hipriv "Hisilicon0 lpm_txopps_set 0|1(off|on|debug) 0|1(contion1 off|on) 0|1(condition2 off|on)" */
    { "lpm_txopps_tx_stub", wal_hipriv_lpm_txopps_tx_stub }, /* txop ps�������Դ�׮����, hipriv "vap0 lpm_txopps_tx_stub 0|1|2(off|address|partial AID) xxx(�ڼ�������׮)" */
    { "lpm_tx_data",        wal_hipriv_lpm_tx_data },               /* ���Է���, hipriv "vap0 lpm_tx_data xxx(����) xxx(����) xx:xx:xx:xx:xx:xx(Ŀ��mac) xxx(AC����)" */
    { "lpm_tx_probe_req",   wal_hipriv_lpm_tx_probe_request }, /* ���Է���, hipriv "vap0 lpm_tx_probe_req 0|1(����|����) xx:xx:xx:xx:xx:xx(����ģʽ��BSSID)" */
    { "remove_lut",         wal_hipriv_remove_user_lut },            /* ɾ���ָ��û�lut��, hipriv "vap0 remove_lut xx:xx:xx:xx:xx:xx(mac��ַ 16����) 0|1(�ָ�/ɾ��)" */
#endif
    { "send_frame", wal_hipriv_send_frame }, /* ָ��tid����bar hipriv "vap0 send_frame (type) (num) (Ŀ��mac)" */
#ifdef _PRE_WLAN_CHIP_TEST
    { "set_rx_pn",      wal_hipriv_set_rx_pn },           /* ����RX_PN_LUT_CONFIG�Ĵ��� */
    { "set_sft_retry",  wal_hipriv_set_soft_retry },  /* ����software_retry ������ hipriv "Hisilicon0 set_sft_retry 0|1(0��ʹ�ܣ�1ʹ��)"  */
    { "open_addr4",     wal_hipriv_open_addr4 },         /* ����macͷ����4��ַ hipriv "Hisilicon0 open_addr4 0|1(0��ʹ�ܣ�1ʹ��)  */
    { "open_wmm_test",  wal_hipriv_open_wmm_test },   /* ����оƬ��֤���� hipriv "Hisilicon0 open_wmm_test 0|1|2|3  ()  */
    { "chip_test",      wal_hipriv_chip_test_open },      /* ����оƬ��֤���� hipriv "Hisilicon0 chip_test 0|1(0��ʹ�ܣ�1ʹ��)  */
    { "coex_ctrl",      wal_hipriv_set_coex },            /* ���ù�����ƿ��� hipriv "Hisilicon0 coex_ctrl xxx(mac ctrlֵ) xxx(rf ctrlֵ))  */
    { "dfx_en",         wal_hipriv_set_dfx },                /* ����DFX���Կ��� sh hipriv.sh "Hisilicon0 dfx_en 0|1  */
    { "clear_all_stat", wal_hipriv_clear_all_stat }, /* ����жϺ͹���֡ͳ����Ϣ hipriv "Hisilicon0 clear_all_stat" */
    { "send_action",    wal_hipriv_test_send_action },  /* ����action֡�ӿ� sh hipriv.sh "vap0 send_action XX(category) xx:xx:xx:xx:xx:xx(Ŀ�ĵ�ַ 16����) " */
    { "send_pspoll",    wal_hipriv_send_pspoll },       /* sta��ps-poll��ap��sh hipriv "vap0 send_pspoll" */
    { "send_nulldata",  wal_hipriv_send_nulldata },   /* sta��null data��ap��֪ͨ����״̬��sh hipriv "vap0 send_nulldata 0|1(�Ƿ�������) 0|1(�Ƿ�qosnull) tid_no" */
#endif                                               /* #ifdef _PRE_WLAN_CHIP_TEST */
#if (_PRE_WLAN_FEATURE_PMF != _PRE_PMF_NOT_SUPPORT)
    { "enable_pmf", wal_hipriv_enable_pmf }, /* ����chip test��ǿ��ʹ��pmf���� (���ڹ���֮��)��sh hipriv.sh "vap0 enable_pmf 0|1|2(0��ʹ�ܣ�1 enable, 2ǿ��)  */
#endif
#ifdef _PRE_WLAN_FEATURE_GREEN_AP
    { "set_free_ratio", wal_hipriv_set_gap_free_ratio }, /* ����green ap��free ratio���� hipriv "Hisilicon0 set_free_ratio 0|1 val" */
#endif
    { "set_default_key", wal_hipriv_set_default_key }, /* ����chip test��ǿ��ʹ��pmf���� (���ڹ���֮��)��sh hipriv.sh "vap0 set_default_key x(key_index) 0|1(en_unicast) 0|1(multicast)"  */
    { "add_key", wal_hipriv_test_add_key },            /* chip test����add key������˽����������ӿڪ�
                          sh hipriv.sh "xxx(cipher) xx(en_pairwise) xx(key_len) xxx(key_index) xxxx:xx:xx:xx:xx:xx...(key С��32�ֽ�) xx:xx:xx:xx:xx:xx(Ŀ�ĵ�ַ)  */

#ifdef _PRE_WLAN_FEATURE_DFR
    { "dfr_start", wal_hipriv_test_dfr_start }, /* dfr���ܴ�׮�����ӿڪ�sh hipriv.sh "vap0 dfr_start 0(dfr�ӹ���:0-device�쳣��λ )" */
#endif                                          // _PRE_WLAN_FEATURE_DFR
    /* �㷨��ص����� */
    { "alg_ar_log",  wal_hipriv_ar_log },   /* autorate�㷨��־��������: */
    { "alg_ar_test", wal_hipriv_ar_test }, /* autorate�㷨ϵͳ�������� */
    { "alg",         wal_hipriv_alg },             /* alg */

#ifdef _PRE_WLAN_FEATURE_RX_AGGR_EXTEND
    { "waveapp_enable", wal_hipriv_waveapp_32plus_user_enable }, /* ʹ��waveapp 32���϶��û����Կ��� hipriv "Hisilicon0 waveapp_enable 0|1" */
#endif

    { "alg_txbf_log", wal_hipriv_txbf_log },         /* txbf�㷨��־��������: */
    { "alg_intf_det_log", wal_hipriv_intf_det_log }, /* ���ż���㷨��־��������: */
#ifdef _PRE_SUPPORT_ACS
    { "acs", wal_hipriv_acs },
    { "chan_stat", wal_hipriv_chan_stat },
#endif

    { "start_priv", wal_hipriv_set_priv_flag }, /* ֻ������ifconfig up������APʱ����Ҫ��ͨ���˽ӿ�����һ����� */
    { "bw_fixed", wal_hipriv_set_bw_fixed },    /* ����apģʽ��vap�����̶� */
#ifdef _PRE_WLAN_PERFORM_STAT
    /* ����ͳ������ */
    { "stat_tid_thrpt",  wal_hipriv_stat_tid_thrpt },   /* ͳ��ָ��tid��������: hipriv "vap0 stat_tid_thrpt xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���)" */
    { "stat_user_thrpt", wal_hipriv_stat_user_thrpt }, /* ͳ��ָ��user��������: hipriv "vap0 stat_user_thrpt xx xx xx xx xx xx(mac��ַ) stat_period(ͳ������ms) stat_num(ͳ�ƴ���)" */
    { "stat_user_bsd",   wal_hipriv_stat_user_bsd },     /* ͳ��ָ��user��������,rssi: hipriv "vap0 stat_user_bsd xx xx xx xx xx xx(mac��ַ) stat_period(ͳ������ms) stat_num(ͳ�ƴ���)" */
    { "stat_vap_thrpt",  wal_hipriv_stat_vap_thrpt },   /* ͳ��ָ��tid��������: hipriv "vap0 stat_vap_thrpt stat_period(ͳ������ms) stat_num(ͳ�ƴ���)" */
    { "stat_tid_per",    wal_hipriv_stat_tid_per },       /* ͳ��ָ��tid��per: hipriv "vap0 stat_tid_per xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���)" */
    { "stat_tid_delay",  wal_hipriv_stat_tid_delay },   /* ͳ��ָ��tid��delay: hipriv "vap0 stat_tid_delay xx xx xx xx xx xx(mac��ַ) tid_num stat_period(ͳ������ms) stat_num(ͳ�ƴ���)" */

    /* ������ʾ���� */
    { "dspl_tid_thrpt",  wal_hipriv_display_tid_thrpt },   /* ͳ��ָ��tid��������: hipriv "vap0 dspl_tid_thrpt xx xx xx xx xx xx(mac��ַ)" */
    { "dspl_user_thrpt", wal_hipriv_display_user_thrpt }, /* ͳ��ָ��user��������: hipriv "vap0 dspl_user_thrpt xx xx xx xx xx xx(mac��ַ)" */
    { "dspl_vap_thrpt",  wal_hipriv_display_vap_thrpt },   /* ͳ��ָ��tid��������: hipriv "vap0 dspl_vap_thrpt" */
    { "dspl_tid_per",    wal_hipriv_display_tid_per },       /* ͳ��ָ��tid��per: hipriv "vap0 dspl_tid_per xx xx xx xx xx xx(mac��ַ) tid_num" */
    { "dspl_tid_delay",  wal_hipriv_display_tid_delay },   /* ͳ��ָ��tid��delay: hipriv "vap0 dspl_tid_delay xx xx xx xx xx xx(mac��ַ) tid_num" */
#endif

#ifdef _PRE_WLAN_FEATURE_EDCA_OPT_AP
    { "set_edca_weight_sta", wal_hipriv_set_edca_opt_weight_sta }, /* STA edca��������Ȩ�� */
    { "set_edca_switch_ap",  wal_hipriv_set_edca_opt_switch_ap },   /* �Ƿ���edca�Ż����� */
    { "set_edca_cycle_ap",   wal_hipriv_set_edca_opt_cycle_ap },     /* ����edca�������������� */
#endif

#ifdef _PRE_WLAN_FEATURE_OFFLOAD_FLOWCTL
    { "get_hipkt_stat",    wal_hipriv_get_hipkt_stat },       /* ��ȡ�����ȼ����ĵ�ͳ����� */
    { "set_flowctl_param", wal_hipriv_set_flowctl_param }, /* ����������ز��� */
    { "get_flowctl_stat",  wal_hipriv_get_flowctl_stat },   /* ��ȡ�������״̬��Ϣ */
#endif

    { "thruput_bypass", wal_hipriv_set_thruput_bypass },   /* ����thruput bypassά��� */
    { "auto_protection", wal_hipriv_set_auto_protection }, /* �����Զ��������� */

    /* ����ά����� */
    { "send_2040_coext", wal_hipriv_send_2040_coext }, /* ����20/40�������֡: hipriv "Hisilicon0 send_2040_coext coext_info chan_report" */
    { "2040_coext_info", wal_hipriv_2040_coext_info }, /* ��ӡvap������20/40���������Ϣ: hipriv "vap0 2040_coext_info" */
    { "get_version",     wal_hipriv_get_version },         /* ��ȡ�����汾: hipriv "vap0 get_version" */

#ifdef _PRE_WLAN_FEATURE_FTM
    /* hipriv.sh "wlan0 ftm_debug ---------------------------------------------------------------------------------------------------------*/
    /* -------------------------- enable_ftm_initiator [0|1] */
    /* -------------------------- send_initial_ftm_request channel[] burst[0|1] ftms_per_burst[n] \
                                  en_lci_civic_request[0|1] asap[0|1] bssid[xx:xx:xx:xx:xx:xx] */
    /* -------------------------- enable [0|1] */
    /* -------------------------- cali [0|1] */
    /* -------------------------- send_ftm bssid[xx:xx:xx:xx:xx:xx] */
    /* -------------------------- set_ftm_time t1[] t2[] t3[] t4[] */
    /* -------------------------- enable_ftm_responder [0|1] */
    /* -------------------------- send_range_req mac[] num_rpt[] delay[] ap_cnt[] bssid[] channel[] \
                                  bssid[] channel[] ... */
    /* -------------------------- enable_ftm_range [0|1] */
    /* -------------------------- get_cali */
    /* -------------------------- set_location type[] mac[] mac[] mac[] */
    /* -------------------------- set_ftm_m2s en_is_mimo[] uc_tx_chain_selection[] */
    { "ftm_debug", wal_hipriv_ftm },
    { "ftm_set", wal_hipriv_set_white_list },
#endif

#ifdef _PRE_WLAN_FEATURE_DAQ
    { "data_acq", wal_hipriv_data_acq }, /* ��ȡ�����汾: hipriv "Hisilicon0 data_acq 0/1/2/3/4 (length num depth) (channel mode data_th bit) (2) () ()" */
#endif

#ifdef _PRE_WLAN_FEATURE_PSD_ANALYSIS
    { "set_psd", wal_hipriv_set_psd }, /* ʹ��PSD�ɼ�: hipriv "Hisilicon0 set_psd 0/1"��ʹ��֮ǰ��config  */
    { "cfg_psd", wal_hipriv_cfg_psd }, /* ʹ��PSD�ɼ�: hipriv "Hisilicon0 cfg_psd" */
#endif
#ifdef _PRE_WLAN_FEATURE_CSI
    { "set_csi", wal_hipriv_set_csi }, /* ʹ��CSI�ϱ�: hipriv "Hisilicon0 set_csi xx xx xx xx xx xx(mac��ַ) ta_check csi_en" */
#endif

#ifdef _PRE_WLAN_FEATURE_OPMODE_NOTIFY
    { "set_opmode_notify", wal_hipriv_set_opmode_notify }, /* ����VAP����ģʽ֪ͨ: hipriv "vap0 set_opmode_notify 0/1"  0-��֧��; 1-֧�� */
    { "get_user_nssbw", wal_hipriv_get_user_nssbw },       /* ���������û�����������: hipriv "vap0 get_user_nssbw xx xx xx xx xx xx(mac��ַ) "  ���������ĳһ��VAP */
#endif

#ifdef _PRE_WLAN_FEATURE_M2S
    { "set_m2s_switch", wal_hipriv_set_m2s_switch }, /* mimo��siso�л�: hipriv "Hisilicon0 set_m2s_switch  0/1/2/3/4/5...(������ѯ/����ģʽ/resv/���л���Ӳ�л�����ģʽ) */
#endif

#ifdef _PRE_WLAN_DFT_REG
    { "dump_reg",      wal_hipriv_dump_reg },           /* ������Ҫ��ȡ�ļĴ�����hipriv "Hisilicon0 dump_reg phy/mac/soc/abb/rf (subtype-����) (flag 0/1)" */
    { "dump_reg_evt",  wal_hipriv_dump_reg_evt },   /* ���ô����Ĵ�����ȡ���¼����¼�������hipriv "Hisilicon0 dump_reg_evt tx/rx/tbtt/prd (tick-����)" */
    { "dump_reg_addr", wal_hipriv_dump_reg_addr }, /* ��ʾ�Ĵ�����ȡ��ص�����Ϣ */

    { "dump_reg_info", wal_hipriv_dump_reg_info }, /* ��ʾ�Ĵ�����ȡ��ص�����Ϣ */
#endif

    { "set_vap_nss", wal_hipriv_set_vap_nss }, /* ����VAP�Ŀռ�������:hipriv "vap0 set_vap_nss <value>" */

#if (_PRE_WLAN_FEATURE_BLACKLIST_LEVEL != _PRE_WLAN_FEATURE_BLACKLIST_NONE)

    { "blacklist_add",  wal_hipriv_blacklist_add },       /* 1 */
    { "blacklist_del",  wal_hipriv_blacklist_del },       /* 2 */
    { "blacklist_mode", wal_hipriv_set_blacklist_mode }, /* 3 */
    { "blacklist_show", wal_hipriv_blacklist_show },     /* 4 wal_config_blacklist_show */
    { "abl_on",         wal_hipriv_set_abl_on },                 /* 5 */
    { "abl_aging",      wal_hipriv_set_abl_aging },           /* 6 */
    { "abl_threshold",  wal_hipriv_set_abl_threshold },   /* 7 */
    { "abl_reset",      wal_hipriv_set_abl_reset },           /* 8 wal_config_set_autoblacklist_reset_time */
#endif
#ifdef _PRE_WLAN_FEATURE_ISOLATION
    { "isolation_mode",  wal_hipriv_set_isolation_mode },   /* 9 */
    { "isolation_type",  wal_hipriv_set_isolation_type },   /* 10 */
    { "isolation_fwd",   wal_hipriv_set_isolation_fwd },     /* 11 */
    { "isolation_clear", wal_hipriv_set_isolation_clear }, /* 12 wal_config_set_isolation_clear */
    { "isolation_show",  wal_hipriv_set_isolation_show },   /* 13 wal_config_isolation_show */

#endif
#if defined(_PRE_WLAN_FEATURE_MCAST) || defined(_PRE_WLAN_FEATURE_HERA_MCAST)
    { "m2u_snoop_on",              wal_hipriv_m2u_snoop_on },                           /* ������ر�snoop���ع��� hipriv "vap0 m2u_snoop_on 0\1" */
    { "m2u_add_deny_table",        wal_hipriv_m2u_add_deny_table },               /* �����鲥������� hipriv "vap0 m2u_add_deny_table 224.1.1.1" */
    { "m2u_del_deny_table",        wal_hipriv_m2u_del_deny_table },               /* ɾ���鲥������� hipriv "vap0 m2u_del_deny_table 224.1.1.1" */
    { "m2u_cfg_deny_table",        wal_hipriv_m2u_cfg_deny_table },               /* ����鲥������� hipriv "vap0 m2u_cfg_deny_table 1 0" */
    { "m2u_show_snoop_table",      wal_hipriv_m2u_show_snoop_table },           /* ��ӡ�鲥�� hipriv "vap0 m2u_show_snoop_table 1" */
    { "m2u_igmp_pkt_xmit",         wal_hipriv_igmp_packet_xmit },                  /* ��Ŀ��STA/AP��������֡: hipriv "vap0 m2u_igmp_pkt_xmit (tid_no) (���ĸ���) (���ĳ���) (RA MAC)" */
    { "m2u_tid_set",               wal_hipriv_m2u_tid_set },                             /* �����鲥ת��������ʹ�õ�tid: hipriv "vap0 m2u_tid_set 0~7" */
    { "m2u_show_snoop_deny_table", wal_hipriv_m2u_show_snoop_deny_table }, /* ��ӡ�鲥������ hipriv "vap0 m2u_show_snoop_deny_table 1" */
#endif
#ifdef _PRE_WLAN_FEATURE_HERA_MCAST
    { "m2u_adaptive_on", wal_hipriv_m2u_adaptive_on },   /* ������ر�����ģʽʶ�𿪹� hipriv "vap0 m2u_adaptive_on 0\1" */
    { "m2u_frequency_on", wal_hipriv_m2u_frequency_on }, /* ������ر���Ƶ�鲥ת������ hipriv "vap0 m2u_frequency_on 0\1" */
#endif

#ifdef _PRE_WLAN_FEATURE_QOS_ENHANCE
    { "qos_enhance_on", wal_hipriv_qos_enhance_on }, /* ������ر���Ƶ�鲥ת������ hipriv "vap0 qos_enhance_on 0\1" */
#endif

    { "vap_classify_en", wal_hipriv_vap_classify_en },   /* device������������ ���û���vap��ҵ������Ƿ�ʹ�� hipriv "Hisilicon0 vap_classify_en 0/1" */
    { "vap_classify_tid", wal_hipriv_vap_classify_tid }, /* ����vap�����ȼ� hipriv "vap0 classify_tid 0~7" */

#ifdef _PRE_WLAN_PROFLING_MIPS
    { "set_mips", wal_hipriv_set_mips },   /* ����ĳ���̵�MIPSͳ�ƿ��أ�sh hipriv.sh "Hisilicon0 set_mips wal_mips_param_enum 0|1" */
    { "show_mips", wal_hipriv_show_mips }, /* ��ӡĳ���̵�MIPSͳ�ƽ����sh hipriv.sh "Hisilicon0 show_mips wal_mips_param_enum" */
#endif

#ifdef _PRE_WLAN_RR_PERFORMENCE_DEBUG
    /* ��ӡRR�����ʱ�����sh hipriv.sh "Hisilicon0 show_rr_timeinfo 0|1" */
    { "show_rr_timeinfo", wal_hipriv_show_rr_time_info },
#endif

    { "txpower", wal_hipriv_set_txpower }, /* ��������͹��ʣ�Ҫ����ֵ��������10�������룬���������Ҫ����Ϊ20������200 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 44))
    { "essid",   wal_hipriv_set_essid_etc },        /* ����AP ssid */
    { "bintval", wal_ioctl_set_beacon_interval }, /* ����AP beacon ���� */
    { "up",      wal_hipriv_start_vap },
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,44)) */

#ifdef _PRE_WLAN_FEATURE_11D
    { "set_rd_by_ie_switch", wal_hipriv_set_rd_by_ie_switch_etc }, /* �����Ƿ���ݹ���ap���¹�������Ϣ hipriv "Hisilicon0 set_rd_by_ie_switch 0/1" */
#endif
#ifdef _PRE_WLAN_FEATURE_P2P
#ifdef _PRE_WLAN_CHIP_TEST
    { "p2p_ps", wal_hipriv_set_p2p_ps }, /* ����P2P ���� sh hipriv.sh "vap0 p2p_ps noa/ops params */
#endif                                   /* #ifdef _PRE_WLAN_CHIP_TEST */
    /* sh hipriv.sh "vap0 p2p_ps ops 0/1(0��ʹ�ܣ�1ʹ��) [0~255] ����OPS ������ct_window ���� */
    /* sh hipriv.sh "vap0 p2p_ps noa start_time duration interval count ����NOA ���ܲ��� */
    /* sh hipriv.sh "vap0 p2p_ps statistics 0/1(0 ���ͳ�ƣ�1�鿴ͳ��) P2P �ж�ͳ�� */
#endif /* _PRE_WLAN_FEATURE_P2P */

#ifdef _PRE_WLAN_FEATURE_ARP_OFFLOAD
    { "arp_offload_enable", wal_hipriv_arp_offload_enable },     /* ARP/ND�������ƺ͹㲥/�鲥���˿���:sh hipriv.sh "wlan0 arp_offload_enable  0/1(0�رգ�1��)" */
    { "show_arpoffload_info", wal_hipriv_show_arpoffload_info }, /* ��ʾDevice���¼��IP��ַ:sh hipriv.sh "wlan0 show_ip_addr" */
#endif

#ifdef _PRE_WLAN_TCP_OPT
    /* ��ʾTCP ACK ����ͳ��ֵ sh hipriv.sh "vap0 get_tx_ack_stream_info */
    { "get_tcp_ack_stream_info", wal_hipriv_get_tcp_ack_stream_info },
    /* ���÷���TCP ACK�Ż�ʹ��  sh hipriv.sh "vap0 tcp_tx_ack_opt_enable 0 | 1 */
    { "tcp_tx_ack_opt_enable",   wal_hipriv_tcp_tx_ack_opt_enable },
    /* ���ý���TCP ACK�Ż�ʹ�� sh hipriv.sh "vap0 tcp_rx_ack_opt_enable 0 | 1 */
    { "tcp_rx_ack_opt_enable",   wal_hipriv_tcp_rx_ack_opt_enable },
    /* ���÷���TCP ACK LIMIT sh hipriv.sh "vap0 tcp_tx_ack_opt_limit X */
    { "tcp_tx_ack_opt_limit",    wal_hipriv_tcp_tx_ack_limit },
    /* ���ý���TCP ACKLIMIT  sh hipriv.sh "vap0 tcp_tx_ack_opt_limit X */
    { "tcp_rx_ack_opt_limit",    wal_hipriv_tcp_rx_ack_limit },

#endif

#ifdef _PRE_WLAN_FEATURE_WAPI
#ifdef _PRE_WAPI_DEBUG
    { "wapi_info", wal_hipriv_show_wapi_info }, /* wapi hipriv "vap0 wal_hipriv_show_wapi_info " */
#endif                                          /* #ifdef _PRE_DEBUG_MODE */
#endif                                          /* #ifdef _PRE_WLAN_FEATURE_WAPI */

#ifdef _PRE_WLAN_DFT_STAT
    /* �������ܴ�ӡ���ƿ��� sh hipriv.sh "wlan0 performance_log_debug X Y, */
    { "performance_log_debug", wal_hipriv_performance_log_switch },
    /* ����X�Ǵ�ӡ�㣬��oal_performance_log_switch_enum���壬Y��ʹ�ܿ���,0�رգ�1�򿪡� */ /* X=255ʱ���������еĴ�ӡ���� */
    /* ʹ��˵��:                                                     */
    /* sh hipriv.sh "wlan0 performance_log_debug 0 0 :�رվۺϴ�ӡ   */
    /* sh hipriv.sh "wlan0 performance_log_debug 0 1 :�򿪾ۺϴ�ӡ   */
    /* sh hipriv.sh "wlan0 performance_log_debug 1 0 :��ӡ����ͳ��   */
    /* sh hipriv.sh "wlan0 performance_log_debug 1 1 :�ۺ�ͳ����0    */
    /* sh hipriv.sh "wlan0 performance_log_debug 255 0 :������п��ƿ��� */
    /* sh hipriv.sh "wlan0 performance_log_debug 255 1 :�������п��ƿ��� */
#endif
#ifdef _PRE_WLAN_FEATURE_ROAM
    { "roam_network_thr", wal_hipriv_roam_homenetwork_thr }, /* ���ü�ͥ���γ��������� */
    { "roam_enable",      wal_hipriv_roam_enable },               /* �������ο��� */
    { "roam_org",         wal_hipriv_roam_org },                     /* ������������ */
    { "roam_band",        wal_hipriv_roam_band },                   /* ��������Ƶ�� */
    { "roam_start",       wal_hipriv_roam_start }, /* ���β������� sh hipriv.sh "wlan0 roam_start 0|1|2|3|4 0/1"
                                                            * 0���߲���ȱʧ��ʾ����ǰ��ɨ��, 1|2|3|4��ʾɨ��+����
                                                            */
    /* �ڶ�������0���߲���ȱʧ��ʾ���ε�����AP, 1��ʾ���ε��Լ� */
    { "roam_info", wal_hipriv_roam_info },                   /* ������Ϣ��ӡ */
#endif                                                       // _PRE_WLAN_FEATURE_ROAM
    { "2040bss_enable", wal_hipriv_enable_2040bss }, /* ����20/40 bssʹ��: hipriv "Hisilicon0 2040bss_enable 0|1" 0��ʾ20/40 bss�жϹرգ�1��ʾʹ�� */

#ifdef _PRE_WLAN_FIT_BASED_REALTIME_CALI
    { "dyn_cali", wal_hipriv_dyn_cali_cfg }, /*  ��̬У׼�������� sh hipriv "wlan0 dyn_cali   " */
#endif
#ifdef _PRE_WLAN_FEATURE_TX_CLASSIFY_LAN_TO_WLAN
    { "set_tx_classify_switch", wal_hipriv_set_tx_classify_switch }, /* ����ҵ��ʶ���ܿ���: sh hipriv.sh "p2p-p2p0-0 set_tx_classify_switch 1/0"(1�򿪣�0�رգ�����Ĭ�Ͽ���) */
#endif                                                               /* _PRE_WLAN_FEATURE_TX_CLASSIFY_LAN_TO_WLAN */

#ifdef _PRE_PLAT_FEATURE_CUSTOMIZE
    { "custom_info", wal_hipriv_dev_customize_info }, /* ��ӡ���ƻ���Ϣ */
#endif

#ifdef _PRE_WLAN_FEATURE_PACKET_CAPTURE
    { "packet_capture", wal_hipriv_packet_capture }, /* ������ر�ץ������: hipriv "Hisilicon0 packet_capture 0|1" */
#endif
#ifdef _PRE_WLAN_11K_STAT
    { "query_stat_info", wal_hipriv_query_stat_info }, /* ��ӡͳ����Ϣ */

#endif
#ifdef _PRE_WLAN_ONLINE_DPD
    { "dpd_cfg", wal_hipriv_dpd_cfg },     /* ��ӡ���ƻ���Ϣ */
    { "dpd_start", wal_hipriv_dpd_start }, /* ��ӡ���ƻ���Ϣ */
#endif

#ifdef _PRE_WLAN_FEATURE_11V
    { "11v_cfg_wl_mgmt", wal_hipriv_11v_cfg_wl_mgmt }, /* 11v wireless mgmt�������� sh hipriv.sh "vap0 11v_cfg_wl_mgmt 0|1"
                                                                        0:�ر�11V���ԣ�1:��11V���� */
#endif
#if (defined(_PRE_WLAN_FEATURE_11V) && defined(_PRE_DEBUG_MODE)) || defined(_PRE_WLAN_FEATURE_11V_ENABLE)
    { "11v_cfg_bsst", wal_hipriv_11v_cfg_bsst }, /* 11v bss transition�������� sh hipriv.sh "vap0 11v_cfg_bsst 0|1"
                                                                        0:�ر�11V���ԣ�1:��11V���� */
    { "11v_tx_query", wal_hipriv_11v_tx_query }, /* 11v��������:  ����sta����11v Query֡: hipriv "vap[x] 11v_tx_query [mac-addr]"   */
#endif

#ifdef _PRE_WLAN_FEATURE_DBDC
    { "dbdc_debug", wal_hipriv_set_dbdc_debug_switch }, /* DBDC���Կ��أ�sh hipriv.sh "wlan0 dbdc_debug [change_hal_dev 0|1] " */
#endif

#ifdef _PRE_MEM_TRACE
    { "mem_trace_show", wal_hipriv_mem_trace_info_show },
#endif

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
    { "pm_debug", wal_hipriv_set_pm_debug_switch }, /* �͹���debug���� */
#endif

    { "set_owe", wal_hipriv_set_owe }, /* ����APUTģʽ֧�ֵ�OWE group: hipriv "wlan0 set_owe <value>" */
    /* bit 0/1/2�ֱ��ʾ֧��owe group 19/20/21 */
    { "fem_lowpower", wal_hipriv_fem_lowpower }, /* fem�͹�������: sh hipriv.sh "wlan0 fem_lowpower 0|1 |2" */

    { "radar_set", wal_hipriv_radar_set }, /* �����״��¼� */

#ifdef _PRE_WLAN_FEATURE_IP_FILTER
    { "icmp_filter", wal_hipriv_set_icmp_filter }, /* icmp��������: sh hipriv.sh "wlan0 icmp_filter 1 0|1" */
#endif

#ifdef _PRE_WLAN_FEATURE_PSM_FLT_STAT
    { "psm_flt_stat", wal_hipriv_psm_flt_stat }, /* ��ӡ�͹���֡����ͳ�� sh hipriv.sh "wlan0 psm_flt_stat [0|1|2|...]" */
#endif

#ifdef _PRE_WLAN_FEATURE_HIEX
    /* ����ĳ���û�hiex������sh hipriv.sh "wlan0 set_user_himit xx:xx:xx:xx:xx:xx(mac��ַ) 0xXXX" */
    { "set_user_hiex", wal_hipriv_set_user_hiex_enable },
#endif
#ifdef _PRE_WLAN_FEATURE_GET_STATION_INFO_EXT
    { "get_station_info", wal_hipriv_get_station_info },
#endif
};

oal_uint32 wal_hipriv_get_debug_cmd_size_etc(oal_void)
{
    return OAL_ARRAY_SIZE(g_ast_hipriv_cmd_debug_etc);
}

#endif