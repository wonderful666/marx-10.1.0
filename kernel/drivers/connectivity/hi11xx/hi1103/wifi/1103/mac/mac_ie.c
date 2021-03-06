


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif


/*****************************************************************************
  1 头文件包含
*****************************************************************************/
#include "mac_ie.h"
#include "mac_frame.h"
#include "mac_device.h"
#include "securec.h"
#include "securectype.h"

#undef  THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_MAC_IE_C
/*****************************************************************************
  2 全局变量定义
*****************************************************************************/


oal_bool_enum_uint8 mac_ie_proc_ht_supported_channel_width_etc(
                                        mac_user_stru    *pst_mac_user_sta ,
                                        mac_vap_stru     *pst_mac_vap,
                                        oal_uint8         uc_supported_channel_width,
                                        oal_bool_enum     en_prev_asoc_ht)
{

    /* 不支持20/40Mhz频宽*/
    if (0 == uc_supported_channel_width)
    {
        if ((OAL_FALSE == en_prev_asoc_ht) || (OAL_TRUE == pst_mac_user_sta->st_ht_hdl.bit_supported_channel_width))
        {
            pst_mac_vap->st_protection.uc_sta_20M_only_num++;
        }

        return OAL_FALSE;
    }
    else/* 支持20/40Mhz频宽 */
    {
        /*  如果STA之前已经作为不支持20/40Mhz频宽的HT站点与AP关联*/
        if ((OAL_TRUE == en_prev_asoc_ht) && (OAL_FALSE == pst_mac_user_sta->st_ht_hdl.bit_supported_channel_width))
        {
            pst_mac_vap->st_protection.uc_sta_20M_only_num--;
        }

        return OAL_TRUE;
    }
}


oal_bool_enum_uint8 mac_ie_proc_ht_green_field_etc(
                                        mac_user_stru    *pst_mac_user_sta ,
                                        mac_vap_stru     *pst_mac_vap,
                                        oal_uint8         uc_ht_green_field,
                                        oal_bool_enum     en_prev_asoc_ht)
{
    /* 不支持Greenfield */
    if (0 == uc_ht_green_field)
    {
        if ((OAL_FALSE == en_prev_asoc_ht ) || (OAL_TRUE == pst_mac_user_sta->st_ht_hdl.bit_ht_green_field))
        {
            pst_mac_vap->st_protection.uc_sta_non_gf_num++;
        }

        return OAL_FALSE;
    }
    else/* 支持Greenfield */
    {
        /*  如果STA之前已经作为不支持GF的HT站点与AP关联*/
        if ((OAL_TRUE == en_prev_asoc_ht ) && (OAL_FALSE == pst_mac_user_sta->st_ht_hdl.bit_ht_green_field))
        {
            pst_mac_vap->st_protection.uc_sta_non_gf_num--;
        }

        return OAL_TRUE;
    }
}


oal_bool_enum_uint8 mac_ie_proc_lsig_txop_protection_support_etc(
                                        mac_user_stru    *pst_mac_user_sta,
                                        mac_vap_stru     *pst_mac_vap,
                                        oal_uint8         uc_lsig_txop_protection_support,
                                        oal_bool_enum     en_prev_asoc_ht)
{
    /* 不支持L-sig txop protection */
    if (0 == uc_lsig_txop_protection_support)
    {
        if ((OAL_FALSE == en_prev_asoc_ht) || (OAL_TRUE == pst_mac_user_sta->st_ht_hdl.bit_lsig_txop_protection))
        {
            pst_mac_vap->st_protection.uc_sta_no_lsig_txop_num++;
        }

        return OAL_FALSE;
    }
    else /* 支持L-sig txop protection */
    {
        /*  如果STA之前已经作为不支持Lsig txop protection的HT站点与AP关联*/
        if ((OAL_TRUE == en_prev_asoc_ht ) && (OAL_FALSE == pst_mac_user_sta->st_ht_hdl.bit_lsig_txop_protection))
        {
            pst_mac_vap->st_protection.uc_sta_no_lsig_txop_num--;
        }

        return OAL_TRUE;
    }
}


oal_uint32  mac_ie_proc_ht_sta_etc(
                   mac_vap_stru            *pst_mac_sta,
                   oal_uint8                *puc_payload,
                   oal_uint16                us_offset,
                   mac_user_stru           *pst_mac_user_ap,
                   oal_uint16               *pus_amsdu_maxsize)
{
    oal_uint8                           uc_mcs_bmp_index;
    oal_uint8                           uc_smps;
    mac_user_ht_hdl_stru               *pst_ht_hdl = OAL_PTR_NULL;
    mac_user_ht_hdl_stru                st_ht_hdl;
    oal_uint16                          us_tmp_info_elem;
    oal_uint16                          us_tmp_txbf_low;
    oal_uint32                          ul_tmp_txbf_elem;
    oal_uint16                          us_ht_cap_info = 0;

    if ((OAL_PTR_NULL == pst_mac_sta) || (OAL_PTR_NULL == puc_payload)
         || (OAL_PTR_NULL == pst_mac_user_ap)
        || (OAL_PTR_NULL == pus_amsdu_maxsize))
    {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{mac_ie_proc_ht_sta_etc::param null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_ht_hdl      = &st_ht_hdl;
    mac_user_get_ht_hdl_etc(pst_mac_user_ap, pst_ht_hdl);

    /* 带有 HT Capability Element 的 AP，标示它具有HT capable. */
    pst_ht_hdl->en_ht_capable = OAL_TRUE;

    us_offset += MAC_IE_HDR_LEN;

    /********************************************/
    /*     解析 HT Capabilities Info Field      */
    /********************************************/
    us_ht_cap_info = OAL_MAKE_WORD16(puc_payload[us_offset], puc_payload[us_offset + 1]);

    /* 检查STA所支持的LDPC编码能力 B0，0:不支持，1:支持 */
    pst_ht_hdl->bit_ldpc_coding_cap = (us_ht_cap_info & BIT0);

    /* 提取AP所支持的带宽能力  */
    pst_ht_hdl->bit_supported_channel_width = ((us_ht_cap_info & BIT1) >> 1);

    /* 检查空间复用节能模式 B2~B3 */
    uc_smps = ((us_ht_cap_info & (BIT3 | BIT2)) >> 2);
    pst_ht_hdl->bit_sm_power_save = mac_ie_proc_sm_power_save_field_etc(pst_mac_user_ap, uc_smps);

    /* 提取AP支持Greenfield情况 */
    pst_ht_hdl->bit_ht_green_field = ((us_ht_cap_info & BIT4) >> 4);

    /* 提取AP支持20MHz Short-GI情况 */
    pst_ht_hdl->bit_short_gi_20mhz = ((us_ht_cap_info & BIT5) >> 5);

    /* 提取AP支持40MHz Short-GI情况 */
    pst_ht_hdl->bit_short_gi_40mhz = ((us_ht_cap_info & BIT6) >> 6);

    /* 提取AP支持STBC PPDU情况 */
    pst_ht_hdl->bit_rx_stbc = (oal_uint8)((us_ht_cap_info & (BIT9 | BIT8)) >> 8);

    /* 提取AP支持最大A-MSDU长度情况 */
    if(0 == (us_ht_cap_info & BIT11))
    {
        *pus_amsdu_maxsize = WLAN_MIB_MAX_AMSDU_LENGTH_SHORT;
    }
    else
    {
        *pus_amsdu_maxsize = WLAN_MIB_MAX_AMSDU_LENGTH_LONG;
    }

    /* 提取AP 40M上DSSS/CCK的支持情况 */
    pst_ht_hdl->bit_dsss_cck_mode_40mhz = ((us_ht_cap_info & BIT12) >> 12);

    /* 提取AP L-SIG TXOP 保护的支持情况 */
    pst_ht_hdl->bit_lsig_txop_protection = ((us_ht_cap_info & BIT15) >> 15);

    us_offset += MAC_HT_CAPINFO_LEN;

    /********************************************/
    /*     解析 A-MPDU Parameters Field         */
    /********************************************/

    /* 提取 Maximum Rx A-MPDU factor (B1 - B0) */
    pst_ht_hdl->uc_max_rx_ampdu_factor = (puc_payload[us_offset] & 0x03);

    /* 提取 Minmum Rx A-MPDU factor (B3 - B2) */
    pst_ht_hdl->uc_min_mpdu_start_spacing = (puc_payload[us_offset] >> 2) & 0x07;

    us_offset += MAC_HT_AMPDU_PARAMS_LEN;

    /********************************************/
    /*     解析 Supported MCS Set Field         */
    /********************************************/
    for(uc_mcs_bmp_index = 0; uc_mcs_bmp_index < WLAN_HT_MCS_BITMASK_LEN; uc_mcs_bmp_index++)
    {
        pst_ht_hdl->uc_rx_mcs_bitmask[uc_mcs_bmp_index] = (*(oal_uint8 *)(puc_payload + us_offset + uc_mcs_bmp_index));


    }

    pst_ht_hdl->uc_rx_mcs_bitmask[WLAN_HT_MCS_BITMASK_LEN - 1] &= 0x1F;

    us_offset += MAC_HT_SUP_MCS_SET_LEN;

    /********************************************/
    /* 解析 HT Extended Capabilities Info Field */
    /********************************************/
    us_ht_cap_info = OAL_MAKE_WORD16(puc_payload[us_offset], puc_payload[us_offset + 1]);

    /* 提取 HTC support Information */
    if ((us_ht_cap_info & BIT10) != 0)
    {
        pst_ht_hdl->uc_htc_support = 1;
    }
    us_offset += MAC_HT_EXT_CAP_LEN;

    /********************************************/
    /*  解析 Tx Beamforming Field               */
    /********************************************/
    us_tmp_info_elem = OAL_MAKE_WORD16(puc_payload[us_offset], puc_payload[us_offset + 1]);
    us_tmp_txbf_low	 = OAL_MAKE_WORD16(puc_payload[us_offset + 2], puc_payload[us_offset + 3]);
	ul_tmp_txbf_elem = OAL_MAKE_WORD32(us_tmp_info_elem, us_tmp_txbf_low);
    pst_ht_hdl->bit_imbf_receive_cap				= (ul_tmp_txbf_elem & BIT0);
    pst_ht_hdl->bit_receive_staggered_sounding_cap  = ((ul_tmp_txbf_elem & BIT1) >> 1);
    pst_ht_hdl->bit_transmit_staggered_sounding_cap = ((ul_tmp_txbf_elem & BIT2) >> 2);
    pst_ht_hdl->bit_receive_ndp_cap					= ((ul_tmp_txbf_elem & BIT3) >> 3);
    pst_ht_hdl->bit_transmit_ndp_cap				= ((ul_tmp_txbf_elem & BIT4) >> 4);
    pst_ht_hdl->bit_imbf_cap						= ((ul_tmp_txbf_elem & BIT5) >> 5);
    pst_ht_hdl->bit_calibration						= ((ul_tmp_txbf_elem & 0x000000C0) >> 6);
    pst_ht_hdl->bit_exp_csi_txbf_cap				= ((ul_tmp_txbf_elem & BIT8) >> 8);
    pst_ht_hdl->bit_exp_noncomp_txbf_cap			= ((ul_tmp_txbf_elem & BIT9) >> 9);
    pst_ht_hdl->bit_exp_comp_txbf_cap				= ((ul_tmp_txbf_elem & BIT10) >> 10);
    pst_ht_hdl->bit_exp_csi_feedback				= ((ul_tmp_txbf_elem & 0x00001800) >> 11);
    pst_ht_hdl->bit_exp_noncomp_feedback			= ((ul_tmp_txbf_elem & 0x00006000) >> 13);
    pst_ht_hdl->bit_exp_comp_feedback				= ((ul_tmp_txbf_elem & 0x0001C000) >> 15);
    pst_ht_hdl->bit_min_grouping					= ((ul_tmp_txbf_elem & 0x00060000) >> 17);
    pst_ht_hdl->bit_csi_bfer_ant_number				= ((ul_tmp_txbf_elem & 0x001C0000) >> 19);
    pst_ht_hdl->bit_noncomp_bfer_ant_number			= ((ul_tmp_txbf_elem & 0x00600000) >> 21);
    pst_ht_hdl->bit_comp_bfer_ant_number			= ((ul_tmp_txbf_elem & 0x01C00000) >> 23);
    pst_ht_hdl->bit_csi_bfee_max_rows				= ((ul_tmp_txbf_elem & 0x06000000) >> 25);
    pst_ht_hdl->bit_channel_est_cap					= ((ul_tmp_txbf_elem & 0x18000000) >> 27);

    mac_user_set_ht_hdl_etc(pst_mac_user_ap, pst_ht_hdl);

    return OAL_SUCC;
}


oal_bool_enum_uint8 mac_ie_check_p2p_action_etc(oal_uint8 *puc_payload)
{
    /* 找到WFA OUI */
    if ((0 == oal_memcmp(puc_payload, g_auc_p2p_oui_etc, MAC_OUI_LEN)) &&
        (MAC_OUITYPE_P2P == puc_payload[MAC_OUI_LEN]))
    {
        /*  找到WFA P2P v1.0 oui type */
        return OAL_TRUE;
    }

    return OAL_FALSE;
}

/*****************************************************************************
 函 数 名  : mac_ie_check_rsn_cipher_format
 功能描述  : 检查rsnie的报文格式
 输入参数  : oal_uint8                   * puc_src_ie
             oal_uint8                     uc_ie_len
 输出参数  : 无
 返 回 值  : oal_uint32
 调用函数  :
 被调函数  :
 修改历史      :
   修改内容   : 新生成函数
*****************************************************************************/
oal_uint32 mac_ie_check_rsn_cipher_format(oal_uint8 *puc_src_ie, oal_uint8 uc_ie_len)
{
    oal_uint8               *puc_end_ie;
    oal_uint8               *puc_tmp_ie;
    oal_uint16               us_pairwise_suite_count;
    /* oal_uint16               us_akm_suite_count; */  /* don't need to check AKM */

    /* 前置条件， RSN-IE的length刚好等于puc_ie缓冲区长度 */
    puc_end_ie = (oal_uint8 *)puc_src_ie + uc_ie_len;
    puc_tmp_ie = puc_src_ie + 6;
    /* Pairwise Cipher Suite List */
    if (puc_tmp_ie+2 >= puc_end_ie)
    {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{mac_ie_check_rsn_cipher_format, Pairwise Cipher count fail.}");
        return  OAL_ERR_CODE_MSG_LENGTH_ERR;
    }

    us_pairwise_suite_count = OAL_MAKE_WORD16(puc_tmp_ie[0], puc_tmp_ie[1]);
    puc_tmp_ie += 2 + us_pairwise_suite_count * 4;
    /* AKM Suite List */
    if (puc_tmp_ie+2 >= puc_end_ie)
    {
        OAM_WARNING_LOG0(0, OAM_SF_CFG, "{mac_ie_check_rsn_cipher_format, Pairwise Cipher fail.}");
        return  OAL_ERR_CODE_MSG_LENGTH_ERR;
    }

    return OAL_SUCC;
}

#ifdef _PRE_WLAN_FEATURE_SMPS

oal_uint32 mac_smps_update_user_status(mac_vap_stru *pst_mac_vap, mac_user_stru *pst_mac_user)
{
    wlan_mib_mimo_power_save_enum_uint8 en_user_smps_mode;

    if ((OAL_PTR_NULL == pst_mac_vap) || (OAL_PTR_NULL == pst_mac_user))
    {
        OAM_ERROR_LOG2(0, OAM_SF_SMPS, "{mac_smps_update_user_status: NULL PTR pst_mac_vap is [%x] and pst_mac_user is [%x].}", (uintptr_t)pst_mac_vap, (uintptr_t)pst_mac_user);
        return OAL_ERR_CODE_PTR_NULL;
    }

    en_user_smps_mode = (wlan_mib_mimo_power_save_enum_uint8)pst_mac_user->st_ht_hdl.bit_sm_power_save;

    switch(en_user_smps_mode)
    {
        case WLAN_MIB_MIMO_POWER_SAVE_STATIC:
            mac_user_set_sm_power_save(pst_mac_user, WLAN_MIB_MIMO_POWER_SAVE_STATIC);
            pst_mac_user->en_avail_num_spatial_stream = WLAN_SINGLE_NSS;
            break;
        case WLAN_MIB_MIMO_POWER_SAVE_DYNAMIC:
            OAM_WARNING_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_SMPS, "{mac_smps_update_user_status:user smps_mode update DYNAMIC!}");
            mac_user_set_sm_power_save(pst_mac_user, WLAN_MIB_MIMO_POWER_SAVE_DYNAMIC);
            pst_mac_user->en_avail_num_spatial_stream = OAL_MIN(pst_mac_vap->en_vap_rx_nss,WLAN_DOUBLE_NSS);
            break;
        case WLAN_MIB_MIMO_POWER_SAVE_MIMO:
            mac_user_set_sm_power_save(pst_mac_user, WLAN_MIB_MIMO_POWER_SAVE_MIMO);
            pst_mac_user->en_avail_num_spatial_stream = OAL_MIN(pst_mac_vap->en_vap_rx_nss,WLAN_DOUBLE_NSS);
            break;
        default:
            OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_SMPS, "{mac_smps_update_user_status: en_user_smps_mode mode[%d] fail!}", en_user_smps_mode);
            return OAL_FAIL;
    }

    return OAL_SUCC;
}
#endif



wlan_mib_mimo_power_save_enum_uint8 mac_ie_proc_sm_power_save_field_etc(mac_user_stru *pst_mac_user, oal_uint8 uc_smps)
{
    if (MAC_SMPS_STATIC_MODE == uc_smps)
    {
        return WLAN_MIB_MIMO_POWER_SAVE_STATIC;
    }
    else if (MAC_SMPS_DYNAMIC_MODE == uc_smps)
    {
        return WLAN_MIB_MIMO_POWER_SAVE_DYNAMIC;
    }
    else
    {
        return WLAN_MIB_MIMO_POWER_SAVE_MIMO;
    }

}
#ifdef _PRE_WLAN_FEATURE_OPMODE_NOTIFY

oal_uint32  mac_check_is_assoc_frame_etc(oal_uint8 uc_mgmt_frm_type)
{
    if ((uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_ASSOC_RSP) ||
        (uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_REASSOC_REQ) ||
        (uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_REASSOC_RSP) ||
        (uc_mgmt_frm_type == WLAN_FC0_SUBTYPE_ASSOC_REQ))
    {
        return OAL_TRUE;
    }

    return OAL_FALSE;
}
#endif









#ifdef _PRE_WLAN_FEATURE_11AX

oal_uint32  mac_ie_parse_he_cap(oal_uint8 *puc_he_cap_ie, mac_frame_he_cap_ie_stru *pst_he_cap_value)
{
    oal_uint8                          *puc_he_buffer = OAL_PTR_NULL;
    mac_frame_he_mac_cap_stru          *pst_he_mac_cap = OAL_PTR_NULL;
    mac_frame_he_phy_cap_stru          *pst_he_phy_cap = OAL_PTR_NULL;
    oal_uint8                           uc_mcs_nss_set_size = 2;
    mac_fram_he_mac_nsss_set_stru      *pst_mac_nss_set = OAL_PTR_NULL;
    oal_int32                           l_ret;

    /* 解析he cap IE */
    if (OAL_ANY_NULL_PTR2(puc_he_cap_ie,pst_he_cap_value))
    {
        OAM_ERROR_LOG2(0, OAM_SF_11AX, "{mac_ie_parse_he_cap::param null,puc_he_cap_ie[0x%x], pst_he_cap_value[0x%x].}", (uintptr_t)puc_he_cap_ie, (uintptr_t)pst_he_cap_value);

        return OAL_ERR_CODE_PTR_NULL;
    }

    /***************************************************************************
    -------------------------------------------------------------------------
    |EID |Length |EID Extension|HE MAC Capa. Info |HE PHY Capa. Info|
    -------------------------------------------------------------------------
    |1   |1      |1            |6                 |9                |
    -------------------------------------------------------------------------
    |Tx Rx HE MCS NSS Support |PPE Thresholds(Optional)|
    -------------------------------------------------------------------------
    |4or8or12                 |Variable                |
    -------------------------------------------------------------------------
    ***************************************************************************/
    if (puc_he_cap_ie[1] < MAC_HE_CAP_MIN_LEN)
    {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{hmac_proc_he_cap_ie::invalid he cap ie len[%d].}", puc_he_cap_ie[1]);
        return OAL_FAIL;
    }

    puc_he_buffer = puc_he_cap_ie + 3;

    /*mac cap */
    pst_he_mac_cap = (mac_frame_he_mac_cap_stru *)puc_he_buffer;
    l_ret = memcpy_s(&pst_he_cap_value->st_he_mac_cap, OAL_SIZEOF(mac_frame_he_mac_cap_stru),
                     pst_he_mac_cap, OAL_SIZEOF(mac_frame_he_mac_cap_stru));
    puc_he_buffer +=  OAL_SIZEOF(mac_frame_he_mac_cap_stru);

    /*PHY Cap */
    pst_he_phy_cap = (mac_frame_he_phy_cap_stru *)puc_he_buffer;
    l_ret += memcpy_s(&pst_he_cap_value->st_he_phy_cap, OAL_SIZEOF(mac_frame_he_phy_cap_stru),
                      pst_he_phy_cap, OAL_SIZEOF(mac_frame_he_phy_cap_stru));
    puc_he_buffer +=  OAL_SIZEOF(mac_frame_he_phy_cap_stru);

    /* 解析Support HE-MCS NSS Set */
    /******************************** HE Supported HE-MCS And NSS Set *********************************
    |-------------------------------------------------------------------------------------------------|
    | Rx HE-MCS Map | Tx HE-MCS Map | Rx HE-MCS Map  | Tx HE-MCS Map  | Rx HE-MCS Map | Tx HE-MCS Map |
    | <= 80 MHz     | <= 80 MHz     | 160 MHz        | 160 MHz        | 80+80 MHz     | 80+80 MHz     |
    |-------------------------------------------------------------------------------------------------|
    | 2 Octets      | 2 Octets      | 0 or 2 Octets  | 0 or 2 Octets  | 0 or 2 Octets | 0 or 2 Octets |
    |-------------------------------------------------------------------------------------------------|
    **************************************************************************************************/
    /*
     * 1. HE PHY Capabilities Info中Channel Width Set字段bit2为1时，
     * HE Supported HE-MCS And NSS Set中存在Rx HE-MCS Map 160 MHz和Tx HE-MCS Map 160 MHz字段
     * 2. HE PHY Capabilities Info中Channel Width Set字段bit3为1时，
     * HE Supported HE-MCS And NSS Set中存在Rx HE-MCS Map 80+80 MHz和Tx HE-MCS Map 80+80 MHz字段
     */

    pst_mac_nss_set = (mac_fram_he_mac_nsss_set_stru *)puc_he_buffer;
    if (0 != (pst_he_cap_value->st_he_phy_cap.bit_channel_width_set & BIT2))
    {
        uc_mcs_nss_set_size += 2;
    }
    if (0 != (pst_he_cap_value->st_he_phy_cap.bit_channel_width_set & BIT3))
    {
        uc_mcs_nss_set_size += 2;
    }

    /* AP只需记录对端发过来的值 */
    l_ret += memcpy_s((oal_uint8 *)(&pst_he_cap_value->st_he_mcs_nss),
                      sizeof(mac_fram_he_mac_nsss_set_stru),
                      (oal_uint8 *)pst_mac_nss_set,
                      uc_mcs_nss_set_size * sizeof(mac_frame_he_mcs_nss_bit_map_stru));
    if (l_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "mac_ie_parse_he_cap::memcpy fail!");
        return OAL_FAIL;
    }

    /* TODO:PPE thresholds 暂不解析 */
    return OAL_SUCC;
}


/*lint -save -e438 */
oal_uint32  mac_ie_parse_he_oper(oal_uint8 *puc_he_oper_ie, mac_frame_he_oper_ie_stru *pst_he_oper_ie_value)
{
    oal_uint8                          *puc_ie_buffer = OAL_PTR_NULL;
    mac_frame_he_operation_param_stru  *pst_he_oper_param = OAL_PTR_NULL;
    mac_frame_he_mcs_nss_bit_map_stru  *pst_he_basic_mcs_nss = OAL_PTR_NULL;
    mac_frame_vht_operation_info_stru  *pst_vht_operation_info = OAL_PTR_NULL;
    mac_frame_he_bss_color_info_stru   *pst_bss_color_info = OAL_PTR_NULL;
    oal_int32                           l_ret;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(puc_he_oper_ie,pst_he_oper_ie_value)))
    {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "{mac_ie_parse_he_oper::param null.}");

        return OAL_ERR_CODE_PTR_NULL;
    }

    /***************************************************************************
    -------------------------------------------------------------------------
    |EID |Length |EID Extension|HE Operation Parameters|Basic HE MCS Ans NSS Set|
    -------------------------------------------------------------------------
    |1   |1          |1                  |           4                     |              3                       |
    -------------------------------------------------------------------------
    |VHT Operation Info  |MaxBssid Indicator|
    -------------------------------------------------------------------------
    |      0 or 3              |0 or More             |
    -------------------------------------------------------------------------
    ***************************************************************************/
    if(puc_he_oper_ie[1] < MAC_HE_OPERAION_MIN_LEN)
    {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{mac_ie_parse_he_oper::invalid he oper ie len[%d].}", puc_he_oper_ie[1]);
        return OAL_FAIL;
    }

    puc_ie_buffer = puc_he_oper_ie + 3;

    /*解析HE Operation Parameters*/
    pst_he_oper_param = (mac_frame_he_operation_param_stru *)puc_ie_buffer;
    puc_ie_buffer    += MAC_HE_OPE_PARAM_LEN;
    l_ret = memcpy_s((oal_void *)&pst_he_oper_ie_value->st_he_oper_param, sizeof(mac_frame_he_operation_param_stru),
                     (void *)pst_he_oper_param, MAC_HE_OPE_PARAM_LEN);

    pst_bss_color_info = (mac_frame_he_bss_color_info_stru *)puc_ie_buffer;
    puc_ie_buffer     += OAL_SIZEOF(mac_frame_he_bss_color_info_stru);
    l_ret += memcpy_s((oal_void *)&pst_he_oper_ie_value->st_bss_color, OAL_SIZEOF(mac_frame_he_bss_color_info_stru),
                      (void *)pst_bss_color_info, OAL_SIZEOF(mac_frame_he_bss_color_info_stru));

    /*解析Basic HE MCS And NSS Set*/
    pst_he_basic_mcs_nss = (mac_frame_he_mcs_nss_bit_map_stru *)puc_ie_buffer;
    puc_ie_buffer       += MAC_HE_OPE_BASIC_MCS_NSS_LEN;
    l_ret += memcpy_s((oal_uint8 *)(&pst_he_oper_ie_value->st_he_basic_mcs_nss),
                      OAL_SIZEOF(mac_frame_he_mcs_nss_bit_map_stru), pst_he_basic_mcs_nss,
                      OAL_SIZEOF(mac_frame_he_mcs_nss_bit_map_stru));

    if(1 == pst_he_oper_ie_value->st_he_oper_param.bit_vht_operation_info_present)
    {
        puc_ie_buffer = puc_ie_buffer + 3;
        pst_vht_operation_info = (mac_frame_vht_operation_info_stru *)puc_ie_buffer;
        puc_ie_buffer += MAC_HE_VHT_OPERATION_INFO_LEN;
        l_ret += memcpy_s((oal_uint8 *)(&pst_he_oper_ie_value->st_vht_operation_info),
                          OAL_SIZEOF(mac_frame_vht_operation_info_stru), pst_vht_operation_info,
                          OAL_SIZEOF(mac_frame_vht_operation_info_stru));
    }
    if (l_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "mac_ie_parse_he_oper::memcpy fail!");
        return OAL_FAIL;
    }

    /*MaxBssid Indicator*/
    //TODO
    return OAL_SUCC;
}
/*lint -restore */


oal_uint32  mac_ie_parse_mu_edca_parameter(oal_uint8 *puc_he_edca_ie,mac_frame_he_mu_edca_parameter_ie_stru *pst_he_mu_edca_value)
{
    mac_frame_he_mu_edca_parameter_ie_stru *pst_he_edca = OAL_PTR_NULL;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(puc_he_edca_ie,pst_he_mu_edca_value)))
    {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "{mac_ie_parse_mu_edca_parameter::param null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    /************************ MU EDCA Parameter Set Element ***************************/
    /* ------------------------------------------------------------------------------------------- */
    /* | EID | LEN | Ext EID|MU Qos Info |MU AC_BE Parameter Record | MU AC_BK Parameter Record  | */
    /* ------------------------------------------------------------------------------------------- */
    /* |  1  |  1  |   1    |    1       |     3                    |        3                   | */
    /* ------------------------------------------------------------------------------------------- */
    /* ------------------------------------------------------------------------------ -------------*/
    /* | MU AC_VI Parameter Record | MU AC_VO Parameter Record                                   | */
    /* ------------------------------------------------------------------------------------------- */
    /* |    3                      |     3                                                       | */

    /******************* QoS Info field when sent from WMM AP *****************/
    /* --------------------------------------------------------------------------------------------*/
    /*    | EDCA Parameter Set Update Count | Q-Ack | Queue Request |TXOP Request | More Data Ack| */
    /*---------------------------------------------------------------------------------------------*/
    /*bit |        0~3                      |  1    |  1            |   1         |     1        | */
    /*---------------------------------------------------------------------------------------------*/
    /**************************************************************************/

    if(puc_he_edca_ie[1] !=  MAC_HE_MU_EDCA_PARAMETER_SET_LEN)
    {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{mac_ie_parse_mu_edca_parameter::invalid mu edca ie len[%d].}", puc_he_edca_ie[1]);
        return OAL_FAIL;
    }

    puc_he_edca_ie = puc_he_edca_ie + 3;

    /*解析HE MU EDCA  Parameters Set Element*/
    pst_he_edca = (mac_frame_he_mu_edca_parameter_ie_stru *)puc_he_edca_ie;
    if (EOK != memcpy_s((oal_uint8 *)(pst_he_mu_edca_value),
                        OAL_SIZEOF(mac_frame_he_mu_edca_parameter_ie_stru), pst_he_edca,
                        OAL_SIZEOF(mac_frame_he_mu_edca_parameter_ie_stru))) {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "mac_ie_parse_mu_edca_parameter::memcpy fail!");
        return OAL_FAIL;
    }

    return OAL_SUCC;
}


/*lint -save -e438 */
oal_uint32 mac_ie_parse_spatial_reuse_parameter(oal_uint8 *puc_he_srp_ie,mac_frame_he_spatial_reuse_parameter_set_ie_stru *pst_he_srp_value)
{
    oal_uint8                      *puc_he_buffer = OAL_PTR_NULL;
    mac_frame_he_sr_control_stru   *pst_he_sr_control = OAL_PTR_NULL;
    oal_int32                       l_ret;

    if (OAL_ANY_NULL_PTR2(puc_he_srp_ie,pst_he_srp_value))
    {
        OAM_ERROR_LOG2(0, OAM_SF_11AX, "{mac_ie_parse_spatial_reuse_parameter::param null,puc_he_srp_ie[0x%x], pst_he_srp_value[0x%x].}", (uintptr_t)puc_he_srp_ie, (uintptr_t)pst_he_srp_value);
        return OAL_ERR_CODE_PTR_NULL;
    }

    /************************ Spatial Reuse Parameter Set Element ***************************/
    /* ------------------------------------------------------------------------------ */
    /* | EID | LEN | Ext EID|SR Control |Non-SRG OBSS PD Max Offset | SRG OBSS PD Min Offset  | */
    /* ------------------------------------------------------------------------------ */
    /* |  1   |  1   |   1       |    1          |     0 or 1                             |        0 or 1                      | */
    /* ------------------------------------------------------------------------------ */
    /* ------------------------------------------------------------------------------ */
    /* |SRG OBSS PD Max Offset |SRG BSS Color Bitmap  | SRG Partial BSSID Bitmap |*/
    /* ------------------------------------------------------------------------------ */
    /* |    0 or 1                       |     0 or 8                    |      0 or 8                       |*/
    /*--------------------------------------------------------------------------------*/
    /**************************************************************************/

    puc_he_buffer = puc_he_srp_ie + 3;

    /*SR Control*/
    pst_he_sr_control = (mac_frame_he_sr_control_stru *)puc_he_buffer;
    puc_he_buffer += OAL_SIZEOF(mac_frame_he_sr_control_stru);
    l_ret = memcpy_s((oal_uint8 *)(&pst_he_srp_value->st_sr_control), OAL_SIZEOF(mac_frame_he_sr_control_stru),
                     (oal_uint8 *)pst_he_sr_control, OAL_SIZEOF(mac_frame_he_sr_control_stru));

    if(1 == pst_he_sr_control->bit_non_srg_offset_present)
    {
        /*Non-SRG OBSS PD Max Offset  */
        l_ret += memcpy_s((oal_uint8 *)(&pst_he_srp_value->uc_non_srg_obss_pd_max_offset), OAL_SIZEOF(oal_uint8),
                         (oal_uint8 *)puc_he_buffer, OAL_SIZEOF(oal_uint8));
        puc_he_buffer += 1;
    }

    if(1 == pst_he_sr_control->bit_srg_information_present)
    {
        /*SRG OBSS PD Min Offset*/
        l_ret += memcpy_s((oal_uint8 *)(&pst_he_srp_value->uc_srg_obss_pd_min_offset), OAL_SIZEOF(oal_uint8),
                          (oal_uint8 *)puc_he_buffer, OAL_SIZEOF(oal_uint8));
        puc_he_buffer += 1;

        /*SRG OBSS PD Max Offset*/
        l_ret += memcpy_s((oal_uint8 *)(&pst_he_srp_value->uc_srg_obss_pd_max_offset), OAL_SIZEOF(oal_uint8),
                          (oal_uint8 *)puc_he_buffer, OAL_SIZEOF(oal_uint8));
        puc_he_buffer += 1;

        /*SRG BSS Color Bitmap*/
        l_ret += memcpy_s((oal_uint8 *)(&pst_he_srp_value->auc_srg_bss_color_bitmap), MAC_HE_SRG_BSS_COLOR_BITMAP_LEN,
                          (oal_uint8 *)puc_he_buffer, MAC_HE_SRG_BSS_COLOR_BITMAP_LEN);
        puc_he_buffer += MAC_HE_SRG_BSS_COLOR_BITMAP_LEN;

        /*SRG BSS Color Bitmap*/
        l_ret += memcpy_s((oal_uint8 *)(&pst_he_srp_value->auc_srg_partial_bssid_bitmap), MAC_HE_SRG_PARTIAL_BSSID_BITMAP_LEN,
                          (oal_uint8 *)puc_he_buffer, MAC_HE_SRG_PARTIAL_BSSID_BITMAP_LEN);
        puc_he_buffer += MAC_HE_SRG_PARTIAL_BSSID_BITMAP_LEN;

    }
    if (l_ret != EOK) {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "mac_ie_parse_spatial_reuse_parameter::memcpy fail!");
        return OAL_FAIL;
    }

    return OAL_SUCC;
}
/*lint -restore */


oal_uint32  mac_ie_proc_he_opern_ie(mac_vap_stru *pst_mac_vap,oal_uint8 *puc_payload,mac_user_stru *pst_mac_user)
{
    mac_frame_he_oper_ie_stru          st_he_oper_ie_value;
    mac_he_hdl_stru                    st_he_hdl;
    //wlan_mib_vht_op_width_enum_uint8   en_channel_width_old;
    //oal_uint8                          uc_channel_center_freq_seg0_old;
    oal_uint32                         ul_ret  = MAC_NO_CHANGE;

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR3(pst_mac_vap,pst_mac_user,puc_payload)))
    {
        OAM_ERROR_LOG3(0, OAM_SF_11AX, "{mac_ie_proc_he_opern_ie::param null,%X %X %X.}", (uintptr_t)pst_mac_vap, (uintptr_t)pst_mac_user, (uintptr_t)puc_payload);
        return MAC_NO_CHANGE;
    }

    memset_s(&st_he_oper_ie_value, OAL_SIZEOF(st_he_oper_ie_value), 0, OAL_SIZEOF(st_he_oper_ie_value));
    ul_ret = mac_ie_parse_he_oper(puc_payload, &st_he_oper_ie_value);
    if(OAL_SUCC != ul_ret)
    {
        return MAC_NO_CHANGE;
    }

    mac_user_get_he_hdl(pst_mac_user, &st_he_hdl);

    /* 解析到he_opern_ie，即存在he_duration_rts_threshold */
    st_he_hdl.bit_he_duration_rts_threshold_exist = 1;

    if(st_he_oper_ie_value.st_bss_color.bit_bss_color != st_he_hdl.st_he_oper_ie.st_bss_color.bit_bss_color)
    {
        ul_ret |= MAC_HE_BSS_COLOR_CHANGE;
        /* 识别bss color需要标记,dmac设置 */
        st_he_hdl.bit_he_oper_bss_color_exist = OAL_TRUE;
    }

    if(st_he_oper_ie_value.st_bss_color.bit_partial_bss_color != st_he_hdl.st_he_oper_ie.st_bss_color.bit_partial_bss_color)
    {
        ul_ret |= MAC_HE_PARTIAL_BSS_COLOR_CHANGE;
    }

    st_he_hdl.st_he_oper_ie               = st_he_oper_ie_value;

    mac_user_set_he_hdl(pst_mac_user,&st_he_hdl);

    return ul_ret;
}


oal_uint32  mac_ie_parse_he_ndp_feedback_report_ie(oal_uint8 *puc_he_ndp_ie, oal_uint8 *puc_resource_req_buff_threshold_exp)
{
    oal_uint8 *puc_data = OAL_PTR_NULL;

    if (MAC_HE_NDP_FEEDBACK_REPORT_LEN != puc_he_ndp_ie[1])
    {
        return OAL_FAIL;
    }

    puc_data = puc_he_ndp_ie + 3;

    *puc_resource_req_buff_threshold_exp = *puc_data;

    return OAL_SUCC;
}


oal_uint32  mac_ie_parse_he_bss_color_change_announcement_ie(oal_uint8 *puc_payload, mac_frame_bss_color_change_annoncement_ie_stru *pst_bss_color)
{
    mac_frame_bss_color_change_annoncement_ie_stru          *pst_bss_color_info = OAL_PTR_NULL;
    oal_uint8                                               *puc_data = OAL_PTR_NULL;

    if(MAC_HE_BSS_COLOR_CHANGE_ANNOUNCEMENT_LEN != puc_payload[1])
    {
        return OAL_FAIL;
    }

    puc_data           = puc_payload + 3;

    pst_bss_color_info = (mac_frame_bss_color_change_annoncement_ie_stru *)puc_data;
    if (EOK != memcpy_s(pst_bss_color, OAL_SIZEOF(mac_frame_bss_color_change_annoncement_ie_stru),
                        pst_bss_color_info, OAL_SIZEOF(mac_frame_bss_color_change_annoncement_ie_stru))) {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "mac_ie_parse_he_bss_color_change_announcement_ie::memcpy fail!");
        return OAL_FAIL;
    }

    return OAL_SUCC;
}


oal_uint32  mac_ie_parse_multi_bssid_ie(oal_uint8 *puc_frame_data, mac_multi_bssid_frame_info_stru *pst_mbssid_frame_info, oal_uint8 *puc_mbssid_body_ie_len)
{
    oal_uint8                           *puc_data = OAL_PTR_NULL;
    oal_uint8                           *puc_ssid = OAL_PTR_NULL;
    oal_uint8                           *puc_non_transmitted_bssid_cap = OAL_PTR_NULL;
    oal_uint8                           *puc_mbssid_index = OAL_PTR_NULL;
    oal_uint8                            uc_ie_len;
    oal_uint8                            uc_left_len;

    /***************************************************************************
    -------------------------------------------------------------------------
    |EID |Length |Max BSSID Indicator| Non-Transmitted BSSID Profile ID| BSSID Profile Len|
    -------------------------------------------------------------------------
    |1   |1      |1                  |               1                 |    1
    -------------------------------------------------------------------------
    |sub_ssid |Length | sub_ssid|
    -------------------------------------------------------------------------
    |1        |1      |    n    |
    -------------------------------------------------------------------------
    |sub_nonTxBssid_Cap |Length |Nontransmitted bssid CAP     |
    -------------------------------------------------------------------------
    |1                  |1      |    2                        |
    -------------------------------------------------------------------------
    |sub_Multi BSSID Index |Length |bssid index|
    -------------------------------------------------------------------------
    |1                     |1      |    1      |
    认证用例规定Non-transmitted BSSID Profile 至少要包含 NonTxBSSID Cap IE(4), SSID IE(34Bytes),M-BSSID Index IE(3)
    ***************************************************************************/
    uc_ie_len     = puc_frame_data[1];
    if (uc_ie_len < MAC_MULTIPLE_BSSID_IE_MIN_LEN)
    {
        OAM_WARNING_LOG1(0, OAM_SF_11AX, "{mac_ie_parse_multi_bssid_ie::invalid mbssid_ie_len=[%d].}", uc_ie_len);
        return OAL_FAIL;
    }

    *puc_mbssid_body_ie_len                        = uc_ie_len;
    uc_left_len                                    = uc_ie_len;

    /*Max BSSID Indicator */
    puc_data                                       = puc_frame_data + MAC_IE_HDR_LEN;
    pst_mbssid_frame_info->uc_maxbssid_indicator   = *(oal_uint8 *)puc_data;
    puc_data                                      += 1;
    uc_left_len                                   -= 1;

    /*判断Non-transmitted BSSID Profile 是否存在  */
    if (0 != puc_data[0])
    {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "{mac_ie_parse_multi_bssid_ie:: bssid profile not exist.}");
        return OAL_FAIL;
    }
    puc_data                                      += MAC_IE_HDR_LEN;
    uc_left_len                                   -= MAC_IE_HDR_LEN;

    /*解析 ssid元素    */
    puc_ssid = mac_find_ie_etc(MAC_EID_SSID, puc_data, uc_left_len);
    if(OAL_PTR_NULL == puc_ssid || puc_ssid[1] > WLAN_SSID_MAX_LEN)
    {
        return OAL_FAIL;
    }
    uc_ie_len  = puc_ssid[1];
    if (EOK != memcpy_s(pst_mbssid_frame_info->auc_non_transmitted_ssid, WLAN_SSID_MAX_LEN,
                        puc_ssid + MAC_IE_HDR_LEN, uc_ie_len)) {
        OAM_ERROR_LOG0(0, OAM_SF_11AX, "mac_ie_parse_multi_bssid_ie::memcpy fail!");
        return OAL_FAIL;

    }
    pst_mbssid_frame_info->uc_non_transmitted_ssid_len = uc_ie_len;

    /*解析non-transmitted bssid cap IE83*/
    puc_non_transmitted_bssid_cap                  = mac_find_ie_etc(MAC_EID_NONTRANSMITTED_BSSID_CAP, puc_data, uc_left_len);
    if(OAL_PTR_NULL == puc_non_transmitted_bssid_cap)
    {
        return OAL_FAIL;
    }

    pst_mbssid_frame_info->us_non_tramsmitted_bssid_cap  = *(oal_uint16 *)(puc_non_transmitted_bssid_cap + MAC_IE_HDR_LEN);

    /*解析Non-transmitted BSSID Profile 中m-bssid index  ie85*/
    puc_mbssid_index = mac_find_ie_etc(MAC_EID_MULTI_BSSID_INDEX, puc_data, uc_left_len);
    if(OAL_PTR_NULL == puc_mbssid_index)
    {
        return OAL_FAIL;
    }

    pst_mbssid_frame_info->uc_bssid_index                =  puc_mbssid_index[MAC_IE_HDR_LEN];

    return OAL_SUCC;
}

#endif //_PRE_WLAN_FEATURE_11AX

oal_uint32  mac_ie_proc_ext_cap_ie_etc(mac_user_stru *pst_mac_user, oal_uint8 *puc_payload)
{
    mac_user_cap_info_stru   *pst_cap_info = OAL_PTR_NULL;
    oal_uint8                 uc_len;
    oal_uint8                 auc_cap[8] = {0};

    if (OAL_UNLIKELY(OAL_ANY_NULL_PTR2(pst_mac_user, puc_payload)))
    {
        OAM_ERROR_LOG0(0, OAM_SF_ANY, "{mac_ie_proc_ext_cap_ie_etc::param null.}");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_cap_info = &(pst_mac_user->st_cap_info);
    uc_len = puc_payload[1];
    if (uc_len >= MAC_IE_HDR_LEN && uc_len <= 8)
    {
        /* ie长度域的值本身不包含IE头长度，此处不需要另行减去头长 */
        if (EOK != memcpy_s(auc_cap, sizeof(auc_cap), &puc_payload[MAC_IE_HDR_LEN], uc_len)) {
            OAM_ERROR_LOG0(0, OAM_SF_ANY, "mac_ie_proc_ext_cap_ie_etc::memcpy fail!");
            return OAL_FAIL;
        }
    }

    /* 提取 BIT12: 支持proxy arp */
    pst_cap_info->bit_proxy_arp = ((auc_cap[1] & BIT4) == 0) ? OAL_FALSE : OAL_TRUE;
#if defined(_PRE_WLAN_FEATURE_11V) || defined(_PRE_WLAN_FEATURE_11V_ENABLE)
    /* 提取 BIT19: 支持bss transition */
    pst_cap_info->bit_bss_transition = ((auc_cap[2] & BIT3) == 0) ? OAL_FALSE : OAL_TRUE;
#endif
    return OAL_SUCC;
}


oal_uint8  *mac_ie_find_vendor_vht_ie(oal_uint8 *puc_frame, oal_uint16 us_frame_len)
{
    oal_uint8    *puc_vendor_ie = OAL_PTR_NULL;
    oal_uint8    *puc_vht_ie = OAL_PTR_NULL;
    oal_uint16    us_offset_vendor_vht = MAC_WLAN_OUI_VENDOR_VHT_HEADER + MAC_IE_HDR_LEN;

    puc_vendor_ie = mac_find_vendor_ie_etc(MAC_WLAN_OUI_BROADCOM_EPIGRAM,
                                MAC_WLAN_OUI_VENDOR_VHT_TYPE,
                                puc_frame,
                                us_frame_len);

    if ((OAL_PTR_NULL != puc_vendor_ie) && (puc_vendor_ie[1] >= MAC_WLAN_OUI_VENDOR_VHT_HEADER))
    {
        puc_vht_ie = mac_find_ie_etc(MAC_EID_VHT_CAP, puc_vendor_ie + us_offset_vendor_vht, puc_vendor_ie[1] - MAC_WLAN_OUI_VENDOR_VHT_HEADER);
    }

    return puc_vht_ie;
}


oal_void mac_proc_ht_opern_ie_cb(mac_vap_stru *pst_mac_vap, oal_uint8 *puc_payload, mac_user_stru *pst_mac_user, oal_uint32 *pst_ch)
{
    mac_ht_opern_ac_stru    *pst_ht_opern = (mac_ht_opern_ac_stru *)(&puc_payload[MAC_IE_HDR_LEN]);
    mac_user_ht_hdl_stru     st_ht_hdl;

    mac_user_get_ht_hdl_etc(pst_mac_user, &st_ht_hdl);
    /* 提取HT Operation IE中的"Secondary Channel Offset" */
    st_ht_hdl.bit_secondary_chan_offset = pst_ht_opern->bit_secondary_chan_offset;

    /* 在2.4G用户声称20M情况下该变量不切换 */
    if ((WLAN_BAND_WIDTH_20M == pst_ht_opern->bit_sta_chan_width) && (WLAN_BAND_2G == pst_mac_vap->st_channel.en_band))
    {
        st_ht_hdl.bit_secondary_chan_offset = MAC_SCN;
    }

    /* 保护相关 */
    st_ht_hdl.bit_rifs_mode                         = pst_ht_opern->bit_rifs_mode;/*发送描述符填写时候需要此值*/
    st_ht_hdl.bit_HT_protection                     = pst_ht_opern->bit_HT_protection;
    st_ht_hdl.bit_nongf_sta_present                 = pst_ht_opern->bit_nongf_sta_present;
    st_ht_hdl.bit_obss_nonht_sta_present            = pst_ht_opern->bit_obss_nonht_sta_present;
    st_ht_hdl.bit_lsig_txop_protection_full_support = pst_ht_opern->bit_lsig_txop_protection_full_support;
    st_ht_hdl.bit_sta_chan_width                    = pst_ht_opern->bit_sta_chan_width;
    st_ht_hdl.uc_chan_center_freq_seg2              = pst_ht_opern->bit_chan_center_freq_seg2;

    mac_user_set_ht_hdl_etc(pst_mac_user, &st_ht_hdl);
}

#ifdef _PRE_WLAN_FEATURE_OPMODE_NOTIFY

OAL_STATIC oal_uint32  mac_ie_check_proc_opmode_param(mac_user_stru *pst_mac_user, mac_opmode_notify_stru *pst_opmode_notify)
{
    /* USER新限定带宽、空间流不允许大于其能力 */
    if ((pst_mac_user->en_bandwidth_cap < pst_opmode_notify->bit_channel_width)
       ||(pst_mac_user->en_user_num_spatial_stream < pst_opmode_notify->bit_rx_nss))
    {
        /* p20pro 2G 1*1热点，beacon携带opmode为80M,造成此处会刷屏,属于对端异常 */
        OAM_WARNING_LOG4(pst_mac_user->uc_vap_id, OAM_SF_OPMODE, "{mac_ie_check_proc_opmode_param::bw or nss over limit! work bw[%d]opmode bw[%d]avail_nss[%d]bit_rx_nss[%d]!}\r\n",
                         pst_mac_user->en_bandwidth_cap, pst_opmode_notify->bit_channel_width,
                         pst_mac_user->en_user_num_spatial_stream, pst_opmode_notify->bit_rx_nss);

        return OAL_FAIL;
    }

    /* Nss Type值为1，则表示beamforming Rx Nss不能超过其声称值 */
    if (1 == pst_opmode_notify->bit_rx_nss_type)
    {
        if (pst_mac_user->st_vht_hdl.bit_num_bf_ant_supported < pst_opmode_notify->bit_rx_nss)
        {
            OAM_WARNING_LOG2(pst_mac_user->uc_vap_id, OAM_SF_OPMODE, "{mac_ie_check_proc_opmode_param::bit_rx_nss is over limit!bit_num_bf_ant_supported = [%d],bit_rx_nss = [%d]!}\r\n",
                             pst_mac_user->st_vht_hdl.bit_num_bf_ant_supported,
                             pst_opmode_notify->bit_rx_nss);
            return OAL_FAIL;
        }
    }

    return OAL_SUCC;
}

OAL_STATIC wlan_bw_cap_enum_uint8  mac_ie_proc_opmode_channel_width_etc(mac_user_stru *pst_mac_user, mac_opmode_notify_stru *pst_opmode_notify)
{
    wlan_bw_cap_enum_uint8      en_opmode_notify_bw = 0;


    if(pst_opmode_notify->bit_channel_width == WLAN_BW_CAP_80M)
    {
        en_opmode_notify_bw = (pst_opmode_notify->bit_160or8080) ? WLAN_BW_CAP_160M : WLAN_BW_CAP_80M;
    }
    else
    {
        en_opmode_notify_bw = pst_opmode_notify->bit_channel_width;
    }

    return en_opmode_notify_bw;
}


oal_uint32  mac_ie_proc_opmode_field_etc(mac_vap_stru *pst_mac_vap, mac_user_stru *pst_mac_user, mac_opmode_notify_stru *pst_opmode_notify)
{
    wlan_bw_cap_enum_uint8      en_bwcap_vap = 0;        /* vap自身带宽能力 */
    wlan_bw_cap_enum_uint8      en_avail_bw  = 0;        /* vap自身带宽能力 */
    wlan_bw_cap_enum_uint8      en_opmode_notify_bw = 0;

    /* 入参指针已经在调用函数保证非空，这里直接使用即可 */
    if (OAL_FAIL == mac_ie_check_proc_opmode_param(pst_mac_user, pst_opmode_notify))
    {
        return OAL_FAIL;
    }

    en_opmode_notify_bw = mac_ie_proc_opmode_channel_width_etc(pst_mac_user, pst_opmode_notify);
    /* 判断channel_width是否与user之前使用channel_width相同 */
    if (en_opmode_notify_bw != pst_mac_user->en_avail_bandwidth)
    {
        OAM_INFO_LOG2(pst_mac_vap->uc_vap_id, OAM_SF_OPMODE, "{mac_ie_proc_opmode_field_etc::pst_opmode_notify->bit_channel_width = [%x], pst_mac_user->en_avail_bandwidth = [%x]!}\r\n",
                      en_opmode_notify_bw, pst_mac_user->en_avail_bandwidth);

        /* 获取vap带宽能力与用户带宽能力的交集 */
        mac_vap_get_bandwidth_cap_etc(pst_mac_vap, &en_bwcap_vap);
        if(en_bwcap_vap == WLAN_BW_CAP_160M && en_opmode_notify_bw == WLAN_BW_CAP_80M && pst_mac_user->en_avail_bandwidth == WLAN_BW_CAP_160M)
        {
            en_avail_bw = OAL_MIN(en_bwcap_vap, WLAN_BW_CAP_160M);
        }
        else
        {
            en_avail_bw = OAL_MIN(en_bwcap_vap, en_opmode_notify_bw);
        }
        mac_user_set_bandwidth_info_etc(pst_mac_user, en_avail_bw, en_avail_bw);

        OAM_INFO_LOG2(pst_mac_vap->uc_vap_id, OAM_SF_OPMODE, "{mac_ie_proc_opmode_field_etc::change bandwidth. en_bwcap_vap = [%x], pst_mac_user->en_avail_bandwidth = [%x]!}\r\n",
                      en_bwcap_vap, pst_mac_user->en_avail_bandwidth);
    }

    /* 判断Rx Nss Type是否为beamforming模式 */
    if (1 == pst_opmode_notify->bit_rx_nss_type)
    {
        OAM_INFO_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_OPMODE, "{mac_ie_proc_opmode_field_etc::pst_opmode_notify->bit_rx_nss_type == 1!}\r\n");

        /* 判断Rx Nss是否与user之前使用Rx Nss相同 */
        if (pst_opmode_notify->bit_rx_nss != pst_mac_user->en_avail_bf_num_spatial_stream)
        {
            /* 需要获取vap和更新nss的取小，如果我们不支持mimo了，对端宣传切换mimo也不执行 */
            mac_user_avail_bf_num_spatial_stream_etc(pst_mac_user, OAL_MIN(pst_mac_vap->en_vap_rx_nss, pst_opmode_notify->bit_rx_nss));
        }
    }
    else
    {
        /* 判断Rx Nss是否与user之前使用Rx Nss相同 */
        if (pst_opmode_notify->bit_rx_nss != pst_mac_user->en_avail_num_spatial_stream)
        {
            OAM_INFO_LOG2(pst_mac_vap->uc_vap_id, OAM_SF_OPMODE, "{mac_ie_proc_opmode_field_etc::pst_opmode_notify->bit_rx_nss = [%x], pst_mac_user->en_avail_num_spatial_stream = [%x]!}\r\n",
                          pst_opmode_notify->bit_rx_nss, pst_mac_user->en_avail_num_spatial_stream);

            /* 需要获取vap和更新nss的取小，如果我们不支持mimo了，对端宣传切换mimo也不执行 */
            mac_user_set_avail_num_spatial_stream_etc(pst_mac_user, OAL_MIN(pst_mac_vap->en_vap_rx_nss, pst_opmode_notify->bit_rx_nss));

            OAM_INFO_LOG2(pst_mac_vap->uc_vap_id, OAM_SF_OPMODE, "{mac_ie_proc_opmode_field_etc::change rss. pst_mac_vap->en_vap_rx_nss = [%x], pst_mac_user->en_avail_num_spatial_stream = [%x]!}\r\n",
                          pst_mac_vap->en_vap_rx_nss, pst_mac_user->en_avail_num_spatial_stream);
        }
    }

    if (OAL_PTR_NULL != g_st_mac_ie_rom_cb.opmode_field_cb)
    {
        g_st_mac_ie_rom_cb.opmode_field_cb(pst_mac_vap, pst_mac_user, pst_opmode_notify);
    }

    return OAL_SUCC;
}
#endif

#ifdef __cplusplus
    #if __cplusplus
        }
    #endif
#endif

