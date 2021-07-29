

#ifndef __HMAC_SMPS_H__
#define __HMAC_SMPS_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#ifdef _PRE_WLAN_FEATURE_SMPS

/* 1 其他头文件包含 */
#include "oal_ext_if.h"
#include "hmac_main.h"
#include "oam_ext_if.h"
#include "mac_resource.h"
#include "dmac_ext_if.h"
#include "mac_device.h"
#include "mac_vap.h"
#include "hmac_vap.h"
#include "mac_user.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_SMPS_H
/* 2 宏定义 */
/* 3 枚举定义 */
/* 4 全局变量声明 */
/* 5 消息头定义 */
/* 6 消息定义 */
/* 7 STRUCT定义 */
/* 8 UNION定义 */
/* 9 OTHERS定义 */
/* 10 函数声明 */
extern oal_uint32 hmac_smps_update_status(mac_vap_stru *pst_mac_vap,
                                          mac_user_stru *pst_mac_user,
                                          oal_bool_enum_uint8 en_plus_user);
extern oal_uint32 hmac_smps_user_asoc_update(oal_uint8 uc_prev_smps_mode,
                                             mac_user_stru *pst_mac_user,
                                             mac_vap_stru *pst_mac_vap);
extern oal_uint32 hmac_mgmt_rx_smps_frame(mac_vap_stru *pst_mac_vap,
                                          hmac_user_stru *pst_hmac_user,
                                          oal_uint8 *puc_data);
extern oal_uint32 hmac_smps_update_user_status(mac_vap_stru *pst_mac_vap, mac_user_stru *pst_mac_user);

#endif /* end of _PRE_WLAN_FEATURE_SMPS */

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* end of hmac_smps.h */
