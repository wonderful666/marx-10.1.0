

#ifndef __HMAC_ACS_H__
#define __HMAC_ACS_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* 1 其他头文件包含 */
#include "oal_ext_if.h"
#include "frw_ext_if.h"
#include "hmac_vap.h"
#include "mac_device.h"
#include "hmac_device.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_ACS_H
/* 2 宏定义 */
#define HMAC_ACS_RECHECK_INTERVAL (15 * 60 * 1000)  // 15 min

/* 3 枚举定义 */
/* 4 全局变量声明 */
/* 5 消息头定义 */
/* 6 消息定义 */
/* 7 STRUCT定义 */
/* 8 UNION定义 */
/* 9 OTHERS定义 */
/* 10 函数声明 */
extern oal_uint32 hmac_acs_init_scan_hook(hmac_scan_record_stru *pst_scan_record,
                                          hmac_device_stru *pst_dev);
extern oal_bool_enum_uint8 hmac_acs_try_switch_channel(hmac_device_stru *pst_hmac_device);
extern oal_uint32 hmac_acs_got_init_rank(hmac_device_stru *pst_hmac_device,
                                         mac_vap_stru *pst_mac_vap,
                                         mac_acs_cmd_stru *pst_cmd);
extern oal_uint32 hmac_acs_process_rescan_event(frw_event_mem_stru *pst_event_mem);
extern oal_void hmac_acs_register_rescan_timer(oal_uint32 ul_dev_id);
extern oal_uint32 hmac_acs_init(oal_void);
extern oal_uint32 hmac_acs_exit(oal_void);
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#endif
