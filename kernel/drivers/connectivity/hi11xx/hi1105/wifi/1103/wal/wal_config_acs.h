

#ifndef __WAL_CONFIG_ACS_H__
#define __WAL_CONFIG_ACS_H__

/* 1 其他头文件包含 */
#include "oal_ext_if.h"
#include "wlan_types.h"
#include "wlan_spec.h"
#include "mac_vap.h"
#include "hmac_ext_if.h"
#include "wal_ext_if.h"
#include "wal_config.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_WAL_CONFIG_ACS_H
/* 2 宏定义 */
/* 3 枚举定义 */
/* 4 全局变量声明 */
/* 5 消息头定义 */
/* 6 消息定义 */
/* 7 STRUCT定义 */
/* 8 UNION定义 */
/* 9 OTHERS定义 */
/* 10 函数声明 */
extern oal_uint32 wal_acs_response_event_handler(frw_event_mem_stru *pst_event_mem);
extern oal_uint32 wal_acs_exit(oal_void);
extern oal_uint32 wal_acs_init(oal_void);
extern oal_uint32 wal_alloc_cfg_event_etc(oal_net_device_stru *pst_net_dev, frw_event_mem_stru **ppst_event_mem,
                                          oal_void *pst_resp_addr, wal_msg_stru **ppst_cfg_msg, oal_uint16 us_len);
#endif /* end of wal_config_acs.h */
