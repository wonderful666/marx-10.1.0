

#ifndef __HMAC_ENCAP_FRAME_H__
#define __HMAC_ENCAP_FRAME_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* 1 其他头文件包含 */
#include "oal_ext_if.h"
#include "hmac_user.h"
#include "mac_vap.h"

/* 2 宏定义 */
/* 3 枚举定义 */
/* 4 全局变量声明 */
/* 5 消息头定义 */
/* 6 消息定义 */
/* 7 STRUCT定义 */
/* 8 UNION定义 */
/* 9 OTHERS定义 */
/* 10 函数声明 */
extern oal_uint16 hmac_mgmt_encap_deauth_etc(mac_vap_stru *pst_mac_vap, oal_uint8 *puc_data,
                                             const unsigned char *puc_da, oal_uint16 us_err_code);
extern oal_uint16 hmac_mgmt_encap_disassoc_etc(mac_vap_stru *pst_mac_vap, oal_uint8 *puc_data,
                                               oal_uint8 *puc_da, oal_uint16 us_err_code);
extern oal_uint16 hmac_encap_sa_query_req_etc(mac_vap_stru *pst_mac_vap, oal_uint8 *puc_data,
                                              oal_uint8 *puc_da, oal_uint16 us_trans_id);
extern oal_uint16 hmac_encap_sa_query_rsp_etc(mac_vap_stru *pst_mac_vap, oal_uint8 *pst_hdr,
                                              oal_uint8 *puc_data);
extern oal_uint16 hmac_encap_notify_chan_width_etc(mac_vap_stru *pst_mac_vap, oal_uint8 *puc_data,
                                                   oal_uint8 *puc_da);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* end of hmac_encap_frame.h */
