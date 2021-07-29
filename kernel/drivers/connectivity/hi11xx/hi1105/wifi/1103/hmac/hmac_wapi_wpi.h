
#ifndef __HMAC_WAPI_WPI_H__
#define __HMAC_WAPI_WPI_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* 1 头文件包含 */
#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_WAPI_WPI_H
#ifdef _PRE_WLAN_FEATURE_WAPI
/* 2 宏定义 */
/* 3 枚举定义 */
/* 4 全局变量声明 */
/* 5 消息头定义 */
/* 6 消息定义 */
/* 7 STRUCT定义 */
/* 8 UNION定义 */
/* 9 OTHERS定义 */
/* 10 函数声明 */
extern oal_uint32 hmac_wpi_encrypt_etc(oal_uint8 *puc_iv, oal_uint8 *puc_bufin, oal_uint32 ul_buflen,
                                       oal_uint8 *puc_key, oal_uint8 *puc_bufout);

extern oal_uint32 hmac_wpi_decrypt_etc(oal_uint8 *puc_iv, oal_uint8 *puc_bufin, oal_uint32 ul_buflen,
                                       oal_uint8 *puc_key, oal_uint8 *puc_bufout);

extern void hmac_wpi_swap_pn_etc(oal_uint8 *puc_pn, oal_uint8 uc_len);

extern oal_uint32 hmac_wpi_pmac_etc(oal_uint8 *puc_iv, oal_uint8 *puc_buf, oal_uint32 ul_pamclen,
                                    oal_uint8 *puc_key, oal_uint8 *puc_mic, oal_uint8 uc_mic_len);

#endif
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* end of hmac_wapi_wpi.h */



