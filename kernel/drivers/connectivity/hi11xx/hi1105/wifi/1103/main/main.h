

#ifndef __MAIN_H__
#define __MAIN_H__

// 此处不加extern "C" UT编译不过
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* 1 其他头文件包含 */
#include "oal_ext_if.h"
#include "oam_ext_if.h"
#include "frw_ext_if.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_MAIN_H
/* 2 宏定义 */
/* 3 枚举定义 */
/* 4 全局变量声明 */
#if defined(_PRE_PRODUCT_ID_HI110X_HOST)
extern oal_completion g_wlan_cali_completed;
#endif
/* 5 消息头定义 */
/* 6 消息定义 */
/* 7 STRUCT定义 */
/* 8 UNION定义 */
/* 9 OTHERS定义 */
/* 10 函数声明 */
#if (defined(_PRE_PRODUCT_ID_HI110X_DEV))
extern oal_int32 hi110x_device_main_init(oal_void);
extern oal_void device_main_init(oal_void);
extern oal_uint8 device_psm_main_function(oal_void);
#elif (defined(_PRE_PRODUCT_ID_HI110X_HOST))
extern oal_int32 hi110x_host_main_init(oal_void);
extern oal_void hi110x_host_main_exit(oal_void);
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif
