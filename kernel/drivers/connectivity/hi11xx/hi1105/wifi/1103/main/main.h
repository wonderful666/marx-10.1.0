

#ifndef __MAIN_H__
#define __MAIN_H__

// �˴�����extern "C" UT���벻��
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* 1 ����ͷ�ļ����� */
#include "oal_ext_if.h"
#include "oam_ext_if.h"
#include "frw_ext_if.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_MAIN_H
/* 2 �궨�� */
/* 3 ö�ٶ��� */
/* 4 ȫ�ֱ������� */
#if defined(_PRE_PRODUCT_ID_HI110X_HOST)
extern oal_completion g_wlan_cali_completed;
#endif
/* 5 ��Ϣͷ���� */
/* 6 ��Ϣ���� */
/* 7 STRUCT���� */
/* 8 UNION���� */
/* 9 OTHERS���� */
/* 10 �������� */
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