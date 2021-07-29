

#ifndef __ALG_EXT_IF_H__
#define __ALG_EXT_IF_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/* 1 ����ͷ�ļ����� */
#include "oal_types.h"
#include "hal_ext_if.h"
#include "dmac_ext_if.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_ALG_EXT_IF_H

/* 2 �궨�� */
#ifdef _PRE_PLAT_FEATURE_CUSTOMIZE
/* ���ʱ���ز��� */
#define ALG_TPC_11B_RATE_NUM 4  /* 11b������Ŀ */
#define ALG_TPC_11G_RATE_NUM 8  /* 11g������Ŀ */
#define ALG_TPC_11A_RATE_NUM 8  /* 11a������Ŀ */
#define ALG_TPC_11AC_20M_NUM 9  /* 11n_11ac_2g������Ŀ */
#define ALG_TPC_11AC_40M_NUM 11 /* 11n_11ac_2g������Ŀ */
#define ALG_TPC_11AC_80M_NUM 10 /* 11n_11ac_2g������Ŀ */
/* rate-tpccode table�����ʸ��� */
#define ALG_TPC_RATE_TPC_CODE_TABLE_LEN \
    (ALG_TPC_11B_RATE_NUM + ALG_TPC_11G_RATE_NUM + ALG_TPC_11AC_20M_NUM + ALG_TPC_11AC_40M_NUM + ALG_TPC_11AC_80M_NUM)
#endif
/* 3 ö�ٶ��� */
/* 4 ȫ�ֱ������� */
/* 5 ��Ϣͷ���� */
/* 6 ��Ϣ���� */
/* 7 STRUCT���� */
/* 8 UNION���� */
/* 9 OTHERS���� */
/* 10 �������� */
extern oal_int32 alg_main_init(oal_void);
extern oal_void alg_main_exit(oal_void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* end of alg_ext_if.h */