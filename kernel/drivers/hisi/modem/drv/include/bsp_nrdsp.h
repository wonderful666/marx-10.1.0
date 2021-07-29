/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
 * foss@huawei.com
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 and
 * * only version 2 as published by the Free Software Foundation.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) Neither the name of Huawei nor the names of its contributors may
 * *    be used to endorse or promote products derived from this software
 * *    without specific prior written permission.
 *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __BSP_NRDSP_H__
#define __BSP_NRDSP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <product_config.h>
#include <osl_types.h>
#include <mdrv_nrdsp.h>

struct nrdsp_sect_desc
{
    u8                               usNo;                   /* ����� */
    u8                               ucCrcOriginal;          /* �ε�ԭʼCRCУ��� */
    u8                               ucCrc;                  /* ��λʱ��CRCУ��� */
    u8                               ucTcmType;              /* ������: 0-����(text); 1-rodata; 2-data; 3-bss */
    u8                               ucCoreMask;             /* ���ص��ĸ�NX��(��ֵbitΪ1�����Ӧ�����д˶Σ�ȡֵ0x01,0x04,0x08,0x10,0x20,0x40,0x80����Ӧ8��NX) */
    u8                               ucLoadType;             /* �������ͣ�0��ʾ����Ҫ���أ� 1��ʾ�ϵ�ֻ��Ҫ����һ�Σ� 2��ʾÿ����Ҫ���� */
    u8                               ucStoreType;            /* �������ͣ�0��ʾ����Ҫ���ݣ�1��ʾ�µ�ʱֻ��Ҫ����һ�Σ�2��ʾÿ�ζ���Ҫ���� */
    u8                               ucSectNeedStore;        /* ϵͳ�쳣ʱ�����Ƿ���Ҫ�����־��0��ʾ�β���Ҫ���棬1��ʾ����Ҫ���棬Ĭ����1 */
    u32                              ulTargetAddr;           /* ���ص�Ŀ���ַ */
    u32                              ulExecuteAddr;          /* ���е�ַ */
    u32                              ulSectSize;             /* �εĴ�С */
    u32                              ulFileOffset;           /* �����ļ��ڵ�ƫ�� */
    u8                               ucRsv[8];
};

/* NR����㾵��ͷ */
struct nrdsp_bin_header
{
    s8                          acDescription[24];      /* �ɹ������ɣ�����Ϊ��������Ǻ����ڡ�ʱ�� */
    u32                         ulFileSize;             /* �ļ���С */
    u32                         ulSectNum;              /* �θ��� */
    u8                          ucCrashCore[8];         /* 0:HL1C, 1~4:LL1C DL, 5~6:LL1C UL, 7:SDR */
    u8                          ucDrxFlag;              /* �����Ƿ��ڵ͹����±��ݣ�1�����ǣ�0�����(����״̬)������ͨ����ͬ�ļ������� */
    u8                          ucRsv[23];
    struct nrdsp_sect_desc      astSect[0];             /* ����Ϣ */
};

struct nrdsp_dump_flag
{
    unsigned int dsp_dump_flag;         /* �쳣�����У���¼dsp����tcm�Ĺ��̱�־ */
};

typedef enum{
    NRDSP_NORMAL = 0,
    NRDSP_WAITI = 1,
}NRDSP_WAIT_MODE;

#if !defined(CONFIG_NRDSP) && !defined(CONFIG_NRDSP_NRFPGA)
#if defined(__OS_RTOSCK__) || defined(__OS_RTOSCK_SMP__) ||defined(__OS_RTOSCK_TVP__) ||defined(__OS_RTOSCK_TSP__)
static inline int bsp_nrdsp_init(void)
{
    return 0;
}

static inline int bsp_nrdsp_load(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_store(nrdsp_type_e etype, nrdsp_wait_edma_type edma_type)
{
    return 0;
}

static inline int bsp_nrdsp_init_reg(nrdsp_type_e etype, nrdsp_init_cb_func pfun, char* param)
{
    return 0;
}

static inline int bsp_nrdsp_subsys_state_reg(nrdsp_type_e etype, nrdsp_state_cb_func pfun)
{
    return 0;
}

static inline int bsp_nrdsp_core_reset(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_core_unreset(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_reset(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_unreset(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_pd_reset(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_pd_unreset(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_auto_gate_cfg(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_clk_enable(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_clk_disable(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_pd_clk_enable(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_pd_clk_disable(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_iso_enable(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_power_on(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_power_off(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_power_up(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_run(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_stop(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_dfs_pll_enable(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_dfs_pll_disable(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_get_waitmode(nrdsp_type_e etype)
{
    return 1;
}

static inline int bsp_nrdsp_tcm_retention(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_tcm_shutdown(nrdsp_type_e etype)
{
    return 0;
}

static inline int bsp_nrdsp_tcm_resume(nrdsp_type_e etype)
{
    return 0;
}
#endif

#ifdef __KERNEL__
static inline void bsp_nrdsp_dump_tcm(char * dst_path)
{
    return;
}
#endif
#else
#if defined(__OS_RTOSCK__) || defined(__OS_RTOSCK_SMP__) ||defined(__OS_RTOSCK_TVP__) ||defined(__OS_RTOSCK_TSP__)
int bsp_nrdsp_init(void);
int bsp_nrdsp_load(nrdsp_type_e etype);
int bsp_nrdsp_store(nrdsp_type_e etype, nrdsp_wait_edma_type edma_type);
int bsp_nrdsp_init_reg(nrdsp_type_e etype, nrdsp_init_cb_func pfun, char* param);
int bsp_nrdsp_subsys_state_reg(nrdsp_type_e etype, nrdsp_state_cb_func pfun);
int bsp_nrdsp_core_reset(nrdsp_type_e etype);
int bsp_nrdsp_core_unreset(nrdsp_type_e etype);
int bsp_nrdsp_reset(nrdsp_type_e etype);
int bsp_nrdsp_unreset(nrdsp_type_e etype);
int bsp_nrdsp_pd_reset(nrdsp_type_e etype);
int bsp_nrdsp_pd_unreset(nrdsp_type_e etype);
int bsp_nrdsp_clk_enable(nrdsp_type_e etype);
int bsp_nrdsp_clk_disable(nrdsp_type_e etype);
int bsp_nrdsp_power_on(nrdsp_type_e etype);
int bsp_nrdsp_power_off(nrdsp_type_e etype);
int bsp_nrdsp_power_up(nrdsp_type_e etype);
int bsp_nrdsp_pd_clk_enable(nrdsp_type_e etype);
int bsp_nrdsp_pd_clk_disable(nrdsp_type_e etype);
int bsp_nrdsp_iso_enable(nrdsp_type_e etype);
int bsp_nrdsp_run(nrdsp_type_e etype);
int bsp_nrdsp_stop(nrdsp_type_e etype);
int bsp_nrdsp_dfs_pll_enable(nrdsp_type_e etype);
int bsp_nrdsp_dfs_pll_disable(nrdsp_type_e etype);
int bsp_nrdsp_get_waitmode(nrdsp_type_e etype);
int bsp_nrdsp_tcm_retention(nrdsp_type_e etype);
int bsp_nrdsp_tcm_shutdown(nrdsp_type_e etype);
int bsp_nrdsp_tcm_resume(nrdsp_type_e etype);
int bsp_nrdsp_auto_gate_cfg(nrdsp_type_e etype);
#endif
#ifdef __KERNEL__
void bsp_nrdsp_dump_tcm(const char * dst_path);
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif
