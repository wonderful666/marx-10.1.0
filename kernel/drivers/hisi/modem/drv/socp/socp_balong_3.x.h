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

#ifndef _SOCP_BALONG_5G_H
#define _SOCP_BALONG_5G_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DIAG_SYSTEM_5G

/* ͷ�ļ����� */
#include "bsp_memmap.h"
#include "soc_interrupts.h"
#include "product_config.h"
#include "hi_socp.h"
#include "bsp_socp.h"
#include "bsp_print.h"
#include <osl_thread.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

// #define SOCP_VIRT_PHY(virt)          (virt)
// #define SOCP_PHY_VIRT(phy)           (phy)

typedef int (*socp_task_entry)(void *data);

/* �Ĵ�������,ƫ�Ƶ�ַ */
/* SOCPȫ�ֿ��ƼĴ����� */
#define SOCP_REG_GBLRST (HI_SOCP_GLOBAL_CTRL_OFFSET)
#define SOCP_REG_ENCRST_L (HI_SOCP_ENC_SRST_CTRL_L_OFFSET)
#define SOCP_REG_ENCRST_H (HI_SOCP_ENC_SRST_CTRL_H_OFFSET)
#define SOCP_REG_DECRST (HI_SOCP_DEC_SRST_CTRL_OFFSET)
#define SOCP_REG_ENCSTAT_L (HI_SOCP_ENC_CH_STATUS_L_OFFSET)
#define SOCP_REG_ENCSTAT_H (HI_SOCP_ENC_CH_STATUS_H_OFFSET)
#define SOCP_REG_DECSTAT (HI_SOCP_DEC_CH_STATUS_OFFSET)
#define SOCP_REG_CLKCTRL (HI_SOCP_CLK_CTRL_OFFSET)
#define SOCP_REG_PRICFG (HI_SOCP_PRIOR_CFG_OFFSET)
#define SOCP_REG_DEC_INT_TIMEOUT (HI_SOCP_DEC_INT_TIMEOUT_OFFSET)
#define SOCP_REG_INTTIMEOUT (HI_SOCP_INT_TIMEOUT_OFFSET)
#define SOCP_REG_BUFTIMEOUT (HI_SOCP_BUFFER_TIMEOUT_OFFSET)
#define SOCP_REG_DEC_PKTLEN (HI_SOCP_DEC_PKT_LEN_CFG_OFFSET)

/* �������жϼĴ����� */
#define SOCP_REG_GBL_INTSTAT (HI_SOCP_GLOBAL_INT_STATUS_OFFSET)
#define SOCP_REG_ENC_CORE0_MASK0 (HI_SOCP_ENC_CORE0_MASK0_OFFSET)
#define SOCP_REG_ENC_CORE1_MASK0 (HI_SOCP_ENC_CORE1_MASK0_OFFSET)
#define SOCP_REG_ENC_CORE2_MASK0 (HI_SOCP_ENC_CORE2_MASK0_OFFSET)
#define SOCP_REG_ENC_RAWINT0 (HI_SOCP_ENC_RAWINT0_OFFSET)
#define SOCP_REG_ENC_CORE0_INT0 (HI_SOCP_ENC_CORE0_INT0_OFFSET)
#define SOCP_REG_ENC_CORE1_INT0 (HI_SOCP_ENC_CORE1_INT0_OFFSET)
#define SOCP_REG_ENC_CORE2_INT0 (HI_SOCP_ENC_CORE2_INT0_OFFSET)
#define SOCP_REG_APP_MASK1_L (HI_SOCP_ENC_CORE0_MASK1_L_OFFSET)
#define SOCP_REG_APP_MASK1_H (HI_SOCP_ENC_CORE0_MASK1_H_OFFSET)
#define SOCP_REG_MODEM_MASK1_L (HI_SOCP_ENC_CORE1_MASK1_L_OFFSET)
#define SOCP_REG_MODEM_MASK1_H (HI_SOCP_ENC_CORE1_MASK1_H_OFFSET)
#define SOCP_REG_CORE2_MASK1_L (HI_SOCP_ENC_CORE2_MASK1_L_OFFSET)
#define SOCP_REG_CORE2_MASK1_H (HI_SOCP_ENC_CORE2_MASK1_H_OFFSET)
#define SOCP_REG_ENC_RAWINT1_L (HI_SOCP_ENC_RAWINT1_L_OFFSET)
#define SOCP_REG_ENC_RAWINT1_H (HI_SOCP_ENC_RAWINT1_H_OFFSET)
#define SOCP_REG_APP_INTSTAT1_L (HI_SOCP_ENC_CORE0_INT1_L_OFFSET)
#define SOCP_REG_APP_INTSTAT1_H (HI_SOCP_ENC_CORE0_INT1_H_OFFSET)
#define SOCP_REG_MODEM_INTSTAT1_L (HI_SOCP_ENC_CORE1_INT1_L_OFFSET)
#define SOCP_REG_MODEM_INTSTAT1_H (HI_SOCP_ENC_CORE1_INT1_H_OFFSET)
#define SOCP_REG_CORE2_INTSTAT1_L (HI_SOCP_ENC_CORE2_INT1_L_OFFSET)
#define SOCP_REG_CORE2_INTSTAT1_H (HI_SOCP_ENC_CORE2_INT1_H_OFFSET)
#define SOCP_REG_ENC_CORE0_MASK2 (HI_SOCP_ENC_CORE0_MASK2_OFFSET)
#define SOCP_REG_ENC_CORE1_MASK2 (HI_SOCP_ENC_CORE1_MASK2_OFFSET)
#define SOCP_REG_ENC_CORE2_MASK2 (HI_SOCP_ENC_CORE2_MASK2_OFFSET)
#define SOCP_REG_ENC_RAWINT2 (HI_SOCP_ENC_RAWINT2_OFFSET)
#define SOCP_REG_ENC_CORE0_INTSTAT2 (HI_SOCP_ENC_CORE0_INT2_OFFSET)
#define SOCP_REG_ENC_CORE1_INTSTAT2 (HI_SOCP_ENC_CORE1_INT2_OFFSET)
#define SOCP_REG_ENC_CORE2_INTSTAT2 (HI_SOCP_ENC_CORE2_INT2_OFFSET)
#define SOCP_REG_APP_MASK3_L (HI_SOCP_ENC_CORE0_MASK3_L_OFFSET)
#define SOCP_REG_APP_MASK3_H (HI_SOCP_ENC_CORE0_MASK3_H_OFFSET)
#define SOCP_REG_MODEM_MASK3_L (HI_SOCP_ENC_CORE1_MASK3_L_OFFSET)
#define SOCP_REG_MODEM_MASK3_H (HI_SOCP_ENC_CORE1_MASK3_L_OFFSET)
#define SOCP_REG_CORE2_MASK3_L (HI_SOCP_ENC_CORE2_MASK3_L_OFFSET)
#define SOCP_REG_CORE2_MASK3_H (HI_SOCP_ENC_CORE2_MASK3_L_OFFSET)
#define SOCP_REG_ENC_RAWINT3_L (HI_SOCP_ENC_RAWINT3_L_OFFSET)
#define SOCP_REG_ENC_RAWINT3_H (HI_SOCP_ENC_RAWINT3_H_OFFSET)
#define SOCP_REG_APP_INTSTAT3_L (HI_SOCP_ENC_CORE0_INT3_L_OFFSET)
#define SOCP_REG_APP_INTSTAT3_H (HI_SOCP_ENC_CORE0_INT3_H_OFFSET)
#define SOCP_REG_MODEM_INTSTAT3_L (HI_SOCP_ENC_CORE1_INT3_L_OFFSET)
#define SOCP_REG_MODEM_INTSTAT3_H (HI_SOCP_ENC_CORE1_INT3_H_OFFSET)
#define SOCP_REG_CORE2_INTSTAT3_L (HI_SOCP_ENC_CORE2_INT3_L_OFFSET)
#define SOCP_REG_CORE2_INTSTAT3_H (HI_SOCP_ENC_CORE2_INT3_H_OFFSET)

/* �������жϼĴ��� */
#define SOCP_REG_DEC_CORE0_MASK0 (HI_SOCP_DEC_CORE0_MASK0_OFFSET)
#define SOCP_REG_DEC_CORE1_MASK0 (HI_SOCP_DEC_CORE1_MASK0_OFFSET)
#define SOCP_REG_DEC_CORE2_MASK0 (HI_SOCP_DEC_CORE2_MASK0_OFFSET)
#define SOCP_REG_DEC_RAWINT0 (HI_SOCP_DEC_RAWINT0_OFFSET)
#define SOCP_REG_DEC_CORE0_INTSTAT0 (HI_SOCP_DEC_CORE0_INT0_OFFSET)
#define SOCP_REG_DEC_CORE1_INTSTAT0 (HI_SOCP_DEC_CORE1_INT0_OFFSET)
#define SOCP_REG_DEC_CORE2_INTSTAT0 (HI_SOCP_DEC_CORE2_INT0_OFFSET)
#define SOCP_REG_DEC_CORE0_MASK1 (HI_SOCP_DEC_CORE0_MASK1_OFFSET)
#define SOCP_REG_DEC_CORE1_MASK1 (HI_SOCP_DEC_CORE0_MASK1_OFFSET)
#define SOCP_REG_DEC_CORE2_MASK1 (HI_SOCP_DEC_CORE2_MASK1_OFFSET)
#define SOCP_REG_DEC_RAWINT1 (HI_SOCP_DEC_RAWINT1_OFFSET)
#define SOCP_REG_DEC_CORE0_INTSTAT1 (HI_SOCP_DEC_CORE0_INT1_OFFSET)
#define SOCP_REG_DEC_CORE1_INTSTAT1 (HI_SOCP_DEC_CORE1_INT1_OFFSET)
#define SOCP_REG_DEC_CORE2_INTSTAT1 (HI_SOCP_DEC_CORE2_INT1_OFFSET)
#define SOCP_REG_DEC_CORE0_MASK2 (HI_SOCP_DEC_CORE0NOTE_MASK2_OFFSET)
#define SOCP_REG_DEC_CORE1_MASK2 (HI_SOCP_DEC_CORE1NOTE_MASK2_OFFSET)
#define SOCP_REG_DEC_CORE2_MASK2 (HI_SOCP_DEC_CORE2NOTE_MASK2_OFFSET)
#define SOCP_REG_DEC_RAWINT2 (HI_SOCP_DEC_RAWINT2_OFFSET)
#define SOCP_REG_DEC_CORE0_INTSTAT2 (HI_SOCP_DEC_CORE0NOTE_INT2_OFFSET)
#define SOCP_REG_DEC_CORE1_INTSTAT2 (HI_SOCP_DEC_CORE1NOTE_INT2_OFFSET)
#define SOCP_REG_DEC_CORE2_INTSTAT2 (HI_SOCP_DEC_CORE2NOTE_INT2_OFFSET)

/* �����жϼĴ����� */
#define SOCP_REG_BUS_ERROR_MASK (HI_SOCP_BUS_ERROR_MASK_OFFSET)
#define SOCP_REG_BUS_ERROR_RAWINT (HI_SOCP_BUS_ERROR_RAWINT_OFFSET)
#define SOCP_REG_BUS_ERROR_INT (HI_SOCP_BUS_ERROR_INT_OFFSET)

/* ����ͨ�������Ĵ����� */
#define SOCP_REG_ENCSRC_RATE_PERIOD (HI_SOCP_ENC_SRC_RATE_PERIOD_OFFSET) /* ����ͨ�������������üĴ��� */
#define SOCP_REG_ENCSRC_RATE_EN_L (HI_SOCP_ENC_SRC_RATE_EN_L_OFFSET)
#define SOCP_REG_ENCSRC_RATE_EN_H (HI_SOCP_ENC_SRC_RATE_EN_H_OFFSET)
#define SOCP_REG_ENCSRC_RATE_THR(m) (HI_SOCP_ENC_SRC_RATE_THR_0_OFFSET + m * 0x40)

/* ����ͨ��ָ�뾵��Ĵ����� */
#define SOCP_REG_ENCSRC_PTRIMG_CFG (HI_SOCP_ENC_SRC_PTR_IMG_CFG_OFFSET) /* ����ͨ������ָ����д��ʱ���üĴ��� */
#define SOCP_REG_ENCSRC_RPTRIMG_STAT_L (HI_SOCP_ENC_SRC_RPTR_IMG_STATE_L_OFFSET)
#define SOCP_REG_ENCSRC_RPTRIMG_STAT_H (HI_SOCP_ENC_SRC_RPTR_IMG_STATE_H_OFFSET)
#define SOCP_REG_ENCSRC_RDIMG_STAT_L (HI_SOCP_ENC_SRC_RD_IMG_STATE_L_OFFSET)
#define SOCP_REG_ENCSRC_RDIMG_STAT_H (HI_SOCP_ENC_SRC_RD_IMG_STATE_H_OFFSET)
#define SOCP_REG_APP_ENCSRC_PTR_MASK (HI_SOCP_ENC_SRC_PTR_CORE0_MASK_OFFSET)
#define SOCP_REG_MODEM_ENCSRC_PTR_MASK (HI_SOCP_ENC_SRC_PTR_CORE1_MASK_OFFSET)
#define SOCP_REG_CORE2_ENCSRC_PTR_MASK (HI_SOCP_ENC_SRC_PTR_CORE2_MASK_OFFSET)
#define SOCP_REG_ENCSRC_PTR_RAWINT (HI_SOCP_ENC_SRC_PTR_RAWINT_OFFSET)
#define SOCP_REG_APP_ENCSRC_PTR_INT (HI_SOCP_ENC_SRC_PTR_CORE0_INT_OFFSET)
#define SOCP_REG_MODEM_ENCSRC_PTR_INT (HI_SOCP_ENC_SRC_PTR_CORE1_INT_OFFSET)
#define SOCP_REG_CORE2_ENCSRC_PTR_INT (HI_SOCP_ENC_SRC_PTR_CORE2_INT_OFFSET)
#define SOCP_REG_ENCSRC_BUFM_RPTRIMG_L(m) (HI_SOCP_ENC_SRC_BUFM_RPTR_IMG_L_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_BUFM_RPTRIMG_H(m) (HI_SOCP_ENC_SRC_BUFM_RPTR_IMG_H_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_RDQ_WPTRIMG_L(m) (HI_SOCP_ENC_SRC_RDQ_WPTR_IMG_L_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_RDQ_WPTRIMG_H(m) (HI_SOCP_ENC_SRC_RDQ_WPTR_IMG_H_0_OFFSET + m * 0x40)

/* ����Դͨ��buffer�Ĵ����� */
#define SOCP_REG_ENCSRC_BUFWPTR(m) (HI_SOCP_ENC_SRC_BUFM_WPTR_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_BUFRPTR(m) (HI_SOCP_ENC_SRC_BUFM_RPTR_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_BUFDEPTH(m) (HI_SOCP_ENC_SRC_BUFM_DEPTH_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_BUFCFG(m) (HI_SOCP_ENC_SRC_BUFM_CFG_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_RDQWPTR(m) (HI_SOCP_ENC_SRC_RDQ_WPTR_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_RDQRPTR(m) (HI_SOCP_ENC_SRC_RDQ_RPTR_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_RDQCFG(m) (HI_SOCP_ENC_SRC_RDQ_CFG_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_BUFADDR_L(m) (HI_SOCP_ENC_SRC_BUFM_ADDR_L_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_BUFADDR_H(m) (HI_SOCP_ENC_SRC_BUFM_ADDR_H_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_RDQADDR_L(m) (HI_SOCP_ENC_SRC_RDQ_BADDR_L_0_OFFSET + m * 0x40)
#define SOCP_REG_ENCSRC_RDQADDR_H(m) (HI_SOCP_ENC_SRC_RDQ_BADDR_H_0_OFFSET + m * 0x40)

/* ����Ŀ��ͨ��buffer�Ĵ��� */
#define SOCP_REG_ENCDEST_BUFWPTR(n) (HI_SOCP_ENC_DEST_BUFN_WPTR_0_OFFSET + (n)*0x40)
#define SOCP_REG_ENCDEST_BUFRPTR(n) (HI_SOCP_ENC_DEST_BUFN_RPTR_0_OFFSET + (n)*0x40)
#define SOCP_REG_ENCDEST_BUFADDR_L(n) (HI_SOCP_ENC_DEST_BUFN_ADDR_L_0_OFFSET + (n)*0x40)
#define SOCP_REG_ENCDEST_BUFADDR_H(n) (HI_SOCP_ENC_DEST_BUFN_ADDR_H_0_OFFSET + (n)*0x40)
#define SOCP_REG_ENCDEST_BUFDEPTH(n) (HI_SOCP_ENC_DEST_BUFN_DEPTH_0_OFFSET + n * 0x40)
#define SOCP_REG_ENCDEST_BUFTHRH(n) (HI_SOCP_ENC_DEST_BUFN_THRH_0_OFFSET + n * 0x40)
#define SOCP_REG_ENCDEST_BUFTHRESHOLD(n) (HI_SOCP_ENC_INT_THRESHOLD_0_OFFSET + n * 0x40)
#define SOCP_REG_ENCDEST_BUFMODE_CFG(n) (HI_SOCP_ENC_DEST_BUF_MODE_0_OFFSET + n * 0x40)

/* ����Ŀ��TRANS ID�Ĵ����� */
#define SOCP_REG_ENCDEST_TRANS_ID(n) (HI_SOCP_ENC_DEST_TRANSID_0_OFFSET + n * 0x40)

/* ����Դͨ��buffer�Ĵ��� */
#define SOCP_REG_DECSRC_BUFWPTR (HI_SOCP_DEC_SRC_BUFX_WPTR_OFFSET)
#define SOCP_REG_DECSRC_BUFRPTR (HI_SOCP_DEC_SRC_BUFX_RPTR_OFFSET)
#define SOCP_REG_DECSRC_BUFADDR_L (HI_SOCP_DEC_SRC_BUFX_ADDR_L_OFFSET)
#define SOCP_REG_DECSRC_BUFADDR_H (HI_SOCP_DEC_SRC_BUFX_ADDR_H_OFFSET)
#define SOCP_REG_DECSRC_BUFCFG (HI_SOCP_DEC_SRC_BUFX_CFG0_OFFSET)
#define SOCP_REG_DEC_BUFSTAT0 (HI_SOCP_DEC_BUFX_STATUS0_OFFSET)
#define SOCP_REG_DEC_BUFSTAT1 (HI_SOCP_DEC_BUFX_STATUS1_OFFSET)

/* ����Ŀ��ͨ��buffer�Ĵ��� */
#define SOCP_REG_DECDEST_BUFWPTR(y) (HI_SOCP_DEC_DEST_BUFY_WPTR_0_OFFSET + y * 0x40)
#define SOCP_REG_DECDEST_BUFRPTR(y) (HI_SOCP_DEC_DEST_BUFY_RPTR_0_OFFSET + y * 0x40)
#define SOCP_REG_DECDEST_BUFADDR_L(y) (HI_SOCP_DEC_DEST_BUFY_ADDR_L_0_OFFSET + y * 0x40)
#define SOCP_REG_DECDEST_BUFADDR_H(y) (HI_SOCP_DEC_DEST_BUFY_ADDR_H_0_OFFSET + y * 0x40)
#define SOCP_REG_DECDEST_BUFCFG(y) (HI_SOCP_DEC_DEST_BUFY_CFG0_0_OFFSET + y * 0x40)

/* DEBUG�Ĵ��� */
#define SOCP_REG_ENC_CD_DBG0 (HI_SOCP_ENC_CD_DBG0_OFFSET)
#define SOCP_REG_ENC_CD_DBG1 (HI_SOCP_ENC_CD_DBG1_OFFSET)
#define SOCP_REG_ENC_IBUF_DBG0 (HI_SOCP_ENC_IBUF_DBG0_OFFSET)
#define SOCP_REG_ENC_IBUF_DBG1 (HI_SOCP_ENC_IBUF_DBG1_OFFSET)
#define SOCP_REG_ENC_IBUF_DBG2 (HI_SOCP_ENC_IBUF_DBG2_OFFSET)
#define SOCP_REG_ENC_IBUF_DBG3 (HI_SOCP_ENC_IBUF_DBG3_OFFSET)
#define SOCP_REG_ENC_OBUF_DBG0 (HI_SOCP_ENC_OBUF_DBG0_OFFSET)
#define SOCP_REG_ENC_OBUF_DBG1 (HI_SOCP_ENC_OBUF_DBG1_OFFSET)
#define SOCP_REG_ENC_OBUF_DBG2 (HI_SOCP_ENC_OBUF_DBG2_OFFSET)
#define SOCP_REG_ENC_OBUF_DBG3 (HI_SOCP_ENC_OBUF_DBG3_OFFSET)
#define SOCP_REG_ENC_PTR_DBG0 (HI_SOCP_ENC_PTR_DBG0_OFFSET)
#define SOCP_REG_ENC_PTR_DBG1 (HI_SOCP_ENC_PTR_DBG1_OFFSET)
#define SOCP_REG_ENC_PTR_DBG2 (HI_SOCP_ENC_PTR_DBG2_OFFSET)
#define SOCP_REG_ENC_PTR_DBG3 (HI_SOCP_ENC_PTR_DBG3_OFFSET)
#define SOCP_REG_ENC_CH_TRH_DBG0 (HI_SOCP_ENC_CH_TRH_DBG0_OFFSET)
#define SOCP_REG_ENC_CH_TRH_DBG1 (HI_SOCP_ENC_CH_TRH_DBG1_OFFSET)
#define SOCP_REG_ENCSRC_PTRIMG_DBG0 (HI_SOCP_ENC_PTR_IMG_DBG0_OFFSET)
#define SOCP_REG_ENCSRC_PTRIMG_DBG1 (HI_SOCP_ENC_PTR_IMG_DBG1_OFFSET)
#define SOCP_REG_ENCSRC_PTRIMG_DBG2 (HI_SOCP_ENC_PTR_IMG_DBG2_OFFSET)
#define SOCP_REG_ENCSRC_PTRIMG_DBG3 (HI_SOCP_ENC_PTR_IMG_DBG3_OFFSET)
#define SOCP_REG_ENCSRC_PTRIMG_DBG4 (HI_SOCP_ENC_PTR_IMG_DBG4_OFFSET)
#define SOCP_REG_RDQ_PTRIMG_DBG0 (HI_SOCP_ENC_PTR_IMG_DBG0_OFFSET)
#define SOCP_REG_RDQ_PTRIMG_DBG1 (HI_SOCP_ENC_PTR_IMG_DBG1_OFFSET)
#define SOCP_REG_RDQ_PTRIMG_DBG2 (HI_SOCP_ENC_PTR_IMG_DBG2_OFFSET)
#define SOCP_REG_RDQ_PTRIMG_DBG3 (HI_SOCP_ENC_PTR_IMG_DBG3_OFFSET)
#define SOCP_REG_RDQ_PTRIMG_DBG4 (HI_SOCP_ENC_PTR_IMG_DBG4_OFFSET)
#define SOCP_REG_ENC_IBUF_STAT_DBG0 (HI_SOCP_ENC_IBUF_STATE_DBG0_OFFSET)
#define SOCP_REG_ENC_IBUF_STAT_DBG1 (HI_SOCP_ENC_IBUF_STATE_DBG1_OFFSET)
#define SOCP_REG_ENC_IBUF_STAT_DBG2 (HI_SOCP_ENC_IBUF_STATE_DBG2_OFFSET)
#define SOCP_REG_HAC_GIF_DBG (HI_SOCP_HAC_GIF_DBG_OFFSET)
#define SOCP_REG_ENC_IBUF_STAT_CLEAR_DBG (HI_SOCP_ENC_IBUF_STATE_CLEAR_OFFSET)

/* socp�������ֽ���ֵ���� */
#define SOCP_REG_MAX_PKG_CFG (HI_SOCP_MAX_PKG_BYTE_CFG_OFFSET)
/* ��ǰ���ߴ���ĵ�ַ */
#define SOCP_REG_BUS_TRANS_ADDR_DBG (HI_SOCP_OBUF_DEBUG_OFFSET)

/* �汾�Ĵ��� */
#define SOCP_REG_SOCP_VERSION (HI_SOCP_SOCP_VERSION_OFFSET)
#define SOCP_206_VERSION (0x206)
#define SOCP_300_VERSION (0x300)

/* ͨ�����ֵ���� */
#define SOCP_TOTAL_ENCSRC_CHN (0x40)
#define SOCP_MAX_ENCSRC_CHN (0x40) /* ����Դͨ�� */
#define SOCP_MAX_ENCDST_CHN (0x04)

/* ��ַ��ȷ�� */
#define BBP_REG_ARM_BASEADDR HI_BBP_DMA_BASE_ADDR_VIRT
#define SOCP_REG_BASEADDR (g_socp_stat.base_addr)

/* SOCP BBP����ʹ�õ�Ԥ���ڴ�ռ�궨�� */
#define PBXA9_DRAM_BBPDS_VIRT (IO_ADDRESS(DDR_SOCP_ADDR)) /* �õ�ַ��Ҫȷ�� */
#define PBXA9_DRAM_BBPDS_PHYS (DDR_SOCP_ADDR)             /* �õ�ַ��Ҫȷ�� */
#define PBXA9_DRAM_BBPDS_SIZE (DDR_SOCP_SIZE)             /* �ÿռ��С��Ҫȷ�� */

typedef enum {
    SOCP_ENCDST_TASK_CNF = 0,
    SOCP_ENCDST_TASK_IND = 1,
    SOCP_ENCDST_TASK_DUMP = 2,
    SOCP_ENCDST_TASK_LOGSEVER = 3,
    SOCP_ENCDST_TASK_BUTT
} eSOCP_ENCDST_TASK_E;

typedef enum {
    SOCP_ENCSRC_TASK_ENC_SRC = 0,
    SOCP_ENCSRC_TASK_BUTT
} eSOCP_ENCSRC_TASK_E;

/* �ṹ���� */

/* ͨ��״̬�ṹ�壬������ */
typedef struct {
    unsigned long Start;
    unsigned long End;
    u32 write;
    u32 read;
    u32 length;
    u32 idle_size;
} socp_ring_buff_s;

typedef struct  {
    u32 chan_id;
    u32 chan_en;
    u32 trans_id_en;
    u32 ptr_img_en;
    u32 dst_chan_id;
    u32 bypass_en;
    u32 alloc_state; /* ͨ���Ѿ���û�з���ı�ʶ */
    u32 last_rd_size;
    u32 rd_threshold;
    SOCP_ENCSRC_CHNMODE_ENUM_UIN32 chan_mode; /* ���ݽṹ���� */
    SOCP_CHAN_PRIORITY_ENUM_UIN32 priority;
    SOCP_DATA_TYPE_ENUM_UIN32 data_type;
    SOCP_DATA_TYPE_EN_ENUM_UIN32 data_type_en;
    SOCP_ENC_DEBUG_EN_ENUM_UIN32 debug_en;
    unsigned long read_ptr_img_phy_addr;
    unsigned long read_ptr_img_vir_addr;
    socp_ring_buff_s enc_src_buff;
    socp_ring_buff_s rd_buff;
    socp_event_cb event_cb;
    socp_rd_cb rd_cb;
} socp_enc_src_chan_s;


typedef struct socp_compress {
    int bcompress;
    socp_compress_ops_stru ops;
} socp_compress_stru;

typedef struct  {
    u32 chan_id;
    u32 set_state; /* ͨ���Ѿ���û�����õı�ʶ */
    u32 threshold;    /* ��ֵ */
    socp_compress_stru struCompress;
    socp_ring_buff_s enc_dst_buff;
    SOCP_EVENT_ENUM_UIN32 chan_event;
    socp_event_cb event_cb;
    socp_read_cb read_cb;
    u32 buf_thrhold;
    u32 thrhold_high;
    u32 thrhold_low;
    u32 send_req;
    u32 enable_state;
} socp_encdst_chan_s;

typedef struct  {
    u32 chan_id;
    u32 chan_en;
    u32 set_state; /* ͨ���Ѿ���û�����õı�ʶ */
    u32 rd_threshold;
    SOCP_DATA_TYPE_EN_ENUM_UIN32 data_type_en;
    SOCP_DECSRC_CHNMODE_ENUM_UIN32 chan_mode; /* ���ݽṹ���� */
    socp_ring_buff_s dec_src_buff;
    socp_ring_buff_s dec_rd_buff;
    socp_event_cb event_cb;
    socp_rd_cb rd_cb;
} socp_decsrc_chan_s;

typedef struct  {
    u32 chan_id;
    u32 alloc_state;
    SOCP_DATA_TYPE_ENUM_UIN32 data_type;
    socp_ring_buff_s dec_dst_buff;
    socp_event_cb event_cb;
    socp_read_cb read_cb;
} socp_decdst_chan_s;

typedef struct tagSOCP_TASK_S {
    char task_name[16];
    OSL_TASK_ID task_id;
    socp_task_entry task_handler;
    u32 task_prio;
    u32 task_stack_size;
    struct semaphore sem_id;
} socp_task_s;

/* ȫ��״̬�ṹ�� */
typedef struct  {
    s32 init_flag;
    u32 version;
    u32 debug_trace_cfg;
    u32 deflate_status;
    unsigned long base_addr;
    struct semaphore enc_src_sem_id;
    struct semaphore enc_dst_sem_id;
    struct semaphore dec_src_sem_id;
    struct semaphore dec_dst_sem_id;
    unsigned long enc_src_task_id;
    unsigned long enc_dst_task_id;
    unsigned long dec_src_task_id;
    unsigned long dec_dst_task_id;
    u64 int_enc_src_header;
    u32 int_enc_dst_tfr;
    u32 int_enc_dst_ovf;
    u32 int_enc_dst_thrh_ovf;
    u32 int_dec_src_err;
    u32 int_dec_dst_trf;
    u32 int_dec_dst_ovf;

    spinlock_t lock;
    u32 int_enc_dst_thrn_ovf;      /* mntn for diag, threhold overflow counts */
    u32 int_enc_dst_isr_trf_cnt; /*  */
    socp_compress_isr compress_isr;
    socp_enc_src_chan_s enc_src_chan[SOCP_MAX_ENCSRC_CHN];
    socp_encdst_chan_s enc_dst_chan[SOCP_MAX_ENCDST_CHN];
#ifdef SOCP_DECODE_ENABLE
    socp_decsrc_chan_s dec_src_chan[SOCP_MAX_DECSRC_CHN];
    socp_decdst_chan_s dec_dst_chan[SOCP_MAX_DECDST_CHN];
#endif
} socp_gbl_state_s;

/* ����ѹ��ö�� */
enum socp_enc_dst_output_compress_e {
    SOCP_NO_COMPRESS = 0,
    SOCP_COMPRESS,
};

typedef struct  {
    u32 alloc_enc_src_cnt;    /* SOCP�������Դͨ���Ĵ��� */
    u32 alloc_enc_src_suc_cnt; /* SOCP�������Դͨ���ɹ��Ĵ��� */
    u32 socp_set_enc_dst_cnt;      /* SOCP���ñ���Ŀ��ͨ���Ĵ��� */
    u32 socp_set_enc_dst_suc_cnt;   /* SOCP���ñ���Ŀ��ͨ���ɹ��Ĵ��� */
    u32 socp_set_dec_src_cnt;      /* SOCP���ý���Դͨ���Ĵ��� */
    u32 socp_set_dec_src_suc_cnt;    /* SOCP���ý���Դͨ���ɹ��Ĵ��� */
    u32 socp_alloc_dec_dst_cnt;    /* SOCP�������Ŀ��ͨ���Ĵ��� */
    u32 socp_alloc_dec_dst_suc_cnt; /* SOCP�������Ŀ��ͨ���ɹ��Ĵ��� */
    u32 socp_app_etr_int_cnt;      /* ����APP�жϵĴ��� */
    u32 socp_app_suc_int_cnt;      /* ����APP�жϴ���ɹ��Ĵ��� */
} socp_debug_gbl_state_s;

typedef struct  {
    u32 index;
    u32 int_handle[10];
    u32 up_sem[3][10];
    u32 task_up[3][10];
} SOCP_DEBUG_TRACE_S;

typedef struct  {
    u32 free_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];            /* SOCP�ͷű���Դͨ���ɹ��Ĵ��� */
    u32 soft_reset_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];       /* SOCP��λ����Դͨ���Ĵ��� */
    u32 start_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];           /* SOCP��������Դͨ���Ĵ��� */
    u32 stop_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];            /* SOCP�رձ���Դͨ���Ĵ��� */
    u32 reg_event_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];        /* SOCPע�����Դͨ���¼��Ĵ��� */
    u32 get_wbuf_enter_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];      /* SOCP����Դͨ�����Ի��дbuffer�Ĵ��� */
    u32 get_wbuf_suc_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];      /* SOCP����Դͨ���ɹ����дbuffer�Ĵ��� */
    u32 write_done_enter_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];  /* SOCP����Դͨ������д��buffer�Ĵ��� */
    u32 write_done_suc_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];  /* SOCP����Դͨ���ɹ�д��buffer�Ĵ��� */
    u32 write_done_fail_enc_src_cnt[SOCP_MAX_ENCSRC_CHN]; /* SOCP����Դͨ��д��bufferʧ�ܵĴ��� */
    u32 reg_rd_cb_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];         /* SOCPע�����Դͨ��RDbuffer�ص������Ĵ��� */
    u32 get_rd_buff_enc_src_enter_cnt[SOCP_MAX_ENCSRC_CHN];     /* SOCP����Դͨ�����Ի��Rdbuffer�Ĵ��� */
    u32 get_rd_buff_suc_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];     /* SOCP����Դͨ���ɹ����Rdbuffer�Ĵ��� */
    u32 read_rd_done_enter_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];   /* SOCP����Դͨ������д��Rdbuffer�Ĵ��� */
    u32 read_rd_suc_enc_src_done_cnt[SOCP_MAX_ENCSRC_CHN];   /* SOCP����Դͨ���ɹ�д��Rdbuffer�Ĵ��� */
    u32 read_rd_done_fail_enc_src_cnt[SOCP_MAX_ENCSRC_CHN];  /* SOCP����Դͨ��д��Rdbufferʧ�ܵĴ��� */
    u32 enc_src_task_head_cb_cnt[SOCP_MAX_ENCSRC_CHN];       /* �����лص�����Դͨ����ͷ�����жϴ�������ɵĴ��� */
    u32 enc_src_task_head_cb_ori_cnt[SOCP_MAX_ENCSRC_CHN];    /* �����лص�����Դͨ����ͷ�����жϴ��������� */
    u32 enc_src_task_rd_cb_cnt[SOCP_MAX_ENCSRC_CHN];         /* �����лص�����Դͨ��Rd ����жϴ�������ɵĴ��� */
    u32 enc_src_task_rd_cb_ori_cnt[SOCP_MAX_ENCSRC_CHN];      /* �����лص�����Դͨ��Rd ����жϴ��������� */
    u32 enc_src_task_isr_head_int_cnt[SOCP_MAX_ENCSRC_CHN];      /* ISR�н������Դͨ����ͷ�����жϴ��� */
    u32 enc_src_task_isr_rd_int_cnt[SOCP_MAX_ENCSRC_CHN];        /* ISR�������Դͨ��Rd ����жϴ��� */
} socp_debug_enc_src_s;

typedef struct  {
    u32 socp_reg_event_encdst_cnt[SOCP_MAX_ENCDST_CHN];        /* SOCPע�����Ŀ��ͨ���¼��Ĵ��� */
    u32 socp_reg_readcb_encdst_cnt[SOCP_MAX_ENCDST_CHN];       /* SOCPע�����Ŀ��ͨ�������ݻص������Ĵ��� */
    u32 socp_get_read_buff_encdst_etr_cnt[SOCP_MAX_ENCDST_CHN];   /* SOCP����Ŀ��ͨ�����Զ����ݵĴ��� */
    u32 socp_get_read_buff_encdst_suc_cnt[SOCP_MAX_ENCDST_CHN];   /* SOCP����Ŀ��ͨ���ɹ������ݵĴ��� */
    u32 socp_read_done_encdst_etr_cnt[SOCP_MAX_ENCDST_CHN];   /* SOCP����Ŀ��ͨ������д��Ŀ��buffer�Ĵ��� */
    u32 socp_read_done_encdst_suc_cnt[SOCP_MAX_ENCDST_CHN];   /* SOCP����Ŀ��ͨ��д��Ŀ��buffer�ɹ��Ĵ��� */
    u32 socp_read_done_encdst_fail_cnt[SOCP_MAX_ENCDST_CHN];  /* SOCP����Ŀ��ͨ��д��Ŀ��bufferʧ�ܵĴ��� */
    u32 socp_read_done_zero_encdst_cnt[SOCP_MAX_ENCDST_CHN];  /* SOCP����Ŀ��ͨ��д��Ŀ��buffer size ����0 �Ĵ��� */
    u32 socp_read_done_vld_encdst_cnt[SOCP_MAX_ENCDST_CHN]; /* SOCP����Ŀ��ͨ��д��Ŀ��buffer size ������0 �Ĵ��� */
    u32 socp_encdst_task_trf_cb_cnt[SOCP_MAX_ENCDST_CHN];        /* �����лص�����Ŀ��ͨ����������жϴ�������ɵĴ��� */
    u32 socp_encdst_task_trf_cb_ori_cnt[SOCP_MAX_ENCDST_CHN];     /* �����лص�����Ŀ��ͨ����������жϴ������Ĵ��� */
    u32 socp_encdst_task_ovf_cb_cnt[SOCP_MAX_ENCDST_CHN];        /* �����лص�����Ŀ��ͨ��buf ����жϴ�������ɵĴ��� */
    u32 socp_encdst_task_ovf_cb_ori_cnt[SOCP_MAX_ENCDST_CHN];     /* �����лص�����Ŀ��ͨ��buf ����жϴ��������� */
    u32 socp_encdst_task_isr_trf_int_cnt[SOCP_MAX_ENCDST_CHN];       /* ISR�н������Ŀ��ͨ����������жϴ��� */
    u32 socp_encdst_task_isr_ovf_int_cnt[SOCP_MAX_ENCDST_CHN];       /* ISR�н������Ŀ��ͨ��buf ����жϴ��� */
    u32 socp_encdst_task_thrh_ovf_cb_cnt[SOCP_MAX_ENCDST_CHN]; /* �����лص�����Ŀ��ͨ��buf��ֵ����жϴ�������ɵĴ��� */
    u32 socp_encdst_task_thrh_ovf_cb_ori_cnt[SOCP_MAX_ENCDST_CHN]; /* �����лص�����Ŀ��ͨ��buf��ֵ����жϴ��������� */
    u32 socp_encdst_isr_thrh_ovf_int_cnt[SOCP_MAX_ENCDST_CHN];   /* ISR�н������Ŀ��ͨ��buf��ֵ����жϴ��� */

    /* log2.0 2014-03-19 Begin: */
    BSP_U32 socp_encdst_soft_overtime_ori_cnt[SOCP_MAX_ENCDST_CHN]; /* SOCP����Ŀ��ͨ�������ʱ������� */
    BSP_U32 socp_encdst_soft_overtime_cnt[SOCP_MAX_ENCDST_CHN];    /* SOCP����Ŀ��ͨ�������ʱ������ɴ��� */
                                                                  /* log2.0 2014-03-19 End: */
    u32 socp_encdst_mode_change_fail_cnt[SOCP_MAX_ENCDST_CHN];      /* SOCP����Ŀ��ͨ��ģʽ�л�ʧ�ܼ��� */
} socp_debug_encdst_s;

#ifdef SOCP_DECODE_ENABLE
typedef struct  {
    u32 socp_soft_reset_decsrc_cnt[SOCP_MAX_DECSRC_CHN];       /* SOCP��λ����Դͨ���Ĵ��� */
    u32 socp_soft_start_decsrc_cnt[SOCP_MAX_DECSRC_CHN];           /* SOCP��������Դͨ���Ĵ��� */
    u32 socp_stop_start_decsrc_cnt[SOCP_MAX_DECSRC_CHN];            /* SOCP�رս���Դͨ���Ĵ��� */
    u32 socp_reg_event_decsrc_cnt[SOCP_MAX_DECSRC_CHN];        /* SOCPע�����Դͨ���¼��Ĵ��� */
    u32 socp_get_wbuf_decsrc_etr_cnt[SOCP_MAX_DECSRC_CHN];      /* SOCP����Դͨ�����Ի��дbuffer�Ĵ��� */
    u32 socp_get_wbuf_decsrc_suc_cnt[SOCP_MAX_DECSRC_CHN];      /* SOCP����Դͨ�����дbuffer�ɹ��Ĵ��� */
    u32 socp_write_done_decsrc_etr_cnt[SOCP_MAX_DECSRC_CHN];  /* SOCP����Դͨ������д��buffer�Ĵ��� */
    u32 socp_write_done_decsrc_suc_cnt[SOCP_MAX_DECSRC_CHN];  /* SOCP����Դͨ��д��buffer�ɹ��Ĵ��� */
    u32 socp_write_done_decsrc_fail_cnt[SOCP_MAX_DECSRC_CHN]; /* SOCP����Դͨ��д��bufferʧ�ܵĴ��� */
    u32 socp_reg_rdcb_decsrc_cnt[SOCP_MAX_DECSRC_CHN];         /* SOCPע�����Դͨ��RDbuffer�ص������Ĵ��� */
    u32 socp_get_rdbuf_decsrc_etr_cnt[SOCP_MAX_DECSRC_CHN];     /* SOCP����Դͨ�����Ի��Rdbuffer�Ĵ��� */
    u32 socp_get_rdbuf_decsrc_suc_cnt[SOCP_MAX_DECSRC_CHN];     /* SOCP����Դͨ�����Rdbuffer�ɹ��Ĵ��� */
    u32 socp_read_rd_done_decsrc_etr_cnt[SOCP_MAX_DECSRC_CHN];   /* SOCP����Դͨ������д��Rdbuffer�Ĵ��� */
    u32 socp_read_rd_done_decsrc_suc_cnt[SOCP_MAX_DECSRC_CHN];   /* SOCP����Դͨ��д��Rdbuffer�ɹ��Ĵ��� */
    u32 socp_read_rd_done_decsrc_fail_cnt[SOCP_MAX_DECSRC_CHN];  /* SOCP����Դͨ��д��Rdbufferʧ�ܵĴ��� */
    u32 socp_decsrc_isr_err_int_cnt[SOCP_MAX_DECSRC_CHN];       /* ISR�н������Դͨ�������жϴ��� */
    u32 socp_decsrc_task_err_cb_cnt[SOCP_MAX_DECSRC_CHN];        /* �����е��ý���Դͨ�������жϴ�������ɵĴ��� */
    u32 socp_decsrc_task_err_cb_ori_cnt[SOCP_MAX_DECSRC_CHN];     /* �����е��ý���Դͨ�������жϴ��������� */
} socp_debug_decsrc_s;

typedef struct  {
    u32 socp_free_decdst_cnt[SOCP_MAX_DECDST_CHN];            /* SOCP�ͷŽ���Ŀ��ͨ���ɹ��Ĵ��� */
    u32 socp_reg_event_decdst_cnt[SOCP_MAX_DECDST_CHN];        /* SOCPע�����Ŀ��ͨ���¼��Ĵ��� */
    u32 socp_reg_readcb_decdst_cnt[SOCP_MAX_DECDST_CHN];       /* SOCPע�����Ŀ��ͨ�������ݻص������Ĵ��� */
    u32 socp_get_readbuf_decdst_etr_cnt[SOCP_MAX_DECDST_CHN];   /* SOCP����Ŀ��ͨ�����Զ����ݵĴ��� */
    u32 socp_get_readbuf_decdst_suc_cnt[SOCP_MAX_DECDST_CHN];   /* SOCP����Ŀ��ͨ�������ݳɹ��Ĵ��� */
    u32 socp_read_done_decdst_etr_cnt[SOCP_MAX_DECDST_CHN];   /* SOCP����Ŀ��ͨ������д��Ŀ��buffer�Ĵ��� */
    u32 socp_read_done_decdst_suc_cnt[SOCP_MAX_DECDST_CHN];   /* SOCP����Ŀ��ͨ��д��Ŀ��buffer�ɹ��Ĵ��� */
    u32 socp_read_done_decdst_fail_cnt[SOCP_MAX_DECDST_CHN];  /* SOCP����Ŀ��ͨ��д��Ŀ��bufferʧ�ܵĴ��� */
    u32 socp_read_done_zero_decdst_cnt[SOCP_MAX_DECDST_CHN];  /* SOCP����Ŀ��ͨ��д��Ŀ��buffer size ����0 �Ĵ��� */
    u32 socp_read_done_vld_decdst_cnt[SOCP_MAX_DECDST_CHN]; /* SOCP����Ŀ��ͨ��д��Ŀ��buffer size ������0 �Ĵ��� */
    u32 socp_decdst_isr_trf_int_cnt[SOCP_MAX_DECDST_CHN];       /* ISR�н������Ŀ��ͨ����������жϴ��� */
    u32 socp_decdst_task_trf_cb_cnt[SOCP_MAX_DECDST_CHN];        /* �����лص�����Ŀ��ͨ����������жϴ�������ɵĴ��� */
    u32 socp_decdst_task_trf_cb_ori_cnt[SOCP_MAX_DECDST_CHN];     /* �����лص�����Ŀ��ͨ����������жϴ��������� */
    u32 socp_decdst_isr_ovf_int_cnt[SOCP_MAX_DECDST_CHN];       /* ISR�н������Ŀ��ͨ��buf ����жϴ��� */
    u32 socp_decdst_task_ovf_cb_cnt[SOCP_MAX_DECDST_CHN];        /* �����н������Ŀ��ͨ��buf ����жϴ�������ɵĴ��� */
    u32 socp_decdst_task_ovf_cb_ori_cnt[SOCP_MAX_DECDST_CHN];     /* �����н������Ŀ��ͨ��buf ����жϴ��������� */
} socp_debug_decdst_s;
#endif

/* ������Ϣ */
typedef struct  {
    socp_debug_gbl_state_s socp_debug_gbl;
    socp_debug_enc_src_s socp_debug_encsrc;
    socp_debug_encdst_s socp_debug_enc_dst;
#ifdef SOCP_DECODE_ENABLE
    socp_debug_decsrc_s socp_debug_dec_src;
    socp_debug_decdst_s socp_debug_dec_dst;
#endif
} socp_debug_info_s;



typedef enum {
    SOCP_DST_CHAN_CFG_ADDR = 0,
    SOCP_DST_CHAN_CFG_SIZE,
    SOCP_DST_CHAN_CFG_TIME,
    SOCP_DST_CHAN_CFG_RSV,
    SOCP_DST_CHAN_CFG_BUTT
} socp_dst_chan_cfg_e;

typedef struct {
    struct device *dev;
    struct clk *socp_clk;
    u32 count;
} socp_clk_s;

struct socp_enc_dst_stat_s {
    u32 int_start_slice;
    u32 int_end_slice;
    u32 task_start_slice;
    u32 task_end_slice;
    u32 read_done_start_slice;
    u32 read_done_end_slice;
};

/* �Ĵ���λ���� */

/* Bit masks for registers: ENCSTAT,DECSTAT, channel state */
#define SOCP_CHN_BUSY ((s32)1) /* ͨ��æ */
#define SOCP_CHN_IDLE ((s32)0) /* ͨ���� */

/* ͨ���򿪺궨�壬�����ڱ���Դͨ���ͽ���Ŀ��ͨ�� */
#define SOCP_CHN_ALLOCATED ((s32)1)   /* ͨ���Ѿ����� */
#define SOCP_CHN_UNALLOCATED ((s32)0) /* ͨ��û�з��� */

/* ͨ��ʹ�ܺ궨�壬ͬʱ�����ڱ���Դͨ���ͽ���Դͨ�� */
#define SOCP_CHN_ENABLE ((s32)1)  /* ͨ���� */
#define SOCP_CHN_DISABLE ((s32)0) /* ͨ���ر� */

/* ͨ���Ƿ��Ѿ����� */
#define SOCP_CHN_SET ((s32)1)   /* ͨ���Ѿ����� */
#define SOCP_CHN_UNSET ((s32)0) /* ͨ��û������ */
/* ������·����ʹ��λ */
#define SOCP_ENCSRC_BYPASS_ENABLE ((s32)1)  /* ͨ����·ʹ�� */
#define SOCP_ENCSRC_BYPASS_DISABLE ((s32)0) /* ͨ����·û��ʹ�� */

/* ������·����ʹ��λ */
#define SOCP_DECSRC_DEBUG_ENBALE ((s32)1)  /* ͨ��DEBUG ʹ�� */
#define SOCP_DECSRC_DEBUG_DISBALE ((s32)0) /* ͨ��DEBUG û��ʹ�� */

/* SOCPȫ���жϼĴ����и�λ�����б仯����Ҫ���ռĴ�������и��� */

#define SOCP_CORE0_DEC_TFRINT_MASK ((s32)(1 << 0))        /* ����core0 ��������ж� */
#define SOCP_CORE1_DEC_TFRINT_MASK ((s32)(1 << 1))        /* ����core1 ��������ж� */
#define SOCP_CORE2_DEC_TFRINT_MASK ((s32)(1 << 2))        /* ����core2 ��������ж� */
#define SOCP_CORE0_DEC_ERROR_MASK ((s32)(1 << 3))         /* ����core0 �쳣ȫ���ж� */
#define SOCP_CORE1_DEC_ERROR_MASK ((s32)(1 << 4))         /* ����core1 �쳣ȫ���ж� */
#define SOCP_CORE2_DEC_ERROR_MASK ((s32)(1 << 5))         /* ����core2 �쳣ȫ���ж� */
#define SOCP_CORE0_DEC_OUTOVFINT_MASK ((s32)(1 << 6))     /* ����core0 Ŀ��BUFFER����ж� */
#define SOCP_CORE1_DEC_OUTOVFINT_MASK ((s32)(1 << 7))     /* ����core1 Ŀ��BUFFER����ж� */
#define SOCP_CORE2_DEC_OUTOVFINT_MASK ((s32)(1 << 8))     /* ����core2 Ŀ��BUFFER����ж� */
#define SOCP_APP_ENC_TFRINT_MASK ((s32)(1 << 9))          /* ����core0 ͨ������ȫ���ж� */
#define SOCP_MODEM_ENC_TFRINT_MASK ((s32)(1 << 10))       /* ����core1 ͨ������ȫ���ж� */
#define SOCP_5G_MODEM_ENC_TFRINT_MASK ((s32)(1 << 11))    /* ����core2 ͨ������ȫ���ж� */
#define SOCP_CORE0_ENC_MODESWT_MASK ((s32)(1 << 12))      /* ����core0 ����Ŀ��bufferģʽ�л���� */
#define SOCP_CORE1_ENC_MODESWT_MASK ((s32)(1 << 13))      /* ����core1 ����Ŀ��bufferģʽ�л���� */
#define SOCP_CORE2_ENC_MODESWT_MASK ((s32)(1 << 14))      /* ����core2 ����Ŀ��bufferģʽ�л���� */
#define SOCP_APP_ENC_FLAGINT_MASK ((s32)(1 << 15))        /* ����core0 ��ͷ�������ж� */
#define SOCP_MODEM_ENC_FLAGINT_MASK ((s32)(1 << 16))      /* ����core1 ��ͷ�������ж� */
#define SOCP_5G_MODEM_ENC_FLAGINT_MASK ((s32)(1 << 17))   /* ����core2 ��ͷ�������ж� */
#define SOCP_APP_ENC_OUTOVFINT_MASK ((s32)(1 << 18))      /* ����core0 buffer���ȫ���ж�״̬ */
#define SOCP_MODEM_ENC_OUTOVFINT_MASK ((s32)(1 << 19))    /* ����core1 buffer���ȫ���ж�״̬ */
#define SOCP_5G_MODEM_ENC_OUTOVFINT_MASK ((s32)(1 << 20)) /* ����core2 buffer���ȫ���ж�״̬ */
#define SOCP_APP_ENC_RDINT_MASK ((s32)(1 << 21))          /* ����core0 ��ѯ����ͨ��Rd���ȫ���ж� */
#define SOCP_MODEM_ENC_RDINT_MASK ((s32)(1 << 22))        /* ����core1 ��ѯ����ͨ��Rd���ȫ���ж� */
#define SOCP_5G_MODEM_ENC_RDINT_MASK ((s32)(1 << 23))     /* ����core2 ��ѯ����ͨ��Rd���ȫ���ж� */
#define SOCP_CORE0_PTR_IMG_TIMEOUT_STATE ((s32)(1 << 24)) /* ����core0 ָ�뾵����д��ʱȫ���ж� */
#define SOCP_CORE1_PTR_IMG_TIMEOUT_STATE ((s32)(1 << 25)) /* ����core1 ָ�뾵����д��ʱȫ���ж� */
#define SOCP_CORE2_PTR_IMG_TIMEOUT_STATE ((s32)(1 << 26)) /* ����core2 ָ�뾵����д��ʱȫ���ж� */
#define SOCP_CORE0_RD_IMG_TIMEOUT_STATE ((s32)(1 << 27))  /* ����core0 RDָ�뾵���д��ʱȫ���ж� */
#define SOCP_CORE1_RD_IMG_TIMEOUT_STATE ((s32)(1 << 28))  /* ����core1 RDָ�뾵���д��ʱȫ���ж� */
#define SOCP_CORE2_RD_IMG_TIMEOUT_STATE ((s32)(1 << 29))  /* ����core2 RDָ�뾵���д��ʱȫ���ж� */
#define SOCP_BUS_ERROR_STATE ((s32)(1 << 30))             /* �����쳣ȫ���ж� */

#define SOCP_DEC_SRCINT_NUM (6) /* ����MODEMCPU��ͷ�������ж� */

#define SOCP_TIMEOUT_TRF_LONG_MAX (0x927c0)  // 10min
#define SOCP_TIMEOUT_TRF_LONG_MIN (0xc8)     // 200ms
#define SOCP_TIMEOUT_TRF_SHORT_VAL (0x0a)    // 10ms

#define SOCP_DEC_PKTLGTH_MAX (0x04)  // dec:4096, ��λΪKB
#define SOCP_DEC_PKTLGTH_MIN (0x06)  // dec:6, ��λΪ�ֽ�
#define SOCP_TIMEOUT_MAX (0xffff)
#define SOCP_DEC_MAXPKT_MAX (0x1000)
#define SOCP_DEC_MINPKT_MAX (0x1f)
#define SOCP_ENC_SRC_BUFLGTH_MAX (0x7ffffff)
#define SOCP_ENC_SRC_RDLGTH_MAX (0xffff)
#define SOCP_ENC_DST_BUFLGTH_MAX (0x1fffff)
#define SOCP_ENC_DST_TH_MAX (0x7ff)

#ifdef SOCP_DECODE_ENABLE
#define SOCP_DEC_SRC_BUFLGTH_MAX (0xffff)
#define SOCP_DEC_SRC_RDLGTH_MAX (0xffff)
#define SOCP_DEC_DST_BUFLGTH_MAX (0xffff)
#define SOCP_DEC_DST_TH_MAX (0xff)
#endif

// ����Ŀ��buffer�����ж�״̬�Ĵ���16-22λָʾ��ֵ����ж�
#define SOCP_ENC_DST_BUFF_THRESHOLD_OVF_MASK (0x007f0000)
#define SOCP_ENC_DST_BUFF_OVF_MASK (0x0000007f)
// ��ֵ�жϼĴ�����ʼλ
#define SOCP_ENC_DST_BUFF_THRESHOLD_OVF_BEGIN (16)

#define SOCP_RESET_TIME (1000)
#define SOCP_GBLRESET_TIME (100)

#define SOCP_MAX_ENC_DST_COUNT (100)

extern socp_gbl_state_s g_socp_stat;

/* �������ȼ����� */

#define SOCP_ENCDST_TASK_PRO 81
#define SOCP_ENCSRC_TASK_PRO 79
#define SOCP_ENCDST_IND_TASK_PRO 80
#define SOCP_ENCDST_CNF_TASK_PRO 81
#define SOCP_DECSRC_TASK_PRO 79
#define SOCP_DECDST_TASK_PRO 81

/* �������� */
s32 socp_check_init(void);
s32 socp_check_buf_addr(unsigned long start, unsigned long end);
s32 socp_check_chan_type(u32 para, u32 type);
s32 socp_check_chan_id(u32 para, u32 id);
s32 socp_check_encsrc_chan_id(u32 id);
s32 socp_check_8bytes_align(unsigned long para);
s32 socp_check_chan_priority(u32 para);
s32 socp_check_data_type(u32 para);
s32 socp_check_encsrc_alloc(u32 id);
s32 socp_check_encdst_set(u32 id);
s32 socp_check_data_type_en(u32 param);
s32 socp_check_enc_debug_en(u32 param);
void socp_global_enc_ctrl_init(void);
void socp_reset_enc_chan_wr_addr(u32 chan_id, socp_enc_src_chan_s *enc_chan);
s32 socp_soft_free_encdst_chan(u32 enc_dst_chan_id);
s32 socp_dec_init_task(void);
void socp_enc_handler(void);
void socp_dec_handler(void);
s32 socp_get_index(u32 size, u32 *index);
s32 socp_can_sleep(void);
s32 bsp_socp_encdst_set_cycle(u32 chanid, u32 cycle);
void socp_debug_set_reg_bits(u32 reg, u32 pos, u32 bits, u32 val);
u32 socp_debug_get_reg_bits(u32 reg, u32 pos, u32 bits);
u32 socp_debug_read_reg(u32 reg);
u32 socp_debug_write_reg(u32 reg, u32 data);
void socp_encdst_overflow_info(void);
void socp_help(void);

int socp_encdst_cnf_task(void *data);
int socp_encdst_ind_task(void *data);
int socp_encdst_dump_task(void *data);
int socp_encdst_logserver_task(void *data);
int socp_encsrc_task(void *data);
/* �жϴ����� */
irqreturn_t socp_app_int_handler(int irq, void *dev_info);
s32 socp_enc_get_read_buff(u32 real_chan_id, SOCP_BUFFER_RW_STRU *p_buffer);

#ifdef SOCP_DECODE_ENABLE
s32 socp_check_decsrc_set(u32 id);
s32 socp_check_decdst_alloc(u32 id);
void socp_global_dec_ctrl_init(void);
void socp_reset_dec_chan_wr_addr(u32 chan_id, socp_decsrc_chan_s *dec_chan);
s32 socp_soft_free_decsrc_chan(u32 dec_src_chan_id);
s32 socp_reset_dec_chan(u32 chan_id);
void socp_set_dec_wr_addr(u32 chan_id, void *attr, unsigned long start, unsigned long end);
s32 bsp_socp_decoder_set_dest_chan(SOCP_DECODER_DST_ENUM_U32 dst_chan_id, SOCP_DECODER_DEST_CHAN_STRU *attr);
s32 bsp_socp_decoder_set_src_chan_attr(u32 src_chan_id, SOCP_DECODER_SRC_CHAN_STRU *input_attr);
s32 bsp_socp_decoder_get_err_cnt(u32 dst_chan_id, SOCP_DECODER_ERROR_CNT_STRU *err_cnt);
s32 bsp_socp_set_dec_pkt_lgth(SOCP_DEC_PKTLGTH_STRU *pkt_length);
s32 bsp_socp_set_debug(u32 dec_chan_id, u32 debug_en);
s32 socp_decdst_free_channel(u32 real_chan_id);
s32 socp_decsrc_start(u32 real_chan_id);
s32 socp_decsrc_stop(u32 real_chan_id);
s32 socp_decode_read_data_done(u32 chan_id, u32 read_size);
s32 socp_dec_get_write_buff(u32 real_chan_id, SOCP_BUFFER_RW_STRU *p_rw_buff);
s32 socp_dec_write_done(u32 real_chan_id, u32 write_size);
#endif

#define BSP_REG_SETBITS(base, reg, pos, bits, val)                                        \
    (BSP_REG(base, reg) = (BSP_REG(base, reg) & (~((((u32)1 << (bits)) - 1) << (pos)))) | \
                          ((u32)((val) & (((u32)1 << (bits)) - 1)) << (pos)))
#define BSP_REG_GETBITS(base, reg, pos, bits) ((BSP_REG(base, reg) >> (pos)) & (((u32)1 << (bits)) - 1))

#define SOCP_REG_WRITE(reg, data) (writel(data, (volatile void *)(uintptr_t)(g_socp_stat.base_addr + reg)))
#define SOCP_REG_READ(reg, result) (result = readl((const volatile void *)(uintptr_t)(g_socp_stat.base_addr + reg)))

#define SOCP_REG_SETBITS(reg, pos, bits, val) BSP_REG_SETBITS(g_socp_stat.base_addr, reg, pos, bits, val)
#define SOCP_REG_GETBITS(reg, pos, bits) BSP_REG_GETBITS(g_socp_stat.base_addr, reg, pos, bits)

#define socp_crit(fmt, ...) printk(KERN_ERR "[%s]:" fmt, BSP_MOD(THIS_MODU), ##__VA_ARGS__)
#define socp_error(fmt, ...) \
    printk(KERN_ERR "[%s]:<%s %d>" fmt, BSP_MOD(THIS_MODU), __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define BIT_N(n) (0x01 << (n))
#define SOCP_DEBUG_READ_DONE BIT_N(0)
#define SOCP_DEBUG_READ_CB BIT_N(1)

#define SOCP_DEBUG_TRACE(ulSwitch, ARG1, ARG2, ARG3, ARG4) do { \
    if (FALSE == g_socp_stat.init_flag) {                             \
        socp_error("0x%x, 0x%x, 0x%x, 0x%x\n", ARG1, ARG2, ARG3, ARG4); \
    }                                                                   \
} while (0)

#endif /* end #ifdef DIAG_SYSTEM_5G */

#ifdef __cplusplus
}
#endif

#endif
