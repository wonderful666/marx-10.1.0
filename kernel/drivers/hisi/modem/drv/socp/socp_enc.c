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

#include "product_config.h"
#include <linux/version.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/vmalloc.h>
#include <linux/clk.h>
#include <osl_thread.h>
#include <securec.h>
#include "bsp_version.h"
#include "socp_balong_3.x.h"
#include "bsp_dump.h"
#include "bsp_nvim.h"
#include "soc_socp_adapter.h"
/* log2.0 2014-03-19 Begin: */
#include "acore_nv_stru_drv.h"
#include "bsp_print.h"
#include "bsp_slice.h"
#include "bsp_softtimer.h"
#include "socp_enc_hal.h"
#include "socp_ind_delay.h"
#ifdef CONFIG_DEFLATE
#include "deflate.h"
#endif
/* log2.0 2014-03-19 End */

#define THIS_MODU mod_socp
socp_gbl_state_s g_socp_stat = {0};
extern socp_debug_info_s g_socp_debug_info;

u32 g_enc_dst_stat_cnt;
struct socp_enc_dst_stat_s g_enc_dst_sta[SOCP_MAX_ENC_DST_COUNT];

extern u32 g_deflate_status;
struct platform_device *modem_socp_pdev;
static const struct of_device_id socp_dev_of_match[] = {
    {
        .compatible = "hisilicon,socp_balong_app",
        .data = NULL,
    },
    {},
};
static int socp_driver_probe(struct platform_device *pdev);
static struct platform_driver socp_driver = {
        .driver = {
                   .name = "modem_socp",

                   .owner = THIS_MODULE,
                   .of_match_table = socp_dev_of_match,
                   .probe_type = PROBE_FORCE_SYNCHRONOUS,
        },
        .probe = socp_driver_probe,
};
void socp_global_ctrl_init(void)
{
    socp_ed_op_init_chan();
    socp_es_op_init_chan();


    return;
}

s32 socp_clk_enable(void)
{
#ifndef BSP_CONFIG_PHONE_TYPE
    struct clk *cSocp;
    int ret;

    /* ��SOCPʱ�� */
    cSocp = clk_get(NULL, "socp_clk");
    ret = clk_prepare(cSocp);
    if (BSP_OK != ret) {
        socp_error("[init]clk prepare failed,ret=0x%x\n", ret);
        return ret;
    }
    ret = clk_enable(cSocp);
    if (BSP_OK != ret) {
        socp_error("[init]clk open failed,ret=0x%x\n", ret);
        return ret;
    }
#endif

    return BSP_OK;
}

/*
 * �� �� ��: socp_get_idle_buffer
 * ��������: ��ѯ���л�����
 * �������:  ring_buffer:����ѯ�Ļ���buffer
 *            p_rw_buff:����Ļ���buffer
 * �������: ��
 * �� �� ֵ:  ��
 */
void socp_get_idle_buffer(socp_ring_buff_s *ring_buffer, SOCP_BUFFER_RW_STRU *p_rw_buff)
{
    if (ring_buffer->write < ring_buffer->read) {
        /* ��ָ�����дָ�룬ֱ�Ӽ��� */
        p_rw_buff->pBuffer = (char *)(uintptr_t)(ring_buffer->Start + (u32)ring_buffer->write);
        p_rw_buff->u32Size = (u32)(ring_buffer->read - ring_buffer->write - 1);
        p_rw_buff->pRbBuffer = (char *)BSP_NULL;
        p_rw_buff->u32RbSize = 0;
    } else {
        p_rw_buff->pBuffer = (char *)(uintptr_t)((unsigned long)ring_buffer->Start + (u32)ring_buffer->write);
        p_rw_buff->u32Size = (u32)(ring_buffer->End - (ring_buffer->Start + ring_buffer->write) + 1);
        p_rw_buff->pRbBuffer = (char *)(uintptr_t)ring_buffer->Start;
        p_rw_buff->u32RbSize = ring_buffer->read;
    }

    return;
}

/*
 * �� �� ��: socp_get_data_buffer
 * ��������: ��ȡ���л�����������
 * �������:  ring_buffer       ����ѯ�Ļ���buffer
 *                 p_rw_buff         ����Ļ���buffer
 * �������: ��
 * �� �� ֵ:  ��
 */
void socp_get_data_buffer(socp_ring_buff_s *ring_buffer, SOCP_BUFFER_RW_STRU *p_rw_buff)
{
    if (ring_buffer->read <= ring_buffer->write) {
        /* дָ����ڶ�ָ�룬ֱ�Ӽ��� */
        p_rw_buff->pBuffer = (char *)(uintptr_t)((unsigned long)ring_buffer->Start + (u32)ring_buffer->read);
        p_rw_buff->u32Size = (u32)(ring_buffer->write - ring_buffer->read);
        p_rw_buff->pRbBuffer = (char *)BSP_NULL;
        p_rw_buff->u32RbSize = 0;
    } else {
        /* ��ָ�����дָ�룬��Ҫ���ǻؾ� */
        p_rw_buff->pBuffer = (char *)(uintptr_t)((unsigned long)ring_buffer->Start + (u32)ring_buffer->read);
        p_rw_buff->u32Size =
            (u32)((unsigned long)ring_buffer->End - ((unsigned long)ring_buffer->Start + ring_buffer->read) + 1);
        p_rw_buff->pRbBuffer = (char *)(uintptr_t)ring_buffer->Start;
        p_rw_buff->u32RbSize = ring_buffer->write;
    }

    return;
}

/*
 * �� �� ��: socp_write_done
 * ��������: ���»�������дָ��
 * �������:  ring_buffer       �����µĻ���buffer
 *                 size          ���µ����ݳ���
 * �������: ��
 * �� �� ֵ:  ��
 */
void socp_write_done(socp_ring_buff_s *ring_buffer, u32 size)
{
    ring_buffer->write = (ring_buffer->write + size) % ring_buffer->length;

    return;
}
/*
 * �� �� ��: socp_read_done
 * ��������: ���»������Ķ�ָ��
 * �������:  ring_buffer       �����µĻ���buffer
 *                 size          ���µ����ݳ���
 * �������: ��
 * �� �� ֵ:  ��
 */
void socp_read_done(socp_ring_buff_s *ring_buffer, u32 size)
{
    ring_buffer->read += size;
    if (ring_buffer->read > (u32)(ring_buffer->End - ring_buffer->Start)) {
        ring_buffer->read -= ring_buffer->length;
    }
}

/*
 * �� �� ��: bsp_socp_clean_encsrc_chan
 * ��������: ��ձ���Դͨ����ͬ��V9 SOCP�ӿ�
 * �������: src_chan_id       ����ͨ����
 * �������: ��
 * �� �� ֵ: BSP_OK
 */
u32 bsp_socp_clean_encsrc_chan(u32 src_chan_id)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(src_chan_id);

    socp_es_op_reset_enc_chan(chan_id);

    return BSP_OK;
}

/*

 * �� �� ��: socp_reset_enc_chan
 * ��������: ��λ����ͨ��
 * �������: chan_id       ����ͨ����
 * �������: ��
 * �� �� ֵ: �ͷųɹ����ı�ʶ��
 */
s32 socp_reset_enc_chan(u32 chan_id)
{
    socp_enc_src_chan_s *p_chan = SOCP_ENCSRC_CHAN(chan_id);
    socp_ring_buff_s *ring_buffer = SOCP_ENCSRC_CHAN_BLOCK(chan_id);
    socp_ring_buff_s *ring_buffer_rd = SOCP_ENCSRC_CHAN_RD(chan_id);
    socp_es_op_reset_enc_chan(chan_id);

    ring_buffer->read = 0;
    ring_buffer->write = 0;
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFWPTR(chan_id), 0);
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFRPTR(chan_id), 0);
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFADDR_L(chan_id), ring_buffer->Start);
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFADDR_H(chan_id), (((u64)ring_buffer->Start) >> 32));

    if (p_chan->chan_mode == SOCP_ENCSRC_CHNMODE_LIST) {
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQADDR_L(chan_id), ring_buffer_rd->Start);
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQADDR_H(chan_id), (u32)(((u64)ring_buffer_rd->Start) >> 32));
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQRPTR(chan_id), 0);
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQWPTR(chan_id), 0);
        ring_buffer_rd->read = 0;
        ring_buffer_rd->write = 0;

        SOCP_REG_SETBITS(SOCP_REG_ENCSRC_RDQCFG(chan_id), 0, 16, ring_buffer_rd->length);
        SOCP_REG_SETBITS(SOCP_REG_ENCSRC_RDQCFG(chan_id), 16, 16, 0);
    }

    if (p_chan->ptr_img_en)  // ��ָ�뾵��ʹ�ܣ���Ҫ����ָ�뾵��Ĵ�����0
    {
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFM_RPTRIMG_L(chan_id), 0);
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFM_RPTRIMG_H(chan_id), 0);

        p_chan->ptr_img_en = 0;
    }

    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 1, 2, p_chan->chan_mode);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 3, 1, p_chan->trans_id_en);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 4, 2, p_chan->dst_chan_id);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 8, 2, p_chan->priority);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 10, 1, p_chan->bypass_en);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 11, 1, p_chan->data_type_en);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 12, 1, p_chan->ptr_img_en);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 16, 8, p_chan->data_type);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 31, 1, p_chan->debug_en);

    /* ���ͨ��������״̬��ʹ��ͨ�� */
    if (p_chan->chan_en == SOCP_CHN_ENABLE) {
        SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 0, 1, 1);
    }
    /*lint -restore +e647*/
    return BSP_OK;
}

/*
 * �� �� ��: socp_soft_free_encdst_chan
 * ��������: ���ͷű���Ŀ��ͨ��
 * �������: enc_dst_chan_id       ����ͨ����
 * �������: ��
 * �� �� ֵ: �ͷųɹ����ı�ʶ��
 */
s32 socp_soft_free_encdst_chan(u32 enc_dst_chan_id)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(enc_dst_chan_id);
    socp_ring_buff_s *ring_buffer = SOCP_ENCDST_CHAN_BLOCK(chan_id);
    socp_encdst_chan_s *p_chan = SOCP_ENCDST_CHAN(chan_id);

    /* д����ʼ��ַ��Ŀ��buffer��ʼ��ַ�Ĵ��� */
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFADDR_L(chan_id), (u32)ring_buffer->Start);
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFADDR_H(chan_id), (u32)((u64)ring_buffer->Start >> 32));
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFRPTR(chan_id), 0);
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFWPTR(chan_id), 0);

    ring_buffer->read = 0;
    ring_buffer->write = 0;
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFWPTR(chan_id), 0);
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFRPTR(chan_id), 0);

    p_chan->set_state = SOCP_CHN_UNSET;

    return BSP_OK;
}

/* cov_verified_start */
/*
 * �� �� ��: socp_get_enc_rd_size
 * ��������:  ��ȡ����Դͨ��RDbuffer
 * �������: ��
 * �������: ��
 * �� �� ֵ: �ͷųɹ����ı�ʶ��
 */
u32 socp_get_enc_rd_size(u32 chan_id)
{
    SOCP_BUFFER_RW_STRU buff;
    socp_ring_buff_s *ring_buffer = SOCP_ENCSRC_CHAN_RD(chan_id);

    SOCP_REG_READ(SOCP_REG_ENCSRC_RDQWPTR(chan_id), ring_buffer->write);
    SOCP_REG_READ(SOCP_REG_ENCSRC_RDQRPTR(chan_id), ring_buffer->read);

    socp_es_op_rd_get_data_buffer(chan_id, &buff);
    return (buff.u32Size + buff.u32RbSize);
}

/*
 * �� �� ��: socp_encsrc_rd_handler
 * ��������:  ����Դͨ��RDbuffer�жϴ�����
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��
 */
void socp_encsrc_rd_handler(u32 rd_size, u32 i)
{
    u32 chan_id;

    if (rd_size == g_socp_stat.enc_src_chan[i].last_rd_size) {
        if (g_socp_stat.enc_src_chan[i].rd_cb) {
            chan_id = SOCP_CHAN_DEF(SOCP_CODER_SRC_CHAN, i);
            (void)g_socp_stat.enc_src_chan[i].rd_cb(chan_id);

            g_socp_debug_info.socp_debug_encsrc.enc_src_task_rd_cb_cnt[i]++;
        }

        g_socp_stat.enc_src_chan[i].last_rd_size = 0;
    } else {
        g_socp_stat.enc_src_chan[i].last_rd_size = rd_size;
    }

    return;
}
/* cov_verified_stop */

/*
 * �� �� ��: socp_rdreq_init
 * ��������: ģ��������:����Դ�жϣ�˫��
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��
 */
static void socp_encdst_rdreq_clr(u32 dst_chan)
{
    g_socp_stat.enc_dst_chan[dst_chan].send_req = 0;
}

static void socp_encdst_rdreq_set(u32 dst_chan)
{
    g_socp_stat.enc_dst_chan[dst_chan].send_req = 1;
}

static u32 socp_encdst_rdreq_get(u32 dst_chan)
{
    return (g_socp_stat.enc_dst_chan[dst_chan].send_req) ? 1 : 0;
}

static u32 socp_encdst_rdreq_getall(void)
{
    u32 rst = 0;
    u32 i = 0;

    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        rst |= g_socp_stat.enc_dst_chan[i].send_req;
    }
    return rst;
}
/*
 * �� �� ��: socp_encsrc_task
 * ��������: ģ��������:����Դ�жϣ�˫��
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��
 */
int socp_encsrc_task(void *data)
{
    u32 i;
    u64 int_head_state = 0;
    u32 chan_id;
    unsigned long lock_flag;

    do {
        /* ��ʱ���߱��жϣ����������� */
        if (0 != down_interruptible(&g_socp_stat.enc_src_sem_id)) {
            continue;
        }

        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        int_head_state = g_socp_stat.int_enc_src_header;
        g_socp_stat.int_enc_src_header = 0;

        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);

        /* ��������ͷ'HISI'������� */
        if (int_head_state) {
            for (i = 0; i < SOCP_MAX_ENCSRC_CHN; i++) {
                if (int_head_state & ((u64)1 << i)) {
                    socp_crit("EncSrcHeaderError ChanId = %d", i);
                    if (g_socp_stat.enc_src_chan[i].event_cb) {
                        g_socp_debug_info.socp_debug_encsrc.enc_src_task_head_cb_ori_cnt[i]++;
                        chan_id = SOCP_CHAN_DEF(SOCP_CODER_SRC_CHAN, i);
                        (void)g_socp_stat.enc_src_chan[i].event_cb(chan_id, SOCP_EVENT_PKT_HEADER_ERROR, 0);
                        g_socp_debug_info.socp_debug_encsrc.enc_src_task_head_cb_cnt[i]++;
                    }
                }
            }
        }
    } while (1);

    return 0;
}

static void socp_encdst_task_trf(u32 chan)
{
    u32 chan_id = 0;

    if (g_socp_stat.enc_dst_chan[chan].read_cb) {
        g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_trf_cb_ori_cnt[chan]++;
        chan_id = SOCP_CHAN_DEF(SOCP_CODER_DEST_CHAN, chan);

        if (chan == 1) {
            g_enc_dst_sta[g_enc_dst_stat_cnt].task_start_slice = bsp_get_slice_value();
        }

        (void)g_socp_stat.enc_dst_chan[chan].read_cb(chan_id);

        if (chan == 1) {
            g_enc_dst_sta[g_enc_dst_stat_cnt].task_end_slice = bsp_get_slice_value();
        }

        g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_trf_cb_cnt[chan]++;
    }
}
static void socp_encdst_task_trf_proc(u32 int_trf_state)
{
    u32 i;

    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        /* ���ͨ���Ƿ����� */
        if ((SOCP_CHN_SET == g_socp_stat.enc_dst_chan[i].set_state) &&
            ((int_trf_state & ((u32)1 << i)) || (socp_encdst_rdreq_get(i)))) {
            socp_encdst_rdreq_clr(i);
            socp_encdst_task_trf(i);
        }
    }
}

static void socp_encdst_task_ovf_proc(u32 int_ovf_state)
{
    u32 chan_id = 0;
    u32 read, write, i;

    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        /* ���ͨ���Ƿ����� */
        if (SOCP_CHN_SET == g_socp_stat.enc_dst_chan[i].set_state) {
            if (int_ovf_state & ((u32)1 << i)) {
                if (g_socp_stat.enc_dst_chan[i].event_cb) {
                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_ovf_cb_ori_cnt[i]++;
                    chan_id = SOCP_CHAN_DEF(SOCP_CODER_DEST_CHAN, i);
                    (void)g_socp_stat.enc_dst_chan[i].event_cb(chan_id, SOCP_EVENT_OUTBUFFER_OVERFLOW, 0);

                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_ovf_cb_cnt[i]++;
                }
                if (g_socp_stat.enc_dst_chan[i].read_cb) {
                    /*lint -save -e732*/
                    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFRPTR(i), read);
                    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFWPTR(i), write);
                    /*lint -restore +e732*/
                    SOCP_DEBUG_TRACE(SOCP_DEBUG_READ_DONE, read, write, 0, 0);
                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_trf_cb_ori_cnt[i]++;
                    chan_id = SOCP_CHAN_DEF(SOCP_CODER_DEST_CHAN, i);
                    (void)g_socp_stat.enc_dst_chan[i].read_cb(chan_id);

                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_trf_cb_cnt[i]++;
                }
            }
        }
    }
}

static void socp_encdst_task_thresholdovf_proc(u32 int_thrh_ovf_state)
{
    u32 chan_id = 0;
    u32 read, write, i;

    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        /* ���ͨ���Ƿ����� */
        if (SOCP_CHN_SET == g_socp_stat.enc_dst_chan[i].set_state) {
            if (int_thrh_ovf_state & ((u32)1 << (i + SOCP_ENC_DST_BUFF_THRESHOLD_OVF_BEGIN))) {
                if (g_socp_stat.enc_dst_chan[i].event_cb) {
                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_ovf_cb_ori_cnt[i]++;
                    chan_id = SOCP_CHAN_DEF(SOCP_CODER_DEST_CHAN, i);
                    (void)g_socp_stat.enc_dst_chan[i].event_cb(chan_id, SOCP_EVENT_OUTBUFFER_THRESHOLD_OVERFLOW, 0);

                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_ovf_cb_cnt[i]++;
                }
                if (g_socp_stat.enc_dst_chan[i].read_cb) {
                    /*lint -save -e732*/
                    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFRPTR(i), read);
                    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFWPTR(i), write);
                    /*lint -restore +e732*/
                    SOCP_DEBUG_TRACE(SOCP_DEBUG_READ_DONE, read, write, 0, 0);
                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_trf_cb_ori_cnt[i]++;
                    chan_id = SOCP_CHAN_DEF(SOCP_CODER_DEST_CHAN, i);
                    (void)g_socp_stat.enc_dst_chan[i].read_cb(chan_id);

                    g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_trf_cb_cnt[i]++;
                }
            }
        }
    }
}

/*
 * �� �� ��: socp_encdst_task
 * ��������: ģ��������:����Ŀ�ģ�App��
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��
 */
int socp_encdst_task(void *data)
{
    u32 int_trf_state = 0;
    u32 int_ovf_state = 0;
    u32 int_thrh_ovf_state = 0;
    unsigned long lock_flag;
    u32 i;

    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        /* ���ͨ���Ƿ����� */
        if ((SOCP_CHN_SET == g_socp_stat.enc_dst_chan[i].set_state) && (socp_encdst_rdreq_get(i))) {
            socp_encdst_rdreq_clr(i);
        }
    }

    do {
        /* ��ʱ���߱��жϣ����������� */
        if (0 != down_interruptible(&g_socp_stat.enc_dst_sem_id)) {
            continue;
        }

        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        int_trf_state = g_socp_stat.int_enc_dst_tfr;
        g_socp_stat.int_enc_dst_tfr = 0;
        int_ovf_state = g_socp_stat.int_enc_dst_ovf;
        g_socp_stat.int_enc_dst_ovf = 0;
        int_thrh_ovf_state = g_socp_stat.int_enc_dst_thrh_ovf;
        g_socp_stat.int_enc_dst_thrh_ovf = 0;
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);

        /* ������봫������ж� */
        if (int_trf_state || socp_encdst_rdreq_getall()) {
            socp_encdst_task_trf_proc(int_trf_state);
        }

        /* �������Ŀ�� buffer ����ж� */
        if (int_ovf_state) {
            socp_encdst_task_ovf_proc(int_ovf_state);
        }

        /* �������Ŀ�� buffer ��ֵ����ж� */
        if (int_thrh_ovf_state) {
            socp_encdst_task_thresholdovf_proc(int_thrh_ovf_state);
        }

    } while (1);

    return 0;
}


/*
 * �� �� ��: socp_create_task
 * ��������: socp���񴴽�����
 * �������: ��
 * �������: ��
 * �� �� ֵ: �����ɹ����ı�ʶ��
 */
s32 socp_create_task(s8 *task_name, unsigned long *task_id, socp_task_entry task_func, u32 task_priority,
                     u32 task_stack_size, void *task_param)
{
    struct task_struct *tsk = NULL;
    struct sched_param param;

    tsk = kthread_run(task_func, task_param, task_name);
    if (IS_ERR(tsk)) {
        socp_error("create kthread failed!\n");
        return BSP_ERROR;
    }

    param.sched_priority = task_priority;
    if (BSP_OK != sched_setscheduler(tsk, SCHED_FIFO, &param)) {
        socp_error("sched_setscheduler Error");
        return BSP_ERROR;
    }

    *task_id = (uintptr_t)tsk;

    return BSP_OK;
}

s32 socp_enc_init_task(void)
{
    /* ����Դͨ������ */
    sema_init(&g_socp_stat.enc_src_sem_id, 0);
    if (!g_socp_stat.enc_src_task_id) {
        if (BSP_OK != osl_task_init("EncSrc", SOCP_ENCSRC_TASK_PRO, 0x1000, (OSL_TASK_FUNC)socp_encsrc_task,
                                         NULL, (OSL_TASK_ID*)&g_socp_stat.enc_src_task_id)) {
            socp_error("create socp_encsrc_task failed.\n");
            return BSP_ERR_SOCP_TSK_CREATE;
        }
    }

    /* �������ͨ������ */
    sema_init(&g_socp_stat.enc_dst_sem_id, 0);
    if (!g_socp_stat.enc_dst_task_id) {
        if (BSP_OK != osl_task_init("EncDst", SOCP_ENCDST_TASK_PRO, 0x1000, (OSL_TASK_FUNC)socp_encdst_task,
                                       NULL, (OSL_TASK_ID*)&g_socp_stat.enc_dst_task_id)) {
            socp_error("create socp_encdst_task failed.\n");
            return BSP_ERR_SOCP_TSK_CREATE;
        }
    }

    return BSP_OK;
}

/*
 * �� �� ��: socp_init_task
 * ��������: �������������
 * �������: ��
 * �������: ��
 * �� �� ֵ: �����ɹ����ı�ʶ��
 */
s32 socp_init_task(void)
{
    s32 ret;
    ret = socp_enc_init_task();
    if (ret) {
        socp_error("enc_init_task fail(%d)\n", ret);
        return ret;
    }
    return BSP_OK;
}

/*
 * �� �� ��: socp_handler_encsrc
 * ��������: ����Դͨ����������RD�������ϲ���ɣ�����RD�жϿ��Բ�������
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��
 */
void socp_handler_encsrc(void)
{
    u32 int_flag = 0;
    u32 int_head_err_state_low = 0;
    u32 int_head_err_state_high = 0;
    int is_handle = BSP_FALSE;
    u32 i = 0;
    u32 reg;

    /* read and clear the interrupt flags */
    SOCP_REG_READ(SOCP_REG_GBL_INTSTAT, int_flag);
    /* �����ͷ���� */
    if (int_flag & SOCP_APP_ENC_FLAGINT_MASK) {
        SOCP_REG_READ(SOCP_REG_APP_INTSTAT1_L, int_head_err_state_low);
        SOCP_REG_READ(SOCP_REG_APP_INTSTAT1_H, int_head_err_state_high);
        SOCP_REG_WRITE(SOCP_REG_ENC_RAWINT1_L, int_head_err_state_low);
        SOCP_REG_WRITE(SOCP_REG_ENC_RAWINT1_H, int_head_err_state_high);

        g_socp_stat.int_enc_src_header |= (((u64)int_head_err_state_high) << 32 | int_head_err_state_low);
        is_handle = BSP_TRUE;

        for (i = 0; i < SOCP_MAX_ENCSRC_CHN; i++) {
            if (g_socp_stat.int_enc_src_header & ((u64)1 << i)) {
                /* debugģʽ���ΰ�ͷ�����ж� */
                if (SOCP_REG_GETBITS(SOCP_REG_ENCSRC_BUFCFG(i), 31, 1)) /*lint !e647*/
                {
                    reg = i < 0x20 ? SOCP_REG_APP_MASK1_L : SOCP_REG_APP_MASK1_H;
                    SOCP_REG_SETBITS(reg, i % 0x20, 1, 1);
                }
                g_socp_debug_info.socp_debug_encsrc.enc_src_task_isr_head_int_cnt[i]++;
            }
        }
    }

    /* ���ٴ���RD����жϣ���ʼ��ʱ�������� */
    if (is_handle) {
        up(&g_socp_stat.enc_src_sem_id);
    }

    return;
}


static void socp_encdst_trf_int_handler(int *is_handle)
{
    u32 int_state = 0;
    u32 i, mask, mask2;
    unsigned long lock_flag;

    spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
    SOCP_REG_READ(SOCP_REG_ENC_CORE0_INT0, int_state);
    SOCP_REG_READ(SOCP_REG_ENC_CORE0_MASK0, mask);
    SOCP_REG_WRITE(SOCP_REG_ENC_CORE0_MASK0, (int_state | mask));
    /* ��������ж� */
    SOCP_REG_READ(SOCP_REG_ENC_CORE0_MASK2, mask2);
    SOCP_REG_WRITE(SOCP_REG_ENC_CORE0_MASK2, ((int_state << 16) | mask2));
    SOCP_REG_WRITE(SOCP_REG_ENC_RAWINT0, int_state);

    g_socp_stat.int_enc_dst_tfr |= int_state;
    spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);
    *is_handle = BSP_TRUE;

    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        if (int_state & ((u32)1 << i)) {
            socp_ed_op_pressure_test(i, is_handle);
            if (i == 1) {
                g_enc_dst_sta[g_enc_dst_stat_cnt].int_start_slice = bsp_get_slice_value();
                g_socp_stat.int_enc_dst_isr_trf_cnt++;
            }

            g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_isr_trf_int_cnt[i]++;
        }
    }
}

static void socp_encdst_overflow_int_handler(int *is_handle)
{
    u32 int_state = 0;
    u32 i, mask;
    unsigned long lock_flag;

    SOCP_REG_READ(SOCP_REG_ENC_CORE0_INTSTAT2, int_state);
    // ����Ŀ��buffer��ֵ�жϴ���
    if (0 != (int_state & SOCP_ENC_DST_BUFF_THRESHOLD_OVF_MASK)) {
        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        SOCP_REG_READ(SOCP_REG_ENC_CORE0_MASK2, mask);
        SOCP_REG_WRITE(SOCP_REG_ENC_CORE0_MASK2, (int_state | mask));
        SOCP_REG_WRITE(SOCP_REG_ENC_RAWINT2, int_state);
        g_socp_stat.int_enc_dst_thrh_ovf |= int_state;
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);

        *is_handle = BSP_TRUE;

        for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
            if (int_state & ((u32)1 << (i + SOCP_ENC_DST_BUFF_THRESHOLD_OVF_BEGIN))) {
                g_socp_debug_info.socp_debug_enc_dst.socp_encdst_isr_thrh_ovf_int_cnt[i]++;
            }
        }
    }
    // ����Ŀ��buffer�����ж�
    if (0 != (int_state & SOCP_ENC_DST_BUFF_OVF_MASK)) {
        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        SOCP_REG_READ(SOCP_REG_ENC_CORE0_MASK2, mask);
        SOCP_REG_WRITE(SOCP_REG_ENC_CORE0_MASK2, (int_state | mask));
        SOCP_REG_WRITE(SOCP_REG_ENC_RAWINT2, int_state);
        g_socp_stat.int_enc_dst_ovf |= int_state;
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);

        *is_handle = BSP_TRUE;

        for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
            if (int_state & ((u32)1 << i)) {
                g_socp_debug_info.socp_debug_enc_dst.socp_encdst_task_isr_ovf_int_cnt[i]++;
            }
        }
    }
}

static void socp_encdst_modeswt_int_handler(void)
{
    u32 int_state = 0;
    u32 i, mask, mode_state;
    unsigned long lock_flag;

    spin_lock_irqsave(&g_socp_stat.lock, lock_flag);

    SOCP_REG_READ(SOCP_REG_ENC_CORE0_INT0, int_state);
    SOCP_REG_READ(SOCP_REG_ENC_CORE0_MASK0, mask);
    SOCP_REG_SETBITS(SOCP_REG_ENC_CORE0_MASK0, 16, 7, (((int_state | mask) >> 16) & 0x7f));

    /* ��ԭʼ�ж�״̬ */
    SOCP_REG_SETBITS(SOCP_REG_ENC_RAWINT0, 16, 7, ((int_state >> 16) & 0x7f));

    mask = 0;
    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        SOCP_REG_READ(SOCP_REG_ENCDEST_BUFMODE_CFG(i), mode_state);
        if (mode_state & 0x02) {
            mask |= (1 << i);
        }
    }

    /* ���δ���ѭ��ģʽͨ���Ĵ����жϺ���ֵ����ж� */
    SOCP_REG_SETBITS(SOCP_REG_ENC_CORE0_MASK0, 0, 7, mask);
    SOCP_REG_SETBITS(SOCP_REG_ENC_CORE0_MASK2, 0, 7, mask);
    SOCP_REG_SETBITS(SOCP_REG_ENC_CORE0_MASK2, 16, 7, mask);

    spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);
}

/*
 * �� �� ��: socp_handler_encdst
 * ��������: ����Ŀ���жϴ�����
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��
 */
/*lint -save -e550*/
void socp_handler_encdst(void)
{
    u32 int_flag = 0;
    int is_handle = BSP_FALSE;

    /* ����Ŀ�Ĵ����ж� */
    SOCP_REG_READ(SOCP_REG_GBL_INTSTAT, int_flag);
    if (int_flag & SOCP_APP_ENC_TFRINT_MASK) {
        socp_encdst_trf_int_handler(&is_handle);
    }
    // �����ж�����ֵ�жϹ���һ���Ĵ���
    else if (int_flag & SOCP_APP_ENC_OUTOVFINT_MASK) {
        socp_encdst_overflow_int_handler(&is_handle);
    } else if (g_socp_stat.compress_isr) {
        g_socp_stat.compress_isr();
        return;
    }

    /* ����Ŀ��bufferģʽ�л���� */
    else if (int_flag & SOCP_CORE0_ENC_MODESWT_MASK) {
        socp_encdst_modeswt_int_handler();
    } else {
        is_handle = BSP_FALSE;
    }

    if (is_handle) {
        up(&g_socp_stat.enc_dst_sem_id);
    }
    return;
}
/*lint -restore +e550*/

void socp_enc_handler(void)
{
    socp_handler_encsrc();
    socp_handler_encdst();
}

/*
 * �� �� ��: socp_app_int_handler
 * ��������: APP ���жϴ�����
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��
 */
irqreturn_t socp_app_int_handler(int irq, void *dev_info)
{
    socp_enc_handler();


    return 1;
}

static int socp_driver_probe(struct platform_device *pdev)
{
    dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));  //lint !e598 !e648
    modem_socp_pdev = pdev;
    bsp_socp_ind_delay_init();

    return BSP_OK;
}

int socp_init_dev_irq(void)
{
    struct device_node *dev = NULL;
    int ret, irq;

    ret = platform_driver_register(&socp_driver);
    if (ret) {
        socp_error("driver_register fail,ret=0x%x\n", ret);
        return ret;
    }

    dev = of_find_compatible_node(NULL, NULL, "hisilicon,socp_balong_app");
    if (NULL == dev) {
        socp_error("[init]Socp dev find failed\n");
        return BSP_ERROR;
    }

    g_socp_stat.base_addr = (uintptr_t)of_iomap(dev, 0);
    if (0 == g_socp_stat.base_addr) {
        socp_error("base addr is NULL\n");
        return BSP_ERROR;
    }

    irq = irq_of_parse_and_map(dev, 0);
    ret = request_irq(irq, (irq_handler_t)socp_app_int_handler, 0, "SOCP_APP_IRQ", BSP_NULL);
    if (BSP_OK != ret) {
        socp_error("[init]connect app core int failed(%d)\n", ret);
        return BSP_ERROR;
    }

    return BSP_OK;
}

/*
 * �� �� ��: socp_init
 * ��������: ģ���ʼ������
   ��ʼ��˳�� :
   1.ȫ�ֱ�����ʼ��
   2.ʹ��ʱ�ӣ��ֻ�ƽ̨��kirin�򿪣�������ӡ� MBB�ɵ�ǰ�����ʱ��
   3.��ʼ������
   4.��ʼ������ַ�����ж�(���ж�֮���п����ж��������ϱ�����ʱ��������ַ������״̬ok)
   5.��λ
 * �������: ��
 * �������: ��
 * �� �� ֵ: ��ʼ���ɹ��ı�ʶ��
 */
s32 socp_init(void)
{
    s32 ret;

    if (BSP_TRUE == g_socp_stat.init_flag) {
        return BSP_OK;
    }

    socp_crit("[init]start\n");

    socp_global_ctrl_init();

    ret = socp_clk_enable();
    if (ret) {
        return ret;
    }

    spin_lock_init(&g_socp_stat.lock);

    /* ������������� */
    ret = socp_init_task();
    if (BSP_OK != ret) {
        socp_error("[init]create task failed(0x%x).\n", ret);
        return (s32)ret;
    }

    ret = socp_init_dev_irq();
    if (ret) {
        return ret;
    }

    memset_s(&g_socp_debug_info, sizeof(g_socp_debug_info), 0x0, sizeof(socp_debug_info_s));

    SOCP_REG_READ(SOCP_REG_SOCP_VERSION, g_socp_stat.version);

    SOCP_REG_SETBITS(SOCP_REG_GBLRST, 0, 1, 0x1);

    while (SOCP_REG_GETBITS(SOCP_REG_GBLRST, 0, 1) != 0) {
        socp_error("socp is soft resetting\n");
    }
    SOCP_REG_SETBITS(SOCP_REG_GBLRST, 1, 1, 0x1);

    /* ��socpԴͨ����ͷ���ж� */
    SOCP_REG_WRITE(SOCP_REG_APP_MASK1_L, 0);
    SOCP_REG_WRITE(SOCP_REG_APP_MASK1_H, 0);

    bsp_socp_set_timeout(SOCP_TIMEOUT_TRF, SOCP_TIMEOUT_TRF_SHORT_VAL);

    /* ���ó�ʼ��״̬ */
    g_socp_stat.init_flag = BSP_TRUE;

    socp_crit("[init]ok\n");

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_encdst_dsm_init
 * ��������: socp����Ŀ�Ķ��ж�״̬��ʼ��
 * �������: enc_dst_chan_id: ����Ŀ�Ķ�ͨ����
 *           b_enable: ��ʼ�����ж�״̬
 * �������: ��
 * �� �� ֵ: ��
 */
void bsp_socp_encdst_dsm_init(u32 enc_dst_chan_id, u32 b_enable)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(enc_dst_chan_id);
    socp_encdst_chan_s *p_chan = SOCP_ENCDST_CHAN(real_chan_id);

    if (SOCP_DEST_DSM_DISABLE == b_enable) {
        socp_ed_op_int_disable(real_chan_id);
        p_chan->enable_state = SOCP_DEST_DSM_DISABLE;
    } else if (SOCP_DEST_DSM_ENABLE == b_enable) {
        socp_ed_op_int_enable(real_chan_id);
        p_chan->enable_state = SOCP_DEST_DSM_ENABLE;
    }
}

void bsp_socp_encdst_read_req(u32 chan)
{
    socp_encdst_rdreq_set(chan);
    up(&g_socp_stat.enc_dst_sem_id);
}

/*
 * �� �� ��: bsp_socp_data_send_continue
 * ��������: socp����Ŀ�Ķ������ϱ�ʹ�ܣ���readdone�е���
 *           ����diag����״̬�ж��Ƿ��ϱ�
 * ע    ��: �ú�������ʱ����Ҫ�����߱�֤ͬ��
 * �������: enc_dst_chan_id: ����Ŀ�Ķ�ͨ����
 *           b_enable: �ж�ʹ��
 * �������: ��
 * �� �� ֵ: ��
 */
void bsp_socp_data_send_continue(u32 enc_dst_chan_id)
{
    SOCP_BUFFER_RW_STRU buffer;
    u32 chan = SOCP_REAL_CHAN_ID(enc_dst_chan_id);
    socp_encdst_chan_s *encdst_info = SOCP_ENCDST_CHAN(chan);

    if (SOCP_DEST_DSM_ENABLE == encdst_info->enable_state) {
        socp_enc_get_read_buff(chan, &buffer);
        if (buffer.u32Size + buffer.u32RbSize >= encdst_info->thrhold_low) {
            socp_encdst_rdreq_set(chan);
            up(&g_socp_stat.enc_dst_sem_id);
        } else {
            socp_ed_op_int_enable(chan);
        }
    }
}
/*
 * �� �� ��: bsp_socp_data_send_manager
 * ��������: socp����Ŀ�Ķ��ϱ�����
 * �������: enc_dst_chan_id: ����Ŀ�Ķ�ͨ����
 *           b_enable: �ж�ʹ��
 * �������: ��
 * �� �� ֵ: ��
 */
void bsp_socp_data_send_manager(u32 enc_dst_chan_id, u32 b_enable)
{
    unsigned long lock_flag;

    u32 real_chan_id = SOCP_REAL_CHAN_ID(enc_dst_chan_id);
    socp_encdst_chan_s *p_chan = NULL;

    p_chan = SOCP_ENCDST_CHAN(real_chan_id);
    if ((SOCP_DEST_DSM_DISABLE == b_enable) && (p_chan->enable_state == SOCP_DEST_DSM_ENABLE)) {
        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        socp_ed_op_int_disable(real_chan_id);
        p_chan->enable_state = SOCP_DEST_DSM_DISABLE;
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);
    } else if ((SOCP_DEST_DSM_ENABLE == b_enable) && (p_chan->enable_state == SOCP_DEST_DSM_DISABLE)) {
        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        socp_ed_op_int_enable(real_chan_id);
        p_chan->enable_state = SOCP_DEST_DSM_ENABLE;
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);
    } else {
        ;
    }
}

/*
 * �� �� ��: socp_set_enc_wr_addr
 * ��������: ���ñ���ͨ������ַ�Ĵ����Ͷ�дָ��Ĵ���
 * �������: chan_id: ͨ���ţ�����ͨ�����ͺ�ͨ��ID
 *           attr: ͨ�����ò���
 *           start: ͨ��buffer��ʼ��ַ
 *           end: ͨ��buffer������ַ
 * �������: ��
 * �� �� ֵ: ��
 */
void socp_set_enc_wr_addr(u32 chan_id, void *attr)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(chan_id);
    SOCP_CODER_SRC_CHAN_S *src_attr = (SOCP_CODER_SRC_CHAN_S *)attr;
    uintptr_t addr_low;

    if (chan_type == SOCP_CODER_SRC_CHAN)  // ����Դͨ��
    {
        addr_low = (uintptr_t)src_attr->sCoderSetSrcBuf.pucInputStart;
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFADDR_L(chan_id), addr_low);
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFADDR_H(chan_id), (u32)((u64)addr_low >> 32));
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFWPTR(chan_id), 0);
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFRPTR(chan_id), 0);

        if (SOCP_ENCSRC_CHNMODE_LIST == src_attr->eMode) {
            addr_low = (uintptr_t)src_attr->sCoderSetSrcBuf.pucRDStart;
            SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQADDR_L(chan_id), addr_low);
            SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQADDR_H(chan_id), (u32)((u64)addr_low >> 32));
            SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQRPTR(chan_id), 0);
            SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQWPTR(chan_id), 0);
        }

        socp_es_op_save_encsrc_attr(real_chan_id, src_attr);
    } else if (chan_type == SOCP_CODER_DEST_CHAN)  // ����Ŀ��ͨ��
    {
        socp_encdst_chan_s *p_chan = SOCP_ENCDST_CHAN(real_chan_id);
        socp_ed_op_save_encdst_attr(real_chan_id, (SOCP_CODER_DEST_CHAN_S *)src_attr);
        socp_ed_op_set_encdst_attr(real_chan_id);

        /* ������ͨ���Ѿ����� */
        p_chan->set_state = SOCP_CHN_SET;
    }
}

/*
 * �� �� ��: bsp_socp_coder_set_src_chan
 * ��������: ����Դͨ�����ú���
 * �������: src_attr     ����Դͨ�����ò���
 *           ulSrcChanID  ����Դͨ��ID
 * �������: ��
 * �� �� ֵ: ���뼰���óɹ����ı�ʶ��
 */
s32 bsp_socp_coder_set_src_chan(u32 src_chan_id, SOCP_CODER_SRC_CHAN_S *src_attr)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    socp_enc_src_chan_s *p_chan = SOCP_ENCSRC_CHAN(src_chan_id);

    if ((SOCP_ENCSRC_CHNMODE_CTSPACKET != src_attr->eMode) && (SOCP_ENCSRC_CHNMODE_LIST != src_attr->eMode)) {
        socp_error("chnnel mode(%d) is invalid\n", src_attr->eMode);
        return BSP_ERR_SOCP_INVALID_PARA;
    }

    /* ��λͨ�� */
    socp_es_op_reset_enc_chan(real_chan_id);

    /* ���ñ���Դͨ������ */
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 1, 2, src_attr->eMode);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 3, 1, src_attr->eTransIdEn);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 4, 2, SOCP_REAL_CHAN_ID(src_attr->u32DestChanID));
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 8, 2, src_attr->ePriority);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 10, 1, src_attr->u32BypassEn);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 11, 1, src_attr->eDataTypeEn);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 12, 1, src_attr->ePtrImgEn);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 16, 8, src_attr->eDataType);
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 31, 1, src_attr->eDebugEn);

    if (src_attr->ePtrImgEn) {
        if (SOCP_REG_GETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 12, 1) == 0) {
            return BSP_ERR_SOCP_SET_FAIL;
        }
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFM_RPTRIMG_L(real_chan_id), src_attr->eRptrImgPhyAddr);
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFM_RPTRIMG_H(real_chan_id), (u32)(src_attr->eRptrImgPhyAddr >> 32));
    }

    SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFDEPTH(real_chan_id), (uintptr_t)src_attr->sCoderSetSrcBuf.pucInputEnd -
                                                            (uintptr_t)src_attr->sCoderSetSrcBuf.pucInputStart + 1);

    if (SOCP_ENCSRC_CHNMODE_LIST == src_attr->eMode) {
        SOCP_REG_SETBITS(SOCP_REG_ENCSRC_RDQCFG(real_chan_id), 0, 16,
                         (uintptr_t)src_attr->sCoderSetSrcBuf.pucRDEnd - (uintptr_t)src_attr->sCoderSetSrcBuf.pucRDStart + 1);
    }

    socp_set_enc_wr_addr(real_chan_id, (void *)src_attr);

    /* ���ͨ��״̬ */
    p_chan->alloc_state = SOCP_CHN_ALLOCATED;
    return BSP_OK;
}


/*
 * �� �� ��: bsp_socp_coder_set_dest_chan_attr
 * ��������: ����Ŀ��ͨ�����ú���
 * �������: dst_chan_id      ����Ŀ��ͨ��ID
 *           dst_attr          ����Ŀ��ͨ�����ò���
 * �������: ��
 * �� �� ֵ: ���óɹ����ı�ʶ��
 */
s32 bsp_socp_coder_set_dest_chan_attr(u32 dst_chan_id, SOCP_CODER_DEST_CHAN_S *dst_attr)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);
    socp_encdst_chan_s *p_chan = NULL;

    /* ����������������ٴ�����,ͨ����λ֮��ֻ����һ�� */
    /* ʹ�����ò����������� */
    /*lint -save -e647*/
    p_chan = SOCP_ENCDST_CHAN(chan_id);
    if (p_chan->set_state == SOCP_CHN_SET) {
        socp_error("chan 0x%x can't be set twice!\n", chan_id);
        return BSP_ERR_SOCP_SET_FAIL;
    }

    socp_ed_op_timeout_proc(chan_id, dst_attr->u32EncDstTimeoutMode);

    socp_set_enc_wr_addr(dst_chan_id, (void *)dst_attr);

    bsp_socp_encdst_dsm_init(dst_chan_id, SOCP_DEST_DSM_DISABLE);
    g_socp_debug_info.socp_debug_gbl.socp_set_enc_dst_suc_cnt++;

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_set_timeout
 * ��������: ��ʱ��ֵ���ú���
 * �������:   time_out_en          ���ö���ѡ��ʹ��
 *             time_out        ��ʱ��ֵ
 * �������:
 * �� �� ֵ: ���óɹ����ı�ʶ��
 */
s32 bsp_socp_set_timeout(SOCP_TIMEOUT_EN_ENUM_UIN32 time_out_en, u32 time_out)
{
    return socp_ed_op_set_timeout(time_out_en, time_out);
}

s32 socp_encsrc_free_channel(u32 real_chan_id)
{
    socp_enc_src_chan_s *p_chan = SOCP_ENCSRC_CHAN(real_chan_id);

    if (p_chan->chan_en == SOCP_CHN_ENABLE) {
        socp_error("chan 0x%x is running!\n", real_chan_id);
        return BSP_ERR_SOCP_CHAN_RUNNING;
    }

    (void)socp_reset_enc_chan(real_chan_id);

    p_chan->alloc_state = SOCP_CHN_UNALLOCATED;

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_free_channel
 * ��������: ͨ���ͷź���
 * �������: chan_id       �����ͨ��ָ��
 * �������: ��
 * �� �� ֵ: �ͷųɹ����ı�ʶ��
 */
s32 bsp_socp_free_channel(u32 chan_id)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(chan_id);
    s32 ret;

    if (SOCP_CODER_SRC_CHAN == chan_type) {
        ret = socp_encsrc_free_channel(real_chan_id);
        if (ret) {
            socp_error("socp_encsrc_free_channel fail(0x%x)\n", ret);
            return ret;
        }
    }
    else {
        socp_error("invalid chan type 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return BSP_OK;
}

s32 socp_encsrc_start(u32 real_chan_id)
{
    u32 dst_chan_id, reg;
    socp_enc_src_chan_s *p_chan = SOCP_ENCSRC_CHAN(real_chan_id);
    socp_encdst_chan_s *encdst_info = NULL;

    /*lint -save -e647*/
    dst_chan_id = SOCP_REG_GETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 4, 2);
    encdst_info = SOCP_ENCDST_CHAN(dst_chan_id);

    if (SOCP_CHN_SET != encdst_info->set_state) {
        socp_error("EncDst chan is not set!\n");
        return BSP_ERR_SOCP_DEST_CHAN;
    }

    reg = real_chan_id < 0x20 ? SOCP_REG_ENC_RAWINT1_L : SOCP_REG_ENC_RAWINT1_H;
    SOCP_REG_SETBITS(reg, real_chan_id % 0x20, 1, 1);

    reg = real_chan_id < 0x20 ? SOCP_REG_APP_MASK1_L : SOCP_REG_APP_MASK1_H;
    SOCP_REG_SETBITS(reg, real_chan_id % 0x20, 1, 0);

    if (p_chan->chan_mode == SOCP_ENCSRC_CHNMODE_LIST) {
        reg = real_chan_id < 0x20 ? SOCP_REG_ENC_RAWINT3_L : SOCP_REG_ENC_RAWINT3_H;
        SOCP_REG_SETBITS(reg, real_chan_id % 0x20, 1, 1);
    }
    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 0, 1, 1);

    p_chan->chan_en = SOCP_CHN_ENABLE;

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_start
 * ��������: ������߽�����������
 * �������: src_chan_id      ͨ��ID
 * �������:
 * �� �� ֵ: �����ɹ����ı�ʶ��
 */
s32 bsp_socp_start(u32 src_chan_id)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);
    s32 ret;

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        ret = socp_encsrc_start(real_chan_id);
    }
    else {
        socp_error("invalid chan type(0x%x)\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }
    /*lint -restore +e647*/
    return ret;
}

s32 socp_encsrc_stop(u32 real_chan_id)
{
    u32 reg;
    socp_enc_src_chan_s *p_chan = SOCP_ENCSRC_CHAN(real_chan_id);

    reg = real_chan_id < 0x20 ? SOCP_REG_APP_MASK1_L : SOCP_REG_APP_MASK1_H;
    SOCP_REG_SETBITS(reg, real_chan_id % 0x20, 1, 0);

    if (p_chan->chan_mode == SOCP_ENCSRC_CHNMODE_LIST) {
        reg = real_chan_id < 0x20 ? SOCP_REG_ENC_RAWINT3_L : SOCP_REG_ENC_RAWINT3_H;
        SOCP_REG_SETBITS(reg, real_chan_id % 0x20, 1, 1);
    }

    SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 0, 1, 0);

    p_chan->chan_en = SOCP_CHN_DISABLE;

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_stop
 * ��������: ������߽���ֹͣ����
 * �������: src_chan_id      ͨ��ID
 * �������:
 * �� �� ֵ: ֹͣ�ɹ����ı�ʶ��
 */
s32 bsp_socp_stop(u32 src_chan_id)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);
    s32 ret;

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        ret = socp_encsrc_stop(real_chan_id);
    }
    else {
        socp_error("invalid chan type:0x%x\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }
    /*lint -restore +e647*/
    return ret;
}

/*
 * �� �� ��: bsp_socp_register_event_cb
 * ��������: �쳣�¼��ص�����ע�ắ��
 * �������: chan_id      ͨ��ID
 *           event_cb        �쳣�¼��Ļص�����
 * �������:
 * �� �� ֵ: ע��ɹ����ı�ʶ��
 */
s32 bsp_socp_register_event_cb(u32 chan_id, socp_event_cb event_cb)
{

    u32 real_chan_id = SOCP_REAL_CHAN_ID(chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(chan_id);
    socp_enc_src_chan_s *encsrc_info = NULL;
    socp_encdst_chan_s *encdst_info = NULL;

    switch (chan_type) {
        case SOCP_CODER_SRC_CHAN: /* ����Դͨ�� */
        {
            encsrc_info = SOCP_ENCSRC_CHAN(real_chan_id);
            encsrc_info->event_cb = event_cb;
            break;
        }
        case SOCP_CODER_DEST_CHAN: /* ����Ŀ��ͨ�� */
        {
            encdst_info = SOCP_ENCDST_CHAN(real_chan_id);
            encdst_info->event_cb = event_cb;
            break;
        }


        default: {
            socp_error("invalid chan type: 0x%x!\n", chan_type);
            return BSP_ERR_SOCP_INVALID_CHAN;
        }
    }

    return BSP_OK;
}
s32 socp_enc_get_write_buff(u32 real_chan_id, SOCP_BUFFER_RW_STRU *p_rw_buff)
{
    socp_enc_src_chan_s *p_chan = SOCP_ENCSRC_CHAN(real_chan_id);
    SOCP_REG_READ(SOCP_REG_ENCSRC_BUFWPTR(real_chan_id), p_chan->enc_src_buff.write);
    if (p_chan->ptr_img_en) /* ��ָ�뾵��ʹ�ܣ���Ҫ��ȡ��ָ�뾵���ַ�еĶ�ָ�� */
    {
        p_chan->enc_src_buff.read = *((u32 *)(uintptr_t)p_chan->read_ptr_img_vir_addr);
    } else /* ��ָ�뾵��ʹ�ܣ�ֱ�ӴӼĴ����ж�ȡ��ָ�� */
    {
        SOCP_REG_READ(SOCP_REG_ENCSRC_BUFRPTR(real_chan_id), p_chan->enc_src_buff.read);
    }

    socp_es_op_get_idle_buffer(real_chan_id, p_rw_buff);
    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_get_write_buff
 * ��������: �ϲ��ȡд����buffer����
 * �������: src_chan_id    Դͨ��ID
 * �������: p_rw_buff           ��ȡ��buffer
 * �� �� ֵ: ��ȡ�ɹ����ı�ʶ��
 */
s32 bsp_socp_get_write_buff(u32 src_chan_id, SOCP_BUFFER_RW_STRU *p_rw_buff)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);
    s32 ret;

    /* �жϲ�����Ч�� */
    if (NULL == p_rw_buff) {
        socp_error("the parameter is NULL\n");
        return BSP_ERR_SOCP_NULL;
    }

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        ret = socp_enc_get_write_buff(chan_id, p_rw_buff);
    }
    else {
        socp_error("invalid chan type: 0x%x\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return ret;
}

s32 socp_enc_write_done(u32 real_chan_id, u32 write_size)
{
    SOCP_BUFFER_RW_STRU rw_buff;
    u32 chan_id = SOCP_REAL_CHAN_ID(real_chan_id);
    socp_ring_buff_s *ring_buffer = SOCP_ENCSRC_CHAN_BLOCK(chan_id);

    if (unlikely((write_size & 7) != 0)) /*lint !e730*/
    {
        write_size = (write_size + 8) & (~7);
    }

    SOCP_REG_READ(SOCP_REG_ENCSRC_BUFWPTR(real_chan_id), ring_buffer->write);
    SOCP_REG_READ(SOCP_REG_ENCSRC_BUFRPTR(real_chan_id), ring_buffer->read);

    socp_get_idle_buffer(ring_buffer, &rw_buff);

    if (rw_buff.u32Size + rw_buff.u32RbSize < write_size) {
        socp_error("rw_buff is not enough, write_size=0x%x\n", write_size);
        return BSP_ERR_SOCP_INVALID_PARA;
    }

    /* ���ö�дָ�� */
    socp_es_state_write_done(real_chan_id, write_size);

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_write_done
 * ��������: д������ɺ���
 * �������: src_chan_id    Դͨ��ID
 *           write_size      д�����ݵĳ���
 * �������:
 * �� �� ֵ: ����������ı�ʶ��
 */
s32 bsp_socp_write_done(u32 src_chan_id, u32 write_size)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);
    s32 ret;

    /* �жϲ�����Ч�� */
    if (0 == write_size) {
        socp_error("the write_size is 0\n");
        return BSP_ERR_SOCP_NULL;
    }

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        ret = socp_enc_write_done(chan_id, write_size);
    }
    else {
        socp_error("invalid chan type: 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return ret;
}

/*
 * �� �� ��: bsp_socp_register_rd_cb
 * ��������: RDbuffer�ص�����ע�ắ��
 * �������: src_chan_id    Դͨ��ID
 *           rd_cb            RDbuffer��ɻص�����
 * �������:
 * �� �� ֵ: ע��ɹ����ı�ʶ��
 */
s32 bsp_socp_register_rd_cb(u32 src_chan_id, socp_rd_cb rd_cb)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        if (SOCP_ENCSRC_CHNMODE_LIST == g_socp_stat.enc_src_chan[real_chan_id].chan_mode) {
            /* ���ö�Ӧͨ���Ļص����� */
            g_socp_stat.enc_src_chan[real_chan_id].rd_cb = rd_cb;
        } else {
            socp_error("invalid chan type!\n");
            return BSP_ERR_SOCP_CHAN_MODE;
        }
    } else {
        socp_error("invalid chan type: 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return BSP_OK;
}

/*
 * �� �� ��  : bsp_socp_get_rd_buffer
 * ��������  : ��ȡRDbuffer����
 * �������  : src_chan_id    Դͨ��ID
 * �������  : p_rw_buff           ��ȡ��RDbuffer
 * �� �� ֵ  : ��ȡ�ɹ����ı�ʶ��
 */
/* cov_verified_start */
s32 bsp_socp_get_rd_buffer(u32 src_chan_id, SOCP_BUFFER_RW_STRU *p_rw_buff)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);
    socp_ring_buff_s *ring_buffer = NULL;

    /* �жϲ�����Ч�� */
    if (NULL == p_rw_buff) {
        socp_error("the parameter is NULL\n");
        return BSP_ERR_SOCP_NULL;
    }

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        g_socp_debug_info.socp_debug_encsrc.get_rd_buff_enc_src_enter_cnt[chan_id]++;

        ring_buffer = SOCP_ENCSRC_CHAN_RD(chan_id);

        /* ���ݶ�дָ���ȡbuffer */
        SOCP_REG_READ(SOCP_REG_ENCSRC_RDQRPTR(chan_id), ring_buffer->read);
        SOCP_REG_READ(SOCP_REG_ENCSRC_RDQWPTR(chan_id), ring_buffer->write);
        socp_get_data_buffer(ring_buffer, p_rw_buff);
        g_socp_debug_info.socp_debug_encsrc.get_rd_buff_suc_enc_src_cnt[chan_id]++;
    } else {
        socp_error("invalid chan type: 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_read_rd_done
 * ��������: ��ȡRDbuffer������ɺ���
 * �������: src_chan_id   Դͨ��ID
 *           rd_size      ��ȡ��RDbuffer���ݳ���
 * �������:
 * �� �� ֵ: ��ȡ�ɹ����ı�ʶ��
 */
s32 bsp_socp_read_rd_done(u32 src_chan_id, u32 rd_size)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);
    SOCP_BUFFER_RW_STRU rw_buff;
    socp_ring_buff_s *ring_buffer = NULL;

    /* �жϲ�����Ч�� */
    if (0 == rd_size) {
        socp_error("the rd_size is 0\n");
        return BSP_ERR_SOCP_NULL;
    }

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        socp_enc_src_chan_s *p_chan = NULL;

        g_socp_debug_info.socp_debug_encsrc.read_rd_done_enter_enc_src_cnt[chan_id]++;

        ring_buffer = SOCP_ENCSRC_CHAN_RD(chan_id);
        p_chan = SOCP_ENCSRC_CHAN(chan_id);
        p_chan->last_rd_size = 0;

        /* ���ö�дָ�� */
        SOCP_REG_READ(SOCP_REG_ENCSRC_RDQWPTR(chan_id), ring_buffer->write);
        SOCP_REG_READ(SOCP_REG_ENCSRC_RDQRPTR(chan_id), ring_buffer->read);
        socp_get_data_buffer(ring_buffer, &rw_buff);

        if (rw_buff.u32Size + rw_buff.u32RbSize < rd_size) {
            socp_error("rw_buff is not enough, rd_size=0x%x\n", rd_size);
            g_socp_debug_info.socp_debug_encsrc.read_rd_done_fail_enc_src_cnt[chan_id]++;
            return BSP_ERR_SOCP_INVALID_PARA;
        }

        socp_read_done(ring_buffer, rd_size);

        /* д���ָ�뵽��ָ��Ĵ��� */
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQRPTR(chan_id), ring_buffer->read);

        g_socp_debug_info.socp_debug_encsrc.read_rd_suc_enc_src_done_cnt[chan_id]++;
    } else {
        socp_error("invalid chan type: 0x%x!", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return BSP_OK;
}
/* cov_verified_stop */

// ����Ŀ��ͨ��ר��
/*
 * �� �� ��: bsp_socp_register_read_cb
 * ��������: �����ݻص�����ע�ắ��
 * �������: dst_chan_id  Ŀ��ͨ����ID
 *           read_cb         �����ݻص�����
 * �������:
 * �� �� ֵ: ע��ɹ����ı�ʶ��
 */
s32 bsp_socp_register_read_cb(u32 dst_chan_id, socp_read_cb read_cb)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(dst_chan_id);

    if (SOCP_CODER_DEST_CHAN == chan_type) /* ����ͨ�� */
    {
        /* ���ö�Ӧͨ���Ļص����� */
        g_socp_stat.enc_dst_chan[real_chan_id].read_cb = read_cb;

        g_socp_debug_info.socp_debug_enc_dst.socp_reg_readcb_encdst_cnt[real_chan_id]++;
    }
    else {
        socp_error("invalid chan type: 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return BSP_OK;
}

s32 socp_enc_get_read_buff(u32 real_chan_id, SOCP_BUFFER_RW_STRU *p_buffer)
{
    socp_ring_buff_s *ring_buffer = SOCP_ENCDST_CHAN_BLOCK(real_chan_id);
    socp_encdst_chan_s *p_chan = SOCP_ENCDST_CHAN(real_chan_id);

    g_socp_debug_info.socp_debug_enc_dst.socp_get_read_buff_encdst_etr_cnt[real_chan_id]++;
    /* deflateʹ�ܻ�ȡdeflate buffer */
    if ((SOCP_COMPRESS == p_chan->struCompress.bcompress) && (p_chan->struCompress.ops.getbuffer)) {
        return p_chan->struCompress.ops.getbuffer(p_buffer);
    }

    /* ���ݶ�дָ���ȡbuffer */
    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFRPTR(real_chan_id), ring_buffer->read);
    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFWPTR(real_chan_id), ring_buffer->write);
    socp_get_data_buffer(ring_buffer, p_buffer);

    g_socp_debug_info.socp_debug_enc_dst.socp_get_read_buff_encdst_suc_cnt[real_chan_id]++;

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_get_read_buff
 * ��������: ��ȡ������buffer����
 * �������: dst_chan_id  Ŀ��ͨ��buffer
 * �������: p_buffer        ��ȡ�Ķ�����buffer
 * �� �� ֵ: ��ȡ�ɹ����ı�ʶ��
 */
s32 bsp_socp_get_read_buff(u32 dst_chan_id, SOCP_BUFFER_RW_STRU *p_buffer)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(dst_chan_id);
    s32 ret;

    /* �жϲ�����Ч�� */
    if (NULL == p_buffer) {
        socp_error("the parameter is NULL\n");
        return BSP_ERR_SOCP_NULL;
    }

    p_buffer->u32Size = 0;
    p_buffer->u32RbSize = 0;

    if (SOCP_CODER_DEST_CHAN == chan_type) {
        ret = socp_enc_get_read_buff(chan_id, p_buffer);
    }
    else {
        socp_error("invalid chan type: 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return ret;
}

s32 socp_encode_read_data_done(u32 chan_id, u32 read_size)
{
    SOCP_BUFFER_RW_STRU rw_buff;
    unsigned long lock_flag;
    u32 curmodestate;
    socp_encdst_chan_s *p_chan = NULL;
    socp_ring_buff_s *ring_buffer = NULL;

    g_socp_debug_info.socp_debug_enc_dst.socp_read_done_encdst_etr_cnt[chan_id]++;

    if (chan_id == 1) {
        g_enc_dst_sta[g_enc_dst_stat_cnt].read_done_start_slice = bsp_get_slice_value();
    }

    ring_buffer = SOCP_ENCDST_CHAN_BLOCK(chan_id);
    p_chan = SOCP_ENCDST_CHAN(chan_id);
    /* �ж�deflateʹ�ܣ�deflate readdone */
    if ((SOCP_COMPRESS == p_chan->struCompress.bcompress) && (p_chan->struCompress.ops.readdone)) {
        return p_chan->struCompress.ops.readdone(read_size);
    }

    if (0 == read_size) {
        g_socp_debug_info.socp_debug_enc_dst.socp_read_done_zero_encdst_cnt[chan_id]++;
    } else {
        g_socp_debug_info.socp_debug_enc_dst.socp_read_done_vld_encdst_cnt[chan_id]++;
    }

    spin_lock_irqsave(&g_socp_stat.lock, lock_flag);

    curmodestate = SOCP_REG_GETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(chan_id), 1, 2);
    if (0 != curmodestate) /* ctrl & state ��������ģʽ */
    {
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);
        socp_error("no block mode: curmodestate=0x%x!\n", curmodestate);
        return BSP_OK;
    }

    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFWPTR(chan_id), ring_buffer->write);
    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFRPTR(chan_id), ring_buffer->read);
    socp_get_data_buffer(ring_buffer, &rw_buff);

    if (rw_buff.u32Size + rw_buff.u32RbSize < read_size) {
        SOCP_REG_SETBITS(SOCP_REG_ENC_RAWINT0, chan_id, 1, 1);
        SOCP_REG_SETBITS(SOCP_REG_ENC_CORE0_MASK0, chan_id, 1, 0);
        /* overflow int */
        SOCP_REG_SETBITS(SOCP_REG_ENC_RAWINT2, chan_id + 16, 1, 1);
        SOCP_REG_SETBITS(SOCP_REG_ENC_CORE0_MASK2, chan_id + 16, 1, 0);

        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);
        g_socp_debug_info.socp_debug_enc_dst.socp_read_done_encdst_fail_cnt[chan_id]++;
        socp_error("rw_buff is not enough, read_size=0x%x\n", read_size);
        return BSP_ERR_SOCP_INVALID_PARA;
    }

    /* ���ö�дָ�� */
    socp_read_done(ring_buffer, read_size);

    /* д���ָ�뵽��ָ��Ĵ��� */
    /*lint -save -e732*/
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFRPTR(chan_id), ring_buffer->read);
    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFRPTR(chan_id), ring_buffer->read);
    /*lint -restore +e732*/

    bsp_socp_data_send_continue(chan_id);

    spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);
    g_socp_debug_info.socp_debug_enc_dst.socp_read_done_encdst_suc_cnt[chan_id]++;

    if (chan_id == 1) {
        g_enc_dst_sta[g_enc_dst_stat_cnt].read_done_end_slice = bsp_get_slice_value();
        g_enc_dst_stat_cnt = (g_enc_dst_stat_cnt + 1) % SOCP_MAX_ENC_DST_COUNT;
    }

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_read_data_done
 * ��������: ��������ɺ���
 * �������: dst_chan_id    Ŀ��ͨ��ID
 *           read_size      ��ȡ���ݴ�С
 * �������: ��
 * �� �� ֵ: �����ݳɹ����ı�ʶ��
 */
s32 bsp_socp_read_data_done(u32 dst_chan_id, u32 read_size)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);
    u32 chan_type = SOCP_REAL_CHAN_TYPE(dst_chan_id);
    s32 ret;

    /* ͳ��socpĿ�Ķ���������������ж��ڽӿ��ڲ�ʵ�� */
    socp_encdst_overflow_info();

    if (SOCP_CODER_DEST_CHAN == chan_type) /* ����ͨ�� */
    {
        ret = socp_encode_read_data_done(chan_id, read_size);
        if (ret) {
            return ret;
        }
    }
    else {
        socp_error("invalid chan type: 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return ret;
}
s32 socp_dst_channel_enable(u32 dst_chan_id)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);

    SOCP_REG_SETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(real_chan_id), 0, 1, 1);  // ��������Ŀ��ͨ��

    return BSP_OK;
}
s32 socp_dst_channel_disable(u32 dst_chan_id)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);

    SOCP_REG_SETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(real_chan_id), 0, 1, 0);  //  ȥʹ�� ����Ŀ��ͨ��

    return BSP_OK;
}
s32 bsp_socp_dst_trans_id_disable(u32 dst_chan_id)
{
    u32 real_chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);

    SOCP_REG_SETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(real_chan_id), 5, 1, 0);

    return BSP_OK;
}

void bsp_socp_set_enc_dst_threshold(bool mode, u32 dst_chan_id)
{
    u32 buf_len;
    u32 chan;
    socp_encdst_chan_s *encdst_info = NULL;

    chan = SOCP_REAL_CHAN_ID(dst_chan_id);
    encdst_info = &g_socp_stat.enc_dst_chan[chan];

    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFDEPTH(chan), buf_len);
    if (mode == true) /* trueΪ��Ҫ����ʱ�ϱ��ĳ��� */
    {
        encdst_info->thrhold_high = (buf_len >> 2) * 3;
        encdst_info->thrhold_low = encdst_info->thrhold_high >> 5;
    } else {
        encdst_info->thrhold_high = encdst_info->buf_thrhold;
        encdst_info->thrhold_low = encdst_info->buf_thrhold;
    }
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFTHRESHOLD(chan), encdst_info->thrhold_high);

    socp_crit("set encdst thrh success! 0x%x\n", encdst_info->thrhold_high);

    return;
}

#ifdef CONFIG_DEFLATE
/*
 * �� �� ��: bsp_socp_compress_enable
 * ��������: deflate ѹ��ʹ��
 * �������: ѹ��Ŀ��ͨ��ID
 * �������: ��
 * �� �� ֵ:ѹ���ɹ�����־
 */
s32 bsp_socp_compress_enable(u32 dst_chan_id)
{
    socp_encdst_chan_s *p_chan = NULL;
    u32 real_chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);
    u32 low_idle_state;
    u32 high_idle_state;

#ifndef FEATURE_SOCP_ADDR_64BITS
    u32 start;
#endif
    u32 cnt = 500;
    SOCP_CODER_DEST_CHAN_S attr;

    p_chan = &g_socp_stat.enc_dst_chan[real_chan_id];

    if (SOCP_COMPRESS == p_chan->struCompress.bcompress) {
        socp_error("socp_compress_enable already!\n");
        return BSP_ERROR;
    }
    if ((NULL == p_chan->struCompress.ops.enable) || NULL == p_chan->struCompress.ops.set ||
        NULL == p_chan->struCompress.ops.register_read_cb || NULL == p_chan->struCompress.ops.readdone ||
        NULL == p_chan->struCompress.ops.getbuffer) {
        socp_error("socp_compress_enable invalid!\n");
        return BSP_ERROR;
    }
    /* ͣSOCP */
    SOCP_REG_SETBITS(SOCP_REG_GBLRST, 16, 1, 1);
    /*lint -save -e732*/

    SOCP_REG_READ(SOCP_REG_ENCSTAT_L, low_idle_state);
    SOCP_REG_READ(SOCP_REG_ENCSTAT_H, high_idle_state);
    while ((high_idle_state | low_idle_state) && cnt) {
        SOCP_REG_READ(SOCP_REG_ENCSTAT_L, low_idle_state);
        SOCP_REG_READ(SOCP_REG_ENCSTAT_H, high_idle_state);
        msleep(1);
        cnt--;
    }

    /*lint -restore +e732*/

    /* ��дָ�����ã���ǰ���ݶ��� */
#ifdef FEATURE_SOCP_ADDR_64BITS
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFRPTR(real_chan_id), 0);
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFWPTR(real_chan_id), 0);
    p_chan->enc_dst_buff.read = 0;
#else
    SOCP_REG_READ(SOCP_REG_ENCDEST_BUFADDR(real_chan_id), start);
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFRPTR(real_chan_id), start);
    SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFWPTR(real_chan_id), start);
    p_chan->enc_dst_buff.read = start;
#endif

    attr.u32EncDstThrh = p_chan->threshold;
    attr.sCoderSetDstBuf.pucOutputStart = (u8 *)(uintptr_t)(p_chan->enc_dst_buff.Start);
    attr.sCoderSetDstBuf.pucOutputEnd = (u8 *)(uintptr_t)(p_chan->enc_dst_buff.End);
    attr.sCoderSetDstBuf.u32Threshold = p_chan->buf_thrhold;

    p_chan->struCompress.ops.set(dst_chan_id, &attr);
    p_chan->struCompress.ops.register_read_cb(p_chan->read_cb);
    p_chan->struCompress.ops.register_event_cb(p_chan->event_cb);
    /* ѹ��ʹ�� */
    p_chan->struCompress.ops.enable(dst_chan_id);
    /* ���ѹ��ͨ��ԭʼ�жϣ������ж�״̬ */
    bsp_socp_data_send_manager(dst_chan_id, 0);

    /* ʹ��SOCP */
    /*lint -save -e845*/
    SOCP_REG_SETBITS(SOCP_REG_GBLRST, 16, 1, 0);
    /*lint -restore +e845*/
    p_chan->struCompress.bcompress = SOCP_COMPRESS;
    g_deflate_status = SOCP_COMPRESS;
    return BSP_OK;
}
/*
 * �� �� ��: bsp_socp_compress_disable
 * ��������: deflate ѹ��ֹͣ
 * �������: ѹ��Ŀ��ͨ��ID
 * �������: ��
 * �� �� ֵ:ѹ��ֹͣ�ɹ�����־
 */
s32 bsp_socp_compress_disable(u32 dst_chan_id)
{
    socp_encdst_chan_s *p_chan = NULL;
    u32 real_chan_id = SOCP_REAL_CHAN_ID(dst_chan_id);
    u32 low_idle_state;
    u32 high_idle_state;
    u32 cnt = 500;

    /* �������Ƿ���Ч */
    p_chan = &g_socp_stat.enc_dst_chan[real_chan_id];

    if (SOCP_NO_COMPRESS == p_chan->struCompress.bcompress) {
        socp_error("socp_compress_disable  already!\n");
        return BSP_ERROR;
    }
    if (NULL == p_chan->struCompress.ops.disable || NULL == p_chan->struCompress.ops.clear) {
        socp_error("socp_compress_disable invalid!\n");
        return BSP_ERROR;
    }

    /*lint -save -e732*/
    /* ͣSOCP */
    SOCP_REG_SETBITS(SOCP_REG_GBLRST, 16, 1, 1);

    SOCP_REG_READ(SOCP_REG_ENCSTAT_L, low_idle_state);
    SOCP_REG_READ(SOCP_REG_ENCSTAT_H, high_idle_state);
    while ((high_idle_state | low_idle_state) && cnt) {
        SOCP_REG_READ(SOCP_REG_ENCSTAT_L, low_idle_state);
        SOCP_REG_READ(SOCP_REG_ENCSTAT_H, high_idle_state);
        msleep(1);
        cnt--;
    }


    p_chan->struCompress.ops.disable(dst_chan_id);
    p_chan->struCompress.ops.clear(dst_chan_id);

    bsp_socp_data_send_manager(dst_chan_id, 1);

    /* ʹ��SOCP */
    SOCP_REG_SETBITS(SOCP_REG_GBLRST, 16, 1, 0);
    p_chan->struCompress.bcompress = SOCP_NO_COMPRESS;
    g_deflate_status = SOCP_NO_COMPRESS;
    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_register_compress
 * ��������: ѹ������ע��
 * �������: ע�ắ���ṹ��
 * �������: ��
 * �� �� ֵ:ע��ɹ�����־
 */
s32 bsp_socp_register_compress(socp_compress_ops_stru *ops)
{
    int i;
    g_socp_stat.compress_isr = ops->isr;

    for (i = 0; i < SOCP_MAX_ENCDST_CHN; i++) {
        g_socp_stat.enc_dst_chan[i].struCompress.ops.register_event_cb = ops->register_event_cb;
        g_socp_stat.enc_dst_chan[i].struCompress.ops.register_read_cb = ops->register_read_cb;
        g_socp_stat.enc_dst_chan[i].struCompress.ops.enable = ops->enable;
        g_socp_stat.enc_dst_chan[i].struCompress.ops.disable = ops->disable;
        g_socp_stat.enc_dst_chan[i].struCompress.ops.set = ops->set;
        g_socp_stat.enc_dst_chan[i].struCompress.ops.getbuffer = ops->getbuffer;
        g_socp_stat.enc_dst_chan[i].struCompress.ops.readdone = ops->readdone;
        g_socp_stat.enc_dst_chan[i].struCompress.ops.clear = ops->clear;
    }
    return BSP_OK;
}

#endif

/*
 * �� �� ��: bsp_socp_encdst_set_cycle
 * ��������: SOCPѭ��ģʽ����
 * �������: ͨ���š�ģʽ
 * �������: ��
 * �� �� ֵ: ��
 */
/*lint -save -e647*/
s32 bsp_socp_encdst_set_cycle(u32 chanid, u32 cycle)
{
    u32 mod_state;
    unsigned long lock_flag;
    u32 chan_id = SOCP_REAL_CHAN_ID(chanid);
    u32 waittime = 10000;

    /* �ر��Զ�ʱ���ſأ������ϱ�ģʽ���ò���Ч */
    SOCP_REG_SETBITS(SOCP_REG_CLKCTRL, 0, 1, 0);

    mod_state = SOCP_REG_GETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(chan_id), 2, 1);

    if ((0 == cycle || 1 == cycle) && mod_state) {
        u32 i;

        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        SOCP_REG_SETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(chan_id), 1, 1, 0);
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);

        for (i = 0; i < waittime; i++) {
            mod_state = SOCP_REG_GETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(chan_id), 2, 1);
            if (0 == mod_state) {
                break;
            }
        }

        if (waittime == i) {
            g_socp_debug_info.socp_debug_enc_dst.socp_encdst_mode_change_fail_cnt[chan_id]++;
            socp_error("set encdst cycle off timeout!\n");
            /* �ر��Զ�ʱ���ſأ������ϱ�ģʽ���ò���Ч */
            SOCP_REG_SETBITS(SOCP_REG_CLKCTRL, 0, 1, 1);
            return BSP_ERROR;
        }

        bsp_socp_data_send_manager(chanid, SOCP_DEST_DSM_ENABLE);

    } else if ((2 == cycle) && (!mod_state)) {
        u32 i;

        bsp_socp_data_send_manager(chanid, SOCP_DEST_DSM_DISABLE);

        spin_lock_irqsave(&g_socp_stat.lock, lock_flag);
        SOCP_REG_SETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(chan_id), 1, 1, 1);
        spin_unlock_irqrestore(&g_socp_stat.lock, lock_flag);

        for (i = 0; i < waittime; i++) {
            mod_state = SOCP_REG_GETBITS(SOCP_REG_ENCDEST_BUFMODE_CFG(chan_id), 2, 1);
            if (1 == mod_state) {
                break;
            }
        }

        if (waittime == i) {
            g_socp_debug_info.socp_debug_enc_dst.socp_encdst_mode_change_fail_cnt[chan_id]++;
            socp_error("set encdst cycle on timeout!\n");
            SOCP_REG_SETBITS(SOCP_REG_CLKCTRL, 0, 1, 1);
            return BSP_ERROR;
        }
    }

    /* �ر��Զ�ʱ���ſأ������ϱ�ģʽ���ò���Ч */
    SOCP_REG_SETBITS(SOCP_REG_CLKCTRL, 0, 1, 1);
    return BSP_OK;
}
/*lint -restore +e647*/

/*
 * �� �� ��: bsp_socp_set_debug
 * ��������: ���ñ���Դͨ��debugģʽ����
 * �������: dec_chan_id  ����Դͨ��ID
 *           debug_en    debugģʽʹ�ܱ�ʶ
 * �������:
 * �� �� ֵ: ���óɹ����ı�ʶ��
 */
s32 bsp_socp_set_debug(u32 dec_chan_id, u32 debug_en)
{
    u32 chan_id = SOCP_REAL_CHAN_ID(dec_chan_id);
    /* �жϸ�ͨ����ģʽ��û�д򿪵Ļ����������� */
    if (g_socp_stat.enc_src_chan[chan_id].chan_en) {
        socp_error("DecSrc chan is open, can't set debug bit\n");
        return BSP_ERR_SOCP_SET_FAIL;
    } else {
        /*lint -save -e647*/
        SOCP_REG_SETBITS(SOCP_REG_ENCSRC_BUFCFG(chan_id), 31, 1, debug_en);
        /*lint -restore +e647*/
    }

    return BSP_OK;
}

/*
 * �� �� ��: socp_encdst_overflow_info
 * ��������: ͳ��socpĿ�Ķ���ֵ����
 * �������: ��
 * �������:
 * �� �� ֵ: ��
 */
void socp_encdst_overflow_info(void)
{
    u32 int_state = 0;

    SOCP_REG_READ(SOCP_REG_ENC_RAWINT2, int_state);
    if (((int_state & 0xf0000) >> 16) == 0x2) {
        g_socp_stat.int_enc_dst_thrn_ovf++;
    }
}

/*
 * �� �� ��: bsp_SocpEncDstQueryIntInfo
 * ��������: �ṩ��diag_debug��ѯsocp����ͨ��Ŀ�Ķ��ж���Ϣ
 * �������: ��
 * �������:
 * �� �� ֵ: ��
 */
void bsp_SocpEncDstQueryIntInfo(u32 *trf_info, u32 *thrh_ovf_info)
{
    *trf_info = g_socp_stat.int_enc_dst_isr_trf_cnt;
    *thrh_ovf_info = g_socp_stat.int_enc_dst_thrn_ovf;
}

/*
 * �� �� ��: bsp_clear_socp_encdst_int_info
 * ��������: ���socpĿ�Ķ�����ͳ��ֵ
 * �������: ��
 * �������:
 * �� �� ֵ: ��
 */
void bsp_clear_socp_encdst_int_info(void)
{
    g_socp_stat.int_enc_dst_isr_trf_cnt = 0;
    g_socp_stat.int_enc_dst_thrn_ovf = 0;
}

/*
 * �� �� ��: bsp_socp_get_state
 * ��������: ��ȡSOCP״̬
 * �� �� ֵ: SOCP_IDLE    ����
 *           SOCP_BUSY    æµ
 */
SOCP_STATE_ENUM_UINT32 bsp_socp_get_state(void)
{
    u32 enc_chan_low_state;
    u32 enc_chan_high_state;
    u32 dec_chan_state;

    SOCP_REG_READ(SOCP_REG_ENCSTAT_L, enc_chan_low_state);
    SOCP_REG_READ(SOCP_REG_ENCSTAT_L, enc_chan_high_state);
    SOCP_REG_READ(SOCP_REG_DECSTAT, dec_chan_state);
    if ((enc_chan_low_state != 0) || (enc_chan_high_state != 0) || (dec_chan_state != 0)) {
        return SOCP_BUSY;
    }

    return SOCP_IDLE;
}

u32 bsp_get_socp_ind_dst_int_slice(void)
{
    return g_enc_dst_sta[g_enc_dst_stat_cnt].int_start_slice;
}

s32 bsp_clear_socp_buff(u32 src_chan_id)
{
    u32 chan_type = SOCP_REAL_CHAN_TYPE(src_chan_id);
    u32 real_chan_id = SOCP_REAL_CHAN_ID(src_chan_id);
    u32 reg;
    u32 state;

    /* ����ͨ�� */
    if (SOCP_CODER_SRC_CHAN == chan_type) {
        reg = real_chan_id < 0x20 ? SOCP_REG_ENCSTAT_L : SOCP_REG_ENCSTAT_H;
        state = SOCP_REG_GETBITS(reg, (real_chan_id % 0x20), 1);
        while (state == SOCP_BUSY) {
            state = SOCP_REG_GETBITS(reg, (real_chan_id % 0x20), 1);
        }

        SOCP_REG_READ(SOCP_REG_ENCSRC_BUFRPTR(real_chan_id), reg);
        SOCP_REG_WRITE(SOCP_REG_ENCSRC_BUFWPTR(real_chan_id), reg);

        state = SOCP_REG_GETBITS(SOCP_REG_ENCSRC_BUFCFG(real_chan_id), 1, 2);
        if(state == SOCP_ENCSRC_CHNMODE_LIST) {
            SOCP_REG_READ(SOCP_REG_ENCSRC_RDQWPTR(real_chan_id), reg);
            SOCP_REG_WRITE(SOCP_REG_ENCSRC_RDQRPTR(real_chan_id), reg);
        }
    } else if (SOCP_CODER_DEST_CHAN == chan_type) {/*Ŀ�Ķ�buffer�����Ҫ��hids���Ӳ���û�п���LOGʱ����*/
        SOCP_REG_READ(SOCP_REG_ENCDEST_BUFWPTR(real_chan_id), reg);
        SOCP_REG_WRITE(SOCP_REG_ENCDEST_BUFRPTR(real_chan_id), reg);
    } else {
        socp_error("invalid chan type: 0x%x!\n", chan_type);
        return BSP_ERR_SOCP_INVALID_CHAN;
    }

    return BSP_OK;
}

/*
 * �� �� ��: bsp_socp_set_rate_ctrl
 * ��������: �ô˽ӿ���������SOCP��������
 * �������: rate_ctrl:��������
 * �������: �ޡ�
 * �� �� ֵ: ��
 */
s32 bsp_socp_set_rate_ctrl(DRV_DIAG_RATE_STRU *rate_ctrl)
{
    u32 index = 0;
    u64 value = 0;

    if (NULL == rate_ctrl) {
        socp_error("rate_ctrl is null\n");
        return BSP_ERR_SOCP_INVALID_PARA;
    }

    if (!rate_ctrl->ulRatePeriod) {
        socp_crit("period is 0 invalid period\n");
        return BSP_ERR_SOCP_INVALID_PARA;
    }

    /* �������ڿ��� */
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_RATE_PERIOD, rate_ctrl->ulRatePeriod);

    /* ������ֵ���� */
    value = rate_ctrl->ulRateEn[0];
    for (index = 0; index < 0x20; index++) {
        if (value & (1 >> index)) {
            SOCP_REG_WRITE(SOCP_REG_ENCSRC_RATE_THR(index), rate_ctrl->ulRateThr[index]);
        }
    }

    value = rate_ctrl->ulRateEn[1];
    for (index = 0x20; index < 0x40; index++) {
        if (value & (1 >> (index - 0x20))) {
            SOCP_REG_WRITE(SOCP_REG_ENCSRC_RATE_THR(index), rate_ctrl->ulRateThr[index]);
        }
    }

    /* ����ʹ�� */
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_RATE_EN_L, rate_ctrl->ulRateEn[0]);
    SOCP_REG_WRITE(SOCP_REG_ENCSRC_RATE_EN_H, rate_ctrl->ulRateEn[1]);

    return BSP_OK;
}

#ifndef CONFIG_HISI_BALONG_MODEM_MODULE
module_init(socp_init);
#endif
EXPORT_SYMBOL(g_socp_stat);
EXPORT_SYMBOL(socp_soft_free_encdst_chan);
EXPORT_SYMBOL(bsp_socp_clean_encsrc_chan);
EXPORT_SYMBOL(socp_init);
EXPORT_SYMBOL(bsp_socp_encdst_dsm_init);
EXPORT_SYMBOL(bsp_socp_data_send_continue);
EXPORT_SYMBOL(bsp_socp_data_send_manager);
EXPORT_SYMBOL(bsp_socp_coder_set_src_chan);
EXPORT_SYMBOL(bsp_socp_coder_set_dest_chan_attr);
EXPORT_SYMBOL(bsp_socp_set_timeout);
EXPORT_SYMBOL(bsp_socp_set_debug);
EXPORT_SYMBOL(bsp_socp_free_channel);
EXPORT_SYMBOL(bsp_socp_start);
EXPORT_SYMBOL(bsp_socp_stop);
EXPORT_SYMBOL(bsp_socp_register_event_cb);
EXPORT_SYMBOL(bsp_socp_get_write_buff);
EXPORT_SYMBOL(bsp_socp_write_done);
EXPORT_SYMBOL(bsp_socp_register_rd_cb);
EXPORT_SYMBOL(bsp_socp_get_rd_buffer);
EXPORT_SYMBOL(bsp_socp_read_rd_done);
EXPORT_SYMBOL(bsp_socp_register_read_cb);
EXPORT_SYMBOL(bsp_socp_get_read_buff);
EXPORT_SYMBOL(bsp_socp_read_data_done);
EXPORT_SYMBOL(bsp_socp_get_state);
EXPORT_SYMBOL(bsp_socp_set_rate_ctrl);
EXPORT_SYMBOL(socp_dst_channel_enable);
EXPORT_SYMBOL(socp_dst_channel_disable);

