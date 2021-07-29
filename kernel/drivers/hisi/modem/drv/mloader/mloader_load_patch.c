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
/*
 * File Name       : mloader_load_image.c
 * Description     : load modem image(ccore image),run in ccore
 * History         :
 */
#include <product_config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#ifdef CONFIG_MLOADER_COLD_PATCH
#include <linux/mtd/hisi_nve_interface.h>
#endif
#include <linux/mutex.h>
#include <stddef.h>
#include <asm/dma-mapping.h>
#include <asm/cacheflush.h>
#include <bsp_shared_ddr.h>
#include <bsp_hardtimer.h>
#include <bsp_version.h>
#include <bsp_slice.h>
#include <bsp_ipc.h>
#include <bsp_ddr.h>
#include <bsp_rfile.h>
#include <of.h>
#include <mdrv_sysboot.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <uapi/linux/sched/types.h>
#endif
#include <bsp_cold_patch.h>
#include <securec.h>
#include "mloader_comm.h"
#include "mloader_load_image.h"
#include "mloader_debug.h"
#include "bsp_dump.h"
#include <hi_mloader.h>

#ifdef BSP_CONFIG_PHONE_TYPE
#include <adrv.h>
#endif
#define THIS_MODU mod_mloader

struct cold_patch_info_s g_patch_info;

#ifdef CONFIG_MLOADER_COLD_PATCH

#define COLD_PATCH_DUMP_SIZE (128)
char *g_cold_patch_dump_base = NULL;

struct patch_ret_value_s g_patch_ret[MAX_PATCH];

static DEFINE_MUTEX(cold_patch_proc_lock);

struct cold_patch_info_s *mloader_get_cold_patch_info_st(void)
{
    return &g_patch_info;
}

int bsp_nvem_cold_patch_read(struct cold_patch_info_s *p_cold_patch_info)
{
    int ret = 0;

    struct hisi_nve_info_user nve_cold_patch = {0};
    ssize_t size = sizeof(struct cold_patch_info_s);

    nve_cold_patch.nv_operation = NV_READ;
    nve_cold_patch.nv_number = NVME_COLD_PATCH_ID;
    nve_cold_patch.valid_size = size;

    ret = hisi_nve_direct_access(&nve_cold_patch);
    if (ret) {
        bsp_err("nvem_cold_patch_read:nvem read failed!\n");

        return ret;
    }
    ret = memcpy_s(p_cold_patch_info, size, &(nve_cold_patch.nv_data[0]), size);
    if (ret != 0) {
        bsp_err("nvem_cold_patch_read:memcpy_s failed!\n");
        return ret;
    }

    return 0;
}
int bsp_nvem_cold_patch_write(struct cold_patch_info_s *p_cold_patch_info)
{
    int ret;
    struct hisi_nve_info_user nve_cold_patch = {0};
    ssize_t size = sizeof(struct cold_patch_info_s);

    nve_cold_patch.nv_operation = NV_WRITE;
    nve_cold_patch.nv_number = NVME_COLD_PATCH_ID;
    nve_cold_patch.valid_size = size;

    bsp_err("start update nve for patch!\n");
    ret = memcpy_s(&(nve_cold_patch.nv_data[0]), size, p_cold_patch_info, sizeof(struct cold_patch_info_s));
    if (ret != 0) {
        bsp_err("cold_patch:memcpy_s failed!\n");
        return ret;
    }
    ret = hisi_nve_direct_access(&nve_cold_patch);
    if (ret) {
        bsp_err("nvem_cold_patch_write:nvem write failed!\n");
        return ret;
    }
    bsp_err("end update nve for patch!\n");
    return 0;
}

void bsp_nvem_cold_patch_read_debug(void)
{
    u32 i;

    bsp_err("cold_patch_status: 0x%x\n", g_patch_info.cold_patch_status);
    bsp_err("modem_patch_update_count: 0x%x\n", g_patch_info.modem_patch_update_count);
    bsp_err("modem_update_fail_count: 0x%x\n", g_patch_info.modem_update_fail_count);
    for (i = 0; i < MAX_PATCH; i++) {
        bsp_err("modem_patch_info[%d] patch_exist: 0x%x\n", g_patch_info.modem_patch_info[i].patch_exist);
        bsp_err("modem_patch_info[%d] patch_status: 0x%x\n", g_patch_info.modem_patch_info[i].patch_status);
        bsp_err("patch_ret_value[%d].load_ret_val: 0x%x\n", g_patch_ret[i].load_ret_val);
        bsp_err("patch_ret_value[%d].splice_ret_val: 0x%x\n", g_patch_ret[i].splice_ret_val);
    }
}

bool bsp_modem_cold_patch_is_exist(void)
{
    int i;

    for (i = 0; i < MAX_PATCH; i++) {
        if ((u32)g_patch_info.modem_patch_info[i].patch_exist == 1) {
            return 1;
        }
    }
    return 0;
}

void bsp_modem_cold_patch_update_modem_fail_count(void)
{
    int ret;

    mutex_lock(&cold_patch_proc_lock);

    if (g_cold_patch_dump_base != NULL) {
        if (memcpy_s(g_cold_patch_dump_base, COLD_PATCH_DUMP_SIZE, &g_patch_info, sizeof(struct cold_patch_info_s))) {
            bsp_err("update_modem_fail_count:memcpy_s failed!\n");
        }
    }

    if (!bsp_modem_cold_patch_is_exist()) {
        mutex_unlock(&cold_patch_proc_lock);
        return;
    }

    if (g_patch_info.modem_update_fail_count > 3) {
        mutex_unlock(&cold_patch_proc_lock);
        return;
    }
    if (g_patch_info.modem_update_fail_count < 3) {
        g_patch_info.modem_update_fail_count++;
    }
    ret = bsp_nvem_cold_patch_write(&g_patch_info);
    if (ret)
        bsp_err("bsp_nvem_cold_patch_write :update nevm failed!\n");
    mutex_unlock(&cold_patch_proc_lock);
}

ssize_t modem_imag_patch_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    ssize_t size = sizeof(struct cold_patch_info_s);
    long modem_cold_patch_clear = 0;
    if ((kstrtol(buf, 10, &modem_cold_patch_clear) < 0) && (modem_cold_patch_clear != 0))
        return 0;

    mutex_lock(&cold_patch_proc_lock);

    (void)memset_s(&g_patch_info, size, 0, size);
    ret = bsp_nvem_cold_patch_write(&g_patch_info);
    if (ret) {
        bsp_err("modem_imag_patch_status_store:write nvem failed!\n");
        mutex_unlock(&cold_patch_proc_lock);
        return 0;
    }

    mutex_unlock(&cold_patch_proc_lock);

    return count;
}

ssize_t modem_imag_patch_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u32 ret;
    u32 i;
    char data_buf[20] = {0};
    ssize_t size = 0;
    if (buf == NULL) {
        return 0;
    }

    mutex_lock(&cold_patch_proc_lock);

    if (g_patch_info.cold_patch_status) {
        size = strlen("HOTA_SUCCESS:");
        ret = (u32)strcat_s(buf, PAGE_SIZE, "HOTA_SUCCESS:");
    } else {
        size = strlen("HOTA_ERR_STACK:");
        ret = (u32)strcat_s(buf, PAGE_SIZE, "HOTA_ERR_STACK:");
    }
    size += snprintf_s(data_buf, sizeof(data_buf), sizeof(data_buf) - 1, "%d,", g_patch_info.cold_patch_status) + 1;
    ret |= (u32)strcat_s(buf, PAGE_SIZE, data_buf);

    size += snprintf_s(data_buf, sizeof(data_buf), sizeof(data_buf) - 1, "%d,", g_patch_info.modem_update_fail_count) +
            1;
    ret |= (u32)strcat_s(buf, PAGE_SIZE, data_buf);

    size += snprintf_s(data_buf, sizeof(data_buf), sizeof(data_buf) - 1, "%d,", g_patch_info.nv_rollback) + 1;
    ret |= (u32)strcat_s(buf, PAGE_SIZE, data_buf);

    for(i = 0; i < MAX_PATCH; i++) {
        size += snprintf_s(data_buf, sizeof(data_buf), sizeof(data_buf) - 1, "%d,", i) + 1;
        ret |= (u32)strcat_s(buf, PAGE_SIZE, data_buf);

        size += snprintf_s(data_buf, sizeof(data_buf), sizeof(data_buf) - 1, "%d,",
                       g_patch_info.modem_patch_info[i].patch_exist) + 1;
        ret |= (u32)strcat_s(buf, PAGE_SIZE, data_buf);

        size += snprintf_s(data_buf, sizeof(data_buf), sizeof(data_buf) - 1, "%d,",
                       g_patch_info.modem_patch_info[i].patch_status) + 1;
        ret |= (u32)strcat_s(buf, PAGE_SIZE, data_buf);

        size += snprintf_s(data_buf, sizeof(data_buf), sizeof(data_buf) - 1, "%x,",
                       g_patch_ret[i].load_ret_val) + 1;
        ret |= (u32)strcat_s(buf, PAGE_SIZE, data_buf);
    }

    mutex_unlock(&cold_patch_proc_lock);

    if (ret != EOK) {
        bsp_err("patch_status_show:strcat_s failed\n");
    }

    return size;
}
int mloader_is_patch_image(char *image_name, u32 buf_len)
{
    int len;

    len = strnlen(image_name, buf_len);
    if ((image_name[len - 1] == 'p') && (image_name[len - 2] == '.')) {
        return 1;
    }
    return 0;
}
int mloader_patch_get_image_name(enum modem_patch_type patch_type, char *image_name, u32 len)
{
    int ret = 0;

    image_name[0] = '\0';
    switch (patch_type) {
        case CCORE_PATCH:
            (void)strncpy_s(image_name, len, "modem_lr_ccpu.bin.p", sizeof("modem_lr_ccpu.bin.p"));
            break;
        case LR_PATCH:
            (void)strncpy_s(image_name, len, "modem_lr_ccpu.bin.p", sizeof("modem_lr_ccpu.bin.p"));
            break;
        case NR_PATCH:
            (void)strncpy_s(image_name, len, "modem_nr_ccpu.bin.p", sizeof("modem_nr_ccpu.bin.p"));
            break;
        case LR_PHY_PATCH:
            (void)strncpy_s(image_name, len, "lr_phy.bin.p", sizeof("lr_phy.bin.p"));
            break;
        case RFDSP1_PATCH:
            (void)strncpy_s(image_name, len, "rfdsp_sub6g.bin.p", sizeof("rfdsp_sub6g.bin.p"));
            break;
        case RFDSP2_PATCH:
            (void)strncpy_s(image_name, len, "rfdsp_hf.bin.p", sizeof("rfdsp_hf.bin.p"));
            break;
        case L2HAC_PATCH:
            (void)strncpy_s(image_name, len, "modem_l2hac.bin.p", sizeof("modem_l2hac.bin.p"));
            break;
        case NR_PHY_PATCH:
            (void)strncpy_s(image_name, len, "nr_phy.bin.p", sizeof("nr_phy.bin.p"));
            break;
        case PDE_PDF_PATCH:
            (void)strncpy_s(image_name, len, "pde_nr_pdf.bin.p", sizeof("pde_nr_pdf.bin.p"));
            break;
        case PDE_CSI_PATCH:
            (void)strncpy_s(image_name, len, "pde_nr_csi.bin.p", sizeof("pde_nr_csi.bin.p"));
            break;
        case RFDSP1_ES_PATCH:
            (void)strncpy_s(image_name, len, "rfdsp_sub6g_es.bin.p", sizeof("rfdsp_sub6g_es.bin.p"));
            break;
        case RFDSP2_ES_PATCH:
            (void)strncpy_s(image_name, len, "rfdsp_hf_es.bin.p", sizeof("rfdsp_hf_es.bin.p"));
            break;
        default:
            (void)strncpy_s(image_name, len, "", sizeof(""));
    }
    return ret;
}

int mloader_patch_change_pt_to_ct(enum modem_patch_type patch_type, u32 *core_type)
{
    int ret = 0;
    switch (patch_type) {
        case LR_PATCH:
            *core_type = MODEM_COLD_PATCH;
            break;
        case NR_PATCH:
        case L2HAC_PATCH:
        case LR_PHY_PATCH:
        case RFDSP1_PATCH:
        case RFDSP2_PATCH:
        case NR_PHY_PATCH:
        case PDE_PDF_PATCH:
        case PDE_CSI_PATCH:
        case RFDSP1_ES_PATCH:
        case RFDSP2_ES_PATCH:
            *core_type = MODEM_COMM_IMG;
            break;
        default:
            ret = -1;
    }
    return ret;
}

u32 mloader_patch_get_type(const char *file_name)
{
    u32 i;
    char image_name[MLOADER_FILE_NAME_LEN];
    char f_name[MLOADER_FILE_NAME_LEN];

    for (i = 0; i < MAX_PATCH; i++) {
        (void)mloader_patch_get_image_name(i, image_name, MLOADER_FILE_NAME_LEN);
        mloader_get_file_name(f_name, image_name, 0);
        if (strncmp((char *)f_name, (char *)file_name, MLOADER_FILE_NAME_LEN) == 0) {
            return i;
        }
    }
    return i;
}

u32 mloader_patch_get_load_position_offset(enum SVC_SECBOOT_IMG_TYPE ecoretype, u32 ddr_size, int img_size)
{
    if (img_size > ddr_size) {
        bsp_err("param img_size is too large,soc_type = MODEM,img_size = 0x%x,ddr_size = 0x%x!\n", img_size, ddr_size);
        return 0;
    }

    if (ecoretype == MODEM_COLD_PATCH) {
        if (g_patch_info.modem_patch_info[LR_PATCH].patch_exist == 0) {
            return 0;
        }
        /* ������patch�������DDR�ռ����λ�� */
        return (u32)(ddr_size - (u32)img_size);
    } else if (ecoretype == MODEM) {
        /*
         * ����Ѿ�����CCORE��������MODEM DDR�ռ�����λ�ô��CCORE��������CCORE��������֮�ϴ��ԭCCORE��������ԭ�����
         * ����λ��=ddr_size - CCORE���������С - ԭCCORE�����С��
         * ���CCORE�������񲻴��ڻ����ʧ�ܣ�ԭCCORE������ص�MODEM DDR�ռ�����λ��
         */
        if (g_patch_info.modem_patch_info[LR_PATCH].patch_status == PUT_PATCH)
            return (u32)(ddr_size - g_patch_info.lr_patch_size - (u32)img_size);
        else
            return (u32)(ddr_size - (u32)img_size);
    } else {
        return 0;
    }
}

int mloader_load_patch_image(enum modem_patch_type patch_type, mloader_img_icc_info_s *mloader_msg, u32 index)
{
    s32 ret;
    u32 err;
    char file_name[MLOADER_FILE_NAME_LEN] = {0};
    char image_name[MLOADER_FILE_NAME_LEN] = {0};
    u32 core_type;
    ccpu_img_info_s *image = NULL;
    mloader_addr_s *mloader_addr;

    mloader_addr = bsp_mloader_get_addr();
    file_name[0] = '\0';
    /* get patch path name */
    err = (u32)strncat_s(file_name, MLOADER_FILE_NAME_LEN, MLOADER_COLD_PATCH_PATH,
                         strnlen(MLOADER_COLD_PATCH_PATH, MLOADER_FILE_NAME_LEN));

    /* get patch file name */
    ret = mloader_patch_get_image_name(patch_type, image_name, MLOADER_FILE_NAME_LEN);
    if (ret) {
        mloader_print_err("fail to get patch(%d) name\n", patch_type);
        return ret;
    }
    err |= (u32)strncat_s(file_name, MLOADER_FILE_NAME_LEN, image_name, strnlen(image_name, MLOADER_FILE_NAME_LEN));

    if (err != EOK) {
        mloader_print_err("error:strncat_s %s failed when getting cold_patch name!\n", file_name);
        return -1;
    }

    if (!bsp_access((s8 *)file_name, RFILE_RDONLY)) {
        g_patch_info.modem_patch_info[patch_type].patch_exist = 1;
        if (g_patch_info.modem_update_fail_count < 3) {
            if (patch_type == LR_PATCH) {
                /* modem��patch��Ҫ�ŵ�modem��ddr�ռ䣬���Ի�õ���modem��image��Ϣ */
                image = mloader_get_image_info(MLOADER_LR_CCPU_IMG);
                if (image == NULL) {
                    mloader_print_err("can't find image, %s\n", image_name);
                    ret = MLOADER_INTERFACE_ERROR;
                    return ret;
                }

                ret = mloader_patch_change_pt_to_ct(patch_type, &core_type);
                if (ret) {
                    mloader_print_err("fail to get core type, patch type is %d\n", patch_type);
                    ret = -1;
                } else {
                    ret = load_image(file_name, core_type, image->ddr_addr, image->ddr_size, index);
                }
            } else {
                ret = mloader_read_file(file_name, mloader_msg->image_offset, mloader_msg->image_size,
                                        (char *)(mloader_addr->mloader_secboot_virt_addr[index]));
                if (ret != mloader_msg->image_size) {
                    mloader_print_err("%s,image_addr:0x%x,ddr_size:0x%x,image_size:0x%x,image_offset:0x%x", file_name,
                                      mloader_msg->image_addr, mloader_msg->ddr_size, mloader_msg->image_size,
                                      mloader_msg->image_offset);
                    mloader_print_err("%s get data fail, ret = %d\n", file_name, ret);
                } else {
                    ret = 0;
                }
            }
            g_patch_ret[patch_type].load_ret_val = ret;
            if (ret) {
                mloader_print_err("load patch img failed, patch_type %d\n", patch_type);
                g_patch_info.modem_patch_info[patch_type].patch_status = LOAD_PATCH_FAIL;
            } else {
                mloader_update_ccore_cold_patch_debug_info();
                g_patch_info.modem_patch_info[patch_type].patch_status = PUT_PATCH;
                g_patch_info.lr_patch_size = mloader_get_file_size(file_name);
                g_patch_info.lr_patch_size -= SECBOOT_VRL_TABLE_SIZE;
            }
        } else {
            mloader_print_err("fail_count = %d,not load patch img, patch_type %d\n",
                              g_patch_info.modem_update_fail_count, patch_type);
            g_patch_info.modem_patch_info[patch_type].patch_status = NOT_LOAD_PATCH;
        }
    } else {
        ret = -1;
        g_patch_info.modem_patch_info[patch_type].patch_exist = 0;
        mloader_print_err("patch img not exist, patch_type %d, patch_name %s.\n", patch_type, file_name);
        g_patch_info.modem_patch_info[patch_type].patch_status = NOT_LOAD_PATCH;
    }
    return ret;
}
void mloader_update_modem_cold_patch_status(enum modem_patch_type epatch_type)
{
    int i = 0;

    if (epatch_type >= MAX_PATCH) {
        return;
    }

    g_patch_info.cold_patch_status = 0;

    if (g_patch_info.modem_update_fail_count >= 3)
        return;

    if (g_patch_info.modem_patch_info[epatch_type].patch_status == PUT_PATCH)
        g_patch_info.modem_patch_info[epatch_type].patch_status = PUT_PATCH_SUCESS;

    /*
     * ����������񲻴��ڻ򲹶�����δ���أ�������״̬ΪNOT_LOAD_PATCH��������������뾵��ƴ�ӳɹ���������״̬ΪPUT_PATCH_SUCESS��
     * ������в��������״̬ΪNOT_LOAD_PATCH����ʾδ���صĲ������񣬽�cold_patch_status����Ϊ0��
     * ������в��������״̬ΪNOT_LOAD_PATCH��PUT_PATCH_SUCESS������һ����������״̬ΪPUT_PATCH_SUCESS������ʾ����������ԭ����ƴ�ӳɹ�����cold_patch_status����Ϊ1
     */
    for (i = 0; i < MAX_PATCH; i++) {
        if ((g_patch_info.modem_patch_info[i].patch_status != PUT_PATCH_SUCESS) &&
            (g_patch_info.modem_patch_info[i].patch_status != NOT_LOAD_PATCH)) {
            mloader_print_err("modify_modem_cold_patch_status,line = %d, patch_type = 0x%x, patch_status = %d\n",
                              __LINE__, i, g_patch_info.modem_patch_info[i].patch_status);
            return;
        }
    }
    for (i = 0; i < MAX_PATCH; i++) {
        if ((g_patch_info.modem_patch_info[i].patch_exist) &&
            (g_patch_info.modem_patch_info[i].patch_status != NOT_LOAD_PATCH)) {
            g_patch_info.cold_patch_status = 1;
            return;
        }
    }
    return;
}

void mloader_record_cold_patch_splicing_ret_val(enum modem_patch_type epatch_type, int value)
{
    if (epatch_type >= MAX_PATCH) {
        return;
    }

    if (g_patch_info.modem_patch_info[epatch_type].patch_status == PUT_PATCH) {
        g_patch_ret[epatch_type].splice_ret_val = value;
        g_patch_info.modem_update_fail_count = 3;
        g_patch_info.cold_patch_status = 0;
    }
}

u32 mloader_patch_get_load_fail_time(void)
{
    return (u32)g_patch_info.modem_update_fail_count;
}

int mloader_load_patch(enum modem_patch_type epatch_type, mloader_img_icc_info_s *mloader_msg, u32 index)
{
    int ret = 0;
    u32 cold_patch_stamp;
    u32 err;
    char file_name[MLOADER_FILE_NAME_LEN] = {0};
    char image_name[MLOADER_FILE_NAME_LEN] = {0};
    file_name[0] = '\0';

    /* get patch path name */
    err = (u32)strncat_s(file_name, MLOADER_FILE_NAME_LEN, MLOADER_COLD_PATCH_PATH,
                         strnlen(MLOADER_COLD_PATCH_PATH, MLOADER_FILE_NAME_LEN));

    /* get patch file name */
    ret = mloader_patch_get_image_name(epatch_type, image_name, MLOADER_FILE_NAME_LEN);
    if (ret) {
        mloader_print_err("fail to get patch(%d) name\n", epatch_type);
        return ret;
    }
    err |= (u32)strncat_s(file_name, MLOADER_FILE_NAME_LEN, image_name, strnlen(image_name, MLOADER_FILE_NAME_LEN));

    if (err != EOK) {
        mloader_print_err("error:strncat_s %s failed when getting cold_patch name!\n", file_name);
        return -1;
    }

    /* get patch info */
    cold_patch_stamp = bsp_get_slice_value();
    mloader_print_err("%d patch load start at 0x%x\n", epatch_type, cold_patch_stamp);
    if (bsp_nvem_cold_patch_read(&g_patch_info)) {
        ret = memset_s(&g_patch_info, sizeof(g_patch_info), 0, sizeof(g_patch_info));
        if (ret != EOK) {
            mloader_print_err("failed to memset_s cold patch info, ret 0x%x\n", ret);
            return ret;
        }
    }

    if (bsp_access((s8 *)file_name, RFILE_RDONLY)) {
        mloader_print_err("patch img not exist, patch_name %s.\n", image_name);
        return 0;
    }
    /* load patch */
    ret = mloader_load_patch_image(epatch_type, mloader_msg, index);
    /* verify patch */
    if ((epatch_type == LR_PATCH) && (g_patch_info.modem_patch_info[LR_PATCH].patch_status == PUT_PATCH)) {
        /*
         * ��patch����ת��Ϊcore���ʹ�����ȫos����ȫos����֪�������ĸ�ccore��patch��
         * ͳһͨ��comm patch���������comm patch��������Ҫ��patch���ͼ���������
         * У����ɺ󷵻ظ�ccore��
         */
        ret = mloader_verify_soc_image(MODEM_COLD_PATCH, 0);
        if (ret) {
            /* ���У��ʧ�ܣ���ô���ټ��غ�ƴ�Ӳ��� */
            mloader_record_cold_patch_splicing_ret_val(LR_PATCH, ret);
            mloader_print_err("verify image MODEM_COLD_PATCH fail, ret = 0x%x\n", ret);
            return ret;
        }
        mloader_print_err("verify image MODEM_COLD_PATCH success\n");
    }
    /* ���patch �����ڣ�load patch image�����᷵�ظ�������������������쳣�������ڷ��ؼ��ؽ��֮ǰ����Ҫ��ret�ó�0�ٷ��� */
    if (g_patch_info.modem_patch_info[epatch_type].patch_exist == 0) {
        ret = 0;
    }
    return ret;
}

int bsp_modem_is_reboot_machine(void)
{
    int ret;
    mutex_lock(&cold_patch_proc_lock);
    if (g_patch_info.nv_rollback == 0) {
        if (((u32)g_patch_info.modem_patch_info[NV_PATCH].patch_exist == 1) && \
            ((u32)g_patch_info.modem_patch_info[NV_PATCH].patch_status == PUT_PATCH_SUCESS) && \
            (g_patch_info.modem_update_fail_count >= 3)) {
            g_patch_info.nv_rollback = 1;
            ret = bsp_nvem_cold_patch_write(&g_patch_info);
            if (ret) {
                bsp_err("bsp_modem_is_reboot_machine :update nevm failed!\n");
            }
            mutex_unlock(&cold_patch_proc_lock);
            return 1;
        }
    }
    mutex_unlock(&cold_patch_proc_lock);
    return 0;
}

int bsp_modem_cold_patch_init(void)
{
    g_cold_patch_dump_base = (char *)bsp_dump_register_field(DUMP_MODEMAP_COLD_PATCH, "cold_patch",
                                                             COLD_PATCH_DUMP_SIZE, 0);
    /*
     * ϵͳ�쳣������system_errorʱ�����ʶ���в������ͻ��������ӿ�
     * ������ӿ�һ�θ���һ�δ򲹶�ʧ�ܵļ�����  ���ʧ�ܴ����ﵽ3�Σ��´�����ʱ�Ͳ��򲹶���
     */
    (void)bsp_dump_register_hook("modem_cold_patch", bsp_modem_cold_patch_update_modem_fail_count);
    bsp_err("[init]bsp_modem_cold_patch_init ok!\n");
    return 0;
}

#ifndef CONFIG_HISI_BALONG_MODEM_MODULE
module_init(bsp_modem_cold_patch_init);
#endif

#else

int mloader_is_patch_image(char *image_name, u32 buf_len)
{
    return 0;
}

struct cold_patch_info_s *mloader_get_cold_patch_info_st(void)
{
    return &g_patch_info;
}

bool bsp_modem_cold_patch_is_exist(void)
{
    return 0;
}

ssize_t modem_imag_patch_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return 0;
}

ssize_t modem_imag_patch_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return 0;
}

u32 mloader_patch_get_type(const char *file_name)
{
    return MAX_PATCH;
}

int mloader_load_patch(enum modem_patch_type epatch_type, mloader_img_icc_info_s *mloader_msg, u32 index)
{
    return 0;
}

u32 mloader_patch_get_load_position_offset(enum SVC_SECBOOT_IMG_TYPE ecoretype, u32 ddr_size, int img_size)
{
    return 0;
}

u32 mloader_patch_get_load_fail_time(void)
{
    return 0;
}

int bsp_modem_cold_patch_init(void)
{
    return 0;
}

void mloader_update_modem_cold_patch_status(enum modem_patch_type epatch_type)
{
    return;
}

void mloader_record_cold_patch_splicing_ret_val(enum modem_patch_type epatch_type, int value)
{
    return;
}

int bsp_nvem_cold_patch_write(struct cold_patch_info_s *p_cold_patch_info)
{
    return 0;
}

int bsp_modem_is_reboot_machine(void)
{
    return 0;
}


#endif

