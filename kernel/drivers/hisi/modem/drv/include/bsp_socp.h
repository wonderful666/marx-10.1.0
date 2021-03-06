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

#ifndef _BSP_SOCP_H
#define _BSP_SOCP_H

#include "osl_common.h"
#include "mdrv_socp_common.h"
#ifdef __KERNEL__
#include "acore_nv_id_drv.h"
#include "acore_nv_stru_drv.h"
#endif
#include "bsp_trace.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
  ??????????
**************************************************************************/
#define BSP_ERR_SOCP_BASE            BSP_DEF_ERR(BSP_MODU_SOCP, 0)
#define BSP_ERR_SOCP_NULL            (BSP_ERR_SOCP_BASE + 0x1)
#define BSP_ERR_SOCP_NOT_INIT        (BSP_ERR_SOCP_BASE + 0x2)
#define BSP_ERR_SOCP_MEM_ALLOC       (BSP_ERR_SOCP_BASE + 0x3)
#define BSP_ERR_SOCP_SEM_CREATE      (BSP_ERR_SOCP_BASE + 0x4)
#define BSP_ERR_SOCP_TSK_CREATE      (BSP_ERR_SOCP_BASE + 0x5)
#define BSP_ERR_SOCP_INVALID_CHAN    (BSP_ERR_SOCP_BASE + 0x6)
#define BSP_ERR_SOCP_INVALID_PARA    (BSP_ERR_SOCP_BASE + 0x7)
#define BSP_ERR_SOCP_NO_CHAN         (BSP_ERR_SOCP_BASE + 0x8)
#define BSP_ERR_SOCP_SET_FAIL        (BSP_ERR_SOCP_BASE + 0x9)
#define BSP_ERR_SOCP_TIMEOUT         (BSP_ERR_SOCP_BASE + 0xa)
#define BSP_ERR_SOCP_NOT_8BYTESALIGN (BSP_ERR_SOCP_BASE + 0xb)
#define BSP_ERR_SOCP_CHAN_RUNNING    (BSP_ERR_SOCP_BASE + 0xc)
#define BSP_ERR_SOCP_CHAN_MODE       (BSP_ERR_SOCP_BASE + 0xd)
#define BSP_ERR_SOCP_DEST_CHAN       (BSP_ERR_SOCP_BASE + 0xe)
#define BSP_ERR_SOCP_DECSRC_SET      (BSP_ERR_SOCP_BASE + 0xf)

#define SOCP_MAX_MEM_SIZE            (200 *1024 *1024)
#define SOCP_MIN_MEM_SIZE            (1 *1024 *1024)
#define SOCP_MAX_TIMEOUT             1200     /*MS*/
#define SOCP_MIN_TIMEOUT             10       /*MS*/
#define SOCP_RESERVED_TRUE           1
#define SOCP_RESERVED_FALSE          0

typedef u32 (*socp_compress_isr)     (void);
typedef u32 (*socp_compress_event_cb)(socp_event_cb event_cb);
typedef u32 (*socp_compress_read_cb) (socp_read_cb read_cb);
typedef u32 (*socp_compress_enable)  (u32 chanid);
typedef u32 (*socp_compress_disable) (u32 chanid);
typedef u32 (*socp_compress_set)     (u32 chanid, SOCP_CODER_DEST_CHAN_S* attr);
typedef u32 (*socp_compress_getbuffer) (SOCP_BUFFER_RW_STRU *pRingBuffer);
typedef u32 (*socp_compress_readdone)(u32 u32Size);
typedef u32 (*socp_compress_clear)   (u32 chanid);

typedef struct socp_compress_ops
{
    socp_compress_isr       isr;
    socp_compress_event_cb  register_event_cb;
    socp_compress_read_cb   register_read_cb;
    socp_compress_enable    enable;
    socp_compress_disable   disable;
    socp_compress_set       set;
    socp_compress_getbuffer getbuffer;
    socp_compress_readdone  readdone;
    socp_compress_clear     clear;
}socp_compress_ops_stru;

#if (FEATURE_SOCP_DECODE_INT_TIMEOUT == FEATURE_ON)
typedef enum timeout_module
{
    DECODE_TIMEOUT_INT_TIMEOUT = 0,
    DECODE_TIMEOUT_DEC_INT_TIMEOUT = 1,
    DECODE_TIMEOUT_BUTTON = 2,

} decode_timeout_module_e;

#endif

typedef struct
{
    SOCP_VOTE_TYPE_ENUM_U32 type;
}socp_vote_req_stru;

typedef struct
{
    u32 vote_rst;   /* 1:????????1:???? */
}socp_vote_cnf_stru;

struct socp_enc_dst_log_cfg
{
    void*           virt_addr;       /* SOCP????????????????????BUFFER????32??????????4????????64??????????8???? */
    unsigned long   phy_addr;        /* SOCP????????????????????BUFFER???? */
    unsigned int    buff_size;       /* SOCP????????????????BUFFER???? */
    unsigned int    over_time;       /* NVE???????????????? */
    unsigned int    log_on_flag;     /* ????????buffer????????????(SOCP_DST_CHAN_CFG_TYPE_ENUM) */
    unsigned int    cur_time_out;    /* SOCP???????????????????????????? */
    unsigned int    flush_flag;
    unsigned int    mem_log_cfg;
    unsigned int    current_mode;
    unsigned int    cps_mode;
};

typedef struct {
    void*           virt_addr;      /* SOCP????????????????????BUFFER????32??????????4????????64??????????8???? */
    unsigned long   phy_addr;       /* SOCP????????????????????BUFFER???? */
    unsigned int    buff_size;      /* SOCP????????????????BUFFER???? */
    unsigned int    buff_useable;    /* ??????kernel buffer?????????????? */
    unsigned int    timeout;        /* SOCP???????????????????????????? */
    unsigned int    init_flag;      /* ???????????????????????? */
}socp_rsv_mem_s;


#ifdef ENABLE_BUILD_SOCP
/*****************************************************************************
* ?? ?? ??  : socp_init
*
* ????????  : ??????????????
*
* ????????  : ??
*
* ????????  : ??
*
* ?? ?? ??  : ??????????????????
*****************************************************************************/
s32 socp_init(void);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_encdst_dsm_init
*
* ????????  : socp????????????????????????
* ????????  : enc_dst_chan_id: ????????????????
*             b_enable: ????????????????
*
* ????????  : ??
*
* ?? ?? ??  : ??
*****************************************************************************/
void bsp_socp_encdst_dsm_init(u32 enc_dst_chan_id, u32 b_enable);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_data_send_manager
*
* ????????  : socp??????????????????
* ????????  : enc_dst_chan_id: ????????????????
*             b_enable: ????????
*
* ????????  : ??
*
* ?? ?? ??  : ??
*****************************************************************************/
void bsp_socp_data_send_manager(u32 enc_dst_chan_id, u32 b_enable);

/*****************************************************************************
 ?? ?? ??  : bsp_socp_coder_set_src_chan
 ????????  : ??????????SOCP??????????????????????????????????????????????????????????????????????????????????????
 ????????  : src_attr:????????????????????????????
             pSrcChanID:??????????????ID??
 ????????  : ????
 ?? ?? ??  : SOCP_OK:????????????????????
             SOCP_ERROR:????????????????????
*****************************************************************************/
s32 bsp_socp_coder_set_src_chan(SOCP_CODER_SRC_ENUM_U32 src_chan_id, SOCP_CODER_SRC_CHAN_S *src_attr);

/*****************************************************************************
 ?? ?? ??  : bsp_socp_coder_set_dest_chan_attr
 ????????  : ??????????????????????????????????????????????????????
 ????????  : dst_chan_id:SOCP????????????????ID??
             dst_attr:SOCP??????????????????????????????
 ????????  : ????
 ?? ?? ??  : SOCP_OK:??????????????????????
             SOCP_ERROR:??????????????????????
*****************************************************************************/
s32 bsp_socp_coder_set_dest_chan_attr(u32 dst_chan_id, SOCP_CODER_DEST_CHAN_S *dst_attr);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_decoder_set_dest_chan
 ????????  :??????????SOCP??????????????????????
                ??????????????????????????????????
                ????????????????????????????????
 ????????  : attr:????????????????????????????
                         pDestChanID:????????????????ID
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????????????????
                             SOCP_ERROR:??????????????????????
*****************************************************************************/
s32 bsp_socp_decoder_set_dest_chan(SOCP_DECODER_DST_ENUM_U32 dst_chan_id, SOCP_DECODER_DEST_CHAN_STRU *attr);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_decoder_set_src_chan_attr
 ????????  :????????????????????????????????????????????????????
 ????????  : src_chan_id:????????????ID
                         input_attr:??????????????????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????????????
                             SOCP_ERROR:??????????????????
*****************************************************************************/
s32 bsp_socp_decoder_set_src_chan_attr ( u32 src_chan_id,SOCP_DECODER_SRC_CHAN_STRU *input_attr);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_decoder_get_err_cnt
 ????????  :??????????????????????????????????????????
 ????????  : chan_id:??????????ID
                         err_cnt:????????????????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:????????????????
                             SOCP_ERROR:????????????????
*****************************************************************************/
s32 bsp_socp_decoder_get_err_cnt (u32 chan_id, SOCP_DECODER_ERROR_CNT_STRU *err_cnt);

/*****************************************************************************
 ?? ?? ??  : bsp_socp_free_channel
 ????????  : ??????????????ID??????????????????????
 ????????  : chan_id:????ID??
 ????????  : ????
 ?? ?? ??  : SOCP_OK:??????????????
             SOCP_ERROR:??????????????
*****************************************************************************/
s32 bsp_socp_free_channel(u32 chan_id);

/*****************************************************************************
* ?? ?? ??  : socp_clean_encsrc_chan
*
* ????????  : ????????????????????V9 SOCP????
*
* ????????  : src_chan_id       ??????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_OK
*****************************************************************************/
u32 bsp_socp_clean_encsrc_chan(SOCP_CODER_SRC_ENUM_U32 src_chan_id);

/*****************************************************************************
 ?? ?? ??  : bsp_socp_register_event_cb
 ????????  : ??????????????????????????????????
 ????????  : chan_id:????ID??
             event_cb:??????????????????socp_event_cb????????
 ????????  : ????
 ?? ?? ??  : SOCP_OK:??????????????????????
             SOCP_ERROR:??????????????????????
*****************************************************************************/
s32 bsp_socp_register_event_cb(u32 chan_id, socp_event_cb event_cb);

/*****************************************************************************
 ?? ?? ??  : bsp_socp_start
 ????????  : ????????????????????????????????????
 ????????  : src_chan_id:??????ID??
 ????????  : ????
 ?? ?? ??  : SOCP_OK:????????????????????
             SOCP_ERROR:????????????????????
*****************************************************************************/
s32 bsp_socp_start(u32 src_chan_id);

/*****************************************************************************
 ?? ?? ??  : bsp_socp_stop
 ????????  : ????????????????????????????????????
 ????????  : src_chan_id:??????ID??
 ????????  : ????
 ?? ?? ??  : SOCP_OK:????????????????????
             SOCP_ERROR:????????????????????
*****************************************************************************/
s32 bsp_socp_stop(u32 src_chan_id);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_set_timeout
 ????????  :????????????????????
 ????????  : time_out:????????

 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????????????????
                             SOCP_ERROR:????????????????????
*****************************************************************************/
s32 bsp_socp_set_timeout (SOCP_TIMEOUT_EN_ENUM_UIN32 time_out_en, u32 time_out);

/*****************************************************************************
 ?? ?? ??   : bsp_socp_set_dec_pkt_lgth
 ????????  :????????????????????
 ????????  : pkt_length:??????????????

 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????
                    ??????:????????
*****************************************************************************/
s32 bsp_socp_set_dec_pkt_lgth(SOCP_DEC_PKTLGTH_STRU *pkt_length);

/*****************************************************************************
 ?? ?? ??   : bsp_socp_set_debug
 ????????  :????????????????debug????
 ????????  : chan_id:????ID
                  debug_en: debug????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????
                     ??????:????????
*****************************************************************************/
s32 bsp_socp_set_debug(u32 dst_chan_id, u32 debug_en);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_get_write_buff
 ????????  :????????????????????buffer??
 ????????  : src_chan_id:??????ID
                  p_rw_buff:           :??????buffer

 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????buffer??????
                             SOCP_ERROR:??????????buffer????
*****************************************************************************/
s32 bsp_socp_get_write_buff( u32 src_chan_id, SOCP_BUFFER_RW_STRU *p_rw_buff);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_write_done
 ????????  :????????????????????????????????????????????
 ????????  : src_chan_id:??????ID
                  write_size:   ????????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????????
                             SOCP_ERROR:????????????
*****************************************************************************/
s32 bsp_socp_write_done(u32 src_chan_id, u32 write_size);


/*****************************************************************************
 ?? ?? ??      : bsp_socp_register_rd_cb
 ????????  :????????????????RD????????????????????????????
 ????????  : src_chan_id:??????ID
                  rd_cb:  ????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:????RD??????????????????????????????
                             SOCP_ERROR:????RD????????????????????????????
*****************************************************************************/
s32 bsp_socp_register_rd_cb(u32 src_chan_id, socp_rd_cb rd_cb);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_get_rd_buffer
 ????????  :????????????????RD buffer????????????
 ????????  : src_chan_id:??????ID
                  p_rw_buff:  RD buffer
 ????????  : ????
 ?? ?? ??      : SOCP_OK:????RD??????????????
                             SOCP_ERROR:????RD??????????????
*****************************************************************************/
s32 bsp_socp_get_rd_buffer( u32 src_chan_id,SOCP_BUFFER_RW_STRU *p_rw_buff);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_read_rd_done
 ????????  :??????????????????SOCP????????RD buffer??????????????????
 ????????  : src_chan_id:??????ID
                  rd_size:  ??RD buffer????????????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:????RDbuffer????????????
                             SOCP_ERROR:????RDbuffer????????????
*****************************************************************************/
s32 bsp_socp_read_rd_done(u32 src_chan_id, u32 rd_size);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_register_read_cb
 ????????  :????????????????????????????????
 ????????  : dst_chan_id:????????ID
                  read_cb: ????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????????????????
                             SOCP_ERROR:??????????????????????
*****************************************************************************/
s32 bsp_socp_register_read_cb( u32 dst_chan_id, socp_read_cb read_cb);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_register_read_cb
 ????????  :??????????????????????????????????
 ????????  : dst_chan_id:????????ID
                  read_cb: ??????buffer
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????????????????
                             SOCP_ERROR:??????????????????????
*****************************************************************************/
s32 bsp_socp_get_read_buff(u32 dst_chan_id,SOCP_BUFFER_RW_STRU *pBuffer);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_read_data_done
 ????????  :??????????????????SOCP??????????????????????????????????
 ????????  : dst_chan_id:????????ID
                  read_size: ????????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:????????????
                             SOCP_ERROR:??????????
*****************************************************************************/
s32 bsp_socp_read_data_done(u32 dst_chan_id,u32 read_size);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_set_bbp_enable
 ????????  :??????????BBP??????
 ????????  : b_enable:????ID
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????
                   ??????:????????
*****************************************************************************/
s32 bsp_socp_set_bbp_enable(int b_enable);

/*****************************************************************************
 ?? ?? ??      : bsp_socp_set_bbp_ds_mode
 ????????  :????BBP DS??????????????????????
 ????????  : ds_mode:DS??????????????????????????
 ????????  : ????
 ?? ?? ??      : SOCP_OK:??????????
                   ??????:????????
*****************************************************************************/
s32 bsp_socp_set_bbp_ds_mode(SOCP_BBP_DS_MODE_ENUM_UIN32 ds_mode);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_get_state
*
* ????????  : ????SOCP????
*
* ?? ?? ??  : SOCP_IDLE    ????
*             SOCP_BUSY    ????
*****************************************************************************/
SOCP_STATE_ENUM_UINT32 bsp_socp_get_state(void);


/*****************************************************************************
* ?? ?? ??  : bsp_socp_vote
* ????????  : SOCP??????????????????????????SOCP????????????????????A??????
* ????????  : id --- ????????ID??type --- ????????
* ????????  : ??
* ?? ?? ??  : BSP_S32 0 --- ??????????0xFFFFFFFF --- ????????
*****************************************************************************/
BSP_S32 bsp_socp_vote(SOCP_VOTE_ID_ENUM_U32 id, SOCP_VOTE_TYPE_ENUM_U32 type);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_vote_to_mcore
* ????????  : SOCP????????????????????C????????????LDSP??????????SOCP????????
* ????????  : type --- ????????
* ????????  : ??
* ?? ?? ??  : BSP_S32 0 --- ??????????0xFFFFFFFF --- ????????
*****************************************************************************/
BSP_S32 bsp_socp_vote_to_mcore(SOCP_VOTE_TYPE_ENUM_U32 type);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_get_log_cfg
* ????????  : ????log????
* ????????  :
* ????????  :
* ?? ?? ??  :
*****************************************************************************/
struct socp_enc_dst_log_cfg * bsp_socp_get_log_cfg(void);
/*****************************************************************************
* ?? ?? ??  : bsp_socp_get_sd_logcfg
* ????????  : ????????
* ????????  :
* ????????  :
* ?? ?? ??  :
*****************************************************************************/
u32 bsp_socp_get_sd_logcfg(SOCP_ENC_DST_BUF_LOG_CFG_STRU* cfg);
/*****************************************************************************
* ?? ?? ??  : socp_set_clk_autodiv_enable
* ????????  : ????clk????clk_disable_unprepare??bypass??0??????????????
* ????????  : ??
* ????????  : ??
* ?? ?? ??  : ??
* ??    ??  :
              clk_prepare_enable ?????? clk_disable_unprepare ????????????????
              clk????????????????????????????????
              ?????????? clk_prepare_enable ???????? clk_disable_unprepare ????
*****************************************************************************/
void bsp_socp_set_clk_autodiv_enable(void);

/*****************************************************************************
* ?? ?? ??  : socp_set_clk_autodiv_disable
* ????????  : ????clk????clk_prepare_enable??bypass??1??????????????
* ????????  : ??
* ????????  : ??
* ?? ?? ??  : ??
* ??    ??  :
              clk_prepare_enable ?????? clk_disable_unprepare ????????????????
              clk????????????????????????????????
              ?????????? clk_prepare_enable ???????? clk_disable_unprepare ????
*****************************************************************************/
void bsp_socp_set_clk_autodiv_disable(void);


#if (FEATURE_SOCP_DECODE_INT_TIMEOUT == FEATURE_ON)
/*****************************************************************************
* ?? ?? ??  : bsp_socp_set_decode_timeout_register
* ????????  :??????????????????
* ????????  : ??
* ????????  : ??
* ?? ?? ??  :
*****************************************************************************/
s32 bsp_socp_set_decode_timeout_register(decode_timeout_module_e module);
#endif
/*****************************************************************************
* ?? ?? ??  : bsp_socp_set_enc_dst_threshold
* ????????  :
* ????????  :
* ????????  :
* ?? ?? ??  :
*****************************************************************************/
void bsp_socp_set_enc_dst_threshold(bool mode,u32 dst_chan_id);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_set_ind_mode
*
* ????????  : ????????????????
*
* ????????  : ????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/
s32 bsp_socp_set_ind_mode(SOCP_IND_MODE_ENUM mode);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_get_log_ind_mode
*
* ????????  : ????????????????
*
* ????????  : ????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/
s32  bsp_socp_get_log_ind_mode(u32 *log_ind_mode);
/*****************************************************************************
* ?? ?? ??  : bsp_report_ind_mode_ajust
*
* ????????  : ????????????
*
* ????????  : ????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/
s32 bsp_report_ind_mode_ajust(SOCP_IND_MODE_ENUM mode);

/*****************************************************************************
* ?? ?? ??  :  bsp_socp_encsrc_chan_open
*
* ????????  : ????SOCP??????????????
*
* ????????  : ??
*
* ????????  : ??
*
* ?? ?? ??  : void
*****************************************************************************/
void bsp_socp_encsrc_chan_open(u32 src_chan_id);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_encsrc_chan_close
*
* ????????  : ????socp????????????????????????socp??????????
              ??????????????socp????????????
*
* ????????  : ??
*
* ????????  : ??
*
* ?? ?? ??  : void
*****************************************************************************/
void bsp_socp_encsrc_chan_close(u32 src_chan_id);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_dump_save
*
* ????????  : ??????????socp??????
*****************************************************************************/
void bsp_socp_dump_save(void);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_check_state
*
* ????????  : ????SOCP??????????????????????????TCM??????????????
*
* ????????  : ????????
*
* ????????  : ??
*
* ?? ?? ??  : SOCP????
*****************************************************************************/
s32 bsp_socp_check_state(u32 src_chan_id);

/*****************************************************************************
 ?? ?? ??  : bsp_socp_update_bbp_ptr
 ????????  : ????????????????????????????????????????
 ????????  : ulSrcChanId:??????ID

 ????????  : ????
 ?? ?? ??  : ??
*****************************************************************************/
void bsp_socp_update_bbp_ptr(u32 src_chan_id);


u32 bsp_get_socp_ind_dst_int_slice(void);
/*****************************************************************************
 ?? ?? ??      : bsp_clear_socp_buff
 ????????  : ????????????????SOCP??buff
 ????????  : src_chan_id:????id
 ????????  : ????
 ?? ?? ??     : ??
*****************************************************************************/
s32 bsp_clear_socp_buff(u32 src_chan_id);

/*****************************************************************************
* ?? ?? ??  : bsp_socp_soft_free_encdst_chan
*
* ????????  : ??????????????????
*
* ????????  : enc_dst_chan_id       ??????????
*
* ????????  : ??
*
* ?? ?? ??  : ????????????????????
*****************************************************************************/
s32 bsp_socp_soft_free_encdst_chan(u32 enc_dst_chan_id);

/*****************************************************************************
* ?? ?? ??  : bsp_SocpEncDstQueryIntInfo
*
* ????????  : ??????diag_debug????socp??????????????????????
*
* ????????  : ??
* ????????  :
*
* ?? ?? ??  : ??
*****************************************************************************/
void bsp_SocpEncDstQueryIntInfo(u32 *trf_info, u32 *thrh_ovf_info);

/*****************************************************************************
* ?? ?? ??  : bsp_clear_socp_encdst_int_info
*
* ????????  : ????socp????????????????
*
* ????????  : ??
* ????????  :
*
* ?? ?? ??  : ??
*****************************************************************************/
void bsp_clear_socp_encdst_int_info(void);

s32 socp_dst_channel_enable(u32 dst_chan_id);

s32 socp_dst_channel_disable(u32 dst_chan_id);
s32 bsp_socp_dst_trans_id_disable(u32 dst_chan_id);


#ifdef __KERNEL__
/*****************************************************************************
 ?? ?? ??      : bsp_socp_set_rate_ctrl
 ????????  : ????????????????SOCP????????
 ????????  : p_rate_ctrl:????????
 ????????  : ????
 ?? ?? ??     : ??
*****************************************************************************/
s32 bsp_socp_set_rate_ctrl(DRV_DIAG_RATE_STRU *p_rate_ctrl);
#endif

#else

static inline void bsp_socp_encsrc_chan_open(u32 src_chan_id)
{
    return;
}

static inline void bsp_socp_encsrc_chan_close(u32 src_chan_id)
{
    return;
}

static inline s32 bsp_socp_check_state(u32 src_chan_id)
{
	return 0;
}

static inline void bsp_socp_dump_save(void)
{
    return;
}

static inline s32 bsp_socp_get_write_buff( u32 src_chan_id, SOCP_BUFFER_RW_STRU *p_rw_buff)
{
    return 0;
}

static inline s32 bsp_socp_write_done(u32 src_chan_id, u32 write_size)
{
    return 0;
}
static inline s32 bsp_socp_coder_set_dest_chan_attr(u32 dst_chan_id, SOCP_CODER_DEST_CHAN_S *dst_attr)
{
    return 0;
}
static inline s32 bsp_socp_coder_set_src_chan(SOCP_CODER_SRC_ENUM_U32 src_chan_id, SOCP_CODER_SRC_CHAN_S *src_attr)
{
    return 0;
}

static inline s32 bsp_socp_start(u32 src_chan_id)
{
    return 0;
}
static inline u32 bsp_get_socp_ind_dst_int_slice(void)
{
    return 0;
}
static inline s32 bsp_socp_set_ind_mode(SOCP_IND_MODE_ENUM mode)
{
	return 0;
}
static inline s32 bsp_socp_get_read_buff(u32 dst_chan_id,SOCP_BUFFER_RW_STRU *p_buffer)
{
	return 0;
}
static inline u32 bsp_socp_get_sd_logcfg(SOCP_ENC_DST_BUF_LOG_CFG_STRU* cfg)
{
	return 0;
}
static inline s32 bsp_socp_read_data_done(u32 dst_chan_id,u32 read_size)
{
	return 0;
}
static inline s32 bsp_socp_register_read_cb( u32 dst_chan_id, socp_read_cb read_cb)
{
	return 0;
}
static inline s32 bsp_socp_register_event_cb(u32 chan_id, socp_event_cb event_cb)
{
	return 0;
}
static inline void bsp_socp_encdst_dsm_init(u32 enc_dst_chan_id, u32 b_enable)
{

}

static inline s32 bsp_clear_socp_buff(u32 src_chan_id)
{
    return 0;
}
static inline s32 bsp_socp_soft_free_encdst_chan(u32 enc_dst_chan_id)
{
    return 0;
}
static inline void socp_m3_init(void)
{
    return;
}

static inline void bsp_SocpEncDstQueryIntInfo(u32 *trf_info, u32 *thrh_ovf_info)
{
    return;
}

static inline void bsp_clear_socp_encdst_int_info(void)
{
    return;
}

static inline s32 socp_dst_channel_enable(u32 dst_chan_id)
{
    return 0;
}

static inline s32 socp_dst_channel_disable(u32 dst_chan_id)
{
    return 0;
}

static inline s32 bsp_socp_set_rate_ctrl(DRV_DIAG_RATE_STRU *p_rate_ctrl)
{
    return 0;
}
static inline s32 bsp_socp_dst_trans_id_disable(u32 dst_chan_id)
{
    return 0;

}

#endif

/*****************************************************************************
* ?? ?? ??  : bsp_socp_register_compress
*
* ????????  : ??????????????????socp??
*
* ????????  : ????????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/

s32 bsp_socp_register_compress(socp_compress_ops_stru *ops);

/*****************************************************************************
* ?? ?? ??  : bsp_deflate_get_log_ind_mode
*
* ????????  : ????????????????????????
*
* ????????  : ????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/
s32  bsp_deflate_get_log_ind_mode(u32 *log_ind_mode);
/*****************************************************************************
* ?? ?? ??  : bsp_deflate_cfg_ind_mode
*
* ????????  : ????????????????
*
* ????????  : ????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/

s32 bsp_deflate_set_ind_mode(SOCP_IND_MODE_ENUM mode);
/*****************************************************************************
* ?? ?? ??  : bsp_socp_compress_disable
*
* ????????  : ????????????
*
* ????????  : ??????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/
s32 bsp_socp_compress_disable(u32 dst_chan_id);
/*****************************************************************************
* ?? ?? ??  : bsp_socp_compress_enable
*
* ????????  : ????????????
*
* ????????  : ??????????
*
* ????????  : ??
*
* ?? ?? ??  : BSP_S32 BSP_OK:???? BSP_ERROR:????
*****************************************************************************/
s32 bsp_socp_compress_enable(u32 dst_chan_id);

s32 bsp_socp_set_cfg_ind_mode(SOCP_IND_MODE_ENUM mode);
s32 bsp_socp_get_cfg_ind_mode(u32 *Cfg_ind_mode);
s32 bsp_socp_set_cps_ind_mode(DEFLATE_IND_COMPRESSS_ENUM mode);
s32 bsp_socp_get_cps_ind_mode(u32 *cps_ind_mode);
s32 bsp_socp_compress_status(void);

#ifdef __cplusplus
}
#endif

#endif /* end of _BSP_SOCP_H*/


