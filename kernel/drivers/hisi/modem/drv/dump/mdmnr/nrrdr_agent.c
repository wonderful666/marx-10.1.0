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
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/kernel.h>
#include <asm/string.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include "osl_types.h"
#include "mdrv_malloc.h"
#include "bsp_sysctrl.h"
#include "bsp_slice.h"
#include "bsp_wdt.h"
#include "bsp_ipc.h"
#include "bsp_fiq.h"
#include "bsp_coresight.h"
#include "bsp_dump.h"
#include "bsp_adump.h"
#include "bsp_ddr.h"
#include "bsp_slice.h"
#include "bsp_noc.h"
#include "bsp_wdt.h"
#include "bsp_dump_mem.h"
#include "dump_area.h"
#include "dump_cp_agent.h"
#include "dump_cp_wdt.h"
#include "dump_config.h"
#include "dump_baseinfo.h"
#include "dump_lphy_tcm.h"
#include "dump_cphy_tcm.h"
#include "dump_exc_handle.h"
#include "nrrdr_agent.h"
#include "dump_debug.h"

#undef THIS_MODU
#define THIS_MODU mod_dump


/*
 * 功能描述: c核异常但是错误要上报到ap的异常
 */

void dump_nrccpu_wdt_handle(u32 mod_id, u32 arg1, u32 arg2, char *data, u32 length)
{
    u32 reason = 0;
    char *desc = NULL;
    dump_base_info_s *modem_cp_base_info = NULL;

    dump_exception_info_s exception_info_s = {
        0,
    };

    dump_ok("[0x%x]modem nrccpu wdt or pdlock enter system error! \n", bsp_get_slice_value());
    dump_ok("mod_id=0x%x arg1=0x%x arg2=0x%x len=0x%x\n", mod_id, arg1, arg2, length);

    if (arg1 == DUMP_REASON_WDT) {
        desc = "Modem CP WDT";
    }
    reason = arg1;

    dump_fill_excption_info(&exception_info_s, mod_id, arg1, arg2, NULL, 0, DUMP_CPU_NRCCPU, reason, desc, 0, 0, 0,
                            NULL);

    dump_register_exception(&exception_info_s);
}

/*
 * 功能描述: cp 看门狗回调函数
 */
void dump_nrccpu_wdt_hook(void)
{
    dump_nrccpu_wdt_handle(DRV_ERRNO_NRCCPU_WDT, DUMP_REASON_WDT, 0, 0, 0);
}
/*
 * 功能描述: nr 子系统上报的错误
 */

void dump_nrrdr_agent_handle(u32 param)
{
    dump_exception_info_s exception_info_s = { 0, DUMP_REASON_NORMAL };

    dump_ok("NR enter system error!,timestamp:0x%x\n", bsp_get_slice_value());

    dump_fill_excption_info(&exception_info_s, DRV_ERRNO_DUMP_ARM_EXC, 0, 0, NULL, 0, DUMP_CPU_NRCCPU,
                            DUMP_REASON_NORMAL, NULL, 0, 0, 0, NULL);
    exception_info_s.rdr_mod_id = NRRDR_MODEM_NR_CCPU_START;
    dump_register_exception(&exception_info_s);

    return;
}
/*
 * 功能描述: nr 子系统dump初始化
 */

__init s32 nrrdr_agent_init(void)
{
    s32 ret = BSP_ERROR;
    ret = bsp_ipc_int_connect(IPC_ACPU_SRC_NRCCPU_DUMP, (voidfuncptr)dump_nrrdr_agent_handle, 0);
    if (unlikely(BSP_OK != ret)) {
        dump_error("fail to connect ipc int\n");
        return BSP_ERROR;
    }
    ret = bsp_ipc_int_enable(IPC_ACPU_SRC_NRCCPU_DUMP);
    if (unlikely(BSP_OK != ret)) {
        dump_error("fail to enbale ipc int\n");
        return BSP_ERROR;
    }
    ret = bsp_wdt_register_hook(WDT_NRCCPU_ID, dump_nrccpu_wdt_hook);
    if (ret == BSP_ERROR) {
        dump_error("fail to bsp_wdt_register_hook\n");
    }

    dump_ok("nrdrr init ok");
    return BSP_OK;
}
