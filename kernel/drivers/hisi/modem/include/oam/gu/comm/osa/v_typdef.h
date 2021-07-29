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



#ifndef _V_TYPDEF_H
#define _V_TYPDEF_H

#include "vos_config.h"
#include "v_basic_type_def.h"

#ifndef VOS_OS_VER
#error "Please include v_configOS.h first!!!"
#endif

#if (VOS_WIN32 == VOS_OS_VER)
#define _WIN32_WINNT 0x0400
#ifdef LLT_OS_WIN
#include "Windows.h"
#endif
#include "stdarg.h"
#endif

#if (VOS_LINUX == VOS_OS_VER)
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/net.h>
#include <linux/in.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/socket.h>
#include <linux/skbuff.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/delay.h>
#include <net/sock.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#endif

#if (VOS_RTOSCK == VOS_OS_VER)
#ifdef VOS_SUPPORT_TSP
#else
#include "sre_task.h"
#include "sre_base.h"
#include "sre_mem.h"
#include "sre_sem.h"
#include "sre_shell.h"
#include "sre_sys.h"
#ifdef __OS_RTOSCK_SMP__
#include "sre_atomic.h"
#endif
#endif
#endif

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#pragma pack(4)

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define VOS_NULL (0)
#define VOS_NULL_PTR (0L)
#define VOS_NULL_BYTE (0XFF)
#define VOS_NULL_WORD (0xFFFF)
#define VOS_NULL_DWORD (0xFFFFFFFFU)
#define VOS_NULL_LONG (VOS_NULL_DWORD)
#define VOS_NULL_OBJ (-1)
#define VOS_NULL_PARA (-1)

#define VOS_OK (0)
#define VOS_ERR (1) /* For VRP VOS adaptation */
#define VOS_ERROR (-1)

#define VOS_MEMORY_CRC         (0x5A5A5A5AUL)

#if (VOS_WIN32 == VOS_OS_VER)
typedef void VOS_VOID;
#else
#ifndef VOS_VOID
#define VOS_VOID void
#endif
#endif

typedef union {
    VOS_UINT32 b32[2];
    VOS_UINT16 b16[4];
    VOS_UINT8  b8[8];
} Register64;

#if (VOS_WIN32 == VOS_OS_VER)
#ifndef LLT_OS_WIN
typedef unsigned int       UINT32;
typedef signed int         INT32;
typedef unsigned short     UINT16;
typedef signed short       INT16;
typedef unsigned char      UINT8;
typedef signed char        INT8;
typedef int                __kernel_pid_t;
typedef __kernel_pid_t     pid_t;

#undef HZ
#define HZ 1000

#define TASK_UNINTERRUPTIBLE 2

#define SCHED_FIFO 1

#define ETIME 62 /* Timer expired */

enum pid_type {
    PIDTYPE_PID,
    PIDTYPE_PGID,
    PIDTYPE_SID,
    PIDTYPE_MAX
};

/* ------------- spinlock_t -------------------- */
typedef struct {
    volatile unsigned int slock;
} arch_spinlock_t;

typedef struct raw_spinlock {
    arch_spinlock_t rawLock;
} raw_spinlock_t;

typedef struct spinlock {
    union {
        struct raw_spinlock rlock;
    };
} spinlock_t;
struct list_head {
    struct list_head *next, *prev;
};
struct semaphore {
    spinlock_t       lock;
    unsigned int     count;
    struct list_head waitList;
};

struct task_struct {
    pid_t pid;
    pid_t tgid;
};

struct task_struct *get_current(void);

#endif

#define inline __inline

#endif

typedef unsigned int VOS_PID;

typedef unsigned int VOS_FID;

typedef unsigned int VOS_SIZE_T;

typedef VOS_INT32 VOS_SOCKET;

typedef VOS_UINT32 TICK_T;

typedef unsigned char NULL_TYPE;

typedef unsigned char BOOL_TYPE;

typedef struct {
    unsigned char placehold;
} NULL_SEQ;

typedef struct {
    unsigned char placehold;
} NULL_SET;

typedef struct CpuTickEx {
    VOS_UINT32 low;
    VOS_UINT32 high;
} CPU_TickEx;

typedef struct {
    VOS_UINT16 year;  /* year  */
    VOS_UINT8  month; /* month */
    VOS_UINT8  date;  /* day   */
} DATE_T;

typedef struct {
    VOS_UINT8 hour;   /* hour   */
    VOS_UINT8 minute; /* minute */
    VOS_UINT8 second; /* second */
    VOS_UINT8 padding;
} TIME_T;

typedef struct SysTimeEx {
    VOS_UINT16 year;
    VOS_UINT8  month;  /* scope is 1 - 12 */
    VOS_UINT8  date;   /* scope is 1 - 31 */
    VOS_UINT8  hour;   /* scope is 0 - 23 */
    VOS_UINT8  minute; /* scope is 0 - 59 */
    VOS_UINT8  second; /* scope is 0 - 59 */
    VOS_UINT8  week;   /* scope is 1 - 7  */
} SYS_T;

enum BoolDefine {
    VOS_FALSE = 0,
    VOS_TRUE  = 1
};

/* function pointer type definition for dopra internal use */
/* ignore input parameter prototype check */
typedef VOS_UINT32 (*VOS_ULFUNCPTR)(VOS_VOID);   /* ptr to function returning VOS_UINT32 */
typedef VOS_INT32 (*VOS_SLFUNCPTR)(VOS_VOID);    /* ptr to function returning VOS_INT32 */
typedef VOS_UINT16 (*VOS_USFUNCPTR)(VOS_VOID);   /* ptr to function returning VOS_UINT16 */
typedef VOS_INT16 (*VOS_SSFUNCPTR)(VOS_VOID);    /* ptr to function returning VOS_INT16 */
typedef VOS_UINT8 (*VOS_UCFUNCPTR)(VOS_VOID);    /* ptr to function returning VOS_UINT8 */
typedef VOS_INT8 (*VOS_SCFUNCPTR)(VOS_VOID);     /* ptr to function returning VOS_INT8 */
typedef VOS_UINT (*VOS_UIFUNCPTR)(VOS_VOID);     /* ptr to function returning VOS_UINT */
typedef VOS_INT (*VOS_SIFUNCPTR)(VOS_VOID);      /* ptr to function returning VOS_INT */
typedef VOS_VOID (*VOS_VOIDFUNCPTR)(VOS_VOID);   /* ptr to function returning VOS_VOID */
typedef VOS_VOID *(*VOS_PVOIDFUNCPTR)(VOS_VOID); /* ptr to function returning PVOS_VOID */
typedef VOS_BOOL (*VOS_BOOLFUNCPTR)(VOS_VOID);   /* ptr to function returning VOS_BOOL */

/* function pointer type definition for application program use */
/* with input parameter prototype check */
typedef VOS_VOID (*HOOK_FUN_TYPE)(VOS_VOID);
typedef VOS_VOID (*FATAL_ERROR_HOOK_FUN)(VOS_CHAR *szFile, VOS_UINT32 ulLine); /* sgsn2 */
typedef VOS_UINT32 (*VOS_PRINT_HADLER)(VOS_CHAR *strFromat, ...);
typedef VOS_UINT32 (*VOS_TIME_FUN_TYPE)(DATE_T *pDate, TIME_T *pTime, VOS_UINT32 *ulMillSecs);
typedef VOS_UINT32 (*VOS_ULULFUNCPTR)(VOS_UINT32 ulPara);
typedef VOS_VOID (*Event_Fun_Type)(VOS_UINT32 ulEvent);

#if (VOS_OS_VER == VOS_WIN32)

#define far

#ifdef FAR
#undef FAR
#endif
#define FAR

#define near

#ifdef NEAR
#undef NEAR
#endif
#define NEAR

#elif (VOS_OS_VER == VOS_RTOSCK)

#define FAR
#define far
#define near
#define NEAR

#elif (VOS_OS_VER == VOS_LINUX)

#define FAR
#define far
#define near
#define NEAR
#define LINUX_KEY_BASE 1024

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif
typedef VOS_INT32 LINUX_MSGQUEUE;
typedef void *(*_LINUX_TASK_ENTRY)(void *);

#endif /* VOS_OS_VER */

enum VOS_InitPhaseDefine {
    VOS_IP_LOAD_CONFIG,
    VOS_IP_FARMALLOC,
    VOS_IP_INITIAL,
    VOS_IP_ENROLLMENT,
    VOS_IP_LOAD_DATA,  /* File System specific     */
    VOS_IP_FETCH_DATA, /* General Inquiry specific */
    VOS_IP_STARTUP,
    VOS_IP_RIVAL,
    VOS_IP_KICKOFF,
    VOS_IP_STANDBY,
    VOS_IP_BROADCAST_STATE,
    VOS_IP_RESTART,
    VOS_IP_BUTT
};

typedef VOS_UINT32 (*InitFunType)(enum VOS_InitPhaseDefine InitPhrase);

enum VOS_StartOrder {
    VOS_START_FIRST,
    VOS_START_SECOND,
    VOS_START_THIRD,
    VOS_START_FOURTH,
    VOS_START_FIFTH,
    VOS_START_SIXTH,
    VOS_START_SEVENTH,
    VOS_START_EIGHTH,
    VOS_START_NINTH,
    VOS_START_TENTH,
    VOS_START_ELEVENTH,
    VOS_START_TWELFTH,
    VOS_START_THIRTEENTH,
    VOS_START_FOURTEENTH,
    VOS_START_FIFTEENTH,

    VOS_START_BUTT
};

enum VOS_TimerTypeDefine {
    VOS_RELATIVE_TIMER,
    VOS_ABSOLUTE_TIMER,
    VOS_PERIODIC_TIMER,
    VOS_TIME_ADJUST_NOTIFY
};

#define VOS_PRIORITY_NUM 14

enum VOS_PriorityDefine {
    VOS_PRIORITY_NULL,
    VOS_PRIORITY_M6,
    VOS_PRIORITY_M5,
    VOS_PRIORITY_M4,
    VOS_PRIORITY_M3,
    VOS_PRIORITY_M2,
    VOS_PRIORITY_M1,
    VOS_PRIORITY_BASE,
    VOS_PRIORITY_P1,
    VOS_PRIORITY_P2,
    VOS_PRIORITY_P3,
    VOS_PRIORITY_P4,
    VOS_PRIORITY_P5,
    VOS_PRIORITY_P6
};

enum VOS_CoreMaskDefine {
    VOS_CORE_MASK_NULL  = 0,
    VOS_CORE_MASK_CORE0 = 1,
    VOS_CORE_MASK_CORE1 = 2,
    VOS_CORE_MASK_CORE2 = 4,
    VOS_CORE_MASK_CORE3 = 8,
    VOS_CORE_MASK_CORE4 = 16,
    VOS_CORE_MASK_CORE5 = 32,
    VOS_CORE_MASK_CORE6 = 64,
    VOS_CORE_MASK_CORE7 = 128
};

#if (VOS_OS_VER == VOS_WIN32)
#pragma pack()
#else
#pragma pack(0)
#endif

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cpluscplus */
#endif /* __cpluscplus */

#endif /* _V_TYPDEF_H */
