/*
 * tc_ns_client.h
 *
 * data structure declaration for nonsecure world
 *
 * Copyright (c) 2012-2018 Huawei Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _TC_NS_CLIENT_H_
#define _TC_NS_CLIENT_H_

#include "teek_client_type.h"

#ifdef SECURITY_AUTH_ENHANCE
#define SCRAMBLING_KEY_LEN      4
#define TOKEN_BUFFER_LEN        42   /* token(32byte) + timestamp(8byte) + kernal_api(1byte) + sync(1byte) */
#define TIMESTAMP_BUFFER_INDEX  32
#define KERNAL_API_INDEX        40
#define SYNC_INDEX              41
#define UUID_LEN                16
#define PARAM_NUM               4
#define TEE_PARAM_NUM           4
#define ADDR_TRANS_NUM          32
#define TEE_PARAM_ONE           0
#define TEE_PARAM_TWO           1
#define TEE_PARAM_THREE         2
#define TEE_PARAM_FOUR          3



#define TIMESTAMP_LEN_DEFAULT \
	((KERNAL_API_INDEX) - (TIMESTAMP_BUFFER_INDEX))
#define KERNAL_API_LEN \
	((TOKEN_BUFFER_LEN) - (KERNAL_API_INDEX))
#define TIMESTAMP_SAVE_INDEX    16
#endif

#ifndef ZERO_SIZE_PTR
#define ZERO_SIZE_PTR ((void *)16)
#define ZERO_OR_NULL_PTR(x) ((unsigned long)(x) <= (unsigned long)ZERO_SIZE_PTR)
#endif

typedef struct {
	__u32 method;
	__u32 mdata;
} tc_ns_client_login;

typedef union {
	struct {
		__u64 buffer;
		__u64 offset;
		__u64 size_addr;
	} memref;
	struct {
		__u64 a_addr;
		__u64 b_addr;
	} value;
} tc_ns_client_param;

typedef struct {
	int code;
	__u32 origin;
} tc_ns_client_return;

typedef struct {
	unsigned char uuid[UUID_LEN];
	__u32 session_id;
	__u32 cmd_id;
	tc_ns_client_return returns;
	tc_ns_client_login login;
	tc_ns_client_param params[PARAM_NUM];
	__u32 param_types;
	__u8 started;
#ifdef SECURITY_AUTH_ENHANCE
	void* teec_token;
	__u32 token_len;
#endif
	__u32 callingPid;
	unsigned int file_size;
	union {
		char *file_buffer;
		unsigned long long file_addr;
	};
} tc_ns_client_context;

typedef struct {
	uint32_t seconds;
	uint32_t millis;
} tc_ns_client_time;

struct load_secfile_ioctl_struct {
	uint32_t file_size;
	union {
		char *file_buffer;
		unsigned long long file_addr;
	};
};

struct agent_ioctl_args {
	uint32_t id;
	uint32_t buffer_size;
	union {
		void *buffer;
		unsigned long long addr;
	};
};

#define vmalloc_addr_valid(kaddr) \
	(((void *)(kaddr) >= (void *)VMALLOC_START) && \
	((void *)(kaddr) < (void *)VMALLOC_END))

#define modules_addr_valid(kaddr) \
	(((void *)(kaddr) >= (void *)MODULES_VADDR) && \
	((void *)(kaddr) < (void *)MODULES_END))

#define TST_CMD_01 1
#define TST_CMD_02 2
#define TST_CMD_03 3
#define TST_CMD_04 4
#define TST_CMD_05 5

#define MAX_SHA_256_SZ 32

#define TC_NS_CLIENT_IOCTL_SES_OPEN_REQ \
	 _IOW(TC_NS_CLIENT_IOC_MAGIC, 1, tc_ns_client_context)
#define TC_NS_CLIENT_IOCTL_SES_CLOSE_REQ \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 2, tc_ns_client_context)
#define TC_NS_CLIENT_IOCTL_SEND_CMD_REQ \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 3, tc_ns_client_context)
#define TC_NS_CLIENT_IOCTL_SHRD_MEM_RELEASE \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 4, unsigned int)
#define TC_NS_CLIENT_IOCTL_WAIT_EVENT \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 5, unsigned int)
#define TC_NS_CLIENT_IOCTL_SEND_EVENT_RESPONSE \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 6, unsigned int)
#define TC_NS_CLIENT_IOCTL_REGISTER_AGENT \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 7, struct agent_ioctl_args)
#define TC_NS_CLIENT_IOCTL_UNREGISTER_AGENT \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 8, unsigned int)
#define TC_NS_CLIENT_IOCTL_LOAD_APP_REQ \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 9, struct load_secfile_ioctl_struct)
#define TC_NS_CLIENT_IOCTL_NEED_LOAD_APP \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 10, tc_ns_client_context)
#define TC_NS_CLIENT_IOCTL_ALLOC_EXCEPTING_MEM \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 12, unsigned int)
#define TC_NS_CLIENT_IOCTL_CANCEL_CMD_REQ \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 13, tc_ns_client_context)
#define TC_NS_CLIENT_IOCTL_LOGIN \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 14, int)
#define TC_NS_CLIENT_IOCTL_TST_CMD_REQ \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 15, int)
#define TC_NS_CLIENT_IOCTL_TUI_EVENT \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 16, int)
#define TC_NS_CLIENT_IOCTL_SYC_SYS_TIME \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 17, tc_ns_client_time)
#define TC_NS_CLIENT_IOCTL_SET_NATIVECA_IDENTITY \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 18, int)
#define TC_NS_CLIENT_IOCTL_LOAD_TTF_FILE_AND_NOTCH_HEIGHT \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 19, unsigned int)
#define TC_NS_CLIENT_IOCTL_LOW_TEMPERATURE_MODE\
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 20, unsigned int)
#define TC_NS_CLIENT_IOCTL_LATEINIT\
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 21, unsigned int)
#define TC_NS_CLIENT_IOCTL_GET_TEE_VERSION \
	_IOWR(TC_NS_CLIENT_IOC_MAGIC, 22, unsigned int)

#endif
