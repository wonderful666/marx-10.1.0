/*
 * teek_client_id.h
 *
 * define exported data for secboot CA
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
#ifndef _TEE_CLIENT_ID_H_
#define _TEE_CLIENT_ID_H_

#define TEE_SERVICE_SECBOOT \
{ \
	0x08080808, \
	0x0808, \
	0x0808, \
	{ \
		0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08 \
	} \
}

/* e7ed1f64-4687-41da-96dc-cbe4f27c838f */
#define TEE_SERVICE_ANTIROOT \
{ \
	0xE7ED1F64, \
	0x4687, \
	0x41DA, \
	{ \
		0x96, 0xDC, 0xCB, 0xE4, 0xF2, 0x7C, 0x83, 0x8F \
	} \
}
/* dca5ae8a-769e-4e24-896b-7d06442c1c0e */
#define TEE_SERVICE_SECISP \
{ \
	0xDCA5AE8A, \
	0x769E, \
	0x4E24, \
	{ \
		0x89, 0x6B, 0x7D, 0x06, 0x44, 0x2C, 0x1C, 0x0E \
	} \
}
/*
 * @ingroup  TEE_COMMON_DATA
 *
 * 安全服务secboot支持的命令ID
 */
enum SVC_SECBOOT_CMD_ID {
	SECBOOT_CMD_ID_INVALID = 0x0,    /* Secboot Task 无效ID */
	SECBOOT_CMD_ID_COPY_VRL,         /* Secboot Task 拷贝VRL */
	SECBOOT_CMD_ID_COPY_DATA,        /* Secboot Task 拷贝镜像*/
	SECBOOT_CMD_ID_VERIFY_DATA,      /* Secboot Task 验证*/
	SECBOOT_CMD_ID_RESET_IMAGE,      /* Secboot Task 复位SoC*/
	SECBOOT_CMD_ID_COPY_VRL_TYPE,    /* Secboot Task 拷贝VRL，并传递SoC Type */
	SECBOOT_CMD_ID_COPY_DATA_TYPE,   /* Secboot Task 拷贝镜像,并传递SoC Type */
	SECBOOT_CMD_ID_VERIFY_DATA_TYPE, /* Secboot Task 校验，并传递SoC Type，校验成功解复位SoC */
	SECBOOT_CMD_ID_VERIFY_DATA_TYPE_LOCAL, /* Secboot Task原地校验，并传递SoC Type,校验成功解复位SoC */
	SECBOOT_CMD_ID_COPY_IMG_TYPE,          /* Secboot Task Copy img from secure buffer to run addr> */
	SECBOOT_CMD_ID_BSP_MODEM_CALL,         /* Secboot Task 执行对应函数*/
	SECBOOT_CMD_ID_BSP_MODULE_VERIFY,      /* Secboot Task modem module校验函数*/
	SECBOOT_CMD_ID_BSP_ICC_OPEN_THREAD,    /* Secboot Task icc open函数*/
	SECBOOT_CMD_ID_BSP_RFILE_RW_THREAD,    /* Secboot Task rfile thread函数*/
	SECBOOT_CMD_ID_GET_RNG_NUM,            /* Secboot Task 获取硬件随机数 */
};

/*
 * @ingroup TEE_COMMON_DATA
 *
 * 安全服务secboot支持的镜像类型
 */
#ifdef CONFIG_HISI_SECBOOT_IMG

#define CAS 0xff
enum SVC_SECBOOT_IMG_TYPE {
    MODEM,
    DSP,
    XDSP,
    TAS,
    WAS,
    MODEM_COMM_IMG,
    MODEM_DTB,
    NVM,
    NVM_S,
    MBN_R,
    MBN_A,
    MODEM_COLD_PATCH,
    DSP_COLD_PATCH,
    MODEM_CERT,
    MAX_SOC_MODEM,
    HIFI,
    ISP,
    IVP,
    SOC_MAX
};
#elif defined(CONFIG_HISI_SECBOOT_IMG_V2)
enum SVC_SECBOOT_IMG_TYPE {
	HIFI,
	ISP,
	IVP,
	MAX_AP_SOC,
	MODEM_START = 0x100,
	MODEM_END = 0x1FF,
	MAX_SOC,
};
#else
enum SVC_SECBOOT_IMG_TYPE {
	MODEM,
	HIFI,
	DSP,
	XDSP,
	TAS,
	WAS,
	CAS,
	MODEM_DTB,
	ISP,
/* miami c30上需要支持冷补丁特性，该分支的modem代码与miami_c30 Modem代码共分支；
  * 增加冷补丁枚举是为了在该分支编译通过miami版本，安全OS中没有增加对应的枚举，该分支上编译的miami版本不支持冷补丁特性；
  * 分支对应关系链接：http://3ms.huawei.com/hi/group/8729/wiki_5190309.html
  */
#ifdef CONFIG_COLD_PATCH
	MODEM_COLD_PATCH,
	DSP_COLD_PATCH,
#endif
	SOC_MAX
};
#endif

#endif
