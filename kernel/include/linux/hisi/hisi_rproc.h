/* hisi_rproc.h */

#ifndef __HISI_RPROC_H__
#define __HISI_RPROC_H__

#include <linux/hisi/hisi_mailbox.h>
#include <linux/errno.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/*
 * cur rproc_id_t total, to init rproc_struct
 * NS_IPC(32) + AO_IPC(6) + NPU_IPC(6) + CFG_IPC(32) = 76
 */
#define	HISI_RPROC_NUM	(76)
typedef enum {
	 HISI_RPROC_LPM3_MBX0 = 0,
	 HISI_RPROC_RDR_MBX1,
	 HISI_RPROC_HIFI_MBX2,
	 HISI_RPROC_DEFAULT_MBX3,
	 HISI_RPROC_IOM3_MBX4,
	 HISI_RPROC_IVP_MBX5,
	 HISI_RPROC_IVP_MBX6,
	 HISI_RPROC_DEFAULT_MBX7,
/* MBX8 and MBX9 are not used in austin and dallas */
	 HISI_RPROC_ISP_MBX8,
	 HISI_RPROC_ISP_MBX9,
	 HISI_RPROC_IOM3_MBX10,
	 HISI_RPROC_IOM3_MBX11,
	 HISI_RPROC_IOM3_MBX12,
	 HISI_RPROC_LPM3_MBX13,
	 HISI_RPROC_LPM3_MBX14,
	 HISI_RPROC_LPM3_MBX15,
	 HISI_RPROC_LPM3_MBX16,
	 HISI_RPROC_LPM3_MBX17,
	 HISI_RPROC_HIFI_MBX18,
	 HISI_RPROC_MODEM_A9_MBX19,
	 HISI_RPROC_MODEM_A9_MBX20,
	 HISI_RPROC_MODEM_A9_MBX21,	 
	 HISI_RPROC_MODEM_BBE16_MBX22,
	 HISI_RPROC_ISP_MBX23,
	 HISI_RPROC_ISP_MBX24,
	 HISI_RPROC_IVP_MBX25,
	 HISI_RPROC_IVP_MBX26,
	 HISI_RPROC_LPM3_MBX27,
	 HISI_RPROC_LPM3_MBX28,
	 HISI_RPROC_HIFI_MBX29,
	 HISI_RPROC_LPM3_MBX30,
	 HISI_RPROC_MBX31,
	 HISI_RPROC_NSIPC_MAX = 32,
	 HISI_RPROC_ISP_MBX0 = 100,
	 HISI_RPROC_ISP_MBX1,
	 HISI_RPROC_ISP_MBX2,
	 HISI_RPROC_ISP_MBX3,
	 HISI_RPROC_AO_MBX0 = 200,
	 HISI_RPROC_AO_MBX1,
	 HISI_RPROC_AO_MBX2,
	 HISI_RPROC_AO_MBX3,
	 HISI_RPROC_AO_MBX4,
	 HISI_RPROC_AO_MBX5,
	 HISI_RPROC_AO_NSIPC_MAX = 206,
	 HISI_RPROC_NPU_MBX0 = 300,
	 HISI_RPROC_NPU_MBX1,
	 HISI_RPROC_NPU_MBX2,
	 HISI_RPROC_NPU_MBX3,
	 HISI_RPROC_NPU_MBX4,
	 HISI_RPROC_NPU_MBX5,
	 HISI_RPROC_NPU_IPC_MAX = 306,
	 HISI_RPROC_CFGIPC_MBX0 = 400,
	 HISI_RPROC_CFGIPC_MBX1,
	 HISI_RPROC_CFGIPC_MBX2,
	 HISI_RPROC_CFGIPC_MBX3,
	 HISI_RPROC_CFGIPC_MBX4,
	 HISI_RPROC_CFGIPC_MBX5,
	 HISI_RPROC_CFGIPC_MBX6,
	 HISI_RPROC_CFGIPC_MBX7,
	 HISI_RPROC_CFGIPC_MBX8,
	 HISI_RPROC_CFGIPC_MBX9,
	 HISI_RPROC_CFGIPC_MBX10,
	 HISI_RPROC_CFGIPC_MBX11,
	 HISI_RPROC_CFGIPC_MBX12,
	 HISI_RPROC_CFGIPC_MBX13,
	 HISI_RPROC_CFGIPC_MBX14,
	 HISI_RPROC_CFGIPC_MBX15,
	 HISI_RPROC_CFGIPC_MBX16,
	 HISI_RPROC_CFGIPC_MBX17,
	 HISI_RPROC_CFGIPC_MBX18,
	 HISI_RPROC_CFGIPC_MBX19,
	 HISI_RPROC_CFGIPC_MBX20,
	 HISI_RPROC_CFGIPC_MBX21,
	 HISI_RPROC_CFGIPC_MBX22,
	 HISI_RPROC_CFGIPC_MBX23,
	 HISI_RPROC_CFGIPC_MBX24,
	 HISI_RPROC_CFGIPC_MBX25,
	 HISI_RPROC_CFGIPC_MBX26,
	 HISI_RPROC_CFGIPC_MBX27,
	 HISI_RPROC_CFGIPC_MBX28,
	 HISI_RPROC_CFGIPC_MBX29,
	 HISI_RPROC_CFGIPC_MBX30,
	 HISI_RPROC_CFGIPC_MBX31,
	 HISI_RPROC_CFGIPC_MAX = 432,
}rproc_id_t;

#define RPROC_SYNC_SEND(rproc_id, msg, len,				\
		 ack_buffer, ack_buffer_len)		\
hisi_rproc_xfer_sync(rproc_id, msg, len,				\
		 ack_buffer, ack_buffer_len)

#define RPROC_ASYNC_SEND(rproc_id, msg, len)		\
hisi_rproc_xfer_async(rproc_id, msg, len)

#define RPROC_MONITOR_REGISTER(rproc_id, nb)				\
hisi_rproc_rx_register(rproc_id, nb)

#define RPROC_MONITOR_UNREGISTER(rproc_id, nb)			\
hisi_rproc_rx_unregister(rproc_id, nb)

#define RPROC_PUT(rproc_id)			hisi_rproc_put(rproc_id)
#define RPROC_FLUSH_TX(rproc_id)	hisi_rproc_flush_tx(rproc_id)
#define RPROC_IS_SUSPEND(rproc_id)			hisi_rproc_is_suspend(rproc_id)

extern int hisi_rproc_xfer_sync(rproc_id_t rproc_id,
				rproc_msg_t *msg,
				rproc_msg_len_t len,
				rproc_msg_t *ack_buffer,
				rproc_msg_len_t ack_buffer_len);
extern int hisi_rproc_xfer_async(rproc_id_t rproc_id,
				rproc_msg_t *msg,
				rproc_msg_len_t len
				);
extern int
hisi_rproc_rx_register(rproc_id_t rproc_id, struct notifier_block *nb);
extern int
hisi_rproc_rx_unregister(rproc_id_t rproc_id, struct notifier_block *nb);
extern int   hisi_rproc_put(rproc_id_t rproc_id);
extern int   hisi_rproc_flush_tx(rproc_id_t rproc_id);
extern int hisi_rproc_is_suspend(rproc_id_t rproc_id);
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* __HISI_RPROC_H__ */
