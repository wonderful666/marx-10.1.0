

#ifndef __HMAC_VAP_H__
#define __HMAC_VAP_H__

#ifdef __cplusplus // windows ut编译不过，后续下线清理
#if __cplusplus
extern "C" {
#endif
#endif

/* 1 其他头文件包含 */
#include "oal_ext_if.h"
#include "mac_vap.h"
#include "hmac_ext_if.h"
#include "hmac_user.h"
#include "hmac_main.h"
#include "mac_resource.h"
#ifdef _PRE_WLAN_TCP_OPT
#include "hmac_tcp_opt_struc.h"
#include "oal_hcc_host_if.h"
#endif
#ifdef _PRE_WLAN_FEATURE_BTCOEX
#include "hmac_btcoex.h"
#endif
#include "hmac_arp_probe.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_VAP_H

/* 2 宏定义 */
#ifdef _PRE_WLAN_DFT_STAT
#define HMAC_VAP_DFT_STATS_PKT_INCR(_member, _cnt) ((_member) += (_cnt))
#else
#define HMAC_VAP_DFT_STATS_PKT_INCR(_member, _cnt)
#endif
#define HMAC_VAP_STATS_PKT_INCR(_member, _cnt) ((_member) += (_cnt))

#ifdef _PRE_WLAN_FEATURE_HS20
#define MAX_QOS_UP_RANGE 8
#define MAX_DSCP_EXCEPT  21 /* maximum of DSCP Exception fields for QoS Map set */
#endif

#define HMAC_MAX_DSCP_VALUE_NUM 64
#define HMAC_DSCP_VALUE_INVALID 0xA5

#ifdef _PRE_WLAN_FEATURE_FTM
#define HMAC_GAS_DIALOG_TOKEN_INITIAL_VALUE 0x80        /* hmac gas的dialog token num */
#endif

/* 3 枚举定义 */
/* 初始化vap特性枚举 */
/* 4 全局变量声明 */
#ifdef _PRE_WLAN_REPORT_PRODUCT_LOG
extern oal_uint8 vapid_to_chipid[2 * WLAN_VAP_MAX_NUM_PER_DEVICE_LIMIT];
#endif

extern oal_uint8 g_uc_host_rx_ampdu_amsdu;
/* 5 消息头定义 */
/* 6 消息定义 */
/* 7 STRUCT定义 */
typedef struct {
    oal_dlist_head_stru st_timeout_head;
} hmac_mgmt_timeout_stru;

typedef struct {
    oal_uint16 us_user_index;
    mac_vap_state_enum_uint8 en_state;
    oal_uint8 uc_vap_id;
    mac_status_code_enum_uint16 en_status_code;
    oal_uint8 auc_rsv[2];
} hmac_mgmt_timeout_param_stru;

#ifdef _PRE_WLAN_FEATURE_HS20
typedef struct {
    oal_uint8 auc_up_low[MAX_QOS_UP_RANGE]; /* User Priority */
    oal_uint8 auc_up_high[MAX_QOS_UP_RANGE];
    oal_uint8 auc_dscp_exception_up[MAX_DSCP_EXCEPT]; /* User Priority of DSCP Exception field */
    oal_uint8 uc_valid;
    oal_uint8 uc_num_dscp_except;
    oal_uint8 auc_dscp_exception[MAX_DSCP_EXCEPT]; /* DSCP exception field  */
} hmac_cfg_qos_map_param_stru;
#endif

/* 修改此结构体需要同步通知SDT，否则上报无法解析 */
typedef struct {
    /***************************************************************************
                                收送包统计
    ***************************************************************************/
    /* 发往lan的数据包统计 */
    oal_uint32 ul_rx_pkt_to_lan;   /* 接收流程发往以太网的数据包数目，MSDU */
    oal_uint32 ul_rx_bytes_to_lan; /* 接收流程发往以太网的字节数 */

    /***************************************************************************
                                发送包统计
    ***************************************************************************/
    /* 从lan接收到的数据包统计 */
    oal_uint32 ul_tx_pkt_num_from_lan; /* 从lan过来的包数目,MSDU */
    oal_uint32 ul_tx_bytes_from_lan;   /* 从lan过来的字节数 */
} hmac_vap_query_stats_stru;
/* 装备测试 */
typedef struct {
    oal_uint32 ul_rx_pkct_succ_num;    /* 接收数据包数 */
    oal_uint32 ul_dbb_num;             /* DBB版本号 */
    oal_uint32 ul_check_fem_pa_status; /* fem和pa是否烧毁标志 */
    oal_int16 s_rx_rssi;
    oal_bool_enum_uint8 uc_get_dbb_completed_flag; /* 获取DBB版本号成功上报标志 */
    oal_bool_enum_uint8 uc_check_fem_pa_flag;      /* fem和pa是否烧毁上报标志 */
    oal_bool_enum_uint8 uc_get_rx_pkct_flag;       /* 接收数据包上报标志位 */
    oal_bool_enum_uint8 uc_lte_gpio_check_flag;    /* 接收数据包上报标志位 */
    oal_bool_enum_uint8 uc_report_efuse_reg_flag;  /* efuse 寄存器读取 */
    oal_bool_enum_uint8 uc_report_reg_flag;        /* 单个寄存器读取 */
    oal_uint32 ul_reg_value;
    oal_uint8 uc_ant_status : 4,
              uc_get_ant_flag : 4;
} hmac_atcmdsrv_get_stats_stru;

typedef struct {
    oal_dlist_head_stru st_entry;
    oal_uint8 auc_bssid[WLAN_MAC_ADDR_LEN];
    oal_uint8 uc_reserved[2];
    oal_uint8 auc_pmkid[WLAN_PMKID_LEN];
} hmac_pmksa_cache_stru;

typedef enum _hmac_tcp_opt_queue_ {
    HMAC_TCP_ACK_QUEUE = 0,
    HMAC_TCP_OPT_QUEUE_BUTT
} hmac_tcp_opt_queue;

#ifdef _PRE_WLAN_TCP_OPT
typedef oal_uint16 (*hmac_trans_cb_func)(void *pst_hmac_device, hmac_tcp_opt_queue type, hcc_chan_type dir, void *data);
/* tcp_ack优化 */
typedef struct {
    struct wlan_perform_tcp hmac_tcp_ack;
    struct wlan_perform_tcp_list hmac_tcp_ack_list;
    wlan_perform_tcp_impls filter_info;
    hmac_trans_cb_func filter[HMAC_TCP_OPT_QUEUE_BUTT];  // 过滤处理钩子函数
    oal_uint64 all_ack_count[HMAC_TCP_OPT_QUEUE_BUTT];   // 丢弃的TCP ACK统计
    oal_uint64 drop_count[HMAC_TCP_OPT_QUEUE_BUTT];      // 丢弃的TCP ACK统计
    oal_netbuf_head_stru data_queue[HMAC_TCP_OPT_QUEUE_BUTT];
    oal_spin_lock_stru data_queue_lock[HMAC_TCP_OPT_QUEUE_BUTT];
} hmac_tcp_ack_stru;
#endif

#if defined(_PRE_WLAN_FEATURE_WDS) || defined(_PRE_WLAN_FEATURE_VIRTUAL_MULTI_STA)
#define WDS_HASH_NUM   (1 << 2)
#define WDS_AGING_TIME (300 * OAL_TIME_HZ)

typedef enum {
    WDS_MODE_NONE = 0,
    WDS_MODE_ROOTAP,
    WDS_MODE_REPEATER_STA,
    WDS_MODE_BUTT
} hmac_vap_wds_vap_mode_enum;
typedef oal_uint8 hmac_vap_wds_vap_mode_enum_uint8;

typedef struct {
    oal_rwlock_stru st_lock;
    frw_timeout_stru st_wds_timer;
    oal_dlist_head_stru st_peer_node[WDS_HASH_NUM];
    oal_dlist_head_stru st_wds_stas[WDS_HASH_NUM];
    oal_dlist_head_stru st_neigh[WDS_HASH_NUM];
    oal_uint32 ul_wds_aging;
    hmac_vap_wds_vap_mode_enum_uint8 en_wds_vap_mode;
    oal_uint8 uc_wds_node_num;
    oal_uint16 uc_wds_stas_num;
    oal_uint16 uc_neigh_num;
    oal_uint8 auc_resv[2];
} hmac_vap_wds_stru;
#endif

/* 查询事件ID枚举 */
typedef enum {
    QUERY_ID_KO_VERSION = 0,
    QUERY_ID_PWR_REF = 1,
    QUERY_ID_BCAST_RATE = 2,
    QUERY_ID_STA_INFO = 3,
    QUERY_ID_ANT_RSSI = 4,
    QUERY_ID_STA_DIAG_INFO = 5,
    QUERY_ID_VAP_DIAG_INFO = 6,
    QUERY_ID_SENSING_BSSID_INFO = 7,
    QUERY_ID_BG_NOISE = 8,
#ifdef _PRE_FEATURE_FAST_AGING
    QUERY_ID_FAST_AGING = 9,
#endif
#ifdef _PRE_WLAN_FEATURE_TCP_ACK_BUFFER
    QUERY_ID_TCP_ACK_BUF = 10,
#endif
    QUERY_ID_MODE_BUTT
} hmac_vap_query_enent_id_enum;
typedef oal_uint8 hmac_vap_query_enent_id_enumm_uint8;

typedef struct {
    oal_uint8 uc_type;
    oal_uint8 uc_eid;
    oal_uint8 auc_resv[2];
} hmac_remove_ie_stru;

typedef enum {

    HMAC_VAP_ARP_PROBE_TYPE_HTC = BIT0, /* htc arp 探测失败会关闭htc触发重关联 */
    HMAC_VAP_ARP_PROBE_TYPE_M2S = BIT1, /* 切换成siso 触发重关联 */
    HMAC_VAP_ARP_PROBE_TYPE_DBAC = BIT2, /* 频繁发送NULL DATA场关闭RX AMSDU触发重关联 */
    HMAC_VAP_ARP_PROBE_TYPE_BTCOEX = BIT3, /* 频繁发送NULL DATA场关闭RX AMSDU触发重关联 */

    HMAC_VAP_ARP_PROBE_TYPE_BUTT,
} hmac_vap_arp_probe_type_enum;
typedef oal_uint8 hmac_vap_arp_probe_type_enum_uint8;


typedef struct {
    frw_timeout_stru st_arp_probe_timer; /* 发送ARP REQ后启动定时器 */
    oal_atomic ul_rx_unicast_pkt_to_lan; /* 接收到的单播帧个数 */

    oal_uint8 uc_rx_no_pkt_count;        /* 超时时间内没有收到帧的次数 */
    oal_bool_enum_uint8 en_arp_probe_on; /* 是否打开arp统计，做重关联保护 */
    oal_uint8 uc_arp_probe_type;
    oal_bool_enum_uint8 en_init_succ;
} hmac_vap_arp_probe_stru;

/* hmac vap结构体 */
/* 在向此结构体中增加成员的时候，请保持整个结构体8字节对齐 */
typedef struct hmac_vap_tag {
    /* ap sta公共字段 */
    oal_net_device_stru *pst_net_device;  /* VAP对应的net_devices */
    oal_uint8 auc_name[OAL_IF_NAME_SIZE]; /* VAP名字 */
    hmac_vap_cfg_priv_stru st_cfg_priv;   /* wal hmac配置通信接口 */

    oal_spin_lock_stru st_lock_state; /* 数据面和控制面对VAP状态进行互斥 */

    oal_mgmt_tx_stru st_mgmt_tx;
    frw_timeout_stru st_mgmt_timer;
    hmac_mgmt_timeout_param_stru st_mgmt_timetout_param;

    frw_timeout_stru st_scan_timeout; /* vap发起扫描时，会启动定时器，做超时保护处理 */
    oal_uint8 uc_scan_upload_num;

#ifdef _PRE_WLAN_FEATURE_AMPDU_TX_HW
    oal_delayed_work st_ampdu_work;
    oal_spin_lock_stru st_ampdu_lock;
    oal_delayed_work st_set_hw_work;
    mac_cfg_ampdu_tx_on_param_stru st_mode_set;
#endif

    hmac_remove_ie_stru st_remove_ie; /* 配置需要屏蔽的IE信息 */

#if (_PRE_WLAN_FEATURE_BLACKLIST_LEVEL != _PRE_WLAN_FEATURE_BLACKLIST_NONE)
    mac_blacklist_info_stru *pst_blacklist_info; /* 黑名单信息 */
#endif
#ifdef _PRE_WLAN_FEATURE_ISOLATION
    mac_isolation_info_stru st_isolation_info; /* 用户隔离信息 */
#endif

#ifdef _PRE_WLAN_FEATURE_P2P
    oal_net_device_stru *pst_p2p0_net_device; /* 指向p2p0 net device */
    oal_net_device_stru *pst_del_net_device;  /* 指向需要通过cfg80211 接口删除的 net device */
    oal_work_stru st_del_virtual_inf_worker;  /* 删除net_device 工作队列 */
    oal_bool_enum_uint8 en_wait_roc_end;

    oal_uint8 bit_in_p2p_listen:1; /* vap是否在p2p listen流程 */
    oal_uint8 bit_resv:7;
    oal_uint8 auc_resv6[2];

    oal_completion st_roc_end_ready; /* roc end completion */

#endif

#ifdef _PRE_WLAN_FEATURE_HS20
    hmac_cfg_qos_map_param_stru st_cfg_qos_map_param;
#endif

#ifdef _PRE_WLAN_FEATURE_ALWAYS_TX
    oal_uint8 bit_init_flag : 1;  /* 常发关闭再次打开标志 */
    oal_uint8 bit_ack_policy : 1; /* ack policy: 0:normal ack 1:normal ack */
    oal_uint8 bit_reserved : 6;
    oal_uint8 auc_resv1[3];
#endif

#ifdef _PRE_WLAN_FEATURE_ROAM
    oal_uint32 *pul_roam_info;
    oal_bool_enum_uint8 en_roam_prohibit_on; /* 是否禁止漫游 */
#endif

    /* 组播转单播字段 */
#if defined(_PRE_WLAN_FEATURE_MCAST) || defined(_PRE_WLAN_FEATURE_HERA_MCAST)
    oal_void *pst_m2u;
#endif

#ifdef _PRE_WLAN_DFT_STAT
    oal_uint8 uc_device_distance;
    oal_uint8 uc_intf_state_cca;
    oal_uint8 uc_intf_state_co;
    oal_uint8 auc_resv2[1];
#endif

    oal_uint16 us_rx_timeout[WLAN_WME_AC_BUTT];     /* 不同业务重排序超时时间 */
    oal_uint16 us_rx_timeout_min[WLAN_WME_AC_BUTT]; /* 不同业务重排序超时时间 */

    oal_uint16 us_del_timeout;                     /* 多长时间超时删除ba会话 如果是0则不删除 */
    mac_cfg_mode_param_stru st_preset_para;        /* STA协议变更时变更前的协议模式 */
    oal_uint8 auc_supp_rates[WLAN_MAX_SUPP_RATES]; /* 支持的速率集 */

#if defined(_PRE_WLAN_FEATURE_DBAC) && defined(_PRE_WLAN_FEATRUE_DBAC_DOUBLE_AP_MODE)
    oal_bool_enum_uint8 en_omit_acs_chan;
#else
    oal_uint8 auc_resv3[1];
#endif

#ifdef _PRE_WLAN_FEATURE_SINGLE_CHIP_DUAL_BAND
    wlan_channel_band_enum en_restrict_band;
#else
    oal_uint8 auc_resv4[1];
#endif

#ifdef _PRE_WLAN_FEATURE_WMMAC
    oal_uint8 uc_ts_dialog_token; /* TS会话创建伪随机值 */
    oal_bool_enum_uint8 en_wmmac_auth_flag;
    oal_uint8 uc_resv5[2];
#endif  // _PRE_WLAN_FEATURE_WMMAC

    oal_bool_enum_uint8 auc_query_flag[QUERY_ID_MODE_BUTT];
    mac_cfg_param_char_stru st_param_char;

    /* sta独有字段 */
    oal_bool_enum_uint8 en_no_beacon;
    oal_uint8 uc_no_smps_user_cnt_ap;              /* AP关联的不支持SMPS的用户数  */
    oal_uint8 uc_ba_dialog_token;                  /* BA会话创建伪随机值 */
    oal_bool_enum_uint8 en_updata_rd_by_ie_switch; /* 是否根据关联的ap跟新自己的国家码 */

    oal_uint8 bit_sta_protocol_cfg : 1;
    oal_uint8 bit_protocol_fall : 1; /* 降协议标志位 */
    oal_uint8 bit_reassoc_flag : 1;  /* 关联过程中判断是否为重关联动作 */
    
    oal_uint8 bit_rx_ampduplusamsdu_active : 2;
    oal_uint8 bit_sae_connect_with_pmkid : 1; /* 0:不包含PMKID的SAE连接；1:包含PMKID的SAE连接 */
#if defined(_PRE_WLAN_FEATURE_11K) || defined(_PRE_WLAN_FEATURE_11R) ||      \
    defined(_PRE_WLAN_FEATURE_11V_ENABLE)
    oal_uint8 bit_11k_auth_flag : 1; /* 11k 认证标志位 */
    oal_uint8 bit_voe_11r_auth : 1;
    oal_uint8 bit_11k_auth_oper_class : 2;
    oal_uint8 bit_11r_over_ds : 1; /* 是否使用11r over ds；0表示over ds走over air流程 */
    oal_uint8 bit_11k_enable : 1;
    oal_uint8 bit_11v_enable : 1;
    oal_uint8 bit_11r_enable : 1;
    oal_uint8 bit_bcn_table_switch : 1;
    oal_uint8 bit_adaptive11r : 1; /* 是否使用adaptive 11r流程 */
    oal_uint8 bit_nb_rpt_11k : 1;
    oal_uint8 bit_11r_private_preauth : 1;
    oal_uint8 bit_resv2 : 7;
    oal_uint8 uc_resv;
#else
    oal_uint8 bit_resv : 2;
#endif                              // _PRE_WLAN_FEATURE_11K
    oal_int8 ac_desired_country[3]; /* 要加入的AP的国家字符串，前两个字节为国家字母，第三个为\0 */

    oal_dlist_head_stru st_pmksa_list_head;

    /* 信息上报 */
    oal_wait_queue_head_stru query_wait_q; /* 查询等待队列 */
    oal_station_info_stru station_info;
    station_info_extend_stru st_station_info_extend; /* CHR2.0使用的STA统计信息 */

    oal_bool_enum_uint8 station_info_query_completed_flag; /* 查询结束标志，OAL_TRUE，查询结束，OAL_FALSE，查询未结束 */
    oal_int16 s_free_power;                                /* 底噪 */
    oal_bool_enum_uint8 en_monitor_mode;

    oal_int32 center_freq; /* 中心频点 */
    mac_cfg_show_dieid_stru st_dieid;
    hmac_atcmdsrv_get_stats_stru st_atcmdsrv_get_status;
    oal_proc_dir_entry_stru *pst_proc_dir; /* vap对应的proc目录 */

#ifdef _PRE_WLAN_DFT_STAT
    /* 统计信息+信息上报新增字段，修改这个字段，必须修改SDT才能解析正确 */
    hmac_vap_query_stats_stru st_query_stats;
#endif

#ifdef _PRE_WLAN_FEATURE_EDCA_OPT_AP
    frw_timeout_stru st_edca_opt_timer; /* edca参数调整定时器 */
    oal_uint32 ul_edca_opt_time_ms;     /* edca参数调整计时器周期 */
    oal_uint8 uc_edca_opt_flag_ap;      /* ap模式下是否使能edca优化特性 */
    oal_uint8 uc_edca_opt_flag_sta;     /* sta模式下是否使能edca优化特性 */
    oal_uint8 uc_edca_opt_weight_sta;   /* 调整beacon中edca参数的权重，最大值为 3 */
    oal_uint8 uc_idle_cycle_num;        /* vap连续处于idle状态的周期个数 */
#endif

#ifdef _PRE_WLAN_TCP_OPT
    hmac_tcp_ack_stru st_hamc_tcp_ack[HCC_DIR_COUNT];
#endif

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
    mac_h2d_protection_stru st_prot;
#endif

#ifdef _PRE_WLAN_FEATURE_STA_PM
    frw_timeout_stru st_ps_sw_timer; /* 低功耗开关 */
    oal_uint8 uc_cfg_sta_pm_manual;  /* 手动设置sta pm mode的标志 */
    oal_uint8 uc_ps_mode;
    oal_uint16 us_check_timer_pause_cnt; /* 低功耗pause计数 */
#endif

#ifdef _PRE_WLAN_FEATURE_SMARTANT
    /* 查询结束标志，OAL_TRUE，查询结束，OAL_FALSE，查询未结束 */
    oal_bool_enum_uint8 en_ant_info_query_completed_flag;
    /* 查询结束标志，OAL_TRUE，查询结束，OAL_FALSE，查询未结束 */
    oal_bool_enum_uint8 en_double_ant_switch_query_completed_flag;
    oal_uint8 auc_resv10[2];
    oal_uint32 ul_double_ant_switch_ret;
#endif

    oal_int32 l_temp;
#if defined(_PRE_WLAN_FEATURE_WDS) || defined(_PRE_WLAN_FEATURE_VIRTUAL_MULTI_STA)
    hmac_vap_wds_stru st_wds_table;
#endif

#ifdef _PRE_WLAN_FEATURE_CAR
    hmac_car_limit_stru st_car_vap_cfg[HMAC_CAR_BUTT]; /* vap限速结构体,0-up 1-down */
#endif
    oal_uint8 auc_dscp_tid_map[HMAC_MAX_DSCP_VALUE_NUM];
    hmac_vap_arp_probe_stru st_hmac_arp_probe;
#ifdef _PRE_WLAN_FEATURE_SAE
    oal_work_stru st_sae_report_ext_auth_worker; /* SAE report external auth req工作队列 */
#endif
    oal_uint8 owe_group[MAC_OWE_GROUP_SUPPORTED_NUM];
    oal_uint8 owe_group_cap;

    oal_bool_enum_uint8 en_is_psk; /* 这次关联是否是psk */
    oal_bool_enum_uint8 en_sae_connect;
    oal_bool_enum_uint8 en_ps_rx_amsdu;
    oal_uint8 auc_resv9[1];

#ifdef _PRE_WLAN_FEATURE_TWT
    mac_cfg_twt_stru st_twt_cfg; /* TWT节能配置参数 */
#endif
#ifdef _PRE_WLAN_FEATURE_11AX
    oal_uint8 auc_resv11[1];
    oal_uint8 uc_mac_padding;
    oal_uint8 uc_disable_mu_edca;
    oal_uint8 uc_htc_order_flag;
    oal_uint32 ul_htc_info;
#endif

#ifdef _PRE_WLAN_FEATURE_FTM // FTM认证，用于gas init comeback定时
    frw_timeout_stru    st_ftm_timer;
    mac_gas_mgmt_stru   st_gas_mgmt;
#endif

    mac_vap_stru st_vap_base_info; /* MAC vap，只能放在最后! */
} hmac_vap_stru;

typedef enum _hmac_cac_event_ {
    HMAC_CAC_STARTED = 0,
    HMAC_CAC_FINISHED,
    HMAC_CAC_ABORTED,

    HMAC_CAC_BUTT
} hmac_cac_event;

typedef struct {
    hmac_cac_event en_type;
    oal_uint32 ul_freq;
    wlan_channel_bandwidth_enum_uint8 en_bw_mode;
    oal_uint8 auc[23];
} hmac_cac_event_stru;

/* 8 UNION定义 */
/* 9 OTHERS定义 */
/* 10 函数声明 */
extern oal_uint32 hmac_vap_destroy_etc(hmac_vap_stru *pst_vap);
extern oal_uint32 hmac_vap_init_etc(hmac_vap_stru *pst_hmac_vap,
                                    oal_uint8 uc_chip_id,
                                    oal_uint8 uc_device_id,
                                    oal_uint8 uc_vap_id,
                                    mac_cfg_add_vap_param_stru *pst_param);

extern oal_uint32 hmac_vap_creat_netdev_etc(hmac_vap_stru *pst_hmac_vap,
                                            oal_int8 *puc_netdev_name,
                                            oal_uint8 uc_netdev_name_len,
                                            oal_int8 *puc_mac_addr);

extern oal_uint16 hmac_vap_check_ht_capabilities_ap_etc(hmac_vap_stru *pst_hmac_vap,
                                                        oal_uint8 *puc_payload,
                                                        oal_uint32 ul_msg_len,
                                                        hmac_user_stru *pst_hmac_user_sta);
extern oal_uint32 hmac_search_ht_cap_ie_ap_etc(hmac_vap_stru *pst_hmac_vap,
                                               hmac_user_stru *pst_hmac_user_sta,
                                               oal_uint8 *puc_payload,
                                               oal_uint16 us_index,
                                               oal_bool_enum en_prev_asoc_ht);
extern oal_bool_enum_uint8 hmac_vap_addba_check(hmac_vap_stru *pst_hmac_vap,
                                                hmac_user_stru *pst_hmac_user,
                                                oal_uint8 uc_tidno);

extern oal_void hmac_vap_net_stopall_etc(oal_void);
extern oal_void hmac_vap_net_startall_etc(oal_void);

#ifdef _PRE_WLAN_FEATURE_OFFLOAD_FLOWCTL
extern oal_bool_enum_uint8 hmac_flowctl_check_device_is_sta_mode_etc(oal_void);
extern oal_void hmac_vap_net_start_subqueue_etc(oal_uint16 us_queue_idx);
extern oal_void hmac_vap_net_stop_subqueue_etc(oal_uint16 us_queue_idx);
#endif
extern oal_void hmac_handle_disconnect_rsp_etc(hmac_vap_stru *pst_hmac_vap, hmac_user_stru *pst_hmac_user,
                                               mac_reason_code_enum_uint16 en_disasoc_reason);
extern oal_uint8 *hmac_vap_get_pmksa_etc(hmac_vap_stru *pst_hmac_vap, oal_uint8 *puc_bssid);
oal_uint32 hmac_tx_get_mac_vap_etc(oal_uint8 uc_vap_id, mac_vap_stru **pst_vap_stru);
extern oal_uint32 hmac_restart_all_work_vap(oal_uint8 uc_chip_id);
#ifdef _PRE_WLAN_FEATURE_AMPDU_TX_HW
extern oal_void hmac_set_ampdu_worker(oal_delayed_work *pst_work);
extern oal_void hmac_set_ampdu_hw_worker(oal_delayed_work *pst_work);
#endif

oal_void hmac_vap_state_restore_etc(hmac_vap_stru *pst_hmac_vap);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif /* end of hmac_vap.h */
