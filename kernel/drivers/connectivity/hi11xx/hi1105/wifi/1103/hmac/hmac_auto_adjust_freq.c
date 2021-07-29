

#if (_PRE_MULTI_CORE_MODE_OFFLOAD_DMAC == _PRE_MULTI_CORE_MODE)
#include "oal_net.h"
#include "oal_types.h"
#include "oam_ext_if.h"
#include "mac_vap.h"
#include "mac_resource.h"
#include "hmac_vap.h"
#include "hmac_auto_adjust_freq.h"
#include "hmac_ext_if.h"
#include "hmac_blockack.h"
#include "hmac_tx_data.h"

#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
#include "plat_pm_wlan.h"
#include <linux/pm_qos.h>
#if defined(_PRE_FEATURE_PLAT_LOCK_CPUFREQ) && !defined(CONFIG_HI110X_KERNEL_MODULES_BUILD_SUPPORT)
#include <linux/hisi/hisi_cpufreq_req.h>
#include <linux/cpufreq.h>
#include <linux/hisi/hisi_core_ctl.h>
#endif
#endif

#include "securec.h"
#include "securectype.h"

#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_MAC_AUTO_ADJUST_FREQ_C

#ifdef _PRE_WLAN_FEATURE_NEGTIVE_DET
#define INVALID_PKMODE_TH (0xFFFFFFFF)
#endif
/*****************************************************************************
  2 ȫ�ֱ�������
*****************************************************************************/
#ifdef WIN32
oal_uint32 jiffies;
#endif

freq_lock_control_stru g_freq_lock_control_etc = { 0 };
wifi_txrx_pkt_stat g_st_wifi_rxtx_total = { 0 };
/* Wi-Fi�����շ�����ʶ�������� */
freq_wifi_load_stru g_st_wifi_load = { 0 };
#if defined(_PRE_FEATURE_PLAT_LOCK_CPUFREQ) && !defined(CONFIG_HI110X_KERNEL_MODULES_BUILD_SUPPORT)
extern struct cpufreq_req g_ast_cpufreq_etc[OAL_BUS_MAXCPU_NUM];
extern hisi_max_cpu_freq g_aul_cpumaxfreq_etc[OAL_BUS_MAXCPU_NUM];
extern struct pm_qos_request g_st_pmqos_requset_etc;
#endif
#ifdef _PRE_WLAN_FEATURE_NEGTIVE_DET
oal_bool_enum_uint8 g_en_pk_mode_swtich = OAL_TRUE;

/* pk mode���ж������ޣ����ֽ�Ϊ��λ */
/*
PKģʽ���޻���:
{(��λMbps)  20M     40M    80M   160M   80+80M
lagency:    {valid, valid, valid, valid, valid},   (����Э��ģʽû��pk mode )
HT:         {62, 123, valid, valid, valid},
VHT:        {77, 167, 329, 660, 660},
HE:         {valid, valid, valid, valid, valid},   (�ݲ�֧��11ax��pk mode)
};


PKģʽ��������:
�ߵ�λ����: g_st_pk_mode_high_th_table = PKģʽ���޻��� * 70% *1024 *1024 /8  (��λ�ֽ�)
�͵�λ����: g_st_pk_mode_low_th_table  = PKģʽ���޻��� * 30% *1024 *1024 /8  (��λ�ֽ�)

 */
#if 1
oal_uint32 g_st_pk_mode_high_th_table[WLAN_PROTOCOL_CAP_BUTT][WLAN_BW_CAP_BUTT] = {
    { INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH },
    { 45,                110,               INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH },
    { 60,                130,               280,               562,               562 },
#ifdef _PRE_WLAN_FEATURE_11AX
    { INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH },
#endif /* #ifdef _PRE_WLAN_FEATURE_11AX */

};

oal_uint32 g_st_pk_mode_low_th_table[WLAN_PROTOCOL_CAP_BUTT][WLAN_BW_CAP_BUTT] = {
    { INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH },
    { 30,                80,                INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH },
    { 40,                100,               240,               400,               400 },
#ifdef _PRE_WLAN_FEATURE_11AX
    { INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH, INVALID_PKMODE_TH },
#endif /* #ifdef _PRE_WLAN_FEATURE_11AX */

};
#endif

#endif

#ifdef _PRE_WLAN_FEATURE_AUTO_FREQ
oal_uint32 g_ul_orig_cpu_min_freq = 0;
oal_uint32 g_ul_orig_cpu_max_freq = 0;
oal_uint32 g_ul_orig_ddr_min_freq = 0;
oal_uint32 g_ul_orig_ddr_max_freq = 0;

#ifndef WIN32
/* �ɶ��ƻ����г�ʼ�� */
host_speed_freq_level_stru g_host_speed_freq_level_etc[4] = {
    /* pps����                   CPU��Ƶ����                     DDR��Ƶ���� */
    { PPS_VALUE_0, CPU_MIN_FREQ_VALUE_0, DDR_MIN_FREQ_VALUE_0 },
    { PPS_VALUE_1, CPU_MIN_FREQ_VALUE_1, DDR_MIN_FREQ_VALUE_1 },
    { PPS_VALUE_2, CPU_MIN_FREQ_VALUE_2, DDR_MIN_FREQ_VALUE_2 },
    { PPS_VALUE_3, CPU_MIN_FREQ_VALUE_3, DDR_MIN_FREQ_VALUE_3 },
};
host_speed_freq_level_stru g_host_no_ba_freq_level_etc[4] = {
    /* pps����                        CPU��Ƶ����                      DDR��Ƶ���� */
    { NO_BA_PPS_VALUE_0, CPU_MIN_FREQ_VALUE_0, DDR_MIN_FREQ_VALUE_0 },
    { NO_BA_PPS_VALUE_1, CPU_MIN_FREQ_VALUE_1, DDR_MIN_FREQ_VALUE_1 },
    { NO_BA_PPS_VALUE_2, CPU_MIN_FREQ_VALUE_2, DDR_MIN_FREQ_VALUE_2 },
    { NO_BA_PPS_VALUE_3, CPU_MIN_FREQ_VALUE_2, DDR_MIN_FREQ_VALUE_2 },
};
device_speed_freq_level_stru g_device_speed_freq_level_etc[] = {
    /* device��Ƶ���� */
    { FREQ_IDLE },
    { FREQ_MIDIUM },
    { FREQ_HIGHER },
    { FREQ_HIGHEST },
};

struct pm_qos_request *g_pst_wifi_auto_ddr_etc = NULL;

#else
host_speed_freq_level_stru g_host_speed_freq_level_etc[] = {
    /* pps����                   CPU��Ƶ����                     DDR��Ƶ���� */
    { PPS_VALUE_0, CPU_MIN_FREQ_VALUE_0, DDR_MIN_FREQ_VALUE_0 },
    { PPS_VALUE_1, CPU_MIN_FREQ_VALUE_1, DDR_MIN_FREQ_VALUE_1 },
    { PPS_VALUE_2, CPU_MIN_FREQ_VALUE_2, DDR_MIN_FREQ_VALUE_2 },
    { PPS_VALUE_3, CPU_MIN_FREQ_VALUE_3, DDR_MIN_FREQ_VALUE_3 },
};
device_speed_freq_level_stru g_device_speed_freq_level_etc[] = {
    /* device��Ƶ���� */
    { FREQ_IDLE },
    { FREQ_MIDIUM },
    { FREQ_HIGHEST },
    { FREQ_HIGHEST },
};

#define mutex_init(mux)
#define mutex_lock(mux)
#define mutex_unlock(mux)
#define spin_lock_init(mux)
#define mutex_destroy(mux)
#define spin_unlock_bh(mux)
#endif

extern hmac_rxdata_thread_stru g_st_rxdata_thread_etc;

#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)

OAL_STATIC oal_bool_enum_uint8 hmac_get_cpu_freq_raw(oal_uint8 uc_freq_type, oal_uint32 *pst_ul_freq_value)
{
    return 0;
}


oal_bool_enum_uint8 hmac_set_cpu_freq_raw_etc(oal_uint8 uc_freq_type, oal_uint32 ul_freq_value)
{
    return 0;
}


oal_bool_enum_uint8 hmac_set_ddr_freq_raw_etc(oal_uint8 uc_freq_type, oal_uint32 ul_freq_value)
{
    return 0;
}

oal_void hmac_wifi_auto_ddr_init_etc(oal_void)
{
    return;
}
oal_void hmac_wifi_auto_ddr_exit_etc(oal_void)
{
}
#endif


oal_uint8 hmac_set_auto_freq_mod_etc(oal_uint8 uc_freq_enable)
{
    g_freq_lock_control_etc.uc_lock_mod = uc_freq_enable;

    /* ����device�Ƿ�ʹ�� */
    if (uc_freq_enable == FREQ_LOCK_ENABLE) {
        /* ��ʼƵ��Ϊidle */
        g_freq_lock_control_etc.uc_curr_lock_level = FREQ_IDLE;
        /* ����Ƶ��Ϊidle */
        g_freq_lock_control_etc.uc_req_lock_level = FREQ_IDLE;
        /* WIFI��ʱ,��ʱ���������� */
        g_freq_lock_control_etc.ul_pre_jiffies = jiffies;
    } else {
        /* �ر�WIFIʱ,��CPUƵ�ʵ���ΪĬ��ֵ */
        mutex_lock(&g_freq_lock_control_etc.st_lock_freq_mtx);
        hmac_set_cpu_freq_raw_etc(SCALING_MIN_FREQ, g_ul_orig_cpu_min_freq);
        hmac_set_cpu_freq_raw_etc(SCALING_MAX_FREQ, g_ul_orig_cpu_max_freq);
        mutex_unlock(&g_freq_lock_control_etc.st_lock_freq_mtx);

        g_freq_lock_control_etc.uc_curr_lock_level = FREQ_BUTT;
        OAM_WARNING_LOG2(0, OAM_SF_ANY, "{hmac_set_auto_freq_mod_etc: freq,min[%d]max[%d]!}",
                         g_ul_orig_cpu_min_freq, g_ul_orig_cpu_max_freq);
    }

    return OAL_SUCC;
}

oal_void hmac_adjust_set_freq(oal_void)
{
    oal_uint8 uc_req_lock_level = g_freq_lock_control_etc.uc_req_lock_level;

    /* ��ͬ����Ҫ����ϵͳ�ļ� */
    if (uc_req_lock_level == g_freq_lock_control_etc.uc_curr_lock_level) {
        return;
    }

    OAM_WARNING_LOG2(0, OAM_SF_PWR, "{hmac_adjust_set_freq: freq [%d]to[%d]}",
                     g_freq_lock_control_etc.uc_curr_lock_level, uc_req_lock_level);

#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
    if (hmac_is_device_ba_setup_etc()) {
        hmac_set_cpu_freq_raw_etc(SCALING_MIN_FREQ, g_host_speed_freq_level_etc[uc_req_lock_level].ul_min_cpu_freq);
        hmac_set_ddr_freq_raw_etc(SCALING_MIN_FREQ, g_host_speed_freq_level_etc[uc_req_lock_level].ul_min_ddr_freq);
    } else {
        hmac_set_cpu_freq_raw_etc(SCALING_MIN_FREQ, g_host_no_ba_freq_level_etc[uc_req_lock_level].ul_min_cpu_freq);
        hmac_set_ddr_freq_raw_etc(SCALING_MIN_FREQ, g_host_no_ba_freq_level_etc[uc_req_lock_level].ul_min_ddr_freq);
    }
#endif

    g_freq_lock_control_etc.uc_curr_lock_level = uc_req_lock_level;
}

oal_uint8 hmac_get_freq_level_etc(oal_uint32 ul_speed)  // ���������ʼ���Ŀ����Ƶ�ȼ�
{
    oal_uint8 level_idx;

    if (hmac_is_device_ba_setup_etc()) {
        if (ul_speed <= g_host_speed_freq_level_etc[1].ul_speed_level) {
            level_idx = 0;
        } else if ((ul_speed > g_host_speed_freq_level_etc[1].ul_speed_level)
                   && (ul_speed <= g_host_speed_freq_level_etc[2].ul_speed_level)) {
            level_idx = 1;
        } else if ((ul_speed > g_host_speed_freq_level_etc[2].ul_speed_level)
                   && (ul_speed <= g_host_speed_freq_level_etc[3].ul_speed_level)) {
            level_idx = 2;
        } else {
            level_idx = 3;
        }
    } else {
        if (ul_speed <= g_host_no_ba_freq_level_etc[1].ul_speed_level) {
            level_idx = 0;
        } else if ((ul_speed > g_host_no_ba_freq_level_etc[1].ul_speed_level)
                   && (ul_speed <= g_host_no_ba_freq_level_etc[2].ul_speed_level)) {
            level_idx = 1;
        } else if ((ul_speed > g_host_no_ba_freq_level_etc[2].ul_speed_level)
                   && (ul_speed <= g_host_no_ba_freq_level_etc[3].ul_speed_level)) {
            level_idx = 2;
        } else {
            level_idx = 3;
        }
    }
    return level_idx;
}


OAL_STATIC oal_void hmac_pps_calc_etc(oal_void)
{
    oal_uint32 ul_cur_jiffies;
    oal_uint32 ul_sdio_dur_ms;  // ʱ��
    oal_uint32 ul_trx_total;
    oal_uint32 ul_tx_total;
    oal_uint32 ul_rx_total;

    ul_tx_total = oal_atomic_read(&g_st_wifi_rxtx_total.ul_tx_pkts);
    ul_rx_total = oal_atomic_read(&g_st_wifi_rxtx_total.ul_rx_pkts);
    ul_trx_total = ul_tx_total + ul_rx_total;
    oal_atomic_set(&g_st_wifi_rxtx_total.ul_rx_pkts, 0);
    oal_atomic_set(&g_st_wifi_rxtx_total.ul_tx_pkts, 0);

    ul_cur_jiffies = jiffies;  // jiffies��Linux�ں��е�һ��ȫ�ֱ�����������¼��ϵͳ�������������Ľ��ĵ�����
    ul_sdio_dur_ms = OAL_JIFFIES_TO_MSECS(ul_cur_jiffies - g_freq_lock_control_etc.ul_pre_jiffies);
    g_freq_lock_control_etc.ul_pre_jiffies = ul_cur_jiffies;

    /* �����ǰͳ��ʱ�䲻�㶨ʱ�����ڵ�һ��,�ᵼ��ͳ��PPSֵƫ�󷵻� */
    if ((ul_sdio_dur_ms == 0) || (ul_sdio_dur_ms < (WLAN_FREQ_TIMER_PERIOD >> 1))) {
        return;
    }

    // ����PPS
    g_freq_lock_control_etc.ul_total_sdio_pps = (ul_trx_total * 1000) / ul_sdio_dur_ms;
    g_freq_lock_control_etc.ul_tx_pps = (ul_tx_total * 1000) / ul_sdio_dur_ms;
    g_freq_lock_control_etc.ul_rx_pps = (ul_rx_total * 1000) / ul_sdio_dur_ms;
}


oal_void hmac_adjust_freq_etc(oal_void)
{
    oal_uint8 uc_req_lock_level;  // Ŀ����Ƶ�ȼ�

    hmac_pps_calc_etc();

    /* host��Ƶ�Ƿ�ʹ�� */
    if (g_freq_lock_control_etc.uc_lock_mod == FREQ_LOCK_DISABLE) {
        return;
    }

    g_freq_lock_control_etc.uc_req_lock_level = hmac_get_freq_level_etc(g_freq_lock_control_etc.ul_total_sdio_pps);

    uc_req_lock_level = g_freq_lock_control_etc.uc_req_lock_level;
    if (uc_req_lock_level == g_freq_lock_control_etc.uc_curr_lock_level) { // Ŀ����Ƶ�͵�ǰ��Ƶ��ͬ������Ҫ��Ƶ
        g_freq_lock_control_etc.ul_adjust_count = 0;
        return;
    }

    mutex_lock(&g_freq_lock_control_etc.st_lock_freq_mtx);

    if (uc_req_lock_level < g_freq_lock_control_etc.uc_curr_lock_level) {
        /* ����MAX_DEGRADE_FREQ_TIME_THRESHOLD��Ž�Ƶ����֤���� */
        g_freq_lock_control_etc.ul_adjust_count++;

        if (g_freq_lock_control_etc.ul_total_sdio_pps) { // ��ǰ����������Ϊ��
            // �а�ʱ����100�����ڶ���Ҫ��Ƶ�Ž�Ƶ
            if (g_freq_lock_control_etc.ul_adjust_count >= MAX_DEGRADE_FREQ_COUNT_THRESHOLD_SUCCESSIVE_10) {
                g_freq_lock_control_etc.ul_adjust_count = 0;
                /* �����������Ƶֵ�ĵ��� */
                hmac_adjust_set_freq();
            }
        } else { // û�а�ʱ����3�����ڶ���Ҫ��Ƶ�ٽ�Ƶ
            if (g_freq_lock_control_etc.ul_adjust_count >= MAX_DEGRADE_FREQ_COUNT_THRESHOLD_SUCCESSIVE_3) {
                g_freq_lock_control_etc.ul_adjust_count = 0;
                hmac_adjust_set_freq();
            }
        }
    } else {
        /* ��Ƶ���ȴ�������ִ�б�֤���� */
        g_freq_lock_control_etc.ul_adjust_count = 0;
        hmac_adjust_set_freq();
    }

    mutex_unlock(&g_freq_lock_control_etc.st_lock_freq_mtx);
}


oal_void hmac_wifi_auto_freq_ctrl_init_etc(void)
{
#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
    if (g_freq_lock_control_etc.en_is_inited != OAL_TRUE) {
        mutex_init(&g_freq_lock_control_etc.st_lock_freq_mtx);
        mutex_lock(&g_freq_lock_control_etc.st_lock_freq_mtx);

        hmac_get_cpu_freq_raw(SCALING_MIN_FREQ, &g_ul_orig_cpu_min_freq);
        hmac_get_cpu_freq_raw(SCALING_MAX_FREQ, &g_ul_orig_cpu_max_freq);

        hmac_wifi_auto_ddr_init_etc();

        OAM_WARNING_LOG4(0, OAM_SF_ANY,
            "{hmac_wifi_auto_freq_ctrl_init_etc: orig_freq:: cpu_min[%d],cpu_max[%d],ddr_min[%d],ddr_max[%d]}",
            g_ul_orig_cpu_min_freq, g_ul_orig_cpu_max_freq, g_ul_orig_ddr_min_freq, g_ul_orig_ddr_max_freq);

        g_freq_lock_control_etc.en_is_inited = OAL_TRUE;

        mutex_unlock(&g_freq_lock_control_etc.st_lock_freq_mtx);
    }
#endif
}

oal_void hmac_wifi_auto_freq_ctrl_deinit_etc(void)
{
#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
    mutex_lock(&g_freq_lock_control_etc.st_lock_freq_mtx);

    if (g_freq_lock_control_etc.en_is_inited == OAL_TRUE) {
        hmac_wifi_auto_ddr_exit_etc();
    } else {
        OAM_WARNING_LOG0(0, OAM_SF_ANY, "{hw_wifi_freq_ctrl_destroy freq lock has already been released!}");
    }

    mutex_unlock(&g_freq_lock_control_etc.st_lock_freq_mtx);
    mutex_destroy(&g_freq_lock_control_etc.st_lock_freq_mtx);

    g_freq_lock_control_etc.en_is_inited = OAL_FALSE;
#endif
}

oal_void hmac_auto_freq_wifi_rx_stat(oal_uint32 ul_pkt_count)
{
    oal_atomic_add(&g_st_wifi_rxtx_total.ul_rx_pkts, ul_pkt_count);
}

oal_void hmac_auto_freq_wifi_tx_stat(oal_uint32 ul_pkt_count)
{
    oal_atomic_add(&g_st_wifi_rxtx_total.ul_tx_pkts, ul_pkt_count);
}
#endif

oal_void hmac_auto_freq_wifi_rx_bytes_stat(oal_uint32 ul_pkt_bytes)
{
    oal_atomic_add(&g_st_wifi_rxtx_total.ul_rx_bytes, ul_pkt_bytes);
}

oal_void hmac_auto_freq_wifi_tx_bytes_stat(oal_uint32 ul_pkt_bytes)
{
    oal_atomic_add(&g_st_wifi_rxtx_total.ul_tx_bytes, ul_pkt_bytes);
}

oal_bool_enum_uint8 hmac_wifi_rx_is_busy(oal_void)
{
    return g_st_wifi_load.en_wifi_rx_busy;
}

#if defined(_PRE_FEATURE_PLAT_LOCK_CPUFREQ) && !defined(CONFIG_HI110X_KERNEL_MODULES_BUILD_SUPPORT)
OAL_STATIC OAL_INLINE oal_void hmac_lock_max_cpu_freq(oal_void)
{
    oal_uint8 uc_cpuid_loop;
    /* ���к˶��������Ƶ�� */
    for (uc_cpuid_loop = 0; uc_cpuid_loop < OAL_BUS_MAXCPU_NUM; uc_cpuid_loop++) {
        /* δ��ȡ����ȷ��cpuƵ�������� */
        if (g_aul_cpumaxfreq_etc[uc_cpuid_loop].valid != OAL_TRUE) {
            continue;
        }

        hisi_cpufreq_update_req(&g_ast_cpufreq_etc[uc_cpuid_loop], g_aul_cpumaxfreq_etc[uc_cpuid_loop].max_cpu_freq);
    }
}

OAL_STATIC OAL_INLINE oal_void hmac_unlock_max_cpu_freq(oal_void)
{
    oal_uint8 uc_cpuid_loop;
    for (uc_cpuid_loop = 0; uc_cpuid_loop < OAL_BUS_MAXCPU_NUM; uc_cpuid_loop++) {
        /* δ��ȡ����ȷ��cpuƵ�������� */
        if (g_aul_cpumaxfreq_etc[uc_cpuid_loop].valid != OAL_TRUE) {
            continue;
        }

        hisi_cpufreq_update_req(&g_ast_cpufreq_etc[uc_cpuid_loop], 0);
    }
}
#endif

#ifdef _PRE_PRODUCT_HI1620S_KUNPENG
OAL_STATIC oal_void hmac_set_cpu_freq(oal_uint8 uc_req_freq_state)
{
    if (uc_req_freq_state == g_freq_lock_control_etc.uc_cur_cpu_freq_state) {
        return;
    }

    g_freq_lock_control_etc.uc_cur_cpu_freq_state = uc_req_freq_state;
}

OAL_STATIC oal_void hmac_adjust_set_irq(oal_uint8 uc_cpu_id)
{
    if (uc_cpu_id == g_freq_lock_control_etc.uc_cur_irq_cpu) {
        return;
    }

    g_freq_lock_control_etc.uc_cur_irq_cpu = uc_cpu_id;

    return;
}
#else


OAL_STATIC oal_void hmac_set_cpu_freq(oal_uint8 uc_req_freq_state)
{
    if (uc_req_freq_state == g_freq_lock_control_etc.uc_cur_cpu_freq_state) {
        return;
    }

    g_freq_lock_control_etc.uc_cur_cpu_freq_state = uc_req_freq_state;

#if defined(_PRE_FEATURE_PLAT_LOCK_CPUFREQ) && !defined(CONFIG_HI110X_KERNEL_MODULES_BUILD_SUPPORT)
#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
    if (g_freq_lock_control_etc.uc_lock_max_cpu_freq == OAL_TRUE) {
        if (uc_req_freq_state == WLAN_CPU_FREQ_SUPER) {
            /* ���к˶��������Ƶ�� */
            hmac_lock_max_cpu_freq();
        } else {
            hmac_unlock_max_cpu_freq();
        }
    }

    if (g_freq_lock_control_etc.uc_lock_dma_latency == OAL_TRUE) {
        if (uc_req_freq_state == WLAN_CPU_FREQ_SUPER) {
            /* �޸�DMA latency,����cpu��������idle state */
            pm_qos_update_request(&g_st_pmqos_requset_etc, g_freq_lock_control_etc.dma_latency_value);
        } else {
            pm_qos_update_request(&g_st_pmqos_requset_etc, PM_QOS_DEFAULT_VALUE);
        }
    }
#endif
#endif
}


OAL_STATIC oal_void hmac_adjust_set_irq(oal_uint8 uc_cpu_id)
{
    if (uc_cpu_id == g_freq_lock_control_etc.uc_cur_irq_cpu) {
        return;
    }

    g_freq_lock_control_etc.uc_cur_irq_cpu = uc_cpu_id;

    /* ���ƻ��ر� */
    if (g_freq_lock_control_etc.en_irq_affinity == OAL_FALSE) {
        return;
    }

    hi110x_hcc_dev_bindcpu(uc_cpu_id >= WLAN_IRQ_AFFINITY_BUSY_CPU ? 1 : 0);

#if defined(CONFIG_ARCH_HISI)
#ifdef CONFIG_NR_CPUS
#if CONFIG_NR_CPUS > OAL_BUS_HPCPU_NUM
    if (hmac_get_feature_switch(HMAC_CORE_BIND_SWITCH)) {
        if (g_st_rxdata_thread_etc.pst_rxdata_thread) {
            struct cpumask fast_cpus;
            if (uc_cpu_id >= WLAN_IRQ_AFFINITY_BUSY_CPU) {
                hisi_get_fast_cpus(&fast_cpus);
                cpumask_clear_cpu(OAL_BUS_HPCPU_NUM, &fast_cpus);
                set_cpus_allowed_ptr(g_st_rxdata_thread_etc.pst_rxdata_thread, &fast_cpus);
            } else {
                set_cpus_allowed_ptr(g_st_rxdata_thread_etc.pst_rxdata_thread, cpumask_of(1));
            }
            g_st_rxdata_thread_etc.uc_allowed_cpus = uc_cpu_id;
        }
    }
#endif
#endif
#endif
}
#endif

#ifdef _PRE_WLAN_FEATURE_NEGTIVE_DET

oal_void hmac_update_pk_mode_info(oal_uint8 *puc_vap_id)
{
    oal_uint8 uc_vap_id;
    mac_vap_stru *pst_mac_vap;
    mac_user_stru *pst_mac_user;
    oal_dlist_head_stru *pst_entry;
    oal_dlist_head_stru *pst_dlist_tmp;

    g_st_wifi_pk_mode_status.en_curr_bw_cap = WLAN_BW_CAP_BUTT;
    g_st_wifi_pk_mode_status.en_curr_protocol_cap = WLAN_PROTOCOL_CAP_BUTT;
    g_st_wifi_pk_mode_status.en_curr_num_spatial_stream = WLAN_SINGLE_NSS;

    for (uc_vap_id = 0; uc_vap_id < WLAN_VAP_SUPPORT_MAX_NUM_LIMIT; uc_vap_id++) {
        pst_mac_vap = (mac_vap_stru *)mac_res_get_mac_vap(uc_vap_id);
        if (pst_mac_vap == OAL_PTR_NULL) {
            OAM_ERROR_LOG1(0, OAM_SF_ANY, "{hmac_update_pk_mode_info::pst_mac_vap(%d) is null.}", uc_vap_id);
            continue;
        }

        if (!IS_LEGACY_STA(pst_mac_vap)) {
            continue;
        }

        OAL_DLIST_SEARCH_FOR_EACH_SAFE(pst_entry, pst_dlist_tmp, &(pst_mac_vap->st_mac_user_list_head))
        {
            pst_mac_user = OAL_DLIST_GET_ENTRY(pst_entry, mac_user_stru, st_user_dlist);
            if (pst_mac_user == OAL_PTR_NULL) {
                OAM_ERROR_LOG0(pst_mac_vap->uc_vap_id, OAM_SF_CFG, "{hmac_update_pk_mode_info::pst_user_tmp null.}");
                continue;
            }

            /* ���û�����Ϣ�и���Ŀǰ������Э��ģʽ */
            /* TBD �����Ŀǰ������֧��һ��LEGACY_STA��LEGACY_STA��һ��ҵ���û�����������޸Ĵ˺�����Ҫ�ع� */
            g_st_wifi_pk_mode_status.en_curr_bw_cap = pst_mac_user->en_bandwidth_cap;
            g_st_wifi_pk_mode_status.en_curr_num_spatial_stream = pst_mac_user->en_avail_num_spatial_stream;

            switch (pst_mac_user->en_protocol_mode) {
                case WLAN_LEGACY_11A_MODE:
                case WLAN_LEGACY_11B_MODE:
                case WLAN_LEGACY_11G_MODE:
                case WLAN_MIXED_ONE_11G_MODE:
                case WLAN_MIXED_TWO_11G_MODE:
                    g_st_wifi_pk_mode_status.en_curr_protocol_cap = WLAN_PROTOCOL_CAP_LEGACY;
                    break;
                case WLAN_HT_MODE:
                case WLAN_HT_ONLY_MODE:
                case WLAN_HT_11G_MODE:
                    g_st_wifi_pk_mode_status.en_curr_protocol_cap = WLAN_PROTOCOL_CAP_HT;
                    break;
                case WLAN_VHT_MODE:
                case WLAN_VHT_ONLY_MODE:
                    g_st_wifi_pk_mode_status.en_curr_protocol_cap = WLAN_PROTOCOL_CAP_VHT;
                    break;
#ifdef _PRE_WLAN_FEATURE_11AX
                case WLAN_HE_MODE:
                    g_st_wifi_pk_mode_status.en_curr_protocol_cap = WLAN_PROTOCOL_CAP_HE;
                    break;
#endif
                case WLAN_PROTOCOL_BUTT:
                default:
                    g_st_wifi_pk_mode_status.en_curr_protocol_cap = WLAN_PROTOCOL_CAP_BUTT;
                    break;
            }

            *puc_vap_id = uc_vap_id;
        }
    }

    return;
}


oal_void hmac_update_pk_mode(oal_uint32 ul_tx_throughput,
                             oal_uint32 ul_rx_throughput,
                             oal_uint32 ul_dur_time)
{
    oal_bool_enum_uint8 en_curr_pk_mode = OAL_FALSE;
    oal_uint8 uc_vap_id;
    mac_cfg_pk_mode_stru st_pk_mode_info;
    mac_vap_stru *pst_mac_vap = OAL_PTR_NULL;
    oal_uint32 ul_ret;
    oal_uint32 ul_txrx_throughput;
    oal_uint32 ul_pk_mode_th;
    mac_device_stru *pst_mac_device;
    wlan_protocol_cap_enum_uint8 en_cur_protocol_cap;

    if (g_en_pk_mode_swtich == OAL_FALSE) {
        return;
    }

    pst_mac_device = mac_res_get_dev_etc(0);
    /* ����ǵ�VAP,�򲻿���Ӳ���ۺ� */
    if (1 != mac_device_calc_up_vap_num_etc(pst_mac_device)) {
        return;
    }

    /* ������ص���Ϣ */
    hmac_update_pk_mode_info(&uc_vap_id);

    /* ����Чģʽ�����������¼��� */
    if ((g_st_wifi_pk_mode_status.en_curr_bw_cap == WLAN_BW_CAP_BUTT) ||
        (g_st_wifi_pk_mode_status.en_curr_protocol_cap == WLAN_PROTOCOL_CAP_BUTT)) {
        return;
    }

    /* ����������: rxtx����֮�� / ͳ�Ƶ�ʱ����(��λs) */
    ul_txrx_throughput = ul_tx_throughput + ul_rx_throughput;

    en_cur_protocol_cap = g_st_wifi_pk_mode_status.en_curr_protocol_cap;

    /* ��������ֵ�ж�pk mode״̬ */
    if (g_st_wifi_pk_mode_status.en_is_pk_mode == OAL_TRUE) {
        ul_pk_mode_th = (g_st_pk_mode_low_th_table[en_cur_protocol_cap][g_st_wifi_pk_mode_status.en_curr_bw_cap]) *
            (1 + g_st_wifi_pk_mode_status.en_curr_num_spatial_stream);
        en_curr_pk_mode = (ul_txrx_throughput < ul_pk_mode_th) ? OAL_FALSE : OAL_TRUE;
    } else {
        ul_pk_mode_th = (g_st_pk_mode_high_th_table[en_cur_protocol_cap][g_st_wifi_pk_mode_status.en_curr_bw_cap]) *
            (1 + g_st_wifi_pk_mode_status.en_curr_num_spatial_stream);
        en_curr_pk_mode = (ul_txrx_throughput > ul_pk_mode_th) ? OAL_TRUE : OAL_FALSE;
    }
    /* dmac�߼�Ų������ֻ������ʱ�Ž���pk mode */
    if ((en_curr_pk_mode == OAL_TRUE) && (ul_rx_throughput < ul_tx_throughput)) {
        en_curr_pk_mode = OAL_FALSE;
    }
    /* PK mode״̬δ�仯����������ֵ */
    if (en_curr_pk_mode == g_st_wifi_pk_mode_status.en_is_pk_mode) {
        return;
    }

    pst_mac_vap = (mac_vap_stru *)mac_res_get_mac_vap(uc_vap_id);
    if (pst_mac_vap == OAL_PTR_NULL) {
        OAM_ERROR_LOG1(0, OAM_SF_ANY, "{hmac_update_pk_mode::pst_mac_vap(%d) is null.}", uc_vap_id);
        return;
    }
    if ((pst_mac_vap->en_protocol != WLAN_VHT_MODE) &&
        (pst_mac_vap->en_protocol != WLAN_VHT_ONLY_MODE)
#ifdef _PRE_WLAN_FEATURE_11AX
        && (pst_mac_vap->en_protocol != WLAN_HE_MODE)
#endif
) {
        return;
    }
    OAM_WARNING_LOG4(0, OAM_SF_ANY,
                     "{hmac_update_pk_mode: pk_mode changed (%d) -> (%d),tx_throughput[%d],rx_throughput[%d]!}",
                     g_st_wifi_pk_mode_status.en_is_pk_mode, en_curr_pk_mode, ul_tx_throughput, ul_rx_throughput);
    g_st_wifi_pk_mode_status.en_is_pk_mode = en_curr_pk_mode;
    g_st_wifi_pk_mode_status.ul_rx_bytes = ul_rx_throughput;
    g_st_wifi_pk_mode_status.ul_tx_bytes = ul_tx_throughput;
    g_st_wifi_pk_mode_status.ul_dur_time = ul_dur_time;

    /***************************************************************************
        ���¼���DMAC��, ͬ��VAP����״̬��DMAC
    ***************************************************************************/
    st_pk_mode_info.ul_rx_bytes = g_st_wifi_pk_mode_status.ul_rx_bytes;
    st_pk_mode_info.ul_tx_bytes = g_st_wifi_pk_mode_status.ul_tx_bytes;
    st_pk_mode_info.ul_dur_time = g_st_wifi_pk_mode_status.ul_dur_time;
    st_pk_mode_info.en_is_pk_mode = g_st_wifi_pk_mode_status.en_is_pk_mode;
    st_pk_mode_info.en_curr_bw_cap = g_st_wifi_pk_mode_status.en_curr_bw_cap;
    st_pk_mode_info.en_curr_protocol_cap = g_st_wifi_pk_mode_status.en_curr_protocol_cap;
    st_pk_mode_info.en_curr_num_spatial_stream = g_st_wifi_pk_mode_status.en_curr_num_spatial_stream;

    ul_ret = hmac_config_send_event_etc(pst_mac_vap, WLAN_CFGID_SYNC_PK_MODE,
                                        OAL_SIZEOF (mac_cfg_pk_mode_stru), (oal_uint8 *)(&st_pk_mode_info));
    if (OAL_UNLIKELY(ul_ret != OAL_SUCC)) {
        OAM_WARNING_LOG1(pst_mac_vap->uc_vap_id, OAM_SF_CFG,
                         "{hmac_update_pk_mode::hmac_config_send_event_etc failed[%d].}", ul_ret);
    }

    return;
}
#endif
#if defined(_PRE_FEATURE_PLAT_LOCK_CPUFREQ) && !defined(CONFIG_HI110X_KERNEL_MODULES_BUILD_SUPPORT)
#if (_PRE_OS_VERSION_LINUX == _PRE_OS_VERSION)
OAL_STATIC OAL_INLINE oal_void hmac_lock_cpu_freq_high_throughput_proc(oal_void)
{
    if (g_freq_lock_control_etc.uc_lock_max_cpu_freq == OAL_TRUE) {
        /* ��ǰ��������Ƶ��Ƶ�ʻ�����������Һ���Ҳ�޷��������Ƶ�ʣ���Ҫ������λ��������Ҫ������Ƶʱ�䡣
         * �Լ�ȷ���Ƿ�ÿ����Ҫ����req�� */
        core_ctl_set_boost(20 * WLAN_FREQ_TIMER_PERIOD * WLAN_THROUGHPUT_STA_PERIOD);
        hmac_lock_max_cpu_freq();
    }
}
#endif
#endif


OAL_STATIC void hmac_calc_throughput_cpu_freq(
    oal_uint32 ul_tx_throughput_mbps, oal_uint32 ul_rx_throughput_mbps)
{
    oal_uint16 us_cpu_freq_high = WLAN_CPU_FREQ_THROUGHPUT_THRESHOLD_HIGH;
    oal_uint16 us_cpu_freq_low = WLAN_CPU_FREQ_THROUGHPUT_THRESHOLD_LOW;

    /* ÿ������������ */
    if ((g_freq_lock_control_etc.us_lock_cpu_th_high) && (g_freq_lock_control_etc.us_lock_cpu_th_low)) {
        us_cpu_freq_high = g_freq_lock_control_etc.us_lock_cpu_th_high;
        us_cpu_freq_low = g_freq_lock_control_etc.us_lock_cpu_th_low;
    }

    /* ����800Mʱ�г�Ƶ������500M�ָ� */
    if ((ul_tx_throughput_mbps > us_cpu_freq_high) ||
        (ul_rx_throughput_mbps > us_cpu_freq_high)) {
        g_freq_lock_control_etc.uc_req_super_cpu_freq = WLAN_CPU_FREQ_SUPER;
    } else if ((ul_tx_throughput_mbps < us_cpu_freq_low) &&
               (ul_rx_throughput_mbps < us_cpu_freq_low)) {
        g_freq_lock_control_etc.uc_req_super_cpu_freq = WLAN_CPU_FREQ_IDLE;
    }

    hmac_set_cpu_freq(g_freq_lock_control_etc.uc_req_super_cpu_freq);
}



void hmac_perform_calc_throughput(oal_uint32 ul_tx_throughput_mbps,
                                  oal_uint32 ul_rx_throughput_mbps,
                                  oal_uint32 ul_trx_pps)
{
    oal_uint32 ul_limit_throughput_high = WLAN_IRQ_THROUGHPUT_THRESHOLD_HIGH;
    oal_uint32 ul_limit_throughput_low = WLAN_IRQ_THROUGHPUT_THRESHOLD_LOW;
    oal_uint32 ul_limit_pps_high = WLAN_IRQ_PPS_THRESHOLD_HIGH;
    oal_uint32 ul_limit_pps_low = WLAN_IRQ_PPS_THRESHOLD_LOW;

    /* ÿ������������ */
    if ((g_freq_lock_control_etc.us_throughput_irq_high) && (g_freq_lock_control_etc.us_throughput_irq_low)) {
        ul_limit_throughput_high = g_freq_lock_control_etc.us_throughput_irq_high;
        ul_limit_throughput_low = g_freq_lock_control_etc.us_throughput_irq_low;
    }

    /* ÿ��PPS���� */
    if ((g_freq_lock_control_etc.ul_irq_pps_high) && (g_freq_lock_control_etc.ul_irq_pps_low)) {
        ul_limit_pps_high = g_freq_lock_control_etc.ul_irq_pps_high;
        ul_limit_pps_low = g_freq_lock_control_etc.ul_irq_pps_low;
    }

    /* ����200Mʱ�д�ˣ�����150M��С�� */
    if ((ul_tx_throughput_mbps > ul_limit_throughput_high) ||
        (ul_rx_throughput_mbps > ul_limit_throughput_high) ||
        (ul_trx_pps >= ul_limit_pps_high)) {
        g_freq_lock_control_etc.uc_req_irq_cpu = WLAN_IRQ_AFFINITY_BUSY_CPU;
    } else if ((ul_tx_throughput_mbps < ul_limit_throughput_low) &&
               (ul_rx_throughput_mbps < ul_limit_throughput_low) &&
               (ul_trx_pps < ul_limit_pps_low)) {
        g_freq_lock_control_etc.uc_req_irq_cpu = WLAN_IRQ_AFFINITY_IDLE_CPU;
    }

    if (g_freq_lock_control_etc.uc_req_irq_cpu != g_freq_lock_control_etc.uc_cur_irq_cpu) {
        OAM_WARNING_LOG4(0, OAM_SF_ANY,
            "{hmac_perform_calc_throughput:throughput rx = %d , tx = %d, high_th = %d,low_th = %d!}",
            ul_rx_throughput_mbps, ul_tx_throughput_mbps, ul_limit_throughput_high, ul_limit_throughput_low);
        OAM_WARNING_LOG3(0, OAM_SF_ANY, "{hmac_perform_calc_throughput: rx pps = %d , tx pps = %d, trx pps = %d,!}",
            g_freq_lock_control_etc.ul_rx_pps, g_freq_lock_control_etc.ul_tx_pps, ul_trx_pps);
        OAM_WARNING_LOG4(0, OAM_SF_ANY,
            "{hmac_perform_calc_throughput: req cpu id[%d],cur cpu id[%d],pps_high = %d, pps low = %d!}",
            g_freq_lock_control_etc.uc_req_irq_cpu, g_freq_lock_control_etc.uc_cur_irq_cpu,
            ul_limit_pps_high, ul_limit_pps_low);
    }

    hmac_adjust_set_irq(g_freq_lock_control_etc.uc_req_irq_cpu);

    hmac_calc_throughput_cpu_freq(ul_tx_throughput_mbps, ul_rx_throughput_mbps);
}


void hmac_adjust_throughput(oal_void)
{
    oal_uint32 ul_tx_throughput;
    oal_uint32 ul_rx_throughput;
    oal_uint32 ul_dur_time;
    oal_uint32 ul_tx_throughput_mbps = 0;
    oal_uint32 ul_rx_throughput_mbps = 0;

    /* ѭ������ͳ�� */
    g_freq_lock_control_etc.uc_timer_cycles++;

    ul_dur_time = OAL_JIFFIES_TO_MSECS(jiffies - g_freq_lock_control_etc.ul_pre_time);

    /* С��һ������,��ͳ�� */
    if (ul_dur_time < WLAN_FREQ_TIMER_PERIOD - 10) {
        return;
    }

    /* ����һ��ʱ����ͳ��,�ų���ʱ���쳣 */
    if (ul_dur_time > ((WLAN_FREQ_TIMER_PERIOD * WLAN_THROUGHPUT_STA_PERIOD) << 2)) {
        g_freq_lock_control_etc.ul_pre_time = jiffies;
        g_freq_lock_control_etc.uc_timer_cycles = 0;

        oal_atomic_set(&g_st_wifi_rxtx_total.ul_rx_bytes, 0);
        oal_atomic_set(&g_st_wifi_rxtx_total.ul_tx_bytes, 0);

        return;
    }

    /* ���ֽ��� */
    ul_rx_throughput = oal_atomic_read(&g_st_wifi_rxtx_total.ul_rx_bytes);
    ul_tx_throughput = oal_atomic_read(&g_st_wifi_rxtx_total.ul_tx_bytes);
    if (ul_dur_time != 0) {
        ul_tx_throughput_mbps = (ul_tx_throughput >> 7) / ul_dur_time;
        ul_rx_throughput_mbps = (ul_rx_throughput >> 7) / ul_dur_time;
    }

    /* Wi-Fiҵ���ر�� */
    if (ul_rx_throughput_mbps <= WLAN_THROUGHPUT_LOAD_LOW) {
        g_st_wifi_load.en_wifi_rx_busy = OAL_FALSE;
    } else {
        g_st_wifi_load.en_wifi_rx_busy = OAL_TRUE;
    }
    if (g_st_wifi_rxtx_total.uc_trx_stat_log_en == OAL_TRUE) {
        OAM_WARNING_LOG4(0, OAM_SF_ANY,
                         "{hmac_tx_tcp_ack_buf_switch: Throught:: rx[%d],tx[%d],smooth[%d],dur_time[%d]ms!}",
                         ul_rx_throughput_mbps, ul_tx_throughput_mbps,
                         g_st_tcp_ack_buf_switch.us_tcp_ack_smooth_throughput, ul_dur_time);
    }

#ifdef _PRE_WLAN_FEATURE_MULTI_NETBUF_AMSDU
    /* AMSDU+AMPDU�ۺ��л���� */
    hmac_tx_amsdu_ampdu_switch(ul_tx_throughput_mbps);
#endif
    hmac_tx_small_amsdu_switch(ul_rx_throughput_mbps, g_freq_lock_control_etc.ul_tx_pps);

#ifdef _PRE_WLAN_FEATURE_TCP_ACK_BUFFER
    hmac_tx_tcp_ack_buf_switch(ul_rx_throughput_mbps);
#endif

#ifdef _PRE_WLAN_FEATURE_AMPDU_TX_HW
    /* �ۺ���Ӳ���л����,ÿ������̽��һ�� */
    hmac_tx_ampdu_switch(ul_tx_throughput_mbps);
#endif
#ifdef _PRE_WLAN_TCP_OPT
    hmac_tcp_ack_filter_switch(ul_rx_throughput_mbps);
#endif
    /* �������������ж��Ƿ���Ҫbypass ����LNA */
    hmac_rx_dyn_bypass_extlna_switch(ul_tx_throughput_mbps, ul_rx_throughput_mbps);

    /* �ж϶�̬���� */
    hmac_perform_calc_throughput(ul_tx_throughput_mbps, ul_rx_throughput_mbps,
                                 g_freq_lock_control_etc.ul_total_sdio_pps);

    /* irq 2s ̽��һ�� */
    if (g_freq_lock_control_etc.uc_timer_cycles < WLAN_THROUGHPUT_STA_PERIOD) {
        return;
    }
    g_freq_lock_control_etc.uc_timer_cycles = 0;

    /* 2s��������һ�� */
    oal_atomic_set(&g_st_wifi_rxtx_total.ul_rx_bytes, 0);
    oal_atomic_set(&g_st_wifi_rxtx_total.ul_tx_bytes, 0);

    g_freq_lock_control_etc.ul_pre_time = jiffies;

#ifdef _PRE_WLAN_FEATURE_NEGTIVE_DET
    /* ����pk mode */
    hmac_update_pk_mode(ul_tx_throughput_mbps, ul_rx_throughput_mbps, ul_dur_time);
#endif
}


OAL_STATIC oal_uint32 hmac_wlan_freq_wdg_timeout(oal_void *p_arg)
{
#ifdef _PRE_WLAN_FEATURE_AUTO_FREQ
    /* CPU��Ƶ */
    hmac_adjust_freq_etc();
#endif
    /* ����ͳ�� */
    hmac_adjust_throughput();

    return OAL_SUCC;
}


oal_void hmac_freq_timer_init(oal_void)
{
    /* ��������ʱ���Ѿ�ע��ɹ��������ٴα�ע�ᣬ��Ȼ�������� */
    if (g_freq_lock_control_etc.hmac_freq_timer.en_is_registerd == OAL_TRUE) {
        return;
    }

    g_freq_lock_control_etc.ul_pre_time = jiffies;

    /* uc_timer_cycles �������㡣����ֹͣ��������͹��ĺ󣬴�С���л���PK mode�ж��޷��ָ����� */
    /* ���ͳ�� */
    memset_s(&g_st_wifi_rxtx_total, OAL_SIZEOF(g_st_wifi_rxtx_total), 0, OAL_SIZEOF(g_st_wifi_rxtx_total));

    FRW_TIMER_CREATE_TIMER(&g_freq_lock_control_etc.hmac_freq_timer,  // pst_timeout
                           hmac_wlan_freq_wdg_timeout,                                   // p_timeout_func
                           WLAN_FREQ_TIMER_PERIOD,                                       // ul_timeout
                           OAL_PTR_NULL,                                                 // p_timeout_arg
                           OAL_TRUE,                                                     // en_is_periodic
                           OAM_MODULE_ID_HMAC, 0);                                       // en_module_id && ul_core_id
}


oal_void hmac_freq_timer_deinit(oal_void)
{
    /* ��ʱ��û��ע�ᣬ�Ͳ���ɾ���ˣ���Ȼ�ᷢ������ */
    if (g_freq_lock_control_etc.hmac_freq_timer.en_is_registerd == OAL_FALSE) {
        return;
    }

    hmac_adjust_set_irq(WLAN_IRQ_AFFINITY_IDLE_CPU);
    hmac_set_cpu_freq(WLAN_CPU_FREQ_IDLE);
    FRW_TIMER_IMMEDIATE_DESTROY_TIMER(&g_freq_lock_control_etc.hmac_freq_timer);
}

oal_void hmac_wifi_pm_state_notify(oal_bool_enum_uint8 en_wake_up)
{
    if (en_wake_up == OAL_TRUE) {
        /* WIFI����,����������ͳ�ƶ�ʱ�� */
        hmac_freq_timer_init();
    } else {
        /* WIFI˯��,�ر�������ͳ�ƶ�ʱ�� */
        hmac_freq_timer_deinit();
    }
}


oal_void hmac_wifi_state_notify(oal_bool_enum_uint8 en_wifi_on)
{
#ifdef _PRE_WLAN_FEATURE_AUTO_FREQ
    if (en_wifi_on == OAL_TRUE) {
        /* WIFI�ϵ�,����������ͳ�ƶ�ʱ��,�򿪵�Ƶ���� */
        hmac_freq_timer_init();
    } else {
        /* WIFI�µ�,�ر�������ͳ�ƶ�ʱ��,�رյ�Ƶ���� */
        hmac_freq_timer_deinit();
    }
#endif
}
#endif
