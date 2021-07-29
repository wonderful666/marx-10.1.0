

/* 头文件包含 */
#include "oam_register.h"
#include "securec.h"
#include "oam_ext_if.h"
#undef THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_REGISTER_C

#ifdef _PRE_WLAN_DFT_REG

/* 内部函数声明 */
/* 全局变量定义 */
oam_reg_manage_stru oam_reg_mng;

OAL_STATIC oam_reg_cfg_stru mac_reg[] = {
    { "PA_CONTROL",                     0x20018000, PG_REG },
    { "PA_STATUS",                      0x20018004, PG_REG },
    { "MAC_HW_ID",                      0x20018008, PG_REG },
    { "HT_CTRL",                        0x2001800C, PG_REG },
    { "STA_MACADDR_H",                  0x20018010, PG_REG },
    { "STA_MACADDR_L",                  0x20018014, PG_REG },
    { "AP0_MACADDR_H",                  0x20018018, PG_REG },
    { "AP0_MACADDR_L",                  0x2001801C, PG_REG },
    { "AP1_MACADDR_H",                  0x20018020, PG_REG },
    { "AP1_MACADDR_L",                  0x20018024, PG_REG },
    { "AP2_MACADDR_H",                  0x20018028, PG_REG },
    { "AP2_MACADDR_L",                  0x2001802C, PG_REG },
    { "AP3_MACADDR_H",                  0x20018030, PG_REG },
    { "AP3_MACADDR_L",                  0x20018034, PG_REG },
    { "STA_BSSID_H",                    0x20018038, PG_REG },
    { "STA_BSSID_L",                    0x2001803C, PG_REG },
    { "LONG_FRAME_THRESHOLD",           0x20018040, PG_REG },
    { "TX_PHY_THRESHOLD",               0x20018044, PG_REG },
    { "STA_PARTIAL_AID",                0x20018048, PG_REG },
    { "DURATION_ADJUST_VAL",            0x2001804C, PG_REG },
    { "BUS_MIN_ADDR",                   0x20018050, PG_REG },
    { "BUS_MAX_ADDR",                   0x20018054, PG_REG },
    { "TKIP_TX_DELAY",                  0x20018058, PG_REG },
    { "BUS_CONTROL",                    0x2001805C, PG_REG },
    { "PEER_ADDR_LUT_CONFIG",           0x20018200, PG_REG },
    { "PEER_ADDRESS_MSB",               0x20018204, PG_REG },
    { "PEER_ADDRESS_LSB",               0x20018208, PG_REG },
    { "RX_BUFFER_LENGTH",               0x2001820C, PG_REG },
    { "RX_HIBUFADDR_STATUS",            0x20018210, PG_REG },
    { "RX_HIBUFADDR_UPDATE",            0x20018214, PG_REG },
    { "RX_BUFADDR_STATUS",              0x20018218, PG_REG },
    { "RX_BUFADDR_UPDATE",              0x2001821C, PG_REG },
    { "RX_MAXLENFILT",                  0x20018220, PG_REG },
    { "RX_FRAMEFILT",                   0x20018224, PG_REG },
    { "PROTOCOL_CTRL",                  0x20018238, PG_REG },
    { "AP0_BCN_PKT_PTR",                0x20018400, PG_REG },
    { "AP0_BCN_TX_LENGTH",              0x20018404, PG_REG },
    { "AP0_BCN_PHY_TX_MODE",            0x20018408, PG_REG },
    { "AP0_BCN_TX_DATA_RATE",           0x2001840C, PG_REG },
    { "AP1_BCN_PKT_PTR",                0x20018410, PG_REG },
    { "AP1_BCN_TX_LENGTH",              0x20018414, PG_REG },
    { "AP1_BCN_PHY_TX_MODE",            0x20018418, PG_REG },
    { "AP1_BCN_TX_DATA_RATE",           0x2001841C, PG_REG },
    { "AP2_BCN_PKT_PTR",                0x20018420, PG_REG },
    { "AP2_BCN_TX_LENGTH",              0x20018424, PG_REG },
    { "AP2_BCN_PHY_TX_MODE",            0x20018428, PG_REG },
    { "AP2_BCN_TX_DATA_RATE",           0x2001842C, PG_REG },
    { "AP3_BCN_PKT_PTR",                0x20018430, PG_REG },
    { "AP3_BCN_TX_LENGTH",              0x20018434, PG_REG },
    { "AP3_BCN_PHY_TX_MODE",            0x20018438, PG_REG },
    { "AP3_BCN_TX_DATA_RATE",           0x2001843C, PG_REG },
    { "NULL_FRM_TX_PHY_MODE",           0x20018440, PG_REG },
    { "NULL_FRM_TX_DATA_RATE",          0x20018444, PG_REG },
    { "PROT_PHY_TX_MODE",               0x20018448, PG_REG },
    { "PROT_DATARATE",                  0x2001844C, PG_REG },
    { "VHT_BF_TX_PHY_MODE",             0x20018450, PG_REG },
    { "VHT_BF_TX_DATA_RATE",            0x20018454, PG_REG },
    { "RESP_PHY_TX_MODE",               0x20018458, PG_REG },
    { "RESP_DATARATE",                  0x2001845C, PG_REG },
    { "DEFAULT_ANTENNA_SET",            0x20018460, PG_REG },
    { "PHYTXPOWLVL",                    0x20018464, PG_REG },
    { "BEACON_TIMEOUT_VAL",             0x20018468, PG_REG },
    { "BEACON_MISS_MAX_NUM",            0x2001846C, PG_REG },
    { "TX_MAX_MSDU_LIFETIME",           0x20018600, PG_REG },
    { "SEQNUM_DUPDET_CTRL",             0x20018604, PG_REG },
    { "TX_SEQNUM",                      0x20018608, PG_REG },
    { "PRBS_SEEDVAL",                   0x2001860C, PG_REG },
    { "AIFSN",                          0x20018610, PG_REG },
    { "CWMINMAXACBK",                   0x20018614, PG_REG },
    { "CWMINMAXACBE",                   0x20018618, PG_REG },
    { "CWMINMAXACVI",                   0x2001861C, PG_REG },
    { "CWMINMAXACVO",                   0x20018620, PG_REG },
    { "EDCA_TXOPLIMIT_ACBKBE",          0x20018624, PG_REG },
    { "EDCA_TXOPLIMIT_ACVIVO",          0x20018628, PG_REG },
    { "AC_BK_FIRST_FRM_PTR_STATUS",     0x2001862C, PG_REG },
    { "AC_BK_FIRST_FRM_PTR_UPDATE",     0x20018630, PG_REG },
    { "AC_BE_FIRST_FRM_PTR_STATUS",     0x20018634, PG_REG },
    { "AC_BE_FIRST_FRM_PTR_UPDATE",     0x20018638, PG_REG },
    { "AC_VI_FIRST_FRM_PTR_STATUS",     0x2001863C, PG_REG },
    { "AC_VI_FIRST_FRM_PTR_UPDATE",     0x20018640, PG_REG },
    { "AC_VO_FIRST_FRM_PTR_STATUS",     0x20018644, PG_REG },
    { "AC_VO_FIRST_FRM_PTR_UPDATE",     0x20018648, PG_REG },
    { "HI_PRI_Q_FIRST_FRM_PTR_STATUS",  0x2001864C, PG_REG },
    { "HI_PRI_Q_FIRST_FRM_PTR_UPDATE",  0x20018650, PG_REG },
    { "ACBKBE_EDCA_LIFETIMELMT",        0x20018654, PG_REG },
    { "ACVIVO_EDCA_LIFETIMELMT",        0x20018658, PG_REG },
    { "EDCA_TXOPLIMIT_HI_PRI",          0x2001865C, PG_REG },
    { "TX_Q_STATUS",                    0x20018660, PG_REG },
    { "CBMAP_BA_CTRL",                  0x20018800, PG_REG },
    { "CBMAP_BA_STAADDRH",              0x20018804, PG_REG },
    { "CBMAP_BA_STAADDRL",              0x20018808, PG_REG },
    { "CBMAP_BA_PARAMS",                0x2001880C, PG_REG },
    { "CBMAP_BA_BMAPH",                 0x20018810, PG_REG },
    { "CBMAP_BA_BMAPL",                 0x20018814, PG_REG },
    { "SLOT_TIME",                      0x20018900, PG_REG },
    { "NDPA_DUR_CALC_PARAM",            0x20018908, PG_REG },
    { "NDP_GROUPID",                    0x2001890C, PG_REG },
    { "NUM_PA_CLKS_ONE_US",             0x20018A00, PG_REG },
    { "NUM_PA_CLKS_DECI_US",            0x20018A04, PG_REG },
    { "SIFS_TIME",                      0x20018A14, PG_REG },
    { "EIFS_TIME",                      0x20018A18, PG_REG },
    { "SEC_CH_SLOT",                    0x20018A1C, PG_REG },
    { "SIFS_TIME2",                     0x20018A20, PG_REG },
    { "RIFS_TIME_CTRL",                 0x20018A24, PG_REG },
    { "VHT_NDP_MAX_TIME",               0x20018A28, PG_REG },
    { "STA_TSF_CTRL",                   0x20018A2C, PG_REG },
    { "STA_TSFTIMER_RDVALH_STATUS",     0x20018A30, PG_REG },
    { "STA_TSFTIMER_RDVALL_STATUS",     0x20018A34, PG_REG },
    { "STA_TSFTIMER_RDVALH_UPDATE",     0x20018A38, PG_REG },
    { "STA_TSFTIMER_RDVALL_UPDATE",     0x20018A3C, PG_REG },
    { "STA_BCN_PERIOD",                 0x20018A40, PG_REG },
    { "STA_TBTT_TIMER_STATUS",          0x20018A44, PG_REG },
    { "STA_TBTT_TIMER_UPDATE",          0x20018A48, PG_REG },
    { "STA_DTIM_PERIOD",                0x20018A4C, PG_REG },
    { "STA_DTIM_COUNT_STATUS",          0x20018A50, PG_REG },
    { "STA_DTIM_COUNT_UPDATE",          0x20018A54, PG_REG },
    { "AP0_TSF_CTRL",                   0x20018A58, PG_REG },
    { "AP0_TSFTIMER_RDVALH_STATUS",     0x20018A5C, PG_REG },
    { "AP0_TSFTIMER_RDVALL_STATUS",     0x20018A60, PG_REG },
    { "AP0_TSFTIMER_RDVALH_UPDATE",     0x20018A64, PG_REG },
    { "AP0_TSFTIMER_RDVALL_UPDATE",     0x20018A68, PG_REG },
    { "AP0_BCN_PERIOD",                 0x20018A6C, PG_REG },
    { "AP0_DTIM_PERIOD",                0x20018A70, PG_REG },
    { "AP0_DTIM_COUNT_STATUS",          0x20018A74, PG_REG },
    { "AP0_DTIM_COUNT_UPDATE",          0x20018A78, PG_REG },
    { "AP0_TBTT_TIMER_STATUS",          0x20018A7C, PG_REG },
    { "AP0_TBTT_TIMER_UPDATE",          0x20018A80, PG_REG },
    { "AP1_TSF_CTRL",                   0x20018A84, PG_REG },
    { "AP1_TSFTIMER_RDVALH_STATUS",     0x20018A88, PG_REG },
    { "AP1_TSFTIMER_RDVALL_STATUS",     0x20018A8C, PG_REG },
    { "AP1_TSFTIMER_RDVALH_UPDATE",     0x20018A90, PG_REG },
    { "AP1_TSFTIMER_RDVALL_UPDATE",     0x20018A94, PG_REG },
    { "AP1_BCN_PERIOD",                 0x20018A98, PG_REG },
    { "AP1_DTIM_PERIOD",                0x20018A9C, PG_REG },
    { "AP1_DTIM_COUNT_STATUS",          0x20018AA0, PG_REG },
    { "AP1_DTIM_COUNT_UPDATE",          0x20018AA4, PG_REG },
    { "AP1_TBTT_TIMER_STATUS",          0x20018AA8, PG_REG },
    { "AP1_TBTT_TIMER_UPDATE",          0x20018AAC, PG_REG },
    { "AP2_TSF_CTRL",                   0x20018AB0, PG_REG },
    { "AP2_TSFTIMER_RDVALH_STATUS",     0x20018AB4, PG_REG },
    { "AP2_TSFTIMER_RDVALL_STATUS",     0x20018AB8, PG_REG },
    { "AP2_TSFTIMER_RDVALH_UPDATE",     0x20018ABC, PG_REG },
    { "AP2_TSFTIMER_RDVALL_UPDATE",     0x20018AC0, PG_REG },
    { "AP2_BCN_PERIOD",                 0x20018AC4, PG_REG },
    { "AP2_DTIM_PERIOD",                0x20018AC8, PG_REG },
    { "AP2_DTIM_COUNT_STATUS",          0x20018ACC, PG_REG },
    { "AP2_DTIM_COUNT_UPDATE",          0x20018AD0, PG_REG },
    { "AP2_TBTT_TIMER_STATUS",          0x20018AD4, PG_REG },
    { "AP2_TBTT_TIMER_UPDATE",          0x20018AD8, PG_REG },
    { "AP3_TSF_CTRL",                   0x20018ADC, PG_REG },
    { "AP3_TSFTIMER_RDVALH_STATUS",     0x20018AE0, PG_REG },
    { "AP3_TSFTIMER_RDVALL_STATUS",     0x20018AE4, PG_REG },
    { "AP3_TSFTIMER_RDVALH_UPDATE",     0x20018AE8, PG_REG },
    { "AP3_TSFTIMER_RDVALL_UPDATE",     0x20018AEC, PG_REG },
    { "AP3_BCN_PERIOD",                 0x20018AF0, PG_REG },
    { "AP3_DTIM_PERIOD",                0x20018AF4, PG_REG },
    { "AP3_DTIM_COUNT_STATUS",          0x20018AF8, PG_REG },
    { "AP3_DTIM_COUNT_UPDATE",          0x20018AFC, PG_REG },
    { "AP3_TBTT_TIMER_STATUS",          0x20018B00, PG_REG },
    { "AP3_TBTT_TIMER_UPDATE",          0x20018B04, PG_REG },
    { "PWR_MGMT_CTRL",                  0x20018B08, PG_REG },
    { "LISTEN_INTERVAL",                0x20018B0C, PG_REG },
    { "OFFSET_INTERVAL",                0x20018B10, PG_REG },
    { "LISTEN_INTERVAL_TIMER_STATUS",   0x20018B14, PG_REG },
    { "LISTEN_INTERVAL_TIMER_UPDATE",   0x20018B18, PG_REG },
    { "SMPS_CTRL",                      0x20018B1C, PG_REG },
    { "TXOP_PS_CTRL",                   0x20018B20, PG_REG },
    { "CH_STATISTIC_CONTROL",           0x20018C00, PG_REG },
    { "CH_LOAD_STAT_PERIOD",            0x20018C04, PG_REG },
    { "PRIMARY_20M_IDLE_COUNT",         0x20018C08, PG_REG },
    { "PRIMARY_40M_IDLE_COUNT",         0x20018C0C, PG_REG },
    { "PRIMARY_80M_IDLE_COUNT",         0x20018C10, PG_REG },
    { "TX_PROGRESS_COUNT",              0x20018C14, PG_REG },
    { "RX_PROGRESS_COUNT",              0x20018C18, PG_REG },
    { "TXBF_LUT_CONFIG",                0x20018C20, PG_REG },
    { "TXBF_LUT_INFO",                  0x20018C24, PG_REG },
    { "ANT_LUT_CONFIG",                 0x20018C28, PG_REG },
    { "SMART_ANTENNA_VALUE",            0x20018C2C, PG_REG },
    { "LEGACY_MATRIX_BUFFER_POINTER",   0x20018C30, PG_REG },
    { "MATRIX_TIMEOUT_VALUE",           0x20018C34, PG_REG },
    { "HT_MATRIX_BUFFER_POINTER",       0x20018C38, PG_REG },
    { "HT_MATRIX_BUFFER_STEP",          0x20018C3C, PG_REG },
    { "HT_MATRIX_BUFFER_NUM",           0x20018C40, PG_REG },
    { "INTERRUPT_STATUS",               0x20018E00, PG_REG },
    { "INTERRUPT_CLEAR",                0x20018E04, PG_REG },
    { "INTERRUPT_MASK",                 0x20018E08, PG_REG },
    { "ERR_INTR_STAT",                  0x20018E0C, PG_REG },
    { "ERR_INTR_CLEAR",                 0x20018E10, PG_REG },
    { "ERR_INTR_MASK",                  0x20018E14, PG_REG },
    { "HIRXBUFF_COUNT",                 0x20018E18, PG_REG },
    { "HIRXFRAME_PTR",                  0x20018E1C, PG_REG },
    { "RXBUFF_COUNT",                   0x20018E20, PG_REG },
    { "RXFRAME_PTR",                    0x20018E24, PG_REG },
    { "TXMPDU_COUNT",                   0x20018E28, PG_REG },
    { "TXFRAME_PTR",                    0x20018E2C, PG_REG },
    { "PHYTXPLCP_DELAY",                0x20019000, PG_REG },
    { "PHYRXPLCP_DELAY",                0x20019004, PG_REG },
    { "PHYRXTX_TURNAROUND_TIME",        0x20019008, PG_REG },
    { "PHYCCADELAY",                    0x20019014, PG_REG },
    { "PHYTXPLCP_ADJUST",               0x20019018, PG_REG },
    { "VHT_TXPLCP_ADJUST_VAL",          0x2001901C, PG_REG },
    { "PHYRXPLCP_DELAY2",               0x20019020, PG_REG },
    { "PHYRXSTARTDELAY",                0x20019024, PG_REG },
    { "VHT_RX_START_DELAY",             0x20019028, PG_REG },
    { "PROXY_STA_EN",                   0x2001902C, PG_REG },
    { "RIFS_RST_WAIT_CLK_NUM",          0x20019030, PG_REG },
    { "CCA_TIME_OUT_CTRL",              0x20019034, PG_REG },
    { "BACKOFF_STAT_CTRL",              0x20019100, PG_REG },
    { "BACKOFF_STAT_PERIOD",            0x20019104, PG_REG },
    { "BACKOFF_STAT_TOTAL_DELAY",       0x20019108, PG_REG },
    { "BACKOFF_STAT_TOTAL_PSDU_CNT",    0x2001910C, PG_REG },
    { "ABORT_SELFCTS_PHY_TX_MODE",      0x20019200, PG_REG },
    { "ABORT_SELFCTS_DATARATE",         0x20019204, PG_REG },
    { "ABORT_SELFCTS_TIMEOUT",          0x20019208, PG_REG },
    { "ABORT_SELFCTS_DUR_VAL",          0x2001920C, PG_REG },
    { "COEX_ABORT_CTRL",                0x20019210, PG_REG },
    { "ABORT_CFEND_PHY_TX_MODE",        0x20019214, PG_REG },
    { "ABORT_CFEND_DATARATE",           0x20019218, PG_REG },
    { "ONE_PKT_CTRL",                   0x20019300, PG_REG },
    { "ONE_PKT_PHY_TX_MODE",            0x20019304, PG_REG },
    { "ONE_PKT_DATARATE",               0x20019308, PG_REG },
    { "ONE_PKT_TIMEOUT",                0x2001930C, PG_REG },
    { "ONE_PKT_DUR_VAL",                0x20019310, PG_REG },
    { "CF_END_PHY_TX_MODE",             0x20019314, PG_REG },
    { "CF_END_DATARATE",                0x20019318, PG_REG },
    { "ONE_PKT_BUF_ADDR",               0x2001931C, PG_REG },
    { "TEST_MODE",                      0x20019400, PG_REG },
    { "BYPASS_CONTROL",                 0x20019404, PG_REG },
    { "RX_TIME_OUT_VALUE",              0x20019408, PG_REG },
    { "TX_TIME_OUT_VALUE",              0x2001940C, PG_REG },
    { "AUTO_RST_CTRL",                  0x20019410, PG_REG },
    { "MAC_DIAG_SEL",                   0x20019414, PG_REG },
    { "MEM_DS_CTRL",                    0x20019418, PG_REG },
    { "MAX_DURATION_VALUE",             0x2001941C, PG_REG },
    { "DEFAULT_DURATION_VALUE",         0x20019420, PG_REG },
    { "DCOL_MODE",                      0x20019424, PG_REG },
    { "DCOL_LEN_UNIT",                  0x20019428, PG_REG },
    { "DCOL_BUF_BASE_ADDR",             0x2001942C, PG_REG },
    { "DCOL_BUF_BANK_NUM",              0x20019430, PG_REG },
    { "DCOL_UNIT_NUM",                  0x20019434, PG_REG },
    { "WOW_EN",                         0x20019500, PG_REG },
    { "WOW_MODE",                       0x20019504, PG_REG },
    { "WOW_AP0_PROBE_RESP_ADDR",        0x20019508, PG_REG },
    { "WOW_AP0_PROBE_RESP_LEN",         0x2001950C, PG_REG },
    { "WOW_AP1_PROBE_RESP_ADDR",        0x20019510, PG_REG },
    { "WOW_AP1_PROBE_RESP_LEN",         0x20019514, PG_REG },
    { "WOW_NULLDATA_WAKEUP_EN",         0x20019518, PG_REG },
    { "WOW_NULLDATA_PERIOD",            0x2001951C, PG_REG },
    { "WOW_AP0_PROBE_RESP_PHY_TX_MODE", 0x20019520, PG_REG },
    { "WOW_AP0_PROBE_RESP_DATARATE",    0x20019524, PG_REG },
    { "WOW_AP1_PROBE_RESP_PHY_TX_MODE", 0x20019528, PG_REG },
    { "WOW_AP1_PROBE_RESP_DATARATE",    0x2001952C, PG_REG },
    { "RX_AMPDU_COUNT",                 0x20019600, PG_REG },
    { "RX_PASSED_MPDU_IN_AMPDU_CNT",    0x20019604, PG_REG },
    { "RX_FAILED_MPDU_IN_AMPDU_CNT",    0x20019608, PG_REG },
    { "RX_OCTECTS_IN_AMPDU",            0x2001960C, PG_REG },
    { "RX_DELIMIT_FAIL_COUNT",          0x20019614, PG_REG },
    { "RX_DUP_MPDU_CNT",                0x20019618, PG_REG },
    { "RX_PASSED_MPDU_CNT",             0x2001961C, PG_REG },
    { "RX_FAILED_MPDU_CNT",             0x20019620, PG_REG },
    { "RX_BCN_CNT",                     0x20019624, PG_REG },
    { "RX_PHY_ERR_MAC_PASSED_CNT",      0x20019628, PG_REG },
    { "RX_PHY_LONGER_ERR_CNT",          0x20019634, PG_REG },
    { "RX_PHY_SHORTER_ERR_CNT",         0x20019638, PG_REG },
    { "DEAUTH_REASON_CODE",             0x2001963C, PG_REG },
    { "RX_TIMER1_VAL",                  0x20019640, PG_REG },
    { "RX_FILTERED_CNT",                0x20019644, PG_REG },
    { "TX_HI_PRI_MPDU_CNT",             0x20019800, PG_REG },
    { "TX_NORMAL_PRI_MPDU_CNT",         0x20019804, PG_REG },
    { "TX_AMPDU_COUNT",                 0x20019808, PG_REG },
    { "TX_MPDU_INAMPDU_COUNT",          0x2001980C, PG_REG },
    { "TX_OCTECTS_IN_AMPDU",            0x20019810, PG_REG },
    { "TX_BCN_COUNT",                   0x20019814, PG_REG },
    { "NORMAL_PRI_RETRY_CNT",           0x20019818, PG_REG },
    { "HI_PRI_RETRY_CNT",               0x2001981C, PG_REG },
    { "TX_TIMER1_VAL",                  0x20019820, PG_REG },
    { "TX_TIMER2_VAL",                  0x20019824, PG_REG },
    { "TX_TIMER3_VAL",                  0x20019828, PG_REG },
    { "COUNTER_CLEAR",                  0x2001982C, PG_REG },
    { "BEACON_MISS_NUM",                0x20019830, PG_REG },
    { "FSM_MON1_ST",                    0x20019A00, PG_REG },
    { "FSM_MON2_ST",                    0x20019A04, PG_REG },
    { "FSM_MON3_ST",                    0x20019A08, PG_REG },
    { "PA_FIFO_STATUS",                 0x20019A0C, PG_REG },
    { "CE_FIFO_STATUS",                 0x20019A10, PG_REG },
    { "NAV_CURR_VAL",                   0x20019A14, PG_REG },
    { "FSM_MON4_ST",                    0x20019A18, PG_REG },
    { "BUS_FIFO_STATUS",                0x20019A1C, PG_REG },
    { "WCH_REQ_BASE",                   0x20019A20, PG_REG },
    { "WCH_REQ_CNT",                    0x20019A24, PG_REG },
    { "RCH_REQ_BASE",                   0x20019A28, PG_REG },
    { "RCH_REQ_CNT",                    0x20019A2C, PG_REG },
    { "PA_CONTROL",                     0x20018000, PA_REG },
    { "PA_STATUS",                      0x20018004, PA_REG },
    { "MAC_HW_ID",                      0x20018008, PA_REG },
    { "HT_CTRL",                        0x2001800C, PA_REG },
    { "STA_MACADDR_H",                  0x20018010, PA_REG },
    { "STA_MACADDR_L",                  0x20018014, PA_REG },
    { "AP0_MACADDR_H",                  0x20018018, PA_REG },
    { "AP0_MACADDR_L",                  0x2001801C, PA_REG },
    { "AP1_MACADDR_H",                  0x20018020, PA_REG },
    { "AP1_MACADDR_L",                  0x20018024, PA_REG },
    { "AP2_MACADDR_H",                  0x20018028, PA_REG },
    { "AP2_MACADDR_L",                  0x2001802C, PA_REG },
    { "AP3_MACADDR_H",                  0x20018030, PA_REG },
    { "AP3_MACADDR_L",                  0x20018034, PA_REG },
    { "STA_BSSID_H",                    0x20018038, PA_REG },
    { "STA_BSSID_L",                    0x2001803C, PA_REG },
    { "LONG_FRAME_THRESHOLD",           0x20018040, PA_REG },
    { "TX_PHY_THRESHOLD",               0x20018044, PA_REG },
    { "STA_PARTIAL_AID",                0x20018048, PA_REG },
    { "DURATION_ADJUST_VAL",            0x2001804C, PA_REG },
    { "BUS_MIN_ADDR",                   0x20018050, PA_REG },
    { "BUS_MAX_ADDR",                   0x20018054, PA_REG },
    { "TKIP_TX_DELAY",                  0x20018058, PA_REG },
    { "BUS_CONTROL",                    0x2001805C, PA_REG },
    { "BACKOFF_PRE_A_DONE_SLOT_NUM",    0x20018060, PA_REG },
    { "PEER_ADDR_LUT_CONFIG",           0x20018200, PA_REG },
    { "PEER_ADDRESS_MSB",               0x20018204, PA_REG },
    { "PEER_ADDRESS_LSB",               0x20018208, PA_REG },
    { "RX_BUFFER_LENGTH",               0x2001820C, PA_REG },
    { "RX_HIBUFADDR_STATUS",            0x20018210, PA_REG },
    { "RX_HIBUFADDR_UPDATE",            0x20018214, PA_REG },
    { "RX_BUFADDR_STATUS",              0x20018218, PA_REG },
    { "RX_BUFADDR_UPDATE",              0x2001821C, PA_REG },
    { "RX_MAXLENFILT",                  0x20018220, PA_REG },
    { "RX_FRAMEFILT",                   0x20018224, PA_REG },
    { "PROTOCOL_CTRL",                  0x20018238, PA_REG },
    { "AP0_BCN_PKT_PTR",                0x20018400, PA_REG },
    { "AP0_BCN_TX_LENGTH",              0x20018404, PA_REG },
    { "AP0_BCN_PHY_TX_MODE",            0x20018408, PA_REG },
    { "AP0_BCN_TX_DATA_RATE",           0x2001840C, PA_REG },
    { "AP1_BCN_PKT_PTR",                0x20018410, PA_REG },
    { "AP1_BCN_TX_LENGTH",              0x20018414, PA_REG },
    { "AP1_BCN_PHY_TX_MODE",            0x20018418, PA_REG },
    { "AP1_BCN_TX_DATA_RATE",           0x2001841C, PA_REG },
    { "AP2_BCN_PKT_PTR",                0x20018420, PA_REG },
    { "AP2_BCN_TX_LENGTH",              0x20018424, PA_REG },
    { "AP2_BCN_PHY_TX_MODE",            0x20018428, PA_REG },
    { "AP2_BCN_TX_DATA_RATE",           0x2001842C, PA_REG },
    { "AP3_BCN_PKT_PTR",                0x20018430, PA_REG },
    { "AP3_BCN_TX_LENGTH",              0x20018434, PA_REG },
    { "AP3_BCN_PHY_TX_MODE",            0x20018438, PA_REG },
    { "AP3_BCN_TX_DATA_RATE",           0x2001843C, PA_REG },
    { "NULL_FRM_TX_PHY_MODE",           0x20018440, PA_REG },
    { "NULL_FRM_TX_DATA_RATE",          0x20018444, PA_REG },
    { "PROT_PHY_TX_MODE",               0x20018448, PA_REG },
    { "PROT_DATARATE",                  0x2001844C, PA_REG },
    { "VHT_BF_TX_PHY_MODE",             0x20018450, PA_REG },
    { "VHT_BF_TX_DATA_RATE",            0x20018454, PA_REG },
    { "RESP_PHY_TX_MODE",               0x20018458, PA_REG },
    { "RESP_DATARATE",                  0x2001845C, PA_REG },
    { "DEFAULT_ANTENNA_SET",            0x20018460, PA_REG },
    { "PHYTXPOWLVL",                    0x20018464, PA_REG },
    { "BEACON_TIMEOUT_VAL",             0x20018468, PA_REG },
    { "BEACON_MISS_MAX_NUM",            0x2001846C, PA_REG },
    { "TX_MAX_MSDU_LIFETIME",           0x20018600, PA_REG },
    { "SEQNUM_DUPDET_CTRL",             0x20018604, PA_REG },
    { "TX_SEQNUM",                      0x20018608, PA_REG },
    { "PRBS_SEEDVAL",                   0x2001860C, PA_REG },
    { "AIFSN",                          0x20018610, PA_REG },
    { "CWMINMAXACBK",                   0x20018614, PA_REG },
    { "CWMINMAXACBE",                   0x20018618, PA_REG },
    { "CWMINMAXACVI",                   0x2001861C, PA_REG },
    { "CWMINMAXACVO",                   0x20018620, PA_REG },
    { "EDCA_TXOPLIMIT_ACBKBE",          0x20018624, PA_REG },
    { "EDCA_TXOPLIMIT_ACVIVO",          0x20018628, PA_REG },
    { "AC_BK_FIRST_FRM_PTR_STATUS",     0x2001862C, PA_REG },
    { "AC_BK_FIRST_FRM_PTR_UPDATE",     0x20018630, PA_REG },
    { "AC_BE_FIRST_FRM_PTR_STATUS",     0x20018634, PA_REG },
    { "AC_BE_FIRST_FRM_PTR_UPDATE",     0x20018638, PA_REG },
    { "AC_VI_FIRST_FRM_PTR_STATUS",     0x2001863C, PA_REG },
    { "AC_VI_FIRST_FRM_PTR_UPDATE",     0x20018640, PA_REG },
    { "AC_VO_FIRST_FRM_PTR_STATUS",     0x20018644, PA_REG },
    { "AC_VO_FIRST_FRM_PTR_UPDATE",     0x20018648, PA_REG },
    { "HI_PRI_Q_FIRST_FRM_PTR_STATUS",  0x2001864C, PA_REG },
    { "HI_PRI_Q_FIRST_FRM_PTR_UPDATE",  0x20018650, PA_REG },
    { "ACBKBE_EDCA_LIFETIMELMT",        0x20018654, PA_REG },
    { "ACVIVO_EDCA_LIFETIMELMT",        0x20018658, PA_REG },
    { "EDCA_TXOPLIMIT_HI_PRI",          0x2001865C, PA_REG },
    { "TX_Q_STATUS",                    0x20018660, PA_REG },
    { "CBMAP_BA_CTRL",                  0x20018800, PA_REG },
    { "CBMAP_BA_STAADDRH",              0x20018804, PA_REG },
    { "CBMAP_BA_STAADDRL",              0x20018808, PA_REG },
    { "CBMAP_BA_PARAMS",                0x2001880C, PA_REG },
    { "CBMAP_BA_BMAPH",                 0x20018810, PA_REG },
    { "CBMAP_BA_BMAPL",                 0x20018814, PA_REG },
    { "SLOT_TIME",                      0x20018900, PA_REG },
    { "NDPA_DUR_CALC_PARAM",            0x20018908, PA_REG },
    { "NDP_GROUPID",                    0x2001890C, PA_REG },
    { "NUM_PA_CLKS_ONE_US",             0x20018A00, PA_REG },
    { "NUM_PA_CLKS_DECI_US",            0x20018A04, PA_REG },
    { "SIFS_TIME",                      0x20018A14, PA_REG },
    { "EIFS_TIME",                      0x20018A18, PA_REG },
    { "SEC_CH_SLOT",                    0x20018A1C, PA_REG },
    { "SIFS_TIME2",                     0x20018A20, PA_REG },
    { "RIFS_TIME_CTRL",                 0x20018A24, PA_REG },
    { "VHT_NDP_MAX_TIME",               0x20018A28, PA_REG },
    { "STA_TSF_CTRL",                   0x20018A2C, PA_REG },
    { "STA_TSFTIMER_RDVALH_STATUS",     0x20018A30, PA_REG },
    { "STA_TSFTIMER_RDVALL_STATUS",     0x20018A34, PA_REG },
    { "STA_TSFTIMER_RDVALH_UPDATE",     0x20018A38, PA_REG },
    { "STA_TSFTIMER_RDVALL_UPDATE",     0x20018A3C, PA_REG },
    { "STA_BCN_PERIOD",                 0x20018A40, PA_REG },
    { "STA_TBTT_TIMER_STATUS",          0x20018A44, PA_REG },
    { "STA_TBTT_TIMER_UPDATE",          0x20018A48, PA_REG },
    { "STA_DTIM_PERIOD",                0x20018A4C, PA_REG },
    { "STA_DTIM_COUNT_STATUS",          0x20018A50, PA_REG },
    { "STA_DTIM_COUNT_UPDATE",          0x20018A54, PA_REG },
    { "AP0_TSF_CTRL",                   0x20018A58, PA_REG },
    { "AP0_TSFTIMER_RDVALH_STATUS",     0x20018A5C, PA_REG },
    { "AP0_TSFTIMER_RDVALL_STATUS",     0x20018A60, PA_REG },
    { "AP0_TSFTIMER_RDVALH_UPDATE",     0x20018A64, PA_REG },
    { "AP0_TSFTIMER_RDVALL_UPDATE",     0x20018A68, PA_REG },
    { "AP0_BCN_PERIOD",                 0x20018A6C, PA_REG },
    { "AP0_DTIM_PERIOD",                0x20018A70, PA_REG },
    { "AP0_DTIM_COUNT_STATUS",          0x20018A74, PA_REG },
    { "AP0_DTIM_COUNT_UPDATE",          0x20018A78, PA_REG },
    { "AP0_TBTT_TIMER_STATUS",          0x20018A7C, PA_REG },
    { "AP0_TBTT_TIMER_UPDATE",          0x20018A80, PA_REG },
    { "AP1_TSF_CTRL",                   0x20018A84, PA_REG },
    { "AP1_TSFTIMER_RDVALH_STATUS",     0x20018A88, PA_REG },
    { "AP1_TSFTIMER_RDVALL_STATUS",     0x20018A8C, PA_REG },
    { "AP1_TSFTIMER_RDVALH_UPDATE",     0x20018A90, PA_REG },
    { "AP1_TSFTIMER_RDVALL_UPDATE",     0x20018A94, PA_REG },
    { "AP1_BCN_PERIOD",                 0x20018A98, PA_REG },
    { "AP1_DTIM_PERIOD",                0x20018A9C, PA_REG },
    { "AP1_DTIM_COUNT_STATUS",          0x20018AA0, PA_REG },
    { "AP1_DTIM_COUNT_UPDATE",          0x20018AA4, PA_REG },
    { "AP1_TBTT_TIMER_STATUS",          0x20018AA8, PA_REG },
    { "AP1_TBTT_TIMER_UPDATE",          0x20018AAC, PA_REG },
    { "AP2_TSF_CTRL",                   0x20018AB0, PA_REG },
    { "AP2_TSFTIMER_RDVALH_STATUS",     0x20018AB4, PA_REG },
    { "AP2_TSFTIMER_RDVALL_STATUS",     0x20018AB8, PA_REG },
    { "AP2_TSFTIMER_RDVALH_UPDATE",     0x20018ABC, PA_REG },
    { "AP2_TSFTIMER_RDVALL_UPDATE",     0x20018AC0, PA_REG },
    { "AP2_BCN_PERIOD",                 0x20018AC4, PA_REG },
    { "AP2_DTIM_PERIOD",                0x20018AC8, PA_REG },
    { "AP2_DTIM_COUNT_STATUS",          0x20018ACC, PA_REG },
    { "AP2_DTIM_COUNT_UPDATE",          0x20018AD0, PA_REG },
    { "AP2_TBTT_TIMER_STATUS",          0x20018AD4, PA_REG },
    { "AP2_TBTT_TIMER_UPDATE",          0x20018AD8, PA_REG },
    { "AP3_TSF_CTRL",                   0x20018ADC, PA_REG },
    { "AP3_TSFTIMER_RDVALH_STATUS",     0x20018AE0, PA_REG },
    { "AP3_TSFTIMER_RDVALL_STATUS",     0x20018AE4, PA_REG },
    { "AP3_TSFTIMER_RDVALH_UPDATE",     0x20018AE8, PA_REG },
    { "AP3_TSFTIMER_RDVALL_UPDATE",     0x20018AEC, PA_REG },
    { "AP3_BCN_PERIOD",                 0x20018AF0, PA_REG },
    { "AP3_DTIM_PERIOD",                0x20018AF4, PA_REG },
    { "AP3_DTIM_COUNT_STATUS",          0x20018AF8, PA_REG },
    { "AP3_DTIM_COUNT_UPDATE",          0x20018AFC, PA_REG },
    { "AP3_TBTT_TIMER_STATUS",          0x20018B00, PA_REG },
    { "AP3_TBTT_TIMER_UPDATE",          0x20018B04, PA_REG },
    { "PWR_MGMT_CTRL",                  0x20018B08, PA_REG },
    { "LISTEN_INTERVAL",                0x20018B0C, PA_REG },
    { "OFFSET_INTERVAL",                0x20018B10, PA_REG },
    { "LISTEN_INTERVAL_TIMER_STATUS",   0x20018B14, PA_REG },
    { "LISTEN_INTERVAL_TIMER_UPDATE",   0x20018B18, PA_REG },
    { "SMPS_CTRL",                      0x20018B1C, PA_REG },
    { "TXOP_PS_CTRL",                   0x20018B20, PA_REG },
    { "CH_STATISTIC_CONTROL",           0x20018C00, PA_REG },
    { "CH_LOAD_STAT_PERIOD",            0x20018C04, PA_REG },
    { "PRIMARY_20M_IDLE_COUNT",         0x20018C08, PA_REG },
    { "PRIMARY_40M_IDLE_COUNT",         0x20018C0C, PA_REG },
    { "PRIMARY_80M_IDLE_COUNT",         0x20018C10, PA_REG },
    { "TX_PROGRESS_COUNT",              0x20018C14, PA_REG },
    { "RX_PROGRESS_COUNT",              0x20018C18, PA_REG },
    { "TXBF_LUT_CONFIG",                0x20018C20, PA_REG },
    { "TXBF_LUT_INFO",                  0x20018C24, PA_REG },
    { "ANT_LUT_CONFIG",                 0x20018C28, PA_REG },
    { "SMART_ANTENNA_VALUE",            0x20018C2C, PA_REG },
    { "LEGACY_MATRIX_BUFFER_POINTER",   0x20018C30, PA_REG },
    { "MATRIX_TIMEOUT_VALUE",           0x20018C34, PA_REG },
    { "HT_MATRIX_BUFFER_POINTER",       0x20018C38, PA_REG },
    { "HT_MATRIX_BUFFER_STEP",          0x20018C3C, PA_REG },
    { "HT_MATRIX_BUFFER_NUM",           0x20018C40, PA_REG },
    { "INTERRUPT_STATUS",               0x20018E00, PA_REG },
    { "INTERRUPT_CLEAR",                0x20018E04, PA_REG },
    { "INTERRUPT_MASK",                 0x20018E08, PA_REG },
    { "ERR_INTR_STAT",                  0x20018E0C, PA_REG },
    { "ERR_INTR_CLEAR",                 0x20018E10, PA_REG },
    { "ERR_INTR_MASK",                  0x20018E14, PA_REG },
    { "HIRXBUFF_COUNT",                 0x20018E18, PA_REG },
    { "HIRXFRAME_PTR",                  0x20018E1C, PA_REG },
    { "RXBUFF_COUNT",                   0x20018E20, PA_REG },
    { "RXFRAME_PTR",                    0x20018E24, PA_REG },
    { "TXMPDU_COUNT",                   0x20018E28, PA_REG },
    { "TXFRAME_PTR",                    0x20018E2C, PA_REG },
    { "PHYTXPLCP_DELAY",                0x20019000, PA_REG },
    { "PHYRXPLCP_DELAY",                0x20019004, PA_REG },
    { "PHYRXTX_TURNAROUND_TIME",        0x20019008, PA_REG },
    { "PHYCCADELAY",                    0x20019014, PA_REG },
    { "PHYTXPLCP_ADJUST",               0x20019018, PA_REG },
    { "VHT_TXPLCP_ADJUST_VAL",          0x2001901C, PA_REG },
    { "PHYRXPLCP_DELAY2",               0x20019020, PA_REG },
    { "PHYRXSTARTDELAY",                0x20019024, PA_REG },
    { "VHT_RX_START_DELAY",             0x20019028, PA_REG },
    { "PROXY_STA_EN",                   0x2001902C, PA_REG },
    { "RIFS_RST_WAIT_CLK_NUM",          0x20019030, PA_REG },
    { "CCA_TIME_OUT_CTRL",              0x20019034, PA_REG },
    { "BACKOFF_STAT_CTRL",              0x20019100, PA_REG },
    { "BACKOFF_STAT_PERIOD",            0x20019104, PA_REG },
    { "BACKOFF_STAT_TOTAL_DELAY",       0x20019108, PA_REG },
    { "BACKOFF_STAT_TOTAL_PSDU_CNT",    0x2001910C, PA_REG },
    { "ABORT_SELFCTS_PHY_TX_MODE",      0x20019200, PA_REG },
    { "ABORT_SELFCTS_DATARATE",         0x20019204, PA_REG },
    { "ABORT_SELFCTS_TIMEOUT",          0x20019208, PA_REG },
    { "ABORT_SELFCTS_DUR_VAL",          0x2001920C, PA_REG },
    { "COEX_ABORT_CTRL",                0x20019210, PA_REG },
    { "ABORT_CFEND_PHY_TX_MODE",        0x20019214, PA_REG },
    { "ABORT_CFEND_DATARATE",           0x20019218, PA_REG },
    { "ONE_PKT_CTRL",                   0x20019300, PA_REG },
    { "ONE_PKT_PHY_TX_MODE",            0x20019304, PA_REG },
    { "ONE_PKT_DATARATE",               0x20019308, PA_REG },
    { "ONE_PKT_TIMEOUT",                0x2001930C, PA_REG },
    { "ONE_PKT_DUR_VAL",                0x20019310, PA_REG },
    { "CF_END_PHY_TX_MODE",             0x20019314, PA_REG },
    { "CF_END_DATARATE",                0x20019318, PA_REG },
    { "ONE_PKT_BUF_ADDR",               0x2001931C, PA_REG },
    { "TEST_MODE",                      0x20019400, PA_REG },
    { "BYPASS_CONTROL",                 0x20019404, PA_REG },
    { "RX_TIME_OUT_VALUE",              0x20019408, PA_REG },
    { "TX_TIME_OUT_VALUE",              0x2001940C, PA_REG },
    { "AUTO_RST_CTRL",                  0x20019410, PA_REG },
    { "MAC_DIAG_SEL",                   0x20019414, PA_REG },
    { "MEM_DS_CTRL",                    0x20019418, PA_REG },
    { "MAX_DURATION_VALUE",             0x2001941C, PA_REG },
    { "DEFAULT_DURATION_VALUE",         0x20019420, PA_REG },
    { "DCOL_MODE",                      0x20019424, PA_REG },
    { "DCOL_LEN_UNIT",                  0x20019428, PA_REG },
    { "DCOL_BUF_BASE_ADDR",             0x2001942C, PA_REG },
    { "DCOL_BUF_BANK_NUM",              0x20019430, PA_REG },
    { "DCOL_UNIT_NUM",                  0x20019434, PA_REG },
    { "WOW_EN",                         0x20019500, PA_REG },
    { "WOW_MODE",                       0x20019504, PA_REG },
    { "WOW_AP0_PROBE_RESP_ADDR",        0x20019508, PA_REG },
    { "WOW_AP0_PROBE_RESP_LEN",         0x2001950C, PA_REG },
    { "WOW_AP1_PROBE_RESP_ADDR",        0x20019510, PA_REG },
    { "WOW_AP1_PROBE_RESP_LEN",         0x20019514, PA_REG },
    { "WOW_NULLDATA_WAKEUP_EN",         0x20019518, PA_REG },
    { "WOW_NULLDATA_PERIOD",            0x2001951C, PA_REG },
    { "WOW_AP0_PROBE_RESP_PHY_TX_MODE", 0x20019520, PA_REG },
    { "WOW_AP0_PROBE_RESP_DATARATE",    0x20019524, PA_REG },
    { "WOW_AP1_PROBE_RESP_PHY_TX_MODE", 0x20019528, PA_REG },
    { "WOW_AP1_PROBE_RESP_DATARATE",    0x2001952C, PA_REG },
    { "RX_AMPDU_COUNT",                 0x20019600, PA_REG },
    { "RX_PASSED_MPDU_IN_AMPDU_CNT",    0x20019604, PA_REG },
    { "RX_FAILED_MPDU_IN_AMPDU_CNT",    0x20019608, PA_REG },
    { "RX_OCTECTS_IN_AMPDU",            0x2001960C, PA_REG },
    { "RX_DELIMIT_FAIL_COUNT",          0x20019614, PA_REG },
    { "RX_DUP_MPDU_CNT",                0x20019618, PA_REG },
    { "RX_PASSED_MPDU_CNT",             0x2001961C, PA_REG },
    { "RX_FAILED_MPDU_CNT",             0x20019620, PA_REG },
    { "RX_BCN_CNT",                     0x20019624, PA_REG },
    { "RX_PHY_ERR_MAC_PASSED_CNT",      0x20019628, PA_REG },
    { "RX_PHY_LONGER_ERR_CNT",          0x20019634, PA_REG },
    { "RX_PHY_SHORTER_ERR_CNT",         0x20019638, PA_REG },
    { "DEAUTH_REASON_CODE",             0x2001963C, PA_REG },
    { "RX_TIMER1_VAL",                  0x20019640, PA_REG },
    { "RX_FILTERED_CNT",                0x20019644, PA_REG },
    { "TX_HI_PRI_MPDU_CNT",             0x20019800, PA_REG },
    { "TX_NORMAL_PRI_MPDU_CNT",         0x20019804, PA_REG },
    { "TX_AMPDU_COUNT",                 0x20019808, PA_REG },
    { "TX_MPDU_INAMPDU_COUNT",          0x2001980C, PA_REG },
    { "TX_OCTECTS_IN_AMPDU",            0x20019810, PA_REG },
    { "TX_BCN_COUNT",                   0x20019814, PA_REG },
    { "NORMAL_PRI_RETRY_CNT",           0x20019818, PA_REG },
    { "HI_PRI_RETRY_CNT",               0x2001981C, PA_REG },
    { "TX_TIMER1_VAL",                  0x20019820, PA_REG },
    { "TX_TIMER2_VAL",                  0x20019824, PA_REG },
    { "TX_TIMER3_VAL",                  0x20019828, PA_REG },
    { "COUNTER_CLEAR",                  0x2001982C, PA_REG },
    { "BEACON_MISS_NUM",                0x20019830, PA_REG },
    { "FSM_MON1_ST",                    0x20019A00, PA_REG },
    { "FSM_MON2_ST",                    0x20019A04, PA_REG },
    { "FSM_MON3_ST",                    0x20019A08, PA_REG },
    { "PA_FIFO_STATUS",                 0x20019A0C, PA_REG },
    { "CE_FIFO_STATUS",                 0x20019A10, PA_REG },
    { "NAV_CURR_VAL",                   0x20019A14, PA_REG },
    { "FSM_MON4_ST",                    0x20019A18, PA_REG },
    { "BUS_FIFO_STATUS",                0x20019A1C, PA_REG },
    { "WCH_REQ_BASE",                   0x20019A20, PA_REG },
    { "WCH_REQ_CNT",                    0x20019A24, PA_REG },
    { "RCH_REQ_BASE",                   0x20019A28, PA_REG },
    { "RCH_REQ_CNT",                    0x20019A2C, PA_REG },
    { "CE_CONTROL",                     0x2001A000, CE_REG },
    { "CE_LUT_CONFIG",                  0x2001A004, CE_REG },
    { "KEY_QTR1",                       0x2001A010, CE_REG },
    { "KEY_QTR2",                       0x2001A014, CE_REG },
    { "KEY_QTR3",                       0x2001A018, CE_REG },
    { "KEY_QTR4",                       0x2001A01C, CE_REG },
    { "TKIP_MIC_KEY_QTR1",              0x2001A020, CE_REG },
    { "TKIP_MIC_KEY_QTR2",              0x2001A024, CE_REG },
    { "TKIP_MIC_KEY_QTR3",              0x2001A028, CE_REG },
    { "TKIP_MIC_KEY_QTR4",              0x2001A02C, CE_REG },
    { "TX_PN_LUT_CONFIG",               0x2001A100, CE_REG },
    { "TX_PN_VALUE_MSB",                0x2001A104, CE_REG },
    { "TX_PN_VALUE_LSB",                0x2001A108, CE_REG },
    { "RX_PN_LUT_CONFIG",               0x2001A10C, CE_REG },
    { "RX_PN_VALUE_MSB",                0x2001A110, CE_REG },
    { "RX_PN_VALUE_LSB",                0x2001A114, CE_REG },
    { "TKIP_REP_FAIL_CNT",              0x2001A20C, CE_REG },
    { "CCMP_REP_FAIL_CNT",              0x2001A210, CE_REG },
};

OAL_STATIC oam_reg_cfg_stru phy_reg[] = {
    { "FREQ_BAND",                        0x20038800, PHY_REG_BANK1 },
    { "PHY_BW_MODE",                      0x20038804, PHY_REG_BANK1 },
    { "PRI20M_CHN_NUM",                   0x20038808, PHY_REG_BANK1 },
    { "SEC20M_OFFSET",                    0x2003880C, PHY_REG_BANK1 },
    { "SEC40M_OFFSET",                    0x20038810, PHY_REG_BANK1 },
    { "ADC_FS",                           0x20038814, PHY_REG_BANK1 },
    { "DAC_FS",                           0x20038818, PHY_REG_BANK1 },
    { "PHY_CTRL0",                        0x2003881C, PHY_REG_BANK1 },
    { "RX_MULT_ANT_SEL",                  0x20038820, PHY_REG_BANK1 },
    { "RX_ONE_ANT_SEL",                   0x20038824, PHY_REG_BANK1 },
    { "TX_CONTROL",                       0x20038828, PHY_REG_BANK1 },
    { "CONT_TX_MODE",                     0x2003882C, PHY_REG_BANK1 },
    { "RX_11B_ADJUST",                    0x20038830, PHY_REG_BANK1 },
    { "RX_OFDM_PLCP_DLY",                 0x20038834, PHY_REG_BANK1 },
    { "TX_TIME_CONST",                    0x20038838, PHY_REG_BANK1 },
    { "TX_DFE_DLY",                       0x2003883C, PHY_REG_BANK1 },
    { "MACIF_FIFO_ERR_RPT",               0x20038840, PHY_REG_BANK1 },
    { "PHY_CTRL1",                        0x20038844, PHY_REG_BANK1 },
    { "PHY_STA_01_EN",                    0x2003884C, PHY_REG_BANK1 },
    { "PHY_STA_23_EN",                    0x2003885C, PHY_REG_BANK1 },
    { "FINE_TIMING_DELAY",                0x20038850, PHY_REG_BANK1 },
    { "LEGA_ADJ_DELAY",                   0x20038854, PHY_REG_BANK1 },
    { "GF_NDP_ADJ_DELAY",                 0x20038858, PHY_REG_BANK1 },
    { "TEST_PIN_SEL",                     0x20038860, PHY_REG_BANK1 },
    { "MACIF_FIFO_STATE_RPT",             0x20038864, PHY_REG_BANK1 },
    { "CONT_PSDU_LENGTH",                 0x20038868, PHY_REG_BANK1 },
    { "STF_Q_SEL",                        0x20038880, PHY_REG_BANK1 },
    { "ALMOSR_1_FULL",                    0x20038884, PHY_REG_BANK1 },
    { "ALMOSR_2_FULL",                    0x20038888, PHY_REG_BANK1 },
    { "TIMIN_CSD0_SHIFT_L",               0x2003888C, PHY_REG_BANK1 },
    { "TIMIN_CSD1_SHIFT_L_40_CFG",        0x20038890, PHY_REG_BANK1 },
    { "TIMIN_CSD1_SHIFT_L_20_CFG",        0x20038894, PHY_REG_BANK1 },
    { "TIMIN_CSD1_SHIFT_L_80_CFG",        0x20038898, PHY_REG_BANK1 },
    { "TIMIN_CSD0_SHIFT_HT",              0x2003889C, PHY_REG_BANK1 },
    { "TIMIN_CSD1_SHIFT_HT",              0x200388A0, PHY_REG_BANK1 },
    { "TIMIN_CSD1_SHIFT_HT_20_CFG",       0x200388A4, PHY_REG_BANK1 },
    { "TIMIN_CSD1_SHIFT_HT_80_CFG",       0x200388A8, PHY_REG_BANK1 },
    { "LEGACY_Q_0_COL_I",                 0x200388AC, PHY_REG_BANK1 },
    { "LEGACY_Q_1_COL_I",                 0x200388B0, PHY_REG_BANK1 },
    { "LEGACY_Q_0_COL_I_12S_CFG",         0x200388B4, PHY_REG_BANK1 },
    { "LEGACY_Q_1_COL_I_12S_CFG",         0x200388B8, PHY_REG_BANK1 },
    { "LEGACY_Q_0_COL_Q",                 0x200388BC, PHY_REG_BANK1 },
    { "LEGACY_Q_1_COL_Q",                 0x200388C0, PHY_REG_BANK1 },
    { "LEGACY_Q_0_COL_Q_12S_CFG",         0x200388C4, PHY_REG_BANK1 },
    { "LEGACY_Q_1_COL_Q_12S_CFG",         0x200388C8, PHY_REG_BANK1 },
    { "HT_Q_0_COL_I",                     0x200388CC, PHY_REG_BANK1 },
    { "HT_Q_1_COL_I",                     0x200388D0, PHY_REG_BANK1 },
    { "HT_Q_0_COL_I_12S_CFG",             0x200388D4, PHY_REG_BANK1 },
    { "HT_Q_1_COL_I_12S_CFG",             0x200388D8, PHY_REG_BANK1 },
    { "HT_Q_0_COL_Q",                     0x200388DC, PHY_REG_BANK1 },
    { "HT_Q_1_COL_Q",                     0x200388E0, PHY_REG_BANK1 },
    { "HT_Q_0_COL_Q_12S_CFG",             0x200388E4, PHY_REG_BANK1 },
    { "HT_Q_1_COL_Q_12S_CFG",             0x200388E8, PHY_REG_BANK1 },
    { "FREQ_0_CSD",                       0x200388EC, PHY_REG_BANK1 },
    { "FREQ_1_CSD",                       0x200388F0, PHY_REG_BANK1 },
    { "PILOT_BF_EN",                      0x200388F4, PHY_REG_BANK1 },
    { "TX_BEAMFORMING_LEGA",              0x20038900, PHY_REG_BANK1 },
    { "L_LTF_MAX_CPU",                    0x20038904, PHY_REG_BANK1 },
    { "LEGACY_Q_0_COL_I_11S_CFG",         0x20038908, PHY_REG_BANK1 },
    { "LEGACY_Q_0_COL_Q_11S_CFG",         0x2003890C, PHY_REG_BANK1 },
    { "HT_Q_0_COL_I_11S_CFG",             0x20038910, PHY_REG_BANK1 },
    { "HT_Q_0_COL_Q_11S_CFG",             0x20038914, PHY_REG_BANK1 },
    { "CHN_EST_CTRL",                     0x20038918, PHY_REG_BANK1 },
    { "CHN_SMOOTHING_FILTER_COEF_01_1SS", 0x2003891C, PHY_REG_BANK1 },
    { "CHN_SMOOTHING_FILTER_COEF_23_1SS", 0x20038920, PHY_REG_BANK1 },
    { "CHN_SMOOTHING_FILTER_COEF_01_2SS", 0x20038924, PHY_REG_BANK1 },
    { "CHN_SMOOTHING_FILTER_COEF_23_2SS", 0x20038928, PHY_REG_BANK1 },
    { "CHN_SMOOTHING_MCS_TH",             0x2003892C, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL1",                    0x20038940, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL2",                    0x20038944, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL3",                    0x20038948, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL4",                    0x2003894C, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL5",                    0x20038950, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL6",                    0x20038954, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL7",                    0x20038958, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL8",                    0x2003895C, PHY_REG_BANK1 },
    { "PHY_RX_CTRL",                      0x20038960, PHY_REG_BANK1 },
    { "PHY_STATE_RPT0",                   0x20038964, PHY_REG_BANK1 },
    { "PHY_ERR_RPT",                      0x20038968, PHY_REG_BANK1 },
    { "PHY_STA0_RPT",                     0x2003896C, PHY_REG_BANK1 },
    { "PHY_STA1_RPT",                     0x20038970, PHY_REG_BANK1 },
    { "PHY_STA2_RPT",                     0x20038974, PHY_REG_BANK1 },
    { "PHY_STA3_RPT",                     0x20038978, PHY_REG_BANK1 },
    { "RX_MODULE_DELAY",                  0x20038984, PHY_REG_BANK1 },
    { "PHY_WARN_RPT",                     0x2003898C, PHY_REG_BANK1 },
    { "PHY_STATE_RPT1",                   0x20038990, PHY_REG_BANK1 },
    { "PHY_STATE_RPT3",                   0x20038994, PHY_REG_BANK1 },
    { "PHY_STATE_RPT4",                   0x20038998, PHY_REG_BANK1 },
    { "FFT_WEIGHT_EN",                    0x20038A30, PHY_REG_BANK1 },
    { "SINGLE_0TONE_0_3CAR",              0x20038A34, PHY_REG_BANK1 },
    { "SINGLE_0TONE_4_7CAR",              0x20038A38, PHY_REG_BANK1 },
    { "SINGLE_1TONE_0_3CAR",              0x20038A3C, PHY_REG_BANK1 },
    { "SINGLE_1TONE_4_7CAR",              0x20038A40, PHY_REG_BANK1 },
    { "SINGLE_2TONE_0_3CAR",              0x20038A44, PHY_REG_BANK1 },
    { "SINGLE_2TONE_4_7CAR",              0x20038A48, PHY_REG_BANK1 },
    { "SINGLE_0TONE_0_7WEIGHT",           0x20038A4C, PHY_REG_BANK1 },
    { "SINGLE_1TONE_0_7WEIGHT",           0x20038A50, PHY_REG_BANK1 },
    { "SINGLE_2TONE_0_7WEIGHT",           0x20038A54, PHY_REG_BANK1 },
    { "DC_OFFSET_WEIGHT_EN",              0x20038A58, PHY_REG_BANK1 },
    { "DC_OFFSET_WEIGHT_FACTOR",          0x20038A5C, PHY_REG_BANK1 },
    { "DC_OFFSET_FREQ_TH",                0x20038A60, PHY_REG_BANK1 },
    { "VT_K1_PARA",                       0x20038A64, PHY_REG_BANK1 },
    { "VT_K2_PARA",                       0x20038A68, PHY_REG_BANK1 },
    { "VT_K3_PARA",                       0x20038A6C, PHY_REG_BANK1 },
    { "VT_K4_PARA",                       0x20038A70, PHY_REG_BANK1 },
    { "VT_K5_PARA",                       0x20038A74, PHY_REG_BANK1 },
    { "VT_K6_PARA",                       0x20038A78, PHY_REG_BANK1 },
    { "VT_K7_PARA",                       0x20038A7C, PHY_REG_BANK1 },
    { "VT_K8_PARA",                       0x20038A80, PHY_REG_BANK1 },
    { "VT_K9_PARA",                       0x20038A84, PHY_REG_BANK1 },
    { "VT_K10_PARA",                      0x20038A88, PHY_REG_BANK1 },
    { "LDPC_K1_PARA",                     0x20038A8C, PHY_REG_BANK1 },
    { "LDPC_K2_PARA",                     0x20038A90, PHY_REG_BANK1 },
    { "LDPC_K3_PARA",                     0x20038A94, PHY_REG_BANK1 },
    { "LDPC_K4_PARA",                     0x20038A98, PHY_REG_BANK1 },
    { "LDPC_K5_PARA",                     0x20038A9C, PHY_REG_BANK1 },
    { "LDPC_K6_PARA",                     0x20038AA0, PHY_REG_BANK1 },
    { "LDPC_K7_PARA",                     0x20038AA4, PHY_REG_BANK1 },
    { "LDPC_K8_PARA",                     0x20038AA8, PHY_REG_BANK1 },
    { "LDPC_K9_PARA",                     0x20038AAC, PHY_REG_BANK1 },
    { "LDPC_K10_PARA",                    0x20038AB0, PHY_REG_BANK1 },
    { "VT_SCALLR1_PARA",                  0x20038AB4, PHY_REG_BANK1 },
    { "VT_SCALLR2_PARA",                  0x20038AB8, PHY_REG_BANK1 },
    { "VT_SCALLR3_PARA",                  0x20038ABC, PHY_REG_BANK1 },
    { "VT_SCALLR4_PARA",                  0x20038AC0, PHY_REG_BANK1 },
    { "VT_SCALLR5_PARA",                  0x20038AC4, PHY_REG_BANK1 },
    { "VT_SCALLR6_PARA",                  0x20038AC8, PHY_REG_BANK1 },
    { "VT_SCALLR7_PARA",                  0x20038ACC, PHY_REG_BANK1 },
    { "VT_SCALLR8_PARA",                  0x20038AD0, PHY_REG_BANK1 },
    { "VT_SCALLR9_PARA",                  0x20038AD4, PHY_REG_BANK1 },
    { "VT_SCALLR10_PARA",                 0x20038AD8, PHY_REG_BANK1 },
    { "LDPC_SCALLR1_PARA",                0x20038ADC, PHY_REG_BANK1 },
    { "LDPC_SCALLR2_PARA",                0x20038AE0, PHY_REG_BANK1 },
    { "LDPC_SCALLR3_PARA",                0x20038AE4, PHY_REG_BANK1 },
    { "LDPC_SCALLR4_PARA",                0x20038AE8, PHY_REG_BANK1 },
    { "LDPC_SCALLR5_PARA",                0x20038AEC, PHY_REG_BANK1 },
    { "LDPC_SCALLR6_PARA",                0x20038AF0, PHY_REG_BANK1 },
    { "LDPC_SCALLR7_PARA",                0x20038AF4, PHY_REG_BANK1 },
    { "LDPC_SCALLR8_PARA",                0x20038AF8, PHY_REG_BANK1 },
    { "LDPC_SCALLR9_PARA",                0x20038AFC, PHY_REG_BANK1 },
    { "LDPC_SCALLR10_PARA",               0x20038B00, PHY_REG_BANK1 },
    { "FLATNESS_0_COEF",                  0x20038B04, PHY_REG_BANK1 },
    { "FLATNESS_1_COEF",                  0x20038B08, PHY_REG_BANK1 },
    { "FLATNESS_2_COEF",                  0x20038B0C, PHY_REG_BANK1 },
    { "VT_K11_PARA",                      0x20038B10, PHY_REG_BANK1 },
    { "RX_SYNC_CTRL9",                    0x20038B14, PHY_REG_BANK1 },
    { "SYNC_ERR_RPT",                     0x20038B18, PHY_REG_BANK1 },
    { "FLATNESS_0_20_COEF",               0x20038B1C, PHY_REG_BANK1 },
    { "FLATNESS_1_20_COEF",               0x20038B20, PHY_REG_BANK1 },
    { "FLATNESS_0_40_COEF",               0x20038B24, PHY_REG_BANK1 },
    { "FLATNESS_1_40_COEF",               0x20038B28, PHY_REG_BANK1 },
    { "FLATNESS_2_40_COEF",               0x20038B2C, PHY_REG_BANK1 },
    { "DOTB_OK_FRM_NUM",                  0x20038B30, PHY_REG_BANK1 },
    { "HT_OK_FRM_NUM",                    0x20038B34, PHY_REG_BANK1 },
    { "VHT_OK_FRM_NUM",                   0x20038B38, PHY_REG_BANK1 },
    { "LEGA_OK_FRM_NUM",                  0x20038B3C, PHY_REG_BANK1 },
    { "DOTB_ERR_FRM_NUM",                 0x20038B40, PHY_REG_BANK1 },
    { "HT_ERR_FRM_NUM",                   0x20038B44, PHY_REG_BANK1 },
    { "VHT_ERR_FRM_NUM",                  0x20038B48, PHY_REG_BANK1 },
    { "LEGA_ERR_FRM_NUM",                 0x20038B4C, PHY_REG_BANK1 },
    { "STAT_CLR",                         0x20038B50, PHY_REG_BANK1 },
    { "STA_EN",                           0x20038B54, PHY_REG_BANK1 },
    { "DC_OFFSET_WEIGHT_2FACTOR",         0x20038B58, PHY_REG_BANK1 },
    { "LDPC_ITERT_NUM0",                  0x20038B5C, PHY_REG_BANK1 },
    { "LDPC_ITERT_NUM1",                  0x20038B60, PHY_REG_BANK1 },
    { "LDPC_ITERT_NUM2",                  0x20038B64, PHY_REG_BANK1 },
    { "LDPC_ITERT_NUM3",                  0x20038B68, PHY_REG_BANK1 },
    { "LDPC_LAST_ITERT_NUM0",             0x20038B6C, PHY_REG_BANK1 },
    { "LDPC_LAST_ITERT_NUM1",             0x20038B70, PHY_REG_BANK1 },
    { "LDPC_LAST_ITERT_NUM2",             0x20038B74, PHY_REG_BANK1 },
    { "LDPC_LAST_ITERT_NUM3",             0x20038B78, PHY_REG_BANK1 },
    { "LDPC_LAST_ITERT_NUM4",             0x20038B7C, PHY_REG_BANK1 },
    { "LDPC_LAST_ITERT_NUM5",             0x20038B80, PHY_REG_BANK1 },
    { "NUM_DC_OFFSET_PKT",                0x20038C00, PHY_REG_BANK2 },
    { "NUM_DC_OFFSET_PIN",                0x20038C04, PHY_REG_BANK2 },
    { "NUM_INCOMING",                     0x20038C08, PHY_REG_BANK2 },
    { "STRONG_SIGNAL_TH",                 0x20038C0C, PHY_REG_BANK2 },
    { "INCOMING_SIGNAL_TH",               0x20038C10, PHY_REG_BANK2 },
    { "NUM_INCOMING_TH",                  0x20038C14, PHY_REG_BANK2 },
    { "SATU_BSYNC_TH",                    0x20038C18, PHY_REG_BANK2 },
    { "SATU_ASYNC_TH",                    0x20038C1C, PHY_REG_BANK2 },
    { "NUM_SATU_BSYNC",                   0x20038C20, PHY_REG_BANK2 },
    { "NUM_SATU_ASYNC",                   0x20038C24, PHY_REG_BANK2 },
    { "NUM_CROSATU_BSYNC_TH",             0x20038C28, PHY_REG_BANK2 },
    { "NUM_CROSATU_ASYNC_TH",             0x20038C2C, PHY_REG_BANK2 },
    { "SAT_AGC_STEP",                     0x20038C30, PHY_REG_BANK2 },
    { "NUM_POWERSUM_PKT",                 0x20038C34, PHY_REG_BANK2 },
    { "NUM_POWERSUM_BSYNC",               0x20038C38, PHY_REG_BANK2 },
    { "NUM_POWERSUM_ASYNC",               0x20038C3C, PHY_REG_BANK2 },
    { "CFG_LOW_REL_ADC_PIN",              0x20038C40, PHY_REG_BANK2 },
    { "MAX_LNA_CODE",                     0x20038C44, PHY_REG_BANK2 },
    { "MAX_VGA_CODE",                     0x20038C48, PHY_REG_BANK2 },
    { "MIN_VGA_CODE",                     0x20038C4C, PHY_REG_BANK2 },
    { "POWER0_REF",                       0x20038C50, PHY_REG_BANK2 },
    { "ADC_TARGET",                       0x20038C54, PHY_REG_BANK2 },
    { "CFG_COARSE_TEL_ERR",               0x20038C58, PHY_REG_BANK2 },
    { "CFG_MAX_COARSE_NUM",               0x20038C5C, PHY_REG_BANK2 },
    { "CFG_FINE_TO_COARSE_TEL_ERR",       0x20038C60, PHY_REG_BANK2 },
    { "CFG_FINE_TEL_ERR",                 0x20038C64, PHY_REG_BANK2 },
    { "CFG_MAX_FINE_NUM",                 0x20038C68, PHY_REG_BANK2 },
    { "CFG_TRACK_ABOVE_ERR_BSYNC",        0x20038C6C, PHY_REG_BANK2 },
    { "CFG_TRACK_ABOVE_ERR_ASYNC",        0x20038C70, PHY_REG_BANK2 },
    { "CFG_TRACK_BELOW_ERR_BSYNC",        0x20038C74, PHY_REG_BANK2 },
    { "CFG_TRACK_BELOW_ERR_ASYNC",        0x20038C78, PHY_REG_BANK2 },
    { "CFG_TARGET_ABOVE_ERR_BSYNC",       0x20038C7C, PHY_REG_BANK2 },
    { "CFG_TARGET_ABOVE_ERR_ASYNC",       0x20038C80, PHY_REG_BANK2 },
    { "CFG_TARGET_BELOW_ERR_BSYNC",       0x20038C84, PHY_REG_BANK2 },
    { "CFG_TARGET_BELOW_ERR_ASYNC",       0x20038C88, PHY_REG_BANK2 },
    { "NUM_RIFS_WAIT_SAMP",               0x20038C8C, PHY_REG_BANK2 },
    { "EXTLNA_CHGPTDBM_TH",               0x20038C90, PHY_REG_BANK2 },
    { "INTLNA_CHGPTDBM0_TH",              0x20038C94, PHY_REG_BANK2 },
    { "INTLNA_CHGPTDBM1_TH",              0x20038C98, PHY_REG_BANK2 },
    { "INTLNA_CHGPTDBM2_TH",              0x20038C9C, PHY_REG_BANK2 },
    { "INTLNA_CHGPTDBM3_TH",              0x20038CA0, PHY_REG_BANK2 },
    { "INTLNA_CHGPTDBM4_TH",              0x20038CA4, PHY_REG_BANK2 },
    { "INTLNA_CHGPTDBM5_TH",              0x20038CA8, PHY_REG_BANK2 },
    { "EXTLNA_01_SETTLE_TIME",            0x20038CAC, PHY_REG_BANK2 },
    { "EXTLNA_10_SETTLE_TIME",            0x20038CB0, PHY_REG_BANK2 },
    { "INT_LNA_SET_TIME",                 0x20038CB4, PHY_REG_BANK2 },
    { "VGA_SET_TIME",                     0x20038CB8, PHY_REG_BANK2 },
    { "EXTLNA_GAIN0_CFG",                 0x20038CBC, PHY_REG_BANK2 },
    { "EXTLNA_GAIN1_CFG",                 0x20038CC0, PHY_REG_BANK2 },
    { "INTLNA0_CFG",                      0x20038CC4, PHY_REG_BANK2 },
    { "INTLNA1_CFG",                      0x20038CC8, PHY_REG_BANK2 },
    { "INTLNA2_CFG",                      0x20038CCC, PHY_REG_BANK2 },
    { "INTLNA3_CFG",                      0x20038CD0, PHY_REG_BANK2 },
    { "INTLNA4_CFG",                      0x20038CD4, PHY_REG_BANK2 },
    { "INTLNA5_CFG",                      0x20038CD8, PHY_REG_BANK2 },
    { "INTLNA6_CFG",                      0x20038CDC, PHY_REG_BANK2 },
    { "CFG_DEFAULT_FLAG",                 0x20038CE0, PHY_REG_BANK2 },
    { "DEFAULT_LNACODE",                  0x20038CE4, PHY_REG_BANK2 },
    { "DEFAULT_VGACODE",                  0x20038CE8, PHY_REG_BANK2 },
    { "ACCUM_WIN_BF_SYNC",                0x20038CEC, PHY_REG_BANK2 },
    { "SLIDE_WIN_BF_SYNC",                0x20038CF0, PHY_REG_BANK2 },
    { "ACCUM_WIN_FOR_INITIALBW",          0x20038CF4, PHY_REG_BANK2 },
    { "SLIDE_WIN_FOR_INITIALBW",          0x20038CF8, PHY_REG_BANK2 },
    { "ACCUM_WIN_TRACKING",               0x20038CFC, PHY_REG_BANK2 },
    { "SLIDE_WIN_TRACKING",               0x20038D00, PHY_REG_BANK2 },
    { "PKT_DET_WIN_TH1",                  0x20038D04, PHY_REG_BANK2 },
    { "PKT_DET_WIN_TH2",                  0x20038D08, PHY_REG_BANK2 },
    { "PKT_END_WAIT",                     0x20038D0C, PHY_REG_BANK2 },
    { "DC_OFFSET_EST_TH",                 0x20038D10, PHY_REG_BANK2 },
    { "SINGLE_TONE_EST_TH",               0x20038D14, PHY_REG_BANK2 },
    { "AGC_ENABLE_OR_BYPASS",             0x20038D18, PHY_REG_BANK2 },
    { "CFG_DEFAULT_TRANS_TIME",           0x20038D1C, PHY_REG_BANK2 },
    { "PIN_CBW20_MIN",                    0x20038D20, PHY_REG_BANK2 },
    { "DAGC_SET_TIME",                    0x20038D24, PHY_REG_BANK2 },
    { "MAX_DIGITAL_AGC",                  0x20038D28, PHY_REG_BANK2 },
    { "DAGC_TARGET_LO",                   0x20038D2C, PHY_REG_BANK2 },
    { "DAGC_TARGET_HI",                   0x20038D30, PHY_REG_BANK2 },
    { "DFE_RSSI_SWITCH_POWER",            0x20038D34, PHY_REG_BANK2 },
    { "PWR_DIFF_HIGH_LOW_TH",             0x20038D38, PHY_REG_BANK2 },
    { "AGC_LOCK_UNLOCK_RPT",              0x20038D3C, PHY_REG_BANK2 },
    { "AGC_TRACK_ANT_SEL",                0x20038D40, PHY_REG_BANK2 },
    { "RADAR_RSSI_CH_SEL",                0x20038D44, PHY_REG_BANK2 },
    { "PRI20_DIFF_HIGH_LOW_TH",           0x20038D48, PHY_REG_BANK2 },
    { "ADC_DC_OFFSET_0CH",                0x20038D4C, PHY_REG_BANK2 },
    { "ADC_DC_OFFSET_1CH",                0x20038D50, PHY_REG_BANK2 },
    { "RPT_VGA_CODE",                     0x20038D54, PHY_REG_BANK2 },
    { "DC_RPT_EN",                        0x20038D58, PHY_REG_BANK2 },
    { "DC_OFFSET_ENABLE",                 0x20038D5C, PHY_REG_BANK2 },
    { "ACTIVE_ANT",                       0x20038D60, PHY_REG_BANK2 },
    { "TRACK_FILTER",                     0x20038D64, PHY_REG_BANK2 },
    { "ADC_PIN_CODE_RPT",                 0x20038D68, PHY_REG_BANK2 },
    { "AGC_RADAR_DET_EN",                 0x20038D6C, PHY_REG_BANK2 },
    { "CCA_BYPASS",                       0x20038D7C, PHY_REG_BANK2 },
    { "SCALING_VALUE_11B",                0x20038D80, PHY_REG_BANK2 },
    { "U1_SCALING_VALUE_11G",             0x20038D84, PHY_REG_BANK2 },
    { "U2_SCALING_VALUE_11G",             0x20038D88, PHY_REG_BANK2 },
    { "U1_SCALING_VALUE_11N_2D4G",        0x20038D8C, PHY_REG_BANK2 },
    { "U2_SCALING_VALUE_11N_2D4G",        0x20038D90, PHY_REG_BANK2 },
    { "U1_SCALING_VALUE_11N40M_2D4G",     0x20038D94, PHY_REG_BANK2 },
    { "U2_SCALING_VALUE_11N40M_2D4G",     0x20038D98, PHY_REG_BANK2 },
    { "U1_SCALING_VALUE_11A",             0x20038D9C, PHY_REG_BANK2 },
    { "U2_SCALING_VALUE_11A",             0x20038DA0, PHY_REG_BANK2 },
    { "U0_SCALING_VALUE_11N_5G",          0x20038DA4, PHY_REG_BANK2 },
    { "U1_SCALING_VALUE_11N_5G",          0x20038DA8, PHY_REG_BANK2 },
    { "U2_SCALING_VALUE_11N_5G",          0x20038DAC, PHY_REG_BANK2 },
    { "U1_SCALING_VALUE_11N40M_5G",       0x20038DB0, PHY_REG_BANK2 },
    { "U2_SCALING_VALUE_11N40M_5G",       0x20038DB4, PHY_REG_BANK2 },
    { "U3_SCALING_VALUE_11N40M_5G",       0x20038DB8, PHY_REG_BANK2 },
    { "U3_SCALING_VALUE_11N40M",          0x20038DBC, PHY_REG_BANK2 },
    { "U1_SCALING_VALUE_11AC",            0x20038DC0, PHY_REG_BANK2 },
    { "U2_SCALING_VALUE_11AC",            0x20038DC4, PHY_REG_BANK2 },
    { "U3_SCALING_VALUE_11AC",            0x20038DC8, PHY_REG_BANK2 },
    { "FFT_WIN_OFFSET",                   0x20038DCC, PHY_REG_BANK2 },
    { "DC_CORRECTION_ENABLE",             0x20038DD0, PHY_REG_BANK2 },
    { "DC_START_SAMPLES",                 0x20038DD4, PHY_REG_BANK2 },
    { "DC_ESTIMATION_WIN",                0x20038DD8, PHY_REG_BANK2 },
    { "IIR_SHIFT_BIT",                    0x20038DDC, PHY_REG_BANK2 },
    { "CARRIER_FREQ_FACTOR",              0x20038DE0, PHY_REG_BANK2 },
    { "LDPC_ITERT_USEMODE",               0x20038DE4, PHY_REG_BANK2 },
    { "NON_HT_RSSI_SEL",                  0x20038DF0, PHY_REG_BANK2 },
    { "PWR_DC_REMOVAL",                   0x20038DF4, PHY_REG_BANK2 },
    { "TH_BW",                            0x20038DF8, PHY_REG_BANK2 },
    { "CCA_FIL_DLY",                      0x20038DFC, PHY_REG_BANK2 },
    { "ED_HYST_TH",                       0x20038E00, PHY_REG_BANK2 },
    { "ED_HIGH_20TH",                     0x20038E04, PHY_REG_BANK2 },
    { "ED_HIGH_40TH",                     0x20038E08, PHY_REG_BANK2 },
    { "ED_LOW_TH_DSSS",                   0x20038E0C, PHY_REG_BANK2 },
    { "ED_LOW_TH_OFDM",                   0x20038E10, PHY_REG_BANK2 },
    { "ED_LOW_TH_40HT",                   0x20038E14, PHY_REG_BANK2 },
    { "ED_LOW_TH_20HTGF",                 0x20038E18, PHY_REG_BANK2 },
    { "ED_LOW_TH_40HTGF",                 0x20038E1C, PHY_REG_BANK2 },
    { "ED_LOW_TH_VHT",                    0x20038E20, PHY_REG_BANK2 },
    { "FREE_ACCUM_WIN",                   0x20038E24, PHY_REG_BANK2 },
    { "CCA_11B_HEADER_DELAY",             0x20038E28, PHY_REG_BANK2 },
    { "CCA_OFDM_HEADER_DELAY",            0x20038E2C, PHY_REG_BANK2 },
    { "CCA_DET",                          0x20038E30, PHY_REG_BANK2 },
    { "PRI20_IDLE_PWR",                   0x20038E34, PHY_REG_BANK2 },
    { "PRI40_IDLE_PWR",                   0x20038E38, PHY_REG_BANK2 },
    { "PRI80_IDLE_PWR",                   0x20038E3C, PHY_REG_BANK2 },
    { "RADARCONTROLREGISTER2",            0x20038E44, PHY_REG_BANK2 },
    { "RADARCONTROLREGISTER3",            0x20038E48, PHY_REG_BANK2 },
    { "RADARCONTROLREGISTER4",            0x20038E4C, PHY_REG_BANK2 },
    { "RADARCONTROLREGISTER8",            0x20038E5C, PHY_REG_BANK2 },
    { "RADARCONTROLREGISTER9",            0x20038E60, PHY_REG_BANK2 },
    { "RADARCONTROLREGISTER10",           0x20038E64, PHY_REG_BANK2 },
    { "RADARCONTROLREGISTER11",           0x20038E68, PHY_REG_BANK2 },
    { "OFDM_80M_COEF0_1",                 0x20038E80, PHY_REG_BANK2 },
    { "OFDM_80M_COEF2_3",                 0x20038E84, PHY_REG_BANK2 },
    { "OFDM_80M_COEF4_5",                 0x20038E88, PHY_REG_BANK2 },
    { "OFDM_80M_COEF6_7",                 0x20038E8C, PHY_REG_BANK2 },
    { "OFDM_80M_COEF8_9",                 0x20038E90, PHY_REG_BANK2 },
    { "OFDM_80M_COEF10_11",               0x20038E94, PHY_REG_BANK2 },
    { "OFDM_80M_COEF12_13",               0x20038E98, PHY_REG_BANK2 },
    { "OFDM_80M_COEF14_15",               0x20038E9C, PHY_REG_BANK2 },
    { "OFDM_80M_COEF16",                  0x20038EA0, PHY_REG_BANK2 },
    { "OFDM_40M_COEF0_1",                 0x20038EA4, PHY_REG_BANK2 },
    { "OFDM_40M_COEF2_3",                 0x20038EA8, PHY_REG_BANK2 },
    { "OFDM_40M_COEF4_5",                 0x20038EAC, PHY_REG_BANK2 },
    { "OFDM_40M_COEF6_7",                 0x20038EB0, PHY_REG_BANK2 },
    { "OFDM_40M_COEF8_9",                 0x20038EB4, PHY_REG_BANK2 },
    { "OFDM_40M_COEF10_11",               0x20038EB8, PHY_REG_BANK2 },
    { "OFDM_40M_COEF12_13",               0x20038EBC, PHY_REG_BANK2 },
    { "OFDM_40M_COEF14_15",               0x20038EC0, PHY_REG_BANK2 },
    { "OFDM_40M_COEF16_17",               0x20038EC4, PHY_REG_BANK2 },
    { "OFDM_40M_COEF18_19",               0x20038EC8, PHY_REG_BANK2 },
    { "OFDM_40M_COEF20_21",               0x20038ECC, PHY_REG_BANK2 },
    { "OFDM_40M_COEF22_23",               0x20038ED0, PHY_REG_BANK2 },
    { "OFDM_40M_COEF24_25",               0x20038ED4, PHY_REG_BANK2 },
    { "OFDM_40M_COEF26_27",               0x20038ED8, PHY_REG_BANK2 },
    { "OFDM_40M_COEF28_29",               0x20038EDC, PHY_REG_BANK2 },
    { "OFDM_40M_COEF30_31",               0x20038EE0, PHY_REG_BANK2 },
    { "OFDM_40M_COEF32",                  0x20038EE4, PHY_REG_BANK2 },
    { "OFDM_20M_COEF0_1",                 0x20038EE8, PHY_REG_BANK2 },
    { "OFDM_20M_COEF2_3",                 0x20038EEC, PHY_REG_BANK2 },
    { "OFDM_20M_COEF4_5",                 0x20038EF0, PHY_REG_BANK2 },
    { "OFDM_20M_COEF6_7",                 0x20038EF4, PHY_REG_BANK2 },
    { "OFDM_20M_COEF8_9",                 0x20038EF8, PHY_REG_BANK2 },
    { "OFDM_20M_COEF10_11",               0x20038EFC, PHY_REG_BANK2 },
    { "OFDM_20M_COEF12_13",               0x20038F00, PHY_REG_BANK2 },
    { "OFDM_20M_COEF14_15",               0x20038F04, PHY_REG_BANK2 },
    { "OFDM_20M_COEF16_17",               0x20038F08, PHY_REG_BANK2 },
    { "OFDM_20M_COEF18_19",               0x20038F0C, PHY_REG_BANK2 },
    { "OFDM_20M_COEF20_21",               0x20038F10, PHY_REG_BANK2 },
    { "OFDM_20M_COEF22_23",               0x20038F14, PHY_REG_BANK2 },
    { "OFDM_20M_COEF24_25",               0x20038F18, PHY_REG_BANK2 },
    { "OFDM_20M_COEF26_27",               0x20038F1C, PHY_REG_BANK2 },
    { "OFDM_20M_COEF28_29",               0x20038F20, PHY_REG_BANK2 },
    { "OFDM_20M_COEF30_31",               0x20038F24, PHY_REG_BANK2 },
    { "OFDM_20M_COEF32_33",               0x20038F28, PHY_REG_BANK2 },
    { "OFDM_20M_COEF34_35",               0x20038F2C, PHY_REG_BANK2 },
    { "OFDM_20M_COEF36_37",               0x20038F30, PHY_REG_BANK2 },
    { "OFDM_20M_COEF38_39",               0x20038F34, PHY_REG_BANK2 },
    { "OFDM_20M_COEF40_41",               0x20038F38, PHY_REG_BANK2 },
    { "OFDM_20M_COEF42_43",               0x20038F3C, PHY_REG_BANK2 },
    { "OFDM_20M_COEF44_45",               0x20038F40, PHY_REG_BANK2 },
    { "OFDM_20M_COEF46_47",               0x20038F44, PHY_REG_BANK2 },
    { "OFDM_20M_COEF48_49",               0x20038F48, PHY_REG_BANK2 },
    { "OFDM_20M_COEF50_51",               0x20038F4C, PHY_REG_BANK2 },
    { "OFDM_20M_COEF52_53",               0x20038F50, PHY_REG_BANK2 },
    { "OFDM_20M_COEF54_55",               0x20038F54, PHY_REG_BANK2 },
    { "OFDM_20M_COEF56_57",               0x20038F58, PHY_REG_BANK2 },
    { "OFDM_20M_COEF58_59",               0x20038F5C, PHY_REG_BANK2 },
    { "OFDM_20M_COEF60_61",               0x20038F60, PHY_REG_BANK2 },
    { "OFDM_20M_COEF62_63",               0x20038F64, PHY_REG_BANK2 },
    { "OFDM_20M_COEF64",                  0x20038F68, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF0_1",              0x20038F6C, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF2_3",              0x20038F70, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF4_5",              0x20038F74, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF6_7",              0x20038F78, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF8_9",              0x20038F7C, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF10_11",            0x20038F80, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF12_13",            0x20038F84, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF14_15",            0x20038F88, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF16_17",            0x20038F8C, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF18_19",            0x20038F90, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF20_21",            0x20038F94, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF22_23",            0x20038F98, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF24_25",            0x20038F9C, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF26_27",            0x20038FA0, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF28_29",            0x20038FA4, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF30_31",            0x20038FA8, PHY_REG_BANK2 },
    { "ELEVENB_40M_COEF32",               0x20038FAC, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER00",              0x20038FB0, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER01",              0x20038FB4, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER10",              0x20038FB8, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER11",              0x20038FBC, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER20",              0x20038FC0, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER21",              0x20038FC4, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER30",              0x20038FD0, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER31",              0x20038FD4, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER40",              0x20038FD8, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER41",              0x20038FDC, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER50",              0x20038FE0, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER51",              0x20038FE4, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER60",              0x20038FE8, PHY_REG_BANK2 },
    { "RADARTYPEREGISTER61",              0x20038FEC, PHY_REG_BANK2 },
    { "CAL0_ACCUMI",                      0x20039000, PHY_REG_BANK3 },
    { "CAL1_ACCUMI",                      0x20039004, PHY_REG_BANK3 },
    { "CAL2_ACCUMI",                      0x20039008, PHY_REG_BANK3 },
    { "CAL3_ACCUMI",                      0x2003900C, PHY_REG_BANK3 },
    { "CAL4_ACCUMI",                      0x20039010, PHY_REG_BANK3 },
    { "CAL5_ACCUMI",                      0x20039014, PHY_REG_BANK3 },
    { "CAL6_ACCUMI",                      0x20039018, PHY_REG_BANK3 },
    { "CAL7_ACCUMI",                      0x2003901C, PHY_REG_BANK3 },
    { "CAL8_ACCUMI",                      0x20039020, PHY_REG_BANK3 },
    { "CAL9_ACCUMI",                      0x20039024, PHY_REG_BANK3 },
    { "CAL10_ACCUMI",                     0x20039028, PHY_REG_BANK3 },
    { "CAL11_ACCUMI",                     0x2003902C, PHY_REG_BANK3 },
    { "CAL12_ACCUMI",                     0x20039030, PHY_REG_BANK3 },
    { "CAL13_ACCUMI",                     0x20039034, PHY_REG_BANK3 },
    { "CAL14_ACCUMI",                     0x20039038, PHY_REG_BANK3 },
    { "CAL15_ACCUMI",                     0x2003903C, PHY_REG_BANK3 },
    { "CAL16_ACCUMI",                     0x20039040, PHY_REG_BANK3 },
    { "CAL17_ACCUMI",                     0x20039044, PHY_REG_BANK3 },
    { "CAL18_ACCUMI",                     0x20039048, PHY_REG_BANK3 },
    { "CAL19_ACCUMI",                     0x2003904C, PHY_REG_BANK3 },
    { "CAL20_ACCUMI",                     0x20039050, PHY_REG_BANK3 },
    { "CAL21_ACCUMI",                     0x20039054, PHY_REG_BANK3 },
    { "CAL22_ACCUMI",                     0x20039058, PHY_REG_BANK3 },
    { "CAL23_ACCUMI",                     0x2003905C, PHY_REG_BANK3 },
    { "CAL24_ACCUMI",                     0x20039060, PHY_REG_BANK3 },
    { "CAL25_ACCUMI",                     0x20039064, PHY_REG_BANK3 },
    { "CAL26_ACCUMI",                     0x20039068, PHY_REG_BANK3 },
    { "CAL27_ACCUMI",                     0x2003906C, PHY_REG_BANK3 },
    { "CAL28_ACCUMI",                     0x20039070, PHY_REG_BANK3 },
    { "CAL29_ACCUMI",                     0x20039074, PHY_REG_BANK3 },
    { "CAL30_ACCUMI",                     0x20039078, PHY_REG_BANK3 },
    { "CAL31_ACCUMI",                     0x2003907C, PHY_REG_BANK3 },
    { "CAL0_ACCUMQ",                      0x20039080, PHY_REG_BANK3 },
    { "CAL1_ACCUMQ",                      0x20039084, PHY_REG_BANK3 },
    { "CAL2_ACCUMQ",                      0x20039088, PHY_REG_BANK3 },
    { "CAL3_ACCUMQ",                      0x2003908C, PHY_REG_BANK3 },
    { "CAL4_ACCUMQ",                      0x20039090, PHY_REG_BANK3 },
    { "CAL5_ACCUMQ",                      0x20039094, PHY_REG_BANK3 },
    { "CAL6_ACCUMQ",                      0x20039098, PHY_REG_BANK3 },
    { "CAL7_ACCUMQ",                      0x2003909C, PHY_REG_BANK3 },
    { "CAL8_ACCUMQ",                      0x200390A0, PHY_REG_BANK3 },
    { "CAL9_ACCUMQ",                      0x200390A4, PHY_REG_BANK3 },
    { "CAL10_ACCUMQ",                     0x200390A8, PHY_REG_BANK3 },
    { "CAL11_ACCUMQ",                     0x200390AC, PHY_REG_BANK3 },
    { "CAL12_ACCUMQ",                     0x200390B0, PHY_REG_BANK3 },
    { "CAL13_ACCUMQ",                     0x200390B4, PHY_REG_BANK3 },
    { "CAL14_ACCUMQ",                     0x200390B8, PHY_REG_BANK3 },
    { "CAL15_ACCUMQ",                     0x200390BC, PHY_REG_BANK3 },
    { "CAL16_ACCUMQ",                     0x200390C0, PHY_REG_BANK3 },
    { "CAL17_ACCUMQ",                     0x200390C4, PHY_REG_BANK3 },
    { "CAL18_ACCUMQ",                     0x200390C8, PHY_REG_BANK3 },
    { "CAL19_ACCUMQ",                     0x200390CC, PHY_REG_BANK3 },
    { "CAL20_ACCUMQ",                     0x200390D0, PHY_REG_BANK3 },
    { "CAL21_ACCUMQ",                     0x200390D4, PHY_REG_BANK3 },
    { "CAL22_ACCUMQ",                     0x200390D8, PHY_REG_BANK3 },
    { "CAL23_ACCUMQ",                     0x200390DC, PHY_REG_BANK3 },
    { "CAL24_ACCUMQ",                     0x200390E0, PHY_REG_BANK3 },
    { "CAL25_ACCUMQ",                     0x200390E4, PHY_REG_BANK3 },
    { "CAL26_ACCUMQ",                     0x200390E8, PHY_REG_BANK3 },
    { "CAL27_ACCUMQ",                     0x200390EC, PHY_REG_BANK3 },
    { "CAL28_ACCUMQ",                     0x200390F0, PHY_REG_BANK3 },
    { "CAL29_ACCUMQ",                     0x200390F4, PHY_REG_BANK3 },
    { "CAL30_ACCUMQ",                     0x200390F8, PHY_REG_BANK3 },
    { "CAL31_ACCUMQ",                     0x200390FC, PHY_REG_BANK3 },
    { "RPT_ACCUM",                        0x20039100, PHY_REG_BANK3 },
    { "RES_CALI",                         0x20039120, PHY_REG_BANK3 },
    { "SA_CFG",                           0x20039124, PHY_REG_BANK3 },
    { "CAL_CFG0",                         0x20039128, PHY_REG_BANK3 },
    { "CAL_CFG1",                         0x2003912C, PHY_REG_BANK3 },
    { "CAL_CFG2",                         0x20039130, PHY_REG_BANK3 },
    { "CAL_CFG3",                         0x20039134, PHY_REG_BANK3 },
    { "CAL_CFG4",                         0x20039138, PHY_REG_BANK3 },
    { "CAL_CFG5",                         0x2003913C, PHY_REG_BANK3 },
    { "CAL_CFG6",                         0x20039140, PHY_REG_BANK3 },
    { "CAL_CFG7",                         0x20039144, PHY_REG_BANK3 },
    { "CAL_CFG8",                         0x20039148, PHY_REG_BANK3 },
    { "CAL_CFG9",                         0x2003914C, PHY_REG_BANK3 },
    { "CAL_CFG10",                        0x20039150, PHY_REG_BANK3 },
    { "CAL_CFG11",                        0x20039154, PHY_REG_BANK3 },
    { "CAL_CFG12",                        0x20039158, PHY_REG_BANK3 },
    { "CAL_CFG13",                        0x2003915C, PHY_REG_BANK3 },
    { "CAL_CFG14",                        0x20039160, PHY_REG_BANK3 },
    { "CAL_CFG15",                        0x20039164, PHY_REG_BANK3 },
    { "CAL_CFG16",                        0x20039168, PHY_REG_BANK3 },
    { "CAL_CFG17",                        0x2003916C, PHY_REG_BANK3 },
    { "TX_POWER_ACCUM_DELAY",             0x20039170, PHY_REG_BANK3 },
    { "TX_POWER_RPT_OVER",                0x20039174, PHY_REG_BANK3 },
    { "EXT_IN_ADC_CLOCK",                 0x20039178, PHY_REG_BANK3 },
    { "FIFOCTRL",                         0x200391D0, PHY_REG_BANK3 },
    { "FIFO_LATENCY",                     0x200391D4, PHY_REG_BANK3 },
    { "FIFO_ALARM",                       0x200391D8, PHY_REG_BANK3 },
    { "SAMPLE_CFG",                       0x200391DC, PHY_REG_BANK3 },
    { "REAL_SAMPLE_CFG",                  0x200391E0, PHY_REG_BANK3 },
    { "TXPOST_0COEFF",                    0x20039200, PHY_REG_BANK3 },
    { "TXPOST_1COEFF",                    0x20039204, PHY_REG_BANK3 },
    { "INTXPOSTFILTBYPASSN",              0x20039208, PHY_REG_BANK3 },
    { "DELAY_TIME_11B",                   0x2003920C, PHY_REG_BANK3 },
    { "CFG1_11B",                         0x20039210, PHY_REG_BANK3 },
    { "CFG2_11B",                         0x20039214, PHY_REG_BANK3 },
    { "CFG3_11B",                         0x20039218, PHY_REG_BANK3 },
    { "CFG4_11B",                         0x2003921C, PHY_REG_BANK3 },
    { "CFG5_11B",                         0x20039220, PHY_REG_BANK3 },
    { "CFG6_11B",                         0x20039224, PHY_REG_BANK3 },
    { "CFG7_11B",                         0x20039228, PHY_REG_BANK3 },
    { "CFG8_11B",                         0x2003922C, PHY_REG_BANK3 },
    { "CFG9_11B",                         0x20039230, PHY_REG_BANK3 },
    { "CFG10_11B",                        0x20039234, PHY_REG_BANK3 },
    { "CFG11_11B",                        0x20039238, PHY_REG_BANK3 },
    { "ONLINE_CFG0",                      0x20039244, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_1",                 0x20039248, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_2",                 0x2003924C, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_3",                 0x20039250, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_4",                 0x20039254, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_5",                 0x20039258, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_6",                 0x2003925C, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_7",                 0x20039260, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_8",                 0x20039264, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_9",                 0x20039268, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_10",                0x2003926C, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_11",                0x20039270, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_12",                0x20039274, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_13",                0x20039278, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_14",                0x2003927C, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_15",                0x20039280, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_16",                0x20039284, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_17",                0x20039288, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_18",                0x2003928C, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_19",                0x20039290, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_20",                0x20039294, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_21",                0x20039298, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_22",                0x2003929C, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_23",                0x200392A0, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_24",                0x200392A4, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_25",                0x200392A8, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_26",                0x200392AC, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_27",                0x200392B0, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_28",                0x200392B4, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_29",                0x200392B8, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_30",                0x200392BC, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_31",                0x200392C0, PHY_REG_BANK3 },
    { "LNA_RF_DBBCOMP_32",                0x200392C4, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_1",                 0x200392C8, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_2",                 0x200392CC, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_3",                 0x200392D0, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_4",                 0x200392D4, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_5",                 0x200392D8, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_6",                 0x200392DC, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_7",                 0x200392E0, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_8",                 0x200392E4, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_9",                 0x200392E8, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_10",                0x200392EC, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_11",                0x200392F0, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_12",                0x200392F4, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_13",                0x200392F8, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_14",                0x200392FC, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_15",                0x20039300, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_16",                0x20039304, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_17",                0x20039308, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_18",                0x2003930C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_19",                0x20039310, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_20",                0x20039314, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_21",                0x20039318, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_22",                0x2003931C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_23",                0x20039320, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_24",                0x20039324, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_25",                0x20039328, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_26",                0x2003932C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_27",                0x20039330, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_28",                0x20039334, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_29",                0x20039338, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_30",                0x2003933C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_31",                0x20039340, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_32",                0x20039344, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_33",                0x20039348, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_34",                0x2003934C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_35",                0x20039350, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_36",                0x20039354, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_37",                0x20039358, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_38",                0x2003935C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_39",                0x20039360, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_40",                0x20039364, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_41",                0x20039368, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_42",                0x2003936C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_43",                0x20039370, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_44",                0x20039374, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_45",                0x20039378, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_46",                0x2003937C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_47",                0x20039380, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_48",                0x20039384, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_49",                0x20039388, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_50",                0x2003938C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_51",                0x20039390, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_52",                0x20039394, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_53",                0x20039398, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_54",                0x2003939C, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_55",                0x200393A0, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_56",                0x200393A4, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_57",                0x200393A8, PHY_REG_BANK3 },
    { "LNA_DC_DBBCOMP_58",                0x200393AC, PHY_REG_BANK3 },
    { "TX_UP_DOWN_TIME",                  0x20039400, PHY_REG_BANK4 },
    { "RX_UP_DOWN_TIME",                  0x20039404, PHY_REG_BANK4 },
    { "RF_CTRL",                          0x20039408, PHY_REG_BANK4 },
    { "RF_EN_ADDR",                       0x2003940C, PHY_REG_BANK4 },
    { "TX_RX_DLY",                        0x20039410, PHY_REG_BANK4 },
    { "PA_DLY",                           0x20039414, PHY_REG_BANK4 },
    { "RX_TX_STATE",                      0x20039418, PHY_REG_BANK4 },
    { "FEM_SEL_CTRL",                     0x2003941C, PHY_REG_BANK4 },
    { "SMART_ANT_DELAY",                  0x20039420, PHY_REG_BANK4 },
    { "PA_GAIN_CODE",                     0x20039424, PHY_REG_BANK4 },
    { "PA_REG_ADDR",                      0x20039428, PHY_REG_BANK4 },
    { "UPC_ADDR",                         0x2003942C, PHY_REG_BANK4 },
    { "UPC1_01_DATA",                     0x20039430, PHY_REG_BANK4 },
    { "UPC1_02_DATA",                     0x20039434, PHY_REG_BANK4 },
    { "UPC1_03_DATA",                     0x20039438, PHY_REG_BANK4 },
    { "UPC1_04_DATA",                     0x2003943C, PHY_REG_BANK4 },
    { "UPC2_01_DATA",                     0x20039440, PHY_REG_BANK4 },
    { "UPC2_02_DATA",                     0x20039444, PHY_REG_BANK4 },
    { "UPC2_03_DATA",                     0x20039448, PHY_REG_BANK4 },
    { "UPC2_04_DATA",                     0x2003944C, PHY_REG_BANK4 },
    { "DAC_ADDR_1",                       0x20039450, PHY_REG_BANK4 },
    { "DAC_DATA",                         0x20039454, PHY_REG_BANK4 },
    { "LPF_DATA",                         0x20039458, PHY_REG_BANK4 },
    { "DYN_TX_POWER_CALI",                0x2003945C, PHY_REG_BANK4 },
    { "DYN_TX_POWER_RPT",                 0x20039460, PHY_REG_BANK4 },
    { "DYN_TX_POWER_DELAY",               0x20039464, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ADDR_1",              0x20039468, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ADDR_2",              0x2003946C, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ADDR_3",              0x20039470, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ADDR_4",              0x20039474, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ON_DATA_1",           0x20039478, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ON_DATA_2",           0x2003947C, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ON_DATA_3",           0x20039480, PHY_REG_BANK4 },
    { "DYN_TX_POWER_ON_DATA_4",           0x20039484, PHY_REG_BANK4 },
    { "DYN_TX_POWER_OFF_DATA_1",          0x20039488, PHY_REG_BANK4 },
    { "DYN_TX_POWER_OFF_DATA_2",          0x2003948C, PHY_REG_BANK4 },
    { "DYN_TX_POWER_OFF_DATA_3",          0x20039490, PHY_REG_BANK4 },
    { "DYN_TX_POWER_OFF_DATA_4",          0x20039494, PHY_REG_BANK4 },
    { "DYN_CHN_CTRL",                     0x20039498, PHY_REG_BANK4 },
    { "DYN_CHN_ADDR_1",                   0x2003949C, PHY_REG_BANK4 },
    { "DYN_CHN_ADDR_2",                   0x200394A0, PHY_REG_BANK4 },
    { "DYN_CHN_ADDR_3",                   0x200394A4, PHY_REG_BANK4 },
    { "DYN_CHN_ADDR_4",                   0x200394A8, PHY_REG_BANK4 },
    { "DYN_CHN_ON_DATA_1",                0x200394AC, PHY_REG_BANK4 },
    { "DYN_CHN_ON_DATA_2",                0x200394B0, PHY_REG_BANK4 },
    { "DYN_CHN_ON_DATA_3",                0x200394B4, PHY_REG_BANK4 },
    { "DYN_CHN_ON_DATA_4",                0x200394B8, PHY_REG_BANK4 },
    { "DYN_CHN_OFF_DATA_1",               0x200394BC, PHY_REG_BANK4 },
    { "DYN_CHN_OFF_DATA_2",               0x200394C0, PHY_REG_BANK4 },
    { "DYN_CHN_OFF_DATA_3",               0x200394C4, PHY_REG_BANK4 },
    { "DYN_CHN_OFF_DATA_4",               0x200394C8, PHY_REG_BANK4 },
    { "RX_DC_ADDR",                       0x2003956C, PHY_REG_BANK4 },
    { "RF_REG_WR_DELAY_1",                0x20039570, PHY_REG_BANK4 },
    { "RF_REG_WR_DELAY_2",                0x20039574, PHY_REG_BANK4 },
    { "TPC_CTRL",                         0x20039578, PHY_REG_BANK4 },
    { "RF_IF_CTRL",                       0x2003957C, PHY_REG_BANK4 },
    { "WR_RF_DC_VLD",                     0x20039580, PHY_REG_BANK4 },
    { "WR_RF_DC0_VALUE",                  0x20039584, PHY_REG_BANK4 },
    { "WR_RF_DC1_VALUE",                  0x20039588, PHY_REG_BANK4 },
    { "WR_RF_DC_DLY",                     0x2003958C, PHY_REG_BANK4 },
    { "WR_RF_TEMP_VLD",                   0x20039590, PHY_REG_BANK4 },
    { "WR_RF_TEMP_VALUE",                 0x20039594, PHY_REG_BANK4 },
    { "WR_RF_TEMP_DLY",                   0x20039598, PHY_REG_BANK4 },
    { "ADDA_DELAY",                       0x2003959C, PHY_REG_BANK4 },
};

OAL_STATIC oam_reg_cfg_stru soc_reg[] = {
    { "SYS_CTL_ID",                  0x20000000, GLB_SYS_CTL_REGBANK },
    { "PAGE0_KEY",                   0x20000004, GLB_SYS_CTL_REGBANK },
    { "PAGE1_KEY",                   0x20000008, GLB_SYS_CTL_REGBANK },
    { "CHIP_ID_L",                   0x2000000C, GLB_SYS_CTL_REGBANK },
    { "CHIP_ID_H",                   0x20000010, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_0",                    0x20000014, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_1",                    0x20000018, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_2",                    0x2000001C, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_3",                    0x20000020, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_4",                    0x20000024, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_5",                    0x20000028, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_6",                    0x2000002C, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_7",                    0x20000030, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_8",                    0x20000034, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_9",                    0x20000038, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_10",                   0x2000003C, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_11",                   0x20000040, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_12",                   0x20000044, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_13",                   0x20000048, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_14",                   0x2000004C, GLB_SYS_CTL_REGBANK },
    { "DIE_ID_15",                   0x20000050, GLB_SYS_CTL_REGBANK },
    { "TEST_VALUE",                  0x20000100, GLB_SYS_CTL_REGBANK },
    { "TEST_RESULT",                 0x20000104, GLB_SYS_CTL_REGBANK },
    { "GP_REG0",                     0x20000108, GLB_SYS_CTL_REGBANK },
    { "GP_REG1",                     0x2000010C, GLB_SYS_CTL_REGBANK },
    { "GP_REG2",                     0x20000110, GLB_SYS_CTL_REGBANK },
    { "GP_REG3",                     0x20000114, GLB_SYS_CTL_REGBANK },
    { "AON_CLK_MUX0",                0x20000118, GLB_SYS_CTL_REGBANK },
    { "AON_CLK_MUX1",                0x2000011C, GLB_SYS_CTL_REGBANK },
    { "VSET_TCXO",                   0x20000120, GLB_SYS_CTL_REGBANK },
    { "RF_PLL_CLK_EN",               0x20000124, GLB_SYS_CTL_REGBANK },
    { "RF_PLL_RST_CFG",              0x20000128, GLB_SYS_CTL_REGBANK },
    { "PIN_MUX0",                    0x2000012C, GLB_SYS_CTL_REGBANK },
    { "PIN_MUX1",                    0x20000130, GLB_SYS_CTL_REGBANK },
    { "PIN_PULL_CTL0",               0x20000134, GLB_SYS_CTL_REGBANK },
    { "PIN_PULL_CTL1",               0x20000138, GLB_SYS_CTL_REGBANK },
    { "FEM_POLA_SEL",                0x2000013C, GLB_SYS_CTL_REGBANK },
    { "FEM_CTL_SEL",                 0x20000140, GLB_SYS_CTL_REGBANK },
    { "CFG_DIAG_SEL",                0x20000144, GLB_SYS_CTL_REGBANK },
    { "PIN_DRIVE_CTL0",              0x20000148, GLB_SYS_CTL_REGBANK },
    { "PIN_DRIVE_CTL1",              0x2000014C, GLB_SYS_CTL_REGBANK },
    { "PIN_SLEW_CTL",                0x20000150, GLB_SYS_CTL_REGBANK },
    { "HD_WAKE_SLEEP_EN",            0x20000200, GLB_SYS_CTL_REGBANK },
    { "SOFT_SLEEP_TIME_L",           0x20000204, GLB_SYS_CTL_REGBANK },
    { "SOFT_SLEEP_TIME_H",           0x20000208, GLB_SYS_CTL_REGBANK },
    { "SOFT_SLEEP_EN",               0x2000020C, GLB_SYS_CTL_REGBANK },
    { "CLDO_HV",                     0x20000210, GLB_SYS_CTL_REGBANK },
    { "CLDO_LV",                     0x20000214, GLB_SYS_CTL_REGBANK },
    { "WAKE_TIME0",                  0x20000218, GLB_SYS_CTL_REGBANK },
    { "WAKE_TIME1",                  0x2000021C, GLB_SYS_CTL_REGBANK },
    { "SLEEP_TIME0",                 0x20000220, GLB_SYS_CTL_REGBANK },
    { "SLEEP_TIME1",                 0x20000224, GLB_SYS_CTL_REGBANK },
    { "PCIE_EN_CFG",                 0x20000228, GLB_SYS_CTL_REGBANK },
    { "PRECHARGE_EN",                0x2000022C, GLB_SYS_CTL_REGBANK },
    { "PERST_L_EN",                  0x20000230, GLB_SYS_CTL_REGBANK },
    { "SSC_CTL",                     0x20000240, GLB_SYS_CTL_REGBANK },
    { "TEST_CLK_CTL",                0x20000244, GLB_SYS_CTL_REGBANK },
    { "BOOT_MAN0",                   0x20000248, GLB_SYS_CTL_REGBANK },
    { "BOOT_MAN1",                   0x2000024C, GLB_SYS_CTL_REGBANK },
    { "BOOT_MAN2",                   0x20000250, GLB_SYS_CTL_REGBANK },
    { "BOOT_MAN3",                   0x20000254, GLB_SYS_CTL_REGBANK },
    { "BOOT_MAN4",                   0x20000258, GLB_SYS_CTL_REGBANK },
    { "BOOT_MAN5",                   0x2000025C, GLB_SYS_CTL_REGBANK },
    { "BOOT_MAN_SEL",                0x20000260, GLB_SYS_CTL_REGBANK },
    { "SOFT_RESET",                  0x20000264, GLB_SYS_CTL_REGBANK },
    { "SOFT_RESET_CFG",              0x20000268, GLB_SYS_CTL_REGBANK },
    { "CLDO_RTS_SEL",                0x2000026C, GLB_SYS_CTL_REGBANK },
    { "VSET_CLDO_MAN",               0x20000280, GLB_SYS_CTL_REGBANK },
    { "VSET_CLDO_SEL",               0x20000284, GLB_SYS_CTL_REGBANK },
    { "AON_CRG_STS",                 0x20000288, GLB_SYS_CTL_REGBANK },
    { "SSI_BOOT_FORCE_RESET",        0x2000028C, GLB_SYS_CTL_REGBANK },
    { "DBG_GLB_STS",                 0x20000290, GLB_SYS_CTL_REGBANK },
    { "DIAG_STS",                    0x20000294, GLB_SYS_CTL_REGBANK },
    { "SYS_CTL_ID",                  0x20004000, PMU_CMU_CTL_REGBANK },
    { "PAGE0_KEY",                   0x20004004, PMU_CMU_CTL_REGBANK },
    { "PAGE1_KEY",                   0x20004008, PMU_CMU_CTL_REGBANK },
    { "TEST_VALUE",                  0x20004100, PMU_CMU_CTL_REGBANK },
    { "TEST_RESULT",                 0x20004104, PMU_CMU_CTL_REGBANK },
    { "CLDO_CFG",                    0x20004108, PMU_CMU_CTL_REGBANK },
    { "RFLD1_CFG",                   0x2000410C, PMU_CMU_CTL_REGBANK },
    { "RFLD2_CFG",                   0x20004110, PMU_CMU_CTL_REGBANK },
    { "PMU_CFG",                     0x20004114, PMU_CMU_CTL_REGBANK },
    { "PMU_LDO_EN_SEL",              0x20004118, PMU_CMU_CTL_REGBANK },
    { "PMU_LDO_EN_STS",              0x2000411C, PMU_CMU_CTL_REGBANK },
    { "PMU_LDO_EN_MAN_STS",          0x20004120, PMU_CMU_CTL_REGBANK },
    { "LDO_EN_DELAY_CNT",            0x20004124, PMU_CMU_CTL_REGBANK },
    { "PMU_RESET",                   0x20004128, PMU_CMU_CTL_REGBANK },
    { "PMU_250K_EN",                 0x2000412C, PMU_CMU_CTL_REGBANK },
    { "EFUSE_VALUE_SEL",             0x20004140, PMU_CMU_CTL_REGBANK },
    { "EFUSE_VALUE_STS",             0x20004144, PMU_CMU_CTL_REGBANK },
    { "EFUSE_VALUE_MAN",             0x20004148, PMU_CMU_CTL_REGBANK },
    { "PMU_ERR_INT_EN",              0x20004160, PMU_CMU_CTL_REGBANK },
    { "PMU_ERR_INT_STS",             0x20004164, PMU_CMU_CTL_REGBANK },
    { "PMU_ERR_INT_CLR",             0x20004168, PMU_CMU_CTL_REGBANK },
    { "BUCK_VSET",                   0x20004200, PMU_CMU_CTL_REGBANK },
    { "BUCK_FRQ_SEL",                0x20004204, PMU_CMU_CTL_REGBANK },
    { "BUCK_RESERVED",               0x20004208, PMU_CMU_CTL_REGBANK },
    { "BUCK_SS_SEL",                 0x2000420C, PMU_CMU_CTL_REGBANK },
    { "BUCK_CTL",                    0x20004210, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_0",                   0x20004300, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_1",                   0x20004304, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_2",                   0x20004308, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_3",                   0x20004310, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_4",                   0x20004314, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_5",                   0x20004318, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_6",                   0x2000431C, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_7",                   0x20004320, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_8",                   0x20004324, PMU_CMU_CTL_REGBANK },
    { "CMU_CFG_9",                   0x20004328, PMU_CMU_CTL_REGBANK },
    { "XLDO_VSET_CFG",               0x2000432C, PMU_CMU_CTL_REGBANK },
    { "CMU_LOCK_STS",                0x20004330, PMU_CMU_CTL_REGBANK },
    { "UNLOCK_INT_CFG",              0x20004334, PMU_CMU_CTL_REGBANK },
    { "UNLOCK_INT_CLR",              0x20004338, PMU_CMU_CTL_REGBANK },
    { "SYS_CTL_ID",                  0x20008000, RF_ABB_CTL_REGBANK },
    { "PAGE0_KEY",                   0x20008004, RF_ABB_CTL_REGBANK },
    { "PAGE1_KEY",                   0x20008008, RF_ABB_CTL_REGBANK },
    { "TEST_VALUE",                  0x20008100, RF_ABB_CTL_REGBANK },
    { "TEST_RESULT",                 0x20008104, RF_ABB_CTL_REGBANK },
    { "RF_AGC_SEL",                  0x20008200, RF_ABB_CTL_REGBANK },
    { "RF_AGC_SOC",                  0x20008204, RF_ABB_CTL_REGBANK },
    { "RF_AGC_STS",                  0x20008208, RF_ABB_CTL_REGBANK },
    { "RF_OVER_TEMP_PRT_EN",         0x2000820C, RF_ABB_CTL_REGBANK },
    { "RF_TEMP_TEST_TIME",           0x20008210, RF_ABB_CTL_REGBANK },
    { "RF_TEMP_INT_STS",             0x20008214, RF_ABB_CTL_REGBANK },
    { "RF_TEMP_INT_THRESH",          0x20008218, RF_ABB_CTL_REGBANK },
    { "RF_OVER_TEMP_CTL",            0x2000821C, RF_ABB_CTL_REGBANK },
    { "RF_OVER_TEMP_INT_EN",         0x20008220, RF_ABB_CTL_REGBANK },
    { "RF_OVER_TEMP_INT_STS",        0x20008224, RF_ABB_CTL_REGBANK },
    { "RF_OVER_TEMP_PRT_CLR",        0x20008228, RF_ABB_CTL_REGBANK },
    { "RF_TEMP_COMP",                0x2000822C, RF_ABB_CTL_REGBANK },
    { "RF_TEMP_FLG_STS",             0x20008230, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_0CH_SEL_L", 0x20008240, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_0CH_SEL_H", 0x20008244, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_0CH_SOC_L", 0x20008248, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_0CH_SOC_H", 0x2000824C, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_0CH_STS_L", 0x20008250, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_0CH_STS_H", 0x20008254, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_0CH_CTRL_REG_SEL", 0x20008258, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_0CH_CTRL_REG",     0x2000825C, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_0CH_CTRL_REG_STS", 0x20008260, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_CTRLIN_0CH",       0x20008264, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_0CH_CTRL_REG_SEL", 0x20008268, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_0CH_CTRL_REG",     0x2000826C, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_0CH_CTRL_REG_STS", 0x20008270, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_1CH_SEL_L", 0x20008280, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_1CH_SEL_H", 0x20008284, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_1CH_SOC_L", 0x20008288, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_1CH_SOC_H", 0x2000828C, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_1CH_STS_L", 0x20008290, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_CTRLIN_1CH_STS_H", 0x20008294, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_1CH_CTRL_REG_SEL", 0x20008298, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_1CH_CTRL_REG",     0x2000829C, RF_ABB_CTL_REGBANK },
    { "ABB_WL_ADC_1CH_CTRL_REG_STS", 0x200082A0, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_CTRLIN_1CH",       0x200082A4, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_1CH_CTRL_REG_SEL", 0x200082A8, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_1CH_CTRL_REG",     0x200082AC, RF_ABB_CTL_REGBANK },
    { "ABB_WL_DAC_1CH_CTRL_REG_STS", 0x200082B0, RF_ABB_CTL_REGBANK },
    { "WL_RF_ABB_REG_SEL",           0x200082C0, RF_ABB_CTL_REGBANK },
    { "WL_RF_ABB_REG_RST_CFG",       0x200082C4, RF_ABB_CTL_REGBANK },
    { "RF_REG_CLK_EN",               0x200082C8, RF_ABB_CTL_REGBANK },
    { "RF_REG_CLK_EN_STS",           0x200082CC, RF_ABB_CTL_REGBANK },
    { "SYS_CTL_ID",                  0x20010000, LCL_CTL_REGBANK },
    { "PAGE0_KEY",                   0x20010004, LCL_CTL_REGBANK },
    { "PAGE1_KEY",                   0x20010008, LCL_CTL_REGBANK },
    { "TEST_VALUE",                  0x20010100, LCL_CTL_REGBANK },
    { "TEST_RESULT",                 0x20010104, LCL_CTL_REGBANK },
    { "PCIE_LP_CONFIG",              0x20010200, LCL_CTL_REGBANK },
    { "PCIE_INT_CONFIG",             0x20010204, LCL_CTL_REGBANK },
    { "PCIE_MSTR_BMISC",             0x20010208, LCL_CTL_REGBANK },
    { "PCIE_MSTR_RMISC",             0x2001020C, LCL_CTL_REGBANK },
    { "PCIE_SLV_AWMISC0",            0x20010210, LCL_CTL_REGBANK },
    { "PCIE_SLV_AWMISC1",            0x20010214, LCL_CTL_REGBANK },
    { "PCIE_SLV_ARMISC0",            0x20010218, LCL_CTL_REGBANK },
    { "PCIE_SLV_ARMISC1",            0x2001021C, LCL_CTL_REGBANK },
    { "PCIE_BUS_CONFIG",             0x20010220, LCL_CTL_REGBANK },
    { "PCIE_LINK_TIMEOUT_L",         0x20010224, LCL_CTL_REGBANK },
    { "PCIE_LINK_TIMEOUT_H",         0x20010228, LCL_CTL_REGBANK },
    { "PCIE_CONFIG",                 0x2001022C, LCL_CTL_REGBANK },
    { "PCIE_NOR_STATUS",             0x20010230, LCL_CTL_REGBANK },
    { "PCIE_ABNOR_STATUS",           0x20010234, LCL_CTL_REGBANK },
    { "PCIE_ABNOR_MASK",             0x20010238, LCL_CTL_REGBANK },
    { "PCIE_ABNOR_CLR",              0x2001023C, LCL_CTL_REGBANK },
    { "PCIE_TRGT_STATUS0",           0x20010240, LCL_CTL_REGBANK },
    { "PCIE_TRGT_STATUS1",           0x20010244, LCL_CTL_REGBANK },
    { "PCIE_TRGT_STATUS2",           0x20010248, LCL_CTL_REGBANK },
    { "PCIE_RADM_STATUS0",           0x2001024C, LCL_CTL_REGBANK },
    { "PCIE_RADM_STATUS1",           0x20010250, LCL_CTL_REGBANK },
    { "PCIE_PM_STATUS",              0x20010254, LCL_CTL_REGBANK },
    { "PCIE_LTSSM_STATUS",           0x20010258, LCL_CTL_REGBANK },
    { "PCIE_DEBUG_STATUS",           0x2001025C, LCL_CTL_REGBANK },
    { "PCIE_MESS_STATUS",            0x20010260, LCL_CTL_REGBANK },
    { "PCIE_MESS_MASK",              0x20010264, LCL_CTL_REGBANK },
    { "PCIE_MESS_CLR",               0x20010268, LCL_CTL_REGBANK },
    { "PCIE_MESS_PHYLOAD0",          0x2001026C, LCL_CTL_REGBANK },
    { "PCIE_MESS_PHYLOAD1",          0x20010270, LCL_CTL_REGBANK },
    { "PCIE_MESS_PHYLOAD2",          0x20010274, LCL_CTL_REGBANK },
    { "PCIE_MESS_PHYLOAD3",          0x20010278, LCL_CTL_REGBANK },
    { "PCIE_MESS_ID",                0x2001027C, LCL_CTL_REGBANK },
    { "PCIE_PHY_CONFIG1",            0x20010280, LCL_CTL_REGBANK },
    { "PCIE_PHY_CONFIG2",            0x20010284, LCL_CTL_REGBANK },
    { "PCIE_PHY_CONFIG3",            0x20010288, LCL_CTL_REGBANK },
    { "PCIE_PHY_CONFIG4",            0x2001028C, LCL_CTL_REGBANK },
    { "PCIE_PHY_CONFIG5",            0x20010290, LCL_CTL_REGBANK },
    { "PCIE_PHY_CONFIG6",            0x20010294, LCL_CTL_REGBANK },
    { "PCIE_PHY_CONFIG7",            0x20010298, LCL_CTL_REGBANK },
    { "PCIE_PHY_CFG_ADDR",           0x2001029C, LCL_CTL_REGBANK },
    { "PCIE_PHY_WR_DATA",            0x200102A0, LCL_CTL_REGBANK },
    { "PCIE_PHY_RD_DATA",            0x200102A4, LCL_CTL_REGBANK },
    { "PCIE_PHY_CFG",                0x200102A8, LCL_CTL_REGBANK },
    { "PCIE_MEM_CONFIG",             0x200102AC, LCL_CTL_REGBANK },
    { "PCIE_STATUS0",                0x200102B0, LCL_CTL_REGBANK },
    { "PCIE_STATUS1",                0x200102B4, LCL_CTL_REGBANK },
    { "PCIE_STATUS2",                0x200102B8, LCL_CTL_REGBANK },
    { "PCIE_STATUS3",                0x200102BC, LCL_CTL_REGBANK },
    { "PCIE_STATUS4",                0x200102C0, LCL_CTL_REGBANK },
    { "PCIE_STATUS5",                0x200102C4, LCL_CTL_REGBANK },
    { "PCIE_STATUS6",                0x200102C8, LCL_CTL_REGBANK },
    { "PCIE_STATUS7",                0x200102CC, LCL_CTL_REGBANK },
    { "PCIE_STATUS8",                0x200102D0, LCL_CTL_REGBANK },
    { "PCIE_STATUS9",                0x200102D4, LCL_CTL_REGBANK },
    { "PCIE_STATUS10",               0x200102D8, LCL_CTL_REGBANK },
    { "PCIE_DIAG_SEL",               0x200102DC, LCL_CTL_REGBANK },
    { "PCIE_LOW_POWER",              0x200102E0, LCL_CTL_REGBANK },
    { "PCIE_MEM_STATUS",             0x200102E4, LCL_CTL_REGBANK },
    { "MST_PRIORITY",                0x20010300, LCL_CTL_REGBANK },
    { "SLV_PRIORITY",                0x20010304, LCL_CTL_REGBANK },
    { "INTR_STATUS",                 0x20010308, LCL_CTL_REGBANK },
    { "INTR_MASK",                   0x2001030C, LCL_CTL_REGBANK },
    { "INTR_CLR",                    0x20010310, LCL_CTL_REGBANK },
    { "MEM_RD_BYPASS",               0x20010314, LCL_CTL_REGBANK },
    { "PRECHARGE_BYAPSS",            0x20010318, LCL_CTL_REGBANK },
    { "SOFT_RESET",                  0x20010400, LCL_CTL_REGBANK },
    { "MAC_RESET_CTRL",              0x20010404, LCL_CTL_REGBANK },
    { "LOCAL_CLK_EN0",               0x20010408, LCL_CTL_REGBANK },
    { "LOCAL_CLK_EN1",               0x2001040C, LCL_CTL_REGBANK },
    { "LOCAL_CLK_EN2",               0x20010410, LCL_CTL_REGBANK },
    { "LOCAL_CLK_EN3",               0x20010414, LCL_CTL_REGBANK },
    { "LOCAL_CLK_MUX0",              0x20010418, LCL_CTL_REGBANK },
    { "LOCAL_CLK_MUX1",              0x2001041C, LCL_CTL_REGBANK },
    { "TEST_MODE",                   0x20010420, LCL_CTL_REGBANK },
    { "MAC_MEM_CONFIG",              0x20010450, LCL_CTL_REGBANK },
    { "MAC_MEM_MARGIN_ADJ",          0x20010454, LCL_CTL_REGBANK },
    { "PHY_MEM_CONFIG",              0x20010458, LCL_CTL_REGBANK },
    { "PHY_MEM_MARGIN_ADJ",          0x2001045C, LCL_CTL_REGBANK },
    { "CFG_ABB_CALI_EN",             0x20010460, LCL_CTL_REGBANK },
    { "RPT_ABB_CALI",                0x20010464, LCL_CTL_REGBANK },
    { "CFG_ABB_CALI",                0x20010468, LCL_CTL_REGBANK },
    { "TESTSTART",                   0x2001046C, LCL_CTL_REGBANK },
    { "SAMPLE_DONE",                 0x20010470, LCL_CTL_REGBANK },
    { "TESTADDRSTART",               0x20010474, LCL_CTL_REGBANK },
    { "TESTADDREND",                 0x20010478, LCL_CTL_REGBANK },
    { "TESTADDROFFSET",              0x2001047C, LCL_CTL_REGBANK },
    { "IQ_EXCHANGE",                 0x20010480, LCL_CTL_REGBANK },
    { "CLEAR_SAMPLE_DONE",           0x20010484, LCL_CTL_REGBANK },
    { "CLEAR_FIFO_ALM",              0x20010488, LCL_CTL_REGBANK },
    { "FIFO_CH0_STS",                0x2001048C, LCL_CTL_REGBANK },
    { "FIFO_CH1_STS",                0x20010490, LCL_CTL_REGBANK },
    { "TEST_CH_SEL",                 0x20010494, LCL_CTL_REGBANK },
    { "CFG_SAMPLE_IQ_SEL",           0x20010498, LCL_CTL_REGBANK },
    { "COEX_CTRL",                   0x20010500, LCL_CTL_REGBANK },
    { "COEX_TIMEOUT",                0x20010504, LCL_CTL_REGBANK },
    { "DEFAULT_SLAVE0",              0x20010508, LCL_CTL_REGBANK },
    { "DEFAULT_SLAVE1",              0x2001050C, LCL_CTL_REGBANK },
    { "DEFAULT_SLAVE2",              0x20010510, LCL_CTL_REGBANK },
    { "DEFAULT_SLAVE_CLR",           0x20010514, LCL_CTL_REGBANK },
    { "LOW_POWER_CFG0",              0x20010518, LCL_CTL_REGBANK },
    { "LOW_POWER_CFG1",              0x2001051C, LCL_CTL_REGBANK },
    { "PCIE_AXI_ERROR",              0x20010520, LCL_CTL_REGBANK },
    { "PCIE_AXI_ERR_CLR",            0x20010524, LCL_CTL_REGBANK },
    { "INT_STATUS",                  0x20010528, LCL_CTL_REGBANK },
    { "LIMIT_TIMEOUT_CNT",           0x2001052C, LCL_CTL_REGBANK },
    { "LINK_UP_EN",                  0x20010530, LCL_CTL_REGBANK },
    { "PCIE_LP_MODE",                0x20010534, LCL_CTL_REGBANK },
    { "PCIE_CLKREQ_CTRL",            0x20010538, LCL_CTL_REGBANK },
    { "AXI_CACTIVE_EN",              0x2001053C, LCL_CTL_REGBANK },
};


/*
 * 函 数 名  : oam_reg_get_cfg
 * 功能描述  : 获取phy/mac/soc/abb/rf 寄存器注册表的指针
 */
oam_reg_cfg_stru *oam_reg_get_cfg(oam_reg_type_enum_uint8 en_type)
{
    oam_reg_cfg_stru *pst_reg_cfg = OAL_PTR_NULL;

    switch (en_type) {
        case OAM_REG_PHY: {
            pst_reg_cfg = phy_reg;
            break;
        }
        case OAM_REG_SOC: {
            pst_reg_cfg = soc_reg;
            break;
        }
        case OAM_REG_MAC: {
            pst_reg_cfg = mac_reg;
            break;
        }
        default:
        {
            break;
        }
    }

    return pst_reg_cfg;
}

/*
 * 函 数 名  : oam_reg_get_num
 * 功能描述  : 获取phy/mac/soc/abb/rf下寄存器的数目,申请内存按满配申请，实际使用
 *             可能不需要这么多
 */
oal_uint32 oam_reg_get_num(oam_reg_type_enum_uint8 en_type)
{
    oal_uint32 ul_reg_num = 0;

    switch (en_type) {
        case OAM_REG_PHY: {
            ul_reg_num = OAM_PHY_REG_NUM;
            break;
        }
        case OAM_REG_SOC: {
            ul_reg_num = OAM_SOC_REG_NUM;
            break;
        }
        case OAM_REG_MAC: {
            ul_reg_num = OAM_MAC_REG_NUM;
            break;
        }
        default:
        {
            break;
        }
    }

    return ul_reg_num;
}

/*
 * 函 数 名  : oam_reg_malloc
 * 功能描述  : 分配并且初始化某类寄存器的内存
 */
oal_uint32 oam_reg_malloc(oam_reg_type_enum_uint8 en_type)
{
    oal_uint32 ul_loop = 0;
    oam_reg_stru *pst_reg = OAL_PTR_NULL;
    oam_reg_cfg_stru *pst_reg_cfg = OAL_PTR_NULL;
    oal_uint32 ul_reg_num;

    /* 已经初始化过，不需要再初始化了 */
    if (en_type >= OAM_REG_BUTT) {
        return OAL_ERR_CODE_ARRAY_OVERFLOW;
    }

    if (oam_reg_mng.past_reg[en_type] != OAL_PTR_NULL) {
        OAL_IO_PRINT("Err! register %u already exist!\n", en_type);
        return OAL_SUCC;
    }

    ul_reg_num = oam_reg_get_num(en_type);
    pst_reg = (oam_reg_stru *)OAL_MEM_ALLOC (OAL_MEM_POOL_ID_LOCAL,
                                             (oal_uint16)(ul_reg_num * sizeof(oam_reg_stru)),
                                             OAL_TRUE);
    oam_reg_mng.past_reg[en_type] = pst_reg;
    if (pst_reg == OAL_PTR_NULL) {
        return OAL_ERR_CODE_ALLOC_MEM_FAIL;
    }
    memset_s(pst_reg, ul_reg_num * sizeof(oam_reg_stru), 0, ul_reg_num * sizeof(oam_reg_stru));

    pst_reg_cfg = oam_reg_get_cfg(en_type);
    if (pst_reg_cfg == OAL_PTR_NULL) {
        OAL_MEM_FREE(pst_reg, OAL_TRUE);
        OAL_IO_PRINT("Err! register %u cfg not exist!\n", en_type);
        return OAL_ERR_CODE_PTR_NULL;
    }

    for (ul_loop = 0; ul_loop <= ul_reg_num; ul_loop++) {
        pst_reg->pst_reg_cfg = &pst_reg_cfg[ul_loop];
        pst_reg->en_rpt_flag = OAL_FALSE;
        pst_reg->ul_value = 0;
        pst_reg++;
    }

    oam_reg_mng.aul_reg_num[en_type] = ul_reg_num;
    return OAL_SUCC;
}

/*
 * 函 数 名  : oam_reg_free
 * 功能描述  : 释放某类寄存器的内存
 */
oal_uint32 oam_reg_free(oam_reg_type_enum_uint8 en_type)
{
    oam_reg_stru *pst_reg = OAL_PTR_NULL;

    if (en_type >= OAM_REG_BUTT) {
        return OAL_FAIL;
    }
    /* 不存在，返回成功 */
    pst_reg = oam_reg_mng.past_reg[en_type];
    if (pst_reg == OAL_PTR_NULL) {
        return OAL_SUCC;
    }

    OAL_MEM_FREE(pst_reg, OAL_TRUE);
    oam_reg_mng.past_reg[en_type] = OAL_PTR_NULL;
    oam_reg_mng.aul_reg_num[en_type] = 0;

    return OAL_SUCC;
}

/*
 * 函 数 名  : oam_reg_set_flag
 * 功能描述  : 设置寄存器是否需要刷新的标志
 */
oal_void oam_reg_set_flag(oam_reg_type_enum_uint8 en_type, oam_reg_subtype_enum_uint32 en_subtype,
                          oal_bool_enum_uint8 en_flag)
{
    oal_uint32 ul_loop = 0;
    oam_reg_stru *pst_reg = OAL_PTR_NULL;
    oal_uint32 ul_subreg_num;

    if (en_type >= OAM_REG_BUTT) {
        return;
    }

    pst_reg = oam_reg_mng.past_reg[en_type];
    if (pst_reg == OAL_PTR_NULL) {
        oam_reg_malloc(en_type);
        pst_reg = oam_reg_mng.past_reg[en_type];
        if (pst_reg == OAL_PTR_NULL) {
            return;
        }
    }

    ul_subreg_num = oam_reg_mng.aul_reg_num[en_type];
    for (ul_loop = 0; ul_loop < ul_subreg_num; ul_loop++) {
        if (en_subtype & pst_reg->pst_reg_cfg->ul_sub_type) {
            pst_reg->en_rpt_flag = en_flag;
        }
        pst_reg++;
    }

    /* 根据寄存器对象的上报标志设置bitmap，上报时使用 */
    pst_reg = oam_reg_mng.past_reg[en_type];
    for (ul_loop = 0; ul_loop < ul_subreg_num; ul_loop++) {
        if (pst_reg->en_rpt_flag) {
            oam_reg_mng.ul_reg_flag_bitmap |= (1 << en_type);
            return;
        }
        pst_reg++;
    }
    oam_reg_mng.ul_reg_flag_bitmap &= (~(oal_uint32)(1 << en_type));
}

/*
 * 函 数 名  : oam_reg_set_flag_addr
 * 功能描述  : 根据超始寄存器地址和结尾寄存器地址，设置寄存器是否需要刷新的标志
 */
oal_void oam_reg_set_flag_addr(oam_reg_type_enum_uint8 en_type, oal_uint32 ul_startaddr,
                               oal_uint32 ul_endaddr, oal_bool_enum_uint8 en_flag)
{
    oal_uint32 ul_loop = 0;
    oam_reg_stru *pst_reg = OAL_PTR_NULL;
    oal_uint32 ul_subreg_num;
    oal_uint32 ul_addr = 0;

    if (en_type >= OAM_REG_BUTT) {
        return;
    }

    pst_reg = oam_reg_mng.past_reg[en_type];
    if (pst_reg == OAL_PTR_NULL) {
        oam_reg_malloc(en_type);
        pst_reg = oam_reg_mng.past_reg[en_type];
        if (pst_reg == OAL_PTR_NULL) {
            return;
        }
    }

    ul_subreg_num = oam_reg_mng.aul_reg_num[en_type];

    for (ul_addr = ul_startaddr; ul_addr <= ul_endaddr; ul_addr += sizeof(oal_uint32)) { /* 每次偏移4字节 */
        for (ul_loop = 0; ul_loop < ul_subreg_num; ul_loop++) {
            if (ul_addr == pst_reg->pst_reg_cfg->ul_addr) {
                pst_reg->en_rpt_flag = en_flag;
                break;
            }
            pst_reg++;
        }
        pst_reg = oam_reg_mng.past_reg[en_type];
    }

    /* 根据寄存器对象的上报标志设置bitmap，上报时使用 */
    pst_reg = oam_reg_mng.past_reg[en_type];
    for (ul_loop = 0; ul_loop < ul_subreg_num; ul_loop++) {
        if (pst_reg->en_rpt_flag) {
            oam_reg_mng.ul_reg_flag_bitmap |= (1 << en_type);
            return;
        }
        pst_reg++;
    }
    oam_reg_mng.ul_reg_flag_bitmap &= (~(oal_uint32)(1 << en_type));
}

/*
 * 函 数 名  : oam_reg_allow_netbuf_add
 * 功能描述  : 设置操作netbuf链表的标志
 */
oal_void oam_reg_allow_netbuf_add(oal_bool_enum_uint8 en_flag)
{
    oam_reg_mng.en_netbuf_flag = en_flag;
}

/*
 * 函 数 名  : oam_reg_is_allow_netbuf_add
 * 功能描述  : 读取操作netbuf链表的标志
 */
oal_uint8 oam_reg_is_allow_netbuf_add(oal_void)
{
    return oam_reg_mng.en_netbuf_flag;
}

/*
 * 函 数 名  : oam_reg_allow_refresh
 * 功能描述  : 设置允许刷新寄存器的标志
 */
oal_void oam_reg_allow_refresh(oal_bool_enum_uint8 en_flag)
{
    oam_reg_mng.en_refresh_flag = en_flag;
}

/*
 * 函 数 名  : oam_reg_is_allow_refresh
 * 功能描述  : 读取是否允许刷新寄存器的标志
 */
oal_uint8 oam_reg_is_allow_refresh(oal_void)
{
    return oam_reg_mng.en_refresh_flag;
}

/*
 * 函 数 名  : oam_reg_is_need_refresh
 * 功能描述  : 判断该事件是否触发寄存器刷新
 */
oal_uint8 oam_reg_is_need_refresh(oam_reg_evt_enum_uint32 en_evt_type)
{
    if (en_evt_type >= OAM_REG_EVT_BUTT) {
        return OAL_FALSE;
    }
    if (!oam_reg_is_allow_refresh()) {
        return OAL_FALSE;
    }
    if (oam_reg_mng.ul_reg_flag_bitmap == 0) {
        return OAL_FALSE;
    }

    /* 判断该事件上报开关是否打开  */
    /* 用户没有设置该事件，不需要上报 */
    if (oam_reg_mng.aul_refresh_evt[en_evt_type] != OAL_TRUE) {
        return OAL_FALSE;
    }

    /* 事件需要判断evt tick有没有计到0，如果计到0，则刷新上报数据，并更新evt_tick  */
    if (oam_reg_mng.aul_evt_tick[en_evt_type] == 0) {
        return OAL_FALSE;
    }

    oam_reg_mng.aul_evt_tick[en_evt_type]--;
    if (oam_reg_mng.aul_evt_tick[en_evt_type] != 0) {
        return OAL_FALSE;
    }

    oam_reg_mng.aul_evt_tick[en_evt_type] = oam_reg_mng.aul_evt_tick_cfg[en_evt_type];
    return OAL_TRUE;
}

/*
 * 函 数 名  : oam_reg_get_evt_name
 * 功能描述  : 获取事件字符名
 * 输入参数  : 事件
 * 输出参数  : 事件字符名
 */
oal_uint32 oam_reg_get_evt_name(oam_reg_evt_enum_uint32 en_evt_type, oal_uint8 *puc_evt_name)
{
    switch (en_evt_type) {
        case OAM_REG_EVT_TX: {
            memcpy_s(puc_evt_name, sizeof("TX"), "TX", sizeof("TX"));
            break;
        }
        case OAM_REG_EVT_RX: {
            memcpy_s(puc_evt_name, sizeof("RX"), "RX", sizeof("RX"));
            break;
        }
        case OAM_REG_EVT_TBTT: {
            memcpy_s(puc_evt_name, sizeof("tbtt"), "tbtt", sizeof("tbtt"));
            break;
        }
        case OAM_REG_EVT_PRD: {
            memcpy_s(puc_evt_name, sizeof("period"), "period", sizeof("period"));
            break;
        }
        case OAM_REG_EVT_ERR: {
            memcpy_s(puc_evt_name, sizeof("err"), "err", sizeof("err"));
            break;
        }
        default:
        {
            memcpy_s(puc_evt_name, sizeof("undefined"), "undefined", sizeof("undefined"));
            break;
        }
    }
    return OAL_SUCC;
}

/*
 * 函 数 名  : oam_reg_set_evt
 * 功能描述  : 设置需要刷新寄存器的事件,供配置命令使用
 */
oal_void oam_reg_set_evt(oam_reg_evt_enum_uint32 en_evt_type, oal_uint32 ul_tick)
{
    oal_uint32 ul_evt_type = 0;
    /* 处理所有事件 */
    if (en_evt_type >= OAM_REG_EVT_BUTT) {
        for (ul_evt_type = 0; ul_evt_type < OAM_REG_EVT_BUTT; ul_evt_type++) {
            oam_reg_mng.aul_refresh_evt[ul_evt_type] = ((ul_tick == 0) ? OAL_FALSE : OAL_TRUE);
            oam_reg_mng.aul_evt_tick_cfg[ul_evt_type] = ul_tick;
            oam_reg_mng.aul_evt_tick[ul_evt_type] = ul_tick;
        }
        return;
    }

    oam_reg_mng.aul_refresh_evt[en_evt_type] = ((ul_tick == 0) ? OAL_FALSE : OAL_TRUE);
    oam_reg_mng.aul_evt_tick_cfg[en_evt_type] = ul_tick;
    oam_reg_mng.aul_evt_tick[en_evt_type] = ul_tick;
}

/*
 * 函 数 名  : oam_reg_recv_msg
 * 功能描述  : netlink接收处理入口函数
 */
oal_uint32 oam_reg_recv_msg(oal_uint8 *puc_data, oal_uint32 ul_len)
{
    oam_reg_mng.uc_ack_flag = OAL_TRUE;

    return OAL_SUCC;
}

/*
 * 函 数 名  : oam_reg_rpt_buff
 * 功能描述  : 将存在buff中的寄存器数据通过netbuff上报给nlc app
 */
oal_uint32 oam_reg_rpt_buff(oal_void)
{
    oal_uint32 ul_pkt_num;
    oam_reg_send_head_stru *pst_send_head;
    oal_int32 ret;

    pst_send_head = (oam_reg_send_head_stru *)oam_reg_mng.puc_send_buff;
    if (pst_send_head == OAL_PTR_NULL) {
        return OAL_ERR_CODE_PTR_NULL;
    }

    ul_pkt_num = pst_send_head->ul_pkt_num;
    if (ul_pkt_num == 0) {
        return OAL_SUCC;
    }

    oam_reg_mng.ul_rpt_count++;
    ret = oam_netlink_kernel_send((oal_uint8 *)pst_send_head,
                                  sizeof(oam_reg_send_head_stru) + ul_pkt_num * sizeof(oam_reg_rpt),
                                  OAM_NL_CMD_REG);
    if (ret <= 0) {
        OAL_IO_PRINT("OAM: netlink send oam reg rpt msg failed, ret = %d\n", ret);
        return OAL_FAIL;
    }

    pst_send_head->ul_pkt_num = 0;

    OAL_IO_PRINT("rpt pkt ok! sn %u", pst_send_head->ul_sn);
    pst_send_head->ul_sn++;
    /* 等待APP侧接收完成 */
    return OAL_SUCC;
}

/*
 * 函 数 名  : oam_reg_nb2buf
 * 功能描述  : 将netbuff内容入buff，以备一次性发送，减少发送次数
 */
oal_uint32 oam_reg_nb2buf(oal_netbuf_stru *pst_netbuf)
{
    oal_uint32 ul_pkt_num;
    oam_reg_send_head_stru *pst_send_buff;
    pst_send_buff = (oam_reg_send_head_stru *)oam_reg_mng.puc_send_buff;
    if (pst_send_buff == OAL_PTR_NULL) {
        OAL_IO_PRINT("pst_send_buff NULL! \n");
        return OAL_FAIL;
    }
    if (pst_netbuf == OAL_PTR_NULL) {
        return OAL_SUCC;
    }

    ul_pkt_num = pst_send_buff->ul_pkt_num;
    if (memcpy_s((oal_uint8 *)pst_send_buff + sizeof(oam_reg_send_head_stru) + ul_pkt_num * sizeof(oam_reg_rpt),
                 OAM_REG_MAX_SEND_BUF_SIZE - sizeof(oam_reg_send_head_stru) - ul_pkt_num * sizeof(oam_reg_rpt),
                 oal_netbuf_data(pst_netbuf),
                 sizeof(oam_reg_rpt)) != EOK) {
        OAL_IO_PRINT("memcpy_s error, destlen=%u, srclen=%u\n ",
                     (oal_uint32)(OAM_REG_MAX_SEND_BUF_SIZE - sizeof(oam_reg_send_head_stru) -
                     ul_pkt_num * sizeof(oam_reg_rpt)),
                     (oal_uint32)sizeof(oam_reg_rpt));
        return -OAL_EFAIL;
    }

    pst_send_buff->ul_pkt_num++;

    return oam_event_report(BROADCAST_MACADDR,
                            0,
                            OAM_MODULE_ID_OAM,
                            OAM_EVENT_REGISTER,
                            oal_netbuf_data(pst_netbuf),
                            sizeof(oam_reg_rpt));
}

/*
 * 函 数 名  : oam_reg_wq
 * 功能描述  : workqueue的入口函数，将nb入缓存后发送
 */
oal_void oam_reg_wq(oal_work_stru *pst_work)
{
    oal_netbuf_stru *pst_netbuf = NULL;
    oal_uint32 ul_nb_num;
    oal_uint32 ul_loop = 0;

    /* 禁止netbuf入队 */
    oam_reg_allow_netbuf_add(0);

    /* 将netbuff数据入缓冲，最多入OAM_REG_MAX_PKT_ONE_BUFF个 */
    ul_nb_num = oam_reg_mng.ul_nb;
    OAL_IO_PRINT("oam_reg_wq: nb num %u\n", ul_nb_num);
    while (ul_nb_num >= OAM_REG_MAX_PKT_ONE_BUFF) {
        /* 查阅2.6.34内核代码，如果没有元素，oal_netbuf_delist返回NULL */
        for (ul_loop = 0; ul_loop < OAM_REG_MAX_PKT_ONE_BUFF; ul_loop++) {
            pst_netbuf = oal_netbuf_delist(&oam_reg_mng.st_wq.st_netbuf_head);
            /* 没有元素，break掉 */
            if (pst_netbuf == OAL_PTR_NULL) {
                break;
            }

            oam_reg_nb2buf(pst_netbuf);
            oal_netbuf_free(pst_netbuf);
        }

        oam_reg_rpt_buff();

        ul_nb_num -= OAM_REG_MAX_PKT_ONE_BUFF;
    }

    /* 将剩余的netbuff入缓冲 */
    if (ul_nb_num > 0) {
        for (ul_loop = 0; ul_loop < ul_nb_num; ul_loop++) {
            pst_netbuf = oal_netbuf_delist(&oam_reg_mng.st_wq.st_netbuf_head);
            /* 没有元素，break掉 */
            if (pst_netbuf == OAL_PTR_NULL) {
                break;
            }

            oam_reg_nb2buf(pst_netbuf);
            oal_netbuf_free(pst_netbuf);
        }

        oam_reg_rpt_buff();
    }
    /* 全发完之后置0 */
    oam_reg_mng.ul_nb = 0;
    oam_reg_allow_netbuf_add(1);
    OAL_IO_PRINT("dmac_data_acq_workqueue end!\n");
}

/*
 * 函 数 名  : oam_reg_init
 * 功能描述  : 初始化寄存器相关内容
 */
oal_void oam_reg_init(oal_void)
{
    memset_s(&oam_reg_mng, sizeof(oam_reg_manage_stru), 0, sizeof(oam_reg_manage_stru));

    oam_reg_mng.puc_send_buff = (oal_uint8 *)OAL_MEM_ALLOC(OAL_MEM_POOL_ID_LOCAL,
                                                           OAM_REG_MAX_SEND_BUF_SIZE, OAL_TRUE);
    if (oam_reg_mng.puc_send_buff == OAL_PTR_NULL) {
        OAL_IO_PRINT("oam_reg_init: puc_send_buff is NULL!\n");
        return;
    }
    memset_s(oam_reg_mng.puc_send_buff, OAM_REG_MAX_SEND_BUF_SIZE, 0, OAM_REG_MAX_SEND_BUF_SIZE);

    /* 注册netlink接收函数 */
    oam_netlink_ops_register(OAM_NL_CMD_REG, oam_reg_recv_msg);

    /* 刷新标志设置为1，初始化允许中断刷新 */
    oam_reg_allow_refresh(1);

    /* 允许增加netbuf */
    oam_reg_allow_netbuf_add(1);

    /* 初始化工作队列 */
    memset_s((void *)&oam_reg_mng.st_wq, OAL_SIZEOF(oam_reg_workqueue_stru),
             0, OAL_SIZEOF(oam_reg_workqueue_stru));
    OAL_INIT_WORK(&oam_reg_mng.st_wq.st_wk, oam_reg_wq);
    oal_netbuf_list_head_init(&oam_reg_mng.st_wq.st_netbuf_head);
}

oal_void oam_reg_exit(oal_void)
{
    oal_uint32 ul_reg = 0;
    /* 将动态申请的内存释放掉 */
    for (ul_reg = 0; ul_reg < OAM_REG_BUTT; ul_reg++) {
        oam_reg_free(ul_reg);
    }

    if (oam_reg_mng.puc_send_buff != OAL_PTR_NULL) {
        OAL_MEM_FREE(oam_reg_mng.puc_send_buff, OAL_TRUE);
        oam_reg_mng.puc_send_buff = OAL_PTR_NULL;
    }

    /* 去注册netlink接收函数 */
    oam_netlink_ops_unregister(OAM_NL_CMD_REG);

    oal_cancel_work_sync(&oam_reg_mng.st_wq.st_wk);
    oal_netbuf_queue_purge(&oam_reg_mng.st_wq.st_netbuf_head);
}

/*
 * 函 数 名  : oam_reg_add_netbuf
 * 功能描述  : 将所需上报的数据装入netbuf，并且入队
 * 输入参数  : pst_reg_rpt 所需要上报的寄存器数据指针
 */
oal_uint32 oam_reg_add_netbuf(oam_reg_rpt *pst_reg_rpt)
{
    oal_netbuf_stru *pst_queue_netbuf = NULL;
    oam_reg_rpt *pst_reg_rpt_nb = NULL;

    if (pst_reg_rpt == OAL_PTR_NULL) {
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_queue_netbuf = oal_netbuf_alloc(OAM_REG_UNIT_MAX_LEN, 0, WLAN_MEM_NETBUF_ALIGN);
    if (pst_queue_netbuf == OAL_PTR_NULL) {
        OAL_IO_PRINT("oam_reg_wk_rpt pst_netbuf alloc failed.Try again!\n");
        return OAL_ERR_CODE_PTR_NULL;
    }

    pst_reg_rpt_nb = (oam_reg_rpt *)oal_netbuf_data(pst_queue_netbuf);
    *pst_reg_rpt_nb = *pst_reg_rpt;
    oal_netbuf_add_to_list_tail(pst_queue_netbuf, &oam_reg_mng.st_wq.st_netbuf_head);
    oam_reg_mng.ul_nb++;

    return OAL_SUCC;
}

/*
 * 函 数 名  : oam_reg_report
 * 功能描述  : 寄存器数据上报,在tasklet中调用
 */
oal_void oam_reg_report(oal_void)
{
    oal_uint32 ul_reg_type = 0;
    oal_uint32 ul_reg_num = 0;
    oam_reg_stru *pst_reg = OAL_PTR_NULL;
    oam_reg_rpt st_rpt;

    /* 刷新标志设置为0，当中断到来时，判断此标志不再重复刷新寄存器 */
    oam_reg_allow_refresh(0);

    /* 没有需要刷新的寄存器，返回 */
    if (oam_reg_mng.ul_reg_flag_bitmap == 0) {
        oam_reg_allow_refresh(1);
        return;
    }

    /* 不能操作netbuf，返回 */
    if (!oam_reg_is_allow_netbuf_add()) {
        oam_reg_allow_refresh(1);
        OAL_IO_PRINT("oam_reg_is_allow_netbuf_add not allow! \n");
        return;
    }

    st_rpt.us_flag = OAM_REG_RPT_START;
    st_rpt.us_sn = 0;
    st_rpt.un_rpt.st_rpt_head.en_evt = oam_reg_mng.en_evt_type;
    st_rpt.un_rpt.st_rpt_head.ul_timestamp = oam_reg_mng.ul_time_stamp_start;
    oam_reg_add_netbuf(&st_rpt);
    oam_reg_mng.ul_rpt_count++;

    for (ul_reg_type = 0; ul_reg_type < OAM_REG_BUTT; ul_reg_type++) {
        pst_reg = oam_reg_mng.past_reg[ul_reg_type];
        if (pst_reg == OAL_PTR_NULL) {
            continue;
        }
        OAL_IO_PRINT("oam_reg_report regtype %u, regnum %u \n", ul_reg_type, oam_reg_mng.aul_reg_num[ul_reg_type]);
        for (ul_reg_num = 0; ul_reg_num < oam_reg_mng.aul_reg_num[ul_reg_type]; ul_reg_num++) {
            if (pst_reg[ul_reg_num].en_rpt_flag != OAL_TRUE) {
                continue;
            }
            st_rpt.us_flag = OAM_REG_RPT_DATA;
            st_rpt.us_sn++;
            st_rpt.un_rpt.st_rpt_data.ul_addr = pst_reg[ul_reg_num].pst_reg_cfg->ul_addr;
            st_rpt.un_rpt.st_rpt_data.ul_value = pst_reg[ul_reg_num].ul_value;
            oam_reg_add_netbuf(&st_rpt);
        }
    }

    st_rpt.us_flag = OAM_REG_RPT_END;
    st_rpt.us_sn++;
    st_rpt.un_rpt.st_rpt_head.en_evt = oam_reg_mng.en_evt_type;
    st_rpt.un_rpt.st_rpt_head.ul_timestamp = oam_reg_mng.ul_time_stamp_end;
    oam_reg_add_netbuf(&st_rpt);

    /* 启动workqueue */
    oal_workqueue_schedule(&oam_reg_mng.st_wq.st_wk);

    /* 允许刷新 */
    oam_reg_allow_refresh(1);
}

oal_void oam_reg_info(oal_void)
{
    const oal_uint32 ul_buf_len = 64;
    oal_int8 ac_tmp_buff[ul_buf_len];
    oal_int32 ret;

    ret = snprintf_s(ac_tmp_buff, OAL_SIZEOF(ac_tmp_buff), OAL_SIZEOF(ac_tmp_buff) - 1, "rpt cnt %u \n",
                     oam_reg_mng.ul_rpt_count);
    if (ret < 0) {
        OAL_IO_PRINT("ref info str format err\n");
        return;
    }

    ac_tmp_buff[ul_buf_len - 1] = '\0';
    oam_print(ac_tmp_buff);
}

/*lint -e19*/
oal_module_symbol(oam_reg_mng);
oal_module_symbol(oam_reg_is_need_refresh);
oal_module_symbol(oam_reg_get_evt_name);
oal_module_symbol(oam_reg_report);
oal_module_symbol(oam_reg_info);
oal_module_symbol(oam_reg_set_evt);
oal_module_symbol(oam_reg_set_flag);
oal_module_symbol(oam_reg_set_flag_addr);
/*lint +e19*/
#endif /* #ifdef _PRE_WLAN_DFT_REG */
