#ifndef __ISP_DDR_MAP_H__
#define __ISP_DDR_MAP_H__ 
#include <global_ddr_map.h>
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif
#define ISP_PCTRL_PERI_STAT_ADDR (0x000000BC)
#define ISP_PCTRL_PERI_FLAG (1 << 5)
#define MEM_ISPFW_SIZE (0x00E00000)
#define MEM_PMD_ADDR_OFFSET (0x00002000)
#define MEM_PTE_ADDR_OFFSET (MEM_ISPFW_SIZE + 0x10000)
#define MEM_RDR_RESERVED_BASE HISI_RESERVED_MNTN_PHYMEM_BASE
#define MEM_ISP_RDR_OFFSET (0x6CE000)
#define MEM_SMMU_ERRRD_ADDR_OFFSET (0x0000F000)
#define MEM_SMMU_ERRWR_ADDR_OFFSET (0x0000F800)
#define MEM_ISP_ERRRD_ADDR_OFFSET (0x00000500)
#define MEM_ISP_ERRWR_ADDR_OFFSET (0x00000600)
#define MEM_ISP_ERR_ADDR_BASE (MEM_RDR_RESERVED_BASE + MEM_ISP_RDR_OFFSET)
#define MEM_RSCTABLE_ADDR_OFFSET (0x00014000)
#define MEM_RSCTABLE_SIZE (0x00004000)
#define MEM_ISPDTS_SIZE (0x02000000)
#define MEM_MDC_DA (0xC1400000)
#define MEM_MDC_SIZE (0x00100000)
#define ISPCPU_COREDUMP_ADDR (0xC4000000)
#define ISPCPU_COREDUMP_SIZE (0x01500000)
#define MEM_RAW2YUV_DA (0xC8000000)
#define SHAREDMEM_BASE (0xc2000000)
#define VQ_BASE (0xc2020000)
#define DEVICE_BASE (0xE0000000)
#define TEXT_BASE (0xc0200000)
#define DATA_BASE (0xc0800000)
#define ISP_TEXT_SIZE (0x00600000)
#define ISP_DATA_SIZE (0x00800000)
#define ISP_BIN_DATA_SIZE (0x00600000)
#define ISP_FW_SIZE (ISP_TEXT_SIZE + ISP_DATA_SIZE)
#define ISP_BIN_FW_SIZE (ISP_TEXT_SIZE + ISP_BIN_DATA_SIZE)
#define ITCM_BASE (0xFFFF0000)
#define DTCM_BASE (0xE83D0000)
#define ISP_CORE_CFG_BASE_ADDR (0xE8200000)
#define ISP_PMC_BASE_ADDR (0xFFF01000)
#define MAX_REGION_NUM (24)
#define STATIC_REGION_NSEC_NUM (12)
#define STATIC_REGION_SEC_NUM (13)
#define ISP_NOC_ENABLE 0x00
#define ISP_TARGETFLOW 0x05
#define SOC_ISPCPU_MID 0x6B
#define SOC_ISPCORE_MID 0x40
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif
#endif