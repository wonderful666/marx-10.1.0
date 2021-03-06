#ifndef __MNTN_INTERFACE_H__
#define __MNTN_INTERFACE_H__ 
#include <product_config.h>
#define MNTN_BASE_ADDR (DDR_MNTN_ADDR)
#define MNTN_BASE_SIZE (DDR_MNTN_SIZE)
#define MNTN_BASEINFO_ADDR (MNTN_BASE_ADDR)
#define MNTN_BASEINFO_SIZE (0x200)
#define MNTN_AREA_MDMAP_ADDR (MNTN_BASEINFO_ADDR+MNTN_BASEINFO_SIZE)
#define MNTN_AREA_MDMAP_SIZE (1024*400)
#define MNTN_AREA_LR_ADDR (MNTN_AREA_MDMAP_ADDR+MNTN_AREA_MDMAP_SIZE)
#define MNTN_AREA_LR_SIZE (1024*1536)
#define MNTN_AREA_NR_ADDR (MNTN_AREA_LR_ADDR+MNTN_AREA_LR_SIZE)
#define MNTN_AREA_NR_SIZE (1024*1024)
#define MNTN_AREA_LPM3_ADDR (MNTN_AREA_NR_ADDR+MNTN_AREA_NR_SIZE)
#define MNTN_AREA_LPM3_SIZE (0x0)
#define MNTN_AREA_TEEOS_ADDR (MNTN_AREA_LPM3_ADDR+MNTN_AREA_LPM3_SIZE)
#define MNTN_AREA_TEEOS_SIZE (0)
#define MNTN_AREA_CBOOT_ADDR (MNTN_AREA_TEEOS_ADDR+MNTN_AREA_TEEOS_SIZE)
#define MNTN_AREA_CBOOT_SIZE (1024)
#define MNTN_AREA_NRCCPU_CBOOT_ADDR (MNTN_AREA_CBOOT_ADDR+MNTN_AREA_CBOOT_SIZE)
#define MNTN_AREA_NRCCPU_CBOOT_SIZE (1024)
#define MNTN_AREA_L2HAC_CBOOT_ADDR (MNTN_AREA_NRCCPU_CBOOT_ADDR+MNTN_AREA_NRCCPU_CBOOT_SIZE)
#define MNTN_AREA_L2HAC_CBOOT_SIZE (1024)
#define MNTN_AREA_RESERVE_ADDR (MNTN_AREA_L2HAC_CBOOT_ADDR+MNTN_AREA_L2HAC_CBOOT_SIZE)
#define MNTN_AREA_RESERVE_SIZE (MNTN_BASE_ADDR+MNTN_BASE_SIZE-MNTN_AREA_RESERVE_ADDR)
#define MNTN_AREA_NR_RDR_ADDR (MNTN_AREA_NR_ADDR)
#define MNTN_AREA_NR_RDR_SIZE (0x200)
#define MNTN_AREA_NR_CCPU_ADDR (MNTN_AREA_NR_RDR_ADDR + MNTN_AREA_NR_RDR_SIZE)
#define MNTN_AREA_NR_CCPU_SIZE (750*1024)
#define MNTN_AREA_NR_L2HAC_ADDR (MNTN_AREA_NR_CCPU_ADDR+MNTN_AREA_NR_CCPU_SIZE)
#define MNTN_AREA_NR_L2HAC_SIZE (200*1024)
#define MNTN_AREA_NR_HL1C_ADDR (MNTN_AREA_NR_L2HAC_ADDR+MNTN_AREA_NR_L2HAC_SIZE)
#define MNTN_AREA_NR_HL1C_SIZE (30*1024)
#define DUMP_TASK_SWITCH_SIZE (0x10000)
#define BITWISE_OP_ENABLE 
#endif
