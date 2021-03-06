
CFG_CONFIG_SAPPER_SETS := 
CFG_NR_MODEM_BUILD :=true

ifeq ($(chip_type),tc)
CFG_CHIP_TYPE_TC := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_CHIP_TYPE_TC
endif

ifeq ($(chip_type),es)
CFG_CHIP_TYPE_ES := FEATURE_ON
CFG_CHIP_TYPE_CS := FEATURE_OFF
CFG_CONFIG_SAPPER_SETS += CFG_CHIP_TYPE_ES
CFG_CONFIG_SAPPER_SETS += CFG_CHIP_TYPE_CS
endif

ifeq ($(chip_type),)
CFG_CHIP_TYPE_CS := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_CHIP_TYPE_CS
endif


ifeq ($(bbit_type),nr)
CFG_BBIT_TYPE_NR := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_BBIT_TYPE_NR
endif

ifeq ($(emulate),)
CFG_UPHY_PLATFORM := UPHY_ASIC
endif

ifeq ($(emulate),esl)
CFG_UPHY_PLATFORM := UPHY_ASIC
endif

ifeq ($(emulate),porting)
CFG_UPHY_PLATFORM := UPHY_PORTING
endif

ifeq ($(emulate),sft)
CFG_UPHY_PLATFORM := UPHY_SFT
endif

ifeq ($(emulate),esl)
CFG_UPHY_PLATFORM := UPHY_ESL
CFG_EMU_TYPE_ESL := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_EMU_TYPE_ESL
endif

ifeq ($(emulate),emu)
CFG_UPHY_PLATFORM := UPHY_EMU
CFG_EMU_TYPE_EMU := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_EMU_TYPE_EMU
endif

ifeq ($(emulate),slt)
CFG_UPHY_PLATFORM := UPHY_PORTING
CFG_EMU_TYPE_SLT := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_EMU_TYPE_SLT
endif

ifeq ($(emulate),eslps)
CFG_UPHY_PLATFORM := UPHY_ESL
CFG_EMU_TYPE_ESL := FEATURE_ON
CFG_EMU_TYPE_ESLPS := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_EMU_TYPE_ESL
CFG_CONFIG_SAPPER_SETS += CFG_EMU_TYPE_ESLPS
endif

ifeq ($(lphy_porting),true)
CFG_LPHY_LATE_MODE := FEATURE_ON
endif

ifneq ($(extra_modem),)
CFG_EXTRA_MODEM_MODE_MBB := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_EXTRA_MODEM_MODE_MBB
endif

ifeq ($(rf_type0),)
CFG_RF0_TYPE_ASIC_HI6365 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF0_TYPE_ASIC_HI6365
endif

ifeq ($(rf_type1),)
CFG_RF1_TYPE_ASIC_HI6370 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF1_TYPE_ASIC_HI6370
endif

ifeq ($(rf_type0),fpga_lf)
CFG_RF0_TYPE_FPGA_LF := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF0_TYPE_FPGA_LF
endif

ifeq ($(rf_type1),fpga_hf)
CFG_RF1_TYPE_FPGA_HF := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF1_TYPE_FPGA_HF
endif

ifeq ($(rf_type0),dv200)
CFG_RF0_TYPE_DV200 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF0_TYPE_DV200
endif

ifeq ($(rf_type1),gv300)
CFG_RF1_TYPE_GV300 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF1_TYPE_GV300
endif

ifeq ($(rf_type0),fpga_hi6365)
CFG_RF0_TYPE_FPGA_HI6365 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF0_TYPE_FPGA_HI6365
endif

ifeq ($(rf_type1),fpga_hi6370)
CFG_RF1_TYPE_FPGA_HI6370 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF1_TYPE_FPGA_HI6370
endif

ifeq ($(rf_type0),asic_hi6365)
CFG_RF0_TYPE_ASIC_HI6365 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF0_TYPE_ASIC_HI6365
endif

ifeq ($(rf_type1),asic_hi6370)
CFG_RF1_TYPE_ASIC_HI6370 := FEATURE_ON
CFG_CONFIG_SAPPER_SETS += CFG_RF1_TYPE_ASIC_HI6370
endif




