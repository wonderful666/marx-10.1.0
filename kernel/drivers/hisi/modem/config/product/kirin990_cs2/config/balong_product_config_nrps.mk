# MD5: 84d6ba8c722503bf308d06e6e18747d8
CFG_FEATURE_ON                                  := 1
CFG_FEATURE_OFF                                 := 0
ifeq ($(EMU_TYPE_ESLPS),FEATURE_ON)
CFG_FEATURE_NPHY_STUB_ESL := FEATURE_ON
else
CFG_FEATURE_NPHY_STUB_ESL := FEATURE_OFF
endif
ifeq ($(EMU_TYPE_ESL),FEATURE_ON)
CFG_FEATURE_NRRC_EMU_ESL := FEATURE_ON
else
CFG_FEATURE_NRRC_EMU_ESL := FEATURE_OFF
endif
CFG_FEATURE_NL2_MAA_ALLOC := FEATURE_ON
CFG_NR_PROTOL_STACK_ENG := YES
CFG_FEATURE_NR_SEC_DISABLE := FEATURE_ON
CFG_NR_MAX_SERVING_CC_NUM    := 1
CFG_NR_MAX_UL_SERVING_CC_NUM := 1
CFG_NR_MAX_CG_NUM    := 1
CFG_FEATURE_NL2_1CC_MEM := FEATURE_ON
CFG_NR_MAX_PER_PLMN_NRSA_BC_NUM := 128
CFG_NR_MAX_PER_PLMN_ENDC_BC_NUM := 512
