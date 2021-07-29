/* MD5: 3da8de11985e73cbc070062b28710d7e*/
/*
 * copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
 * foss@huawei.com
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 and
 * * only version 2 as published by the Free Software Foundation.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) Neither the name of Huawei nor the names of its contributors may
 * *    be used to endorse or promote products derived from this software
 * *    without specific prior written permission.
 *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if !defined(__PRODUCT_CONFIG_NRPS_H__)
#define __PRODUCT_CONFIG_NRPS_H__

#ifndef FEATURE_ON
#define FEATURE_ON 1 
#endif 

#ifndef FEATURE_OFF
#define FEATURE_OFF 0 
#endif 

#ifdef EMU_TYPE_ESLPS 
#ifndef FEATURE_NPHY_STUB_ESL
#define FEATURE_NPHY_STUB_ESL FEATURE_ON 
#endif 

#else
#ifndef FEATURE_NPHY_STUB_ESL
#define FEATURE_NPHY_STUB_ESL FEATURE_OFF 
#endif 

#endif
#ifdef EMU_TYPE_ESL 
#ifndef FEATURE_NRRC_EMU_ESL
#define FEATURE_NRRC_EMU_ESL FEATURE_ON 
#endif 

#else
#ifndef FEATURE_NRRC_EMU_ESL
#define FEATURE_NRRC_EMU_ESL FEATURE_OFF 
#endif 

#endif
#ifndef FEATURE_NL2_MAA_ALLOC
#define FEATURE_NL2_MAA_ALLOC FEATURE_ON 
#endif 

#ifndef NR_PROTOL_STACK_ENG
#endif 

#ifndef FEATURE_NR_SEC_DISABLE
#define FEATURE_NR_SEC_DISABLE FEATURE_ON 
#endif 

#ifndef NR_MAX_SERVING_CC_NUM
#define NR_MAX_SERVING_CC_NUM 1 
#endif 

#ifndef NR_MAX_UL_SERVING_CC_NUM
#define NR_MAX_UL_SERVING_CC_NUM 1 
#endif 

#ifndef NR_MAX_CG_NUM
#define NR_MAX_CG_NUM 1 
#endif 

#ifndef FEATURE_NL2_1CC_MEM
#define FEATURE_NL2_1CC_MEM FEATURE_ON 
#endif 

#ifndef NR_MAX_PER_PLMN_NRSA_BC_NUM
#define NR_MAX_PER_PLMN_NRSA_BC_NUM 128 
#endif 

#ifndef NR_MAX_PER_PLMN_ENDC_BC_NUM
#define NR_MAX_PER_PLMN_ENDC_BC_NUM 512 
#endif 

#endif /*__PRODUCT_CONFIG_H__*/ 
