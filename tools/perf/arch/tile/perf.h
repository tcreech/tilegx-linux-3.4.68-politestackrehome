/*
 * Copyright 2013 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 *
 * Tilera TILEPro and TILEGx specific performance events.
 *
 * We have a sort-order requirement if some event names share the same
 * beginning. Take TLB_EXCEPTION and TLB for example, we should put
 * TLB_EXCEPTION before TLB, otherwise perf tool will take TLB as a correct
 * event name, and then continue to parse the rest string "_EXCEPTION",
 * then failed.
 */

#if defined(__tilepro__)
#define PERF_COUNT_RAW_ZERO                             0x000
#define PERF_COUNT_RAW_ONE                              0x001
#define PERF_COUNT_RAW_PASS_WRITTEN                     0x002
#define PERF_COUNT_RAW_FAIL_WRITTEN                     0x003
#define PERF_COUNT_RAW_DONE_WRITTEN                     0x004

#define PERF_COUNT_RAW_MP_BUNDLE_RETIRED                0x006
#define PERF_COUNT_RAW_MP_BUNDLE_1_INSTR_RETIRED        0x007
#define PERF_COUNT_RAW_MP_BUNDLE_2_INSTR_RETIRED        0x008
#define PERF_COUNT_RAW_MP_BUNDLE_3_INSTR_RETIRED        0x009
#define PERF_COUNT_RAW_MP_UDN_READ_STALL                0x00a

#define PERF_COUNT_RAW_MP_IDN_READ_STALL                0x00b
#define PERF_COUNT_RAW_MP_SN_READ_STALL                 0x00c
#define PERF_COUNT_RAW_MP_UDN_WRITE_STALL               0x00d
#define PERF_COUNT_RAW_MP_IDN_WRITE_STALL               0x00e
#define PERF_COUNT_RAW_MP_SN_WRITE_STALL                0x00f

#define PERF_COUNT_RAW_MP_DATA_CACHE_STALL              0x010
#define PERF_COUNT_RAW_MP_INSTRUCTION_CACHE_STALL       0x011
#define PERF_COUNT_RAW_MP_ICACHE_HIT_ISSUED             0x012
#define PERF_COUNT_RAW_MP_ITLB_HIT_ISSUED               0x013
#define PERF_COUNT_RAW_MP_CONDITIONAL_BRANCH_MISSPREDICT 0x014

#define PERF_COUNT_RAW_MP_INDIRECT_BRANCH_MISSPREDICT   0x015
#define PERF_COUNT_RAW_MP_CONDITIONAL_BRANCH_ISSUED     0x016
#define PERF_COUNT_RAW_MP_INDIRECT_BRANCH_ISSUED        0x017
#define PERF_COUNT_RAW_MP_CACHE_BUSY_STALL              0x018
#define PERF_COUNT_RAW_MP_NAP_STALL                     0x019

#define PERF_COUNT_RAW_MP_SPR_STALL             0x01a
#define PERF_COUNT_RAW_MP_ALU_STALL             0x01b
#define PERF_COUNT_RAW_MP_LOAD_MISS_REPLAY_TRAP 0x01c
#define PERF_COUNT_RAW_TLB_CNT                  0x01d
#define PERF_COUNT_RAW_RD_CNT                   0x01e

#define PERF_COUNT_RAW_WR_CNT                   0x01f
#define PERF_COUNT_RAW_TLB_EXCEPTION            0x020
#define PERF_COUNT_RAW_RD_MISS                  0x021
#define PERF_COUNT_RAW_WR_MISS                  0x022
#define PERF_COUNT_RAW_SNP_INC_RD_CNT           0x023

#define PERF_COUNT_RAW_SNP_NINC_RD_CNT          0x024
#define PERF_COUNT_RAW_SNP_WR_CNT               0x025
#define PERF_COUNT_RAW_SNP_IO_RD_CNT            0x026
#define PERF_COUNT_RAW_SNP_IO_WR_CNT            0x027
#define PERF_COUNT_RAW_SNP_LOCAL_DRD_CNT        0x028

#define PERF_COUNT_RAW_LOCAL_WR_CNT             0x029
#define PERF_COUNT_RAW_LOCAL_IRD_CNT            0x02a
#define PERF_COUNT_RAW_REMOTE_DRD_CNT           0x02b
#define PERF_COUNT_RAW_REMOTE_WR_CNT            0x02c
#define PERF_COUNT_RAW_REMOTE_IRD_CNT           0x02d

#define PERF_COUNT_RAW_COH_INV_CNT              0x02e
#define PERF_COUNT_RAW_SNP_INC_RD_MISS          0x02f
#define PERF_COUNT_RAW_SNP_NINC_RD_MISS         0x030
#define PERF_COUNT_RAW_SNP_WR_MISS              0x031
#define PERF_COUNT_RAW_SNP_IO_RD_MISS           0x032

#define PERF_COUNT_RAW_SNP_IO_WR_MISS           0x033
#define PERF_COUNT_RAW_LOCAL_DRD_MISS           0x034
#define PERF_COUNT_RAW_LOCAL_WR_MISS            0x035
#define PERF_COUNT_RAW_LOCAL_IRD_MISS           0x036
#define PERF_COUNT_RAW_REMOTE_DRD_MISS          0x037

#define PERF_COUNT_RAW_REMOTE_WR_MISS           0x038
#define PERF_COUNT_RAW_REMOTE_IRD_MISS          0x039
#define PERF_COUNT_RAW_COH_INV_HIT              0x03a
#define PERF_COUNT_RAW_VIC_WB_CNT               0x03b
#define PERF_COUNT_RAW_TDN_STARVED              0x03c

#define PERF_COUNT_RAW_DMA_STARVED              0x03d
#define PERF_COUNT_RAW_MDN_STARVED              0x03e
#define PERF_COUNT_RAW_RTF_STARVED              0x03f
#define PERF_COUNT_RAW_IREQ_STARVED             0x040
#define PERF_COUNT_RAW_RRTF_STARVED             0x041

#define PERF_COUNT_RAW_UDN_PKT_SNT              0x054
#define PERF_COUNT_RAW_UDN_SNT                  0x055
#define PERF_COUNT_RAW_UDN_BUBBLE               0x056
#define PERF_COUNT_RAW_UDN_CONGESTION           0x057
#define PERF_COUNT_RAW_IDN_PKT_SNT              0x058

#define PERF_COUNT_RAW_IDN_SNT                  0x059
#define PERF_COUNT_RAW_IDN_BUBBLE               0x05a
#define PERF_COUNT_RAW_IDN_CONGESTION           0x05b
#define PERF_COUNT_RAW_MDN_PKT_SNT              0x05c
#define PERF_COUNT_RAW_MDN_SNT                  0x05d

#define PERF_COUNT_RAW_MDN_BUBBLE               0x05e
#define PERF_COUNT_RAW_MDN_CONGESTION           0x05f
#define PERF_COUNT_RAW_TDN_PKT_SNT              0x060
#define PERF_COUNT_RAW_TDN_SNT                  0x061
#define PERF_COUNT_RAW_TDN_BUBBLE               0x062

#define PERF_COUNT_RAW_TDN_CONGESTION           0x063
#define PERF_COUNT_RAW_VDN_PKT_SNT              0x064
#define PERF_COUNT_RAW_VDN_SNT                  0x065
#define PERF_COUNT_RAW_VDN_BUBBLE               0x066
#define PERF_COUNT_RAW_VDN_CONGESTION           0x067

#define PERF_COUNT_RAW_UDN_DMUX_STALL           0x068
#define PERF_COUNT_RAW_IDN_DMUX_STALL           0x069

#endif /* __tilepro__*/

#ifdef __tilegx__
#define PERF_COUNT_RAW_ZERO			0x180
#define PERF_COUNT_RAW_ONE			0x181
#define PERF_COUNT_RAW_PASS_WRITTEN		0x182
#define PERF_COUNT_RAW_FAIL_WRITTEN		0x183
#define PERF_COUNT_RAW_DONE_WRITTEN		0x184
#define PERF_COUNT_RAW_CBOX_QUIESCED		0x186

#define PERF_COUNT_RAW_L1D_FILL_STALL           0x0c4
#define PERF_COUNT_RAW_CBOX_FULL_STALL		0x0c5
#define PERF_COUNT_RAW_LOAD_HIT_STALL   	0x0c6
#define PERF_COUNT_RAW_LOAD_STALL		0x0c7
#define PERF_COUNT_RAW_ALU_SRC_STALL        	0x0c8
#define PERF_COUNT_RAW_IDN_SRC_STALL            0x0c9

#define PERF_COUNT_RAW_UDN_SRC_STALL		0x0ca
#define PERF_COUNT_RAW_MF_STALL                 0x0cb
#define PERF_COUNT_RAW_SLOW_SPR_STALL           0x0cc
#define PERF_COUNT_RAW_NETWORK_DEST_STALL       0x0cd
#define PERF_COUNT_RAW_INSTRUCTION_STALL        0x0ce
#define PERF_COUNT_RAW_PFB_HIT_IN_PFB		0x0cf

#define PERF_COUNT_RAW_PFB_HIT			0x0d0
#define PERF_COUNT_RAW_CBOX_RESP		0x0d1
#define PERF_COUNT_RAW_CBOX_REQ			0x0d3
#define PERF_COUNT_RAW_ITLB_MISS_INTERRUPT	0x0d4
#define PERF_COUNT_RAW_INTERRUPT		0x0d5
#define PERF_COUNT_RAW_ICACHE_FILL_PEND		0x0d6
#define PERF_COUNT_RAW_ICACHE_FILL		0x0d7

#define PERF_COUNT_RAW_WAY_MISPREDICT		0x0d8
#define PERF_COUNT_RAW_COND_BRANCH_PRED_CORRECT		0x0d9
#define PERF_COUNT_RAW_COND_BRANCH_PRED_INCORRECT	0x0da
#define PERF_COUNT_RAW_INSTRUCTION_BUNDLE	0x0db
#define PERF_COUNT_RAW_USED_RAS			0x0dd
#define PERF_COUNT_RAW_RAS_CORRECT		0x0de
#define PERF_COUNT_RAW_COND_BRANCH_NO_PREDICT	0x0df
#define PERF_COUNT_RAW_TLB			0x040
#define PERF_COUNT_RAW_READ			0x041

#define PERF_COUNT_RAW_WRITE			0x042
#define PERF_COUNT_RAW_TLB_EXCEPTION		0x043
#define PERF_COUNT_RAW_READ_MISS		0x044
#define PERF_COUNT_RAW_WRITE_MISS		0x045
#define PERF_COUNT_RAW_L1_OPCODE_VALID		0x046

#define PERF_COUNT_RAW_L1_OPCODE_LD_VALID	0x047
#define PERF_COUNT_RAW_L1_OPCODE_ST_VALID	0x048
#define PERF_COUNT_RAW_L1_OPCODE_ATOMIC_VALID	0x049
#define PERF_COUNT_RAW_L1_OPCODE_FLUSH_VALID	0x04a
#define PERF_COUNT_RAW_L1_OPCODE_INV_VALID	0x04b

#define PERF_COUNT_RAW_L1_OPCODE_FINV_VALID	0x04c
#define PERF_COUNT_RAW_L1_OPCODE_MF_VALID	0x04d
#define PERF_COUNT_RAW_L1_OPCODE_PFETCH_VALID	0x04e
#define PERF_COUNT_RAW_L1_OPCODE_WH64_VALID	0x04f
#define PERF_COUNT_RAW_L1_OPCODE_DTLBPR_VALID	0x050

#define PERF_COUNT_RAW_L1_OPCODE_FWB_VALID	0x051
#define PERF_COUNT_RAW_L1_OPCODE_LD_NON_TEMPORAL_VALID	0x052
#define PERF_COUNT_RAW_L1_OPCODE_ST_NON_TEMPORAL_VALID	0x053
#define PERF_COUNT_RAW_SNOOP_INCREMENT_READ	0x080

#define PERF_COUNT_RAW_SNOOP_NON_INCREMENT_READ	0x081
#define PERF_COUNT_RAW_SNOOP_WRITE		0x082
#define PERF_COUNT_RAW_SNOOP_IO_READ		0x083
#define PERF_COUNT_RAW_SNOOP_IO_WRITE		0x084
#define PERF_COUNT_RAW_LOCAL_DATA_READ		0x085

#define PERF_COUNT_RAW_LOCAL_WRITE		0x086
#define PERF_COUNT_RAW_LOCAL_INSTRUCTION_READ	0x087
#define PERF_COUNT_RAW_REMOTE_DATA_READ		0x088
#define PERF_COUNT_RAW_REMOTE_WRITE		0x089
#define PERF_COUNT_RAW_REMOTE_INSTRUCTION_READ	0x08a

#define PERF_COUNT_RAW_COHERENCE_INVALIDATION	0x08b
#define PERF_COUNT_RAW_SNOOP_INCREMENT_READ_MISS	0x08c
#define PERF_COUNT_RAW_SNOOP_NON_INCREMENT_READ_MISS	0x08d
#define PERF_COUNT_RAW_SNOOP_WRITE_MISS		0x08e
#define PERF_COUNT_RAW_SNOOP_IO_READ_MISS	0x08f

#define PERF_COUNT_RAW_SNOOP_IO_WRITE_MISS	0x090
#define PERF_COUNT_RAW_LOCAL_DATA_READ_MISS	0x091
#define PERF_COUNT_RAW_LOCAL_WRITE_MISS		0x092
#define PERF_COUNT_RAW_LOCAL_INSTRUCTION_READ_MISS	0x093
#define PERF_COUNT_RAW_REMOTE_DATA_READ_MISS	0x094

#define PERF_COUNT_RAW_REMOTE_WRITE_MISS	0x095
#define PERF_COUNT_RAW_REMOTE_INSTRUCTION_READ_MISS	0x096
#define PERF_COUNT_RAW_COHERENCE_INVALIDATION_HIT	0x097
#define PERF_COUNT_RAW_CACHE_WRITEBACK		0x098
#define PERF_COUNT_RAW_SDN_STARVED		0x099

#define PERF_COUNT_RAW_RDN_STARVED		0x09a
#define PERF_COUNT_RAW_QDN_STARVED		0x09b
#define PERF_COUNT_RAW_SKF_STARVED		0x09c
#define PERF_COUNT_RAW_RTF_STARVED		0x09d
#define PERF_COUNT_RAW_IREQ_STARVED		0x09e

#define PERF_COUNT_RAW_LOCAL_WRITE_BUFFER_ALLOC	0x0a1
#define PERF_COUNT_RAW_REMOTE_WRITE_BUFFER_ALLOC	0x0a2
#define PERF_COUNT_RAW_ARB_VALID		0x0a3
#define PERF_COUNT_RAW_MDF_WRITE		0x0a4
#define PERF_COUNT_RAW_LDB_READ			0x0a5

#define PERF_COUNT_RAW_L2_OPCODE_LD_VALID	0x0a6
#define PERF_COUNT_RAW_L2_OPCODE_ST_VALID	0x0a7
#define PERF_COUNT_RAW_L2_OPCODE_ATOMIC_VALID	0x0a8
#define PERF_COUNT_RAW_L2_OPCODE_FLUSH_VALID	0x0a9
#define PERF_COUNT_RAW_L2_OPCODE_INV_VALID	0x0aa

#define PERF_COUNT_RAW_L2_OPCODE_FINV_VALID	0x0ab
#define PERF_COUNT_RAW_L2_OPCODE_MF_VALID	0x0ac
#define PERF_COUNT_RAW_L2_OPCODE_PFETCH_VALID	0x0ad
#define PERF_COUNT_RAW_L2_OPCODE_WH64_VALID	0x0ae
#define PERF_COUNT_RAW_L2_OPCODE_FWB_VALID	0x0af

#define PERF_COUNT_RAW_L2_OPCODE_LD_NON_TEMPORAL_VALID	0x0b0
#define PERF_COUNT_RAW_L2_OPCODE_ST_NON_TEMPORAL_VALID	0x0b1
#define PERF_COUNT_RAW_L2_OPCODE_LD_NOFIL_VALID		0x0b2
#define PERF_COUNT_RAW_L2_OPCODE_LD_NOFIL_NON_TEMPORAL_VALID	0x0b3
#define PERF_COUNT_RAW_L2_OPCODE_RDN_VALID			0x0b4

#define PERF_COUNT_RAW_L2_OPCODE_QDN_VALID	0x0b5
#define PERF_COUNT_RAW_L2_OPCODE_IO_READ_VALID	0x0b6
#define PERF_COUNT_RAW_L2_OPCODE_IO_WRITE_VALID	0x0b7
#define PERF_COUNT_RAW_L2_OPCODE_I_STREAM_VALID	0x0b8
#define PERF_COUNT_RAW_L2_OPCODE_MDF_VALID	0x0b9

#define PERF_COUNT_RAW_L2_OPCODE_IREQ_IV_VALID	0x0ba
#define PERF_COUNT_RAW_UDN_PACKET_SENT		0x000
#define PERF_COUNT_RAW_UDN_WORD_SENT		0x001
#define PERF_COUNT_RAW_UDN_BUBBLE		0x002

#define PERF_COUNT_RAW_UDN_CONGESTION		0x003
#define PERF_COUNT_RAW_IDN_PACKET_SENT		0x004
#define PERF_COUNT_RAW_IDN_WORD_SENT		0x005
#define PERF_COUNT_RAW_IDN_BUBBLE		0x006
#define PERF_COUNT_RAW_IDN_CONGESTION		0x007

#define PERF_COUNT_RAW_RDN_PACKET_SENT		0x008
#define PERF_COUNT_RAW_RDN_WORD_SENT		0x009
#define PERF_COUNT_RAW_RDN_BUBBLE		0x00a
#define PERF_COUNT_RAW_RDN_CONGESTION		0x00b
#define PERF_COUNT_RAW_SDN_PACKET_SENT		0x00c

#define PERF_COUNT_RAW_SDN_WORD_SENT		0x00d
#define PERF_COUNT_RAW_SDN_BUBBLE		0x00e
#define PERF_COUNT_RAW_SDN_CONGESTION		0x00f
#define PERF_COUNT_RAW_QDN_PACKET_SENT		0x010
#define PERF_COUNT_RAW_QDN_WORD_SENT		0x011

#define PERF_COUNT_RAW_QDN_BUBBLE		0x012
#define PERF_COUNT_RAW_QDN_CONGESTION		0x013
#define PERF_COUNT_RAW_UDN_DEMUX_STALL		0x014
#define PERF_COUNT_RAW_IDN_DEMUX_STALL		0x015

#endif
