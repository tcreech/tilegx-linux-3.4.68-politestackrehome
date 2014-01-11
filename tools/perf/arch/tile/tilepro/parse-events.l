
%option prefix="parse_events_"

%{
#include <errno.h>
#include "../perf.h"
#include "parse-events-bison.h"
#include "parse-events.h"
#include "../arch/tile/perf.h"

static int raw_sym(int config)
{
	parse_events_lval.num = config;
	return PE_RAW;
}

static int __value(char *str, int base, int token)
{
	long num;

	errno = 0;
	num = strtoul(str, NULL, base);
	if (errno)
		return PE_ERROR;

	parse_events_lval.num = num;
	return token;
}

static int value(int base)
{
	return __value(parse_events_text, base, PE_VALUE);
}

static int raw(void)
{
	return __value(parse_events_text + 1, 16, PE_RAW);
}

static int str(int token)
{
	parse_events_lval.str = strdup(parse_events_text);
	return token;
}

static int sym(int type, int config)
{
	parse_events_lval.num = (type << 16) + config;
	return PE_VALUE_SYM;
}

static int term(int type)
{
	parse_events_lval.num = type;
	return PE_TERM;
}

%}

num_dec		[0-9]+
num_hex		0x[a-fA-F0-9]+
num_raw_hex	[a-fA-F0-9]+
name		[a-zA-Z_*?][a-zA-Z0-9_*?]*
modifier_event	[ukhpGH]{1,8}
modifier_bp	[rwx]

%%
cpu-cycles|cycles				{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES); }
stalled-cycles-frontend|idle-cycles-frontend	{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_STALLED_CYCLES_FRONTEND); }
stalled-cycles-backend|idle-cycles-backend	{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_STALLED_CYCLES_BACKEND); }
instructions					{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS); }
cache-references				{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_CACHE_REFERENCES); }
cache-misses					{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_CACHE_MISSES); }
branch-instructions|branches			{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_INSTRUCTIONS); }
branch-misses					{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_MISSES); }
bus-cycles					{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_BUS_CYCLES); }
ref-cycles					{ return sym(PERF_TYPE_HARDWARE, PERF_COUNT_HW_REF_CPU_CYCLES); }
cpu-clock					{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CPU_CLOCK); }
task-clock					{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_TASK_CLOCK); }
page-faults|faults				{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS); }
minor-faults					{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS_MIN); }
major-faults					{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_PAGE_FAULTS_MAJ); }
context-switches|cs				{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CONTEXT_SWITCHES); }
cpu-migrations|migrations			{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CPU_MIGRATIONS); }
alignment-faults				{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_ALIGNMENT_FAULTS); }
emulation-faults				{ return sym(PERF_TYPE_SOFTWARE, PERF_COUNT_SW_EMULATION_FAULTS); }

ZERO						{ return raw_sym(PERF_COUNT_RAW_ZERO); }
ONE						{ return raw_sym(PERF_COUNT_RAW_ONE); }
PASS_WRITTEN					{ return raw_sym(PERF_COUNT_RAW_PASS_WRITTEN); }
FAIL_WRITTEN					{ return raw_sym(PERF_COUNT_RAW_FAIL_WRITTEN); }
DONE_WRITTEN					{ return raw_sym(PERF_COUNT_RAW_DONE_WRITTEN); }

MP_BUNDLE_RETIRED				{ return raw_sym(PERF_COUNT_RAW_MP_BUNDLE_RETIRED); }
MP_BUNDLE_1_INSTR_RETIRED			{ return raw_sym(PERF_COUNT_RAW_MP_BUNDLE_1_INSTR_RETIRED); }
MP_BUNDLE_2_INSTR_RETIRED			{ return raw_sym(PERF_COUNT_RAW_MP_BUNDLE_2_INSTR_RETIRED); }
MP_BUNDLE_3_INSTR_RETIRED			{ return raw_sym(PERF_COUNT_RAW_MP_BUNDLE_3_INSTR_RETIRED); }
MP_UDN_READ_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_UDN_READ_STALL); }

MP_IDN_READ_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_IDN_READ_STALL); }
MP_SN_READ_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_SN_READ_STALL); }
MP_UDN_WRITE_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_UDN_WRITE_STALL); }
MP_IDN_WRITE_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_IDN_WRITE_STALL); }
MP_SN_WRITE_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_SN_WRITE_STALL); }

MP_DATA_CACHE_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_DATA_CACHE_STALL); }
MP_INSTRUCTION_CACHE_STALL			{ return raw_sym(PERF_COUNT_RAW_MP_INSTRUCTION_CACHE_STALL); }
MP_ICACHE_HIT_ISSUED				{ return raw_sym(PERF_COUNT_RAW_MP_ICACHE_HIT_ISSUED); }
MP_ITLB_HIT_ISSUED				{ return raw_sym(PERF_COUNT_RAW_MP_ITLB_HIT_ISSUED); }
MP_CONDITIONAL_BRANCH_MISSPREDICT		{ return raw_sym(PERF_COUNT_RAW_MP_CONDITIONAL_BRANCH_MISSPREDICT); }

MP_INDIRECT_BRANCH_MISSPREDICT			{ return raw_sym(PERF_COUNT_RAW_MP_INDIRECT_BRANCH_MISSPREDICT); }
MP_CONDITIONAL_BRANCH_ISSUED			{ return raw_sym(PERF_COUNT_RAW_MP_CONDITIONAL_BRANCH_ISSUED); }
MP_INDIRECT_BRANCH_ISSUED			{ return raw_sym(PERF_COUNT_RAW_MP_INDIRECT_BRANCH_ISSUED); }
MP_CACHE_BUSY_STALL				{ return raw_sym(PERF_COUNT_RAW_MP_CACHE_BUSY_STALL); }
MP_NAP_STALL					{ return raw_sym(PERF_COUNT_RAW_MP_NAP_STALL); }

MP_SPR_STALL					{ return raw_sym(PERF_COUNT_RAW_MP_SPR_STALL); }
MP_ALU_STALL					{ return raw_sym(PERF_COUNT_RAW_MP_ALU_STALL); }
MP_LOAD_MISS_REPLAY_TRAP			{ return raw_sym(PERF_COUNT_RAW_MP_LOAD_MISS_REPLAY_TRAP); }
TLB_CNT						{ return raw_sym(PERF_COUNT_RAW_TLB_CNT); }
RD_CNT						{ return raw_sym(PERF_COUNT_RAW_RD_CNT); }

WR_CNT						{ return raw_sym(PERF_COUNT_RAW_WR_CNT); }
TLB_EXCEPTION					{ return raw_sym(PERF_COUNT_RAW_TLB_EXCEPTION); }
RD_MISS						{ return raw_sym(PERF_COUNT_RAW_RD_MISS); }
WR_MISS						{ return raw_sym(PERF_COUNT_RAW_WR_MISS); }
SNP_INC_RD_CNT					{ return raw_sym(PERF_COUNT_RAW_SNP_INC_RD_CNT); }

SNP_NINC_RD_CNT					{ return raw_sym(PERF_COUNT_RAW_SNP_NINC_RD_CNT); }
SNP_WR_CNT					{ return raw_sym(PERF_COUNT_RAW_SNP_WR_CNT); }
SNP_IO_RD_CNT					{ return raw_sym(PERF_COUNT_RAW_SNP_IO_RD_CNT); }
SNP_IO_WR_CNT					{ return raw_sym(PERF_COUNT_RAW_SNP_IO_WR_CNT); }
SNP_LOCAL_DRD_CNT				{ return raw_sym(PERF_COUNT_RAW_SNP_LOCAL_DRD_CNT); }

LOCAL_WR_CNT					{ return raw_sym(PERF_COUNT_RAW_LOCAL_WR_CNT); }
LOCAL_IRD_CNT					{ return raw_sym(PERF_COUNT_RAW_LOCAL_IRD_CNT); }
REMOTE_DRD_CNT					{ return raw_sym(PERF_COUNT_RAW_REMOTE_DRD_CNT); }
REMOTE_WR_CNT					{ return raw_sym(PERF_COUNT_RAW_REMOTE_WR_CNT); }
REMOTE_IRD_CNT					{ return raw_sym(PERF_COUNT_RAW_REMOTE_IRD_CNT); }

COH_INV_CNT					{ return raw_sym(PERF_COUNT_RAW_COH_INV_CNT); }
SNP_INC_RD_MISS					{ return raw_sym(PERF_COUNT_RAW_SNP_INC_RD_MISS); }
SNP_NINC_RD_MISS				{ return raw_sym(PERF_COUNT_RAW_SNP_NINC_RD_MISS); }
SNP_WR_MISS					{ return raw_sym(PERF_COUNT_RAW_SNP_WR_MISS); }
SNP_IO_RD_MISS					{ return raw_sym(PERF_COUNT_RAW_SNP_IO_RD_MISS); }

SNP_IO_WR_MISS					{ return raw_sym(PERF_COUNT_RAW_SNP_IO_WR_MISS); }
LOCAL_DRD_MISS					{ return raw_sym(PERF_COUNT_RAW_LOCAL_DRD_MISS); }
LOCAL_WR_MISS					{ return raw_sym(PERF_COUNT_RAW_LOCAL_WR_MISS); }
LOCAL_IRD_MISS					{ return raw_sym(PERF_COUNT_RAW_LOCAL_IRD_MISS); }
REMOTE_DRD_MISS					{ return raw_sym(PERF_COUNT_RAW_REMOTE_DRD_MISS); }

REMOTE_WR_MISS					{ return raw_sym(PERF_COUNT_RAW_REMOTE_WR_MISS); }
REMOTE_IRD_MISS					{ return raw_sym(PERF_COUNT_RAW_REMOTE_IRD_MISS); }
COH_INV_HIT					{ return raw_sym(PERF_COUNT_RAW_COH_INV_HIT); }
VIC_WB_CNT					{ return raw_sym(PERF_COUNT_RAW_VIC_WB_CNT); }
TDN_STARVED					{ return raw_sym(PERF_COUNT_RAW_TDN_STARVED); }

DMA_STARVED					{ return raw_sym(PERF_COUNT_RAW_DMA_STARVED); }
MDN_STARVED					{ return raw_sym(PERF_COUNT_RAW_MDN_STARVED); }
RTF_STARVED					{ return raw_sym(PERF_COUNT_RAW_RTF_STARVED); }
IREQ_STARVED					{ return raw_sym(PERF_COUNT_RAW_IREQ_STARVED); }
RRTF_STARVED					{ return raw_sym(PERF_COUNT_RAW_RRTF_STARVED); }

UDN_PKT_SNT					{ return raw_sym(PERF_COUNT_RAW_UDN_PKT_SNT); }
UDN_SNT						{ return raw_sym(PERF_COUNT_RAW_UDN_SNT); }
UDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_UDN_BUBBLE); }
UDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_UDN_CONGESTION); }
IDN_PKT_SNT					{ return raw_sym(PERF_COUNT_RAW_IDN_PKT_SNT); }

IDN_SNT						{ return raw_sym(PERF_COUNT_RAW_IDN_SNT); }
IDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_IDN_BUBBLE); }
IDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_IDN_CONGESTION); }
MDN_PKT_SNT					{ return raw_sym(PERF_COUNT_RAW_MDN_PKT_SNT); }
MDN_SNT						{ return raw_sym(PERF_COUNT_RAW_MDN_SNT); }

MDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_MDN_BUBBLE); }
MDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_MDN_CONGESTION); }
TDN_PKT_SNT					{ return raw_sym(PERF_COUNT_RAW_TDN_PKT_SNT); }
TDN_SNT						{ return raw_sym(PERF_COUNT_RAW_TDN_SNT); }
TDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_TDN_BUBBLE); }

TDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_TDN_CONGESTION); }
VDN_PKT_SNT					{ return raw_sym(PERF_COUNT_RAW_VDN_PKT_SNT); }
VDN_SNT						{ return raw_sym(PERF_COUNT_RAW_VDN_SNT); }
VDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_VDN_BUBBLE); }
VDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_VDN_CONGESTION); }

UDN_DMUX_STALL					{ return raw_sym(PERF_COUNT_RAW_UDN_DMUX_STALL); }
IDN_DMUX_STALL					{ return raw_sym(PERF_COUNT_RAW_IDN_DMUX_STALL); }

L1-dcache|l1-d|l1d|L1-data		|
L1-icache|l1-i|l1i|L1-instruction	|
LLC|L2					|
dTLB|d-tlb|Data-TLB			|
iTLB|i-tlb|Instruction-TLB		|
branch|branches|bpu|btb|bpc		|
node					{ return str(PE_NAME_CACHE_TYPE); }

load|loads|read				|
store|stores|write			|
prefetch|prefetches			|
speculative-read|speculative-load	|
refs|Reference|ops|access		|
misses|miss				{ return str(PE_NAME_CACHE_OP_RESULT); }

	/*
	 * These are event config hardcoded term names to be specified
	 * within xxx/.../ syntax. So far we dont clash with other names,
	 * so we can put them here directly. In case the we have a conflict
	 * in future, this needs to go into '//' condition block.
	 */
config			{ return term(PARSE_EVENTS__TERM_TYPE_CONFIG); }
config1			{ return term(PARSE_EVENTS__TERM_TYPE_CONFIG1); }
config2			{ return term(PARSE_EVENTS__TERM_TYPE_CONFIG2); }
period			{ return term(PARSE_EVENTS__TERM_TYPE_SAMPLE_PERIOD); }
branch_type		{ return term(PARSE_EVENTS__TERM_TYPE_BRANCH_SAMPLE_TYPE); }

mem:			{ return PE_PREFIX_MEM; }
r{num_raw_hex}		{ return raw(); }
{num_dec}		{ return value(10); }
{num_hex}		{ return value(16); }

{modifier_event}	{ return str(PE_MODIFIER_EVENT); }
{modifier_bp}		{ return str(PE_MODIFIER_BP); }
{name}			{ return str(PE_NAME); }
"/"			{ return '/'; }
-			{ return '-'; }
,			{ return ','; }
:			{ return ':'; }
=			{ return '='; }

%%

int parse_events_wrap(void)
{
	return 1;
}
