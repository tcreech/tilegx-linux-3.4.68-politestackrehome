
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
CBOX_QUIESCED					{ return raw_sym(PERF_COUNT_RAW_CBOX_QUIESCED); }

L1D_FILL_STALL					{ return raw_sym(PERF_COUNT_RAW_L1D_FILL_STALL); }
CBOX_FULL_STALL					{ return raw_sym(PERF_COUNT_RAW_CBOX_FULL_STALL); }
LOAD_HIT_STALL					{ return raw_sym(PERF_COUNT_RAW_LOAD_HIT_STALL); }
LOAD_STALL					{ return raw_sym(PERF_COUNT_RAW_LOAD_STALL); }
ALU_SRC_STALL					{ return raw_sym(PERF_COUNT_RAW_ALU_SRC_STALL); }
IDN_SRC_STALL					{ return raw_sym(PERF_COUNT_RAW_IDN_SRC_STALL); }

UDN_SRC_STALL					{ return raw_sym(PERF_COUNT_RAW_UDN_SRC_STALL); }
MF_STALL					{ return raw_sym(PERF_COUNT_RAW_MF_STALL); }
SLOW_SPR_STALL					{ return raw_sym(PERF_COUNT_RAW_SLOW_SPR_STALL); }
NETWORK_DEST_STALL				{ return raw_sym(PERF_COUNT_RAW_NETWORK_DEST_STALL); }
INSTRUCTION_STALL				{ return raw_sym(PERF_COUNT_RAW_INSTRUCTION_STALL); }
PFB_HIT_IN_PFB					{ return raw_sym(PERF_COUNT_RAW_PFB_HIT_IN_PFB); }

PFB_HIT						{ return raw_sym(PERF_COUNT_RAW_PFB_HIT); }
RESP						{ return raw_sym(PERF_COUNT_RAW_CBOX_RESP); }
CBOX_REQ					{ return raw_sym(PERF_COUNT_RAW_CBOX_REQ); }
ITLB_MISS_INTERRUPT				{ return raw_sym(PERF_COUNT_RAW_ITLB_MISS_INTERRUPT); }
INTERRUPT					{ return raw_sym(PERF_COUNT_RAW_INTERRUPT); }
ICACHE_FILL_PEND				{ return raw_sym(PERF_COUNT_RAW_ICACHE_FILL_PEND); }

ICACHE_FILL					{ return raw_sym(PERF_COUNT_RAW_ICACHE_FILL); }
WAY_MISPREDICT					{ return raw_sym(PERF_COUNT_RAW_WAY_MISPREDICT); }
COND_BRANCH_PRED_CORRECT			{ return raw_sym(PERF_COUNT_RAW_COND_BRANCH_PRED_CORRECT); }
COND_BRANCH_PRED_INCORRECT			{ return raw_sym(PERF_COUNT_RAW_COND_BRANCH_PRED_INCORRECT); }
INSTRUCTION_BUNDLE				{ return raw_sym(PERF_COUNT_RAW_INSTRUCTION_BUNDLE); }
USED_RAS					{ return raw_sym(PERF_COUNT_RAW_USED_RAS); }

RAS_CORRECT					{ return raw_sym(PERF_COUNT_RAW_RAS_CORRECT); }
COND_BRANCH_NO_PREDICT				{ return raw_sym(PERF_COUNT_RAW_COND_BRANCH_NO_PREDICT); }
TLB						{ return raw_sym(PERF_COUNT_RAW_TLB); }
READ						{ return raw_sym(PERF_COUNT_RAW_READ); }
WRITE						{ return raw_sym(PERF_COUNT_RAW_WRITE); }
TLB_EXCEPTION					{ return raw_sym(PERF_COUNT_RAW_TLB_EXCEPTION); }

READ_MISS					{ return raw_sym(PERF_COUNT_RAW_READ_MISS); }
WRITE_MISS					{ return raw_sym(PERF_COUNT_RAW_WRITE_MISS); }
L1_OPCODE_VALID					{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_VALID); }
L1_OPCODE_LD_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_LD_VALID); }
L1_OPCODE_ST_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_ST_VALID); }
L1_OPCODE_ATOMIC_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_ATOMIC_VALID); }

L1_OPCODE_FLUSH_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_FLUSH_VALID); }
L1_OPCODE_INV_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_INV_VALID); }
L1_OPCODE_FINV_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_FINV_VALID); }
L1_OPCODE_MF_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_MF_VALID); }
L1_OPCODE_PFETCH_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_PFETCH_VALID); }
L1_OPCODE_WH64_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_WH64_VALID); }

L1_OPCODE_DTLBPR_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_DTLBPR_VALID); }
L1_OPCODE_FWB_VALID				{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_FWB_VALID); }
L1_OPCODE_LD_NON_TEMPORAL_VALID			{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_LD_NON_TEMPORAL_VALID); }
L1_OPCODE_ST_NON_TEMPORAL_VALID			{ return raw_sym(PERF_COUNT_RAW_L1_OPCODE_ST_NON_TEMPORAL_VALID); }
SNOOP_INCREMENT_READ				{ return raw_sym(PERF_COUNT_RAW_SNOOP_INCREMENT_READ); }
SNOOP_NON_INCREMENT_READ			{ return raw_sym(PERF_COUNT_RAW_SNOOP_NON_INCREMENT_READ); }

SNOOP_WRITE					{ return raw_sym(PERF_COUNT_RAW_SNOOP_WRITE); }
SNOOP_IO_READ					{ return raw_sym(PERF_COUNT_RAW_SNOOP_IO_READ); }
SNOOP_IO_WRITE					{ return raw_sym(PERF_COUNT_RAW_SNOOP_IO_WRITE); }
LOCAL_DATA_READ					{ return raw_sym(PERF_COUNT_RAW_LOCAL_DATA_READ); }
LOCAL_WRITE					{ return raw_sym(PERF_COUNT_RAW_LOCAL_WRITE); }
LOCAL_INSTRUCTION_READ				{ return raw_sym(PERF_COUNT_RAW_LOCAL_INSTRUCTION_READ); }

REMOTE_DATA_READ				{ return raw_sym(PERF_COUNT_RAW_REMOTE_DATA_READ); }
REMOTE_WRITE					{ return raw_sym(PERF_COUNT_RAW_REMOTE_WRITE); }
REMOTE_INSTRUCTION_READ				{ return raw_sym(PERF_COUNT_RAW_REMOTE_INSTRUCTION_READ); }
COHERENCE_INVALIDATION				{ return raw_sym(PERF_COUNT_RAW_COHERENCE_INVALIDATION); }
SNOOP_INCREMENT_READ_MISS			{ return raw_sym(PERF_COUNT_RAW_SNOOP_INCREMENT_READ_MISS); }
SNOOP_NON_INCREMENT_READ_MISS			{ return raw_sym(PERF_COUNT_RAW_SNOOP_NON_INCREMENT_READ_MISS); }

SNOOP_WRITE_MISS				{ return raw_sym(PERF_COUNT_RAW_SNOOP_WRITE_MISS); }
SNOOP_IO_READ_MISS				{ return raw_sym(PERF_COUNT_RAW_SNOOP_IO_READ_MISS); }
SNOOP_IO_WRITE_MISS				{ return raw_sym(PERF_COUNT_RAW_SNOOP_IO_WRITE_MISS); }
LOCAL_DATA_READ_MISS				{ return raw_sym(PERF_COUNT_RAW_LOCAL_DATA_READ_MISS); }
LOCAL_WRITE_MISS				{ return raw_sym(PERF_COUNT_RAW_LOCAL_WRITE_MISS); }
LOCAL_INSTRUCTION_READ_MISS			{ return raw_sym(PERF_COUNT_RAW_LOCAL_INSTRUCTION_READ_MISS); }

REMOTE_DATA_READ_MISS				{ return raw_sym(PERF_COUNT_RAW_REMOTE_DATA_READ_MISS); }
REMOTE_WRITE_MISS				{ return raw_sym(PERF_COUNT_RAW_REMOTE_WRITE_MISS); }
REMOTE_INSTRUCTION_READ_MISS			{ return raw_sym(PERF_COUNT_RAW_REMOTE_INSTRUCTION_READ_MISS); }
COHERENCE_INVALIDATION_HIT			{ return raw_sym(PERF_COUNT_RAW_COHERENCE_INVALIDATION_HIT); }
CACHE_WRITEBACK					{ return raw_sym(PERF_COUNT_RAW_CACHE_WRITEBACK); }
SDN_STARVED					{ return raw_sym(PERF_COUNT_RAW_SDN_STARVED); }

RDN_STARVED					{ return raw_sym(PERF_COUNT_RAW_RDN_STARVED); }
QDN_STARVED					{ return raw_sym(PERF_COUNT_RAW_QDN_STARVED); }
SKF_STARVED					{ return raw_sym(PERF_COUNT_RAW_SKF_STARVED); }
RTF_STARVED					{ return raw_sym(PERF_COUNT_RAW_RTF_STARVED); }
IREQ_STARVED					{ return raw_sym(PERF_COUNT_RAW_IREQ_STARVED); }
LOCAL_WRITE_BUFFER_ALLOC			{ return raw_sym(PERF_COUNT_RAW_LOCAL_WRITE_BUFFER_ALLOC); }

REMOTE_WRITE_BUFFER_ALLOC			{ return raw_sym(PERF_COUNT_RAW_REMOTE_WRITE_BUFFER_ALLOC); }
ARB_VALID					{ return raw_sym(PERF_COUNT_RAW_ARB_VALID); }
MDF_WRITE					{ return raw_sym(PERF_COUNT_RAW_MDF_WRITE); }
LDB_READ					{ return raw_sym(PERF_COUNT_RAW_LDB_READ); }
L2_OPCODE_LD_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_LD_VALID); }
L2_OPCODE_ST_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_ST_VALID); }

L2_OPCODE_ATOMIC_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_ATOMIC_VALID); }
L2_OPCODE_FLUSH_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_FLUSH_VALID); }
L2_OPCODE_INV_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_INV_VALID); }
L2_OPCODE_FINV_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_FINV_VALID); }
L2_OPCODE_MF_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_MF_VALID); }
L2_OPCODE_PFETCH_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_PFETCH_VALID); }

L2_OPCODE_WH64_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_WH64_VALID); }
L2_OPCODE_FWB_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_FWB_VALID); }
L2_OPCODE_LD_NON_TEMPORAL_VALID			{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_LD_NON_TEMPORAL_VALID); }
L2_OPCODE_ST_NON_TEMPORAL_VALID			{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_ST_NON_TEMPORAL_VALID); }
L2_OPCODE_LD_NOFIL_VALID			{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_LD_NOFIL_VALID); }
L2_OPCODE_LD_NOFIL_NON_TEMPORAL_VALID		{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_LD_NOFIL_NON_TEMPORAL_VALID); }

L2_OPCODE_RDN_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_RDN_VALID); }
L2_OPCODE_QDN_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_QDN_VALID); }
L2_OPCODE_IO_READ_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_IO_READ_VALID); }
L2_OPCODE_IO_WRITE_VALID			{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_IO_WRITE_VALID); }
L2_OPCODE_I_STREAM_VALID			{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_I_STREAM_VALID); }
L2_OPCODE_MDF_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_MDF_VALID); }

L2_OPCODE_IREQ_IV_VALID				{ return raw_sym(PERF_COUNT_RAW_L2_OPCODE_IREQ_IV_VALID); }
UDN_PACKET_SENT					{ return raw_sym(PERF_COUNT_RAW_UDN_PACKET_SENT); }
UDN_WORD_SENT					{ return raw_sym(PERF_COUNT_RAW_UDN_WORD_SENT); }
UDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_UDN_BUBBLE); }
UDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_UDN_CONGESTION); }
IDN_PACKET_SENT					{ return raw_sym(PERF_COUNT_RAW_IDN_PACKET_SENT); }
IDN_WORD_SENT					{ return raw_sym(PERF_COUNT_RAW_IDN_WORD_SENT); }

IDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_IDN_BUBBLE); }
IDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_IDN_CONGESTION); }
RDN_PACKET_SENT					{ return raw_sym(PERF_COUNT_RAW_RDN_PACKET_SENT); }
RDN_WORD_SENT					{ return raw_sym(PERF_COUNT_RAW_RDN_WORD_SENT); }
RDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_RDN_BUBBLE); }
RDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_RDN_CONGESTION); }

SDN_PACKET_SENT					{ return raw_sym(PERF_COUNT_RAW_SDN_PACKET_SENT); }
SDN_WORD_SENT					{ return raw_sym(PERF_COUNT_RAW_SDN_WORD_SENT); }
SDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_SDN_BUBBLE); }
SDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_SDN_CONGESTION); }
QDN_PACKET_SENT					{ return raw_sym(PERF_COUNT_RAW_QDN_PACKET_SENT); }
QDN_WORD_SENT					{ return raw_sym(PERF_COUNT_RAW_QDN_WORD_SENT); }

QDN_BUBBLE					{ return raw_sym(PERF_COUNT_RAW_QDN_BUBBLE); }
QDN_CONGESTION					{ return raw_sym(PERF_COUNT_RAW_QDN_CONGESTION); }
UDN_DEMUX_STALL					{ return raw_sym(PERF_COUNT_RAW_UDN_DEMUX_STALL); }
IDN_DEMUX_STALL					{ return raw_sym(PERF_COUNT_RAW_IDN_DEMUX_STALL); }

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
