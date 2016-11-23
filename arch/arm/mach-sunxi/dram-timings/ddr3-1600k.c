#include <common.h>
#include "timings.h"

/* Refer to JEDEC 79-3F for the source of the following contraints and timings:
 *   - "DDR3-1600 Speed Bins and Operating Conditions"
 *   - "Timing Parameters by Speed Bin"
 *   - "Refresh parameters by device density"
 */

const struct dram_cl_cwl_entry DDR3_1600K_cl_cwl_table[] =
	{ { .CL =  5, .CWL = 5, .tCKmin = 3000, .tCKmax = 3300 },
	  { .CL =  6, .CWL = 5, .tCKmin = 2500, .tCKmax = 3300 },
	  { .CL =  8, .CWL = 6, .tCKmin = 1875, .tCKmax = 2500 },
	  { .CL = 10, .CWL = 7, .tCKmin = 1500, .tCKmax = 1875 },
	  { .CL = 11, .CWL = 8, .tCKmin = 1250, .tCKmax = 1500 } };

const struct dram_bin DDR3_1600K = {
	.cl_cwl_table = DDR3_1600K_cl_cwl_table,
	.cl_cwl_numentries = sizeof(DDR3_1600K_cl_cwl_table) / sizeof(struct dram_cl_cwl_entry),
	/* timings */
	.tREFI = 7800,   /* 7.8us (up to 85 degC) */
	//	  .tRFC  = 260,    /* 260ns for 4GBit devices */ // 350ns @ 8GBit
	.tRFC = 350,
	.tRCD  = 13750,
	.tRP   = 13750,
	.tRC   = 48750,
	.tRAS  = 35000,

	.tDLLK = 512,
	.tRTP  = { .ck = 4, .ps = 7500 },
	.tWTR  = { .ck = 4, .ps = 7500 },
	.tWR   = 15,
	.tMRD  = 4,
	.tMOD  = { .ck = 12, .ps = 15000 },
	.tCCD  = 4,
	.tRRD  = { .ck = 4, .ps = 7500 },
	.tFAW  = 40,

	/* calibration timing */
	//		.tZQinit = { .ck = 512, .ps = 640000 },
	.tZQoper = { .ck = 256, .ps = 320000 },
	.tZQCS   = { .ck = 64,  .ps = 80000 },

	/* reset timing */
	//		.tXPR  = { .ck = 5, .ps = 10000 },

	/* self-refresh timings */
	.tXS  = { .ck = 5, .ps = 10000 },
	.tXSDLL = 512,
	.tCKSRE = { .ck = 5, .ps = 10000 },
	.tCKSRX = { .ck = 5, .ps = 10000 },

	/* power-down timings */
	.tXP = { .ck = 3, .ps = 6000 },
	.tXPDLL = { .ck = 10, .ps = 24000 },
	.tCKE = { .ck = 3, .ps = 5000 },

	/* write leveling timings */
	.tWLMRD = 40,
	//		.tWLDQSEN = 25,
	.tWLO = 7500,
};
