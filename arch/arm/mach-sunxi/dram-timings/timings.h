/*
 * sunxi DRAM timings
 *
 * (C) Copyright 2015-2016 Theobroma Systems Design und Consulting GmbH
 *                         Philipp Tomsich <philipp.tomsich@theobroma-systems.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SUNXI_DRAM_TIMINGS__
#define __SUNXI_DRAM_TIMINGS__

/* A number of DDR3 timings are given as "the greater of a fixed number of
   clock cycles (CK) or nanoseconds.  We express these using a structure
   that holds a cycle count and a duration in picoseconds (so we can model
   sub-ns timings, such as 7.5ns without losing precision or resorting to
   rounding up early. */
struct dram_timing
{
	u32 ck;
	u32 ps;
};

/* We use an array of entries that define the minimum/maximum cycle time 
   for each CL/CWL combination. */
struct dram_cl_cwl_entry
{
	u8 CL;
	u8 CWL;
	u32 tCKmin;  /* in ps */
	u32 tCKmax;  /* in ps */
};

struct dram_bin
{
	/* Timing information for each speed-bin */
	struct dram_cl_cwl_entry const * cl_cwl_table;
	u32 cl_cwl_numentries;

	/* For the timings, we try to keep the order and grouping used in
	   JEDEC Standard No. 79-3F */

	/* timings */
	u32 tREFI; /* in ns */
	u32 tRFC;  /* in ns */
	u32 tRAS;  /* in ps */

	/* command and address timing */
	u32 tDLLK; /* in nCK */
	struct dram_timing tRTP;
	struct dram_timing tWTR;
	u32 tWR;   /* in nCK */
	u32 tMRD;  /* in nCK */
	struct dram_timing tMOD;
	u32 tRCD;  /* in ps */
	u32 tRP;   /* in ps */
	u32 tRC;   /* in ps */
	u32 tCCD;  /* in nCK */
	struct dram_timing tRRD;
	u32 tFAW;  /* in ps */

	/* calibration timing */
	/* currently not used --- struct dram_timing tZQinit; */
	struct dram_timing tZQoper;
	struct dram_timing tZQCS;

	/* reset timing */
	/*  currently not used --- struct dram_timing tXPR; */

	/* self-refresh timings */
	struct dram_timing tXS;
	u32 tXSDLL; /* in nCK */
	/*  currently not used --- struct dram_timing tCKESR; */
	struct dram_timing tCKSRE;
	struct dram_timing tCKSRX;

	/* power-down timings */
	struct dram_timing tXP;
	struct dram_timing tXPDLL;
	struct dram_timing tCKE;

	/* write leveling timings */
	u32 tWLMRD;    /* min, in nCK */
	//	u32 tWLDQSEN;  /* min, in nCK */
	u32 tWLO;      /* max, in ns */
	//	u32 tWLOE;     /* max, in ns */

	//	u32 tCKDPX;  /* in nCK */
	//	u32 tCKCSX;  /* in nCK */
};


/* DRAM timings known to this library */
extern const struct dram_bin DDR3_1600K;


/* Utility functions to convert */
int dram_calculate_CL_CWL(const struct dram_bin* const, u32, u8*, u8*);


#endif

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * tab-width: 8
 * End:
 */
