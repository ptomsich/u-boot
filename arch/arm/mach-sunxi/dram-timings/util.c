#include <common.h>
#include "timings.h"


int dram_calculate_CL_CWL(const struct dram_bin * const para, u32 freq, u8* CL, u8* CWL)
{
	int i;
	const u32 tCK = 1000000 / freq;  /* cycle time in ps */
		
	for (i = 0; i < para->cl_cwl_numentries; ++i) {
		if ((para->cl_cwl_table[i].tCKmin <= tCK) && (tCK < para->cl_cwl_table[i].tCKmax)) {
			*CL = para->cl_cwl_table[i].CL;
			*CWL = para->cl_cwl_table[i].CWL;

			return 0;
		}
	}

	printf("DRAM timings: failed to find valid CL/CWL for %d MHz\n", freq);
	return 1;
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * tab-width: 8
 * End:
 */
