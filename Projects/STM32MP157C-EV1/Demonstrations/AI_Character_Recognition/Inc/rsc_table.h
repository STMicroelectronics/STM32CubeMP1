/*
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 */

/* This file populates resource table for BM remote
 * for use by the Linux Master */

#ifndef RSC_TABLE_H_
#define RSC_TABLE_H_

#include "openamp/open_amp.h"
#include "openamp_conf.h"

/* Place resource table in special ELF section */
//#define __section_t(S)          __attribute__((__section__(#S)))
//#define __resource              __section_t(.resource_table)




/* Resource table for the given remote */
struct shared_resource_table {
	unsigned int version;
	unsigned int num;
	unsigned int reserved[2];
	unsigned int offset[NUM_RESOURCE_ENTRIES];
	/* text carveout entry */

	/* rpmsg vdev entry */
	struct fw_rsc_vdev vdev;
	struct fw_rsc_vdev_vring vring0;
	struct fw_rsc_vdev_vring vring1;
	struct fw_rsc_trace cm_trace;
};

void resource_table_init(int RPMsgRole, void **table_ptr, int *length);

#endif /* RSC_TABLE_H_ */

