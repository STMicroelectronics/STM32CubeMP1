/**
  ******************************************************************************
  * @file    rsc_table.c
  * @author  MCD Application Team
  * @brief   Ressource table
  *
  *   This file provides a default resource table requested by remote proc to
  *  load the elf file. It also allows to add debug trace using a shared buffer.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/** @addtogroup RSC_TABLE
  * @{
  */

/** @addtogroup resource_table
  * @{
  */

/** @addtogroup resource_table_Private_Includes
  * @{
  */

#if defined(__ICCARM__)
#include <stddef.h> /* needed  for offsetof definition*/
#endif
#include "rsc_table.h"
#include "openamp/open_amp.h"

/**
  * @}
  */

/** @addtogroup resource_table_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup resource_table_Private_Defines
  * @{
  */

/* Place resource table in special ELF section */
#if defined(__ICCARM__)
//#pragma section =".resource_table"
#elif defined(__GNUC__)
#define __section_t(S)          __attribute__((__section__(#S)))
#define __resource              __section_t(.resource_table)
#endif

#define RPMSG_IPU_C0_FEATURES       1
#define VRING_COUNT         		2

/* VirtIO rpmsg device id */
#define VIRTIO_ID_RPMSG_            7

#if defined (__LOG_TRACE_IO_)
extern char system_log_buf[];

#if defined(__ICCARM__)
__root struct shared_resource_table resource_table @ ".resource_table" = {
	.version = 1,
	.num = 2,
	.reserved = {0, 0},
	.offset = {
		offsetof(struct shared_resource_table, vdev),
		offsetof(struct shared_resource_table, cm_trace),
	},

	/* Virtio device entry */
	.vdev= {
		RSC_VDEV, VIRTIO_ID_RPMSG_, 0, RPMSG_IPU_C0_FEATURES, 0, 0, 0,
		VRING_COUNT, {0, 0},
	},

	/* Vring rsc entry - part of vdev rsc entry */
	.vring0 = {VRING_TX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING0_ID, 0},
	.vring1 = {VRING_RX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING1_ID, 0},

	.cm_trace = {
		RSC_TRACE,
		(uint32_t)system_log_buf, SYSTEM_TRACE_BUF_SZ, 0, "cm4_log",
	},
};

#elif defined(__GNUC__)

struct shared_resource_table __resource __attribute__((used)) resource_table = {

	.version = 1,
	.num = 2,
	.reserved = {0, 0},
	.offset = {
		offsetof(struct shared_resource_table, vdev),
		offsetof(struct shared_resource_table, cm_trace),
	},
	/* Virtio device entry */
	.vdev= {
		RSC_VDEV, VIRTIO_ID_RPMSG_, 0, RPMSG_IPU_C0_FEATURES, 0, 0, 0,
		VRING_COUNT, {0, 0},
	},

	/* Vring rsc entry - part of vdev rsc entry */
	.vring0 = {VRING_TX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING0_ID, 0},
	.vring1 = {VRING_RX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING1_ID, 0},

	.cm_trace = {
		RSC_TRACE,
		(uint32_t)system_log_buf, SYSTEM_TRACE_BUF_SZ, 0, "cm4_log",
	},
};
#endif

#else /* !__LOG_TRACE_IO_ */

#if defined(__ICCARM__)
__root struct shared_resource_table resource_table @ ".resource_table" = {
	.version = 1,
	.num = 1,
	.reserved = {0, 0},
	.offset = { offsetof(struct shared_resource_table, vdev) },

	/* Virtio device entry */
	.vdev= {
		RSC_VDEV, VIRTIO_ID_RPMSG_, 0, RPMSG_IPU_C0_FEATURES, 0, 0, 0,
		VRING_COUNT, {0, 0},
	},

	/* Vring rsc entry - part of vdev rsc entry */
	.vring0 = {VRING_TX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING0_ID, 0},
	.vring1 = {VRING_RX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING1_ID, 0},
};

#elif defined(__GNUC__)
struct shared_resource_table __resource __attribute__((used)) resource_table = {
	.version = 1,
	.num = 1,
	.reserved = {0, 0},
	.offset = { offsetof(struct shared_resource_table, vdev) },

	/* Virtio device entry */
	.vdev= {
		RSC_VDEV, VIRTIO_ID_RPMSG_, 0, RPMSG_IPU_C0_FEATURES, 0, 0, 0,
		VRING_COUNT, {0, 0},
	},

	/* Vring rsc entry - part of vdev rsc entry */
	.vring0 = {VRING_TX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING0_ID, 0},
	.vring1 = {VRING_RX_ADDRESS, VRING_ALIGNMENT, VRING_NUM_BUFFS, VRING1_ID, 0},
};
#endif


#endif /* __LOG_TRACE_IO_ */

void resource_table_init(int RPMsgRole, void **table_ptr, int *length)
{
  (void)RPMsgRole;
  *length = sizeof(resource_table);
  *table_ptr = &resource_table;
}
