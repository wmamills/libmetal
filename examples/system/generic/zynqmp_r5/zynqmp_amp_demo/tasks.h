 /*
 * Copyright (c) 2017, Xilinx Inc. and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Xilinx nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <metal/atomic.h>
#include <metal/alloc.h>
#include <metal/irq.h>
#include <errno.h>
#include <metal/sys.h>
#include <metal/cpu.h>
#include <metal/io.h>
#include <metal/device.h>
#include <sys/types.h>
#include "sys_init.h"

#include "xparameters.h"

#define IPI_DEV_NAME    "ff310000.ipi"
#define SHM_DEV_NAME    "3ed00000.shm"
#define TTC0_DEV_NAME   "ff110000.ttc"
#define BUS_NAME        "generic"


/* Shared memory 	0x3ed00000 - 0x3ed40000*/
/* APU to RPU 		0x3ed40000 - 0x3ed50000 */
/* APU to RPU 		0x3ed50000 - 0x3ed60000 */
/* Used for IPI wth share mem. demo */
#define SHM_OFFSET 		0x00000
#define D0_SHM_OFFSET   0x00000
#define D1_SHM_OFFSET   0x20000
#define SHM0_OFFSET 	0x40000
#define SHM1_OFFSET  	0x50000
#define SHM_SIZE 	  	  60000


#define IPI_TRIG_OFFSET 0x0
#define IPI_OBS_OFFSET  0x4
#define IPI_ISR_OFFSET  0x10
#define IPI_IMR_OFFSET  0x14
#define IPI_IER_OFFSET  0x18
#define IPI_IDR_OFFSET  0x1C

#define IPI_MASK        0x1000000

#define MAX_INTERVAL_VAL 0xFFFFFFFF
#define XTTCPS_COUNT_VALUE_OFFSET	 0x18

#define CNT_CNTRL_TIMER_OFFSET 0xC
#define XTTCPS_CNT_CNTRL_RST_MASK  0x10U  /* reset timer value */
#define XTTCPS_CNT_CNTRL_DIS_MASK  0x1U  /* turn bit ON to stop timer */
#define XTTCPS_CLK_CNTRL_OFFSET 0x0
#define XTTCPS_CNT_CNTRL_OFFSET 0xC
#define APU_TO_RPU_TIMER_OFFSET 0x8
#define RPU_TO_APU_TIMER_OFFSET 0x4

#ifndef APU_RPU_OFFSETS_GUARD
#define MESSAGE_OFFSET_APU_TO_RPU 0
#define MESSAGE_OFFSET_RPU_TO_APU 256
#endif

#define NUM_TIMES  10

#define LPRINTF(format, ...) \
  xil_printf("\r\nSERVER> " format, ##__VA_ARGS__)

#define LPERROR(format, ...) LPRINTF("ERROR: " format, ##__VA_ARGS__)

struct channel_s {
	struct metal_device *ipi_dev;
	struct metal_io_region *ipi_io;
	unsigned int ipi_mask;
	struct metal_device *shm_dev;
	struct metal_io_region *shm_io;
	atomic_int notified;
	unsigned long d0_start_offset;
	unsigned long d1_start_offset;
	struct metal_io_region *timer_io;
	struct metal_device *timer_device;
};

extern void wait_for_interrupt();

/**
 * @breif wait_for_notified() - Loop until notified bit
 *		  in channel is set.
 * @param[in]     ch - channel pointing to shared memory device
 */
inline void  wait_for_notified(struct channel_s* channel)
{
	unsigned int flags;

	do {
			flags = metal_irq_save_disable();
			if (!atomic_flag_test_and_set(&channel->notified)) {
				metal_irq_restore_enable(flags);
				break;
			}
			wait_for_interrupt();
			metal_irq_restore_enable(flags);
		} while(1);
}

/**
 * @brief   atomic_shmem_taskd() - Shared memory atomic operation demo
 *          This task will:
 *          * Wait for the remote to write to shared memory.
 *          * Once it receives the notification via polling, start atomic add by
 *			  1 for 1000 times to first 32 bits of memory in the
 *			  shared memory location at 3ed00000 which is pointed to by shm_io.
 *          * Write to shared mem to notify the remote once it finishes calculation.
 *
 * @param[in]     channel - channel pointing to shared memory device
 * @return - If setup failed, return the corresponding error number. Otherwise
 *          return 0 on success.
 */
int atomic_shmem_taskd(struct channel_s* channel);
/**
 * @brief ipi_latency_demo_task() - Show performance of  IPI with Libmetal.
 *		  Loop until APU tells RPU to stop via shared memory.
 *		  In loop, wait for interrupt (interrupt handler stops APU to
 *		  RPU timer). Then reset count on RPU to APU timer to 0, start
 *		  counting and send interrupt to notify APU.
 *
 * @param[in] channel - channel holding timer device
 * @return - 0 on success, error code if failure.
 */
int ipi_latency_demo_taskd(struct channel_s * channel);
/**
 * @brief   ipi_shmem_taskd() - shared memory IPI demo
 *          This task will:
 *          * Wait for IPI interrupt from the remote
 *          * Once it received the interrupt, copy the content from
 *            the ping buffer to the pong buffer.
 *          * Update the shared memory descriptor for the new available
 *            pong buffer.
 *          * Trigger IPI to notifty the remote.
 *
 * @param[in] channel - channel information
 */
int ipi_shmem_taskd(struct channel_s *channel);
/**
 * @brief shmem_task() - Show use of shared memory with Libmetal.
 *		  Until KEEP_GOING signal is stopped, keep looping.
 *		  In the loop, read message from remote, add one to message and then
 *		  respond. After the loop, cleanup resources.
 *
 * @param[in]     channel - channel holding and shared mem device
 * @return - return 0 on success, otherwise return error number indicating
 *  		 type of error
 */
int shmem_taskd(struct channel_s* channel);
/**
 * @brief shmem_latency_demo_taskd() - Show performance of  shared mem. with Libmetal.
 *		  Loop until APU tells RPU to stop via shared memory.
 *		  In loop, wait for interrupt (interrupt handler stops APU to
 *		  RPU timer). Then reset count on RPU to APU timer to 0, start
 *		  counting and send interrupt to notify APU.
 *
 * @param[in] channel - channel pointing to timer and shared memory devices
 * @return - 0 on success, error code if failure.
 */
int shmem_latency_demo_taskd(struct channel_s *channel);
/**
 * @brief shmem_throughput_demo_taskd() - Show throughput of shared mem. with Libmetal.
 *		  At signal of remote, record total time to do block read and write operation
 *		  Loop until APU tells RPU to stop via shared memory.
 *		  In loop, wait for interrupt (interrupt handler stops APU to
 *		  RPU timer). Then reset count on RPU to APU timer to 0, start
 *		  counting and send interrupt to notify APU.
 * @param[in]     channel - channel holding timer, IPI and shared mem devices
 * @return - 0 on success, error code if failure.
 */
int shmem_throughput_demo_taskd(struct channel_s* channel);
