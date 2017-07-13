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
#include <sys/types.h>
#include <metal/irq.h>
#include <metal/atomic.h>
#include <metal/cpu.h>
#include <stdio.h>

#define IPI_DEV_NAME    "ff340000.ipi"
#define SHM_DEV_NAME    "3ed00000.shm"
#define BUS_NAME        "platform"
#define TTC0_DEV_NAME   "ff110000.timer"
/*  Apply this snippet to the device tree in an overlay so that
	Linux userspace can see and use TTC0:

	&TTC0 {
	compatible = "TTC0";
	status = "okay";
	};

*/
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

#define XTTCPS_CLK_CNTRL_OFFSET 0x0
#define XTTCPS_CNT_CNTRL_OFFSET 0xC

#define XTTCPS_COUNT_VALUE_OFFSET	 0x18

/* XTTCPS_CLK_CNTRL_OFFSET registers */
#define XTTCPS_INTERVAL_VAL_OFFSET	0x24
#define MAX_INTERVAL_VAL 0xFFFFFFFF

/* XTTCPS_CNT_CNTRL_DECR_MASK registers */
#define XTTCPS_CNT_CNTRL_RST_MASK  0x10U  /* reset timer value */
#define XTTCPS_CNT_CNTRL_MATCH_MASK 0x8U /* match mode */
#define XTTCPS_CNT_CNTRL_DECR_MASK 0x4U /* when this bit is high, timer counts down */
#define XTTCPS_CNT_CNTRL_INT_MASK 0x2U /* interval mode */
#define XTTCPS_CNT_CNTRL_DIS_MASK  0x1U  /* turn bit ON to stop timer */ 

#define MS_PER_SEC 						1000000
#define NS_PER_SEC 						1000000000

#define LPRINTF(format, ...) \
	printf("CLIENT> " format, ##__VA_ARGS__)

#define LPERROR(format, ...) LPRINTF("ERROR: " format, ##__VA_ARGS__)
	
#define IPI_TRIG_OFFSET 0x0
#define IPI_OBS_OFFSET  0x4
#define IPI_ISR_OFFSET  0x10
#define IPI_IMR_OFFSET  0x14
#define IPI_IER_OFFSET  0x18
#define IPI_IDR_OFFSET  0x1C

#define IPI_MASK        0x100

#define APU_TO_RPU_TIMER_OFFSET 0x8 
#define RPU_TO_APU_TIMER_OFFSET 0x4

#define NS_PER_S        (1000 * 1000 * 1000)

#define TTC0_2_OFFSET 0x8 
#define TTC0_1_OFFSET 0x4

#define NUM_TIMES  10

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

/**
 * @brief   ipi_shmem_task() - shared memory IPI demo
 *          This task will:
 *          * Get the timestamp and put it into the ping shared memory
 *          * Update the shared memory descriptor for the new available
 *            ping buffer.
 *          * Trigger IPI to notifty the remote.
 *          * Repeat the above steps until it sends out all the packages.
 *          * Monitor IPI interrupt, verify every received package.
 *          * After all the packages are received, it sends out shutdown
 *            message to the remote.
 *
 * @param[in] channel - hold shared memory and IPI devices
 * @return - return 0 on success, otherwise return error number indicating
 *  		 type of error.
 */
int ipi_shmem_task(struct channel_s *channel);
/**
 * @brief   atomic_shmem_task() - Shared memory atomic operation demo
 *          This task will:
 *          * Write to shared memory to notify the remote to start atomic add 
 *			  on the shared memory descriptor memory for 1000 times.
 *          * Start atomic add by 1 for 1000 times to first 32 bits of memory 
 *			  in the shared memory location at 3ed00000 which is pointed to by 
 *			  shm_io.
 *          * Wait for the remote to write to shared memory
 *          * Once it received the polling kick from the remote, it will check 
 *			  if the value stored in the shared memory for the atomic add is 
 *			  2000.
 *          * It will print if the atomic add test has passed or not.
 * @param[in]     channel- hold shared mem. device
 * @return - If setup failed, return the corresponding error number. Otherwise
 *          return 0 on success.
 */
int atomic_shmem_task(struct channel_s *channel);
/**
 * @brief ipi_latency_demo_task() - Show performance of  IPI with Libmetal.
 *        For NUM_TIMES times, repeatedly send an IPI from APU and then 
 *		  detect this IPI from RPU and measure the latency. Similarly, 
 *		  measure the latency from RPU to APU. Each iteration, record this 
 *		  latency and after the loop has finished, report the total 
 *		  latency in nanseconds.
 *		  Notes:
 *		  -The RPU will repeatedly wait for IPI from APU until APU 
 *		   notifies remote by changing the KEEPGOING value in shared 
 *		   memory.
 *		  -To further ensure the accuracy of the readings a different 
 *		   thread (i.e. the IRQ handler) will stop the timer measuring RPU
 *		   to APU latency.
 *
 * @param[in]     channel- hold IPI and timer devices
 * @return - 0 on success, error code if failure.
 */
int ipi_latency_demo_task(struct channel_s *channel);
/**
 * @brief shmem_task() - Show use of shared memory with Libmetal.
 *        For NUM_TIMES times, send message to RPU and notify RPU by 
 *		  writing to shared mem that RPU is polling. Once 
 *		  detected, RPU will then similarly write message and notify APU
 *		  and the APU will then verify the response. 
 *		  If the message does not match expected response, record error.
 *		  Afterwards, report test result and clean up.
 *		  Notes:
 *		  -The RPU will repeatedly wait for shared mem. from APU until APU 
 *		   notifies remote by changing the KEEP_GOING value in shared 
 *		   memory.
 *
 * @param[in]     channel- hold shared mem. device
 * @return - return 0 on success, otherwise return error number indicating
 *  		 type of error
 */
int shmem_task(struct channel_s *channel);
/**
 * @brief shmem_latency_demo_task() - Show performance of shared memory with Libmetal.
 *		  For 8, 512, and 1024 bytes, measure latency from block write to block read 
 *		  on remote side in shared memory. For each size, find average latency by 
 *		  running NUM_TIMES times and reporting the average latency for both
 *		  APU block write to RPU block read as well as RPU block write to APU
 *		  block read.
 *
 * @param[in]     channel- hold timer, IPI and shared memory device
 * @return - 0 on success, error code if failure.
 */
int shmem_latency_demo_task(struct channel_s *channel);
/**
 * @brief shmem_throughput_demo_task() - Show performance of shared memory with Libmetal.
 *		  Record average throughput for APU block read, write, RPU block 
 *		  read and write for sizes 1/2 K, 1K and 2K.
 *		  For each size, run 1000 times each operation and record average.
 * @param[in]    channel - hold shared memory and timer devices
 * @return - 0 on success, error code if failure.
 */
int shmem_throughput_demo_task(struct channel_s *channel);

/**
 * @breif wait_for_notified() - Loop until notified bit 
 *		  in channel is set.
 */
inline void  wait_for_notified(struct channel_s *channel)
{	
	unsigned int flags;

	do {

		flags = metal_irq_save_disable();
		if (!atomic_flag_test_and_set(&channel->notified)) {
			metal_irq_restore_enable(flags);
			break;
		}
		metal_cpu_yield();
		metal_irq_restore_enable(flags);
	} while(1);
	
}
