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

 /*****************************************************************************
  * shmem_throughput_demo_task.c
  * This demo acts as the remote side to receive to the shared mem. throughput demo
  * run in Linux userspace.
  * This demo demo does so via the following:
  * Try to initialize shared memory and TTC0 libmetal resources.
  * If intialization fails, cleanup libmetal resources. Otherwise At APU
  * signal, record average throughput for target sizes: 1/2K, 1K and 2K
  * by timing block write and read operations to shared memory 1000 times
  * each and recording average throughput.
  * After the loop, cleanup libmetal resources.
  */

#include <sys/time.h>
#include <sys/types.h>
#include "tasks.h"

#define TIMER_KEEP_GOING_OFFSET 			32
#define TEST_STATUS_OFFSET 					64
// #define START_NEW_TEST_OFFSET 				96
#define TEST_SIZE_OFFSET 					128
#define DATA_LOCATION_OFFSET 				160
#define SHMEM_THROUGHPUT_RPU_TO_APU_OFFSET 	192
#define SHMEM_THROUGHPUT_APU_TO_RPU_OFFSET 	96
#define RESPOND 							1
#define START 								2
/* number of packet sizes to measure */
#define NUM_SIZES 							3
#define DONE_TESTING 						5
#define START_TESTING 						6
#define START_NEW_TEST 						8
#define START_READ 							9
#define KEEP_GOING 							10
#define TOTAL_ITERATIONS					-1
#define MS_PER_SEC 							1000000
#define TTC0_2_OFFSET 						0x8
#define TTC0_1_OFFSET 						0x4
#define XTTCPS_COUNT_VALUE_OFFSET 			0x18

/**
 * @brief start_timer() - function to start or enable TTC timer
 *        Turn off the disable bit in the Count Control Reg, which
 *		  results in the timer to start or resume counting.
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel - channel pointing to timer
 */
static void start_timer(uint32_t offset,  struct channel_s* channel)
{
	uint32_t val;
	val = metal_io_read32(channel->timer_io,
		offset+CNT_CNTRL_TIMER_OFFSET);
	val &= ~XTTCPS_CNT_CNTRL_DIS_MASK;
	metal_io_write32(channel->timer_io, offset+CNT_CNTRL_TIMER_OFFSET, val);
}

/**
 * @brief stop_timer() - function to start or enable TTC timer
 *        Turn on the disable bit in the Count Control Reg, which
 *		  results in the timer to stop counting.
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel - channel pointing to timer
 */
static void stop_timer(uint32_t offset, struct channel_s* channel)
{
	metal_io_write32(channel->timer_io,
		offset+XTTCPS_CNT_CNTRL_OFFSET,
		metal_io_read32(channel->timer_io,
			offset+XTTCPS_CNT_CNTRL_OFFSET) | XTTCPS_CNT_CNTRL_DIS_MASK
		);
}

/**
 * @brief reset_timer() - function to reset count in TTC timer
 *        Turn on the RST bit in the Count Control Reg. Setting
 *		  this bit high resets the counter value and restarts counting.
 *		  the RST bit is automatically cleared on restart.
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel- hold timer device
 */
static  void reset_timer(uint32_t offset, struct channel_s*channel)
{
	uint32_t val;
	val = metal_io_read32(channel->timer_io, offset+XTTCPS_CNT_CNTRL_OFFSET);
	val |= XTTCPS_CNT_CNTRL_RST_MASK;
	metal_io_write32(channel->timer_io, offset+XTTCPS_CNT_CNTRL_OFFSET, val);
}

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
int shmem_throughput_demo_taskd(struct channel_s* channel)
{
	int i, len;
	int sizes[NUM_SIZES] = { 512, 1024, 2048 };
	void*data_location;
	uint32_t local_counter;

	LPRINTF("Starting shared mem throughput demo\n");

	/* for each data size, meaasure block read and write throughput */
	for (i = 0; i < NUM_SIZES; i++){
		len = sizes[i];
		data_location = metal_allocate_memory(len * sizeof(char));
		if (!data_location){
			LPRINTF("Unable to malloc %i bytes\n", len);
			return -ENOSPC;
		}
		memset(data_location,1,len);
		LPRINTF("starting measurement with size %i\n", len);

		/* block read */
		reset_timer(TTC0_2_OFFSET, channel);
		start_timer(TTC0_2_OFFSET, channel);
		for (local_counter = 0; local_counter < NUM_TIMES; ){
			wait_for_notified(channel);
			atomic_thread_fence(memory_order_acq_rel);
			while (local_counter < metal_io_read32(channel->shm_io, SHMEM_THROUGHPUT_APU_TO_RPU_OFFSET)){
				metal_io_block_read(channel->shm_io, DATA_LOCATION_OFFSET, data_location, len);
				local_counter++;
			}
		}
		stop_timer(TTC0_2_OFFSET, channel);
		/* notify remote to start their block read */
		metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);

		/* block write */
		metal_io_write32(channel->shm_io, SHMEM_THROUGHPUT_RPU_TO_APU_OFFSET, 0);
		reset_timer(TTC0_2_OFFSET, channel);
		start_timer(TTC0_2_OFFSET, channel);
		for(local_counter = 0; local_counter < NUM_TIMES ; local_counter++ ){
			metal_io_block_write(channel->shm_io, DATA_LOCATION_OFFSET, data_location, len);
			metal_io_write32(channel->shm_io, SHMEM_THROUGHPUT_RPU_TO_APU_OFFSET, local_counter+1);
			metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
		}
		stop_timer(TTC0_2_OFFSET, channel);

		metal_free_memory(data_location);
		LPRINTF("Done with current measurement\n");

	}

	LPRINTF("Finished shared mem throughput demo\n");
	return 0;
}
