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
  * This demo demonstrates the shared mem. throughput between the APU and RPU.
  * This demo does so via the following steps:
  * 1. Initialize two of the three timers found on TTC0 and shared memory.
  * 2. At 1/2 K bytes, 1K and 2K, measure throughput of shared mem writing 
  		one of the target sizes to the other side. In other words, for each
  		target size, time a block write and read operation from APU to RPU 1000 times
  		and report the average throughput. Similarly, find time for RPU to APU.
  * 4. If there is an error during initialization, cleanup the libmetal 
  *		resources. Otherwise, cleanup after the report.
  */
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <metal/atomic.h>
#include <metal/cpu.h>
#include <metal/io.h>
#include <sys/time.h>
#include <sys/types.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <metal/alloc.h>
#include <errno.h>

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
#define XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ 		100000000

/**
 * @brief start_timer() - function to start or enable TTC timer
 *        Turn off the disable bit in the Count Control Reg, which 
 *		  results in the timer to start or resume counting. 
 *
 * @param[in]     timer_base_offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel- hold timer device
 */
 static void start_timer(uint32_t timer_base_offset, struct channel_s*channel)
{
	uint32_t val;
	val = metal_io_read32(channel->timer_io, timer_base_offset+XTTCPS_CNT_CNTRL_OFFSET);
	val &= ~XTTCPS_CNT_CNTRL_DIS_MASK;
	metal_io_write32(channel->timer_io, timer_base_offset+XTTCPS_CNT_CNTRL_OFFSET, val);
}

/**
 * @brief read_timer() - return current counter value in TTC0 timer 
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel- hold timer device
 */
static  uint32_t read_timer(uint32_t offset, struct channel_s*channel)
{
	return metal_io_read32(channel->timer_io, offset+XTTCPS_COUNT_VALUE_OFFSET);
}

/**
 * @brief reset_timer() - function to reset count in TTC timer
 *        Turn on the RST bit in the Count Control Reg. Setting 
 *		  this bit high resets the counter value and restarts counting; 
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
 * @brief stop_timer() - function to start or enable TTC timer
 *        Turn on the disable bit in the Count Control Reg, which 
 *		  results in the timer to stop counting. 
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel- hold timer device
 */
static void stop_timer(uint32_t offset, struct channel_s*channel)
{
	metal_io_write32(channel->timer_io, offset+XTTCPS_CNT_CNTRL_OFFSET, 
		metal_io_read32(channel->timer_io, 
			offset+XTTCPS_CNT_CNTRL_OFFSET) | XTTCPS_CNT_CNTRL_DIS_MASK
		);
}

/**
 * @brief shmem_throughput_demo_task() - Show performance of shared memory with Libmetal.
 *		  Record average throughput for APU block read, write, RPU block 
 *		  read and write for sizes 1/2 K, 1K and 2K.
 *		  For each size, run 1000 times each operation and record average.
 * @param[in]    channel - hold shared memory and timer devices
 * @return - 0 on success, error code if failure.
 */
int shmem_throughput_demo_task(struct channel_s* channel)
{
	int i, len;
	int sizes[NUM_SIZES] = { 512, 1024, 2048 };
	void*data_location;
	uint32_t local_counter;

	LPRINTF("Starting shared mem throughput task\n");
	/* for each data size, meaasure block read and write throughput */
	for (i = 0; i < NUM_SIZES; i++){
		LPRINTF("Starting test with size: %i\n", sizes[i]);
		len = sizes[i];
		data_location = metal_allocate_memory(len * sizeof(char));
		/* check if metal_allocate_memory worked */
		if (!data_location){
			LPRINTF("Unable to malloc %i bytes\n", len);
			return -ENOSPC;
		}
		memset(data_location,1,len);

		/* block write */
		metal_io_write32(channel->shm_io, SHMEM_THROUGHPUT_APU_TO_RPU_OFFSET, 0);
		reset_timer(TTC0_1_OFFSET, channel);
		start_timer(TTC0_1_OFFSET, channel);
		for(local_counter = 0; local_counter < NUM_TIMES ; local_counter++ ){
			metal_io_block_write(channel->shm_io, DATA_LOCATION_OFFSET, data_location, len);
			metal_io_write32(channel->shm_io, SHMEM_THROUGHPUT_APU_TO_RPU_OFFSET, local_counter+1);
			metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
		}
		stop_timer(TTC0_1_OFFSET, channel);

		LPRINTF("Shared mem throughput report for size: %u\n", len);
		LPRINTF("APU block write: %u bytes/s\n", 
			len * XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ / (read_timer(TTC0_1_OFFSET, channel) ) );
		
		/* wait for remote to finish block read */
		wait_for_notified(channel);
		
		LPRINTF("RPU block read: %u bytes/s\n", 
			len * XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ / (read_timer(TTC0_2_OFFSET, channel) ) );

		/* block read */
		reset_timer(TTC0_1_OFFSET, channel);
		start_timer(TTC0_1_OFFSET, channel);
		for (local_counter = 0; local_counter < NUM_TIMES; ){
			wait_for_notified(channel);
			atomic_thread_fence(memory_order_acq_rel);
			while (local_counter < metal_io_read32(channel->shm_io, SHMEM_THROUGHPUT_RPU_TO_APU_OFFSET)){
				metal_io_block_read(channel->shm_io, DATA_LOCATION_OFFSET, data_location, len);
				local_counter++;
				
			}
		}
		stop_timer(TTC0_1_OFFSET, channel);
		LPRINTF("APU block read: %u bytes/s\n", 
			len * XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ / (read_timer(TTC0_2_OFFSET, channel) ) );
		LPRINTF("RPU block write: %u bytes/s\n", 
			len * XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ / (read_timer(TTC0_1_OFFSET, channel) ) );
		
		metal_free_memory(data_location);
	}

	LPRINTF("Finished shared mem throughput task\n");
	return 0;

}
