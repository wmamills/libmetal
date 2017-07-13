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
  * shmem_latency_task.c
  * This demo demonstrates the shared mem. latency between the APU and RPU.
  * This demo does so via the following steps:
  * 1. Initialize two of the three timers found on TTC0 and shared memory.
  * 2. At 8 bytes, 1/2 K and 1K, measure latency of shared mem latency.
  		Do this by finding average latency for each size. For 1000 times,
  		the APU will set the first timer in TTC0 to 0 and 
  *		write to shared memory section then notify remote. The RPU will then
  * 	timer to record the APU to RPU latency. Then the RPU will reset the
  *     second timer in TTC0 to 0, start this timer and similarly notify 
  *		remote. Finally, the APU will detect this and measure this latency.
  * 3. After the loop, report the average latency each way in nanoseconds.
  * 4. If there is an error during initialization, cleanup the libmetal 
  *		resources. Otherwise, cleanup after the report.
  */
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <metal/atomic.h>
#include <metal/cpu.h>
#include <metal/alloc.h>
#include <metal/io.h>
#include <sys/time.h>
#include <sys/types.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <errno.h>

#include "tasks.h"
#define DONE_TESTING 					1 /* end demo */
/* number of packet sizes to measure */
#define NUM_SIZES 						3
#define TEST_STATUS_OFFSET 				64
#define TEST_SIZE_OFFSET 				128 
#define DATA_LOCATION_OFFSET 			160
#define NS_PER_SEC 						1000000000
#define XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ 	100000000

extern void wait_for_interrupt(void);

/**
 * @brief start_timer() - function to start or enable TTC timer
 *        Turn off the disable bit in the Count Control Reg, which 
 *		  results in the timer to start or resume counting. 
 *
 * @param[in]     timer_base_offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel- hold timer device
 */
 static void start_timer(struct channel_s* channel, uint32_t timer_base_offset)
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
static  uint32_t read_timer(struct channel_s *channel, uint32_t offset)
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
static  void reset_timer(struct channel_s*channel, uint32_t offset)
{
	uint32_t val;
	val = metal_io_read32(channel->timer_io, offset+XTTCPS_CNT_CNTRL_OFFSET);
	val |= XTTCPS_CNT_CNTRL_RST_MASK;
	metal_io_write32(channel->timer_io, offset+XTTCPS_CNT_CNTRL_OFFSET, val);
} 

/**
 *
 * @brief stop_timer() - function to start or enable TTC timer
 *        Turn on the disable bit in the Count Control Reg, which 
 *		  results in the timer to stop counting. 
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel- hold timer device
 */
static void stop_timer(struct channel_s*channel, uint32_t offset)
{
	uint32_t val;
	val = metal_io_read32(channel->timer_io, offset + XTTCPS_CNT_CNTRL_OFFSET);
	metal_io_write32(channel->timer_io, offset + XTTCPS_CNT_CNTRL_OFFSET, 
		val | XTTCPS_CNT_CNTRL_DIS_MASK);
}


/** @brief generate_latency_measurement Generate latency measurement for size
*			Generate latency and store in table
* @param[in]	 len - size in bytes to measure latency for 
* @param[in]     channel- hold timer device
* @return - ptr to array of times on success, NULL if failure.
*			times[0] will hold APU to RPU latency, and times[1] will hold RPU
*			to APU latency.
*/
static uint32_t* generate_latency_measurement(uint32_t len, struct channel_s * channel)
{
	int i;
	void*data_location;
	uint32_t * times; /* hold two times for apu->rpu and rpu->apu latency */
	
	data_location = metal_allocate_memory(len * sizeof(char));
	if (!data_location){
		LPERROR("Unable to malloc %i bytes\n", len);
		return NULL;
	}

	times = metal_allocate_memory(2 * sizeof(uint32_t));
	if (!data_location){
		LPERROR("Unable to malloc %i uint32_t's\n", 2);
		return NULL;
	}	

	/* tell remote how big src is */
	metal_io_write32(channel->shm_io, TEST_SIZE_OFFSET, len);
	for ( i = 0; i < NUM_TIMES; i++){
		/* set count back to 0, then start couting again */
		reset_timer(channel, APU_TO_RPU_TIMER_OFFSET);
		start_timer(channel, APU_TO_RPU_TIMER_OFFSET);
		metal_io_block_write(channel->shm_io, DATA_LOCATION_OFFSET, data_location, len);
		/* kick remote */
		metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
		/* wait for response; other side stopped latency timer for apu to rpu */
		wait_for_notified(channel);
		/* now measure rpu to apu */
		metal_io_block_read(channel->shm_io, DATA_LOCATION_OFFSET, data_location, len);
		stop_timer(channel, RPU_TO_APU_TIMER_OFFSET);
		/* latency_table[table_offset][1] is apu->rpu latency, and [2] is vice versa*/
		times[0] += read_timer(channel, APU_TO_RPU_TIMER_OFFSET);
		times[1] += read_timer(channel, RPU_TO_APU_TIMER_OFFSET);	
	}
	LPRINTF("Finishing test with size: %i\n", len);
	metal_free_memory(data_location);
	return times;
}


/**
 * @brief shmem_latency_demo_task() - Show performance of shared memory with Libmetal.
 *		  For 8, 512, and 1024 bytes, measure latency from block write to block read 
 *		  on remote side in shared memory. For each size, find average latency by 
 *		  running NUM_TIMES times and reporting the average latency for both
 *		  APU block write to RPU block read as well as RPU block write to APU
 *		  block read.
 *
 * @param[in]     channel- hold timer device
 * @return - 0 on success, error code if failure.
 */
int shmem_latency_demo_task(struct channel_s * channel)
{
	int sizes[NUM_SIZES] = {8,512,2048}; /* set up size table */
	uint32_t * times; /* hold times for each test */
	int i;

	LPRINTF("Starting shared mem latency demo\n");
	for ( i = 0; i < NUM_SIZES; i++){
		
		times = generate_latency_measurement(sizes[i], channel);
		if (!times)
			LPERROR("shmem_latency_demo_task: generate_latency_measurement failed\n");
		/* then report */
		LPRINTF("APU to RPU latency for size %i is: %u ns\n", 
			sizes[i],
			times[0] / NUM_TIMES * NS_PER_SEC / XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ
			);
		LPRINTF("RPU to APU latency for size %i is: %u ns\n", 
			sizes[i],
			times[1] / NUM_TIMES * NS_PER_SEC / XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ
			);
	}

	/* notify remote that shared mem. latency test is done */
	metal_io_write32(channel->shm_io, TEST_STATUS_OFFSET, DONE_TESTING); 
	LPRINTF("Finished shared mem latency demo\n");
	return 0;


}
