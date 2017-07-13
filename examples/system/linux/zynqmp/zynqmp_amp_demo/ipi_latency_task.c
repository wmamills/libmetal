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
  * ipi_latency_task.c
  * This demo demonstrates the latency between the APU and RPU.
  * This demo does so via the following steps:
  * 1. Initialize two of the three timers found on TTC0 and shared memory which
  * 	will be used for polling.
  * 2. For 1000 times, the APU will set the first timer in TTC0 to 0 and 
  *		trigger an interrupt which the RPU will then detect and stop said 
  * 	timer to record the APU to RPU latency. Then the RPU will reset the
  *     second timer in TTC0 to 0, start this timer and trigger an interrupt.
  *	    Finally, the APU will detect this interrupt and measure this latency.
  * 3. After the loop, report the average latency each way in nanoseconds.
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
#include <errno.h>
#include <metal/alloc.h>
#include <time.h>

#include "tasks.h"

#define KEEP_GOING 						1
#define TIMER_KEEP_GOING_OFFSET 		64
#define XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ 	100000000

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
 * @param[in]     channel- hold timer device
 * @return - 0 on success, error code if failure.
 */
int ipi_latency_demo_task(struct channel_s *channel)
{
	int APU_to_RPU_total, RPU_to_APU_total, i;

	APU_to_RPU_total = 0;
	RPU_to_APU_total = 0;	
	LPRINTF("Starting IPI latency task\n");
	metal_io_write32(channel->shm_io, TIMER_KEEP_GOING_OFFSET, KEEP_GOING);
	for ( i = 1; i <= NUM_TIMES; i++){
		
		/* tell RPU to respond */
		// atomic_store(&(channel->notified), 0);
		metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
		/* set count back to 0, then start couting again */
		reset_timer(channel, APU_TO_RPU_TIMER_OFFSET);
		start_timer(channel, APU_TO_RPU_TIMER_OFFSET);
		/* irq handler stops timer for rpu->apu irq */
		wait_for_notified(channel);
		
		RPU_to_APU_total += read_timer(channel, RPU_TO_APU_TIMER_OFFSET);
		APU_to_RPU_total += read_timer(channel, APU_TO_RPU_TIMER_OFFSET);	
	}
	
	/* tell RPU that we are done */
	metal_io_write32(channel->shm_io, TIMER_KEEP_GOING_OFFSET, 0); 
	metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);

	/* report avg latencies */
	LPRINTF("With %i iterations, the latency results are as follows: \n", NUM_TIMES);
	LPRINTF("APU to RPU average latency: %u ns \n", 
		APU_to_RPU_total / NUM_TIMES * NS_PER_SEC / XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ );
	LPRINTF("RPU to APU average latency: %u ns \n", 
		RPU_to_APU_total / NUM_TIMES* NS_PER_SEC / XPAR_XTTCPS_0_TTC_CLK_FREQ_HZ );
	LPRINTF("Finished IPI latency task\n");
	return 0;
}
