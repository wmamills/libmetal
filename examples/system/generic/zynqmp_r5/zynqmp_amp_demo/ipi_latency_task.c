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
  * This demo acts as the remote side to receive to the IPI latency demo 
  * run in Linux userspace. 
  * This demo demo does so via the following:
  * Try to initialize shared memory, interrupt and TTC0 libmetal resources. 
  * If intialization fails, cleanup libmetal resources. Otherwise:
  * - Loop until APU signals that test is over.
  * - In the loop, wait for interrupt. In the interrupt handler, stop the 
  *		timer that measures APU to RPU latency. Then reset and start the
  *		timer that measures RPU to APU latency and send an interrupt.
  * - After the loop, cleanup libmetal resources.
  */
#include <sys/time.h>
#include <sys/types.h>
#include "tasks.h"

#define KEEP_GOING 				1
#define TIMER_KEEP_GOING_OFFSET 64

/**
 * @brief reset_timer() - function to reset count in TTC timer
 *        Turn on the RST bit in the Count Control Reg. Setting 
 *		  this bit high resets the counter value and restarts counting; 
 *		  the RST bit is automatically cleared on restart.
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     channel - channel pointing to timer
 */
static void reset_timer(uint32_t offset, struct channel_s* channel)
{
	uint32_t val;
	val = metal_io_read32(channel->timer_io, 
		offset + CNT_CNTRL_TIMER_OFFSET);
	val |= XTTCPS_CNT_CNTRL_RST_MASK;
	metal_io_write32(channel->timer_io, offset + CNT_CNTRL_TIMER_OFFSET, val);
}

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
		offset + CNT_CNTRL_TIMER_OFFSET);
	val &= ~XTTCPS_CNT_CNTRL_DIS_MASK;
	metal_io_write32(channel->timer_io, offset + CNT_CNTRL_TIMER_OFFSET, val);
}


/**
 * @brief ipi_latency_demo_task() - Show performance of  IPI with Libmetal.
 *		  Loop until APU tells RPU to stop via shared memory.
 *		  In loop, wait for interrupt (interrupt handler stops APU to
 *		  RPU timer). Then reset count on RPU to APU timer to 0, start 
 *		  counting and send interrupt to notify APU.
 *
 * @param[in] 	channel - channel holding timer device
 * @return - 0 on success, error code if failure.
 */
int ipi_latency_demo_taskd(struct channel_s *channel)
{
	LPRINTF("Starting IPI latency demo\r\n");
	metal_io_write32(channel->shm_io, TIMER_KEEP_GOING_OFFSET, KEEP_GOING);
	while( metal_io_read32(channel->shm_io, TIMER_KEEP_GOING_OFFSET) == KEEP_GOING){
		/* ipi handler already stoppped apu->rpu timer, so just reset and start 
		rpu->apu timer */
		metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
		wait_for_notified(channel);
		reset_timer(RPU_TO_APU_TIMER_OFFSET, channel);
		start_timer(RPU_TO_APU_TIMER_OFFSET, channel);
		/* send interrupt to apu */
		
	}
	LPRINTF("Finished IPI latency demo\r\n");
	return 0;
}
