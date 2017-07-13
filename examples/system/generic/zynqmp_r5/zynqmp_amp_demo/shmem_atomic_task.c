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
/**
 *    atomic_shmem_task.c - Shared memory atomic operation demo
 *          This task will:
 *          * Wait for the APU to write to shared memory.
 *          * Once notification is received via polling, start atomic add by
 *			  1 for 1000 times to first 32 bits of memory in the
 *			  shared memory location at 3ed00000 which is pointed to by shm_io.
 *          * Write to shared mem to notify the remote once it finishes calculation.
 */
#include <metal/shmem.h>
#include <metal/atomic.h>
#include <metal/device.h>
#include <metal/io.h>
#include <sys/time.h>
#include <stdio.h>
#include "tasks.h"
#include "sys_init.h"

#define READY			 	1
#define ATOMIC_INT_OFFSET 128

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
int atomic_shmem_taskd(struct channel_s* channel)
{
	atomic_int *shm_int;
	int i;

	shm_int = (atomic_int *)metal_io_virt(channel->shm_io, ATOMIC_INT_OFFSET);

	LPRINTF("Starting shared memory with atomics test\n");
	while (1) {
		wait_for_notified(channel);

		for (i = 0; i < 1000; i++)
			atomic_fetch_add(shm_int, 1);
		/* memory barrier */
		atomic_thread_fence(memory_order_acq_rel);

		/* Send the message */
		LPRINTF("SENDING message...\n");
		metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
		break;
	}

	LPRINTF("Shared memory with atomics test finished\n");
	return 0;
}
