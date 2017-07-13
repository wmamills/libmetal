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
/** atomic_shmem_task.c Shared memory atomic operation demo
 *          This task will:
 *          *  Write to shared memory to notify the remote to start atomic add on the
 *            shared memory descriptor memory for 1000 times.
 *          * Start atomic add by 1 for 1000 times to first 32 bits of memory in the
 *			  shared memory location at 3ed00000 which is pointed to by shm_io.
 *          * Wait for the remote to write to shared memory
 *          * Once the polling kick is received from the remote, check if
 *            the value stored in the shared memory for the atomic add is 2000.
 *          * Print if the atomic add test has passed or not.
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
#include <time.h>

#include "tasks.h"

#define READY 				1
#define ATOMIC_INT_OFFSET 128

/**
 * @brief   atomic_shmem_task() - Shared memory atomic operation demo
 *          This task will:
 *          * Write to shared memory to notify the remote to start atomic add on the
 *            shared memory descriptor memory for 1000 times.
 *          * Start atomic add by 1 for 1000 times to first 32 bits of memory in the
 *			  shared memory location at 3ed00000 which is pointed to by shm_io.
 *          * Wait for the remote to write to shared memory
 *          * Once it received the polling kick from the remote, it will check if
 *            the value stored in the shared memory for the atomic add is 2000.
 *          * It will print if the atomic add test has passed or not.
 * @param[in]     channel- hold timer device
 * @return - If setup failed, return the corresponding error number. Otherwise
 *          return 0 on success.
 */
int atomic_shmem_task(struct channel_s *channel)
{
	int i, ret;
	atomic_int *shm_int;

	LPRINTF("Starting atomic shared memory task.\n");

	shm_int = (atomic_int *)metal_io_virt(channel->shm_io, ATOMIC_INT_OFFSET);
	atomic_store(shm_int, 0);
	LPRINTF("Stored atomic int.\n");
	for (i = 0; i < 1000; i++) {
		atomic_fetch_add(shm_int, 1);
	}
	LPRINTF("counted to 1000.\n");
	/* Kick the remote to start counting */
	LPRINTF("Kicked off to remote.\n");
	metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
	
	/* wait for response from rpu */
	wait_for_notified(channel);

	if (atomic_load(shm_int) == 2000) {
		LPRINTF("shm atomic testing PASS!\n");
		ret = 0;
	} else {
		LPRINTF("shm atomic testing FAILED. actual: %u\n", atomic_load(shm_int));
		ret = -1;
	}

	return ret;
}

