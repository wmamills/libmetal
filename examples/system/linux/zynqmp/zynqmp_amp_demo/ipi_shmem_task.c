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
 *   ipi_shmem_task.c - shared memory with IPI demo
 *          This task will:
 *          * Get the timestamp and put it into the ping shared memory
 *          * Update the shared memory descriptor for the new available
 *            ping buffer.
 *          * Trigger IPI to notify the remote.
 *          * Repeat the above steps until all packages are sent.
 *          * Monitor IPI interrupt, verify every received package.
 *          * After all the packages are received, send out shutdown
 *            message to the remote.
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <metal/sys.h>
#include <metal/atomic.h>
#include <metal/cpu.h>
#include <metal/io.h>
#include <metal/alloc.h>
#include <sys/time.h>
#include <sys/types.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <errno.h>

#include "tasks.h"

#define PKGS_TOTAL 1024

#define BUF_SIZE_MAX 512
#define SHUTDOWN "shutdown"


struct shm_mg_s {
	uint32_t avails;
	uint32_t used;
};

typedef uint64_t shm_addr_t;

struct msg_hdr_s {
	uint32_t index;
	uint32_t len;
};

/**
 * @breif get_timestamp() - Get the timestamp
 *        IT gets the timestamp and return nanoseconds.
 *
 * @return nano seconds.
 */
static unsigned long long get_timestamp (void)
{
	unsigned long long t = 0;
	struct timespec tp;
	int r;

	r = clock_gettime(CLOCK_MONOTONIC, &tp);
	if (r == -1) {
		LPERROR("Bad clock_gettime!\n");
		return t;
	} else {
		t = tp.tv_sec * (NS_PER_S);
		t += tp.tv_nsec;
	}
	return t;
}

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
 * @param[in] channel - channel information
 * @return - return 0 on success, otherwise return error number indicating
 *  		 type of error.
 */
int ipi_shmem_task(struct channel_s *channel)
{
	int i, ret;
	void *d0, *d1, *lbuf, *tmpptr;
	lbuf = metal_allocate_memory(BUF_SIZE_MAX);
	struct shm_mg_s *shm0_mg, *shm1_mg;
	shm_addr_t *shm0_addr_array, *shm1_addr_array;
	struct msg_hdr_s *msg_hdr, *msg_hdr_echo;
	
	metal_phys_addr_t d1_pa;
	unsigned long long tstart, tend;
	long long tdiff;
	long long tdiff_avg_s = 0, tdiff_avg_ns = 0;

	shm0_mg = (struct shm_mg_s *)metal_io_virt(channel->shm_io, SHM0_OFFSET);
	shm1_mg = (struct shm_mg_s *)metal_io_virt(channel->shm_io, SHM1_OFFSET);
	shm0_addr_array = (void *)shm0_mg + sizeof(struct shm_mg_s);
	shm1_addr_array = (void *)shm1_mg + sizeof(struct shm_mg_s);
	d0 = metal_io_virt(channel->shm_io, channel->d0_start_offset);
	

	if (!lbuf) {
		LPERROR("Failed to allocate local buffer for msg.\n");
		return -ENOSPC;
	}

	metal_io_block_set(channel->shm_io, 0, 0, SHM_SIZE);

	/* copy message to shared buffer */

	LPRINTF("Start echo flood testing....\n");
	LPRINTF("It sends msgs to the remote.\n");
	
	/* Clear shared memory descriptors of both directions */
	shm0_mg->avails = 0;
	shm0_mg->used = 0;
	shm1_mg->avails = 0;
	shm1_mg->used = 0;
	for (i = 0; i < PKGS_TOTAL; i++) {
		/* Construct a message to send */
		tmpptr = lbuf;
		msg_hdr = tmpptr;
		msg_hdr->index = i;
		msg_hdr->len = sizeof(tstart);
		tmpptr += sizeof(struct msg_hdr_s);
		tstart = get_timestamp();
		*(unsigned long long *)tmpptr = tstart;

		/* copy message to shared buffer */
		metal_io_block_write(channel->shm_io,
			metal_io_virt_to_offset(channel->shm_io, d0),
			msg_hdr,
			sizeof(struct msg_hdr_s) + msg_hdr->len);

		/* Update the shared memory management information
		 * Tell the other end where the d0 buffer is.
		 */
		shm0_addr_array[i] = (shm_addr_t)metal_io_virt_to_phys(
				channel->shm_io, d0);
		d0 += sizeof(struct msg_hdr_s) + msg_hdr->len;
		shm0_mg->avails++;

		/* memory barrier */
		atomic_thread_fence(memory_order_acq_rel);
		/* Send the message */	
		metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);
	}
	LPRINTF("And then it waits for msgs to echo back and verify.\n");
	i = 0;
	d0 = metal_io_virt(channel->shm_io, channel->d0_start_offset);
	while (shm1_mg->used != PKGS_TOTAL) {
		wait_for_notified(channel);
		atomic_thread_fence(memory_order_acq_rel);
		while ((shm1_mg->used != PKGS_TOTAL) &&
			(shm1_mg->used != shm1_mg->avails)) {
			/* Received pong from the other side */

			/* Get the d1 buffer location from the shared memory
			 * management.
			 */
			d1_pa = (metal_phys_addr_t)shm1_addr_array[shm1_mg->used];
			d1 = metal_io_phys_to_virt(channel->shm_io, d1_pa);
			if (!d1) {
				LPERROR("failed to get rx address: 0x%lx.\n",
					d1_pa);
				ret = -EINVAL;
				goto out;
			}
			msg_hdr_echo = (struct msg_hdr_s *)d1;

			/* Verify the message */
			if (msg_hdr_echo->index != (uint32_t)i) {
				LPERROR("wrong msg: expected: %d, actual: %d\n",
					i, msg_hdr_echo->index);
				ret = -EINVAL;
				goto out;
			}
			d1 += sizeof(struct msg_hdr_s);
			d0 += sizeof(struct msg_hdr_s);
			if (*(unsigned long long *)d0 !=
				*(unsigned long long *)d1) {
				LPERROR("wrong message, [%d], %llu:%llu\n",
					i, *(unsigned long long *)d0,
					*(unsigned long long *)d1);
				ret = -EINVAL;
				goto out;
			}
			d0 += msg_hdr_echo->len;
			shm1_mg->used++;
			i++;
		}
	}
	tend = get_timestamp();
	tdiff = tend - tstart;

	/* Send shutdown message */
	tmpptr = lbuf;
	msg_hdr = tmpptr;
	msg_hdr->index = i;
	msg_hdr->len = strlen(SHUTDOWN);
	tmpptr += sizeof(struct msg_hdr_s);
	sprintf(tmpptr, SHUTDOWN);
	/* copy message to shared buffer */
	metal_io_block_write(channel->shm_io,
		metal_io_virt_to_offset(channel->shm_io, d0),
		msg_hdr,
		sizeof(struct msg_hdr_s) + msg_hdr->len);

	shm0_addr_array[i] = (uint64_t)metal_io_virt_to_phys(
				channel->shm_io, d0);
	shm0_mg->avails++;
	atomic_thread_fence(memory_order_acq_rel);
	LPRINTF("Sending shutdown message...\n");
	metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET, channel->ipi_mask);

	tdiff /= i;
	tdiff_avg_s = tdiff / NS_PER_S;
	tdiff_avg_ns = tdiff % NS_PER_S;
	LPRINTF("Total packages: %d, time_avg = %lds, %ldns\n",
		i, (long int)tdiff_avg_s, (long int)tdiff_avg_ns);

	ret = 0;
out:
	metal_free_memory(lbuf);
	return ret;
}
