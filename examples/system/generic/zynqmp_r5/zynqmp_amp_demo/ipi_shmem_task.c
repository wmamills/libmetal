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
 *          * Update the shared memory descriptor for the new available
 *            ping buffer.
 *          * Trigger IPI to notify the remote.
 *          * Repeat the above steps until all packages are sent.
 *          * Monitor IPI interrupt, verify every received package.
 *          * After all the packages are received, send out shutdown
 *            message to the remote.
 *
 */
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include "tasks.h"

#define BUF_SIZE_MAX 512
#define SHUTDOWN "shutdown"

typedef struct shm_mg_s {
	uint32_t avails;
	uint32_t used;
} shm_mg_s;

typedef uint64_t shm_addr_t;

typedef struct msg_hdr_s {
	uint32_t index;
	uint32_t len;
} msg_hdr_s;

/**
 * @brief   ipi_task_echod() - shared memory IPI demo
 *          This task will:
 *          * Wait for IPI interrupt from the remote
 *          * Once it received the interrupt, copy the content from
 *            the ping buffer to the pong buffer.
 *          * Update the shared memory descriptor for the new available
 *            pong buffer.
 *          * Trigger IPI to notifty the remote.
 * @param[in] channel - channel holding IPI and shared mem devices
 * @param[in] arg - channel information
 */
int ipi_shmem_taskd(struct channel_s *channel)
{
	int ret;
	struct shm_mg_s *shm0_mg, *shm1_mg;
	shm_addr_t *shm0_addr_array, *shm1_addr_array;
	struct msg_hdr_s *msg_hdr;
	void *d0, *d1, *lbuf;
	metal_phys_addr_t d0_pa;
	int len;

	shm0_mg = (struct shm_mg_s *)metal_io_virt(channel->shm_io, SHM0_OFFSET);
	shm1_mg = (struct shm_mg_s *)metal_io_virt(channel->shm_io, SHM1_OFFSET);
	shm0_addr_array = (void *)shm0_mg + sizeof(struct shm_mg_s);
	shm1_addr_array = (void *)shm1_mg + sizeof(struct shm_mg_s);
	d1 = metal_io_virt(channel->shm_io, channel->d1_start_offset);
	lbuf = metal_allocate_memory(BUF_SIZE_MAX);
	if (!lbuf) {
		LPERROR("Failed to allocate local buffer for msg.\n");
		ret = -EINVAL;
		goto out;
	}
	ret = 0;

	LPRINTF("Wait for echo test to start.\n");
	while (1) {
		wait_for_notified(channel);
		atomic_thread_fence(memory_order_acq_rel);
		while(shm0_mg->used != shm0_mg->avails) {
			d0_pa = (metal_phys_addr_t)
				shm0_addr_array[shm0_mg->used];
			d0 = metal_io_phys_to_virt(channel->shm_io, d0_pa);
			if (!d0) {
				LPERROR("failed to get rx addr:0x%lx.\n",
					d0_pa);
				ret = -EINVAL;
				goto out;
			}
			/* Copy msg header from shared buf to local mem */
			len = metal_io_block_read(channel->shm_io,
				metal_io_virt_to_offset(channel->shm_io, d0),
				lbuf, sizeof(struct msg_hdr_s));
			if (len < (int)sizeof(struct msg_hdr_s)) {
				LPERROR("Failed to get msg header.\n");
				goto out;
			}
			msg_hdr = lbuf;
			if (msg_hdr->len <= 0) {
				LPERROR("wrong msg length: %d.\n",
					(int)msg_hdr->len);
				ret = -EINVAL;
				goto out;
			} else {
				/* Copy msg data from shared buf to local mem */
				d0 += sizeof(struct msg_hdr_s);
				len = metal_io_block_read(channel->shm_io,
					metal_io_virt_to_offset(channel->shm_io, d0),
					lbuf + sizeof(struct msg_hdr_s),
					msg_hdr->len);
				#if LDEBUG
					LPRINTF("\nreceived: %d, %d, %s\n",
						(int)msg_hdr->index, 
						(int)msg_hdr->len, 
						(lbuf + sizeof(struct msg_hdr_s))
					);
					LPRINTF("address: %p\n", d0_pa);

				#endif
				/* Check if the it is the shutdown message */
				if (!strncmp(SHUTDOWN, 
				(lbuf + sizeof(struct msg_hdr_s)),
				strlen(SHUTDOWN))) {
					LPRINTF("Received shutdown message\n");
					goto out;
				}
			}
			/* Copy the message back to the other end */
			metal_io_block_write(channel->shm_io,
				metal_io_virt_to_offset(channel->shm_io, d1),
				lbuf,
				sizeof(struct msg_hdr_s) + msg_hdr->len);

			/* Update the d1 address */
			shm1_addr_array[shm1_mg->avails] = 
					(uint64_t)metal_io_virt_to_phys(
						channel->shm_io, d1);
			d1 += (sizeof(struct msg_hdr_s) + msg_hdr->len);
			shm0_mg->used++;
			shm1_mg->avails++;
			/* memory barrier */
			atomic_thread_fence(memory_order_acq_rel);
			/* Send the message */
			metal_io_write32(channel->ipi_io, IPI_TRIG_OFFSET,channel->ipi_mask);
		}

	}

	
out:
	LPRINTF("\n\nIPI with shared memory demo finished with exit code: %i.\n", ret);

	metal_free_memory(lbuf);
	return ret;
}
