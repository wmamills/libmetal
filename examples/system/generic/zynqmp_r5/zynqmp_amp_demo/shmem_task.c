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
  * shmem_task.c
  * This demo demonstrates the use of shared mem. between the APU and RPU.
  * This demo does so via the following steps:
  * 1. Initialize  shared memory.
  * 2. Wait for message from APU. Once received, read and echo it back.
  * 3. If there is an error during initialization, cleanup the libmetal 
  *		resources. Otherwise, cleanup after the report.
  */
#include <unistd.h>

#include <metal/sys.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <metal/atomic.h>
#include <metal/cpu.h>
#include <metal/io.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "tasks.h"
#include "sys_init.h"

#define TEST_STATUS_OFFSET 	32
#define MESSAGE_OFFSET 		64
#define KEEP_GOING 			10
#define REQUEST 			11
#define MESSAGE_LENGTH		5

/**
 * @brief shmem_task() - Show use of shared memory with Libmetal.
 *        Wait for message from APU. Once received, read and echo it back.
 *
 * @param[in]     channel - channel holding and shared mem device
 * @return - return 0 on success, otherwise return error number indicating
 *  		 type of error
 */
int shmem_taskd(struct channel_s *channel)
{
	char * received;
	int ret;
	
	/* ensure test isnt prematurely started */ 
	metal_io_write32(channel->shm_io, TEST_STATUS_OFFSET, KEEP_GOING);

	received = metal_allocate_memory((MESSAGE_LENGTH+1) * sizeof(char));
	memset(received, 0, MESSAGE_LENGTH+1);
	LPRINTF("Wait for shared memory demo to start.\n");
	while (metal_io_read32(channel->shm_io, TEST_STATUS_OFFSET) == KEEP_GOING);
	/* read message */
	LPRINTF("Reading message from remote.\n");
	ret = metal_io_block_read(channel->shm_io, MESSAGE_OFFSET_APU_TO_RPU, 
		(void*) received, MESSAGE_LENGTH);
	if (ret < 0){
		LPERROR("Unable to metal_io_block_read()\n");
		return ret;
	}
	else ret = 0;
	LPRINTF("message from remote: %s\n", received);

	LPRINTF("Echo message back\n");
	ret = metal_io_block_write(channel->shm_io, MESSAGE_OFFSET_RPU_TO_APU, 
		(void*) received, MESSAGE_LENGTH);
	if (ret < 0){
		LPERROR("Unable to metal_io_block_write()\n");
		return ret;
	}
	else ret = 0;

	metal_io_write32(channel->shm_io, TEST_STATUS_OFFSET, 0);
	metal_free_memory(received);
	LPRINTF("Shared memory test finished\r\n");
	return ret;
}
