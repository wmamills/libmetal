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
  * 2. APU will write message to shared memory section. RPU will be polling
  *	   for message, and then read and echo it back. Finally, the APU will 
  *	   detect this and verify message.
  * 3. If there is an error during initialization, cleanup the libmetal 
  *		resources. Otherwise, cleanup after the report.
  */

#include "tasks.h"
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <metal/atomic.h>
#include <metal/cpu.h>
#include <metal/io.h>
#include <metal/alloc.h>
#include <sys/time.h>
#include <sys/types.h>
#include <metal/device.h>
#include <metal/irq.h>
#include <errno.h>
#include <string.h>

#define TEST_STATUS_OFFSET 	32
#define MESSAGE_OFFSET 		64
#define KEEP_GOING			10
#define REQUEST 			11
#define MESSAGE 			"hello"
#define MESSAGE_OFFSET_APU_TO_RPU 	0
#define MESSAGE_OFFSET_RPU_TO_APU 	256

/**
 * @brief shmem_task() - Show use of shared memory with Libmetal.
 *				Write message to RPU. RPU will then read and echo
 *				back. Confirm if echoed message is identical.
 *				If messages differ, report error.
 *
 * @param[in]     channel- hold shared mem. device
 * @return - return 0 on success, otherwise return error number indicating
 *  		 type of error
 */
int shmem_task(struct channel_s *channel)
{
	int ret;
	char * received;
	
	LPRINTF("Setting up shared memory task.\n");
	/* ensure test isnt prematurely started */ 
	metal_io_write32(channel->shm_io, TEST_STATUS_OFFSET, KEEP_GOING);
	received = metal_allocate_memory(strlen(MESSAGE) * sizeof(char));
	memset(received, 0, strlen(MESSAGE));
	ret = 0;

	LPRINTF("Starting shared memory demo.\n");
	/* send message */
	ret = metal_io_block_write(channel->shm_io, MESSAGE_OFFSET_APU_TO_RPU, 
		(void*) MESSAGE, strlen(MESSAGE));
	if (ret < 0){
		LPERROR("Unable to metal_io_block_write()\n");
		goto out;
	}
	/* tell RPU to respond */
	metal_io_write32(channel->shm_io, TEST_STATUS_OFFSET, REQUEST);
	/* wait for remote */
	while (metal_io_read32(channel->shm_io, TEST_STATUS_OFFSET) == REQUEST);
	/* read message */
	metal_io_block_read(channel->shm_io, MESSAGE_OFFSET_RPU_TO_APU, 
		(void*) received, strlen(MESSAGE));
	if (ret < 0){
		LPERROR("Unable to metal_io_block_read()\n");
		goto out;
	}
	LPRINTF("Sent the following to remote: %s, got back: %s\n", MESSAGE, received);
	ret = strcmp(MESSAGE, received);
	
	out:
	metal_free_memory(received);
	LPRINTF("Shared memory demo: %s.\n", ret ? "Failed": "Passed" );
	return ret;
}
