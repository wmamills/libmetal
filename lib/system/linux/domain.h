/*
 * Copyright (c) 2015, Xilinx Inc. and Contributors. All rights reserved.
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

/*
 * @file	linux/domain.h
 * @brief	Linux domain primitives for libmetal.
 */

#ifndef __METAL_DOMAIN__H__
#error "Include metal/domain.h instead of metal/linux/domain.h"
#endif

#ifndef __METAL_LINUX_DOMAIN__H__
#define __METAL_LINUX_DOMAIN__H__

#include <assert.h>
#include <stddef.h>
#include <linux/limits.h>

#include <metal/compiler.h>
#include <metal/mutex.h>
#include <metal/utilities.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Metal domain data structure for Linux platforms. */
struct metal_domain {

	/** File data_path for shared data file. */
	char					data_path[PATH_MAX];

	/** File descriptor for shared data file. */
	int					data_fd;

	/** Memory mapped data shared across processes. */
	struct metal_domain_data		*data;

	/** Size of shared data. */
	size_t					data_size;

	/** FIFO for peers to confirm that we're alive. */
	char					fifo_path[PATH_MAX];

	/** File descriptor for FIFO. */
	int					fifo_fd;

	/** Self index in shared data table. */
	unsigned int				self;
};

#ifdef METAL_INTERNAL

#define METAL_DOMAIN_MAX_ELEMENTS	128
#define METAL_DOMAIN_REF_MAP_SIZE	\
	metal_bitmap_longs(METAL_DOMAIN_MAX_ELEMENTS)

enum metal_resource_type {
	METAL_ELEMENT_FREE = 0,
	METAL_ELEMENT_PEER,
	METAL_ELEMENT_SHMEM,
};

union metal_resource_info {
	struct {
		unsigned long	refs[METAL_DOMAIN_REF_MAP_SIZE];
	} peer;
	struct {
		unsigned long	pages;
		unsigned long	page_size;
		unsigned long	page_shift;
	} shmem;
};

/** Single resource within shared data list. */
struct metal_resource {
	enum metal_resource_type	type;
	unsigned long			refs;
	char				path[PATH_MAX];
	char				name[PATH_MAX];
	union metal_resource_info	info;
} metal_align(64);

/** Shared data across a libmetal domain. */
struct metal_domain_data {

	/** Mutex to serialize domain accesses. */
	struct metal_mutex		mutex;

	/** Number of shared resources. */
	unsigned int			num_resources;

	/** Shared data resources. */
	struct metal_resource		resources[0];
};

typedef int (*metal_resource_constructor_t)(struct metal_domain *domain,
					    struct metal_resource *resource,
					    int create, unsigned int index,
					    void *arg);

int metal_resource_open(struct metal_domain *domain,
			struct metal_resource *template,
			metal_resource_constructor_t ctor,
			void *ctor_arg);

void metal_resource_close(struct metal_domain *domain, unsigned int index);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __METAL_LINUX_DOMAIN__H__ */
