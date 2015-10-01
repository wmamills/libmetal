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
 * @file	shmem.h
 * @brief	Shared memory primitives for libmetal.
 */

#ifndef __METAL_SHMEM__H__
#define __METAL_SHMEM__H__

#include <metal/domain.h>
#include <metal/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Shared memory data structure. */
struct metal_shmem {
	const char		*name;
	struct metal_io_region	io;
	struct metal_domain	*domain;
	unsigned int		index;
	struct metal_list	node;
};

/**
 * @brief	Statically register a generic shared memory region.
 *
 * Shared memory regions may be statically registered at application
 * initialization, or may be dynamically opened via a shared domain.  This
 * interface is used for static registration of regions.  Subsequent calls to
 * metal_shmem_open() look up in this list of pre-registered regions.
 *
 * @param[in]	shmem	Generic shmem structure.
 * @return 0 on success, or -errno on failure.
 */
extern int metal_register_generic_shmem(struct metal_shmem *shmem);

/**
 * @brief	Open a libmetal shared memory segment.
 *
 * Open a memory segment shared with other peers within a libmetal domain.  The
 * segment must have been previously created.
 *
 * @param[in]		domain	Domain in which to open memory segment.
 * @param[in]		name	Name (unique within domain) of segment to open.
 * @param[in, out]	size	Size of segment.
 * @param[out]		shmem	Shared memory segment handle, if successful.
 * @return	0 on success, or -errno on failure.
 *
 * @see metal_shmem_create
 */
extern int metal_shmem_open(struct metal_domain *domain,
			    const char *name, size_t *size,
			    struct metal_shmem **shmem);

/**
 * @brief	Close a libmetal shared memory segment.
 *
 * Close a memory segment shared with other peers within a domain.  The calling
 * process must no longer have any part of the shared memory segment mapped.
 *
 * @param[in]	shmem	Shared memory segment handle.
 */
extern void metal_shmem_close(struct metal_shmem *shmem);

/**
 * @brief	Get an I/O region accessor for a shared memory region.
 *
 * @param[in]	shmem	Shared memory segment handle.
 * @return I/O accessor handle, or NULL on failure.
 */
static inline struct metal_io_region *
metal_shmem_to_io_region(struct metal_shmem *shmem)
{
	return &shmem->io;
}

#ifdef __cplusplus
}
#endif

#endif /* __METAL_SHMEM__H__ */
