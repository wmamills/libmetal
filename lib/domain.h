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
 * @file	domain.h
 * @brief	Domain initialization routines for libmetal.
 */

#ifndef __METAL_DOMAIN__H__
#define __METAL_DOMAIN__H__

#include <metal/config.h>

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup domain Domain Interfaces
 *  @{ */


/** Opaque domain data structure. */
struct metal_domain;

#include <metal/system/@PROJECT_SYSTEM@/domain.h>

/**
 * @brief	Initialize a libmetal domain.
 *
 * Initialize a libmetal domain.  A domain consists of a set of interacting
 * processes that share system resources (memory, locks, etc.).
 *
 * @param[in]	name	Name of the domain.  On Linux systems, the name is
 *			really a path to a file mapped shared by all processes
 *			within the domain.  A NULL value for this argument
 *			signifies that the application wants to open a
 *			private temporary domain which is not shared with any
 *			other process.
 * @param[out]	domain	A domain handle initialized by this call.
 *
 * @return	0 on success, or -errno on failure.
 *
 * @see metal_domain_finish
 */
extern int metal_domain_init(struct metal_domain *domain, const char *name);

/**
 * @brief	Shutdown a libmetal domain.
 *
 * Shutdown a libmetal domain previously opened by metal_domain_init.
 *
 * @see metal_domain_init
 */
extern void metal_domain_finish(struct metal_domain *domain);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __METAL_DOMAIN__H__ */
