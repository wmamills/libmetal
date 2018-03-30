/*
 * Copyright (c) 2017, Linaro Limited. and Contributors. All rights reserved.
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
 * 3. Neither the name of Linaro nor the names of its contributors may be used
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
 * @file	zephyr/mutex.h
 * @brief	Zephyr mutex primitives for libmetal.
 */

#ifndef __METAL_MUTEX__H__
#error "Include metal/mutex.h instead of metal/zephyr/mutex.h"
#endif

#ifndef __METAL_ZEPHYR_MUTEX__H__
#define __METAL_ZEPHYR_MUTEX__H__

#include <metal/atomic.h>
#include <kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct k_sem metal_mutex_t;

#define METAL_MUTEX_DEFINE(m) K_SEM_DEFINE(m, 1, 1)

static inline void __metal_mutex_init(metal_mutex_t *m)
{
	k_sem_init(m, 1, 1);
}

static inline void __metal_mutex_deinit(metal_mutex_t *m)
{
	(void)m;
}

static inline int __metal_mutex_try_acquire(metal_mutex_t *m)
{
        int key = irq_lock(), ret = 1;

        if (m->count) {
                m->count = 0;
                ret = 0;
        }

        irq_unlock(key);

        return ret;
}

static inline int __metal_mutex_is_acquired(metal_mutex_t *m)
{
        int key = irq_lock(), ret;

	ret = m->count;

        irq_unlock(key);

	return ret;
}

static inline void __metal_mutex_acquire(metal_mutex_t *m)
{
	k_sem_take(m, K_FOREVER);
}

static inline void __metal_mutex_release(metal_mutex_t *m)
{
	k_sem_give(m);
}

#ifdef __cplusplus
}
#endif

#endif /* __METAL_ZEPHYR_MUTEX__H__ */
