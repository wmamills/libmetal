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
 * @file	linux/shmem.c
 * @brief	Linux libmetal shared memory handling.
 */

#include <metal/domain.h>
#include <metal/shmem.h>
#include <metal/sys.h>

struct metal_shmem_ctor_args
{
	struct metal_shmem	*shmem;
	unsigned long		pages;
	unsigned long		page_size;
	unsigned long		page_shift;
};

static void metal_shmem_io_close(struct metal_io_region *io)
{
	struct metal_shmem *shmem;

	shmem = metal_container_of(io, struct metal_shmem, io);
	metal_unmap(io->virt, io->size);
	free((void *)io->physmap);
	metal_resource_close(shmem->domain, shmem->index);
}

static const struct metal_io_ops metal_shmem_io_ops = {
	NULL, NULL, metal_shmem_io_close
};

static int metal_shmem_ctor(struct metal_domain *domain,
			    struct metal_resource *resource,
			    int create, unsigned int index,
			    void *_args)
{
	struct metal_shmem_ctor_args *args = _args;
	struct metal_shmem *shmem = args->shmem;
	metal_phys_addr_t *phys;
	size_t phys_size, size;
	unsigned long page;
	void *mem = NULL;
	int result, fd;
	uint8_t *virt;

	if (create) {
		resource->info.shmem.pages	= args->pages;
		resource->info.shmem.page_size	= args->page_size;
		resource->info.shmem.page_shift	= args->page_shift;
	} else {
		args->pages	 = resource->info.shmem.pages;
		args->page_size	 = resource->info.shmem.page_size;
		args->page_shift = resource->info.shmem.page_shift;
	}

	phys_size = sizeof(*phys) * args->pages;
	phys = malloc(phys_size);
	if (!phys) {
		metal_log(LOG_ERROR, "shmem phys alloc failed\n");
		return -ENOMEM;
	}

	result = (create
		  ? metal_mktemp(resource->path, 0)
		  : metal_open(resource->path));
	if (result < 0) {
		metal_log(LOG_ERROR, "shmem mktemp %s failed - %s\n",
			  resource->path, strerror(-result));
		free(phys);
		return result;
	}
	fd = result;

	shmem->domain = domain;
	shmem->index = index;
	shmem->name = resource->name;
	size = args->pages << args->page_shift;

	result = metal_map(fd, 0, size, 1, &mem);
	result = result ? result : metal_mlock(mem, size);
	close(fd);

	if (result) {
		metal_log(LOG_ERROR, "failed mmap/mlock on %s - %s\n",
			  resource->path, strerror(-result));
		free(phys);
		if (mem)
			metal_unmap(mem, size);
		if (create)
			unlink(resource->path);
		return result;
	}

	for (virt = mem, page = 0; page < args->pages; page++) {
		size_t offset = page * args->page_size;
		result = metal_virt2phys(virt + offset, &phys[page]);
		if (result < 0)
			phys[page] = METAL_BAD_OFFSET;
	}

	metal_io_init(&shmem->io, mem, phys, size, args->page_shift,
		      &metal_shmem_io_ops);

	return 0;
}

int metal_shmem_open(struct metal_domain *domain,
		     const char *name, size_t *size,
		     struct metal_shmem **result)
{
	struct metal_shmem_ctor_args args = { };
	struct metal_resource template = { };
	struct metal_page_size *ps;
	struct metal_shmem *shmem;
	int error;

	error = metal_shmem_open_generic(domain, name, size, result);
	if (!error)
		return error;

	shmem = malloc(sizeof(*shmem));
	if (!shmem)
		return -ENOMEM;

	memset(shmem, 0, sizeof(*shmem));
	ps = metal_best_fit_page_size(*size);
	template.type = METAL_ELEMENT_SHMEM;
	strncpy(template.name, name, PATH_MAX);
	snprintf(template.path, PATH_MAX, "%s/metal-data-XXXXXX", ps->path);

	args.shmem	= shmem;
	args.page_size	= ps->page_size;
	args.page_shift	= ps->page_shift;
	args.pages	= metal_div_round_up(*size, args.page_size);

	error = metal_resource_open(domain, &template, metal_shmem_ctor, &args);
	if (error) {
		metal_log(LOG_ERROR, "shmem open of %s failed - %s\n",
			  name, strerror(-error));
		free(shmem);
		return error;
	}

	*size = metal_io_region_size(&shmem->io);
	*result = shmem;

	return 0;
}
