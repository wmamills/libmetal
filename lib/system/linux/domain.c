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
 * @file	linux/domain.c
 * @brief	Linux libmetal domain handling.
 */

#include <string.h>

#include <metal/compiler.h>
#include <metal/domain.h>
#include <metal/mutex.h>
#include <metal/sys.h>
#include <metal/utilities.h>

static int metal_domain_remap_data(struct metal_domain *domain, int delta);

/**
 * @brief	Find the index for a domain shared resource.
 * @param[in]	domain		Domain data structure.
 * @param[in]	resource	Shared resource.
 * @return	resource index.
 */
static inline unsigned int
metal_resource_to_index(struct metal_domain *domain,
			struct metal_resource *resource)
{
	unsigned int index = resource - &domain->data->resources[0];
	assert(index < domain->data->num_resources);
	return index;
}

/**
 * @brief	Locate a domain shared resource from its index.
 * @param[in]	domain		Domain data structure.
 * @param[in]	index		Index of shared resource.
 * @return	Domain shared resource.
 */
static inline struct metal_resource *
metal_resource_from_index(struct metal_domain *domain, unsigned int index)
{
	assert(index < domain->data->num_resources);
	return &domain->data->resources[index];
}

#define for_each_resource(domain, resource)			\
	for ((resource) = ((domain)->data->resources);		\
	     (resource) < ((domain)->data->resources +		\
			   (domain)->data->num_resources);	\
	     (resource) ++)

/**
 * @brief	Try to lock a domain.
 * @param[in]	domain		Domain data structure.
 * @return	Non-zero if locked, 0 otherwise.
 */
static inline int metal_domain_try_lock(struct metal_domain *domain)
{
	assert(domain && domain->data);
	if (metal_mutex_try_acquire(&domain->data->mutex)) {
		assert(metal_domain_remap_data(domain, 0) == 0);
		return 1;
	}
	return 0;
}

/**
 * @brief	Lock a domain, block if required.
 * @param[in]	domain		Domain data structure.
 */
static inline void metal_domain_lock(struct metal_domain *domain)
{
	assert(domain && domain->data);
	metal_mutex_acquire(&domain->data->mutex);
	assert(metal_domain_remap_data(domain, 0) == 0);
}

/**
 * @brief	Unlock a previously locked domain.
 * @param[in]	domain		Domain data structure.
 */
static inline void metal_domain_unlock(struct metal_domain *domain)
{
	assert(domain && domain->data);
	metal_mutex_release(&domain->data->mutex);
}

/**
 * @brief	Open libmetal shared data file.
 * @param[in]	name	Shared data file path for the domain.
 * @param[in]	domain	Domain data structure.
 * @return	0 on success, or -errno on failure.
 */
static int metal_domain_open_data(struct metal_domain *domain, const char *name)
{
	int result;

	if (name) {
		strncpy(domain->data_path, name, PATH_MAX);
		result = metal_open(domain->data_path);
		if (result < 0) {
			metal_log(LOG_ERROR, "Failed to open %s (%s)\n",
				  domain->data_path, strerror(errno));
			return result;
		}
	} else {
		metal_mktemp_template(domain->data_path, "data");
		result = metal_mktemp_unlinked(domain->data_path);
		if (result < 0)
			return result;
	}
	domain->data_fd = result;

	return 0;
}

/**
 * @brief	Close libmetal shared data file.
 * @param[in]	domain	Domain data structure.
 */
static void metal_domain_close_data(struct metal_domain *domain)
{
	close(domain->data_fd);
}

/**
 * @brief	Open libmetal fifo.
 * @param[in]	name	Shared fifo path for the domain.
 * @param[in]	domain	Domain fifo structure.
 * @return	0 on success, or -errno on failure.
 */
static int metal_domain_open_fifo(struct metal_domain *domain)
{
	int result;

	metal_mktemp_template(domain->fifo_path, "fifo");
	result = metal_mktemp(domain->fifo_path, 1);
	if (result < 0)
		return result;
	domain->fifo_fd = result;

	return 0;
}

/**
 * @brief	Close libmetal domain fifo.
 * @param[in]	domain	Domain fifo structure.
 */
static void metal_domain_close_fifo(struct metal_domain *domain)
{
	close(domain->fifo_fd);
	unlink(domain->fifo_path);
}

/**
 * @brief	Calculate size of shared domain data.
 * @param[in]	resources	Number of shared data resources.
 * @return	Page size aligned data structure size.
 */
static inline size_t metal_domain_data_size(int resources)
{
	size_t size = ((sizeof(struct metal_domain_data)) +
		       (sizeof(struct metal_resource) * resources));
	return metal_align_up(size, _metal.page_size);
}

/**
 * @brief	Map (or remap) libmetal domain data.
 * @param[in]	domain	Domain data structure.
 * @param[in]	delta	Number of shared data resources to add.
 * @return	0 on success, or -errno on failure.
 */
static int metal_domain_remap_data(struct metal_domain *domain, int delta)
{
	size_t size = metal_domain_data_size(0);
	struct metal_resource *resource;
	int error = 0, index;
	void *data;

	if (domain->data->num_resources + delta > METAL_DOMAIN_MAX_ELEMENTS)
		return -ENOSPC;

	size = metal_domain_data_size(domain->data->num_resources + delta);
	if (size != domain->data_size) {
		error = metal_map(domain->data_fd, 0, size, 1, &data);
		if (error)
			return error;
		metal_unmap(domain->data, domain->data_size);
		domain->data = data;
		domain->data_size = size;
		domain->data->num_resources += delta;
	} else if (delta) {
		index = domain->data->num_resources;
		domain->data->num_resources += delta;
		resource = metal_resource_from_index(domain, index);
		memset(resource, 0, sizeof(*resource) * delta);
	}

	return error;
}

/**
 * @brief	Unmap libmetal domain data.
 * @param[in]	domain	Domain data structure.
 */
static void metal_domain_unmap_data(struct metal_domain *domain)
{
	if (!domain->data)
		return;
	metal_unmap(domain->data, domain->data_size);
	domain->data = NULL;
}

/**
 * @brief	Allocate an resource in the shared data table.
 * @param[in]	domain	Domain data structure.
 * @param[out]	result	Allocated resource.
 * @return	allocated index on success, or -errno on failure.
 */
static int metal_resource_alloc(struct metal_domain *domain,
				struct metal_resource **result)
{
	struct metal_resource *resource;
	int error = 0, index;

	for (;;) {
		for_each_resource(domain, resource) {
			index = metal_resource_to_index(domain, resource);
			if (resource->type == METAL_ELEMENT_FREE) {
				if (result)
					*result = resource;
				return index;
			}
		}

		error = metal_domain_remap_data(domain, 1);
		if (error)
			return error;
	}
}

/**
 * @brief	Free a resource in the shared data table.
 * @param[in]	domain		Domain data structure.
 * @param[in]	resource	Allocated resource.
 */
static void metal_resource_free(struct metal_domain *domain,
				struct metal_resource *resource)
{
	unsigned int index = metal_resource_to_index(domain, resource);
	assert(resource == metal_resource_from_index(domain, index));
	memset(resource, 0, sizeof(*resource));
}

/**
 * @brief	Scan libmetal domain data and free dead peers.
 * @param[in]	domain		Domain data structure.
 * @return	number of free resources on success, or -errno on failure.
 */
static int metal_domain_scan(struct metal_domain *domain)
{
	int flags = O_WRONLY | O_CLOEXEC | O_NONBLOCK;
	struct metal_resource *resource;
	unsigned int index;
	int fd;

	for_each_resource(domain, resource) {
		if (resource->type != METAL_ELEMENT_PEER)
			continue;
		fd = open(resource->path, flags);
		if (fd >= 0) {
			close(fd);
			continue;
		}

		metal_log(LOG_DEBUG, "removing dead peer @%s\n", resource->path);
		unlink(resource->path);

		/* Drop references on shared resources. */
		metal_bitmap_for_each_set_bit(resource->info.peer.refs, index,
					      METAL_DOMAIN_MAX_ELEMENTS) {
			metal_resource_from_index(domain, index)->refs--;
		}

		metal_resource_free(domain, resource);
	}

	for_each_resource(domain, resource) {
		if (resource->type == METAL_ELEMENT_PEER ||
		    resource->type == METAL_ELEMENT_FREE ||
		    resource->refs)
			continue;
		unlink(resource->path);
		metal_resource_free(domain, resource);
	}

	return 0;
}

/**
 * @brief	Register self as a peer in a shared libmetal domain.
 * @param[in]	domain		Domain data structure.
 * @return	0 on success, or -errno on failure.
 */
static int metal_domain_register(struct metal_domain *domain)
{
	struct metal_resource *resource;
	int result;

	result = metal_resource_alloc(domain, &resource);
	if (result < 0)
		return result;

	resource->type = METAL_ELEMENT_PEER;
	strncpy(resource->path, domain->fifo_path, PATH_MAX);
	domain->self = result;

	return 0;
}

/**
 * @brief	Unregister self as a peer in a shared libmetal domain.
 * @param[in]	domain		Domain data structure.
 * @return	0 on success, or -errno on failure.
 */
static void metal_domain_unregister(struct metal_domain *domain)
{
	struct metal_resource *resource;
	resource = metal_resource_from_index(domain, domain->self);
	metal_resource_free(domain, resource);
}

/**
 * @brief	Initialize a libmetal domain.
 * @param[out]	domain	Domain data structure.
 * @param[in]	name	Shared data file path for the domain.
 * @return	0 on success, or -errno on failure.
 */
int metal_domain_init(struct metal_domain *domain, const char *name)
{
	size_t size = metal_domain_data_size(0);
	void *data;
	int error;

	memset(domain, 0, sizeof(*domain));
	error = metal_domain_open_fifo(domain);
	if (error)
		return error;

	error = metal_domain_open_data(domain, name);
	if (error) {
		metal_domain_close_fifo(domain);
		return error;
	}

	error = metal_map(domain->data_fd, 0, size, 1, &data);
	if (error) {
		metal_domain_close_data(domain);
		metal_domain_close_fifo(domain);
		return error;
	}

	domain->data = data;
	domain->data_size = size;
	metal_domain_lock(domain);

	error = metal_domain_scan(domain);
	error = error ? error : metal_domain_register(domain);

	metal_domain_unlock(domain);

	if (error) {
		metal_domain_unmap_data(domain);
		metal_domain_close_data(domain);
		metal_domain_close_fifo(domain);
	}

	return error;
}

/**
 * @brief	Shutdown a libmetal domain.
 * @param[in]	domain	Domain data structure.
 */
void metal_domain_finish(struct metal_domain *domain)
{
	metal_domain_lock(domain);
	metal_domain_unregister(domain);
	metal_domain_scan(domain);
	metal_domain_unlock(domain);
	metal_domain_unmap_data(domain);
	metal_domain_close_data(domain);
	metal_domain_close_fifo(domain);
}

int metal_resource_open(struct metal_domain *domain,
			struct metal_resource *template,
			metal_resource_constructor_t ctor,
			void *ctor_arg)
{
	struct metal_resource *resource, *self;
	int error = 0, create = 1;
	unsigned long *refs;
	unsigned int index;

	metal_domain_lock(domain);

	/* Find the resource if it already exists. */
	for_each_resource(domain, resource) {
		if (resource->type != template->type)
			continue;
		if (strcmp(resource->name, template->name) != 0)
			continue;
		create = 0;
		break;
	}

	/* Create a resource from the template if one doesn't already exist. */
	if (create) {
		error = metal_resource_alloc(domain, &resource);
		if (error < 0) {
			metal_domain_unlock(domain);
			return error;
		}
		resource->type = template->type;
		resource->info = template->info;
		strncpy(resource->name, template->name, PATH_MAX);
		strncpy(resource->path, template->path, PATH_MAX);
	}

	/* Locate the reference tracking bits. */
	self = metal_resource_from_index(domain, domain->self);
	refs = self->info.peer.refs;

	/* Make sure that we open a resource only once. */
	index = metal_resource_to_index(domain, resource);
	error = metal_bitmap_is_bit_set(refs, index) ? -EBUSY : 0;

	/* Construct the resource. */
	error = error ? error : (*ctor)(domain, resource, create,
					index, ctor_arg);
	if (!error) {
		/* Bump ref counts (we're still under the lock). */
		resource->refs++;
		metal_bitmap_set_bit(refs, index);
	} else if (create) {
		metal_resource_free(domain, resource);
	}

	metal_domain_unlock(domain);
	return error;
}

void metal_resource_close(struct metal_domain *domain, unsigned int index)
{
	struct metal_resource *resource, *self;
	unsigned long *refs;

	metal_domain_lock(domain);

	self = metal_resource_from_index(domain, domain->self);
	refs = self->info.peer.refs;
	assert(metal_bitmap_is_bit_set(refs, index));
	resource = metal_resource_from_index(domain, index);

	metal_bitmap_clear_bit(refs, index);
	resource->refs--;
	metal_domain_scan(domain);

	metal_domain_unlock(domain);
}
