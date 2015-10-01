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
 * @file	linux/init.c
 * @brief	Linux libmetal initialization.
 */

#include <metal/sys.h>
#include <metal/utilities.h>

struct metal_state _metal;

/** Sort function for page size array. */
static int metal_pagesize_compare(const void *_a, const void *_b)
{
	const struct metal_page_size *a = _a, *b = _b;
	long diff = a->page_size - b->page_size;
	return metal_sign(diff);
}

static int metal_add_hugepage_size(unsigned long size, const char *path)
{
	int index = _metal.num_page_sizes;

	if (index >= MAX_PAGE_SIZES || !path) {
		metal_log(LOG_WARNING, "skipped page size %ld (%s)\n",
			  size, path ? "too many sizes" : "not mounted");
		return -EINVAL;
	}

	_metal.page_sizes[index].page_size = size;
	_metal.page_sizes[index].page_shift = metal_log2(size);
	strncpy(_metal.page_sizes[index].path, path, PATH_MAX);
	_metal.num_page_sizes ++;

	metal_log(LOG_DEBUG, "added page size %ld @%s\n", size, path);

	return 0;
}

int metal_sys_init(const struct metal_init_params *params)
{
	const int max_sizes = MAX_PAGE_SIZES - 1;
	static char sysfs_path[SYSFS_PATH_MAX];
	long sizes[max_sizes];
	int count, i, result;
	const char *tmp_path;
	unsigned int seed;
	FILE* urandom;

	/* Determine system page size. */
	result = getpagesize();
	if (result <= 0) {
		metal_log(LOG_ERROR, "failed to get page size\n");
		return -ENOSYS;
	}
	_metal.page_size  = result;
	_metal.page_shift = metal_log2(result);

	/* Determine sysfs mount point. */
	result = sysfs_get_mnt_path(sysfs_path, sizeof(sysfs_path));
	if (result) {
		metal_log(LOG_ERROR, "failed to get sysfs path (%s)\n",
			  strerror(-result));
		return result;
	}
	_metal.sysfs_path = sysfs_path;

	/* Find the temporary directory location. */
	tmp_path = getenv("TMPDIR");
	if (!tmp_path)
		tmp_path = "/tmp";
	_metal.tmp_path = tmp_path;

	/* Initialize the pseudo-random number generator. */
	urandom = fopen("/dev/urandom", "r");
	if (!urandom) {
		metal_log(LOG_ERROR, "failed to open /dev/urandom (%s)\n",
			  strerror(errno));
		return -errno;
	}
	fread(&seed, sizeof(int), 1, urandom);
	fclose(urandom);
	srand(seed);

	/* Setup page size list. */
	metal_add_hugepage_size(_metal.page_size, _metal.tmp_path);
	count = gethugepagesizes(sizes, max_sizes);
	for (i = 0; i < count; i++) {
		const char *loc = hugetlbfs_find_path_for_size(sizes[i]);
		metal_add_hugepage_size(sizes[i], loc);
	}

	qsort(_metal.page_sizes, _metal.num_page_sizes,
	      sizeof(struct metal_page_size), metal_pagesize_compare);

	result = metal_linux_bus_init();
	if (result < 0) {
		metal_log(LOG_DEBUG, "Failed bus initialization - %s\n",
			  strerror(-result));
		return result;
	}

	result = open("/proc/self/pagemap", O_RDONLY | O_CLOEXEC);
	if (result < 0) {
		metal_log(LOG_DEBUG, "Failed pagemap open - %s\n",
			  strerror(errno));
	}
	_metal.pagemap_fd = result;

	metal_unused(params);

	return 0;
}

void metal_sys_finish(void)
{
	metal_linux_bus_finish();
	close(_metal.pagemap_fd);
}
