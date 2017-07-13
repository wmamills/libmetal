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
  * libmetal_amp_demo.c
  *
  * This application shows how to use IPI to trigger interrupt and how to
  * setup shared memory with libmetal API for communication between processors.
  *
  * This app does the following:
  * 1.  Run the shared memory echo demo task ipi_shmem_task()
  *     * Write message to the APU to RPU shared buffer.
  *     * Update the APU to RPU shared memory available index.
  *     * Trigger IPI to the remote.
  *     * Repeat the above 3 sub steps until it sends all the packages.
  *     * Wait for IPI to receive all the packages
  *     * If "shutdown" message is received, cleanup the libmetal source.
  * 2.  Run shared memory demo with shmem_task().
  *     * Open shared memory device.
  *     * For 1000 times, communicate between local and remote processes
  *       using shared memory and polling via shared memory.
  *     * Cleanup shared memory device.
  * 3.  Run the atomic demo task atomic_shmem_task():
  *     * Trigger the IPI to the remote, the remote will then start doing atomic
  *       add calculation.
  *     * Start atomic add by 1 for 1000 times to the first 32bit of the shared
  *       memory descriptor location.
  *     * Once it receives the IPI interrupt, it will check if the value stored
  *       in the shared memory descriptor location is 2000. If yes, the atomic
  *       across the shared memory passed, otherwise, it failed.
  * 4.  Demonstrate IPI latency with ipi_latency_demo_task()
  *     * Open IPI and timer devices.
  *     * For 1000 times, record APU to RPU IPI latency and RPU to APU
  *     latency. Then report average time for each direction.
  *     * Cleanup libmetal resources
  * 5.  Demonstrate shared memory latency with shmem_latency_demo_task()
  *     * Open shared memory and timer devices.
  *     * For 1000 times, record APU to RPU shared memory latency and RPU to APU
  *     latency for 8 bytes, 1/2K and 1K. Then report average time for each direction.
  *     * Cleanup libmetal resources
  * 6.  Demonstrate shared memory throughput with shmem_throughput_demo_task()
  *     * Open shared memory, IPI and timer devices.
  *     * For 1000 times, record APU block read and write times. Notify remote to run 
  *       test, then similarly record RPU block read and write times for 1/2K, 1K and 
  *       2K. Then report average throughput for each data size and operation.
  *     * Cleanup libmetal resources
  */

#include <stdio.h>
#include <metal/io.h>
#include <metal/device.h>

#include "tasks.h"

static struct channel_s channel; /* channel holding devices, and information needed for demos */

/**
 * @brief ipi_irq_isr() - IPI interrupt handler
 *        It will clear the notified flag to mark it's got an IPI interrupt.
 *
 * @param[in]     vect_id - IPI interrupt vector ID
 * @param[in/out] priv    - communication channel data for this application.
 *
 * @return - If the IPI interrupt is triggered by its remote, it returns
 *          METAL_IRQ_HANDLED. It returns METAL_IRQ_NOT_HANDLED, if it is
 *          not the interrupt it expected.
 */
static int ipi_irq_isr (int vect_id, void *priv)
{
	
	(void)vect_id;
	struct channel_s *ch = (struct channel_s *)priv;
	uint64_t val = 1;
	if (!ch)
		return METAL_IRQ_NOT_HANDLED;
	val = metal_io_read32(ch->ipi_io, IPI_ISR_OFFSET);
	if (val & IPI_MASK) {
		metal_io_write32(ch->ipi_io, IPI_ISR_OFFSET, IPI_MASK);
		atomic_flag_clear(&ch->notified);
		return METAL_IRQ_HANDLED;
	}
	return METAL_IRQ_NOT_HANDLED;
}

/**
 *
 * @brief stop_timer() - function to stop TTC timer
 *        Turn on the disable bit in the Count Control Reg, which 
 *      results in the timer to stop counting. 
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     ch - channel pointing to timer
 */
static void stop_timer(struct channel_s*ch, uint32_t offset)
{
	uint32_t val;

	val = metal_io_read32(ch->timer_io, offset + XTTCPS_CNT_CNTRL_OFFSET);
	metal_io_write32(ch->timer_io, offset + XTTCPS_CNT_CNTRL_OFFSET, 
		val | XTTCPS_CNT_CNTRL_DIS_MASK);
}

/**
 * @brief ipi_irq_handler() - IPI interrupt handler
 *        It will clear the notified flag to mark it's got an IPI interrupt.
 *      Additionally, the IPI handler thread will stop the RPU->APU timer to 
 *      further increase the accuracy of IPI latency measurement.
 *
 * @param[in]     vect_id - IPI interrupt vector ID
 * @param[in/out] priv    - communication channel data for this application.
 *
 * @return - If the IPI interrupt is triggered by its remote, it returns
 *          METAL_IRQ_HANDLED. It returns METAL_IRQ_NOT_HANDLED, if it is
 *          not the interrupt it expected.
 */
static int ipi_irq_handler (int vect_id, void *priv)
{
	(void)vect_id;
	struct channel_s *ch = (struct channel_s *)priv;
	uint64_t val = 1;

	if (!ch)
		return METAL_IRQ_NOT_HANDLED;
	val = metal_io_read32(ch->ipi_io, IPI_ISR_OFFSET);
	if (val & ch->ipi_mask) {
		/* stop RPU -> APU timer */
		stop_timer(ch, RPU_TO_APU_TIMER_OFFSET);
		metal_io_write32(ch->ipi_io, IPI_ISR_OFFSET, ch->ipi_mask);
		atomic_flag_clear(&ch->notified);
		return METAL_IRQ_HANDLED;
	}

	return METAL_IRQ_NOT_HANDLED;
}


/**
 * @brief setup_timer() - function to initialize TTC timer
 *        Sets a timer as follows:
 *      -Set Max Interval Value to MAX_INTERVAL_VAL
 *      -Disable Prescaler
 *      -Set Max Interval Value to MAX_INTERVAL_VAL
 *      -Disable Decrement mode
 *      -Enable Interval mode
 *
 * @param[in]     timer_base_offset - offset to one of the 3 timers in the TTC
 */
static void setup_timer(struct channel_s* ch, uint32_t timer_base_offset)
{
	uint32_t val;

	/* set max counter value */
	metal_io_write32(ch->timer_io, 
	timer_base_offset + XTTCPS_INTERVAL_VAL_OFFSET, MAX_INTERVAL_VAL); 

	/* set clock control register */
	metal_io_write32(ch->timer_io, 
	timer_base_offset + XTTCPS_CLK_CNTRL_OFFSET, 0);

	/* set clock operational mode */
	val = 0;
	/* enable interval mode */
	val |= XTTCPS_CNT_CNTRL_INT_MASK;
	metal_io_write32(ch->timer_io, 
	timer_base_offset + XTTCPS_CNT_CNTRL_OFFSET, val);
}


/**
 * @brief    cleanup - cleanup the application
 *           The cleanup funciton will disable the IPI interrupt
 *           close the metal devices and finish the libmetal environment.
 */
static void cleanup(void)
{
	int irq;
	/* Disable IPI interrupt */
	if (channel.ipi_io) {
		metal_io_write32(channel.ipi_io, IPI_IDR_OFFSET, channel.ipi_mask);
		irq = (intptr_t)channel.ipi_dev->irq_info;
		metal_irq_register(irq, 0, channel.ipi_dev, &channel);
	}
	metal_io_block_set(channel.shm_io, 0, 0, SHM_SIZE);
	if (channel.ipi_dev)
		metal_device_close(channel.ipi_dev);
	if (channel.shm_dev)
		metal_device_close(channel.shm_dev);
	if (channel.timer_device)
		metal_device_close(channel.timer_device);
	if (channel.timer_io)
		metal_io_finish(channel.timer_io);
	metal_finish();
}

/**
 * @brief irq_handler_registration() - handle all steps of irq handler registration
 *        It will disable interrupt, deregister other handlers, register the 
 *        handler passed in, and then enable the interrupt.
 *
 * @param[in]     irq_handler - handler to register
 * @param[in]     deregister - deregister other handlers
 *
 * @return   0 - succeeded, non-zero for failures.
 */
static int irq_handler_registration( metal_irq_handler irq_handler, int deregister)
{
	int irq, ret;
	uint32_t val;

	/* Disable IPI interrupt */
	metal_io_write32(channel.ipi_io, IPI_IDR_OFFSET, channel.ipi_mask);
	LPRINTF("Disabled IPI interrupt.\n");

	/* Get interrupt ID from IPI metal device */
	irq = (intptr_t)channel.ipi_dev->irq_info;

	LPRINTF("Try to deregister IPI interrupt handler.\n");
	if (deregister){
		ret = metal_irq_register(irq, 0, 0, 0);
		LPRINTF("successfully deregistered IPI interrupt handler.\n");
		if (ret){
			LPERROR("Unable to deregister old irq handler handler.\n");
			goto out;
		}	
	}
	

	LPRINTF("Try to register new IPI handler.\n");
	ret =  metal_irq_register(irq, irq_handler, channel.ipi_dev, &channel);
	if (ret){
		LPERROR("Unable to register new irq handler.\n");
		goto out;
	}
	LPRINTF("registered IPI handler.\n");
	/* Enable interrupt */
	metal_io_write32(channel.ipi_io, IPI_IER_OFFSET, channel.ipi_mask);
	val = metal_io_read32(channel.ipi_io, IPI_IMR_OFFSET);
	if (val & channel.ipi_mask) {
		LPERROR("Failed to enable IPI interrupt.\n");
		return -1;
	}
	LPRINTF("enabled IPI interrupt.\n");

	out:
	return ret;
}

static int ipi_init()
{
	struct metal_device *device;
	struct metal_io_region *io;
	int ret;

	/* Open IPI device */
	ret = metal_device_open(BUS_NAME, IPI_DEV_NAME, &device);
	if (ret) {
		LPERROR("Failed to open device ff340000.ipi.\n");
		return ret;
	}

	/* Map IPI device IO region */
	io = metal_device_io_region(device, 0);
	if (!io) {
		LPERROR("Failed to map io regio for %s.\n",
		device->name);
		metal_device_close(device);
		return -ENODEV;
	}

	/* Store the IPI device and I/O region */
	channel.ipi_dev = device;
	channel.ipi_io = io;
	channel.ipi_mask = IPI_MASK;
	
	return 0;
}

static int timer_init()
{
	struct metal_device *device;
	struct metal_io_region *io;
	int ret;

	/* Open timer device */
	ret = metal_device_open(BUS_NAME, TTC0_DEV_NAME, &device);
	if (ret) {
		if (ret == -ENOMEM)
		  LPERROR("-ENOMEM\n");
		if (ret == -ENODEV)
		  LPERROR("-ENODEV\n");

		LPERROR("Failed to open device %s.\n", TTC0_DEV_NAME);
		return ret;
	}

	/* Map timer device IO region */
	io = metal_device_io_region(device, 0);
	if (!io) {
		LPERROR("Failed to map io region for %s.\n",
			device->name);
		metal_device_close(device);
		return -ENODEV;
	}
	/* store timer device and I/O region */
	channel.timer_device = device;
	channel.timer_io = io;
	setup_timer(&channel, TTC0_2_OFFSET);
	setup_timer(&channel, TTC0_1_OFFSET);


	if (!channel.timer_io){
		LPERROR("unable to get timer device %s\n", TTC0_DEV_NAME);
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief    setup - open and map shared memory devices for demo
 * @return - return 0 on success, otherwise return error number indicating
 *       type of error.
 */
static int setup(void)
{
	struct metal_device *device;
	struct metal_io_region *io;
	int ret = 0;
	struct metal_init_params init_param = METAL_INIT_DEFAULTS;

	if (metal_init(&init_param)) {
		LPERROR("Failed to run metal initialization\n");
		return -1;
	}
	memset(&channel, 0, sizeof(channel));
	atomic_store(&channel.notified, 1);

	/* Open shared memory device */
	ret = metal_device_open(BUS_NAME, SHM_DEV_NAME, &device);
	if (ret) {
		LPERROR("Failed to open device %s.\n", SHM_DEV_NAME);
		goto out;
	}

	/* get shared memory device IO region */
	io = metal_device_io_region(device, 0);
	if (!io) {
		LPERROR("Failed to get io region for %s.\n",
		device->name);
		metal_device_close(device);
		ret = -ENODEV;
		goto out;
	}

	/* Store the shared memory device and I/O region */
	channel.shm_dev = device;
	channel.shm_io = io;
	channel.d0_start_offset = D0_SHM_OFFSET;

	ret = ipi_init();
	if (ret)
		goto out;

	ret = timer_init();
	if (ret)
		goto out;

	out:
	return ret;
}

/**
 * @brief    main function of the demo application.
 *           Here are the steps for the main function:
 *           * Setup libmetal resources
 *           * Run the IPI with shared memory task.
 *           * Run the shared memory task.
 *           * Run the atomic across shared memory task.
 *           * Run the ipi latency task.
 *           * Run the shared memory latency task.
 *           * Run the shared memory throughput task.
 *           * Cleanup libmetal resources
 *           Report if any of the above tasks failed.
 * @return   0 - succeeded, non-zero for failures.
 */
int main(void)
{
	int ret;

	ret = setup();
	if (ret)
		return ret;
	
	ret = shmem_task(&channel);
	if (ret) {
		LPRINTF( "ERROR: Failed to run shared memory task.\n");
		goto out;
	}
	ret = irq_handler_registration(ipi_irq_handler, 0);
	if (ret){
		LPERROR("Failed to register: ipi_irq_handler.\n");
		goto out;
	}
	ret = atomic_shmem_task(&channel);
	if (ret) {
		LPRINTF( "ERROR: Failed to run atomics with shared memory task.\n");
		goto out;
	}


	ret = ipi_shmem_task(&channel);
	if (ret) {
		LPRINTF( "ERROR: Failed to run IPI shared memory task.\n");
		goto out;
	}
	ret = ipi_latency_demo_task(&channel);
	if (ret) {
		LPERROR("Failed to run IPI latency task.\n");
		goto out;
	}
	/* 
	shared memory latency and throughput tests need a
	timer to stop outside of irq handler, so deregister
	current handler and register new irq handler that doesn't stop 
	a timer. 
	*/ 
	irq_handler_registration(ipi_irq_isr,1);
	if (ret){
		LPERROR("Failed to register handler for shared memory throughput task.\n");
		goto out;
	}
	ret = shmem_throughput_demo_task(&channel);
	if (ret) {
		LPERROR("Failed to run shared mem. throughput task.\n");
		goto out;
	}
	ret = shmem_latency_demo_task(&channel);
	if (ret) {
		LPERROR("Failed to run shared mem. latency task.\n");
		goto out;
	}

	out:
	cleanup();

	return ret;
}
