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

 /***************************************************************************
  * libmetal_amp_demo.c
  *
  * This application shows how to use IPI to trigger interrupt and how to
  * setup shared memory with libmetal API for communication between processors.
  *
  * This app does the following:
  * 1.  Initialize the platform hardware such as UART, GIC.
  * 2.  Connect the IPI interrupt.
  * 3.  Register IPI device, shared memory descriptor device and shared memory
  *     device with libmetal in the intialization.
  * 4.  In the main application it does the following,
  *     * open the registered libmetal devices: IPI device, shared memory
  *       descriptor device and shared memory device.
  *     * Map the shared memory descriptor as non-cached memory.
  *     * Map the shared memory as non-cached memory. If you do not map the
  *       shared memory as non-cached memory, make sure you flush the cache,
  *       before you notify the remote.
  * 7.  Register the IPI interrupt handler with libmetal.
  * 8.  Run the atomic demo task ipi_task_shm_atomicd():
  *     * Wait for the IPI interrupt from the remote.
  *     * Once it receives the interrupt, it does atomic add by 1 to the
  *       first 32bit of the shared memory descriptor location by 1000 times.
  *     * It will then notify the remote after the calucation.
  *     * As the remote side also does 1000 times add after it has notified
  *       this end. The remote side will check if the result is 2000, if not,
  *       it will error.
  * 9.  Run the shared memory echo demo task ipi_task_echod()
  *     * Wait for the IPI interrupt from the other end.
  *     * If an IPI interrupt is received, copy the message to the current
  *       available RPU to APU buffer, increase the available buffer indicator,
  *       and trigger IPI to notify the remote.
  *     * If "shutdown" message is received, cleanup the libmetal source.
  */
#include <metal/utilities.h>
#include "tasks.h"
#include <unistd.h>

struct channel_s channel; /* channel holding devices, and information needed for demos */

/**
 *
 * @brief disable_timer() - function to stop TTC timer
 *        Turn on the disable bit in the Count Control Reg, which 
 *      results in the timer to stop counting. 
 *
 * @param[in]     offset - offset to one of the 3 timers in the TTC
 * @param[in]     ch - channel pointing to timer
 */
static void disable_timer(uint32_t offset, struct channel_s*ch )
{
	uint32_t val;

	val = metal_io_read32(ch->timer_io, offset + XTTCPS_CNT_CNTRL_OFFSET);
	metal_io_write32(ch->timer_io, offset + XTTCPS_CNT_CNTRL_OFFSET, 
		val | XTTCPS_CNT_CNTRL_DIS_MASK);
}

/**
 * @brief ipi_irq_isr() - IPI interrupt handler
 *        It will clear the notified flag to mark it's got an IPI interrupt.
 *        This handler will stop a timer as well for latency testing
 * @param[in]     vect_id - IPI interrupt vector ID
 * @param[in/out] priv    - communication channel data for this application.
 *
 * @return - If the IPI interrupt is triggered by its remote, it returns
 *          METAL_IRQ_HANDLED. It returns METAL_IRQ_NOT_HANDLED, if it is
 *          not the interupt it expected.
 */
static int ipi_irq_handler (int vect_id, void *priv)
{
	(void) vect_id;
	struct channel_s *ch = (struct channel_s *)priv;
	uint64_t val = 1;
	if (!ch){
		return METAL_IRQ_NOT_HANDLED;
	}
	val = metal_io_read32(ch->ipi_io, IPI_ISR_OFFSET);
	if (val & ch->ipi_mask) {  
		disable_timer(APU_TO_RPU_TIMER_OFFSET, ch);
		metal_io_write32(ch->ipi_io, IPI_ISR_OFFSET, ch->ipi_mask);
		atomic_flag_clear(&(channel.notified));
		return METAL_IRQ_HANDLED;
	}
	return METAL_IRQ_NOT_HANDLED;
}

/**
 * @brief ipi_irq_isr() - IPI interrupt handler
 *        It will clear the notified flag to mark it's got an IPI interrupt.
 *      This handler will not stop a timer
 *
 * @param[in]     vect_id - IPI interrupt vector ID
 * @param[in/out] priv    - communication channel data for this application.
 *
 * @return - If the IPI interrupt is triggered by its remote, it returns
 *          METAL_IRQ_HANDLED. It returns METAL_IRQ_NOT_HANDLED, if it is
 *          not the interupt it expected.
 */
static int ipi_irq_isr (int vect_id, void *priv)
{
	(void)vect_id;
	struct channel_s *ch = (struct channel_s *)priv;

	uint64_t val = 1;
	if (!ch)
		return METAL_IRQ_NOT_HANDLED;
	val = metal_io_read32(ch->ipi_io, IPI_ISR_OFFSET);
	if (val & ch->ipi_mask) {
		metal_io_write32(ch->ipi_io, IPI_ISR_OFFSET, ch->ipi_mask);
		atomic_flag_clear(&ch->notified);
		return METAL_IRQ_HANDLED;
	}
	return METAL_IRQ_NOT_HANDLED;
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
		/* Get interrupt ID from IPI metal device  */
		irq = (intptr_t)channel.ipi_dev->irq_info;
		metal_irq_register(irq, 0, channel.ipi_dev, &channel);
	}

	if (channel.ipi_dev)
		metal_device_close(channel.ipi_dev);

	if (channel.shm_dev)
		metal_device_close(channel.shm_dev);

sys_cleanup();
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
		ret = metal_irq_register(irq, 0, channel.ipi_dev, &channel);
		LPRINTF("successfully deregistered IPI interrupt handler.\n");
		if (ret){
			LPERROR("Unable to deregister new irq handler.\n");
			goto out;
		}    
	}
	
	LPRINTF("Try to register IPI handler.\n");
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


/**
 * @brief    ipi_init - open and map IPI device for demo
 * @return   0 - succeeded, non-zero for failures.
 */
int ipi_init()
{
	int ret;
	struct metal_device *device;
	struct metal_io_region *io;

	/* Open IPI device */
	ret = metal_device_open(BUS_NAME, IPI_DEV_NAME, &device);
	if (ret) {
		LPERROR("Failed to open device ff310000.ipi.\n");
		return -ENODEV;
	}

	/* Map IPI device IO region */
	io = metal_device_io_region(device, 0);
	if (!io) {
		LPERROR("Failed to map io region for %s.\n",
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

/**
 * @brief    timer_init - open and map timer device for demo
 * @return   0 - succeeded, non-zero for failures.
 */
int timer_init()
{
	int ret;
	struct metal_device *device;
	struct metal_io_region *io;

	/* Open TTC0 device */
	ret = metal_device_open(BUS_NAME, TTC0_DEV_NAME, &device);
	if (ret) {
		LPERROR("Failed to open device %s.\n", TTC0_DEV_NAME);
		return ret;
	}
	/* Map TTC0 device IO region */
	io = metal_device_io_region(device, 0);
	if (!io) {
		LPERROR("Failed to map io region for %s.\n",
			device->name);
		metal_device_close(device);
		return -ENODEV;
	}

	channel.timer_device = device;
	channel.timer_io = io;
	return 0;
}

/**
 * @brief    setup - open and map shared memory devices for demo
 * @return   0 - succeeded, non-zero for failures.
 */
int setup(void)
{
	struct metal_device *device;
	struct metal_io_region *io;
	int ret = 0;

	if (sys_init()) {
		LPERROR("Failed to initialize system\n");
		ret = -1;
		goto out;
	}
	memset(&channel, 0, sizeof(channel));
	atomic_store(&channel.notified, 1);

	/* Open shared memory device */
	ret = metal_device_open(BUS_NAME, SHM_DEV_NAME, &device);
	if (ret) {
		LPERROR("Failed to open device %s.\n", SHM_DEV_NAME);
		goto out;
	}

	/* Map shared memory device IO region */
	io = metal_device_io_region(device, 0);
	if (!io) {
		LPERROR("Failed to map io regio for %s.\n",
			device->name);
		metal_device_close(device);
		ret = -ENODEV;
		goto out;
	}

	/* Store the shared memory device and I/O region */
	channel.shm_dev = device;
	channel.shm_io = io;
	channel.d0_start_offset = D0_SHM_OFFSET;
	channel.d1_start_offset = D1_SHM_OFFSET;

	ret = ipi_init();
	if (ret) {
		goto out;
	}

	ret = timer_init();
	if (ret) {
		goto out;
	} 

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
	if (ret){
		ret = -1;
		LPERROR("Failed to run setup.\n");
		goto out;
	}
	ret = shmem_taskd(&channel);
	if (ret){
		LPERROR("Failed to run shared memory task.\n");
		goto out;
	}  
	ret = irq_handler_registration(ipi_irq_handler, 0);
	if (ret){
		LPERROR("Failed to register: ipi_irq_handler.\n");
		goto out;
	}
	ret = atomic_shmem_taskd(&channel);
	if (ret){
		LPERROR("Failed to run shared memory with atomics task.\n");
		goto out;
	}
	
	
	ret = ipi_shmem_taskd(&channel);
	if (ret){
		LPERROR("Failed to run IPI shared memory task.\n");
		goto out;
	}
	ret = ipi_latency_demo_taskd(&channel);
	if (ret){
		LPERROR("Failed to run IPI shared memory task.\n");
		goto out;
	}

	/* 
	shared memory latency and throughput tests need a
	timer to stop outside of irq handler, so deregister
	current handler and register new irq handler that doesn't stop 
	a timer. 
	*/
	ret = irq_handler_registration(ipi_irq_isr,1);
	if (ret){
		LPERROR("Failed to register handler for shared memory throughput task.\n");
		goto out;
	}
	ret = shmem_throughput_demo_taskd(&channel);
	if (ret) {
		LPERROR("Failed to run shared mem. throughput task.\n");
		goto out;
	} 
	LPRINTF("enabled IPI interrupt.\n");
	ret = shmem_latency_demo_taskd(&channel);
	if (ret) {
		LPERROR("Failed to run shared mem. latency task.\n");
		goto out;
	}

	out:
	cleanup();
	return ret;
}
