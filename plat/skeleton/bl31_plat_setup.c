/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
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

#include <platform.h>

/*******************************************************************************
 * Declarations of linker defined symbols which will help us find the layout
 * of trusted SRAM
 ******************************************************************************/
extern unsigned long __RO_START__;
extern unsigned long __RO_END__;

extern unsigned long __COHERENT_RAM_START__;
extern unsigned long __COHERENT_RAM_END__;

/*
 * The next 2 constants identify the extents of the code & RO data region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __RO_START__ and __RO_END__ linker symbols refer to page-aligned addresses.
 */
#define BL31_RO_BASE (unsigned long)(&__RO_START__)
#define BL31_RO_LIMIT (unsigned long)(&__RO_END__)

/*
 * The next 2 constants identify the extents of the coherent memory region.
 * These addresses are used by the MMU setup code and therefore they must be
 * page-aligned.  It is the responsibility of the linker script to ensure that
 * __COHERENT_RAM_START__ and __COHERENT_RAM_END__ linker symbols
 * refer to page-aligned addresses.
 */
#define BL31_COHERENT_RAM_BASE (unsigned long)(&__COHERENT_RAM_START__)
#define BL31_COHERENT_RAM_LIMIT (unsigned long)(&__COHERENT_RAM_END__)

/*******************************************************************************
 * This data structure holds information copied by BL31 from BL2 to pass
 * control to the normal world software images.
 * TODO: Can this be moved out of device memory.
 ******************************************************************************/
static el_change_info ns_entry_info
__attribute__ ((aligned(PLATFORM_CACHE_LINE_SIZE),
		section("tzfw_coherent_mem")));

/* Data structure which holds the extents of the trusted SRAM for BL31 */
static meminfo bl31_tzram_layout
__attribute__ ((aligned(PLATFORM_CACHE_LINE_SIZE),
		section("tzfw_coherent_mem")));

meminfo *bl31_plat_sec_mem_layout(void)
{
	return &bl31_tzram_layout;
}

/*******************************************************************************
 * Return information about passing control to the non-trusted software images
 * to common code.TODO: In the initial architecture, the image after BL31 will
 * always run in the non-secure state. In the final architecture there
 * will be a series of images. This function will need enhancement then
 ******************************************************************************/
el_change_info *bl31_get_next_image_info(void)
{
	return &ns_entry_info;
}

/*******************************************************************************
 * Perform any BL31 specific platform actions. Here we copy parameters passed
 * by the calling EL (S-EL1 in BL2 & S-EL3 in BL1) before they are lost
 * (potentially). This is done before the MMU is initialized so that the memory
 * layout can be used while creating page tables.
 ******************************************************************************/
void bl31_early_platform_setup(meminfo *mem_layout,
			       void *data)
{
	el_change_info *image_info = (el_change_info *) data;

	/* Setup the BL31 memory layout */
	bl31_tzram_layout.total_base = mem_layout->total_base;
	bl31_tzram_layout.total_size = mem_layout->total_size;
	bl31_tzram_layout.free_base = mem_layout->free_base;
	bl31_tzram_layout.free_size = mem_layout->free_size;
	bl31_tzram_layout.attr = mem_layout->attr;
	bl31_tzram_layout.next = 0;

	/* Save information about jumping into the normal world */
	ns_entry_info.entrypoint = image_info->entrypoint;
	ns_entry_info.spsr = image_info->spsr;
	ns_entry_info.args = image_info->args;
	ns_entry_info.security_state = image_info->security_state;
	ns_entry_info.next = image_info->next;

	/* Initialize the platform config for future decision making */
	platform_config_setup();
}

/*******************************************************************************
 * Initialize the gic, configure the CLCD and zero out variables needed by the
 * secondaries to boot up correctly.
 ******************************************************************************/
void bl31_platform_setup()
{
	/* Initialize the gic cpu and distributor interfaces */
	gic_setup();

	/* PLAT_TODO: Allow access to the System counter timer module */

	/* PLAT_TODO: Intialize the power controller */

	/* Topologies are best known to the platform. */
	plat_setup_topology();
}

/*******************************************************************************
 * Perform the very early platform specific architectural setup here. At the
 * moment this is only intializes the mmu in a quick and dirty way.
 ******************************************************************************/
void bl31_plat_arch_setup()
{
	configure_mmu(&bl31_tzram_layout,
		      BL31_RO_BASE,
		      BL31_RO_LIMIT,
		      BL31_COHERENT_RAM_BASE,
		      BL31_COHERENT_RAM_LIMIT);
}
