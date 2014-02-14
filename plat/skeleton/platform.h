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

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <arch.h>
#include <mmio.h>
#include <psci.h>
#include <bl_common.h>


/*******************************************************************************
 * Platform binary types for linking
 ******************************************************************************/
#define PLATFORM_LINKER_FORMAT          "elf64-littleaarch64"
#define PLATFORM_LINKER_ARCH            aarch64

/*******************************************************************************
 * Generic platform constants
 ******************************************************************************/
#define PLATFORM_STACK_SIZE		0x800

#define FIRMWARE_WELCOME_STR		"Booting boot loader stage 1\n\r"
#define BL2_IMAGE_NAME			"bl2.bin"
#define BL31_IMAGE_NAME			"bl31.bin"
#define NS_IMAGE_OFFSET			FLASH0_BASE

#define PLATFORM_CACHE_LINE_SIZE	64
#define PLATFORM_CLUSTER_COUNT		2ull
#define PLATFORM_CLUSTER0_CORE_COUNT	4
#define PLATFORM_CLUSTER1_CORE_COUNT	4
#define PLATFORM_CORE_COUNT		(PLATFORM_CLUSTER1_CORE_COUNT + \
						PLATFORM_CLUSTER0_CORE_COUNT)
#define PLATFORM_MAX_CPUS_PER_CLUSTER	4
#define PRIMARY_CPU			0x0

/* Constants for accessing platform configuration */
#define CONFIG_GICD_ADDR		0
#define CONFIG_GICC_ADDR		1
#define CONFIG_GICH_ADDR		2
#define CONFIG_GICV_ADDR		3
#define CONFIG_MAX_AFF0			4
#define CONFIG_MAX_AFF1			5
/* Indicate whether the CPUECTLR SMP bit should be enabled. */
#define CONFIG_CPU_SETUP		6
#define CONFIG_BASE_MMAP		7
/* Indicates whether CCI should be enabled on the platform. */
#define CONFIG_HAS_CCI			8
#define CONFIG_LIMIT			9

/*******************************************************************************
 * Platform memory map related constants
 ******************************************************************************/
#define TZROM_BASE		0x00000000
#define TZROM_SIZE		0x04000000

#define TZRAM_BASE		0x04000000
#define TZRAM_SIZE		0x40000

#define FLASH0_BASE		0x08000000
#define FLASH0_SIZE		TZROM_SIZE

#define FLASH1_BASE		0x0c000000
#define FLASH1_SIZE		0x04000000

#define PSRAM_BASE		0x14000000
#define PSRAM_SIZE		0x04000000

#define VRAM_BASE		0x18000000
#define VRAM_SIZE		0x02000000

/* Aggregate of all devices in the first GB */
#define DEVICE0_BASE		0x1a000000
#define DEVICE0_SIZE		0x12200000

#define DEVICE1_BASE		0x2f000000
#define DEVICE1_SIZE		0x200000

#define NSRAM_BASE		0x2e000000
#define NSRAM_SIZE		0x10000

#define TZDRAM_BASE		0x06000000
#define TZDRAM_SIZE		0x02000000
#define MBOX_OFF		0x1000
#define AFFMAP_OFF		0x1200

#define DRAM_BASE              0x80000000ull
#define DRAM_SIZE              0x80000000ull

#define PCIE_EXP_BASE		0x40000000
#define TZRNG_BASE		0x7fe60000
#define TZNVCTR_BASE		0x7fe70000
#define TZROOTKEY_BASE		0x7fe80000

/* PLAT_TODO: Memory mapped Generic timer interfaces */
/* the following definition is used by generic code, but
 * is platform specific... can be removed once the
 * referece is removed.
 */
#define SYS_CNTCTL_BASE		0x2a430000

/*******************************************************************************
 * Platform specific per affinity states. Distinction between off and suspend
 * is made to allow reporting of a suspended cpu as still being on e.g. in the
 * affinity_info psci call.
 ******************************************************************************/
#define PLATFORM_MAX_AFF0	4
#define PLATFORM_MAX_AFF1	2
#define PLAT_AFF_UNK		0xff

#define PLAT_AFF0_OFF		0x0
#define PLAT_AFF0_ONPENDING	0x1
#define PLAT_AFF0_SUSPEND	0x2
#define PLAT_AFF0_ON		0x3

#define PLAT_AFF1_OFF		0x0
#define PLAT_AFF1_ONPENDING	0x1
#define PLAT_AFF1_SUSPEND	0x2
#define PLAT_AFF1_ON		0x3

/*******************************************************************************
 * BL2 specific defines.
 ******************************************************************************/
#define BL2_BASE			0x0402D000

/*******************************************************************************
 * BL31 specific defines.
 ******************************************************************************/
#define BL31_BASE			0x0400E000

/*******************************************************************************
 * Platform specific page table and MMU setup constants
 ******************************************************************************/
#define EL3_ADDR_SPACE_SIZE		(1ull << 32)
#define EL3_NUM_PAGETABLES		2
#define EL3_TROM_PAGETABLE		0
#define EL3_TRAM_PAGETABLE		1

#define ADDR_SPACE_SIZE			(1ull << 32)

#define NUM_L2_PAGETABLES		2
#define GB1_L2_PAGETABLE		0
#define GB2_L2_PAGETABLE		1

#define NUM_L3_PAGETABLES		2
#define TZRAM_PAGETABLE			0
#define NSRAM_PAGETABLE			1

/*******************************************************************************
 * CCI-400 related constants
 ******************************************************************************/
#define CCI400_BASE			0x2c090000
#define CCI400_SL_IFACE_CLUSTER0	3
#define CCI400_SL_IFACE_CLUSTER1	4
#define CCI400_SL_IFACE_INDEX(mpidr)	(mpidr & MPIDR_CLUSTER_MASK ? \
					 CCI400_SL_IFACE_CLUSTER1 :   \
					 CCI400_SL_IFACE_CLUSTER0)

/*******************************************************************************
 * GIC-400 & interrupt handling related constants
 ******************************************************************************/
#define BASE_GICD_BASE			0x2f000000
#define BASE_GICR_BASE			0x2f100000
#define BASE_GICC_BASE			0x2c000000
#define BASE_GICH_BASE			0x2c010000
#define BASE_GICV_BASE			0x2c02f000

#define IRQ_TZ_WDOG			56
#define IRQ_SEC_PHY_TIMER		29
#define IRQ_SEC_SGI_0			8
#define IRQ_SEC_SGI_1			9
#define IRQ_SEC_SGI_2			10
#define IRQ_SEC_SGI_3			11
#define IRQ_SEC_SGI_4			12
#define IRQ_SEC_SGI_5			13
#define IRQ_SEC_SGI_6			14
#define IRQ_SEC_SGI_7			15
#define IRQ_SEC_SGI_8			16

/*******************************************************************************
 * Declarations and constants to access the mailboxes safely. Each mailbox is
 * aligned on the biggest cache line size in the platform. This is known only
 * to the platform as it might have a combination of integrated and external
 * caches. Such alignment ensures that two maiboxes do not sit on the same cache
 * line at any cache level. They could belong to different cpus/clusters &
 * get written while being protected by different locks causing corruption of
 * a valid mailbox address.
 ******************************************************************************/
#define CACHE_WRITEBACK_SHIFT   6
#define CACHE_WRITEBACK_GRANULE (1 << CACHE_WRITEBACK_SHIFT)

#ifndef __ASSEMBLY__

typedef volatile struct {
	unsigned long value
	__attribute__((__aligned__(CACHE_WRITEBACK_GRANULE)));
} mailbox;

/*******************************************************************************
 * Function and variable prototypes
 ******************************************************************************/
extern unsigned long *bl1_normal_ram_base;
extern unsigned long *bl1_normal_ram_len;
extern unsigned long *bl1_normal_ram_limit;
extern unsigned long *bl1_normal_ram_zi_base;
extern unsigned long *bl1_normal_ram_zi_len;

extern unsigned long *bl1_coherent_ram_base;
extern unsigned long *bl1_coherent_ram_len;
extern unsigned long *bl1_coherent_ram_limit;
extern unsigned long *bl1_coherent_ram_zi_base;
extern unsigned long *bl1_coherent_ram_zi_len;
extern unsigned long warm_boot_entrypoint;

extern void bl1_plat_arch_setup(void);
extern void bl2_plat_arch_setup(void);
extern void bl31_plat_arch_setup(void);
extern int platform_setup_pm(plat_pm_ops **);
extern unsigned int platform_get_core_pos(unsigned long mpidr);
extern void disable_mmu(void);
extern void enable_mmu(void);
extern void configure_mmu(meminfo *,
			  unsigned long,
			  unsigned long,
			  unsigned long,
			  unsigned long);
extern unsigned long platform_get_cfgvar(unsigned int);
extern int platform_config_setup(void);
extern void plat_report_exception(unsigned long);
extern unsigned long plat_get_ns_image_entrypoint(void);
extern unsigned long platform_get_stack(unsigned long mpidr);

extern void gic_cpuif_deactivate(unsigned int);
extern void gic_cpuif_setup(unsigned int);
extern void gic_pcpu_distif_setup(unsigned int);
extern void gic_setup(void);

extern int plat_setup_topology(void);
extern int plat_get_max_afflvl(void);
extern unsigned int plat_get_aff_count(unsigned int, unsigned long);
extern unsigned int plat_get_aff_state(unsigned int, unsigned long);

/* nasty hack to prevent semihosting being included on this platform */
#define __SEMIHOSTING_H__
#define semihosting_get_flen(file_name) 0
#define semihosting_download_file(file_name, buf_size, buf) 0

#endif /*__ASSEMBLY__*/

#endif /* __PLATFORM_H__ */
