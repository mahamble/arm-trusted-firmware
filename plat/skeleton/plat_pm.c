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

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <arch_helpers.h>
#include <console.h>
#include <platform.h>
#include <bl_common.h>
#include <bl31.h>
#include <bakery_lock.h>
#include <cci400.h>
#include <gic.h>
/* Only included for error codes */
#include <psci.h>

/*******************************************************************************
 * Handler called when an affinity instance is about to be turned on. The
 * level and mpidr determine the affinity instance.
 ******************************************************************************/
static int affinst_on(unsigned long mpidr,
		   unsigned long sec_entrypoint,
		   unsigned long ns_entrypoint,
		   unsigned int afflvl,
		   unsigned int state)
{
	int rc = PSCI_E_SUCCESS;

	/* PLAT_TODO: add code here to turn on affinst */
	return rc;
}

/*******************************************************************************
 * Handler called when an affinity instance is about to be turned off. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions.
 *
 * CAUTION: This function is called with coherent stacks so that caches can be
 * turned off, flushed and coherency disabled. There is no guarantee that caches
 * will remain turned on across calls to this function as each affinity level is
 * dealt with. So do not write & read global variables across calls. It will be
 * wise to do flush a write to the global to prevent unpredictable results.
 ******************************************************************************/
static int affinst_off(unsigned long mpidr,
		    unsigned int afflvl,
		    unsigned int state)
{
	int rc = PSCI_E_SUCCESS;
	unsigned int gicc_base, ectlr;
	unsigned long cpu_setup, cci_setup;

	switch (afflvl) {
	case MPIDR_AFFLVL1:
		if (state == PSCI_STATE_OFF) {
			/*
			 * Disable coherency if this cluster is to be
			 * turned off
			 */
			cci_setup = platform_get_cfgvar(CONFIG_HAS_CCI);
			if (cci_setup)
				cci_disable_coherency(mpidr);

			/* PLAT_TODO: add code here to turn off affinst1 */

		}
		break;

	case MPIDR_AFFLVL0:
		if (state == PSCI_STATE_OFF) {

			/*
			 * Take this cpu out of intra-cluster coherency if
			 * the platform supports the SMP bit.
			 */
			/* PLAT_TODO: - remove this code if other entities do
			 * coherency management
			 */
			cpu_setup = platform_get_cfgvar(CONFIG_CPU_SETUP);
			if (cpu_setup) {
				ectlr = read_cpuectlr();
				ectlr &= ~CPUECTLR_SMP_BIT;
				write_cpuectlr(ectlr);
			}

			/*
			 * Prevent interrupts from spuriously waking up
			 * this cpu
			 */
			gicc_base = platform_get_cfgvar(CONFIG_GICC_ADDR);
			gic_cpuif_deactivate(gicc_base);

			/* PLAT_TODO: add code here to turn off this cpu
			 * (affinst0)
			 */
		}
		break;

	default:
		assert(0);
	}

	return rc;
}

/*******************************************************************************
 * Handler called when an affinity instance is about to be suspended. The
 * level and mpidr determine the affinity instance. The 'state' arg. allows the
 * platform to decide whether the cluster is being turned off and take apt
 * actions.
 *
 * CAUTION: This function is called with coherent stacks so that caches can be
 * turned off, flushed and coherency disabled. There is no guarantee that caches
 * will remain turned on across calls to this function as each affinity level is
 * dealt with. So do not write & read global variables across calls. It will be
 * wise to do flush a write to the global to prevent unpredictable results.
 ******************************************************************************/
static int affinst_suspend(unsigned long mpidr,
			unsigned long sec_entrypoint,
			unsigned long ns_entrypoint,
			unsigned int afflvl,
			unsigned int state)
{
	int rc = PSCI_E_SUCCESS;
	unsigned int gicc_base, ectlr;
	unsigned long cpu_setup, cci_setup, linear_id;
	mailbox *mboxes;

	switch (afflvl) {
	case MPIDR_AFFLVL1:
		if (state == PSCI_STATE_OFF) {
			/*
			 * Disable coherency if this cluster is to be
			 * turned off
			 */
			cci_setup = platform_get_cfgvar(CONFIG_HAS_CCI);
			if (cci_setup)
				cci_disable_coherency(mpidr);

			/* PLAT_TODO: add code to turn the cluster off */

		}
		break;

	case MPIDR_AFFLVL0:
		if (state == PSCI_STATE_OFF) {
			/*
			 * Take this cpu out of intra-cluster coherency if
			 * the platform supports the SMP bit.
			 */
			/* PLAT_TODO: remove this code if coherency control
			 * is handled elsewhere
			 */
			cpu_setup = platform_get_cfgvar(CONFIG_CPU_SETUP);
			if (cpu_setup) {
				ectlr = read_cpuectlr();
				ectlr &= ~CPUECTLR_SMP_BIT;
				write_cpuectlr(ectlr);
			}

			/* Program the jump address for the target cpu */
			linear_id = platform_get_core_pos(mpidr);
			mboxes = (mailbox *) (TZDRAM_BASE + MBOX_OFF);
			mboxes[linear_id].value = sec_entrypoint;
			flush_dcache_range((unsigned long) &mboxes[linear_id],
					   sizeof(unsigned long));

			/*
			 * Prevent interrupts from spuriously waking up
			 * this cpu
			 */
			gicc_base = platform_get_cfgvar(CONFIG_GICC_ADDR);
			gic_cpuif_deactivate(gicc_base);

			/* PLAT_TODO: add code to power this cpu off and enable
			 * wakeup interrupts.
			 */
		}
		break;

	default:
		assert(0);
	}

	return rc;
}

/*******************************************************************************
 * Handler called when an affinity instance has just been powered on after
 * being turned off earlier. The level and mpidr determine the affinity
 * instance. The 'state' arg. allows the platform to decide whether the cluster
 * was turned off prior to wakeup and do what's necessary to setup it up
 * correctly.
 ******************************************************************************/
static int affinst_on_finish(unsigned long mpidr,
			  unsigned int afflvl,
			  unsigned int state)
{
	int rc = PSCI_E_SUCCESS;
	unsigned long linear_id, cpu_setup, cci_setup;
	mailbox *mboxes;
	unsigned int gicd_base, gicc_base, ectlr;

	switch (afflvl) {

	case MPIDR_AFFLVL1:
		/* Enable coherency if this cluster was off */
		if (state == PSCI_STATE_OFF) {

			/*
			 * PLAT_TODO: add platform code here to clear
			 * outstanding request.
			 */
			cci_setup = platform_get_cfgvar(CONFIG_HAS_CCI);
			if (cci_setup)
				cci_enable_coherency(mpidr);
		}
		break;

	case MPIDR_AFFLVL0:
		/*
		 * Ignore the state passed for a cpu. It could only have
		 * been off if we are here.
		 */

		/*
		 * Turn on intra-cluster coherency if the platform supports
		 * it.
		 */
		cpu_setup = platform_get_cfgvar(CONFIG_CPU_SETUP);
		if (cpu_setup) {
			ectlr = read_cpuectlr();
			ectlr |= CPUECTLR_SMP_BIT;
			write_cpuectlr(ectlr);
		}

		/*
		 * PLAT_TODO: platform specific setup to ensure interrupts do
		 * not interfere with a cpu power down unless the bit is set
		 * again
		 */

		/* Zero the jump address in the mailbox for this cpu */
		mboxes = (mailbox *) (TZDRAM_BASE + MBOX_OFF);
		linear_id = platform_get_core_pos(mpidr);
		mboxes[linear_id].value = 0;
		flush_dcache_range((unsigned long) &mboxes[linear_id],
				   sizeof(unsigned long));

		gicd_base = platform_get_cfgvar(CONFIG_GICD_ADDR);
		gicc_base = platform_get_cfgvar(CONFIG_GICC_ADDR);

		/* Enable the gic cpu interface */
		gic_cpuif_setup(gicc_base);

		/* PLAT_TODO: This setup is needed only after a cold boot */
		gic_pcpu_distif_setup(gicd_base);

		/* PLAT_TODO: Allow access to the System counter timer module */
		break;

	default:
		assert(0);
	}

	return rc;
}

/*******************************************************************************
 * Handler called when an affinity instance has just been powered on after
 * having been suspended earlier. The level and mpidr determine the affinity
 * instance.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
static int affinst_suspend_finish(unsigned long mpidr,
			       unsigned int afflvl,
			       unsigned int state)
{
	return affinst_on_finish(mpidr, afflvl, state);
}


/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static plat_pm_ops pm_ops = {
	0,
	affinst_on,
	affinst_off,
	affinst_suspend,
	affinst_on_finish,
	affinst_suspend_finish,
};

/*******************************************************************************
 * Export the platform specific power ops & initialize the power controller
 ******************************************************************************/
int platform_setup_pm(plat_pm_ops **plat_ops)
{
	*plat_ops = &pm_ops;
	return 0;
}
