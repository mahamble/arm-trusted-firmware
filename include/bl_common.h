/*
 * Copyright (c) 2013, ARM Limited and Contributors. All rights reserved.
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

#ifndef __BL_COMMON_H__
#define __BL_COMMON_H__

#define SECURE		0
#define NON_SECURE	1

#define UP	1
#define DOWN	0

/*******************************************************************************
 * Constants for loading images. When BLx wants to load BLy, it looks at a
 * meminfo structure to find the extents of free memory. Then depending upon
 * how it has been configured, it can either load BLy at the top or bottom of
 * the free memory. These constants indicate the choice.
 * TODO: Make this configurable while building the trusted firmware.
 ******************************************************************************/
#define TOP_LOAD	0x1
#define BOT_LOAD	!TOP_LOAD
#define LOAD_MASK	(1 << 0)

/*******************************************************************************
 * Size of memory for sharing data while changing exception levels.
 *
 * There are 2 cases where this memory buffer is used:
 *
 *   - when BL1 (running in EL3) passes control to BL2 (running in S-EL1).
 *     BL1 needs to pass the memory layout to BL2, to allow BL2 to find out
 *     how much free trusted ram remains;
 *
 *   - when BL2 (running in S-EL1) passes control back to BL1 (running in EL3)
 *     to make it run BL31.  BL2 needs to pass the memory layout, as well as
 *     information on how to pass control to the non-trusted software image.
 ******************************************************************************/
#define EL_CHANGE_MEM_SIZE	(sizeof(meminfo) + sizeof(el_change_info))

/*******************************************************************************
 * Macro to flag a compile time assertion. It uses the preprocessor to generate
 * an invalid C construct if 'cond' evaluates to false.
 * The following  compilation error is triggered if the assertion fails:
 * "error: size of array 'msg' is negative"
 ******************************************************************************/
#define CASSERT(cond, msg)	typedef char msg[(cond) ? 0 : -1]

/******************************************************************************
 * Opcode passed in x0 to tell next EL that we want to run an image.
 * Corresponds to the function ID of the only SMC that the BL1 exception
 * handlers service. That's why the chosen value is the first function ID of
 * the ARM SMC64 range.
 *****************************************************************************/
#define RUN_IMAGE	0xC0000000


#ifndef __ASSEMBLY__
/*******************************************************************************
 * Structure used for telling the next BL how much of a particular type of
 * memory is available for its use and how much is already used.
 ******************************************************************************/
typedef struct {
	unsigned long total_base;
	long total_size;
	unsigned long free_base;
	long free_size;
	unsigned long attr;
	unsigned long next;
} meminfo;

typedef struct {
	unsigned long arg0;
	unsigned long arg1;
	unsigned long arg2;
	unsigned long arg3;
	unsigned long arg4;
	unsigned long arg5;
	unsigned long arg6;
	unsigned long arg7;
} aapcs64_params;

/*******************************************************************************
 * This structure represents the superset of information needed while switching
 * exception levels. The only two mechanisms to do so are ERET & SMC. In case of
 * SMC all members apart from 'aapcs64_params' will be ignored. The 'next'
 * member is a placeholder for a complicated case in the distant future when BL2
 * will load multiple BL3x images as well as a non-secure image. So multiple
 * such structures will have to be passed to BL31 in S-EL3.
 ******************************************************************************/
typedef struct {
	unsigned long entrypoint;
	unsigned long spsr;
	unsigned long security_state;
	aapcs64_params args;
	unsigned long next;
} el_change_info;

/*******************************************************************************
 * Function & variable prototypes
 ******************************************************************************/
extern unsigned long page_align(unsigned long, unsigned);
extern void change_security_state(unsigned int);
extern int drop_el(aapcs64_params *, unsigned long, unsigned long);
extern long raise_el(aapcs64_params *);
extern long change_el(el_change_info *);
extern unsigned long make_spsr(unsigned long, unsigned long, unsigned long);
extern void init_bl2_mem_layout(meminfo *,
			        meminfo *,
			        unsigned int,
			        unsigned long) __attribute__((weak));
extern void init_bl31_mem_layout(const meminfo *,
				 meminfo *,
				 unsigned int) __attribute__((weak));
extern unsigned long load_image(meminfo *, const char *, unsigned int, unsigned long);
extern int run_image(unsigned long,
		     unsigned long,
		     unsigned long,
		     meminfo *,
		     void *);
extern unsigned long *get_el_change_mem_ptr(void);

#endif /*__ASSEMBLY__*/

#endif /* __BL_COMMON_H__ */
