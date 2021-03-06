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

#include <stdio.h>
#include <console.h>

#if defined (__GNUC__)

#include <stdio.h>
#include <stddef.h> /* size_t */
#include <stdarg.h> /* va_list */

// Code from VTB.
#include "mem.c"

// Make mem functions that will operate on DEV mem. "memset_io"?


//Code from VTB
#include "strlen.c"

int puts(const char *s)
{
	int count = 0;
	while(*s)
	{
		if (console_putc(*s++)) {
			count++;
		} else {
			count = EOF; // -1 in stdio.h
			break;
		}
	}
	return count;
}

// From VTB
#include "ctype.h"
#include "subr_prf.c"

 // Choose max of 128 chars for now.
#define PRINT_BUFFER_SIZE 128
int printf(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	char buf[PRINT_BUFFER_SIZE];
	vsnprintf(buf, sizeof(buf) - 1, fmt, args);
	buf[PRINT_BUFFER_SIZE - 1] = '\0';
	return puts(buf);
}


// I just made this up. Probably make it beter.
void __assert_func (const char *file, int l, const char *func, const char *error)
{
	printf("ASSERT: %s <%d> : %s\n\r", func, l, error);
	while(1);
}

extern void __assert_fail (const char *assertion, const char *file,
			   unsigned int line, const char *function)
{
	printf("ASSERT: %s <%d> : %s\n\r", function, line, assertion);
	while(1);
}


// I just made this up. Probably make it beter.
void abort (void)
{
	printf("ABORT\n\r");
	while(1);
}


#else
#error "No standard library binding defined."
#endif
