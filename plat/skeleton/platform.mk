#
# Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

PLAT_INCLUDES		:=	-Idrivers/arm/interconnect/cci-400	\
				-Iplat/skeleton

PLAT_BL1_C_VPATH	:=	drivers/arm/interconnect/cci-400	\
				lib/stdlib

PLAT_BL1_S_VPATH	:=

PLAT_BL2_C_VPATH	:=	drivers/arm/interconnect/cci-400	\
				lib/stdlib

PLAT_BL2_S_VPATH	:=

PLAT_BL31_C_VPATH	:=	drivers/arm/interconnect/cci-400	\
				lib/stdlib

PLAT_BL31_S_VPATH	:=

PLAT_BL_COMMON_OBJS	:=	mmio.o					\
				sysreg_helpers.o

BL1_OBJS		+=	bl1_plat_setup.o			\
				bl1_plat_helpers.o			\
				plat_helpers.o				\
				plat_common.o				\
				cci400.o

BL2_OBJS		+=	bl2_plat_setup.o			\
				plat_common.o

BL31_OBJS		+=	bl31_plat_setup.o			\
				plat_helpers.o				\
				plat_common.o				\
				plat_pm.o				\
				plat_topology.o				\
				plat_gic.o				\
				cci400.o				\
				gic_v2.o				\
				gic_v3.o
