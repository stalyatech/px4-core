############################################################################
#
#   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

option(PX4IO_PERF "Enable px4io perf counters" OFF)

if(${PX4_CHIP} STREQUAL "sg2000")
	set(px4iofirmware_arch_sources
		sg2000/adc.cpp
		sg2000/serial.cpp
		fw_manifest.c
	)
	set(px4iofirmware_arch_libs
		arch_watchdog_iwdg
		tpu_fwmgmt_proto
	)
else()
	set(px4iofirmware_arch_sources
		stm32/adc.cpp
		stm32/serial.cpp
	)
	set(px4iofirmware_arch_libs
		arch_watchdog_iwdg
	)
endif()

add_library(px4iofirmware
	${px4iofirmware_arch_sources}
	controls.cpp
	mixer.cpp
	px4io.cpp
	registers.c
	safety_button.cpp
)

set_property(GLOBAL APPEND PROPERTY PX4_MODULE_LIBRARIES px4iofirmware)
target_include_directories(px4iofirmware PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(px4iofirmware
	PUBLIC
		arch_io_pins
		${px4iofirmware_arch_libs}
		nuttx_apps
		nuttx_arch
		nuttx_c
		nuttx_mm
		rc
)

if(PX4IO_PERF)
	target_compile_definitions(px4iofirmware PRIVATE PX4IO_PERF)
	target_link_libraries(px4iofirmware PRIVATE perf)
endif()
