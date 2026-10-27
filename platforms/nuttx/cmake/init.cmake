############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS_EQUAL 7)
		message(FATAL_ERROR "GCC 7 or older no longer supported. https://docs.px4.io/main/en/dev_setup/dev_env.html")
	endif()
endif()

if(NOT PX4_BOARD)
	message(FATAL_ERROR "PX4_BOARD must be set (eg px4_fmu-v2)")
endif()

if(NOT PX4_BINARY_DIR)
	message(FATAL_ERROR "PX4_BINARY_DIR must be set")
endif()

if(NOT PX4_BOARD_DIR)
	message(FATAL_ERROR "PX4_BOARD_DIR must be set")
endif()

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config CACHE FILEPATH "PX4 NuttX config" FORCE)

# NuttX defconfig
#  cmake should trigger reconfigure if defconfig changes
set(NUTTX_DEFCONFIG ${NUTTX_CONFIG_DIR}/${NUTTX_CONFIG}/defconfig CACHE FILEPATH "path to defconfig" FORCE)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${NUTTX_DEFCONFIG})

set(NUTTX_SRC_DIR  ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX)
set(NUTTX_DIR      ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/nuttx CACHE FILEPATH "NuttX directory" FORCE)
set(NUTTX_APPS_DIR ${PX4_SOURCE_DIR}/platforms/nuttx/NuttX/apps CACHE FILEPATH "NuttX apps directory" FORCE)

px4_add_git_submodule(TARGET git_nuttx PATH "${NUTTX_SRC_DIR}/nuttx")
px4_add_git_submodule(TARGET git_nuttx_apps PATH "${NUTTX_SRC_DIR}/apps")

# make olddefconfig (inflate defconfig to full .config)
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${NUTTX_CONFIG_DIR}/src) # needed for NuttX build
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/Make.defs.in ${NUTTX_DIR}/Make.defs) # Create a temporary Toplevel Make.defs for the oldconfig step

# Inflate the defconfig into nuttx/.config only when its content changed.
# This step rewrites .config and touches defconfig_inflate_stamp, and the
# NuttX context step then regenerates include/nuttx/config.h, which is
# included by every PX4 object: doing it on every configure turned each
# build into a full rebuild.
file(SHA256 ${NUTTX_DEFCONFIG} _nuttx_defconfig_hash)
set(_nuttx_inflate_hash_file ${CMAKE_CURRENT_BINARY_DIR}/defconfig_inflate_stamp.hash)
set(_nuttx_inflate_hash "")
if(EXISTS ${_nuttx_inflate_hash_file})
	file(READ ${_nuttx_inflate_hash_file} _nuttx_inflate_hash)
	string(STRIP "${_nuttx_inflate_hash}" _nuttx_inflate_hash)
endif()

if(NOT _nuttx_inflate_hash STREQUAL _nuttx_defconfig_hash)
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config)
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/defconfig)

	execute_process(
		COMMAND ${NUTTX_SRC_DIR}/tools/px4_nuttx_make_olddefconfig.sh
		WORKING_DIRECTORY ${NUTTX_DIR}
		OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/nuttx_olddefconfig.log
		RESULT_VARIABLE ret
	)
	execute_process(
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/defconfig_inflate_stamp
		WORKING_DIRECTORY ${NUTTX_DIR}
	)
	file(WRITE ${_nuttx_inflate_hash_file} "${_nuttx_defconfig_hash}\n")
endif()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${NUTTX_DIR}/defconfig)

execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/.config ${PX4_BINARY_DIR}/NuttX/nuttx/.config)

###############################################################################
# NuttX cmake defconfig
###############################################################################

# parse nuttx config options for cmake
file(STRINGS ${PX4_BINARY_DIR}/NuttX/nuttx/.config ConfigContents)
foreach(NameAndValue ${ConfigContents})
	# Strip leading spaces
	string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

	# Find variable name
	string(REGEX MATCH "^CONFIG[^=]+" Name ${NameAndValue})

	if(Name)
		# Find the value
		string(REPLACE "${Name}=" "" Value ${NameAndValue})

		# remove extra quotes
		string(REPLACE "\"" "" Value ${Value})

		# Set the variable
		#message(STATUS "${Name} ${Value}")
		set(${Name} ${Value} CACHE INTERNAL "NUTTX DEFCONFIG: ${Name}" FORCE)
	endif()
endforeach()
