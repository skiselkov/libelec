# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Copyright 2023 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 3.9)

project(nettest C)

option(READLINE_INSTALL_PREFIX "Where GNU readline's headers and libs are")

# Source file setup
file(GLOB LIBACFUTILS "${CMAKE_SOURCE_DIR}/libacfutils-redist-v0.37")
set(ACFUTILS_DLL_VERSION "37")
file(GLOB LIBELEC "${CMAKE_SOURCE_DIR}/../src")

file(GLOB LIBELEC_SRCS
    "${LIBELEC}/libelec.c"
    "${LIBELEC}/libelec_drawing.c"
    )
file(GLOB LIBELEC_HDRS
    "${LIBELEC}/libelec.h"
    "${LIBELEC}/libelec_types_impl.h"
    "${LIBELEC}/libelec_drawing.h"
    )
set(SRCS ${LIBELEC_SRCS} "${CMAKE_SOURCE_DIR}/nettest.c")
set(HDRS ${LIBELEC_HDRS})

set(ALL_SRCS ${SRCS} ${HDRS})
list(SORT ALL_SRCS)

execute_process(COMMAND git rev-parse --short HEAD
    OUTPUT_VARIABLE LIBELEC_VERSION)
string(REGEX REPLACE "\n$" "" LIBELEC_VERSION "${LIBELEC_VERSION}")
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S UTC" UTC)

if(APPLE)
	# Build universal binaries on macOS
	set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64")
endif()

# Instantiates project
add_executable(nettest ${ALL_SRCS})

# Compiler setup
if(MINGW)
	set(PLAT_SHORT "mingw64")
elseif(WIN32)
	set(PLAT_SHORT "win64")
elseif(APPLE)
	set(PLAT_SHORT "mac64")
else()
	set(PLAT_SHORT "lin64")
endif()

include_directories(nettest PUBLIC
    "${CMAKE_SOURCE_DIR}/../src"
    "${LIBACFUTILS}/include"
    "${LIBACFUTILS}/${PLAT_SHORT}/include"
    )

# Compiler flags
set_target_properties(nettest PROPERTIES C_STANDARD_REQUIRED "99")
if(UNIX)
	# Enable maximum warnings and errors
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -W -Wall -Wextra -Werror \
	    -Wno-missing-field-initializers -pedantic")
endif()
if(UNIX AND NOT APPLE)
	# Disable annoying warnings on GCC
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format-truncation")
	if (${SANITIZE})
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address")
	endif()
endif()

# Defines
add_definitions(-DDEBUG)
add_definitions(-DLIBELEC_VERSION="${LIBELEC_VERSION}")
add_definitions(-DBUILD_TIMESTAMP="${BUILD_TIMESTAMP}")

if(WIN32)
	# Needed for threading primitives
	add_definitions(-D_WIN32_WINNT=0x0600)
else()
	# Needed for gmtime_r in libacfutils
	add_definitions(-D_GNU_SOURCE)
endif()

# Platform type switches for libacfutils
if(WIN32)
	add_definitions(-DIBM=1 -DLIN=0 -DAPL=0)
elseif(APPLE)
	add_definitions(-DIBM=0 -DLIN=0 -DAPL=1)
else()
	add_definitions(-DIBM=0 -DLIN=1 -DAPL=0)
endif()

# When we're building with XPLM features (network visualization),
# we need to set XPLM version support flags
if(${LIBELEC_VIS})
	add_definitions(-DXPLM200=1 -DXPLM210=1)
	add_definitions(-DXPLM300=1 -DXPLM301=1 -DXPLM302=1)
else()
	# Stops libacfutils/log.h from complaining about log_xplm_cb()
	add_definitions(-D_LACF_WITHOUT_XPLM)
endif()

if(UNIX)
	# Do not export any symbols as externally visible
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden")
endif()

if(MINGW)
	set(PLATFORM_LIBS
	    "-static"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libcairo.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libcurl.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libpixman-1.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libfreetype.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libpng16.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libssl.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libcrypto.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libz.a"
	    "-lssp"
	    "-lws2_32"
	    "-lcrypt32"
	    "-ldbghelp"
	    "-lpsapi"
	    "-lwinmm")
	find_library(LIBACFUTILS_LIB acfutils
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib")
elseif(WIN32)
	set(PLATFORM_LIBS "")
	find_library(LIBACFUTILS_LIB "acfutils${ACFUTILS_DLL_VERSION}"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib")
else()
	find_library(LIBACFUTILS_LIB acfutils
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib")
	if (NOT APPLE)
		find_library(MATH_LIB "m")
	endif()
	set(PLATFORM_LIBS
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libcairo.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libcurl.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libpixman-1.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libfreetype.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libpng16.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libssl.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libcrypto.a"
	    "${LIBACFUTILS}/${PLAT_SHORT}/lib/libz.a"
	    "${MATH_LIB}"
	    )
endif()

if(READLINE_INSTALL_PREFIX)
	set(READLINE_LIB_PREFIX "${READLINE_INSTALL_PREFIX}/lib/")
	set(READLINE_HDR "${READLINE_INSTALL_PREFIX}/include")
elseif(MINGW)
	# If the option wasn't specified, try to fall back to using our
	# bundled GNU readline
	set(READLINE_LIB_PREFIX "${CMAKE_SOURCE_DIR}/mingw-readline/lib/")
	set(READLINE_HDR "${CMAKE_SOURCE_DIR}/mingw-readline/include")
else()
	find_path(READLINE_HDR "readline/readline.h")
endif()

find_library(READLINE_LIB "libreadline.a" ${READLINE_LIB_PREFIX})
if(NOT READLINE_LIB)
	# Fall back to trying the dynamic version of readline
	find_library(READLINE_LIB "readline" ${READLINE_LIB_PREFIX})
elseif(MINGW)
	# Under MinGW, we use the MSYS2 version of readline, which depends
	# on libncurses, and which in turn depends on winpthreads (due to
	# clock_gettime() being implemented there in MinGW).
	find_library(NCURSES_LIB "libncurses.a" ${READLINE_LIB_PREFIX})
	find_library(WINPTHREAD_LIB "libwinpthread.a")
	set(READLINE_PLATFORM_LIBS ${NCURSES_LIB} ${WINPTHREAD_LIB})
elseif(APPLE)
	# On macOS, static GNU readline depends on libncurses
	find_library(READLINE_PLATFORM_LIBS "libncurses.a")
elseif(UNIX)
	# On Linux, static GNU readline depends on libtinfo
	find_library(READLINE_PLATFORM_LIBS "libtinfo.a")
endif()

if(READLINE_HDR AND READLINE_LIB)
	message("GNU readline found")
	add_definitions(-DWITH_READLINE)
	include_directories(nettest PUBLIC "${READLINE_HDR}")
	set(PLATFORM_LIBS ${PLATFORM_LIBS}
	    ${READLINE_LIB} ${READLINE_PLATFORM_LIBS})
else()
	message("GNU readline NOT found")
endif()

target_link_libraries(nettest
    ${LIBACFUTILS_LIB}
    ${MATH_LIB}
    ${PLATFORM_LIBS}
    )
