\mainpage

# Introduction

This is an aircraft electrical system simulation library for use in flight
simulators such as X-Plane and MSFS. It can also be used outside of any
particular flight simulator.

## Donations

If you like my work, consider leaving a donation by following the PayPal
link below:

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=8DN9LYD5VP4NY)

## License

This library is licensed under the Mozilla Public License v2.0. This lets
you incorporate it into your projects (even closed-source/payware ones),
**without having to disclose your proprietary source code**. The license
only requires that you disclose modifications which you have made to the
source code contained in this library. Any files that are not part of
this library are **not** covered by this requirement.

For the full text of the license, see [COPYING](COPYING.md).

\see For a quick summary of things you must do to comply with libelec's
license when used in your project, see Q8 through Q10 here:
[MPL v2.0 Frequently Asked Questions](https://www.mozilla.org/en-US/MPL/2.0/FAQ/)

## Documentation

To access the full documentation set for libelec, go to:
https://skiselkov.github.io/libelec

## History

libelec was developed for the [Hot Start CL650](https://www.x-aviation.com/catalog/product_info.php/take-command-hot-start-challenger-650-p-212)
for X-Plane, whiich uses libelec to implement its entire electrical system.
This includes:

- ~450 electrical buses
- >800 devices connected to those buses
- ~400 circuit breakers

libelec gives you the power to do that same level of sophistication in
your project without having to rewrite thousands of lines of
multi-threaded spaghetti code. Enjoy!

## Prerequisites

libelec depends on libacfutils, which you can get here:
https://github.com/skiselkov/libacfutils

You will also require a C99-compliant compiler. Most modern C compilers
should be able to compile libelec, including GCC (v3.1+), Clang (v1.0+)
and MSVC (Visual Studio 2015+).

## Building

You can build libelec in one of two ways:

1. **The preferred method:** simply include the source files of the
   library in your project. The library's core functionality is
   implemented in `libelec.c`, so that's the only file you need to add.
   See "Directly Incorporating Into Your Project" below for more
   information on some build-control macros you can define to fine-tune
   the build.

   If you wish to include the state visualization capability, you will
   need to also add `libelec_drawing.c` and `libelec_vis.c` to your build
   as well. See `libelec_drawing.h` and `libelec_vis.h` for more
   information.

   \note The built-in visualization capability currently depends on
   X-Plane's SDK. If you want to use it from another simulator, you might
   need to do some porting work. As always, pull requests are welcome.

2. Use CMake and the included `src/CMakeLists.txt` file to build a static
   library. You will then want to add this library to your linker inputs.
   See "Building Using CMake" below for more details.

In both cases, you will need to include `libelec.h` from your project to
control the library.

### Directly Incorporating Into Your Project

libelec is written in C99, so you will want to make sure your compiler's
C language mode is set to at least C99 (newer is ok, but ANSI C90, ISO
C89 or K&R C are too old). You can also define the following optional
compiler macros to fine-tune the libelec build:

- `XPLANE` - if this macro is defined, libelec will depend on running
   inside of X-Plane and having the XPLM available. The benefit is that
   libelec will be able to automatically adapt its simulation rate to
   that of X-Plane. If the user enters time acceleration, libelec will
   automatically "follow" along. If you want to utilize libelec in a
   different (non-XPlane) environment, you will need to control the
   simulation rate yourself by calling libelec_sys_set_time_factor()
   regularly.

- `LIBELEC_WITH_DRS` - if defined, libelec will expose all component
   state via X-Plane read-only datarefs. Every component will have a set
   aof double-precision floating point datarefs constructed for it, using
   the following pattern:
   * `libelec/comp/<COMPONENT_NAME>/in_volts`
   * `libelec/comp/<COMPONENT_NAME>/out_volts`
   * `libelec/comp/<COMPONENT_NAME>/in_amps`
   * `libelec/comp/<COMPONENT_NAME>/out_amps`
   * `libelec/comp/<COMPONENT_NAME>/in_pwr`
   * `libelec/comp/<COMPONENT_NAME>/out_pwr`

### Building Using CMake

Building the project using CMake will produce a static library, which you
can then link into your project. To build using the included
`CMakeLists.txt` file, do the following:

1. Create a `build` subdirectory under `src` to hold the build products:
```
$ mkdir src/build
$ cd src/build
```
2. Configure CMake. Please note that you will require the redistributable
   package of libacfutils installed somewhere (say at path `${LIBACFUTILS}`):
```
$ cmake .. -DLIBACFUTILS=${LIBACFUTILS}
```
3. Now just run the build:
```
$ make
```

This will produce a static library named either `libelec.lib` (on
Windows), or `libelec.a` (on macOS and Linux). On macOS, the built
library will contain both the x86-64 and arm64 bits and can thus be used
to build universal software. On Linux, you can cross-compile to Windows
(MinGW) using the following CMake command instead (you will obviously
also need to have the MinGW toolchain installed separately):

```
$ cmake .. -DLIBACFUTILS=${LIBACFUTILS} \
    -DCMAKE_TOOLCHAIN_FILE=../XCompile.cmake -DHOST=x86_64-w64-mingw32
$ make
```

## Usage

To initialize the library, use libelec_new(). This loads an electrical
network definition file and returns a network handle. This network can
then be started using libelec_sys_start(). Once started, you can then
control the network component state using the various component functions
contained in `libelec.h` while the network operates asynchronously on a
background thread.

Before shutting down, you must first stop the network using
libelec_sys_stop() and subsequently deallocate the network using
libelec_destroy().

## Thread-Safety

Unless stated otherwise, all libelec functions are thread safe.

## Configuration File Format

libelec requires that you specify a configuration file every time you
call libelec_new(). This file defines the entire electrical network which
is to be simulated.

See [Configuration File Format](ConfFileFormat.md) for more information
on how the configuration file must be constructed.
