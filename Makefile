# The TARGET variable determines what target system the application is
# compiled for. It either refers to an XN file in the source directories
# or a valid argument for the --target option when compiling
TARGET = XUF208-256-TQ64-C10

# The APP_NAME variable determines the name of the final .xe file. It should
# not include the .xe postfix. If left blank the name will default to
# the project name
APP_NAME = 

# The USED_MODULES variable lists other module used by the application.
USED_MODULES = lib_gpio(>=1.0.0) lib_i2s(>=2.1.0) lib_logging(>=2.0.0) lib_spdif(>=2.0.2) lib_src(>=1.0.0)

# The flags passed to xcc when building the application
# You can also set the following to override flags for a particular language:
# XCC_XC_FLAGS, XCC_C_FLAGS, XCC_ASM_FLAGS, XCC_CPP_FLAGS
# If the variable XCC_MAP_FLAGS is set it overrides the flags passed to
# xcc for the final link (mapping) stage.
XCC_FLAGS = -O2 -g -report
XCC_XC_FLAGS = -DDEBUG_PRINT_ENABLE=1 -fxscope
#-Wxcore-fptrgroup Add this when we move to 14.2.0 for build infrastructure

# The XCORE_ARM_PROJECT variable, if set to 1, configures this
# project to create both xCORE and ARM binaries.
XCORE_ARM_PROJECT = 0

# The VERBOSE variable, if set to 1, enables verbose output from the make system.
VERBOSE = 0
ENABLE_STAGED_BUILD=0

XMOS_MAKE_PATH ?= ../..
-include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common
