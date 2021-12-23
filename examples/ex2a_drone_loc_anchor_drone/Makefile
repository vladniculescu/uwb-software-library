# enable app support
APP=1
APP_STACKSIZE=3000
CRAZYFLIE_BASE=../../../crazyflie-firmware
UWB_API_BASE=../..

# ID ?= 0
# PX ?= 0
# PY ?= 0
# PZ ?= 0
# CFLAGS += -DNODE_ID=$(ID) -DPOSX=$(PX) -DPOSX=$(PY) -DPOSX=$(PZ)

# $(info    ID is $(ID))

# Decawave API
VPATH += $(UWB_API_BASE)/
VPATH += $(UWB_API_BASE)/decawave_api/
INCLUDES += -I$(UWB_API_BASE)/decawave_api -I$(UWB_API_BASE) -I$(UWB_API_BASE)/inc
PROJ_OBJ += deca_device.o deca_params_init.o dwm_utils.o

# UWB API
PROJ_OBJ += uwb_api.o
PROJ_OBJ += platform_config.o

# Example file
PROJ_OBJ += ex2a_main.o

include $(CRAZYFLIE_BASE)/Makefile

