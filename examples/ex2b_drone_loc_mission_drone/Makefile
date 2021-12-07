# enable app support
APP=1
APP_STACKSIZE=3000
CRAZYFLIE_BASE=../../../crazyflie-firmware
UWB_API_BASE=../..

# Decawave API
VPATH += $(UWB_API_BASE)/
VPATH += $(UWB_API_BASE)/decawave_api/
INCLUDES += -I$(UWB_API_BASE)/decawave_api -I$(UWB_API_BASE) -I$(UWB_API_BASE)/inc
PROJ_OBJ += deca_device.o deca_params_init.o dwm_utils.o

# UWB API
PROJ_OBJ += uwb_api.o
PROJ_OBJ += platform_config.o

# Example file
PROJ_OBJ += ex2b_main.o
PROJ_OBJ += trajectory.o

include $(CRAZYFLIE_BASE)/Makefile
