# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := /home/tom/code/cfbl/crazyflie-firmware

#
# We override the default OOT_CONFIG here, we could also name our config
# to oot-config and that would be the default.
#
OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk

# CUSTOM MAKEFILE CODE #########################################################
APP_NAME := $(shell basename $(CURDIR))

tb:
	rm -rf $(CRAZYFLIE_BASE)/apps/$(APP_NAME) || true
	mkdir -p $(CRAZYFLIE_BASE)/apps/$(APP_NAME)
	cp -rf . $(CRAZYFLIE_BASE)/apps/$(APP_NAME)
	sed -i 's+^CRAZYFLIE_BASE.*+CRAZYFLIE_BASE := ../..+g' $(CRAZYFLIE_BASE)/apps/$(APP_NAME)/Makefile
	bash -i -c "cd $(CRAZYFLIE_BASE) && tb make_app apps/$(APP_NAME)"
	cp -rf $(CRAZYFLIE_BASE)/apps/$(APP_NAME)/build .
.PHONY: tb

upload:
	CLOAD_CMDS="-w radio://0/66/2M" make cload
.PHONY: upload

freeze:
	pip freeze --local | grep -v "pkg-resources" > requirements.txt
.PHONY: freeze
