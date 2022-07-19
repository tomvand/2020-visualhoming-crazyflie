# Variables
PYTHON ?= python3.8

CRAZYFLIE_BASE ?= /home/tom/code/cfbl/crazyflie-firmware
CLOAD_CMDS ?= "-w radio://0/66/2M"

# Commands

all: venv build
.PHONY: all

build:
	cd app && $(MAKE) CRAZYFLIE_BASE=$(CRAZYFLIE_BASE) 
.PHONY: build

upload:
	cd app && $(MAKE) CRAZYFLIE_BASE=$(CRAZYFLIE_BASE) CLOAD_CMDS=$(CLOAD_CMDS) cload
.PHONY: upload

clean:
	cd app && $(MAKE) CRAZYFLIE_BASE=$(CRAZYFLIE_BASE) clean
.PHONY: clean

freeze:
	pip freeze --local | grep -v "pkg-resources" > requirements.txt
.PHONY: freeze


# Targets

venv:
	rm -rf venv
	virtualenv -p $(PYTHON) venv
	bash -c "source venv/bin/activate && pip install -r requirements.txt"
	touch venv
