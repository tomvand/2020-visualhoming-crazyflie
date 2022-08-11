# Variables

include config


# Commands

all: venv build
.PHONY: all

build: pprzlink
	cd app && $(MAKE) CRAZYFLIE_BASE=$(CRAZYFLIE_BASE) 
.PHONY: build

pprzlink: app/src/pprzlink/pprzlink.h
.PHONY: pprzlink

upload:
	cd app && $(MAKE) CRAZYFLIE_BASE=$(CRAZYFLIE_BASE) CLOAD_CMDS=$(CLOAD_CMDS) cload
.PHONY: upload

clean:
	rm -r app/src/pprzlink
	cd app && $(MAKE) CRAZYFLIE_BASE=$(CRAZYFLIE_BASE) clean
.PHONY: clean

freeze:
	pip freeze --local | grep -v "pkg-resources" | grep -v "pkg_resources" > requirements.txt
.PHONY: freeze


# Targets

venv:
	rm -rf venv
	virtualenv -p $(PYTHON) venv
	bash -c "source venv/bin/activate && pip install -r requirements.txt"
	touch venv

app/src/pprzlink/pprzlink.h: messages.xml
	ext/pprzlink/tools/generator/gen_messages.py --protocol 2.0 --lang C_standalone -o $@ messages.xml intermcu --opt VISUALHOMING_COMMAND,VISUALHOMING_STATE,VISUALHOMING_VECTOR,VISUALHOMING_MAP,VISUALHOMING_CAMERA,VISUALHOMING_INS_CORRECTION,IMCU_DEBUG
