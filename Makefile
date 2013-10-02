#!/bin/make

# Author: Anne C. van Rossum
# Date: Mar. 27, 2013

RUR_BIN_PATH=/usr/bin
RUR_INCLUDE_PATH=/usr/include
RUR_SHARE_PATH=/usr/share/rur
RUR_TEMPLATE_PATH=$(RUR_SHARE_PATH)/templates
RUR_BACKENDS_PATH=$(RUR_SHARE_PATH)/backends
RUR_HELPER_PATH=$(RUR_SHARE_PATH)/helper
RUR_CONFIG_PATH=/etc/rur

BACKENDS_INSTALL_PATH:=$(DESTDIR)/$(RUR_BACKENDS_PATH)

all:
	@echo "[#] Just python and bash scripts. Nothing needs to be compiled"

install:
	@echo "[#] The installation will be in the path: $(RUR_SHARE_PATH)"
	@mkdir -p $(BACKENDS_INSTALL_PATH)/helper
	@cp backends/*.py $(BACKENDS_INSTALL_PATH)
	@cp backends/helper/*.py $(BACKENDS_INSTALL_PATH)/helper
	@install rur-builder $(DESTDIR)/${RUR_BIN_PATH}
	@cp third/zmq.hpp $(DESTDIR)/${RUR_INCLUDE_PATH}
	@mkdir -p $(RUR_HELPER_PATH)
	@cp helper/color.sh $(RUR_HELPER_PATH)

clean:
	@echo "[#] Just python and bash scripts. Just removing the .pyc files"
	@rm backends/*.pyc
	@rm backends/helper/*.pyc
	
uninstall:
	@echo "[#] Removing files from path: $(BACKENDS_INSTALL_PATH)"
	@rm $(BACKENDS_INSTALL_PATH)/helper/*.py
	@rm $(BACKENDS_INSTALL_PATH)/*.py
	@rmdir $(BACKENDS_INSTALL_PATH)/helper
	@rmdir $(BACKENDS_INSTALL_PATH)
	@install $(DESTDIR)/${RUR_BIN_PATH}/rur-builder 
	@rm $(DESTDIR)/${RUR_INCLUDE_PATH}/zmq.hpp
	@rm $(RUR_HELPER_PATH)/color.sh
	@rmdir $(RUR_HELPER_PATH)

