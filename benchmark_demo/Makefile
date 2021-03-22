
# Get directory that contains this Makefile
DEMO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# User-supplied paths to dependencies
include $(MAKEENV)

all: rpmsg_json_app
clean: rpmsg_json_app_clean

.PHONY: rpmsg_json_app rpmsg_json_app_clean

RPMSG_JSON_APP_BUILDDIR = webserver_app/linux_app/

rpmsg_json_app:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR)

rpmsg_json_app_clean:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR) clean
