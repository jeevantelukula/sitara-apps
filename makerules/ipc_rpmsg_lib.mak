all: ipc_rpmsg_lib
clean: ipc_rpmsg_lib_clean

ipc_rpmsg_lib:
	$(MAKE) -C ./apps/common/ipc_rpmsg_lib

ipc_rpmsg_lib_clean:
	$(MAKE) -C ./apps/common/ipc_rpmsg_lib clean scrub
