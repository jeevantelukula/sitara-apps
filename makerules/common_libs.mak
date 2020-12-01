all: common_libs
clean: common_libs_clean

common_libs:
	$(MAKE) -C ./common/libs

common_libs_clean:
	$(MAKE) -C ./common/libs clean scrub

