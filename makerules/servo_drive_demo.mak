all: servo_drive_demo
clean: servo_drive_demo_clean

#set BUILD_ALL_TEST_TARGETS=1 to build all module levels tests like
# - MBX IPC standalone test
# - EtherCAT extended loop-back test  
# - Position Speed Control extended loop-back test  
export BUILD_ALL_TEST_TARGETS ?= 0

servo_drive_demo:
	$(MAKE) -C ./apps/servo_drive_demo

servo_drive_demo_clean:
	$(MAKE) -C ./apps/servo_drive_demo clean scrub

