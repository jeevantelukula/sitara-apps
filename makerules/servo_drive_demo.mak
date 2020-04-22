all: servo_drive_demo
clean: servo_drive_demo_clean

servo_drive_demo:
	$(MAKE) -C ./apps/servo_drive_demo

servo_drive_demo_clean:
	$(MAKE) -C ./apps/servo_drive_demo clean scrub

