all: timesync_example
clean: timesync_example_clean

timesync_example:
	$(MAKE) -C ./examples/timesync

timesync_example_clean:
	$(MAKE) -C ./examples/timesync clean scrub

