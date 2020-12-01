all: r5f_hello_world
clean: r5f_hello_world_clean

r5f_hello_world:
	$(MAKE) -C ./examples/r5f_hello_world

r5f_hello_world_clean:
	$(MAKE) -C ./examples/r5f_hello_world clean scrub

