all: benchmark_demo
clean: benchmark_demo_clean

benchmark_demo:
	$(MAKE) -C ./apps/benchmark_demo

benchmark_demo_clean:
	$(MAKE) -C ./apps/benchmark_demo clean scrub

