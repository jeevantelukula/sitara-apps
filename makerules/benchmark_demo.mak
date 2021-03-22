
all: benchmark_demo_build
clean: benchmark_demo_clean

benchmark_demo_build:
	$(MAKE) -C ./benchmark_demo

benchmark_demo_clean:
	$(MAKE) -C ./benchmark_demo clean
