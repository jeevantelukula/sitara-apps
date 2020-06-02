INCLUDE=../../common/include
CFLAGS=-I${INCLUDE}

all: rpmsg_json
rpmsg_json: rpmsg_json.o
	$(CC) $(CFLAGS) rpmsg_json.c -o rpmsg_json

clean:
	rm -f rpmsg_json.o rpmsg_json
