INCLUDE=../../common/include
CFLAGS=-I${INCLUDE} -I./ti-rpmsg-char/include

all: rpmsg_json
rpmsg_json: rpmsg_json.o
	$(CC) $(CFLAGS) rpmsg_json.c ./ti-rpmsg-char/src/rpmsg_char.c ./ti-rpmsg-char/src/soc.c ./ti-rpmsg-char/src/utils.c -o rpmsg_json

clean:
	rm -f rpmsg_json.o rpmsg_char.o soc.o utils.o rpmsg_json
