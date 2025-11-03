INCLUDE=../../common/include
CFLAGS=-I${INCLUDE}

all: audio_utils

audio_utils: audio_utils.c
	$(CC) audio_utils.c -o audio_utils -pthread

clean:
	rm -f audio_utils
