/*
 * Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

#define MAX_DEVICES 10

typedef struct {
    char display_name[128];
    char alsa_device[128];
} audio_device_info;

audio_device_info audio_devices[MAX_DEVICES];
int device_count = 0;
char *g_selected_device = NULL;
char fifo_path[256] = "/tmp/audio_classification_fifo";
volatile int running = 0;

// Get list of audio recording devices
char* get_arecord_devices() {
    FILE *fp;
    char path[1035];
    char *device_list = malloc(4096);

    if (!device_list) return NULL;
    device_list[0] = '\0';

    fp = popen("arecord -l", "r");
    if (fp == NULL) {
        fprintf(stderr, "Failed to run command: arecord -l\n");
        free(device_list);
        return NULL;
    }

    char current_card_name[256] = {0};
    int card_num = -1;
    device_count = 0;

    while (fgets(path, sizeof(path), fp) != NULL) {
        if (strncmp(path, "card", 4) == 0) {
            char *card_str = strstr(path, "card ");
            if (card_str) {
                sscanf(card_str, "card %d:", &card_num);
            }

            char *name_start = strchr(path, '[');
            char *name_end = strchr(path, ']');
            if (name_start && name_end && name_end > name_start && device_count < MAX_DEVICES) {
                int name_len = name_end - name_start - 1;
                if (name_len > 0 && name_len < sizeof(current_card_name)) {
                    strncpy(current_card_name, name_start + 1, name_len);
                    current_card_name[name_len] = '\0';

                    // Skip HDMI/playback-only devices
                    if (strstr(current_card_name, "HDMI") != NULL || strstr(current_card_name, "hdmi") != NULL || strstr(current_card_name, "cape") != NULL) {
                        fprintf(stderr, "Skipping playback-only device: %s (card %d)\n", current_card_name, card_num);
                        continue;
                    }

                    strncpy(audio_devices[device_count].display_name,
                            current_card_name,
                            sizeof(audio_devices[device_count].display_name) - 1);
                    audio_devices[device_count].display_name[sizeof(audio_devices[device_count].display_name) - 1] = '\0';

                    snprintf(audio_devices[device_count].alsa_device,
                             sizeof(audio_devices[device_count].alsa_device),
                             "plughw:%d,0", card_num);

                    if (strlen(device_list) + strlen(current_card_name) + 2 < 4095) {
                        if (device_count > 0) {
                            strcat(device_list, "\n");
                        }
                        strcat(device_list, current_card_name);
                    }

                    fprintf(stderr, "Found capture device: %s -> %s\n",
                           current_card_name, audio_devices[device_count].alsa_device);

                    device_count++;
                }
            }
        }
    }

    pclose(fp);
    return device_list;
}

// Placeholder for update_label_text - will be handled by WebSocket in JS
void update_label_text(const char* text) {
    // In a real scenario, this would send data over a socket or write to a shared memory.
    // For this demo, the GStreamer pipeline will write to a FIFO, and the JS server will read from it.
    fprintf(stderr, "Classification: %s\n", text);
}

void* gst_launch_thread(void *arg) {
    char buffer[128];

    unlink(fifo_path);
    mkfifo(fifo_path, 0666);

    char gst_command[2048]; // Increased buffer size for gst_command
    snprintf(gst_command, sizeof(gst_command),
             "gst-launch-1.0 alsasrc device=%s ! audioconvert ! audio/x-raw,format=S16LE,channels=1,rate=16000,layout=interleaved ! "
             " tensor_converter frames-per-tensor=3900 ! "
             "tensor_aggregator frames-in=3900 frames-out=15600 frames-flush=3900 frames-dim=1 ! "
             "tensor_transform mode=arithmetic option=typecast:float32,add:0.5,div:32767.5 ! "
             "tensor_transform mode=transpose option=1:0:2:3 ! "
             "queue leaky=2 max-size-buffers=10 ! "
             "tensor_filter framework=tensorflow2-lite model=/usr/share/oob-demo-assets/models/yamnet_audio_classification.tflite custom=Delegate:XNNPACK,NumThreads:2 ! "
             "tensor_decoder mode=image_labeling option1=/usr/share/oob-demo-assets/labels/yamnet_label_list.txt ! "
             "filesink buffer-mode=2 location=%s 1> /dev/null", g_selected_device ? g_selected_device : "plughw:1,0", fifo_path);

    fprintf(stderr, "Starting GStreamer with command: %s\n", gst_command);
    fprintf(stderr, "Starting GStreamer with device: %s\n", g_selected_device ? g_selected_device : "plughw:1,0");

    FILE *pipe = popen(gst_command, "r");

    if (!pipe) {
        perror("popen failed!");
        return NULL;
    }

    // The JS server will read from the FIFO, so this thread doesn't need to read from it.
    // This part of the code will be removed or adapted.
    /*
    int fifo_fd = open(fifo_path, O_RDONLY);
    if (fifo_fd == -1) {
        perror("open fifo failed!");
        pclose(pipe);
        return NULL;
    }

    // Read from the named pipe
    while (running && fgets(buffer, sizeof(buffer), fdopen(fifo_fd, "r")) != NULL) {
        buffer[strcspn(buffer, "$")] = 0;
        update_label_text(buffer);
    }

    close(fifo_fd);
    */

    // Keep the pipe open as long as running is true
    while (running) {
        sleep(1); // Sleep to prevent busy-waiting
    }

    pclose(pipe);
    unlink(fifo_path); // Remove the named pipe
    fprintf(stderr, "GStreamer pipeline stopped and FIFO unlinked.\n");
    return NULL;
}

// Main function for testing or direct execution
int main(int argc, char *argv[]) {
    if (argc > 1 && strcmp(argv[1], "devices") == 0) {
        char *devices = get_arecord_devices();
        if (devices) {
            // Print each audio device on a separate line for UI parsing
            for (int i = 0; i < device_count; i++) {
                printf("%s\n", audio_devices[i].display_name);
            }
            free(devices);
        } else {
            return 1;
        }
    } else if (argc > 1 && strcmp(argv[1], "start_gst") == 0) {
        if (argc > 2) {
            // Map display name to ALSA device
            g_selected_device = NULL;
            char *requested_device = argv[2];

            // First check if it's directly an ALSA device format (plughw:X,Y)
            if (strncmp(requested_device, "plughw:", 7) == 0) {
                g_selected_device = requested_device;
            } else {
                // Otherwise, try to match by display name
                for (int i = 0; i < device_count; i++) {
                    if (strcmp(audio_devices[i].display_name, requested_device) == 0) {
                        g_selected_device = audio_devices[i].alsa_device;
                        fprintf(stderr, "Mapped device '%s' to ALSA device: %s\n",
                               requested_device, g_selected_device);
                        break;
                    }
                }

                // If no match found, default to the first device if available
                if (!g_selected_device && device_count > 0) {
                    g_selected_device = audio_devices[0].alsa_device;
                    fprintf(stderr, "No exact match for '%s', using default device: %s\n",
                           requested_device, g_selected_device);
                }
            }

            // If still no device, use a default
            if (!g_selected_device) {
                g_selected_device = "plughw:0,0";
                fprintf(stderr, "Using fallback default device: %s\n", g_selected_device);
            }
        } else if (device_count > 0) {
            // No device specified, use the first available one
            g_selected_device = audio_devices[0].alsa_device;
            fprintf(stderr, "No device specified, using first device: %s\n", g_selected_device);
        }

        running = 1;
        pthread_t gst_thread;
        pthread_create(&gst_thread, NULL, gst_launch_thread, NULL);
        pthread_join(gst_thread, NULL);
    } else if (argc > 1 && strcmp(argv[1], "stop_gst") == 0) {
        running = 0;
    } else {
        printf("Usage:\n");
        printf("  %s devices        - List audio recording devices\n", argv[0]);
        printf("  %s start_gst [device] - Start GStreamer pipeline (e.g., plughw:1,0)\n", argv[0]);
        printf("  %s stop_gst       - Stop GStreamer pipeline\n", argv[0]);
        return 1;
    }
    return 0;
}
