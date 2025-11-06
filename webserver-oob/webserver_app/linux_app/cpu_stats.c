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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>  /* For usleep() */
#include <time.h>    /* For nanosleep() */

#define HISTORY_SIZE 300  /* Store 5 minutes of history (assuming 1 second per sample) */
#define SAMPLE_COUNT 3  /* Number of samples to take for one measurement */
#define SAMPLE_INTERVAL_MS 200  /* Time between samples in milliseconds */

/* Structure to hold CPU time stats */
typedef struct {
    long long int user;
    long long int nice;
    long long int system;
    long long int idle;
    long long int iowait;
    long long int irq;
    long long int softirq;
    long long int steal;
    long long int total;
    long long int idle_total;
} CpuTimes;

/* History tracking structure */
typedef struct {
    double history[HISTORY_SIZE];
    int history_index;
    int history_count;
    double max_cpu_usage;
    double avg_cpu_usage;
    time_t last_update;
} CpuHistory;

/* Global history state - safer than using file-based state which can be corrupted */
static CpuHistory cpu_history = {{0}, 0, 0, 0.0, 0.0, 0};

/* Sleep for specified milliseconds */
void sleep_ms(int milliseconds) {
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

/* Read CPU times from /proc/stat */
int read_cpu_times(CpuTimes *times) {
    FILE *fp = fopen("/proc/stat", "r");
    if (fp == NULL) {
        perror("Error opening /proc/stat");
        return 0;
    }

    /* Read the first line with overall CPU stats */
    if (fscanf(fp, "cpu %lld %lld %lld %lld %lld %lld %lld %lld",
               &times->user, &times->nice, &times->system, &times->idle,
               &times->iowait, &times->irq, &times->softirq, &times->steal) != 8) {
        fclose(fp);
        return 0;
    }

    fclose(fp);

    /* Calculate totals */
    times->idle_total = times->idle + times->iowait;
    times->total = times->user + times->nice + times->system + times->idle +
                   times->iowait + times->irq + times->softirq + times->steal;

    return 1;
}

/* Get CPU usage by taking multiple samples and averaging */
double get_cpu_usage() {
    CpuTimes samples[SAMPLE_COUNT+1];  /* +1 for the initial reading */
    double cpu_percentage_sum = 0.0;
    time_t now = time(NULL);
    int valid_samples = 0;

    /* Take initial sample */
    if (!read_cpu_times(&samples[0])) {
        return 0.0;
    }

    /* Take SAMPLE_COUNT additional samples and calculate average */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        /* Wait for a short period */
        sleep_ms(SAMPLE_INTERVAL_MS);

        /* Take next sample */
        if (!read_cpu_times(&samples[i+1])) {
            continue; /* Skip this sample if read fails */
        }

        /* Calculate differences between this sample and previous */
        long long int total_diff = samples[i+1].total - samples[i].total;
        long long int idle_diff = samples[i+1].idle_total - samples[i].idle_total;

        /* Calculate CPU usage percentage for this interval */
        if (total_diff > 0) {
            double interval_percentage = ((total_diff - idle_diff) * 100.0) / total_diff;

            /* Skip samples that seem too high (likely avahi-daemon spikes) */
            if (interval_percentage > 95.0 && valid_samples > 0) {
                /* Skip this suspiciously high value */
                continue;
            }

            /* Ensure value is in valid range */
            if (interval_percentage < 0) interval_percentage = 0.0;
            if (interval_percentage > 100) interval_percentage = 100.0;

            cpu_percentage_sum += interval_percentage;
            valid_samples++;
        }
    }

    /* Calculate the average CPU percentage from the valid samples */
    double cpu_percentage = (valid_samples > 0) ? (cpu_percentage_sum / valid_samples) : 0.0;

    /* Only update history if enough time has passed (avoid too frequent updates) */
    if (now - cpu_history.last_update >= 1) {
        /* Update history */
        cpu_history.history[cpu_history.history_index] = cpu_percentage;
        cpu_history.history_index = (cpu_history.history_index + 1) % HISTORY_SIZE;
        if (cpu_history.history_count < HISTORY_SIZE) {
            cpu_history.history_count++;
        }

        /* Calculate average and max */
        double sum = 0;
        double max_cpu = 0;
        for (int i = 0; i < cpu_history.history_count; i++) {
            sum += cpu_history.history[i];
            if (cpu_history.history[i] > max_cpu) {
                max_cpu = cpu_history.history[i];
            }
        }
        cpu_history.avg_cpu_usage = (cpu_history.history_count > 0) ?
                               (sum / cpu_history.history_count) : 0.0;

        if (max_cpu > cpu_history.max_cpu_usage) {
            cpu_history.max_cpu_usage = max_cpu;
        }

        cpu_history.last_update = now;
    }

    return cpu_percentage;
}

int main(int argc, char *argv[]) {
    // Get current CPU usage - now takes measurements with a short delay
    double current_cpu_usage = get_cpu_usage();

    // Basic mode: return just the current CPU load
    if (argc == 1) {
        printf("%.0f\n", current_cpu_usage);
        return 0;
    }

    // Enhanced mode: return more detailed CPU stats in JSON format
    if (argc > 1 && strcmp(argv[1], "enhanced") == 0) {
        // Construct JSON with current usage, average, max, and history
        printf("{\"current_cpu_usage\":%.1f,\"average_cpu_usage\":%.1f,\"max_cpu_usage\":%.1f,\"history\":[",
               current_cpu_usage, cpu_history.avg_cpu_usage, cpu_history.max_cpu_usage);

        // Add history array in chronological order
        for (int i = 0; i < cpu_history.history_count; i++) {
            int idx = (cpu_history.history_index - cpu_history.history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
            printf("%.1f%s", cpu_history.history[idx], (i < cpu_history.history_count - 1) ? "," : "");
        }

        printf("]}\n");
        return 0;
    }

    // Info mode: return CPU information in JSON format
    if (argc > 1 && strcmp(argv[1], "info") == 0) {
        // Use lscpu command instead of reading /proc/cpuinfo directly
        FILE *lscpu_output = popen("lscpu", "r");
        if (lscpu_output == NULL) {
            printf("{\"error\":\"Could not run lscpu command\"}\n");
            return 1;
        }

        char line[256];
        char architecture[64] = "Unknown";
        char vendor_id[64] = "Unknown";
        char model_name[256] = "Unknown";

        while (fgets(line, sizeof(line), lscpu_output)) {
            // Extract Architecture
            if (strncmp(line, "Architecture:", 13) == 0) {
                char *value = line + 13;
                while (*value == ' ' || *value == '\t') value++; // Skip whitespace
                sscanf(value, "%63[^\n]", architecture);
            }
            // Extract Vendor ID
            else if (strncmp(line, "Vendor ID:", 10) == 0) {
                char *value = line + 10;
                while (*value == ' ' || *value == '\t') value++; // Skip whitespace
                sscanf(value, "%63[^\n]", vendor_id);
            }
            // Extract Model name
            else if (strncmp(line, "Model name:", 11) == 0) {
                char *value = line + 11;
                while (*value == ' ' || *value == '\t') value++; // Skip whitespace
                sscanf(value, "%255[^\n]", model_name);
            }
        }
        pclose(lscpu_output);

        // Return the CPU information in JSON format
        printf("{\"architecture\":\"%s\",\"vendor\":\"%s\",\"model\":\"%s\",\"current_usage\":%.1f}\n",
               architecture, vendor_id, model_name, current_cpu_usage);
        return 0;
    }

    return 0;
}
