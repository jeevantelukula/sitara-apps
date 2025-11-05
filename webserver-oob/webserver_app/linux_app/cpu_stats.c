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

#define HISTORY_SIZE 60
#define STATE_FILE "/tmp/cpu_stats_history.dat"

typedef struct {
    /* Values from previous CPU time measurement */
    long long int prev_user, prev_nice, prev_system, prev_idle,
                 prev_iowait, prev_irq, prev_softirq, prev_steal;
    /* History for tracking CPU usage over time */
    double history[HISTORY_SIZE];
    int history_index;
    int history_count;
    double max_cpu_usage;
    double avg_cpu_usage;
} CpuState;

/* Function to read state from file */
void read_state(CpuState *state) {
    FILE *fp = fopen(STATE_FILE, "rb");
    if (fp) {
        fread(state, sizeof(CpuState), 1, fp);
        fclose(fp);
    } else {
        /* Initialize state if file doesn't exist */
        memset(state, 0, sizeof(CpuState));
    }
}

/* Function to write state to file */
void write_state(const CpuState *state) {
    FILE *fp = fopen(STATE_FILE, "wb");
    if (fp) {
        fwrite(state, sizeof(CpuState), 1, fp);
        fclose(fp);
    } else {
        perror("Could not write state file");
    }
}

/* Read CPU stats and calculate usage percentage */
double get_cpu_usage() {
    CpuState state;
    read_state(&state);

    long long int user, nice, system, idle, iowait, irq, softirq, steal;
    long long int total, prev_total;
    long long int idle_total, prev_idle_total;
    double cpu_percentage;

    /* Read current CPU times */
    FILE *fp = fopen("/proc/stat", "r");
    if (fp == NULL) {
        perror("Error opening /proc/stat");
        return -1;
    }

    /* Read the first line with overall CPU stats */
    fscanf(fp, "cpu %lld %lld %lld %lld %lld %lld %lld %lld",
           &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);
    fclose(fp);

    /* Calculate totals */
    idle_total = idle + iowait;
    prev_idle_total = state.prev_idle + state.prev_iowait;

    total = user + nice + system + idle + iowait + irq + softirq + steal;
    prev_total = state.prev_user + state.prev_nice + state.prev_system +
                 state.prev_idle + state.prev_iowait + state.prev_irq +
                 state.prev_softirq + state.prev_steal;

    /* Calculate differences */
    long long int total_diff = total - prev_total;
    long long int idle_diff = idle_total - prev_idle_total;

    /* Calculate CPU usage percentage */
    if (total_diff > 0) {
        cpu_percentage = ((total_diff - idle_diff) * 100.0) / total_diff;
    } else {
        /* No time has passed or counter wrapped, use previous value */
        cpu_percentage = 0.0;
        for (int i = 0; i < state.history_count; i++) {
            cpu_percentage += state.history[i];
        }
        cpu_percentage = (state.history_count > 0) ?
                         (cpu_percentage / state.history_count) : 0.0;
    }

    /* Ensure value is in valid range */
    if (cpu_percentage < 0) cpu_percentage = 0.0;
    if (cpu_percentage > 100) cpu_percentage = 100.0;

    /* Update history */
    state.history[state.history_index] = cpu_percentage;
    state.history_index = (state.history_index + 1) % HISTORY_SIZE;
    if (state.history_count < HISTORY_SIZE) {
        state.history_count++;
    }

    /* Calculate average and max */
    double sum = 0;
    double max_cpu = 0;
    for (int i = 0; i < state.history_count; i++) {
        sum += state.history[i];
        if (state.history[i] > max_cpu) {
            max_cpu = state.history[i];
        }
    }
    state.avg_cpu_usage = (state.history_count > 0) ?
                        (sum / state.history_count) : 0.0;
    state.max_cpu_usage = max_cpu;

    /* Update state for next run */
    state.prev_user = user;
    state.prev_nice = nice;
    state.prev_system = system;
    state.prev_idle = idle;
    state.prev_iowait = iowait;
    state.prev_irq = irq;
    state.prev_softirq = softirq;
    state.prev_steal = steal;

    /* Save state to file */
    write_state(&state);

    return cpu_percentage;
}

int main(int argc, char *argv[]) {
    // Get current CPU usage using our persistent state approach
    double current_cpu_usage = get_cpu_usage();
    CpuState state;
    read_state(&state); // Read current state to get history, avg, max

    // Basic mode: return just the current CPU load
    if (argc == 1) {
        printf("%.0f\n", current_cpu_usage);
        return 0;
    }

    // Enhanced mode: return more detailed CPU stats in JSON format
    if (argc > 1 && strcmp(argv[1], "enhanced") == 0) {
        // Construct JSON with current usage, average, max, and history
        printf("{\"current_cpu_usage\":%.1f,\"average_cpu_usage\":%.1f,\"max_cpu_usage\":%.1f,\"history\":[",
               current_cpu_usage, state.avg_cpu_usage, state.max_cpu_usage);

        // Add history array in chronological order
        for (int i = 0; i < state.history_count; i++) {
            int idx = (state.history_index - state.history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
            printf("%.1f%s", state.history[idx], (i < state.history_count - 1) ? "," : "");
        }

        printf("]}\n");
        return 0;
    }

    // Info mode: return CPU information in JSON format
    if (argc > 1 && strcmp(argv[1], "info") == 0) {
        FILE *cpu_info = fopen("/proc/cpuinfo", "r");
        if (cpu_info == NULL) {
            printf("{\"error\":\"Could not read CPU info\"}\n");
            return 1;
        }

        char line[256];
        char model_name[256] = "";
        int cpu_cores = 0;
        float cpu_mhz = 0.0;

        while (fgets(line, sizeof(line), cpu_info)) {
            if (strstr(line, "model name") != NULL || strstr(line, "Processor") != NULL) {
                char *value = strchr(line, ':');
                if (value) {
                    sscanf(value + 1, "%255[^\n]", model_name);
                    // Count cores
                    cpu_cores++;
                }
            }

            if (strstr(line, "cpu MHz") != NULL || strstr(line, "BogoMIPS") != NULL) {
                char *value = strchr(line, ':');
                if (value) {
                    sscanf(value + 1, "%f", &cpu_mhz);
                }
            }
        }
        fclose(cpu_info);

        // If no model name was found, provide a default for ARM systems
        if (strlen(model_name) == 0) {
            strcpy(model_name, "ARM Processor");
        }

        printf("{\"model\":\"%s\",\"cores\":%d,\"frequency\":\"%.2f MHz\",\"current_usage\":%.1f}\n",
               model_name, (cpu_cores > 0 ? cpu_cores : 1), cpu_mhz, current_cpu_usage);
        return 0;
    }

    return 0;
}
