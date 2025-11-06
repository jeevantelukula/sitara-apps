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
#include <math.h>    /* For fabs() */

#define HISTORY_SIZE 300        /* Store 5 minutes of history (assuming 1 second per sample) */
#define SAMPLE_COUNT 5          /* Number of samples to take for one measurement */
#define SAMPLE_INTERVAL_MS 200  /* Time between samples in milliseconds */
#define SPIKE_THRESHOLD 30.0    /* Percentage change to consider a spike */
#define EMA_ALPHA 0.3           /* Weight for current sample in exponential moving average */

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
    long long int active;       /* Non-idle time */
} CpuTimes;

/* History tracking structure */
typedef struct {
    double history[HISTORY_SIZE];
    int history_index;
    int history_count;
    double max_cpu_usage;
    double avg_cpu_usage;
    time_t last_update;
    double last_reading;        /* Last valid CPU percentage for spike detection */
    double ema;                 /* Exponential Moving Average for smoothing */
    time_t start_time;          /* When the program first started */
} CpuHistory;

/* Global history state */
static CpuHistory cpu_history = {{0}, 0, 0, 0.0, 0.0, 0, 0.0, 0.0, 0};

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
    times->active = times->user + times->nice + times->system +
                   times->irq + times->softirq + times->steal;
    times->total = times->active + times->idle_total;

    return 1;
}

/* This section previously contained process-specific detection code that was removed
 * to make the CPU measurement more generic and robust without relying on
 * specific process names.
 */

/* Calculate CPU usage with multiple samples and intelligent filtering */
double get_cpu_usage() {
    CpuTimes samples[SAMPLE_COUNT+1];
    double measurements[SAMPLE_COUNT];
    int valid_measurements = 0;
    double cpu_percentage = 0.0;
    time_t now = time(NULL);

    /* Take initial sample */
    if (!read_cpu_times(&samples[0])) {
        return 0.0;
    }

    /* Take multiple samples */
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sleep_ms(SAMPLE_INTERVAL_MS);

        if (!read_cpu_times(&samples[i+1])) {
            continue;
        }

        /* Calculate differences */
        long long int total_diff = samples[i+1].total - samples[i].total;
        long long int idle_diff = samples[i+1].idle_total - samples[i].idle_total;

        if (total_diff > 0) {
            double interval_percentage = ((total_diff - idle_diff) * 100.0) / total_diff;

            /* Basic validity checks */
            if (interval_percentage < 0) interval_percentage = 0.0;
            if (interval_percentage > 100) interval_percentage = 100.0;

            /* Store this measurement */
            measurements[valid_measurements++] = interval_percentage;
        }
    }

    if (valid_measurements == 0) {
        return cpu_history.last_reading; /* Return last valid reading if no valid samples */
    }

    /* Sort measurements to find median */
    for (int i = 0; i < valid_measurements-1; i++) {
        for (int j = 0; j < valid_measurements-i-1; j++) {
            if (measurements[j] > measurements[j+1]) {
                double temp = measurements[j];
                measurements[j] = measurements[j+1];
                measurements[j+1] = temp;
            }
        }
    }

    /* Get median value (more robust than average) */
    double median = measurements[valid_measurements / 2];

    /* Calculate standard deviation to detect outliers */
    double sum = 0.0, sum_sq = 0.0, stddev = 0.0;

    for (int i = 0; i < valid_measurements; i++) {
        sum += measurements[i];
        sum_sq += measurements[i] * measurements[i];
    }

    double mean = sum / valid_measurements;

    /* Only calculate stddev if we have multiple measurements */
    if (valid_measurements > 1) {
        stddev = sqrt((sum_sq / valid_measurements) - (mean * mean));
    }

    /* Generic spike detection using statistical methods and history */
    int is_spike = 0;

    /* Method 1: Detect sudden large changes from previous value */
    if (cpu_history.history_count > 0 &&
        fabs(median - cpu_history.last_reading) > SPIKE_THRESHOLD) {
        is_spike = 1;
    }

    /* Method 2: Detect if value is far from the mean (>2 standard deviations) */
    if (valid_measurements > 2 && stddev > 0 &&
        fabs(median - mean) > (2 * stddev)) {
        is_spike = 1;
    }

    /* Method 3: Detect if value is very high and very different from recent values */
    if (cpu_history.history_count > 5 && median > 90.0 &&
        cpu_history.avg_cpu_usage < 70.0) {
        is_spike = 1;
    }

    /* If we detect a spike, use a more stable value instead */
    if (is_spike) {
        /* Use previous reading or mean of measurements (excluding highest value) */
        if (cpu_history.history_count > 0) {
            cpu_percentage = cpu_history.last_reading;
        } else if (valid_measurements > 1) {
            /* Calculate mean excluding highest value */
            double sum_without_highest = sum - measurements[valid_measurements-1];
            cpu_percentage = sum_without_highest / (valid_measurements - 1);
        } else {
            cpu_percentage = median; /* Fall back to median if no better option */
        }
    } else {
        /* No spike detected, use median */
        cpu_percentage = median;
    }

    /* Apply exponential moving average for smoothing */
    if (cpu_history.history_count > 0) {
        cpu_history.ema = (EMA_ALPHA * cpu_percentage) +
                          ((1.0 - EMA_ALPHA) * cpu_history.ema);
        cpu_percentage = cpu_history.ema;
    } else {
        cpu_history.ema = cpu_percentage;
    }

    /* Initialize start_time if this is the first run */
    if (cpu_history.start_time == 0) {
        cpu_history.start_time = now;
    }

    /* Update history if enough time has passed */
    if (now - cpu_history.last_update >= 1) {
        /* Save this reading */
        cpu_history.history[cpu_history.history_index] = cpu_percentage;
        cpu_history.history_index = (cpu_history.history_index + 1) % HISTORY_SIZE;
        if (cpu_history.history_count < HISTORY_SIZE) {
            cpu_history.history_count++;
        }

        /* Update stats with a different approach for average and max */
        double sum = 0;
        double current_max = 0;  // Track max in current window only

        /* Calculate statistics from stored history */
        for (int i = 0; i < cpu_history.history_count; i++) {
            sum += cpu_history.history[i];

            /* Find maximum in current window (not persistent) */
            if (cpu_history.history[i] > current_max) {
                current_max = cpu_history.history[i];
            }
        }

        /* Calculate running average based on history window */
        double new_avg = (cpu_history.history_count > 0) ?
                        (sum / cpu_history.history_count) : 0.0;

        /* Apply gentle smoothing to average */
        if (cpu_history.history_count > 1) {
            cpu_history.avg_cpu_usage = (0.9 * new_avg) + (0.1 * cpu_history.avg_cpu_usage);
        } else {
            cpu_history.avg_cpu_usage = new_avg;
        }

        /* For max CPU usage, use the greater of:
         * 1. Current window maximum
         * 2. Existing max value, but decay it slightly over time
         *    (about 10% reduction per hour so it's not permanent)
         */
        double time_elapsed = difftime(now, cpu_history.start_time);
        double decay_factor = 1.0;

        /* Apply decay to max after 10 minutes, max decay rate about 10% per hour */
        if (time_elapsed > 600) {  // 10 minutes
            decay_factor = 1.0 - ((time_elapsed - 600) / 36000.0);
            if (decay_factor < 0.5) decay_factor = 0.5;  // Never decay below 50%
        }

        /* Apply decay to previous max */
        double decayed_max = cpu_history.max_cpu_usage * decay_factor;

        /* Use greater of current window max and decayed historical max */
        if (current_max > decayed_max) {
            cpu_history.max_cpu_usage = current_max;
        } else {
            cpu_history.max_cpu_usage = decayed_max;
        }

        cpu_history.last_update = now;
    }

    /* Store current reading for next comparison */
    cpu_history.last_reading = cpu_percentage;

    return cpu_percentage;
}

int main(int argc, char *argv[]) {
    /* Get current CPU usage with robust measurement */
    double current_cpu_usage = get_cpu_usage();

    /* Basic mode: return just the current CPU load */
    if (argc == 1) {
        printf("%.0f\n", current_cpu_usage);
        return 0;
    }

    /* Enhanced mode: return detailed CPU stats in JSON format */
    if (argc > 1 && strcmp(argv[1], "enhanced") == 0) {
        /* Construct JSON with current usage, average, max, and history */
        printf("{\"current_cpu_usage\":%.1f,\"average_cpu_usage\":%.1f,\"max_cpu_usage\":%.1f,\"history\":[",
               current_cpu_usage, cpu_history.avg_cpu_usage, cpu_history.max_cpu_usage);

        /* Add history array in chronological order */
        for (int i = 0; i < cpu_history.history_count; i++) {
            int idx = (cpu_history.history_index - cpu_history.history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
            printf("%.1f%s", cpu_history.history[idx], (i < cpu_history.history_count - 1) ? "," : "");
        }

        printf("]}\n");
        return 0;
    }

    /* Info mode: return CPU information in JSON format */
    if (argc > 1 && strcmp(argv[1], "info") == 0) {
        /* Use lscpu command for CPU information */
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
            /* Extract Architecture */
            if (strncmp(line, "Architecture:", 13) == 0) {
                char *value = line + 13;
                while (*value == ' ' || *value == '\t') value++; /* Skip whitespace */
                sscanf(value, "%63[^\n]", architecture);
            }
            /* Extract Vendor ID */
            else if (strncmp(line, "Vendor ID:", 10) == 0) {
                char *value = line + 10;
                while (*value == ' ' || *value == '\t') value++; /* Skip whitespace */
                sscanf(value, "%63[^\n]", vendor_id);
            }
            /* Extract Model name */
            else if (strncmp(line, "Model name:", 11) == 0) {
                char *value = line + 11;
                while (*value == ' ' || *value == '\t') value++; /* Skip whitespace */
                sscanf(value, "%255[^\n]", model_name);
            }
        }
        pclose(lscpu_output);

        /* Return CPU information in JSON format */
        printf("{\"architecture\":\"%s\",\"vendor\":\"%s\",\"model\":\"%s\"}\n",
               architecture, vendor_id, model_name);
        return 0;
    }

    /* Note: Process list functionality was removed to keep the tool generic */

    return 0;
}
