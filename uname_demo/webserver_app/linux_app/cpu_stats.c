#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define HISTORY_SIZE 60
#define STATE_FILE "/tmp/cpu_stats_history.dat"

typedef struct {
    long long int prev_user, prev_nice, prev_system, prev_idle, prev_iowait, prev_irq, prev_softirq, prev_steal;
    double history[HISTORY_SIZE];
    int history_index;
    int history_count;
} CpuState;

// Function to read state from file
void read_state(CpuState *state) {
    FILE *fp = fopen(STATE_FILE, "rb");
    if (fp) {
        fread(state, sizeof(CpuState), 1, fp);
        fclose(fp);
    } else {
        // Initialize state if file doesn't exist
        memset(state, 0, sizeof(CpuState));
    }
}

// Function to write state to file
void write_state(const CpuState *state) {
    FILE *fp = fopen(STATE_FILE, "wb");
    if (fp) {
        fwrite(state, sizeof(CpuState), 1, fp);
        fclose(fp);
    } else {
        perror("Could not write state file");
    }
}

int main() {
    CpuState state;
    read_state(&state);

    long long int user, nice, system, idle, iowait, irq, softirq, steal;
    long long int total, prev_total;
    long long int idle_diff, total_diff;
    double current_cpu_usage;

    // Read current CPU times
    FILE *fp_stat = fopen("/proc/stat", "r");
    if (fp_stat == NULL) {
        perror("Error opening /proc/stat");
        return 1;
    }
    fscanf(fp_stat, "cpu %lld %lld %lld %lld %lld %lld %lld %lld", &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);
    fclose(fp_stat);

    total = user + nice + system + idle + iowait + irq + softirq + steal;
    prev_total = state.prev_user + state.prev_nice + state.prev_system + state.prev_idle + state.prev_iowait + state.prev_irq + state.prev_softirq + state.prev_steal;

    total_diff = total - prev_total;
    idle_diff = idle - state.prev_idle;

    if (total_diff > 0) {
        current_cpu_usage = (1.0 - (double)idle_diff / (double)total_diff) * 100.0;
    } else {
        current_cpu_usage = 0.0;
    }
    
    // Handle potential negative values if the counters wrapped around
    if (current_cpu_usage < 0) {
        current_cpu_usage = 0.0;
    }


    // Update history
    state.history[state.history_index] = current_cpu_usage;
    state.history_index = (state.history_index + 1) % HISTORY_SIZE;
    if (state.history_count < HISTORY_SIZE) {
        state.history_count++;
    }

    // Calculate avg and max
    double sum = 0;
    double max_cpu = 0;
    for (int i = 0; i < state.history_count; i++) {
        sum += state.history[i];
        if (state.history[i] > max_cpu) {
            max_cpu = state.history[i];
        }
    }
    double avg_cpu = (state.history_count > 0) ? (sum / state.history_count) : 0.0;

    // Update state for next run
    state.prev_user = user;
    state.prev_nice = nice;
    state.prev_system = system;
    state.prev_idle = idle;
    state.prev_iowait = iowait;
    state.prev_irq = irq;
    state.prev_softirq = softirq;
    state.prev_steal = steal;

    write_state(&state);

    // Print JSON output
    printf("Content-Type: application/json\n\n");
    printf("{\n");
    printf("  \"current_cpu_usage\": %.2f,\n", current_cpu_usage);
    printf("  \"average_cpu_usage\": %.2f,\n", avg_cpu);
    printf("  \"max_cpu_usage\": %.2f,\n", max_cpu);
    printf("  \"history\": [");
    for (int i = 0; i < state.history_count; i++) {
        // Print history in chronological order
        int index = (state.history_index - state.history_count + i + HISTORY_SIZE) % HISTORY_SIZE;
        printf("%.2f", state.history[index]);
        if (i < state.history_count - 1) {
            printf(", ");
        }
    }
    printf("]\n");
    printf("}\n");

    return 0;
}
