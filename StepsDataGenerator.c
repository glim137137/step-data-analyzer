#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// Global variables for stimulating acceleration data
typedef struct {
    double x;
    double y;
    double z;
} Acceleration;

typedef struct {
    double MOD;
    double FilteringSignal;
} ProcessedData;

typedef struct {
    double time;
    int cur_steps;
    int valid;
} StepRecords;

// Constant data
#define PI 3.1415926
#define SAMPLE_RATE 50
#define PERIOD 300
#define VALID_STEPS_THRESHOLD 8
const int SAMPLE_NUM = SAMPLE_RATE * PERIOD;
const int WINDOW_SIZE = 15;

// Function defined here
double generate_normal_data (double mean, double stddev);
void simulate_acceleration (Acceleration *acceleration, ProcessedData *processed);
void low_pass_filtering (ProcessedData *proce);
int is_peak (int index, ProcessedData *proce, int ws, char mode);
void detect_steps (ProcessedData *proce, StepRecords *steps);
void record_step_data (StepRecords *steps, int *step_count, int start, int end, int valid);
void user_interface (StepRecords *steps);
char menu();



int main () {
    // Initialize the random number generator
    srand(time(NULL));

    Acceleration AccelData[SAMPLE_NUM];
    ProcessedData ProceData[SAMPLE_NUM];
    StepRecords StepRecds[SAMPLE_NUM/10];

    // Simulate the acceleration data
    simulate_acceleration(AccelData, ProceData);

    // Generate averaged signal
    low_pass_filtering(ProceData);

    // Peak detection with window size of 15 (300ms)
    detect_steps(ProceData, StepRecds);

    // User interface
    user_interface(StepRecds);
    
}



// function to generate normally distributed data using Box-Muller transforms
double generate_normal_data (double mean, double stddev) {   
    double u1 = (double) rand()/RAND_MAX;
    double u2 = (double) rand()/RAND_MAX;  
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
    return mean + stddev * z;
}

// function to simulate the acceleration data and their the sum of mod
void simulate_acceleration (Acceleration *accel, ProcessedData *proce) {
    int i = 0;
    while (i < SAMPLE_NUM) {
        accel[i].x = generate_normal_data(1.200, 0.133);
        accel[i].y = generate_normal_data(0.500, 0.066);
        accel[i].z = generate_normal_data(0.300, 0.033);
        proce[i].MOD = fabs(accel[i].x) + fabs(accel[i].y) + fabs(accel[i].z);
        i++;
    }
}

// function to low pass filtering
void low_pass_filtering (ProcessedData *proce) {
    
    static double buffer[4] = {0.000, 0.000, 0.000, 0.000};
    static int buffer_index = 0;
    // double sum = 0.000;

    for (int i = 0; i < SAMPLE_NUM; i ++) {
        buffer[buffer_index] = proce[i].MOD;
        buffer_index = (buffer_index + 1) % 4;
        // Sum the values in the buffer
        double sum = 0.000;
        for (int j = 0; j < 4; j++) {
            sum += buffer[j];
        }
        proce[i].FilteringSignal = sum / 4.000;
    }
}

// function to check if it is a peak
int is_peak (int index, ProcessedData *proce, int ws, char mode) {
    int start = index - ws;
    int end = index + ws;
    if (start < 0) start = 0;
    if (end >= SAMPLE_NUM) end = SAMPLE_NUM - 1;

    double peak_value = proce[index].FilteringSignal;

    // Find max or min value in the given window
    for (int i = start; i <= end; i++) {
        if ((mode == 'm' && proce[i].FilteringSignal > peak_value) ||
            (mode == 'n' && proce[i].FilteringSignal < peak_value)) {
            return 0; // Not a peak
        }
    }

    return 1;
}

// function to detect valid steps
void detect_steps (ProcessedData *proce, StepRecords *steps) {
    double threshold_buffer[4] = {0.000, 0.000, 0.000, 0.000};
    double dynamic_threshold = 0.000;
    const double sensitivity = 0.100;
    int consecutive_steps = 0;
    int regular_mode_on = 0;
    int total_steps = 0;
    int step_count = 0;
    int threshold_index = 0;


    for (int i = 0; i < SAMPLE_NUM; i++) {
        // If it is a max peak, then continue to seek for the min peak in 1s
        if (is_peak(i, proce, WINDOW_SIZE / 2, 'm')) {
            int min_peak_found = 0;
            // After a maximum peak is detected, searche for a minimum for up to 1 sec
            for (int j = i; j < i + SAMPLE_RATE && j < SAMPLE_NUM; j++) {
                if (is_peak(j, proce, WINDOW_SIZE / 2, 'n')) {
                    // min peak is found
                    min_peak_found = 1;
                    // update dynamic threshold
                    double max_peak = proce[i].FilteringSignal;
                    double min_peak = proce[j].FilteringSignal;
                    
                    double diff = max_peak - min_peak;
                    if (diff > sensitivity) {
                        threshold_buffer[threshold_index] = (max_peak + min_peak) / 2;
                        dynamic_threshold = 0.000;
                        for (int k = 0; k < 4; k++) {
                            dynamic_threshold += threshold_buffer[k];
                        }
                        threshold_index = (threshold_index + 1) % 4;
                        dynamic_threshold /= 4.0;
                    }
                    
                    // check if it is a valid step
                    if (max_peak > dynamic_threshold + sensitivity / 2 && min_peak < dynamic_threshold - sensitivity / 2) {
                        if (regular_mode_on) {
                            total_steps += 1;
                            record_step_data(steps, &step_count, i, j, 1);
                        } else {
                            consecutive_steps++;
                            record_step_data(steps, &step_count, i, j, 1);
                            if (consecutive_steps > VALID_STEPS_THRESHOLD) {
                                total_steps += 8;
                                regular_mode_on = 1;
                            }
                        }
                    } else {
                        consecutive_steps = 0;
                        regular_mode_on = 0;
                    }
                    break;
                }
            }
            // regular mode off
            if (!min_peak_found) {
                consecutive_steps = 0;
                regular_mode_on = 0;
            }     
        }    
    }
    record_step_data(steps, &step_count, 0, 0, 0);
}
    
// function to record step data
void record_step_data (StepRecords *steps, int *step_count, int start, int end, int valid) {
    steps[*step_count].time = (start + end) / 2.0 * (1.0 / SAMPLE_RATE);
    steps[*step_count].valid = valid;
    if (valid) ++(*step_count);
    steps[*step_count].cur_steps = *step_count;
}

// function for user interface
void user_interface (StepRecords *steps) {
    int isQuit = 0;
    while (!isQuit)
    {
        switch (menu()) {
            // A: Display the total number of steps     
            case 'A':
                int total;
                for (int i = 0; steps[i].valid; i++) {
                    total = i;
                }
                printf("Total Steps Detected: %d\n", total + 1);
                printf("------------------------\n");
                break;
            // B: Display the all steps time
            case 'B':
                for (int i = 0; steps[i].valid; i++) {
                    printf("Step %d Detected At: %.3f second\n", i+1 ,steps[i].time);
                }
                printf("--------------------------------\n");
                break;
            // C: Download Steps Record
            case 'C':
                FILE *file = fopen("Steps_Record.csv", "w");
                char head[50] = "Step,Time\n";
                fputs(head, file);
                for (int i = 0; steps[i].valid; i++) {
                    char data[50];
                    sprintf(data, "%d,%.3fs\n", i+1 ,steps[i].time);
                    fputs(data, file);
                }
                fclose(file);
                printf("Data sorted and written to Steps_Record.csv\n");
                printf("-------------------------------------------\n");
                break;
            // Q: Quit
            case 'Q':
                isQuit = 1;
                break;
            default:
                printf("Invalid choice. Try again.\n");
                break;
        }
    }
    
}

// function for menu
char menu() {
    printf(
        "Menu Options:\n"
        "A: Display the total number of steps\n"
        "B: Display the steps time\n"
        "C: Download Steps Record\n"
        "Q: Quit\n"
        "Enter choice: "
        );
        char option;
        scanf(" %c", &option);
        return option;
}
