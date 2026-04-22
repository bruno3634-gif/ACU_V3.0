#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    IDLE = 0,
    AS_ON,
    Start,
    EMERGENCY
} Main_state_machine_t;

typedef enum {
    Watchdog_check = 0,
    Pressure_check,
    HV_activation,
    Pressure_correlation_check,
    MB1_Check,
    MB2_Check,
    Monitor_sequence,
    Error_state,
    Finish,
    AS_Emergency
} startup_sequence_state_t;

typedef enum {
    AS_STATE_OFF = 0,
    AS_STATE_READY,
    AS_STATE_DRIVING
} as_state_t;

struct pressure_s {
    float Pneumatic;
    float Hydraulic;
};

struct car {
    int SDC_feedback;
    int HW_WDT_Enable;
    struct pressure_s Front_Pressure;
    struct pressure_s Rear_Pressure;
    int Ignition_Request;
    int Ignition_Status;
    int Solenoid1_Request;
    int Solenoid2_Request;
    as_state_t Autonomous_State;
};

#define TIMEOUT_SDC_MS        500
#define TIMEOUT_PRESSURE_MS   2000
#define TIMEOUT_HV_MS         5000
#define EBS_MIN_BAR           6.0f
#define EBS_MAX_BAR           10.0f
#define EBS_HYD_GAIN          10.0f
#define EBS_HYD_UNLOADED_BAR  1.0f

#define IN_RANGE(val, min, max) ((val) > (min) && (val) < (max))
#define IS_CORRELATED(hyd, pneu) ((hyd) > (EBS_HYD_GAIN * (pneu)))
#define IS_UNLOADED(hyd) ((hyd) < EBS_HYD_UNLOADED_BAR)

static uint32_t mock_millis = 0;
static uint32_t state_timer = 0;

uint32_t millis(void) {
    return mock_millis;
}

bool check_timeout(uint32_t start_time, uint32_t limit) {
    return (millis() - start_time) > limit;
}

void initial_sequence(struct car *t24,
                      startup_sequence_state_t *seq_status,
                      Main_state_machine_t *vehicle_state_machine) {
    switch (*seq_status) {
        case Watchdog_check:
            if (t24->SDC_feedback == 1) {
                t24->HW_WDT_Enable = 1;
                state_timer = millis();
                *seq_status = Pressure_check;
            } else if (check_timeout(state_timer, TIMEOUT_SDC_MS)) {
                *seq_status = Error_state;
            }
            break;

        case Pressure_check:
            if (IN_RANGE(t24->Front_Pressure.Pneumatic, EBS_MIN_BAR, EBS_MAX_BAR) &&
                IN_RANGE(t24->Rear_Pressure.Pneumatic, EBS_MIN_BAR, EBS_MAX_BAR)) {
                state_timer = millis();
                *seq_status = HV_activation;
            } else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
                *seq_status = Error_state;
            }
            break;

        case HV_activation:
            t24->Ignition_Request = 1;
            if (t24->Ignition_Status) {
                state_timer = millis();
                *seq_status = Pressure_correlation_check;
            } else if (check_timeout(state_timer, TIMEOUT_HV_MS)) {
                *seq_status = Error_state;
            }
            break;

        case Pressure_correlation_check:
            if (IS_CORRELATED(t24->Front_Pressure.Hydraulic, t24->Front_Pressure.Pneumatic) &&
                IS_CORRELATED(t24->Rear_Pressure.Hydraulic, t24->Rear_Pressure.Pneumatic)) {
                if (t24->Ignition_Status == 1) {
                    state_timer = millis();
                    *seq_status = MB1_Check;
                }
            } else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
                *seq_status = Error_state;
            }
            break;

        case MB1_Check:
            t24->Solenoid1_Request = 1;
            t24->Solenoid2_Request = 0;

            if (IS_CORRELATED(t24->Front_Pressure.Hydraulic, t24->Front_Pressure.Pneumatic) &&
                IS_UNLOADED(t24->Rear_Pressure.Hydraulic)) {
                state_timer = millis();
                *seq_status = MB2_Check;
            } else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
                *seq_status = Error_state;
            }
            break;

        case MB2_Check:
            t24->Solenoid1_Request = 0;
            t24->Solenoid2_Request = 1;

            if (IS_CORRELATED(t24->Rear_Pressure.Hydraulic, t24->Rear_Pressure.Pneumatic) &&
                IS_UNLOADED(t24->Front_Pressure.Hydraulic)) {
                t24->Autonomous_State = AS_STATE_READY;
                t24->Solenoid1_Request = 0;
                t24->Solenoid2_Request = 0;
            } else if (check_timeout(state_timer, TIMEOUT_PRESSURE_MS)) {
                *seq_status = Error_state;
            }
            break;

        case Error_state:
            *vehicle_state_machine = EMERGENCY;
            break;

        default:
            *seq_status = Error_state;
            break;
    }
}

const char *get_state_name(startup_sequence_state_t state) {
    switch (state) {
        case Watchdog_check: return "Watchdog_check";
        case Pressure_check: return "Pressure_check";
        case HV_activation: return "HV_activation";
        case Pressure_correlation_check: return "Pressure_correlation_check";
        case MB1_Check: return "MB1_Check";
        case MB2_Check: return "MB2_Check";
        case Error_state: return "Error_state";
        default: return "Unknown";
    }
}

void simulate_sensors(struct car *c,
                      startup_sequence_state_t state,
                      int scenario) {
    if (scenario != 1)
        c->SDC_feedback = 1;

    if (state >= Pressure_check) {
        if (scenario != 2) {
            c->Front_Pressure.Pneumatic = 8.0f;
            c->Rear_Pressure.Pneumatic = 8.0f;
        } else {
            c->Front_Pressure.Pneumatic = 4.0f;
        }
    }

    if (scenario != 3 && c->Ignition_Request)
        c->Ignition_Status = 1;

    if (scenario != 4 && state == Pressure_correlation_check) {
        c->Front_Pressure.Hydraulic = 85.0f;
        c->Rear_Pressure.Hydraulic = 85.0f;
    }

    if (state == MB1_Check && c->Solenoid1_Request) {
        if (scenario != 5) {
            c->Front_Pressure.Hydraulic = 85.0f;
            c->Rear_Pressure.Hydraulic = 0.5f;
        } else {
            c->Rear_Pressure.Hydraulic = 85.0f;
        }
    }

    if (state == MB2_Check && c->Solenoid2_Request) {
        c->Front_Pressure.Hydraulic = 0.5f;
        c->Rear_Pressure.Hydraulic = 85.0f;
    }
}

void run_test(const char *test_name,
              int scenario,
              startup_sequence_state_t expected_final_state,
              Main_state_machine_t expected_vehicle_state) {
    printf("==================================================\n");
    printf("TEST: %s\n", test_name);

    struct car t24 = {0};
    startup_sequence_state_t seq_status = Watchdog_check;
    Main_state_machine_t vehicle_state = IDLE;

    mock_millis = 0;
    state_timer = 0;

    for (int i = 0; i < 1500; i++) {
        simulate_sensors(&t24, seq_status, scenario);
        initial_sequence(&t24, &seq_status, &vehicle_state);

        if (vehicle_state == EMERGENCY ||
            t24.Autonomous_State == AS_STATE_READY)
            break;

        mock_millis += 10;
    }

    bool passed = (seq_status == expected_final_state) &&
                  (vehicle_state == expected_vehicle_state);

    printf("Final sequence state: %s\n", get_state_name(seq_status));
    printf("Final vehicle state : %s\n",
           vehicle_state == EMERGENCY ? "EMERGENCY" : "IDLE / OTHER");
    printf("RESULT: %s\n", passed ? "[PASS]" : "[FAIL]");
}

int main(void) {
    printf("==================================================\n");
    printf("\tAUTONOMOUS SEQUENCE TESTS\n");

    run_test("Scenario 0: Happy path", 0, MB2_Check, IDLE);
    run_test("Scenario 1: SDC timeout", 1, Error_state, EMERGENCY);
    run_test("Scenario 2: Pressure timeout", 2, Error_state, EMERGENCY);
    run_test("Scenario 3: Ignition timeout", 3, Error_state, EMERGENCY);
    run_test("Scenario 4: Correlation timeout", 4, Error_state, EMERGENCY);
    run_test("Scenario 5: MB1 failure", 5, Error_state, EMERGENCY);

    return 0;
}