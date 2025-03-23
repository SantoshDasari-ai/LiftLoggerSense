#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "lvgl.h"

/* Workout tracking enums and structs */
typedef enum {
    SCREEN_HOME,
    SCREEN_EXERCISE_SELECT,
    SCREEN_WORKOUT,
    SCREEN_SET_SUMMARY,
    SCREEN_WORKOUT_SUMMARY
} app_screen_t;

typedef enum {
    WORKOUT_STATE_IDLE,
    WORKOUT_STATE_SET_ACTIVE,
    WORKOUT_STATE_SET_DONE
} workout_state_t;

typedef struct {
    int set_number;
    int rep_count;
} set_data_t;

/* IMU data structure for rep detection */
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float temp;
} imu_data_t;

/* Function declarations */
void create_workout_app_ui(lv_disp_t *disp);
void update_rep_counter(int count);
bool detect_rep(imu_data_t *data);
void process_imu_data(imu_data_t *data);

/* Button control handlers */
void start_workout_handler(void);
void back_to_home_handler(void);
void select_bicep_curl_handler(void);
void start_set_handler(void);
void next_set_handler(void);
void finish_exercise_handler(void);
void new_workout_handler(void); 