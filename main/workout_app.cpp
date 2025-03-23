#include "workout_app.h"
#include <cstring>
#include <cmath>
#include "esp_log.h"

static const char *TAG = "WORKOUT_APP";

/* LVGL objects */
static lv_obj_t *home_screen;
static lv_obj_t *exercise_screen;
static lv_obj_t *workout_screen;
static lv_obj_t *set_summary_screen;
static lv_obj_t *workout_summary_screen;

/* Workout screen objects */
static lv_obj_t *workout_title;
static lv_obj_t *set_label;
static lv_obj_t *rep_counter;
static lv_obj_t *start_set_btn;
static lv_obj_t *finish_exercise_btn;

/* Workout tracking variables */
#define MAX_SETS 10

// Make these variables accessible from other files
app_screen_t current_screen = SCREEN_HOME;
workout_state_t workout_state = WORKOUT_STATE_IDLE;
static int current_set = 0;
static int current_rep_count = 0;
static set_data_t set_history[MAX_SETS];
static int total_sets = 0;
static int total_reps = 0;

/* Rep detection variables */
static float prev_acc_z = 0;
static float rep_threshold = 0.3f;  // Lower threshold for more sensitivity
static bool in_rep = false;
static int detection_cooldown = 0;  // Prevent multiple detections

// Debug variables
static float peak_value = 0;

/* Function declarations */
static void create_home_screen(void);
static void create_exercise_screen(void);
static void create_workout_screen(void);
static void create_set_summary_screen(void);
static void create_workout_summary_screen(void);

/* Button callbacks */
static void start_workout_cb(lv_event_t *e);
static void back_to_home_cb(lv_event_t *e);
static void select_bicep_curl_cb(lv_event_t *e);
static void start_set_cb(lv_event_t *e);
static void next_set_cb(lv_event_t *e);
static void finish_exercise_cb(lv_event_t *e);
static void new_workout_cb(lv_event_t *e);

/* External handler functions for button control */
void start_workout_handler(void)
{
    // Reset workout data
    current_set = 0;
    total_sets = 0;
    total_reps = 0;
    memset(set_history, 0, sizeof(set_history));
    
    // Switch to exercise selection screen
    current_screen = SCREEN_EXERCISE_SELECT;
    lv_scr_load(exercise_screen);
}

void back_to_home_handler(void)
{
    current_screen = SCREEN_HOME;
    lv_scr_load(home_screen);
}

void select_bicep_curl_handler(void)
{
    // Initialize for first set
    current_set = 1;
    current_rep_count = 0;
    workout_state = WORKOUT_STATE_IDLE;
    
    // Update set label
    lv_label_set_text_fmt(set_label, "Set: %d", current_set);
    
    // Reset rep counter
    lv_label_set_text(rep_counter, "0");
    
    // Update button text
    lv_obj_t *btn_label = lv_obj_get_child(start_set_btn, 0);
    lv_label_set_text(btn_label, "START SET");
    
    // Switch to workout screen
    current_screen = SCREEN_WORKOUT;
    lv_scr_load(workout_screen);
}

void start_set_handler(void)
{
    if (workout_state == WORKOUT_STATE_IDLE) {
        // Start the set
        workout_state = WORKOUT_STATE_SET_ACTIVE;
        current_rep_count = 0;
        
        // Update rep counter
        lv_label_set_text(rep_counter, "0");
        
        // Update button text
        lv_obj_t *btn_label = lv_obj_get_child(start_set_btn, 0);
        lv_label_set_text(btn_label, "SET DONE");
    } else if (workout_state == WORKOUT_STATE_SET_ACTIVE) {
        // End the set
        workout_state = WORKOUT_STATE_SET_DONE;
        
        // Save set data
        set_history[total_sets].set_number = current_set;
        set_history[total_sets].rep_count = current_rep_count;
        total_sets++;
        total_reps += current_rep_count;
        
        // Update set summary screen
        create_set_summary_screen();
        
        // Switch to set summary screen
        current_screen = SCREEN_SET_SUMMARY;
        lv_scr_load(set_summary_screen);
    }
}

void next_set_handler(void)
{
    // Increment set number
    current_set++;
    current_rep_count = 0;
    workout_state = WORKOUT_STATE_IDLE;
    
    // Update set label
    lv_label_set_text_fmt(set_label, "Set: %d", current_set);
    
    // Reset rep counter
    lv_label_set_text(rep_counter, "0");
    
    // Update button text
    lv_obj_t *btn_label = lv_obj_get_child(start_set_btn, 0);
    lv_label_set_text(btn_label, "START SET");
    
    // Switch to workout screen
    current_screen = SCREEN_WORKOUT;
    lv_scr_load(workout_screen);
}

void finish_exercise_handler(void)
{
    // If a set is active, save it first
    if (workout_state == WORKOUT_STATE_SET_ACTIVE) {
        workout_state = WORKOUT_STATE_SET_DONE;
        
        // Save set data
        set_history[total_sets].set_number = current_set;
        set_history[total_sets].rep_count = current_rep_count;
        total_sets++;
        total_reps += current_rep_count;
    }
    
    // Update workout summary screen
    create_workout_summary_screen();
    
    // Switch to workout summary screen
    current_screen = SCREEN_WORKOUT_SUMMARY;
    lv_scr_load(workout_summary_screen);
}

void new_workout_handler(void)
{
    // Reset workout data
    current_set = 0;
    total_sets = 0;
    total_reps = 0;
    memset(set_history, 0, sizeof(set_history));
    
    // Switch to home screen
    current_screen = SCREEN_HOME;
    lv_scr_load(home_screen);
}

/* Create the home screen */
static void create_home_screen(void)
{
    home_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(home_screen, LV_OBJ_FLAG_SCROLLABLE);
    
    // Create title with larger font
    lv_obj_t *title = lv_label_create(home_screen);
    lv_label_set_text(title, "Workout Rep & Set Tracker");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    
    // Create subtitle with proper spacing
    lv_obj_t *subtitle = lv_label_create(home_screen);
    lv_label_set_text(subtitle, "Track your bicep curl reps automatically");
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
    
    // Create start workout button with larger size
    lv_obj_t *start_btn = lv_btn_create(home_screen);
    lv_obj_set_size(start_btn, 220, 70);  // Even larger size for better touch target
    lv_obj_align(start_btn, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_event_cb(start_btn, start_workout_cb, LV_EVENT_CLICKED, NULL);
    
    // Add more distinguishable styling to button
    lv_obj_set_style_bg_color(start_btn, lv_color_hex(0x0D47A1), 0);  // Deep blue
    lv_obj_set_style_shadow_width(start_btn, 10, 0);
    lv_obj_set_style_shadow_color(start_btn, lv_color_hex(0x888888), 0);
    lv_obj_set_style_shadow_opa(start_btn, LV_OPA_50, 0);
    lv_obj_set_style_border_width(start_btn, 2, 0);
    lv_obj_set_style_border_color(start_btn, lv_color_hex(0x0A3D8C), 0);  // Darker blue for border
    lv_obj_set_style_radius(start_btn, 10, 0);  // Rounded corners
    
    lv_obj_t *btn_label = lv_label_create(start_btn);
    lv_label_set_text(btn_label, "START WORKOUT");
    lv_obj_center(btn_label);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), 0);  // White text
    
    // Create version label with proper spacing
    lv_obj_t *version = lv_label_create(home_screen);
    lv_label_set_text(version, "v1.0");
    lv_obj_align(version, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
    lv_obj_set_style_text_font(version, &lv_font_montserrat_14, 0);
}

/* Create the exercise selection screen */
static void create_exercise_screen(void)
{
    exercise_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(exercise_screen, LV_OBJ_FLAG_SCROLLABLE);
    
    // Create title with larger font
    lv_obj_t *title = lv_label_create(exercise_screen);
    lv_label_set_text(title, "Select Exercise");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    
    // Create bicep curl button with larger size
    lv_obj_t *bicep_btn = lv_btn_create(exercise_screen);
    lv_obj_set_size(bicep_btn, 220, 70);  // Even larger size for better touch target
    lv_obj_align(bicep_btn, LV_ALIGN_CENTER, 0, -20);
    lv_obj_add_event_cb(bicep_btn, select_bicep_curl_cb, LV_EVENT_CLICKED, NULL);
    
    // Add more distinguishable styling to button
    lv_obj_set_style_bg_color(bicep_btn, lv_color_hex(0x0D47A1), 0);  // Deep blue
    lv_obj_set_style_shadow_width(bicep_btn, 10, 0);
    lv_obj_set_style_shadow_color(bicep_btn, lv_color_hex(0x888888), 0);
    lv_obj_set_style_shadow_opa(bicep_btn, LV_OPA_50, 0);
    lv_obj_set_style_border_width(bicep_btn, 2, 0);
    lv_obj_set_style_border_color(bicep_btn, lv_color_hex(0x0A3D8C), 0);  // Darker blue for border
    lv_obj_set_style_radius(bicep_btn, 10, 0);  // Rounded corners
    
    lv_obj_t *btn_label = lv_label_create(bicep_btn);
    lv_label_set_text(btn_label, "BICEP CURL");
    lv_obj_center(btn_label);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(btn_label, lv_color_hex(0xFFFFFF), 0);  // White text
    
    // Create back button with proper spacing
    lv_obj_t *back_btn = lv_btn_create(exercise_screen);
    lv_obj_set_size(back_btn, 140, 60);  // Larger size for better touch target
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_LEFT, 15, -15);
    lv_obj_add_event_cb(back_btn, back_to_home_cb, LV_EVENT_CLICKED, NULL);
    
    // Back button styling
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x888888), 0);  // Gray
    lv_obj_set_style_shadow_width(back_btn, 8, 0);
    lv_obj_set_style_radius(back_btn, 8, 0);  // Rounded corners
    
    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "BACK");
    lv_obj_center(back_label);
    lv_obj_set_style_text_font(back_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), 0);  // White text
}

/* Create the workout screen */
static void create_workout_screen(void)
{
    workout_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(workout_screen, LV_OBJ_FLAG_SCROLLABLE);
    
    // Create title with larger font
    workout_title = lv_label_create(workout_screen);
    lv_label_set_text(workout_title, "Bicep Curl");
    lv_obj_align(workout_title, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_text_font(workout_title, &lv_font_montserrat_14, 0);
    
    // Create set label with proper spacing
    set_label = lv_label_create(workout_screen);
    lv_label_set_text_fmt(set_label, "Set: %d", current_set);
    lv_obj_align(set_label, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_text_font(set_label, &lv_font_montserrat_14, 0);
    
    // Create rep counter with larger font
    rep_counter = lv_label_create(workout_screen);
    lv_label_set_text(rep_counter, "0");
    lv_obj_align(rep_counter, LV_ALIGN_CENTER, 0, -30);
    lv_obj_set_style_text_font(rep_counter, &lv_font_montserrat_14, 0);
    
    // Create rep label with proper spacing
    lv_obj_t *rep_label = lv_label_create(workout_screen);
    lv_label_set_text(rep_label, "REPS");
    lv_obj_align(rep_label, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_text_font(rep_label, &lv_font_montserrat_14, 0);
    
    // Create start set button with proper spacing and improved styling
    start_set_btn = lv_btn_create(workout_screen);
    lv_obj_set_size(start_set_btn, 160, 65);  // Larger size for better touch target
    lv_obj_align(start_set_btn, LV_ALIGN_BOTTOM_LEFT, 20, -20);
    lv_obj_add_event_cb(start_set_btn, start_set_cb, LV_EVENT_CLICKED, NULL);
    
    // Improved button styling
    lv_obj_set_style_bg_color(start_set_btn, lv_color_hex(0x4CAF50), 0);  // Green
    lv_obj_set_style_shadow_width(start_set_btn, 8, 0);
    lv_obj_set_style_shadow_color(start_set_btn, lv_color_hex(0x888888), 0);
    lv_obj_set_style_shadow_opa(start_set_btn, LV_OPA_50, 0);
    lv_obj_set_style_border_width(start_set_btn, 2, 0);
    lv_obj_set_style_border_color(start_set_btn, lv_color_hex(0x3D9140), 0);  // Darker green for border
    lv_obj_set_style_radius(start_set_btn, 8, 0);  // Rounded corners
    
    lv_obj_t *start_label = lv_label_create(start_set_btn);
    lv_label_set_text(start_label, "START SET");
    lv_obj_center(start_label);
    lv_obj_set_style_text_font(start_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(start_label, lv_color_hex(0xFFFFFF), 0);  // White text
    
    // Create finish exercise button with proper spacing and improved styling
    finish_exercise_btn = lv_btn_create(workout_screen);
    lv_obj_set_size(finish_exercise_btn, 160, 65);  // Larger size for better touch target
    lv_obj_align(finish_exercise_btn, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_obj_add_event_cb(finish_exercise_btn, finish_exercise_cb, LV_EVENT_CLICKED, NULL);
    
    // Improved button styling
    lv_obj_set_style_bg_color(finish_exercise_btn, lv_color_hex(0xFF5722), 0);  // Orange
    lv_obj_set_style_shadow_width(finish_exercise_btn, 8, 0);
    lv_obj_set_style_shadow_color(finish_exercise_btn, lv_color_hex(0x888888), 0);
    lv_obj_set_style_shadow_opa(finish_exercise_btn, LV_OPA_50, 0);
    lv_obj_set_style_border_width(finish_exercise_btn, 2, 0);
    lv_obj_set_style_border_color(finish_exercise_btn, lv_color_hex(0xD84315), 0);  // Darker orange for border
    lv_obj_set_style_radius(finish_exercise_btn, 8, 0);  // Rounded corners
    
    lv_obj_t *finish_label = lv_label_create(finish_exercise_btn);
    lv_label_set_text(finish_label, "FINISH EXERCISE");
    lv_obj_center(finish_label);
    lv_obj_set_style_text_font(finish_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(finish_label, lv_color_hex(0xFFFFFF), 0);  // White text
}

/* Create the set summary screen */
static void create_set_summary_screen(void)
{
    // Create or recreate the screen
    if (set_summary_screen != NULL) {
        lv_obj_del(set_summary_screen);
    }
    
    set_summary_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(set_summary_screen, LV_OBJ_FLAG_SCROLLABLE);
    
    // Create title with larger font
    lv_obj_t *title = lv_label_create(set_summary_screen);
    lv_label_set_text(title, "Set Complete!");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    
    // Create set number label with proper spacing
    lv_obj_t *set_num = lv_label_create(set_summary_screen);
    lv_label_set_text_fmt(set_num, "Set %d", current_set);
    lv_obj_align(set_num, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_set_style_text_font(set_num, &lv_font_montserrat_14, 0);
    
    // Create rep count label with larger font
    lv_obj_t *reps = lv_label_create(set_summary_screen);
    lv_label_set_text_fmt(reps, "%d REPS", current_rep_count);
    lv_obj_align(reps, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(reps, &lv_font_montserrat_14, 0);
    
    // Create next set button with improved styling
    lv_obj_t *next_btn = lv_btn_create(set_summary_screen);
    lv_obj_set_size(next_btn, 180, 65);  // Larger size for better touch target
    lv_obj_align(next_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_add_event_cb(next_btn, next_set_cb, LV_EVENT_CLICKED, NULL);
    
    // Improved button styling
    lv_obj_set_style_bg_color(next_btn, lv_color_hex(0x2196F3), 0);  // Blue
    lv_obj_set_style_shadow_width(next_btn, 8, 0);
    lv_obj_set_style_shadow_color(next_btn, lv_color_hex(0x888888), 0);
    lv_obj_set_style_shadow_opa(next_btn, LV_OPA_50, 0);
    lv_obj_set_style_border_width(next_btn, 2, 0);
    lv_obj_set_style_border_color(next_btn, lv_color_hex(0x1976D2), 0);  // Darker blue for border
    lv_obj_set_style_radius(next_btn, 8, 0);  // Rounded corners
    
    lv_obj_t *next_label = lv_label_create(next_btn);
    lv_label_set_text(next_label, "NEXT SET");
    lv_obj_center(next_label);
    lv_obj_set_style_text_font(next_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(next_label, lv_color_hex(0xFFFFFF), 0);  // White text
}

/* Create the workout summary screen */
static void create_workout_summary_screen(void)
{
    // Create or recreate the screen
    if (workout_summary_screen != NULL) {
        lv_obj_del(workout_summary_screen);
    }
    
    workout_summary_screen = lv_obj_create(NULL);
    lv_obj_clear_flag(workout_summary_screen, LV_OBJ_FLAG_SCROLLABLE);
    
    // Create title with larger font
    lv_obj_t *title = lv_label_create(workout_summary_screen);
    lv_label_set_text(title, "Workout Complete!");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    
    // Create total sets label with proper spacing
    lv_obj_t *sets = lv_label_create(workout_summary_screen);
    lv_label_set_text_fmt(sets, "Total Sets: %d", total_sets);
    lv_obj_align(sets, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_set_style_text_font(sets, &lv_font_montserrat_14, 0);
    
    // Create total reps label with proper spacing
    lv_obj_t *reps = lv_label_create(workout_summary_screen);
    lv_label_set_text_fmt(reps, "Total Reps: %d", total_reps);
    lv_obj_align(reps, LV_ALIGN_TOP_MID, 0, 110);
    lv_obj_set_style_text_font(reps, &lv_font_montserrat_14, 0);
    
    // Create set details with proper spacing
    lv_obj_t *set_details = lv_label_create(workout_summary_screen);
    char details[256] = "";
    for (int i = 0; i < total_sets; i++) {
        char set_line[32];
        snprintf(set_line, sizeof(set_line), "Set %d: %d reps\n", i+1, set_history[i].rep_count);
        strcat(details, set_line);
    }
    lv_label_set_text(set_details, details);
    lv_obj_align(set_details, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(set_details, &lv_font_montserrat_14, 0);
    
    // Create new workout button with improved styling
    lv_obj_t *new_btn = lv_btn_create(workout_summary_screen);
    lv_obj_set_size(new_btn, 200, 65);  // Larger size for better touch target
    lv_obj_align(new_btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_add_event_cb(new_btn, new_workout_cb, LV_EVENT_CLICKED, NULL);
    
    // Improved button styling
    lv_obj_set_style_bg_color(new_btn, lv_color_hex(0x673AB7), 0);  // Purple
    lv_obj_set_style_shadow_width(new_btn, 8, 0);
    lv_obj_set_style_shadow_color(new_btn, lv_color_hex(0x888888), 0);
    lv_obj_set_style_shadow_opa(new_btn, LV_OPA_50, 0);
    lv_obj_set_style_border_width(new_btn, 2, 0);
    lv_obj_set_style_border_color(new_btn, lv_color_hex(0x512DA8), 0);  // Darker purple for border
    lv_obj_set_style_radius(new_btn, 8, 0);  // Rounded corners
    
    lv_obj_t *new_label = lv_label_create(new_btn);
    lv_label_set_text(new_label, "NEW WORKOUT");
    lv_obj_center(new_label);
    lv_obj_set_style_text_font(new_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(new_label, lv_color_hex(0xFFFFFF), 0);  // White text
}

/* Button callbacks */
static void start_workout_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Start workout button pressed");
    
    // Add visual feedback
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
    
    // Add small delay for visual feedback
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0D47A1), 0); // Reset color
    
    // Reset workout data
    current_set = 0;
    total_sets = 0;
    total_reps = 0;
    memset(set_history, 0, sizeof(set_history));
    
    // Switch to exercise selection screen
    current_screen = SCREEN_EXERCISE_SELECT;
    lv_scr_load(exercise_screen);
}

static void back_to_home_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Back button pressed");
    
    // Add visual feedback
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
    
    // Add small delay for visual feedback
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0D47A1), 0); // Reset color
    
    current_screen = SCREEN_HOME;
    lv_scr_load(home_screen);
}

static void select_bicep_curl_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Bicep curl button pressed");
    
    // Add visual feedback
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
    
    // Add small delay for visual feedback
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0D47A1), 0); // Reset color
    
    // Initialize for first set
    current_set = 1;
    current_rep_count = 0;
    workout_state = WORKOUT_STATE_IDLE;
    
    // Update set label
    lv_label_set_text_fmt(set_label, "Set: %d", current_set);
    
    // Reset rep counter
    lv_label_set_text(rep_counter, "0");
    
    // Update button text
    lv_obj_t *btn_label = lv_obj_get_child(start_set_btn, 0);
    lv_label_set_text(btn_label, "START SET");
    
    // Switch to workout screen
    current_screen = SCREEN_WORKOUT;
    lv_scr_load(workout_screen);
}

static void start_set_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Start/end set button pressed, state: %d", workout_state);
    
    // Add visual feedback
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
    
    // Add small delay for visual feedback
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0D47A1), 0); // Reset color
    
    if (workout_state == WORKOUT_STATE_IDLE) {
        // Start the set
        workout_state = WORKOUT_STATE_SET_ACTIVE;
        current_rep_count = 0;
        
        // Update rep counter
        lv_label_set_text(rep_counter, "0");
        
        // Update button text
        lv_obj_t *btn_label = lv_obj_get_child(start_set_btn, 0);
        lv_label_set_text(btn_label, "SET DONE");
    } else if (workout_state == WORKOUT_STATE_SET_ACTIVE) {
        // End the set
        workout_state = WORKOUT_STATE_SET_DONE;
        
        // Save set data
        set_history[total_sets].set_number = current_set;
        set_history[total_sets].rep_count = current_rep_count;
        total_sets++;
        total_reps += current_rep_count;
        
        // Update set summary screen
        create_set_summary_screen();
        
        // Switch to set summary screen
        current_screen = SCREEN_SET_SUMMARY;
        lv_scr_load(set_summary_screen);
    }
}

static void next_set_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Next set button pressed");
    
    // Add visual feedback
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
    
    // Add small delay for visual feedback
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0D47A1), 0); // Reset color
    
    // Increment set number
    current_set++;
    current_rep_count = 0;
    workout_state = WORKOUT_STATE_IDLE;
    
    // Update set label
    lv_label_set_text_fmt(set_label, "Set: %d", current_set);
    
    // Reset rep counter
    lv_label_set_text(rep_counter, "0");
    
    // Update button text
    lv_obj_t *btn_label = lv_obj_get_child(start_set_btn, 0);
    lv_label_set_text(btn_label, "START SET");
    
    // Switch to workout screen
    current_screen = SCREEN_WORKOUT;
    lv_scr_load(workout_screen);
}

static void finish_exercise_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Finish exercise button pressed");
    
    // Add visual feedback
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
    
    // Add small delay for visual feedback
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0D47A1), 0); // Reset color
    
    // If a set is active, save it first
    if (workout_state == WORKOUT_STATE_SET_ACTIVE) {
        workout_state = WORKOUT_STATE_SET_DONE;
        
        // Save set data
        set_history[total_sets].set_number = current_set;
        set_history[total_sets].rep_count = current_rep_count;
        total_sets++;
        total_reps += current_rep_count;
    }
    
    // Update workout summary screen
    create_workout_summary_screen();
    
    // Switch to workout summary screen
    current_screen = SCREEN_WORKOUT_SUMMARY;
    lv_scr_load(workout_summary_screen);
}

static void new_workout_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "New workout button pressed");
    
    // Add visual feedback
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
    
    // Add small delay for visual feedback
    vTaskDelay(pdMS_TO_TICKS(100));
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x0D47A1), 0); // Reset color
    
    // Reset workout data
    current_set = 0;
    total_sets = 0;
    total_reps = 0;
    memset(set_history, 0, sizeof(set_history));
    
    // Switch to home screen
    current_screen = SCREEN_HOME;
    lv_scr_load(home_screen);
}

/* Create UI for workout app */
void create_workout_app_ui(lv_disp_t *disp)
{
    // Create all screens
    create_home_screen();
    create_exercise_screen();
    create_workout_screen();
    
    // Load the home screen
    lv_scr_load(home_screen);
}

/* Update rep counter with animation */
void update_rep_counter(int count)
{
    if (workout_state == WORKOUT_STATE_SET_ACTIVE) {
        current_rep_count = count;
        
        // Update counter text
        lv_label_set_text_fmt(rep_counter, "%d", count);
        
        // Change color to green on count
        lv_obj_set_style_text_color(rep_counter, lv_color_hex(0x4CAF50), 0);
        
        // Schedule a timer to reset the color after 500ms
        lv_timer_create([](lv_timer_t *t) {
            lv_obj_set_style_text_color((lv_obj_t*)t->user_data, lv_color_hex(0x0D47A1), 0); // Back to blue
            lv_timer_del(t); // Delete the timer after use
        }, 500, rep_counter);
    }
}

/* Detect a rep from IMU data */
bool detect_rep(imu_data_t *data)
{
    // We'll use a combination of axes for better bicep curl detection
    // For bicep curls, multiple axes show significant changes
    
    // Calculate magnitude of acceleration (overall movement intensity)
    float acc_magnitude = sqrt(data->acc_x * data->acc_x + 
                              data->acc_y * data->acc_y + 
                              data->acc_z * data->acc_z);
    
    // Track z-axis which is most relevant for up/down motion
    float acc_z = data->acc_z;
    
    // Track peak values to help with debugging
    if (fabs(acc_z) > fabs(peak_value)) {
        peak_value = acc_z;
    }
    
    // Calculate delta from previous reading
    float delta = acc_z - prev_acc_z;
    
    // Store current value for next comparison
    prev_acc_z = acc_z;
    
    // Filter out extreme values that might be due to I2C errors
    if (fabs(acc_z) > 20.0f || fabs(acc_magnitude) > 20.0f) {
        ESP_LOGW(TAG, "Ignoring extreme acceleration value: Z=%.2f, Mag=%.2f", 
                acc_z, acc_magnitude);
        return false;
    }
    
    // Enhanced logging for debugging
    if (workout_state == WORKOUT_STATE_SET_ACTIVE) {
        // Log every 10th value to avoid flooding the log
        static int log_counter = 0;
        if (++log_counter % 10 == 0) {
            ESP_LOGI(TAG, "ACC_Z: %.2f, Mag: %.2f, Delta: %.2f, Peak: %.2f, In rep: %d", 
                    acc_z, acc_magnitude, delta, peak_value, in_rep);
        }
    }
    
    // Decrease cooldown counter if it's active
    if (detection_cooldown > 0) {
        detection_cooldown--;
        return false;
    }
    
    // Enhanced algorithm with hysteresis for bicep curl detection
    // This makes it more robust against noisy data
    
    // State machine approach for rep detection
    if (!in_rep) {
        // Looking for start of rep (downward motion)
        // We require a consistent negative acceleration beyond threshold
        static int down_count = 0;
        
        if (acc_z < -rep_threshold && acc_magnitude > 1.0f) {
            down_count++;
            if (down_count >= 2) { // Require multiple consistent readings
                in_rep = true;
                down_count = 0;
                ESP_LOGI(TAG, "Rep started! ACC_Z: %.2f, Mag: %.2f", acc_z, acc_magnitude);
            }
        } else {
            // Reset counter for inconsistent readings
            if (down_count > 0) down_count--;
        }
    } else {
        // Already in a rep, looking for completion (upward motion)
        static int up_count = 0;
        
        if (acc_z > rep_threshold && acc_magnitude > 1.0f) {
            up_count++;
            if (up_count >= 2) { // Require multiple consistent readings
                in_rep = false;
                up_count = 0;
                detection_cooldown = 15; // Prevent new rep detection for a few samples
                peak_value = 0; // Reset peak tracking
                ESP_LOGI(TAG, "Rep completed! ACC_Z: %.2f, Mag: %.2f", acc_z, acc_magnitude);
                return true; // Rep detected!
            }
        } else {
            // Reset counter for inconsistent readings
            if (up_count > 0) up_count--;
        }
        
        // Timeout mechanism - if in a rep for too long, reset state
        static int in_rep_counter = 0;
        in_rep_counter++;
        if (in_rep_counter > 100) { // Roughly 3-5 seconds depending on sampling rate
            ESP_LOGW(TAG, "Rep detection timeout - resetting state");
            in_rep = false;
            in_rep_counter = 0;
        }
    }
    
    return false;
}

/* Process IMU data for rep counting */
void process_imu_data(imu_data_t *data)
{
    if (workout_state == WORKOUT_STATE_SET_ACTIVE) {
        // Print all acceleration values for debugging
        ESP_LOGD(TAG, "ACC XYZ: %.2f, %.2f, %.2f", data->acc_x, data->acc_y, data->acc_z);
        
        if (detect_rep(data)) {
            // Increment rep count
            current_rep_count++;
            
            // Update UI
            update_rep_counter(current_rep_count);
            
            ESP_LOGI(TAG, "Rep counted! Total: %d", current_rep_count);
        }
    }
} 