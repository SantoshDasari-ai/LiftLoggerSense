idf_component_register(
    SRCS "main.cpp" "workout_app.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd esp_lvgl_port SensorLib
) 