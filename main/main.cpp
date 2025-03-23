/*
 * IMU Sensor Display with Touch Interface
 * Combines LCD display with QMI8658C IMU sensor
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_check.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "../components/SensorLib/src/SensorQMI8658.hpp"
#include "../components/SensorLib/src/SensorLib.h"
#include "workout_app.h"

/* LCD size */
#define EXAMPLE_LCD_H_RES (240)
#define EXAMPLE_LCD_V_RES (280)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM (SPI2_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS (8)
#define EXAMPLE_LCD_PARAM_BITS (8)
#define EXAMPLE_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_RGB)
#define EXAMPLE_LCD_BITS_PER_PIXEL (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)
#define EXAMPLE_LCD_BL_ON_LEVEL (1)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK (GPIO_NUM_6)
#define EXAMPLE_LCD_GPIO_MOSI (GPIO_NUM_7)
#define EXAMPLE_LCD_GPIO_RST (GPIO_NUM_8)
#define EXAMPLE_LCD_GPIO_DC (GPIO_NUM_4)
#define EXAMPLE_LCD_GPIO_CS (GPIO_NUM_5)
#define EXAMPLE_LCD_GPIO_BL (GPIO_NUM_15)

/* Touch settings */
#define EXAMPLE_USE_TOUCH 1
#define TOUCH_HOST I2C_NUM_0
#define EXAMPLE_PIN_NUM_TOUCH_SCL (GPIO_NUM_10)
#define EXAMPLE_PIN_NUM_TOUCH_SDA (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_TOUCH_RST (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_TOUCH_INT (GPIO_NUM_14)

/* QMI8658 IMU settings */
#define QMI8658_ADDRESS 0x6B

static const char *TAG = "IMU_DISPLAY";

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

/* LVGL display and touch */
static lv_disp_t *lvgl_disp = NULL;
static esp_lcd_touch_handle_t tp = NULL;

/* IMU sensor */
static SensorQMI8658 qmi;
static IMUdata acc;
static IMUdata gyr;

/* Queue for IMU data */
static QueueHandle_t imu_data_queue = NULL;

/* LCD initialization */
static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;

    /* LCD backlight */
    gpio_config_t bk_gpio_config;
    memset(&bk_gpio_config, 0, sizeof(bk_gpio_config));
    bk_gpio_config.mode = GPIO_MODE_OUTPUT;
    bk_gpio_config.intr_type = GPIO_INTR_DISABLE;
    bk_gpio_config.pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL;
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = GPIO_NUM_NC;
    buscfg.mosi_io_num = EXAMPLE_LCD_GPIO_MOSI;
    buscfg.sclk_io_num = EXAMPLE_LCD_GPIO_SCLK;
    buscfg.quadwp_io_num = GPIO_NUM_NC;
    buscfg.quadhd_io_num = GPIO_NUM_NC;
    buscfg.max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t);
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = EXAMPLE_LCD_GPIO_DC;
    io_config.cs_gpio_num = EXAMPLE_LCD_GPIO_CS;
    io_config.pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ;
    io_config.lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS;
    io_config.lcd_param_bits = EXAMPLE_LCD_PARAM_BITS;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "New panel IO failed");
        goto err;
    }
    lcd_io = io_handle;

    ESP_LOGD(TAG, "Install LCD driver");
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = EXAMPLE_LCD_GPIO_RST;
    panel_config.color_space = EXAMPLE_LCD_COLOR_SPACE;
    panel_config.bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL;
    
    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "New panel failed");
        goto err;
    }
    lcd_panel = panel_handle;

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));

    esp_lcd_panel_set_gap(lcd_panel, 0, 20);
    esp_lcd_panel_invert_color(lcd_panel, true);

    return ret;

err:
    if (panel_handle)
    {
        esp_lcd_panel_del(panel_handle);
    }
    if (io_handle)
    {
        esp_lcd_panel_io_del(io_handle);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

/* Touch read callback */
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;
    assert(tp);

    uint16_t touch_x[1];
    uint16_t touch_y[1];
    uint8_t touch_cnt = 0;

    // Read touch data with retry logic
    int retry_count = 0;
    esp_err_t ret;
    do {
        ret = esp_lcd_touch_read_data(tp);
        if (ret == ESP_OK) {
            break;
        }
        
        if (retry_count < 3) {
            ESP_LOGW(TAG, "Touch read failed, attempt %d/3: %s", retry_count + 1, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            retry_count++;
        }
    } while (retry_count < 3);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read touch data after 3 attempts: %s", esp_err_to_name(ret));
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    // Get touch coordinates
    bool tp_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, NULL, &touch_cnt, 1);

    if (tp_pressed && touch_cnt > 0) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
        ESP_LOGI(TAG, "Touch position: %d,%d", touch_x[0], touch_y[0]);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* LVGL initialization */
static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    lvgl_port_cfg_t lvgl_cfg;
    memset(&lvgl_cfg, 0, sizeof(lvgl_cfg));
    lvgl_cfg.task_priority = 4;       /* LVGL task priority */
    lvgl_cfg.task_stack = 4096;       /* LVGL task stack size */
    lvgl_cfg.task_affinity = -1;      /* LVGL task pinned to core (-1 is no affinity) */
    lvgl_cfg.task_max_sleep_ms = 500; /* Maximum sleep in LVGL task */
    lvgl_cfg.timer_period_ms = 5;     /* LVGL timer tick period in ms */
    
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    lvgl_port_display_cfg_t disp_cfg;
    memset(&disp_cfg, 0, sizeof(disp_cfg));
    disp_cfg.io_handle = lcd_io;
    disp_cfg.panel_handle = lcd_panel;
    disp_cfg.buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t);
    disp_cfg.double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE;
    disp_cfg.hres = EXAMPLE_LCD_H_RES;
    disp_cfg.vres = EXAMPLE_LCD_V_RES;
    disp_cfg.monochrome = false;
    /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
    disp_cfg.rotation.swap_xy = false;
    disp_cfg.rotation.mirror_x = false;
    disp_cfg.rotation.mirror_y = false;
    disp_cfg.flags.buff_dma = true;
    
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    return ESP_OK;
}

/* Initialize I2C bus */
static esp_err_t app_i2c_init(void)
{
    // Configure I2C with improved settings for multiple devices
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,  // Increased to 100kHz for better performance while still maintaining reliability
        }
    };

    ESP_LOGI(TAG, "Initializing I2C bus: SDA=%d, SCL=%d, speed=%d Hz", 
             (int)EXAMPLE_PIN_NUM_TOUCH_SDA, (int)EXAMPLE_PIN_NUM_TOUCH_SCL, (int)i2c_conf.master.clk_speed);

    // Make sure no I2C driver is installed
    i2c_driver_delete(TOUCH_HOST);
    vTaskDelay(50 / portTICK_PERIOD_MS); // Short delay after deletion
    
    // Install I2C driver with error handling
    esp_err_t ret = i2c_param_config(TOUCH_HOST, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(TOUCH_HOST,
                            i2c_conf.mode,
                            SENSORLIB_I2C_MASTER_RX_BUF_DISABLE,
                            SENSORLIB_I2C_MASTER_TX_BUF_DISABLE,
                            0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set I2C timeout to be more tolerant of bus issues
    i2c_set_timeout(TOUCH_HOST, 80000); // Longer timeout for reliability
    
    // Scan I2C bus to detect connected devices
    ESP_LOGI(TAG, "Scanning I2C bus...");
    uint8_t device_count = 0;
    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(TOUCH_HOST, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            device_count++;
            
            const char* device_name = "Unknown device";
            if (address == 0x15) {
                device_name = "CST816S Touch Controller";
            } else if (address == QMI8658_ADDRESS) {
                device_name = "QMI8658 IMU";
            }
            
            ESP_LOGI(TAG, "Found I2C device at address 0x%02X (%s)", address, device_name);
        }
    }
    
    if (device_count == 0) {
        ESP_LOGW(TAG, "No I2C devices found. Check your connections!");
    } else {
        ESP_LOGI(TAG, "I2C scan complete, found %d device(s)", device_count);
    }

    return ESP_OK;
}

/* Initialize touch controller */
static esp_err_t app_touch_init(void)
{
    // Configure touch controller pins with explicit input/output settings
    ESP_LOGI(TAG, "Configuring touch controller pins...");
    
    // Configure RST as output
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << EXAMPLE_PIN_NUM_TOUCH_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_conf));
    
    // Configure INT as input with pull-up
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << EXAMPLE_PIN_NUM_TOUCH_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // Interrupt on falling edge
    };
    ESP_ERROR_CHECK(gpio_config(&int_conf));

    // More aggressive reset of touch controller with longer delays
    ESP_LOGI(TAG, "Resetting touch controller...");
    gpio_set_level(EXAMPLE_PIN_NUM_TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));  // Longer reset pulse
    gpio_set_level(EXAMPLE_PIN_NUM_TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));  // Much longer startup time

    // Create I2C panel IO with CST816S config from LVGL
    ESP_LOGI(TAG, "Creating touch panel I2C IO handle...");
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    // Set the SCL speed explicitly
    tp_io_config.scl_speed_hz = 100000;  // 100kHz
    
    esp_err_t ret = esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch panel IO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure touch controller with proper settings for CST816S
    ESP_LOGI(TAG, "Initializing touch controller...");
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,  // Active low reset
            .interrupt = 0,  // Active low interrupt
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .process_coordinates = NULL,
        .interrupt_callback = NULL,
        .user_data = NULL,
        .driver_data = NULL
    };

    // Initialize touch controller with retry logic and detailed error reporting
    int retry_count = 0;
    const int max_retries = 5;
    
    do {
        ret = esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Touch controller initialized successfully");
            break;
        }
        
        ESP_LOGW(TAG, "Touch controller initialization failed (attempt %d/%d): %s", 
                 retry_count + 1, max_retries, esp_err_to_name(ret));
        retry_count++;
        
        if (retry_count < max_retries) {
            // Reset I2C bus more aggressively
            i2c_driver_delete(TOUCH_HOST);
            vTaskDelay(pdMS_TO_TICKS(200));  // Longer delay between retries
            ESP_LOGI(TAG, "Reconfiguring I2C bus...");
            app_i2c_init();
            
            // Recreate the panel IO
            if (tp_io_handle) {
                esp_lcd_panel_io_del(tp_io_handle);
                tp_io_handle = NULL;
            }
            
            ret = esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to recreate touch panel IO: %s", esp_err_to_name(ret));
                return ret;
            }
            
            // Reset touch controller again with longer delays
            ESP_LOGI(TAG, "Re-resetting touch controller...");
            gpio_set_level(EXAMPLE_PIN_NUM_TOUCH_RST, 0);
            vTaskDelay(pdMS_TO_TICKS(100));  // Even longer reset pulse for retry
            gpio_set_level(EXAMPLE_PIN_NUM_TOUCH_RST, 1);
            vTaskDelay(pdMS_TO_TICKS(300));  // Much longer startup time for retry
        }
    } while (retry_count < max_retries);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch controller after %d attempts", max_retries);
        if (tp_io_handle) {
            esp_lcd_panel_io_del(tp_io_handle);
        }
        return ret;
    }

    ESP_LOGI(TAG, "Touch controller initialization complete");
    return ESP_OK;
}

/* IMU initialization */
static esp_err_t app_imu_init(void)
{
    // Initialize QMI8658 sensor with more detailed error reporting
    ESP_LOGI(TAG, "Initializing QMI8658 IMU sensor at address 0x%02x", QMI8658_ADDRESS);
    
    // Try multiple times to initialize the sensor
    int retry_count = 0;
    const int max_retries = 5;
    
    // Reset I2C bus before trying to initialize IMU
    // This can help clear any lingering transactions from the touch controller
    i2c_reset_tx_fifo(TOUCH_HOST);
    i2c_reset_rx_fifo(TOUCH_HOST);
    
    // Add a small delay to let the I2C bus settle
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    while (retry_count < max_retries) {
        if (qmi.begin(TOUCH_HOST, QMI8658_ADDRESS, EXAMPLE_PIN_NUM_TOUCH_SDA, EXAMPLE_PIN_NUM_TOUCH_SCL)) {
            break;
        }
        
        ESP_LOGW(TAG, "Failed to find QMI8658 (attempt %d/%d) - retrying...", 
                 retry_count + 1, max_retries);
        retry_count++;
        
        // Reset I2C bus between retries
        i2c_reset_tx_fifo(TOUCH_HOST);
        i2c_reset_rx_fifo(TOUCH_HOST);
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (retry_count >= max_retries) {
        ESP_LOGE(TAG, "Failed to find QMI8658 after %d attempts - check your wiring!", max_retries);
        return ESP_FAIL;
    }

    // Get chip ID
    uint8_t chip_id = qmi.getChipID();
    ESP_LOGI(TAG, "QMI8658 Device ID: 0x%02x", chip_id);
    
    if (chip_id == 0 || chip_id == 0xFF) {
        ESP_LOGE(TAG, "Invalid chip ID (0x%02x) - possible I2C communication issue", chip_id);
        return ESP_FAIL;
    }

    // Configure accelerometer with higher sensitivity for rep detection
    ESP_LOGI(TAG, "Configuring accelerometer");
    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,    // Increased range to capture the full motion
        SensorQMI8658::ACC_ODR_1000Hz,  // High sample rate
        SensorQMI8658::LPF_MODE_0,
        true
    );

    // Configure gyroscope
    ESP_LOGI(TAG, "Configuring gyroscope");
    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );

    // Enable gyroscope and accelerometer
    ESP_LOGI(TAG, "Enabling sensors");
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    ESP_LOGI(TAG, "IMU sensor initialized with increased sensitivity");
    return ESP_OK;
}

/* Task to read IMU data */
static void read_imu_task(void *arg)
{
    imu_data_t imu_data;
    int error_count = 0;
    const int max_errors = 10;
    TickType_t last_success_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "IMU reading task started");
    
    // Wait a bit for the system to stabilize before starting to read IMU data
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    
    while (1) {
        bool data_ready = false;
        
        // Reset I2C bus periodically to clear any lingering issues
        if (error_count > 3) {
            ESP_LOGW(TAG, "Resetting I2C bus due to errors (count: %d)", error_count);
            i2c_reset_tx_fifo(TOUCH_HOST);
            i2c_reset_rx_fifo(TOUCH_HOST);
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Increased delay after reset
        }
        
        // Add a delay between attempts to reduce I2C bus contention
        vTaskDelay(30 / portTICK_PERIOD_MS);  // More time between reads (was implicit before)
        
        // Check if data is ready with error handling
        data_ready = qmi.getDataReady();
        
        if (data_ready) {
            // Add a small delay before reading to let any touch I2C transactions complete
            vTaskDelay(10 / portTICK_PERIOD_MS);
            
            bool read_success = true;
            
            // Read accelerometer data with error handling
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                imu_data.acc_x = acc.x;
                imu_data.acc_y = acc.y;
                imu_data.acc_z = acc.z;
                ESP_LOGD(TAG, "ACCEL: %f, %f, %f", acc.x, acc.y, acc.z);
            } else {
                ESP_LOGW(TAG, "Failed to read accelerometer data");
                read_success = false;
            }

            // Small delay between reading different sensors on the same device
            vTaskDelay(5 / portTICK_PERIOD_MS);

            // Read gyroscope data with error handling
            if (read_success && qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
                imu_data.gyr_x = gyr.x;
                imu_data.gyr_y = gyr.y;
                imu_data.gyr_z = gyr.z;
                ESP_LOGD(TAG, "GYRO: %f, %f, %f", gyr.x, gyr.y, gyr.z);
            } else {
                ESP_LOGW(TAG, "Failed to read gyroscope data");
                read_success = false;
            }
            
            // Read temperature
            if (read_success) {
                imu_data.temp = qmi.getTemperature_C();
                ESP_LOGD(TAG, "Temperature: %.2f °C", imu_data.temp);
            }
            
            if (read_success) {
                // Reset error counter on successful read
                error_count = 0;
                last_success_time = xTaskGetTickCount();
                
                // Process IMU data for rep detection
                process_imu_data(&imu_data);
                
                // Send data to the queue
                xQueueSend(imu_data_queue, &imu_data, 0);
            } else {
                error_count++;
                if (error_count >= max_errors) {
                    ESP_LOGE(TAG, "Too many consecutive I2C errors (%d). Attempting to reinitialize IMU...", error_count);
                    
                    // Reset I2C bus before reinitializing
                    i2c_reset_tx_fifo(TOUCH_HOST);
                    i2c_reset_rx_fifo(TOUCH_HOST);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    
                    // Try to reinitialize the IMU
                    if (app_imu_init() == ESP_OK) {
                        ESP_LOGI(TAG, "IMU reinitialized successfully");
                        error_count = 0;
                    } else {
                        ESP_LOGE(TAG, "Failed to reinitialize IMU");
                    }
                }
            }
        } else {
            // Check if we haven't received data for a long time
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_success_time) > pdMS_TO_TICKS(5000)) {  // 5 seconds timeout
                ESP_LOGW(TAG, "No IMU data received for 5 seconds. Attempting to reinitialize...");
                
                // Reset I2C bus before reinitializing
                i2c_reset_tx_fifo(TOUCH_HOST);
                i2c_reset_rx_fifo(TOUCH_HOST);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                
                // Try to reinitialize the IMU
                if (app_imu_init() == ESP_OK) {
                    ESP_LOGI(TAG, "IMU reinitialized successfully");
                    last_success_time = current_time;
                } else {
                    ESP_LOGE(TAG, "Failed to reinitialize IMU");
                }
            }
        }
        
        // Use a longer delay if we're having errors to reduce I2C traffic
        if (error_count > 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        } else {
            vTaskDelay(50 / portTICK_PERIOD_MS); // Faster sampling for better rep detection
        }
    }
}

/* Main application entry point */
extern "C" void app_main(void)
{
    // Initialize I2C bus with improved settings first
    ESP_ERROR_CHECK(app_i2c_init());
    
    // Add a short delay after I2C init to allow bus to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Initialize LCD
    ESP_ERROR_CHECK(app_lcd_init());
    
    // Add delay after LCD init before initializing touch
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Initialize touch controller with improved initialization
    esp_err_t touch_ret = app_touch_init();
    bool touch_available = (touch_ret == ESP_OK);
    
    if (!touch_available) {
        ESP_LOGE(TAG, "Touch controller initialization failed!");
        // Reset I2C bus since touch init failed
        i2c_reset_tx_fifo(TOUCH_HOST);
        i2c_reset_rx_fifo(TOUCH_HOST);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Initialize LVGL
    ESP_ERROR_CHECK(app_lvgl_init());
    
    // Register touch input device with more detailed logging
    if (touch_available && tp != NULL) {
        ESP_LOGI(TAG, "Registering touch input with LVGL");
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.disp = lvgl_disp;
        indev_drv.read_cb = example_lvgl_touch_cb;
        indev_drv.user_data = tp;
        lv_indev_t* indev = lv_indev_drv_register(&indev_drv);
        
        if (indev) {
            ESP_LOGI(TAG, "Touch input registered successfully with LVGL");
            
            // Configure LVGL to better handle touch (increase read period for better responsiveness)
            lv_indev_set_read_timer(indev, 10);  // Read touch more frequently (10ms)
            
            // Test read of touch coordinates to verify touch is working
            ESP_LOGI(TAG, "Testing touch read...");
            uint16_t touch_x[1];
            uint16_t touch_y[1];
            uint8_t touch_cnt = 0;
            
            esp_err_t test_ret = esp_lcd_touch_read_data(tp);
            if (test_ret == ESP_OK) {
                bool tp_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, NULL, &touch_cnt, 1);
                ESP_LOGI(TAG, "Touch read test: pressed=%d, count=%d", tp_pressed, touch_cnt);
                if (tp_pressed) {
                    ESP_LOGI(TAG, "Touch coordinates: x=%d, y=%d", touch_x[0], touch_y[0]);
                }
            } else {
                ESP_LOGW(TAG, "Touch read test failed: %s", esp_err_to_name(test_ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to register touch input with LVGL");
        }
    } else {
        ESP_LOGE(TAG, "Touch initialization failed. Touch input will not be available!");
        ESP_LOGE(TAG, "Please check your hardware connections and try again.");
    }
    
    // Clear I2C bus before IMU initialization
    i2c_reset_tx_fifo(TOUCH_HOST);
    i2c_reset_rx_fifo(TOUCH_HOST);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Initialize IMU sensor after touch to avoid I2C conflicts
    ESP_LOGI(TAG, "Initializing IMU sensor...");
    esp_err_t imu_ret = app_imu_init();
    bool imu_available = (imu_ret == ESP_OK);
    
    if (!imu_available) {
        ESP_LOGE(TAG, "IMU initialization failed! Retrying once...");
        // Try one more time after a longer delay
        vTaskDelay(500 / portTICK_PERIOD_MS);
        imu_ret = app_imu_init();
        imu_available = (imu_ret == ESP_OK);
    }
    
    // Create data queue
    imu_data_queue = xQueueCreate(10, sizeof(imu_data_t));
    
    // Initialize workout app UI
    ESP_LOGI(TAG, "Creating workout app UI...");
    create_workout_app_ui(lvgl_disp);
    
    // Create task for reading IMU data (only if IMU is available)
    if (imu_available) {
        xTaskCreate(read_imu_task, "imu_task", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "IMU reading task created");
    } else {
        ESP_LOGW(TAG, "IMU not available - rep counting will not work");
    }
    
    ESP_LOGI(TAG, "Application started successfully");
} 