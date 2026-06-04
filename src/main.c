/**
 * ============================================================
 *  ESP32-S3-N16R8 Composites-Mixer Control Firmware
 *  Framework: ESP-IDF 5.x (pure C)
 * ============================================================
 *
 *  Devices:
 *    A   = ESP32-S3 board (this MCU)
 *    B1  = DC brushless motor
 *    B3  = AQMD22A04BLS-Ex brushless motor driver (RS-485 Modbus-RTU)
 *    C2  = 24 V solid state relay (SSR)
 *    C3  = Heating belt
 *    C4  = PT100 RTD via MAX31865 (hardware SPI)
 *    D1  = PCF8563 real-time clock (I2C)
 *
 *  Pin map:
 *    RS-485 (UART1, auto-direction) : TX=GPIO13, RX=GPIO8
 *    MAX31865 (SPI2, PT100 RTD)     : CLK=GPIO2, MISO/SDO=GPIO39,
 *                                     MOSI/SDI=GPIO41, CS=GPIO36
 *    PCF8563 (I2C0)                 : SCL=GPIO11, SDA=GPIO9
 *    SSR (C2 / heater)              : GPIO5
 *
 *  Motor driver notes (AQMD22A04BLS-Ex):
 *    Default Modbus: 9600 8E1, slave addr = 0x01.  The driver's DIP switch
 *    5 must be ON (communication-control mode) and 1-4 set the slave addr.
 *    Register highlights:
 *      0x0040 stop       (0=normal 1=brake 2=free)
 *      0x0042 duty       (-1000..+1000  → -100% ..+100%)
 *      0x0043 closed-loop speed target (×0.1 Hz commutation freq)
 *      0x0034 real RPM   (×10 if 0x0035=1, ×1 otherwise)
 *      0x0033 error code (0 ok, 1..9 various faults)
 *      0x0037 driver temp (×0.1 °C)
 *      0x0038 supply V    (×0.1 V)
 *
 *  Telnet commands (port 23):
 *    allon                          motor on + enable heater auto-control
 *    alloff                         motor off + heater off
 *    motoron / motoroff             drive motor at current duty / stop normally
 *    setduty <-100..100>            motor duty cycle (% of full forward)
 *    heaton / heatoff               manual heater override
 *    heatauto / heatmanual          switch heater between auto and manual modes
 *    settemp <C>                    set target temperature (enables auto mode)
 *    settempband <C>                set hysteresis half-width (default 2 °C)
 *    settempoffset <C>              add manual offset to RTD temperature
 *    setquiet <warm_dps> <cool_dps> <step_C> <pause_s>
 *                                  tune quiet RTD thermal model timing
 *    profiles                       list material cycle profiles
 *    setprofile <slot> <name> <wait_s> <run_s> <target_C>
 *                                  set/rename and activate material profile
 *    useprofile <slot|name|none>    choose active material cycle profile
 *    ls                             print full snapshot (local + driver regs)
 *    top                            repeat status until q/Enter/Ctrl-C
 *    time                           show current RTC time
 *    settime YYYY-MM-DD HH:MM:SS    set RTC + system time
 *    help                           show help
 * ============================================================
 */

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

// ============================================================
// User-configurable settings
// ============================================================

// WiFi AP
#define AP_SSID             "ScrewExtruders-Heated"
#define AP_PASS             "12345678"
#define AP_MAX_CONN         2
#define AP_CHANNEL          6

// RS-485 / UART — AQMD22A04BLS-Ex defaults to 9600 8E1
#define RS485_UART_NUM      UART_NUM_1
#define RS485_BAUD          9600
#define RS485_PARITY        UART_PARITY_EVEN
#define RS485_TX_PIN        GPIO_NUM_13
#define RS485_RX_PIN        GPIO_NUM_8
#define MODBUS_SLAVE        0x01
#define MODBUS_READ_RETRIES 5
#define MODBUS_WRITE_RETRIES 6
#define MODBUS_RX_TIMEOUT_MS 350
#define DRIVER_OFFLINE_CONFIRM 20
#define DEFAULT_DRIVER_TELEM_PAUSE_FAILS 8
#define DEFAULT_DRIVER_TELEM_PAUSE_MS 1200
#define DRIVER_TELEM_PAUSE_MIN_FAILS 1
#define DRIVER_TELEM_PAUSE_MAX_FAILS 60
#define DRIVER_TELEM_PAUSE_MIN_MS 300
#define DRIVER_TELEM_PAUSE_MAX_MS 10000
#define MOTOR_STOP_RETRY_INTERVAL_MS 500
#define STATUS_STREAM_INTERVAL_MS 2000

// MAX31865 RTD-to-digital (hardware SPI on SPI2_HOST)
#define RTD_SPI_HOST        SPI2_HOST
#define RTD_CLK_PIN         GPIO_NUM_2
#define RTD_MISO_PIN        GPIO_NUM_39   // MAX31865 SDO
#define RTD_MOSI_PIN        GPIO_NUM_41   // MAX31865 SDI
#define RTD_CS_PIN          GPIO_NUM_36

#define RTD_WIRES           3             // 2, 3, or 4 wire PT100
#define RTD_NOMINAL_OHM     100.0f        // PT100 = 100, PT1000 = 1000
#define RTD_RREF_OHM        430.0f        // Adafruit breakout: 430 Ω for PT100, 4300 Ω for PT1000
#define RTD_FILTER_50HZ     1             // 1 = 50 Hz mains, 0 = 60 Hz
#define RTD_SPI_CLOCK_HZ    (250 * 1000)  // slow edge rate helps with long/noisy wiring

#define TEMP_READ_INTERVAL  500           // ms; controller loop runs at the same cadence
#define TEMP_SPI_READ_RETRIES     2
#define TEMP_FAULT_CONFIRM_COUNT  3       // debounce motor-induced MAX31865 fault spikes
#define TEMP_RECOVER_FAULT_COUNT  8       // reconfigure MAX31865 after persistent latch
#define TEMP_RECOVER_COOLDOWN_MS  3000
#define TEMP_MIN_VALID_C          -50.0f
#define TEMP_MAX_VALID_C          300.0f
#define TEMP_MAX_STEP_C           20.0f    // reject impossible 500 ms jumps below hard cutoff
#define TEMP_OFFSET_MIN_C         -50.0f
#define TEMP_OFFSET_MAX_C          50.0f

// PCF8563 RTC
#define RTC_I2C_PORT        I2C_NUM_0
#define RTC_SCL_PIN         GPIO_NUM_11
#define RTC_SDA_PIN         GPIO_NUM_9
#define RTC_I2C_FREQ        100000
#define PCF8563_ADDR        0x51

// SSR (logic-level → SSR input)
#define SSR_PIN             GPIO_NUM_5

// Heater safety / control
#define HEATER_MIN_CYCLE_MS 2000          // minimum time between SSR state changes
#define HEATER_MAX_TEMP_C   250.0f        // hard over-temperature cutoff
#define DEFAULT_TEMP_TARGET 80.0f
#define DEFAULT_TEMP_BAND   2.0f          // ±2 °C hysteresis

// Quiet anti-interference sampling: allon briefly pauses motor noise, reads
// RTD, then holds heater state until the thermal model says another read is due.
#define DEFAULT_SOFTANTI_WARM_DPS     0.357f
#define DEFAULT_SOFTANTI_COOL_DPS     0.040f
#define DEFAULT_SOFTANTI_STEP_C       3.0f
#define DEFAULT_SOFTANTI_PAUSE_MS     1500
#define SOFTANTI_MIN_RATE_DPS         0.001f
#define SOFTANTI_MAX_RATE_DPS         10.0f
#define SOFTANTI_MIN_STEP_C           0.1f
#define SOFTANTI_MAX_STEP_C           50.0f
#define SOFTANTI_MIN_PAUSE_MS         300
#define SOFTANTI_MAX_PAUSE_MS         10000
#define SOFTANTI_MIN_INTERVAL_MS      1000
#define SOFTANTI_MAX_INTERVAL_MS      600000
#define SOFTANTI_COOL_PROBE_MIN_MS    HEATER_MIN_CYCLE_MS
#define SOFTANTI_COOL_PROBE_MAX_MS    5000
#define SOFTANTI_COOL_PROBE_MIN_C     0.10f
#define SOFTANTI_COOL_PROBE_MAX_C     0.25f
#define SOFTANTI_COOL_PROBE_STEP_FRAC 0.05f

#define WAIT_PROFILE_COUNT        6
#define WAIT_PROFILE_NAME_LEN     16
#define WAIT_PROFILE_MAX_MS       (3600U * 1000U)
#define WAIT_PROFILE_CHUNK_MS     500
#define STATUS_GRAPH_WIDTH        48

// Telnet
#define TELNET_PORT         23
#define CMD_BUF_SIZE        96

// ============================================================
// Log tags
// ============================================================

static const char *TAG       = "MAIN";
static const char *TAG_WIFI  = "WIFI";
static const char *TAG_RS485 = "RS485";
static const char *TAG_MOTOR = "MOTOR";
static const char *TAG_HEAT  = "HEATER";
static const char *TAG_TEMP  = "TEMP";
static const char *TAG_RTC   = "RTC";
static const char *TAG_TEL   = "TELNET";

#define ANSI_RESET   "\x1b[0m"
#define ANSI_DIM     "\x1b[2m"
#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_CYAN    "\x1b[36m"

// ============================================================
// Motor driver (AQMD22A04BLS-Ex) register map
// ============================================================

// Real-time status (read with FC 0x03)
#define MREG_RT_PWM        0x0020   // 0..1000 (×0.1 %)
#define MREG_RT_CURRENT    0x0021   // ×0.01 A
#define MREG_RT_FREQ       0x0022   // commutation freq, signed
#define MREG_STALL         0x0032
#define MREG_ERROR         0x0033   // 0 ok; 1..9 faults
#define MREG_RPM           0x0034
#define MREG_RPM_X10       0x0035
#define MREG_DRV_TEMP      0x0037   // ×0.1 °C
#define MREG_SUPPLY_V      0x0038   // ×0.1 V
#define MREG_CTRL_SRC      0x0039   // 0 local, 1 comms

// Command registers (write with FC 0x06)
#define MREG_STOP          0x0040
#define MREG_DUTY          0x0042   // -1000..+1000
#define MREG_SPEED_TGT     0x0043

#define MOTOR_STOP_NORMAL  0
#define MOTOR_STOP_BRAKE   1
#define MOTOR_STOP_FREE    2

static const char *MOTOR_ERROR_TEXT[] = {
    "OK", "not-learned", "stall-stop", "hall-error", "speed-not-reached",
    "coil-error", "over-current", "over-temp", "over-voltage", "under-voltage",
};

// ============================================================
// Heater control mode
// ============================================================

typedef enum {
    HEATER_MANUAL = 0,
    HEATER_AUTO   = 1,
} heater_mode_t;

typedef struct {
    char name[WAIT_PROFILE_NAME_LEN];
    uint32_t wait_ms;
    uint32_t run_ms;
    float target_temp_c;
} wait_profile_t;

// ============================================================
// Global state
// ============================================================

static float   g_temperature    = 0.0f;
static float   g_temp_offset_c  = 0.0f;
static bool    g_temperature_valid = false;
static bool    g_tc_error       = false;
static uint8_t g_tc_fault_code  = 0;
static uint8_t g_temp_fault_streak = 0;
static int64_t g_temp_last_recover_us = 0;

static bool    g_motor_on       = false;
static int     g_target_duty    = 500;     // ×0.1 %: 500 = 50 %; used when starting the motor
static bool    g_motor_stop_pending = false;
static bool    g_allstart_session = false;

static bool    g_heater_on      = false;
static heater_mode_t g_heater_mode   = HEATER_MANUAL;
static bool    g_heater_control_enabled = false;
static float   g_target_temp_c      = DEFAULT_TEMP_TARGET;
static float   g_temp_band_c        = DEFAULT_TEMP_BAND;
static int64_t g_heater_last_edge_us = 0;

static float   g_softanti_warm_dps = DEFAULT_SOFTANTI_WARM_DPS;
static float   g_softanti_cool_dps = DEFAULT_SOFTANTI_COOL_DPS;
static float   g_softanti_step_c = DEFAULT_SOFTANTI_STEP_C;
static uint32_t g_softanti_pause_ms = DEFAULT_SOFTANTI_PAUSE_MS;
static bool    g_softanti_paused = false;
static int64_t g_softanti_next_sample_us = 0;
static int64_t g_softanti_last_sample_us = 0;
static float   g_softanti_last_sample_c = 0.0f;
static bool    g_softanti_heating_model = false;
static bool    g_softanti_model_valid = false;
static uint8_t g_driver_telem_pause_fails = DEFAULT_DRIVER_TELEM_PAUSE_FAILS;
static uint32_t g_driver_telem_pause_ms = DEFAULT_DRIVER_TELEM_PAUSE_MS;
static char g_status_graph[STATUS_GRAPH_WIDTH];
static uint8_t g_status_graph_pos = 0;
static uint8_t g_status_graph_count = 0;
static wait_profile_t g_wait_profiles[WAIT_PROFILE_COUNT] = {
    {"none", 0, 0, DEFAULT_TEMP_TARGET},
    {"PE", 60000, 60000, DEFAULT_TEMP_TARGET},
    {"PVC", 120000, 60000, DEFAULT_TEMP_TARGET},
    {"profile3", 0, 60000, DEFAULT_TEMP_TARGET},
    {"profile4", 0, 60000, DEFAULT_TEMP_TARGET},
    {"profile5", 0, 60000, DEFAULT_TEMP_TARGET},
};
static uint8_t g_active_wait_profile = 0;

// Last-read driver telemetry (cached for `status`)
static bool     g_drv_read_ok   = false;
static uint16_t g_drv_rpm       = 0;
static int16_t  g_drv_freq      = 0;
static uint16_t g_drv_pwm_pm    = 0;   // ×0.1 %
static uint16_t g_drv_current_cA = 0;
static int16_t  g_drv_temp_dC   = 0;
static uint16_t g_drv_supply_dV = 0;
static uint8_t  g_drv_error     = 0;
static bool     g_drv_rpm_scaled = false;
static uint8_t  g_drv_fail_count = 0;
static int64_t  g_drv_last_ok_us = 0;

static spi_device_handle_t g_rtd_spi = NULL;
static SemaphoreHandle_t   g_modbus_mutex = NULL;

static int               g_telnet_client_fd = -1;
static SemaphoreHandle_t g_telnet_mutex = NULL;

// ============================================================
// NVS
// ============================================================

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// ============================================================
// Config persistence — target duty, target temp, hysteresis band, softanti
// ============================================================

#define CFG_NAMESPACE   "mixer"
#define CFG_KEY_DUTY    "duty"
#define CFG_KEY_TEMP    "temp_c"
#define CFG_KEY_BAND    "band_c"
#define CFG_KEY_SOFTANTI_WARM  "sa_warm"
#define CFG_KEY_SOFTANTI_COOL  "sa_cool"
#define CFG_KEY_SOFTANTI_STEP  "sa_step"
#define CFG_KEY_SOFTANTI_PAUSE "sa_pause_ms"
#define CFG_KEY_DRV_TEL_FAILS  "dt_fails"
#define CFG_KEY_DRV_TEL_PAUSE  "dt_pause_ms"
#define CFG_KEY_TEMP_OFFSET    "temp_offset"
#define CFG_KEY_ACTIVE_PROFILE "active_prof"

static char ascii_lower(char c)
{
    if (c >= 'A' && c <= 'Z') return (char)(c - 'A' + 'a');
    return c;
}

static bool str_eq_ci(const char *a, const char *b)
{
    if (a == NULL || b == NULL) return false;
    while (*a != '\0' && *b != '\0') {
        if (ascii_lower(*a) != ascii_lower(*b)) return false;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static bool profile_name_char_ok(char c)
{
    return (c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') ||
           c == '_' || c == '-';
}

static bool profile_name_valid(const char *name)
{
    if (name == NULL || name[0] == '\0') return false;
    size_t len = strlen(name);
    if (len >= WAIT_PROFILE_NAME_LEN) return false;
    for (size_t i = 0; i < len; i++) {
        if (!profile_name_char_ok(name[i])) return false;
    }
    return true;
}

static const char *heater_mode_name(heater_mode_t mode)
{
    return (mode == HEATER_AUTO) ? "auto" : "manual";
}

static int wait_profile_find(const char *token)
{
    if (token == NULL || token[0] == '\0') return -1;
    if (str_eq_ci(token, "none") || str_eq_ci(token, "off")) return 0;

    char *end = NULL;
    long slot = strtol(token, &end, 10);
    if (end != token && *end == '\0' &&
        slot >= 0 && slot < WAIT_PROFILE_COUNT) {
        return (int)slot;
    }

    for (int i = 0; i < WAIT_PROFILE_COUNT; i++) {
        if (str_eq_ci(token, g_wait_profiles[i].name)) return i;
    }
    return -1;
}

static uint32_t wait_profile_active_ms(void)
{
    if (g_active_wait_profile >= WAIT_PROFILE_COUNT) return 0;
    return g_wait_profiles[g_active_wait_profile].wait_ms;
}

static uint32_t wait_profile_active_run_ms(void)
{
    if (g_active_wait_profile == 0 || g_active_wait_profile >= WAIT_PROFILE_COUNT) {
        return 0;
    }
    return g_wait_profiles[g_active_wait_profile].run_ms;
}

static void wait_profile_apply_active(void)
{
    g_heater_mode = HEATER_AUTO;
    if (g_active_wait_profile == 0 || g_active_wait_profile >= WAIT_PROFILE_COUNT) {
        return;
    }
    g_target_temp_c = g_wait_profiles[g_active_wait_profile].target_temp_c;
}

static void config_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CFG_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "config_save: nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    // Store duty as i32, temps as blobs (NVS has no native float).
    nvs_set_i32(h, CFG_KEY_DUTY, g_target_duty);
    nvs_set_blob(h, CFG_KEY_TEMP, &g_target_temp_c, sizeof(g_target_temp_c));
    nvs_set_blob(h, CFG_KEY_BAND, &g_temp_band_c,   sizeof(g_temp_band_c));
    nvs_set_blob(h, CFG_KEY_TEMP_OFFSET, &g_temp_offset_c, sizeof(g_temp_offset_c));
    nvs_set_blob(h, CFG_KEY_SOFTANTI_WARM, &g_softanti_warm_dps, sizeof(g_softanti_warm_dps));
    nvs_set_blob(h, CFG_KEY_SOFTANTI_COOL, &g_softanti_cool_dps, sizeof(g_softanti_cool_dps));
    nvs_set_blob(h, CFG_KEY_SOFTANTI_STEP, &g_softanti_step_c, sizeof(g_softanti_step_c));
    nvs_set_u32(h, CFG_KEY_SOFTANTI_PAUSE, g_softanti_pause_ms);
    nvs_set_u8(h, CFG_KEY_DRV_TEL_FAILS, g_driver_telem_pause_fails);
    nvs_set_u32(h, CFG_KEY_DRV_TEL_PAUSE, g_driver_telem_pause_ms);
    nvs_set_u8(h, CFG_KEY_ACTIVE_PROFILE, g_active_wait_profile);
    for (int i = 0; i < WAIT_PROFILE_COUNT; i++) {
        char key[16];
        snprintf(key, sizeof(key), "wp%d_name", i);
        nvs_set_str(h, key, g_wait_profiles[i].name);
        snprintf(key, sizeof(key), "wp%d_ms", i);
        nvs_set_u32(h, key, g_wait_profiles[i].wait_ms);
        snprintf(key, sizeof(key), "wp%d_run", i);
        nvs_set_u32(h, key, g_wait_profiles[i].run_ms);
        snprintf(key, sizeof(key), "wp%d_temp", i);
        nvs_set_blob(h, key, &g_wait_profiles[i].target_temp_c,
                     sizeof(g_wait_profiles[i].target_temp_c));
    }
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "config saved: duty=%d temp=%.1f band=%.1f offset=%+.2f quiet warm=%.3f cool=%.3f step=%.1f pause=%ums profile=%s wait=%ums run=%ums target=%.1f drvquiet=%us/%ums",
             g_target_duty, g_target_temp_c, g_temp_band_c,
             g_temp_offset_c,
             g_softanti_warm_dps, g_softanti_cool_dps, g_softanti_step_c,
             (unsigned)g_softanti_pause_ms,
             g_wait_profiles[g_active_wait_profile].name,
             (unsigned)g_wait_profiles[g_active_wait_profile].wait_ms,
             (unsigned)g_wait_profiles[g_active_wait_profile].run_ms,
             g_wait_profiles[g_active_wait_profile].target_temp_c,
             (unsigned)g_driver_telem_pause_fails,
             (unsigned)g_driver_telem_pause_ms);
}

static void config_load(void)
{
    nvs_handle_t h;
    if (nvs_open(CFG_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "config_load: using defaults (no saved config yet)");
        return;
    }
    int32_t duty;
    if (nvs_get_i32(h, CFG_KEY_DUTY, &duty) == ESP_OK &&
        duty >= -1000 && duty <= 1000) {
        g_target_duty = (int)duty;
    }
    size_t sz = sizeof(float);
    float tmp;
    if (nvs_get_blob(h, CFG_KEY_TEMP, &tmp, &sz) == ESP_OK && sz == sizeof(float) &&
        tmp >= 0.0f && tmp <= HEATER_MAX_TEMP_C) {
        g_target_temp_c = tmp;
    }
    sz = sizeof(float);
    if (nvs_get_blob(h, CFG_KEY_BAND, &tmp, &sz) == ESP_OK && sz == sizeof(float) &&
        tmp >= 0.1f && tmp <= 50.0f) {
        g_temp_band_c = tmp;
    }
    sz = sizeof(float);
    if (nvs_get_blob(h, CFG_KEY_TEMP_OFFSET, &tmp, &sz) == ESP_OK &&
        sz == sizeof(float) && tmp >= TEMP_OFFSET_MIN_C && tmp <= TEMP_OFFSET_MAX_C) {
        g_temp_offset_c = tmp;
    }
    sz = sizeof(float);
    if (nvs_get_blob(h, CFG_KEY_SOFTANTI_WARM, &tmp, &sz) == ESP_OK &&
        sz == sizeof(float) && tmp >= SOFTANTI_MIN_RATE_DPS && tmp <= SOFTANTI_MAX_RATE_DPS) {
        g_softanti_warm_dps = tmp;
    }
    sz = sizeof(float);
    if (nvs_get_blob(h, CFG_KEY_SOFTANTI_COOL, &tmp, &sz) == ESP_OK &&
        sz == sizeof(float) && tmp >= SOFTANTI_MIN_RATE_DPS && tmp <= SOFTANTI_MAX_RATE_DPS) {
        g_softanti_cool_dps = tmp;
    }
    sz = sizeof(float);
    if (nvs_get_blob(h, CFG_KEY_SOFTANTI_STEP, &tmp, &sz) == ESP_OK &&
        sz == sizeof(float) && tmp >= SOFTANTI_MIN_STEP_C && tmp <= SOFTANTI_MAX_STEP_C) {
        g_softanti_step_c = tmp;
    }
    uint32_t u32 = 0;
    if (nvs_get_u32(h, CFG_KEY_SOFTANTI_PAUSE, &u32) == ESP_OK &&
        u32 >= SOFTANTI_MIN_PAUSE_MS && u32 <= SOFTANTI_MAX_PAUSE_MS) {
        g_softanti_pause_ms = u32;
    }
    uint8_t u8 = 0;
    if (nvs_get_u8(h, CFG_KEY_DRV_TEL_FAILS, &u8) == ESP_OK &&
        u8 >= DRIVER_TELEM_PAUSE_MIN_FAILS && u8 <= DRIVER_TELEM_PAUSE_MAX_FAILS) {
        g_driver_telem_pause_fails = u8;
    }
    if (nvs_get_u32(h, CFG_KEY_DRV_TEL_PAUSE, &u32) == ESP_OK &&
        u32 >= DRIVER_TELEM_PAUSE_MIN_MS && u32 <= DRIVER_TELEM_PAUSE_MAX_MS) {
        g_driver_telem_pause_ms = u32;
    }
    for (int i = 0; i < WAIT_PROFILE_COUNT; i++) {
        char key[16];
        char name[WAIT_PROFILE_NAME_LEN];
        size_t name_len = sizeof(name);
        snprintf(key, sizeof(key), "wp%d_name", i);
        if (nvs_get_str(h, key, name, &name_len) == ESP_OK &&
            profile_name_valid(name)) {
            strncpy(g_wait_profiles[i].name, name, sizeof(g_wait_profiles[i].name));
            g_wait_profiles[i].name[sizeof(g_wait_profiles[i].name) - 1] = '\0';
        }
        snprintf(key, sizeof(key), "wp%d_ms", i);
        if (nvs_get_u32(h, key, &u32) == ESP_OK && u32 <= WAIT_PROFILE_MAX_MS) {
            g_wait_profiles[i].wait_ms = u32;
        }
        snprintf(key, sizeof(key), "wp%d_run", i);
        if (nvs_get_u32(h, key, &u32) == ESP_OK && u32 <= WAIT_PROFILE_MAX_MS) {
            g_wait_profiles[i].run_ms = u32;
        }
        snprintf(key, sizeof(key), "wp%d_temp", i);
        sz = sizeof(float);
        if (nvs_get_blob(h, key, &tmp, &sz) == ESP_OK &&
            sz == sizeof(float) && tmp >= 0.0f && tmp <= HEATER_MAX_TEMP_C) {
            g_wait_profiles[i].target_temp_c = tmp;
        }
    }
    if (nvs_get_u8(h, CFG_KEY_ACTIVE_PROFILE, &u8) == ESP_OK &&
        u8 < WAIT_PROFILE_COUNT) {
        g_active_wait_profile = u8;
    }
    strncpy(g_wait_profiles[0].name, "none", sizeof(g_wait_profiles[0].name));
    g_wait_profiles[0].name[sizeof(g_wait_profiles[0].name) - 1] = '\0';
    g_wait_profiles[0].wait_ms = 0;
    g_wait_profiles[0].run_ms = 0;
    g_wait_profiles[0].target_temp_c = DEFAULT_TEMP_TARGET;
    wait_profile_apply_active();
    nvs_close(h);
    ESP_LOGI(TAG, "config loaded: duty=%d temp=%.1f band=%.1f offset=%+.2f quiet warm=%.3f cool=%.3f step=%.1f pause=%ums profile=%s wait=%ums run=%ums target=%.1f drvquiet=%us/%ums",
             g_target_duty, g_target_temp_c, g_temp_band_c,
             g_temp_offset_c,
             g_softanti_warm_dps, g_softanti_cool_dps, g_softanti_step_c,
             (unsigned)g_softanti_pause_ms,
             g_wait_profiles[g_active_wait_profile].name,
             (unsigned)g_wait_profiles[g_active_wait_profile].wait_ms,
             (unsigned)g_wait_profiles[g_active_wait_profile].run_ms,
             g_wait_profiles[g_active_wait_profile].target_temp_c,
             (unsigned)g_driver_telem_pause_fails,
             (unsigned)g_driver_telem_pause_ms);
}

// ============================================================
// Wi-Fi soft-AP
// ============================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG_WIFI, "station " MACSTR " joined, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG_WIFI, "station " MACSTR " left, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void init_wifi_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .password = AP_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .required = true },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "AP started | SSID=%s PASS=%s CH=%d", AP_SSID, AP_PASS, AP_CHANNEL);
}

// ============================================================
// RS-485 / UART — auto-direction module (no DE/RE pin)
// ============================================================

static void init_rs485(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = RS485_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = RS485_PARITY,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG_RS485, "UART%d init | baud=%d 8E1 TX=%d RX=%d (auto-direction)",
             RS485_UART_NUM, RS485_BAUD, RS485_TX_PIN, RS485_RX_PIN);
}

// ============================================================
// Modbus-RTU
// ============================================================

static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else              { crc >>= 1; }
        }
    }
    return crc;
}

// Drain any stale bytes from the RX fifo before a new transaction.
static void modbus_flush_rx(void)
{
    uart_flush_input(RS485_UART_NUM);
}

// Writes `len` bytes of frame[] (CRC is appended here) and waits for TX done.
static esp_err_t modbus_tx(uint8_t *frame, size_t len)
{
    uint16_t crc = modbus_crc16(frame, len);
    frame[len]     = (uint8_t)(crc & 0xFF);
    frame[len + 1] = (uint8_t)(crc >> 8);

    ESP_LOG_BUFFER_HEX_LEVEL(TAG_RS485, frame, len + 2, ESP_LOG_DEBUG);

    modbus_flush_rx();
    vTaskDelay(pdMS_TO_TICKS(5));   // quiet bus gap; helps noisy auto-direction modules
    int written = uart_write_bytes(RS485_UART_NUM, (const char *)frame, len + 2);
    if (written != (int)(len + 2)) return ESP_FAIL;
    return uart_wait_tx_done(RS485_UART_NUM, pdMS_TO_TICKS(100));
}

// Read `nbytes` from UART with a short timeout (frame-level wait).
static int modbus_rx(uint8_t *buf, size_t nbytes, uint32_t timeout_ms)
{
    return uart_read_bytes(RS485_UART_NUM, buf, nbytes, pdMS_TO_TICKS(timeout_ms));
}

static esp_err_t modbus_validate_crc_frame(const uint8_t *frame, size_t len)
{
    if (len < 4) return ESP_ERR_INVALID_SIZE;
    uint16_t crc_calc = modbus_crc16(frame, len - 2);
    uint16_t crc_recv = (uint16_t)frame[len - 2] |
                        ((uint16_t)frame[len - 1] << 8);
    return (crc_calc == crc_recv) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

// FC 0x06 — write single register
static esp_err_t modbus_write_reg(uint8_t slave, uint16_t reg, uint16_t value)
{
    esp_err_t last_err = ESP_FAIL;

    for (int attempt = 1; attempt <= MODBUS_WRITE_RETRIES; attempt++) {
        xSemaphoreTake(g_modbus_mutex, portMAX_DELAY);

        uint8_t frame[10];
        frame[0] = slave;
        frame[1] = 0x06;
        frame[2] = (uint8_t)(reg >> 8);
        frame[3] = (uint8_t)(reg & 0xFF);
        frame[4] = (uint8_t)(value >> 8);
        frame[5] = (uint8_t)(value & 0xFF);

        esp_err_t err = modbus_tx(frame, 6);
        if (err == ESP_OK) {
            uint8_t resp[8];
            int got = modbus_rx(resp, sizeof(resp), MODBUS_RX_TIMEOUT_MS);
            if (got != (int)sizeof(resp)) {
                err = ESP_ERR_TIMEOUT;
            } else if (modbus_validate_crc_frame(resp, sizeof(resp)) != ESP_OK) {
                err = ESP_ERR_INVALID_CRC;
            } else if (memcmp(resp, frame, sizeof(resp)) != 0) {
                err = ESP_ERR_INVALID_RESPONSE;
            }
        }

        xSemaphoreGive(g_modbus_mutex);

        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            return ESP_OK;
        }

        last_err = err;
        ESP_LOGD(TAG_RS485, "FC06 reg 0x%04X attempt %d/%d failed: %s",
                 reg, attempt, MODBUS_WRITE_RETRIES, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return last_err;
}

// FC 0x03 — read `count` holding registers into `out` (big-endian per-register).
static esp_err_t modbus_read_regs(uint8_t slave, uint16_t start, uint16_t count, uint16_t *out)
{
    if (count == 0 || count > 16) return ESP_ERR_INVALID_ARG;

    esp_err_t last_err = ESP_FAIL;
    size_t expected = 5 + 2 * count;

    for (int attempt = 1; attempt <= MODBUS_READ_RETRIES; attempt++) {
        xSemaphoreTake(g_modbus_mutex, portMAX_DELAY);

        uint8_t req[10];
        req[0] = slave;
        req[1] = 0x03;
        req[2] = (uint8_t)(start >> 8);
        req[3] = (uint8_t)(start & 0xFF);
        req[4] = (uint8_t)(count >> 8);
        req[5] = (uint8_t)(count & 0xFF);

        uint8_t resp[5 + 2 * 16];
        esp_err_t err = modbus_tx(req, 6);
        int got = 0;
        if (err == ESP_OK) {
            got = modbus_rx(resp, expected, MODBUS_RX_TIMEOUT_MS);
        }

        xSemaphoreGive(g_modbus_mutex);

        if (err == ESP_OK && got < (int)expected) {
            ESP_LOGD(TAG_RS485, "FC03 short response: got=%d want=%u",
                     got, (unsigned)expected);
            err = ESP_ERR_TIMEOUT;
        }
        if (err == ESP_OK &&
            (resp[0] != slave || resp[1] != 0x03 || resp[2] != count * 2)) {
            ESP_LOGD(TAG_RS485, "FC03 bad header %02X %02X %02X",
                     resp[0], resp[1], resp[2]);
            err = ESP_ERR_INVALID_RESPONSE;
        }
        if (err == ESP_OK) {
            err = modbus_validate_crc_frame(resp, expected);
        }
        if (err == ESP_OK) {
            for (uint16_t i = 0; i < count; i++) {
                out[i] = ((uint16_t)resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
            }
            return ESP_OK;
        }

        last_err = err;
        ESP_LOGD(TAG_RS485, "FC03 start 0x%04X count %u attempt %d/%d failed: %s",
                 start, count, attempt, MODBUS_READ_RETRIES, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return last_err;
}

// ============================================================
// Motor control (AQMD22A04BLS-Ex over Modbus)
// ============================================================

static esp_err_t motor_poll_status(void);

static bool motor_write_may_have_applied(esp_err_t err)
{
    return err == ESP_ERR_TIMEOUT ||
           err == ESP_ERR_INVALID_CRC ||
           err == ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t motor_accept_write_result(const char *op, esp_err_t err)
{
    if (err == ESP_OK) return ESP_OK;

    if (motor_write_may_have_applied(err)) {
        ESP_LOGW(TAG_MOTOR, "%s unconfirmed (%s); treating command as accepted",
                 op, esp_err_to_name(err));
        return ESP_OK;
    }

    ESP_LOGW(TAG_MOTOR, "%s failed: %s", op, esp_err_to_name(err));
    return err;
}

static esp_err_t motor_apply_duty(int duty_tenths)
{
    if (duty_tenths > 1000)  duty_tenths = 1000;
    if (duty_tenths < -1000) duty_tenths = -1000;
    uint16_t v = (uint16_t)(int16_t)duty_tenths;   // signed 16-bit on the wire
    esp_err_t err = modbus_write_reg(MODBUS_SLAVE, MREG_DUTY, v);
    return motor_accept_write_result("duty write", err);
}

static esp_err_t motor_start(void)
{
    ESP_LOGI(TAG_MOTOR, "start duty=%.1f%%", g_target_duty / 10.0f);
    g_motor_stop_pending = false;
    esp_err_t err = motor_apply_duty(g_target_duty);
    if (err == ESP_OK) {
        g_motor_on = true;
        esp_err_t poll_err = motor_poll_status();
        if (poll_err != ESP_OK) {
            ESP_LOGW(TAG_MOTOR, "start accepted; telemetry confirmation failed: %s",
                     esp_err_to_name(poll_err));
        }
        return ESP_OK;
    } else {
        ESP_LOGW(TAG_MOTOR, "start command failed: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t motor_stop_once(bool confirm)
{
    // The AQMD22 keeps spinning if the duty register is still non-zero even after a
    // stop command, so zero the output FIRST, then trigger the stop-brake register.
    esp_err_t duty_err = motor_apply_duty(0);
    esp_err_t stop_err = motor_accept_write_result(
        "stop register write",
        modbus_write_reg(MODBUS_SLAVE, MREG_STOP, MOTOR_STOP_NORMAL));
    if (duty_err == ESP_OK && stop_err == ESP_OK) {
        if (!confirm) {
            g_motor_on = false;
            return ESP_OK;
        }
        esp_err_t confirm_err = motor_poll_status();
        if (confirm_err == ESP_OK) {
            g_motor_on = false;
            return ESP_OK;
        }
        ESP_LOGW(TAG_MOTOR, "stop not confirmed: %s", esp_err_to_name(confirm_err));
        return confirm_err;
    }
    ESP_LOGW(TAG_MOTOR, "stop command incomplete: duty=%s stop=%s",
             esp_err_to_name(duty_err), esp_err_to_name(stop_err));
    return (duty_err != ESP_OK) ? duty_err : stop_err;
}

static esp_err_t motor_stop(void)
{
    ESP_LOGI(TAG_MOTOR, "stop requested (latched retry until confirmed)");
    g_motor_stop_pending = true;
    g_allstart_session = false;
    g_softanti_paused = false;

    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 3; i++) {
        err = motor_stop_once(true);
        if (err == ESP_OK) {
            g_motor_stop_pending = false;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGW(TAG_MOTOR, "stop still pending: %s", esp_err_to_name(err));
    return err;
}

// Poll driver telemetry into the global cache.
static esp_err_t motor_poll_status(void)
{
    uint16_t regs[6];  // 0x0020..0x0022 (3) plus 0x0033..0x0038 block we'll re-read
    // Block 1: 0x0020..0x0022 (PWM, current, freq)
    esp_err_t err = modbus_read_regs(MODBUS_SLAVE, MREG_RT_PWM, 3, regs);
    if (err != ESP_OK) return err;
    g_drv_pwm_pm     = regs[0];
    g_drv_current_cA = regs[1];
    g_drv_freq       = (int16_t)regs[2];

    // Block 2: 0x0033..0x0038 (error, rpm, rpm_x10, reserved 0x0036, driver temp, supply)
    uint16_t regs2[6];
    err = modbus_read_regs(MODBUS_SLAVE, MREG_ERROR, 6, regs2);
    if (err != ESP_OK) return err;
    g_drv_error      = (uint8_t)(regs2[0] & 0xFF);
    g_drv_rpm        = regs2[1];
    g_drv_rpm_scaled = (regs2[2] != 0);
    g_drv_temp_dC    = (int16_t)regs2[4];
    g_drv_supply_dV  = regs2[5];
    g_drv_fail_count = 0;
    g_drv_last_ok_us = esp_timer_get_time();
    g_drv_read_ok    = true;
    return ESP_OK;
}

// ============================================================
// Heater / SSR
// ============================================================

static void init_ssr(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << SSR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);
    gpio_set_level(SSR_PIN, 0);
    ESP_LOGI(TAG_HEAT, "SSR pin GPIO%d -> LOW", SSR_PIN);
}

// Low-level SSR flip with minimum cycle time (protects the SSR from chatter).
static bool heater_set_relay(bool on)
{
    int64_t now = esp_timer_get_time();
    if (on == g_heater_on) return true;
    if (on && g_heater_last_edge_us != 0 &&
        (now - g_heater_last_edge_us) < (int64_t)HEATER_MIN_CYCLE_MS * 1000) {
        return false;   // too soon to turn back on — caller retries later
    }
    gpio_set_level(SSR_PIN, on ? 1 : 0);
    g_heater_on = on;
    g_heater_last_edge_us = now;
    ESP_LOGI(TAG_HEAT, "SSR %s", on ? "ON" : "OFF");
    return true;
}

static void heater_manual_on(void)
{
    g_heater_mode = HEATER_MANUAL;
    g_heater_control_enabled = true;
    heater_set_relay(true);
}

static void heater_manual_off(void)
{
    g_heater_mode = HEATER_MANUAL;
    g_heater_control_enabled = false;
    heater_set_relay(false);
}

// Bang-bang with hysteresis + safety cutoffs. Runs from temperature_task.
static void heater_control_tick(void)
{
    // Safety interlocks — always force OFF when the sensor is bad or over max
    if (g_tc_error || !g_temperature_valid) {
        if (g_heater_on) ESP_LOGW(TAG_HEAT, "sensor invalid/fault — forcing SSR OFF");
        heater_set_relay(false);
        return;
    }
    if (g_temperature > HEATER_MAX_TEMP_C) {
        if (g_heater_on) ESP_LOGW(TAG_HEAT, "over-temp %.1f°C — forcing SSR OFF", g_temperature);
        heater_set_relay(false);
        return;
    }

    if (!g_heater_control_enabled) {
        heater_set_relay(false);
        return;
    }

    if (g_heater_mode != HEATER_AUTO) return;

    float upper = g_target_temp_c + g_temp_band_c;
    float lower = g_target_temp_c - g_temp_band_c;

    if (g_temperature >= upper && g_heater_on) {
        heater_set_relay(false);
    } else if (g_temperature <= lower && !g_heater_on) {
        heater_set_relay(true);
    }
}

static void heater_apply_session_start(void)
{
    if (!g_heater_control_enabled) {
        heater_set_relay(false);
        return;
    }

    if (g_heater_mode == HEATER_MANUAL) {
        heater_set_relay(true);
        return;
    }

    if (g_heater_mode == HEATER_AUTO && g_temperature_valid && !g_tc_error) {
        if (g_temperature < g_target_temp_c && g_temperature < HEATER_MAX_TEMP_C) {
            heater_set_relay(true);
            return;
        }
        heater_control_tick();
    }
}

// ============================================================
// MAX31865 PT100 (hardware SPI)
// ============================================================
#define MAX31865_REG_CONFIG     0x00
#define MAX31865_REG_RTD_MSB    0x01
#define MAX31865_REG_FAULT      0x07
#define MAX31865_WRITE_BIT      0x80

#define MAX31865_CFG_VBIAS      0x80
#define MAX31865_CFG_AUTO       0x40
#define MAX31865_CFG_3WIRE      0x10
#define MAX31865_CFG_FAULT_CLR  0x02
#define MAX31865_CFG_50HZ       0x01

#if RTD_WIRES == 3
  #define MAX31865_CFG_WIREBIT  MAX31865_CFG_3WIRE
#else
  #define MAX31865_CFG_WIREBIT  0
#endif

#if RTD_FILTER_50HZ
  #define MAX31865_CFG_FILTER   MAX31865_CFG_50HZ
#else
  #define MAX31865_CFG_FILTER   0
#endif

#define MAX31865_CFG_NORMAL \
    (MAX31865_CFG_VBIAS | MAX31865_CFG_AUTO | MAX31865_CFG_WIREBIT | MAX31865_CFG_FILTER)

static esp_err_t max31865_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg | MAX31865_WRITE_BIT), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_transmit(g_rtd_spi, &t);
}

static esp_err_t max31865_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    if (len == 0 || len > 7) return ESP_ERR_INVALID_SIZE;
    uint8_t tx[8] = { (uint8_t)(reg & 0x7F), 0 };
    uint8_t rx[8] = { 0 };
    spi_transaction_t t = {
        .length    = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(g_rtd_spi, &t);
    if (err == ESP_OK) memcpy(buf, &rx[1], len);
    return err;
}

static esp_err_t max31865_configure(void)
{
    esp_err_t err = max31865_write_reg(MAX31865_REG_CONFIG, 0);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(20));

    err = max31865_write_reg(MAX31865_REG_CONFIG,
                             MAX31865_CFG_NORMAL | MAX31865_CFG_FAULT_CLR);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(80));
    return ESP_OK;
}

static esp_err_t max31865_recover(const char *reason)
{
    int64_t now = esp_timer_get_time();
    if ((now - g_temp_last_recover_us) <
        (int64_t)TEMP_RECOVER_COOLDOWN_MS * 1000LL) {
        return ESP_ERR_INVALID_STATE;
    }
    g_temp_last_recover_us = now;

    ESP_LOGW(TAG_TEMP, "MAX31865 recovery: %s", reason ? reason : "fault latch");
    esp_err_t err = max31865_configure();
    if (err != ESP_OK) {
        ESP_LOGW(TAG_TEMP, "MAX31865 recovery failed: %s", esp_err_to_name(err));
        return err;
    }

    g_temp_fault_streak = 0;
    g_tc_error = false;
    g_tc_fault_code = 0;
    ESP_LOGI(TAG_TEMP, "MAX31865 recovery complete");
    return ESP_OK;
}

static void init_max31865(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = RTD_MOSI_PIN,
        .miso_io_num   = RTD_MISO_PIN,
        .sclk_io_num   = RTD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(RTD_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = RTD_SPI_CLOCK_HZ,
        .mode           = 1,
        .spics_io_num   = RTD_CS_PIN,
        .queue_size     = 1,
        .cs_ena_pretrans  = 4,
        .cs_ena_posttrans = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(RTD_SPI_HOST, &dev_cfg, &g_rtd_spi));

    ESP_ERROR_CHECK(max31865_configure());

    ESP_LOGI(TAG_TEMP,
             "MAX31865 ready | CLK=%d MISO=%d MOSI=%d CS=%d | %d-wire Rref=%.0fΩ R0=%.0fΩ",
             RTD_CLK_PIN, RTD_MISO_PIN, RTD_MOSI_PIN, RTD_CS_PIN,
             RTD_WIRES, RTD_RREF_OHM, RTD_NOMINAL_OHM);
}

static float rtd_ohms_to_celsius(float Rrtd)
{
    const float A = 3.9083e-3f;
    const float B = -5.775e-7f;
    const float R0 = RTD_NOMINAL_OHM;

    float Z1 = -A;
    float Z2 = A * A - 4.0f * B;
    float Z3 = 4.0f * B / R0;
    float Z4 = 2.0f * B;
    float t_pos = (Z1 + sqrtf(Z2 + Z3 * Rrtd)) / Z4;
    if (t_pos >= 0.0f) return t_pos;

    float r  = Rrtd * (100.0f / R0);
    float r2 = r * r;
    float r3 = r2 * r;
    float r4 = r3 * r;
    float r5 = r4 * r;
    return -242.02f + 2.2228f * r + 2.5859e-3f * r2
           - 4.8260e-6f * r3 - 2.8183e-8f * r4 + 1.5243e-10f * r5;
}

static esp_err_t max31865_read_temperature_once(float *out_temp_c, uint8_t *out_fault)
{
    if (out_temp_c == NULL || out_fault == NULL) return ESP_ERR_INVALID_ARG;
    *out_temp_c = -999.0f;
    *out_fault = 0;

    uint8_t rx[2];
    if (max31865_read_bytes(MAX31865_REG_RTD_MSB, rx, 2) != ESP_OK) {
        *out_fault = 0xFF;
        return ESP_FAIL;
    }

    uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];

    if (raw & 0x0001) {
        uint8_t fault = 0;
        if (max31865_read_bytes(MAX31865_REG_FAULT, &fault, 1) != ESP_OK) {
            fault = 0xFF;
        }
        max31865_write_reg(MAX31865_REG_CONFIG,
                           MAX31865_CFG_NORMAL | MAX31865_CFG_FAULT_CLR);
        *out_fault = fault;
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t rtd_raw = raw >> 1;
    float Rrtd = (float)rtd_raw * RTD_RREF_OHM / 32768.0f;
    float temp_c = rtd_ohms_to_celsius(Rrtd) + g_temp_offset_c;
    if (!isfinite(temp_c)) {
        *out_fault = 0xFE;
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out_temp_c = temp_c;
    return ESP_OK;
}

static float max31865_read_temperature(void)
{
    float candidate = -999.0f;
    uint8_t fault = 0xFF;
    esp_err_t err = ESP_FAIL;

    for (int attempt = 1; attempt <= TEMP_SPI_READ_RETRIES; attempt++) {
        err = max31865_read_temperature_once(&candidate, &fault);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (err == ESP_OK &&
        (candidate < TEMP_MIN_VALID_C || candidate > TEMP_MAX_VALID_C)) {
        fault = 0xFE;
        err = ESP_ERR_INVALID_RESPONSE;
    }

    if (err == ESP_OK && g_temperature_valid &&
        fabsf(candidate - g_temperature) > TEMP_MAX_STEP_C &&
        candidate < HEATER_MAX_TEMP_C) {
        fault = 0xFE;
        err = ESP_ERR_INVALID_RESPONSE;
    }

    if (err != ESP_OK) {
        if (g_temp_fault_streak < UINT8_MAX) g_temp_fault_streak++;
        if (!g_temperature_valid ||
            g_temp_fault_streak >= TEMP_FAULT_CONFIRM_COUNT) {
            if (!g_tc_error || fault != g_tc_fault_code) {
                ESP_LOGW(TAG_TEMP, "MAX31865 fault 0x%02X (%s)",
                         fault, esp_err_to_name(err));
            }
            g_tc_error = true;
            g_tc_fault_code = fault;
        } else {
            ESP_LOGD(TAG_TEMP, "ignored transient MAX31865 fault 0x%02X (%u/%u)",
                     fault, g_temp_fault_streak, TEMP_FAULT_CONFIRM_COUNT);
        }
        if (g_temp_fault_streak >= TEMP_RECOVER_FAULT_COUNT) {
            max31865_recover("persistent read/fault state");
        }
        return g_temperature_valid ? g_temperature : -999.0f;
    }

    if (g_tc_error) ESP_LOGI(TAG_TEMP, "MAX31865 recovered");
    g_temp_fault_streak = 0;
    g_tc_error = false;
    g_tc_fault_code = 0;
    g_temperature_valid = true;
    return candidate;
}

// ============================================================
// PCF8563 RTC (I2C)
// ============================================================
#define PCF_REG_CTRL1       0x00
#define PCF_REG_VL_SECONDS  0x02

static uint8_t bcd2bin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t bin2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static esp_err_t pcf8563_write(uint8_t reg, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8563_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(RTC_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t pcf8563_read(uint8_t reg, uint8_t *data, size_t len)
{
    if (len == 0) return ESP_ERR_INVALID_SIZE;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8563_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8563_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(RTC_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

// Century convention used here: C=0 → 2000-2099, C=1 → 1900-1999.
static esp_err_t rtc_read_tm(struct tm *out)
{
    uint8_t buf[7];
    esp_err_t err = pcf8563_read(PCF_REG_VL_SECONDS, buf, 7);
    if (err != ESP_OK) return err;
    if (buf[0] & 0x80) return ESP_ERR_INVALID_STATE;

    out->tm_sec  = bcd2bin(buf[0] & 0x7F);
    out->tm_min  = bcd2bin(buf[1] & 0x7F);
    out->tm_hour = bcd2bin(buf[2] & 0x3F);
    out->tm_mday = bcd2bin(buf[3] & 0x3F);
    out->tm_wday = buf[4] & 0x07;
    out->tm_mon  = bcd2bin(buf[5] & 0x1F) - 1;
    int century_19xx = (buf[5] & 0x80) ? 1 : 0;
    int year_in_century = bcd2bin(buf[6]);
    int full_year = (century_19xx ? 1900 : 2000) + year_in_century;
    out->tm_year = full_year - 1900;
    out->tm_isdst = 0;
    return ESP_OK;
}

static esp_err_t rtc_write_tm(const struct tm *in)
{
    int full_year = in->tm_year + 1900;
    uint8_t century_bit = (full_year < 2000) ? 0x80 : 0x00;
    int year_in_century = full_year - (century_bit ? 1900 : 2000);

    uint8_t buf[7];
    buf[0] = bin2bcd(in->tm_sec) & 0x7F;
    buf[1] = bin2bcd(in->tm_min) & 0x7F;
    buf[2] = bin2bcd(in->tm_hour) & 0x3F;
    buf[3] = bin2bcd(in->tm_mday) & 0x3F;
    buf[4] = (uint8_t)(in->tm_wday & 0x07);
    buf[5] = (uint8_t)(bin2bcd(in->tm_mon + 1) | century_bit);
    buf[6] = bin2bcd((uint8_t)year_in_century);
    return pcf8563_write(PCF_REG_VL_SECONDS, buf, 7);
}

static void init_rtc(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = RTC_SDA_PIN,
        .scl_io_num       = RTC_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = RTC_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(RTC_I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(RTC_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    uint8_t zero[2] = { 0x00, 0x00 };
    esp_err_t err = pcf8563_write(PCF_REG_CTRL1, zero, 2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_RTC, "PCF8563 not responding (err=0x%x) — check wiring/pullups", err);
        return;
    }

    struct tm tm_rtc = {0};
    err = rtc_read_tm(&tm_rtc);
    if (err == ESP_OK) {
        setenv("TZ", "UTC-8", 1);
        tzset();
        struct timeval tv = { .tv_sec = mktime(&tm_rtc), .tv_usec = 0 };
        settimeofday(&tv, NULL);
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_rtc);
        ESP_LOGI(TAG_RTC, "PCF8563 init | SCL=%d SDA=%d | time=%s",
                 RTC_SCL_PIN, RTC_SDA_PIN, buf);
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG_RTC,
                 "PCF8563 reports voltage-low (battery dead?) — use `settime` to initialize");
    } else {
        ESP_LOGW(TAG_RTC, "PCF8563 read failed: 0x%x", err);
    }
}

// ============================================================
// Telnet helpers
// ============================================================

static void telnet_send(int fd, const char *msg)
{
    if (fd >= 0) send(fd, msg, strlen(msg), 0);
}

static void telnet_sendf(int fd, const char *fmt, ...)
{
    char buf[320];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    telnet_send(fd, buf);
}

static void telnet_prompt(int fd) { telnet_send(fd, "\r\nesp32# "); }

// ============================================================
// Telnet commands
// ============================================================

static bool softanti_active(void);
static uint32_t softanti_cool_probe_ms(void);
static bool softanti_estimated_temp(float *out_temp_c);

static void print_help(int fd)
{
    telnet_send(fd, "+------------------------------+-----------------------------------+\r\n");
    telnet_send(fd, "| allon / alloff               | motor+heat-auto on / all off      |\r\n");
    telnet_send(fd, "| motoron / motoroff           | motor at current duty / off       |\r\n");
    telnet_send(fd, "| setduty <-100..100>          | motor duty cycle (%% of full)     |\r\n");
    telnet_send(fd, "| heaton / heatoff             | manual heater override            |\r\n");
    telnet_send(fd, "| heatauto / heatmanual        | switch heater control mode        |\r\n");
    telnet_send(fd, "| settemp <C>                  | target temperature (enables auto) |\r\n");
    telnet_send(fd, "| settempband <C>              | hysteresis half-width             |\r\n");
    telnet_send(fd, "| settempoffset <C>            | add offset to RTD reading         |\r\n");
    telnet_send(fd, "| setquiet <w> <c> <step> <pause_s> | tune quiet sampler     |\r\n");
    telnet_send(fd, "| profiles                     | list material cycle profiles      |\r\n");
    telnet_send(fd, "| setprofile <slot> <name> <wait_s> <run_s> <temp_C> |\r\n");
    telnet_send(fd, "| useprofile <slot|name|none>  | select active material profile    |\r\n");
    telnet_send(fd, "| driverquiet                  | show driver quiet-poll settings   |\r\n");
    telnet_send(fd, "| setdriverquiet <offline_s> <pause_s> | pause for driver info|\r\n");
    telnet_send(fd, "| rtdreset                     | recover MAX31865 after fault latch|\r\n");
    telnet_send(fd, "| ls                           | full snapshot inc. driver regs    |\r\n");
    telnet_send(fd, "| top                          | repeat ls until quit              |\r\n");
    telnet_send(fd, "| time                         | show RTC time                     |\r\n");
    telnet_send(fd, "| settime YYYY-MM-DD HH:MM:SS  | set RTC + system time             |\r\n");
    telnet_send(fd, "| help                         | show this help                    |\r\n");
    telnet_send(fd, "+------------------------------+-----------------------------------+\r\n");
}

static void print_time(int fd)
{
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_now);
    telnet_sendf(fd, "[TIME] %s\r\n", buf);
}

static void print_profiles(int fd)
{
    telnet_send(fd, ANSI_CYAN "---- Material Cycle Profiles ----" ANSI_RESET "\r\n");
    for (int i = 0; i < WAIT_PROFILE_COUNT; i++) {
        telnet_sendf(fd, "  %s%u: %-15s wait=%.1fs run=%.1fs target=%.1fC%s\r\n",
                     (i == g_active_wait_profile) ? ANSI_GREEN : "",
                     (unsigned)i,
                     g_wait_profiles[i].name,
                     g_wait_profiles[i].wait_ms / 1000.0f,
                     g_wait_profiles[i].run_ms / 1000.0f,
                     g_wait_profiles[i].target_temp_c,
                     (i == g_active_wait_profile) ? "  ACTIVE" ANSI_RESET : "");
    }
    telnet_send(fd, "  edit : setprofile <slot 1..5> <name> <wait_s> <run_s> <target_C>  (also activates)\r\n");
    telnet_send(fd, "  use  : useprofile <slot|name|none>\r\n");
}

static char status_graph_state_char(void)
{
    if (g_tc_error || !g_temperature_valid) return 'F';
    if (g_softanti_paused) return 'P';
    if (g_motor_on && g_heater_on) return 'H';
    if (g_motor_on) return 'M';
    if (g_heater_on) return 'h';
    return '.';
}

static const char *status_graph_color(char c)
{
    switch (c) {
    case 'F': return ANSI_RED;
    case 'P': return ANSI_YELLOW;
    case 'H': return ANSI_RED;
    case 'M': return ANSI_GREEN;
    case 'h': return ANSI_RED;
    default:  return ANSI_DIM;
    }
}

static const char *status_graph_symbol(char c)
{
    switch (c) {
    case 'F': return "!";
    case 'P': return "▣";
    case 'H': return "█";
    case 'M': return "▰";
    case 'h': return "▮";
    default:  return "·";
    }
}

static void status_graph_push(char c)
{
    g_status_graph[g_status_graph_pos] = c;
    g_status_graph_pos = (uint8_t)((g_status_graph_pos + 1) % STATUS_GRAPH_WIDTH);
    if (g_status_graph_count < STATUS_GRAPH_WIDTH) g_status_graph_count++;
}

static void print_status_graph(int fd)
{
    status_graph_push(status_graph_state_char());
    telnet_send(fd, "  Graph      : ");
    uint8_t start = (uint8_t)((g_status_graph_pos + STATUS_GRAPH_WIDTH -
                               g_status_graph_count) % STATUS_GRAPH_WIDTH);
    for (uint8_t i = 0; i < g_status_graph_count; i++) {
        char c = g_status_graph[(start + i) % STATUS_GRAPH_WIDTH];
        telnet_send(fd, status_graph_color(c));
        telnet_send(fd, status_graph_symbol(c));
    }
    telnet_send(fd, ANSI_RESET "  " ANSI_DIM "!=fault ▣=pause █=heat+run ▰=run ▮=heat ·=idle" ANSI_RESET "\r\n");
}

static void print_status(int fd)
{
    telnet_send(fd, ANSI_CYAN "-------- System Status --------" ANSI_RESET "\r\n");
    float estimated_c = 0.0f;
    bool has_estimate = softanti_estimated_temp(&estimated_c);
    if (!g_temperature_valid) {
        telnet_sendf(fd, "  RTD C4     : " ANSI_YELLOW "[NO VALID SAMPLE, fault 0x%02X]" ANSI_RESET "\r\n",
                     g_tc_fault_code);
    } else if (g_tc_error) {
        telnet_sendf(fd, "  RTD C4     : " ANSI_RED "[FAULT 0x%02X]" ANSI_RESET " last %.2f C\r\n",
                     g_tc_fault_code, g_temperature);
    } else if (has_estimate) {
        telnet_sendf(fd, "  RTD C4     : " ANSI_GREEN "%.2f C" ANSI_RESET
                         "  est %.2f C  " ANSI_DIM "(quiet model)" ANSI_RESET "\r\n",
                     g_temperature, estimated_c);
    } else {
        telnet_sendf(fd, "  RTD C4     : " ANSI_GREEN "%.2f C" ANSI_RESET "\r\n", g_temperature);
    }
    telnet_sendf(fd, "  Temp offset: %+.2f C\r\n", g_temp_offset_c);
    const char *heater_note = "";
    if (g_heater_mode == HEATER_AUTO && softanti_active()) {
        heater_note = has_estimate ? " [MODEL]" : " [WAIT CLEAN RTD]";
    } else if (g_heater_mode == HEATER_AUTO && (!g_temperature_valid || g_tc_error)) {
        heater_note = " [RTD FAULT BLOCK]";
    }
    telnet_sendf(fd, "  Heater     : %s%s" ANSI_RESET "  mode=%s  control=%s  target=%.1f±%.1f °C%s\r\n",
                 g_heater_on ? ANSI_RED : ANSI_DIM,
                 g_heater_on ? "ON" : "OFF",
                 g_heater_mode == HEATER_AUTO ? "AUTO" : "MANUAL",
                 g_heater_control_enabled ? "enabled" : "idle",
                 g_target_temp_c, g_temp_band_c, heater_note);
    telnet_sendf(fd, "  Motor cmd  : %s%s" ANSI_RESET "  duty=%.1f%%\r\n",
                 g_motor_on ? ANSI_GREEN : ANSI_DIM,
                 g_motor_on ? "RUN" : "STOP", g_target_duty / 10.0f);
    telnet_sendf(fd, "  Quiet RTD  : %s%s" ANSI_RESET "  allon=%s  warm=%.3fC/s cool=%.3fC/s step=%.1fC pause=%.1fs probe=%.1fs%s\r\n",
                 softanti_active() ? ANSI_GREEN : ANSI_DIM,
                 softanti_active() ? "ACTIVE" : "IDLE",
                 g_allstart_session ? "yes" : "no",
                 g_softanti_warm_dps, g_softanti_cool_dps, g_softanti_step_c,
                 g_softanti_pause_ms / 1000.0f,
                 softanti_cool_probe_ms() / 1000.0f,
                 g_softanti_paused ? " [PAUSED]" : "");
    telnet_sendf(fd, "  Material   : %s  wait=%.1fs run=%.1fs target=%.1fC%s\r\n",
                 g_wait_profiles[g_active_wait_profile].name,
                 wait_profile_active_ms() / 1000.0f,
                 wait_profile_active_run_ms() / 1000.0f,
                 g_wait_profiles[g_active_wait_profile].target_temp_c,
                 softanti_active() ? " (inside quiet loop)" : "");
    telnet_sendf(fd, "  Quiet poll : after=%us pause=%.1fs%s\r\n",
                 (unsigned)g_driver_telem_pause_fails,
                 g_driver_telem_pause_ms / 1000.0f,
                 softanti_active() ? " (quiet loop handles this)" : "");
    print_status_graph(fd);

    if (g_drv_read_ok || g_drv_last_ok_us != 0) {
        uint16_t rpm = g_drv_rpm_scaled ? (uint16_t)(g_drv_rpm * 10) : g_drv_rpm;
        const char *err_txt = (g_drv_error < sizeof(MOTOR_ERROR_TEXT) / sizeof(MOTOR_ERROR_TEXT[0]))
                              ? MOTOR_ERROR_TEXT[g_drv_error] : "unknown";
        if (g_drv_read_ok) {
            telnet_sendf(fd, "  Driver     : " ANSI_GREEN "online" ANSI_RESET "  err=%u (%s)\r\n",
                         g_drv_error, err_txt);
        } else {
            long long stale_s = (esp_timer_get_time() - g_drv_last_ok_us) / 1000000LL;
            telnet_sendf(fd, "  Driver     : " ANSI_YELLOW "telemetry stale %llds" ANSI_RESET "  err=%u (%s)\r\n",
                         stale_s, g_drv_error, err_txt);
        }
        telnet_sendf(fd, "  Driver PWM : %.1f%%  current=%.2fA\r\n",
                     g_drv_pwm_pm * 0.1f, g_drv_current_cA * 0.01f);
        telnet_sendf(fd, "  Driver RPM : %u  commutation=%.1f Hz\r\n",
                     rpm, g_drv_freq * 0.1f);
        telnet_sendf(fd, "  Driver temp: %.1f C  supply=%.1f V\r\n",
                     g_drv_temp_dC * 0.1f, g_drv_supply_dV * 0.1f);
    } else {
        telnet_send(fd, "  Driver     : " ANSI_RED "[no response - check DIP5=ON, wiring, slave addr]" ANSI_RESET "\r\n");
    }
    print_time(fd);
    telnet_send(fd, ANSI_CYAN "-------------------------------" ANSI_RESET "\r\n");
}

static void cmd_settime(int fd, const char *arg)
{
    struct tm tm_in = {0};
    int y, mo, d, h, mi, s;
    if (sscanf(arg, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) != 6) {
        telnet_send(fd, "[ERR] format: settime YYYY-MM-DD HH:MM:SS\r\n");
        return;
    }
    if (y < 1970 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
        h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59) {
        telnet_send(fd, "[ERR] value out of range\r\n");
        return;
    }
    tm_in.tm_year = y - 1900;
    tm_in.tm_mon  = mo - 1;
    tm_in.tm_mday = d;
    tm_in.tm_hour = h;
    tm_in.tm_min  = mi;
    tm_in.tm_sec  = s;
    tm_in.tm_isdst = 0;

    time_t epoch = mktime(&tm_in);
    if (epoch == (time_t)-1) { telnet_send(fd, "[ERR] invalid date\r\n"); return; }

    if (rtc_write_tm(&tm_in) != ESP_OK) {
        telnet_send(fd, "[ERR] RTC write failed\r\n");
        return;
    }
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    telnet_sendf(fd, "[OK] time set to %04d-%02d-%02d %02d:%02d:%02d\r\n", y, mo, d, h, mi, s);
}

// ============================================================
// Status stream
// ============================================================

static bool status_stream_socket_timeout(void)
{
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
}

static void status_stream_discard_iac(int fd, bool *disconnected)
{
    uint8_t verb;
    int n = recv(fd, &verb, 1, 0);
    if (n == 0) { *disconnected = true; return; }
    if (n < 0) { if (!status_stream_socket_timeout()) *disconnected = true; return; }

    if (verb == 0xFA) {
        uint8_t prev = 0;
        for (int i = 0; i < 64; i++) {
            uint8_t b;
            n = recv(fd, &b, 1, 0);
            if (n == 0) { *disconnected = true; return; }
            if (n < 0) { if (!status_stream_socket_timeout()) *disconnected = true; return; }
            if (prev == 0xFF && b == 0xF0) return;   // IAC SE
            prev = b;
        }
    } else if (verb >= 0xFB && verb <= 0xFE) {
        uint8_t opt;
        n = recv(fd, &opt, 1, 0);
        if (n == 0) *disconnected = true;
        else if (n < 0 && !status_stream_socket_timeout()) *disconnected = true;
    }
}

static bool status_stream_quit_requested(int fd, bool *disconnected)
{
    *disconnected = false;
    uint8_t byte;
    int n = recv(fd, &byte, 1, 0);
    if (n > 0) {
        if (byte == 0xFF) {
            status_stream_discard_iac(fd, disconnected);
            return false;
        }
        return byte == 'q' || byte == 'Q' || byte == '\r' ||
               byte == '\n' || byte == 0x03;
    }
    if (n == 0) {
        *disconnected = true;
    } else if (!status_stream_socket_timeout()) {
        *disconnected = true;
    }
    return false;
}

static void status_stream_run(int fd)
{
    struct timeval old_tv = {0};
    socklen_t old_tv_len = sizeof(old_tv);
    bool have_old_tv = (getsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                                   &old_tv, &old_tv_len) == 0);
    struct timeval tv = {
        .tv_sec = STATUS_STREAM_INTERVAL_MS / 1000,
        .tv_usec = (STATUS_STREAM_INTERVAL_MS % 1000) * 1000,
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    telnet_sendf(fd, "[top] repeating every %.1f s; press q, Enter, or Ctrl-C to quit\r\n",
                 STATUS_STREAM_INTERVAL_MS / 1000.0f);

    while (1) {
        print_status(fd);
        telnet_send(fd, ANSI_DIM "[top] q/Enter/Ctrl-C to quit" ANSI_RESET "\r\n\r\n");

        bool disconnected = false;
        if (status_stream_quit_requested(fd, &disconnected) || disconnected) break;
    }

    if (have_old_tv) {
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &old_tv, sizeof(old_tv));
    } else {
        tv.tv_sec = 0; tv.tv_usec = 0;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    telnet_send(fd, "[top closed]\r\n");
}

static void softanti_schedule_next(void)
{
    g_softanti_next_sample_us = esp_timer_get_time();
}

static bool softanti_active(void)
{
    return g_allstart_session && g_motor_on && !g_motor_stop_pending;
}

static void softanti_reset_model(void)
{
    g_softanti_paused = false;
    g_softanti_model_valid = false;
    g_softanti_last_sample_us = 0;
    g_softanti_next_sample_us = 0;
}

static bool softanti_estimated_temp(float *out_temp_c)
{
    if (!softanti_active() || !g_softanti_model_valid ||
        g_softanti_last_sample_us == 0 || out_temp_c == NULL) {
        return false;
    }

    float dt_s = (esp_timer_get_time() - g_softanti_last_sample_us) / 1000000.0f;
    if (dt_s < 0.0f) dt_s = 0.0f;

    float rate = g_softanti_heating_model ? g_softanti_warm_dps : g_softanti_cool_dps;
    float estimate = g_softanti_last_sample_c +
                     (g_softanti_heating_model ? rate * dt_s : -rate * dt_s);

    float lower = g_target_temp_c - g_temp_band_c;
    float upper = g_target_temp_c + g_temp_band_c;
    if (g_softanti_heating_model && estimate > upper) estimate = upper;
    if (!g_softanti_heating_model && estimate < lower) estimate = lower;

    *out_temp_c = estimate;
    return true;
}

static bool softanti_pause_wait_ms(uint32_t wait_ms)
{
    uint32_t remaining = wait_ms;
    while (remaining > 0) {
        if (!softanti_active()) return false;
        uint32_t slice = (remaining > WAIT_PROFILE_CHUNK_MS) ?
                         WAIT_PROFILE_CHUNK_MS : remaining;
        vTaskDelay(pdMS_TO_TICKS(slice));
        remaining -= slice;
    }
    return softanti_active();
}

static uint32_t softanti_cool_probe_ms(void)
{
    float probe_c = g_softanti_step_c * SOFTANTI_COOL_PROBE_STEP_FRAC;
    if (probe_c < SOFTANTI_COOL_PROBE_MIN_C) probe_c = SOFTANTI_COOL_PROBE_MIN_C;
    if (probe_c > SOFTANTI_COOL_PROBE_MAX_C) probe_c = SOFTANTI_COOL_PROBE_MAX_C;

    float rate = g_softanti_cool_dps;
    if (rate < SOFTANTI_MIN_RATE_DPS) rate = SOFTANTI_MIN_RATE_DPS;

    uint32_t probe_ms = (uint32_t)((probe_c / rate) * 1000.0f);
    if (probe_ms < SOFTANTI_COOL_PROBE_MIN_MS) probe_ms = SOFTANTI_COOL_PROBE_MIN_MS;
    if (probe_ms > SOFTANTI_COOL_PROBE_MAX_MS) probe_ms = SOFTANTI_COOL_PROBE_MAX_MS;
    return probe_ms;
}

static uint32_t softanti_next_interval_ms(float temp_c, bool heating)
{
    float lower = g_target_temp_c - g_temp_band_c;
    float upper = g_target_temp_c + g_temp_band_c;
    float delta_c = g_softanti_step_c;

    if (heating && temp_c < upper) {
        float to_upper = upper - temp_c;
        if (to_upper < delta_c) delta_c = to_upper;
    } else if (!heating && temp_c > lower) {
        float to_lower = temp_c - lower;
        if (to_lower < delta_c) delta_c = to_lower;
    }
    if (delta_c < SOFTANTI_MIN_STEP_C) delta_c = SOFTANTI_MIN_STEP_C;

    float rate = heating ? g_softanti_warm_dps : g_softanti_cool_dps;
    if (rate < SOFTANTI_MIN_RATE_DPS) rate = SOFTANTI_MIN_RATE_DPS;

    uint32_t interval_ms = (uint32_t)((delta_c / rate) * 1000.0f);
    if (interval_ms < SOFTANTI_MIN_INTERVAL_MS) interval_ms = SOFTANTI_MIN_INTERVAL_MS;
    if (interval_ms > SOFTANTI_MAX_INTERVAL_MS) interval_ms = SOFTANTI_MAX_INTERVAL_MS;
    return interval_ms;
}

static void softanti_calibrate(float sample_c, int64_t sample_us)
{
    if (g_softanti_last_sample_us == 0) return;

    float dt_s = (sample_us - g_softanti_last_sample_us) / 1000000.0f;
    if (dt_s < 1.0f) return;

    float delta_c = sample_c - g_softanti_last_sample_c;
    if (g_softanti_heating_model && delta_c > 0.05f) {
        float measured = delta_c / dt_s;
        if (measured >= SOFTANTI_MIN_RATE_DPS && measured <= SOFTANTI_MAX_RATE_DPS) {
            g_softanti_warm_dps = g_softanti_warm_dps * 0.70f + measured * 0.30f;
            ESP_LOGI(TAG_TEMP,
                     "softanti calibrate warm: measured=%.4f C/s new=%.4f C/s dt=%.1fs dT=%.2f",
                     measured, g_softanti_warm_dps, dt_s, delta_c);
        }
    } else if (!g_softanti_heating_model && delta_c < -0.05f) {
        float measured = -delta_c / dt_s;
        if (measured >= SOFTANTI_MIN_RATE_DPS && measured <= SOFTANTI_MAX_RATE_DPS) {
            g_softanti_cool_dps = g_softanti_cool_dps * 0.70f + measured * 0.30f;
            ESP_LOGI(TAG_TEMP,
                     "softanti calibrate cool: measured=%.4f C/s new=%.4f C/s dt=%.1fs dT=%.2f",
                     measured, g_softanti_cool_dps, dt_s, delta_c);
        }
    }
}

static void softanti_calibrate_cool_probe(float start_c, int64_t start_us,
                                          float end_c, int64_t end_us)
{
    float dt_s = (end_us - start_us) / 1000000.0f;
    if (dt_s < 1.0f) return;

    float delta_c = start_c - end_c;
    if (delta_c <= 0.02f) return;

    float measured = delta_c / dt_s;
    if (measured >= SOFTANTI_MIN_RATE_DPS && measured <= SOFTANTI_MAX_RATE_DPS) {
        g_softanti_cool_dps = g_softanti_cool_dps * 0.70f + measured * 0.30f;
        ESP_LOGI(TAG_TEMP,
                 "softanti cool-probe: start=%.2fC end=%.2fC measured=%.4f C/s new=%.4f C/s dt=%.1fs",
                 start_c, end_c, measured, g_softanti_cool_dps, dt_s);
    }
}

static void softanti_plan_from_sample(float sample_c, int64_t sample_us)
{
    if (!g_heater_control_enabled) {
        heater_set_relay(false);
    } else if (g_heater_mode == HEATER_AUTO) {
        bool heat_on = (sample_c < g_target_temp_c);
        if (sample_c >= HEATER_MAX_TEMP_C) heat_on = false;
        heater_set_relay(heat_on);
    } else if (sample_c >= HEATER_MAX_TEMP_C) {
        heater_set_relay(false);
    }

    g_softanti_heating_model = g_heater_on;
    g_softanti_last_sample_c = sample_c;
    g_softanti_last_sample_us = sample_us;
    g_softanti_model_valid = true;

    uint32_t interval_ms = wait_profile_active_run_ms();
    if (interval_ms == 0) {
        interval_ms = softanti_next_interval_ms(sample_c, g_softanti_heating_model);
    }
    g_softanti_next_sample_us = sample_us + (int64_t)interval_ms * 1000LL;

    ESP_LOGI(TAG_TEMP,
             "softanti sample: T=%.2fC target=%.1f heater=%s next-run=%ums profile=%s warm=%.4f cool=%.4f step=%.1f pause=%ums",
             sample_c, g_target_temp_c, g_softanti_heating_model ? "ON" : "OFF",
             (unsigned)interval_ms, g_wait_profiles[g_active_wait_profile].name,
             g_softanti_warm_dps, g_softanti_cool_dps,
             g_softanti_step_c, (unsigned)g_softanti_pause_ms);
}

static bool softanti_sample_profile_wait(uint32_t wait_ms,
                                         float *last_c,
                                         int64_t *last_us,
                                         bool *last_ok)
{
    uint32_t remaining = wait_ms;
    while (remaining > 0) {
        if (!softanti_active()) return false;

        uint32_t slice = (remaining > TEMP_READ_INTERVAL) ?
                         TEMP_READ_INTERVAL : remaining;
        vTaskDelay(pdMS_TO_TICKS(slice));
        remaining -= slice;

        if (!softanti_active()) return false;

        g_temperature = max31865_read_temperature();
        int64_t sample_us = esp_timer_get_time();
        bool sample_ok = g_temperature_valid && !g_tc_error &&
                         g_temp_fault_streak == 0;

        if (last_c != NULL) *last_c = g_temperature;
        if (last_us != NULL) *last_us = sample_us;
        if (last_ok != NULL) *last_ok = sample_ok;

        if (sample_ok) {
            softanti_calibrate(g_temperature, sample_us);
            softanti_plan_from_sample(g_temperature, sample_us);
        } else {
            heater_set_relay(false);
            ESP_LOGW(TAG_TEMP, "profile wait RTD sample failed; heater OFF");
        }
    }
    return true;
}

static void handle_command(int fd, const char *cmd)
{
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    if (*cmd == '\0') return;

    ESP_LOGI(TAG_TEL, "cmd: %s", cmd);

    if (strcmp(cmd, "allon") == 0 || strcmp(cmd, "allstart") == 0) {
        if (!g_motor_on) {
            g_temperature = max31865_read_temperature();
        }
        esp_err_t err = motor_start();
        if (err == ESP_OK) {
            wait_profile_apply_active();
            g_heater_control_enabled = true;
            g_allstart_session = true;
            softanti_reset_model();
            softanti_schedule_next();
            heater_apply_session_start();
            telnet_sendf(fd,
                         "[OK] motor started + heater %s (%s) + quiet RTD profile=%s wait=%.1fs run=%.1fs target=%.1fC\r\n",
                         heater_mode_name(g_heater_mode),
                         g_heater_on ? "ON" : "waiting RTD/sample",
                         g_wait_profiles[g_active_wait_profile].name,
                         wait_profile_active_ms() / 1000.0f,
                         wait_profile_active_run_ms() / 1000.0f,
                         g_target_temp_c);
        } else {
            telnet_sendf(fd, "[ERR] motor start failed: %s; heater unchanged\r\n",
                         esp_err_to_name(err));
        }

    } else if (strcmp(cmd, "alloff") == 0 || strcmp(cmd, "allstop") == 0) {
        esp_err_t err = motor_stop();
        g_heater_mode = HEATER_AUTO;
        g_heater_control_enabled = false;
        heater_set_relay(false);
        softanti_reset_model();
        max31865_recover("alloff requested");
        if (err == ESP_OK) {
            telnet_send(fd, "[OK] motor stopped + heater OFF (AUTO idle)\r\n");
        } else {
            telnet_sendf(fd, "[WARN] heater OFF; motor stop failed: %s\r\n",
                         esp_err_to_name(err));
        }

    } else if (strcmp(cmd, "motoron") == 0 || strcmp(cmd, "motorstart") == 0) {
        esp_err_t err = motor_start();
        if (err == ESP_OK) {
            g_allstart_session = false;
            softanti_reset_model();
            telnet_sendf(fd, "[OK] motor running at duty %.1f%%\r\n", g_target_duty / 10.0f);
        } else {
            telnet_sendf(fd, "[ERR] motor start failed: %s\r\n", esp_err_to_name(err));
        }

    } else if (strcmp(cmd, "motoroff") == 0 || strcmp(cmd, "motorstop") == 0) {
        esp_err_t err = motor_stop();
        softanti_reset_model();
        max31865_recover("motoroff requested");
        if (err == ESP_OK) {
            telnet_send(fd, "[OK] motor stopped\r\n");
        } else {
            telnet_sendf(fd, "[ERR] motor stop failed: %s\r\n", esp_err_to_name(err));
        }

    } else if (strncmp(cmd, "setduty ", 8) == 0) {
        int duty_pct = atoi(cmd + 8);
        if (duty_pct < -100 || duty_pct > 100) {
            telnet_send(fd, "[ERR] duty out of range (-100..100)\r\n");
        } else {
            g_target_duty = duty_pct * 10;
            config_save();
            if (g_motor_on) {
                esp_err_t err = motor_apply_duty(g_target_duty);
                if (err == ESP_OK) {
                    telnet_sendf(fd, "[OK] target duty = %d%% (saved/applied)\r\n", duty_pct);
                } else {
                    telnet_sendf(fd, "[WARN] target duty saved, apply failed: %s\r\n",
                                 esp_err_to_name(err));
                }
            } else {
                telnet_sendf(fd, "[OK] target duty = %d%% (saved)\r\n", duty_pct);
            }
        }

    } else if (strcmp(cmd, "heaton") == 0) {
        heater_manual_on();
        telnet_send(fd, "[OK] heater MANUAL ON\r\n");

    } else if (strcmp(cmd, "heatoff") == 0) {
        heater_manual_off();
        telnet_send(fd, "[OK] heater MANUAL OFF\r\n");

    } else if (strcmp(cmd, "heatauto") == 0) {
        g_heater_mode = HEATER_AUTO;
        g_heater_control_enabled = true;
        telnet_sendf(fd, "[OK] heater AUTO enabled, target=%.1f±%.1f°C\r\n",
                     g_target_temp_c, g_temp_band_c);

    } else if (strcmp(cmd, "heatmanual") == 0) {
        g_heater_mode = HEATER_MANUAL;
        g_heater_control_enabled = g_heater_on;
        telnet_send(fd, "[OK] heater MANUAL — SSR holds current state\r\n");

    } else if (strncmp(cmd, "settemp ", 8) == 0) {
        float t = strtof(cmd + 8, NULL);
        if (t < 0.0f || t > HEATER_MAX_TEMP_C) {
            telnet_sendf(fd, "[ERR] temp out of range (0..%.0f)\r\n", HEATER_MAX_TEMP_C);
        } else {
            g_target_temp_c = t;
            g_heater_mode = HEATER_AUTO;
            g_heater_control_enabled = true;
            config_save();
            telnet_sendf(fd, "[OK] target=%.1f°C band=±%.1f°C (AUTO enabled, saved)\r\n",
                         g_target_temp_c, g_temp_band_c);
        }

    } else if (strncmp(cmd, "settempband ", 12) == 0) {
        float b = strtof(cmd + 12, NULL);
        if (b < 0.1f || b > 50.0f) {
            telnet_send(fd, "[ERR] band out of range (0.1..50)\r\n");
        } else {
            g_temp_band_c = b;
            config_save();
            telnet_sendf(fd, "[OK] hysteresis half-width=%.1f°C (saved)\r\n", b);
        }

    } else if (strcmp(cmd, "tempoffset") == 0) {
        telnet_sendf(fd, "[TEMPOFFSET] %+.2f C (applied to RTD before control/display)\r\n",
                     g_temp_offset_c);

    } else if (strncmp(cmd, "settempoffset ", 14) == 0) {
        float offset = strtof(cmd + 14, NULL);
        if (offset < TEMP_OFFSET_MIN_C || offset > TEMP_OFFSET_MAX_C) {
            telnet_sendf(fd, "[ERR] temp offset out of range (%.0f..%+.0f C)\r\n",
                         TEMP_OFFSET_MIN_C, TEMP_OFFSET_MAX_C);
        } else {
            g_temp_offset_c = offset;
            g_temperature_valid = false;
            g_tc_error = false;
            g_temp_fault_streak = 0;
            softanti_reset_model();
            if (g_allstart_session) softanti_schedule_next();
            config_save();
            telnet_sendf(fd, "[OK] temp offset=%+.2f C (saved; next RTD sample refreshes control)\r\n",
                         g_temp_offset_c);
        }

    } else if (strcmp(cmd, "profiles") == 0 ||
               strcmp(cmd, "profile") == 0 ||
               strcmp(cmd, "waitprofile") == 0) {
        print_profiles(fd);

    } else if (strncmp(cmd, "setprofile ", 11) == 0) {
        int slot = -1;
        char name[WAIT_PROFILE_NAME_LEN];
        float wait_s = -1.0f;
        float run_s = -1.0f;
        float target_c = -1.0f;
        if (sscanf(cmd + 11, "%d %15s %f %f %f",
                   &slot, name, &wait_s, &run_s, &target_c) != 5) {
            telnet_send(fd, "[ERR] format: setprofile <slot 1..5> <name> <wait_s> <run_s> <target_C>\r\n");
        } else if (slot <= 0 || slot >= WAIT_PROFILE_COUNT) {
            telnet_sendf(fd, "[ERR] slot out of range (1..%u); slot 0 is fixed as none\r\n",
                         (unsigned)(WAIT_PROFILE_COUNT - 1));
        } else if (!profile_name_valid(name) ||
                   str_eq_ci(name, "none") || str_eq_ci(name, "off")) {
            telnet_sendf(fd, "[ERR] name must be 1..%u chars: A-Z a-z 0-9 _ -\r\n",
                         (unsigned)(WAIT_PROFILE_NAME_LEN - 1));
        } else if (wait_s < 0.0f || wait_s > (WAIT_PROFILE_MAX_MS / 1000.0f) ||
                   run_s < 1.0f || run_s > (WAIT_PROFILE_MAX_MS / 1000.0f)) {
            telnet_sendf(fd, "[ERR] wait/run out of range (wait 0..%u s, run 1..%u s)\r\n",
                         (unsigned)(WAIT_PROFILE_MAX_MS / 1000U),
                         (unsigned)(WAIT_PROFILE_MAX_MS / 1000U));
        } else if (target_c < 0.0f || target_c > HEATER_MAX_TEMP_C) {
            telnet_sendf(fd, "[ERR] target temp out of range (0..%.0f C)\r\n",
                         HEATER_MAX_TEMP_C);
        } else {
            int duplicate = wait_profile_find(name);
            if (duplicate >= 0 && duplicate != slot) {
                telnet_sendf(fd, "[ERR] profile name '%s' already used by slot %d\r\n",
                             name, duplicate);
            } else {
                strncpy(g_wait_profiles[slot].name, name,
                        sizeof(g_wait_profiles[slot].name));
                g_wait_profiles[slot].name[sizeof(g_wait_profiles[slot].name) - 1] = '\0';
                g_wait_profiles[slot].wait_ms = (uint32_t)(wait_s * 1000.0f + 0.5f);
                g_wait_profiles[slot].run_ms = (uint32_t)(run_s * 1000.0f + 0.5f);
                g_wait_profiles[slot].target_temp_c = target_c;
                g_active_wait_profile = (uint8_t)slot;
                wait_profile_apply_active();
                softanti_reset_model();
                if (g_allstart_session) softanti_schedule_next();
                config_save();
                telnet_sendf(fd, "[OK] active profile %d='%s' wait=%.1fs run=%.1fs target=%.1fC mode=AUTO control=%s%s\r\n",
                             slot,
                             g_wait_profiles[slot].name,
                             g_wait_profiles[slot].wait_ms / 1000.0f,
                             g_wait_profiles[slot].run_ms / 1000.0f,
                             g_wait_profiles[slot].target_temp_c,
                             g_heater_control_enabled ? "enabled" : "idle",
                             g_allstart_session ? " (quiet loop rescheduled)" : "");
            }
        }

    } else if (strncmp(cmd, "useprofile ", 11) == 0) {
        char token[WAIT_PROFILE_NAME_LEN];
        if (sscanf(cmd + 11, "%15s", token) != 1) {
            telnet_send(fd, "[ERR] format: useprofile <slot|name|none>\r\n");
        } else {
            int slot = wait_profile_find(token);
            if (slot < 0) {
                telnet_sendf(fd, "[ERR] profile '%s' not found; use profiles\r\n", token);
            } else {
                g_active_wait_profile = (uint8_t)slot;
                wait_profile_apply_active();
                softanti_reset_model();
                if (g_allstart_session) softanti_schedule_next();
                config_save();
                telnet_sendf(fd, "[OK] active profile=%s wait=%.1fs run=%.1fs target=%.1fC mode=AUTO control=%s%s\r\n",
                             g_wait_profiles[g_active_wait_profile].name,
                             wait_profile_active_ms() / 1000.0f,
                             wait_profile_active_run_ms() / 1000.0f,
                             g_target_temp_c,
                             g_heater_control_enabled ? "enabled" : "idle",
                             g_allstart_session ? " (quiet loop rescheduled)" : "");
            }
        }

    } else if (strcmp(cmd, "driverquiet") == 0 || strcmp(cmd, "driverquiet status") == 0) {
        telnet_sendf(fd, "[DRIVERQUIET] after=%us pause=%.1fs active_when=motor-running-without-quiet-loop\r\n",
                     (unsigned)g_driver_telem_pause_fails,
                     g_driver_telem_pause_ms / 1000.0f);

    } else if (strcmp(cmd, "rtdreset") == 0) {
        esp_err_t err = max31865_recover("manual rtdreset");
        if (err == ESP_OK) {
            telnet_send(fd, "[OK] RTD/MAX31865 recovery complete\r\n");
        } else {
            telnet_sendf(fd, "[WARN] RTD/MAX31865 recovery skipped/failed: %s\r\n",
                         esp_err_to_name(err));
        }

    } else if (strncmp(cmd, "setdriverquiet ", 15) == 0) {
        float offline_s = 0.0f;
        float pause_s = 0.0f;
        if (sscanf(cmd + 15, "%f %f", &offline_s, &pause_s) != 2) {
            telnet_send(fd, "[ERR] format: setdriverquiet <offline_s> <pause_s>\r\n");
        } else {
            uint32_t fails = (uint32_t)(offline_s + 0.5f);
            uint32_t pause_ms = (uint32_t)(pause_s * 1000.0f + 0.5f);
            if (fails < DRIVER_TELEM_PAUSE_MIN_FAILS ||
                fails > DRIVER_TELEM_PAUSE_MAX_FAILS ||
                pause_ms < DRIVER_TELEM_PAUSE_MIN_MS ||
                pause_ms > DRIVER_TELEM_PAUSE_MAX_MS) {
                telnet_sendf(fd, "[ERR] range: offline %.0f..%.0f s, pause %.1f..%.1f s\r\n",
                             (float)DRIVER_TELEM_PAUSE_MIN_FAILS,
                             (float)DRIVER_TELEM_PAUSE_MAX_FAILS,
                             DRIVER_TELEM_PAUSE_MIN_MS / 1000.0f,
                             DRIVER_TELEM_PAUSE_MAX_MS / 1000.0f);
            } else {
                g_driver_telem_pause_fails = (uint8_t)fails;
                g_driver_telem_pause_ms = pause_ms;
                config_save();
                telnet_sendf(fd, "[OK] driver quiet-poll after=%us pause=%.1fs (saved)\r\n",
                             (unsigned)g_driver_telem_pause_fails,
                             g_driver_telem_pause_ms / 1000.0f);
            }
        }

    } else if (strncmp(cmd, "setquiet ", 9) == 0 ||
               strncmp(cmd, "setsoftanti ", 12) == 0) {
        const char *arg = strncmp(cmd, "setquiet ", 9) == 0 ? cmd + 9 : cmd + 12;
        float warm = 0.0f;
        float cool = 0.0f;
        float step = 0.0f;
        float pause_s = 0.0f;
        if (sscanf(arg, "%f %f %f %f", &warm, &cool, &step, &pause_s) != 4) {
            telnet_send(fd, "[ERR] format: setquiet <warm_C_per_s> <cool_C_per_s> <step_C> <pause_s>\r\n");
        } else {
            uint32_t pause_ms = (uint32_t)(pause_s * 1000.0f + 0.5f);
            if (warm < SOFTANTI_MIN_RATE_DPS || warm > SOFTANTI_MAX_RATE_DPS ||
                cool < SOFTANTI_MIN_RATE_DPS || cool > SOFTANTI_MAX_RATE_DPS ||
                step < SOFTANTI_MIN_STEP_C || step > SOFTANTI_MAX_STEP_C ||
                pause_ms < SOFTANTI_MIN_PAUSE_MS || pause_ms > SOFTANTI_MAX_PAUSE_MS) {
                telnet_sendf(fd,
                             "[ERR] range: warm/cool %.3f..%.1f C/s, step %.1f..%.1f C, pause %.1f..%.1f s\r\n",
                             SOFTANTI_MIN_RATE_DPS, SOFTANTI_MAX_RATE_DPS,
                             SOFTANTI_MIN_STEP_C, SOFTANTI_MAX_STEP_C,
                             SOFTANTI_MIN_PAUSE_MS / 1000.0f,
                             SOFTANTI_MAX_PAUSE_MS / 1000.0f);
            } else {
                g_softanti_warm_dps = warm;
                g_softanti_cool_dps = cool;
                g_softanti_step_c = step;
                g_softanti_pause_ms = pause_ms;
                softanti_reset_model();
                if (g_allstart_session) softanti_schedule_next();
                config_save();
                telnet_sendf(fd, "[OK] quiet sampler warm=%.3f cool=%.3f step=%.1f pause=%.1fs (saved)\r\n",
                             g_softanti_warm_dps, g_softanti_cool_dps, g_softanti_step_c,
                             g_softanti_pause_ms / 1000.0f);
            }
        }

    } else if (strcmp(cmd, "ls") == 0 || strcmp(cmd, "status") == 0 || strcmp(cmd, "log") == 0) {
        print_status(fd);

    } else if (strcmp(cmd, "dash") == 0 || strcmp(cmd, "top") == 0) {
        status_stream_run(fd);

    } else if (strcmp(cmd, "time") == 0) {
        print_time(fd);

    } else if (strncmp(cmd, "settime ", 8) == 0) {
        cmd_settime(fd, cmd + 8);

    } else if (strcmp(cmd, "help") == 0) {
        print_help(fd);

    } else {
        telnet_sendf(fd, "[ERR] unknown command: '%s' (type help)\r\n", cmd);
    }
}

// ============================================================
// Telnet server task
// ============================================================

static void telnet_server_task(void *arg)
{
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TELNET_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) { ESP_LOGE(TAG_TEL, "socket() failed: %d", errno); vTaskDelete(NULL); }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG_TEL, "bind() failed: %d", errno); close(listen_fd); vTaskDelete(NULL);
    }
    if (listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG_TEL, "listen() failed: %d", errno); close(listen_fd); vTaskDelete(NULL);
    }

    ESP_LOGI(TAG_TEL, "listening on port %d", TELNET_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (client_fd < 0) {
            ESP_LOGE(TAG_TEL, "accept() failed: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        xSemaphoreTake(g_telnet_mutex, portMAX_DELAY);
        if (g_telnet_client_fd >= 0) {
            xSemaphoreGive(g_telnet_mutex);
            const char *busy_msg = "[BUSY] another client is connected\r\n";
            send(client_fd, busy_msg, strlen(busy_msg), 0);
            close(client_fd);
            continue;
        }
        g_telnet_client_fd = client_fd;
        xSemaphoreGive(g_telnet_mutex);

        char ip_str[INET_ADDRSTRLEN];
        inet_ntoa_r(client_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG_TEL, "client connected from %s", ip_str);

        // --- Telnet option negotiation ---
        // Tell the client: "I will echo every character, and neither side uses go-ahead"
        // so the client switches to character-at-a-time mode with LOCAL ECHO OFF.
        // Without this most clients (Windows telnet, PuTTY, macOS telnet) echo locally
        // and our server-side echo produces doubled characters like "hheellpp".
        static const uint8_t telnet_init[] = {
            0xFF, 0xFB, 0x01,   // IAC WILL  ECHO
            0xFF, 0xFB, 0x03,   // IAC WILL  SUPPRESS-GO-AHEAD
            0xFF, 0xFD, 0x03,   // IAC DO    SUPPRESS-GO-AHEAD
            0xFF, 0xFE, 0x22,   // IAC DON'T LINEMODE (force char mode)
        };
        send(client_fd, telnet_init, sizeof(telnet_init), 0);

        telnet_send(client_fd,
            "\r\n+==========================================+\r\n"
            "|    ESP32-S3 Composites-Mixer v2.3        |\r\n"
            "|    Type 'help' for available commands    |\r\n"
            "+==========================================+\r\n");
        telnet_prompt(client_fd);

        char cmd_buf[CMD_BUF_SIZE];
        int cmd_len = 0;
        uint8_t rx_byte;

        while (1) {
            int n = recv(client_fd, &rx_byte, 1, 0);
            if (n <= 0) { ESP_LOGI(TAG_TEL, "client disconnected"); break; }

            if (rx_byte == 0xFF) {
                // Telnet IAC sequence: verb byte, then 1-byte option for WILL/WONT/DO/DONT,
                // or a run until IAC SE for subnegotiation (SB).
                uint8_t verb;
                if (recv(client_fd, &verb, 1, 0) <= 0) break;
                if (verb == 0xFA) {
                    uint8_t b;
                    while (recv(client_fd, &b, 1, 0) > 0) {
                        if (b == 0xFF) {
                            uint8_t se;
                            if (recv(client_fd, &se, 1, 0) <= 0) break;
                            if (se == 0xF0) break;   // IAC SE
                        }
                    }
                } else if (verb >= 0xFB && verb <= 0xFE) {
                    uint8_t opt;
                    recv(client_fd, &opt, 1, 0);
                }
                continue;
            }
            if (rx_byte == '\r') continue;

            if (rx_byte == '\n') {
                cmd_buf[cmd_len] = '\0';
                telnet_send(client_fd, "\r\n");
                handle_command(client_fd, cmd_buf);
                cmd_len = 0;
                telnet_prompt(client_fd);
            } else if (rx_byte == 8 || rx_byte == 127) {
                if (cmd_len > 0) { cmd_len--; telnet_send(client_fd, "\b \b"); }
            } else if (rx_byte >= 0x20 && rx_byte < 0x7F) {
                if (cmd_len < CMD_BUF_SIZE - 1) {
                    cmd_buf[cmd_len++] = (char)rx_byte;
                    char echo[2] = {(char)rx_byte, '\0'};
                    telnet_send(client_fd, echo);
                }
            }
        }

        xSemaphoreTake(g_telnet_mutex, portMAX_DELAY);
        close(client_fd);
        g_telnet_client_fd = -1;
        xSemaphoreGive(g_telnet_mutex);
    }
}

// ============================================================
// Temperature + heater control task (single loop)
// ============================================================

static bool softanti_tick(void)
{
    if (!softanti_active() || g_softanti_paused) return false;

    int64_t now = esp_timer_get_time();
    if (g_softanti_next_sample_us == 0) {
        softanti_schedule_next();
        now = esp_timer_get_time();
    }
    if (now < g_softanti_next_sample_us) return false;

    uint32_t profile_wait_ms = wait_profile_active_ms();
    uint32_t profile_run_ms = wait_profile_active_run_ms();
    uint32_t cool_probe_ms = softanti_cool_probe_ms();
    bool profile_cycle = (profile_run_ms > 0);
    uint32_t quiet_wait_ms = (profile_run_ms > 0) ?
                             profile_wait_ms :
                             (g_softanti_pause_ms + cool_probe_ms);
    if (quiet_wait_ms < g_softanti_pause_ms) quiet_wait_ms = g_softanti_pause_ms;
    ESP_LOGI(TAG_TEMP,
             "softanti pause: profile=%s wait=%ums run=%ums settle=%ums",
             g_wait_profiles[g_active_wait_profile].name,
             (unsigned)quiet_wait_ms,
             (unsigned)profile_run_ms,
             (unsigned)g_softanti_pause_ms);
    g_softanti_paused = true;

    esp_err_t pause_err = motor_apply_duty(0);
    if (pause_err != ESP_OK) {
        ESP_LOGW(TAG_TEMP, "softanti pause command failed: %s",
                 esp_err_to_name(pause_err));
    }

    bool restore_manual_heat = (!profile_cycle &&
                                g_heater_mode == HEATER_MANUAL &&
                                g_heater_on);
    if (!profile_cycle && g_heater_on) heater_set_relay(false);

    if (!softanti_pause_wait_ms(g_softanti_pause_ms)) {
        if (g_allstart_session && !g_motor_stop_pending && g_motor_on) {
            motor_apply_duty(g_target_duty);
        }
        g_softanti_paused = false;
        return true;
    }

    g_temperature = max31865_read_temperature();
    int64_t start_us = esp_timer_get_time();
    float start_c = g_temperature;
    bool start_ok = g_temperature_valid && !g_tc_error && g_temp_fault_streak == 0;
    float end_c = start_c;
    int64_t end_us = start_us;
    bool end_ok = start_ok;

    if (profile_cycle && start_ok) {
        softanti_calibrate(start_c, start_us);
        softanti_plan_from_sample(start_c, start_us);
    }

    uint32_t cool_wait_ms = quiet_wait_ms - g_softanti_pause_ms;
    bool wait_ok = profile_cycle ?
                   softanti_sample_profile_wait(cool_wait_ms, &end_c, &end_us, &end_ok) :
                   softanti_pause_wait_ms(cool_wait_ms);
    if (!wait_ok) {
        if (g_allstart_session && !g_motor_stop_pending && g_motor_on) {
            motor_apply_duty(g_target_duty);
        }
        g_softanti_paused = false;
        return true;
    }

    if (!profile_cycle) {
        g_temperature = max31865_read_temperature();
        end_us = esp_timer_get_time();
        end_c = g_temperature;
        end_ok = g_temperature_valid && !g_tc_error && g_temp_fault_streak == 0;
    }

    if (!profile_cycle && start_ok && end_ok) {
        softanti_calibrate_cool_probe(start_c, start_us, end_c, end_us);
    }

    esp_err_t drv_err = motor_poll_status();
    if (drv_err != ESP_OK) {
        ESP_LOGW(TAG_MOTOR, "softanti driver poll failed during pause: %s",
                 esp_err_to_name(drv_err));
    }

    if (g_allstart_session && !g_motor_stop_pending && g_motor_on) {
        esp_err_t resume_err = motor_apply_duty(g_target_duty);
        if (resume_err != ESP_OK) {
            ESP_LOGW(TAG_TEMP, "softanti resume failed: %s",
                     esp_err_to_name(resume_err));
        }
    }

    if (restore_manual_heat && end_ok && end_c < HEATER_MAX_TEMP_C) {
        heater_set_relay(true);
    }

    if (end_ok) {
        if (start_ok) {
            softanti_calibrate(start_c, start_us);
        } else {
            softanti_calibrate(end_c, end_us);
        }
        softanti_plan_from_sample(end_c, end_us);
    } else {
        heater_set_relay(false);
        g_softanti_next_sample_us = end_us + 2000000LL;
        ESP_LOGW(TAG_TEMP, "softanti RTD sample failed; heater OFF, retry in 2s");
    }

    g_softanti_paused = false;
    return true;
}

static void temperature_task(void *arg)
{
    while (1) {
        if (softanti_active()) {
            softanti_tick();
            if (g_temperature_valid && g_temperature > HEATER_MAX_TEMP_C) {
                if (g_heater_on) ESP_LOGW(TAG_HEAT, "softanti over-temp %.1fC - forcing SSR OFF", g_temperature);
                heater_set_relay(false);
            }
        } else {
            g_temperature = max31865_read_temperature();
            heater_control_tick();
        }

        vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL));
    }
}

static esp_err_t motor_poll_with_quiet_window(void)
{
    if (!g_motor_on || g_motor_stop_pending || softanti_active()) {
        return motor_poll_status();
    }

    ESP_LOGW(TAG_MOTOR, "driver telemetry stale; pausing motor %ums for quiet poll",
             (unsigned)g_driver_telem_pause_ms);

    esp_err_t pause_err = motor_apply_duty(0);
    if (pause_err != ESP_OK) return pause_err;

    vTaskDelay(pdMS_TO_TICKS(g_driver_telem_pause_ms));
    esp_err_t poll_err = motor_poll_status();

    if (g_motor_on && !g_motor_stop_pending) {
        esp_err_t resume_err = motor_apply_duty(g_target_duty);
        if (resume_err != ESP_OK) {
            ESP_LOGW(TAG_MOTOR, "driver telemetry quiet-poll resume failed: %s",
                     esp_err_to_name(resume_err));
        }
    }

    return poll_err;
}

// ============================================================
// Motor-driver poller (keeps g_drv_* cache fresh) + safety latch
// ============================================================

static void motor_poller_task(void *arg)
{
    while (1) {
        if (g_motor_stop_pending) {
            esp_err_t stop_err = motor_stop_once(true);
            if (stop_err == ESP_OK) {
                g_motor_stop_pending = false;
                ESP_LOGI(TAG_MOTOR, "pending stop command accepted");
            } else {
                ESP_LOGW(TAG_MOTOR, "pending stop retry failed: %s",
                         esp_err_to_name(stop_err));
            }
            vTaskDelay(pdMS_TO_TICKS(MOTOR_STOP_RETRY_INTERVAL_MS));
            continue;
        }

        esp_err_t err = motor_poll_status();
        if (err == ESP_OK) {
            // Auto-stop on critical driver faults (over-current / over-temp / voltage faults)
            if (g_motor_on &&
                (g_drv_error == 2 || g_drv_error == 6 || g_drv_error == 7 ||
                 g_drv_error == 8 || g_drv_error == 9)) {
                ESP_LOGW(TAG_MOTOR, "driver fault %u (%s) — auto-stopping",
                         g_drv_error,
                         MOTOR_ERROR_TEXT[g_drv_error]);
                motor_stop();
            }
        } else {
            if (g_drv_fail_count < UINT8_MAX) g_drv_fail_count++;
            if (g_drv_fail_count >= g_driver_telem_pause_fails &&
                ((g_drv_fail_count - g_driver_telem_pause_fails) %
                 g_driver_telem_pause_fails) == 0 &&
                g_motor_on && !softanti_active()) {
                esp_err_t quiet_err = motor_poll_with_quiet_window();
                if (quiet_err == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
                ESP_LOGW(TAG_MOTOR, "quiet driver poll failed: %s",
                         esp_err_to_name(quiet_err));
            }
            if (g_drv_fail_count >= DRIVER_OFFLINE_CONFIRM) {
                if (g_drv_read_ok) {
                    ESP_LOGW(TAG_MOTOR, "driver offline after %u failed polls: %s",
                             g_drv_fail_count, esp_err_to_name(err));
                }
                g_drv_read_ok = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// Heartbeat log
// ============================================================

static void heartbeat_task(void *arg)
{
    while (1) {
        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);
        char ts[20];
        strftime(ts, sizeof(ts), "%H:%M:%S", &tm_now);

        char temp_txt[24];
        if (!g_temperature_valid) snprintf(temp_txt, sizeof(temp_txt), "NO-SAMPLE");
        else if (g_tc_error)      snprintf(temp_txt, sizeof(temp_txt), "FAULT 0x%02X", g_tc_fault_code);
        else                      snprintf(temp_txt, sizeof(temp_txt), "%.1fC", g_temperature);

        if (g_drv_read_ok) {
            uint16_t rpm = g_drv_rpm_scaled ? (uint16_t)(g_drv_rpm * 10) : g_drv_rpm;
            ESP_LOGI(TAG, "[%s] T=%s heat=%s(%s/%s) motor=%s duty=%.0f%% RPM=%u drv-err=%u V=%.1f",
                     ts, temp_txt,
                     g_heater_on ? "on" : "off",
                     g_heater_mode == HEATER_AUTO ? "AUTO" : "MAN",
                     g_heater_control_enabled ? "enabled" : "idle",
                     g_motor_on ? "run" : "stop",
                     g_target_duty / 10.0f, rpm, g_drv_error, g_drv_supply_dV * 0.1f);
        } else {
            ESP_LOGI(TAG, "[%s] T=%s heat=%s(%s/%s) motor=%s duty=%.0f%% (driver offline)",
                     ts, temp_txt,
                     g_heater_on ? "on" : "off",
                     g_heater_mode == HEATER_AUTO ? "AUTO" : "MAN",
                     g_heater_control_enabled ? "enabled" : "idle",
                     g_motor_on ? "run" : "stop",
                     g_target_duty / 10.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================
// app_main
// ============================================================

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32-S3 Composites-Mixer Firmware v2.2");
    ESP_LOGI(TAG, " Framework: ESP-IDF");
    ESP_LOGI(TAG, "========================================");

    g_telnet_mutex = xSemaphoreCreateMutex();
    g_modbus_mutex = xSemaphoreCreateMutex();

    init_nvs();
    config_load();          // restore saved duty/temp/band before peripherals come up
    init_wifi_ap();

    init_rs485();
    init_ssr();
    init_rtc();
    init_max31865();

    xTaskCreate(telnet_server_task, "telnet",    4096, NULL, 5, NULL);
    xTaskCreate(temperature_task,   "temp",      3072, NULL, 6, NULL);
    xTaskCreate(motor_poller_task,  "motorpoll", 3072, NULL, 4, NULL);
    xTaskCreate(heartbeat_task,     "heartbeat", 3072, NULL, 3, NULL);

    ESP_LOGI(TAG, "all systems initialized, waiting for telnet connection...");
}
