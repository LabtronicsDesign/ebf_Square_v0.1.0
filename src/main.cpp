#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <lvgl.h>     //v9.1.0
#include <TFT_eSPI.h> //v2.5.43
#include <NimBLEDevice.h>
#include <ui.h>
#include <EBF.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

// #define DEBUG
#ifdef DEBUG
#define debugprint(...) {if(serialMutex){xSemaphoreTake(serialMutex, portMAX_DELAY);Serial.print(__VA_ARGS__);xSemaphoreGive(serialMutex);}}
#define debugprintln(...) {if(serialMutex){xSemaphoreTake(serialMutex, portMAX_DELAY);Serial.println(__VA_ARGS__);xSemaphoreGive(serialMutex);}}
#define debugprintf(...) {if(serialMutex){xSemaphoreTake(serialMutex, portMAX_DELAY);Serial.printf(__VA_ARGS__);xSemaphoreGive(serialMutex);}}
#else
#define debugprint(...)
#define debugprintln(...)
#define debugprintf(...)
#endif

// FreeRTOS Configuration
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// Pin Definitions (aligned with schematic)
#define BATT_CHRG_PIN 1
#define BATT_STBY_PIN 2
#define ON_OFF_PIN 4
#define BUTTON1 8
#define BUTTON2 7
#define BUTTON3 6
#define BUTTON4 5
#define I2C_SDA 9
#define I2C_SCL 10
#define BL_CTRL_PIN 12
#define SCREEN_RD_PIN 13
#define SCREEN_WR_PIN 14
#define UART_RX_PIN 18
#define UART_TX_PIN 17
#define PIEZO_PIN 21
#define SCREEN_RST_PIN 33
#define SCREEN_CS_PIN 34
#define LED_PIN 38

// Display Configuration
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// Auto-repeat configuration
#define AUTO_REPEAT_INITIAL_DELAY 500 // ms before auto-repeat starts
#define AUTO_REPEAT_INTERVAL 100      // ms between auto-repeats

// Battery reading interval
#define BATTERY_READ_INTERVAL 5000 // Read battery every 5 seconds

// Piezo beep configuration
#define BEEP_FREQUENCY 2000            // Hz - frequency of beep
#define BEEP_DURATION 50               // ms - duration of beep
#define BEEP_AUTOREPEAT_FREQUENCY 1500 // Hz - frequency of auto-repeat beep (lower pitch)
#define BEEP_AUTOREPEAT_DURATION 30    // ms - duration of auto-repeat beep (shorter)

// UART parameter reporting configuration
#define PARAM_SETTLE_DELAY 200 // ms - wait time after last change before sending UART message

// BLE UUIDs
#define SERVICE_UUID "7ece5c94-6705-4551-9704-c8a6b848897f"
#define CHARACTERISTIC_FREQ_UUID "35b570e4-05b2-4055-8043-7d1346e3f0c3"
#define CHARACTERISTIC_STR_UUID "41538e4b-1fa3-4215-a23f-f34115487382"
#define CHARACTERISTIC_INTEN_UUID "ff348571-87df-4ec5-b6d0-2cb248faa18e"
#define CHARACTERISTIC_BATT_UUID "5f3f6ecd-3f39-45c3-a987-a7cc8725e67c"

// MAX17048 fuel gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048);

// BLE Components
NimBLEServer *bleServer;
NimBLECharacteristic *pCharFrequency;
NimBLECharacteristic *pCharStrength;
NimBLECharacteristic *pCharIntensity;
NimBLECharacteristic *pCharBattery;

TaskHandle_t taskHandleDisplay;
TaskHandle_t taskHandleGPIO;
TaskHandle_t taskHandleBLE;
TaskHandle_t taskHandleComm;

SemaphoreHandle_t serialMutex;

bool standbyStatus, chargeBattStatus, fullBattStatus, bleConnected;

// Global therapy mode variable for screen switching
bool therapy = false;

int this_width, initial_width, delta_width, response, local_rings;
int seconds, skin_display, dose, no_info;

// Global battery percentage variable
float batteryPercentage = 0.0;
bool max17048Available = false;

// Parameter change tracking for UART reporting
struct ParameterChangeTracker
{
    bool pending_change;
    uint32_t last_change_time;
    uint16_t changed_param_index;
    bool is_auto_repeating;
} param_change_tracker = {false, 0, 0, false};

// Volatile variables for ISR communication
volatile bool button_event_pending = false;
volatile uint32_t button_states = 0;
volatile uint32_t button_timestamp = 0;

// Button state tracking for auto-repeat
struct ButtonState
{
    bool is_pressed;
    uint32_t press_start_time;
    uint32_t last_repeat_time;
    bool auto_repeating;
} button_states_tracker[4] = {0};

parameter_block_t param = {
    .strength = 50,
    .freq_cycling = 0,
    .base_freq = FREQ_DEF,
    .intensity = INTENSITY_MIN,
    .interval_gap = Z_DEF,
    .filter = 0,
    .modulation = 0};

// Display buffer
static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 320;
enum
{
    SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10
};
static lv_color_t buf[SCREENBUFFER_SIZE_PIXELS];
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

void processIncomingUART();
void bleStateFunc();
void chargeStateFunc();
void onOffSliderFunc();
void therapyModeFunc();
void IRAM_ATTR buttonFunc();
void handleButtonPress(lv_obj_t *btn);
void handleButtonRelease(lv_obj_t *btn);
void adjustRoller(lv_obj_t *roller, bool up);
void adjustParameter(bool increase);
void updateReadingsDisplay();
void updateParameterDisplay();
void processButtonEvents();
void setupButtonInterrupts();
void initializeParameterDisplay();
void initializeMAX17048();
void readBatteryPercentage();
void playBeep();
void playAutoRepeatBeep();
void sendParameterUpdate(uint16_t param_index);
void checkParameterSettling();
void notifyParameterChange(uint16_t param_index, bool is_auto_repeat);
void beep(int duration);
void beepLow(int duration);
void beepPeriodic(int duration, int period);
void endBeep(int duration);
void audioWarble(int times);
void audioPipNTimes(int times);

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
    {
        debugprintf("Client address: %s\n", connInfo.getAddress().toString().c_str());

        /**
         *  We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments.
         */
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
    }

    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
    {
        debugprintf("Client disconnected - start advertising\n");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
    {
        debugprintf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
    }

    /********************* Security handled here *********************/
    uint32_t onPassKeyDisplay() override
    {
        debugprintf("Server Passkey Display\n");
        /**
         * This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
        return 123456;
    }

    void onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t pass_key) override
    {
        debugprintf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
        /** Inject false if passkeys don't match. */
        NimBLEDevice::injectConfirmPasskey(connInfo, true);
    }

    void onAuthenticationComplete(NimBLEConnInfo &connInfo) override
    {
        /** Check that encryption was successful, if not we disconnect the client */
        if (!connInfo.isEncrypted())
        {
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            debugprintf("Encrypt connection failed - disconnecting client\n");
            return;
        }

        debugprintf("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
    }
} serverCallbacks;

// Function to initialize LVGL
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Handle color format
#if LV_VERSION_CHECK(9, 0, 0)
    // Use TFT_eSPI's built-in swap option
    bool swapBytes = true; // Most displays need this
#else
    bool swapBytes = true;
#endif

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)pixelmap, w * h, swapBytes);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

static uint32_t my_tick_get_cb(void) { return millis(); }

void taskDisplay(void *pvParameters)
{
    lv_init();

    tft.begin();        /* TFT init */
    tft.setRotation(2); /* Landscape orientation, flipped */
    tft.fillScreen(TFT_BLUE);
    digitalWrite(BL_CTRL_PIN, HIGH);

    static lv_disp_t *disp;
    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_buffers(disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Set the color format based on LVGL version
#if LV_VERSION_CHECK(9, 0, 0)
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
#endif

    lv_display_set_flush_cb(disp, my_disp_flush);

    lv_tick_set_cb(my_tick_get_cb);

    ui_init();

    initializeParameterDisplay();

    for (;;)
    {
        onOffSliderFunc();
        chargeStateFunc();
        bleStateFunc();
        therapyModeFunc();
        processButtonEvents();
        // every 1000ms
        //  uint32_t now = millis();
        //  if(now % 1000 == 0) {
        updateReadingsDisplay();
        // }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Function to parse comma-separated beep commands
void processBeepCommand(const String &command, int param1, int param2, int param3)
{
    if (command == "Beep")
    {
        beep(param1);
        debugprintf("Beep: duration=%d\n", param1);
    }
    else if (command == "BeepLow")
    {
        beepLow(param1);
        debugprintf("BeepLow: duration=%d\n", param1);
    }
    else if (command == "BeepPeriod")
    {
        beepPeriodic(param1, param2);
        debugprintf("BeepPeriod: duration=%d, period=%d\n", param1, param2);
    }
    else if (command == "EndBeep")
    {
        endBeep(param1);
        debugprintf("EndBeep: duration=%d\n", param1);
    }
    else if (command == "AudioWarble")
    {
        audioWarble(param1);
        debugprintf("AudioWarble: times=%d\n", param1);
    }
    else if (command == "AudioPipNTimes")
    {
        audioPipNTimes(param1);
        debugprintf("AudioPipNTimes: times=%d\n", param1);
    }
    else
    {
        debugprintf("Unknown beep command: %s\n", command.c_str());
    }
}

// Function to parse space-separated values or comma-separated commands
void processUARTCommand(const String &command)
{
    // Check if this is a comma-separated beep command
    if (command.indexOf(',') != -1)
    {
        // Parse comma-separated beep command
        int commaIndex1 = command.indexOf(',');
        int commaIndex2 = command.indexOf(',', commaIndex1 + 1);
        int commaIndex3 = command.indexOf(',', commaIndex2 + 1);

        if (commaIndex1 != -1 && commaIndex2 != -1 && commaIndex3 != -1)
        {
            String beepCommand = command.substring(0, commaIndex1);
            int param1 = command.substring(commaIndex1 + 1, commaIndex2).toInt();
            int param2 = command.substring(commaIndex2 + 1, commaIndex3).toInt();
            int param3 = command.substring(commaIndex3 + 1).toInt();

            processBeepCommand(beepCommand, param1, param2, param3);
        }
        else
        {
            debugprintf("Error: Invalid beep command format: %s\n", command.c_str());
        }
    }
    // Check if this is a space-separated numeric message
    else if (command.indexOf(' ') != -1)
    {
        // Parse the space-separated values
        int values[9]; // Array to hold the 9 expected values
        int valueCount = 0;
        int startIndex = 0;

        // Parse each space-separated value
        for (int i = 0; i <= command.length(); i++)
        {
            if (i == command.length() || command.charAt(i) == ' ')
            {
                if (startIndex < i && valueCount < 9)
                {
                    String valueStr = command.substring(startIndex, i);
                    values[valueCount] = valueStr.toInt();
                    valueCount++;
                }
                startIndex = i + 1;
            }
        }

        // Assign values to global variables if we got all 9 values
        if (valueCount == 9)
        {
            this_width = values[0];
            initial_width = values[1];
            delta_width = values[2];
            response = values[3];
            local_rings = values[4];
            seconds = values[5];
            skin_display = values[6];
            dose = values[7];
            no_info = values[8];

            debugprintf("Parsed values: tw=%d iw=%d dw=%d resp=%d lr=%d sec=%d sd=%d dose=%d ni=%d\n",
                        this_width, initial_width, delta_width, response, local_rings,
                        seconds, skin_display, dose, no_info);
        }
        else
        {
            debugprintf("Error: Expected 9 values, got %d\n", valueCount);
        }
    }
    else if (command.startsWith("THERAPY:"))
    {
        if (command.endsWith("ON"))
        {
            therapy = true;
            debugprintln("Therapy activated via UART1");
        }
        else if (command.endsWith("OFF"))
        {
            therapy = false;
            debugprintln("Therapy deactivated via UART1");
        }
    }
    // Add more command parsing here as needed
}

void processIncomingUART()
{
    // Check if data is available on UART1
    while (Serial1.available() > 0)
    {
        // Read until newline character (\n)
        String uart_message = Serial1.readStringUntil('\n');

        // Remove any trailing carriage return (\r) if present
        uart_message.trim();

        // Only process if we have actual content
        if (uart_message.length() > 0)
        {
            debugprintf("UART1 RX: %s\n", uart_message.c_str());
            processUARTCommand(uart_message);
        }
    }
}

void bleStateFunc()
{
    static bool prev_ble_state = false;
    bool current_ble_state = bleConnected;
    if (current_ble_state != prev_ble_state)
    {
        debugprintf("BLE state changed: %d -> %d\n", prev_ble_state, current_ble_state);
        if (current_ble_state)
        {
            lv_obj_clear_flag(ui_BT_Icon1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_BT_Icon2, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(ui_BT_Icon1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(ui_BT_Icon2, LV_OBJ_FLAG_HIDDEN);
        }
        playBeep();
    }
    prev_ble_state = current_ble_state;
}

void chargeStateFunc()
{
    // Define variables to track previous state
    static bool prev_charging = false;
    static bool prev_full = false;
    static int prev_battery_state = -1; // -1: initial, 0: normal, 1: charging, 2: full
    static float prev_battery_percentage = -1.0;

    // Get current state
    bool charging = chargeBattStatus && !fullBattStatus;
    bool full = !chargeBattStatus && fullBattStatus;
    int current_battery_state;
    int battery_icon_state = 0; // 0: low (0-30%), 1: medium (31-80%), 2: high (81-100%), 3: charging/full

    // Determine the current charging state
    if (charging)
    {
        current_battery_state = 1; // charging
        battery_icon_state = 3;    // USER_4 state for charging
    }
    else if (full)
    {
        current_battery_state = 2; // full
        battery_icon_state = 3;    // USER_4 state for full
    }
    else
    {
        current_battery_state = 0; // normal
        // Determine icon state based on battery percentage
        if (batteryPercentage <= 30.0)
        {
            battery_icon_state = 0; // USER_1 state (0-30%)
        }
        else if (batteryPercentage <= 80.0)
        {
            battery_icon_state = 1; // USER_2 state (31-80%)
        }
        else
        {
            battery_icon_state = 2; // USER_3 state (81-100%)
        }
    }

    // Check if any values have changed
    bool state_changed = (current_battery_state != prev_battery_state) ||
                         (abs(batteryPercentage - prev_battery_percentage) >= 1.0); // Update if percentage changes by 1% or more

    if (state_changed)
    {
        // Debug message for state change
        debugprintf("Battery state changed: State: %d->%d, Percentage: %.1f%%->%.1f%%, Icon: %d\n",
                    prev_battery_state, current_battery_state,
                    prev_battery_percentage, batteryPercentage, battery_icon_state);

        // Handle battery text visibility and content based on charging/full state
        if (charging || full)
        {
            // Hide battery percentage text when charging or full
            if (ui_Header_Battery1)
            {
                lv_obj_add_flag(ui_Header_Battery1, LV_OBJ_FLAG_HIDDEN);
            }
            if (ui_Header_Battery2)
            {
                lv_obj_add_flag(ui_Header_Battery2, LV_OBJ_FLAG_HIDDEN);
            }
            debugprintln("Battery text hidden - charging/full state");
        }
        else
        {
            // Show and update battery percentage text when not charging/full
            if (ui_Header_Battery1)
            {
                lv_obj_clear_flag(ui_Header_Battery1, LV_OBJ_FLAG_HIDDEN);
            }
            if (ui_Header_Battery2)
            {
                lv_obj_clear_flag(ui_Header_Battery2, LV_OBJ_FLAG_HIDDEN);
            }

            // Update battery percentage display
            char battery_text[8];
            if (max17048Available)
            {
                snprintf(battery_text, sizeof(battery_text), "%.0f%%", batteryPercentage);
            }
            else
            {
                strcpy(battery_text, "---%");
            }

            if (ui_Header_Battery1)
            {
                lv_label_set_text(ui_Header_Battery1, battery_text);
            }
            if (ui_Header_Battery2)
            {
                lv_label_set_text(ui_Header_Battery2, battery_text);
            }
            debugprintf("Battery text shown: %s\n", battery_text);
        }

        // Clear all states first
        if (ui_Batt_Icon1)
        {
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_1);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_2);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_3);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_4);
        }
        if (ui_Batt_Icon2)
        {
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_1);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_2);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_3);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_4);
        }

        // Set the appropriate state based on battery_icon_state
        switch (battery_icon_state)
        {
        case 0: // Low battery (0-30%) - USER_1
            if (ui_Batt_Icon1)
                lv_obj_add_state(ui_Batt_Icon1, LV_STATE_USER_1);
            if (ui_Batt_Icon2)
                lv_obj_add_state(ui_Batt_Icon2, LV_STATE_USER_1);
            break;
        case 1: // Medium battery (31-80%) - USER_2
            if (ui_Batt_Icon1)
                lv_obj_add_state(ui_Batt_Icon1, LV_STATE_USER_2);
            if (ui_Batt_Icon2)
                lv_obj_add_state(ui_Batt_Icon2, LV_STATE_USER_2);
            break;
        case 2: // High battery (81-100%) - USER_3
            if (ui_Batt_Icon1)
                lv_obj_add_state(ui_Batt_Icon1, LV_STATE_USER_3);
            if (ui_Batt_Icon2)
                lv_obj_add_state(ui_Batt_Icon2, LV_STATE_USER_3);
            break;
        case 3: // Charging/Full - USER_4
            if (ui_Batt_Icon1)
                lv_obj_add_state(ui_Batt_Icon1, LV_STATE_USER_4);
            if (ui_Batt_Icon2)
                lv_obj_add_state(ui_Batt_Icon2, LV_STATE_USER_4);
            break;
        }

        // Update previous state variables
        playBeep();
        prev_battery_state = current_battery_state;
        prev_battery_percentage = batteryPercentage;
    }
}

void onOffSliderFunc()
{
    static byte prevSS = !standbyStatus;
    if (prevSS != standbyStatus)
    {
        if (standbyStatus == LOW)
        {
            debugprintf("Standby mode activated\n");
            if (ui_MainScreen == NULL)
            {
                debugprintln("ui_MainScreen is NULL!");
            }
            else
            {
                lv_scr_load(ui_MainScreen);
                debugprintln("Loaded MainScreen");
                // reset some values
                therapy = false;
                bleConnected = false;
            }
            // lv_scr_load_anim(ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
        }
        else
        {
            debugprintf("Standby mode deactivated\n");
            if (ui_ChargingScreen == NULL)
            {
                debugprintln("ui_ChargingScreen is NULL!");
            }
            else
            {
                lv_scr_load(ui_ChargingScreen);
                debugprintln("Loaded ChargingScreen");
            }
        }
        playBeep();
        prevSS = standbyStatus;
    }
}

void therapyModeFunc()
{
    static bool prevTherapy = false;

    // Only act when therapy state changes
    if (prevTherapy != therapy)
    {
        lv_obj_t *current_screen = lv_scr_act();

        // Only switch screens when on MainScreen or TherapyScreen
        if (current_screen == ui_MainScreen || current_screen == ui_TherapyScreen)
        {
            if (therapy)
            {
                // Switch to TherapyScreen
                debugprintf("Therapy mode activated - switching to TherapyScreen\n");
                if (ui_TherapyScreen == NULL)
                {
                    debugprintln("ui_TherapyScreen is NULL!");
                }
                else
                {
                    lv_scr_load(ui_TherapyScreen);
                    debugprintln("Loaded TherapyScreen");
                }
            }
            else
            {
                // Switch to MainScreen
                debugprintf("Therapy mode deactivated - switching to MainScreen\n");
                if (ui_MainScreen == NULL)
                {
                    debugprintln("ui_MainScreen is NULL!");
                }
                else
                {
                    lv_scr_load(ui_MainScreen);
                    debugprintln("Loaded MainScreen");
                }
            }
        }
        else
        {
            debugprintf("Therapy mode changed to %s, but not on Main/Therapy screen - no switch performed\n", therapy ? "ON" : "OFF");
        }
        playBeep();
        prevTherapy = therapy;
    }
}

void taskBLE(void *pvParameters)
{
    /** sets device name */
    uint8_t baseMac[6] = {0};
    esp_read_mac(baseMac, ESP_MAC_BT);
    char buff[40];
    sprintf(buff, "EBF %02X:%02X:%02X", baseMac[3], baseMac[4], baseMac[5]);
    debugprintln(buff);
    NimBLEDevice::init(buff);

    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(&serverCallbacks);

    NimBLEService *pService = bleServer->createService(SERVICE_UUID);

    pCharFrequency = pService->createCharacteristic(
        CHARACTERISTIC_FREQ_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    pCharStrength = pService->createCharacteristic(
        CHARACTERISTIC_STR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    pCharIntensity = pService->createCharacteristic(
        CHARACTERISTIC_INTEN_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

    pCharBattery = pService->createCharacteristic(
        CHARACTERISTIC_BATT_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    pService->start();

    /** Create an advertising instance and add the services to the advertised data */
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(buff);
    pAdvertising->addServiceUUID(SERVICE_UUID);
    /**
     *  If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    pAdvertising->enableScanResponse(true);
    pAdvertising->start();

    for (;;)
    {
        if (bleServer->getConnectedCount())
        {
            bleConnected = true;
            //   uint8_t battLevel = static_cast<uint8_t>(batterySOC);
            //   pCharBattery->setValue(&battLevel, 1);
            pCharBattery->setValue(4);
            pCharBattery->notify();
        }
        else
        {
            bleConnected = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskGPIO(void *pvParametes)
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(SCREEN_CS_PIN, OUTPUT);
    pinMode(BL_CTRL_PIN, OUTPUT);
    pinMode(ON_OFF_PIN, INPUT);
    pinMode(BATT_CHRG_PIN, INPUT_PULLUP);
    pinMode(BATT_STBY_PIN, INPUT_PULLUP);
    pinMode(PIEZO_PIN, OUTPUT);
    setupButtonInterrupts();

    // Initialize MAX17048
    initializeMAX17048();

    uint32_t lastBatteryRead = 0;

    for (;;)
    {
        // Previous state variables to track changes
        static bool prevStandbyStatus = false;
        static bool prevChargeBattStatus = false;
        static bool prevFullBattStatus = false;

        // Current state readings
        bool currentStandbyStatus = digitalRead(ON_OFF_PIN);
        bool currentChargeBattStatus = !digitalRead(BATT_CHRG_PIN);
        bool currentFullBattStatus = !digitalRead(BATT_STBY_PIN);

        // Check if any values have changed
        if (currentStandbyStatus != prevStandbyStatus ||
            currentChargeBattStatus != prevChargeBattStatus ||
            currentFullBattStatus != prevFullBattStatus)
        {

            // Send serial message about the change
            debugprintf("Status change detected: STBY: %d->%d CHRG: %d->%d FULL: %d->%d\n", prevStandbyStatus, currentStandbyStatus, prevChargeBattStatus, currentChargeBattStatus, prevFullBattStatus, currentFullBattStatus);

            // Update previous state variables
            prevStandbyStatus = currentStandbyStatus;
            prevChargeBattStatus = currentChargeBattStatus;
            prevFullBattStatus = currentFullBattStatus;
        }

        // Update the pointer values
        standbyStatus = currentStandbyStatus;
        chargeBattStatus = currentChargeBattStatus;
        fullBattStatus = currentFullBattStatus;

        // Read battery percentage periodically
        uint32_t currentTime = millis();
        if (currentTime - lastBatteryRead >= BATTERY_READ_INTERVAL)
        {
            readBatteryPercentage();
            lastBatteryRead = currentTime;
            // TEST REmove this
            //  therapy = true;
            // TEST
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void taskComm(void *pvParametes)
{
    for (;;)
    {
        checkParameterSettling();
        processIncomingUART();
        if (skin_display)
            therapy = true;
        else
            therapy = false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void initializeMAX17048()
{
    debugprintln("Initializing MAX17048...");

    // Try to initialize the MAX17048
    if (lipo.begin() == false)
    {
        debugprintln("MAX17048 not detected. Please check wiring. Battery percentage will show as ---");
        max17048Available = false;
        batteryPercentage = 0.0;
        return;
    }

    debugprintln("MAX17048 detected successfully!");
    max17048Available = true;

    // Quick start - forces the IC to restart fuel-gauge calculations
    lipo.quickStart();

    // Initial battery reading
    readBatteryPercentage();

    debugprintf("Initial battery percentage: %.2f%%\n", batteryPercentage);
}

void readBatteryPercentage()
{
    if (!max17048Available)
    {
        return;
    }

    // Read battery percentage from MAX17048
    float newPercentage = lipo.getSOC();

    // Basic validation - percentage should be between 0 and 100
    if (newPercentage >= 0.0 && newPercentage <= 100.0)
    {
        batteryPercentage = newPercentage;
        // debugprintf("Battery: %.2f%% (%.2fV)\n", batteryPercentage, lipo.getVoltage());
    }
}

void playBeep()
{
    // Generate a short beep on the piezo
    tone(PIEZO_PIN, BEEP_FREQUENCY, BEEP_DURATION);
    debugprintf("Beep played: %dHz for %dms\n", BEEP_FREQUENCY, BEEP_DURATION);
}

void playAutoRepeatBeep()
{
    // Generate a shorter, lower-pitched beep for auto-repeat
    tone(PIEZO_PIN, BEEP_AUTOREPEAT_FREQUENCY, BEEP_AUTOREPEAT_DURATION);
    debugprintf("Auto-repeat beep played: %dHz for %dms\n", BEEP_AUTOREPEAT_FREQUENCY, BEEP_AUTOREPEAT_DURATION);
}

void sendParameterUpdate(uint16_t param_index)
{
    const char *param_names[] = {
        "Settings",    // 0
        "Strength",    // 1
        "FreqCycle",   // 2
        "Frequency",   // 3
        "Intensity",   // 4
        "IntervalGap", // 5
        "Filter",      // 6
        "Modulation"   // 7
    };

    if (param_index >= sizeof(param_names) / sizeof(param_names[0]))
    {
        return; // Invalid parameter index
    }

    char uart_message[256];

    switch (param_index)
    {
    case 0: // Settings - no value to send
        return;

    case 1: // Strength
        snprintf(uart_message, sizeof(uart_message), "%s:%d", param_names[param_index], param.strength);
        break;

    case 2: // FreqCycle
        snprintf(uart_message, sizeof(uart_message), "%s:%s", param_names[param_index], param.freq_cycling ? "ON" : "OFF");
        break;

    case 3: // Frequency
        snprintf(uart_message, sizeof(uart_message), "%s:%d", param_names[param_index], param.base_freq);
        break;

    case 4: // Intensity
        snprintf(uart_message, sizeof(uart_message), "%s:%d", param_names[param_index], param.intensity);
        break;

    case 5: // IntervalGap
        snprintf(uart_message, sizeof(uart_message), "%s:%d", param_names[param_index], param.interval_gap);
        break;

    case 6: // Filter
        snprintf(uart_message, sizeof(uart_message), "%s:%s", param_names[param_index], param.filter ? "ON" : "OFF");
        break;

    case 7: // Modulation
        if (param.modulation == 0)
        {
            snprintf(uart_message, sizeof(uart_message), "%s:OFF", param_names[param_index]);
        }
        else
        {
            snprintf(uart_message, sizeof(uart_message), "%s:%d", param_names[param_index], param.modulation);
        }
        break;

    default:
        return;
    }

    // Send over UART1
    Serial1.println(uart_message);
    debugprintf("UART1 sent: %s\n", uart_message);
}

void notifyParameterChange(uint16_t param_index, bool is_auto_repeat)
{
    param_change_tracker.pending_change = true;
    param_change_tracker.last_change_time = millis();
    param_change_tracker.changed_param_index = param_index;
    param_change_tracker.is_auto_repeating = is_auto_repeat;

    // If it's not auto-repeating, send immediately
    if (!is_auto_repeat)
    {
        sendParameterUpdate(param_index);
        param_change_tracker.pending_change = false;
    }
}

void checkParameterSettling()
{
    if (!param_change_tracker.pending_change)
    {
        return;
    }

    uint32_t current_time = millis();
    uint32_t time_since_change = current_time - param_change_tracker.last_change_time;

    // Check if enough time has passed since the last change
    if (time_since_change >= PARAM_SETTLE_DELAY)
    {
        sendParameterUpdate(param_change_tracker.changed_param_index);
        param_change_tracker.pending_change = false;
        debugprintln("Parameter settled - UART message sent");
    }
}

// Simple ISR - just capture button states and exit quickly
void IRAM_ATTR buttonFunc()
{
    uint32_t current_time = millis();

    // Simple debounce - ignore if too soon after last event
    if (current_time - button_timestamp < 100)
        return;

    // Capture all button states in one read
    button_states = (digitalRead(BUTTON1) << 0) |
                    (digitalRead(BUTTON2) << 1) |
                    (digitalRead(BUTTON3) << 2) |
                    (digitalRead(BUTTON4) << 3);

    button_timestamp = current_time;
    button_event_pending = true; // Signal main loop to process
}

// Helper functions (moved out of ISR)
void handleButtonPress(lv_obj_t *btn)
{
    if (!btn)
        return;
    lv_obj_add_state(btn, LV_STATE_PRESSED);
    lv_obj_send_event(btn, LV_EVENT_PRESSED, NULL);
}

void handleButtonRelease(lv_obj_t *btn)
{
    if (!btn)
        return;
    lv_obj_remove_state(btn, LV_STATE_PRESSED);
    lv_obj_send_event(btn, LV_EVENT_RELEASED, NULL);
    lv_obj_send_event(btn, LV_EVENT_CLICKED, NULL);
}

void adjustRoller(lv_obj_t *roller, bool up)
{
    if (!roller)
        return;

    uint16_t current = lv_roller_get_selected(roller);
    uint16_t option_count = lv_roller_get_option_count(roller);

    if (up)
    {
        uint16_t new_selected = (current == 0) ? option_count - 1 : current - 1;
        lv_roller_set_selected(roller, new_selected, LV_ANIM_ON);
    }
    else
    {
        uint16_t new_selected = (current + 1) % option_count;
        lv_roller_set_selected(roller, new_selected, LV_ANIM_ON);
    }
}

// Adjust parameter based on current roller selection
void adjustParameter(bool increase)
{
    lv_obj_t *current_screen = lv_scr_act();
    if (current_screen == ui_MainScreen)
    {

        uint16_t selected = lv_roller_get_selected(ui_Roller_Topic1);
        bool param_changed = false;

        switch (selected)
        {
        case 0: // Settings - no parameter to adjust
            debugprintln("Settings selected - no parameter to adjust");
            break;

        case 1: // Strength (10-100)
            if (increase)
            {
                if (param.strength < STRENGTH_MAX)
                {
                    param.strength++;
                    param_changed = true;
                }
            }
            else
            {
                if (param.strength > STRENGTH_MIN)
                {
                    param.strength--;
                    param_changed = true;
                }
            }
            debugprintf("Strength: %d\n", param.strength);
            break;

        case 2: // Freq Cycle (0 or 1) - Toggle ON/OFF
            if (increase)
            {
                if (param.freq_cycling < 1)
                {
                    param.freq_cycling++;
                    param_changed = true;
                }
            }
            else
            {
                if (param.freq_cycling > 0)
                {
                    param.freq_cycling--;
                    param_changed = true;
                }
            }
            debugprintf("Freq Cycling: %s\n", param.freq_cycling ? "ON" : "OFF");
            break;

        case 3: // Frequency (15-350)
            if (increase)
            {
                if (param.base_freq < FREQ_MAX)
                {
                    param.base_freq++;
                    param_changed = true;
                }
            }
            else
            {
                if (param.base_freq > FREQ_MIN)
                {
                    param.base_freq--;
                    param_changed = true;
                }
            }
            debugprintf("Base Frequency: %d Hz\n", param.base_freq);
            break;

        case 4: // Intensity (1-8)
            if (increase)
            {
                if (param.intensity < INTENSITY_MAX)
                {
                    param.intensity++;
                    param_changed = true;
                }
            }
            else
            {
                if (param.intensity > INTENSITY_MIN)
                {
                    param.intensity--;
                    param_changed = true;
                }
            }
            debugprintf("Intensity: %d\n", param.intensity);
            break;

        case 5: // Interval Gap (10-80)
            if (increase)
            {
                if (param.interval_gap < Z_MAX)
                {
                    param.interval_gap++;
                    param_changed = true;
                }
            }
            else
            {
                if (param.interval_gap > Z_MIN)
                {
                    param.interval_gap--;
                    param_changed = true;
                }
            }
            debugprintf("Interval Gap: %d\n", param.interval_gap);
            break;

        case 6: // Filter (0 or 1) - Toggle ON/OFF
            if (increase)
            {
                if (param.filter < 1)
                {
                    param.filter++;
                    param_changed = true;
                }
            }
            else
            {
                if (param.filter > 0)
                {
                    param.filter--;
                    param_changed = true;
                }
            }
            debugprintf("Filter: %s\n", param.filter ? "ON" : "OFF");
            break;

        case 7: // Modulation (0-5)
            if (increase)
            {
                if (param.modulation < MODULATION_MAX)
                {
                    param.modulation++;
                    param_changed = true;
                }
            }
            else
            {
                if (param.modulation > MODULATION_MIN)
                {
                    param.modulation--;
                    param_changed = true;
                }
            }
            debugprintf("Modulation: %d\n", param.modulation);
            break;

        default:
            debugprintln("Unknown roller selection");
            break;
        }

        // Notify parameter change if something actually changed
        if (param_changed)
        {
            // Check if we're in auto-repeat mode by looking at button state
            bool is_auto_repeat = false;
            for (int i = 2; i < 4; i++)
            { // Check BUTTON3 and BUTTON4
                if (button_states_tracker[i].auto_repeating)
                {
                    is_auto_repeat = true;
                    break;
                }
            }

            notifyParameterChange(selected, is_auto_repeat);
        }
    }
    else if (current_screen == ui_TherapyScreen)
    {
        // On TherapyScreen, BUTTON3 increases intensity, BUTTON4 decreases intensity
        uint16_t selected = lv_roller_get_selected(ui_Roller_Topic2);
        bool param_changed = false;
        // Strength (10-100)
        if (increase)
        {
            if (param.strength < STRENGTH_MAX)
            {
                param.strength++;
                param_changed = true;
            }
        }
        else
        {
            if (param.strength > STRENGTH_MIN)
            {
                param.strength--;
                param_changed = true;
            }
        }
        debugprintf("Strength: %d\n", param.strength);
        // Notify parameter change if something actually changed
        if (param_changed)
        {
            // Check if we're in auto-repeat mode by looking at button state
            bool is_auto_repeat = false;
            for (int i = 2; i < 4; i++)
            { // Check BUTTON3 and BUTTON4
                if (button_states_tracker[i].auto_repeating)
                {
                    is_auto_repeat = true;
                    break;
                }
            }

            notifyParameterChange(selected, is_auto_repeat);
        }
    }
    // Update the display after parameter change
    updateParameterDisplay();
}

void updateReadingsDisplay()
{
    lv_obj_t *current_screen = lv_scr_act();
    if (current_screen != ui_TherapyScreen)
        return;

    char buffer[16];
    lv_snprintf(buffer, sizeof(buffer), "%d", this_width);
    if (ui_Label_Current)
        lv_label_set_text(ui_Label_Current, buffer);
    lv_snprintf(buffer, sizeof(buffer), "%d", initial_width);
    if (ui_Label_Initial)
        lv_label_set_text(ui_Label_Initial, buffer);
    lv_snprintf(buffer, sizeof(buffer), "%d", delta_width);
    if (ui_Extra_Value1)
        lv_label_set_text(ui_Extra_Value1, buffer);
    lv_snprintf(buffer, sizeof(buffer), "%d", response);
    if (ui_Label_Response)
        lv_label_set_text(ui_Label_Response, buffer);
    lv_snprintf(buffer, sizeof(buffer), "%d", local_rings);
    if (ui_Extra_Value2)
        lv_label_set_text(ui_Extra_Value2, buffer);
}

// Update the display with current parameter values
void updateParameterDisplay()
{
    if (!ui_Roller_Topic1)
        return;

    uint16_t selected = lv_roller_get_selected(ui_Roller_Topic1);
    char buffer[16];
    uint16_t arc_value = 0;

    // Update BUTTON3 and BUTTON4 states based on roller selection
    if (selected == 0)
    { // Settings selected
        // Set BUTTON3 and BUTTON4 to User1 state
        if (ui_ButtonMidRight1)
        {
            lv_obj_clear_state(ui_ButtonMidRight1, LV_STATE_USER_2);
            lv_obj_add_state(ui_ButtonMidRight1, LV_STATE_USER_1);
            lv_obj_add_state(ui_ButtonMidRight1, LV_STATE_DISABLED);
        }
        if (ui_ButtonRight1)
        {
            lv_obj_clear_state(ui_ButtonRight1, LV_STATE_USER_2);
            lv_obj_add_state(ui_ButtonRight1, LV_STATE_USER_1);
            lv_obj_add_state(ui_ButtonRight1, LV_STATE_DISABLED);
        }
    }
    else
    { // Any other selection
        // Set BUTTON3 and BUTTON4 to User2 state
        if (ui_ButtonMidRight1)
        {
            lv_obj_clear_state(ui_ButtonMidRight1, LV_STATE_USER_1);
            lv_obj_clear_state(ui_ButtonMidRight1, LV_STATE_DISABLED);
            lv_obj_add_state(ui_ButtonMidRight1, LV_STATE_USER_2);
        }
        if (ui_ButtonRight1)
        {
            lv_obj_clear_state(ui_ButtonRight1, LV_STATE_USER_1);
            lv_obj_clear_state(ui_ButtonRight1, LV_STATE_DISABLED);
            lv_obj_add_state(ui_ButtonRight1, LV_STATE_USER_2);
        }
    }

    // Handle frequency display visibility and content based on roller selection and freq_cycling
    if (selected == 3)
    { // Frequency is selected - hide frequency display
        if (ui_Freq_Text1)
            lv_obj_add_flag(ui_Freq_Text1, LV_OBJ_FLAG_HIDDEN);
        if (ui_Freq_Value1)
            lv_obj_add_flag(ui_Freq_Value1, LV_OBJ_FLAG_HIDDEN);
    }
    else
    { // Other selections - show frequency display
        if (ui_Freq_Text1)
            lv_obj_clear_flag(ui_Freq_Text1, LV_OBJ_FLAG_HIDDEN);
        if (ui_Freq_Value1)
            lv_obj_clear_flag(ui_Freq_Value1, LV_OBJ_FLAG_HIDDEN);

        // Update frequency display based on freq_cycling state
        if (param.freq_cycling)
        {
            if (ui_Freq_Value1)
                lv_label_set_text(ui_Freq_Value1, "Cycle");
        }
        else
        {
            lv_snprintf(buffer, sizeof(buffer), "%dHz", param.base_freq);
            if (ui_Freq_Value1)
                lv_label_set_text(ui_Freq_Value1, buffer);
        }
    }

    switch (selected)
    {
    case 0: // Settings
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, "---");
        arc_value = 0;
        break;

    case 1: // Strength (10-100) -> Arc (0-100)
        lv_snprintf(buffer, sizeof(buffer), "%d", param.strength);
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, buffer);
        if (ui_Strength_Value1)
            lv_label_set_text(ui_Strength_Value1, buffer);
        arc_value = (param.strength - STRENGTH_MIN) * 100 / (STRENGTH_MAX - STRENGTH_MIN);
        break;

    case 2: // Freq Cycle (0 or 1) -> ON/OFF display
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, param.freq_cycling ? "ON" : "OFF");
        arc_value = param.freq_cycling ? 100 : 0;
        break;

    case 3: // Frequency (15-350) -> Arc (0-100)
        lv_snprintf(buffer, sizeof(buffer), "%d", param.base_freq);
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, buffer);
        // Don't update Freq_Value1 here since it's hidden when frequency is selected
        arc_value = (param.base_freq - FREQ_MIN) * 100 / (FREQ_MAX - FREQ_MIN);
        break;

    case 4: // Intensity (1-8) -> Arc (0-100)
        lv_snprintf(buffer, sizeof(buffer), "%d", param.intensity);
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, buffer);
        arc_value = (param.intensity - INTENSITY_MIN) * 100 / (INTENSITY_MAX - INTENSITY_MIN);
        break;

    case 5: // Interval Gap (10-80) -> Arc (0-100)
        lv_snprintf(buffer, sizeof(buffer), "%d", param.interval_gap);
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, buffer);
        arc_value = (param.interval_gap - Z_MIN) * 100 / (Z_MAX - Z_MIN);
        break;

    case 6: // Filter (0 or 1) -> ON/OFF display
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, param.filter ? "ON" : "OFF");
        arc_value = param.filter ? 100 : 0;
        break;

    case 7: // Modulation (0-5) -> Arc (0-100)
        if (param.modulation == 0)
        {
            if (ui_Label_Value1)
                lv_label_set_text(ui_Label_Value1, "OFF");
        }
        else
        {
            lv_snprintf(buffer, sizeof(buffer), "%d", param.modulation);
            if (ui_Label_Value1)
                lv_label_set_text(ui_Label_Value1, buffer);
        }
        arc_value = (param.modulation - MODULATION_MIN) * 100 / (MODULATION_MAX - MODULATION_MIN);
        break;

    default:
        if (ui_Label_Value1)
            lv_label_set_text(ui_Label_Value1, "---");
        arc_value = 0;
        break;
    }

    // Update the arc with the mapped value (0-100)
    if (ui_Arc1)
    {
        lv_arc_set_value(ui_Arc1, arc_value);
    }

    // Debug output
    // debugprintf("Display updated - Selection: %d, Value: %s, Arc: %d%%, Freq_Cycling: %s, BTN3/4: %s\n",
    //    selected, (ui_Label_Value1 ? lv_label_get_text(ui_Label_Value1) : "NULL"), arc_value,
    //    param.freq_cycling ? "ON" : "OFF", selected == 0 ? "USER_1" : "USER_2");
}

// Process button events in main loop (not ISR)
void processButtonEvents()
{
    uint32_t current_time = millis();

    // Check if there's a pending button event from ISR
    if (button_event_pending)
    {
        // Get current button states (atomic read)
        uint32_t current_states = button_states;
        button_event_pending = false; // Clear the flag

        // Static variables to track previous states
        static uint32_t prev_states = 0x0F; // Initialize as all buttons HIGH (not pressed)
        static bool first_run = true;

        if (first_run)
        {
            prev_states = current_states;
            first_run = false;
            return;
        }

        // Get current screen
        lv_obj_t *current_screen = lv_scr_act();

        // Button mappings
        struct
        {
            int bit_pos;
            const char *name;
            lv_obj_t *main_btn;
            lv_obj_t *therapy_btn;
        } buttons[] = {
            {0, "BTN1", ui_ButtonLeft1, ui_ButtonLeft2},
            {1, "BTN2", ui_ButtonMidLeft1, ui_ButtonMidLeft2},
            {2, "BTN3", ui_ButtonMidRight1, ui_ButtonMidRight2},
            {3, "BTN4", ui_ButtonRight1, ui_ButtonRight2}};

        // Process each button
        for (int i = 0; i < 4; i++)
        {
            bool prev_state = (prev_states >> buttons[i].bit_pos) & 1;
            bool curr_state = (current_states >> buttons[i].bit_pos) & 1;

            // Button press detected (HIGH -> LOW, since buttons are active LOW)
            if (prev_state == 1 && curr_state == 0)
            {
                debugprintf("%s-PRESS on %s\n", buttons[i].name,
                            (current_screen == ui_MainScreen) ? "MainScreen" : (current_screen == ui_TherapyScreen) ? "TherapyScreen"
                                                                                                                    : "Unknown");

                // Play beep for successful button press
                playBeep();

                // Update button state tracking
                button_states_tracker[i].is_pressed = true;
                button_states_tracker[i].press_start_time = current_time;
                button_states_tracker[i].last_repeat_time = current_time;
                button_states_tracker[i].auto_repeating = false;

                if (current_screen == ui_MainScreen)
                {
                    handleButtonPress(buttons[i].main_btn);

                    // Special controls for MainScreen
                    if (i == 0)
                    { // BUTTON1 - Roller UP
                        adjustRoller(ui_Roller_Topic1, true);
                        updateParameterDisplay(); // Update display after roller change
                        // debugprintln("Roller UP");
                    }
                    else if (i == 1)
                    { // BUTTON2 - Roller DOWN
                        adjustRoller(ui_Roller_Topic1, false);
                        updateParameterDisplay(); // Update display after roller change
                        // debugprintln("Roller DOWN");
                    }
                    else if (i == 2)
                    { // BUTTON3 - Increase Parameter
                        adjustParameter(true);
                        // debugprintln("Parameter INCREASE");
                    }
                    else if (i == 3)
                    { // BUTTON4 - Decrease Parameter
                        adjustParameter(false);
                        // debugprintln("Parameter DECREASE");
                    }
                }
                else if (current_screen == ui_TherapyScreen)
                {
                    handleButtonPress(buttons[i].therapy_btn);
                    if (i == 2)
                    { // BUTTON3 - Increase Parameter
                        adjustParameter(true);
                        // debugprintln("Parameter INCREASE");
                    }
                    else if (i == 3)
                    { // BUTTON4 - Decrease Parameter
                        adjustParameter(false);
                        // debugprintln("Parameter DECREASE");
                    }
                }
            }

            // Button release detected (LOW -> HIGH)
            else if (prev_state == 0 && curr_state == 1)
            {
                // debugprintf("%s-RELEASE on %s\n", buttons[i].name,
                //            (current_screen == ui_MainScreen) ? "MainScreen" :
                //            (current_screen == ui_TherapyScreen) ? "TherapyScreen" : "Unknown");

                // Update button state tracking
                button_states_tracker[i].is_pressed = false;
                button_states_tracker[i].auto_repeating = false;

                if (current_screen == ui_MainScreen)
                {
                    handleButtonRelease(buttons[i].main_btn);
                }
                else if (current_screen == ui_TherapyScreen)
                {
                    handleButtonRelease(buttons[i].therapy_btn);
                }
            }
        }

        // Update previous states
        prev_states = current_states;
    }

    // Handle auto-repeat for BUTTON3 and BUTTON4 (parameter adjustment buttons)
    lv_obj_t *current_screen = lv_scr_act();
    if (current_screen == ui_MainScreen)
    {
        // Check if we're not on Settings (which has no adjustable parameters)
        uint16_t selected = ui_Roller_Topic1 ? lv_roller_get_selected(ui_Roller_Topic1) : 0;
        if (selected != 0)
        { // Not on Settings
            for (int i = 2; i < 4; i++)
            { // Only BUTTON3 and BUTTON4
                if (button_states_tracker[i].is_pressed)
                {
                    uint32_t hold_duration = current_time - button_states_tracker[i].press_start_time;

                    // Check if we should start auto-repeat
                    if (!button_states_tracker[i].auto_repeating && hold_duration >= AUTO_REPEAT_INITIAL_DELAY)
                    {
                        button_states_tracker[i].auto_repeating = true;
                        button_states_tracker[i].last_repeat_time = current_time;
                        // debugprintf("Auto-repeat started for BTN%d\n", i + 1);
                    }

                    // Process auto-repeat
                    if (button_states_tracker[i].auto_repeating)
                    {
                        uint32_t time_since_last_repeat = current_time - button_states_tracker[i].last_repeat_time;

                        if (time_since_last_repeat >= AUTO_REPEAT_INTERVAL)
                        {
                            button_states_tracker[i].last_repeat_time = current_time;

                            // Play auto-repeat beep
                            playAutoRepeatBeep();

                            if (i == 2)
                            { // BUTTON3 - Increase Parameter
                                adjustParameter(true);
                                // debugprintln("Parameter INCREASE (auto-repeat)");
                            }
                            else if (i == 3)
                            { // BUTTON4 - Decrease Parameter
                                adjustParameter(false);
                                // debugprintln("Parameter DECREASE (auto-repeat)");
                            }
                        }
                    }
                }
            }
        }
    }
}

// Setup function for button interrupts
void setupButtonInterrupts()
{
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    pinMode(BUTTON3, INPUT);
    pinMode(BUTTON4, INPUT);

    // Attach interrupts on CHANGE
    attachInterrupt(BUTTON1, buttonFunc, CHANGE);
    attachInterrupt(BUTTON2, buttonFunc, CHANGE);
    attachInterrupt(BUTTON3, buttonFunc, CHANGE);
    attachInterrupt(BUTTON4, buttonFunc, CHANGE);
}

// Initialize parameter display (call this after UI initialization)
void initializeParameterDisplay()
{
    updateParameterDisplay();
    debugprintln("Parameter display initialized");
}

// Beep function implementations - customize for your hardware
void beep(int duration)
{
    tone(PIEZO_PIN, 4000, duration); // 1000 Hz tone
}

void beepLow(int duration)
{
    // Generate low frequency beep using tone() if available
    // or use PWM for lower frequency
    tone(PIEZO_PIN, 500, duration); // 500 Hz low tone
}

void beepPeriodic(int duration, int period)
{
    // This should probably be handled non-blocking in your main loop
    // For now, just do a single beep - implement proper periodic beeping elsewhere
    beep(duration);
    debugprintf("Note: BeepPeriod needs non-blocking implementation in main loop\n");
}

void endBeep(int duration)
{
    // Could be a different tone or pattern to indicate "end"
    tone(PIEZO_PIN, 1000, duration); // Higher pitch for end signal
}

void audioWarble(int times)
{
    for (int i = 0; i < times; i++)
    {
        tone(PIEZO_PIN, 800, 100); // High tone
        tone(PIEZO_PIN, 400, 100); // Low tone
    }
}

void audioPipNTimes(int times)
{
    for (int i = 0; i < times; i++)
    {
        tone(PIEZO_PIN, 1500, 50); // Short high pip
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    playBeep();
    Serial.print("Init...");

    Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(35, 37, 36, -1); // SCLK, MISO, MOSI, SS
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    serialMutex = xSemaphoreCreateMutex();

    // Initialize GPIO
    xTaskCreate(
        taskGPIO, "GPIO",
        4096, NULL, 2, &taskHandleGPIO);

    // Initialize Comm
    xTaskCreate(
        taskComm, "Comm",
        4096, NULL, 5, &taskHandleComm);

    // Initialize BLE
    xTaskCreate(
        taskBLE, "BLE",
        4096, NULL, 4, &taskHandleBLE);

    // Initialize TFT display
    xTaskCreatePinnedToCore(
        taskDisplay, "Display",
        16384, NULL, 3, &taskHandleDisplay,
        ARDUINO_RUNNING_CORE);

    playBeep();
    Serial.println(" done");
}

void loop()
{
    vTaskDelete(NULL);
}