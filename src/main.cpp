#include "Arduino.h"
#include <SPI.h>
#include <lvgl.h>           //v9.1.0
#include "TFT_eSPI.h"       //v2.5.43
#include <NimBLEDevice.h>
#include "ui.h"

#define DEBUG
#ifdef DEBUG
#define debugprint(...)    { if(serialMutex) { xSemaphoreTake(serialMutex, portMAX_DELAY); Serial.print(__VA_ARGS__); xSemaphoreGive(serialMutex); } }
#define debugprintln(...)  { if(serialMutex) { xSemaphoreTake(serialMutex, portMAX_DELAY); Serial.println(__VA_ARGS__); xSemaphoreGive(serialMutex); } }
#define debugprintf(...)   { if(serialMutex) { xSemaphoreTake(serialMutex, portMAX_DELAY); Serial.printf(__VA_ARGS__); xSemaphoreGive(serialMutex); } }
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

// Peripheral Addresses
#define TCA9534A_ADDR 0x38
#define PCA9685_ADDR 0x40

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
#define UART_RX_PIN 17
#define UART_TX_PIN 18
#define PIEZO_PIN 21
#define SCREEN_RST_PIN 33
#define SCREEN_CS_PIN 34
#define LED_PIN 38

// Display Configuration
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// BLE UUIDs
#define SERVICE_UUID "7ece5c94-6705-4551-9704-c8a6b848897f"
#define CHARACTERISTIC_FREQ_UUID "35b570e4-05b2-4055-8043-7d1346e3f0c3"
#define CHARACTERISTIC_STR_UUID "41538e4b-1fa3-4215-a23f-f34115487382"
#define CHARACTERISTIC_INTEN_UUID "ff348571-87df-4ec5-b6d0-2cb248faa18e"
#define CHARACTERISTIC_BATT_UUID "5f3f6ecd-3f39-45c3-a987-a7cc8725e67c"
// BLE Components
NimBLEServer *bleServer;
NimBLECharacteristic *pCharFrequency;
NimBLECharacteristic *pCharStrength;
NimBLECharacteristic *pCharIntensity;
NimBLECharacteristic *pCharBattery;

TaskHandle_t taskHandleDisplay;
TaskHandle_t taskHandleGPIO;
TaskHandle_t taskHandleBLE;

SemaphoreHandle_t serialMutex;

bool standbyStatus, chargeBattStatus, fullBattStatus, bleConnected;


// Volatile variables for ISR communication
volatile bool button_event_pending = false;
volatile uint32_t button_states = 0;
volatile uint32_t button_timestamp = 0;

// Display buffer
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 320;
enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];
TFT_eSPI tft = TFT_eSPI( screenWidth, screenHeight ); /* TFT instance */

void bleStateFunc();
void chargeStateFunc();
void onOffSliderFunc();
void IRAM_ATTR buttonFunc();
void handleButtonPress(lv_obj_t *btn);
void handleButtonRelease(lv_obj_t *btn);
void adjustRoller(lv_obj_t *roller, bool up);
void IRAM_ATTR buttonFunc();
void processButtonEvents();
void setupButtonInterrupts();

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
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

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        debugprintf("Client disconnected - start advertising\n");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        debugprintf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
    }

    /********************* Security handled here *********************/
    uint32_t onPassKeyDisplay() override {
        debugprintf("Server Passkey Display\n");
        /**
         * This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
        return 123456;
    }

    void onConfirmPassKey(NimBLEConnInfo& connInfo, uint32_t pass_key) override {
        debugprintf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
        /** Inject false if passkeys don't match. */
        NimBLEDevice::injectConfirmPasskey(connInfo, true);
    }

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
        /** Check that encryption was successful, if not we disconnect the client */
        if (!connInfo.isEncrypted()) {
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            debugprintf("Encrypt connection failed - disconnecting client\n");
            return;
        }

        debugprintf("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
    }
} serverCallbacks;



// Function to initialize LVGL
void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    // Handle color format
#if LV_VERSION_CHECK(9, 0, 0)
    // Use TFT_eSPI's built-in swap option
    bool swapBytes = true;  // Most displays need this
#else
    bool swapBytes = true;
#endif

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t*)pixelmap, w * h, swapBytes);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

static uint32_t my_tick_get_cb (void) { return millis(); }

void taskDisplay(void *pvParameters)
{
    lv_init();

    tft.begin();          /* TFT init */
    tft.setRotation( 2 ); /* Landscape orientation, flipped */
    tft.fillScreen(TFT_BLUE);
    digitalWrite(BL_CTRL_PIN, HIGH);

    static lv_disp_t* disp;
    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_buffers(disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Set the color format based on LVGL version
#if LV_VERSION_CHECK(9, 0, 0)
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
#endif
    
    lv_display_set_flush_cb(disp, my_disp_flush);

    lv_tick_set_cb( my_tick_get_cb );

    ui_init();

    for (;;)
    {
        onOffSliderFunc();
        chargeStateFunc();
        bleStateFunc();
        processButtonEvents();
        //inputParametersFunc();
        //outputParametersFunc();

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
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
    }
    prev_ble_state = current_ble_state;
}

void chargeStateFunc()
{
    // Define variables to track previous state
    static bool prev_charging = false;
    static bool prev_full = false;
    static int prev_battery_state = -1; // -1: initial, 0: normal, 1: charging, 2: full

    // Get current state
    bool charging = chargeBattStatus && !fullBattStatus;
    bool full = !chargeBattStatus && fullBattStatus;
    int current_battery_state;

    // Determine the current state
    if (charging)
    {
        current_battery_state = 1; // charging
    }
    else if (full)
    {
        current_battery_state = 2; // full
    }
    else
    {
        current_battery_state = 0; // normal
    }

    // Only update the UI if the state has changed
    if (current_battery_state != prev_battery_state)
    {
        // Debug message for state change
        debugprintf("Battery state changed: %d -> %d\n", prev_battery_state, current_battery_state);

        // Update battery icons based on current state
        if (charging)
        {
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_1);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_2);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_3);
            lv_obj_add_state(ui_Batt_Icon1, LV_STATE_USER_4);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_1);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_2);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_3);
            lv_obj_add_state(ui_Batt_Icon2, LV_STATE_USER_4);
        }
        else if (full)
        {
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_1);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_2);
            lv_obj_add_state(ui_Batt_Icon1, LV_STATE_USER_3);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_4);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_1);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_2);
            lv_obj_add_state(ui_Batt_Icon2, LV_STATE_USER_3);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_4);
        }
        else
        {
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_1);
            lv_obj_add_state(ui_Batt_Icon1, LV_STATE_USER_2);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_3);
            lv_obj_clear_state(ui_Batt_Icon1, LV_STATE_USER_4);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_1);
            lv_obj_add_state(ui_Batt_Icon2, LV_STATE_USER_2);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_3);
            lv_obj_clear_state(ui_Batt_Icon2, LV_STATE_USER_4);
        }
        /* //normal
        else{
        }
        */
        // Update previous state
        prev_battery_state = current_battery_state;

        //update batt SoC
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
            if (ui_MainScreen == NULL) {
                debugprintln("ui_MainScreen is NULL!");
            } else {
                lv_scr_load(ui_MainScreen);
                debugprintln("Loaded MainScreen");
            }
            //lv_scr_load_anim(ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
        }
        else
        {
            debugprintf("Standby mode deactivated\n");
            if (ui_ChargingScreen == NULL) {
                debugprintln("ui_ChargingScreen is NULL!");
            } else {
                lv_scr_load(ui_ChargingScreen);
                debugprintln("Loaded ChargingScreen");
            }
        }
        prevSS = standbyStatus;
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
    setupButtonInterrupts();
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
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Simple ISR - just capture button states and exit quickly
void IRAM_ATTR buttonFunc() {
    uint32_t current_time = millis();
    
    // Simple debounce - ignore if too soon after last event
    if (current_time - button_timestamp < 20) return;
    
    // Capture all button states in one read
    button_states = (digitalRead(BUTTON1) << 0) |
                   (digitalRead(BUTTON2) << 1) |
                   (digitalRead(BUTTON3) << 2) |
                   (digitalRead(BUTTON4) << 3);
    
    button_timestamp = current_time;
    button_event_pending = true;  // Signal main loop to process
}

// Helper functions (moved out of ISR)
void handleButtonPress(lv_obj_t *btn) {
    if (!btn) return;
    lv_obj_add_state(btn, LV_STATE_PRESSED);
    lv_obj_send_event(btn, LV_EVENT_PRESSED, NULL);
}

void handleButtonRelease(lv_obj_t *btn) {
    if (!btn) return;
    lv_obj_remove_state(btn, LV_STATE_PRESSED);
    lv_obj_send_event(btn, LV_EVENT_RELEASED, NULL);
    lv_obj_send_event(btn, LV_EVENT_CLICKED, NULL);
}

void adjustRoller(lv_obj_t *roller, bool up) {
    if (!roller) return;
    
    uint16_t current = lv_roller_get_selected(roller);
    uint16_t option_count = lv_roller_get_option_count(roller);
    
    if (up) {
        uint16_t new_selected = (current == 0) ? option_count - 1 : current - 1;
        lv_roller_set_selected(roller, new_selected, LV_ANIM_ON);
    } else {
        uint16_t new_selected = (current + 1) % option_count;
        lv_roller_set_selected(roller, new_selected, LV_ANIM_ON);
    }
}

// Process button events in main loop (not ISR)
void processButtonEvents() {
    // Check if there's a pending button event
    if (!button_event_pending) return;
    
    // Get current button states (atomic read)
    uint32_t current_states = button_states;
    button_event_pending = false;  // Clear the flag
    
    // Static variables to track previous states
    static uint32_t prev_states = 0x0F;  // Initialize as all buttons HIGH (not pressed)
    static bool first_run = true;
    
    if (first_run) {
        prev_states = current_states;
        first_run = false;
        return;
    }
    
    // Get current screen
    lv_obj_t *current_screen = lv_scr_act();
    
    // Button mappings
    struct {
        int bit_pos;
        const char* name;
        lv_obj_t *main_btn;
        lv_obj_t *therapy_btn;
    } buttons[] = {
        {0, "BTN1", ui_ButtonLeft1, ui_ButtonLeft2},
        {1, "BTN2", ui_ButtonMidLeft1, ui_ButtonMidLeft2},
        {2, "BTN3", ui_ButtonMidRight1, ui_ButtonMidRight2},
        {3, "BTN4", ui_ButtonRight1, ui_ButtonRight2}
    };
    
    // Process each button
    for (int i = 0; i < 4; i++) {
        bool prev_state = (prev_states >> buttons[i].bit_pos) & 1;
        bool curr_state = (current_states >> buttons[i].bit_pos) & 1;
        
        // Button press detected (HIGH -> LOW, since buttons are active LOW)
        if (prev_state == 1 && curr_state == 0) {
            debugprintf("%s-PRESS on %s\n", buttons[i].name, 
                       (current_screen == ui_MainScreen) ? "MainScreen" : 
                       (current_screen == ui_TherapyScreen) ? "TherapyScreen" : "Unknown");
            
            if (current_screen == ui_MainScreen) {
                handleButtonPress(buttons[i].main_btn);
                
                // Special roller control for BUTTON1 and BUTTON2
                if (i == 0) { // BUTTON1 - Roller UP
                    adjustRoller(ui_Roller_Topic1, true);
                    debugprintln("Roller UP");
                } else if (i == 1) { // BUTTON2 - Roller DOWN
                    adjustRoller(ui_Roller_Topic1, false);
                    debugprintln("Roller DOWN");
                }
                
            } else if (current_screen == ui_TherapyScreen) {
                handleButtonPress(buttons[i].therapy_btn);
            }
        }
        
        // Button release detected (LOW -> HIGH)
        else if (prev_state == 0 && curr_state == 1) {
            debugprintf("%s-RELEASE on %s\n", buttons[i].name,
                       (current_screen == ui_MainScreen) ? "MainScreen" : 
                       (current_screen == ui_TherapyScreen) ? "TherapyScreen" : "Unknown");
            
            if (current_screen == ui_MainScreen) {
                handleButtonRelease(buttons[i].main_btn);
            } else if (current_screen == ui_TherapyScreen) {
                handleButtonRelease(buttons[i].therapy_btn);
            }
        }
    }
    
    // Update previous states
    prev_states = current_states;
}

// Setup function for button interrupts
void setupButtonInterrupts() {
    pinMode(BUTTON1, INPUT_PULLUP);
    pinMode(BUTTON2, INPUT_PULLUP);
    pinMode(BUTTON3, INPUT_PULLUP);
    pinMode(BUTTON4, INPUT_PULLUP);
    
    // Attach interrupts on CHANGE
    attachInterrupt(digitalPinToInterrupt(BUTTON1), buttonFunc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON2), buttonFunc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON3), buttonFunc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON4), buttonFunc, CHANGE);
}


void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.print("Init...");
    SPI.begin(35, 37, 36, -1); // SCLK, MISO, MOSI, SS
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    serialMutex = xSemaphoreCreateMutex();

    // Initialize GPIO
    xTaskCreate(
        taskGPIO, "GPIO",
        2048, NULL, 2, &taskHandleGPIO);

    // Initialize BLE
    xTaskCreate(
        taskBLE, "BLE",
        4096, NULL, 4, &taskHandleBLE);

    // Initialize TFT display
    xTaskCreatePinnedToCore(
        taskDisplay, "Display",
        32768, NULL, 3, &taskHandleDisplay,
        ARDUINO_RUNNING_CORE);

    Serial.println(" done");
}

void loop()
{
    vTaskDelete(NULL);
}