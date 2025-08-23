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

SemaphoreHandle_t serialMutex;


// BLE Components
NimBLEServer *bleServer;
NimBLECharacteristic *pCharFrequency;
NimBLECharacteristic *pCharStrength;
NimBLECharacteristic *pCharIntensity;
NimBLECharacteristic *pCharBattery;

TaskHandle_t taskHandleDisplay;
TaskHandle_t taskHandleGPIO;
TaskHandle_t taskHandleBLE;

bool standbyStatus, chargeBattStatus, fullBattStatus, bleConnected;

// Display buffer
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 320;

enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI( screenWidth, screenHeight ); /* TFT instance */


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

void IRAM_ATTR buttonFunc();
void bleStateFunc();
void chargeStateFunc();
void onOffSliderFunc();

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
        //buttonFunc();
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
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    pinMode(BUTTON3, INPUT);
    pinMode(BUTTON4, INPUT);
    attachInterrupt(BUTTON1, buttonFunc, FALLING);
    attachInterrupt(BUTTON2, buttonFunc, FALLING);
    attachInterrupt(BUTTON3, buttonFunc, FALLING);
    attachInterrupt(BUTTON4, buttonFunc, FALLING);
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

void IRAM_ATTR buttonFunc(){
    static uint32_t lastButtonPress = 0;
    uint32_t currentTime = millis();

    // Debounce logic: ignore button presses within 20ms of the last press
    if (currentTime - lastButtonPress < 20) {
        return;
    }
    lastButtonPress = currentTime;

    // Handle button presses
    if (digitalRead(BUTTON1) == LOW) {
        debugprintln("BTN1-P");
        // Simulate press and release of ButtonLeft1
        if (ui_ButtonLeft1) {
            lv_obj_add_state(ui_ButtonLeft1, LV_STATE_PRESSED);
            lv_obj_send_event(ui_ButtonLeft1, LV_EVENT_PRESSED, NULL);
            
            // Schedule release after a short delay
            lv_timer_t * timer = lv_timer_create([](lv_timer_t * timer) {
                lv_obj_t *btn = (lv_obj_t*)timer->user_data;
                lv_obj_remove_state(btn, LV_STATE_PRESSED);
                lv_obj_send_event(btn, LV_EVENT_RELEASED, NULL);
                lv_obj_send_event(btn, LV_EVENT_CLICKED, NULL);
                lv_timer_del(timer);
            }, 100, ui_ButtonLeft1);
        }
    }
    if (digitalRead(BUTTON2) == LOW) {
        debugprintln("BTN2-P");
        // Simulate press and release of ButtonMidLeft1
        if (ui_ButtonMidLeft1) {
            lv_obj_add_state(ui_ButtonMidLeft1, LV_STATE_PRESSED);
            lv_obj_send_event(ui_ButtonMidLeft1, LV_EVENT_PRESSED, NULL);
            
            // Schedule release after a short delay
            lv_timer_t * timer = lv_timer_create([](lv_timer_t * timer) {
                lv_obj_t *btn = (lv_obj_t*)timer->user_data;
                lv_obj_remove_state(btn, LV_STATE_PRESSED);
                lv_obj_send_event(btn, LV_EVENT_RELEASED, NULL);
                lv_obj_send_event(btn, LV_EVENT_CLICKED, NULL);
                lv_timer_del(timer);
            }, 100, ui_ButtonMidLeft1);
        }
    }
    if (digitalRead(BUTTON3) == LOW) {
        debugprintln("BTN3-P");
        // Simulate press and release of ButtonMidRight1
        if (ui_ButtonMidRight1) {
            lv_obj_add_state(ui_ButtonMidRight1, LV_STATE_PRESSED);
            lv_obj_send_event(ui_ButtonMidRight1, LV_EVENT_PRESSED, NULL);
            
            // Schedule release after a short delay
            lv_timer_t * timer = lv_timer_create([](lv_timer_t * timer) {
                lv_obj_t *btn = (lv_obj_t*)timer->user_data;
                lv_obj_remove_state(btn, LV_STATE_PRESSED);
                lv_obj_send_event(btn, LV_EVENT_RELEASED, NULL);
                lv_obj_send_event(btn, LV_EVENT_CLICKED, NULL);
                lv_timer_del(timer);
            }, 100, ui_ButtonMidRight1);
        }
    }
    if (digitalRead(BUTTON4) == LOW) {
        debugprintln("BTN4-P");
        // Simulate press and release of ButtonRight1
        if (ui_ButtonRight1) {
            lv_obj_add_state(ui_ButtonRight1, LV_STATE_PRESSED);
            lv_obj_send_event(ui_ButtonRight1, LV_EVENT_PRESSED, NULL);
            
            // Schedule release after a short delay
            lv_timer_t * timer = lv_timer_create([](lv_timer_t * timer) {
                lv_obj_t *btn = (lv_obj_t*)timer->user_data;
                lv_obj_remove_state(btn, LV_STATE_PRESSED);
                lv_obj_send_event(btn, LV_EVENT_RELEASED, NULL);
                lv_obj_send_event(btn, LV_EVENT_CLICKED, NULL);
                lv_timer_del(timer);
            }, 100, ui_ButtonRight1);
        }
    }
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
        16384, NULL, 3, &taskHandleDisplay,
        ARDUINO_RUNNING_CORE);

    Serial.println(" done");
}

void loop()
{
    vTaskDelete(NULL);
}