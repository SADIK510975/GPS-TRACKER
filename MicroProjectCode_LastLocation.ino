#include <Arduino.h>
#include <TinyGPS++.h>
#include <NimBLEDevice.h>   // NimBLE-Arduino v2.x
#include <Preferences.h>    // flash storage (survives power-off)
#include <esp_system.h>

// GPS on the board's RX/TX pins (UART0). No custom pins.
// Set Tools > USB CDC On Boot > Enabled, and use the native "USB" port.
#if ARDUINO_USB_CDC_ON_BOOT
#define GPS_PORT Serial0
#define USB_LOG 1
#else
#define GPS_PORT Serial
#define USB_LOG 0
#endif

#define GPS_BAUD     9600
#define MONITOR_BAUD 115200
#define DEVICE_NAME  "ESP32-GPS"

#define GPS_OFF_TIMEOUT_MS  5000    // no bytes for 5 s -> GPS considered OFF
#define SAVE_INTERVAL_MS    60000   // save last fix to flash at most once/min

// Nordic UART Service - Serial Bluetooth Terminal detects this automatically
#define SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> ESP
#define TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP -> phone

TinyGPSPlus gps;
Preferences prefs;

NimBLEServer*         pServer = nullptr;
NimBLECharacteristic* pTxChar = nullptr;

volatile bool     deviceConnected = false;
volatile bool     sendWelcome     = false;
volatile uint16_t bleMtu          = 23;

bool          streamRawNmea    = false;
unsigned long updateIntervalMs = 2000;

// Command received from the phone, handled later in loop()
char          cmdBuf[64];
volatile bool cmdReady = false;

// Time of the last byte received from the GPS (0 = never)
unsigned long lastGpsByteTime = 0;


// ------------------------------------------------------------
// Last known location
// ------------------------------------------------------------

struct LastFix {
    bool          valid;
    double        lat;
    double        lon;
    double        alt;
    uint32_t      sats;
    float         hdop;
    char          utc[10];     // GPS UTC time when the fix was taken
    char          date[11];    // GPS date when the fix was taken
    unsigned long atMillis;    // uptime when taken (0 = loaded from flash)
};

LastFix lastFix = { false };
unsigned long lastSaveTime = 0;
bool          unsavedFix   = false;


bool gpsIsOff() {
    if (lastGpsByteTime == 0) {
        return millis() > GPS_OFF_TIMEOUT_MS;   // never received anything
    }
    return millis() - lastGpsByteTime > GPS_OFF_TIMEOUT_MS;
}


bool hasFix() {
    return !gpsIsOff() &&
           gps.location.isValid() &&
           gps.location.age() < 3000;
}


void saveLastFix() {
    prefs.putBytes("fix", &lastFix, sizeof(lastFix));
    lastSaveTime = millis();
    unsavedFix = false;
}


void loadLastFix() {
    if (prefs.getBytesLength("fix") == sizeof(lastFix)) {
        prefs.getBytes("fix", &lastFix, sizeof(lastFix));
        lastFix.atMillis = 0;   // uptime from a previous power-on is meaningless
    }
}


// Copy the current live fix into lastFix
void updateLastFix() {
    lastFix.valid    = true;
    lastFix.lat      = gps.location.lat();
    lastFix.lon      = gps.location.lng();
    lastFix.alt      = gps.altitude.meters();
    lastFix.sats     = gps.satellites.value();
    lastFix.hdop     = gps.hdop.hdop();
    lastFix.atMillis = millis();

    if (gps.time.isValid()) {
        snprintf(lastFix.utc, sizeof(lastFix.utc), "%02d:%02d:%02d",
                 gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        strcpy(lastFix.utc, "--:--:--");
    }

    if (gps.date.isValid()) {
        snprintf(lastFix.date, sizeof(lastFix.date), "%02d/%02d/%04d",
                 gps.date.day(), gps.date.month(), gps.date.year());
    } else {
        strcpy(lastFix.date, "--/--/----");
    }

    unsavedFix = true;
    if (millis() - lastSaveTime > SAVE_INTERVAL_MS || lastSaveTime == 0) {
        saveLastFix();
    }
}


// ------------------------------------------------------------
// Output helpers
// ------------------------------------------------------------

void bleSend(const String& line) {
    if (!deviceConnected || pTxChar == nullptr) {
        return;
    }

    String s = line + "\r\n";
    size_t chunk = (bleMtu > 23) ? (bleMtu - 3) : 20;

    for (size_t i = 0; i < s.length(); i += chunk) {
        size_t n = min(chunk, (size_t)(s.length() - i));
        pTxChar->setValue((const uint8_t*)s.c_str() + i, n);
        pTxChar->notify();
        delay(5);
    }
}


// Print to USB (if available) and to the phone
void out(const String& s) {
#if USB_LOG
    Serial.println(s);
#endif
    bleSend(s);
}


// ------------------------------------------------------------
// BLE callbacks (kept short - real work happens in loop)
// ------------------------------------------------------------

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override {
        deviceConnected = true;
    }

    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        bleMtu = 23;
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        bleMtu = MTU;
    }
};


class TxCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& connInfo,
                     uint16_t subValue) override {
        if (subValue > 0) {
            sendWelcome = true;
        }
    }
};


class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        if (cmdReady) {
            return;  // previous command not handled yet
        }
        std::string v = c->getValue();
        size_t n = min(v.length(), sizeof(cmdBuf) - 1);
        memcpy(cmdBuf, v.data(), n);
        cmdBuf[n] = '\0';
        cmdReady = true;
    }
};


// ------------------------------------------------------------
// Text builders
// ------------------------------------------------------------

String utcString() {
    char utc[10] = "--:--:--";
    if (gps.time.isValid()) {
        snprintf(utc, sizeof(utc), "%02d:%02d:%02d",
                 gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    return String(utc);
}


// "how long ago" text for the last fix
String lastFixAge() {
    if (lastFix.atMillis == 0) {
        return "before last power-off";
    }
    unsigned long sec = (millis() - lastFix.atMillis) / 1000;
    if (sec < 60)   return String(sec) + " s ago";
    if (sec < 3600) return String(sec / 60) + " min ago";
    return String(sec / 3600) + " h " + String((sec % 3600) / 60) + " min ago";
}


String lastFixLine(const char* prefix) {
    char line[200];
    snprintf(line, sizeof(line),
        "%s %.6f %c, %.6f %c | Alt %.1f m | Taken %s %s UTC (%s)",
        prefix,
        fabs(lastFix.lat), lastFix.lat >= 0 ? 'N' : 'S',
        fabs(lastFix.lon), lastFix.lon >= 0 ? 'E' : 'W',
        lastFix.alt,
        lastFix.date, lastFix.utc,
        lastFixAge().c_str());
    return String(line);
}


String telemetryLine() {
    char line[160];

    // 1) GPS is on and has a live fix
    if (hasFix()) {
        updateLastFix();

        double lat = gps.location.lat();
        double lon = gps.location.lng();

        snprintf(line, sizeof(line),
            "[%s] LIVE %.6f %c, %.6f %c | Sats %lu | Alt %.1f m | %.1f km/h | HDOP %.1f",
            utcString().c_str(),
            fabs(lat), lat >= 0 ? 'N' : 'S',
            fabs(lon), lon >= 0 ? 'E' : 'W',
            (unsigned long)gps.satellites.value(),
            gps.altitude.meters(), gps.speed.kmph(), gps.hdop.hdop());
        return String(line);
    }

    // 2) GPS is OFF (no data coming in)
    if (gpsIsOff()) {
        if (lastFix.valid) {
            return lastFixLine("[GPS OFF] Last location:");
        }
        return "[GPS OFF] No saved location yet.";
    }

    // 3) GPS is on but has no fix yet
    if (lastFix.valid) {
        return lastFixLine("[Searching] Last location:");
    }

    snprintf(line, sizeof(line),
        "[%s] No fix yet | Sats %lu | GPS bytes %lu",
        utcString().c_str(),
        (unsigned long)gps.satellites.value(),
        (unsigned long)gps.charsProcessed());
    return String(line);
}


void printHelp() {
    out("Commands:");
    out("  help      - show this list");
    out("  status    - GPS and link details");
    out("  last      - show last known location");
    out("  raw on    - stream raw NMEA sentences");
    out("  raw off   - stop raw NMEA");
    out("  rate N    - update every N seconds (1-60)");
    out("  map       - Google Maps link (live or last location)");
}


void printStatus() {
    out("---- Status ----");
    out("GPS power: " + String(gpsIsOff() ? "OFF / no data" : "ON"));
    out("Fix: " + String(hasFix() ? "YES" : "NO"));
    out("Satellites: " + String(gps.satellites.value()));
    out("GPS bytes received: " + String(gps.charsProcessed()));
    out("Checksum errors: " + String(gps.failedChecksum()));
    out("Saved location: " + String(lastFix.valid ? "YES" : "NO"));
    out("Update interval: " + String(updateIntervalMs / 1000) + " s");
    out("Raw NMEA: " + String(streamRawNmea ? "ON" : "OFF"));
    out("BLE packet size: " + String(bleMtu - 3) + " bytes");
    out("Uptime: " + String(millis() / 1000) + " s");
    out("----------------");
}


// ------------------------------------------------------------
// Command handling
// ------------------------------------------------------------

void handleCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.length() == 0) {
        return;
    }

    if (cmd == "help" || cmd == "?") {
        printHelp();
    }
    else if (cmd == "status") {
        printStatus();
    }
    else if (cmd == "last") {
        if (lastFix.valid) out(lastFixLine("Last location:"));
        else               out("No saved location yet.");
    }
    else if (cmd == "raw on" || cmd == "raw:1") {
        streamRawNmea = true;
        out("Raw NMEA: ON");
    }
    else if (cmd == "raw off" || cmd == "raw:0") {
        streamRawNmea = false;
        out("Raw NMEA: OFF");
    }
    else if (cmd.startsWith("rate")) {
        int sec = cmd.substring(4).toInt();
        if (sec >= 1 && sec <= 60) {
            updateIntervalMs = (unsigned long)sec * 1000UL;
            out("Update interval set to " + String(sec) + " s");
        } else {
            out("Use: rate N   (N = 1 to 60 seconds)");
        }
    }
    else if (cmd == "map") {
        if (hasFix()) {
            out("Live: https://maps.google.com/?q=" +
                String(gps.location.lat(), 6) + "," +
                String(gps.location.lng(), 6));
        } else if (lastFix.valid) {
            out("Last known (" + lastFixAge() + "): https://maps.google.com/?q=" +
                String(lastFix.lat, 6) + "," + String(lastFix.lon, 6));
        } else {
            out("No location available yet.");
        }
    }
    else {
        out("Unknown command: " + cmd + "   (type help)");
    }
}


// ------------------------------------------------------------

void setup() {

#if USB_LOG
    Serial.begin(MONITOR_BAUD);
    delay(500);
    Serial.printf("Reset reason: %d  (9 = brownout / power problem)\n",
                  esp_reset_reason());
#endif

    // Load the last saved location from flash
    prefs.begin("gpstrack", false);
    loadLastFix();

#if USB_LOG
    if (lastFix.valid) {
        Serial.println(lastFixLine("Loaded from flash:"));
    } else {
        Serial.println("No saved location in flash.");
    }
#endif

    GPS_PORT.begin(GPS_BAUD);   // default RX/TX pins

    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setMTU(247);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pTxChar = pService->createCharacteristic(
        TX_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pTxChar->setCallbacks(new TxCallbacks());

    NimBLECharacteristic* pRxChar = pService->createCharacteristic(
        RX_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRxChar->setCallbacks(new RxCallbacks());

    pService->start();

    // Advertising: name in main packet, service UUID in scan response
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();

    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.setName(DEVICE_NAME);

    NimBLEAdvertisementData scanData;
    scanData.addServiceUUID(SERVICE_UUID);

    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponseData(scanData);
    pAdvertising->enableScanResponse(true);

    bool advOk = pAdvertising->start();

#if USB_LOG
    Serial.println("Bluetooth ready: " DEVICE_NAME);
    Serial.println(advOk ? "Advertising started OK" : "Advertising FAILED to start");
#endif
}


// ------------------------------------------------------------

void loop() {

    // Every 5 s: report BLE state and restart advertising if it stopped
    static unsigned long lastBleCheck = 0;

    if (millis() - lastBleCheck > 5000) {
        lastBleCheck = millis();

        int conns = pServer->getConnectedCount();
        bool adv = NimBLEDevice::getAdvertising()->isAdvertising();

#if USB_LOG
        Serial.printf("BLE: connected=%d  advertising=%s\n",
                      conns, adv ? "yes" : "no");
#endif

        if (conns == 0 && !adv) {
            deviceConnected = false;
            NimBLEDevice::startAdvertising();
#if USB_LOG
            Serial.println("BLE: advertising was stopped - restarted");
#endif
        }
    }

    // Greet the phone after it connects
    if (sendWelcome) {
        sendWelcome = false;
        delay(300);  // let the app finish setting up
        out("=== " DEVICE_NAME " connected ===");
        printHelp();
        out(telemetryLine());
    }

    // Command from the phone
    if (cmdReady) {
        String cmd = String(cmdBuf);
        cmdReady = false;
        handleCommand(cmd);
    }

#if USB_LOG
    // Same commands from the USB Serial Monitor
    static String usbCmd;
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            handleCommand(usbCmd);
            usbCmd = "";
        } else if (usbCmd.length() < 60) {
            usbCmd += c;
        }
    }
#endif

    // Read GPS
    static String nmeaLine;

    while (GPS_PORT.available() > 0) {
        char c = GPS_PORT.read();
        gps.encode(c);
        lastGpsByteTime = millis();

        if (c == '\n') {
            nmeaLine.trim();
            if (streamRawNmea && nmeaLine.length() > 0) {
                out(nmeaLine);
            }
            nmeaLine = "";
        } else if (c != '\r' && nmeaLine.length() < 100) {
            nmeaLine += c;
        }
    }

    // Detect GPS turning OFF / ON (report once per change)
    static bool wasOff = false;
    bool nowOff = gpsIsOff();

    if (nowOff && !wasOff) {
        out("GPS turned OFF (no data). Showing last known location.");
        if (unsavedFix) saveLastFix();          // make sure the latest fix is kept
        if (lastFix.valid) out(lastFixLine("Last location:"));
    }
    else if (!nowOff && wasOff) {
        out("GPS turned ON - searching for fix...");
    }
    wasOff = nowOff;

    // Telemetry
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > updateIntervalMs) {
        lastUpdate = millis();
        out(telemetryLine());
    }
}
