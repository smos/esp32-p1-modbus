#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>         
#include <PubSubClient.h>
#include <Preferences.h>
#include <ModbusServer.h> 
#include "ModbusServerTCPasync.h"
#include <map>
#include <list> 
#include <NTPClient.h>
#include <WiFiUdp.h>        

// Include FastLED library for ESP32-S3 RGB LED control
#if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    #include <FastLED.h>
#endif

// ======================================================
// 1. GLOBAL VARIABLE DECLARATIONS
// ======================================================

#define MQTT_BASE_TOPIC "ESP32-P1-Modbus" 

// --- CONDITIONAL LED PINS & CONFIGURATION ---

#if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    #define LED_TYPE WS2812B
    #define COLOR_ORDER GRB
    #define DATA_PIN 21     // Pin 21 for onboard S3 RGB LED
    #define NUM_LEDS 1
    CRGB leds[NUM_LEDS];
    const CRGB COLOR_WHITE  = CRGB(255, 255, 255);
    const CRGB COLOR_GREEN  = CRGB(0, 255, 0);
    const CRGB COLOR_ORANGE = CRGB(255, 165, 0);
#else 
    const int LED_BUILTIN_PIN = 2; 
#endif

// P1 Serial Config
const int P1_RX_PIN = 16;        
const int P1_BAUD = 115200;      

// P1 Passthrough Server
#define P1_PASSTHROUGH_PORT 8023
AsyncServer passthroughServer(P1_PASSTHROUGH_PORT);
std::list<AsyncClient*> passthroughClients; 

// NEW: NTP and Logging
WiFiUDP ntpUDP;
// NTP server 0.nl.pool.ntp.org, offset +3600 (CET)
NTPClient timeClient(ntpUDP, "0.nl.pool.ntp.org", 3600, 60000); 
std::list<String> logBuffer;
#define MAX_LOG_LINES 50 

// State Variables
unsigned long ledP1FlashStartTime = 0;
unsigned long ledModbusFlashStartTime = 0;
const int LED_FLASH_DURATION = 50; 
bool isP1Connecting = false; 
const char* CONFIG_PINCODE = "1234"; 

// --- WATCHDOG VARIABLES ---
unsigned long lastP1TelegramTime = 0;
const unsigned long P1_TIMEOUT_MS = 60000; // 60 seconds
// --------------------------

Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiClient mqttNetClient;
PubSubClient mqtt(mqttNetClient);
WiFiClient p1Client;
ModbusServerTCPasync MBserver;

// Settings
String p1Host = "192.168.11.223";
uint16_t p1Port = 8088;
bool p1UseSerial = false;
bool p1InvertSignal = true; 
uint8_t modbusAddress = 1; 

String mqttHost = "192.168.1.238";
uint16_t mqttPort = 1883;
String mqttUser = "mqttuser";
String mqttPass = "mqttpass";
bool mqttEnabled = false; 

String preset = "Eastron";
String telegramBuffer;
std::map<uint16_t, uint16_t> holdingRegs;

// ======================================================
// 2. DATA STRUCTURES & MODBUS MAPS (UNCHANGED)
// ======================================================

struct RegisterInfo { uint16_t address; const char* dataType; const char* obis; };
struct ModbusMap { const char* name; RegisterInfo* registers; size_t count; };
struct CustomReg { uint16_t address = 0; char dataType[6] = "float"; char obis[12] = ""; };

#define MAX_CUSTOM_REGS 10
CustomReg customRegs[MAX_CUSTOM_REGS];

struct PhaseData { float voltage = 0.0f; float power = 0.0f; float current = 0.0f; };
PhaseData phases[3];

float energyImport = 0.0f, energyImportT1 = 0.0f, energyImportT2 = 0.0f; 
float energyExport = 0.0f, energyExportT1 = 0.0f, energyExportT2 = 0.0f; 
float frequencyHz  = 50.0f;
float totalDeliveredW = 0.0f, totalReceivedW  = 0.0f, netTotalPowerW  = 0.0f; 
float modbusImportPowerW = 0.0f, modbusExportPowerW = 0.0f; 

// P1 internal variables mapped to display/Modbus names
struct P1Value { const char* name; float* ptr; };
P1Value allP1Values[] = {
    {"Net Total Power (W)", &netTotalPowerW}, {"Total Import Power (W)", &modbusImportPowerW}, 
    {"Total Export Power (W)", &modbusExportPowerW}, 
    {"L1 Voltage", &phases[0].voltage}, {"L2 Voltage", &phases[1].voltage}, {"L3 Voltage", &phases[2].voltage},
    {"L1 Current", &phases[0].current}, {"L2 Current", &phases[1].current}, {"L3 Current", &phases[2].current},
    {"L1 Net Power (W)", &phases[0].power}, {"L2 Net Power (W)", &phases[1].power}, {"L3 Net Power (W)", &phases[2].power},
    {"Total Import kWh", &energyImport}, {"Total Export kWh", &energyExport},
    {"Import T1 kWh", &energyImportT1}, {"Import T2 kWh", &energyImportT2},
    {"Export T1 kWh", &energyExportT1}, {"Export T2 kWh", &energyExportT2}, 
    {"Frequency (Hz)", &frequencyHz},
};
#define P1_VALUE_COUNT (sizeof(allP1Values) / sizeof(P1Value))

static inline uint16_t HR(uint16_t ref4xxxx) { 
    return (ref4xxxx >= 40001) ? (ref4xxxx - 40001) : ref4xxxx; 
}

RegisterInfo eastronRegisters[] = {
    {0x0000, "float", "L1 Voltage"}, {0x0002, "float", "L2 Voltage"}, {0x0004, "float", "L3 Voltage"},
    {0x0006, "float", "L1 Current"}, {0x0008, "float", "L2 Current"}, {0x000A, "float", "L3 Current"},
    {0x000C, "float", "L1 Net Power (W)"}, {0x000E, "float", "L2 Net Power (W)"}, {0x0010, "float", "L3 Net Power (W)"},
    {0x0012, "float", "Net Total Power (W)"}, {0x0014, "float", "Total Import Power (W)"}, 
    {0x0016, "float", "Total Export Power (W)"}, {0x0046, "float", "Total Import kWh"}, {0x0048, "float", "Total Export kWh"},
};

RegisterInfo froniusRegisters[] = {
    {HR(40071), "float", "L1 Current"}, {HR(40079), "float", "L1 Voltage"}, 
    {HR(40095), "float", "Frequency (Hz)"}, {HR(40097), "float", "Net Total Power (W)"},
    {HR(40117), "float", "Total Import kWh"}, {HR(40119), "float", "Total Export kWh"},
};

ModbusMap maps[] = {
    {"Eastron", eastronRegisters, sizeof(eastronRegisters) / sizeof(RegisterInfo)},
    {"Fronius", froniusRegisters, sizeof(froniusRegisters) / sizeof(RegisterInfo)},
    {"Custom", NULL, 0}
};

enum FloatFormat {
    FLOAT_ABCD, 
    FLOAT_CDAB, 
    FLOAT_BADC, 
    FLOAT_DCBA  
};

struct ModbusType {
    const char* name;
    FloatFormat format;
};

const ModbusType supportedFormats[] = {
    {"Modbus Big Endian (ABCD)", FLOAT_ABCD},
    {"Swapped Word (CDAB)", FLOAT_CDAB},
    {"Swapped Byte (BADC)", FLOAT_BADC},
    {"Little Endian Swap (DCBA)", FLOAT_DCBA},
};
#define NUM_SUPPORTED_FORMATS (sizeof(supportedFormats) / sizeof(ModbusType))


// ======================================================
// 3. LOGIC FUNCTIONS & MACROS 
// ======================================================

// Generate human-readable timestamp (or millis if NTP is not ready)
String getTimestamp() {
    if (timeClient.isTimeSet()) {
        time_t rawTime = timeClient.getEpochTime();
        struct tm* timeinfo = localtime(&rawTime);
        char buffer[20];
        // Format: YYYY-MM-DD HH:MM:SS
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
        return String(buffer);
    } else {
        // Fallback to time since boot (millis)
        return "[" + String(millis()) + "ms]";
    }
}

// Custom logging function (UNCHANGED)
void logMessage(const char *format, ...) {
    char loc_buf[256];
    char log_buf[300]; 
    va_list arg;
    va_start(arg, format);
    int len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(arg);

    // 1. Prefix with timestamp and store
    String timestamp = getTimestamp();
    snprintf(log_buf, sizeof(log_buf), "%s %s", timestamp.c_str(), loc_buf);
    
    // 2. Log to serial
    Serial.println(log_buf); 

    // 3. Store in buffer
    logBuffer.push_back(String(log_buf));
    
    // 4. Limit buffer size
    while (logBuffer.size() > MAX_LOG_LINES) {
        logBuffer.pop_front();
    }
    
    // 5. Broadcast to WebSocket clients on the logging tab (prefixed with 'LOG:')
    if (ws.count() > 0) {
        ws.textAll("LOG:" + String(log_buf));
    }
}

// --- CONDITIONAL LED FUNCTIONS (UNCHANGED) ---

#if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)

void setLEDColor(CRGB color) {
    leds[0] = color;
    FastLED.show();
}

void flashLEDColor(CRGB color, unsigned long& flashStartTime) {
    setLEDColor(color);
    flashStartTime = millis();
}

#else

void setLEDState(int state) {
    digitalWrite(LED_BUILTIN_PIN, state);
}

void flashLEDState(unsigned long& flashStartTime) {
    setLEDState(HIGH);
    flashStartTime = millis();
}

#endif

// --- END CONDITIONAL LED FUNCTIONS ---


float extractObisValue(const String& line) {
  int s = line.indexOf('(');
  int e = line.indexOf('*', s + 1);
  if (s >= 0 && e > s) return line.substring(s + 1, e).toFloat();
  return 0.0f;
}

void connectToP1() {
  if (p1UseSerial) return;
  if (!p1Client.connected()) {
    isP1Connecting = true; 
    if (!p1Client.connect(p1Host.c_str(), p1Port)) {
        p1Client.stop(); 
        isP1Connecting = false; 
        logMessage("P1 TCP connection failed for %s:%d", p1Host.c_str(), p1Port); 
    } else {
        isP1Connecting = false; 
        logMessage("P1 TCP connection established to %s:%d", p1Host.c_str(), p1Port); 
        
        #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
            setLEDColor(CRGB::Black); // LED OFF when connected
        #else
            setLEDState(LOW); 
        #endif
        // Reset watchdog upon successful connection
        lastP1TelegramTime = millis(); 
    }
  }
}

void cleanupClient(AsyncClient* client) {
    for (auto it = passthroughClients.begin(); it != passthroughClients.end(); ++it) {
        if (*it == client) {
            delete *it; 
            it = passthroughClients.erase(it);
            return;
        }
    }
}


void parseP1Line(const String& line) {
  telegramBuffer += line + "\n";
  if (line.startsWith("!")) {
    
    // --- WATCHDOG: Reset timer on full telegram ---
    lastP1TelegramTime = millis();
    // ---------------------------------------------
    
    String completeTelegram = telegramBuffer;
    if (!passthroughClients.empty()) {
        for (AsyncClient* client : passthroughClients) {
            if (client->connected() && client->space() > completeTelegram.length()) {
                client->add((const char*)completeTelegram.c_str(), completeTelegram.length());
                client->send();
            }
        }
    }

    int pos = 0;
    float L1_Import_W = 0.0f, L1_Export_W = 0.0f;
    float L2_Import_W = 0.0f, L2_Export_W = 0.0f;
    float L3_Import_W = 0.0f, L3_Export_W = 0.0f;

    while (true) {
      int end = telegramBuffer.indexOf('\n', pos);
      if (end < 0) break;
      String obisLine = telegramBuffer.substring(pos, end);
      pos = end + 1;

      if      (obisLine.startsWith("1-0:32.7.0")) phases[0].voltage = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:52.7.0")) phases[1].voltage = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:72.7.0")) phases[2].voltage = extractObisValue(obisLine);
      
      else if (obisLine.startsWith("1-0:21.7.0")) L1_Import_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:22.7.0")) L1_Export_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:41.7.0")) L2_Import_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:42.7.0")) L2_Export_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:61.7.0")) L3_Import_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:62.7.0")) L3_Export_W = extractObisValue(obisLine) * 1000.0f; 

      else if (obisLine.startsWith("1-0:1.7.0")) totalDeliveredW = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:2.7.0")) totalReceivedW  = extractObisValue(obisLine) * 1000.0f; 
      
      else if (obisLine.startsWith("1-0:1.8.1")) energyImportT1 = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:1.8.2")) energyImportT2 = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:2.8.1")) energyExportT1 = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:2.8.2")) energyExportT2 = extractObisValue(obisLine);
      
      else if (obisLine.startsWith("1-0:14.7.0")) frequencyHz = extractObisValue(obisLine);
    }
    
    phases[0].power = L1_Import_W - L1_Export_W;
    phases[1].power = L2_Import_W - L2_Export_W;
    phases[2].power = L3_Import_W - L3_Export_W;

    for (int i = 0; i < 3; i++) {
        if (abs(phases[i].voltage) > 1.0f) { 
            phases[i].current = abs(phases[i].power / phases[i].voltage);
        } else {
            phases[i].current = 0.0f;
        }
    }
    
    energyImport = energyImportT1 + energyImportT2;
    energyExport = energyExportT1 + energyExportT2;
    netTotalPowerW = totalDeliveredW - totalReceivedW; 
    
    modbusImportPowerW = (netTotalPowerW > 0) ? netTotalPowerW : 0.0f;
    modbusExportPowerW = (netTotalPowerW < 0) ? abs(netTotalPowerW) : 0.0f;
    
    telegramBuffer = "";

    logMessage("Received and processed P1 telegram. Net Power: %.1f W", netTotalPowerW); 

    // --- ENHANCED P1 LED FLASH ---
    #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
        flashLEDColor(COLOR_GREEN, ledP1FlashStartTime); // S3: Blink Green
    #else
        // ...
    #endif
  }
}

// Unified Modbus worker for FC 03 (Holding) and FC 04 (Input)
ModbusMessage readModbusRegisters(ModbusMessage request) {
    uint16_t addr = 0, words = 0;
    request.get(2, addr); request.get(4, words);
    
    // --- ENHANCED MODBUS LED FLASH ---
    #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
        flashLEDColor(COLOR_ORANGE, ledModbusFlashStartTime); // S3: Blink Orange
    #else
        // ...
    #endif
    
    logMessage("Modbus request received (FC:%d, Addr:0x%X, Words:%d)", request.getFunctionCode(), addr, words); 

    ModbusMessage response;
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    for (uint16_t i = 0; i < words; i++) {
        uint16_t regAddr = addr + i;
        response.add(holdingRegs.count(regAddr) ? holdingRegs[regAddr] : (uint16_t)0);
    }
    return response;
}

void writeModbusValue(uint16_t addr, float value, FloatFormat format) {
    union {
        float f;
        uint8_t bytes[4];
    } converter;
    converter.f = value;

    uint8_t byte0 = converter.bytes[0];
    uint8_t byte1 = converter.bytes[1];
    uint8_t byte2 = converter.bytes[2];
    uint8_t byte3 = converter.bytes[3];

    uint16_t W1 = 0; 
    uint16_t W2 = 0; 

    switch (format) {
        case FLOAT_ABCD: 
            W1 = (byte3 << 8) | byte2;
            W2 = (byte1 << 8) | byte0;
            break;
        case FLOAT_CDAB: 
            W1 = (byte1 << 8) | byte0;
            W2 = (byte3 << 8) | byte2;
            break;
        case FLOAT_BADC: 
            W1 = (byte2 << 8) | byte3;
            W2 = (byte0 << 8) | byte1;
            break;
        case FLOAT_DCBA: 
            W1 = (byte0 << 8) | byte1;
            W2 = (byte2 << 8) | byte3;
            break;
        default:
            W1 = (byte3 << 8) | byte2;
            W2 = (byte1 << 8) | byte0;
            break;
    }
    
    holdingRegs[addr] = W1;
    holdingRegs[addr + 1] = W2;
}


// ======================================================
// 4. WEB UI & CONFIGURATION (UNCHANGED)
// ======================================================

void sendLiveData() {
    String html = "<table><tr><th>Metric</th><th>Value</th><th>Unit</th></tr>";
    
    for (size_t i = 0; i < P1_VALUE_COUNT; i++) {
        const char* name = allP1Values[i].name;
        float value = *allP1Values[i].ptr;
        String unit = "";
        
        if (strstr(name, "Voltage")) unit = "V";
        else if (strstr(name, "Current")) unit = "A";
        else if (strstr(name, "Power")) unit = "W";
        else if (strstr(name, "kWh")) unit = "kWh";
        else if (strstr(name, "Hz")) unit = "Hz";
        
        int precision = (unit == "kWh") ? 3 : ((unit == "A") ? 2 : 1);
        
        html += "<tr><td>" + String(name) + "</td><td>" + String(value, precision) + "</td><td>" + unit + "</td></tr>";
    }

    html += "</table>";
    ws.textAll(html);
}

String generateStatusTabContent() {
  String html = "<h3>System Status</h3><table>";
  html += "<tr><td>Modbus TCP IP</td><td><strong>" + WiFi.localIP().toString() + ":502</strong></td></tr>";
  html += "<tr><td>P1 Passthrough IP</td><td><strong>" + WiFi.localIP().toString() + ":" + String(P1_PASSTHROUGH_PORT) + "</strong></td></tr>";
  html += "<tr><td>Modbus Slave ID</td><td><strong>" + String(modbusAddress) + "</strong></td></tr>";
  html += "<tr><td>P1 Ingestion</td><td><strong>" + String(p1UseSerial ? "Serial (RX16)" : "TCP (" + p1Host + ")") + "</strong></td></tr>";
  String mqttStatus = mqttEnabled ? (mqtt.connected() ? "CONNECTED" : "DISCONNECTED") : "DISABLED";
  html += "<tr><td>MQTT Status</td><td><strong>" + mqttStatus + "</strong></td></tr>";
  html += "<tr><td>NTP Time</td><td><strong>" + getTimestamp() + "</strong></td></tr></table>";
  html += "<hr><h3>P1 Live Data</h3><div id='data'>Loading...</div>";
  return html;
}

String generateLogTabContent() {
    String html = "<h3>System Logs</h3><button onclick='clearLogs()'>Clear Display</button><br><br>";
    html += "<pre id='logs' style='white-space: pre-wrap; word-wrap: break-word; height: 400px; overflow-y: scroll; background: #eee; padding: 10px;'>";
    // Populate with existing buffer
    for (const String& log : logBuffer) {
        html += log + "\n";
    }
    html += "</pre>";
    return html;
}

String generateConfigTabContent() {
    String html = "<form action='/config' method='POST' id='configForm'>";
    html += "<h2>Settings</h2><p style='color:red;'>Enter Pincode to Save:</p>";
    html += "Pincode: <input type='password' id='pincode' value='' style='width: 100px;'/>";
    html += "<input type='hidden' name='pincode' id='pincodeHidden' value=''><br><br>";
    
    html += "<h3>Ingestion</h3><table>";
    html += "<tr><td>Mode:</td><td><select name='useSerial'>";
    html += "<option value='0'" + String(!p1UseSerial ? " selected" : "") + ">TCP Client</option>";
    html += "<option value='1'" + String(p1UseSerial ? " selected" : "") + ">Hardware Serial (RX16)</option></select></td></tr>";
    html += "<tr><td>Invert Serial:</td><td><input type='checkbox' name='invert' " + String(p1InvertSignal ? "checked" : "") + "></td></tr>";
    html += "<tr><td>P1 Host:</td><td><input type='text' name='host' value='" + p1Host + "'></td></tr>";
    html += "<tr><td>P1 Port:</td><td><input type='number' name='port' value='" + String(p1Port) + "'></td></tr>";
    html += "<tr><td>Modbus ID:</td><td><input type='number' name='modbusId' value='" + String(modbusAddress) + "' min='1' max='247'></td></tr>";
    html += "<tr><td>Preset:</td><td><select name='preset'><option " + String(preset=="Eastron"?"selected":"") + ">Eastron</option>";
    html += "<option " + String(preset=="Fronius"?"selected":"") + ">Fronius</option><option " + String(preset=="Custom"?"selected":"") + ">Custom</option></select></td></tr></table>";

    html += "<h3>MQTT</h3><table>";
    html += "<tr><td>Enable:</td><td><input type='checkbox' name='mqttEnabled' " + String(mqttEnabled ? "checked" : "") + "></td></tr>";
    html += "<tr><td>Host:</td><td><input type='text' name='mqttHost' value='" + mqttHost + "'></td></tr>";
    html += "<tr><td>User:</td><td><input type='text' name='mqttUser' value='" + mqttUser + "'></td></tr>";
    html += "<tr><td>Pass:</td><td><input type='password' name='mqttPass' value='" + mqttPass + "'></td></tr></table>";
    
    html += "<br><input type='button' value='Save & Restart' onclick='submitConfig()'></form>";
    
    // --- MODBUS REGISTER TABLES ---
    
    html += "<h2>Modbus Register Maps (Slave ID: " + String(modbusAddress) + ")</h2>";
    
    // 1. Eastron Map
    html += "<h3>Eastron Preset (FC 04 recommended)</h3>";
    html += "<table style='width: auto;'><thead><tr><th>Modbus IR/HR Address (4xxxx)</th><th>Address (Hex)</th><th>P1 Value (OBIS Name)</th><th>Data Type</th></tr></thead><tbody>";
    for (size_t i = 0; i < maps[0].count; i++) {
        uint16_t hr_index = maps[0].registers[i].address;
        uint16_t modbus_addr = hr_index + 40001;
        html += "<tr><td>" + String(modbus_addr) + "</td><td>0x" + String(hr_index, HEX) + "</td><td>" + maps[0].registers[i].obis + "</td><td>" + maps[0].registers[i].dataType + "</td></tr>";
    }
    html += "</tbody></table>";
    
    // 2. Fronius Map
    html += "<h3>Fronius Preset (FC 03 recommended)</h3>";
    html += "<table style='width: auto;'><thead><tr><th>Modbus IR/HR Address (4xxxx)</th><th>Address (Hex)</th><th>P1 Value (OBIS Name)</th><th>Data Type</th></tr></thead><tbody>";
    for (size_t i = 0; i < maps[1].count; i++) {
        uint16_t hr_index = maps[1].registers[i].address;
        uint16_t modbus_addr = hr_index + 40001;
        html += "<tr><td>" + String(modbus_addr) + "</td><td>0x" + String(hr_index, HEX) + "</td><td>" + maps[1].registers[i].obis + "</td><td>" + maps[1].registers[i].dataType + "</td></tr>";
    }
    html += "</tbody></table>";
    
    // 3. Float Formats
    html += "<h3>Modbus Float Formats (4 Bytes)</h3><p>Current format used: <strong>Swapped Word (CDAB) - Confirmed working with `-Ble` and `-Bmixed` client flags.</strong></p>";
    html += "<table style='width: auto;'><thead><tr><th>Format Name</th><th>Byte Order (W1 W2)</th><th>Works with Client Flag (Example)</th><th></th></tr></thead><tbody>";
    for (size_t i = 0; i < NUM_SUPPORTED_FORMATS; i++) {
        String name = supportedFormats[i].name;
        String byteOrder = "";
        String clientFlag = "";
        
        switch (supportedFormats[i].format) {
            case FLOAT_ABCD: byteOrder = "[B3 B2] [B1 B0]"; clientFlag = "-Bbf (Big Endian)"; break;
            case FLOAT_CDAB: byteOrder = "[B1 B0] [B3 B2]"; clientFlag = "-Ble, -Bmixed"; break; 
            case FLOAT_BADC: byteOrder = "[B2 B3] [B0 B1]"; clientFlag = "N/A (Middle Endian)"; break;
            case FLOAT_DCBA: byteOrder = "[B0 B1] [B2 B3]"; clientFlag = "-Bbe (Big Endian Word Swap)"; break; 
        }
        
        html += "<tr><td>" + name + "</td><td>" + byteOrder + "</td><td>" + clientFlag + "</td><td>" + (supportedFormats[i].format == FLOAT_CDAB ? "RECOMMENDED" : "") + "</td></tr>";
    }
    html += "</tbody></table>";
    return html;
}

// Setup P1 Passthrough TCP Server (UNCHANGED)
void setupPassthroughServer() {
    passthroughServer.onClient([](void* arg, AsyncClient* client) {
        logMessage("P1 Passthrough: New client connected from %s", client->remoteIP().toString().c_str());
        
        passthroughClients.push_back(client);
        
        client->onData([](void* arg, AsyncClient* c, void* data, size_t len) {
            // Ignore incoming data
        }, NULL);
        
        client->onDisconnect([](void* arg, AsyncClient* c) {
            logMessage("P1 Passthrough: Client disconnected from %s", c->remoteIP().toString().c_str());
            cleanupClient(c);
        }, NULL);
        
        client->onError([](void* arg, AsyncClient* c, int error) {
            logMessage("P1 Passthrough: Client error %d", error);
        }, NULL);
    }, NULL);
    
    passthroughServer.begin();
    logMessage("P1 Passthrough Server started on port %d", P1_PASSTHROUGH_PORT);
}


void setupWebServer() {
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = "<html><head><style>body{font-family:sans-serif;} table{border:1px solid #000; border-collapse:collapse; width:100%;} td,th{padding:8px; border:1px solid #ccc; text-align:left;}";
        html += ".tab{overflow:hidden; border:1px solid #ccc; background:#f1f1f1;} .tab button{float:left; border:none; padding:14px 16px; cursor:pointer;} .tab button:hover{background:#ddd;} .tab button.active{background:#ccc;} .tabcontent{display:none; padding:12px; border:1px solid #ccc; border-top:none;}</style></head><body>";
        html += "<h1>ESP32 P1 Bridge</h1><div class='tab'><button class='tablinks' onclick=\"openTab(event, 'Status')\" id='defaultOpen'>Status</button><button class='tablinks' onclick=\"openTab(event, 'Config')\">Config</button><button class='tablinks' onclick=\"openTab(event, 'Logs')\">Logs</button><a href='/update' style='float:right; padding:14px;'>OTA Update</a></div>";
        
        html += "<div id='Status' class='tabcontent'>" + generateStatusTabContent() + "</div>";
        html += "<div id='Config' class='tabcontent'>" + generateConfigTabContent() + "</div>";
        html += "<div id='Logs' class='tabcontent'>" + generateLogTabContent() + "</div>"; 
        
        html += "<script>function openTab(evt, tn){ var i, tc, tl; tc=document.getElementsByClassName('tabcontent'); for(i=0;i<tc.length;i++)tc[i].style.display='none'; tl=document.getElementsByClassName('tablinks'); for(i=0;i<tl.length;i++)tl[i].className=tl[i].className.replace(' active',''); document.getElementById(tn).style.display='block'; evt.currentTarget.className+=' active'; }";
        
        html += "document.getElementById('defaultOpen').click(); var logsDiv = document.getElementById('logs'); var ws=new WebSocket('ws://'+location.host+'/ws'); ws.onmessage=function(e){";
        
        // JS: Handle live data OR log message
        html += "if (e.data.startsWith('LOG:')) { logsDiv.innerHTML += e.data.substring(4) + '\\n'; logsDiv.scrollTop = logsDiv.scrollHeight; }";
        html += "else { document.getElementById('data').innerHTML=e.data; }";
        
        html += "};";
        html += "function submitConfig(){ var p=document.getElementById('pincode').value; if(p==='" + String(CONFIG_PINCODE) + "'){document.getElementById('pincodeHidden').value=p; document.getElementById('configForm').submit();}else{alert('Wrong Pin');} }";
        html += "function clearLogs() { logsDiv.innerHTML = ''; }</script></body></html>"; // JS clear logs
        request->send(200, "text/html", html);
    });

    server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("pincode", true) || request->getParam("pincode", true)->value() != CONFIG_PINCODE) {
            request->send(403, "text/plain", "Forbidden"); return;
        }
        logMessage("Saving configuration and restarting...");
        prefs.begin("config", false);
        p1UseSerial = request->getParam("useSerial", true)->value() == "1";
        p1InvertSignal = request->hasParam("invert", true);
        p1Host = request->getParam("host", true)->value();
        p1Port = request->getParam("port", true)->value().toInt();
        modbusAddress = request->getParam("modbusId", true)->value().toInt(); 
        preset = request->getParam("preset", true)->value();
        mqttEnabled = request->hasParam("mqttEnabled", true);
        mqttHost = request->getParam("mqttHost", true)->value();
        mqttUser = request->getParam("mqttUser", true)->value();
        mqttPass = request->getParam("mqttPass", true)->value();
        
        prefs.putBool("p1UseSerial", p1UseSerial);
        prefs.putBool("p1Invert", p1InvertSignal);
        prefs.putString("p1Host", p1Host);
        prefs.putUInt("p1Port", p1Port);
        prefs.putUChar("modbusId", modbusAddress);
        prefs.putString("preset", preset);
        prefs.putBool("mqttEnabled", mqttEnabled);
        prefs.putString("mqttHost", mqttHost);
        prefs.putString("mqttUser", mqttUser);
        prefs.putString("mqttPass", mqttPass);
        prefs.end();
        
        request->send(200, "text/html", "Restarting... <script>setTimeout(()=>location.href='/', 5000);</script>");
        delay(1000); ESP.restart();
    });

    ElegantOTA.begin(&server, "admin", "otapassword"); 
    server.begin();
}


// ======================================================
// 5. SETUP & LOOP 
// ======================================================

void setup() {
  Serial.begin(115200);
  
  // --- CONDITIONAL LED INITIALIZATION ---
  #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(25); 
    setLEDColor(CRGB::Red); 
  #else
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW); 
  #endif
  // --- END CONDITIONAL LED INITIALIZATION ---

  prefs.begin("config", true);
  p1UseSerial = prefs.getBool("p1UseSerial", false);
  p1InvertSignal = prefs.getBool("p1Invert", true);
  p1Host = prefs.getString("p1Host", "192.168.1.100");
  p1Port = prefs.getUInt("p1Port", 8088);
  modbusAddress = prefs.getUChar("modbusId", 1); 
  preset = prefs.getString("preset", "Eastron");
  mqttEnabled = prefs.getBool("mqttEnabled", false);
  mqttHost = prefs.getString("mqttHost", "");
  mqttUser = prefs.getString("mqttUser", "");
  mqttPass = prefs.getString("mqttPass", "");
  prefs.end();

  if (p1UseSerial) {
      Serial1.begin(P1_BAUD, SERIAL_8N1, P1_RX_PIN, -1, p1InvertSignal);
  }

  WiFiManager wm;
  logMessage("Connecting to WiFi...");
  wm.autoConnect("P1-Bridge-Setup");
  logMessage("WiFi connected. IP: %s", WiFi.localIP().toString().c_str());
  
  // Initialize NTP Client
  ntpUDP.begin(2390); 
  timeClient.begin();
  timeClient.update();
  logMessage("NTP client started. Server: 0.nl.pool.ntp.org");
  
  setupWebServer();
  setupPassthroughServer(); 
  
  MBserver.registerWorker(modbusAddress, READ_HOLD_REGISTER, &readModbusRegisters); 
  MBserver.registerWorker(modbusAddress, READ_INPUT_REGISTER, &readModbusRegisters); 
  MBserver.start(502, modbusAddress, 20000);
  logMessage("Modbus TCP server started on port 502 (ID: %d)", modbusAddress);
  mqtt.setServer(mqttHost.c_str(), 1883);
  
  // Initialize watchdog timer
  lastP1TelegramTime = millis();
}

void loop() {
  // --- CONDITIONAL LED MANAGEMENT (UPDATED) ---
  #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    if (isP1Connecting) {
        // Red blinking while attempting to connect
        if (millis() % 400 < 200) {
            setLEDColor(CRGB::Red);
        } else {
            setLEDColor(CRGB::Black);
        }
    } else if (ledP1FlashStartTime > 0 && millis() - ledP1FlashStartTime >= LED_FLASH_DURATION) {
        setLEDColor(CRGB::Black); // LED goes Black/OFF after P1 flash
        ledP1FlashStartTime = 0;
    } else if (ledModbusFlashStartTime > 0 && millis() - ledModbusFlashStartTime >= LED_FLASH_DURATION) {
        setLEDColor(CRGB::Black); // LED goes Black/OFF after Modbus flash
        ledModbusFlashStartTime = 0;
    } else if (p1Client.connected() || p1UseSerial) {
        // IDLE: Connected and waiting (LED OFF)
        setLEDColor(CRGB::Black); 
    } else {
        // IDLE: Disconnected (continuous Red)
        setLEDColor(CRGB::Red);
    }
  #else
    // Default LED logic for non-RGB boards (unchanged)
    if (isP1Connecting) {
        digitalWrite(LED_BUILTIN_PIN, (millis() % 400 < 200) ? HIGH : LOW);
    } else if (ledP1FlashStartTime > 0 && millis() - ledP1FlashStartTime >= LED_FLASH_DURATION) {
        digitalWrite(LED_BUILTIN_PIN, LOW); 
        ledP1FlashStartTime = 0;
    } else if (ledModbusFlashStartTime > 0 && millis() - ledModbusFlashStartTime >= LED_FLASH_DURATION) {
        digitalWrite(LED_BUILTIN_PIN, LOW); 
        ledModbusFlashStartTime = 0;
    }
  #endif
  // --- END CONDITIONAL LED MANAGEMENT ---

  // Update NTP client periodically
  timeClient.update();

  static unsigned long lastP1Try = 0, lastUpdate = 0;
  
  // P1 INGESTION LOGIC (Non-blocking character read)
  static String currentP1Line;
  Stream* p1Stream = nullptr;
  
  if (p1UseSerial) {
      p1Stream = &Serial1;
  } else if (p1Client.connected()) {
      p1Stream = &p1Client;
  }

  if (p1Stream) {
      while (p1Stream->available()) {
          char c = p1Stream->read();
          if (c == '\n') {
              parseP1Line(currentP1Line); 
              currentP1Line = ""; 
          } else if (c != '\r') {
              currentP1Line += c;
          }
      }
  } else if (!p1UseSerial) {
      // TCP connection attempt logic (only if using TCP)
      if (millis() - lastP1Try > 5000) {
          lastP1Try = millis(); 
          logMessage("Attempting to connect to P1: %s:%d", p1Host.c_str(), p1Port);
          connectToP1();
      }
  }
  
  // --- P1 WATCHDOG CHECK (NEW) ---
  // If we are connected (Serial or TCP) but haven't seen a telegram in P1_TIMEOUT_MS
  if ((p1UseSerial || p1Client.connected()) && (millis() - lastP1TelegramTime > P1_TIMEOUT_MS)) {
      logMessage("P1 Watchdog: No telegram received for over %d seconds. Forcing reconnect/reset.", P1_TIMEOUT_MS / 1000);
      
      // If using TCP, force disconnect to trigger a reconnect attempt immediately
      if (p1Client.connected()) {
          p1Client.stop(); 
      }
      
      // Reset lastP1Try to 0 to make the TCP connect logic fire on the next loop iteration
      // For Serial, this just resets the logic and logs the issue.
      lastP1Try = 0; 
      
      // Reset watchdog time to prevent immediate re-triggering while connecting
      lastP1TelegramTime = millis();
  }
  // -------------------------------

  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    
    // MODBUS REGISTER MAPPING LOGIC (UNCHANGED)
    holdingRegs.clear();
    const FloatFormat currentFloatFormat = FLOAT_CDAB; // Using Swapped Word (CDAB) as recommended

    for (auto& m : maps) {
        if (preset == m.name && m.registers) {
            for (size_t i=0; i<m.count; i++) {
                for (auto& p : allP1Values) {
                    if (strcmp(m.registers[i].obis, p.name) == 0) {
                        writeModbusValue(m.registers[i].address, *p.ptr, currentFloatFormat);
                    }
                }
            }
        }
    }
    
    sendLiveData(); 
    if (mqttEnabled && mqtt.connected()) {
        mqtt.publish((String(MQTT_BASE_TOPIC) + "/Power").c_str(), String(netTotalPowerW).c_str());
    }
  }

  if (mqttEnabled && !mqtt.connected()) mqtt.connect("ESP32P1", mqttUser.c_str(), mqttPass.c_str());
  mqtt.loop();
}