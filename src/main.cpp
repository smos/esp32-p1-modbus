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

#if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    #include <FastLED.h>
#endif

// ======================================================
// 1. GLOBAL VARIABLE DECLARATIONS
// ======================================================

#define MQTT_BASE_TOPIC "ESP32-P1-Modbus" 

#if defined(ARDUINO_ESP32S3_DEV) 
    const int P1_RX_PIN = 18;        
    const char* P1_PIN_LABEL = "GPIO18";
    const int P1_BAUD = 115200;      
#else 
    const int P1_RX_PIN = 16;        
    const char* P1_PIN_LABEL = "GPIO16";
    const int P1_BAUD = 115200;      
#endif

#if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    #define LED_TYPE WS2812B
    #define COLOR_ORDER GRB
    #define DATA_PIN 21     
    #define NUM_LEDS 1
    CRGB leds[NUM_LEDS];
    const CRGB COLOR_GREEN  = CRGB(0, 255, 0);
    const CRGB COLOR_ORANGE = CRGB(255, 165, 0);
#else 
    const int LED_BUILTIN_PIN = 2; 
#endif

// Put near your LED helpers
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 0  // set to 0 if your LED turns ON with LOW
#endif

uint16_t p1PassthroughPort = 8088;
AsyncServer* passthroughServer = nullptr;
std::list<AsyncClient*> passthroughClients; 

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "0.nl.pool.ntp.org", 3600, 60000); 
std::list<String> logBuffer;
#define MAX_LOG_LINES 50 

unsigned long ledP1FlashStartTime = 0;
unsigned long ledModbusFlashStartTime = 0;
const int LED_FLASH_DURATION = 50; 
bool isP1Connecting = false; 

String configPincode = "1234"; 
unsigned long lastP1TelegramTime = 0;
const unsigned long P1_TIMEOUT_MS = 60000; 

Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiClient mqttNetClient;
PubSubClient mqtt(mqttNetClient);
WiFiClient p1Client;
ModbusServerTCPasync MBserver;

String p1Host = "192.168.11.223";
uint16_t p1Port = 8088;
bool p1UseSerial = false;
bool p1InvertSignal = true; 

// Modbus Settings
uint8_t modbusAddress = 1; 
String preset = "Eastron";
enum FloatFormat { FLOAT_ABCD, FLOAT_CDAB, FLOAT_BADC, FLOAT_DCBA };
uint8_t mbFormat = FLOAT_ABCD; 
uint8_t mbDataType = 0; // 0: Float, 1: INT32
float mbDivision = 1.0;

String mqttHost = "";
uint16_t mqttPort = 1883;
String mqttUser = "";
String mqttPass = "";
bool mqttEnabled = false; 

String telegramBuffer;
String rawTelegramCapture; 

// --- Speedwire (SMA Energy Meter over UDP multicast) ---
WiFiUDP speedwireUDP;

bool    swEnabled = false;
String  swAddr    = "239.12.255.254";   // default multicast
uint16_t swPort   = 9522;               // default port

// Identity of the emulated meter (6 bytes = SUSyID (2) + Serial (4))
// Use EMETER-20 SUSyID 0x015D by default. Serial derived from ESP32 chip ID.
// (Alternatives seen in the field: EMETER-10 0x010E, SHM2.0 0x0174)  [5](http://gitmemories.com/RalfOGit/sma-emeter-simulator)
uint16_t swSusyId = 0x015D;
uint32_t swSerial = 0;  // filled in setup from efuse MAC lower 4 bytes

// --- Speedwire device type selection ---
enum SwDeviceType : uint8_t { SW_DEV_EMETER10 = 0, SW_DEV_EMETER20 = 1, SW_DEV_SHM20 = 2, SW_DEV_CUSTOM = 3 };
uint8_t swDeviceType = SW_DEV_EMETER20;   // default EMETER-20 (0x015D)

// Map device type to default SUSyID (can be overridden in UI)
static inline uint16_t defaultSusyFor(uint8_t dt) {
  switch (dt) {
    case SW_DEV_EMETER10: return 0x010E; // EMETER-10
    case SW_DEV_EMETER20: return 0x015D; // EMETER-20
    case SW_DEV_SHM20:    return 0x0174; // Sunny Home Manager 2.0
    default:              return 0x015D;
  }
}

// internal pacing
unsigned long lastSpeedwireTx = 0;

// ======================================================
// 2. DATA STRUCTURES & P1 VALUES
// ======================================================

// Virtual line-to-line voltages (derived): AB, BC, CA
float voltageAB = 0.0f;
float voltageBC = 0.0f;
float voltageCA = 0.0f;

struct PhaseData {
  float voltage = 0.0f;
  float power = 0.0f;
  float current = 0.0f;
};
PhaseData phases[3];

float energyImport = 0.0f, energyImportT1 = 0.0f, energyImportT2 = 0.0f; 
float energyExport = 0.0f, energyExportT1 = 0.0f, energyExportT2 = 0.0f; 
float frequencyHz  = 50.0f;
float totalDeliveredW = 0.0f, totalReceivedW  = 0.0f, netTotalPowerW  = 0.0f; 
float modbusImportPowerW = 0.0f, modbusExportPowerW = 0.0f; 

struct P1Value { 
    const char* name; 
    float* ptr; 
    uint16_t regAddr; 
};

P1Value allP1Values[] = {
    {"Net Total Power (W)", &netTotalPowerW, 0x0012},
    {"Total Import Power (W)", &modbusImportPowerW, 0x0014}, 
    {"Total Export Power (W)", &modbusExportPowerW, 0x0016}, 
    {"L1 Voltage", &phases[0].voltage, 0x0000},
    {"L2 Voltage", &phases[1].voltage, 0x0002},
    {"L3 Voltage", &phases[2].voltage, 0x0004},
    {"L1 Current", &phases[0].current, 0x0006},
    {"L2 Current", &phases[1].current, 0x0008},
    {"L3 Current", &phases[2].current, 0x000A},
    {"L1 Power (W)", &phases[0].power, 0x0018},
    {"L2 Power (W)", &phases[1].power, 0x001A},
    {"L3 Power (W)", &phases[2].power, 0x001C},
    {"Voltage AB", &voltageAB, 0x000C},
    {"Voltage BC", &voltageBC, 0x000E},
    {"Voltage CA", &voltageCA, 0x0010},
    {"Total Import kWh", &energyImport, 0x0046},
    {"Total Export kWh", &energyExport, 0x0048},
    {"Frequency (Hz)", &frequencyHz, 0x004C}
};
#define P1_VALUE_COUNT (sizeof(allP1Values) / sizeof(P1Value))

// Per-register mapping (parallel to allP1Values[])
enum RegType : uint8_t { REG_HOLDING = 0, REG_INPUT = 1 };

uint8_t  mbRegType[P1_VALUE_COUNT];    // 0=Holding, 1=Input
uint16_t mbRegAddr[P1_VALUE_COUNT];    // 4xxxx or 3xxxx (your server space)
float    mbRegMul[P1_VALUE_COUNT];     // NEW: multiplier (for INT32 accuracy)

// Separate maps for serving Modbus function 3 & 4
std::map<uint16_t, uint16_t> inputRegs;    // new
std::map<uint16_t, uint16_t> holdingRegs;  // existing

// --- Helpers to append big-endian values into a byte vector ---
static inline void be16(std::vector<uint8_t>& b, uint16_t v){ b.push_back(v>>8); b.push_back(v&0xFF); }
static inline void be32(std::vector<uint8_t>& b, uint32_t v){ b.push_back(v>>24); b.push_back((v>>16)&0xFF); b.push_back((v>>8)&0xFF); b.push_back(v&0xFF); }
static inline void be64(std::vector<uint8_t>& b, uint64_t v){
  for (int i=7;i>=0;--i) b.push_back((uint8_t)((v >> (i*8)) & 0xFF));
}

// OBIS helper: bytes B,C,D,E as per SMA EMETER protocol (big-endian)  [1](https://cdn.sma.de/fileadmin/content/www.developer.sma.de/docs/EMETER-Protokoll-TI-en-10.pdf?v=1699276024)
static inline void obis(std::vector<uint8_t>& b, uint8_t B, uint8_t C, uint8_t D, uint8_t E){
  b.push_back(B); b.push_back(C); b.push_back(D); b.push_back(E);
}

// ======================================================
// 3. CORE LOGIC FUNCTIONS
// ======================================================

String getUptime() {
    unsigned long sec = millis() / 1000;
    int days = sec / 86400;
    int hours = (sec % 86400) / 3600;
    int mins = (sec % 3600) / 60;
    int secs = sec % 60;
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%dd %02dh %02dm %02ds", days, hours, mins, secs);
    return String(buffer);
}

String getTimestamp() {
    if (timeClient.isTimeSet()) {
        time_t rawTime = timeClient.getEpochTime();
        struct tm* timeinfo = localtime(&rawTime);
        char buffer[20];
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
        return String(buffer);
    }
    return "[" + String(millis()) + "ms]";
}

void logMessage(const char *format, ...) {
    char loc_buf[256];
    char log_buf[300]; 
    va_list arg;
    va_start(arg, format);
    vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(arg);
    String timestamp = getTimestamp();
    snprintf(log_buf, sizeof(log_buf), "%s %s", timestamp.c_str(), loc_buf);
    Serial.println(log_buf); 
    logBuffer.push_back(String(log_buf));
    while (logBuffer.size() > MAX_LOG_LINES) logBuffer.pop_front();
    if (ws.count() > 0) ws.textAll("LOG:" + String(log_buf));
}

#if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
void setLEDColor(CRGB color) { leds[0] = color; FastLED.show(); }
void flashLEDColor(CRGB color, unsigned long& flashStartTime) { setLEDColor(color); flashStartTime = millis(); }
#else
void setLEDState(int state) { digitalWrite(LED_BUILTIN_PIN, state); }
void flashLEDState(unsigned long& flashStartTime) { setLEDState(HIGH); flashStartTime = millis(); }
#endif

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
    } else {
        isP1Connecting = false; 
        logMessage("P1 TCP connection established to %s:%d", p1Host.c_str(), p1Port); 
        #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
            setLEDColor(CRGB::Black); 
        #else
            setLEDState(LOW); 
        #endif
        lastP1TelegramTime = millis(); 
    }
  }
}

void setupPassthroughServer() {
    if (passthroughServer) { passthroughServer->end(); delete passthroughServer; }
    passthroughServer = new AsyncServer(p1PassthroughPort);
    passthroughServer->onClient([](void* arg, AsyncClient* client) {
        passthroughClients.push_back(client);
        client->onDisconnect([](void* arg, AsyncClient* c) { 
            for (auto it = passthroughClients.begin(); it != passthroughClients.end(); ++it) {
                if (*it == c) { delete *it; passthroughClients.erase(it); return; }
            }
        }, NULL);
    }, NULL);
    passthroughServer->begin();
}

void parseP1Line(const String& line) {
  telegramBuffer += line + "\n";
  if (line.startsWith("!")) {
    lastP1TelegramTime = millis();
    int pos = 0;
    float L1_Import_W = 0.0f, L1_Export_W = 0.0f, L2_Import_W = 0.0f, L2_Export_W = 0.0f, L3_Import_W = 0.0f, L3_Export_W = 0.0f;

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
        if (abs(phases[i].voltage) > 1.0f) phases[i].current = abs(phases[i].power / phases[i].voltage);
        else phases[i].current = 0.0f;
    }

    // Derive L-L voltages assuming 120° phase separation:
    // |Vab| = sqrt(Va^2 + Vb^2 + Va*Vb), similarly for BC, CA.  (cos 120° = -1/2)
    auto Vll = [](float Va, float Vb) -> float {
      if (fabsf(Va) < 1.0f && fabsf(Vb) < 1.0f) return 0.0f;
      return sqrtf(Va*Va + Vb*Vb + Va*Vb);
    };
    voltageAB = Vll(phases[0].voltage, phases[1].voltage);
    voltageBC = Vll(phases[1].voltage, phases[2].voltage);
    voltageCA = Vll(phases[2].voltage, phases[0].voltage);
    
    energyImport = energyImportT1 + energyImportT2;
    energyExport = energyExportT1 + energyExportT2;
    netTotalPowerW = totalDeliveredW - totalReceivedW; 
    modbusImportPowerW = (netTotalPowerW > 0) ? netTotalPowerW : 0.0f;
    modbusExportPowerW = (netTotalPowerW < 0) ? abs(netTotalPowerW) : 0.0f;
    telegramBuffer = "";

    #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
        flashLEDColor(COLOR_GREEN, ledP1FlashStartTime);
    #endif
    #if !defined(ARDUINO_ESP32S3_DEV) && !defined(BOARD_HAS_RGB_LED)
        flashLEDState(ledP1FlashStartTime);  // turn LED ON briefly for P1 telegram
    #endif

  }
}

void writeHoldingWithScale(uint16_t addr, float value, float multiplier) {
  uint16_t W1 = 0, W2 = 0;

  if (mbDataType == 1) { // INT32: apply multiplier to shift decimals
    double scaled = (double)value * (multiplier == 0.0 ? 1.0 : multiplier);
    int32_t iVal = (int32_t) llround(scaled);
    W1 = (uint16_t)(iVal >> 16);
    W2 = (uint16_t)(iVal & 0xFFFF);
  } else {               // FLOAT: do NOT apply multiplier (by design)
    union { float f; uint8_t b[4]; } conv;
    conv.f = value;
    switch (mbFormat) {
      case FLOAT_ABCD: W1 = (conv.b[3]<<8)|conv.b[2]; W2 = (conv.b[1]<<8)|conv.b[0]; break;
      case FLOAT_CDAB: W1 = (conv.b[1]<<8)|conv.b[0]; W2 = (conv.b[3]<<8)|conv.b[2]; break;
      case FLOAT_BADC: W1 = (conv.b[2]<<8)|conv.b[3]; W2 = (conv.b[0]<<8)|conv.b[1]; break;
      case FLOAT_DCBA: W1 = (conv.b[0]<<8)|conv.b[1]; W2 = (conv.b[2]<<8)|conv.b[3]; break;
    }
  }

  holdingRegs[addr]     = W1;
  holdingRegs[addr + 1] = W2;
}

void writeInputWithScale(uint16_t addr, float value, float multiplier) {
  uint16_t W1 = 0, W2 = 0;

  if (mbDataType == 1) { // INT32
    double scaled = (double)value * (multiplier == 0.0 ? 1.0 : multiplier);
    int32_t iVal = (int32_t) llround(scaled);
    W1 = (uint16_t)(iVal >> 16);
    W2 = (uint16_t)(iVal & 0xFFFF);
  } else {               // FLOAT
    union { float f; uint8_t b[4]; } conv;
    conv.f = value;
    switch (mbFormat) {
      case FLOAT_ABCD: W1 = (conv.b[3]<<8)|conv.b[2]; W2 = (conv.b[1]<<8)|conv.b[0]; break;
      case FLOAT_CDAB: W1 = (conv.b[1]<<8)|conv.b[0]; W2 = (conv.b[3]<<8)|conv.b[2]; break;
      case FLOAT_BADC: W1 = (conv.b[2]<<8)|conv.b[3]; W2 = (conv.b[0]<<8)|conv.b[1]; break;
      case FLOAT_DCBA: W1 = (conv.b[0]<<8)|conv.b[1]; W2 = (conv.b[2]<<8)|conv.b[3]; break;
    }
  }

  inputRegs[addr]     = W1;
  inputRegs[addr + 1] = W2;
}


ModbusMessage readModbusRegisters(ModbusMessage request) {
  uint16_t addr = 0, words = 0;
  request.get(2, addr);
  request.get(4, words);

  // Flash LED (existing behavior)
  #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    flashLEDColor(COLOR_ORANGE, ledModbusFlashStartTime);
  #else
    flashLEDState(ledModbusFlashStartTime);
  #endif

  // Decide the source map based on function code
  const uint8_t fc = request.getFunctionCode();     // e.g., READ_HOLD_REGISTER (=3) or READ_INPUT_REGISTER (=4)
  const std::map<uint16_t, uint16_t>& src =
      (fc == READ_HOLD_REGISTER) ? holdingRegs : inputRegs;

  // Build response
  ModbusMessage response;
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  // Collect missing register addresses for logging
  uint16_t missingCount = 0;
  uint16_t missingSample[5];
  uint8_t  sampleIdx = 0;

  for (uint16_t i = 0; i < words; ++i) {
    const uint16_t regAddr = addr + i;
    auto it = src.find(regAddr);
    if (it != src.end()) {
      response.add(it->second);
    } else {
      // Not populated → add 0 to response, remember for logging
      response.add((uint16_t)0);
      ++missingCount;
      if (sampleIdx < 5) missingSample[sampleIdx++] = regAddr;
    }
  }

  // Emit a single log line per request if something was missing
  if (missingCount > 0) {
    // Build a compact sample list: e.g., "41002,41003,41007..."
    String sampleList;
    for (uint8_t j = 0; j < sampleIdx; ++j) {
      if (j) sampleList += ",";
      sampleList += String(missingSample[j]);
    }
    logMessage(
      "Modbus read: FC=%u SID=%u start=%u words=%u -> missing=%u (sample: '%s')",
      (unsigned)fc,
      (unsigned)request.getServerID(),
      (unsigned)addr,
      (unsigned)words,
      (unsigned)missingCount,
      sampleList.c_str()
    );
  }

  return response;
}

// Build a single Energy-Meter payload (Protocol 0x6069) using current P1 values  [1](https://cdn.sma.de/fileadmin/content/www.developer.sma.de/docs/EMETER-Protokoll-TI-en-10.pdf?v=1699276024)
void buildEmeterPayload(std::vector<uint8_t>& pld) {
  // Device address (6 bytes): SUSyID(2) + Serial(4)
  be16(pld, swSusyId);
  be32(pld, swSerial);

  // Ticker: measuring time in ms (32-bit, overflow ok)
  be32(pld, millis());

  // ===== OBIS PAIRS =====
  // Resolution per spec: power in W (int32), energy in Ws (uint64), current in mA, voltage in mV  [1](https://cdn.sma.de/fileadmin/content/www.developer.sma.de/docs/EMETER-Protokoll-TI-en-10.pdf?v=1699276024)

  // Sum active energy import (1:1.8.0) - in Ws
  obis(pld, 1, 1, 8, 0);
  uint64_t impWs = (uint64_t) llround( (double)energyImport * 3600000.0 ); // kWh -> Ws
  be64(pld, impWs);

  // Sum active energy export (1:2.8.0) - in Ws
  obis(pld, 1, 2, 8, 0);
  uint64_t expWs = (uint64_t) llround( (double)energyExport * 3600000.0 );
  be64(pld, expWs);

  // Sum active power import (1:1.4.0) - in W (positive)
  obis(pld, 1, 1, 4, 0);
  int32_t pImpW = (int32_t) llround( totalDeliveredW );   // already W
  be32(pld, (uint32_t)pImpW);

  // Sum active power export (1:2.4.0) - in W (positive)
  obis(pld, 1, 2, 4, 0);
  int32_t pExpW = (int32_t) llround( totalReceivedW );    // already W
  be32(pld, (uint32_t)pExpW);

  // Phase 1 current (1:31.4.0) mA
  obis(pld, 1, 31, 4, 0); be32(pld, (uint32_t) llround(phases[0].current * 1000.0));
  // Phase 1 voltage (1:32.4.0) mV
  obis(pld, 1, 32, 4, 0); be32(pld, (uint32_t) llround(phases[0].voltage * 1000.0));

  // Phase 2
  obis(pld, 1, 51, 4, 0); be32(pld, (uint32_t) llround(phases[1].current * 1000.0));
  obis(pld, 1, 52, 4, 0); be32(pld, (uint32_t) llround(phases[1].voltage * 1000.0));

  // Phase 3
  obis(pld, 1, 71, 4, 0); be32(pld, (uint32_t) llround(phases[2].current * 1000.0));
  obis(pld, 1, 72, 4, 0); be32(pld, (uint32_t) llround(phases[2].voltage * 1000.0));

  // (Optional: frequency as manufacturer specific or SHM2.0 extended; skipped for strict EMETER-20)
}

// Wrap payload into Speedwire/SMA Data2+ envelope: "SMA\0" + Tag0 + Data2(0x0010, proto 0x6069) + End  [3](https://github.com/RalfOGit/libspeedwire)
void buildSpeedwireDatagram(std::vector<uint8_t>& pkt) {
  std::vector<uint8_t> emeter;
  buildEmeterPayload(emeter);

  // "SMA\0"
  pkt.push_back('S'); pkt.push_back('M'); pkt.push_back('A'); pkt.push_back(0x00);

  // Tag0 (0x02A0), length=4, GroupID=1
  be16(pkt, 4);     // tag length
  be16(pkt, 0x02A0);// tag id
  be32(pkt, 0x00000001); // group 1

  // Data2 tag (0x0010): length = 2 (protocol id) + emeter bytes
  uint16_t data2Len = (uint16_t)(2 + emeter.size());
  be16(pkt, data2Len);
  be16(pkt, 0x0010);     // Data2 tag id
  be16(pkt, 0x6069);     // Protocol: Energy Meter  [1](https://cdn.sma.de/fileadmin/content/www.developer.sma.de/docs/EMETER-Protokoll-TI-en-10.pdf?v=1699276024)
  pkt.insert(pkt.end(), emeter.begin(), emeter.end());

  // End-of-data tag
  be16(pkt, 0x0000);
  be16(pkt, 0x0000);
}

void sendSpeedwireOnce() {
  if (!swEnabled) return;
  IPAddress maddr; maddr.fromString(swAddr);
  std::vector<uint8_t> pkt; pkt.reserve(640);
  buildSpeedwireDatagram(pkt);
  speedwireUDP.beginPacket(maddr, swPort);
  speedwireUDP.write(pkt.data(), pkt.size());
  speedwireUDP.endPacket();
}

// Unified "saved/OTA -> restarting" response with auto-redirect back to "/"
void sendRestartAndAutoRedirect(AsyncWebServerRequest* request, const char* msg = "Restarting...") {
  String html;
  html += "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Restarting...</title>";
  html += "<style>body{font-family:sans-serif;margin:2rem;}button{padding:.5rem 1rem;}small{color:#666}</style>";
  html += "<script>";
  html += R"JS(
    (function(){
      const target = "/";
      let delay = 500;

      async function tryRefresh() {
        try {
          // Opaque fetch (no-cors) + image ping to detect availability
          await fetch(target, { cache: "no-store", mode: "no-cors" });
          const img = new Image();
          img.onload = () => { window.location.replace(target); };
          img.onerror = () => { scheduleNext(); };
          img.src = target + "?ping=" + Date.now();
        } catch (e) {
          scheduleNext();
        }
      }
      function scheduleNext(){ delay = Math.min(5000, Math.floor(delay * 1.5)); setTimeout(tryRefresh, delay); }
      window.manualGoHome = () => { window.location.replace(target); };
      setTimeout(tryRefresh, delay);
    })();
  )JS";
  html += "</script></head><body>";
  html += "<h2>" + String(msg) + "</h2>";
  html += "<p>The device is rebooting. You will be returned to the homepage automatically.</p>";
  html += "<button onclick='manualGoHome()'>Go to Home Now</button><br><br>";
  html += "<small>If not redirected within ~30 seconds, click the button above.</small>";
  html += "</body></html>";

  AsyncWebServerResponse* resp = request->beginResponse(200, "text/html; charset=utf-8", html);
  resp->addHeader("Connection", "close");
  request->send(resp);

  // Allow response to flush cleanly before reboot
  delay(1000);
  ESP.restart();
}


// ======================================================
// 4. WEB UI & CONFIGURATION
// ======================================================

void sendLiveData() {
    String html = "<table><tr><th>Metric</th><th>Value</th><th>Unit</th></tr>";
    for (size_t i = 0; i < P1_VALUE_COUNT; i++) {
        float val = *allP1Values[i].ptr;
        String unit = "";
        const char* n = allP1Values[i].name;
        if (strstr(n, "Voltage")) unit = "V"; else if (strstr(n, "Current")) unit = "A"; else if (strstr(n, "Power")) unit = "W"; else if (strstr(n, "kWh")) unit = "kWh"; else if (strstr(n, "Hz")) unit = "Hz";
        int prec = (unit == "kWh") ? 3 : ((unit == "A") ? 2 : 1);
        html += "<tr><td>" + String(n) + "</td><td>" + String(val, prec) + "</td><td>" + unit + "</td></tr>";
    }
    html += "</table>";
    ws.textAll("DATA:" + html); 
    ws.textAll("UPTIME:" + getUptime()); 
}

String generateStatusTabContent() {
  String html = "<h3>System Status</h3><table>";
  html += "<tr><td>Modbus TCP IP</td><td><strong>" + WiFi.localIP().toString() + ":502</strong></td></tr>";
  html += "<tr><td>P1 Passthrough</td><td><strong>" + WiFi.localIP().toString() + ":" + String(p1PassthroughPort) + "</strong></td></tr>";
  html += "<tr><td>System Uptime</td><td><strong id='uptime_val'>" + getUptime() + "</strong></td></tr>";
  String ingestionMode = p1UseSerial ? "Hardware Serial (" + String(P1_PIN_LABEL) + ")" : "TCP Client (" + p1Host + ")";
  html += "<tr><td>P1 Ingestion</td><td><strong>" + ingestionMode + "</strong></td></tr>";
  String mqttStatus = mqttEnabled ? (mqtt.connected() ? "CONNECTED" : "DISCONNECTED") : "DISABLED";
  html += "<tr><td>MQTT Status</td><td><strong>" + mqttStatus + "</strong></td></tr>";
  String devLabel = (swDeviceType==SW_DEV_EMETER10) ? "EMeter-10" :
                    (swDeviceType==SW_DEV_EMETER20) ? "EMeter-20" :
                    (swDeviceType==SW_DEV_SHM20)    ? "Sunny Home Manager 2.0" : "Custom";
  html += "<tr><td>Speedwire</td><td><strong>" + String(swEnabled ? (swAddr + ":" + String(swPort) + " " + devLabel) : "DISABLED") + "</strong></td></tr>";
  html += "<tr><td>NTP Time</td><td><strong>" + getTimestamp() + "</strong></td></tr></table>";
  html += "<hr><h3>P1 Live Data</h3><div id='data'>Loading...</div>";
return html;
}

String generateLogTabContent() {
    String html = "<h3>System Logs</h3><button onclick='clearLogs()'>Clear Display</button><br><br>";
    html += "<pre id='logs' style='white-space: pre-wrap; word-wrap: break-word; height: 400px; overflow-y: scroll; background: #eee; padding: 10px;'>";
    for (const String& log : logBuffer) html += log + "\n";
    html += "</pre>";
    return html;
}

String generateConfigTabContent() {
    String html = "<form action='/config' method='POST' id='configForm'><h2>Settings</h2><p style='color:red;'>Enter Current Pincode to authorize changes:</p>Pincode: <input type='password' id='pincode' value='' style='width: 100px;'/>";
    html += "<input type='hidden' name='pincode' id='pincodeHidden' value=''><br><br><h3>Security</h3><table><tr><td>New Pincode:</td><td><input type='text' name='newPin' placeholder='Leave empty to keep current' style='width: 200px;'></td></tr></table>";
    html += "<h3>Ingestion & Passthrough</h3><table><tr><td>Mode:</td><td><select name='useSerial'>";
    html += "<option value='0'" + String(!p1UseSerial ? " selected" : "") + ">TCP Client</option><option value='1'" + String(p1UseSerial ? " selected" : "") + ">Hardware Serial (" + String(P1_PIN_LABEL) + ")</option></select></td></tr>";
    html += "<tr><td>Invert Serial:</td><td><input type='checkbox' name='invert' " + String(p1InvertSignal ? "checked" : "") + "></td></tr>";
    html += "<tr><td>Ingestion Host:</td><td><input type='text' name='host' value='" + p1Host + "'></td></tr><tr><td>Ingestion Port:</td><td><input type='number' name='port' value='" + String(p1Port) + "'></td></tr>";
    html += "<tr><td>Passthrough Port:</td><td><input type='number' name='ptPort' value='" + String(p1PassthroughPort) + "'></td></tr>";
    html += "</table>";
    html += "<h3>MQTT</h3><table><tr><td>Enable:</td><td><input type='checkbox' name='mqttEnabled' " + String(mqttEnabled ? "checked" : "") + "></td></tr><tr><td>Host:</td><td><input type='text' name='mqttHost' value='" + mqttHost + "'></td></tr><tr><td>User:</td><td><input type='text' name='mqttUser' value='" + mqttUser + "'></td></tr><tr><td>Pass:</td><td><input type='password' name='mqttPass' value='" + mqttPass + "'></td></tr></table><br><input type='button' value='Save & Restart' onclick='submitConfig()'></form>";
    html += "<br><hr><a href='/update'>Go to Firmware Update (OTA)</a>";
    return html;
}


String generateModbusTabContent() {
  String html = "<h3>Modbus Configuration</h3><form action='/config_mb' method='POST'>";

  // Meter-level settings
  html += "<table>";
  html += "<tr><td>Slave ID:</td><td><input type='number' name='modbusId' value='" + String(modbusAddress) + "'></td></tr>";
  html += "<tr><td>Byte Order:</td><td><select name='mbFormat' id='mbFormat'>";
  const char* formats[] = {"ABCD (Big Endian)", "CDAB (Swapped Word)", "BADC (Swapped Byte)", "DCBA (Little Endian)"};
  for (int i=0; i<4; ++i) {
    html += "<option value='" + String(i) + "' " + (mbFormat==i ? "selected" : "") + ">" + String(formats[i]) + "</option>";
  }
  html += "</select></td></tr>";
  html += "<tr><td>Data Type:</td><td><select name='mbDataType' id='mbDataType'>";
  html += "<option value='0' " + String(mbDataType==0 ? "selected" : "") + ">Float (4-byte)</option>";
  html += "<option value='1' " + String(mbDataType==1 ? "selected" : "") + ">INT32 (4-byte)</option>";
  html += "</select></td></tr>";
  html += "</table>";

  // Preset selector (JS-driven; optional)
  html += "<h4>Apply Preset</h4>";
  html += "<select id='meterPreset' onchange='applyPreset()'></select> ";
  html += "<small>(Selecting a preset pre-fills the form; you can still edit.)</small>";

  // Register grid

  html += "<h4>Register Mapping</h4><table><tr><th>P1 Metric</th><th>Type</th><th>Address</th><th>Multiplier</th></tr>";
  for (int i=0; i<P1_VALUE_COUNT; i++) {
    html += "<tr>";
    html += "<td>" + String(allP1Values[i].name) + "</td>";

    html += "<td><select name='rtype_" + String(i) + "' id='rtype_" + String(i) + "'>";
    html += "<option value='holding' " + String(mbRegType[i]==REG_HOLDING ? "selected":"") + ">Holding (4xxxx)</option>";
    html += "<option value='input' "   + String(mbRegType[i]==REG_INPUT   ? "selected":"") + ">Input (3xxxx)</option>";
    html += "</select></td>";

    html += "<td><input type='number' name='reg_" + String(i) + "' id='reg_" + String(i) +
            "' value='" + String(mbRegAddr[i]) + "'></td>";

    // NEW: Multiplier field; default 1.0; integer steps common, but allow decimals too
    html += "<td><input type='number' step='0.001' name='mul_" + String(i) + "' id='mul_" + String(i) +
            "' value='" + String(mbRegMul[i], 3) + "'></td>";

    html += "</tr>";
  }
  html += "</table><br>";
  html += "<input type='submit' value='Save Modbus Settings'>";
  html += "</form>";

  // --- JS presets: all variants defined client-side ---
  html += "<script>";

  // Define metrics count for JS
  html += "const P1_COUNT=" + String(P1_VALUE_COUNT) + ";";

  // Example presets (you can expand/tweak freely)
  html += R"JS(
    const METERS = [
      {
        name: 'Eastron SDM72D-M',
        byteOrder: 0,         // ABCD
        dataType: 0,          // Float
        registers: [
          // {type:'holding'|'input', address:<num>, mulitplier:<num>}
          // Order must match the rows in the table
        ]
      },
      {
        name: 'Fronius Smart Energy Meter',
        byteOrder: 0,
        dataType: 0,
        registers: []
      }
    ];

    // Fill each preset's registers programmatically: contiguous example blocks
    (function initPresets(){
      // Build Eastron example
      const eastron = METERS[0];
      eastron.registers = [
        {type:'input', address:52, multiplier:1},  // Net Total Power (W)
        {type:'input', address:280, multiplier:1},  // Total Import Power (W)
        {type:'input', address:282, multiplier:1},  // Total Export Power (W)
        {type:'input', address:0, multiplier:1},  // L1 Voltage
        {type:'input', address:2, multiplier:1},  // L2 Voltage
        {type:'input', address:4, multiplier:1},  // L3 Voltage
        {type:'input', address:6, multiplier:1},  // L1 Current
        {type:'input', address:8, multiplier:1},  // L2 Current
        {type:'input', address:10, multiplier:1},  // L3 Current
        {type:'input', address:12, multiplier:1},  // L1 Power
        {type:'input', address:14, multiplier:1},  // L2 Power
        {type:'input', address:16, multiplier:1},  // L3 Power
        {type:'input', address:200, multiplier:1},  // VoltageAB
        {type:'input', address:202, multiplier:1},  // VoltageBC
        {type:'input', address:204, multiplier:1},  // VoltageCA
        {type:'input', address:72, multiplier:1},  // Total Import kWh
        {type:'input', address:74, multiplier:1},  // Total Export kWh
        {type:'input', address:70, multiplier:1}   // Frequency (Hz)
      ];

      // Build Fronius example (different block)
      const fronius = METERS[1];
      fronius.registers = [
        {type:'holding', address:96, multiplier:1},  // Net Total Power (W)
        {type:'holding', address:9280, multiplier:1},  // Total Import Power (W)
        {type:'holding', address:9282, multiplier:1},  // Total Export Power (W)
        {type:'holding', address:80, multiplier:1},  // L1 Voltage
        {type:'holding', address:82, multiplier:1},  // L2 Voltage
        {type:'holding', address:84, multiplier:1},  // L3 Voltage
        {type:'holding', address:72, multiplier:1},  // L1 Current
        {type:'holding', address:74, multiplier:1},  // L2 Current
        {type:'holding', address:76, multiplier:1},  // L3 Current
        {type:'holding', address:98, multiplier:1},  // L1 Power
        {type:'holding', address:100, multiplier:1},  // L2 Power
        {type:'holding', address:102, multiplier:1},  // L3 Power
        {type:'holding', address:88, multiplier:1},  // VoltageAB
        {type:'holding', address:90, multiplier:1},  // VoltageBC
        {type:'holding', address:92, multiplier:1},  // VoltageCA
        {type:'holding', address:9972, multiplier:1},  // Total Import kWh
        {type:'holding', address:9974, multiplier:1},  // Total Export kWh
        {type:'holding', address:94, multiplier:1}   // Frequency (Hz)
      ];
    })();

    // Fill preset dropdown
    (function fillPresetDropdown(){
      const sel = document.getElementById('meterPreset');
      sel.innerHTML = "<option value=''>-- choose preset --</option>";
      METERS.forEach((m,i)=> {
        const opt = document.createElement('option');
        opt.value = i;
        opt.textContent = m.name;
        sel.appendChild(opt);
      });
    })();

    // Apply preset -> sets meter-level and per-register fields
    function applyPreset(){
      const sel = document.getElementById('meterPreset');
      const idx = parseInt(sel.value);
      if (isNaN(idx)) return;
      const m = METERS[idx];

      // Meter-level
      document.getElementById('mbFormat').value   = String(m.byteOrder);
      document.getElementById('mbDataType').value = String(m.dataType);

      // Per-register fields
      for (let i=0; i<P1_COUNT; i++){
        const r = m.registers[i];
        if (!r) continue;
        document.getElementById('rtype_'+i).value = (r.type === 'input') ? 'input' : 'holding';
        document.getElementById('reg_'+i).value   = String(r.address);
        document.getElementById('mul_'+i).value = String(r.multiplier ?? 1);      }
    }
  )JS";

  html += "</script>";

  return html;
}


String generateSpeedwireTabContent() {
  String html;
  html += "<h3>Speedwire (SMA)</h3><form action='/config_sw' method='POST'>";

  // A tiny script to auto-fill SUSyID when the device type changes
  html += "<script>"
          "function swOnTypeChanged(){"
          "  var sel=document.getElementById('swDevType');"
          "  var susy=document.getElementById('swSusyId');"
          "  var map={'0':'010E','1':'015D','2':'0174','3':''};"
          "  if(map[sel.value]!=='' && (susy.dataset.useredit!=='1' || susy.value==='')){"
          "     susy.value=map[sel.value];"
          "     susy.dataset.useredit='0';"
          "  }"
          "}"
          "function swMarkUserEdit(e){ e.dataset.useredit='1'; }"
          "</script>";

  html += "<p><b>Energy Meter multicast</b> frames are sent once per second when enabled.</p>";
  html += "<table>";

  html += "<tr><td>Enable:</td><td><input type='checkbox' name='swEnabled' " + String(swEnabled ? "checked" : "") + "></td></tr>";
  html += "<tr><td>Device type:</td><td>"
          "<select name='swDevType' id='swDevType' onchange='swOnTypeChanged()'>"
          "<option value='0' " + String(swDeviceType==0 ? "selected":"") + ">EMeter-10 (0x010E)</option>"
          "<option value='1' " + String(swDeviceType==1 ? "selected":"") + ">EMeter-20 (0x015D)</option>"
          "<option value='2' " + String(swDeviceType==2 ? "selected":"") + ">Sunny Home Manager 2.0 (0x0174)</option>"
          "<option value='3' " + String(swDeviceType==3 ? "selected":"") + ">Custom</option>"
          "</select></td></tr>";

  html += "<tr><td>Multicast address:</td><td><input type='text' name='swAddr' value='" + swAddr + "'></td></tr>";
  html += "<tr><td>Port:</td><td><input type='number' name='swPort' value='" + String(swPort) + "'></td></tr>";

  // SUSyID as hex; tag the field so we can detect user edits
  html += "<tr><td>SUSyID (hex):</td><td><input type='text' name='swSusyId' id='swSusyId' "
          " value='" + String(swSusyId, HEX) + "' oninput='swMarkUserEdit(this)' data-useredit='0'></td></tr>";

  html += "<tr><td>Serial (uint32):</td><td><input type='text' name='swSerial' value='" + String(swSerial) + "'>"
          "<div style='font-size:12px;color:#555'>Tip: set to 0 to derive from device chip ID</div></td></tr>";

  html += "</table><br>";
  html += "<input type='submit' value='Save & Restart'>";
  html += "</form>";

  // Initialize the SUSyID field if needed on first load
  html += "<script>swOnTypeChanged();</script>";

  return html;
}

void setupWebServer() {
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = "<html><head><style>body{font-family:sans-serif;} table{border:1px solid #000; border-collapse:collapse; width:100%;} td,th{padding:8px; border:1px solid #ccc; text-align:left;} .tab{overflow:hidden; border:1px solid #ccc; background:#f1f1f1;} .tab button{float:left; border:none; padding:14px 16px; cursor:pointer;} .tabcontent{display:none; padding:12px; border:1px solid #ccc; border-top:none;}</style>";
        html += "<script>";
        html += "function updateMbPresetFields() {";
        html += "  var p = document.getElementById('presetSelect').value;";
        html += "  var f = document.getElementById('mbFormat'); var t = document.getElementById('mbDataType'); var d = document.getElementById('mbDiv'); var regs = document.getElementsByClassName('reg-field');";
        html += "  if(p !== 'Custom') { f.value = '0'; t.value = '0'; d.value = '1'; f.disabled=true; t.disabled=true; d.disabled=true; for(var i=0;i<regs.length;i++) regs[i].readOnly=true; }";
        html += "  else { f.disabled=false; t.disabled=false; d.disabled=false; for(var i=0;i<regs.length;i++) regs[i].readOnly=false; }";
        html += "}";
        html += "function openTab(evt, tn){ var i, tc, tl; tc=document.getElementsByClassName('tabcontent'); for(i=0;i<tc.length;i++)tc[i].style.display='none'; tl=document.getElementsByClassName('tablinks'); for(i=0;i<tl.length;i++)tl[i].className=tl[i].className.replace(' active',''); document.getElementById(tn).style.display='block'; evt.currentTarget.className+=' active'; }";
        html += "function submitConfig(){ var p=document.getElementById('pincode').value; document.getElementById('pincodeHidden').value=p; document.getElementById('configForm').submit(); }";
        html += "function clearLogs() { document.getElementById('logs').innerHTML = ''; }";
        //html += "var ws=new WebSocket('ws://'+location.host+'/ws'); ws.onmessage=function(e){ if (e.data.startsWith('LOG:')) { var l=document.getElementById('logs'); if(l) { l.innerHTML += e.data.substring(4) + '\\n'; l.scrollTop = l.scrollHeight; } } else if (e.data.startsWith('DATA:')) { var d=document.getElementById('data'); if(d) d.innerHTML=e.data.substring(5); } else if (e.data.startsWith('UPTIME:')) { var u=document.getElementById('uptime_val'); if(u) u.innerText=e.data.substring(7); } };";

        html += R"JS(
        // Resilient WebSocket with exponential backoff and auto-reconnect
        (function(){
            let ws;
            let retryDelay = 800;               // start at 0.8s
            const maxDelay = 5000;              // cap at 5s
            let heartbeatTimer = null;

            function startHeartbeat(){
            stopHeartbeat();
            heartbeatTimer = setInterval(function(){
                if (ws && ws.readyState === WebSocket.OPEN) {
                try { ws.send("PING"); } catch(e) {}
                }
            }, 30000);                        // keep-alive every 30s
            }
            function stopHeartbeat(){
            if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
            }

            function scheduleReconnect(){
            retryDelay = Math.min(maxDelay, Math.floor(retryDelay * 1.5));
            setTimeout(connectWs, retryDelay);
            }

            function connectWs(){
            try {
                ws = new WebSocket('ws://' + location.host + '/ws');

                ws.onopen = function(){
                // Connected -> reset backoff and start heartbeat
                retryDelay = 800;
                startHeartbeat();
                // You could request initial data here if you add a server handler:
                // ws.send("HELLO");
                };

                ws.onmessage = function(e){
                // Your existing handlers preserved:
                if (e.data.startsWith('LOG:')) {
                    var l=document.getElementById('logs');
                    if(l){ l.innerHTML += e.data.substring(4) + '\n'; l.scrollTop = l.scrollHeight; }
                } else if (e.data.startsWith('DATA:')) {
                    var d=document.getElementById('data'); if(d) d.innerHTML = e.data.substring(5);
                } else if (e.data.startsWith('UPTIME:')) {
                    var u=document.getElementById('uptime_val'); if(u) u.innerText = e.data.substring(7);
                }
                };

                ws.onclose = function(){
                stopHeartbeat();
                scheduleReconnect();
                };

                ws.onerror = function(){
                try { ws.close(); } catch(e){}
                // onclose will schedule the reconnect
                };
            } catch(e) {
                scheduleReconnect();
            }
            }

            // Kick it off
            connectWs();
        })();
        )JS";

        html += "</script></head><body onload=\"updateMbPresetFields(); document.getElementById('defaultOpen').click();\">";
        html += "<h1>ESP32 P1 Bridge</h1><div class='tab'><button class='tablinks' onclick=\"openTab(event, 'Status')\" id='defaultOpen'>Status</button><button class='tablinks' onclick=\"openTab(event, 'Modbus')\">Modbus Config</button><button class='tablinks' onclick=\"openTab(event, 'Speedwire')\">Speedwire</button><button class='tablinks' onclick=\"openTab(event, 'Config')\">General Config</button><button class='tablinks' onclick=\"openTab(event, 'Logs')\">Logs</button></div>";
        html += "<div id='Status' class='tabcontent'>" + generateStatusTabContent() + "</div>";
        html += "<div id='Modbus' class='tabcontent'>" + generateModbusTabContent() + "</div>";
        html += "<div id='Speedwire' class='tabcontent'>" + generateSpeedwireTabContent() + "</div>";
        html += "<div id='Config' class='tabcontent'>" + generateConfigTabContent() + "</div>";
        html += "<div id='Logs' class='tabcontent'>" + generateLogTabContent() + "</div></body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/config_mb", HTTP_POST, [](AsyncWebServerRequest *request) {
        prefs.begin("config", false);

        // Meter-level settings set by UI/preset JS
        if (request->hasParam("mbFormat", true))
            prefs.putUChar("mbFormat", request->getParam("mbFormat", true)->value().toInt());
        if (request->hasParam("mbDataType", true))
            prefs.putUChar("mbDataType", request->getParam("mbDataType", true)->value().toInt());

        // Slave ID
        prefs.putUChar("modbusId", request->getParam("modbusId", true)->value().toInt());

        // Per-register fields
        for (int i = 0; i < P1_VALUE_COUNT; i++) {
          // type
          String rtypeName = "rtype_" + String(i);
          if (request->hasParam(rtypeName, true)) {
            uint8_t tval = (request->getParam(rtypeName, true)->value() == "input") ? REG_INPUT : REG_HOLDING;
            prefs.putUChar(("t_" + String(i)).c_str(), tval);
          }

          // address
          String regName = "reg_" + String(i);
          if (request->hasParam(regName, true)) {
            prefs.putUInt(("r_" + String(i)).c_str(), request->getParam(regName, true)->value().toInt());
          }

          // multiplier (NEW)
          String mulName = "mul_" + String(i);   // change the field name in UI (see below)
          if (request->hasParam(mulName, true)) {
            prefs.putFloat(("m_" + String(i)).c_str(), request->getParam(mulName, true)->value().toFloat());
          }
        }

        prefs.end();
        sendRestartAndAutoRedirect(request, "Modbus settings saved. Restarting...");
    });
    
    server.on("/config_sw", HTTP_POST, [] (AsyncWebServerRequest *request) {
        prefs.begin("config", false);

        // Enable/address/port (already present)
        bool en = request->hasParam("swEnabled", true);
        prefs.putBool("swEnabled", en);
        if (request->hasParam("swAddr", true)) prefs.putString("swAddr", request->getParam("swAddr", true)->value());
        if (request->hasParam("swPort", true)) prefs.putUInt("swPort", request->getParam("swPort", true)->value().toInt());

        // Device type
        if (request->hasParam("swDevType", true)) {
            uint8_t dt = (uint8_t) request->getParam("swDevType", true)->value().toInt();
            prefs.putUChar("swDevType", dt);

            // If SUSyID field is empty, preset to default for selected device
            // We check presence/length to decide if user wants a custom override.
            bool userProvidedSusy = request->hasParam("swSusyId", true) && request->getParam("swSusyId", true)->value().length() > 0;
            if (!userProvidedSusy) {
            prefs.putUShort("swSusyId", defaultSusyFor(dt));
            }
        }

        // SUSyID (hex) — user override (optional)
        if (request->hasParam("swSusyId", true) && request->getParam("swSusyId", true)->value().length() > 0) {
            String s = request->getParam("swSusyId", true)->value();
            uint16_t v = (uint16_t) strtol(s.c_str(), nullptr, 16);
            prefs.putUShort("swSusyId", v);
        }

        // Serial (uint32) — 0 keeps "derive from chip"
        if (request->hasParam("swSerial", true)) {
            String s = request->getParam("swSerial", true)->value();
            uint32_t v = (uint32_t) strtoul(s.c_str(), nullptr, 10);
            prefs.putUInt("swSerial", v);
        }

        prefs.end();
        sendRestartAndAutoRedirect(request, "Speedwire settings saved. Restarting...");
    });

    server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("pincode", true) || request->getParam("pincode", true)->value() != configPincode) { 
            request->send(403, "text/plain", "Forbidden: Invalid Pincode"); return; 
        }
        prefs.begin("config", false);
        if (request->hasParam("newPin", true) && request->getParam("newPin", true)->value().length() > 0) prefs.putString("pincode", request->getParam("newPin", true)->value());
        prefs.putBool("p1UseSerial", request->getParam("useSerial", true)->value() == "1");
        prefs.putBool("p1Invert", request->hasParam("invert", true));
        prefs.putString("p1Host", request->getParam("host", true)->value());
        prefs.putUInt("p1Port", request->getParam("port", true)->value().toInt());
        prefs.putUInt("ptPort", request->getParam("ptPort", true)->value().toInt());
        prefs.putBool("mqttEnabled", request->hasParam("mqttEnabled", true));
        prefs.putString("mqttHost", request->getParam("mqttHost", true)->value());
        prefs.putString("mqttUser", request->getParam("mqttUser", true)->value());
        prefs.putString("mqttPass", request->getParam("mqttPass", true)->value());
        prefs.end();

        sendRestartAndAutoRedirect(request, "Config Settings saved. Restarting...");
    });

    ElegantOTA.setAutoReboot(true);
    ElegantOTA.begin(&server); 
    server.begin();
}

// ======================================================
// 5. SETUP & LOOP 
// ======================================================

void setup() {
  Serial.begin(115200);
  #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(25); setLEDColor(CRGB::Red); 
  #else
    pinMode(LED_BUILTIN_PIN, OUTPUT); digitalWrite(LED_BUILTIN_PIN, LOW); 
  #endif

  prefs.begin("config", true);
  configPincode = prefs.getString("pincode", "1234");
  p1UseSerial = prefs.getBool("p1UseSerial", false);
  p1InvertSignal = prefs.getBool("p1Invert", true);
  p1Host = prefs.getString("p1Host", "192.168.11.223");
  p1Port = prefs.getUInt("p1Port", 8088);
  p1PassthroughPort = prefs.getUInt("ptPort", 8088);
  modbusAddress = prefs.getUChar("modbusId", 1); 
  preset = prefs.getString("preset", "Eastron");
  // mbDivision = prefs.getFloat("mbDiv", 1.0);
  mqttEnabled = prefs.getBool("mqttEnabled", false);
  mqttHost = prefs.getString("mqttHost", "");
  mqttUser = prefs.getString("mqttUser", "");
  mqttPass = prefs.getString("mqttPass", "");

  // Load per-register mapping (default: Holding, current addr/default, multiplier=1.0)
  for (int i = 0; i < P1_VALUE_COUNT; i++) {
    // type (t_i)
    String kt = "t_" + String(i);
    mbRegType[i] = prefs.getUChar(kt.c_str(), REG_HOLDING);

    // address (r_i) fallback to legacy r{i}
    String kr_new = "r_" + String(i);
    String kr_old = "r" + String(i);
    mbRegAddr[i] = prefs.getUInt(kr_new.c_str(),
                      prefs.getUInt(kr_old.c_str(), allP1Values[i].regAddr));

    // multiplier (m_i); migrate from old division (d_i) if present
    String km = "m_" + String(i);
    String kd = "d_" + String(i);
    if (prefs.isKey(km.c_str())) {
      mbRegMul[i] = prefs.getFloat(km.c_str(), 1.0f);
    } else if (prefs.isKey(kd.c_str())) {
      mbRegMul[i] = prefs.getFloat(kd.c_str(), 1.0f); // migrate division → multiplier
    } else {
      mbRegMul[i] = 1.0f;
    }
  }


  // Meter-level defaults remain as you have them:
  mbFormat   = prefs.getUChar("mbFormat", 0);  // ABCD by default
  mbDataType = prefs.getUChar("mbDataType", 0); // 0=Float, 1=INT32
  // mbDivision can still exist but is no longer used for per-register publishing

  swEnabled    = prefs.getBool("swEnabled", false);
  swAddr       = prefs.getString("swAddr", "239.12.255.254");
  swPort       = prefs.getUInt("swPort", 9522);
  swDeviceType = prefs.getUChar("swDevType", SW_DEV_EMETER20);
  swSusyId     = prefs.getUShort("swSusyId", 0xFFFF); // 0xFFFF means "not set" -> use default for type
  swSerial     = prefs.getUInt("swSerial", 0);        // 0 => derive from chip

  // If SUSyID not explicitly set, use the default for the chosen device type
  if (swSusyId == 0xFFFF) swSusyId = defaultSusyFor(swDeviceType);

  // Derive serial from efuse MAC if not explicitly provided
  if (swSerial == 0) {
    uint64_t chipmac = ESP.getEfuseMac();
    swSerial = (uint32_t)(chipmac & 0xFFFFFFFF);
  }
  prefs.end();

  if (p1UseSerial) Serial1.begin(P1_BAUD, SERIAL_8N1, P1_RX_PIN, -1, p1InvertSignal);
  
    // Start UDP socket for sending (bind to ephemeral local port)
  if (swEnabled) {
    // speedwireUDP.begin();  // no need to join group for sending
    logMessage("Speedwire enabled -> %s:%u", swAddr.c_str(), swPort);
  }

  WiFiManager wm;
  wm.autoConnect("P1-Bridge-Setup");
  logMessage("WiFi Connected. IP: %s", WiFi.localIP().toString().c_str());
  ntpUDP.begin(2390); timeClient.begin();
  setupWebServer(); setupPassthroughServer(); 
  MBserver.registerWorker(modbusAddress, READ_HOLD_REGISTER, &readModbusRegisters); 
  MBserver.registerWorker(modbusAddress, READ_INPUT_REGISTER, &readModbusRegisters); 
  MBserver.start(502, modbusAddress, 20000);
  mqtt.setServer(mqttHost.c_str(), 1883);
  lastP1TelegramTime = millis();
  
}

void loop() {
  #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
    if (isP1Connecting) { if (millis() % 400 < 200) setLEDColor(CRGB::Red); else setLEDColor(CRGB::Black); }
    else if (ledP1FlashStartTime > 0 && millis() - ledP1FlashStartTime >= LED_FLASH_DURATION) { setLEDColor(CRGB::Black); ledP1FlashStartTime = 0; }
    else if (ledModbusFlashStartTime > 0 && millis() - ledModbusFlashStartTime >= LED_FLASH_DURATION) { setLEDColor(CRGB::Black); ledModbusFlashStartTime = 0; }
    else if (!(p1Client.connected() || p1UseSerial)) setLEDColor(CRGB::Red); else setLEDColor(CRGB::Black); 
    #else
    // --- Single onboard LED path ---
    // NOTE: LED_BUILTIN_PIN was set as OUTPUT in setup(), and LOW means OFF on most ESP32 dev boards.
    if (isP1Connecting) {
        // Blink while connecting: 200ms ON, 200ms OFF
        if (millis() % 400 < 200) setLEDState(HIGH);
        else                      setLEDState(LOW);
    }
    else if (ledP1FlashStartTime > 0) {
        // If we recently flashed for a P1 telegram, turn OFF after duration
        if (millis() - ledP1FlashStartTime >= LED_FLASH_DURATION) {
        setLEDState(LOW); ledP1FlashStartTime = 0;
        } else {
        setLEDState(HIGH);
        }
    }
    else if (ledModbusFlashStartTime > 0) {
        // If we recently flashed for a Modbus read, turn OFF after duration
        if (millis() - ledModbusFlashStartTime >= LED_FLASH_DURATION) {
        setLEDState(LOW); ledModbusFlashStartTime = 0;
        } else {
        setLEDState(HIGH);
        }
    }
    else if (!(p1Client.connected() || p1UseSerial)) {
        // No P1 source connected (TCP client disconnected AND not using serial): solid ON to signal issue
        setLEDState(HIGH);
    } else {
        // Normal operation: OFF
        setLEDState(LOW);
    }
    #endif


  timeClient.update();
  static unsigned long lastP1Try = 0, lastUpdate = 0;
  static String currentP1Line;
  Stream* p1Stream = p1UseSerial ? (Stream*)&Serial1 : (p1Client.connected() ? (Stream*)&p1Client : nullptr);

  if (p1Stream) {
      while (p1Stream->available()) {
          char c = p1Stream->read();
          rawTelegramCapture += c; 
          if (c == '\n') { parseP1Line(currentP1Line); currentP1Line = ""; } 
          else if (c != '\r') { currentP1Line += c; }

          if (c == '!') {
              unsigned long waitCrc = millis();
              while (rawTelegramCapture.length() < (rawTelegramCapture.indexOf('!') + 5) && (millis() - waitCrc < 100)) {
                  if (p1Stream->available()) rawTelegramCapture += (char)p1Stream->read();
              }
              for (AsyncClient* client : passthroughClients) {
                  if (client->connected()) client->write(rawTelegramCapture.c_str(), rawTelegramCapture.length());
              }
              rawTelegramCapture = ""; 
          }
      }
  } else if (!p1UseSerial && millis() - lastP1Try > 5000) {
      lastP1Try = millis(); connectToP1();
  }
  
  if ((p1UseSerial || p1Client.connected()) && (millis() - lastP1TelegramTime > P1_TIMEOUT_MS)) {
      if (p1Client.connected()) p1Client.stop(); 
      lastP1Try = 0; lastP1TelegramTime = millis();
  }

  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();

    // Clear both maps before repopulating
    holdingRegs.clear();
    inputRegs.clear();


  for (int i = 0; i < P1_VALUE_COUNT; i++) {
    float val = *allP1Values[i].ptr;
    if (mbRegType[i] == REG_HOLDING) {
      writeHoldingWithScale(mbRegAddr[i], val, mbRegMul[i]);
    } else {
      writeInputWithScale(mbRegAddr[i], val, mbRegMul[i]);
    }
  }
    sendLiveData(); 
    sendSpeedwireOnce();
    if (mqttEnabled && mqtt.connected()) mqtt.publish((String(MQTT_BASE_TOPIC) + "/Power").c_str(), String(netTotalPowerW).c_str());
  }
  if (mqttEnabled && !mqtt.connected()) mqtt.connect("ESP32-P1-Bridge", mqttUser.c_str(), mqttPass.c_str());
  mqtt.loop();

  // after sendLiveData() etc., still inside the 1-second block:

}