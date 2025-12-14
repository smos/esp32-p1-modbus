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

// ======================================================
// 1. GLOBAL VARIABLE DECLARATIONS
// ======================================================

#define MQTT_BASE_TOPIC "ESP32-P1-Modbus" 

// LED Pins
const int LED_P1_STATUS = 2;       
const int LED_MODBUS_ACTIVITY = 4; 

// P1 Serial Config
const int P1_RX_PIN = 16;        
const int P1_BAUD = 115200;      

// P1 Passthrough Server
#define P1_PASSTHROUGH_PORT 8023
AsyncServer passthroughServer(P1_PASSTHROUGH_PORT);
std::list<AsyncClient*> passthroughClients; // List of connected clients for P1 passthrough

// State Variables
unsigned long ledP1FlashStartTime = 0;
unsigned long ledModbusFlashStartTime = 0;
const int LED_FLASH_DURATION = 50; 
bool isP1Connecting = false; 
const char* CONFIG_PINCODE = "1234"; 

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
uint8_t modbusAddress = 1; // Default Modbus Slave Address

String mqttHost = "192.168.1.238";
uint16_t mqttPort = 1883;
String mqttUser = "mqttuser";
String mqttPass = "mqttpass";
bool mqttEnabled = false; 

String preset = "Eastron";
String telegramBuffer;
std::map<uint16_t, uint16_t> holdingRegs;

// ======================================================
// 2. DATA STRUCTURES & MODBUS MAPS
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

// ======================================================
// 3. LOGIC FUNCTIONS & MACROS 
// ======================================================

// Helper macro to convert 4xxxx Modbus register numbers to 0-based index
static inline uint16_t HR(uint16_t ref4xxxx) { 
    return (ref4xxxx >= 40001) ? (ref4xxxx - 40001) : ref4xxxx; 
}

// --- Modbus Register Maps ---
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
// --- End Modbus Register Maps ---

// --- MODBUS FLOAT ENUMERATION AND HELPER STRUCT ---

// Float Formats (4-byte float: B3 B2 B1 B0, where B3 is MSB, B0 is LSB)
enum FloatFormat {
    FLOAT_ABCD, // Standard Big-Endian: W1(B3 B2), W2(B1 B0). Client: -Bbf
    FLOAT_CDAB, // Swapped Word/Little-Endian: W1(B1 B0), W2(B3 B2). Client: -Ble, -Bmixed (Confirmed working)
    FLOAT_BADC, // Swapped Byte/Middle Endian: W1(B2 B3), W2(B0 B1). Client: N/A
    FLOAT_DCBA  // Little-Endian Byte Swap: W1(B0 B1), W2(B2 B3). Client: -Bbe (often)
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
// --- END MODBUS FLOAT ENUMERATION ---


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
        digitalWrite(LED_P1_STATUS, LOW); 
    }
  }
}

// Function to safely remove and clean up a passthrough client
void cleanupClient(AsyncClient* client) {
    // Find and remove the client from the list
    for (auto it = passthroughClients.begin(); it != passthroughClients.end(); ++it) {
        if (*it == client) {
            delete *it; // Free memory used by the AsyncClient object
            it = passthroughClients.erase(it);
            return;
        }
    }
}


void parseP1Line(const String& line) {
  telegramBuffer += line + "\n";
  if (line.startsWith("!")) {
    
    // --- P1 PASSTHROUGH LOGIC ---
    // 1. Capture the complete telegram 
    String completeTelegram = telegramBuffer;
    
    // 2. Broadcast the telegram to all connected passthrough clients
    if (!passthroughClients.empty()) {
        for (AsyncClient* client : passthroughClients) {
            // Check if client is still connected and has buffer space
            if (client->connected() && client->space() > completeTelegram.length()) {
                client->add((const char*)completeTelegram.c_str(), completeTelegram.length());
                client->send();
            }
        }
    }
    // --- END P1 PASSTHROUGH LOGIC ---

    int pos = 0;
    
    // Local variables for per-phase power parsing (W)
    float L1_Import_W = 0.0f, L1_Export_W = 0.0f;
    float L2_Import_W = 0.0f, L2_Export_W = 0.0f;
    float L3_Import_W = 0.0f, L3_Export_W = 0.0f;

    while (true) {
      int end = telegramBuffer.indexOf('\n', pos);
      if (end < 0) break;
      String obisLine = telegramBuffer.substring(pos, end);
      pos = end + 1;

      // Phase Voltage Parsing (V)
      if      (obisLine.startsWith("1-0:32.7.0")) phases[0].voltage = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:52.7.0")) phases[1].voltage = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:72.7.0")) phases[2].voltage = extractObisValue(obisLine);
      
      // Phase Current Parsing (A) - Now ignored (calculated later)
      
      // Phase Instantaneous Power Parsing (kW) -> Convert to W
      else if (obisLine.startsWith("1-0:21.7.0")) L1_Import_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:22.7.0")) L1_Export_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:41.7.0")) L2_Import_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:42.7.0")) L2_Export_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:61.7.0")) L3_Import_W = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:62.7.0")) L3_Export_W = extractObisValue(obisLine) * 1000.0f; 

      // Total Instantaneous Power (kW) -> Convert to W
      else if (obisLine.startsWith("1-0:1.7.0")) totalDeliveredW = extractObisValue(obisLine) * 1000.0f; 
      else if (obisLine.startsWith("1-0:2.7.0")) totalReceivedW  = extractObisValue(obisLine) * 1000.0f; 
      
      // Energy Totals (kWh)
      else if (obisLine.startsWith("1-0:1.8.1")) energyImportT1 = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:1.8.2")) energyImportT2 = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:2.8.1")) energyExportT1 = extractObisValue(obisLine);
      else if (obisLine.startsWith("1-0:2.8.2")) energyExportT2 = extractObisValue(obisLine);
      
      // Frequency (Hz)
      else if (obisLine.startsWith("1-0:14.7.0")) frequencyHz = extractObisValue(obisLine);
    }
    
    // --- CORRECTED POWER AND CURRENT CALCULATION ---
    // 1. Calculate Net Phase Power from parsed Import/Export values
    phases[0].power = L1_Import_W - L1_Export_W;
    phases[1].power = L2_Import_W - L2_Export_W;
    phases[2].power = L3_Import_W - L3_Export_W;

    // 2. Calculate Phase Current based on Power and Voltage (I = P/V)
    for (int i = 0; i < 3; i++) {
        // Calculate current. Use abs() for current and check for voltage near zero to prevent division by zero.
        if (abs(phases[i].voltage) > 1.0f) { 
            phases[i].current = abs(phases[i].power / phases[i].voltage);
        } else {
            phases[i].current = 0.0f;
        }
    }
    
    // 3. Calculate Energy Totals
    energyImport = energyImportT1 + energyImportT2;
    energyExport = energyExportT1 + energyExportT2;
    netTotalPowerW = totalDeliveredW - totalReceivedW; 
    
    // 4. Modbus-specific Import/Export Power (always positive)
    modbusImportPowerW = (netTotalPowerW > 0) ? netTotalPowerW : 0.0f;
    modbusExportPowerW = (netTotalPowerW < 0) ? abs(netTotalPowerW) : 0.0f;
    // --- END CORRECTED POWER AND CURRENT CALCULATION ---
    
    telegramBuffer = "";
    ledP1FlashStartTime = millis();
    digitalWrite(LED_P1_STATUS, HIGH); 
  }
}

// Unified Modbus worker for FC 03 (Holding) and FC 04 (Input)
ModbusMessage readModbusRegisters(ModbusMessage request) {
    uint16_t addr = 0, words = 0;
    request.get(2, addr); request.get(4, words);
    ledModbusFlashStartTime = millis();
    digitalWrite(LED_MODBUS_ACTIVITY, HIGH); 

    ModbusMessage response;
    // Use the request's function code for the response (FC03 or FC04)
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    for (uint16_t i = 0; i < words; i++) {
        uint16_t regAddr = addr + i;
        response.add(holdingRegs.count(regAddr) ? holdingRegs[regAddr] : (uint16_t)0);
    }
    return response;
}

// Flexible function to write a float value using a specified Modbus format.
// Note: This function only handles 32-bit float values (2 registers)
void writeModbusValue(uint16_t addr, float value, FloatFormat format) {
    // Union to safely access float bytes regardless of platform endianness (ESP32 is Little Endian)
    // F = [B3 B2 B1 B0], where B3 is MSB, B0 is LSB.
    // ESP32: bytes[0]=B0, bytes[1]=B1, bytes[2]=B2, bytes[3]=B3
    union {
        float f;
        uint8_t bytes[4];
    } converter;
    converter.f = value;

    // Renamed variables to avoid collision with Arduino B0, B1, B2, B3 macros
    uint8_t byte0 = converter.bytes[0];
    uint8_t byte1 = converter.bytes[1];
    uint8_t byte2 = converter.bytes[2];
    uint8_t byte3 = converter.bytes[3];

    // Modbus words are always Big Endian (High Byte, Low Byte)
    uint16_t W1 = 0; // Word 1 (address addr)
    uint16_t W2 = 0; // Word 2 (address addr + 1)

    switch (format) {
        case FLOAT_ABCD: // Standard Big-Endian (W1: B3 B2, W2: B1 B0)
            W1 = (byte3 << 8) | byte2;
            W2 = (byte1 << 8) | byte0;
            break;
        case FLOAT_CDAB: // Swapped Word/Little-Endian (W1: B1 B0, W2: B3 B2) <-- CONFIRMED WORKING
            W1 = (byte1 << 8) | byte0;
            W2 = (byte3 << 8) | byte2;
            break;
        case FLOAT_BADC: // Swapped Byte/Middle Endian (W1: B2 B3, W2: B0 B1)
            W1 = (byte2 << 8) | byte3;
            W2 = (byte0 << 8) | byte1;
            break;
        case FLOAT_DCBA: // Little-Endian Byte Swap (W1: B0 B1, W2: B2 B3)
            W1 = (byte0 << 8) | byte1;
            W2 = (byte2 << 8) | byte3;
            break;
        default:
            // Should not happen, but safe fallback to standard Big Endian
            W1 = (byte3 << 8) | byte2;
            W2 = (byte1 << 8) | byte0;
            break;
    }
    
    holdingRegs[addr] = W1;
    holdingRegs[addr + 1] = W2;
}


// ======================================================
// 4. WEB UI & CONFIGURATION
// ======================================================

// FORWARD DECLARATIONS (To resolve compiler scoping issues within setupWebServer lambda)
void sendLiveData(); 
String generateStatusTabContent();
String generateConfigTabContent();

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
  html += "<tr><td>MQTT Status</td><td><strong>" + mqttStatus + "</strong></td></tr></table>";
  html += "<hr><h3>P1 Live Data</h3><div id='data'>Loading...</div>";
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
    html += "<table style='width: auto;'><thead><tr><th>Format Name</th><th>Byte Order (W1 W2)</th><th>Works with Client Flag (Example)</th></tr></thead><tbody>";
    for (size_t i = 0; i < NUM_SUPPORTED_FORMATS; i++) {
        String name = supportedFormats[i].name;
        String byteOrder = "";
        String clientFlag = "";
        
        switch (supportedFormats[i].format) {
            case FLOAT_ABCD: byteOrder = "[B3 B2] [B1 B0]"; clientFlag = "-Bbf (Big Endian)"; break;
            case FLOAT_CDAB: byteOrder = "[B1 B0] [B3 B2]"; clientFlag = "-Ble, -Bmixed"; break; // Updated
            case FLOAT_BADC: byteOrder = "[B2 B3] [B0 B1]"; clientFlag = "N/A (Middle Endian)"; break;
            case FLOAT_DCBA: byteOrder = "[B0 B1] [B2 B3]"; clientFlag = "-Bbe (Big Endian Word Swap)"; break; 
        }
        
        html += "<tr><td>" + name + "</td><td>" + byteOrder + "</td><td>" + clientFlag + "</td></tr>";
    }
    html += "</tbody></table>";

    // --- END MODBUS REGISTER TABLES ---

    return html;
}

// New: Setup P1 Passthrough TCP Server
void setupPassthroughServer() {
    passthroughServer.onClient([](void* arg, AsyncClient* client) {
        Serial.printf("P1 Passthrough: New client connected from %s\n", client->remoteIP().toString().c_str());
        
        // Add the new client to the list
        passthroughClients.push_back(client);
        
        // Set up handlers for the new client
        client->onData([](void* arg, AsyncClient* c, void* data, size_t len) {
            // Ignore incoming data, this is an output-only server
        }, NULL);
        
        client->onDisconnect([](void* arg, AsyncClient* c) {
            Serial.printf("P1 Passthrough: Client disconnected from %s\n", c->remoteIP().toString().c_str());
            // Use cleanup function to safely remove the client
            cleanupClient(c);
        }, NULL);
        
        client->onError([](void* arg, AsyncClient* c, int error) {
            Serial.printf("P1 Passthrough: Client error %d\n", error);
        }, NULL);
    }, NULL);
    
    passthroughServer.begin();
    Serial.printf("P1 Passthrough Server started on port %d\n", P1_PASSTHROUGH_PORT);
}


void setupWebServer() {
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = "<html><head><style>body{font-family:sans-serif;} table{border:1px solid #000; border-collapse:collapse; width:100%;} td,th{padding:8px; border:1px solid #ccc; text-align:left;}";
        html += ".tab{overflow:hidden; border:1px solid #ccc; background:#f1f1f1;} .tab button{float:left; border:none; padding:14px 16px; cursor:pointer;} .tab button:hover{background:#ddd;} .tab button.active{background:#ccc;} .tabcontent{display:none; padding:12px; border:1px solid #ccc; border-top:none;}</style></head><body>";
        html += "<h1>ESP32 P1 Bridge</h1><div class='tab'><button class='tablinks' onclick=\"openTab(event, 'Status')\" id='defaultOpen'>Status</button><button class='tablinks' onclick=\"openTab(event, 'Config')\">Config</button><a href='/update' style='float:right; padding:14px;'>OTA Update</a></div>";
        html += "<div id='Status' class='tabcontent'>" + generateStatusTabContent() + "</div>";
        html += "<div id='Config' class='tabcontent'>" + generateConfigTabContent() + "</div>";
        html += "<script>function openTab(evt, tn){ var i, tc, tl; tc=document.getElementsByClassName('tabcontent'); for(i=0;i<tc.length;i++)tc[i].style.display='none'; tl=document.getElementsByClassName('tablinks'); for(i=0;i<tl.length;i++)tl[i].className=tl[i].className.replace(' active',''); document.getElementById(tn).style.display='block'; evt.currentTarget.className+=' active'; }";
        html += "document.getElementById('defaultOpen').click(); var ws=new WebSocket('ws://'+location.host+'/ws'); ws.onmessage=function(e){document.getElementById('data').innerHTML=e.data;};";
        html += "function submitConfig(){ var p=document.getElementById('pincode').value; if(p==='" + String(CONFIG_PINCODE) + "'){document.getElementById('pincodeHidden').value=p; document.getElementById('configForm').submit();}else{alert('Wrong Pin');} }</script></body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("pincode", true) || request->getParam("pincode", true)->value() != CONFIG_PINCODE) {
            request->send(403, "text/plain", "Forbidden"); return;
        }
        prefs.begin("config", false);
        p1UseSerial = request->getParam("useSerial", true)->value() == "1";
        p1InvertSignal = request->hasParam("invert", true);
        p1Host = request->getParam("host", true)->value();
        p1Port = request->getParam("port", true)->value().toInt();
        modbusAddress = request->getParam("modbusId", true)->value().toInt(); // Save Modbus ID
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
  pinMode(LED_P1_STATUS, OUTPUT);
  pinMode(LED_MODBUS_ACTIVITY, OUTPUT);

  prefs.begin("config", true);
  p1UseSerial = prefs.getBool("p1UseSerial", false);
  p1InvertSignal = prefs.getBool("p1Invert", true);
  p1Host = prefs.getString("p1Host", "192.168.1.100");
  p1Port = prefs.getUInt("p1Port", 8088);
  modbusAddress = prefs.getUChar("modbusId", 1); // Load Modbus ID
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
  wm.autoConnect("P1-Bridge-Setup");
  setupWebServer();
  setupPassthroughServer(); // Initialize the new Passthrough server
  
  // Register the same worker function for BOTH Holding Registers (FC 03) and Input Registers (FC 04)
  MBserver.registerWorker(modbusAddress, READ_HOLD_REGISTER, &readModbusRegisters); // For Fronius/General use
  MBserver.registerWorker(modbusAddress, READ_INPUT_REGISTER, &readModbusRegisters); // For Eastron use
  MBserver.start(502, modbusAddress, 20000);
  mqtt.setServer(mqttHost.c_str(), 1883);
}

void loop() {
  // LED Management
  if (isP1Connecting) {
      digitalWrite(LED_P1_STATUS, (millis() % 400 < 200));
  } else if (ledP1FlashStartTime > 0 && millis() - ledP1FlashStartTime >= LED_FLASH_DURATION) {
      digitalWrite(LED_P1_STATUS, LOW); ledP1FlashStartTime = 0;
  }
  if (ledModbusFlashStartTime > 0 && millis() - ledModbusFlashStartTime >= LED_FLASH_DURATION) {
      digitalWrite(LED_MODBUS_ACTIVITY, LOW); ledModbusFlashStartTime = 0;
  }

  static unsigned long lastP1Try = 0, lastUpdate = 0;

  // Ingestion: Serial or TCP
  if (p1UseSerial) {
      while (Serial1.available()) parseP1Line(Serial1.readStringUntil('\n'));
  } else {
      if (p1Client.connected()) {
          while (p1Client.available()) parseP1Line(p1Client.readStringUntil('\n'));
      } else if (millis() - lastP1Try > 5000) {
          lastP1Try = millis(); connectToP1();
      }
  }

  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    
    // MODBUS REGISTER MAPPING LOGIC
    holdingRegs.clear();
    
    // FLOAT_ABCD -Bbe, FLOAT_CDAB -Bmixed, FLOAT_DCBA -Ble
    const FloatFormat currentFloatFormat = FLOAT_ABCD; 

    for (auto& m : maps) {
        if (preset == m.name && m.registers) {
            for (size_t i=0; i<m.count; i++) {
                for (auto& p : allP1Values) {
                    // Match the OBIS name from the map to the P1 variable name
                    if (strcmp(m.registers[i].obis, p.name) == 0) {
                        // Use the new flexible function
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