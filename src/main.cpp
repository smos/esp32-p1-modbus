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
std::map<uint16_t, uint16_t> holdingRegs;

// ======================================================
// 2. DATA STRUCTURES & P1 VALUES
// ======================================================

struct PhaseData { float voltage = 0.0f; float power = 0.0f; float current = 0.0f; };
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
    {"Total Import kWh", &energyImport, 0x0046},
    {"Total Export kWh", &energyExport, 0x0048},
    {"Frequency (Hz)", &frequencyHz, 0x004C}
};
#define P1_VALUE_COUNT (sizeof(allP1Values) / sizeof(P1Value))

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
    
    energyImport = energyImportT1 + energyImportT2;
    energyExport = energyExportT1 + energyExportT2;
    netTotalPowerW = totalDeliveredW - totalReceivedW; 
    modbusImportPowerW = (netTotalPowerW > 0) ? netTotalPowerW : 0.0f;
    modbusExportPowerW = (netTotalPowerW < 0) ? abs(netTotalPowerW) : 0.0f;
    telegramBuffer = "";

    #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
        flashLEDColor(COLOR_GREEN, ledP1FlashStartTime);
    #endif
  }
}

void writeModbusValue(uint16_t addr, float value) {
    float processedVal = value / mbDivision;
    uint16_t W1 = 0, W2 = 0;

    if (mbDataType == 0) { // FLOAT
        union { float f; uint8_t bytes[4]; } conv;
        conv.f = processedVal;
        switch (mbFormat) {
            case FLOAT_ABCD: W1 = (conv.bytes[3] << 8) | conv.bytes[2]; W2 = (conv.bytes[1] << 8) | conv.bytes[0]; break;
            case FLOAT_CDAB: W1 = (conv.bytes[1] << 8) | conv.bytes[0]; W2 = (conv.bytes[3] << 8) | conv.bytes[2]; break;
            case FLOAT_BADC: W1 = (conv.bytes[2] << 8) | conv.bytes[3]; W2 = (conv.bytes[0] << 8) | conv.bytes[1]; break;
            case FLOAT_DCBA: W1 = (conv.bytes[0] << 8) | conv.bytes[1]; W2 = (conv.bytes[2] << 8) | conv.bytes[3]; break;
        }
    } else { // INT32
        int32_t iVal = (int32_t)processedVal;
        W1 = (uint16_t)(iVal >> 16);
        W2 = (uint16_t)(iVal & 0xFFFF);
    }
    holdingRegs[addr] = W1;
    holdingRegs[addr + 1] = W2;
}

ModbusMessage readModbusRegisters(ModbusMessage request) {
    uint16_t addr = 0, words = 0;
    request.get(2, addr); request.get(4, words);
    #if defined(ARDUINO_ESP32S3_DEV) || defined(BOARD_HAS_RGB_LED)
        flashLEDColor(COLOR_ORANGE, ledModbusFlashStartTime);
    #endif
    ModbusMessage response;
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    for (uint16_t i = 0; i < words; i++) {
        uint16_t regAddr = addr + i;
        response.add(holdingRegs.count(regAddr) ? holdingRegs[regAddr] : (uint16_t)0);
    }
    return response;
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
    html += "<table><tr><td>Preset:</td><td><select name='preset' id='presetSelect' onchange='updateMbPresetFields()'>";
    html += "<option value='Eastron' " + String(preset=="Eastron"?"selected":"") + ">Eastron</option>";
    html += "<option value='Fronius' " + String(preset=="Fronius"?"selected":"") + ">Fronius</option>";
    html += "<option value='Custom' " + String(preset=="Custom"?"selected":"") + ">Custom</option></select></td></tr>";
    
    html += "<tr><td>Slave ID:</td><td><input type='number' name='modbusId' value='" + String(modbusAddress) + "'></td></tr>";
    
    html += "<tr><td>Byte Order:</td><td><select name='mbFormat' id='mbFormat'>";
    const char* formats[] = {"ABCD (Big Endian)", "CDAB (Swapped Word)", "BADC (Swapped Byte)", "DCBA (Little Endian)"};
    for(int i=0; i<4; i++) html += "<option value='"+String(i)+"' "+(mbFormat==i?"selected":"")+">"+formats[i]+"</option>";
    html += "</select></td></tr>";

    html += "<tr><td>Data Type:</td><td><select name='mbDataType' id='mbDataType'>";
    html += "<option value='0' "+String(mbDataType==0?"selected":"")+">Float (4-byte)</option>";
    html += "<option value='1' "+String(mbDataType==1?"selected":"")+">INT32 (4-byte)</option></select></td></tr>";
    
    html += "<tr><td>Division Factor:</td><td><input type='number' step='0.001' name='mbDiv' id='mbDiv' value='" + String(mbDivision) + "'></td></tr></table>";
    
    html += "<h4>Register Mapping (4xxxx)</h4><table><tr><th>P1 Metric</th><th>Start Address (Dec)</th></tr>";
    for(int i=0; i<P1_VALUE_COUNT; i++) {
        html += "<tr><td>"+String(allP1Values[i].name)+"</td><td><input type='number' name='reg_"+String(i)+"' value='"+String(allP1Values[i].regAddr)+"' class='reg-field'></td></tr>";
    }
    html += "</table><br><input type='submit' value='Save Modbus Settings'></form>";
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
        html += "  if(p !== 'Custom') { f.value = '1'; t.value = '0'; d.value = '1'; f.disabled=true; t.disabled=true; d.disabled=true; for(var i=0;i<regs.length;i++) regs[i].readOnly=true; }";
        html += "  else { f.disabled=false; t.disabled=false; d.disabled=false; for(var i=0;i<regs.length;i++) regs[i].readOnly=false; }";
        html += "}";
        html += "function openTab(evt, tn){ var i, tc, tl; tc=document.getElementsByClassName('tabcontent'); for(i=0;i<tc.length;i++)tc[i].style.display='none'; tl=document.getElementsByClassName('tablinks'); for(i=0;i<tl.length;i++)tl[i].className=tl[i].className.replace(' active',''); document.getElementById(tn).style.display='block'; evt.currentTarget.className+=' active'; }";
        html += "function submitConfig(){ var p=document.getElementById('pincode').value; document.getElementById('pincodeHidden').value=p; document.getElementById('configForm').submit(); }";
        html += "function clearLogs() { document.getElementById('logs').innerHTML = ''; }";
        html += "var ws=new WebSocket('ws://'+location.host+'/ws'); ws.onmessage=function(e){ if (e.data.startsWith('LOG:')) { var l=document.getElementById('logs'); if(l) { l.innerHTML += e.data.substring(4) + '\\n'; l.scrollTop = l.scrollHeight; } } else if (e.data.startsWith('DATA:')) { var d=document.getElementById('data'); if(d) d.innerHTML=e.data.substring(5); } else if (e.data.startsWith('UPTIME:')) { var u=document.getElementById('uptime_val'); if(u) u.innerText=e.data.substring(7); } };";
        html += "</script></head><body onload=\"updateMbPresetFields(); document.getElementById('defaultOpen').click();\">";
        html += "<h1>ESP32 P1 Bridge</h1><div class='tab'><button class='tablinks' onclick=\"openTab(event, 'Status')\" id='defaultOpen'>Status</button><button class='tablinks' onclick=\"openTab(event, 'Modbus')\">Modbus Config</button><button class='tablinks' onclick=\"openTab(event, 'Config')\">General Config</button><button class='tablinks' onclick=\"openTab(event, 'Logs')\">Logs</button></div>";
        html += "<div id='Status' class='tabcontent'>" + generateStatusTabContent() + "</div>";
        html += "<div id='Modbus' class='tabcontent'>" + generateModbusTabContent() + "</div>";
        html += "<div id='Config' class='tabcontent'>" + generateConfigTabContent() + "</div>";
        html += "<div id='Logs' class='tabcontent'>" + generateLogTabContent() + "</div></body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/config_mb", HTTP_POST, [](AsyncWebServerRequest *request) {
        prefs.begin("config", false);
        if(request->hasParam("preset", true)) {
            String p = request->getParam("preset", true)->value();
            prefs.putString("preset", p);
            if(p == "Custom") {
                prefs.putUChar("mbFormat", request->getParam("mbFormat", true)->value().toInt());
                prefs.putUChar("mbDataType", request->getParam("mbDataType", true)->value().toInt());
                prefs.putFloat("mbDiv", request->getParam("mbDiv", true)->value().toFloat());
                for(int i=0; i<P1_VALUE_COUNT; i++) {
                    String key = "r" + String(i);
                    prefs.putUInt(key.c_str(), request->getParam("reg_"+String(i), true)->value().toInt());
                }
            } else { 
                prefs.putUChar("mbFormat", 1); prefs.putUChar("mbDataType", 0); prefs.putFloat("mbDiv", 1.0);
            }
        }
        prefs.putUChar("modbusId", request->getParam("modbusId", true)->value().toInt());
        prefs.end();
        request->send(200, "text/html", "Modbus settings saved. Restarting..."); delay(1000); ESP.restart();
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
        request->send(200, "text/html", "<html><head><script type=\"text/javascript\">window.location = \"/\";</script></head></html>"); delay(1000); ESP.restart();
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
  mbFormat = prefs.getUChar("mbFormat", 0);
  mbDataType = prefs.getUChar("mbDataType", 0);
  mbDivision = prefs.getFloat("mbDiv", 1.0);
  mqttEnabled = prefs.getBool("mqttEnabled", false);
  mqttHost = prefs.getString("mqttHost", "");
  mqttUser = prefs.getString("mqttUser", "");
  mqttPass = prefs.getString("mqttPass", "");
  for(int i=0; i<P1_VALUE_COUNT; i++) {
      String key = "r" + String(i);
      allP1Values[i].regAddr = prefs.getUInt(key.c_str(), allP1Values[i].regAddr);
  }
  prefs.end();

  if (p1UseSerial) Serial1.begin(P1_BAUD, SERIAL_8N1, P1_RX_PIN, -1, p1InvertSignal);
  
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
    holdingRegs.clear();
    for (int i=0; i < P1_VALUE_COUNT; i++) {
        writeModbusValue(allP1Values[i].regAddr, *allP1Values[i].ptr);
    }
    sendLiveData(); 
    if (mqttEnabled && mqtt.connected()) mqtt.publish((String(MQTT_BASE_TOPIC) + "/Power").c_str(), String(netTotalPowerW).c_str());
  }
  if (mqttEnabled && !mqtt.connected()) mqtt.connect("ESP32P1", mqttUser.c_str(), mqttPass.c_str());
  mqtt.loop();
}