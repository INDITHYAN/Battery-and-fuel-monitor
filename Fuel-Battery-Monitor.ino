/*
 * ============================================================
 *  MULTI-MODE MONITOR v3  —  ESP32 Edition
 *  
 *  HC-SR04 + Potentiometer + Toggle Switch + I2C LCD
 *  WiFi Web Server  +  Server-Sent Events  +  SPIFFS Logging
 *
 *  PIN MAP (ESP32):
 *    GPIO 18  —  HC-SR04 TRIG
 *    GPIO 19  —  HC-SR04 ECHO
 *    GPIO 34  —  Potentiometer wiper  (ADC1 — must use ADC1 on ESP32)
 *    GPIO  4  —  Toggle Switch  (INPUT_PULLUP, other pin → GND)
 *    GPIO 21  —  I2C SDA  (LCD)
 *    GPIO 22  —  I2C SCL  (LCD)
 *
 *  LIBRARIES (install via Arduino Library Manager):
 *    - LiquidCrystal_I2C  by Frank de Brabander
 *    - ESPAsyncWebServer   by ESP Async Web Server (or me-no-dev)
 *    - AsyncTCP            by me-no-dev
 *    - ArduinoJson         by Benoit Blanchon
 *
 *  BOARD SETUP:
 *    Board:       "ESP32 Dev Module"  (or your specific board)
 *    Partition:   "Default 4MB with spiffs"  ← important for file storage
 *    Flash Size:  4MB
 *
 *  SETUP STEPS:
 *    1. Set your WiFi SSID and password below
 *    2. Set your bottle height in BOTTLE_HEIGHT_CM
 *    3. Upload this sketch
 *    4. Open Serial Monitor at 115200 baud
 *    5. Note the IP address printed (e.g. 192.168.1.42)
 *    6. Open that IP in any browser on the same WiFi
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <time.h>

/* ── WiFi Credentials ─────────────────────────────────────── */
const char* WIFI_SSID     = "YOUR_WIFI_NAME";      // ← Change this
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";  // ← Change this

/* ── NTP Time (for log timestamps) ───────────────────────── */
const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET_S  = 19800;   // IST = UTC+5:30 = 19800s
const int   DAYLIGHT_S    = 0;

/* ── Pin Definitions ──────────────────────────────────────── */
#define TRIG_PIN       18
#define ECHO_PIN       19
#define POT_PIN        34    /* ADC1 channel 6 */
#define SWITCH_PIN      4

/* ── Bottle / Sensor Config ───────────────────────────────── */
#define BOTTLE_HEIGHT_CM  20.0f   /* usable water height in cm */
#define SENSOR_MIN_CM      2.0f   /* min sensor distance (air gap at top) */

/* ── Smoothing ────────────────────────────────────────────── */
#define SMOOTH_N       8

/* ── Timings ──────────────────────────────────────────────── */
#define SENSOR_MS      500    /* sensor read + SSE push interval */
#define LOG_SECS       30     /* log to SPIFFS every 30 seconds */
#define ALERT_PCT      10.0f  /* critical fuel threshold */
#define LCD_MS         600    /* LCD update interval */

/* ── Vehicle Constants ────────────────────────────────────── */
#define CAR_TANK_L     50.0f
#define CAR_RANGE_KM  600.0f
#define GEN_TANK_L     25.0f
#define GEN_LPH         2.5f
#define PB_MAH      20000.0f
#define PB_DRAIN      500.0f
#define EV_RANGE_KM   400.0f
#define HYB_TANK_L     40.0f
#define HYB_RANGE_KM  500.0f
#define HYB_EV_KM      50.0f

/* ── LCD ──────────────────────────────────────────────────── */
#define LCD_ADDR       0x27    /* try 0x3F if blank */
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

/* ── Web Server + SSE ─────────────────────────────────────── */
AsyncWebServer  server(80);
AsyncEventSource events("/events");   /* SSE endpoint */

/* ── Modes ────────────────────────────────────────────────── */
typedef enum {
    MODE_NONE = 0,
    MODE_POWERBANK,
    MODE_CAR,
    MODE_HYBRID,
    MODE_EV,
    MODE_GENERATOR
} AppMode;

const char* modeName(AppMode m) {
    switch(m) {
        case MODE_POWERBANK: return "PowerBank";
        case MODE_CAR:       return "Car";
        case MODE_HYBRID:    return "Hybrid";
        case MODE_EV:        return "EV";
        case MODE_GENERATOR: return "Generator";
        default:             return "None";
    }
}

/* ── Global State ─────────────────────────────────────────── */
int       fuel_smooth[SMOOTH_N] = {50};
int       batt_smooth[SMOOTH_N] = {50};
int       smooth_idx  = 0;
int       smooth_cnt  = 0;

volatile int  g_fuel   = 50;
volatile int  g_batt   = 50;
volatile int  g_sw     = 0;
volatile bool g_alert  = false;
volatile AppMode g_mode = MODE_NONE;

unsigned long last_sensor = 0;
unsigned long last_lcd    = 0;
unsigned long last_log    = 0;

bool  wifi_ok    = false;
bool  spiffs_ok  = false;
bool  time_synced = false;

/* ============================================================
 *  SENSOR FUNCTIONS
 * ============================================================ */
float read_ultrasonic_cm() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
    if (dur == 0) return SENSOR_MIN_CM + BOTTLE_HEIGHT_CM;
    float d = (dur * 0.0343f) / 2.0f;
    d = constrain(d, SENSOR_MIN_CM, SENSOR_MIN_CM + BOTTLE_HEIGHT_CM);
    return d;
}

int dist_to_pct(float dist) {
    float pct = (SENSOR_MIN_CM + BOTTLE_HEIGHT_CM - dist)
                / BOTTLE_HEIGHT_CM * 100.0f;
    return (int)constrain(pct, 0, 100);
}

int smooth_val(int* buf, int new_val) {
    buf[smooth_idx] = new_val;
    long sum = 0;
    int  cnt = (smooth_cnt < SMOOTH_N) ? smooth_cnt + 1 : SMOOTH_N;
    for (int i = 0; i < cnt; i++) sum += buf[i];
    return (int)(sum / cnt);
}

/* ============================================================
 *  LCD UPDATE
 * ============================================================ */
void update_lcd(int fuel, int batt, int sw_on, AppMode mode) {
    char L1[17], L2[17];
    switch (mode) {
        case MODE_NONE:
            snprintf(L1,17,"Waiting...      ");
            snprintf(L2,17,"Open browser    ");
            break;
        case MODE_POWERBANK: {
            int mah = (int)(batt / 100.0f * PB_MAH);
            snprintf(L1,17,"POWER BANK      ");
            snprintf(L2,17,"Bat:%d%%  %dmAh  ",batt,mah);
            break;
        }
        case MODE_CAR: {
            float lit = fuel / 100.0f * CAR_TANK_L;
            snprintf(L1,17,"CAR FUEL        ");
            snprintf(L2,17,"%d%%  %.1fL      ",fuel,lit);
            break;
        }
        case MODE_HYBRID: {
            const char* s = sw_on ? "ELEC" : "FUEL";
            snprintf(L1,17,"HYBRID  [%-4s]   ",s);
            snprintf(L2,17,"F:%3d%%  B:%3d%% ",fuel,batt);
            break;
        }
        case MODE_EV: {
            int rng = (int)(batt / 100.0f * EV_RANGE_KM);
            snprintf(L1,17,"ELECTRIC CAR    ");
            snprintf(L2,17,"Bat:%3d%% %3dkm  ",batt,rng);
            break;
        }
        case MODE_GENERATOR: {
            float lit = fuel / 100.0f * GEN_TANK_L;
            float hrs = lit / GEN_LPH;
            int   hh  = (int)hrs;
            int   mm  = (int)((hrs - hh) * 60);
            snprintf(L1,17,"GENERATOR       ");
            snprintf(L2,17,"%d%%  %.1fL %dh%02dm ",fuel,lit,hh,mm);
            break;
        }
    }
    L1[16] = L2[16] = '\0';
    while (strlen(L1) < 16) strcat(L1," ");
    while (strlen(L2) < 16) strcat(L2," ");
    lcd.setCursor(0,0); lcd.print(L1);
    lcd.setCursor(0,1); lcd.print(L2);
}

/* ============================================================
 *  SPIFFS LOGGING  —  CSV append
 *  File: /data/log.csv
 * ============================================================ */
void spiffs_log(int fuel, int batt, int sw_on, AppMode mode, bool is_alert) {
    if (!spiffs_ok) return;
    File f = SPIFFS.open("/log.csv", FILE_APPEND);
    if (!f) return;

    /* Timestamp */
    char ts[24] = "no-time";
    if (time_synced) {
        struct tm ti;
        time_t now = time(NULL);
        localtime_r(&now, &ti);
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &ti);
    }

    float liters = 0, range = 0;
    if (mode == MODE_CAR)       { liters = fuel/100.0f*CAR_TANK_L; range = fuel/100.0f*CAR_RANGE_KM; }
    else if (mode == MODE_GENERATOR)  { liters = fuel/100.0f*GEN_TANK_L; }
    else if (mode == MODE_EV)   { range  = batt/100.0f*EV_RANGE_KM; }

    f.printf("%s,%s,%d,%d,%d,%.2f,%.1f,%s\n",
             ts, modeName(mode), fuel, batt, sw_on,
             liters, range, is_alert ? "ALERT" : "OK");
    f.close();

    /* Also log alert separately */
    if (is_alert) {
        File af = SPIFFS.open("/alerts.csv", FILE_APPEND);
        if (af) {
            af.printf("%s,%s,%.1f,CRITICAL: Fuel below %d%%!\n",
                      ts, modeName(mode), (float)fuel, (int)ALERT_PCT);
            af.close();
        }
    }
    Serial.printf("[LOG] %s  %s  fuel=%d%%  batt=%d%%  alert=%s\n",
                  ts, modeName(mode), fuel, batt,
                  is_alert ? "YES" : "no");
}

/* ============================================================
 *  BUILD JSON  (sent via SSE and /api/data)
 * ============================================================ */
String build_json() {
    StaticJsonDocument<256> doc;
    doc["fuel"]  = g_fuel;
    doc["batt"]  = g_batt;
    doc["sw"]    = g_sw;
    doc["mode"]  = (int)g_mode;
    doc["alert"] = g_alert;

    /* Calculated values */
    float fuel = g_fuel, batt = g_batt;
    doc["liters_fuel"] = fuel/100.0f * CAR_TANK_L;
    doc["range_car"]   = fuel/100.0f * CAR_RANGE_KM;
    doc["liters_gen"]  = fuel/100.0f * GEN_TANK_L;
    doc["hrs_gen"]     = (fuel/100.0f * GEN_TANK_L) / GEN_LPH;
    doc["mah"]         = batt/100.0f * PB_MAH;
    doc["hrs_pb"]      = (batt/100.0f * PB_MAH) / PB_DRAIN;
    doc["range_ev"]    = batt/100.0f * EV_RANGE_KM;
    doc["kwh"]         = batt/100.0f * 75.0f;
    doc["range_hyb_f"] = fuel/100.0f * HYB_RANGE_KM;
    doc["range_hyb_e"] = batt/100.0f * HYB_EV_KM;

    String out;
    serializeJson(doc, out);
    return out;
}

/* ============================================================
 *  READ LOG CSV  →  JSON array (for history screen)
 *  Two-pass: first count lines, then read last N
 * ============================================================ */
String read_log_json(int limit) {
    if (!spiffs_ok) return "[]";
    File f = SPIFFS.open("/log.csv", FILE_READ);
    if (!f) return "[]";

    /* Pass 1: count total non-header lines */
    int total = 0;
    while (f.available()) {
        String ln = f.readStringUntil('\n');
        ln.trim();
        if (ln.length() > 5 && !ln.startsWith("Timestamp")) total++;
    }
    f.close();

    /* Pass 2: skip to last `limit` lines and build JSON */
    f = SPIFFS.open("/log.csv", FILE_READ);
    if (!f) return "[]";

    int skip = max(0, total - limit);
    int skipped = 0;
    String out = "[";
    bool first = true;

    while (f.available()) {
        String ln = f.readStringUntil('\n');
        ln.trim();
        if (ln.length() < 5) continue;
        if (ln.startsWith("Timestamp")) continue;  /* skip CSV header */
        if (skipped < skip) { skipped++; continue; }

        /* Parse CSV: ts,mode,fuel,batt,sw,liters,range,alert */
        String parts[8];
        int pi = 0, prev = 0;
        for (int j = 0; j <= (int)ln.length() && pi < 8; j++) {
            if (j == (int)ln.length() || ln[j] == ',') {
                parts[pi++] = ln.substring(prev, j);
                prev = j + 1;
            }
        }
        if (pi < 8) continue;

        if (!first) out += ",";
        first = false;
        out += String("{") +
               "\"ts\":\"" + parts[0] + "\"," +
               "\"mode\":\"" + parts[1] + "\"," +
               "\"fuel\":" + parts[2] + "," +
               "\"batt\":" + parts[3] + "," +
               "\"sw\":" + parts[4] + "," +
               "\"liters\":" + parts[5] + "," +
               "\"range\":" + parts[6] + "," +
               "\"alert\":" + (parts[7].indexOf("ALERT") >= 0 ? "true" : "false") +
               "}";
    }
    f.close();
    return out + "]";
}

/* ============================================================
 *  FORWARD DECLARATION  —  dashboard_html defined at bottom
 * ============================================================ */
extern const char dashboard_html[] PROGMEM;
String read_log_json(int limit = 60);

/* ============================================================
 *  WEB SERVER SETUP
 * ============================================================ */
void setup_server() {
    /* ── /  →  main dashboard HTML (served inline) ── */
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send_P(200, "text/html", dashboard_html);
    });

    /* ── /api/data  →  current JSON snapshot ── */
    server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send(200, "application/json", build_json());
    });

    /* ── /api/mode  →  set mode from browser ── */
    server.on("/api/mode", HTTP_GET, [](AsyncWebServerRequest* req){
        if (req->hasParam("m")) {
            int m = req->getParam("m")->value().toInt();
            if (m >= 0 && m <= 5) {
                g_mode = (AppMode)m;
                lcd.clear();
            }
        }
        req->send(200, "text/plain", "OK");
    });

    /* ── /api/history  →  log CSV as JSON ── */
    server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest* req){
        int limit = 60;
        if (req->hasParam("n"))
            limit = req->getParam("n")->value().toInt();
        req->send(200, "application/json", read_log_json(limit));
    });

    /* ── /api/export  →  raw CSV download ── */
    server.on("/api/export", HTTP_GET, [](AsyncWebServerRequest* req){
        if (!spiffs_ok || !SPIFFS.exists("/log.csv")) {
            req->send(404, "text/plain", "No log file yet");
            return;
        }
        req->send(SPIFFS, "/log.csv", "text/csv",
                  true /* download */);
    });

    /* ── /api/clear  →  wipe logs ── */
    server.on("/api/clear", HTTP_GET, [](AsyncWebServerRequest* req){
        if (spiffs_ok) {
            SPIFFS.remove("/log.csv");
            SPIFFS.remove("/alerts.csv");
        }
        req->send(200, "text/plain", "Logs cleared");
    });

    /* ── SSE endpoint  /events ── */
    events.onConnect([](AsyncEventSourceClient* client){
        Serial.printf("[SSE] Client connected. IP: %s\n",
                      client->client()->remoteIP().toString().c_str());
        client->send(build_json().c_str(), "data", millis(), 1000);
    });
    server.addHandler(&events);

    server.begin();
    Serial.println("[Server] Started on port 80");
}

/* ============================================================
 *  SETUP
 * ============================================================ */
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[Boot] Multi-Mode Monitor v3 — ESP32");

    /* ── Pins ── */
    pinMode(TRIG_PIN,   OUTPUT);
    pinMode(ECHO_PIN,   INPUT);
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    /* GPIO 34 is input-only on ESP32, no pinMode needed */

    /* ── I2C LCD ── */
    Wire.begin(21, 22);   /* SDA=21, SCL=22 */
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0); lcd.print("  ESP32 Monitor ");
    lcd.setCursor(0,1); lcd.print("  Booting...    ");

    /* ── SPIFFS ── */
    spiffs_ok = SPIFFS.begin(true);
    if (spiffs_ok) {
        Serial.println("[SPIFFS] OK");
        /* Write CSV header if file doesn't exist */
        if (!SPIFFS.exists("/log.csv")) {
            File f = SPIFFS.open("/log.csv", FILE_WRITE);
            if (f) {
                f.print("Timestamp,Mode,Fuel%,Batt%,"
                        "Switch,Liters,RangeKM,Alert\n");
                f.close();
            }
        }
    } else {
        Serial.println("[SPIFFS] Failed — logging disabled");
    }

    /* ── WiFi ── */
    lcd.setCursor(0,1); lcd.print("WiFi: Joining...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 30) {
        delay(500); Serial.print("."); tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifi_ok = true;
        Serial.printf("\n[WiFi] Connected! IP: %s\n",
                      WiFi.localIP().toString().c_str());
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("WiFi Connected! ");
        lcd.setCursor(0,1);
        char ipbuf[17];
        snprintf(ipbuf,17,"%s",WiFi.localIP().toString().c_str());
        lcd.print(ipbuf);
        delay(2000);

        /* NTP time sync */
        configTime(GMT_OFFSET_S, DAYLIGHT_S, NTP_SERVER);
        struct tm ti;
        if (getLocalTime(&ti, 5000)) {
            time_synced = true;
            Serial.println("[NTP] Time synced");
        }

        /* Start web server */
        setup_server();
    } else {
        Serial.println("\n[WiFi] Failed — running offline");
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("WiFi FAILED     ");
        lcd.setCursor(0,1); lcd.print("No web server   ");
        delay(2000);
    }

    /* ── Show IP on LCD ── */
    lcd.clear();
    if (wifi_ok) {
        lcd.setCursor(0,0); lcd.print("Open browser:   ");
        char ipbuf[17];
        snprintf(ipbuf,17,"%s",WiFi.localIP().toString().c_str());
        lcd.setCursor(0,1); lcd.print(ipbuf);
    } else {
        lcd.setCursor(0,0); lcd.print("  Monitor v3    ");
        lcd.setCursor(0,1); lcd.print("  No WiFi  :(   ");
    }
    delay(1500);
    lcd.clear();
    update_lcd(50, 50, 0, MODE_NONE);

    Serial.println("[Boot] Ready.");
    if (wifi_ok)
        Serial.printf("  →  Dashboard:  http://%s\n",
                      WiFi.localIP().toString().c_str());
    Serial.printf("  →  Export CSV:  http://%s/api/export\n\n",
                  WiFi.localIP().toString().c_str());
}

/* ============================================================
 *  MAIN LOOP
 * ============================================================ */
void loop() {
    unsigned long now = millis();

    /* ── Read sensors every 500ms ── */
    if (now - last_sensor >= SENSOR_MS) {
        last_sensor = now;

        float dist    = read_ultrasonic_cm();
        int   fuel_r  = dist_to_pct(dist);
        int   batt_r  = map(analogRead(POT_PIN), 0, 4095, 0, 100);
        int   sw_val  = (digitalRead(SWITCH_PIN) == LOW) ? 1 : 0;

        int fuel = smooth_val(fuel_smooth, fuel_r);
        int batt = smooth_val(batt_smooth, batt_r);
        smooth_idx = (smooth_idx + 1) % SMOOTH_N;
        if (smooth_cnt < SMOOTH_N) smooth_cnt++;

        g_fuel  = fuel;
        g_batt  = batt;
        g_sw    = sw_val;
        g_alert = (fuel < (int)ALERT_PCT);

        /* Push SSE to all connected browsers */
        if (wifi_ok) {
            events.send(build_json().c_str(), "data", millis());
        }

        /* Serial debug */
        Serial.printf("FUEL=%d BATT=%d SW=%d ALERT=%s MODE=%s\n",
                      fuel, batt, sw_val,
                      g_alert ? "YES" : "no",
                      modeName(g_mode));
    }

    /* ── LCD update every 600ms ── */
    if (now - last_lcd >= LCD_MS) {
        last_lcd = now;
        update_lcd(g_fuel, g_batt, g_sw, g_mode);
    }

    /* ── Log to SPIFFS every 30 seconds ── */
    if (now - last_log >= (unsigned long)LOG_SECS * 1000UL) {
        last_log = now;
        if (g_mode != MODE_NONE) {
            spiffs_log(g_fuel, g_batt, g_sw, g_mode, g_alert);
        }
    }

    delay(20);
}

/* ============================================================
 *  DASHBOARD HTML  —  full dark UI served inline
 *  (saves needing separate SPIFFS file upload)
 *
 *  Stored in program flash (PROGMEM) to save RAM
 * ============================================================ */
const char dashboard_html[] PROGMEM = R"RAWHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Multi-Mode Monitor v3</title>
<style>
:root{
  --bg:#0A0B0F;--card:#14161C;--card2:#1C1E28;--sep:#282B37;
  --dim:#373945;--white:#fff;--offwhite:#DCE0E8;--gray:#828491;
  --green:#27C96E;--cyan:#22D3EE;--blue:#3B82F6;
  --yellow:#EAB40C;--orange:#EA6C0A;--red:#DC2626;
  --amber:#F59E2B;--purple:#8B5CF6;
  --lcdg:#50C850;--lcdb:#143814;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--white);
     font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh}

/* ── TOP NAV ── */
.nav{background:var(--card);border-bottom:1px solid var(--sep);
     padding:14px 24px;display:flex;align-items:center;justify-content:space-between}
.nav h1{font-size:1.2rem;font-weight:700;letter-spacing:.5px}
.nav-right{display:flex;align-items:center;gap:16px;font-size:.82rem;color:var(--gray)}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:5px}
.dot.on{background:var(--green);box-shadow:0 0 6px var(--green)}
.dot.off{background:var(--red)}

/* ── MODE SELECT STRIP ── */
.modes{display:flex;gap:10px;padding:18px 24px;
       background:var(--card2);border-bottom:1px solid var(--sep);
       flex-wrap:wrap}
.mbtn{flex:1;min-width:130px;padding:14px 10px 12px;
      border-radius:10px;border:1.5px solid transparent;
      background:var(--card);cursor:pointer;text-align:center;
      transition:.2s;color:var(--white)}
.mbtn:hover{transform:translateY(-2px);filter:brightness(1.15)}
.mbtn.active{border-color:currentColor;background:var(--card2)}
.mbtn .icon{font-size:.85rem;font-weight:700;margin-bottom:5px}
.mbtn .mname{font-size:.92rem;font-weight:600}
.mbtn .mdesc{font-size:.72rem;color:var(--gray);margin-top:3px}
.mbtn[data-m="1"]{color:var(--cyan)}
.mbtn[data-m="2"]{color:var(--yellow)}
.mbtn[data-m="3"]{color:var(--green)}
.mbtn[data-m="4"]{color:var(--blue)}
.mbtn[data-m="5"]{color:var(--orange)}
.mbtn[data-m="h"]{color:var(--purple);min-width:110px;flex:0.8}

/* ── MAIN CONTENT ── */
.content{padding:24px;max-width:1100px;margin:0 auto}

/* ── DASH PANEL ── */
.dash{display:grid;grid-template-columns:1fr 1fr;gap:18px;align-items:start}
@media(max-width:700px){.dash{grid-template-columns:1fr}}

/* ── GAUGE ── */
.gauge-wrap{background:var(--card);border-radius:14px;
            padding:24px;display:flex;flex-direction:column;
            align-items:center;gap:12px;border:1px solid var(--sep)}
.gauge-svg{width:100%;max-width:280px}
.gauge-pct{font-size:2.8rem;font-weight:800;text-align:center;
           line-height:1;margin-top:-10px}
.gauge-label{font-size:.78rem;color:var(--gray);text-align:center;
             letter-spacing:1px;text-transform:uppercase}

/* ── STAT CARDS ── */
.stats{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.stat{background:var(--card2);border-radius:10px;
      padding:14px 16px;border:1px solid var(--sep)}
.stat-label{font-size:.72rem;color:var(--gray);text-transform:uppercase;
            letter-spacing:.8px;margin-bottom:6px}
.stat-val{font-size:1.25rem;font-weight:700}

/* ── BAR ── */
.bar-track{background:var(--dim);border-radius:99px;height:8px;
           overflow:hidden;margin:4px 0 2px}
.bar-fill{height:100%;border-radius:99px;transition:width .4s}

/* ── BATTERY BARS ── */
.bat-wrap{display:flex;align-items:flex-start;gap:18px}
.bat-vert{display:flex;flex-direction:column-reverse;gap:3px;
          width:48px;flex-shrink:0}
.bat-seg{height:22px;border-radius:4px;transition:background .3s}
.bat-cap{width:20px;height:7px;border-radius:2px;background:var(--gray);
         margin:0 auto 2px}

/* ── LCD WIDGET ── */
.lcd{background:var(--lcdb);border:1.5px solid var(--lcdg);
     border-radius:8px;padding:10px 14px;font-family:'Courier New',monospace;
     font-size:.9rem;font-weight:700;color:var(--lcdg);
     letter-spacing:.5px;line-height:1.7;white-space:pre}
.lcd-label{font-size:.7rem;color:var(--lcdg);opacity:.6;
           margin-bottom:4px;letter-spacing:1px}

/* ── HYBRID PANELS ── */
.hyb-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.hyb-panel{background:var(--card);border-radius:12px;
           padding:18px;border:1px solid var(--sep);
           text-align:center}
.hyb-panel.active-panel{border-color:currentColor;
                         background:var(--card2)}
.hyb-title{font-size:.82rem;font-weight:700;letter-spacing:1px;
           text-transform:uppercase;margin-bottom:12px}

/* ── ALERT OVERLAY ── */
.alert-overlay{display:none;position:fixed;inset:0;z-index:999;
               pointer-events:none;animation:blink-bg .5s infinite alternate}
.alert-overlay.show{display:block}
@keyframes blink-bg{
  from{background:rgba(220,38,38,.08)}
  to  {background:rgba(220,38,38,.22)}
}
.alert-box{position:fixed;left:50%;top:50%;transform:translate(-50%,-70%);
           background:#220000;border:2.5px solid var(--red);
           border-radius:14px;padding:22px 36px;text-align:center;
           z-index:1000;animation:pulse-border .5s infinite alternate}
@keyframes pulse-border{
  from{border-color:rgba(220,38,38,.5)}
  to  {border-color:var(--red)}
}
.alert-box h2{color:var(--red);font-size:1.35rem;margin-bottom:8px}
.alert-box p{color:var(--offwhite);font-size:1rem}
.alert-bar{position:fixed;top:0;left:0;right:0;height:6px;
           background:var(--red);z-index:1001;
           animation:blink-bar .5s infinite alternate}
.alert-bar.bottom{top:auto;bottom:0}
@keyframes blink-bar{from{opacity:1}to{opacity:.2}}

/* ── HISTORY ── */
.history-wrap{display:grid;grid-template-columns:1.5fr 1fr;
              gap:20px;align-items:start}
@media(max-width:700px){.history-wrap{grid-template-columns:1fr}}
.hist-graph{background:var(--card);border-radius:12px;
            padding:20px;border:1px solid var(--sep)}
.hist-graph h3{font-size:.9rem;color:var(--gray);
               text-transform:uppercase;letter-spacing:1px;margin-bottom:14px}
#graphCanvas{width:100%;height:240px;display:block}
.hist-right{display:flex;flex-direction:column;gap:14px}
.stat-row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.hist-table{background:var(--card);border-radius:12px;
            overflow:hidden;border:1px solid var(--sep)}
.hist-table table{width:100%;border-collapse:collapse;font-size:.8rem}
.hist-table th{background:var(--card2);color:var(--gray);
               padding:8px 10px;text-align:left;font-weight:600;
               text-transform:uppercase;font-size:.7rem;letter-spacing:.5px}
.hist-table td{padding:7px 10px;border-top:1px solid var(--sep)}
.hist-table tr:hover td{background:var(--card2)}
td.alert-yes{color:var(--red);font-weight:700}
td.alert-no{color:var(--green)}
.export-btn{background:rgba(39,201,110,.1);border:1.5px solid var(--green);
            color:var(--green);border-radius:10px;padding:12px;
            text-align:center;cursor:pointer;font-weight:700;
            font-size:.9rem;text-decoration:none;display:block;
            transition:.2s}
.export-btn:hover{background:rgba(39,201,110,.2)}
.clear-btn{background:rgba(220,38,38,.08);border:1px solid var(--sep);
           color:var(--gray);border-radius:10px;padding:8px;
           text-align:center;cursor:pointer;font-size:.8rem;margin-top:4px}
.clear-btn:hover{color:var(--red);border-color:var(--red)}

/* ── HIDDEN ── */
.screen{display:none}.screen.active{display:block}
.hidden{display:none!important}
</style>
</head>
<body>

<!-- TOP NAV -->
<div class="nav">
  <h1>⚡ Multi-Mode Monitor v3</h1>
  <div class="nav-right">
    <span><span class="dot" id="connDot"></span><span id="connText">Connecting...</span></span>
    <span id="dbText">DB: –</span>
    <span id="clockText">–</span>
  </div>
</div>

<!-- MODE BUTTONS -->
<div class="modes">
  <button class="mbtn" data-m="1" onclick="setMode(1)">
    <div class="icon">[PB]</div>
    <div class="mname">Power Bank</div>
    <div class="mdesc">Battery%/mAh/Time</div>
  </button>
  <button class="mbtn" data-m="2" onclick="setMode(2)">
    <div class="icon">[CAR]</div>
    <div class="mname">Normal Car</div>
    <div class="mdesc">Fuel%/Liters/Range</div>
  </button>
  <button class="mbtn" data-m="3" onclick="setMode(3)">
    <div class="icon">[HYB]</div>
    <div class="mname">Hybrid Car</div>
    <div class="mdesc">Fuel+Battery+Switch</div>
  </button>
  <button class="mbtn" data-m="4" onclick="setMode(4)">
    <div class="icon">[EV]</div>
    <div class="mname">Electric Car</div>
    <div class="mdesc">Charge%/Range/kWh</div>
  </button>
  <button class="mbtn" data-m="5" onclick="setMode(5)">
    <div class="icon">[GEN]</div>
    <div class="mname">Generator</div>
    <div class="mdesc">Fuel%/Runtime Hrs</div>
  </button>
  <button class="mbtn" data-m="h" onclick="openHistory()">
    <div class="icon">[DB]</div>
    <div class="mname">History</div>
    <div class="mdesc">Graph+Table+CSV</div>
  </button>
</div>

<!-- ALERT OVERLAY -->
<div class="alert-overlay" id="alertOverlay">
  <div class="alert-bar"></div>
  <div class="alert-bar bottom"></div>
</div>
<div class="alert-box hidden" id="alertBox">
  <h2>⚠ CRITICAL FUEL ALERT ⚠</h2>
  <p id="alertMsg">Fuel critically low — REFUEL IMMEDIATELY!</p>
</div>

<!-- SCREENS -->
<div class="content">

  <!-- WELCOME -->
  <div class="screen active" id="screen-0">
    <div style="text-align:center;padding:60px 20px;color:var(--gray)">
      <div style="font-size:3rem;margin-bottom:16px">⚡</div>
      <h2 style="color:var(--white);margin-bottom:10px">Select a Mode Above</h2>
      <p>Choose a dashboard to start monitoring</p>
      <p style="margin-top:8px;font-size:.85rem">Data updates every 500ms via live connection</p>
    </div>
  </div>

  <!-- POWER BANK -->
  <div class="screen" id="screen-1">
    <div class="dash">
      <div class="gauge-wrap">
        <svg class="gauge-svg" viewBox="0 0 280 160" id="gaugePB">
          <path d="M30 150 A110 110 0 0 1 250 150" stroke="#373945" stroke-width="14" fill="none" stroke-linecap="round"/>
          <path d="M30 150 A110 110 0 0 1 250 150" stroke="#22D3EE" stroke-width="14" fill="none" stroke-linecap="round"
                stroke-dasharray="346" stroke-dashoffset="346" id="arcPB" style="transition:.5s"/>
          <line x1="140" y1="150" x2="140" y2="60" stroke="white" stroke-width="3" stroke-linecap="round" id="needlePB"/>
          <circle cx="140" cy="150" r="8" fill="#22D3EE"/>
          <text x="140" y="135" text-anchor="middle" fill="#22D3EE" font-size="26" font-weight="800" id="pctPB">--%</text>
          <text x="140" y="155" text-anchor="middle" fill="#828491" font-size="11">CHARGE LEVEL</text>
          <text x="30" y="162" fill="#373945" font-size="11">E</text>
          <text x="246" y="162" fill="#373945" font-size="11">F</text>
        </svg>
        <div class="bat-cap"></div>
        <div class="bat-vert" id="batVertPB"></div>
      </div>
      <div style="display:flex;flex-direction:column;gap:12px">
        <div class="stats">
          <div class="stat"><div class="stat-label">Battery Level</div><div class="stat-val" id="pb-pct" style="color:var(--cyan)">--%</div></div>
          <div class="stat"><div class="stat-label">Status</div><div class="stat-val" id="pb-status">–</div></div>
          <div class="stat"><div class="stat-label">mAh Remaining</div><div class="stat-val" id="pb-mah" style="color:var(--offwhite)">–</div></div>
          <div class="stat"><div class="stat-label">Time Remaining</div><div class="stat-val" id="pb-time" style="color:var(--cyan)">–</div></div>
        </div>
        <div class="bar-track"><div class="bar-fill" id="barPB" style="background:var(--cyan)"></div></div>
        <div class="lcd-label">I2C LCD 16×2</div>
        <div class="lcd" id="lcdPB">POWER BANK      &#10;Loading...      </div>
      </div>
    </div>
  </div>

  <!-- CAR -->
  <div class="screen" id="screen-2">
    <div class="dash">
      <div class="gauge-wrap">
        <svg class="gauge-svg" viewBox="0 0 280 160" id="gaugeCAR">
          <path d="M30 150 A110 110 0 0 1 250 150" stroke="#373945" stroke-width="14" fill="none" stroke-linecap="round"/>
          <path d="M30 150 A110 110 0 0 1 250 150" stroke="#EAB40C" stroke-width="14" fill="none" stroke-linecap="round"
                stroke-dasharray="346" stroke-dashoffset="346" id="arcCAR" style="transition:.5s"/>
          <line x1="140" y1="150" x2="140" y2="60" stroke="white" stroke-width="3" stroke-linecap="round" id="needleCAR"/>
          <circle cx="140" cy="150" r="8" fill="#EAB40C"/>
          <text x="140" y="135" text-anchor="middle" fill="#EAB40C" font-size="26" font-weight="800" id="pctCAR">--%</text>
          <text x="140" y="155" text-anchor="middle" fill="#828491" font-size="11">FUEL LEVEL</text>
          <text x="30" y="162" fill="#373945" font-size="11">E</text>
          <text x="246" y="162" fill="#373945" font-size="11">F</text>
        </svg>
      </div>
      <div style="display:flex;flex-direction:column;gap:12px">
        <div class="stats">
          <div class="stat"><div class="stat-label">Fuel Level</div><div class="stat-val" id="car-pct" style="color:var(--yellow)">--%</div></div>
          <div class="stat"><div class="stat-label">Status</div><div class="stat-val" id="car-status">–</div></div>
          <div class="stat"><div class="stat-label">Fuel in Tank</div><div class="stat-val" id="car-lit" style="color:var(--offwhite)">–</div></div>
          <div class="stat"><div class="stat-label">Est. Range</div><div class="stat-val" id="car-range" style="color:var(--yellow)">–</div></div>
        </div>
        <div class="bar-track"><div class="bar-fill" id="barCAR" style="background:var(--yellow)"></div></div>
        <div class="lcd-label">I2C LCD 16×2</div>
        <div class="lcd" id="lcdCAR">CAR FUEL        &#10;Loading...      </div>
      </div>
    </div>
  </div>

  <!-- HYBRID -->
  <div class="screen" id="screen-3">
    <div style="margin-bottom:14px;text-align:center">
      <div id="hybSwBadge" style="display:inline-block;background:var(--card2);
           border:1.5px solid var(--yellow);border-radius:99px;
           padding:6px 22px;font-weight:700;font-size:.9rem;color:var(--yellow)">
        [ FUEL MODE ]
      </div>
      <div style="font-size:.72rem;color:var(--dim);margin-top:4px">Toggle Switch on GPIO 4</div>
    </div>
    <div class="hyb-grid">
      <div class="hyb-panel" id="hybFuelPanel" style="color:var(--yellow)">
        <div class="hyb-title">⛽ Fuel Tank</div>
        <svg viewBox="0 0 220 130" style="width:100%;max-width:220px;display:block;margin:0 auto">
          <path d="M20 120 A90 90 0 0 1 200 120" stroke="#373945" stroke-width="12" fill="none" stroke-linecap="round"/>
          <path d="M20 120 A90 90 0 0 1 200 120" stroke="#EAB40C" stroke-width="12" fill="none" stroke-linecap="round"
                stroke-dasharray="283" stroke-dashoffset="283" id="arcHF" style="transition:.5s"/>
          <line x1="110" y1="120" x2="110" y2="52" stroke="white" stroke-width="3" stroke-linecap="round" id="needleHF"/>
          <circle cx="110" cy="120" r="7" fill="#EAB40C"/>
          <text x="110" y="106" text-anchor="middle" fill="#EAB40C" font-size="22" font-weight="800" id="pctHF">--%</text>
        </svg>
        <div id="hybFuelSub" style="font-size:.82rem;color:var(--offwhite);margin-top:8px">–</div>
        <div id="hybFuelRange" style="font-size:.78rem;color:var(--yellow);margin-top:4px">–</div>
      </div>
      <div class="hyb-panel" id="hybBattPanel" style="color:var(--cyan)">
        <div class="hyb-title">🔋 Battery</div>
        <svg viewBox="0 0 220 130" style="width:100%;max-width:220px;display:block;margin:0 auto">
          <path d="M20 120 A90 90 0 0 1 200 120" stroke="#373945" stroke-width="12" fill="none" stroke-linecap="round"/>
          <path d="M20 120 A90 90 0 0 1 200 120" stroke="#22D3EE" stroke-width="12" fill="none" stroke-linecap="round"
                stroke-dasharray="283" stroke-dashoffset="283" id="arcHB" style="transition:.5s"/>
          <line x1="110" y1="120" x2="110" y2="52" stroke="white" stroke-width="3" stroke-linecap="round" id="needleHB"/>
          <circle cx="110" cy="120" r="7" fill="#22D3EE"/>
          <text x="110" y="106" text-anchor="middle" fill="#22D3EE" font-size="22" font-weight="800" id="pctHB">--%</text>
        </svg>
        <div id="hybBattRange" style="font-size:.82rem;color:var(--cyan);margin-top:8px">–</div>
      </div>
    </div>
    <div style="margin-top:14px">
      <div class="lcd-label">I2C LCD 16×2</div>
      <div class="lcd" id="lcdHYB">HYBRID  [FUEL]  &#10;Loading...      </div>
    </div>
  </div>

  <!-- EV -->
  <div class="screen" id="screen-4">
    <div class="dash">
      <div class="gauge-wrap">
        <div class="bat-cap" style="width:50px"></div>
        <div class="bat-vert" style="width:70px;height:300px" id="batVertEV"></div>
        <svg class="gauge-svg" viewBox="0 0 280 160" style="margin-top:8px">
          <path d="M30 150 A110 110 0 0 1 250 150" stroke="#373945" stroke-width="14" fill="none" stroke-linecap="round"/>
          <path d="M30 150 A110 150 0 0 1 250 150" stroke="#3B82F6" stroke-width="14" fill="none" stroke-linecap="round"
                stroke-dasharray="346" stroke-dashoffset="346" id="arcEV" style="transition:.5s"/>
          <line x1="140" y1="150" x2="140" y2="60" stroke="white" stroke-width="3" stroke-linecap="round" id="needleEV"/>
          <circle cx="140" cy="150" r="8" fill="#3B82F6"/>
          <text x="140" y="135" text-anchor="middle" fill="#3B82F6" font-size="26" font-weight="800" id="pctEV">--%</text>
          <text x="140" y="155" text-anchor="middle" fill="#828491" font-size="11">CHARGE</text>
          <text x="30" y="162" fill="#373945" font-size="11">E</text>
          <text x="246" y="162" fill="#373945" font-size="11">F</text>
        </svg>
      </div>
      <div style="display:flex;flex-direction:column;gap:12px">
        <div class="stats">
          <div class="stat"><div class="stat-label">Battery</div><div class="stat-val" id="ev-pct" style="color:var(--blue)">--%</div></div>
          <div class="stat"><div class="stat-label">Status</div><div class="stat-val" id="ev-status">–</div></div>
          <div class="stat"><div class="stat-label">Est. Range</div><div class="stat-val" id="ev-range" style="color:var(--blue)">–</div></div>
          <div class="stat"><div class="stat-label">Energy Left</div><div class="stat-val" id="ev-kwh" style="color:var(--cyan)">–</div></div>
        </div>
        <div style="display:flex;gap:4px" id="cellsEV"></div>
        <div class="lcd-label">I2C LCD 16×2</div>
        <div class="lcd" id="lcdEV">ELECTRIC CAR    &#10;Loading...      </div>
      </div>
    </div>
  </div>

  <!-- GENERATOR -->
  <div class="screen" id="screen-5">
    <div class="dash">
      <div class="gauge-wrap">
        <svg class="gauge-svg" viewBox="0 0 280 160">
          <path d="M30 150 A110 110 0 0 1 250 150" stroke="#373945" stroke-width="14" fill="none" stroke-linecap="round"/>
          <path d="M30 150 A110 110 0 0 1 250 150" stroke="#EA6C0A" stroke-width="14" fill="none" stroke-linecap="round"
                stroke-dasharray="346" stroke-dashoffset="346" id="arcGEN" style="transition:.5s"/>
          <line x1="140" y1="150" x2="140" y2="60" stroke="white" stroke-width="3" stroke-linecap="round" id="needleGEN"/>
          <circle cx="140" cy="150" r="8" fill="#EA6C0A"/>
          <text x="140" y="135" text-anchor="middle" fill="#EA6C0A" font-size="26" font-weight="800" id="pctGEN">--%</text>
          <text x="140" y="155" text-anchor="middle" fill="#828491" font-size="11">FUEL LEVEL</text>
          <text x="30" y="162" fill="#373945" font-size="11">E</text>
          <text x="246" y="162" fill="#373945" font-size="11">F</text>
        </svg>
      </div>
      <div style="display:flex;flex-direction:column;gap:12px">
        <div class="stats">
          <div class="stat"><div class="stat-label">Fuel Level</div><div class="stat-val" id="gen-pct" style="color:var(--orange)">--%</div></div>
          <div class="stat"><div class="stat-label">Status</div><div class="stat-val" id="gen-status">–</div></div>
          <div class="stat"><div class="stat-label">Fuel in Tank</div><div class="stat-val" id="gen-lit" style="color:var(--offwhite)">–</div></div>
          <div class="stat"><div class="stat-label">Runtime Left</div><div class="stat-val" id="gen-hrs" style="color:var(--orange)">–</div></div>
        </div>
        <div style="font-size:.75rem;color:var(--dim);text-align:center">
          Consumption: 2.5 L/hr  |  Tank: 25 L
        </div>
        <div class="bar-track"><div class="bar-fill" id="barGEN" style="background:var(--orange)"></div></div>
        <div class="lcd-label">I2C LCD 16×2</div>
        <div class="lcd" id="lcdGEN">GENERATOR       &#10;Loading...      </div>
      </div>
    </div>
  </div>

  <!-- HISTORY -->
  <div class="screen" id="screen-h">
    <div class="history-wrap">
      <div>
        <div class="hist-graph">
          <h3>📈 Fuel Level History</h3>
          <canvas id="graphCanvas"></canvas>
          <div style="display:flex;gap:16px;margin-top:8px;font-size:.75rem;color:var(--gray)">
            <span style="color:var(--green)">● OK</span>
            <span style="color:var(--yellow)">● Low</span>
            <span style="color:var(--red)">◆ Alert</span>
            <span style="color:var(--dim)">— — 10% threshold</span>
          </div>
        </div>
      </div>
      <div class="hist-right">
        <div class="stat-row">
          <div class="stat"><div class="stat-label">Total Readings</div><div class="stat-val" id="h-total" style="color:var(--cyan)">–</div></div>
          <div class="stat"><div class="stat-label">Alerts Fired</div><div class="stat-val" id="h-alerts" style="color:var(--red)">–</div></div>
          <div class="stat"><div class="stat-label">Avg Fuel</div><div class="stat-val" id="h-avg" style="color:var(--offwhite)">–</div></div>
          <div class="stat"><div class="stat-label">Min Recorded</div><div class="stat-val" id="h-min">–</div></div>
        </div>
        <div class="hist-table">
          <table>
            <thead><tr><th>Time</th><th>Mode</th><th>Fuel%</th><th>Batt%</th><th>Alert</th></tr></thead>
            <tbody id="histTbody"></tbody>
          </table>
        </div>
        <a class="export-btn" href="/api/export" download="monitor_export.csv">⬇ Download CSV</a>
        <div class="clear-btn" onclick="clearLogs()">🗑 Clear All Logs</div>
      </div>
    </div>
  </div>

</div><!-- /content -->

<script>
/* ── STATE ─────────────────────────────────────────────────── */
let cur = {fuel:50,batt:50,sw:0,mode:0,alert:false};
let activeMode = 0;
let evtSrc = null;

/* ── SSE CONNECTION ─────────────────────────────────────────── */
function connect() {
  if(evtSrc) evtSrc.close();
  evtSrc = new EventSource('/events');
  evtSrc.addEventListener('data', e => {
    try { cur = JSON.parse(e.data); updateUI(); } catch(_){}
  });
  evtSrc.onopen  = () => setConn(true);
  evtSrc.onerror = () => {
    setConn(false);
    setTimeout(connect, 3000);
  };
}

function setConn(ok) {
  document.getElementById('connDot').className  = 'dot ' + (ok?'on':'off');
  document.getElementById('connText').textContent = ok ? 'Connected' : 'Reconnecting…';
}

/* ── CLOCK ──────────────────────────────────────────────────── */
setInterval(() => {
  document.getElementById('clockText').textContent =
    new Date().toLocaleTimeString();
}, 1000);

/* ── MODE SELECT ────────────────────────────────────────────── */
function setMode(m) {
  if(m === 'h') { openHistory(); return; }
  activeMode = m;
  document.querySelectorAll('.mbtn').forEach(b =>
    b.classList.toggle('active', b.dataset.m == m));
  document.querySelectorAll('.screen').forEach(s => s.classList.remove('active'));
  const sc = document.getElementById('screen-'+m);
  if(sc) sc.classList.add('active');
  fetch('/api/mode?m='+m);
}

function openHistory() {
  activeMode = 'h';
  document.querySelectorAll('.mbtn').forEach(b =>
    b.classList.toggle('active', b.dataset.m === 'h'));
  document.querySelectorAll('.screen').forEach(s => s.classList.remove('active'));
  document.getElementById('screen-h').classList.add('active');
  loadHistory();
}

/* ── GAUGE HELPERS ──────────────────────────────────────────── */
const ARC_LEN = 346;  // path length for the gauge arc (half-circle r=110)
function setArc(id, pct) {
  const el = document.getElementById(id);
  if(!el) return;
  el.style.strokeDashoffset = ARC_LEN * (1 - pct/100);
}
function setArc283(id, pct) {  // for hybrid mini-gauges r=90
  const el = document.getElementById(id);
  if(!el) return;
  el.style.strokeDashoffset = 283 * (1 - pct/100);
}
function setNeedle(id, pct) {
  const el = document.getElementById(id);
  if(!el) return;
  // Needle rotates from -90° (empty, left) to +90° (full, right)
  const angle = -90 + pct/100*180;
  const cx=140, cy=150, r=80;
  const rad = (angle-90)*Math.PI/180;
  el.setAttribute('x2', cx + r*Math.cos(rad));
  el.setAttribute('y2', cy + r*Math.sin(rad));
}
function setNeedle90(id, pct) {  // hybrid mini
  const el = document.getElementById(id);
  if(!el) return;
  const angle = -90 + pct/100*180;
  const cx=110, cy=120, r=60;
  const rad = (angle-90)*Math.PI/180;
  el.setAttribute('x2', cx + r*Math.cos(rad));
  el.setAttribute('y2', cy + r*Math.sin(rad));
}

/* ── BAT BARS ───────────────────────────────────────────────── */
function buildBatBars(id, pct, col) {
  const el = document.getElementById(id);
  if(!el) return;
  el.innerHTML = '';
  const n=10, filled=Math.round(pct/100*n);
  for(let i=0;i<n;i++) {
    const seg=document.createElement('div');
    seg.className='bat-seg';
    seg.style.background = i<filled ? col : '#373945';
    seg.style.height = '22px';
    el.appendChild(seg);
  }
}

/* ── CELL SEGMENTS (EV) ─────────────────────────────────────── */
function buildCells(id, pct, col) {
  const el=document.getElementById(id);
  if(!el) return;
  el.innerHTML='';
  const n=10, filled=Math.round(pct/10);
  for(let i=0;i<n;i++){
    const seg=document.createElement('div');
    seg.style.cssText=`flex:1;height:20px;border-radius:4px;`+
      `background:${i<filled?col:'#373945'};transition:background .3s`;
    el.appendChild(seg);
  }
}

/* ── COLOR HELPERS ──────────────────────────────────────────── */
function fuelCol(p){return p>30?'#27C96E':p>15?'#EAB40C':p>10?'#EA6C0A':'#DC2626'}
function battCol(p){return p>50?'#22D3EE':p>25?'#3B82F6':p>10?'#F59E2B':'#DC2626'}
function setText(id, txt, col){
  const el=document.getElementById(id);
  if(!el)return; el.textContent=txt; if(col) el.style.color=col;
}

/* ── LCD LINE BUILDER ───────────────────────────────────────── */
function lcdLines(fuel, batt, sw, mode) {
  if(mode===1){const mah=Math.round(batt/100*20000);
    return['POWER BANK      ',`Bat:${batt}%  ${mah}mAh  `.padEnd(16)]}
  if(mode===2){const l=(fuel/100*50).toFixed(1);
    return['CAR FUEL        ',`${fuel}%  ${l}L`.padEnd(16)]}
  if(mode===3){const s=sw?'ELEC':'FUEL';
    return[`HYBRID  [${s.padEnd(4)}]   `,
           `F:${String(fuel).padStart(3)}%  B:${String(batt).padStart(3)}%  `]}
  if(mode===4){const r=Math.round(batt/100*400);
    return['ELECTRIC CAR    ',
           `Bat:${String(batt).padStart(3)}% ${String(r).padStart(3)}km  `]}
  if(mode===5){const l=fuel/100*25;const h=l/2.5;
    const hh=Math.floor(h),mm=String(Math.round((h%1)*60)).padStart(2,'0');
    return['GENERATOR       ',`${fuel}%  ${l.toFixed(1)}L ${hh}h${mm}m`.padEnd(16)]}
  return['                ','                ']
}
function setLcd(id, fuel, batt, sw, mode){
  const el=document.getElementById(id);
  if(!el)return;
  const[l1,l2]=lcdLines(fuel,batt,sw,mode);
  el.textContent=l1.slice(0,16)+'\n'+l2.slice(0,16);
}

/* ── MAIN UI UPDATE ─────────────────────────────────────────── */
function updateUI() {
  const {fuel,batt,sw,mode,alert} = cur;
  const fc=fuelCol(fuel), bc=battCol(batt);

  // Alert
  document.getElementById('alertOverlay').classList.toggle('show', alert);
  document.getElementById('alertBox').classList.toggle('hidden', !alert);
  if(alert)
    document.getElementById('alertMsg').textContent=
      `Fuel at ${fuel}% — REFUEL IMMEDIATELY!`;

  // DB reading count in nav
  document.getElementById('dbText').textContent = `Logged: –`;

  if(activeMode===1) {
    // POWER BANK
    setArc('arcPB',batt); setNeedle('needlePB',batt);
    document.getElementById('pctPB').textContent=batt+'%';
    buildBatBars('batVertPB',batt,bc);
    const mah=(batt/100*20000).toFixed(0);
    const hrs=mah/500; const h=Math.floor(hrs),m=Math.round((hrs%1)*60);
    const st=batt>80?'CHARGED':batt>20?'NORMAL':'LOW!';
    setText('pb-pct',  batt+'%',   bc);
    setText('pb-status',st,        batt>20?'#27C96E':'#DC2626');
    setText('pb-mah',  mah+' mAh', '#DCE0E8');
    setText('pb-time', h+'h '+String(m).padStart(2,'0')+'m','#22D3EE');
    document.getElementById('barPB').style.width=batt+'%';
    setLcd('lcdPB',fuel,batt,sw,1);
  }
  else if(activeMode===2) {
    // CAR
    setArc('arcCAR',fuel); setNeedle('needleCAR',fuel);
    document.getElementById('pctCAR').textContent=fuel+'%';
    const lit=(fuel/100*50).toFixed(1);
    const rng=Math.round(fuel/100*600);
    const st=fuel>30?'GOOD':fuel>15?'LOW – REFUEL':'CRITICAL!';
    setText('car-pct',   fuel+'%',         fc);
    setText('car-status',st,               fuel>30?'#27C96E':'#DC2626');
    setText('car-lit',   lit+' / 50 L',    '#DCE0E8');
    setText('car-range', '~'+rng+' km',    fc);
    document.getElementById('barCAR').style.cssText=
      `width:${fuel}%;background:${fc}`;
    setLcd('lcdCAR',fuel,batt,sw,2);
  }
  else if(activeMode===3) {
    // HYBRID
    setArc283('arcHF',fuel); setNeedle90('needleHF',fuel);
    setArc283('arcHB',batt); setNeedle90('needleHB',batt);
    document.getElementById('pctHF').textContent=fuel+'%';
    document.getElementById('pctHB').textContent=batt+'%';
    const active=sw?'ELEC':'FUEL';
    document.getElementById('hybSwBadge').textContent=
      sw?'[ ELECTRIC MODE ]':'[ FUEL MODE ]';
    document.getElementById('hybSwBadge').style.borderColor=
      sw?'#22D3EE':'#EAB40C';
    document.getElementById('hybSwBadge').style.color=
      sw?'#22D3EE':'#EAB40C';
    const fpan=document.getElementById('hybFuelPanel');
    const bpan=document.getElementById('hybBattPanel');
    fpan.classList.toggle('active-panel',!sw);
    bpan.classList.toggle('active-panel', sw);
    fpan.style.opacity = sw?'0.4':'1';
    bpan.style.opacity = sw?'1':'0.4';
    setText('hybFuelSub', (fuel/100*40).toFixed(1)+' / 40 L');
    setText('hybFuelRange','Range: ~'+Math.round(fuel/100*500)+' km');
    setText('hybBattRange','EV Range: ~'+Math.round(batt/100*50)+' km');
    setLcd('lcdHYB',fuel,batt,sw,3);
  }
  else if(activeMode===4) {
    // EV
    setArc('arcEV',batt); setNeedle('needleEV',batt);
    document.getElementById('pctEV').textContent=batt+'%';
    buildBatBars('batVertEV',batt,bc);
    buildCells('cellsEV',batt,bc);
    const rng=Math.round(batt/100*400);
    const kwh=(batt/100*75).toFixed(1);
    const st=batt>80?'CHARGED':batt>50?'GOOD':batt>20?'LOW – CHARGE SOON':'CRITICAL!';
    setText('ev-pct',   batt+'%',  bc);
    setText('ev-status',st,        batt>20?'#27C96E':'#DC2626');
    setText('ev-range', '~'+rng+' km', bc);
    setText('ev-kwh',   kwh+' kWh','#22D3EE');
    setLcd('lcdEV',fuel,batt,sw,4);
  }
  else if(activeMode===5) {
    // GENERATOR
    setArc('arcGEN',fuel); setNeedle('needleGEN',fuel);
    document.getElementById('pctGEN').textContent=fuel+'%';
    const lit=(fuel/100*25).toFixed(1);
    const hrs=parseFloat(lit)/2.5;
    const h=Math.floor(hrs),m=String(Math.round((hrs%1)*60)).padStart(2,'0');
    const st=fuel>30?'RUNNING – GOOD':fuel>15?'LOW – REFUEL':'CRITICAL!';
    setText('gen-pct',   fuel+'%',       fc);
    setText('gen-status',st,             fuel>30?'#27C96E':'#DC2626');
    setText('gen-lit',   lit+' / 25 L',  '#DCE0E8');
    setText('gen-hrs',   h+'h '+m+'m',   fc);
    document.getElementById('barGEN').style.cssText=
      `width:${fuel}%;background:${fc}`;
    setLcd('lcdGEN',fuel,batt,sw,5);
  }
}

/* ── HISTORY ─────────────────────────────────────────────────── */
async function loadHistory() {
  const res = await fetch('/api/history?n=60');
  const data = await res.json();
  if(!Array.isArray(data)||data.length===0){
    document.getElementById('histTbody').innerHTML=
      '<tr><td colspan="5" style="text-align:center;color:#373945;padding:20px">'
      +'No data yet — start a mode and wait 30s</td></tr>';
    return;
  }

  // Stats
  const alerts=data.filter(r=>r.alert).length;
  const avg=(data.reduce((s,r)=>s+r.fuel,0)/data.length).toFixed(1);
  const min=Math.min(...data.map(r=>r.fuel)).toFixed(1);
  setText('h-total',  data.length, '#22D3EE');
  setText('h-alerts', alerts,      alerts?'#DC2626':'#27C96E');
  setText('h-avg',    avg+'%',     '#DCE0E8');
  setText('h-min',    min+'%',     fuelCol(parseFloat(min)));

  // Graph
  drawGraph(data);

  // Table (last 10, newest first)
  const tbody=document.getElementById('histTbody');
  tbody.innerHTML='';
  [...data].reverse().slice(0,10).forEach(r=>{
    const time=r.ts.length>=19?r.ts.substr(11,8):r.ts;
    const tr=document.createElement('tr');
    tr.innerHTML=
      `<td style="color:#828491">${time}</td>`+
      `<td style="color:#22D3EE">${r.mode}</td>`+
      `<td style="color:${fuelCol(r.fuel)}">${r.fuel}%</td>`+
      `<td style="color:${battCol(r.batt)}">${r.batt}%</td>`+
      `<td class="${r.alert?'alert-yes':'alert-no'}">${r.alert?'⚠ ALERT':'OK'}</td>`;
    tbody.appendChild(tr);
  });
}

function drawGraph(data) {
  const canvas=document.getElementById('graphCanvas');
  if(!canvas||data.length<2) return;
  const ctx=canvas.getContext('2d');
  const W=canvas.offsetWidth||500, H=200;
  canvas.width=W; canvas.height=H;

  const pad={l:36,r:24,t:14,b:28};
  const gw=W-pad.l-pad.r, gh=H-pad.t-pad.b;

  ctx.clearRect(0,0,W,H);

  // Zone fills
  ctx.fillStyle='rgba(220,38,38,.12)';
  ctx.fillRect(pad.l, pad.t+gh-gh*.1, gw, gh*.1);
  ctx.fillStyle='rgba(234,180,12,.06)';
  ctx.fillRect(pad.l, pad.t+gh-gh*.3, gw, gh*.2);

  // Grid
  for(let i=0;i<=4;i++){
    const y=pad.t+gh-gh*i*.25;
    ctx.strokeStyle='#282B37'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(pad.l,y); ctx.lineTo(pad.l+gw,y); ctx.stroke();
    ctx.fillStyle='#828491'; ctx.font='9px Segoe UI';
    ctx.fillText(i*25+'%', 2, y+4);
  }

  // 10% threshold dashed red
  const thresh=pad.t+gh-gh*.1;
  ctx.strokeStyle='#DC2626'; ctx.lineWidth=1.5;
  ctx.setLineDash([6,4]);
  ctx.beginPath(); ctx.moveTo(pad.l,thresh); ctx.lineTo(pad.l+gw,thresh); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle='#DC2626'; ctx.font='9px Segoe UI';
  ctx.fillText('10%', pad.l+gw+2, thresh+4);

  // Data line
  const n=data.length;
  for(let i=0;i<n-1;i++){
    const x1=pad.l+i/(n-1)*gw, x2=pad.l+(i+1)/(n-1)*gw;
    const y1=pad.t+gh-data[i].fuel/100*gh;
    const y2=pad.t+gh-data[i+1].fuel/100*gh;
    const avg=(data[i].fuel+data[i+1].fuel)/2;
    ctx.strokeStyle=fuelCol(avg); ctx.lineWidth=2.5;
    ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
  }

  // Markers
  for(let i=0;i<n;i++){
    const x=pad.l+i/(n-1)*gw;
    const y=pad.t+gh-data[i].fuel/100*gh;
    if(data[i].alert){
      ctx.fillStyle='#DC2626';
      ctx.beginPath();
      ctx.moveTo(x,y-8); ctx.lineTo(x+7,y);
      ctx.lineTo(x,y+8); ctx.lineTo(x-7,y);
      ctx.closePath(); ctx.fill();
    } else {
      ctx.fillStyle=fuelCol(data[i].fuel);
      ctx.beginPath(); ctx.arc(x,y,3.5,0,2*Math.PI); ctx.fill();
    }
  }

  // Time labels
  if(n>0){
    ctx.fillStyle='#828491'; ctx.font='9px Segoe UI';
    const t1=data[0].ts.length>=16?data[0].ts.substr(11,5):data[0].ts;
    const t2=data[n-1].ts.length>=16?data[n-1].ts.substr(11,5):data[n-1].ts;
    ctx.fillText(t1,pad.l,H-6);
    ctx.textAlign='right';
    ctx.fillText(t2,pad.l+gw,H-6);
    ctx.textAlign='left';
  }
}

async function clearLogs() {
  if(!confirm('Clear all logged data? This cannot be undone.')) return;
  await fetch('/api/clear');
  alert('Logs cleared.');
  loadHistory();
}

/* ── INIT ────────────────────────────────────────────────────── */
connect();
</script>
</body>
</html>
)RAWHTML";
