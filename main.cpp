#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <U8g2lib.h>

/* ---------------- UART (RX only) ---------------- */
HardwareSerial DeviceSerial(1);
constexpr int UART_RX_PIN = 13;   // 只接收
constexpr int UART_TX_PIN = -1;  // 不使用 TX

/* ---------------- I2C (NFP1315 OLED) ---------------- */
constexpr int I2C_SDA = 16;
constexpr int I2C_SCL = 17;

// 多數 NFP1315/SSD1315/SSD1306 128x64 I2C 可用此設定
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

/* ---------------- GPIO ---------------- */
constexpr int PIN_CEILING   = 23; // 0/1
constexpr int PIN_WORKBENCH = 19; // PWM 0~100%

constexpr int RGB_R = 25; // PWM
constexpr int RGB_G = 26; // PWM
constexpr int RGB_B = 27; // PWM

/* ---------------- PWM ---------------- */
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RES  = 8;    // 0~255

constexpr int CH_WORKBENCH = 0;
constexpr int CH_R = 1;
constexpr int CH_G = 2;
constexpr int CH_B = 3;

/* ---------------- State ---------------- */
int blindsVal = 0;     // 0~100
int workbenchVal = 0;  // 0~100
int ceilingVal = 0;    // 0/1
int climateVal = 20;   // 16~35

/* ---------------- Utils ---------------- */
static inline int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline uint8_t percentToPWM(int v) {
  v = clampInt(v, 0, 100);
  return (uint8_t)map(v, 0, 100, 0, 255);
}

static inline uint8_t lerpU8(uint8_t a, uint8_t b, float t) {
  return (uint8_t)roundf(a + (b - a) * t);
}

static inline float norm(int x, int x0, int x1) {
  if (x1 == x0) return 0.0f;
  float t = (float)(x - x0) / (float)(x1 - x0);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t;
}

static bool parseJsonInt(JsonVariantConst v, int* out) {
  if (v.is<int>()) {
    *out = v.as<int>();
    return true;
  }
  if (v.is<long>()) {
    *out = (int)v.as<long>();
    return true;
  }
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    if (s == nullptr || s[0] == '\0') return false;
    char* end = nullptr;
    long val = strtol(s, &end, 10);
    if (end != nullptr && *end == '\0') {
      *out = (int)val;
      return true;
    }
  }
  return false;
}

/* ---------------- blinds -> OLED roll-down ----------------
   val=0   -> 全黑
   val=100 -> 全亮
   由上到下亮起：亮區高度 = val% * 64
*/
void applyBlindsToOLED(int valPercent) {
  blindsVal = clampInt(valPercent, 0, 100);

  const int W = 128;
  const int H = 64;
  int litH = map(blindsVal, 0, 100, 0, H);

  oled.clearBuffer();
  if (litH > 0) {
    oled.drawBox(0, 0, W, litH);     // 上方亮起
  }
  oled.sendBuffer();
}

/* ---------------- workbench -> PWM brightness ---------------- */
void applyWorkbench(int valPercent) {
  workbenchVal = clampInt(valPercent, 0, 100);
  ledcWrite(CH_WORKBENCH, percentToPWM(workbenchVal));
}

/* ---------------- ceiling -> digital ---------------- */
void applyCeiling(int v01) {
  ceilingVal = (v01 != 0) ? 1 : 0;
  digitalWrite(PIN_CEILING, ceilingVal ? HIGH : LOW);
}

/* ---------------- climate -> RGB LED gradient ----------------
   16 -> Blue (0,0,255)
   20 -> White(255,255,255)
   35 -> Orange-Red(255,69,0)
   其他溫度分段漸變：
     16..20: Blue -> White
     20..35: White -> Orange
*/
void applyClimateToRGB(int tempC) {
  climateVal = clampInt(tempC, 16, 35);

  uint8_t r, g, b;

  if (climateVal <= 20) {
    // Blue -> White
    float t = norm(climateVal, 16, 20);
    r = lerpU8(0,   255, t);
    g = lerpU8(0,   255, t);
    b = lerpU8(255, 255, t);
  } else {
    // White -> Orange-Red (255,69,0)
    float t = norm(climateVal, 20, 35);
    r = lerpU8(255, 255, t);
    g = lerpU8(255, 69, t);
    b = lerpU8(255, 0,   t);
  }

  ledcWrite(CH_R, r);
  ledcWrite(CH_G, g);
  ledcWrite(CH_B, b);
}

/* ---------------- Command handler ---------------- */
void handleCommand(int id, const char* dev, int val) {
  // 不回覆、不 ACK。僅套用合法指令；非法則直接忽略。

  if (strcmp(dev, "blinds") == 0) {
    if (val < 0 || val > 100) return;
    applyBlindsToOLED(val);
    return;
  }

  if (strcmp(dev, "workbench") == 0) {
    if (val < 0 || val > 100) return;
    applyWorkbench(val);
    return;
  }

  if (strcmp(dev, "ceiling") == 0) {
    if (val != 0 && val != 1) return;
    applyCeiling(val);
    return;
  }

  if (strcmp(dev, "climate") == 0) {
    if (val < 16 || val > 35) return;
    applyClimateToRGB(val);
    return;
  }
}

/* ---------------- Setup ---------------- */
void setup() {
  Serial.begin(115200);

  // UART RX only
  DeviceSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // GPIO
  pinMode(PIN_CEILING, OUTPUT);
  digitalWrite(PIN_CEILING, LOW);

  // PWM channels
  ledcSetup(CH_WORKBENCH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_WORKBENCH, CH_WORKBENCH);
  ledcWrite(CH_WORKBENCH, 0);

  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(CH_G, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(RGB_R, CH_R);
  ledcAttachPin(RGB_G, CH_G);
  ledcAttachPin(RGB_B, CH_B);
  ledcWrite(CH_R, 0);
  ledcWrite(CH_G, 0);
  ledcWrite(CH_B, 0);

  // I2C + OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  oled.begin();
  oled.setPowerSave(0);

  // 初始狀態
  applyBlindsToOLED(blindsVal);
  applyWorkbench(workbenchVal);
  applyCeiling(ceilingVal);
  applyClimateToRGB(climateVal);
}

/* ---------------- Loop: line-based JSON RX ---------------- */
void loop() {
  static String line;

  while (DeviceSerial.available()) {
    char c = (char)DeviceSerial.read();

    if (c == '\n') {
      line.trim();           // 去空白
      if (line.length() == 0) {
        line = "";
        continue;            // 空行忽略、不回覆
      }

      Serial.println(line);

      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, line);
      line = "";

      if (err) {
        // 解析失敗且無法取得 id -> 不回覆；此版本也不做任何事
        continue;
      }

      Serial.print(F("[RX JSON] "));
      serializeJson(doc, Serial);
      Serial.println();

      // 必須有 id/dev/val
      if (!doc.containsKey("id") || !doc.containsKey("dev") || !doc.containsKey("val")) {
        continue;
      }

      int id = 0;
      int val = 0;
      if (!parseJsonInt(doc["id"], &id)) continue;
      if (!parseJsonInt(doc["val"], &val)) continue;
      String devStr = doc["dev"].as<String>();
      devStr.trim();
      devStr.toLowerCase();
      if (devStr.length() == 0) continue;

      handleCommand(id, devStr.c_str(), val);
      continue;
    }

    if (c != '\r') line += c; // 接受 \r\n
  }
}
