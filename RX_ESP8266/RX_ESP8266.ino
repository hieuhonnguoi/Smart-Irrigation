/*********** BLYNK CLOUD ***********/
#define BLYNK_TEMPLATE_ID "TMPL6A0CvYsN9"
#define BLYNK_TEMPLATE_NAME "Smart Irrigation System"
#define BLYNK_AUTH_TOKEN "0kH8EWOf3OQkHZvtonbWYZlK6RFuw_ab"

/*********** LIBS ***********/
#include <WiFi.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager
#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

/*********** LCD ***********/
LiquidCrystal_I2C lcd(0x27, 16, 2);

/*********** LoRa pins & params ***********/
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_FREQ 433E6

/*********** Button to open WiFi portal ***********/
#define BTN_PORTAL   13       // chọn 0 (BOOT) hoặc pin khác kéo lên với INPUT_PULLUP
#define LONG_PRESS_MS 1500

/*********** LoRa state & data ***********/
volatile float temp = 0, humi = 0, soil = 0,waterValue = 0;
volatile int   auto_mode = 0, RELAY1 = 0, RELAY2 = 0;

unsigned long lastReceiveTime = 0;
const unsigned long LORA_TIMEOUT = 15000;
volatile bool loraConnected = false;

/*********** Screen cache - chống nhấp nháy ***********/
String lastLine1 = "";
String lastLine2 = "";

/*********** RTOS handles ***********/
TaskHandle_t TaskLoRaRx_handle   = nullptr;
TaskHandle_t TaskLCD_handle      = nullptr;
TaskHandle_t TaskBlynk_handle    = nullptr;
TaskHandle_t TaskButton_handle   = nullptr;

/*********** Mutex bảo vệ LoRa ***********/
SemaphoreHandle_t xLoRaMutex;

/*********** WiFiManager ***********/
WiFiManager wm;

/*********** Blynk Timer (đẩy dữ liệu định kỳ) ***********/
BlynkTimer timer;

/*------------------- Tiện ích LCD -------------------*/
void lcdPrintNoFlicker(uint8_t col, uint8_t row, const String &text, String &last) {
  if (text != last) {
    lcd.setCursor(col, row);
    String pad = text;
    while (pad.length() < 16) pad += ' ';
    lcd.print(pad);
    last = text;
  }
}

/*------------------- Gửi JSON qua LoRa (có mutex + timeout) -------------------*/
bool loraSendJson(const String &json, uint32_t beginTimeoutMs = 120, bool async = true) {
  if (xSemaphoreTake(xLoRaMutex, pdMS_TO_TICKS(80)) != pdTRUE) {
    return false; // đang bận
  }

  unsigned long t0 = millis();
  while (LoRa.beginPacket() == 0) { // radio đang bận (đang TX)
    if (millis() - t0 > beginTimeoutMs) {
      xSemaphoreGive(xLoRaMutex);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  LoRa.print(json);
  LoRa.endPacket(async); // async=true: non-blocking
  xSemaphoreGive(xLoRaMutex);
  return true;
}

/*------------------- Gửi lệnh do Blynk thay đổi -------------------*/
void sendCommand() {
  StaticJsonDocument<128> doc;
  doc["auto_mode"] = auto_mode;
  doc["MAYBOM"]    = RELAY1;
  doc["CUACHAN"]   = RELAY2;
  for (int i=0; i<3;i++){
    String json; serializeJson(doc, json);
    bool ok = loraSendJson(json);
    Serial.print("[LoRa TX cmd] ");
    Serial.print(json);
    Serial.println(ok ? "  -> OK" : "  -> BUSY/FAIL");
  }
}

/*********** Blynk handlers ***********/
BLYNK_WRITE(V3) { auto_mode = param.asInt(); sendCommand(); }
BLYNK_WRITE(V4) { RELAY1    = param.asInt(); sendCommand(); }
BLYNK_WRITE(V5) { RELAY2    = param.asInt(); sendCommand(); }

/*********** Đẩy data lên Blynk định kỳ ***********/
void pushTelemetry() {
 Serial.printf("[pushTelemetry] T=%.1f H=%.1f S=%.1f A=%d R1=%d R2=%d W=%d (connected=%d)\n",
                temp, humi, soil, auto_mode, RELAY1, RELAY2,waterValue, Blynk.connected());

  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, int(temp));
    Blynk.virtualWrite(V1, humi);
    Blynk.virtualWrite(V2, soil);
    Blynk.virtualWrite(V3, auto_mode);
    Blynk.virtualWrite(V4, RELAY1);
    Blynk.virtualWrite(V5, RELAY2);
    Blynk.virtualWrite(V6, waterValue);
  }
}

/*------------------- Task: Nhận LoRa -------------------*/
void TaskLoRaRx(void *pv) {
  for (;;) {
    if (xSemaphoreTake(xLoRaMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        String json = "";
        while (LoRa.available()) {
          json += (char)LoRa.read();
          // giới hạn độ dài nếu cần
          if (json.length() > 240) break;
        }
        xSemaphoreGive(xLoRaMutex);

        StaticJsonDocument<256> doc;
        auto err = deserializeJson(doc, json);
        if (!err) {
          Serial.print("[LoRa RX] Raw: ");
          Serial.println(json);  // debug nguyên gói nhận được
          // đọc an toàn (biến volatile)
          temp      = doc["temp"]     | temp;
          humi      = doc["humi"]     | humi;
          soil      = doc["soil"]     | soil;
          auto_mode = doc["auto_mode"]| auto_mode;
          RELAY1    = doc["MAYBOM"]   | RELAY1;
          RELAY2    = doc["CUACHAN"]  | RELAY2;
          waterValue = doc["waterValue"] | waterValue;

          lastReceiveTime = millis();
          loraConnected = true;
          // đẩy lên Blynk nhẹ nhàng (đã có timer mỗi 1s)
        } else {
          Serial.print("[LoRa RX] JSON err: "); Serial.println(err.c_str());
        }
      } else {
        xSemaphoreGive(xLoRaMutex);
      }
    }

    // watchdog mất kết nối
    if (millis() - lastReceiveTime > LORA_TIMEOUT) {
      loraConnected = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/*------------------- Task: LCD -------------------*/
void TaskLCD(void *pv) {
  for (;;) {
    String l1, l2;
    if (!loraConnected) {
      l1 = "LOST LORA";
      l2 = "Waiting data...";
    } else {
      l1 = "T:" + String(temp,1) + (char)223 + "C H:" + String(humi,1) + "%";
      l2 = "S:" + String(soil,1) + "% A:" + String(auto_mode) + " P:" + String(RELAY1) + String(RELAY2);
    }
    lcdPrintNoFlicker(0, 0, l1, lastLine1);
    lcdPrintNoFlicker(0, 1, l2, lastLine2);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

/*------------------- Task: Blynk.run() -------------------*/
void TaskBlynk(void *pv) {
  for (;;) {
    Blynk.run();
    timer.run();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/*------------------- Task: Button mở lại portal -------------------*/
void TaskButton(void *pv) {
  uint32_t pressStart = 0;
  bool pressed = false;

  for (;;) {
    bool level = (digitalRead(BTN_PORTAL) == LOW);
    if (level && !pressed) {
      pressed = true;
      pressStart = millis();
    } else if (!level && pressed) {
      pressed = false;
    }

    // giữ > LONG_PRESS_MS => mở portal (blocking trong task này)
    if (pressed && (millis() - pressStart > LONG_PRESS_MS)) {
      pressed = false;
      Serial.println("[BTN] Open WiFi Portal");
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("AP MODE");
      lcd.setCursor(0, 1); lcd.print("ESP32_Config");

      wm.setConfigPortalTimeout(180);
      bool ok = wm.startConfigPortal("ESP32_Config", "12345678");
      if (ok) {
        Serial.println("[WiFi] Reconfigured OK");
        lcd.clear(); lcd.setCursor(0,0); lcd.print("WiFi OK");
      } else {
        Serial.println("[WiFi] Portal timeout/exit");
        lcd.clear(); lcd.setCursor(0,0); lcd.print("WiFi FAIL");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/*------------------- SETUP -------------------*/
void setup() {
  Serial.begin(115200);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("WiFi start...");

  // Button
  pinMode(BTN_PORTAL, INPUT_PULLUP);

  // WiFiManager: cố gắng kết nối WiFi cũ; nếu chưa có → mở portal 180s
  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect("ESP32_Config", "12345678")) {
    Serial.println("[WiFi] No connect -> continue offline");
    lcd.setCursor(0, 1); lcd.print("WiFi FAIL");
  } else {
    Serial.println("[WiFi] Connected");
    lcd.setCursor(0, 1); lcd.print("WiFi OK        ");
  }

  // Blynk: dùng config (không tự quản WiFi)
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(3000); // thử 3s
  timer.setInterval(5000, pushTelemetry);

  // LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("LoRa FAILED");
    Serial.println("[LoRa] init failed. Keep running.");
  } else {
    Serial.println("[LoRa] OK");
  }

  // Mutex
  xLoRaMutex = xSemaphoreCreateMutex();

  // Tasks
  xTaskCreatePinnedToCore(TaskLoRaRx, "TaskLoRaRx", 4096, nullptr, 2, &TaskLoRaRx_handle, 1);
  xTaskCreatePinnedToCore(TaskLCD,    "TaskLCD",    3072, nullptr, 1, &TaskLCD_handle,    1);
  xTaskCreatePinnedToCore(TaskBlynk,  "TaskBlynk",  4096, nullptr, 2, &TaskBlynk_handle,  1);
  xTaskCreatePinnedToCore(TaskButton, "TaskButton", 3072, nullptr, 1, &TaskButton_handle, 1);
}

void loop() {
  // không dùng loop — mọi thứ chạy trong task
}
