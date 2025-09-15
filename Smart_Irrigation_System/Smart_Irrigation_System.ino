
// Import required libraries
#include <SimpleKalmanFilter.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <icon.h>

// ------------------ KHAI BÁO MENU CÂY ------------------
String crops[] = {"Rau mam", "Bap Cai", "Ca chua", "Xa Lach", "Dua chuot", "Sen da", "Rau mui", "Tu chon"};
const int numCrops = sizeof(crops) / sizeof(crops[0]);

volatile int select_option = 0;  //cây đang chọn
volatile int menu_option = 0;
int page_start = 0;          // mục đầu tiên của trang
const int itemsPerPage = 4;  // OLED hiển thị tối đa 4 dòng

volatile int temp_min_cachua = 21, temp_max_cachua = 27, 
             humi_min_cachua = 50, humi_max_cachua = 70,
             soil_min_cachua = 60, soil_max_cachua = 80;

volatile int temp_min_bapcai = 15, temp_max_bapcai = 24, 
             humi_min_bapcai = 60, humi_max_bapcai = 70,
             soil_min_bapcai = 60, soil_max_bapcai = 80;

volatile int temp_min_raumam = 15, temp_max_raumam = 25, 
             humi_min_raumam = 50, humi_max_raumam = 80,
             soil_min_raumam = 60, soil_max_raumam = 80;

volatile int temp_min_xalach = 7, temp_max_xalach = 24, 
             humi_min_xalach = 60, humi_max_xalach = 70,
             soil_min_xalach = 60, soil_max_xalach = 80;

volatile int temp_min_duachuot = 24, temp_max_duachuot = 29, 
             humi_min_duachuot = 50, humi_max_duachuot = 70,
             soil_min_duachuot = 60, soil_max_duachuot = 80;

volatile int temp_min_senda = 15, temp_max_senda = 27, 
             humi_min_senda = 35, humi_max_senda = 50,
             soil_min_senda = 30, soil_max_senda = 40;

volatile int temp_min_raumui = 15, temp_max_raumui = 21, 
             humi_min_raumui = 50, humi_max_raumui = 70,
             soil_min_raumui = 60, soil_max_raumui = 80;

//----------------------Khai báo các chân SX1270----------------
#include <SPI.h>
#include <LoRa.h>

#define SCK   27     
#define MOSI  14
#define MISO  12
#define CS    5
#define RST   16
#define DIO0  4

SPIClass SPI2(HSPI);
//----------------------Một số Macro----------------------------
#define ENABLE    1
#define DISABLE   0

//------------------------Khai báo button------------------------
#include "mybutton.h"
#define BUTTON_UP_PIN 35
#define BUTTON_DOWN_PIN 34
#define BUTTON_SELECT_PIN 33
#define BUTTON_MENU_PIN 32
#define BUTTON_MAY_BOM 17
#define BUTTON_CUA_CHAN 23

#define BUTTON1_ID 1
#define BUTTON2_ID 2
#define BUTTON3_ID 3
#define BUTTON4_ID 4
#define BUTTON5_ID 5
#define BUTTON6_ID 6

Button buttonUP;
Button buttonDOWN;
Button buttonSELECT;
Button buttonMENU;
Button buttonMAYBOM;
Button buttonCUACHAN;

void button_press_short_callback(uint8_t button_id);
void button_press_long_callback(uint8_t button_id);




//----------------------Khai báo cho OLED 1.3--------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define i2c_Address 0x3C //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G oled = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define OLED_SDA      21
#define OLED_SCL      22

typedef enum {
  SCREEN0,
  SCREEN1,
  SCREEN2,
  SCREEN3,
  SCREEN4,
  SCREEN5,
  SCREEN6,
  SCREEN7
}SCREEN;
int screenOLED = SCREEN0;

bool enableShow = DISABLE;

//---------------------Cảm biến DHT11------------------------
#include "DHT.h"
#define DHT11_PIN         26
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);
float tempValue = 30;
float humiValue   = 60;
SimpleKalmanFilter tempfilter(2, 2, 0.001);
SimpleKalmanFilter humifilter(2, 2, 0.001);
bool dht11ReadOK = true;

// ---------------------Cảm biến đo độ ẩm đất--------------------
#define SOIL_MOISTURE 39
float soilMoistureValue = 0;

// --------------- Khai báo cảm biến khoảng cách srf04 -----------
#include <HCSR04.h>
#define SRF04_TRIG    19
#define SRF04_ECHO    18
HCSR04 ultrasonicSensor(SRF04_TRIG, SRF04_ECHO, 20, 400);
SimpleKalmanFilter srf04filter(2, 2, 0.1);
int srf04Value = 0;     // biến đo khoảng cách hiện tại
int waterValue = 0;     // biến chiều cao mực nước,   waterValue = EheightInstallSensor - srf04Value;
int EheightInstallSensorTemp = 0;      // biến tạm khoảng cách từ cảm biến đến mặt đất
int valueThresholdTemp = 0;           // biến tạm ngưỡng
bool waterWarning = DISABLE;
int  EheightInstallSensor = 30;          // chiều cao lắp cảm biến
int  EthresholdWarning = 1;             // ngưỡng cảnh báo

//----------------------Khai báo cho Servo-----------------------
#include <ESP32Servo.h>
Servo myservo;
const int servoPin = 13;   // chân servo
bool servoState = false;   // lưu trạng thái servo
int lastButtonState = HIGH;
int currentPos = 0;

//-------------------------Khai báo ngưỡng-----------------------
int  EtempThreshold1 = 20;             // ngưỡng nhiệt độ 1   
int  EtempThreshold2 = 40;             // ngưỡng nhiệt độ 2 
int  EhumiThreshold1 = 40;             // ngưỡng độ ẩm 1
int  EhumiThreshold2 = 60;             // ngưỡng độ ẩm 2
int  EsoilMoistureThreshold1 = 60;             // ngưỡng bụi 1
int  EsoilMoistureThreshold2 = 80;             // ngưỡng bụi 2

//----------------------Một số cảm biến khác----------------------
//Khai báo loa
#define BUZZER 2 
//Khai báo Rờ le
#define RELAY1 25
#define RELAY2 13

bool auto_mode = DISABLE;
int RELAY1_STATE;
int RELAY2_STATE;

//----------------------Khai báo TaskHandle-----------------------
TaskHandle_t TaskOLEDDisplay_handle = NULL;
TaskHandle_t TaskDHT11_handle = NULL;
TaskHandle_t TaskSoilMoistureSensor_handle = NULL;
TaskHandle_t TaskButton_handle = NULL;
TaskHandle_t TaskSendData_handle = NULL;

//-----------------------------Set up-----------------------------
void setup(){
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  SPI2.begin(SCK, MISO, MOSI, CS);
  LoRa.setSPI(SPI2);
  LoRa.setPins(CS, RST, DIO0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //868E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
  // Đọc data setup từ eeprom
  EEPROM.begin(512);
  readEEPROM();
  //Khởi tạo loa
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, DISABLE);
  //Khởi tạo Relay
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  //Khởi tạo OLED
  oled.begin(i2c_Address, true);
  oled.setTextSize(2);
  oled.setTextColor(SH110X_WHITE);
  //Khởi tạo DHT11
  dht.begin();
  //Khởi tạo servo
  myservo.attach(servoPin);
  myservo.write(0);  // ban đầu ở 0°
  // Khởi tạo SRF04
  ultrasonicSensor.begin();
  //Khởi tạo nút nhấn
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MENU_PIN, INPUT_PULLUP);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MAY_BOM, INPUT_PULLUP);
  pinMode(BUTTON_CUA_CHAN, INPUT_PULLUP);
  button_init(&buttonUP, BUTTON_UP_PIN, BUTTON1_ID);
  button_init(&buttonDOWN,   BUTTON_DOWN_PIN,   BUTTON2_ID);
  button_init(&buttonMENU, BUTTON_SELECT_PIN, BUTTON3_ID);
  button_init(&buttonSELECT, BUTTON_MENU_PIN, BUTTON4_ID);
  button_init(&buttonMAYBOM, BUTTON_MAY_BOM, BUTTON5_ID);
  button_init(&buttonCUACHAN, BUTTON_CUA_CHAN, BUTTON6_ID);
  button_pressshort_set_callback(button_press_short_callback);
  button_presslong_set_callback(button_press_long_callback);
  
  xTaskCreatePinnedToCore(TaskSRF04Sensor,     "TaskSRF04Sensor",      1024*10 ,  NULL,  10  ,  NULL  , 1 );
  xTaskCreatePinnedToCore(TaskOLEDDisplay,      "TaskOLEDDisplay",    1024*16,  NULL,  20,  &TaskOLEDDisplay_handle  , 1);
  xTaskCreatePinnedToCore(TaskDHT11,            "TaskDHT11",          1024*10,  NULL,  20,  &TaskDHT11_handle        , 1);
  xTaskCreatePinnedToCore(TaskSoilMoistureSensor,  "TaskSoilMoistureSensor",  1024*10,  NULL,  10, &TaskSoilMoistureSensor_handle,  1);
  xTaskCreatePinnedToCore(TaskButton,          "TaskButton" ,          1024*10 ,  NULL,  20 ,  &TaskButton_handle       , 1);
}

void loop(){
  vTaskDelete(NULL);
}
//---------------------Task đo nhiệt độ độ ẩm------------------------
void TaskDHT11(void *pvParameters) {
  while(1) {
    float humi = dht.readHumidity() ;
    float temp = dht.readTemperature();
    if(isnan(humi) || isnan(temp)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      delay(200);
      dht11ReadOK = false;
    }
    else if (humi <= 100 && temp < 100) {
      dht11ReadOK = true;
    }
    humiValue = humi;
    tempValue = temp;
    if (temp > 35){
      if (auto_mode == ENABLE) {
          servoState = !servoState; // đảo trạng thái
          if (servoState) {
          myservo.write(90);   // quay 90°
          Serial.println("Servo -> 90°");
          digitalWrite(RELAY2,1);
          } else {
          myservo.write(0);    // quay về 0°
          Serial.println("Servo -> 0°");
          digitalWrite(RELAY2,0);
        }
      }
    }
    
    JsonSendData();

 
  }
  
}
int tempMAYBOM = 0;
//--------------------------Task đô độ ẩm đất----------------------------
void TaskSoilMoistureSensor(void *pvParameters) {
  while(1) {
    soilMoistureValue = analogRead(SOIL_MOISTURE);
    soilMoistureValue = 4095 - soilMoistureValue;
    soilMoistureValue = map(soilMoistureValue, 0, 4095, 0, 100);
    if (auto_mode == ENABLE) {
      if(soilMoistureValue < EsoilMoistureThreshold1 - 10){
          delay(2000);
          digitalWrite(RELAY1, ENABLE);
          RELAY1_STATE = digitalRead(RELAY1);
          if (RELAY1_STATE != tempMAYBOM){
            sendLoraMAYBOM();
            tempMAYBOM = RELAY1_STATE;
          }
      }
      else if(soilMoistureValue > EsoilMoistureThreshold1 + 10){
        delay(2000);
        digitalWrite(RELAY1, DISABLE);
        RELAY1_STATE = digitalRead(RELAY1);
          if (RELAY1_STATE != tempMAYBOM){
            sendLoraMAYBOM();
            tempMAYBOM = RELAY1_STATE;
          }
      }
    }
   receiveLoRaCommand();
   delay(300);
  }
}

//-------------------- Task đọc cảm biến srf04 ---------------
bool outRange = 0;
void TaskSRF04Sensor(void *pvParameters) {
  while (1) {
    // Đo khoảng cách đã qua median filter
    int distanceMeasure = ultrasonicSensor.getMedianFilterDistance();
    distanceMeasure = srf04filter.updateEstimate(distanceMeasure);

    // Chỉ nhận giá trị hợp lệ trong khoảng 20cm – 400cm
    if ( distanceMeasure < 400) {
      outRange = 0;

      Serial.print("distanceMeasure: ");
      Serial.print(distanceMeasure);
      Serial.println(F(" cm"));

      // Tính mực nước
      srf04Value = distanceMeasure;
      waterValue = EheightInstallSensor - srf04Value;
      if (waterValue < 0) waterValue = 0;

      Serial.print("waterValue: ");
      Serial.print(waterValue);
      Serial.println(F(" cm"));

      // Kiểm tra cảnh báo
      if (waterValue < EthresholdWarning) {
        waterWarning = ENABLE;   // mực nước thấp hơn ngưỡng → bật cảnh báo
      } else {
        waterWarning = DISABLE;  // an toàn
      }
    }
    else {
      // Ngoài vùng đo
      outRange = 1;
      Serial.println(F("SRF04 out of range"));
      waterWarning = ENABLE;          // coi như lỗi, bật cảnh báo
      waterValue = EheightInstallSensor; // giữ mức đầy để tránh báo sai
    }

    delay(200); // hoặc vTaskDelay(pdMS_TO_TICKS(200)) nếu dùng FreeRTOS
  }
}


void drawWaterLevel() {
  // Vẽ khung bể
  oled.drawRect(115, 1, 10, 61, SH110X_WHITE);

  int waterLevel = waterValue;

  // Map giá trị cảm biến thành chiều cao cột nước (0 -> 57 pixel)
  int level = map(waterLevel, 0, EheightInstallSensor, 0, 57);

  // Xóa vùng cũ trước khi vẽ
  oled.fillRect(117, 3, 6, 57, SH110X_BLACK);

  // Vẽ mức nước
  oled.fillRect(117, 60 - level, 6, level, SH110X_WHITE);

  // Nếu thấp hơn ngưỡng cảnh báo → vẽ cảnh báo
  if (waterLevel < EthresholdWarning) {
    // Ví dụ: vẽ đường ngang màu trắng ở mức cảnh báo
    int warnLevel = map(EthresholdWarning, 0, EheightInstallSensor, 0, 57);
    oled.drawLine(115, 60 - warnLevel, 125, 60 - warnLevel, SH110X_WHITE);

    // Bạn cũng có thể in chữ "LOW" hoặc chớp cảnh báo
    oled.setCursor(95, 47);
    oled.print("LOW!");
  }
}

// Hàm quay chậm đến góc mong muốn
void moveServoSlow(int targetAngle) {
  if (targetAngle > currentPos) {
    for (int pos = currentPos; pos <= targetAngle; pos++) {
      myservo.write(pos);
      delay(15); // càng lớn thì càng chậm
    }
  } else {
    for (int pos = currentPos; pos >= targetAngle; pos--) {
      myservo.write(pos);
      delay(15);
    }
  }
  currentPos = targetAngle;
}


//----------------------------Hàm truyền dữ liệu-----------------------------
void JsonSendData() {
    String json = "{\"temp\":" + String(tempValue, 1) +
                ",\"humi\":" + String(humiValue, 1) +
                ",\"soil\":" + String(soilMoistureValue, 1) + 
                ",\"MAYBOM\":" + String(digitalRead(RELAY1)) +
                 ",\"CUACHAN\":" + String(digitalRead(RELAY2)) +
                 ",\"waterValue\":"+ String(float(waterValue)) + "}";
    Serial.println(json);

    LoRa.beginPacket();
    LoRa.print(json);
    LoRa.endPacket(true);
    delay(10000);
}

void receiveLoRaCommand() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String json = "";
    while (LoRa.available()) {
      json += (char)LoRa.read();
    }
    Serial.println("Nhận lệnh: " + json);

    StaticJsonDocument<200> doc;
    if (!deserializeJson(doc, json)) {
      if (doc.containsKey("auto_mode")) {
        auto_mode = doc["auto_mode"];
      }
      if (doc.containsKey("MAYBOM")) {
        RELAY1_STATE = doc["MAYBOM"];
        digitalWrite(RELAY1, RELAY1_STATE);
      }
      if (doc.containsKey("CUACHAN")) {
        RELAY2_STATE = doc["CUACHAN"];
        digitalWrite(RELAY2, RELAY2_STATE);
      }
    }
  }
}
//---------------------------Hàm gửi tín hiệu lora khi có nút nhấn-----------------------
void sendLoraMAYBOM(){
  RELAY1_STATE = digitalRead(RELAY1);
  String json = "{\"MAYBOM\":" + String(RELAY1_STATE) + "}";
  for(int i=0; i<5; i++){
    Serial.println(json);
    LoRa.beginPacket();
    LoRa.print(json);
    LoRa.endPacket(true);
    delay(50);
  }
}

void sendLoraCUACHAN(){
  RELAY2_STATE = servoState;
  String json = "{\"CUACHAN\":" + String(RELAY2_STATE) + "}";
  for(int i=0; i<5; i++){
    Serial.println(json);
    LoRa.beginPacket();
    LoRa.print(json);
    LoRa.endPacket(true);
    delay(50);
  }
}

void sendLoraAUTO(){
  String json = "{\"auto_mode\":" + String(auto_mode) + "}";
  for(int i=0; i<5; i++){
    Serial.println(json);
    LoRa.beginPacket();
    LoRa.print(json);
    LoRa.endPacket(true);
    delay(50);
  }
}

void clearOLED() {
  oled.clearDisplay();
  oled.display();
}

void clearRectangle(int x1, int y1, int x2, int y2) {
   for(int i = y1; i < y2; i++) {
     oled.drawLine(x1, i, x2, i, 0);
   }
}
// ==== VẼ ICON NHIỆT KẾ ====
void drawThermometer(int x, int y, float tempValue) {
  oled.drawRect(x, y, 6, 20, SH110X_WHITE);   // thân
  oled.fillCircle(x+3, y+20, 6, SH110X_WHITE); // bóng tròn
  oled.fillCircle(x+3, y+20, 4, SH110X_BLACK); // khoét trong
  oled.fillRect(x+1, y+1, 4, 18, SH110X_BLACK); // rỗng thân

  // mực thủy ngân (0–50°C)
  int level = map((int)tempValue, 0, 50, 0, 18);
  oled.fillRect(x+1, y+19-level, 4, level, SH110X_WHITE);
  oled.fillCircle(x+3, y+20, 4, SH110X_WHITE);
}

// ==== VẼ ICON GIỌT NƯỚC ====
void drawWaterDrop(int x, int y, int humiValue) {
  oled.fillCircle(x, y, 7, SH110X_WHITE);
  oled.fillTriangle(x-6, y, x+6, y, x, y-12, SH110X_WHITE);
  oled.fillCircle(x, y, 5, SH110X_BLACK);

  // mức đầy (0–100%)
  int fill = map(humiValue, 0, 100, 0, 10);
  for(int i=0;i<fill;i++){
    oled.drawLine(x-4, y+3-i, x+4, y+3-i, SH110X_WHITE);
  }
}

// ==== VẼ ICON ĐẤT ====
void drawSoilMoisture(int x, int y, int soilValue) {
  oled.fillRect(x, y, 30, 8, SH110X_WHITE); // nền đất
  int dots = map(soilValue, 0, 100, 0, 5);  // số chấm theo % ẩm
  for (int i=0;i<dots;i++){
    oled.fillCircle(x+6+i*5, y-3, 2, SH110X_WHITE);
  }
}

int countSCREEN9 = 0;
int countSCREEN1 = 0 ;
//Task hiển thị OLED
void TaskOLEDDisplay(void *pvParameters) {
  while(1) {
    switch(screenOLED) {
      case SCREEN0:
    oled.clearDisplay();

    // Nhiệt độ
    oled.drawBitmap(0, 0, epd_bitmap_temperature_celsius_1, 16, 16, SH110X_WHITE);
    oled.setCursor(20, 4);
    oled.setTextSize(1);
    oled.print(tempValue, 1);
    oled.write(247); // ký hiệu °
    oled.print("C");

    // Độ ẩm KK
    oled.drawBitmap(0, 20, drop16x16, 16, 16, SH110X_WHITE);
    oled.setCursor(20, 24);
    oled.print(humiValue,1);
    oled.print("%");

    // Độ ẩm đất
    oled.drawBitmap(0, 40, plant16x16, 20, 20, SH110X_WHITE);
    oled.setCursor(20, 50);
    oled.print(soilMoistureValue,1);
    oled.print("%");

    oled.setTextSize(1);
          oled.setCursor(62, 4);
          oled.print("Muc nuoc: ");
          if(waterWarning == ENABLE) {
              if(outRange == 1) {
                oled.setTextSize(2);
                oled.setCursor(95, 47);
                oled.print("!!"); 
               }
            }
          oled.setTextSize(2);
          oled.setCursor(62, 12);
          oled.print(waterValue); 
          oled.print("/"); 
          oled.print(EheightInstallSensor); 
          oled.setTextSize(1);
          oled.print(" cm"); 
           
          oled.setTextSize(1);
          oled.setCursor(60, 34);
          oled.print("Warning: ");
          oled.setTextSize(2);
          oled.setCursor(60, 46);
          oled.print(EthresholdWarning); 
          oled.setTextSize(1);
          oled.print(" cm"); 

          drawWaterLevel();
    oled.display();
    delay(200);
    break;

      case SCREEN1: // lựa chọn các loại cây trồng
        oled.clearDisplay();
        oled.setTextSize(1.5);
        oled.setCursor(0,10);
        for (int i = 0; i < itemsPerPage; i++) {
          int index = page_start + i;
          if (index >= numCrops) break;
          oled.setCursor(0, i * 16); // mỗi dòng cách 16 pixel
          if (index == select_option) {
            oled.print(">"); // đánh dấu cây đang chọn
          } else {
          oled.print("  ");
          }
          oled.print(crops[index]);
          }
          oled.display();
        break;
      case SCREEN2: //menu chon
      if(select_option < 7) {
        oled.clearDisplay();
        oled.setTextSize(1.5);
        oled.setCursor(0, 20);
        oled.print("Ban da chon:");
        oled.setCursor(0, 40);
        oled.print(crops[select_option]);
        oled.display();
        delay(2000);

    // Gán ngưỡng theo từng loại cây (ví dụ)
        switch (select_option) {
          case 0: // Raumam
            EtempThreshold2 = temp_max_raumam;
            EhumiThreshold2 = humi_max_raumam;
            EsoilMoistureThreshold1 = soil_min_raumam;
            writeEEPROM();
          break;
          case 1: // Bap cai
            EtempThreshold2 = temp_max_bapcai;
            EhumiThreshold2 = humi_max_bapcai;
            EsoilMoistureThreshold1 = soil_min_bapcai;
            writeEEPROM();
          break;
          case 2: // Ca chua
            EtempThreshold2 = temp_max_cachua;
            EhumiThreshold2 = humi_max_cachua;
            EsoilMoistureThreshold1 = soil_min_cachua;
            writeEEPROM();
          break;
          case 3: // Xalach
            EtempThreshold2 = temp_max_xalach;
            EhumiThreshold2 = humi_max_xalach;
            EsoilMoistureThreshold1 = soil_min_xalach;
            writeEEPROM();
          break;
          case 4: // Dua chuot
            EtempThreshold2 = temp_max_duachuot;
            EhumiThreshold2 = humi_max_duachuot;
            EsoilMoistureThreshold1 = soil_min_duachuot;
            writeEEPROM();
          break;
          case 5: // sen da
            EtempThreshold2 = temp_max_senda;
            EhumiThreshold2 = humi_max_senda;
            EsoilMoistureThreshold1 = soil_min_senda;
            writeEEPROM();
          break;
          case 6: // Rau mui
            EtempThreshold2 = temp_max_raumui;
            EhumiThreshold2 = humi_max_raumui;
            EsoilMoistureThreshold1 = soil_min_raumui;
            writeEEPROM();
          break;
        }
      // TODO: thêm case cho các loại khác
    select_option = 0;
    page_start = 0;
    screenOLED = SCREEN0;// quay về màn hình chính
    } 
    break;
      case SCREEN3: //Nếu tự thiết lập
        oled.clearDisplay();
        oled.setTextSize(1.5);
        oled.setCursor(0,10);
        oled.print("Nhiet do max:");
        oled.print(EtempThreshold2);
        oled.drawCircle(93,10,2,SH110X_WHITE);
        oled.print(" C");
        oled.setCursor(0,30);
        oled.print("Do am max:");
        oled.print(EhumiThreshold2);
        oled.print(" %");
        oled.setCursor(0,50);
        oled.print("Do am dat min:");
        oled.print(EsoilMoistureThreshold1);
        oled.print(" %");
        if (select_option == 0) {
          clearRectangle(96, 0, 128, 64);
          oled.setCursor(120,10);
          oled.print("<");
          oled.display();
          delay(200);
        }
        if (select_option == 1) {
          clearRectangle(96, 0, 128, 64);
          oled.setCursor(120,30);
          oled.print("<");
          oled.display();
          delay(200);
        }
        if (select_option == 2) {
          clearRectangle(96, 0, 128, 64);
          oled.setCursor(120,50);
          oled.print("<");
          oled.display();
          delay(200);
        }
        writeEEPROM();
        page_start = 0;
        break;
      case SCREEN4: //Tự thiết lập hoàn tất
        oled.clearDisplay();
        oled.setTextSize(1.5);
        oled.setCursor(0,30);
        oled.print("Thiet lap hoan tat");
        oled.display();
        delay(2000);
        menu_option = 0;
        screenOLED = SCREEN0;
        break;
      case SCREEN5:    // auto : on
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 20);
            oled.print("Auto Mode:");
            oled.setTextSize(2);
            oled.setCursor(40, 32);
            oled.print("DISABLE"); 
            for(int i = 0; i < FRAME_COUNT_autoOnOLED; i++) {
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, autoOnOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }
            clearRectangle(40, 32, 128, 64);
            oled.setCursor(40, 32);
            oled.print("ENABLE"); 
            oled.display();   
            delay(2000);
            screenOLED = SCREEN0;
            enableShow = ENABLE;
            break;
          case SCREEN6:     // auto : off
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(40, 20);
            oled.print("Auto Mode:");
            oled.setTextSize(2);
            oled.setCursor(40, 32);
            oled.print("ENABLE");
            for(int i = 0; i < FRAME_COUNT_autoOffOLED; i++) {
              clearRectangle(0, 0, 32, 64);
              oled.drawBitmap(0, 16, autoOffOLED[i], FRAME_WIDTH_32, FRAME_HEIGHT_32, 1);
              oled.display();
              delay(FRAME_DELAY);
            }
            clearRectangle(40, 32, 128, 64);
            oled.setCursor(40, 32);
            oled.print("DISABLE"); 
            oled.display();    
            delay(2000);
            screenOLED = SCREEN0;  
            enableShow = ENABLE;
            break;
    }
    oled.display();
    delay(500);
  }
}

//-----------------------Task Task Button ----------
void TaskButton(void *pvParameters) {
    while(1) {
      handle_button(&buttonSELECT);
      handle_button(&buttonUP);
      handle_button(&buttonDOWN);
      handle_button(&buttonMENU);
      handle_button(&buttonMAYBOM);
      handle_button(&buttonCUACHAN);
      delay(50);
    }
}
//-----------------Hàm xử lí nút nhấn nhả ----------------------
void button_press_short_callback(uint8_t button_id) {
    switch(button_id) {
      case BUTTON1_ID : 
        buzzerBeep(1);
        Serial.println("btUP press short");
        Serial.println(select_option);
        if (screenOLED == SCREEN1){
          select_option--;
          if (select_option < 0) select_option = numCrops - 1;

          // cuộn lên
          if (select_option < page_start) {
          page_start = select_option;
          }
          delay(200);
        }
        if (screenOLED == SCREEN3) {
          if (select_option == 0) EtempThreshold2++;
          if (select_option == 1) EhumiThreshold2++;
          if (select_option == 2) EsoilMoistureThreshold1++;
        }
        break;

      case BUTTON2_ID : 
        buzzerBeep(1);
        Serial.println("btDOWN press short");
        Serial.println(select_option);
        if(screenOLED == SCREEN1){
          select_option++;
          if (select_option >= numCrops) select_option = 0;

          // cuộn xuống
          if (select_option >= page_start + itemsPerPage) {
            page_start = select_option - itemsPerPage + 1;
          }
          delay(200);
         }
       if (screenOLED == SCREEN3) {
          if (select_option == 0) EtempThreshold2--;
          if (select_option == 1) EhumiThreshold2--;
          if (select_option == 2) EsoilMoistureThreshold1--;
       }
        break;

      case BUTTON3_ID :
        buzzerBeep(1);
        Serial.println("btAUTO press short");
        
          auto_mode = 1 - auto_mode;
          if(auto_mode == 0) screenOLED = SCREEN6;
          else screenOLED = SCREEN5;
          sendLoraAUTO();
          delay(2000);
          sendLoraMAYBOM();
          delay(30);
        
        
        
        break;  

      case BUTTON4_ID :
        buzzerBeep(1);
        Serial.println("btMENU press short");
        Serial.println(select_option);
        Serial.println(menu_option);
        menu_option++;
        if (screenOLED == SCREEN3) {
          select_option++;
          if (select_option > 2) select_option = 0;
        }
        if(screenOLED == SCREEN0 && menu_option == 1) {
          screenOLED = SCREEN1;
        }
        if (screenOLED == SCREEN1 && select_option == 7) {
          screenOLED = SCREEN3;
          menu_option = 0;
          select_option = 0;
          break;
        }
        if(screenOLED == SCREEN1 && menu_option == 2 ) {
          screenOLED = SCREEN2;
          menu_option = 0;
        }
        Serial.println(menu_option);
        if (screenOLED == SCREEN1 && select_option == 7) {
          screenOLED = SCREEN3;
          menu_option = 0;
          break;
        }
        break; 
      case BUTTON5_ID:
        buzzerBeep(1);
        Serial.println("btMAYBOM press short");
        if (auto_mode == DISABLE) {
          digitalWrite(RELAY1,!digitalRead(RELAY1));
        }
        sendLoraMAYBOM();
        delay(30);
      break;

      case BUTTON6_ID:
        buzzerBeep(1);
        Serial.println("btCUACHAN press short");
        int buttonState = digitalRead(RELAY2);
        if (auto_mode == DISABLE) {
          servoState = !servoState; // đảo trạng thái
          if (servoState) {
          moveServoSlow(60);   // quay 90°
          Serial.println("Servo -> 90°");
          digitalWrite(RELAY2,1);
          } else {
          moveServoSlow(0);    // quay về 0°
          Serial.println("Servo -> 0°");
          digitalWrite(RELAY2,0);
        }
        delay(300); // chống dội nút
        }
        lastButtonState = buttonState;
        sendLoraCUACHAN();
        delay(30);
        
      break;
    } 
} 

//-----------------Hàm xử lí nút nhấn giữ ----------------------

void button_press_long_callback(uint8_t button_id) {
  switch(button_id) {
    case BUTTON1_ID :
      enableShow = DISABLE;
      Serial.println("btUP press long");
      clearOLED();
      break;
    case BUTTON2_ID :
      Serial.println("btDOWN press short");
      break;
    case BUTTON3_ID :{
      Serial.println("btSELECT press short");
      buzzerBeep(2);
      enableShow = DISABLE;
      auto_mode = 1 - auto_mode;
      if(auto_mode == 0) screenOLED = SCREEN6;
      else screenOLED = SCREEN5;
      sendLoraAUTO();
      delay(2000);
      sendLoraMAYBOM();
      delay(30);
      break; 
    } 
    case BUTTON4_ID :
    buzzerBeep(2);
      Serial.println("btMENU press short");
      if (screenOLED == SCREEN3) {
            screenOLED = SCREEN4;
            writeEEPROM();
            break;
          }
      break;  
  } 
} 

// ---------------------- Hàm điều khiển còi -----------------------------
void buzzerBeep(int numberBeep) {
  for(int i = 0; i < numberBeep; ++i) {
    digitalWrite(BUZZER, ENABLE);
    delay(100);
    digitalWrite(BUZZER, DISABLE);
    delay(100);
  }  
}

/*
 * Các hàm liên quan đến lưu dữ liệu cài đặt vào EEPROM
*/
//--------------------------- Read Eeprom  --------------------------------
void readEEPROM() {
   
    EtempThreshold1 = EEPROM.read(200);
    EtempThreshold2 = EEPROM.read(201);

    EhumiThreshold1 = EEPROM.read(202);
    EhumiThreshold2 = EEPROM.read(203);

    EsoilMoistureThreshold1 = EEPROM.read(204) * 100 + EEPROM.read(205);
    EsoilMoistureThreshold2 = EEPROM.read(206) * 100 + EEPROM.read(207);  

   
}

// ------------------------ Clear Eeprom ------------------------

void clearEeprom() {
    Serial.println("Clearing Eeprom");
    for (int i = 0; i < 250; ++i) 
      EEPROM.write(i, 0);
}

// -------------------- Hàm ghi data vào EEPROM ------------------
void writeEEPROM() {
    

    EEPROM.write(200, EtempThreshold1);          // lưu ngưỡng nhiệt độ 1
    EEPROM.write(201, EtempThreshold2);          // lưu ngưỡng nhiệt độ 2

    EEPROM.write(202, EhumiThreshold1);          // lưu ngưỡng độ ẩm 1
    EEPROM.write(203, EhumiThreshold2);          // lưu ngưỡng độ ẩm 2

    EEPROM.write(204, EsoilMoistureThreshold1 / 100);      // lưu hàng nghìn + trăm bụi 1
    EEPROM.write(205, EsoilMoistureThreshold1 % 100);      // lưu hàng chục + đơn vị bụi 1

    EEPROM.write(206, EsoilMoistureThreshold2 / 100);      // lưu hàng nghìn + trăm bụi 2
    EEPROM.write(207, EsoilMoistureThreshold2 % 100);      // lưu hàng chục + đơn vị bụi 2
    
    EEPROM.commit();

    Serial.println("write eeprom");
    delay(500);
}
