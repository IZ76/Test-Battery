#pragma region include & defs
#include <Arduino.h>
#include "FS.h"
#include <ArduinoJson.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <driver/adc.h>
#include <soc/adc_channel.h>
#include "esp_adc_cal.h"
#include <OneWire.h>
#include <DallasTemperature.h>
//
#define DEVICE_NAME "Test Battery v.05"
#define AUTHOR_NAME "IvanZah (github - IZ76)"
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#define MAX(x,y)(x>y?x:y)
#define MIN(x,y)(x<y?x:y)
#define IWIDTH  240 // Розмір зображення спрайту для тексту, що прокручується, для цього потрібно ~14 Кбайт оперативної пам’яті
#define IHEIGHT 30
#define WIDTH 470
#define WIDTH_X 310
#define WIDTH_S 24
#define HEIGHT 240
#define CALIBRATION_FILE "/calibrationData2"
#define REPEAT_CAL false
#define KEY_TEXTSIZE 1
#define KEY_XB 62
#define KEY_YB 212
#define KEY_YC 252
#define KEY_WB 98
#define KEY_HB 30
#define KEY_G 80
#define KEY_YG 31
#define KEY_DG 40
#define KEY_SPACING_XB 21
#define KEY_SPACING_YB 10
// Using two fonts since numbers are nice when bold
#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2
#define LABEL3_FONT &FreeSerif12pt7b   // Key label font 2
// Numeric display box size and location
#define DISP_H 50
#define DISP_I 6
#define DISP_WB 113
#define DISP_HB 252
#define DISP_TSIZE 3
// Number length, buffer for storing it and character index
#define VOL_X 112
#define VOL_Y 70
#define VOL_YR 90
#define VOL_YI 110
#define VOL_YT 130
#define VOL_YW 150
#define VOL_YC 170
#pragma endregion include & defs

#pragma region SETTINGS
struct {
  float vol_start[4]          = {0.39, 0.39,  0.39,  0.39};   // мінімальне значення яке бачить порт ЕСП
  float vol_opor[4]           = {10.5, 10.39, 10.35, 10.427}; // Калібровка зарядженного аккумулятора
  float rezist[4]             = {4.00, 3.91,  3.93,  3.84};   // Калібровка току при розрядці
  float vol_min[4]            = {2.75, 2.75,  2.75,  2.75};
  float vol_max[4]            = {4.2,  4.2,   4.2,   4.2};
  uint8_t pin_rele[4] = {25, 26, 27, 14};  // порти керування нагрузкою
  uint8_t pin_rele_summ[3] = {16, 13, 17}; // порти реле обєднання
  uint8_t pin_ADC0 = 34; // порти для вимірювання параметрів аккумулятору
  uint8_t pin_ADC1 = 35;
  uint8_t pin_ADC2 = 32;
  uint8_t pin_ADC3 = 33;
} set_fs;
#pragma endregion SETTINGS

#pragma region working variables
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite img = TFT_eSprite(&tft);
uint8_t text_y = 0;
uint8_t text_step = 24;               // відступ друкування строк
uint8_t update_ffs = 0;               // маркер необхідності оновленя налаштувань в SPIFFS
String str_var = "";                  // для проміжних результатів
const String myFileExt = ".txt";      // розширення файлу
String logStringU[4] = {};             // змінна для береження логу перез записом
String logStringI[4] = {};             // змінна для береження логу перез записом
String fullFilesNameU[4];
String fullFilesNameI[4];
uint8_t currentFileNumer[4] = {0, 0, 0, 0}; // поточний номер ім'я файлу
uint8_t currentFileAmount[4] = {0, 0, 0, 0}; // 0 - готовий до використання, 1 - відкритий для запису, 2 - фіналізувати
const uint16_t LOG_STRING_MAX_SIZE = 1024; // буфер змінної логу
const uint8_t  FILENAME_DIGITS = 2;   // кількість знаків в імені файлу
//const uint16_t MAX_LOG_SIZE = 30000;  // максимальний розмір файлу в SPIFFS
uint8_t disp_stat = 0;                // 0-тестування, 1-налаштування
uint8_t conf_test = 0;                // 0-(1_2_3_4); 1-(1+2_3_4); 2-(1+2_3+4); 3-(1+2+3_4); 4-(1+2+3+4)
uint8_t setup_num = 0;                // номер вкладки меню налаштування
uint8_t setup_num_old = 0;            // попередньє значення
uint64_t speed_counter = 0;           // швидкість зміни значення
float vol_settings[4]       = {0};    // змінна для коригування налаштувань
float vol_settings_old[4]   = {0};    // попередньє її значення
float adcDataVoltage[4]         = {0};    // поточна напруга аккуму
float prev_adcDataVoltage[4]    = {0};    // попереднє значення напруги
float adc_delta             = 0.01;   // різниця напруг для запису в лог
float test_start_vol[4]     = {0};    // початкова напруга аккуму
uint16_t test_opir[4]          = {0};    // внутрішній опор аккуму
float test_amperage[4]      = {0};    // ток аккуму
float test_cap[4]           = {0};    // емність аккуму
float test_pow[4]           = {0};    // потужність аккуму
uint8_t test_stat[4]        = {0};    // 0-стоп, 1-вимірювання, 3- кінець тесту
uint16_t test_time[4]       = {0};
uint64_t test_prevMillis[4] = {0};    // Змінні відрізку розрахункового часу емності батарей
uint64_t test_timeStart[4]  = {0};    // Змінні часу розряду батарей
uint8_t conf_key[5][8] = {{1, 1, 1, 1, 1, 1, 1, 1},
                          {1, 0, 1, 1, 1, 0, 1, 1},
                          {1, 0, 1, 0, 1, 0, 1, 0},
                          {1, 0, 0, 1, 1, 0, 0, 1},
                          {1, 0, 0, 0, 1, 0, 0, 0}};
// тестування
char keyLabel_test[7][6] = {"TEST", "RESET", "STOP", "  ",  "GRAPH", "SETUP", "MODE"};
TFT_eSPI_Button key_test[10];
// налаштування
char keyLabel_set[7][7] = {"+", "-", "SET", "BACK", "NEXT", "Cancel", "SAVE"};
TFT_eSPI_Button key_set[14];
// графіки
TFT_eSPI_Button key_graph[2];
// ADC vars
const adc1_channel_t nativeADC[4] = {ADC1_GPIO34_CHANNEL, ADC1_GPIO35_CHANNEL, ADC1_GPIO32_CHANNEL, ADC1_GPIO33_CHANNEL};
const uint32_t frameLength_mS = 50;  // якщо змінювати то робить секунди більш насиченими вимірами. При зменшені
                                     // frameLength_mS вимірів дадуть одне усреднене значення в adcData
                                     // adcData готовий раз на секунду: 
                                     // frameLength_mS=100 => в adcData 10 показників
                                     // frameLength_mS=10 => в adcData 100 показників
float  adcAccum[frameLength_mS][4] = {0}; // сюді данні каждого тику таймера. Кожному каналу одне вимірювання массиву 
int16_t  adcAccumIndex = 0;
int16_t  adcData[1000 / frameLength_mS][4] = {0};     // ц усреднення накопичених adcAccum[]
int16_t  adcDataIndex = 0;
int16_t  adcDataSafe[1000 / frameLength_mS][4] = {0}; // буфер данних adcData для неспішної обробки у основному потоці
                                                      //uint16_t LOG_ACCUM_COUNTER = 30;   // через скільки секунд спостереження буде запис у лозі
                                                      //uint8_t logCounter[4]     =  {0}; // лічильник для логів
                                                      //esp_adc_cal_characteristics_t *adc_chars; //esp_adc_cal_value_t val_type;
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t semSampleNext;    // по таймера запускаємо завдання читання ADC
volatile SemaphoreHandle_t semSamplesReady;  // по закінченю наповненню даними adcIN дозволяємо працювати loop()
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
OneWire oneWire(5);  // on pin 2
DallasTemperature sensors(&oneWire);
float tDallas;
#pragma endregion working variables

//-------------------------------------------------------------------------------------------------
#pragma region timer task
void IRAM_ATTR onTimerReadADC(){                 // сигналізує по таймеру що час читати ADC
  if(adcDataIndex < ARRAY_SIZE(adcData)){        // сюди потрапляємо 1000 разів в секунду
    xSemaphoreGiveFromISR(semSampleNext, NULL);
  }
}
void IRAM_ATTR tskReadADC(void *pvParameters){  
  (void) pvParameters;
  uint8_t startIndex = 0;
  for(;;){
    if(xSemaphoreTake(semSampleNext, portMAX_DELAY) == pdTRUE){
      // отримуємо сигнал, робимо одне зчитування, сюди потрапляємо 1000 раз на секунду, змінюємо порядок читанняя каналів
      startIndex++;
      for(uint8_t chan = 0; chan < 4; chan++){
        uint8_t ch = (chan + startIndex) % 4;
        adcAccum[adcAccumIndex][ch] = adc1_get_raw(nativeADC[ch]);
        adcAccum[adcAccumIndex][ch] *= adcAccum[adcAccumIndex][ch]; // потім будемо робити середньоквадратичність
      }
      portENTER_CRITICAL(&timerMux);
      // зміщуємо покажчик, при переповненні усереднюємо зчитане (накопичене)
      if(++adcAccumIndex >= ARRAY_SIZE(adcAccum)){
        adcAccumIndex = 0;
        float A0 = 0, A1 = 0, A2 = 0, A3 = 0;
        for(uint8_t i = 0; i < ARRAY_SIZE(adcAccum); i++){
          A0 += adcAccum[i][0];
          A1 += adcAccum[i][1];
          A2 += adcAccum[i][2];
          A3 += adcAccum[i][3];
        }
        adcData[adcDataIndex][0] = sqrt(A0 / ARRAY_SIZE(adcAccum));// середньоквадратичне
        adcData[adcDataIndex][1] = sqrt(A1 / ARRAY_SIZE(adcAccum));
        adcData[adcDataIndex][2] = sqrt(A2 / ARRAY_SIZE(adcAccum));
        adcData[adcDataIndex][3] = sqrt(A3 / ARRAY_SIZE(adcAccum));
        // зміщуємо покажчик накопичувача, при переповненні повідомляємо loop()
        if(++adcDataIndex >= ARRAY_SIZE(adcData)){
          xSemaphoreGive(semSamplesReady);
        } 
      }
      portEXIT_CRITICAL(&timerMux);
    }        
  }
  vTaskDelete(NULL);
}
#pragma endregion timer task
//------------------------------------------------------------------------------------------------
#pragma region settings in memory
// Зберігаємо налаштування

void saveSpiffs(){
  String json_str="{}";
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json_str);
  doc["pin_ADC0"] = set_fs.pin_ADC0;
  doc["pin_ADC1"] = set_fs.pin_ADC1;
  doc["pin_ADC2"] = set_fs.pin_ADC2;
  doc["pin_ADC3"] = set_fs.pin_ADC3;
  JsonArray arr = doc.createNestedArray("arr");
  for(uint8_t i = 0; i < 4; i++){
    arr.add(set_fs.vol_start[i]);
    arr.add(set_fs.vol_opor[i]);
    arr.add(set_fs.rezist[i]);
    arr.add(set_fs.vol_min[i]);
    arr.add(set_fs.vol_max[i]);
    arr.add(set_fs.pin_rele[i]);
    if(i < 3) arr.add(set_fs.pin_rele_summ[i]);
  }
  json_str = "";
  serializeJson(doc, json_str);
  File a = SPIFFS.open("/settings.json", FILE_WRITE);
  if(!a || serializeJson(doc, a) == 0){
    Serial.println(F("Failed to write to /settings.json"));
    a.close();
    return;
  }
  a.close();
  Serial.println("SAVE /settings.json=" + json_str);
}
void loadSpiffs(){
  if(!SPIFFS.exists("/settings.json")){
    saveSpiffs();
    return;
  }
  String json_str="{}";
  File a = SPIFFS.open("/settings.json", FILE_READ);
  size_t size = a.size();
  if(size>1024) Serial.println("AHTUNG!!! /settings.json - size>1024");
  json_str = a.readString();
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json_str);
  a.close();
  set_fs.pin_ADC0 = doc["pin_ADC0"];
  set_fs.pin_ADC1 = doc["pin_ADC1"];
  set_fs.pin_ADC2 = doc["pin_ADC2"];
  set_fs.pin_ADC3 = doc["pin_ADC3"];
  for(uint8_t i = 0; i < 4; i++){
    set_fs.vol_start[i] = doc["arr"][i*7];
    set_fs.vol_opor[i] = doc["arr"][i*7+1];
    set_fs.rezist[i] = doc["arr"][i*7+2];
    set_fs.vol_min[i] = doc["arr"][i*7+3];
    set_fs.vol_max[i] = doc["arr"][i*7+4];
    set_fs.pin_rele[i] = doc["arr"][i*7+5];
    if(i < 3) set_fs.pin_rele_summ[i] = doc["arr"][i*7+6];
  }
  Serial.println("LOAD /settings.json(" + String(size) + "): " + json_str);
}
#pragma endregion settings in memory
//--------------------------------------------------------------------------------------------
#pragma region spiffs
// робота з пам'яттю
void viewFilesSpifss(){
  File d = SPIFFS.open("/", FILE_READ);                         // open() - перший аргумент є символьний або рядковий покажчик на шлях до файлу, а другий режим відкриття,
  if(!d){
    Serial.println("spifsDir: Error opening");
    d.close();
    return;
  } 
  d.rewindDirectory();                                           // rewindDirectory() – повертає до першого файлу в директорії
  File e;
  while(e = d.openNextFile()){                                   // openNextFile() - повертає покажчик на наступний файл в кореню, інакше NULL
    String entry = e.name();                                     // name() - повертає ім'я файлу
    entry += " " + String(e.size());
    Serial.println("FILES: " + entry);
    e.close();
  }
  d.close();
}
void checkFileName(uint8_t flush = 33){ // NAN - оновити імена, 0...3 - стерти файл
  if(flush == 33){
    String prefixName[4] = {"A", "B", "C", "D"};
    for(uint8_t i = 0; i < 4; i++){
      fullFilesNameU[i] = "/" + prefixName[i] + "_U" + myFileExt;
      fullFilesNameI[i] = "/" + prefixName[i] + "_I" + myFileExt;
      if(!SPIFFS.exists(fullFilesNameU[i])){
        File e = SPIFFS.open(fullFilesNameU[i], FILE_WRITE);
        Serial.println("NEW_fullFilesNameU: " + fullFilesNameU[i] + "  " + String(e.size()));
        e.close();
      }else{ 
        Serial.println("Yes_fullFilesNameU: " + fullFilesNameU[i]);
      }
      if(!SPIFFS.exists(fullFilesNameI[i])){
        File r = SPIFFS.open(fullFilesNameI[i], FILE_WRITE);
        Serial.println("NEW_fullFilesNameI: " + fullFilesNameI[i] + "  " + String(r.size()));
        r.close();
      }else{
        Serial.println("Yes_fullFilesNameI: " + fullFilesNameI[i]);
      }
    }
  }else{
    File e = SPIFFS.open(fullFilesNameU[flush], FILE_WRITE);
    e.close();
    File r = SPIFFS.open(fullFilesNameI[flush], FILE_WRITE);
    r.close();
    Serial.println("Erase: " + fullFilesNameU[flush]);
    Serial.println("Erase: " + fullFilesNameI[flush]);
  }
}
#pragma endregion spiffs
//-----------------------------------------------------------------------------------------------------------------------------
#pragma region calibration
// калібровка та малювання на екрані
void touch_calibrate(){
  uint16_t calData[5];
  uint8_t calDataOK = 0;
  // check if calibration file exists and size is correct
  if(SPIFFS.exists(CALIBRATION_FILE)){
    if(REPEAT_CAL){
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    } else {
      File f = SPIFFS.open(CALIBRATION_FILE, FILE_READ);
      if(f){
        if(f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }
  if(calDataOK && !REPEAT_CAL){
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("Touch corners as indicated");
    tft.setTextFont(1);
    tft.println();
    if(REPEAT_CAL){
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }
    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");
    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, FILE_WRITE);
    if(f){
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}
#pragma endregion calibration
//--------------------------------------------------------------------------------
#pragma region drawing
uint16_t read16(fs::File &f){
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  Serial.println("Res: " + String(result));
  return result;
}
uint32_t read32(fs::File &f){
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
// Малюємо віконця
void draw_table(){
  tft.fillRect(DISP_I, DISP_I, WIDTH, DISP_H, TFT_BLACK);
  tft.drawRect(DISP_I, DISP_I, WIDTH, DISP_H, TFT_WHITE);
  if(conf_test == 0 || disp_stat){
    tft.fillRect(DISP_I + DISP_WB, DISP_I * 2 + DISP_H, DISP_I, DISP_HB, TFT_DARKGREY);
    tft.fillRect(DISP_I * 2 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_I, DISP_HB, TFT_DARKGREY);
    tft.fillRect(DISP_I * 3 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_I, DISP_HB, TFT_DARKGREY);
    tft.fillRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
    tft.fillRect(DISP_I * 2 + DISP_WB, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I * 2 + DISP_WB, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
    tft.fillRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
    tft.fillRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
  }else if(conf_test == 1){
    tft.fillRect(DISP_I * 2 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_I, DISP_HB, TFT_DARKGREY);
    tft.fillRect(DISP_I * 3 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_I, DISP_HB, TFT_DARKGREY);
    tft.fillRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 2 + DISP_I, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 2 + DISP_I, DISP_HB, TFT_WHITE);
    tft.fillRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
    tft.fillRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
  }else if(conf_test == 2){
    tft.fillRect(DISP_I * 2 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_I, DISP_HB, TFT_DARKGREY);
    tft.fillRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 2 + DISP_I, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 2 + DISP_I, DISP_HB, TFT_WHITE);
    tft.fillRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB * 2 + DISP_I, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB * 2 + DISP_I, DISP_HB, TFT_WHITE);
  }else if(conf_test == 3){
    tft.fillRect(DISP_I * 3 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_I, DISP_HB, TFT_DARKGREY);
    tft.fillRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 3 + DISP_I * 2, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 3 + DISP_I * 2, DISP_HB, TFT_WHITE);
    tft.fillRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
  }else{
    tft.fillRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 4 + DISP_I * 3, DISP_HB, TFT_BLACK);
    tft.drawRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB * 4 + DISP_I * 3, DISP_HB, TFT_WHITE);
  }

  
}
void draw_graph(){
  tft.fillRect(DISP_I, DISP_I, WIDTH, DISP_H, TFT_BLACK);
  tft.drawRect(DISP_I, DISP_I, WIDTH, DISP_H, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextSize(2);
  for(uint8_t i = 0; i < 20; i++){
    tft.drawLine(14 + (i * WIDTH_S), 70, 14 + (i * WIDTH_S), WIDTH_X,TFT_DARKGREY);
  }
  for(uint8_t i = 0; i < 11; i++){
    tft.drawLine(14, WIDTH_X-(i*WIDTH_S), WIDTH, WIDTH_X-(i*WIDTH_S),TFT_DARKGREY);
    if(i % 2 == 0) tft.drawString((String(i / 2)), 12, 302-(i*WIDTH_S));
  }
}
// Малюємо клавіатури
void draw_klaw_graph(){
  tft.setFreeFont(LABEL2_FONT);
  key_graph[0].initButton(&tft, KEY_XB,
                                KEY_YG,
                                KEY_WB, KEY_DG, TFT_WHITE,
                                TFT_DARKGREY, TFT_GREENYELLOW,
                                keyLabel_set[5], KEY_TEXTSIZE);
  key_graph[0].drawButton();
}
void draw_klaw_test(){
  for(uint8_t row = 0; row < 2; row++){
    for(uint8_t col = 0; col < 4; col++){
      uint8_t b = col + row * 4;
      tft.setFreeFont(LABEL2_FONT);
      if(conf_key[conf_test][b] == 1){
        key_test[b].initButton(&tft, KEY_XB + col * (KEY_WB + KEY_SPACING_XB),
                                     KEY_YC + row * (KEY_HB + KEY_SPACING_YB),
                                     KEY_WB, KEY_HB, TFT_WHITE,
                                     b > 3 || test_stat[col] == 0 ? TFT_DARKGREY : test_stat[col] == 1 ? TFT_RED : TFT_BLUE, TFT_GREENYELLOW,
                                     keyLabel_test[(b < 4 && test_stat[col] == 2) ? 4 : b < 4 ? 0 : test_stat[col] == 0 ? 3 : test_stat[col] == 1 ? 2 : 1], KEY_TEXTSIZE);
        key_test[b].drawButton();
      }
    }
  }
  if(test_stat[0] == 0 && test_stat[1] == 0 && test_stat[2] == 0 && test_stat[3] == 0){ // якщо всі тести в "0", то малюємо сетап та моде
    key_test[8].initButton(&tft, KEY_XB + 358, // SETUP
                                 KEY_YG,
                                 KEY_WB, KEY_DG, TFT_WHITE,
                                 TFT_DARKGREY, TFT_GREENYELLOW,
                                 keyLabel_test[5], KEY_TEXTSIZE);
    key_test[8].drawButton();                   // MODE
    key_test[9].initButton(&tft, KEY_XB,
                                 KEY_YG,
                                 KEY_WB, KEY_DG, TFT_WHITE,
                                 TFT_DARKGREY, TFT_GREENYELLOW,
                                 keyLabel_test[6], KEY_TEXTSIZE);
    key_test[9].drawButton();
  }
}
void draw_klaw_set(){
  for(uint8_t row = 0; row < 3; row++){
    for(uint8_t col = 0; col < 4; col++){
      uint8_t b = col + row * 4;
      tft.setFreeFont(LABEL2_FONT);
      key_set[b].initButton(&tft, KEY_XB + col * (KEY_WB + KEY_SPACING_XB),
                                  KEY_YB + row * (KEY_HB + KEY_SPACING_YB),
                                  KEY_WB, KEY_HB, TFT_WHITE,
                                  TFT_DARKGREY, TFT_GREENYELLOW,
                                  b < 4 ? keyLabel_set[0] : b < 8 ? keyLabel_set[1] : keyLabel_set[2], KEY_TEXTSIZE);
      key_set[b].drawButton();
    }
  }
  key_set[12].initButton(&tft, KEY_XB,       // Cancel OR BACK
                               KEY_YG,
                               KEY_WB, KEY_DG, TFT_WHITE,
                               TFT_DARKGREY, TFT_GREENYELLOW,
                               setup_num ? keyLabel_set[3] :  keyLabel_set[5], KEY_TEXTSIZE);
  key_set[12].drawButton();
  key_set[13].initButton(&tft, KEY_XB + 358,  // NEXT OR SAVE
                               KEY_YG,
                               KEY_WB, KEY_DG, TFT_WHITE,
                               TFT_DARKGREY, TFT_GREENYELLOW,
                               setup_num != 4 ? keyLabel_set[4] : update_ffs ? keyLabel_set[6] : keyLabel_set[5], KEY_TEXTSIZE);
  key_set[13].drawButton();
}
void first_start(){
  tft.fillScreen(TFT_DARKGREY);
  draw_table();
  draw_klaw_test();
  tft.setTextColor(TFT_GOLD ,TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("IZ76 " + String(DEVICE_NAME), HEIGHT, 20);
}
#pragma endregion drawing
//-------------------------------------------------------------------------------------------------------
#pragma region log
void logWrite(uint8_t flush = 5){
  for(uint8_t i = 0; i < 4; i++){
    if(test_stat[i] == 1 && flush == 5){
      if(MAX(adcDataVoltage[i],prev_adcDataVoltage[i]) - MIN(adcDataVoltage[i],prev_adcDataVoltage[i]) > adc_delta){
        prev_adcDataVoltage[i] = adcDataVoltage[i];
        char toLogData[64];
        sprintf(toLogData, "%05d%03d", test_time[i], ((int)(adcDataVoltage[i]*1000))/10);
        logStringU[i] += toLogData;
        Serial.println("BATT#" + String((i == 0) ? "A" : (i == 1) ? "B" : (i == 2) ? "C" : "D") + ": \n" + String(logStringU[i]));
        char toLogData1[64];
        sprintf(toLogData1, "%03d", ((int)(test_amperage[i]*1000))/10);
        logStringI[i] += toLogData1;
        Serial.println("I_batt#" + String((i == 0) ? "A" : (i == 1) ? "B" : (i == 2) ? "C" : "D") + ": \n" + String(logStringI[i]));
      }
    }else if(flush == (i)){
      while(adcDataIndex < ARRAY_SIZE(adcData)) delay(1); // ждём пока ADC свои дела поделает
      char toLogData[64];
      sprintf(toLogData, "%05d%03d", test_time[i], ((int)(adcDataVoltage[i]*1000))/10);
      logStringU[i] += toLogData;
      Serial.println(fullFilesNameU[i] + ": \n" + String(logStringU[i]));
      File log = SPIFFS.open(fullFilesNameU[i], FILE_WRITE);
      log.print(logStringU[i]);
      log.close();
      logStringU[i].remove(0);
      char toLogData1[64];
      sprintf(toLogData1, "%03d", ((int)(test_amperage[i]*1000))/10);
      logStringI[i] += toLogData1;
      Serial.println(fullFilesNameI[i] + ": \n" + String(logStringI[i]));
      File log1 = SPIFFS.open(fullFilesNameI[i], FILE_WRITE);
      log1.print(logStringI[i]);
      log1.close();
      logStringI[i].remove(0);
    }
  }
}
#pragma endregion log

//------------------------------------------------------------------------------------------------------------------
void view_graph(uint8_t batt_num){
  tft.fillScreen(TFT_BLACK);
  draw_graph();
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t num = 0;
  while(!a){
    if(!b){
      File e = SPIFFS.open(fullFilesNameU[batt_num], FILE_READ);
      String log_str_U = e.readString();
      Serial.println("log_U= " + log_str_U);
      e.close();
      File r = SPIFFS.open(fullFilesNameI[batt_num], FILE_READ);
      String log_str_I = r.readString();
      Serial.println("log_I= " + log_str_I);
      r.close();
      uint16_t numChar = log_str_U.length();   // кількість символів в рядку
      uint16_t numMeasPoints =  numChar / 8;   // кількість вимірів
      uint16_t measTime = log_str_U.substring(numChar - 8, numChar - 3).toInt();
      Serial.println("log (" + fullFilesNameU[batt_num] + "): \n" + log_str_U + "  (" + String(numChar) + ")");
      uint16_t interval[4] = {20, 40, 60, 120};
      uint8_t div_fac = 0;
      for(uint8_t i = 0; i < 4; i++){
        if(interval[i] > (measTime / 60)) break;
        div_fac++;
      }
      char shkala[5][3][5] = {{"5m.", "10m", "15m"}, {"10m", "20m", "30m"}, {"15m", "30m", "45m"}, {"0.5h", "1.0h", "1.5h"}, {"1h", "2h", "3h"}};
      for(uint8_t i = 0; i < 3; i++){
        tft.drawString(shkala[div_fac][i], 154 + (i * 14) + (i * 106), 306);
      }
      float divisionValue[5] = {2.5, 5, 7.5, 15, 30};
      float u_opor = 2.083;
      uint16_t x = 14;
      uint16_t y = WIDTH_X - (log_str_U.substring(5, 8).toInt() / u_opor);
      uint16_t y_I = WIDTH_X - (log_str_I.substring(0, 3).toInt() / u_opor);
      uint16_t x_s;
      for(uint16_t i = 0; i < numMeasPoints; i++){
        uint16_t sec_test = log_str_U.substring(0, 5).toInt();
        log_str_U.remove(0, 5);
        uint16_t result = log_str_U.substring(0, 3).toInt();
        log_str_U.remove(0, 3);
        uint16_t result_I = log_str_I.substring(0, 3).toInt();
        log_str_I.remove(0, 3);
        x_s = 14 + sec_test / (divisionValue[div_fac]);
        tft.drawLine(x, y, x_s, WIDTH_X - (result / u_opor), TFT_YELLOW);
        tft.drawLine(x, y_I, x_s, WIDTH_X - (result_I / u_opor), TFT_RED);
        y = WIDTH_X - (result / u_opor);
        y_I = WIDTH_X - (result_I / u_opor);
        x = x_s;
      }
      //tft.drawLine(x, y_I, x_s, WIDTH_X, TFT_RED);
      tft.drawLine(x, y, x_s, WIDTH_X, TFT_YELLOW);
      b = 1;
      draw_klaw_graph();
    }
    uint16_t t_x = 0, t_y = 0;
    bool pressed = tft.getTouch(&t_x, &t_y);
    for(uint8_t b = 0; b < 2; b++){
      if(pressed && key_graph[b].contains(t_x, t_y)){
        key_graph[b].press(true);
      }else{
        key_graph[b].press(false);
      }
    }
    for(uint8_t b = 0; b < 2; b++){
      if(key_graph[b].justReleased()) key_graph[b].drawButton();
      if(key_graph[b].justPressed()){
        key_graph[b].drawButton(true);
        if(b == 0) return;
      }
    }
  }
}
//------------------------------------------------------------------------------------------------------------------
void view_test(){
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextSize(2);
  for(uint8_t i = 0; i < 4; i++){
    if(conf_key[conf_test][i] == 1){  // якщо дозволено робити тест цій позиції
      if(adcDataVoltage[i] == set_fs.vol_start[i]){ // якщо батарея не підключена
        tft.drawString(("NoBatt"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
      }else{ // якщо підключена то 
        tft.drawString((String(adcDataVoltage[i]) + " V"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
      }
      if(test_stat[i] > 0){ // якющо тест або кінець тесту то відображаємо показники
        tft.drawString((String(test_opir[i]) + " mR"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YR);
        tft.drawString((String(test_amperage[i]) + " A"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YI);
        tft.drawString((String(test_pow[i]) + " Wh"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YW);
        tft.drawString((String((int)test_cap[i]) + " mAh"), VOL_X + (DISP_I+DISP_WB) * i, VOL_YC);
        if(test_time[i] < 600){
          tft.drawString((String(test_time[i]) + " s"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YT);
        } else {
          tft.drawString("  " + (String(test_time[i] / 60) + " m"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YT);
        }
      }
    }
  }
  uint16_t t_x = 0, t_y = 0;
  bool pressed = tft.getTouch(&t_x, &t_y); // Натиснення буде встановлено як істинне, якщо є дійсний дотик до екрана
  // Перевірте, чи будь-які ключові поля координат містять координати дотику
  uint8_t d = 8; 
  if(test_stat[0] == 0 && test_stat[1] == 0 && test_stat[2] == 0 && test_stat[3] == 0) d += 2;  // дозволяємо сетап
  if(d == 8 && millis() % 5000 < 500){
    sensors.requestTemperatures(); // Send the command to get temperatures
    tDallas = sensors.getTempCByIndex(0);
    tft.drawString(String(tDallas, 1) + "`C", 460, WIDTH_S);
  }
  for(uint8_t b = 0; b < d; b++){
    if(conf_key[conf_test][b < 4 ? b : b < 8 ? b - 4 : b - 8] == 1 || b > 7){
      if(pressed && key_test[b].contains(t_x, t_y)){
        key_test[b].press(true);  // скажіть кнопці, що вона натиснута
      } else {
        key_test[b].press(false);  // скажіть кнопці, що вона НЕ натиснута
      }
    }
  }
  // Перевірте, чи змінився стан будь-якої клавіші
  for(uint8_t b = 0; b < d; b++){
    tft.setFreeFont(LABEL2_FONT); // гладкий ;)
    if(key_test[b].justReleased()) key_test[b].drawButton();     // малювати нормально
    if(key_test[b].justPressed()){
      key_test[b].drawButton(true);  // інвертувати
      if(b < 4){ // ========================  ЯКЩО НАТИСНУТА КНОПКА TEST ==================================
        if(test_stat[b] == 0 && adcDataVoltage[b] > 1.0){
          test_start_vol[b] = prev_adcDataVoltage[b] = adcDataVoltage[b];
          digitalWrite(set_fs.pin_rele[b], HIGH);
          if(b == 0 && conf_test > 0)  digitalWrite(set_fs.pin_rele[1], HIGH);
          if(b == 2 && conf_test == 2) digitalWrite(set_fs.pin_rele[3], HIGH);
          if(b == 0 && conf_test > 2)  digitalWrite(set_fs.pin_rele[2], HIGH);
          if(b == 0 && conf_test == 4) digitalWrite(set_fs.pin_rele[3], HIGH);
          test_timeStart[b] = millis();                      // Скидуємо лічильник часу тривання тесту батареї №1
          test_prevMillis[b] = millis();                     // Скидуємо лічильник відрізку розрахункового часу емності батареї
          test_stat[b] = 1;
          test_opir[b] = 0;
          test_cap [b] = 0;
          test_pow [b] = 0;
          test_amperage[b] = 0;
          // малюємо тестову кнопку в червоний колір
          key_test[b].initButton(&tft, KEY_XB + b * (KEY_WB + KEY_SPACING_XB),
                                      KEY_YC,
                                      KEY_WB, KEY_HB, TFT_WHITE,
                                      TFT_RED, TFT_GREENYELLOW,
                                      keyLabel_test[0], KEY_TEXTSIZE);
          key_test[b].drawButton();
          // змінюємо "Графік" на "Стоп"
          key_test[b + 4].initButton(&tft, KEY_XB + b * (KEY_WB + KEY_SPACING_XB),
                                          KEY_YC  + 1 * (KEY_HB + KEY_SPACING_YB),
                                          KEY_WB, KEY_HB, TFT_WHITE,
                                          TFT_DARKGREY, TFT_GREENYELLOW,
                                          keyLabel_test[2], KEY_TEXTSIZE);
          key_test[b + 4].drawButton();
          tft.fillRect(371, 11, KEY_WB, KEY_DG, TFT_BLACK);
          tft.fillRect(12, 11, KEY_WB + 2, KEY_DG, TFT_BLACK);
          char toLogData[64];
          sprintf(toLogData, "00000%03d", (int)((test_start_vol[b]*1000)/10));
          logStringU[b] = toLogData;   // Перший запис в логі
          char toLogData1[64];
          sprintf(toLogData1, "000");
          logStringI[b] = toLogData1;
        }else if(test_stat[b] == 2){
          view_graph(b);
          first_start();
        }
      }else if(b > 3 && b < 8){ // =============== ЯКЩО НАТИСНУТА КНОПКА RESET ====================================
        if(test_stat[b - 4] == 2){ // якщо був кінець тесту, то скидаємо всі параметри
          key_test[b - 4].initButton(&tft, KEY_XB + (b - 4) * (KEY_WB + KEY_SPACING_XB),
                                           KEY_YC, 
                                           KEY_WB, KEY_HB, TFT_WHITE,
                                           TFT_DARKGREY, TFT_GREENYELLOW, 
                                           keyLabel_test[0], KEY_TEXTSIZE);
          key_test[b - 4].drawButton();
          // змінюємо "Ресет" на " "
          key_test[b].initButton(&tft, KEY_XB + (b - 4) * (KEY_WB + KEY_SPACING_XB),
                                       KEY_YC + 1 * (KEY_HB + KEY_SPACING_YB),
                                       KEY_WB, KEY_HB, TFT_WHITE,
                                       TFT_DARKGREY, TFT_GREENYELLOW,
                                       keyLabel_test[3], KEY_TEXTSIZE);
          key_test[b].drawButton();
          test_stat[b - 4] = 0;
          tft.fillRect(DISP_I * (b - 3) + DISP_WB * (b - 4) + 1, VOL_Y, DISP_WB - 2, 126, TFT_BLACK);
          if(test_stat[0] == 0 && test_stat[1] == 0 && test_stat[2] == 0 && test_stat[3] == 0){ // малюємо сетап
            key_test[8].initButton(&tft, KEY_XB + 358,
                                          KEY_YG,
                                          KEY_WB, KEY_DG, TFT_WHITE,
                                          TFT_DARKGREY, TFT_GREENYELLOW,
                                          keyLabel_test[5], KEY_TEXTSIZE);
            key_test[8].drawButton();
            key_test[9].initButton(&tft, KEY_XB,
                                         KEY_YG,
                                         KEY_WB, KEY_DG, TFT_WHITE,
                                         TFT_DARKGREY, TFT_GREENYELLOW,
                                         keyLabel_test[6], KEY_TEXTSIZE);
            key_test[9].drawButton();
          }
        }else if(test_stat[b - 4] == 1){ // якщо був тест, то перемикаємо в режим кінець тесту
          // малюємо тестову кнопку в синій колір та напис "Графік"
          key_test[b - 4].initButton(&tft, KEY_XB + (b - 4) * (KEY_WB + KEY_SPACING_XB),
                                           KEY_YC, 
                                           KEY_WB, KEY_HB, TFT_WHITE,
                                           TFT_BLUE, TFT_GREENYELLOW, 
                                           keyLabel_test[4], KEY_TEXTSIZE);
          key_test[b - 4].drawButton();
          // змінюємо "Стоп" на "Ресет"
          key_test[b].initButton(&tft, KEY_XB + (b - 4) * (KEY_WB + KEY_SPACING_XB),
                                       KEY_YC + 1 * (KEY_HB + KEY_SPACING_YB),
                                       KEY_WB, KEY_HB, TFT_WHITE,
                                       TFT_DARKGREY, TFT_GREENYELLOW,
                                       keyLabel_test[1], KEY_TEXTSIZE);
          key_test[b].drawButton();
          logWrite(b - 4); // запис логу в пам'ять
          digitalWrite(set_fs.pin_rele[b - 4], LOW);
          if(b - 4 == 0 && conf_test > 0)  digitalWrite(set_fs.pin_rele[1], LOW);
          if(b - 4 == 2 && conf_test == 2) digitalWrite(set_fs.pin_rele[3], LOW);
          if(b - 4 == 0 && conf_test > 2)  digitalWrite(set_fs.pin_rele[2], LOW);
          if(b - 4 == 0 && conf_test == 4) digitalWrite(set_fs.pin_rele[3], LOW);
          test_stat[b - 4] = 2;
        }
      }else if(b == 8){  // ======================= Натиснута кнопка SETUP ========================
        disp_stat = 1;
        setup_num = 0;
        setup_num_old = 1;
      }else if(b == 9){  // ======================= Натиснута кнопка MODE =========================
        conf_test++;
        if(conf_test > 4) conf_test = 0;
        if(conf_test == 0){
          digitalWrite(set_fs.pin_rele_summ[0], HIGH);
          digitalWrite(set_fs.pin_rele_summ[1], HIGH);
          digitalWrite(set_fs.pin_rele_summ[2], HIGH);
        }else if(conf_test == 1){
          digitalWrite(set_fs.pin_rele_summ[0], LOW);
          digitalWrite(set_fs.pin_rele_summ[1], HIGH);
          digitalWrite(set_fs.pin_rele_summ[2], HIGH);
        }else if(conf_test == 2){
          digitalWrite(set_fs.pin_rele_summ[0], LOW);
          digitalWrite(set_fs.pin_rele_summ[1], HIGH);
          digitalWrite(set_fs.pin_rele_summ[2], LOW);
        }else if(conf_test == 3){
          digitalWrite(set_fs.pin_rele_summ[0], LOW);
          digitalWrite(set_fs.pin_rele_summ[1], LOW);
          digitalWrite(set_fs.pin_rele_summ[2], HIGH);
        }else{
          digitalWrite(set_fs.pin_rele_summ[0], LOW);
          digitalWrite(set_fs.pin_rele_summ[1], LOW);
          digitalWrite(set_fs.pin_rele_summ[2], LOW);
        }
        draw_table();
        draw_klaw_test();
        tft.setTextColor(TFT_GOLD ,TFT_BLACK);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("IZ76 " + String(DEVICE_NAME), HEIGHT, 20);
      }
    }
    
  }
  for(u_int8_t i = 0; i < 4; i++){
    if(test_stat[i] == 1) test_time[i] = (test_prevMillis[i] - test_timeStart[i]) / 1000;
  }
}
//-----------------------------------------------------------------------------------------------------------------------
void setup_vol(){
  if(setup_num != setup_num_old){ // малюемо екран налаштувань
    setup_num_old = setup_num;
    draw_table();
    draw_klaw_set();
    tft.setTextColor(TFT_GOLD ,TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    String mes[5] = {"SETUP: MIN_BATT", "SETUP: MAX_BATT", "SETUP: LOW_LEVEL", "SETUP: HIGH_LEVEL", "SETUP: REZISTOR"};
    tft.drawString(mes[setup_num], HEIGHT, 20);
    tft.setTextDatum(TR_DATUM);
    for(uint8_t i = 0; i < 4; i++){
      vol_settings_old[i] = vol_settings[i] = setup_num == 0 ? set_fs.vol_min[i] : 
                                              setup_num == 1 ? set_fs.vol_max[i] :
                                              setup_num == 2 ? set_fs.vol_start[i] :
                                              setup_num == 3 ? set_fs.vol_opor[i] : set_fs.rezist[i];
      if(setup_num != 4) tft.drawString(("  " + String(vol_settings[i]) + " V"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YI);
      else tft.drawString(("  " + String(vol_settings[i]) + " R"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YI);
    }
  }
  for(uint8_t i = 0; i < 4; i++){ // якщо були зміни, то малюємо їх червоними
    if(vol_settings[i] != vol_settings_old[i]){
      vol_settings_old[i] = vol_settings[i];
      tft.setTextColor(TFT_RED, TFT_BLACK);
      if(setup_num != 4) tft.drawString(("  " + String(vol_settings[i]) + " V"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YI);
      else tft.drawString(("  " + String(vol_settings[i]) + " R"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YI);
    }
    if(setup_num > 1 && setup_num < 5){
      tft.setTextFont(1); //від 1 та вище
      tft.setTextSize(2);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      if(setup_num != 4){
        tft.drawString((String(adcDataVoltage[i]) + " V"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
      } else {
        digitalWrite(set_fs.pin_rele[i], HIGH);
        tft.drawString((String(adcDataVoltage[i] / vol_settings[i]) + " A"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString(("Rele-ON!"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YW);
      }
      tft.setFreeFont(LABEL2_FONT);
      tft.setTextSize(1);
    }
  }
  uint16_t t_x = 0, t_y = 0;
  bool pressed = tft.getTouch(&t_x, &t_y); // Натиснення буде встановлено як істинне, якщо є дійсний дотик до екрана
  // Перевірте, чи будь-які ключові поля координат містять координати дотику
  for(uint8_t b = 0; b < 14; b++){
    if(pressed && key_set[b].contains(t_x, t_y)){
      key_set[b].press(true);  // скажіть кнопці, що вона натиснута
    } else {
      key_set[b].press(false);  // скажіть кнопці, що вона НЕ натиснута
    }
  }
  for(uint8_t b = 0; b < 14; b++){
    if(key_set[b].justReleased()){  // тількино відпустили кнопку
      key_set[b].drawButton();      // малювати нормально
    } 
    if(key_set[b].justPressed()){   // тількино натиснули кнопку
      key_set[b].drawButton(true);  // інвертувати
      if(b < 8){
        speed_counter = millis();
      }
      if(b > 7 && b < 12){
        tft.setTextColor(TFT_GOLD, TFT_BLACK);
        if(setup_num != 4) tft.drawString(("  " + String(vol_settings[b - 8]) + " V"), VOL_X + (DISP_I + DISP_WB) * (b - 8), VOL_YI);
        else tft.drawString(("  " + String(vol_settings[b - 8]) + " R"), VOL_X + (DISP_I + DISP_WB) * (b - 8), VOL_YI);
        switch(setup_num){
          case 0:
            set_fs.vol_min[b - 8] = vol_settings[b - 8]; break;
          case 1:
            set_fs.vol_max[b - 8] = vol_settings[b - 8]; break;
          case 2:
            set_fs.vol_start[b - 8] = vol_settings[b - 8]; break;
          case 3:
            set_fs.vol_opor[b - 8] = vol_settings[b - 8]; break;
          case 4:
            set_fs.rezist[b - 8] = vol_settings[b - 8]; break;
        }
        update_ffs = 1;  // були зміни в налаштуванні - треба записати їх в пам'ять
      }
      if(b == 12){
        switch(setup_num){
        case 0:
          if(update_ffs) loadSpiffs(); // якщо були зміни, то відновлюємо з флеш
          update_ffs = 0;
          disp_stat = 0;
          first_start();
          break;
        case 1:
          setup_num = 0; break;
        case 2:
          setup_num = 1; break;
        case 3:
          setup_num = 2; break;
        case 4:
          setup_num = 3;
          for(uint8_t i = 0; i < 4; i++){
            digitalWrite(set_fs.pin_rele[i], LOW);
          }
          break;
        default:
          break;
        }
      }
      if(b == 13){
        switch (setup_num){
        case 0:
          setup_num = 1; break;
        case 1:
          setup_num = 2; break;
        case 2:
          setup_num = 3; break;
        case 3:
          setup_num = 4; break;
        case 4:
          for(uint8_t i = 0; i < 4; i++){
            digitalWrite(set_fs.pin_rele[i], LOW);
          }
          if(update_ffs) saveSpiffs();
          update_ffs = 0;
          disp_stat = 0;
          first_start();  
          break;
        default:
          break;
        }
      }
    }
    if(key_set[b].isPressed()){
      if(b < 4){ // ЯКЩО НАТИСНУТА КНОПКА "+"
        vol_settings[b] += 0.01;
        if(millis() - speed_counter < 3000){
          delay(200);
        } else delay(500 / ((millis() - speed_counter) / 500));
        
      }
      if(b > 3 && b < 8){ // А ЦЕ ЯКЩО "-"
        vol_settings[b - 4] -= 0.01;
        if(millis() - speed_counter < 3000){
          delay(200);
        } else delay(500 / ((millis() - speed_counter) / 500));
      }
    }
  }
}
//-------------------------------
void param_calc(){
  if(xSemaphoreTake(semSamplesReady, 1) == pdTRUE){
    // копіюємо дані з буферу adcData в безпечний по часу обробки adcDataSafe
    memcpy(adcDataSafe, adcData, sizeof(adcData));
    portENTER_CRITICAL(&timerMux);
    adcDataIndex = 0; // дозволяємо ADC працювати далі
    portEXIT_CRITICAL(&timerMux);
    for(uint8_t i = 0; i < 4; i++){
      adcDataVoltage[i] = float(adcDataSafe[0][i]) * set_fs.vol_opor[i] / 4095 + set_fs.vol_start[i];
      if(adcDataVoltage[i] < set_fs.vol_min[i] && test_stat[i] == 1){ // Кінець тесту по розряду аккумулятора
        key_test[i].initButton(&tft, KEY_XB + i * (KEY_WB + KEY_SPACING_XB),
                                     KEY_YC,
                                     KEY_WB, KEY_HB, TFT_WHITE,
                                     TFT_BLUE, TFT_GREENYELLOW,
                                     keyLabel_test[4], KEY_TEXTSIZE);
        key_test[i].drawButton();
        key_test[i + 4].initButton(&tft, KEY_XB + i * (KEY_WB + KEY_SPACING_XB),
                                         KEY_YC + 1 * (KEY_HB + KEY_SPACING_YB),
                                         KEY_WB, KEY_HB, TFT_WHITE,
                                         TFT_DARKGREY, TFT_GREENYELLOW,
                                         keyLabel_test[1], KEY_TEXTSIZE);
        key_test[i + 4].drawButton();
        logWrite(i);    // запис логу в пам'ять
        test_stat[i] = 2;
        digitalWrite(set_fs.pin_rele[i], LOW);
        if(i == 0 && conf_test > 0)  digitalWrite(set_fs.pin_rele[1], LOW);
        if(i == 2 && conf_test == 2) digitalWrite(set_fs.pin_rele[3], LOW);
        if(i == 0 && conf_test > 2)  digitalWrite(set_fs.pin_rele[2], LOW);
        if(i == 0 && conf_test == 4) digitalWrite(set_fs.pin_rele[3], LOW);
      }
      if(test_stat[i] == 1){ // Розрахунок параметрів аккумулятора в режимі тесту
        float r_test = set_fs.rezist[i];
        if(conf_test == 1 && i == 0)      r_test = 1 / (1 / r_test + 1 / set_fs.rezist[i + 1]);
        else if(conf_test == 2)           r_test = 1 / (1 / r_test + 1 / set_fs.rezist[i + 1]);
        else if(conf_test == 3 && i == 0) r_test = 1 / (1 / r_test + 1 / set_fs.rezist[i + 1] + 1 / set_fs.rezist[i + 2]);
        else if(conf_test == 4)           r_test = 1 / (1 / r_test + 1 / set_fs.rezist[i + 1] + 1 / set_fs.rezist[i + 2] + 1 / set_fs.rezist[i + 3]);
        test_amperage[i] = adcDataVoltage[i] / r_test;     // Розрахунок струму розряду батареї
        test_cap[i] += test_amperage[i] * (millis() - test_prevMillis[i]) / 3600; // Розрахунок емности батареї в мА/годину
        test_pow[i] += test_amperage[i] * (millis() - test_prevMillis[i]) / 1000 * adcDataVoltage[i] / 3600;  // Розрахунок емности батареї в Вт/годину
        if(test_opir[i] == 0 && test_time[i] > 1) test_opir[i] = ((test_start_vol[i] - adcDataVoltage[i]) / test_amperage[i])*1000;
        test_prevMillis[i] = millis(); // Скидуемо лічильник відрізку розрахункового часу емності батареї
        test_time[i] = (millis() - test_timeStart[i]) / 1000;
      }
    }
    logWrite();
  }
  switch (disp_stat){
    case 0: //режим тестування
      view_test();
      break;
    case 1: //режим налаштування
      setup_vol();
      break;
  }
}

//======================================================================================================
// ------------------S-E-T-U-P--------------------------------------------------------------------------
//======================================================================================================
#pragma region setup
void setup_TFT(){
  tft.init();
  tft.setRotation(3); // 0-Портрет, 1-Пейзаж (Портрет+90), 2-перевернутий портрет, 3-Перевернутий пейзаж
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(LABEL3_FONT);
  tft.setTextSize(1);
  String mess = "Welcome to " + String(DEVICE_NAME);
  tft.drawString(mess, 0, text_y);
  text_y += text_step;
  mess = "By " + String(AUTHOR_NAME);
  tft.drawString(mess, 0, text_y);
  text_y += text_step;
}
void setup_PIN(){
  for(uint8_t i = 0; i < 4; i++){
    pinMode(set_fs.pin_rele[i], OUTPUT);
    digitalWrite(set_fs.pin_rele[i], LOW);
    if(i < 3){
      pinMode(set_fs.pin_rele_summ[i], OUTPUT);
      digitalWrite(set_fs.pin_rele_summ[i], HIGH);
    }
  }
}
void setup_SPIFFS(){
  if(!SPIFFS.begin()){
    Serial.println("Formating file system");
    tft.drawString("SPIFFS - invalid (Formating file system)", 0, text_y);
    text_y += text_step;
    SPIFFS.format();
    SPIFFS.begin();
  }
  String mess = "SPIFFS initialized. Total space:" + String(SPIFFS.totalBytes()) + ",";
  Serial.print(mess);
  tft.drawString(mess, 0, text_y);
  text_y += text_step;
  mess = "  used:" + String(SPIFFS.usedBytes()) + ", avail:" +  String(SPIFFS.totalBytes()-SPIFFS.usedBytes());
  Serial.println(mess);
  tft.drawString(mess, 0, text_y);
  text_y += text_step;
}
void setup_ADC(){
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);
  adcAttachPin(set_fs.pin_ADC0);
  adcAttachPin(set_fs.pin_ADC1);
  adcAttachPin(set_fs.pin_ADC2);
  adcAttachPin(set_fs.pin_ADC3);
  analogSetPinAttenuation(set_fs.pin_ADC0, ADC_11db);
  analogSetPinAttenuation(set_fs.pin_ADC1, ADC_11db);
  analogSetPinAttenuation(set_fs.pin_ADC2, ADC_11db);
  analogSetPinAttenuation(set_fs.pin_ADC3, ADC_11db);
  for(uint8_t  i = 0; i < 100; i++){
    analogRead(set_fs.pin_ADC0);
    analogRead(set_fs.pin_ADC1);
    analogRead(set_fs.pin_ADC2);
    analogRead(set_fs.pin_ADC3);
  }
  randomSeed(analogRead(set_fs.pin_ADC0));
}
void setup_tasks(){
  // Create semaphore to inform us when the timer has fired
  semSampleNext   = xSemaphoreCreateBinary(); // поднімає таймер, час брати семпл
  semSamplesReady = xSemaphoreCreateBinary(); // поднімає сбиральник семплів, час запускати FFT
  xTaskCreatePinnedToCore(
    tskReadADC,
    "tskReadADC",
    configMINIMAL_STACK_SIZE, // Stack size
    NULL,
    1,                        // Priority
    NULL,
    ARDUINO_RUNNING_CORE      // NO_ARDUINO_RUNNING_CORE
  );
}
void setup_timer(){
  timer = timerBegin   (0, 80, true);        // тік таймеру 1 мкс
  timerAttachInterrupt (timer, &onTimerReadADC, true);
  timerAlarmWrite      (timer, 1000, true);  // 1 раз на мілліс буде викликана onTimerReadADC()
  timerAlarmEnable     (timer);              // Start an alarm
}
//****************************************************************************************************
void setup(void){
  Serial.begin(115200);
  while(!Serial && (millis() <= 1000));
  setup_TFT();
  setup_PIN();
  setup_SPIFFS();
  touch_calibrate(); // Calibrate the touch screen and retrieve the scaling factors
  //saveSpiffs();
  loadSpiffs();
  viewFilesSpifss();
  setup_ADC();
  setup_tasks();
  setup_timer();
  delay(1);
  checkFileName();
  first_start();
  sensors.begin();
}
#pragma endregion setup
// -------------------------------------------------------------------------
// Main loop
// -------------------------------------------------------------------------
void loop() {
  param_calc();
}
