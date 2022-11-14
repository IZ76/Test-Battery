#pragma region include & defs
#include <Arduino.h>
#include "FS.h"
#include <ArduinoJson.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
//#include <pgmspace.h>
#include <driver/adc.h>
#include <soc/adc_channel.h>
#include "esp_adc_cal.h"
//
#define DEVICE_NAME "Test Battery v.003"
#define AUTHOR_NAME "IvanZah (github - IZ76)"
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
//#define MAX(x,y) ( x>y?x:y )
//#define MIN(x,y) ( x<y?x:y )
#define IWIDTH  240 // Розмір зображення спрайту для тексту, що прокручується, для цього потрібно ~14 Кбайт оперативної пам’яті
#define IHEIGHT 30
#define CALIBRATION_FILE "/calibrationData2"
#define REPEAT_CAL false
#define KEY_TEXTSIZE 1
#define KEY_XB 62
#define KEY_YB 212
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
#define NUM_LEN 12       
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
  String ssid={"IvanZah"};
  String pass={"11223344"};
  float vol_start[4]          = {0.39, 0.39,  0.39,  0.39}; // мінімальне значення яке бачить порт ЕСП
  float vol_opor[4]           = {10.5, 10.39, 10.32, 10.427};  // Калібровка зарядженного аккумулятора
  float rezist[4]             = {3.73, 3.62,  3.66,  3.6}; // Калібровка току при розрядці
  float vol_min[4]            = {2.5,  2.5,   2.5,   2.5};
  float vol_max[4]            = {4.2,  4.2,   4.2,   4.2};
  uint8_t pin_rele[4] = {25, 26, 27, 14}; // порти керування нагрузкою
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
String logString[4] = {};             // змінна для береження логу перез записом
String fullFilesName[4];
uint8_t currentFileNumer[4] = {0, 0, 0, 0}; // поточний номер ім'я файлу
uint8_t currentFileAmount[4] = {0, 0, 0, 0}; // 0 - готовий до використання, 1 - відкритий для запису, 2 - фіналізувати
const uint16_t LOG_STRING_MAX_SIZE = 1024; // буфер змінної логу
const uint8_t  FILENAME_DIGITS = 2;   // кількість знаків в імені файлу
//const uint16_t MAX_LOG_SIZE = 30000;  // максимальний розмір файлу в SPIFFS
uint8_t disp_stat = 0;                // 0-малюемо табличку, 1-заповнюємо табличку, 9-
uint8_t setup_num = 0;                // номер вкладки меню налаштування
uint8_t setup_num_old = 0;            // попередньє значення
uint64_t speed_counter = 0;           // швидкість зміни значення
float vol_settings[4]       = {0};    // змінна для коригування налаштувань
float vol_settings_old[4]   = {0};    // попередньє її значення
float adcDataVol[4]         = {0};    // поточна напруга аккуму
float test_start_vol[4]     = {0};    // початкова напруга аккуму
float test_opir[4]          = {0};    // внутрішній опор аккуму
float test_amperage[4]      = {0};    // ток аккуму
float test_cap[4]           = {0};    // емність аккуму
float test_pow[4]           = {0};    // потужність аккуму
uint8_t test_stat[4]        = {0};    // 0-стоп, 1-вимірювання , 2- пауза, 3- кінець тесту
uint16_t test_time[4]       = {0};
uint8_t test_time_minute[4] = {0};
uint64_t test_prevMillis[4] = {0};    // Змінні відрізку розрахункового часу емності батарей
uint64_t test_timeStart[4]  = {0};    // Змінні часу розріду батарей
// Створюємо клавіші клавіатур
// тестування
char keyLabel_test[14][6] = {"TEST", "TEST", "TEST", "TEST", "PAUSE", "PAUSE", "PAUSE", "PAUSE", "RESET", "RESET", "RESET", "RESET", "SETUP", "GRAPH"};
char end_test[4] = {"END"};
uint16_t keyColor_test[13] = {TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
                              TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
                              TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
                              TFT_DARKGREY};
TFT_eSPI_Button key_test[13];
// налаштування
char keyLabel_set[14][6] = {"+", "+", "+", "+", "-", "-", "-", "-", "SET", "SET", "SET", "SET", "BACK", "NEXT"};
char esc_set[7] = {"Cancel"};
char save_set[5] = {"SAVE"};
uint16_t keyColor_set[14] = {TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
                             TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
                             TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
                             TFT_DARKGREY, TFT_DARKGREY};
TFT_eSPI_Button key_set[14];
// графіки
char keyLabel_graph[2][7] = {"Cancel", "NEXT"};
uint16_t keyColor_graph[2] = {TFT_DARKGREY, TFT_DARKGREY};
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
int16_t  adcDataSafe[1000 / frameLength_mS][4] = {0}; // в loop() копіюємо сюди adcData для неспішної обробки у основному потоці
uint16_t LOG_ACCUM_COUNTER = 30;   // т.я. adcDataSafe готовий раз на секунду, це фактично каже скільки секунд спостереження
                                  // буде відповідати одному запису в логі
uint8_t logCounter[4]     =  {0}; // лічильник для логів
//esp_adc_cal_characteristics_t *adc_chars;
//esp_adc_cal_value_t val_type;
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t semSampleNext;  // по тику таймера запускаем задачу (task) чтения ADC
volatile SemaphoreHandle_t semSamplesReady;// по наполнению данными adcIN запускаем loop()
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#pragma endregion working variables

//-------------------------------------------------------------------------------------------------
#pragma region timer task
// сигналізує по таймеру що час читати ADC
void IRAM_ATTR onTimerReadADC(){
  // сюди потрапляємо 1000 разів в секунду
  if(adcDataIndex < ARRAY_SIZE(adcData)){
    xSemaphoreGiveFromISR(semSampleNext, NULL);
  }
}
void IRAM_ATTR tskReadADC(void *pvParameters){  
  (void) pvParameters;
  uint8_t startIndex = 0;
  for (;;){
    if( xSemaphoreTake( semSampleNext, portMAX_DELAY) == pdTRUE){
      // отримуємо сигнал, робимо одне зчитування
      // сюди потрапляємо 1000 раз у секунду
      // змінюємо порядок читанняя каналів
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
  }
  json_str="";
  serializeJson(doc, json_str);
  File a = SPIFFS.open("/settings.json", FILE_WRITE);
  if(!a || serializeJson(doc, a) == 0){
    Serial.println(F("Failed to write to /settings.json"));
    a.close();
    return;
  }
  a.close();
  //Serial.println("SAVE /settings.json=" + json_str);
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
    set_fs.vol_start[i] = doc["arr"][i*6];
    set_fs.vol_opor[i] = doc["arr"][i*6+1];
    set_fs.rezist[i] = doc["arr"][i*6+2];
    set_fs.vol_min[i] = doc["arr"][i*6+3];
    set_fs.vol_max[i] = doc["arr"][i*6+4];
    set_fs.pin_rele[i] = doc["arr"][i*6+5];
  }
  //Serial.println("LOAD /settings.json(" + String(size) + "): " + json_str);
}
#pragma endregion settings in memory
//--------------------------------------------------------------------------------------------
#pragma region spiffs
// робота з пам'яттю
void viewFilesSpifss(){
  File d = SPIFFS.open("/", FILE_READ);                         // open() - перший аргумент є символьний або рядковий покажчик на шлях до файлу, а другий режим відкриття,
  if(!d){
    Serial.println("spifsDir: Error opening");
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
      fullFilesName[i] = "/" + prefixName[i] + String(currentFileNumer[i]) + myFileExt;;
      if(!SPIFFS.exists(fullFilesName[i])){
        File e = SPIFFS.open(fullFilesName[i], FILE_WRITE);
        Serial.println("NEW_fullFilesName: " + fullFilesName[i] + "  " + String(e.size()));
        e.close();
      }
    }
  } else {
    File e = SPIFFS.open(fullFilesName[flush], FILE_WRITE);
    e.close();
    Serial.println("Erase: " + fullFilesName[flush]);
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
  Serial.println("Res: "+String(result));
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
void drawBmp(const char *filename, int16_t x, int16_t y){
  if((x >= tft.width()) || (y >= tft.height())) return;
  fs::File bmpFS;
  bmpFS = SPIFFS.open(filename, FILE_READ);
  if(!bmpFS){
    Serial.print("File not found:");
    Serial.println(filename);
    return;
  }
  uint32_t seekOffset;
  uint16_t w, h, row;
  uint8_t r, g, b;
  if(read16(bmpFS) == 0x4D42){
    read32(bmpFS);
    read32(bmpFS);
    seekOffset = read32(bmpFS);
    read32(bmpFS);
    w = read32(bmpFS);
    h = read32(bmpFS);
    if((read16(bmpFS) == 1) && (read16(bmpFS) == 24) && (read32(bmpFS) == 0)){
      y += h - 1;
      bool oldSwapBytes = tft.getSwapBytes();
      tft.setSwapBytes(true);
      bmpFS.seek(seekOffset);
      uint16_t padding = (4 - ((w * 3) & 3)) & 3;
      uint8_t lineBuffer[w * 3 + padding];
      for(row = 0; row < h; row++){
        bmpFS.read(lineBuffer, sizeof(lineBuffer));
        uint8_t *bptr = lineBuffer;
        uint16_t *tptr = (uint16_t *)lineBuffer;
        // Convert 24 to 16 bit colours
        for(uint16_t col = 0; col < w; col++){
          b = *bptr++;
          g = *bptr++;
          r = *bptr++;
          *tptr++ = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        }
        // Push the pixel row to screen, pushImage will crop the line if needed
        // y is decremented as the BMP image is drawn bottom up
        tft.pushImage(x, y--, w, 1, (uint16_t *)lineBuffer);
      }
      tft.setSwapBytes(oldSwapBytes);
    } else Serial.println("[WARNING]: BMP format not recognized.");
  } else Serial.println("BMP format not recognized.");
  bmpFS.close();
}
// Малюємо віконця
void draw_table(){
  tft.fillRect(DISP_I, DISP_I, 470, DISP_H, TFT_BLACK);
  tft.drawRect(DISP_I, DISP_I, 470, DISP_H, TFT_WHITE);
  tft.fillRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
  tft.drawRect(DISP_I, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
  tft.fillRect(DISP_I * 2 + DISP_WB, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
  tft.drawRect(DISP_I * 2 + DISP_WB, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
  tft.fillRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
  tft.drawRect(DISP_I * 3 + DISP_WB * 2, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
  tft.fillRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_BLACK);
  tft.drawRect(DISP_I * 4 + DISP_WB * 3, DISP_I * 2 + DISP_H, DISP_WB, DISP_HB, TFT_WHITE);
}
void draw_graph(){
  tft.fillRect(DISP_I, DISP_I, 470, DISP_H, TFT_BLACK);
  tft.drawRect(DISP_I, DISP_I, 470, DISP_H, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextSize(2);
  for(uint8_t i = 0; i < 20; i++){
    tft.drawLine(14 + (i * 24), 70, 14 + (i * 24), 310,TFT_DARKGREY);
  }
  for(uint8_t i = 0; i < 11; i++){
    tft.drawLine(14, 310-(i*24), 470, 310-(i*24),TFT_DARKGREY);
    if(i % 2 == 0) tft.drawString((String(i / 2)), 12, 302-(i*24));
  }
}
// Малюємо клавіатури
void draw_klaw_graph(){
  tft.setFreeFont(LABEL2_FONT);
  key_graph[0].initButton(&tft, KEY_XB,
                                KEY_YG,
                                KEY_WB, KEY_DG, TFT_WHITE,
                                keyColor_graph[0], TFT_GREENYELLOW,
                                keyLabel_graph[0], KEY_TEXTSIZE);
  key_graph[0].drawButton();
  //key_graph[1].initButton(&tft, KEY_XB + 358,
  //                              KEY_YG,
  //                              KEY_WB, KEY_DG, TFT_WHITE,
  //                              keyColor_graph[1], TFT_GREENYELLOW,
  //                              keyLabel_graph[1], KEY_TEXTSIZE);
  //key_graph[1].drawButton();
}
void draw_klaw_test(){
  for(uint8_t row = 0; row < 3; row++){
    for(uint8_t col = 0; col < 4; col++){
      uint8_t b = col + row * 4;
      tft.setFreeFont(LABEL2_FONT);
      key_test[b].initButton(&tft, KEY_XB + col * (KEY_WB + KEY_SPACING_XB), // x,
                                   KEY_YB + row * (KEY_HB + KEY_SPACING_YB), // y, 
                                   KEY_WB, KEY_HB, TFT_WHITE,                // w, h, outline, 
                                   keyColor_test[b], TFT_GREENYELLOW,        // fill, 
                                   keyLabel_test[(b > 7 && b < 12 && test_stat[col] == 0) ? 13 : b], KEY_TEXTSIZE);          // text
      key_test[b].drawButton();
    }
  }
  key_test[12].initButton(&tft, KEY_XB + 358,
                                KEY_YG,
                                KEY_WB, KEY_DG, TFT_WHITE,
                                keyColor_test[12], TFT_GREENYELLOW,
                                keyLabel_test[12], KEY_TEXTSIZE);
  key_test[12].drawButton();
  //drawBmp("/settings.bmp", 405, 5);
}
void draw_klaw_set(){
  for(uint8_t row = 0; row < 3; row++){
    for(uint8_t col = 0; col < 4; col++){
      uint8_t b = col + row * 4;
      tft.setFreeFont(LABEL2_FONT);
      key_set[b].initButton(&tft, KEY_XB + col * (KEY_WB + KEY_SPACING_XB),
                                  KEY_YB + row * (KEY_HB + KEY_SPACING_YB),
                                  KEY_WB, KEY_HB, TFT_WHITE,
                                  keyColor_set[b], TFT_GREENYELLOW,
                                  keyLabel_set[b], KEY_TEXTSIZE);
      key_set[b].drawButton();
    }
  }
  key_set[12].initButton(&tft, KEY_XB,
                               KEY_YG,
                               KEY_WB, KEY_DG, TFT_WHITE,
                               keyColor_set[12], TFT_GREENYELLOW,
                               setup_num ? keyLabel_set[12] :  esc_set, KEY_TEXTSIZE);
  key_set[12].drawButton();
  key_set[13].initButton(&tft, KEY_XB + 358,
                               KEY_YG,
                               KEY_WB, KEY_DG, TFT_WHITE,
                               keyColor_set[13], TFT_GREENYELLOW,
                               setup_num != 4 ? keyLabel_set[13] : update_ffs ? save_set : end_test, KEY_TEXTSIZE);
  key_set[13].drawButton();
}
void first_start(){
  tft.fillScreen(TFT_DARKGREY);
  draw_table();
  draw_klaw_test();
  tft.setTextColor(TFT_GOLD ,TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.drawString("IZ76 " + String(DEVICE_NAME), 180, 20);
}
#pragma endregion drawing
//-------------------------------------------------------------------------------------------------------
#pragma region log
void logWrite(uint8_t flush = 5){
  for(uint8_t i = 0; i < 4; i++){
    if(test_stat[i] == 1 || flush != 5){
      if(logCounter[i] >= LOG_ACCUM_COUNTER || flush == (i)){
        char toLogData[64];
        sprintf(toLogData, "%04d", (int)(adcDataVol[i]*100));
        //sprintf(toLogData, "%04x%04x\n", (test_time[i]/60), (int)(adcDataVol[i]*100));
        logString[i] += toLogData;
        Serial.println("BATT#" + String((i == 0) ? "A" : (i == 1) ? "B" : (i == 2) ? "C" : "D") + ": \n" + String(logString[i]));
        logCounter[i] = 0;
        if(flush == (i)){//         } || logString[i].length() > LOG_STRING_MAX_SIZE){
          while(adcDataIndex < ARRAY_SIZE(adcData)) delay(1); // ждём пока ADC свои дела поделает
          sprintf(toLogData, "%04d", (0));
          logString[i] += toLogData;
          File log = SPIFFS.open(fullFilesName[i], FILE_WRITE);
          log.print(logString[i]);
          log.close();
          logString[i].remove(0);
        }
      }else{
        logCounter[i]++;
      }
    }
  }
}
#pragma endregion log

//------------------------------------------------------------------------------------------------------
void view_graph(uint_fast8_t batt_num){
  tft.fillScreen(TFT_BLACK);
  draw_graph();
  uint8_t a = 0, b = 0;
  uint8_t num = 0;
  while(!a){
    if(!b){
      File log = SPIFFS.open(fullFilesName[batt_num], FILE_READ);
      String log_str = log.readString();
      log.close();
      uint16_t len = log_str.length();
      Serial.println("log (" + fullFilesName[batt_num] + "): \n" + log_str + "  (" + String(len) + ")");
      uint16_t interval[4] = {239, 479, 959, 1919};
      uint8_t div_fac = 1 * (len > interval[2]) + 2 * (len > interval[1] && len <= interval[2]) + 4 * (len > interval[0] && len <= interval[1]) + 8 * (len <= interval[0]) ;
      char shkala[4][4][5] = {{"6m.", "12m", "18m", "24m"}, {"12m", "24m", "36m", "48m"}, {"0.5", "1.0", "1.5", ""}, {"1h", "2h", "3h", ""}};
      uint8_t s = div_fac;
      uint8_t j = 4; 
      while(s){s = s >> 1; j--;}
      for(uint8_t i = 0; i < 4; i++){
       if(j < 2) tft.drawString(shkala[j][i], 132 + (i * 14) + (i * 82),  306);
       else      tft.drawString(shkala[j][i], 154 + (i * 14) + (i * 106), 306);
      }
      float u_opor = 2.083;
      uint16_t x = 14;
      uint16_t y = 310 - (log_str.substring(0, 4).toInt() / u_opor);
      for(uint16_t i = 0; i < len / 4; i++){
        uint16_t result = log_str.substring(0, 4).toInt();
        log_str.remove(0, 4);
        tft.drawLine(x, y, x+div_fac, 310 - (result / u_opor),TFT_YELLOW);
        x += div_fac;
        y = 310 - (result / u_opor);
      }
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
        if(b == 1){}
      }
    }
  }
}

void view_test(){
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextSize(2);
  for(uint8_t i = 0; i < 4; i++){
    if(adcDataVol[i] == set_fs.vol_start[i]){
      tft.drawString(("NoBatt"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
    } else {
      tft.drawString((String(adcDataVol[i]) + " V"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
    }
    if(test_stat[i] > 0){
      tft.drawString((String(test_opir[i]) + " R"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YR);
      tft.drawString((String(test_amperage[i]) + " A"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YI);
      tft.drawString((String(test_pow[i]) + " Wh"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YW);
      tft.drawString((String((int)test_cap[i]) + " mAh"), VOL_X + (DISP_I+DISP_WB) * i, VOL_YC);
    }
    if(test_stat[i] > 0){
      if(test_time[i] < 600){
        tft.drawString((String(test_time[i]) + " s"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YT);
      } else {
        //if(test_time_minute[i] == 0){
        //  tft.fillRect(DISP_I * ( i + 2) + DISP_WB * i - 4, VOL_YT, DISP_WB - 4, 16, TFT_BLACK);
        //  test_time_minute[i] = 1;
        //} 
        tft.drawString("  " + (String(test_time[i] / 60) + " m"), VOL_X + (DISP_I + DISP_WB) * i, VOL_YT);
      }
    }
  }
  uint16_t t_x = 0, t_y = 0;
  bool pressed = tft.getTouch(&t_x, &t_y); // Натиснення буде встановлено як істинне, якщо є дійсний дотик до екрана
  // Перевірте, чи будь-які ключові поля координат містять координати дотику
  for(uint8_t b = 0; b < 13; b++){
    if(pressed && key_test[b].contains(t_x, t_y)){
      key_test[b].press(true);  // скажіть кнопці, що вона натиснута
    } else {
      key_test[b].press(false);  // скажіть кнопці, що вона НЕ натиснута
    }
  }
  // Перевірте, чи змінився стан будь-якої клавіші
  for(uint8_t b = 0; b < 13; b++){
    tft.setFreeFont(LABEL2_FONT); // гладкий ;)
    if(key_test[b].justReleased()) key_test[b].drawButton();     // малювати нормально
    if(key_test[b].justPressed()){
      key_test[b].drawButton(true);  // інвертувати
      if(b < 4){ // ЯКЩО НАТИСНУТА КНОПКА TEST
        if((test_stat[b] == 0 || test_stat[b] == 2) && adcDataVol[b] > 1.0){
          if(test_stat[b] == 0){ // ЦЕ ПЕРШИЙ СТАРТ ТЕСТУ
            test_start_vol[b] = adcDataVol[b];
            test_opir[b] = 0;
            char toLogData[64];
            sprintf(toLogData, "%04d", (int)(test_start_vol[b]*100));
            logString[b] = toLogData;   // Перший запис в логі
            key_test[b + 8].initButton(&tft, KEY_XB + b * (KEY_WB + KEY_SPACING_XB),
                                         KEY_YB  + 2 * (KEY_HB + KEY_SPACING_YB),
                                         KEY_WB, KEY_HB, TFT_WHITE,
                                         keyColor_test[b + 8], TFT_GREENYELLOW,
                                         keyLabel_test[b + 8], KEY_TEXTSIZE);
            key_test[b + 8].drawButton();
          }
          digitalWrite(set_fs.pin_rele[b], HIGH);
          keyColor_test[b] = TFT_RED;
          key_test[b].initButton(&tft, KEY_XB + b * (KEY_WB + KEY_SPACING_XB),
                                       KEY_YB,
                                       KEY_WB, KEY_HB, TFT_WHITE,
                                       keyColor_test[b], TFT_GREENYELLOW,
                                       keyLabel_test[b], KEY_TEXTSIZE);
          key_test[b].drawButton();
          if(test_stat[b] == 2){
            keyColor_test[b + 4] = TFT_DARKGREY;
            key_test[b + 4].initButton(&tft, KEY_XB + b * (KEY_WB + KEY_SPACING_XB),
                                             KEY_YB + (KEY_HB + KEY_SPACING_YB),
                                             KEY_WB, KEY_HB, TFT_WHITE, 
                                             keyColor_test[b + 4], TFT_GREENYELLOW,
                                             keyLabel_test[b + 4], KEY_TEXTSIZE);
            
            key_test[b + 4].drawButton();
          }
          if(test_stat[b] == 0){
            test_timeStart[b] = millis();                      // Скидуємо лічильник часу тривання тесту батареї №1
            test_prevMillis[b] = millis();                     // Скидуємо лічильник відрізку розрахункового часу емності батареї №1
          }
          test_stat[b] = 1;
        }
      }
      if(b > 3 && b < 8){  // ЯКЩО НАТИСНУТА КНОПКА PAUSE
        if(test_stat[b - 4] == 1){
          keyColor_test[b - 4] = TFT_DARKGREY;
          key_test[b - 4].initButton(&tft, KEY_XB + (b - 4) * (KEY_WB + KEY_SPACING_XB),
                                           KEY_YB,
                                           KEY_WB, KEY_HB, TFT_WHITE,
                                           keyColor_test[b - 4], TFT_GREENYELLOW,
                                           keyLabel_test[b - 4], KEY_TEXTSIZE);
          key_test[b - 4].drawButton();
          keyColor_test[b] = TFT_RED;
          key_test[b].initButton(&tft, KEY_XB + (b - 4) * (KEY_WB + KEY_SPACING_XB),
                                   KEY_YB +  (KEY_HB + KEY_SPACING_YB),
                                   KEY_WB, KEY_HB, TFT_WHITE,
                                   keyColor_test[b], TFT_GREENYELLOW, 
                                   keyLabel_test[b], KEY_TEXTSIZE);
          key_test[b].drawButton();
          test_stat[b - 4] = 2;
          //logWrite(b - 4);   // зпис в логі перед паузою
          digitalWrite(set_fs.pin_rele[b - 4], LOW);
        }
      }
      if(b > 7 && b < 12){ //ЯКЩО НАТИСНУТА КНОПКА RESET
        if(test_stat[b - 8] > 0){
          key_test[b].initButton(&tft, KEY_XB + (b - 8) * (KEY_WB + KEY_SPACING_XB),
                                       KEY_YB + 2 * (KEY_HB + KEY_SPACING_YB),
                                       KEY_WB, KEY_HB, TFT_WHITE,
                                       keyColor_test[b], TFT_GREENYELLOW,
                                       keyLabel_test[13], KEY_TEXTSIZE);
          key_test[b].drawButton();
          if(test_stat[b - 8] == 2){
            keyColor_test[b - 4] = TFT_DARKGREY;
            key_test[b - 4].initButton(&tft, KEY_XB + (b - 8) * (KEY_WB + KEY_SPACING_XB),
                                             KEY_YB + (KEY_HB + KEY_SPACING_YB),
                                             KEY_WB, KEY_HB, TFT_WHITE,
                                             keyColor_test[b - 4], TFT_GREENYELLOW,
                                             keyLabel_test[b - 4], KEY_TEXTSIZE);
            key_test[b - 4].drawButton();
          } else {
            keyColor_test[b - 8] = TFT_DARKGREY;
            key_test[b - 8].initButton(&tft, KEY_XB + (b - 8) * (KEY_WB + KEY_SPACING_XB),
                                             KEY_YB, 
                                             KEY_WB, KEY_HB, TFT_WHITE,
                                             keyColor_test[b - 8], TFT_GREENYELLOW, 
                                             keyLabel_test[b - 8], KEY_TEXTSIZE);
            key_test[b - 8].drawButton();
          }
          if(test_stat[b - 8] != 3) logWrite(b - 8); // запис логу в пам'ять
          test_stat[b - 8] = 0;
          test_cap [b - 8] = 0;
          test_pow [b - 8] = 0;
          test_time_minute[b - 8] = 0;
          tft.fillRect(DISP_I * (b - 7) + DISP_WB * (b - 8) + 1, VOL_Y, DISP_WB - 2, 126, TFT_BLACK);
          digitalWrite(set_fs.pin_rele[b - 8], LOW);
        } else {
          view_graph(b - 8);
          first_start();
        }
      }
      if(b == 12){
        disp_stat = 1;
        setup_num = 0;
        setup_num_old = 1;
      }
    }
  }
  for(u_int8_t i = 0; i < 4; i++){
    if(test_stat[i] < 3) test_time[i] = (test_prevMillis[i] - test_timeStart[i]) / 1000;
  }
}
void setup_vol(){
  if(setup_num != setup_num_old){ // малюемо екран налаштувань
    setup_num_old = setup_num;
    draw_table();
    draw_klaw_set();
    tft.setTextColor(TFT_GOLD ,TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    String mes[5] = {"SETUP: MIN_BATT", "SETUP: MAX_BATT", "SETUP: LOW_LEVEL", "SETUP: HIGH_LEVEL", "SETUP: REZISTOR"};
    tft.drawString(mes[setup_num], 240, 20);
    tft.setTextDatum(TR_DATUM);
    for(uint8_t i = 0; i < 4; i++){
      vol_settings_old[i] = vol_settings[i] = (set_fs.vol_min[i] * (setup_num == 0))
                                            + (set_fs.vol_max[i] * (setup_num == 1))
                                            + (set_fs.vol_start[i]*(setup_num == 2))
                                            + (set_fs.vol_opor[i] *(setup_num == 3))
                                            + (set_fs.rezist[i] *  (setup_num == 4));
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
        tft.drawString((String(adcDataVol[i]) + " V"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
      } else {
        digitalWrite(set_fs.pin_rele[i], HIGH);
        tft.drawString((String(adcDataVol[i]/set_fs.rezist[i]) + " A"), VOL_X + (DISP_I + DISP_WB) * i, VOL_Y);
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
          if(update_ffs) loadSpiffs();
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
void display_led(){
  switch (disp_stat){
    case 0: //режим тестування
      view_test();
      break;
    case 1: //режим налаштування
      setup_vol();
      break;
    default:
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
  //loadChFN();
  
}
void setup_createAP(){
    Serial.print( "Starting Access point... IP=");
    WiFi.softAP("ESP_TEST_BATT","11112222", 6);                        
    IPAddress myIP = WiFi.softAPIP();
    Serial.println(myIP);
}
void setup_WIFI(){
    String mess = "SSID: " + String(set_fs.ssid) + ", PASS: " +  String(set_fs.pass);
    Serial.println(mess);
    tft.drawString(mess, 0, text_y);
    text_y += text_step;
    //WiFi.begin(set_fs.ssid.c_chr(), set_fs.pass);             // Connect to the network
    uint8_t tryCount=10;
    while(tryCount-- > 0 && !WiFi.isConnected()) { // Wait for the Wi-Fi to connect
      delay(500);
      Serial.print('.');
      tft.drawString("*", (10-tryCount)*12, text_y);
    }
    text_y += text_step;
    Serial.println('\n');
    if(WiFi.isConnected() ){
      Serial.println("Connection established");  
      Serial.print("IP address:\t");
      Serial.println(WiFi.localIP());
      tft.drawString("IP address: " + WiFi.localIP().toString(), 0, text_y);
      text_y += text_step;
    }else{
      Serial.println("Can't connect to "+String(set_fs.ssid));  
      WiFi.disconnect();
      setup_createAP();
      tft.drawString("IP_AP: " + WiFi.softAPIP().toString(), 0, text_y);
      text_y += text_step;
    }
    delay(500);
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
  //setup_WIFI();
  setup_ADC();
  setup_tasks();
  setup_timer();
  Serial.println("Setup done");
  delay(1);
  checkFileName();
  first_start();
}
#pragma endregion setup
// -------------------------------------------------------------------------
// Main loop
// -------------------------------------------------------------------------
void loop() {
  display_led();
  if(xSemaphoreTake(semSamplesReady, 1) == pdTRUE){
    // копіюємо дані з буферу adcData в безпечний по часу обробки adcDataSafe
    memcpy(adcDataSafe,adcData,sizeof(adcData));
    portENTER_CRITICAL(&timerMux);
    adcDataIndex = 0; // дозволяємо ADC працювати далі
    portEXIT_CRITICAL(&timerMux);

    for(uint8_t i = 0; i < 4; i++){
      adcDataVol[i] = float(adcDataSafe[0][i]) * set_fs.vol_opor[i] / 4095 + set_fs.vol_start[i];
      if(adcDataVol[i] < set_fs.vol_min[i] && test_stat[i] == 1){ // Кінець тесту по розряду аккумулятора
        test_stat[i] = 3;
        keyColor_test[i] = TFT_BLUE;
        key_test[i].initButton(&tft, KEY_XB + i * (KEY_WB + KEY_SPACING_XB),
                                     KEY_YB,
                                     KEY_WB, KEY_HB, TFT_WHITE,
                                     keyColor_test[i], TFT_GREENYELLOW,
                                     end_test, KEY_TEXTSIZE);
        key_test[i].drawButton();
        logWrite(i);    // запис логу в пам'ять
        digitalWrite(set_fs.pin_rele[i], LOW);
      }
      test_amperage[i] = 0;
      if(test_stat[i] == 1){ // Розрахунок параметрів аккумулятора
        test_amperage[i] = adcDataVol[i] / set_fs.rezist[i];     // Розрахунок струму розряду батареї
        test_cap[i] += test_amperage[i] * (millis() - test_prevMillis[i]) / 3600; // Розрахунок емности батареї в мА/годину
        test_pow[i] += test_amperage[i] * (millis() - test_prevMillis[i]) / 1000 * adcDataVol[i] / 3600;  // Розрахунок емности батареї в Вт/годину
        if(test_opir[i] == 0 && test_time[i] > 2) test_opir[i] = (test_start_vol[i] - adcDataVol[i]) / test_amperage[i];
      }
      test_prevMillis[i] = millis(); // Скидуемо лічильник відрізку розрахункового часу емності батареї №1
    }

    logWrite();
    //Serial.print  ("A= " + String(adcDataVol[0], 3) + "    ");
    //Serial.print  ("B= " + String(adcDataVol[1], 3) + "    ");
    //Serial.print  ("C= " + String(adcDataVol[2], 3) + "    ");
    //Serial.println("D= " + String(adcDataVol[3], 3));
  }
}

