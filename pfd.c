#include <Wire.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Preferences.h> 

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite pfdSprite = TFT_eSprite(&tft); 
TFT_eSprite spdSprite = TFT_eSprite(&tft); 
TFT_eSprite altSprite = TFT_eSprite(&tft); 
TFT_eSprite hdgSprite = TFT_eSprite(&tft); 

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280 bmp(&Wire);

TinyGPSPlus gps;                 
TinyGPSCustom gpgsv_sats(gps, "GPGSV", 3); 
TinyGPSCustom gngsv_sats(gps, "GNGSV", 3); 
int sats_in_view = 0; 

HardwareSerial GPS_Serial(2);    
HardwareSerial Audio_Serial(1);
Preferences prefs; 

enum SystemState { SYSTEM_POST, PFD_NORMAL, MENU_ACTIVE, MENU_EDIT, MENU_FAULTS, CAL_STEP_1, CAL_STEP_2 };
SystemState currentState = SYSTEM_POST; 
bool forceRedraw = true;

bool bno_alive = false; bool bmp_alive = false; bool gps_alive = false;
bool show_alt_err = false; 

// ==========================================
// 纯软件向量映射矩阵 
// ==========================================
int mapX = 0, sgnX = 1; // 右机翼轴
int mapY = 1, sgnY = 1; // 飞机机鼻轴
int mapZ = 2, sgnZ = 1; // 天地重力轴
bool inv_hdg = false;   // 航向反转补偿
int cal_map_Z = 2, cal_sign_Z = 1; // 标定中转变量

float raw_pitch = 0.0, raw_roll = 0.0, raw_heading = 0.0;
float pitch_cal = 0.0, roll_cal = 0.0, hdg_cal = 0.0; 
float sim_pitch = 0.0, sim_roll = 0.0, sim_heading = 0.0;
float sim_altitude = 0.0, sim_speed = 0.0, sim_vs = 0.0, display_alt = 0.0; 

float speed_trend_diff = 0.0; bool show_trend_arrow = false;
float baro_ref = 1013.0; bool use_gps_alt = false; 

bool en_spd = true; float target_spd = 250.0;
int v_mode = 1; float target_alt = 10000.0; float target_vs = 1000.0; 
bool en_hdg = true; float target_hdg = 360.0; 
int audio_volume = 20; 

bool faults[4] = {false, false, false, false};
const char* fault_names[4] = {"PULL UP", "TERR AHEAD", "STALL", "OVERSPEED"};

volatile int encoderPos = 0; volatile bool buttonPressed = false;
unsigned long lastButtonPress = 0;
int currentMenuSelection = 0; int lastMenuSelection = -1; 
int savedMenuPos = 0; int lastMenuEncoderPos = -999; 
int menuScrollOffset = 0;
const int MAX_VISIBLE_ITEMS = 6;

#define RGB_TO_BGR(r, g, b) tft.color565(b, g, r)
#define A_SKY    RGB_TO_BGR(30, 144, 255)
#define A_BROWN  RGB_TO_BGR(150, 75, 0)
#define A_GREEN  RGB_TO_BGR(0, 255, 0)
#define A_CYAN   RGB_TO_BGR(0, 255, 255)
#define A_YELLOW RGB_TO_BGR(255, 255, 0)
#define A_AMBER  RGB_TO_BGR(255, 153, 0)  
#define A_RED    RGB_TO_BGR(255, 0, 0)
#define A_WHITE  0xFFFF
#define A_BLACK  0x0000
#define A_GREY   RGB_TO_BGR(100, 100, 100)

void IRAM_ATTR encoderISR() {
  static uint8_t old_AB = 0; static int8_t encval = 0;
  static const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  old_AB <<= 2; old_AB |= ((digitalRead(25) << 1) | digitalRead(26));
  encval += enc_states[(old_AB & 0x0f)];
  if (encval > 3) { encoderPos++; encval = 0; } 
  else if (encval < -3) { encoderPos--; encval = 0; }
}
void IRAM_ATTR buttonISR() {
  if (millis() - lastButtonPress > 200) { buttonPressed = true; lastButtonPress = millis(); }
}

const byte UBLOX_5HZ[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

void playWarningAudio(uint8_t track) {
  uint16_t sum = -(0xFF + 0x06 + 0x03 + 0x00 + 0x00 + track);
  byte playCmd[10] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, track, (byte)(sum >> 8), (byte)(sum & 0xFF), 0xEF};
  Audio_Serial.write(playCmd, 10);
}
void setAudioVolume(uint8_t vol) {
  uint16_t sum = -(0xFF + 0x06 + 0x06 + 0x00 + 0x00 + vol);
  byte volCmd[10] = {0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, vol, (byte)(sum >> 8), (byte)(sum & 0xFF), 0xEF};
  Audio_Serial.write(volCmd, 10);
}

void setup() {
  Serial.begin(115200); tft.init(); tft.setRotation(2); tft.fillScreen(A_BLACK);
  prefs.begin("avionics", false);
  pitch_cal = prefs.getFloat("pCal", 0.0); roll_cal = prefs.getFloat("rCal", 0.0); hdg_cal = prefs.getFloat("hCal", 0.0);
  
  mapX = prefs.getInt("mapX", 0); sgnX = prefs.getInt("sgnX", 1);
  mapY = prefs.getInt("mapY", 1); sgnY = prefs.getInt("sgnY", 1);
  mapZ = prefs.getInt("mapZ", 2); sgnZ = prefs.getInt("sgnZ", 1);
  inv_hdg = prefs.getBool("invH", false);
  
  baro_ref = prefs.getFloat("qnh", 1013.0); use_gps_alt = prefs.getBool("gpsAlt", false);
  en_spd = prefs.getBool("enSpd", true); target_spd = prefs.getFloat("tgtSpd", 250.0);
  v_mode = prefs.getInt("vMode", 1); target_alt = prefs.getFloat("tgtAlt", 10000.0); target_vs = prefs.getFloat("tgtVs", 1000.0);
  en_hdg = prefs.getBool("enHdg", true); target_hdg = prefs.getFloat("tgtHdg", 360.0);
  audio_volume = prefs.getInt("vol", 20); 

  pfdSprite.createSprite(130, 260); spdSprite.createSprite(45, 260);
  altSprite.createSprite(65, 260); hdgSprite.createSprite(130, 30); 
  pfdSprite.setSwapBytes(true); spdSprite.setSwapBytes(true);
  altSprite.setSwapBytes(true); hdgSprite.setSwapBytes(true);

  pinMode(25, INPUT_PULLUP); pinMode(26, INPUT_PULLUP); pinMode(33, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(25), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(26), encoderISR, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(33), buttonISR, FALLING);

  Wire.begin(21, 22);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  GPS_Serial.write(UBLOX_5HZ, sizeof(UBLOX_5HZ)); 
  Audio_Serial.begin(9600, SERIAL_8N1, 14, 27); 

  delay(500); setAudioVolume(audio_volume);
}

void drawRotatedLine(int cx, int cy, float pitch_y, float roll_rad, float x_off, float y_off, float dx, float dy, uint16_t color) {
    float cosR = cos(roll_rad), sinR = sin(roll_rad);
    float sx1 = x_off, sy1 = pitch_y + y_off;
    int rx1 = cx + (sx1 * cosR - sy1 * sinR), ry1 = cy + (sx1 * sinR + sy1 * cosR);
    float sx2 = x_off + dx, sy2 = pitch_y + y_off + dy;
    int rx2 = cx + (sx2 * cosR - sy2 * sinR), ry2 = cy + (sx2 * sinR + sy2 * cosR);
    pfdSprite.drawLine(rx1, ry1, rx2, ry2, color);
}

void runSystemPOST() {
  static unsigned long lastPostUpdate = 0;
  if (forceRedraw) {
    tft.fillScreen(A_BLACK); tft.setTextColor(A_CYAN, A_BLACK); tft.setTextSize(2); tft.drawString("A320 FLIGHT POST", 25, 20); tft.drawFastHLine(0, 45, 240, A_WHITE);
    tft.setTextColor(A_WHITE, A_BLACK); tft.drawString("BNO055 IMU :", 10, 60); tft.drawString("BMP280 BARO:", 10, 90); tft.drawString("BN-220 GPS :", 10, 120); tft.drawString("DFPlayer   :", 10, 150); tft.drawString("EC11 KNOB  :", 10, 180); tft.drawFastHLine(0, 215, 240, A_WHITE);
    tft.setTextSize(1); tft.setTextColor(A_GREY, A_BLACK); tft.drawString("PRESS KNOB TO FORCE BOOT", 45, 300);
    forceRedraw = false;
  }
  while (GPS_Serial.available() > 0) { gps.encode(GPS_Serial.read()); if (gps.charsProcessed() > 10) gps_alive = true; }
  
  if (millis() - lastPostUpdate > 500) {
    lastPostUpdate = millis(); tft.setTextSize(2);
    Wire.beginTransmission(0x28); bno_alive = (Wire.endTransmission() == 0);
    tft.setTextColor(bno_alive ? A_GREEN : A_RED, A_BLACK); tft.drawString(bno_alive ? "[ OK ]" : "[ERR ]", 160, 60); 
    Wire.beginTransmission(0x76); bmp_alive = (Wire.endTransmission() == 0);
    tft.setTextColor(bmp_alive ? A_GREEN : A_RED, A_BLACK); tft.drawString(bmp_alive ? "[ OK ]" : "[ERR ]", 160, 90); 
    tft.setTextColor(gps_alive ? A_GREEN : A_YELLOW, A_BLACK); tft.drawString(gps_alive ? "[ OK ]" : "[WAIT]", 160, 120); 
    tft.setTextColor(A_GREEN, A_BLACK); tft.drawString("[ OK ]", 160, 150); 
    bool knob_ok = (digitalRead(25) == HIGH && digitalRead(26) == HIGH);
    tft.setTextColor(knob_ok ? A_GREEN : A_RED, A_BLACK); tft.drawString(knob_ok ? "[ OK ]" : "[ERR ]", 160, 180); 

    if (bno_alive && bmp_alive && gps_alive && knob_ok) {
      tft.setTextColor(A_GREEN, A_BLACK); tft.drawString("ALL SYSTEMS GO! ", 35, 230);
      delay(1000); bno.begin(); 
      bmp.begin(0x76); bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_1);
      currentState = PFD_NORMAL; forceRedraw = true; buttonPressed = false;
    } 
  }
  if (buttonPressed) {
    buttonPressed = false; tft.setTextColor(A_YELLOW, A_BLACK); tft.drawString("MANUAL OVERRIDE...", 30, 230);
    delay(500); if (bno_alive) bno.begin(); 
    if (bmp_alive) { bmp.begin(0x76); bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_1); }
    currentState = PFD_NORMAL; forceRedraw = true;
  }
}

void drawAttitudeIndicator() {
  int cx = 65, cy = 130;
  float pitch_offset = sim_pitch * 4.0; 
  
  // 【核心修正】：反转渲染层的旋转角，修复姿态仪视觉悖论
  float roll_rad = -sim_roll * PI / 180.0; 
  
  float cosR = cos(roll_rad); float sinR = sin(roll_rad);

  pfdSprite.fillSprite(A_SKY); 
  float hx = cx - pitch_offset * sinR; float hy = cy + pitch_offset * cosR;
  float lx = hx - 2000 * cosR; float ly = hy - 2000 * sinR;
  float rx = hx + 2000 * cosR; float ry = hy + 2000 * sinR;
  float nx = -sinR; float ny = cosR;
  float gx1 = lx + 2000 * nx; float gy1 = ly + 2000 * ny;
  float gx2 = rx + 2000 * nx; float gy2 = ry + 2000 * ny;

  pfdSprite.fillTriangle(lx, ly, rx, ry, gx1, gy1, A_BROWN);
  pfdSprite.fillTriangle(rx, ry, gx1, gy1, gx2, gy2, A_BROWN);
  pfdSprite.drawLine(lx, ly, rx, ry, A_WHITE);

  pfdSprite.setTextDatum(MC_DATUM); pfdSprite.setTextColor(A_WHITE); pfdSprite.setTextSize(1);
  for (int p_int = -900; p_int <= 900; p_int += 25) {
    if (p_int == 0) continue; 
    float py = -p_int * 0.4; float sy = pitch_offset + py; 
    if (sy < -200 || sy > 200) continue; 

    int line_len = (abs(p_int) % 100 == 0) ? 40 : ((abs(p_int) % 50 == 0) ? 20 : 10);  
    float x1 = cx + (-line_len/2.0 * cosR - sy * sinR); float y1 = cy + (-line_len/2.0 * sinR + sy * cosR);
    float x2 = cx + (line_len/2.0 * cosR - sy * sinR);  float y2 = cy + (line_len/2.0 * sinR + sy * cosR);
    pfdSprite.drawLine(x1, y1, x2, y2, A_WHITE);
    
    if (abs(p_int) % 100 == 0) {
      int trend_dir = (p_int > 0) ? 5 : -5; 
      float xt1 = x1 + (-trend_dir * sinR); float yt1 = y1 + (trend_dir * cosR); pfdSprite.drawLine(x1, y1, xt1, yt1, A_WHITE); 
      float xt2 = x2 + (-trend_dir * sinR); float yt2 = y2 + (trend_dir * cosR); pfdSprite.drawLine(x2, y2, xt2, yt2, A_WHITE); 
      float sx_l = -line_len/2.0 - 12.0; float sx_r = line_len/2.0 + 12.0;
      pfdSprite.drawString(String(abs(p_int)/10), cx + (sx_l * cosR - sy * sinR), cy + (sx_l * sinR + sy * cosR));
      pfdSprite.drawString(String(abs(p_int)/10), cx + (sx_r * cosR - sy * sinR), cy + (sx_r * sinR + sy * cosR));
    }
  }

  pfdSprite.drawTriangle(cx, 15, cx-5, 5, cx+5, 5, A_YELLOW); pfdSprite.drawCircle(cx, cy, 115, A_WHITE); 
  
  bool show_fd_pitch = false; float fd_target_pitch = 0.0;
  if (v_mode == 1 && en_spd) { show_fd_pitch = true; fd_target_pitch = (target_alt - display_alt) * 0.05; } 
  else if (v_mode == 2 && en_spd) { show_fd_pitch = true; fd_target_pitch = (target_vs - sim_vs) * 0.01; }
  if (fd_target_pitch > 15.0) fd_target_pitch = 15.0; if (fd_target_pitch < -10.0) fd_target_pitch = -10.0; 

  bool show_fd_roll = false; float fd_target_roll = 0.0;
  if (en_hdg) {
    show_fd_roll = true; float hdg_error = target_hdg - sim_heading;
    if (hdg_error > 180.0) hdg_error -= 360.0; if (hdg_error < -180.0) hdg_error += 360.0;
    fd_target_roll = hdg_error * 0.5;
    if (fd_target_roll > 25.0) fd_target_roll = 25.0; if (fd_target_roll < -25.0) fd_target_roll = -25.0;
  }

  int fd_y = cy - (fd_target_pitch - sim_pitch) * 4.0; int fd_x = cx + (fd_target_roll - sim_roll) * 2.0;   
  if (show_fd_pitch) pfdSprite.fillRect(cx - 40, fd_y - 1, 80, 2, A_GREEN); 
  if (show_fd_roll) pfdSprite.fillRect(fd_x - 1, cy - 40, 2, 80, A_GREEN); 

  pfdSprite.fillRect(cx - 30, cy - 2, 20, 4, A_BLACK); pfdSprite.fillRect(cx - 29, cy - 1, 18, 2, A_YELLOW); 
  pfdSprite.fillRect(cx + 10, cy - 2, 20, 4, A_BLACK); pfdSprite.fillRect(cx + 11, cy - 1, 18, 2, A_YELLOW); 
  pfdSprite.fillRect(cx - 2, cy - 2, 5, 5, A_BLACK);   pfdSprite.fillRect(cx - 1, cy - 1, 3, 3, A_YELLOW);   
  
  pfdSprite.setTextSize(2); 
  if (!bno_alive) {
    pfdSprite.fillRect(cx - 50, cy - 8, 100, 16, A_BLACK); pfdSprite.setTextColor(A_RED); pfdSprite.drawString("IMU FAULT", cx, cy);
  } else {
    if (faults[2]) { pfdSprite.fillRect(cx - 36, cy - 68, 72, 16, A_BLACK); pfdSprite.setTextColor(A_RED); pfdSprite.drawString("STALL", cx, cy - 60); }
    if (faults[0]) { pfdSprite.fillRect(cx - 48, cy + 42, 96, 16, A_BLACK); pfdSprite.setTextColor(A_RED); pfdSprite.drawString("PULL UP", cx, cy + 50); } 
    else if (faults[1]) { pfdSprite.fillRect(cx - 60, cy + 42, 120, 16, A_BLACK); pfdSprite.setTextColor(A_RED); pfdSprite.drawString("TERR AHEAD", cx, cy + 50); }
  }
  pfdSprite.pushSprite(45, 30); 
}

void drawSpeedTape() {
  spdSprite.fillSprite(A_BLACK); spdSprite.drawFastVLine(44, 0, 260, A_WHITE); 
  int spd = (int)sim_speed;
  
  if (faults[3]) { 
    int vmax_y = 130 - (sim_speed + 5 - sim_speed) * 3.0; 
    for (int y = 0; y < vmax_y; y += 10) { spdSprite.fillRect(40, y, 4, 5, A_RED); spdSprite.fillRect(40, y+5, 4, 5, A_BLACK); }
  }
  
  if (!gps.speed.isValid() || gps.speed.age() > 3000) {
    spdSprite.fillRect(0, 4, 42, 28, A_RED); spdSprite.setTextColor(A_WHITE); spdSprite.setTextSize(1); spdSprite.setTextDatum(MC_DATUM);
    spdSprite.drawString("NO GPS", 21, 11);
    if (gps.charsProcessed() < 10 && millis() > 5000) spdSprite.drawString("RX ERR", 21, 23);
    else {
      uint32_t sats_in_use = gps.satellites.isValid() ? gps.satellites.value() : 0;
      if (sats_in_view < sats_in_use) sats_in_view = sats_in_use; 
      spdSprite.drawString(String(sats_in_use) + "/" + String(sats_in_view), 21, 23);
    }
  }

  spdSprite.setTextColor(A_WHITE); spdSprite.setTextSize(1); spdSprite.setTextDatum(MR_DATUM);
  for (int i = spd - 45; i <= spd + 45; i++) {
    if (i >= 0 && i % 10 == 0) {
      int y = 130 - (i - sim_speed) * 3.0; 
      spdSprite.drawFastHLine(38, y, 6, A_WHITE); if (i % 20 == 0) spdSprite.drawString(String(i), 34, y); 
    }
  }

  if (show_trend_arrow) {
    int trend_y = 130 - (speed_trend_diff * 3.0); 
    if (trend_y < 10) trend_y = 10; if (trend_y > 250) trend_y = 250;
    spdSprite.drawLine(38, 130, 38, trend_y, A_YELLOW); spdSprite.drawLine(39, 130, 39, trend_y, A_YELLOW); 
    if (speed_trend_diff > 0) spdSprite.drawTriangle(38, trend_y, 34, trend_y+6, 42, trend_y+6, A_YELLOW);
    else spdSprite.drawTriangle(38, trend_y, 34, trend_y-6, 42, trend_y-6, A_YELLOW);
  }

  if (en_spd) {
    int t_s_y = 130 - (target_spd - sim_speed) * 3.0;
    if (t_s_y >= 0 && t_s_y <= 260) spdSprite.drawTriangle(36, t_s_y, 44, t_s_y-6, 44, t_s_y+6, A_CYAN);
  }

  spdSprite.drawFastHLine(30, 130, 14, A_YELLOW); spdSprite.drawTriangle(20, 130, 30, 125, 30, 135, A_YELLOW);
  spdSprite.pushSprite(0, 30);
}

float getVsiOffset(float vs) {
  float abs_vs = abs(vs), offset = 0;
  if (abs_vs <= 500)       offset = (abs_vs / 500.0) * 25.0;                      
  else if (abs_vs <= 1000) offset = 25.0 + ((abs_vs - 500.0) / 500.0) * 25.0;     
  else if (abs_vs <= 2000) offset = 50.0 + ((abs_vs - 1000.0) / 1000.0) * 30.0;   
  else                     offset = 80.0 + ((abs_vs - 2000.0) / 4000.0) * 30.0;   
  if (offset > 115) offset = 115; return (vs > 0) ? -offset : offset; 
}

void drawAltTape() {
  altSprite.fillSprite(A_BLACK);
  if (show_alt_err) {
    altSprite.drawFastVLine(0, 0, 260, A_WHITE); altSprite.drawFastVLine(45, 0, 260, A_WHITE); 
    altSprite.drawRect(2, 118, 43, 24, A_GREEN); altSprite.fillRect(3, 119, 41, 22, A_BLACK);
    altSprite.setTextColor(A_AMBER); altSprite.setTextSize(2); altSprite.setTextDatum(MC_DATUM); altSprite.drawString("ERR", 23, 130);
  } else {
    int alt = max(0, (int)display_alt); int prefix = alt / 10;        
    int current_unit = alt % 10; int next_unit = (current_unit + 1) % 10; int prev_unit = (current_unit - 1 + 10) % 10; 
    float fraction = display_alt - floor(display_alt); int font_h = 16; int roll_y = fraction * font_h; 
    altSprite.setTextColor(A_GREEN); altSprite.setTextSize(2); altSprite.setTextDatum(MR_DATUM); altSprite.drawString(String(prefix), 29, 130); 
    altSprite.setTextDatum(ML_DATUM);
    altSprite.drawString(String(current_unit), 30, 130 + roll_y);             
    altSprite.drawString(String(next_unit), 30, 130 + roll_y - font_h); 
    altSprite.drawString(String(prev_unit), 30, 130 + roll_y + font_h); 
    altSprite.fillRect(0, 0, 45, 118, A_BLACK); altSprite.fillRect(0, 142, 45, 118, A_BLACK);
    altSprite.drawRect(2, 118, 43, 24, A_GREEN);
    altSprite.drawFastVLine(0, 0, 260, A_WHITE); altSprite.drawFastVLine(45, 0, 260, A_WHITE); 

    if (v_mode == 1) {
      int t_a_y = 130 - (target_alt - display_alt) * 0.25;
      if (t_a_y >= 0 && t_a_y <= 260 && (t_a_y < 118 || t_a_y > 142)) {
        altSprite.fillRect(1, t_a_y - 3, 10, 7, A_CYAN); altSprite.fillRect(1, t_a_y - 1, 8, 3, A_BLACK); 
      }
    }
    altSprite.setTextColor(A_WHITE); altSprite.setTextSize(1); altSprite.setTextDatum(ML_DATUM);
    int start_i = ((int)(display_alt - 550) / 20) * 20;
    for (int i = start_i; i <= display_alt + 550; i += 20) {
      if (i >= 0) {
        int y = 130 - (i - display_alt) * 0.25; 
        if (y < 118 || y > 142) { 
          int tick_len = (i % 100 == 0) ? 10 : 5; altSprite.drawFastHLine(1, y, tick_len, A_WHITE);
          if (i % 100 == 0) altSprite.drawString(String(i / 100), 15, y); 
        }
      }
    }
  }

  int offsets[] = {25, 50, 80, 110}; String labels[] = {".5", "1", "2", "6"};
  altSprite.drawFastHLine(45, 130, 5, A_WHITE); altSprite.setTextColor(A_WHITE); altSprite.setTextSize(1); altSprite.setTextDatum(ML_DATUM);
  for(int i=0; i<4; i++) {
    altSprite.drawFastHLine(45, 130 - offsets[i], 5, A_WHITE); altSprite.drawFastHLine(45, 130 + offsets[i], 5, A_WHITE); 
    altSprite.drawString(labels[i], 52, 130 - offsets[i]); altSprite.drawString(labels[i], 52, 130 + offsets[i]);
  }
  int vsi_y = 130 + getVsiOffset(sim_vs);
  altSprite.drawFastHLine(46, vsi_y, 18, A_GREEN); altSprite.drawFastHLine(46, vsi_y-1, 18, A_GREEN); altSprite.drawLine(46, 130, 46, vsi_y, A_GREEN); 

  if (v_mode == 2) {
    int t_vs_y = 130 + getVsiOffset(target_vs);
    altSprite.drawTriangle(45, t_vs_y, 50, t_vs_y-4, 50, t_vs_y+4, A_CYAN);
  }
  altSprite.pushSprite(175, 30);
}

void drawHeadingTape() {
  hdgSprite.fillSprite(A_BLACK); hdgSprite.drawFastHLine(0, 0, 130, A_WHITE); 
  
  if (en_hdg) {
    float hdg_err = target_hdg - sim_heading;
    if (hdg_err > 180.0) hdg_err -= 360.0; if (hdg_err < -180.0) hdg_err += 360.0;
    int t_h_x = 65 + hdg_err * 5.0;
    if (t_h_x >= 0 && t_h_x <= 130) hdgSprite.drawTriangle(t_h_x, 8, t_h_x-4, 0, t_h_x+4, 0, A_CYAN);
  }

  int hdg = (int)sim_heading;
  hdgSprite.setTextColor(A_WHITE); hdgSprite.setTextSize(1); hdgSprite.setTextDatum(TC_DATUM);
  for (int i = hdg - 25; i <= hdg + 25; i++) {
    if (i % 5 == 0) {
      int x = 65 + (i - sim_heading) * 5; 
      if (x < 0 || x > 130) continue; 
      int display_hdg = ((i % 360) + 360) % 360; 
      int tick_len = (i % 10 == 0) ? 8 : 4;
      hdgSprite.drawFastVLine(x, 1, tick_len, A_WHITE);
      if (i % 10 == 0) {
        char buf[4]; sprintf(buf, "%02d", (display_hdg == 0) ? 36 : (display_hdg / 10)); 
        hdgSprite.drawString(buf, x, 12); 
      }
    }
  }
  hdgSprite.drawTriangle(65, 1, 60, 8, 70, 8, A_YELLOW); hdgSprite.drawFastVLine(65, 8, 22, A_YELLOW);
  hdgSprite.pushSprite(45, 290); 
}

void drawMenuOverlay() {
  bool needsTextRedraw = false;
  if (forceRedraw) {
    tft.fillRect(35, 70, 170, 180, A_GREY); tft.drawRect(35, 70, 170, 180, A_WHITE);
    tft.setTextColor(A_WHITE, A_GREY); tft.setTextSize(2); tft.drawString(currentState == MENU_FAULTS ? "WARNINGS" : "SETTINGS", 75, 80);
    forceRedraw = false; needsTextRedraw = true;
  }

  // 17 项完美对齐菜单
  int total_items = (currentState == MENU_FAULTS) ? 5 : 17; 
  
  if (currentState != MENU_EDIT) {
    int newSelection = ((encoderPos % total_items) + total_items) % total_items; 
    if (newSelection != lastMenuSelection) {
      currentMenuSelection = newSelection;
      if (currentMenuSelection >= menuScrollOffset + MAX_VISIBLE_ITEMS) menuScrollOffset = currentMenuSelection - MAX_VISIBLE_ITEMS + 1;
      else if (currentMenuSelection < menuScrollOffset) menuScrollOffset = currentMenuSelection;
      lastMenuSelection = currentMenuSelection; needsTextRedraw = true;
    }
  } else {
    if (encoderPos != lastMenuEncoderPos) { needsTextRedraw = true; lastMenuEncoderPos = encoderPos; }
  }

  if (needsTextRedraw) {
    tft.fillRect(36, 100, 168, 148, A_GREY);

    if (currentState == MENU_FAULTS) {
      String fItems[5] = {
        String("1. PULL UP: ") + (faults[0] ? "ON " : "OFF"),
        String("2. TERRAIN: ") + (faults[1] ? "ON " : "OFF"),
        String("3. STALL  : ") + (faults[2] ? "ON " : "OFF"),
        String("4. OVSPEED: ") + (faults[3] ? "ON " : "OFF"),
        "Back to Main"
      };
      for(int i = 0; i < MAX_VISIBLE_ITEMS && (i + menuScrollOffset) < total_items; i++) {
        int actual_idx = i + menuScrollOffset;
        if(actual_idx == currentMenuSelection) tft.setTextColor(A_BLACK, A_CYAN); else tft.setTextColor(A_WHITE, A_GREY); 
        tft.drawString(fItems[actual_idx], 45, 110 + i * 22);
      }
    } else {
      String menuItems[17];
      menuItems[0] = String("Spd Tgt: ") + (en_spd ? "ON " : "OFF");
      menuItems[1] = "Spd Val: " + String((int)target_spd);
      menuItems[2] = String("V-Mode : ") + (v_mode == 1 ? "ALT" : (v_mode == 2 ? "V/S" : "OFF"));
      menuItems[3] = "Alt Val: " + String((int)target_alt);
      menuItems[4] = "V/S Val: " + String((int)target_vs);
      menuItems[5] = String("Hdg Tgt: ") + (en_hdg ? "ON " : "OFF");
      menuItems[6] = "Hdg Val: " + String((int)target_hdg);
      menuItems[7] = "QNH: " + String((int)baro_ref);
      menuItems[8] = String("Src: ") + (use_gps_alt ? "GPS" : "BARO");
      menuItems[9] = "Volume : " + String(audio_volume);
      menuItems[10] = "Align Axes >"; 
      menuItems[11] = String("Inv HDG: ") + (inv_hdg ? "ON " : "OFF");
      menuItems[12] = "Zero Ptc/Rol"; 
      menuItems[13] = "Zero HDG"; 
      menuItems[14] = "Reset All"; 
      menuItems[15] = "Warnings... >"; 
      menuItems[16] = "Exit"; // 完美归位！

      for(int i = 0; i < MAX_VISIBLE_ITEMS && (i + menuScrollOffset) < total_items; i++) {
        int actual_idx = i + menuScrollOffset;
        if(actual_idx == currentMenuSelection) {
          if (currentState == MENU_EDIT) tft.setTextColor(A_BLACK, A_YELLOW); else tft.setTextColor(A_BLACK, A_CYAN); 
        } else tft.setTextColor(A_WHITE, A_GREY); 
        tft.drawString(menuItems[actual_idx], 45, 110 + i * 22);
      }
    }
  }
}

void loop() {
  if (currentState == SYSTEM_POST) { runSystemPOST(); return; }
  
  if (currentState == CAL_STEP_1) {
    if (forceRedraw) {
      tft.fillScreen(A_BLACK); tft.setTextColor(A_CYAN); tft.setTextSize(2); tft.setTextDatum(MC_DATUM);
      tft.drawString("AUTO ALIGN (1/2)", 120, 60);
      tft.setTextColor(A_WHITE); tft.setTextSize(1);
      tft.drawString("Place module FLAT (Z-Up).", 120, 120);
      tft.drawString("Then press the knob.", 120, 160);
      forceRedraw = false;
    }
    if (buttonPressed) {
      buttonPressed = false;
      imu::Vector<3> g1 = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
      float max1 = 0; float g_arr[3] = {g1.x(), g1.y(), g1.z()};
      for(int i=0; i<3; i++) { if(abs(g_arr[i]) > max1) { max1 = abs(g_arr[i]); cal_map_Z = i; } }
      cal_sign_Z = (g_arr[cal_map_Z] < 0) ? 1 : -1;
      currentState = CAL_STEP_2; forceRedraw = true;
    }
    return;
  }
  else if (currentState == CAL_STEP_2) {
    if (forceRedraw) {
      tft.fillScreen(A_BLACK); tft.setTextColor(A_CYAN); tft.setTextSize(2); tft.setTextDatum(MC_DATUM);
      tft.drawString("AUTO ALIGN (2/2)", 120, 60);
      tft.setTextColor(A_WHITE); tft.setTextSize(1);
      tft.drawString("Point NOSE UP (>45 deg).", 120, 120);
      tft.drawString("Then press the knob.", 120, 160);
      forceRedraw = false;
    }
    if (buttonPressed) {
      buttonPressed = false;
      imu::Vector<3> g2 = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
      float max2 = 0; float g_arr[3] = {g2.x(), g2.y(), g2.z()};
      int cal_map_Y = 0, cal_sign_Y = 1;
      
      for(int i=0; i<3; i++) { 
        if(i == cal_map_Z) continue;
        if(abs(g_arr[i]) > max2) { max2 = abs(g_arr[i]); cal_map_Y = i; } 
      }
      cal_sign_Y = (g_arr[cal_map_Y] < 0) ? 1 : -1;

      float vZ[3] = {0,0,0}; vZ[cal_map_Z] = cal_sign_Z;
      float vY[3] = {0,0,0}; vY[cal_map_Y] = cal_sign_Y;
      float vX[3];
      vX[0] = vZ[1]*vY[2] - vZ[2]*vY[1];
      vX[1] = vZ[2]*vY[0] - vZ[0]*vY[2];
      vX[2] = vZ[0]*vY[1] - vZ[1]*vY[0];

      for(int i=0; i<3; i++) { 
        if(abs(vX[i]) > 0.5) { mapX = i; sgnX = (vX[i] > 0) ? 1 : -1; } 
      }
      mapY = cal_map_Y; sgnY = cal_sign_Y;
      mapZ = cal_map_Z; sgnZ = cal_sign_Z;

      prefs.putInt("mapX", mapX); prefs.putInt("sgnX", sgnX);
      prefs.putInt("mapY", mapY); prefs.putInt("sgnY", sgnY);
      prefs.putInt("mapZ", mapZ); prefs.putInt("sgnZ", sgnZ);

      currentState = PFD_NORMAL; forceRedraw = true;
    }
    return;
  }

  while (GPS_Serial.available() > 0) { 
    gps.encode(GPS_Serial.read()); 
    if (gngsv_sats.isUpdated() && gngsv_sats.isValid()) sats_in_view = atoi(gngsv_sats.value());
    else if (gpgsv_sats.isUpdated() && gpgsv_sats.isValid()) sats_in_view = atoi(gpgsv_sats.value());
  }

  static unsigned long lastUpdate = 0;
  static unsigned long lastVsiTime = 0; static unsigned long lastTrendTime = 0; 
  static float last_vsi_alt = 0.0; static float last_trend_speed = 0.0; static bool alt_inited = false;
  
  float raw_spd = 0.0;
  if (gps_alive && gps.speed.isValid() && gps.speed.age() < 3000) raw_spd = gps.speed.knots(); 
  sim_speed += (raw_spd - sim_speed) * 0.15; 

  if (millis() - lastUpdate > 30) {
    lastUpdate = millis();

    if (millis() - lastTrendTime >= 200) {
      float dt_sec = (millis() - lastTrendTime) / 1000.0;
      float instant_accel = (sim_speed - last_trend_speed) / dt_sec; 
      static float smoothed_accel = 0.0;
      smoothed_accel = smoothed_accel * 0.7 + instant_accel * 0.3; 
      float temp_trend = smoothed_accel * 10.0;
      if (!show_trend_arrow && abs(temp_trend) > 2.0) show_trend_arrow = true;
      else if (show_trend_arrow && abs(temp_trend) <= 1.0) show_trend_arrow = false;
      if (show_trend_arrow) speed_trend_diff = temp_trend; else speed_trend_diff = 0.0;
      last_trend_speed = sim_speed; lastTrendTime = millis();
    }

    if (bno_alive) {
      // 1. 提取物理重力矢量并投影至定义的机身框架
      imu::Vector<3> raw_g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
      float phys_g[3] = {raw_g.x(), raw_g.y(), raw_g.z()};
      float gx = phys_g[mapX] * sgnX; float gy = phys_g[mapY] * sgnY; float gz = phys_g[mapZ] * sgnZ;
      float norm_g = sqrt(gx*gx + gy*gy + gz*gz);
      if (norm_g > 0.1) {
        raw_pitch = asin(constrain(-gy / norm_g, -1.0, 1.0)) * 180.0 / M_PI;
        raw_roll = atan2(gx, -gz) * 180.0 / M_PI;
      }

      // 2. 【航向终极解法】四元数空间投影旋转！
      // 让虚拟磁罗盘永远只附着在飞机的“机鼻”上，彻底免疫偏滚！
      imu::Quaternion q = bno.getQuat();
      float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
      
      // 提取真正的飞机机鼻 (Y轴) 物理指向向量
      // 修正：由于 Pitch 算法隐式将 mapped Y 定义为机尾，真正的机鼻向量需要反转 (-sgnY)
      float vx = 0, vy = 0, vz = 0;
      if (mapY == 0) vx = -sgnY; else if (mapY == 1) vy = -sgnY; else if (mapY == 2) vz = -sgnY;
      
      // 将机鼻向量旋转至真实地球坐标系 (Quaternion Vector Rotation)
      float tx = 2.0 * (qy * vz - qz * vy);
      float ty = 2.0 * (qz * vx - qx * vz);
      float tz = 2.0 * (qx * vy - qy * vx);
      float ex = vx + qw * tx + (qy * tz - qz * ty); // 地球 X (东西)
      float ey = vy + qw * ty + (qz * tx - qx * tz); // 地球 Y (南北)
      
      // 算出极其完美的物理航向角
      raw_heading = atan2(ex, ey) * 180.0 / M_PI;
      if (inv_hdg) raw_heading = 360.0 - raw_heading; 

      sim_pitch = raw_pitch - pitch_cal; 
      sim_roll = raw_roll - roll_cal;
      
      // 越过天顶时的姿态球折叠
      if (sim_pitch > 90.0) { sim_pitch = 180.0 - sim_pitch; sim_roll += 180.0; } 
      else if (sim_pitch < -90.0) { sim_pitch = -180.0 - sim_pitch; sim_roll += 180.0; }
      while (sim_roll > 180.0) sim_roll -= 360.0;
      while (sim_roll <= -180.0) sim_roll += 360.0;

      sim_heading = raw_heading - hdg_cal;
      while (sim_heading < 0) sim_heading += 360.0;
      while (sim_heading >= 360.0) sim_heading -= 360.0;
    }

    show_alt_err = false;
    if (use_gps_alt) {
      if (gps_alive && gps.altitude.isValid() && gps.altitude.age() < 3000) sim_altitude = gps.altitude.feet();
      else show_alt_err = true; 
    } else {
      if (bmp_alive) sim_altitude = bmp.readAltitude(baro_ref) * 3.28084;
      else show_alt_err = true; 
    }
    if (sim_altitude < 0) sim_altitude = 0.0;

    if (!alt_inited) { last_vsi_alt = sim_altitude; display_alt = sim_altitude; alt_inited = true; } 
    display_alt += (sim_altitude - display_alt) * 0.08; 
    if (millis() - lastVsiTime >= 200) {
      float dt_min = (millis() - lastVsiTime) / 60000.0; 
      float raw_vs = (sim_altitude - last_vsi_alt) / dt_min; 
      sim_vs = sim_vs * 0.8 + raw_vs * 0.2; 
      last_vsi_alt = sim_altitude; lastVsiTime = millis();
    }

    if (currentState == PFD_NORMAL) {
      if (forceRedraw) {
        tft.fillRect(0, 0, 240, 30, A_BLACK); tft.drawFastHLine(0, 30, 240, A_WHITE);
        tft.setTextColor(A_GREEN); tft.setTextSize(1); tft.setTextDatum(MC_DATUM);
        tft.drawString("SPEED", 40, 8); tft.drawString("G/S", 120, 8); tft.drawString("LOC", 200, 8);
        tft.setTextColor(A_CYAN); tft.drawString(en_spd ? String((int)target_spd) : "---", 40, 22);
        String v_str = "---"; if (v_mode == 1) v_str = String((int)target_alt); else if (v_mode == 2) v_str = (target_vs > 0 ? "+" : "") + String((int)target_vs);
        tft.drawString(v_str, 120, 22); tft.drawString(en_hdg ? String((int)target_hdg) : "---", 200, 22);
        tft.fillRect(0, 290, 240, 30, A_BLACK); tft.drawFastHLine(0, 290, 240, A_WHITE); 
        tft.setTextDatum(TL_DATUM); tft.setTextColor(A_CYAN, A_BLACK); tft.setTextSize(1);
        tft.drawString("QNH", 180, 295); tft.drawNumber((int)baro_ref, 180, 305);
        forceRedraw = false;
      }
      drawAttitudeIndicator(); drawSpeedTape(); drawAltTape(); drawHeadingTape();
      if ((millis() / 500) % 2 == 0) tft.fillCircle(230, 305, 3, A_RED); else tft.fillCircle(230, 305, 3, A_BLACK);
      if (buttonPressed) { buttonPressed = false; currentState = MENU_ACTIVE; lastMenuSelection = -1; forceRedraw = true; }
    } 
    else if (currentState == MENU_ACTIVE) {
      drawMenuOverlay();
      if (buttonPressed) { 
        buttonPressed = false; 
        if(currentMenuSelection == 16) { currentState = PFD_NORMAL; forceRedraw = true; } // Exit 功能回归
        else if(currentMenuSelection == 0) { en_spd = !en_spd; prefs.putBool("enSpd", en_spd); lastMenuSelection = -1; }
        else if(currentMenuSelection == 1) { currentState = MENU_EDIT; savedMenuPos = encoderPos; encoderPos = target_spd; lastMenuEncoderPos = -999; }
        else if(currentMenuSelection == 2) { v_mode = (v_mode + 1) % 3; prefs.putInt("vMode", v_mode); lastMenuSelection = -1; }
        else if(currentMenuSelection == 3) { currentState = MENU_EDIT; savedMenuPos = encoderPos; encoderPos = target_alt / 100; lastMenuEncoderPos = -999; }
        else if(currentMenuSelection == 4) { currentState = MENU_EDIT; savedMenuPos = encoderPos; encoderPos = target_vs / 100; lastMenuEncoderPos = -999; }
        else if(currentMenuSelection == 5) { en_hdg = !en_hdg; prefs.putBool("enHdg", en_hdg); lastMenuSelection = -1; }
        else if(currentMenuSelection == 6) { currentState = MENU_EDIT; savedMenuPos = encoderPos; encoderPos = target_hdg; lastMenuEncoderPos = -999; }
        else if(currentMenuSelection == 7) { currentState = MENU_EDIT; savedMenuPos = encoderPos; encoderPos = baro_ref; lastMenuEncoderPos = -999; }
        else if(currentMenuSelection == 8) { use_gps_alt = !use_gps_alt; prefs.putBool("gpsAlt", use_gps_alt); lastMenuSelection = -1; }
        else if(currentMenuSelection == 9) { currentState = MENU_EDIT; savedMenuPos = encoderPos; encoderPos = audio_volume; lastMenuEncoderPos = -999; }
        else if(currentMenuSelection == 10) { currentState = CAL_STEP_1; forceRedraw = true; } 
        else if(currentMenuSelection == 11) { inv_hdg = !inv_hdg; prefs.putBool("invH", inv_hdg); lastMenuSelection = -1; }
        else if(currentMenuSelection == 12) { pitch_cal = raw_pitch; roll_cal = raw_roll; prefs.putFloat("pCal", pitch_cal); prefs.putFloat("rCal", roll_cal); currentState = PFD_NORMAL; forceRedraw = true; }
        else if(currentMenuSelection == 13) { hdg_cal = raw_heading; prefs.putFloat("hCal", hdg_cal); currentState = PFD_NORMAL; forceRedraw = true; }
        else if(currentMenuSelection == 14) { 
          pitch_cal = 0.0; roll_cal = 0.0; hdg_cal = 0.0; 
          mapX = 0; sgnX = 1; mapY = 1; sgnY = 1; mapZ = 2; sgnZ = 1; inv_hdg = false;
          baro_ref = 1013.0; use_gps_alt = false;
          en_spd = true; target_spd = 250.0; v_mode = 1; target_alt = 10000.0; target_vs = 1000.0; en_hdg = true; target_hdg = 360.0;
          audio_volume = 20; 
          
          prefs.putFloat("pCal", 0.0); prefs.putFloat("rCal", 0.0); prefs.putFloat("hCal", 0.0);
          prefs.putInt("mapX", 0); prefs.putInt("sgnX", 1); prefs.putInt("mapY", 1); prefs.putInt("sgnY", 1); prefs.putInt("mapZ", 2); prefs.putInt("sgnZ", 1);
          prefs.putBool("invH", false); prefs.putFloat("qnh", 1013.0); prefs.putBool("gpsAlt", false);
          prefs.putBool("enSpd", true); prefs.putFloat("tgtSpd", 250.0);
          prefs.putInt("vMode", 1); prefs.putFloat("tgtAlt", 10000.0); prefs.putFloat("tgtVs", 1000.0);
          prefs.putBool("enHdg", true); prefs.putFloat("tgtHdg", 360.0); prefs.putInt("vol", 20);
          
          setAudioVolume(20); currentState = PFD_NORMAL; forceRedraw = true; 
        }
        else if(currentMenuSelection == 15) { currentState = MENU_FAULTS; encoderPos = 0; menuScrollOffset = 0; forceRedraw = true; }
      }
    }
    else if (currentState == MENU_FAULTS) {
      drawMenuOverlay();
      if (buttonPressed) {
        buttonPressed = false;
        if(currentMenuSelection == 4) { currentState = MENU_ACTIVE; encoderPos = 15; menuScrollOffset = 10; forceRedraw = true; } 
        else { faults[currentMenuSelection] = !faults[currentMenuSelection]; if (faults[currentMenuSelection]) playWarningAudio(currentMenuSelection + 1); lastMenuSelection = -1; }
      }
    }
    else if (currentState == MENU_EDIT) {
      if (currentMenuSelection == 1) { target_spd = encoderPos; if(target_spd<100) target_spd=100; if(target_spd>350) target_spd=350; encoderPos=target_spd; }
      else if (currentMenuSelection == 3) { target_alt = encoderPos * 100; if(target_alt<0) target_alt=0; if(target_alt>40000) target_alt=40000; encoderPos=target_alt/100; }
      else if (currentMenuSelection == 4) { target_vs = encoderPos * 100; if(target_vs<-6000) target_vs=-6000; if(target_vs>6000) target_vs=6000; encoderPos=target_vs/100; }
      else if (currentMenuSelection == 6) { target_hdg = ((encoderPos % 360) + 360) % 360; encoderPos=target_hdg; }
      else if (currentMenuSelection == 7) { baro_ref = encoderPos; if(baro_ref<800) baro_ref=800; if(baro_ref>1200) baro_ref=1200; encoderPos=baro_ref; }
      else if (currentMenuSelection == 9) { 
        audio_volume = encoderPos; if(audio_volume<0) audio_volume=0; if(audio_volume>30) audio_volume=30; encoderPos=audio_volume; 
        static int last_sent_vol = -1; if (audio_volume != last_sent_vol) { setAudioVolume(audio_volume); last_sent_vol = audio_volume; }
      }

      drawMenuOverlay();
      if (buttonPressed) { 
        buttonPressed = false; currentState = MENU_ACTIVE; 
        if(currentMenuSelection == 1) prefs.putFloat("tgtSpd", target_spd);
        else if(currentMenuSelection == 3) prefs.putFloat("tgtAlt", target_alt);
        else if(currentMenuSelection == 4) prefs.putFloat("tgtVs", target_vs);
        else if(currentMenuSelection == 6) prefs.putFloat("tgtHdg", target_hdg);
        else if(currentMenuSelection == 7) prefs.putFloat("qnh", baro_ref); 
        else if(currentMenuSelection == 9) prefs.putInt("vol", audio_volume);
        encoderPos = savedMenuPos; lastMenuSelection = -1; 
      }
    }
  }
}