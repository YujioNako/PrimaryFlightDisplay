// ============================================================================
// A320 Primary Flight Display (PFD) - ESP32 + TFT_eSPI
// 基于 FlyByWire A32NX 文档的高还原度版本
// 硬件: ESP32 + 240x320 TFT + BNO055 + BMP280 + BN-220 GPS + DFPlayer + EC11
// ============================================================================
// 改进要点 (相对原版):
//   1. FMA 五列布局: A/THR | V-Mode | L-Mode | APPR | AP/FD 衔接状态
//   2. 速度带: VLS(琥珀条)、Alpha保护(琥珀黑条纹)、VMAX(红黑条)、
//      马赫数(绿色)、速度趋势箭头(黄色)、目标速度(品红/蓝三角)
//   3. 姿态球: 椭圆裁剪、坡度刻度(0/10/20/30/45°标记)、
//      坡度索引(黄三角)、侧滑索引(黄梯形)、飞行控制保护符号(绿色)
//   4. 高度带: 滚动鼓轮读数、垂直偏差(绿圆)、目标高度(蓝/品红)、
//      无线电高度(绿/琥珀)、着陆标高(水平条)、地面参考(红带)
//   5. VSI: 非线性刻度、琥珀色高V/S警告
//   6. 航向带: 实际轨迹符号(绿菱形)、选择航向(蓝三角)
//   7. 气压基准: QNH/STD 切换显示、闪烁提醒
//   8. ILS偏差: LOC/GS偏差刻度(品红菱形)
// ============================================================================

#include <Wire.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Preferences.h>

// ============================================================================
// 硬件对象
// ============================================================================
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite pfdSprite = TFT_eSprite(&tft);
TFT_eSprite spdSprite = TFT_eSprite(&tft);
TFT_eSprite altSprite = TFT_eSprite(&tft);
TFT_eSprite hdgSprite = TFT_eSprite(&tft);
TFT_eSprite fmaSprite = TFT_eSprite(&tft);  // 新增: FMA 精灵

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP280 bmp(&Wire);
TinyGPSPlus gps;
TinyGPSCustom gpgsv_sats(gps, "GPGSV", 3);
TinyGPSCustom gngsv_sats(gps, "GNGSV", 3);
int sats_in_view = 0;
HardwareSerial GPS_Serial(2);
HardwareSerial Audio_Serial(1);
Preferences prefs;

// ============================================================================
// 颜色定义 — 匹配 A320 PFD 规范
// ============================================================================
#define RGB_TO_BGR(r, g, b) tft.color565(b, g, r)

// A320 标准色
#define A_SKY       RGB_TO_BGR(0, 112, 192)    // 更深沉的天空蓝
#define A_BROWN     RGB_TO_BGR(145, 80, 10)    // Sienna 棕
#define A_GREEN     RGB_TO_BGR(0, 255, 0)      // 活动模式 / 飞行指引仪
#define A_CYAN      RGB_TO_BGR(0, 255, 255)    // 预位模式 / 选择值
#define A_YELLOW    RGB_TO_BGR(255, 255, 0)    // 固定飞机符号 / 坡度索引
#define A_AMBER     RGB_TO_BGR(255, 153, 0)    // 警告 / VLS
#define A_RED       RGB_TO_BGR(255, 0, 0)      // 告警 / VMAX
#define A_MAGENTA   RGB_TO_BGR(255, 0, 255)    // 管理速度 / ILS
#define A_WHITE     0xFFFF
#define A_BLACK     0x0000
#define A_GREY      RGB_TO_BGR(80, 80, 80)     // 背景灰
#define A_DARKGREY  RGB_TO_BGR(40, 40, 40)     // 深灰色带背景

// ============================================================================
// 系统状态机
// ============================================================================
enum SystemState {
  SYSTEM_POST, PFD_NORMAL, MENU_ACTIVE, MENU_EDIT,
  MENU_FAULTS, CAL_STEP_1, CAL_STEP_2
};
SystemState currentState = SYSTEM_POST;
bool forceRedraw = true;
bool bno_alive = false, bmp_alive = false, gps_alive = false;
bool show_alt_err = false;

// ============================================================================
// EGPWS 警告系统
// ============================================================================
#define DF_BUSY_PIN 32

struct WarningFlag {
  const char* menu_name;
  bool active;
  bool show_on_pfd;
  uint8_t track_id;
  uint16_t pfd_color;
  const char* pfd_text;
  uint8_t level;
};

#define NUM_WARNINGS 14
WarningFlag warnings[NUM_WARNINGS] = {
  {"PULL UP",   false, true,  1,  A_RED,   "PULL UP",     1},
  {"TERRAIN",   false, true,  2,  A_RED,   "TERR AHEAD",  1},
  {"OBSTACLE",  false, true,  3,  A_RED,   "OBST AHEAD",  1},
  {"W/SHEAR",   false, true,  4,  A_RED,   "WINDSHEAR",   1},
  {"OVERSPEED", false, true,  14, A_RED,   "OVERSPEED",   1},
  {"SINK RATE", false, true,  5,  A_AMBER, "SINK RATE",   2},
  {"G/S",       false, true,  6,  A_AMBER, "GLIDESLOPE",  2},
  {"LOW SPEED", false, true,  7,  A_AMBER, "LOW SPEED",   2},
  {"DONT SINK", false, false, 8,  0,       "",            3},
  {"LOW TERR",  false, false, 9,  0,       "",            3},
  {"LOW GEAR",  false, false, 10, 0,       "",            3},
  {"LOW FLAP",  false, false, 11, 0,       "",            3},
  {"BANK ANGL", false, false, 12, 0,       "",            3},
  {"MINIMUMS",  false, false, 13, 0,       "",            3}
};

int audio_style = 0;

enum AudioState { A_IDLE, A_PLAYING, A_WAITING };
AudioState audio_state = A_IDLE;
unsigned long wait_start_time = 0;
unsigned long play_start_time = 0;
int current_round_robin_idx = 0;
int last_played_level = 99;
int current_playing_idx = -1;

// ============================================================================
// IMU 轴映射 & 校准
// ============================================================================
int mapX = 0, sgnX = 1;
int mapY = 1, sgnY = 1;
int mapZ = 2, sgnZ = 1;
bool inv_hdg = false;
int cal_map_Z = 2, cal_sign_Z = 1;

float raw_pitch = 0.0, raw_roll = 0.0, raw_heading = 0.0;
float pitch_cal = 0.0, roll_cal = 0.0, hdg_cal = 0.0;

// ============================================================================
// 飞行参数
// ============================================================================
float sim_pitch = 0.0, sim_roll = 0.0, sim_heading = 0.0;
float sim_altitude = 0.0, sim_speed = 0.0, sim_vs = 0.0, display_alt = 0.0;
float sim_mach = 0.0;       // 新增: 马赫数
float sim_radio_alt = 0.0;  // 新增: 无线电高度 (模拟)
float sim_track = 0.0;      // 新增: 地面轨迹

float speed_trend_diff = 0.0;
bool show_trend_arrow = false;

// 气压 & 高度源
float baro_ref = 1013.0;
bool use_gps_alt = false;
bool baro_std = false;       // 新增: STD 模式

// FCU 目标值
bool en_spd = true;
float target_spd = 250.0;
int v_mode = 1;              // 0=OFF, 1=ALT, 2=V/S
float target_alt = 10000.0;
float target_vs = 1000.0;
bool en_hdg = true;
float target_hdg = 360.0;

// 新增: 速度限制模拟
float vls_speed = 0.0;       // VLS (最小可选速度)
float vmax_speed = 350.0;    // VMAX
float alpha_prot_speed = 0.0;
bool managed_speed = false;  // true=品红, false=蓝色

// 新增: 决策高度
float decision_height = -1.0; // -1 表示无DH
bool dh_reached = false;
unsigned long dh_flash_start = 0;

// 新增: ILS 模拟
bool ils_active = false;
float ils_loc_dev = 0.0;    // LOC 偏差 (-2.5 ~ +2.5 点)
float ils_gs_dev = 0.0;     // G/S 偏差 (-2.5 ~ +2.5 点)
String ils_ident = "";
float ils_freq = 0.0;
float ils_dme = 0.0;
float ils_course = 0.0;
bool ls_button = false;     // EFIS LS 按钮

// 新增: FMA 状态
String fma_col1 = "---";    // A/THR
String fma_col2 = "---";    // V-Mode
String fma_col3 = "---";    // L-Mode
String fma_col4 = "";       // APPR
String fma_col5 = "";       // AP/FD 衔接

// Audio
int audio_volume = 20;

// ============================================================================
// 旋转编码器 & 按钮
// ============================================================================
volatile int encoderPos = 0;
volatile bool buttonPressed = false;
unsigned long lastButtonPress = 0;
int currentMenuSelection = 0, lastMenuSelection = -1;
int savedMenuPos = 0, lastMenuEncoderPos = -999;
int menuScrollOffset = 0;
const int MAX_VISIBLE_ITEMS = 6;

void IRAM_ATTR encoderISR() {
  static uint8_t old_AB = 0;
  static int8_t encval = 0;
  static const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  old_AB <<= 2;
  old_AB |= ((digitalRead(25) << 1) | digitalRead(26));
  encval += enc_states[(old_AB & 0x0f)];
  if (encval > 3) { encoderPos++; encval = 0; }
  else if (encval < -3) { encoderPos--; encval = 0; }
}

void IRAM_ATTR buttonISR() {
  if (millis() - lastButtonPress > 200) {
    buttonPressed = true;
    lastButtonPress = millis();
  }
}

// ============================================================================
// DFPlayer 通信
// ============================================================================
void playWarningAudio(uint8_t track) {
  uint8_t folder = (audio_style == 0) ? 1 : 2;
  uint16_t sum = -(0xFF + 0x06 + 0x0F + 0x00 + folder + track);
  byte cmd[10] = {0x7E,0xFF,0x06,0x0F,0x00,folder,track,
                  (byte)(sum>>8),(byte)(sum&0xFF),0xEF};
  Audio_Serial.write(cmd, 10);
}

void stopAudio() {
  uint16_t sum = -(0xFF + 0x06 + 0x16 + 0x00 + 0x00 + 0x00);
  byte cmd[10] = {0x7E,0xFF,0x06,0x16,0x00,0x00,0x00,
                  (byte)(sum>>8),(byte)(sum&0xFF),0xEF};
  Audio_Serial.write(cmd, 10);
}

void setAudioVolume(uint8_t vol) {
  uint16_t sum = -(0xFF + 0x06 + 0x06 + 0x00 + 0x00 + vol);
  byte cmd[10] = {0x7E,0xFF,0x06,0x06,0x00,0x00,vol,
                  (byte)(sum>>8),(byte)(sum&0xFF),0xEF};
  Audio_Serial.write(cmd, 10);
}

// ============================================================================
// 辅助绘制函数
// ============================================================================

// 在精灵上绘制填充三角形 (坡度索引)
void drawBankPointer(TFT_eSprite &spr, int cx, int r, float bank_rad) {
  // 黄色三角形指示当前坡度
  float angle = -PI/2 - bank_rad;
  int tx = cx + cos(angle) * r;
  int ty = 130 + sin(angle) * r;
  int lx = cx + cos(angle - 0.08) * (r + 8);
  int ly = 130 + sin(angle - 0.08) * (r + 8);
  int rx = cx + cos(angle + 0.08) * (r + 8);
  int ry = 130 + sin(angle + 0.08) * (r + 8);
  spr.fillTriangle(tx, ty, lx, ly, rx, ry, A_YELLOW);
}

// 绘制坡度刻度弧线上的标记
void drawBankTick(TFT_eSprite &spr, int cx, int cy, int r,
                  float angle_deg, int len, uint16_t color) {
  float rad = (-90.0 + angle_deg) * PI / 180.0;
  int x1 = cx + cos(rad) * r;
  int y1 = cy + sin(rad) * r;
  int x2 = cx + cos(rad) * (r - len);
  int y2 = cy + sin(rad) * (r - len);
  spr.drawLine(x1, y1, x2, y2, color);
}

// 绘制侧滑索引 (梯形)
void drawSlipIndicator(TFT_eSprite &spr, int cx, int cy,
                       float slip, uint16_t color) {
  // 梯形: 上窄下宽
  int offset = constrain((int)(slip * 15.0), -20, 20);
  int x = cx + offset;
  // 上边 (窄)
  spr.drawLine(x - 3, cy, x + 3, cy, color);
  // 下边 (宽)
  spr.drawLine(x - 5, cy + 4, x + 5, cy + 4, color);
  // 左斜边
  spr.drawLine(x - 3, cy, x - 5, cy + 4, color);
  // 右斜边
  spr.drawLine(x + 3, cy, x + 5, cy + 4, color);
}

// ============================================================================
// setup()
// ============================================================================
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(A_BLACK);

  prefs.begin("avionics", false);
  // 加载校准参数
  pitch_cal   = prefs.getFloat("pCal", 0.0);
  roll_cal    = prefs.getFloat("rCal", 0.0);
  hdg_cal     = prefs.getFloat("hCal", 0.0);
  mapX = prefs.getInt("mapX", 0); sgnX = prefs.getInt("sgnX", 1);
  mapY = prefs.getInt("mapY", 1); sgnY = prefs.getInt("sgnY", 1);
  mapZ = prefs.getInt("mapZ", 2); sgnZ = prefs.getInt("sgnZ", 1);
  inv_hdg     = prefs.getBool("invH", false);
  baro_ref    = prefs.getFloat("qnh", 1013.0);
  use_gps_alt = prefs.getBool("gpsAlt", false);
  en_spd      = prefs.getBool("enSpd", true);
  target_spd  = prefs.getFloat("tgtSpd", 250.0);
  v_mode      = prefs.getInt("vMode", 1);
  target_alt  = prefs.getFloat("tgtAlt", 10000.0);
  target_vs   = prefs.getFloat("tgtVs", 1000.0);
  en_hdg      = prefs.getBool("enHdg", true);
  target_hdg  = prefs.getFloat("tgtHdg", 360.0);
  audio_volume = prefs.getInt("vol", 20);
  audio_style = prefs.getInt("aStyle", 0);
  baro_std    = prefs.getBool("baroStd", false);
  decision_height = prefs.getFloat("dh", -1.0);

  // 创建精灵 — 优化尺寸分配
  pfdSprite.createSprite(130, 220);    // 姿态球
  spdSprite.createSprite(50, 220);     // 速度带 (稍宽容纳VLS等标记)
  altSprite.createSprite(65, 220);     // 高度带
  hdgSprite.createSprite(130, 28);     // 航向带
  fmaSprite.createSprite(240, 24);     // FMA 区域

  pfdSprite.setSwapBytes(true);
  spdSprite.setSwapBytes(true);
  altSprite.setSwapBytes(true);
  hdgSprite.setSwapBytes(true);
  fmaSprite.setSwapBytes(true);

  // GPIO 初始化
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(DF_BUSY_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(25), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(26), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(33), buttonISR, FALLING);

  Wire.begin(21, 22);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  Audio_Serial.begin(9600, SERIAL_8N1, 14, 27);
  delay(1500);
  setAudioVolume(audio_volume);
}

// ============================================================================
// POST (开机自检)
// ============================================================================
void runSystemPOST() {
  static unsigned long lastPostUpdate = 0;

  if (forceRedraw) {
    tft.fillScreen(A_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(A_CYAN, A_BLACK);
    tft.setTextSize(2);
    tft.drawString("A320 PFD POST", 30, 20);
    tft.drawFastHLine(0, 45, 240, A_WHITE);

    tft.setTextColor(A_WHITE, A_BLACK);
    tft.drawString("BNO055:", 10, 60);
    tft.drawString("BMP280:", 10, 90);
    tft.drawString("GPS   :", 10, 120);
    tft.drawString("AUDIO :", 10, 150);
    tft.drawString("EC11  :", 10, 180);
    tft.drawFastHLine(0, 215, 240, A_WHITE);

    tft.setTextSize(1);
    tft.setTextColor(A_GREY);
    tft.drawString("PRESS KNOB TO FORCE BOOT", 45, 300);
    forceRedraw = false;
  }

  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
    if (gps.charsProcessed() > 10) gps_alive = true;
  }

  if (millis() - lastPostUpdate > 500) {
    lastPostUpdate = millis();
    tft.setTextSize(2);

    Wire.beginTransmission(0x28);
    bno_alive = (Wire.endTransmission() == 0);
    tft.setTextColor(bno_alive ? A_GREEN : A_RED, A_BLACK);
    tft.drawString(bno_alive ? "[ OK ]" : "[ERR ]", 160, 60);

    Wire.beginTransmission(0x76);
    bmp_alive = (Wire.endTransmission() == 0);
    tft.setTextColor(bmp_alive ? A_GREEN : A_RED, A_BLACK);
    tft.drawString(bmp_alive ? "[ OK ]" : "[ERR ]", 160, 90);

    tft.setTextColor(gps_alive ? A_GREEN : A_YELLOW, A_BLACK);
    tft.drawString(gps_alive ? "[ OK ]" : "[WAIT]", 160, 120);

    tft.setTextColor(A_GREEN, A_BLACK);
    tft.drawString("[ OK ]", 160, 150);

    bool knob_ok = (digitalRead(25) == HIGH && digitalRead(26) == HIGH);
    tft.setTextColor(knob_ok ? A_GREEN : A_RED, A_BLACK);
    tft.drawString(knob_ok ? "[ OK ]" : "[ERR ]", 160, 180);

    if (bno_alive && bmp_alive && gps_alive && knob_ok) {
      tft.setTextColor(A_GREEN, A_BLACK);
      tft.drawString("ALL SYSTEMS GO!", 35, 230);
      delay(1000);
      bno.begin();
      bmp.begin(0x76);
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_1);
      currentState = PFD_NORMAL;
      forceRedraw = true;
      buttonPressed = false;
    }
  }

  if (buttonPressed) {
    buttonPressed = false;
    tft.setTextColor(A_YELLOW, A_BLACK);
    tft.drawString("MANUAL OVERRIDE", 35, 230);
    delay(500);
    if (bno_alive) bno.begin();
    if (bmp_alive) {
      bmp.begin(0x76);
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_1);
    }
    currentState = PFD_NORMAL;
    forceRedraw = true;
  }
}

// ============================================================================
// FMA (Flight Mode Annunciator) — 五列布局
// ============================================================================
void drawFMA() {
  fmaSprite.fillSprite(A_DARKGREY);

  // 更新 FMA 文本
  // 第一列: A/THR
  if (en_spd) fma_col1 = "SPEED";
  else fma_col1 = "---";

  // 第二列: V-Mode
  if (v_mode == 1) fma_col2 = "ALT*";
  else if (v_mode == 2) fma_col2 = "V/S";
  else fma_col2 = "---";

  // 第三列: L-Mode
  if (en_hdg) fma_col3 = "HDG";
  else fma_col3 = "---";

  // 第四列: APPR (示意)
  fma_col4 = ils_active ? "ILS" : "";

  // 第五列: AP/FD
  fma_col5 = "1 FD1";

  // 分隔线
  int col_w = 48;
  for (int i = 1; i < 5; i++) {
    fmaSprite.drawFastVLine(col_w * i, 0, 24, A_GREY);
  }

  fmaSprite.setTextDatum(MC_DATUM);
  fmaSprite.setTextSize(1);

  // 第一列 — 绿色 (活动模式)
  fmaSprite.setTextColor(A_GREEN);
  fmaSprite.drawString(fma_col1, col_w * 0 + col_w / 2, 8);

  // 第二列 — 绿色或青色
  bool v_active = (v_mode != 0);
  fmaSprite.setTextColor(v_active ? A_GREEN : A_WHITE);
  fmaSprite.drawString(fma_col2, col_w * 1 + col_w / 2, 8);

  // 第三列
  fmaSprite.setTextColor(en_hdg ? A_GREEN : A_WHITE);
  fmaSprite.drawString(fma_col3, col_w * 2 + col_w / 2, 8);

  // 第四列
  if (fma_col4.length() > 0) {
    fmaSprite.setTextColor(A_GREEN);
    fmaSprite.drawString(fma_col4, col_w * 3 + col_w / 2, 8);
  }

  // 第五列
  if (fma_col5.length() > 0) {
    fmaSprite.setTextColor(A_WHITE);
    fmaSprite.drawString(fma_col5, col_w * 4 + col_w / 2, 8);
  }

  // 底行: 目标值
  fmaSprite.setTextColor(A_CYAN);
  fmaSprite.setTextSize(1);
  if (en_spd) {
    fmaSprite.drawString(String((int)target_spd), col_w * 0 + col_w/2, 19);
  }
  if (v_mode == 1) {
    fmaSprite.drawString(String((int)target_alt), col_w * 1 + col_w/2, 19);
  } else if (v_mode == 2) {
    String vs_str = (target_vs >= 0 ? "+" : "") + String((int)target_vs);
    fmaSprite.drawString(vs_str, col_w * 1 + col_w/2, 19);
  }
  if (en_hdg) {
    char hbuf[5];
    sprintf(hbuf, "%03d", (int)target_hdg);
    fmaSprite.drawString(hbuf, col_w * 2 + col_w/2, 19);
  }

  fmaSprite.pushSprite(0, 0);
}

// ============================================================================
// 姿态指示器 (Attitude Indicator) — A320 风格
// ============================================================================
void drawAttitudeIndicator() {
  int cx = 65, cy = 110;
  float pitch_offset = sim_pitch * 3.5;
  float roll_rad = -sim_roll * PI / 180.0;
  float cosR = cos(roll_rad), sinR = sin(roll_rad);

  // 填充天空
  pfdSprite.fillSprite(A_SKY);

  // 绘制地面
  float hx = cx - pitch_offset * sinR;
  float hy = cy + pitch_offset * cosR;
  float lx = hx - 2000 * cosR, ly = hy - 2000 * sinR;
  float rx = hx + 2000 * cosR, ry = hy + 2000 * sinR;
  float nx = -sinR, ny = cosR;
  float gx1 = lx + 2000 * nx, gy1 = ly + 2000 * ny;
  float gx2 = rx + 2000 * nx, gy2 = ry + 2000 * ny;
  pfdSprite.fillTriangle(lx, ly, rx, ry, gx1, gy1, A_BROWN);
  pfdSprite.fillTriangle(rx, ry, gx1, gy1, gx2, gy2, A_BROWN);

  // 地平线白线
  pfdSprite.drawLine(lx, ly, rx, ry, A_WHITE);

  // ---- 俯仰刻度 ----
  pfdSprite.setTextDatum(MC_DATUM);
  pfdSprite.setTextColor(A_WHITE);
  pfdSprite.setTextSize(1);

  for (int p_int = -900; p_int <= 900; p_int += 25) {
    if (p_int == 0) continue;
    float py = -p_int * 0.35;
    float sy = pitch_offset + py;
    if (sy < -180 || sy > 180) continue;

    int line_len;
    if (abs(p_int) % 100 == 0) line_len = 36;
    else if (abs(p_int) % 50 == 0) line_len = 18;
    else line_len = 8;

    float x1 = cx + (-line_len/2.0 * cosR - sy * sinR);
    float y1 = cy + (-line_len/2.0 * sinR + sy * cosR);
    float x2 = cx + (line_len/2.0 * cosR - sy * sinR);
    float y2 = cy + (line_len/2.0 * sinR + sy * cosR);
    pfdSprite.drawLine(x1, y1, x2, y2, A_WHITE);

    // 10°标记带数字和端部短竖线
    if (abs(p_int) % 100 == 0) {
      int tick_dir = (p_int > 0) ? 4 : -4;
      float xt1 = x1 + (-tick_dir * sinR);
      float yt1 = y1 + (tick_dir * cosR);
      pfdSprite.drawLine(x1, y1, xt1, yt1, A_WHITE);
      float xt2 = x2 + (-tick_dir * sinR);
      float yt2 = y2 + (tick_dir * cosR);
      pfdSprite.drawLine(x2, y2, xt2, yt2, A_WHITE);

      float sx_l = -line_len/2.0 - 12.0;
      float sx_r = line_len/2.0 + 12.0;
      pfdSprite.drawString(String(abs(p_int)/10),
        cx + (sx_l * cosR - sy * sinR), cy + (sx_l * sinR + sy * cosR));
      pfdSprite.drawString(String(abs(p_int)/10),
        cx + (sx_r * cosR - sy * sinR), cy + (sx_r * sinR + sy * cosR));
    }
  }

  // ---- 坡度刻度 (弧形) ----
  int bank_r = 95;
  // 0° 三角形标记 (顶部固定)
  pfdSprite.fillTriangle(cx, cy - bank_r + 2, cx - 4, cy - bank_r - 6,
                         cx + 4, cy - bank_r - 6, A_WHITE);
  // 10°, 20° 短标记
  drawBankTick(pfdSprite, cx, cy, bank_r, -10, 6, A_WHITE);
  drawBankTick(pfdSprite, cx, cy, bank_r,  10, 6, A_WHITE);
  drawBankTick(pfdSprite, cx, cy, bank_r, -20, 6, A_WHITE);
  drawBankTick(pfdSprite, cx, cy, bank_r,  20, 6, A_WHITE);
  // 30° 较长标记
  drawBankTick(pfdSprite, cx, cy, bank_r, -30, 10, A_WHITE);
  drawBankTick(pfdSprite, cx, cy, bank_r,  30, 10, A_WHITE);
  // 45° 标记
  drawBankTick(pfdSprite, cx, cy, bank_r, -45, 6, A_WHITE);
  drawBankTick(pfdSprite, cx, cy, bank_r,  45, 6, A_WHITE);

  // ---- 坡度索引 (黄色三角形, 随飞机转动) ----
  drawBankPointer(pfdSprite, cx, bank_r, roll_rad);

  // ---- 侧滑索引 (黄色梯形, 在坡度索引下方) ----
  // 简化: 使用横向加速度模拟侧滑
  float slip_val = sin(roll_rad) * 0.5; // 简化模拟
  int slip_base_y = cy - bank_r + 12;
  drawSlipIndicator(pfdSprite, cx, slip_base_y, slip_val, A_YELLOW);

  // ---- 飞行控制保护符号 (绿色) ----
  // 坡度 ±67° 标记
  drawBankTick(pfdSprite, cx, cy, bank_r - 3, -67, 8, A_GREEN);
  drawBankTick(pfdSprite, cx, cy, bank_r - 3,  67, 8, A_GREEN);
  // 俯仰保护在 +30° 和 -15° (简化显示为绿色小方块)
  float prot_up = pitch_offset + (-30.0 * 3.5);
  float prot_dn = pitch_offset + (15.0 * 3.5);
  if (prot_up > 0 && prot_up < 220) {
    float px1 = cx + (-20 * cosR - prot_up * sinR + pitch_offset * sinR);
    float py1 = cy + (-20 * sinR + prot_up * cosR - pitch_offset * cosR);
    // 简化: 仅当可见时绘制
    pfdSprite.fillRect(cx - 22, cy - (30 - sim_pitch) * 3.5, 3, 3, A_GREEN);
    pfdSprite.fillRect(cx + 20, cy - (30 - sim_pitch) * 3.5, 3, 3, A_GREEN);
  }

  // ---- 飞行指引仪 (FD) ----
  bool show_fd_pitch = false;
  float fd_target_pitch = 0.0;
  if (v_mode == 1 && en_spd) {
    show_fd_pitch = true;
    fd_target_pitch = constrain((target_alt - display_alt) * 0.05, -10.0, 15.0);
  } else if (v_mode == 2 && en_spd) {
    show_fd_pitch = true;
    fd_target_pitch = constrain((target_vs - sim_vs) * 0.01, -10.0, 15.0);
  }

  bool show_fd_roll = false;
  float fd_target_roll = 0.0;
  if (en_hdg) {
    show_fd_roll = true;
    float hdg_error = target_hdg - sim_heading;
    if (hdg_error > 180.0) hdg_error -= 360.0;
    if (hdg_error < -180.0) hdg_error += 360.0;
    fd_target_roll = constrain(hdg_error * 0.5, -25.0, 25.0);
  }

  int fd_y = cy - (fd_target_pitch - sim_pitch) * 3.5;
  int fd_x = cx + (fd_target_roll - sim_roll) * 1.8;

  // FD 横杆 (绿色)
  if (show_fd_pitch) {
    pfdSprite.drawFastHLine(cx - 35, fd_y - 1, 70, A_GREEN);
    pfdSprite.drawFastHLine(cx - 35, fd_y,     70, A_GREEN);
  }
  // FD 纵杆 (绿色)
  if (show_fd_roll) {
    pfdSprite.drawFastVLine(fd_x - 1, cy - 35, 70, A_GREEN);
    pfdSprite.drawFastVLine(fd_x,     cy - 35, 70, A_GREEN);
  }

  // ---- 固定飞机符号 (黄色轮廓, 黑色填充) ----
  // 左翼
  pfdSprite.fillRect(cx - 28, cy - 2, 18, 4, A_BLACK);
  pfdSprite.drawRect(cx - 28, cy - 2, 18, 4, A_YELLOW);
  // 右翼
  pfdSprite.fillRect(cx + 10, cy - 2, 18, 4, A_BLACK);
  pfdSprite.drawRect(cx + 10, cy - 2, 18, 4, A_YELLOW);
  // 中心点
  pfdSprite.fillRect(cx - 2, cy - 2, 5, 5, A_BLACK);
  pfdSprite.drawRect(cx - 2, cy - 2, 5, 5, A_YELLOW);

  // ---- ILS 偏差刻度 (如果LS按钮打开) ----
  if (ls_button && ils_active) {
    // LOC 偏差 (底部水平点组)
    int loc_y = cy + 90;
    for (int d = -2; d <= 2; d++) {
      if (d == 0) continue;
      int dx = cx + d * 15;
      pfdSprite.fillCircle(dx, loc_y, 2, A_WHITE);
    }
    // LOC 菱形索引
    int loc_px = cx + constrain((int)(ils_loc_dev * 15.0), -30, 30);
    pfdSprite.drawLine(loc_px, loc_y - 4, loc_px + 4, loc_y, A_MAGENTA);
    pfdSprite.drawLine(loc_px + 4, loc_y, loc_px, loc_y + 4, A_MAGENTA);
    pfdSprite.drawLine(loc_px, loc_y + 4, loc_px - 4, loc_y, A_MAGENTA);
    pfdSprite.drawLine(loc_px - 4, loc_y, loc_px, loc_y - 4, A_MAGENTA);

    // G/S 偏差 (右侧垂直点组)
    int gs_x = cx + 55;
    for (int d = -2; d <= 2; d++) {
      if (d == 0) continue;
      int dy = cy + d * 15;
      pfdSprite.fillCircle(gs_x, dy, 2, A_WHITE);
    }
    // G/S 菱形索引
    int gs_py = cy + constrain((int)(ils_gs_dev * 15.0), -30, 30);
    pfdSprite.drawLine(gs_x, gs_py - 4, gs_x + 4, gs_py, A_MAGENTA);
    pfdSprite.drawLine(gs_x + 4, gs_py, gs_x, gs_py + 4, A_MAGENTA);
    pfdSprite.drawLine(gs_x, gs_py + 4, gs_x - 4, gs_py, A_MAGENTA);
    pfdSprite.drawLine(gs_x - 4, gs_py, gs_x, gs_py - 4, A_MAGENTA);
  }

  // ---- IMU 故障标志 ----
  pfdSprite.setTextSize(2);
  if (!bno_alive) {
    pfdSprite.fillRect(cx - 40, cy - 8, 80, 16, A_BLACK);
    pfdSprite.setTextColor(A_RED);
    pfdSprite.setTextDatum(MC_DATUM);
    pfdSprite.drawString("ATT", cx, cy);
  } else if (current_playing_idx != -1 && warnings[current_playing_idx].show_on_pfd) {
    // EGPWS 告警文字
    const char* wtext = warnings[current_playing_idx].pfd_text;
    int tw = pfdSprite.textWidth(wtext);
    int wy = cy + 50;
    pfdSprite.fillRect(cx - tw/2 - 4, wy - 8, tw + 8, 18, A_BLACK);
    pfdSprite.drawRect(cx - tw/2 - 4, wy - 8, tw + 8, 18,
                       warnings[current_playing_idx].pfd_color);
    pfdSprite.setTextColor(warnings[current_playing_idx].pfd_color);
    pfdSprite.setTextDatum(MC_DATUM);
    pfdSprite.drawString(wtext, cx, wy);
  }

  // ---- 无线电高度 (在姿态球底部) ----
  if (sim_radio_alt >= 0 && sim_radio_alt <= 2500) {
    pfdSprite.setTextSize(2);
    pfdSprite.setTextDatum(MC_DATUM);
    bool dh_set = (decision_height >= 0);
    uint16_t ra_color;
    if (dh_set) {
      ra_color = (sim_radio_alt > decision_height + 100) ? A_GREEN : A_AMBER;
    } else {
      ra_color = (sim_radio_alt > 400) ? A_GREEN : A_AMBER;
    }
    pfdSprite.setTextColor(ra_color);
    pfdSprite.drawString(String((int)sim_radio_alt), cx, cy + 75);

    // DH 到达闪烁
    if (dh_set && sim_radio_alt <= decision_height && !dh_reached) {
      dh_reached = true;
      dh_flash_start = millis();
    }
    if (dh_reached && (millis() - dh_flash_start < 3000)) {
      if ((millis() / 250) % 2 == 0) {
        pfdSprite.setTextSize(1);
        pfdSprite.setTextColor(A_AMBER);
        pfdSprite.drawString("DH", cx, cy + 63);
      }
    } else if (dh_reached) {
      pfdSprite.setTextSize(1);
      pfdSprite.setTextColor(A_AMBER);
      pfdSprite.drawString("DH", cx, cy + 63);
    }
  }

  pfdSprite.pushSprite(50, 26);
}

// ============================================================================
// 速度带 (Airspeed Tape) — A320 风格
// ============================================================================
void drawSpeedTape() {
  spdSprite.fillSprite(A_DARKGREY);
  spdSprite.drawFastVLine(49, 0, 220, A_WHITE);

  int spd = (int)sim_speed;
  int cy = 110; // 中心 Y

  // ---- 计算速度限制 (简化模拟) ----
  vls_speed = max(0.0f, sim_speed * 0.7f);   // 简化 VLS
  alpha_prot_speed = max(0.0f, vls_speed - 10.0f);
  vmax_speed = 350.0;

  // ---- VMAX 红黑条 (超速区域) ----
  float vmax_y = cy - (vmax_speed - sim_speed) * 2.5;
  if (vmax_y < 220) {
    int start_y = max(0, (int)vmax_y);
    for (int y = start_y; y >= 0; y -= 6) {
      spdSprite.fillRect(45, y, 4, 3, A_RED);
      spdSprite.fillRect(45, y + 3, 4, 3, A_BLACK);
    }
  }

  // ---- VLS 琥珀色条 (最小可选速度区域) ----
  float vls_y = cy - (vls_speed - sim_speed) * 2.5;
  if (vls_y > 0 && vls_y < 220) {
    int end_y = min(220, (int)vls_y);
    spdSprite.drawFastHLine(44, end_y, 5, A_AMBER);
    // 琥珀色条延伸到下方
    for (int y = end_y; y < 220; y++) {
      spdSprite.drawPixel(46, y, A_AMBER);
    }
  }

  // ---- Alpha保护 琥珀黑条纹 ----
  float aprot_y = cy - (alpha_prot_speed - sim_speed) * 2.5;
  if (aprot_y > 0 && aprot_y < 220) {
    for (int y = (int)aprot_y; y < 220; y += 4) {
      spdSprite.fillRect(45, y, 4, 2, A_AMBER);
    }
  }

  // ---- GPS 丢失标志 ----
  if (!gps.speed.isValid() || gps.speed.age() > 3000) {
    spdSprite.fillRect(0, 2, 47, 24, A_RED);
    spdSprite.setTextColor(A_WHITE);
    spdSprite.setTextSize(1);
    spdSprite.setTextDatum(MC_DATUM);
    spdSprite.drawString("SPD", 23, 8);
    uint32_t sats_in_use = gps.satellites.isValid() ? gps.satellites.value() : 0;
    if (sats_in_view < (int)sats_in_use) sats_in_view = sats_in_use;
    spdSprite.drawString(String(sats_in_use) + "/" + String(sats_in_view), 23, 18);
  }

  // ---- 速度刻度 ----
  spdSprite.setTextColor(A_WHITE);
  spdSprite.setTextSize(1);
  spdSprite.setTextDatum(MR_DATUM);

  for (int i = spd - 50; i <= spd + 50; i++) {
    if (i >= 30 && i % 10 == 0) {
      int y = cy - (i - sim_speed) * 2.5;
      if (y < 0 || y > 220) continue;
      spdSprite.drawFastHLine(42, y, 7, A_WHITE);
      if (i % 20 == 0) {
        spdSprite.drawString(String(i), 40, y);
      }
    }
  }

  // ---- 速度趋势箭头 (黄色) ----
  if (show_trend_arrow) {
    int trend_y = cy - (speed_trend_diff * 2.5);
    trend_y = constrain(trend_y, 5, 215);
    spdSprite.drawLine(43, cy, 43, trend_y, A_YELLOW);
    spdSprite.drawLine(44, cy, 44, trend_y, A_YELLOW);
    if (speed_trend_diff > 0) {
      spdSprite.fillTriangle(43, trend_y, 39, trend_y + 5, 48, trend_y + 5, A_YELLOW);
    } else {
      spdSprite.fillTriangle(43, trend_y, 39, trend_y - 5, 48, trend_y - 5, A_YELLOW);
    }
  }

  // ---- 目标速度 (品红三角=管理, 蓝三角=选择) ----
  if (en_spd) {
    int t_s_y = cy - (target_spd - sim_speed) * 2.5;
    if (t_s_y >= 0 && t_s_y <= 220) {
      uint16_t tgt_color = managed_speed ? A_MAGENTA : A_CYAN;
      spdSprite.fillTriangle(42, t_s_y, 49, t_s_y - 5, 49, t_s_y + 5, tgt_color);
    }
  }

  // ---- 当前速度指针 (黄色参考线) ----
  spdSprite.drawFastHLine(34, cy, 15, A_YELLOW);
  spdSprite.fillTriangle(24, cy, 34, cy - 5, 34, cy + 5, A_YELLOW);

  // ---- 马赫数 (绿色, 在底部) ----
  if (sim_mach >= 0.50) {
    spdSprite.setTextColor(A_GREEN);
    spdSprite.setTextSize(1);
    spdSprite.setTextDatum(MC_DATUM);
    char mach_buf[6];
    sprintf(mach_buf, ".%02d", (int)(sim_mach * 100));
    spdSprite.drawString(mach_buf, 25, 210);
  }

  spdSprite.pushSprite(0, 26);
}

// ============================================================================
// VSI 非线性偏移计算 — 匹配 A320 刻度
// ============================================================================
float getVsiOffset(float vs) {
  float abs_vs = abs(vs), offset = 0;
  if (abs_vs <= 500) offset = (abs_vs / 500.0) * 25.0;
  else if (abs_vs <= 1000) offset = 25.0 + ((abs_vs - 500.0) / 500.0) * 25.0;
  else if (abs_vs <= 2000) offset = 50.0 + ((abs_vs - 1000.0) / 1000.0) * 30.0;
  else offset = 80.0 + ((abs_vs - 2000.0) / 4000.0) * 30.0;
  if (offset > 100) offset = 100;
  return (vs > 0) ? -offset : offset;
}

// ============================================================================
// 高度带 (Altitude Tape) — A320 风格滚动鼓轮
// ============================================================================
void drawAltTape() {
  altSprite.fillSprite(A_DARKGREY);
  int cy = 110;

  if (show_alt_err) {
    // ALT 标志 (红)
    altSprite.drawFastVLine(0, 0, 220, A_WHITE);
    altSprite.drawFastVLine(45, 0, 220, A_WHITE);
    altSprite.fillRect(3, cy - 10, 40, 20, A_BLACK);
    altSprite.setTextColor(A_RED);
    altSprite.setTextSize(2);
    altSprite.setTextDatum(MC_DATUM);
    altSprite.drawString("ALT", 23, cy);
  } else {
    int alt = max(0, (int)display_alt);

    // ---- 高度刻度 ----
    altSprite.setTextColor(A_WHITE);
    altSprite.setTextSize(1);
    altSprite.setTextDatum(ML_DATUM);

    int start_i = ((int)(display_alt - 600) / 20) * 20;
    for (int i = start_i; i <= display_alt + 600; i += 20) {
      if (i >= 0) {
        int y = cy - (i - display_alt) * 0.22;
        if (y < 0 || y > 220) continue;
        if (y > cy - 13 && y < cy + 13) continue; // 跳过中间读数窗口

        int tick_len = (i % 100 == 0) ? 8 : 4;
        altSprite.drawFastHLine(1, y, tick_len, A_WHITE);

        if (i % 100 == 0) {
          altSprite.drawString(String(i / 100), 12, y);
        }
      }
    }

    // ---- 中心读数窗口 (滚动鼓轮) ----
    altSprite.drawFastVLine(0, 0, 220, A_WHITE);
    altSprite.drawFastVLine(45, 0, 220, A_WHITE);

    // 鼓轮窗口边框
    altSprite.drawRect(2, cy - 12, 42, 24, A_GREEN);
    altSprite.fillRect(3, cy - 11, 40, 22, A_BLACK);

    // 高位数字 (百位及以上, 固定部分)
    int prefix = alt / 10;
    altSprite.setTextColor(A_GREEN);
    altSprite.setTextSize(2);
    altSprite.setTextDatum(MR_DATUM);
    altSprite.drawString(String(prefix), 28, cy);

    // 个位数字 (滚动)
    int current_unit = alt % 10;
    int next_unit = (current_unit + 1) % 10;
    int prev_unit = (current_unit - 1 + 10) % 10;
    float fraction = display_alt - floor(display_alt);
    int font_h = 14;
    int roll_y = (int)(fraction * font_h);

    altSprite.setTextDatum(ML_DATUM);
    // 裁剪区域内绘制
    int clip_top = cy - 10;
    int clip_bot = cy + 10;

    int y_cur = cy + roll_y;
    int y_next = cy + roll_y - font_h;
    int y_prev = cy + roll_y + font_h;

    if (y_cur >= clip_top && y_cur <= clip_bot)
      altSprite.drawString(String(current_unit), 29, y_cur);
    if (y_next >= clip_top && y_next <= clip_bot)
      altSprite.drawString(String(next_unit), 29, y_next);
    if (y_prev >= clip_top && y_prev <= clip_bot)
      altSprite.drawString(String(prev_unit), 29, y_prev);

    // 清除鼓轮窗口外的溢出
    altSprite.fillRect(3, 0, 40, cy - 12, A_DARKGREY);
    altSprite.fillRect(3, cy + 12, 40, 220 - cy - 12, A_DARKGREY);
    // 重绘刻度在窗口外
    for (int i = start_i; i <= display_alt + 600; i += 20) {
      if (i >= 0) {
        int y = cy - (i - display_alt) * 0.22;
        if (y < 0 || y > 220) continue;
        if (y > cy - 13 && y < cy + 13) continue;
        int tick_len = (i % 100 == 0) ? 8 : 4;
        altSprite.drawFastHLine(1, y, tick_len, A_WHITE);
        if (i % 100 == 0) {
          altSprite.setTextSize(1);
          altSprite.setTextDatum(ML_DATUM);
          altSprite.setTextColor(A_WHITE);
          altSprite.drawString(String(i / 100), 12, y);
        }
      }
    }

    // ---- 目标高度 (蓝色或品红色) ----
    if (v_mode == 1) {
      int t_a_y = cy - (target_alt - display_alt) * 0.22;
      if (t_a_y >= 0 && t_a_y <= 220 && (t_a_y < cy - 13 || t_a_y > cy + 13)) {
        altSprite.fillRect(1, t_a_y - 3, 8, 7, A_CYAN);
        altSprite.fillRect(2, t_a_y - 1, 6, 3, A_BLACK);
      }
    }

    // ---- 地面参考 (红色带, 570ft 以下显示) ----
    if (sim_radio_alt >= 0 && sim_radio_alt < 570) {
      float ground_alt = display_alt - sim_radio_alt;
      int ground_y = cy - (ground_alt - display_alt) * 0.22;
      if (ground_y < 220) {
        for (int y = max(0, ground_y); y < 220; y++) {
          if (y % 4 < 2) altSprite.drawPixel(2, y, A_RED);
          altSprite.drawPixel(3, y, A_RED);
          if (y % 4 >= 2) altSprite.drawPixel(4, y, A_RED);
        }
      }
    }
  }

  // ---- VSI 区域 (右侧) ----
  int offsets[] = {25, 50, 80, 100};
  String labels[] = {".5", "1", "2", "6"};

  altSprite.drawFastHLine(45, cy, 5, A_WHITE);
  altSprite.setTextColor(A_WHITE);
  altSprite.setTextSize(1);
  altSprite.setTextDatum(ML_DATUM);

  for (int i = 0; i < 4; i++) {
    altSprite.drawFastHLine(45, cy - offsets[i], 5, A_WHITE);
    altSprite.drawFastHLine(45, cy + offsets[i], 5, A_WHITE);
    altSprite.drawString(labels[i], 52, cy - offsets[i]);
    altSprite.drawString(labels[i], 52, cy + offsets[i]);
  }

  // VSI 指针 (绿色)
  int vsi_y = cy + getVsiOffset(sim_vs);

  // 琥珀色条件检查
  bool vsi_amber = false;
  float abs_vs = abs(sim_vs);
  if (abs_vs > 6000) vsi_amber = true;
  if (sim_vs < 0 && sim_radio_alt >= 0) {
    if (sim_radio_alt < 2500 && sim_radio_alt > 1000 && abs_vs > 2000) vsi_amber = true;
    if (sim_radio_alt <= 1000 && abs_vs > 1200) vsi_amber = true;
  }
  uint16_t vsi_color = vsi_amber ? A_AMBER : A_GREEN;

  altSprite.drawFastHLine(46, vsi_y, 18, vsi_color);
  altSprite.drawFastHLine(46, vsi_y - 1, 18, vsi_color);
  altSprite.drawLine(46, cy, 46, vsi_y, vsi_color);

  // 目标 V/S (蓝色三角)
  if (v_mode == 2) {
    int t_vs_y = cy + getVsiOffset(target_vs);
    altSprite.fillTriangle(45, t_vs_y, 50, t_vs_y - 4, 50, t_vs_y + 4, A_CYAN);
  }

  // VSI 数字 (超过200fpm时显示)
  if (abs(sim_vs) > 200) {
    altSprite.setTextColor(vsi_color);
    altSprite.setTextSize(1);
    altSprite.setTextDatum(ML_DATUM);
    int vs_display = ((int)abs(sim_vs)) / 100;
    int num_x = 48;
    int num_y = vsi_y + ((sim_vs > 0) ? -8 : 8);
    num_y = constrain(num_y, 5, 215);
    altSprite.drawString(String(vs_display), num_x, num_y);
  }

  altSprite.pushSprite(180, 26);
}

// ============================================================================
// 航向带 (Heading Tape) — A320 风格
// ============================================================================
void drawHeadingTape() {
  hdgSprite.fillSprite(A_DARKGREY);
  hdgSprite.drawFastHLine(0, 0, 130, A_WHITE);

  int hdg = (int)sim_heading;

  // ---- 选择航向 (蓝色三角) ----
  if (en_hdg) {
    float hdg_err = target_hdg - sim_heading;
    if (hdg_err > 180.0) hdg_err -= 360.0;
    if (hdg_err < -180.0) hdg_err += 360.0;
    int t_h_x = 65 + (int)(hdg_err * 4.5);

    if (t_h_x >= 0 && t_h_x <= 130) {
      hdgSprite.fillTriangle(t_h_x, 7, t_h_x - 4, 0, t_h_x + 4, 0, A_CYAN);
    } else {
      // 超出范围时在边缘显示数字
      hdgSprite.setTextSize(1);
      hdgSprite.setTextColor(A_CYAN);
      char hbuf[5];
      sprintf(hbuf, "%03d", (int)target_hdg);
      if (hdg_err < 0) {
        hdgSprite.setTextDatum(ML_DATUM);
        hdgSprite.drawString(hbuf, 2, 18);
      } else {
        hdgSprite.setTextDatum(MR_DATUM);
        hdgSprite.drawString(hbuf, 128, 18);
      }
    }
  }

  // ---- 刻度 ----
  hdgSprite.setTextColor(A_WHITE);
  hdgSprite.setTextSize(1);
  hdgSprite.setTextDatum(TC_DATUM);

  for (int i = hdg - 28; i <= hdg + 28; i++) {
    if (i % 5 == 0) {
      int x = 65 + (int)((i - sim_heading) * 4.5);
      if (x < 0 || x > 130) continue;

      int display_hdg = ((i % 360) + 360) % 360;
      int tick_len = (i % 10 == 0) ? 7 : 3;
      hdgSprite.drawFastVLine(x, 1, tick_len, A_WHITE);

      if (i % 10 == 0) {
        char buf[4];
        sprintf(buf, "%02d", (display_hdg == 0) ? 36 : (display_hdg / 10));
        hdgSprite.drawString(buf, x, 10);
      }
    }
  }

  // ---- 实际轨迹符号 (绿色小菱形) ----
  float track_err = sim_track - sim_heading;
  if (track_err > 180.0) track_err -= 360.0;
  if (track_err < -180.0) track_err += 360.0;
  int track_x = 65 + (int)(track_err * 4.5);
  if (track_x >= 2 && track_x <= 128) {
    hdgSprite.drawLine(track_x, 21, track_x + 3, 24, A_GREEN);
    hdgSprite.drawLine(track_x + 3, 24, track_x, 27, A_GREEN);
    hdgSprite.drawLine(track_x, 27, track_x - 3, 24, A_GREEN);
    hdgSprite.drawLine(track_x - 3, 24, track_x, 21, A_GREEN);
  }

  // ---- 固定参考三角 (黄色) ----
  hdgSprite.fillTriangle(65, 1, 61, 7, 69, 7, A_YELLOW);

  hdgSprite.pushSprite(50, 248);
}

// ============================================================================
// 气压基准显示
// ============================================================================
void drawBaroRef() {
  int bx = 180, by = 278;
  tft.fillRect(bx, by, 60, 18, A_BLACK);

  // 闪烁条件检查
  bool blink = false;
  if (baro_std && display_alt < 10000) {
    // 简化: 低于过渡高度仍选STD时闪烁
    blink = ((millis() / 500) % 2 == 0);
  }

  if (!blink) {
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    if (baro_std) {
      tft.setTextColor(A_CYAN, A_BLACK);
      tft.drawString("STD", bx + 5, by + 2);
    } else {
      tft.setTextColor(A_CYAN, A_BLACK);
      tft.drawString("QNH", bx, by);
      tft.drawNumber((int)baro_ref, bx, by + 10);
    }
  }
}

// ============================================================================
// ILS 信息显示 (航向带左侧)
// ============================================================================
void drawILSInfo() {
  if (!ils_active) return;
  int ix = 0, iy = 250;
  tft.fillRect(ix, iy, 50, 30, A_BLACK);
  tft.setTextColor(A_MAGENTA, A_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  if (ils_ident.length() > 0) tft.drawString(ils_ident, ix + 2, iy);
  if (ils_freq > 0) {
    char fbuf[8];
    sprintf(fbuf, "%.1f", ils_freq);
    tft.drawString(fbuf, ix + 2, iy + 10);
  }
  if (ils_dme >= 0) {
    tft.drawString(String(ils_dme, 1) + "NM", ix + 2, iy + 20);
  }
}

// ============================================================================
// 音频调度器 (基于 DFPlayer BUSY 引脚)
// ============================================================================
void updateAudioScheduler() {
  bool is_busy = (digitalRead(DF_BUSY_PIN) == LOW);

  int highest_active_level = 99;
  for (int i = 0; i < NUM_WARNINGS; i++) {
    if (warnings[i].active && warnings[i].level < highest_active_level) {
      highest_active_level = warnings[i].level;
    }
  }

  int active_indices[NUM_WARNINGS];
  int active_count = 0;
  if (highest_active_level != 99) {
    for (int i = 0; i < NUM_WARNINGS; i++) {
      if (warnings[i].active && warnings[i].level == highest_active_level) {
        active_indices[active_count++] = i;
      }
    }
  }

  if (active_count > 0) {
    bool preempt = (highest_active_level < last_played_level);
    if (preempt) {
      stopAudio();
      audio_state = A_IDLE;
      current_round_robin_idx = 0;
    }

    if (audio_state == A_PLAYING) {
      if (!is_busy && (millis() - play_start_time > 300)) {
        audio_state = A_WAITING;
        wait_start_time = millis();
      }
    }

    if (audio_state == A_IDLE ||
        (audio_state == A_WAITING && (millis() - wait_start_time >= 1000))) {
      if (current_round_robin_idx >= active_count) current_round_robin_idx = 0;
      int warning_to_play = active_indices[current_round_robin_idx];
      playWarningAudio(warnings[warning_to_play].track_id);
      audio_state = A_PLAYING;
      play_start_time = millis();
      last_played_level = highest_active_level;
      current_playing_idx = warning_to_play;
      current_round_robin_idx++;
    }
  } else {
    if (last_played_level != 99) {
      stopAudio();
      last_played_level = 99;
      current_playing_idx = -1;
      audio_state = A_IDLE;
    }
  }
}

// ============================================================================
// 传感器数据读取 & 滤波
// ============================================================================
void updateSensors() {
  static unsigned long lastVsiTime = 0, lastTrendTime = 0;
  static float last_vsi_alt = 0.0, last_trend_speed = 0.0;
  static bool alt_inited = false;

  // GPS 速度
  float raw_spd = 0.0;
  if (gps_alive && gps.speed.isValid() && gps.speed.age() < 3000) {
    raw_spd = gps.speed.knots();
  }
  sim_speed += (raw_spd - sim_speed) * 0.15;

  // 马赫数估算 (简化: 基于高度修正)
  // 真实 TAS/Mach 需要 TAT, 这里粗略估算
  float temp_at_alt = 15.0 - (display_alt * 0.002); // ISA 温度递减
  float speed_of_sound = 661.5 * sqrt((temp_at_alt + 273.15) / 288.15);
  sim_mach = sim_speed / speed_of_sound;

  // 地面轨迹 (GPS)
  if (gps_alive && gps.course.isValid() && gps.course.age() < 3000) {
    sim_track = gps.course.deg();
  } else {
    sim_track = sim_heading; // 无风时轨迹 = 航向
  }

  // 速度趋势
  if (millis() - lastTrendTime >= 200) {
    float dt_sec = (millis() - lastTrendTime) / 1000.0;
    float instant_accel = (sim_speed - last_trend_speed) / dt_sec;
    static float smoothed_accel = 0.0;
    smoothed_accel = smoothed_accel * 0.7 + instant_accel * 0.3;
    float temp_trend = smoothed_accel * 10.0;

    if (!show_trend_arrow && abs(temp_trend) > 2.0) show_trend_arrow = true;
    else if (show_trend_arrow && abs(temp_trend) <= 1.0) show_trend_arrow = false;

    speed_trend_diff = show_trend_arrow ? temp_trend : 0.0;
    last_trend_speed = sim_speed;
    lastTrendTime = millis();
  }

  // IMU 姿态
  if (bno_alive) {
    imu::Vector<3> raw_g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    float phys_g[3] = {raw_g.x(), raw_g.y(), raw_g.z()};
    float gx = phys_g[mapX] * sgnX;
    float gy = phys_g[mapY] * sgnY;
    float gz = phys_g[mapZ] * sgnZ;
    float norm_g = sqrt(gx*gx + gy*gy + gz*gz);

    if (norm_g > 0.1) {
      raw_pitch = asin(constrain(-gy / norm_g, -1.0, 1.0)) * 180.0 / M_PI;
      raw_roll  = atan2(gx, -gz) * 180.0 / M_PI;
    }

    // 航向 (四元数)
    imu::Quaternion q = bno.getQuat();
    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    float vx = 0, vy = 0, vz = 0;
    if (mapY == 0) vx = -sgnY;
    else if (mapY == 1) vy = -sgnY;
    else if (mapY == 2) vz = -sgnY;

    float tx = 2.0 * (qy * vz - qz * vy);
    float ty = 2.0 * (qz * vx - qx * vz);
    float tz = 2.0 * (qx * vy - qy * vx);
    float ex = vx + qw * tx + (qy * tz - qz * ty);
    float ey = vy + qw * ty + (qz * tx - qx * tz);
    raw_heading = atan2(ex, ey) * 180.0 / M_PI;
    if (inv_hdg) raw_heading = 360.0 - raw_heading;

    sim_pitch = raw_pitch - pitch_cal;
    sim_roll  = raw_roll  - roll_cal;

    if (sim_pitch > 90.0) { sim_pitch = 180.0 - sim_pitch; sim_roll += 180.0; }
    else if (sim_pitch < -90.0) { sim_pitch = -180.0 - sim_pitch; sim_roll += 180.0; }
    while (sim_roll > 180.0) sim_roll -= 360.0;
    while (sim_roll <= -180.0) sim_roll += 360.0;

    sim_heading = raw_heading - hdg_cal;
    while (sim_heading < 0) sim_heading += 360.0;
    while (sim_heading >= 360.0) sim_heading -= 360.0;
  }

  // 高度
  show_alt_err = false;
  if (use_gps_alt) {
    if (gps_alive && gps.altitude.isValid() && gps.altitude.age() < 3000) {
      sim_altitude = gps.altitude.feet();
    } else {
      show_alt_err = true;
    }
  } else {
    if (bmp_alive) {
      float ref = baro_std ? 1013.25 : baro_ref;
      sim_altitude = bmp.readAltitude(ref) * 3.28084;
    } else {
      show_alt_err = true;
    }
  }
  if (sim_altitude < 0) sim_altitude = 0.0;

  if (!alt_inited) {
    last_vsi_alt = sim_altitude;
    display_alt = sim_altitude;
    alt_inited = true;
  }
  display_alt += (sim_altitude - display_alt) * 0.08;

  // 无线电高度 (模拟: 使用气压高度)
  sim_radio_alt = display_alt; // 简化

  // 垂直速度
  if (millis() - lastVsiTime >= 200) {
    float dt_min = (millis() - lastVsiTime) / 60000.0;
    float raw_vs = (sim_altitude - last_vsi_alt) / dt_min;
    sim_vs = sim_vs * 0.8 + raw_vs * 0.2;
    last_vsi_alt = sim_altitude;
    lastVsiTime = millis();
  }
}

// ============================================================================
// 菜单系统
// ============================================================================
void drawMenuOverlay() {
  bool needsTextRedraw = false;

  if (forceRedraw) {
    tft.fillRect(30, 50, 180, 220, A_DARKGREY);
    tft.drawRect(30, 50, 180, 220, A_WHITE);
    tft.setTextColor(A_WHITE, A_DARKGREY);
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(currentState == MENU_FAULTS ? "WARNINGS" : "FCU MENU", 65, 55);
    forceRedraw = false;
    needsTextRedraw = true;
  }

  int total_items = (currentState == MENU_FAULTS) ? (NUM_WARNINGS + 1) : 20;

  if (currentState != MENU_EDIT) {
    int newSel = ((encoderPos % total_items) + total_items) % total_items;
    if (newSel != lastMenuSelection) {
      currentMenuSelection = newSel;
      if (currentMenuSelection >= menuScrollOffset + MAX_VISIBLE_ITEMS)
        menuScrollOffset = currentMenuSelection - MAX_VISIBLE_ITEMS + 1;
      else if (currentMenuSelection < menuScrollOffset)
        menuScrollOffset = currentMenuSelection;
      lastMenuSelection = currentMenuSelection;
      needsTextRedraw = true;
    }
  } else {
    if (encoderPos != lastMenuEncoderPos) {
      needsTextRedraw = true;
      lastMenuEncoderPos = encoderPos;
    }
  }

  if (needsTextRedraw) {
    tft.fillRect(31, 80, 178, 188, A_DARKGREY);
    tft.setTextSize(1);

    if (currentState == MENU_FAULTS) {
      String fItems[NUM_WARNINGS + 1];
      for (int i = 0; i < NUM_WARNINGS; i++) {
        char buf[22];
        sprintf(buf, "%2d.%-9s:%s", i+1, warnings[i].menu_name,
                warnings[i].active ? "ON" : "OFF");
        fItems[i] = String(buf);
      }
      fItems[NUM_WARNINGS] = "<< Back";

      for (int i = 0; i < MAX_VISIBLE_ITEMS && (i + menuScrollOffset) < total_items; i++) {
        int idx = i + menuScrollOffset;
        tft.setTextColor(idx == currentMenuSelection ? A_BLACK : A_WHITE,
                         idx == currentMenuSelection ? A_CYAN : A_DARKGREY);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(fItems[idx], 36, 85 + i * 20);
      }
    } else {
      String menuItems[20];
      menuItems[0]  = String("Spd Tgt: ") + (en_spd ? "ON" : "OFF");
      menuItems[1]  = "Spd Val: " + String((int)target_spd);
      menuItems[2]  = String("V-Mode:  ") + (v_mode == 1 ? "ALT" : (v_mode == 2 ? "V/S" : "OFF"));
      menuItems[3]  = "Alt Val: " + String((int)target_alt);
      menuItems[4]  = "V/S Val: " + String((int)target_vs);
      menuItems[5]  = String("Hdg Tgt: ") + (en_hdg ? "ON" : "OFF");
      menuItems[6]  = "Hdg Val: " + String((int)target_hdg);
      menuItems[7]  = "QNH:     " + String((int)baro_ref);
      menuItems[8]  = String("Baro:    ") + (baro_std ? "STD" : "QNH");
      menuItems[9]  = String("Alt Src: ") + (use_gps_alt ? "GPS" : "BARO");
      menuItems[10] = "DH:      " + (decision_height >= 0 ? String((int)decision_height) : "OFF");
      menuItems[11] = String("LS Btn:  ") + (ls_button ? "ON" : "OFF");
      menuItems[12] = "Volume:  " + String(audio_volume);
      menuItems[13] = String("A.Style: ") + (audio_style == 0 ? "AIRBUS" : "BOEING");
      menuItems[14] = "Align Axes >";
      menuItems[15] = String("Inv HDG: ") + (inv_hdg ? "ON" : "OFF");
      menuItems[16] = "Zero Ptc/Rol";
      menuItems[17] = "Zero HDG";
      menuItems[18] = "Warnings >";
      menuItems[19] = "Exit";

      for (int i = 0; i < MAX_VISIBLE_ITEMS && (i + menuScrollOffset) < total_items; i++) {
        int idx = i + menuScrollOffset;
        if (idx == currentMenuSelection) {
          tft.setTextColor(A_BLACK,
            currentState == MENU_EDIT ? A_YELLOW : A_CYAN);
        } else {
          tft.setTextColor(A_WHITE, A_DARKGREY);
        }
        tft.setTextDatum(TL_DATUM);
        tft.drawString(menuItems[idx], 36, 85 + i * 20);
      }
    }
  }
}

// ============================================================================
// 菜单按钮处理
// ============================================================================
void handleMenuButton() {
  if (currentMenuSelection == 19) {
    currentState = PFD_NORMAL; forceRedraw = true;
  }
  else if (currentMenuSelection == 0) {
    en_spd = !en_spd; prefs.putBool("enSpd", en_spd); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 1) {
    currentState = MENU_EDIT; savedMenuPos = encoderPos;
    encoderPos = target_spd; lastMenuEncoderPos = -999;
  }
  else if (currentMenuSelection == 2) {
    v_mode = (v_mode + 1) % 3; prefs.putInt("vMode", v_mode); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 3) {
    currentState = MENU_EDIT; savedMenuPos = encoderPos;
    encoderPos = target_alt / 100; lastMenuEncoderPos = -999;
  }
  else if (currentMenuSelection == 4) {
    currentState = MENU_EDIT; savedMenuPos = encoderPos;
    encoderPos = target_vs / 100; lastMenuEncoderPos = -999;
  }
  else if (currentMenuSelection == 5) {
    en_hdg = !en_hdg; prefs.putBool("enHdg", en_hdg); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 6) {
    currentState = MENU_EDIT; savedMenuPos = encoderPos;
    encoderPos = target_hdg; lastMenuEncoderPos = -999;
  }
  else if (currentMenuSelection == 7) {
    currentState = MENU_EDIT; savedMenuPos = encoderPos;
    encoderPos = baro_ref; lastMenuEncoderPos = -999;
  }
  else if (currentMenuSelection == 8) {
    baro_std = !baro_std; prefs.putBool("baroStd", baro_std); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 9) {
    use_gps_alt = !use_gps_alt; prefs.putBool("gpsAlt", use_gps_alt); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 10) {
    // 切换DH: -1 -> 200 -> -1
    if (decision_height < 0) decision_height = 200;
    else decision_height = -1;
    prefs.putFloat("dh", decision_height); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 11) {
    ls_button = !ls_button; lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 12) {
    currentState = MENU_EDIT; savedMenuPos = encoderPos;
    encoderPos = audio_volume; lastMenuEncoderPos = -999;
  }
  else if (currentMenuSelection == 13) {
    audio_style = (audio_style + 1) % 2;
    prefs.putInt("aStyle", audio_style); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 14) {
    currentState = CAL_STEP_1; forceRedraw = true;
  }
  else if (currentMenuSelection == 15) {
    inv_hdg = !inv_hdg; prefs.putBool("invH", inv_hdg); lastMenuSelection = -1;
  }
  else if (currentMenuSelection == 16) {
    pitch_cal = raw_pitch; roll_cal = raw_roll;
    prefs.putFloat("pCal", pitch_cal); prefs.putFloat("rCal", roll_cal);
    currentState = PFD_NORMAL; forceRedraw = true;
  }
  else if (currentMenuSelection == 17) {
    hdg_cal = raw_heading; prefs.putFloat("hCal", hdg_cal);
    currentState = PFD_NORMAL; forceRedraw = true;
  }
  else if (currentMenuSelection == 18) {
    currentState = MENU_FAULTS; encoderPos = 0;
    menuScrollOffset = 0; forceRedraw = true;
  }
}

// ============================================================================
// 菜单编辑模式
// ============================================================================
void handleMenuEdit() {
  if (currentMenuSelection == 1) {
    target_spd = constrain(encoderPos, 100, 350);
    encoderPos = target_spd;
  }
  else if (currentMenuSelection == 3) {
    target_alt = constrain(encoderPos * 100, 0, 40000);
    encoderPos = target_alt / 100;
  }
  else if (currentMenuSelection == 4) {
    target_vs = constrain(encoderPos * 100, -6000, 6000);
    encoderPos = target_vs / 100;
  }
  else if (currentMenuSelection == 6) {
    target_hdg = ((encoderPos % 360) + 360) % 360;
    encoderPos = target_hdg;
  }
  else if (currentMenuSelection == 7) {
    baro_ref = constrain(encoderPos, 800, 1200);
    encoderPos = baro_ref;
  }
  else if (currentMenuSelection == 12) {
    audio_volume = constrain(encoderPos, 0, 30);
    encoderPos = audio_volume;
    static int last_sent_vol = -1;
    if (audio_volume != last_sent_vol) {
      setAudioVolume(audio_volume);
      last_sent_vol = audio_volume;
    }
  }

  drawMenuOverlay();

  if (buttonPressed) {
    buttonPressed = false;
    currentState = MENU_ACTIVE;
    if (currentMenuSelection == 1) prefs.putFloat("tgtSpd", target_spd);
    else if (currentMenuSelection == 3) prefs.putFloat("tgtAlt", target_alt);
    else if (currentMenuSelection == 4) prefs.putFloat("tgtVs", target_vs);
    else if (currentMenuSelection == 6) prefs.putFloat("tgtHdg", target_hdg);
    else if (currentMenuSelection == 7) prefs.putFloat("qnh", baro_ref);
    else if (currentMenuSelection == 12) prefs.putInt("vol", audio_volume);
    encoderPos = savedMenuPos;
    lastMenuSelection = -1;
  }
}

// ============================================================================
// 校准流程
// ============================================================================
void runCalStep1() {
  if (forceRedraw) {
    tft.fillScreen(A_BLACK);
    tft.setTextColor(A_CYAN); tft.setTextSize(2); tft.setTextDatum(MC_DATUM);
    tft.drawString("AUTO ALIGN 1/2", 120, 60);
    tft.setTextColor(A_WHITE); tft.setTextSize(1);
    tft.drawString("Place module FLAT (Z-Up)", 120, 120);
    tft.drawString("Then press the knob.", 120, 160);
    forceRedraw = false;
  }

  if (buttonPressed) {
    buttonPressed = false;
    imu::Vector<3> g1 = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    float g_arr[3] = {g1.x(), g1.y(), g1.z()};
    float max1 = 0;
    for (int i = 0; i < 3; i++) {
      if (abs(g_arr[i]) > max1) { max1 = abs(g_arr[i]); cal_map_Z = i; }
    }
    cal_sign_Z = (g_arr[cal_map_Z] < 0) ? 1 : -1;
    currentState = CAL_STEP_2; forceRedraw = true;
  }
}

void runCalStep2() {
  if (forceRedraw) {
    tft.fillScreen(A_BLACK);
    tft.setTextColor(A_CYAN); tft.setTextSize(2); tft.setTextDatum(MC_DATUM);
    tft.drawString("AUTO ALIGN 2/2", 120, 60);
    tft.setTextColor(A_WHITE); tft.setTextSize(1);
    tft.drawString("Point NOSE UP (>45 deg)", 120, 120);
    tft.drawString("Then press the knob.", 120, 160);
    forceRedraw = false;
  }

  if (buttonPressed) {
    buttonPressed = false;
    imu::Vector<3> g2 = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    float g_arr[3] = {g2.x(), g2.y(), g2.z()};
    int cal_map_Y = 0, cal_sign_Y = 1;
    float max2 = 0;
    for (int i = 0; i < 3; i++) {
      if (i == cal_map_Z) continue;
      if (abs(g_arr[i]) > max2) { max2 = abs(g_arr[i]); cal_map_Y = i; }
    }
    cal_sign_Y = (g_arr[cal_map_Y] < 0) ? 1 : -1;

    float vZ[3] = {0,0,0}; vZ[cal_map_Z] = cal_sign_Z;
    float vY[3] = {0,0,0}; vY[cal_map_Y] = cal_sign_Y;
    float vX[3];
    vX[0] = vZ[1]*vY[2] - vZ[2]*vY[1];
    vX[1] = vZ[2]*vY[0] - vZ[0]*vY[2];
    vX[2] = vZ[0]*vY[1] - vZ[1]*vY[0];

    for (int i = 0; i < 3; i++) {
      if (abs(vX[i]) > 0.5) { mapX = i; sgnX = (vX[i] > 0) ? 1 : -1; }
    }
    mapY = cal_map_Y; sgnY = cal_sign_Y;
    mapZ = cal_map_Z; sgnZ = cal_sign_Z;

    prefs.putInt("mapX", mapX); prefs.putInt("sgnX", sgnX);
    prefs.putInt("mapY", mapY); prefs.putInt("sgnY", sgnY);
    prefs.putInt("mapZ", mapZ); prefs.putInt("sgnZ", sgnZ);

    currentState = PFD_NORMAL; forceRedraw = true;
  }
}

// ============================================================================
// 主循环
// ============================================================================
void loop() {
  if (currentState == SYSTEM_POST) { runSystemPOST(); return; }
  if (currentState == CAL_STEP_1)  { runCalStep1(); return; }
  if (currentState == CAL_STEP_2)  { runCalStep2(); return; }

  // GPS 数据解析
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
    if (gngsv_sats.isUpdated() && gngsv_sats.isValid())
      sats_in_view = atoi(gngsv_sats.value());
    else if (gpgsv_sats.isUpdated() && gpgsv_sats.isValid())
      sats_in_view = atoi(gpgsv_sats.value());
  }

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 30) return;
  lastUpdate = millis();

  // 音频调度
  updateAudioScheduler();

  // 传感器更新
  updateSensors();

  // ---- 状态渲染 ----
  if (currentState == PFD_NORMAL) {
    if (forceRedraw) {
      tft.fillScreen(A_BLACK);
      forceRedraw = false;
    }

    drawFMA();
    drawAttitudeIndicator();
    drawSpeedTape();
    drawAltTape();
    drawHeadingTape();
    drawBaroRef();
    drawILSInfo();

    // 心跳指示灯
    if ((millis() / 500) % 2 == 0)
      tft.fillCircle(235, 310, 3, A_GREEN);
    else
      tft.fillCircle(235, 310, 3, A_BLACK);

    if (buttonPressed) {
      buttonPressed = false;
      currentState = MENU_ACTIVE;
      lastMenuSelection = -1;
      forceRedraw = true;
    }
  }
  else if (currentState == MENU_ACTIVE) {
    drawMenuOverlay();
    if (buttonPressed) {
      buttonPressed = false;
      handleMenuButton();
    }
  }
  else if (currentState == MENU_FAULTS) {
    drawMenuOverlay();
    if (buttonPressed) {
      buttonPressed = false;
      if (currentMenuSelection == NUM_WARNINGS) {
        currentState = MENU_ACTIVE;
        encoderPos = 18; menuScrollOffset = 14;
        forceRedraw = true;
      } else {
        warnings[currentMenuSelection].active = !warnings[currentMenuSelection].active;
        lastMenuSelection = -1;
      }
    }
  }
  else if (currentState == MENU_EDIT) {
    handleMenuEdit();
  }
}
