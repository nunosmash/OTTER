/* 

1. 입력 및 상태 제어 (Input & Navigation)
handlePageNavigation() : 페이지 엔코더 입력 처리 및 페이지 전환/확정 로직 제어
processMainEncoders() : 4개 메인 엔코더의 회전 감지, 가속도 계산 및 MIDI 전송
handleGlobalButton() : 상단 글로벌 버튼의 상태 체크 및 지정된 MIDI CC 전송
handleEncoderSwitches() : 엔코더 클릭 감지 (일반 모드: 레이어 전환 / 설정 모드: 값 변경)
handleMidiInput() : 외부에서 들어오는 MIDI 메시지(Note/CC) 수신 및 처리

2. UI 및 피드백 (Display & Feedback)
updateUIDisplay() : 각 노브/버튼의 변경된 상태를 TFT 화면에 점진적으로 갱신
updateZone() : 화면의 특정 구역(상단/하단)을 다시 그려주는 실질적인 UI 렌더링
drawStaticLayout() : 현재 페이지의 전체 틀(가이드라인, 라벨명 등)을 화면에 출력
drawPreviewAnimated() : 페이지 전환 시 부드러운 애니메이션 효과 구현
drawSettingUI() : 글로벌 설정 변경 시 나타나는 중앙 팝업 UI 생성
handleVisualFlash() : 센터 도달이나 한계값 터치 시 LED/UI가 깜빡이는 효과 제어

3. 시스템 유지관리 (System & Logic)
handleSystemMaintenance() : 절전 모드, 진동 정지 타이머, 시리얼 통신, 페이지 자동 저장 관리
loadConfig() : 플래시 메모리(LittleFS)에서 설정값 및 마지막 페이지 로드
saveSystemSettings() : 현재 설정 및 페이지 번호를 JSON 파일로 플래시 메모리에 저장
syncEncodersToPage() : 페이지 변경 시 해당 페이지의 MIDI 값들을 엔코더 변수에 동기화
stabilizeAfterPageChange() : 페이지 전환 직후 원치 않는 입력을 방지하기 위한 안정화 로직
userActivity() : 모든 사용자 조작 시 호출되어 절전 모드 타이머를 초기화

*/

#include <Adafruit_TinyUSB.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Fonts/FreeSans9pt7b.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <hardware/watchdog.h>
#include "pico/bootrom.h"

#define MY_USB_MANUFACTURER "ASH SOUND WORKS"
#define MY_USB_PRODUCT "OTTER"   // 기기 B는 B로 변경
#define MY_USB_PID 0x044B        // 기기 B는 0x044B로 변경
#define MY_USB_SERIAL "MC-E44s"  // 시리얼도 다르게!

// 1. TFT 및 핀 설정
#define TFT_CS 17
#define TFT_DC 20
#define TFT_RST 21
#define TFT_BL 22
#define TFT_SCL 18
#define TFT_SDA 19

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(284, 22);

// -------LED 설정 수정
#define LED_PIN 27
#define LED_COUNT 9  // 2020(4) + 5050(4) + 5050(1)
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define MIDI_UART uart1
#define MIDI_TX_PIN 8


// 인덱스 정의 (가독성용)
#define LED_IDX_BTN_START 0  // 0, 1, 2, 3 (2020 스트립)
#define LED_IDX_ENC_START 4  // 4, 5, 6, 7 (엔코더 위 5050)
#define LED_IDX_GLOBAL 8     // 8 (페이지 엔코더 옆 5050)

#define NUM_ENCODERS 4
const uint8_t pinA[NUM_ENCODERS] = { 1, 4, 7, 10 };
const uint8_t pinB[NUM_ENCODERS] = { 0, 3, 6, 9 };
const uint8_t pinSW[NUM_ENCODERS] = { 2, 5, 26, 11 };

#define PAGE_ENC_A 13
#define PAGE_ENC_B 12
#define GLOBAL_BTN_PIN 14

#define VIB_MOTOR_PIN 15  // 진동 모터 핀 번호

unsigned long pressStartTime = 0;  // 버튼이 눌리기 시작한 시간 저장
bool isEnteringBootloader = false;

unsigned long vibStopTime = 0;
bool isVibrating = false;

Adafruit_USBD_MIDI usb_midi;
// 컴파일러가 USB 장치 정보를 읽어갈 때 사용하는 콜백 함수들
const uint16_t *tud_descriptor_product_cb(void) {
  static uint16_t _desc_str[32];
  uint8_t chr_len = strlen(MY_USB_PRODUCT);
  if (chr_len > 31) chr_len = 31;  // 안전장치 추가
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_len + 2);
  for (uint8_t i = 0; i < chr_len; i++) _desc_str[i + 1] = MY_USB_PRODUCT[i];
  return _desc_str;
}

const uint16_t *tud_descriptor_manufacturer_cb(void) {
  static uint16_t _desc_str[32];
  uint8_t chr_len = strlen(MY_USB_MANUFACTURER);
  if (chr_len > 31) chr_len = 31;
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_len + 2);
  for (uint8_t i = 0; i < chr_len; i++) _desc_str[i + 1] = MY_USB_MANUFACTURER[i];
  return _desc_str;
}

const uint16_t *tud_descriptor_serial_cb(void) {
  static uint16_t _desc_str[32];
  uint8_t chr_len = strlen(MY_USB_SERIAL);
  if (chr_len > 31) chr_len = 31;
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_len + 2);
  for (uint8_t i = 0; i < chr_len; i++) _desc_str[i + 1] = MY_USB_SERIAL[i];
  return _desc_str;
}

// 2. 데이터 구조
struct Pot {
  char knobName[16];
  uint8_t knobChannel, knobCC;
  char knobColorCode;
  char btnName[16];
  uint8_t btnChannel, btnCC;
  char btnColorCode;
  bool isToggle;
  char mode;
  uint8_t activeLayer;
  uint8_t knobValue;
  uint8_t btnCcValue;
  bool btnState;
  bool needsUpdateUpper;
  bool needsUpdateLower;

  Pot() {
    memset(knobName, 0, 16);
    memset(btnName, 0, 16);
    knobValue = 64;
    btnCcValue = 64;
    btnState = false;
    activeLayer = 0;
    needsUpdateUpper = true;
    needsUpdateLower = true;
  }
};

struct PageConfig {
  char title[20];
  char colorCode;
};
#define NUM_PAGES 4
#define POTS_PER_PAGE 4

Pot pots[NUM_PAGES][POTS_PER_PAGE];
PageConfig pageSettings[NUM_PAGES];

struct GlobalBtnConfig {
  char name[16];
  uint8_t channel, cc;
  bool isToggle, state;
};
GlobalBtnConfig globalBtn;

// ==========================================
// [OTTER System Configurations & Variables]
// ==========================================

// --- 시스템 및 절전 설정 ---
unsigned long DIM_TIMEOUT = 300000;        // 화면 자동 절전 시간 (ms, 기본 5분)
bool startInStudio = false;                // 시작 시 스튜디오 모드 여부
int currentBrightness = 0;                 // 현재 밝기 (0: Full, 255: Dark)
int targetBrightness = 0;                  // 목표 밝기 (페이드 효과용)
unsigned long lastFadeTime = 0;            // 밝기 전환 타이머
unsigned long lastAnyInteractionTime = 0;  // 마지막 조작 시간 (절전 체크용)
bool isLedOn = true;                       // LED 활성화 상태 플래그

// --- 페이지 및 상태 관리 ---
bool justChangedPage = false;                  // 페이지 전환 직후 플래그
int currentPage = 0, previewPage = 0;          // 현재 페이지 및 전환 예고 페이지
bool isStudioMode = false;                     // 운영 모드 (False: Live, True: Studio)
bool isPagePending = false;                    // 페이지 확정 대기 상태
unsigned long lastPageMoveTime = 0;            // 마지막 페이지 이동 시간
unsigned long lastUiUpdateTime = 0;            // UI 갱신 타이머
const unsigned long PAGE_CONFIRM_DELAY = 800;  // 페이지 확정 대기 시간 (ms)
unsigned long lastPageChangeTime = 0;          // 마지막 페이지 변경 시점 저장
bool pageNeedsSave = false;                    // 저장 필요 여부 플래그

// --- 메인 엔코더 데이터 (4개) ---
volatile uint8_t encState[NUM_ENCODERS] = { 0 };
long targetLong[NUM_ENCODERS] = { 6400, 6400, 6400, 6400 };   // 목표 MIDI값 (x100 해상도)
long currentLong[NUM_ENCODERS] = { 6400, 6400, 6400, 6400 };  // 현재 MIDI값 (x100 해상도)
int currentMidi[NUM_ENCODERS] = { 64, 64, 64, 64 };           // 현재 MIDI 정수값 (0-127)
unsigned long lastDetentTime[NUM_ENCODERS] = { 0 };           // 마지막 엔코더 물리 멈춤 시간
unsigned long lastTickTime[NUM_ENCODERS] = { 0 };             // 마지막 인터럽트 발생 시간
float speedBlend[NUM_ENCODERS] = { 0 };                       // 가속도 계산용 블렌드값

// --- 인코더 잠금(Lock) 메커니즘 ---
bool isRotLocked[NUM_ENCODERS] = { false };  // 현재 엔코더 잠금 상태
int lockAccumulator[NUM_ENCODERS] = { 0 };   // 잠금 해제를 위한 회전 누적치
const int ROT_THRESHOLD = 4;                 // 잠금 해제 임계값 (클릭 수)
const unsigned long LOCK_TIMEOUT = 4000;     // 자동 잠금 대기 시간 (4초)

// --- 시각적 피드백 (LED/LCD Flash) ---
uint8_t flashCount[8] = { 0 };           // 깜빡임 횟수 (0-3: 상단, 4-7: 하단)
unsigned long lastFlashTime[8] = { 0 };  // 마지막 깜빡임 시간
bool isFlashOn[8] = { false };           // 현재 깜빡임 상태 (On/Off)

// --- 엔코더 스위치 및 디바운스 ---
bool lastSwState[NUM_ENCODERS] = { HIGH, HIGH, HIGH, HIGH };
unsigned long swDebounceTime[NUM_ENCODERS] = { 0 };
const unsigned long BTN_DEBOUNCE_TIME = 25;  // 버튼 디바운스 시간 (ms)

// --- UI 및 내비게이션 관련 ---
int currentUiIndex = 0;                // 루프 내 점진적 UI 갱신 인덱스
volatile int pageStepAccumulator = 0;  // 페이지 엔코더 회전 누적
volatile int lastPageEncA = HIGH;      // 페이지 엔코더 A상 상태
const int PAGE_ENC_THRESHOLD = 2;      // 페이지 전환 감도
int animDirection = 1;                 // 페이지 전환 애니메이션 방향
int lastDrawnPage = -1;                // 마지막으로 그려진 페이지 번호

// --- MIDI 입출력 보조 ---
uint8_t lastSentValue[128] = { 255 };     // 마지막으로 전송된 CC 데이터 캐시
unsigned long lastMoveTime[8] = { 0 };    // 각 파라미터별 마지막 조작 시간
unsigned long globalBtnDebounceTime = 0;  // 글로벌 버튼 디바운스 타이머
int liveStepAccum[NUM_ENCODERS] = { 0 };  // 라이브 모드 전용 보간 누적기
unsigned long ignoreMidiUntil = 0;        // 수신 무시 타이머 (충돌 방지)

// --- 스마트 스무딩 (저속 구간) ---
int sentMidi[NUM_ENCODERS];                // 실제 전송된 마지막 MIDI 값
unsigned long nextStepTime[NUM_ENCODERS];  // 다음 보간 스텝 예정 시간
bool isSlowMove[NUM_ENCODERS];             // 저속 회전 상태 판별
static unsigned long lastMidiSendTime[NUM_ENCODERS];

// --- 하드웨어 밝기 및 LED 설정 ---
uint8_t globalBrightness = 40;  // NeoPixel 밝기 (기본 15)
const uint8_t BRIGHTNESS_MIN = 10;
const uint8_t BRIGHTNESS_MAX = 20;
float currentDimRatio = 0.15f;      // 절전 시 밝기 배율
float globalLedMultiplier = 0.10f;  // 전체 LED 광량 계수

// --- 시스템 설정 모드 (Setting UI) ---
bool isSettingTimeout = false;                                // 설정 모드 활성화 상태
unsigned long settingDisplayTime = 0;                         // 설정 화면 유지 시간 체크
const int TIMEOUT_STEPS[] = { 1, 3, 5, 10, 20, 30, 40, 60 };  // 절전 시간 옵션
int currentTimeoutIdx = 1;                                    // 현재 선택된 절전 시간 인덱스
bool settingsChanged = false;                                 // 설정 변경 여부 (저장 트리거)

// --- 피드백 상태 (진동/센터) ---
static bool isAtLimit[8] = { false };   // 끝값(0/127) 도달 상태
static bool isAtCenter[8] = { false };  // 센터(64) 진입 상태
int vibStrengthMax = 160;               // 진동 모터 최대 강도
float rainbowHue = 0;                   // 6번 LED 레인보우 효과용

// ==========================================

// ===============================
// 유틸리티 및 그래픽 루틴
// ===============================

uint16_t getColor(char c) {
  switch (c) {
    case 'R': return 0xF000;  // Neon Crimson (밝고 진한 레드)
    case 'G': return 0x0740;  // Neo Emerald (생기 있는 에메랄드)
    case 'B': return 0x341F;  // Electric Sky (밝은 일렉트릭 블루)
    case 'Y': return 0xFFE0;  // Luminous Amber (빛나는 앰버)
    case 'M': return 0xF01F;  // Vivid Magenta (팝한 핑크 마젠타)
    case 'C': return 0x07FF;  // Ice Cyan (네온 청록)
    case 'O': return 0xFD00;  // Neon Orange (선명한 오렌지)
    case 'W': return 0xFFFF;  // Pure White
    default: return 0x07FF;
  }
}

// 네오픽셀용 RGB 색상을 반환하는 함수
uint32_t getLEDColor(char c) {
  switch (c) {
    case 'R':
      return pixels.Color(255, 5, 5);  // Neon Crimson 느낌: 순수 레드보다 살짝 핑크빛이 도는 진한 레드
    case 'G':
      return pixels.Color(40, 240, 40);  // Neo Emerald 느낌: 초록에 청록을 살짝 섞어 생기 있는 에메랄드 톤
    case 'B':
      return pixels.Color(0, 50, 255);  // Electric Sky 느낌: 너무 어두운 파랑이 아닌, 밝고 시원한 스카이 블루
    case 'Y':
      return pixels.Color(255, 190, 0);  // Luminous Amber 느낌: 노랑보다 오렌지 쪽으로 더 깊은 따뜻한 앰버
    case 'M':
      return pixels.Color(200, 10, 250);  // Vivid Magenta 느낌: 보라색보다는 팝한 핫핑크 톤
    case 'C':
      return pixels.Color(0, 255, 150);  // Ice Cyan 느낌: 형광 느낌이 나는 시원한 민트-사이언 톤
    case 'O':
      return pixels.Color(255, 100, 5);  // Neon Orange 느낌: 붉은 기가 도는 선명하고 진한 오렌지
    case 'W':
      return pixels.Color(140, 140, 140);  // 보정된 White: 눈부심을 줄이고 살짝 푸른기를 돌아 더 깨끗해 보이는 흰색
    default:
      return pixels.Color(0, 255, 150);  // 기본값 (Ice Cyan 계열)
  }
}

uint16_t getDimColor(uint16_t color) {
  return (((color & 0xF800) >> 1) & 0x7800) | (((color & 0x07E0) >> 1) & 0x03E0) | (((color & 0x001F) >> 1) & 0x000F);
}

uint16_t getContrastColor(char colorCode) {
  if (colorCode == 'W' || colorCode == 'Y' || colorCode == 'C' || colorCode == 'A' || colorCode == 'G') return 0x0000;
  return 0xFFFF;
}

// 함수 위치는 setup() 위쪽 어디든 상관없습니다.
void pageEncISR() {
  int a = digitalRead(PAGE_ENC_A);
  int b = digitalRead(PAGE_ENC_B);

  if (a != lastPageEncA) {
    if (b != a) pageStepAccumulator++;
    else pageStepAccumulator--;
  }
  lastPageEncA = a;
}

void triggerFlash(int i) {
  flashCount[i] = 6;  // 6번 반전 = 3번 깜빡임
  lastFlashTime[i] = millis();
  isFlashOn[i] = true;

  // UI 갱신 예약 (이미 있는 코드일 것입니다)
  if (i < 4) pots[currentPage][i].needsUpdateUpper = true;
  else pots[currentPage][i - 4].needsUpdateLower = true;
}

//모든 사용자 조작 시 호출되어 절전 모드 타이머를 초기화
void userActivity() {
  lastAnyInteractionTime = millis();
  targetBrightness = 0;
}

void updateLabelName(int i, bool isUpper) {
  Pot &p = pots[currentPage][i];
  int x = 2 + i * 70;
  tft.setFont(&FreeSans9pt7b);
  if (isUpper) {
    if (p.mode == 'E') {
      tft.setTextColor((p.activeLayer == 1) ? 0xFFFF : 0x8CF1);
      tft.fillRect(x, 15, 70, 20, 0x0000);
      tft.setCursor(max(0, x + 1), 29);
      tft.print(p.btnName);
    }
  } else {
    tft.setTextColor((p.mode == 'E' && p.activeLayer != 0) ? 0x8CF1 : 0xFFFF);
    tft.fillRect(x, 44, 70, 20, 0x0000);
    tft.setCursor(x + 1, 59);
    tft.print(p.knobName);
  }
}


void renderGauge(int i, bool isUpper) {
  Pot &p = pots[currentPage][i];
  int fIdx = isUpper ? i : i + 4;
  int value = isUpper ? p.btnCcValue : p.knobValue;
  unsigned long now = millis();
  int x = 2 + i * 70;
  int y = isUpper ? 35 : 65;

  bool isActive = isUpper ? (p.activeLayer == 1) : (p.mode == 'S' || p.activeLayer == 0);
  char colorC = isUpper ? p.btnColorCode : p.knobColorCode;
  int w = map(value, 0, 127, 0, 68);

  // 숫자 표시 조건
  bool showNumber = !justChangedPage && (isStudioMode || (now - lastMoveTime[fIdx] < 2000));

  // 업데이트가 필요 없는 상황이면 리턴
  if (!p.needsUpdateUpper && !p.needsUpdateLower && !showNumber && flashCount[fIdx] == 0) {
    return;
  }
  uint16_t border = isActive ? 0xCE79 : 0x4208;
  uint16_t barColor = isActive ? getColor(colorC) : getDimColor(getColor(colorC));
  int tempW = w;

  if (flashCount[fIdx] > 0 && isFlashOn[fIdx]) {
    border = 0xF800;
    if (value == 127) barColor = 0x4208;
    else if (value == 0) {
      barColor = 0x8410;
      tempW = 68;
    }
  }

  tft.startWrite();

  // 1. 게이지 테두리
  tft.drawRect(x, y - 1, 70, 11, border);

  // 2. [수정] 잠금 표시 점 로직
  // 잠겨있고(isRotLocked) + 현재 활성화된 바(isActive)일 때만 점을 표시함
  if (isRotLocked[i] && isActive) {
    // 현재 페이지의 설정 색상 코드를 가져와서 실제 16비트 색상으로 변환
    uint16_t pageThemeColor = getColor(pageSettings[currentPage].colorCode);

    // x + 33: 정중앙 위치, y - 4: 테두리 바로 위
    tft.fillRect(x + 67, y - 4, 3, 3, pageThemeColor);
  } else {
    // 잠금이 풀렸거나 비활성이면 검은색으로 지움
    tft.fillRect(x + 67, y - 4, 3, 3, 0x0000);
  }

  // 3. 게이지 바 채우기
  tft.fillRect(x + 1, y, tempW, 9, barColor);
  tft.fillRect(x + 1 + tempW, y, 68 - tempW, 9, 0x0000);

  // 4. 숫자 표시 로직
  if (showNumber) {
    char buf[4];
    itoa(value, buf, 10);
    int len = strlen(buf);
    int startX = (x + 70) - 4 - (len * 6);
    tft.setFont(NULL);
    tft.setTextSize(1);
    for (int n = 0; n < len; n++) {
      int charX = startX + (n * 6);
      bool isUnderBar = (tempW > (charX - x + 2));
      uint16_t txtColor = 0xFFFF;
      if (isActive) {
        bool isBrightBar = (colorC == 'Y' || colorC == 'W' || colorC == 'G' || colorC == 'C') || (flashCount[fIdx] > 0 && isFlashOn[fIdx] && value == 0);
        if (isUnderBar && isBrightBar) txtColor = 0x0000;
        else if (!isUnderBar) txtColor = 0xC618;
      } else {
        txtColor = 0x52AA;
      }
      tft.setTextColor(txtColor, isUnderBar ? barColor : 0x0000);
      tft.setCursor(charX, y + 1);
      tft.print(buf[n]);
    }
  }
  tft.endWrite();

  if (isUpper) p.needsUpdateUpper = false;
  else p.needsUpdateLower = false;
}

//화면의 특정 구역(상단/하단)을 다시 그려주는 실질적인 UI 렌더링
void updateZone(int i, bool isUpper) {
  Pot &p = pots[currentPage][i];
  int x = 2 + i * 70;
  if (isUpper && p.mode != 'E') {
    tft.fillRect(x, 15, 69, 19, p.btnState ? getColor(p.btnColorCode) : 0x0000);
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(p.btnState ? getContrastColor(p.btnColorCode) : 0x8CF1);
    tft.setCursor(max(0, x + 1), 29);
    tft.print(p.btnName);
    tft.setFont(NULL);
    tft.setTextColor(getColor(pageSettings[currentPage].colorCode));
    tft.setCursor(x + 21, 36);
    tft.print(p.isToggle ? "T-SWITCH" : "M-SWITCH");
    p.needsUpdateUpper = false;
  } else {
    renderGauge(i, isUpper);
  }
}

void updateModeDisplay() {
  PageConfig &cfg = pageSettings[currentPage];
  tft.fillRect(240, 0, 44, 12, getColor(cfg.colorCode));
  tft.setFont(NULL);
  tft.setTextColor(getContrastColor(cfg.colorCode));
  int yOffset = (cfg.colorCode == 'B') ? 2 : 3;
  tft.setCursor(245, yOffset);
  tft.print(isStudioMode ? "STUDIO" : "  LIVE");
}

//현재 페이지의 전체 틀(가이드라인, 라벨명 등)을 화면에 출력
void drawStaticLayout() {
  tft.fillScreen(0x0000);
  PageConfig &cfg = pageSettings[currentPage];
  uint16_t themeCol = getColor(cfg.colorCode);
  uint16_t contrastCol = getContrastColor(cfg.colorCode);

  // 1. 상단 기본 바 배경 및 페이지 정보
  tft.fillRect(0, 0, 284, 12, themeCol);
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(contrastCol);

  int yOffset = (cfg.colorCode == 'B') ? 2 : 3;
  tft.setCursor(5, yOffset);
  tft.print("[ ");
  tft.print(currentPage + 1);
  tft.print("/4 ] ");
  tft.print(cfg.title);

  // 2. 글로벌 버튼 상태 복구 (중복 제거 → 함수 호출)
  drawGlobalBtnUI();

  // 3. 포트별 이름 및 게이지 출력
  for (int i = 0; i < 4; i++) {
    updateLabelName(i, true);
    updateLabelName(i, false);
    updateZone(i, true);
    updateZone(i, false);
  }
}

//페이지 전환 시 부드러운 애니메이션 효과 구현
void drawPreviewAnimated() {
  // 1. 입구 컷 방지: 마지막에 그린 페이지와 지금 그리려는 페이지가 같고, 조작도 없다면 정지만 유지
  if (lastDrawnPage == previewPage && pageStepAccumulator == 0) {
    PageConfig &pCfg = pageSettings[previewPage];
    uint16_t pBarCol = getColor(pCfg.colorCode);
    uint16_t pTextCol = getContrastColor(pCfg.colorCode);

    int numW = (previewPage + 1 >= 10) ? 24 : 12;
    int totalW = 30 + numW + 18 + (int)(strlen(pCfg.title) * 12);
    int startX = (284 - totalW) / 2;

    // 캔버스에 그리기 (Y좌표는 27을 뺀 상대 좌표로 계산: 35->8, 30->3)
    canvas.fillScreen(0x0000);
    canvas.fillRect(0, 0, 284, 22, pBarCol);
    canvas.setTextColor(pTextCol);
    canvas.setTextSize(1);
    canvas.setCursor(startX, 8);
    canvas.print("Page");
    canvas.setTextSize(2);
    canvas.setCursor(startX + 30, 3);
    canvas.print(previewPage + 1);
    canvas.setTextSize(1);
    canvas.setCursor(startX + 30 + numW, 8);
    canvas.print(" -  ");
    canvas.setTextSize(2);
    canvas.setCursor(startX + 30 + numW + 18, 3);
    canvas.print(pCfg.title);

    // 도장 찍기! (0, 27) 위치에 한 번에 출력
    tft.drawRGBBitmap(0, 27, canvas.getBuffer(), 284, 22);
    return;
  }

  // 2. 애니메이션 준비
  tft.fillScreen(0x0000);
  tft.setTextWrap(false);

  int sourcePage = (lastDrawnPage == -1) ? currentPage : lastDrawnPage;
  PageConfig &oldCfg = pageSettings[sourcePage];
  PageConfig &newCfg = pageSettings[previewPage];
  uint16_t oldBarCol = getColor(oldCfg.colorCode);
  uint16_t newBarCol = getColor(newCfg.colorCode);
  uint16_t oldTextCol = getContrastColor(oldCfg.colorCode);
  uint16_t newTextCol = getContrastColor(newCfg.colorCode);

  auto getFullWidthSize2 = [](int pageNum, const char *title) {
    int numW = (pageNum >= 10) ? 24 : 12;
    return 30 + numW + 18 + (int)(strlen(title) * 12);
  };

  int oldStartX = (284 - getFullWidthSize2(sourcePage + 1, oldCfg.title)) / 2;
  int newStartX = (284 - getFullWidthSize2(previewPage + 1, newCfg.title)) / 2;

  const int totalSteps = 7;  // 프레임 수를 7로 유지

  for (int i = 0; i <= totalSteps; i++) {
    // 탈출 조건
    if (i > 0 && abs(pageStepAccumulator) >= (PAGE_ENC_THRESHOLD + 1)) {
      tft.setTextSize(1);
      return;
    }

    int slidePos = (i * 284) / totalSteps;
    int newX = (animDirection == 1) ? (-284 + slidePos) : (284 - slidePos);
    int oldX = (animDirection == 1) ? (slidePos) : (-slidePos);

    tft.startWrite();

    // --- 나가는 페이지 (Old Page) ---
    if (sourcePage != previewPage && oldX > -284 && oldX < 284) {
      canvas.fillScreen(0x0000);
      canvas.fillRect(0, 0, 284, 22, oldBarCol);
      canvas.setTextColor(oldTextCol);
      canvas.setTextSize(1);
      canvas.setCursor(oldStartX, 8);
      canvas.print("Page");
      canvas.setTextSize(2);
      canvas.setCursor(oldStartX + 30, 3);
      canvas.print(sourcePage + 1);
      int numW = (sourcePage + 1 >= 10) ? 24 : 12;
      canvas.setTextSize(1);
      canvas.setCursor(oldStartX + 30 + numW, 8);
      canvas.print(" -  ");
      canvas.setTextSize(2);
      canvas.setCursor(oldStartX + 30 + numW + 18, 3);
      canvas.print(oldCfg.title);

      tft.drawRGBBitmap(oldX, 27, canvas.getBuffer(), 284, 22);
    }

    // --- 들어오는 페이지 (New Page) ---
    if (newX > -284 && newX < 284) {
      canvas.fillScreen(0x0000);
      canvas.fillRect(0, 0, 284, 22, newBarCol);
      canvas.setTextColor(newTextCol);
      canvas.setTextSize(1);
      canvas.setCursor(newStartX, 8);
      canvas.print("Page");
      canvas.setTextSize(2);
      canvas.setCursor(newStartX + 30, 3);
      canvas.print(previewPage + 1);
      int numW = (previewPage + 1 >= 10) ? 24 : 12;
      canvas.setTextSize(1);
      canvas.setCursor(newStartX + 30 + numW, 8);
      canvas.print(" -  ");
      canvas.setTextSize(2);
      canvas.setCursor(newStartX + 30 + numW + 18, 3);
      canvas.print(newCfg.title);

      tft.drawRGBBitmap(newX, 27, canvas.getBuffer(), 284, 22);
    }

    tft.endWrite();
    delay(7);  // 8ms 딜레이 유지
  }

  lastDrawnPage = previewPage;
  tft.setTextWrap(true);
  tft.setTextSize(1);
}
// ===============================
// JSON 및 설정 관리
// ===============================

void setDefaults() {
  // 파일이 없거나 깨졌을 때 화면이 비지 않도록 기본값 설정
  globalBrightness = 10;
  pixels.setBrightness(globalBrightness);
  startInStudio = true;
  isStudioMode = true;
  strlcpy(globalBtn.name, "GLOBAL", 16);
  globalBtn.channel = 0;
  globalBtn.cc = 120;
  for (int i = 0; i < NUM_PAGES; i++) {
    sprintf(pageSettings[i].title, "PAGE %d", i + 1);
    pageSettings[i].colorCode = (i == 0) ? 'R' : (i == 1) ? 'G'
                                               : (i == 2) ? 'B'
                                                          : 'Y';
    for (int j = 0; j < 4; j++) {
      sprintf(pots[i][j].knobName, "KNOB %d", j + 1);
      pots[i][j].knobColorCode = 'C';
      sprintf(pots[i][j].btnName, "BTN %d", j + 1);
      pots[i][j].btnColorCode = 'W';
      pots[i][j].mode = 'E';
    }
  }
}

void applyConfig(JsonDocument &doc) {
  // 1. 시스템 설정 처리
  if (doc["system"].is<JsonObject>()) {
    const char *m = doc["system"]["mode"] | "Studio";
    startInStudio = (strcmp(m, "Studio") == 0);
    isStudioMode = startInStudio;
    DIM_TIMEOUT = (doc["system"]["scr"] | 5) * 60000;

    // --- LED 밝기 로직 추가 (system 안에 포함) ---
    // 에디터에서 "br" 이라는 키값으로 보낸다고 가정 (짧은 키값 선호 시)
    // 혹은 "brightness"로 길게 쓰셔도 됩니다. 여기서는 가독성을 위해 brightness로 합니다.
    globalBrightness = doc["system"]["brightness"] | 20;
    vibStrengthMax = doc["system"]["vib"] | 160;
  }

  // 안전 범위 제한 (10~30) 및 즉시 적용
  globalBrightness = constrain(globalBrightness, 5, 255);
  pixels.setBrightness(globalBrightness);

  // 2. 글로벌 버튼 설정 처리
  if (doc["global"].is<JsonObject>()) {
    strlcpy(globalBtn.name, doc["global"]["n"] | "GLOBAL", 16);
    globalBtn.channel = doc["global"]["ch"];
    globalBtn.cc = doc["global"]["cc"];
    globalBtn.isToggle = (doc["global"]["t"] == 1);
  }

  // 3. 페이지 및 포트 설정 처리
  JsonArray pArr = doc["pages"];
  for (int i = 0; i < NUM_PAGES; i++) {
    if (i >= pArr.size()) break;
    strlcpy(pageSettings[i].title, pArr[i]["t"] | "PAGE", 20);
    pageSettings[i].colorCode = (pArr[i]["c"] | "R")[0];

    JsonArray kArr = pArr[i]["k"];
    for (int j = 0; j < POTS_PER_PAGE; j++) {
      if (j >= kArr.size()) break;
      strlcpy(pots[i][j].knobName, kArr[j]["rn"] | "", 16);
      pots[i][j].knobChannel = kArr[j]["rc"];
      pots[i][j].knobCC = kArr[j]["rcc"];
      pots[i][j].knobColorCode = (kArr[j]["rv"] | "C")[0];
      pots[i][j].mode = (kArr[j]["bm"] | "E")[0];
      strlcpy(pots[i][j].btnName, kArr[j]["bn"] | "", 16);
      pots[i][j].btnChannel = kArr[j]["bc"];
      pots[i][j].btnCC = kArr[j]["bcc"];
      pots[i][j].btnColorCode = (kArr[j]["bv"] | "W")[0];
      pots[i][j].isToggle = (kArr[j]["bt"] == 1);

      pots[i][j].needsUpdateUpper = true;
      pots[i][j].needsUpdateLower = true;
    }
  }
}

//현재 설정 및 페이지 번호를 JSON 파일로 플래시 메모리에 저장
void saveSystemSettings() {
  if (!LittleFS.exists("/config.json")) return;

  File f = LittleFS.open("/config.json", "r");
  JsonDocument doc;
  deserializeJson(doc, f);
  f.close();

  // 기존 값 업데이트
  doc["system"]["brightness"] = globalBrightness;
  doc["system"]["vib"] = vibStrengthMax;
  doc["system"]["mode"] = isStudioMode ? "Studio" : "Live";
  doc["system"]["scr"] = DIM_TIMEOUT / 60000;  // ms를 분(min)으로 변환하여 저장
  doc["system"]["lastPage"] = currentPage;

  f = LittleFS.open("/config.json", "w");
  if (f) {
    serializeJson(doc, f);
    f.close();
    Serial.println("SYSTEM_SAVED");
  }
}

//플래시 메모리(LittleFS)에서 설정값 및 마지막 페이지 로드
void loadConfig() {
  // --- 기본값 초기화 ---
  globalBrightness = 40;
  vibStrengthMax = 110;  // 이전 설정값(MID)에 맞춰 조정
  currentPage = 0;

  if (!LittleFS.exists("/config.json")) {
    setDefaults();
    return;
  }

  File f = LittleFS.open("/config.json", "r");
  if (f) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    if (!err) {
      applyConfig(doc);

      if (doc["system"]["lastPage"].is<int>()) {
        currentPage = doc["system"]["lastPage"];

        // 페이지 범위 안전장치
        if (currentPage < 0 || currentPage >= NUM_PAGES) {
          currentPage = 0;
        }
      }
    } else {
      setDefaults();
    }
    f.close();
  } else {
    setDefaults();
  }
}



// 기존 handleDownloadJson 함수 교체
void handleDownloadJson() {
  // 데이터가 들어올 때까지 잠시 대기 (타임아웃 설정)
  unsigned long startWait = millis();
  while (!Serial.available()) {
    if (millis() - startWait > 3000) {  // 3초 동안 데이터 안 오면 취소
      Serial.println("ERR_TIMEOUT");
      return;
    }
  }

  JsonDocument doc;
  // Serial 스트림에서 직접 읽어서 파싱 (버퍼 오버플로우 방지)
  DeserializationError err = deserializeJson(doc, Serial);

  if (err) {
    // 에러 발생 시 에러 내용 전송
    Serial.print("ERR_JSON: ");
    Serial.println(err.c_str());

    // 남은 버퍼 비우기
    while (Serial.available()) Serial.read();
    return;
  }

  // 성공 시 파일 저장
  File f = LittleFS.open("/config.json", "w");
  if (f) {
    serializeJson(doc, f);
    f.close();
    applyConfig(doc);
    tft.fillScreen(0x0000);
    drawStaticLayout();
    Serial.println("SAVE_OK");  // 성공 메시지 전송
  } else {
    Serial.println("ERR_FS");
  }
}


void updateExtraLeds() {
  unsigned long now = millis();

  // [준비] 글로벌 버튼용 상시 심장 박동 밝기 계산 (0.3f ~ 1.0f 범위)
  float breathe = 0.65f + 0.35f * sin(now / 500.0 * PI);

  // --- [1] 버튼/스위치 LED (0~3번): 모드별 차별화된 피드백 ---
  for (int i = 0; i < 4; i++) {
    Pot &p = pots[currentPage][i];

    if (p.mode == 'E') {
      // [엔코더 모드] 윗쪽 레이어 활성 시: 숨쉬는(Breathing) 효과
      if (p.activeLayer == 1) {
        uint32_t col = getLEDColor(p.btnColorCode);

        // 1.5초 주기로 숨쉬는 효과 계산 (0.4 ~ 1.0 사이를 왕복)
        float breathe = (sin(now * 2.0f * PI / 700.0f) + 1.0f) / 2.0f;
        float breathIntensity = 0.0f + (breathe * 0.9f);

        // 절전 모드 시 전체적인 밝기 스케일 다운 (최대 40%)
        float dimScale = isLedOn ? 0.8f : 0.3f;
        float finalIntensity = breathIntensity * dimScale;

        uint8_t r = (uint8_t)((col >> 16 & 0xFF) * finalIntensity);
        uint8_t g = (uint8_t)((col >> 8 & 0xFF) * finalIntensity);
        uint8_t b = (uint8_t)((col & 0xFF) * finalIntensity);
        pixels.setPixelColor(i, pixels.Color(r, g, b));
      } else {
        pixels.setPixelColor(i, 0);  // 아래쪽 레이어는 소등
      }
    } else {
      // [스위치 모드] 켜졌을 때: 고정 점등 (Static)
      if (p.btnState) {
        uint32_t col = getLEDColor(p.btnColorCode);
        float finalIntensity = isLedOn ? 0.8f : 0.3f;  // 절전 시 40%

        uint8_t r = (uint8_t)((col >> 16 & 0xFF) * finalIntensity);
        uint8_t g = (uint8_t)((col >> 8 & 0xFF) * finalIntensity);
        uint8_t b = (uint8_t)((col & 0xFF) * finalIntensity);
        pixels.setPixelColor(i, pixels.Color(r, g, b));
      } else {
        pixels.setPixelColor(i, 0);
      }
    }
  }

  // --- [2] 엔코더/앰비언트 LED (4~7번) ---
  for (int i = 0; i < 4; i++) {
    int ledIdx = 7 - i;
    bool flashActive = (flashCount[i] > 0 || flashCount[i + 4] > 0);
    bool flashOn = (isFlashOn[i] || isFlashOn[i + 4]);

    if (flashActive && flashOn) {
      Pot &p = pots[currentPage][i];
      char activeColorCode = (p.mode == 'E' && p.activeLayer == 1) ? p.btnColorCode : p.knobColorCode;
      pixels.setPixelColor(ledIdx, getLEDColor(activeColorCode));
    } else {
      if (!isLedOn) {
        // [절전 모드] 순환 애니메이션
        unsigned long cycleTime = 4000;  // 8초로 더 느리고 우아하게
        float phase = (now % cycleTime) / (float)cycleTime;
        float myOffset = (float)i / 4.0f;
        float intensity = pow((sin((phase - myOffset) * 2.0f * PI) + 1.0f) / 2.0f, 5);

        uint32_t pageCol = getLEDColor(pageSettings[currentPage].colorCode);

        // [밝기 수정] 평상시(0.25)보다 확실히 낮은 0.10f (10%) 정도로 설정
        // 이렇게 해야 평소보다 '잠든' 느낌이 납니다.
        float dimPeak = 0.25f;
        uint8_t r = (uint8_t)((pageCol >> 16 & 0xFF) * intensity * dimPeak);
        uint8_t g = (uint8_t)((pageCol >> 8 & 0xFF) * intensity * dimPeak);
        uint8_t b = (uint8_t)((pageCol & 0xFF) * intensity * dimPeak);

        pixels.setPixelColor(ledIdx, pixels.Color(r, g, b));
      } else {
        // [평상시] 앰비언트 25% 유지
        uint32_t pageCol = getLEDColor(pageSettings[currentPage].colorCode);
        float ambientIntensity = 0.25f;
        uint8_t r = (uint8_t)((pageCol >> 16 & 0xFF) * ambientIntensity);
        uint8_t g = (uint8_t)((pageCol >> 8 & 0xFF) * ambientIntensity);
        uint8_t b = (uint8_t)((pageCol & 0xFF) * ambientIntensity);
        pixels.setPixelColor(ledIdx, pixels.Color(r, g, b));
      }
    }
  }

  // --- [3] 글로벌/페이지 LED (Super Deep Crimson Heartbeat - Ultra Peak) ---
  if (globalBtn.state) {
    float t = (millis() % 1100) / 1100.0f;
    float pulse;

    // 박동 곡선은 유지하되 피크에서의 존재감을 강화
    if (t < 0.15f) pulse = sin(t * PI / 0.15f);
    else if (t < 0.25f) pulse = 0;
    else if (t < 0.40f) pulse = sin((t - 0.25f) * PI / 0.20f) * 0.6f;
    else pulse = 0;

    float peakPulse = pow(pulse, 0.8f);
    float enhancedPulse = 0.15f + (peakPulse * 0.85f);

    // 기본 레드
    uint8_t r = (uint8_t)(245 * enhancedPulse);

    // 화이트 믹스
    uint8_t whiteMix = (pulse > 0.75f) ? (uint8_t)(100 * (pulse - 0.75f) * 4.0f) : 0;

    uint8_t g = whiteMix;
    uint8_t b = (uint8_t)(50 * enhancedPulse) + whiteMix;

    // --- 피크 근처에서 블루톤으로 자연스럽게 전환 ---
    if (pulse > 0.8f) {

      float blend = (pulse - 0.8f) / 0.2f;  // 0~1

      uint8_t blueR = 40;
      uint8_t blueG = 100;
      uint8_t blueB = 255;

      r = r * (1.0f - blend) + blueR * blend;
      g = g * (1.0f - blend) + blueG * blend;
      b = b * (1.0f - blend) + blueB * blend;
    }

    // 잔광
    if (pulse < 0.1f) {
      r = (globalBrightness <= 18) ? 120 : 70;
      g = 0;
      b = 10;
    }

    // --- 절전(Dim) 상황 시 처리
    if (!isLedOn) {
      float globalDimScale = 0.75f;  // 절전 시에도 조금 더 밝게 유지
      r = (uint8_t)(r * globalDimScale);
      g = (uint8_t)(g * globalDimScale);
      b = (uint8_t)(b * globalDimScale);
    }

    pixels.setPixelColor(LED_IDX_GLOBAL, pixels.Color(r, g, b));
  } else {
    pixels.setPixelColor(LED_IDX_GLOBAL, 0);
  }
}


// 레인보우용 색상 휠 함수 추가
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) { return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3); }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// ===============================
// MIDI 및 제어 로직
// ===============================

void sendCC(uint8_t ch, uint8_t cc, uint8_t val) {
  uint8_t msg[3] = { (uint8_t)(0xB0 | ch), cc, val };

  // 1. USB MIDI 전송 (비교적 안전)
  usb_midi.write(msg, 3);

  // 2. DIN MIDI (GPIO 8) 전송 - 안정화 버전
  // uart_is_writable은 '최소 1바이트' 공간이 있는지 확인합니다.
  // 3바이트를 연속으로 보낼 때 안전을 위해 하드웨어 버퍼를 체크합니다.

  for (int i = 0; i < 3; i++) {
    int timeout = 1000;  // 아주 짧은 타임아웃 (무한 대기 방지)
    while (!uart_is_writable(MIDI_UART) && timeout > 0) {
      timeout--;
    }

    if (timeout > 0) {
      uart_putc_raw(MIDI_UART, msg[i]);
    } else {
      // 버퍼가 가득 차서 전송 실패 시 루프 탈출 (재부팅 방지)
      break;
    }
  }
}

//페이지 변경 시 해당 페이지의 MIDI 값들을 엔코더 변수에 동기화
void syncEncodersToPage(int page) {
  for (int i = 0; i < NUM_ENCODERS; i++) {
    Pot &p = pots[page][i];
    int activeVal = (p.mode == 'E' && p.activeLayer == 1) ? p.btnCcValue : p.knobValue;

    // [핵심] 현재 페이지의 실제 값으로 전송 상태를 강제 동기화 (유령 현상 해결)
    sentMidi[i] = activeVal;
    currentMidi[i] = activeVal;
    currentLong[i] = targetLong[i] = (long)activeVal * 100;

    // 타이머와 상태 초기화
    nextStepTime[i] = 0;
    isSlowMove[i] = false;

    p.needsUpdateUpper = true;
    p.needsUpdateLower = true;
  }
}

volatile int encoderDelta[NUM_ENCODERS] = { 0 };

void updateEncoder(int i) {
  static const int8_t lookupTable[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

  uint8_t pA = digitalRead(pinA[i]);
  uint8_t pB = digitalRead(pinB[i]);
  encState[i] = (encState[i] << 2) | (pA << 1) | pB;
  int8_t motion = lookupTable[encState[i] & 0x0F];

  if (motion != 0) {
    encoderDelta[i] += motion;  // 회전 방향(+/-)만 누적
  }
}

void drawGlobalBtnUI() {
  if (!isSettingTimeout) {
    uint16_t theme = getColor(pageSettings[currentPage].colorCode);
    uint16_t txtC = getContrastColor(pageSettings[currentPage].colorCode);
    int yOffset = (pageSettings[currentPage].colorCode == 'B') ? 2 : 3;

    if (globalBtn.state) {
      tft.fillRect(176, 0, 60, 12, theme);
      tft.drawRect(176, 0, 60, 12, txtC);
      tft.setFont(NULL);
      tft.setTextSize(1);
      tft.setTextColor(txtC);
      int16_t x1, y1;
      uint16_t w, h;
      tft.getTextBounds(globalBtn.name, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(176 + (60 - w) / 2, yOffset);
      tft.print(globalBtn.name);
    } else {
      tft.fillRect(176, 0, 60, 12, theme);
    }
    updateModeDisplay();
  }
}

//페이지 전환 직후 원치 않는 입력을 방지하기 위한 안정화 로직
void stabilizeAfterPageChange(int page) {
  noInterrupts();
  // 기존 엔코더 누적값 초기화
  for (int i = 0; i < NUM_ENCODERS; i++) {
    encoderDelta[i] = 0;
    lockAccumulator[i] = 0;
  }
  // 페이지 인코더 누적값 초기화 (페이지 인코더는 ISR에서 pageStepAccumulator를 사용)
  pageStepAccumulator = 0;
  interrupts();

  // (이하 기존 스무딩/타이머/잠금 초기화)
  unsigned long now = millis();
  for (int i = 0; i < NUM_ENCODERS; i++) {
    sentMidi[i] = currentMidi[i];
    nextStepTime[i] = 0;
    isSlowMove[i] = false;
    currentLong[i] = targetLong[i] = (long)currentMidi[i] * 100;
    lastTickTime[i] = now;
    lastDetentTime[i] = now;

    isRotLocked[i] = true;
    lockAccumulator[i] = 0;

    lastMoveTime[i] = now - 2000;
    lastMoveTime[i + 4] = now - 2000;

    pots[page][i].needsUpdateUpper = true;
    pots[page][i].needsUpdateLower = true;
  }

  ignoreMidiUntil = millis() + 80;
}

void isr0() {
  updateEncoder(0);
}
void isr1() {
  updateEncoder(1);
}
void isr2() {
  updateEncoder(2);
}
void isr3() {
  updateEncoder(3);
}

//절전 모드, 진동 정지 타이머, 시리얼 통신, 페이지 자동 저장 관리
void handleSystemMaintenance(unsigned long now) {
  // 시리얼 커맨드 처리 (에디터 통신)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "HELLO_ASW") Serial.println("ASW_READY");
    else if (cmd == "DOWNLOAD_JSON") {
      Serial.println("JSON_READY");
      handleDownloadJson();
    } else if (cmd == "GET_CFG") {
      File f = LittleFS.open("/config.json", "r");
      if (f) {
        while (f.available()) Serial.write(f.read());
        Serial.println();
        f.close();
      }
    }
  }
  // 페이지 자동 저장 로직 (변경 후 30초 경과 시)
  if ((settingsChanged || pageNeedsSave) && (now - lastAnyInteractionTime >= 30000)) {
    saveSystemSettings();
    settingsChanged = false;
    pageNeedsSave = false;
  }
  // 화면 절전(Dimming) 및 페이드 로직
  if (now - lastAnyInteractionTime > DIM_TIMEOUT) {
    if (isLedOn) {
      isLedOn = false;
      targetBrightness = 230;
    }
  } else {
    if (!isLedOn) {
      isLedOn = true;
      targetBrightness = 0;
    }
  }
  if (now - lastFadeTime > 10) {
    if (currentBrightness < targetBrightness) currentBrightness += 2;
    else if (currentBrightness > targetBrightness) currentBrightness -= 2;
    currentBrightness = constrain(currentBrightness, 0, 255);
    analogWrite(TFT_BL, currentBrightness);
    lastFadeTime = now;
  }
  // 부트셀 진입 (핀 2 + 11 3초 유지)
  bool btn1 = (digitalRead(2) == LOW);
  bool btn2 = (digitalRead(11) == LOW);
  if (btn1 && btn2) {
    if (pressStartTime == 0) pressStartTime = now;
    else if (now - pressStartTime >= 5000) {
      analogWrite(VIB_MOTOR_PIN, 100);
      delay(120);
      analogWrite(VIB_MOTOR_PIN, 0);
      pixels.setBrightness(10);
      pixels.setPixelColor(0, getLEDColor('R'));  // Neon Crimson
      pixels.setPixelColor(1, getLEDColor('G'));  // Neo Emerald
      pixels.setPixelColor(2, getLEDColor('B'));  // Electric Sky
      pixels.setPixelColor(3, getLEDColor('Y'));  // Luminous Amber
      pixels.setPixelColor(4, getLEDColor('M'));  // Vivid Magenta
      pixels.setPixelColor(5, getLEDColor('C'));  // Ice Cyan
      pixels.setPixelColor(6, getLEDColor('O'));  // Neon Orange
      pixels.setPixelColor(7, getLEDColor('W'));  // 보정된 White
      pixels.setPixelColor(8, getLEDColor('C'));  // Ice Cyan
      pixels.show();                              // LED 즉시 출력

      // 3. 화면 처리 (블랙아웃)
      tft.fillScreen(0x0000);
      analogWrite(TFT_BL, 255);  // 백라이트 끄기
      reset_usb_boot(0, 0);
    }
  } else pressStartTime = 0;
  // 진동 모터 정지 체크
  if (isVibrating && now >= vibStopTime) {
    analogWrite(VIB_MOTOR_PIN, 0);
    isVibrating = false;
  }
  // LED 픽셀 최종 출력
  static unsigned long lastLedShow = 0;
  if (now - lastLedShow > 20) {
    updateExtraLeds();
    pixels.show();
    lastLedShow = now;
  }
}

//외부에서 들어오는 MIDI 메시지(Note/CC) 수신 및 처리
void handleMidiInput() {
  uint8_t rx[4];
  while (usb_midi.available()) {
    usb_midi.readPacket(rx);
    uint8_t status = rx[1] & 0xF0;
    uint8_t channel = rx[1] & 0x0F;
    uint8_t d1 = rx[2];
    uint8_t d2 = rx[3];

    // [1] MIDI Note로 페이지 전환 (C1, D1, E1, F1)
    if (channel == 0 && status == 0x90 && d2 > 0) {
      int targetPage = -1;
      if (d1 == 36) targetPage = 0;
      else if (d1 == 38) targetPage = 1;
      else if (d1 == 40) targetPage = 2;
      else if (d1 == 41) targetPage = 3;

      if (targetPage != -1 && targetPage != currentPage) {
        currentPage = targetPage;
        isPagePending = false;
        for (int i = 0; i < NUM_ENCODERS; i++) {
          isRotLocked[i] = true;
          lockAccumulator[i] = 0;
          lastMoveTime[i] = lastMoveTime[i + 4] = millis() - 2000;
          pots[targetPage][i].needsUpdateUpper = pots[targetPage][i].needsUpdateLower = true;
        }
        justChangedPage = !isStudioMode;
        stabilizeAfterPageChange(currentPage);
        syncEncodersToPage(currentPage);
        updateExtraLeds();
        lastPageChangeTime = millis();
        pageNeedsSave = true;
        userActivity();
        drawStaticLayout();
      }
    }

    // [2] CC 수신 및 스무딩 동기화
    if (status == 0xB0) {
      lastSentValue[d1] = d2;
      if (globalBtn.cc == d1 && globalBtn.channel == channel) {
        bool newState = (d2 >= 64);
        if (globalBtn.state != newState) {
          globalBtn.state = newState;
          updateExtraLeds();
          drawGlobalBtnUI();
        }
      }
      for (int p = 0; p < NUM_PAGES; p++) {
        for (int i = 0; i < 4; i++) {
          Pot &tp = pots[p][i];
          if (tp.knobCC == d1 && tp.knobChannel == channel) {
            tp.knobValue = d2;
            if (p == currentPage && (tp.mode != 'E' || tp.activeLayer == 0)) {
              sentMidi[i] = currentMidi[i] = d2;
              currentLong[i] = targetLong[i] = (long)d2 * 100;
              nextStepTime[i] = 0;
              isSlowMove[i] = false;
              tp.needsUpdateLower = true;
            }
          }
          if (tp.btnCC == d1 && tp.btnChannel == channel) {
            if (tp.mode == 'E') {
              tp.btnCcValue = d2;
              if (p == currentPage && tp.activeLayer == 1) {
                sentMidi[i] = currentMidi[i] = d2;
                currentLong[i] = targetLong[i] = (long)d2 * 100;
                nextStepTime[i] = 0;
                isSlowMove[i] = false;
                tp.needsUpdateUpper = true;
              }
            } else {
              tp.btnState = (d2 >= 64);
              if (p == currentPage) tp.needsUpdateUpper = true;
            }
          }
        }
      }
    }
  }
}

//페이지 엔코더 입력 처리 및 페이지 전환/확정 로직 제어
void handlePageNavigation(unsigned long now) {
  if (abs(pageStepAccumulator) >= PAGE_ENC_THRESHOLD) {
    userActivity();
    int moveStep = (pageStepAccumulator > 0) ? 1 : -1;
    pageStepAccumulator = 0;

    if (digitalRead(GLOBAL_BTN_PIN) != LOW && !isSettingTimeout) {
      int basePage = isPagePending ? previewPage : currentPage;
      int nextTarget = basePage + moveStep;

      // [수정] 실제 이동 가능한 페이지 범위 내에 있을 때
      if (nextTarget >= 0 && nextTarget < NUM_PAGES) {
        // --- 짧고 날카로운 진동 피드백 (틱! 하는 느낌) ---
        if (vibStrengthMax > 0) {
          int boostedStrength = constrain((vibStrengthMax * 14) / 10, 0, 255);
          analogWrite(VIB_MOTOR_PIN, boostedStrength);
          delay(50);                      // 페이지전환 진동 길이
          analogWrite(VIB_MOTOR_PIN, 0);  // 즉시 정지
          isVibrating = false;            // 상태 초기화
        }

        animDirection = moveStep;
        previewPage = nextTarget;
        lastPageMoveTime = now;
        isPagePending = true;
        drawPreviewAnimated();  // 이제 애니메이션 시간과 상관없이 진동은 끝난 상태입니다.
      }
      // [유지] 범위를 벗어났을 때 (첫/마지막 페이지)
      else {
        lastPageMoveTime = now;
        if (isPagePending) {
          int temp = currentPage;
          currentPage = previewPage;
          tft.fillRect(0, 27, 284, 22, 0xFFFF);
          delay(30);
          drawPreviewAnimated();
          currentPage = temp;
        } else {
          tft.fillRect(0, 0, 284, 12, 0xFFFF);
          delay(30);

          PageConfig &cfg = pageSettings[currentPage];
          tft.setTextSize(1);
          tft.setFont(NULL);
          tft.fillRect(0, 0, 284, 12, getColor(cfg.colorCode));
          tft.setTextColor(getContrastColor(cfg.colorCode));
          tft.setCursor(5, (cfg.colorCode == 'B' ? 2 : 3));
          tft.printf("[ %d/%d ] %s", currentPage + 1, NUM_PAGES, cfg.title);
          updateModeDisplay();
          tft.setTextSize(1);
          tft.setFont(NULL);
        }
        // 범위 초과 시에는 진동 없이 시각적 반전만 수행 (사용자 요청 반영)
      }
    }
  }

  // 페이지 확정 로직
  if (!isSettingTimeout && isPagePending && (now - lastPageMoveTime >= PAGE_CONFIRM_DELAY)) {
    // [추가] 실제 변경이 일어났을 때만 저장 플래그를 ON
    if (currentPage != previewPage) {
      pageNeedsSave = true;
      Serial.printf("System: Page changed (%d -> %d), save reserved.\n", currentPage + 1, previewPage + 1);
    } else {
      // 원래 페이지로 돌아온 경우 저장하지 않음
      Serial.println("System: Returned to original page, no save needed.");
    }

    currentPage = previewPage;  // 최종 페이지 확정
    isPagePending = false;

    // 엔코더 및 상태 초기화 로직
    for (int i = 0; i < NUM_ENCODERS; i++) {
      lastTickTime[i] = now;
      nextStepTime[i] = 0;
      isSlowMove[i] = false;
      lastMoveTime[i] = lastMoveTime[i + 4] = now - 2000;
      isRotLocked[i] = true;
    }

    justChangedPage = !isStudioMode;
    stabilizeAfterPageChange(currentPage);
    syncEncodersToPage(currentPage);

    for (int k = 0; k < 8; k++) isAtLimit[k] = false;

    updateExtraLeds();
    userActivity();
    drawStaticLayout();
  }

  // 설정 모드 복구 로직
  if (isSettingTimeout && (now - settingDisplayTime > 1000)) {
    isSettingTimeout = false;

    tft.fillScreen(0x0000);
    drawStaticLayout();
    for (int i = 0; i < 4; i++) {
      pots[currentPage][i].needsUpdateUpper = pots[currentPage][i].needsUpdateLower = true;
      if (pots[currentPage][i].mode == 'E') {
        updateLabelName(i, true);
        updateLabelName(i, false);
      }
    }
    updateExtraLeds();
    updateModeDisplay();
  }
}

//4개 메인 엔코더의 회전 감지, 가속도 계산 및 MIDI 전송
void processMainEncoders(unsigned long now) {
  for (int i = 0; i < NUM_ENCODERS; i++) {
    noInterrupts();
    int motion = encoderDelta[i];
    encoderDelta[i] = 0;
    interrupts();

    // 잠금 처리
    if (isRotLocked[i]) {
      if (motion != 0) {
        userActivity();
        if (now - lastTickTime[i] > 500) lockAccumulator[i] = 0;
        lockAccumulator[i] += motion;
        lastTickTime[i] = now;
        if (abs(lockAccumulator[i]) >= ROT_THRESHOLD) {
          isRotLocked[i] = false;
          lockAccumulator[i] = 0;
          lastDetentTime[i] = now;
          pots[currentPage][i].needsUpdateUpper = pots[currentPage][i].needsUpdateLower = true;
        }
      }
      continue;
    }

    // 움직임 계산 (가속도 및 라이브/스튜디오 모드)
    if (motion != 0) {
      userActivity();
      lastDetentTime[i] = now;
      unsigned long interval = now - lastTickTime[i];
      isSlowMove[i] = (interval > 220);
      if (isStudioMode) {
        int move = (interval > 130) ? 100 : (interval > 40 ? 150 : 170);
        targetLong[i] = constrain(targetLong[i] + (motion * move), 0, 12700);
        currentLong[i] = targetLong[i];
      } else {
        liveStepAccum[i] += motion * 17;  // LIVE_STEP
        int deltaMidi = liveStepAccum[i] / 10;
        if (deltaMidi != 0) {
          liveStepAccum[i] -= deltaMidi * 10;
          targetLong[i] = (long)constrain(currentMidi[i] + deltaMidi, 0, 127) * 100;
          currentLong[i] = targetLong[i];
        }
      }
      lastTickTime[i] = now;
    }

    // 자동 잠금 타이머
    if (!isRotLocked[i] && (now - lastDetentTime[i] > LOCK_TIMEOUT)) {
      isRotLocked[i] = true;
      lockAccumulator[i] = 0;
      pots[currentPage][i].needsUpdateUpper = pots[currentPage][i].needsUpdateLower = true;
    }

    // MIDI 전송 및 스마트 스무딩
    int targetMidi = currentLong[i] / 100;
    int diff = targetMidi - sentMidi[i];
    if (diff != 0) {
      lastDetentTime[i] = now;
      Pot &p = pots[currentPage][i];
      // [고무줄 스무딩] 조건: 라이브모드 AND 느린회전 AND (차이>=2 OR 보간진행중)
      if (!isStudioMode && isSlowMove[i] && (abs(diff) >= 2 || now < nextStepTime[i])) {
        if (now >= nextStepTime[i]) {
          sentMidi[i] += (diff > 0 ? 1 : -1);
          if (p.mode == 'E' && p.activeLayer == 1) {
            p.btnCcValue = sentMidi[i];
            sendCC(p.btnChannel, p.btnCC, sentMidi[i]);
            p.needsUpdateUpper = true;
            lastMoveTime[i] = now;
          } else {
            p.knobValue = sentMidi[i];
            sendCC(p.knobChannel, p.knobCC, sentMidi[i]);
            p.needsUpdateLower = true;
            lastMoveTime[i + 4] = now;
          }
          nextStepTime[i] = now + 150;
        }
      } else {  // [즉시 전송]
        sentMidi[i] = targetMidi;
        if (p.mode == 'E' && p.activeLayer == 1) {
          p.btnCcValue = sentMidi[i];
          sendCC(p.btnChannel, p.btnCC, sentMidi[i]);
          p.needsUpdateUpper = true;
          lastMoveTime[i] = now;
        } else {
          p.knobValue = sentMidi[i];
          sendCC(p.knobChannel, p.knobCC, sentMidi[i]);
          p.needsUpdateLower = true;
          lastMoveTime[i + 4] = now;
        }
      }

      // 센터/제한값 피드백 (진동/플래시)
      int flashIdx = (p.mode == 'E' && p.activeLayer == 1) ? i : i + 4;
      bool hitCenter = (sentMidi[i] == 64) || (currentMidi[i] < 64 && sentMidi[i] > 64) || (currentMidi[i] > 64 && sentMidi[i] < 64);
      if (hitCenter) {
        if (!isAtCenter[flashIdx]) {
          flashCount[flashIdx] = 2;
          isFlashOn[flashIdx] = true;
          lastFlashTime[flashIdx] = now;

          // --- 30% 부스트 및 오버플로우 방지 적용 ---
          if (vibStrengthMax > 0) {
            int boostedStrength = constrain((vibStrengthMax * 13) / 10, 0, 255);
            analogWrite(VIB_MOTOR_PIN, boostedStrength);
            vibStopTime = now + 70;  // 센터 피드백은 페이지 이동(10~15ms)보다 긴 60ms 유지
            isVibrating = true;
          }

          if (flashIdx < 4) pots[currentPage][i].needsUpdateUpper = true;
          else pots[currentPage][i].needsUpdateLower = true;
          isAtCenter[flashIdx] = true;
        }
      } else if (sentMidi[i] < 62 || sentMidi[i] > 66) isAtCenter[flashIdx] = false;

      if (sentMidi[i] == 0 || sentMidi[i] == 127) {
        if (!isAtLimit[flashIdx]) {
          triggerFlash(flashIdx);
          isAtLimit[flashIdx] = true;
          analogWrite(VIB_MOTOR_PIN, vibStrengthMax);
          vibStopTime = now + 160;
          isVibrating = true;
        }
      } else isAtLimit[flashIdx] = false;

      currentMidi[i] = sentMidi[i];
    }
  }
}

//상단 글로벌 버튼의 상태 체크 및 지정된 MIDI CC 전송
void handleGlobalButton(unsigned long now) {
  static bool lastGlobalBtnState = HIGH;
  bool currentGlobalBtn = digitalRead(GLOBAL_BTN_PIN);

  // 디바운스 및 상태 변화 체크
  if (!isPagePending && (currentGlobalBtn != lastGlobalBtnState) && (now - globalBtnDebounceTime > BTN_DEBOUNCE_TIME)) {
    userActivity();

    if (currentGlobalBtn == LOW) {  // [1. 버튼을 눌렀을 때]
      if (!isSettingTimeout) {
        globalBtn.state = globalBtn.isToggle ? !globalBtn.state : true;
        sendCC(globalBtn.channel, globalBtn.cc, globalBtn.state ? 127 : 0);

        // UI 표시 (상단 바)
        uint16_t theme = getColor(pageSettings[currentPage].colorCode);
        uint16_t txtC = getContrastColor(pageSettings[currentPage].colorCode);
        if (globalBtn.state) {
          tft.fillRect(176, 0, 60, 12, theme);
          tft.drawRect(176, 0, 60, 12, txtC);
          tft.setFont(NULL);
          tft.setTextSize(1);
          tft.setTextColor(txtC);
          int16_t x1, y1;
          uint16_t w, h;
          tft.getTextBounds(globalBtn.name, 0, 0, &x1, &y1, &w, &h);
          tft.setCursor(176 + (60 - w) / 2, (pageSettings[currentPage].colorCode == 'B' ? 2 : 3));
          tft.print(globalBtn.name);
        } else {
          tft.fillRect(176, 0, 60, 12, theme);
        }
        updateModeDisplay();
      }
    } else {  // [2. 버튼을 뗄 때]
      if (!globalBtn.isToggle) {
        globalBtn.state = false;
        sendCC(globalBtn.channel, globalBtn.cc, 0);

        if (!isSettingTimeout) {
          tft.fillRect(176, 0, 60, 12, getColor(pageSettings[currentPage].colorCode));
          updateModeDisplay();
        }
      }
    }

    lastGlobalBtnState = currentGlobalBtn;
    globalBtnDebounceTime = now;
  }
}

//엔코더 클릭 감지 (일반 모드: 레이어 전환 / 설정 모드: 값 변경)
void handleEncoderSwitches(unsigned long now) {
  for (int i = 0; i < NUM_ENCODERS; i++) {
    bool reading = digitalRead(pinSW[i]);
    if (reading != lastSwState[i] && (now - swDebounceTime[i] > BTN_DEBOUNCE_TIME)) {
      userActivity();

      if (reading == LOW) {  // 눌렀을 때
        // 글로벌 버튼이 눌린 상태라면 설정 모드로 진입/조작
        if (digitalRead(GLOBAL_BTN_PIN) == LOW) {
          bool showUI = false;
          const char *displayTitle = "";
          char valueStr[16] = "";
          const char *unitStr = "";

          // 이미 설정 화면이 떠 있는 상태(isSettingTimeout == true)여야 값이 변경됨 (원본 로직)
          bool shouldChange = isSettingTimeout;

          // --- [0] SCREEN DIM TIME ---
          if (i == 0) {
            const int STEPS[] = { 1, 3, 5, 10, 20, 30, 40, 60 };
            int curMin = DIM_TIMEOUT / 60000;
            for (int k = 0; k < 8; k++)
              if (STEPS[k] == curMin) {
                currentTimeoutIdx = k;
                break;
              }
            if (shouldChange) {
              currentTimeoutIdx = (currentTimeoutIdx + 1) % 8;
              DIM_TIMEOUT = (unsigned long)STEPS[currentTimeoutIdx] * 60000;
              settingsChanged = true;
              lastAnyInteractionTime = now;
            }
            displayTitle = "SCREEN DIM TIME";
            sprintf(valueStr, "%d", STEPS[currentTimeoutIdx]);
            unitStr = "MIN";
            showUI = true;
          }
          // --- [1] LED 밝기 ---
          else if (i == 1) {
            if (shouldChange) {
              globalBrightness = (globalBrightness <= 18) ? 40 : (globalBrightness <= 40 ? 70 : 18);
              pixels.setBrightness(globalBrightness);
              pixels.show();
              settingsChanged = true;
              lastAnyInteractionTime = now;
            }
            displayTitle = "LED BRIGHTNESS";
            strcpy(valueStr, (globalBrightness <= 18) ? "LOW" : (globalBrightness <= 40 ? "MID" : "HIGH"));
            showUI = true;
          }
          // --- [2] 진동 강도 (MID: 100 / MAX: 160 정의 활용) ---
          else if (i == 2) {
            if (shouldChange) {
              if (vibStrengthMax == 0) vibStrengthMax = 70;
              else if (vibStrengthMax == 70) vibStrengthMax = 110;   // MID 수준
              else if (vibStrengthMax == 110) vibStrengthMax = 160;  // MAX 수준
              else vibStrengthMax = 0;

              if (vibStrengthMax > 0) {
                analogWrite(VIB_MOTOR_PIN, vibStrengthMax);
                vibStopTime = now + 120;
                isVibrating = true;
              }
              settingsChanged = true;
              lastAnyInteractionTime = now;
            }
            displayTitle = "VIBRATION";
            if (vibStrengthMax == 0) strcpy(valueStr, "OFF");
            else if (vibStrengthMax <= 70) strcpy(valueStr, "LOW");
            else if (vibStrengthMax <= 110) strcpy(valueStr, "MID");
            else strcpy(valueStr, "HIGH");
            showUI = true;
          }
          // --- [3] 인코더 모드 (STUDIO / LIVE) ---
          else if (i == 3) {
            if (shouldChange) {
              isStudioMode = !isStudioMode;
              for (int k = 0; k < 4; k++) pots[currentPage][k].needsUpdateUpper = pots[currentPage][k].needsUpdateLower = true;
              updateModeDisplay();
              settingsChanged = true;
              lastAnyInteractionTime = now;
            }
            displayTitle = "ENCODER MODE";
            strcpy(valueStr, isStudioMode ? "STUDIO" : "LIVE");
            showUI = true;
          }

          if (showUI) {
            isSettingTimeout = true;
            settingDisplayTime = now;
            drawSettingUI(displayTitle, valueStr, unitStr);  // UI 그리기 함수 (중복 방지용 별도 분리 추천)
          }

          lastSwState[i] = reading;
          swDebounceTime[i] = now;
          continue;  // 설정 조작 시 레이어 토글 등 일반 동작 방지
        }

        // --- 일반 동작 (글로벌 버튼 안 눌렸을 때) ---
        Pot &p = pots[currentPage][i];
        if (p.mode == 'E') {
          p.activeLayer = !p.activeLayer;
          noInterrupts();
          encoderDelta[i] = 0;
          interrupts();
          int nxt = p.activeLayer ? p.btnCcValue : p.knobValue;
          currentMidi[i] = sentMidi[i] = nxt;
          currentLong[i] = targetLong[i] = (long)nxt * 100;
          isRotLocked[i] = true;
          updateLabelName(i, true);
          updateLabelName(i, false);
          p.needsUpdateUpper = p.needsUpdateLower = true;
        } else {
          p.btnState = p.isToggle ? !p.btnState : true;
          sendCC(p.btnChannel, p.btnCC, p.btnState ? 127 : 0);
          p.needsUpdateUpper = true;
          lastMoveTime[i] = now;
        }
      } else {  // 뗐을 때
        Pot &p = pots[currentPage][i];
        if (p.mode != 'E' && !p.isToggle) {
          p.btnState = false;
          sendCC(p.btnChannel, p.btnCC, 0);
          p.needsUpdateUpper = true;
        }
      }
      lastSwState[i] = reading;
      swDebounceTime[i] = now;
    }
  }
}

// 글로벌 설정 변경 시 나타나는 중앙 팝업 UI 생성
void drawSettingUI(const char *displayTitle, const char *valueStr, const char *unitStr) {
  tft.fillScreen(0x0000);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(getColor('G'));  // 가이드라인 색상 (초록계열)

  int16_t x1, y1;
  uint16_t w, h;

  // 타이틀 가운데 정렬
  tft.getTextBounds(displayTitle, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((tft.width() - w) / 2, 30);
  tft.print(displayTitle);

  // 구분선
  tft.fillRect(65, 37, 157, 1, 0xF800);  // 빨간색 선

  // 설정값 가운데 정렬
  tft.setTextColor(getColor('W'));  // 흰색
  int16_t nx1, ny1;
  uint16_t nw, nh;
  tft.getTextBounds(valueStr, 0, 0, &nx1, &ny1, &nw, &nh);

  // 단위(unit)가 있을 경우 간격 계산
  int totalW = nw + (strlen(unitStr) > 0 ? 45 : 0);
  tft.setCursor((tft.width() - totalW) / 2, 30 + nh + 13);
  tft.print(valueStr);

  // 단위 출력 (MIN 등)
  if (strlen(unitStr) > 0) {
    tft.setCursor(tft.getCursorX() + 10, 30 + nh + 15);
    tft.print(unitStr);
  }
}

//각 노브/버튼의 변경된 상태를 TFT 화면에 점진적으로 갱신
void updateUIDisplay(unsigned long now) {
  if (now - lastUiUpdateTime > 1) {
    Pot &p = pots[currentPage][currentUiIndex];
    if (!isStudioMode) {
      if (p.mode == 'E') {
        unsigned long diffU = now - lastMoveTime[currentUiIndex];
        if (diffU >= 2000 && diffU < 2050) p.needsUpdateUpper = true;
      }
      unsigned long diffL = now - lastMoveTime[currentUiIndex + 4];
      if (diffL >= 2000 && diffL < 2050) p.needsUpdateLower = true;
    }
    if (p.needsUpdateUpper) updateZone(currentUiIndex, true);
    if (p.needsUpdateLower) updateZone(currentUiIndex, false);
    currentUiIndex = (currentUiIndex + 1) % 4;
    lastUiUpdateTime = now;
    if (justChangedPage) justChangedPage = false;
  }
}

//센터 도달이나 한계값 터치 시 LED/UI가 깜빡이는 효과 제어
void handleVisualFlash(unsigned long now) {
  for (int f = 0; f < 8; f++) {
    if (flashCount[f] > 0 && now - lastFlashTime[f] > 60) {
      flashCount[f]--;
      isFlashOn[f] = !isFlashOn[f];
      lastFlashTime[f] = now;
      if (f < 4) pots[currentPage][f].needsUpdateUpper = true;
      else pots[currentPage][f - 4].needsUpdateLower = true;
    } else if (flashCount[f] <= 0 && isFlashOn[f]) {
      isFlashOn[f] = false;
      if (f < 4) pots[currentPage][f].needsUpdateUpper = true;
      else pots[currentPage][f - 4].needsUpdateLower = true;
    }
  }
}




void setup() {
  delay(20);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  SPI.setSCK(TFT_SCL);
  SPI.setTX(TFT_SDA);
  SPI.begin();

  // --- LittleFS 초기화 로직 강화 ---
  if (!LittleFS.begin()) {
    Serial.println("FS_MOUNT_FAILED. Formatting...");
    LittleFS.format();  // 마운트 실패 시 강제 포맷
    if (!LittleFS.begin()) {
      Serial.println("FS_CRITICAL_ERROR");  // 포맷 후에도 안될 경우
    }
  }

  loadConfig();  // 설정 로드 완료

  TinyUSBDevice.setID(0x2E8A, MY_USB_PID);  // 제조사ID(라즈베리파이), 제품ID(우리꺼)
  TinyUSBDevice.setManufacturerDescriptor(MY_USB_MANUFACTURER);
  TinyUSBDevice.setProductDescriptor(MY_USB_PRODUCT);
  TinyUSBDevice.setSerialDescriptor(MY_USB_SERIAL);
  usb_midi.begin();
  Serial.begin(115200);
  // UART1 초기화 (31250bps)
  uart_init(MIDI_UART, 31250);

  // GPIO 8번을 UART1 TX 핀으로 설정
  gpio_set_function(MIDI_TX_PIN, GPIO_FUNC_UART);

  // 데이터 비트, 정지 비트 등 MIDI 규격 설정
  uart_set_format(MIDI_UART, 8, 1, UART_PARITY_NONE);

  // 하드웨어 FIFO 사용 (버퍼링에 도움 됨)
  uart_set_fifo_enabled(MIDI_UART, true);

  tft.init(76, 284);
  tft.setSPISpeed(62500000);
  tft.setRotation(3);
  tft.invertDisplay(false);
  analogWrite(TFT_BL, 0);

  delay(20);
  // --- OTTER 로고 섹션 ---
  tft.fillScreen(0x0000);

  // 1. 메인 타이틀: OTTER (커스텀 폰트)
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(0xFFFF);  // White
  tft.setCursor(118, 36);
  tft.print("OTTER");
  tft.setFont(NULL);  // 기본 폰트로 복구
  tft.setTextSize(1);
  tft.setTextColor(0xF800);  // 약간 어두운 회색/녹색 톤으로 대비 감소
  tft.setCursor(91, 43);
  tft.print("C O N T R O L L E R");

  for (int b = 255; b >= 0; b--) {
    analogWrite(TFT_BL, b);
    delay(5);
  }
  delay(20);

  tft.fillScreen(0x0000);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(0xFFFF);
  tft.setCursor(73, 32);
  tft.print("Ash Sound Works");
  tft.fillRect(71, 38, 148, 1, 0xF800);
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(0xAD55);
  tft.setCursor(109, 44);
  tft.print("ASW MC-E44s");
  tft.setTextColor(0xFFFF);
  tft.setCursor(100, 55);
  tft.print("Ver. 20260350");

  for (int b = 255; b >= 0; b--) {
    analogWrite(TFT_BL, b);
    delay(5);
  }

  delay(20);
  tft.fillScreen(0x0000);
  tft.setTextSize(1);

  int lineY = 12;                // 시작 높이
  int spacing = 10;              // 줄 간격
  uint16_t labelColor = 0xC618;  // 항목 이름용 회색
  uint16_t valueColor = 0xFFFF;  // 설정값용 흰색

  tft.setCursor(70, lineY);
  tft.setTextColor(valueColor);
  tft.print("[ SYSTEM CONFIG ]");

  // 4. Screen Timeout
  lineY += spacing + 5;
  tft.setCursor(70, lineY);
  tft.setTextColor(labelColor);
  tft.print("- Screen Timeout : ");
  tft.setTextColor(valueColor);
  tft.printf("%d min", DIM_TIMEOUT / 60000);

  // 1. LED Brightness
  lineY += spacing;
  tft.setCursor(70, lineY);
  tft.setTextColor(labelColor);
  tft.print("- LED Brightness : ");
  tft.setTextColor(valueColor);
  tft.print((globalBrightness <= 18) ? "LOW" : (globalBrightness <= 40 ? "MID" : "HIGH"));

  // 2. Vibration Strength
  lineY += spacing;
  tft.setCursor(70, lineY);
  tft.setTextColor(labelColor);
  tft.print("- Vib Strength   : ");
  tft.setTextColor(valueColor);
  if (vibStrengthMax == 0) tft.print("OFF");
  else if (vibStrengthMax <= 70) tft.print("LOW");
  else if (vibStrengthMax <= 110) tft.print("MID");
  else tft.print("HIGH");

  // 3. Encoder Mode
  lineY += spacing;
  tft.setCursor(70, lineY);
  tft.setTextColor(labelColor);
  tft.print("- Encoder Mode   : ");
  tft.setTextColor(valueColor);
  tft.print(isStudioMode ? "STUDIO" : "LIVE");

  delay(600);

  pixels.begin();
  pixels.setBrightness(20);  // 부팅 시 너무 밝지 않게 조절 (원하는 만큼 수정 가능)

  // 사용자 정의 순서 (1,2,3,4,8,7,6,5,9번 순서를 인덱스 0~8로 변환)
  int customOrder[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };

  for (int rotation = 0; rotation < 1; rotation++) {
    for (int i = 0; i < 9; i++) {
      int ledIndex = customOrder[i];  // 배열에서 켤 LED 번호를 가져옴

      pixels.setPixelColor(ledIndex, pixels.Color(100, 100, 200));  // 켜기
      pixels.show();
      delay(40);  // 순서가 복잡해지면 약간 더 느린 게(40ms) 훨씬 예쁘게 보입니다.

      pixels.setPixelColor(ledIndex, pixels.Color(0, 0, 0));  // 끄기
      pixels.show();
      delay(5);
    }
  }
  pixels.setBrightness(globalBrightness);

  delay(400);
  // --- 인코더 및 버튼 핀 설정 ---
  pinMode(PAGE_ENC_A, INPUT_PULLUP);
  pinMode(PAGE_ENC_B, INPUT_PULLUP);
  lastPageEncA = digitalRead(PAGE_ENC_A);
  pinMode(GLOBAL_BTN_PIN, INPUT_PULLUP);

  // 4개의 메인 엔코더 설정
  void (*funcs[])() = { isr0, isr1, isr2, isr3 };  // 함수 포인터 배열

  for (int i = 0; i < NUM_ENCODERS; i++) {
    pinMode(pinA[i], INPUT_PULLUP);
    pinMode(pinB[i], INPUT_PULLUP);
    pinMode(pinSW[i], INPUT_PULLUP);

    // 초기 상태 저장
    encState[i] = (digitalRead(pinA[i]) << 1) | digitalRead(pinB[i]);

    // 메인 엔코더 인터럽트 등록
    attachInterrupt(digitalPinToInterrupt(pinA[i]), funcs[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB[i]), funcs[i], CHANGE);
  }

  // ★ 페이지 엔코더 인터럽트는 루프 밖에서 한 번만 등록 (최적화)
  attachInterrupt(digitalPinToInterrupt(PAGE_ENC_A), pageEncISR, CHANGE);

  syncEncodersToPage(currentPage);
  drawStaticLayout();

  pinMode(VIB_MOTOR_PIN, OUTPUT);
  digitalWrite(VIB_MOTOR_PIN, LOW);
  // --- 부팅 진동 알림 추가 ---
  analogWrite(VIB_MOTOR_PIN, 70);  // 250 강도로 시작
  delay(150);                      // 0.3초(300ms) 대기
  analogWrite(VIB_MOTOR_PIN, 0);   // 진동 정지
  // ---------------------------
  stabilizeAfterPageChange(currentPage);

  watchdog_enable(3000, 1);  //
}

void loop() {
  watchdog_update();
  unsigned long now = millis();

  handleSystemMaintenance(now);  // 시리얼, 부트셀, 절전, 진동 타이머
  handleMidiInput();             // 외부 MIDI 수신
  handlePageNavigation(now);     // 페이지 인코더 및 설정 모드 복구(타이머 종료) 로직

  // 1. 버튼 입력은 '설정 모드 중'에도 계속 읽어야 하므로 조건문 밖으로 뺍니다.
  if (!isPagePending) {
    handleEncoderSwitches(now);
  }

  // 2. 나머지 로직(인코더 회전, 일반 UI 갱신 등)은 설정 모드가 아닐 때만 수행합니다.
  if (!isSettingTimeout && !isPagePending) {
    processMainEncoders(now);  // 4개 인코더 회전 및 스무딩
    handleGlobalButton(now);   // 글로벌 버튼 단독 동작 (설정 중엔 무시됨)
    updateUIDisplay(now);      // 일반 UI 점진적 갱신
    handleVisualFlash(now);    // LED 깜빡임 타이머
  }
}