#include <Arduino.h>

// 2-sensor straddle line tracer (UNO + L298N)
// 테이프는 두 센서 "사이"로 지나간다. 한쪽이 검정을 보면 반대쪽 출력을 올려 되돌린다.

// ---- 핀 ----
const uint8_t IR_L = A3, IR_R = A1, LDR = A2;
const uint8_t TRIG = 4, ECHO = 2, BUZZ = 3;
const uint8_t ENA = 5, IN1 = 7,  IN2 = 8;    // 오른쪽 모터
const uint8_t ENB = 6, IN3 = 9,  IN4 = 10;   // 왼쪽 모터

// ---- 캘리브레이션 (실측 후 조정) ----
#define CALIBRATE 1          // 1로 두면 주행 안 하고 센서 원시값만 출력
const int  IR_TH_L    = 35;   // 왼쪽 흑/백 경계 (실측값)
const int  IR_TH_R    = 30;   // 오른쪽 흑/백 경계 (실측값)
const bool BLACK_HIGH = true; // 검정에서 analog 값이 커지면 true, 작아지면 false
const bool FLIP_R = false, FLIP_L = false; // 모터가 반대로 돌면 true

const int BASE  = 130;   // 직진 PWM (방지턱 넘을 토크 확보: 너무 낮추지 말 것)
const int DIFF  = 70;    // 완만한 곡선 보정량
const int PIVOT = 170;   // 90도 제자리 선회 PWM
const unsigned long CORNER_MS = 120; // 한쪽이 이만큼 계속 검정 = 급커브로 판정
const unsigned long FINISH_MS = 100; // 양쪽 동시에 이만큼 계속 검정 = T 피니시라인

const int LDR_DARK = 850;            // INPUT_PULLUP: 어두울수록 값 큼
const unsigned long DARK_MS = 400;   // 그림자 오판 방지
const int STOP_CM = 15;              // 초음파 정지 거리

// ---- 조향 상태 ----
struct Drive { int l, r; };
static Drive mk(int l, int r) { Drive d; d.l = l; d.r = r; return d; }
int8_t blackSide = 0;            // -1 왼쪽 검정, +1 오른쪽 검정, 0 없음
unsigned long blackSince = 0;

Drive decide(bool L, bool R, unsigned long now) {
  int8_t side = 0;
  if (L && !R)      side = -1;
  else if (R && !L) side = +1;
  else if (L && R)  side = blackSide;   // 교차선/코너 진입: 직전 방향 유지

  if (side != blackSide) { blackSide = side; blackSince = now; }
  if (side == 0) return mk(BASE, BASE);

  bool corner = (now - blackSince) > CORNER_MS; // 계속 검정 = 라인이 꺾여 달아남
  if (side < 0) // 왼쪽 검정 -> 왼쪽으로 복귀 (오른쪽 출력 up)
    return corner ? mk(-PIVOT, PIVOT) : mk(BASE - DIFF, BASE + DIFF);
  else          // 오른쪽 검정 -> 오른쪽으로 복귀 (왼쪽 출력 up)
    return corner ? mk(PIVOT, -PIVOT) : mk(BASE + DIFF, BASE - DIFF);
}

// ---- 하드웨어 ----
bool isBlack(uint8_t pin, int th) {
  int v = analogRead(pin);
  return BLACK_HIGH ? (v > th) : (v < th);
}

void motor(uint8_t en, uint8_t a, uint8_t b, int v, bool flip) {
  if (flip) v = -v;
  digitalWrite(a, v >= 0);
  digitalWrite(b, v <  0);
  analogWrite(en, constrain(abs(v), 0, 255));
}
void drive(Drive d) {
  motor(ENB, IN3, IN4, d.l, FLIP_L);
  motor(ENA, IN1, IN2, d.r, FLIP_R);
}
void halt() { drive(mk(0, 0)); }

long pingCm() {                      // 60ms마다만 호출
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000UL);
  return us ? us / 58 : 999;         // 타임아웃 = 멀리 있음
}

// ---- 피니시 라인 (T자: 좌우 동시 검정) ----
unsigned long bothSince = 0;
bool isFinish(bool L, bool R, unsigned long now) {
  if (!(L && R)) { bothSince = 0; return false; }  // 한쪽이라도 흰색이면 초기화
  if (!bothSince) bothSince = now;
  return now - bothSince > FINISH_MS;              // 코너에서 스치는 순간은 무시
}

// ---- 조향 로직 self-check (부팅 시 1회, Serial로 결과) ----
#define CHECK(c) do { if (!(c)) { Serial.print(F("SELFTEST FAIL @")); Serial.println(__LINE__); ok = false; } } while (0)
void selfTest() {
  bool ok = true;
  blackSide = 0; blackSince = 0;
  Drive d = decide(false, false, 0);              CHECK(d.l == BASE && d.r == BASE);
  d = decide(true, false, 1000);                  CHECK(d.r > d.l && d.l > 0);       // 완만 좌보정
  d = decide(true, false, 1000 + CORNER_MS + 1);  CHECK(d.l < 0 && d.r > 0);         // 90도 좌선회
  d = decide(true, true,  1000 + CORNER_MS + 2);  CHECK(d.l < 0 && d.r > 0);         // 둘 다 검정=방향 유지
  blackSide = 0; blackSince = 0;
  d = decide(false, true, 2000);                  CHECK(d.l > d.r && d.r > 0);       // 완만 우보정
  d = decide(false, true, 2000 + CORNER_MS + 1);  CHECK(d.l > 0 && d.r < 0);         // 90도 우선회
  d = decide(false, false, 3000);                 CHECK(d.l == BASE && d.r == BASE); // 복귀
  bothSince = 0;
  CHECK(!isFinish(true,  true,  1000));            // 막 닿은 순간은 아직 아님
  CHECK(!isFinish(true,  true,  1000 + FINISH_MS));// 경계값은 아직 아님
  CHECK( isFinish(true,  true,  1001 + FINISH_MS));// 계속 물리면 피니시
  CHECK(!isFinish(true,  false, 2000));            // 한쪽만 검정 = 코너, 피니시 아님
  CHECK(!isFinish(true,  true,  2050));            // 초기화됐으니 다시 처음부터
  bothSince = 0;

  Serial.println(ok ? F("SELFTEST OK") : F("SELFTEST FAILED"));
  blackSide = 0; blackSince = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_L, INPUT); pinMode(IR_R, INPUT);
  pinMode(LDR, INPUT_PULLUP);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(BUZZ, OUTPUT);
  const uint8_t outs[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (uint8_t i = 0; i < sizeof(outs); i++) pinMode(outs[i], OUTPUT);
  halt();
  selfTest();
  delay(1000);                       // 손 떼는 시간
}

void loop() {
#if CALIBRATE
  Serial.print(F("L=")); Serial.print(analogRead(IR_L));
  Serial.print(isBlack(IR_L, IR_TH_L) ? F("(BLACK)\t") : F("(white)\t"));
  Serial.print(F("R=")); Serial.print(analogRead(IR_R));
  Serial.print(isBlack(IR_R, IR_TH_R) ? F("(BLACK)\t") : F("(white)\t"));
  Serial.print(F("LDR=")); Serial.print(analogRead(LDR));
  Serial.println(analogRead(LDR) > LDR_DARK ? F("(dark)") : F(""));
  halt(); delay(200); return;   // 0.2초마다
#endif

  unsigned long now = millis();

  // 1) 장애물
  static unsigned long lastPing = 0; static long dist = 999;
  if (now - lastPing > 60) { lastPing = now; dist = pingCm(); }
  // 2) 그림자
  static unsigned long darkSince = 0;
  if (analogRead(LDR) > LDR_DARK) { if (!darkSince) darkSince = now; }
  else darkSince = 0;
  bool shadowed = darkSince && (now - darkSince > DARK_MS);

  if (dist < STOP_CM || shadowed) {  // 치울 때까지 정지 대기
    halt();
    static unsigned long lastBeep = 0;
    if (now - lastBeep > 800) { lastBeep = now; tone(BUZZ, 2000, 120); }
    blackSince = now;                // 재출발 시 코너 오판 방지
    return;
  }

  // 3) 피니시 라인: 좌우 동시 검정 -> 완전 정지 (RESET 눌러야 재출발)
  static bool finished = false;
  bool L = isBlack(IR_L, IR_TH_L), R = isBlack(IR_R, IR_TH_R);
  if (!finished && isFinish(L, R, now)) {
    finished = true;
    tone(BUZZ, 2600, 600);           // 완주 신호
  }
  if (finished) { halt(); return; }  // 버튼(또는 리셋) 누를 때까지 정지 유지

  drive(decide(L, R, now));
}
