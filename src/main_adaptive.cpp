#include <Arduino.h>

// 2-sensor straddle line tracer — 적응형 흑백 판정 버전 (UNO + L298N)
// main.cpp와 주행 메커니즘은 동일하다. 다른 건 딱 하나:
// 흑/백 경계를 상수로 박지 않고, 센서가 실제로 본 최소/최대의 중간을 경계로 쓴다.
// 바닥 재질/조명/센서 높이가 바뀌어도 IR_TH를 다시 재지 않아도 된다.

// ---- 핀 ----
const uint8_t IR_L = A3, IR_R = A1, LDR = A2;
const uint8_t TRIG = 4, ECHO = 2, BUZZ = 3;
const uint8_t ENA = 5, IN1 = 7,  IN2 = 8;    // 오른쪽 모터
const uint8_t ENB = 6, IN3 = 9,  IN4 = 10;   // 왼쪽 모터

// ---- 캘리브레이션 ----
#define CALIBRATE 0           // 1로 두면 주행 안 하고 센서 원시값 + 학습된 범위 출력
const bool BLACK_HIGH = true; // 검정에서 analog 값이 커지면 true, 작아지면 false
// 적응형 판정 knob. IR_TH_L/IR_TH_R은 여기서 사라졌다.
const int SPAN_MIN = 30;  // 흑-백 차이가 이보다 작으면 아직 둘 다 못 본 것 = 판단 보류
const int BAND_DIV = 6;   // 히스테리시스 폭 = span / BAND_DIV (작을수록 둔감)
const unsigned long DECAY_MS = 500;  // 이 주기마다 min/max를 1씩 안으로 좁힌다

const bool FLIP_R = false, FLIP_L = true;  // 모터가 반대로 돌면 true
const int TRIM_L = 100;  // 좌우 속도 보정 (%). 느린 쪽을 100으로 두고
const int TRIM_R = 95;   // 빠른 쪽을 내려서 직진을 맞춘다

const int BASE  = 100;   // 직진 PWM (방지턱 넘을 토크 확보: 너무 낮추지 말 것)
const int DIFF  = 150;   // 완만한 곡선 보정량
const int MIN_PWM = 80;  // 이보다 낮으면 모터가 정지마찰을 못 이긴다 (실측해서 조정)
const int PIVOT = 150;   // 90도 제자리 선회 PWM
const unsigned long CORNER_MS = 40; // 한쪽이 이만큼 계속 검정 = 급커브로 판정
// 코너 감지 시점엔 회전축(뒷바퀴)이 꼭짓점보다 센서~바퀴축 거리만큼 뒤에 있다.
// 그만큼 전진해서 회전축을 꼭짓점에 맞춘 뒤 제자리 회전한다.
// ADVANCE_MS = (센서~바퀴축 거리 cm / 주행속도 cm/s) * 1000
const unsigned long ADVANCE_MS = 500;
const unsigned long FINISH_MS = 120; // 양쪽 동시에 이만큼 계속 검정 = T 피니시라인

const int LDR_DARK = 850;            // INPUT_PULLUP: 어두울수록 값 큼
const unsigned long DARK_MS = 400;   // 그림자 오판 방지
const int STOP_CM = 10;              // 초음파 정지 거리
const unsigned long LOG_MS = 300;    // 상태 출력 주기 (0이면 끔)

// ---- 적응형 흑백 판정 ----
// 센서마다 "지금까지 본 가장 밝은 값 / 가장 어두운 값"을 들고 있다가
// 그 중간을 경계로 쓴다. 경계 근처에서는 직전 판정을 유지해 떨림을 막는다.
struct Track { int lo, hi; bool black; unsigned long lastDecay; };
Track tL = {1023, 0, false, 0}, tR = {1023, 0, false, 0};

bool track(Track &t, int v, unsigned long now) {
  if (v < t.lo) t.lo = v;
  if (v > t.hi) t.hi = v;

  // 한 번 본 극값을 영원히 기억하면 순간적인 글리치(로봇을 들어올림, 그림자)에
  // 범위가 영구히 벌어진다. 천천히 좁혀서 현재 바닥에 다시 맞춘다.
  // ponytail: 고정 속도 감쇠. 조명이 급변하는 환경이면 DECAY_MS를 줄인다.
  if (now - t.lastDecay >= DECAY_MS) {
    t.lastDecay = now;
    if (t.hi - t.lo > SPAN_MIN) { t.lo++; t.hi--; }
  }

  int span = t.hi - t.lo;
  // 아직 흑/백 둘 다 못 봤다 -> 직전 판정 유지 (부팅 직후엔 흰색으로 시작).
  // 출발 시 라인 위가 아니라 옆에 놓는 게 정상이므로 흰색 가정이 맞다.
  if (span < SPAN_MIN) return t.black;

  int mid  = t.lo + span / 2;
  int band = span / BAND_DIV;
  if      (v > mid + band) t.black =  BLACK_HIGH;
  else if (v < mid - band) t.black = !BLACK_HIGH;
  return t.black;                       // 밴드 안이면 유지
}

// ---- 조향 상태 ----
struct Drive { int l, r; };
static Drive mk(int l, int r) { Drive d; d.l = l; d.r = r; return d; }
int8_t blackSide = 0;            // -1 왼쪽 검정, +1 오른쪽 검정, 0 없음
unsigned long blackSince = 0;

// 코너 처리 래치: 감지 시점에 방향을 확정하고 [전진 -> 회전]을 끝까지 수행한다.
// 회전은 "반대편 센서가 검정을 물 때"까지 유지한다 (흰 바닥으로는 종료하지 않는다).
int8_t cornerDir = 0;            // 0 = 코너 아님, -1 좌(반시계), +1 우(시계)
unsigned long advanceUntil = 0;  // 이 시각까지는 축 맞추기 전진

Drive decide(bool L, bool R, unsigned long now) {
  if (cornerDir) {
    // 1단계: 회전축을 꼭짓점에 맞추는 전진. 꼭짓점 통과 구간이라 센서는 무시한다
    if (now < advanceUntil) return mk(BASE, BASE);

    // 2단계: 기억한 방향으로 제자리 회전. 반대편 센서가 라인을 물 때까지 계속
    bool opposite = (cornerDir < 0) ? R : L;
    if (!opposite)
      return cornerDir < 0 ? mk(-PIVOT, PIVOT) : mk(PIVOT, -PIVOT);
    cornerDir = 0;      // 반대편이 잡았다 -> 종료, 아래 완만 보정으로 인계
    blackSide = 0;      // 타이머 리셋해서 곧바로 재승격되지 않게
  }

  int8_t side = 0;
  if (L && !R)      side = -1;
  else if (R && !L) side = +1;
  else if (L && R)  side = blackSide;   // 교차선/코너 진입: 직전 방향 유지

  if (side != blackSide) { blackSide = side; blackSince = now; }

  if (side == 0) return mk(BASE, BASE);

  if ((now - blackSince) > CORNER_MS) {  // 계속 검정 = 라인이 꺾여 달아남 -> 코너 확정
    cornerDir = side;                    // 감지된 방향 기억 (이후 센서와 무관하게 이 방향으로 회전)
    advanceUntil = now + ADVANCE_MS;
    return mk(BASE, BASE);
  }

  if (side < 0) return mk(BASE - DIFF, BASE + DIFF);  // 왼쪽 검정 -> 오른쪽 출력 up
  else          return mk(BASE + DIFF, BASE - DIFF);  // 오른쪽 검정 -> 왼쪽 출력 up
}

// ---- 하드웨어 ----
void motor(uint8_t en, uint8_t a, uint8_t b, int v, bool flip) {
  if (flip) v = -v;
  int p = constrain(abs(v), 0, 255);
  if (p > 0 && p < MIN_PWM) p = MIN_PWM;   // 돌라고 시켰으면 최소한 돌 수는 있게
  digitalWrite(a, v >= 0);
  digitalWrite(b, v <  0);
  analogWrite(en, p);
}
void drive(Drive d) {
  motor(ENB, IN3, IN4, d.l * TRIM_L / 100, FLIP_L);
  motor(ENA, IN1, IN2, d.r * TRIM_R / 100, FLIP_R);
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

// ---- 로직 self-check (부팅 시 1회, Serial로 결과) ----
#define CHECK(c) do { if (!(c)) { Serial.print(F("SELFTEST FAIL @")); Serial.println(__LINE__); ok = false; } } while (0)
void selfTest() {
  bool ok = true;

  // --- 적응형 판정 (BLACK_HIGH = true 기준) ---
  Track t = {1023, 0, false, 0};
  CHECK(!track(t, 200, 0));       // 첫 샘플: 범위가 없으니 판단 보류 = 흰색
  CHECK(!track(t, 210, 10));      // span 10 < SPAN_MIN, 아직 보류
  CHECK( track(t, 400, 20));      // 검정 등장. span 200, mid 300 -> 검정 확정
  CHECK( track(t, 380, 30));      // 계속 검정
  CHECK( track(t, 300, 40));      // 밴드 안(267~333) -> 직전 상태(검정) 유지
  CHECK(!track(t, 210, 50));      // 흰색 복귀
  CHECK(!track(t, 300, 60));      // 밴드 안 -> 직전 상태(흰색) 유지

  blackSide = 0; blackSince = 0;
  Drive d = decide(false, false, 0);              CHECK(d.l == BASE && d.r == BASE);
  d = decide(true, false, 1000);                  CHECK(d.r > d.l);   // 완만 좌보정 (DIFF>BASE면 안쪽은 역회전)
  // --- 우코너: 감지 -> 전진 -> 왼쪽이 물 때까지 시계방향 회전 ---
  blackSide = 0; blackSince = 0; cornerDir = 0; advanceUntil = 0;
  d = decide(false, true, 2000);  CHECK(d.l > d.r);                             // 완만 우보정
  d = decide(false, true, 2121);  CHECK(cornerDir == 1 && d.l == BASE);         // 코너 확정 -> 전진
  d = decide(false, false, 2200); CHECK(d.l == BASE && d.r == BASE);            // 전진 중 (센서 무시)
  d = decide(false, false, 2700); CHECK(d.l > 0 && d.r < 0);                    // 정렬 끝 -> 시계방향 회전
  d = decide(false, false, 2900); CHECK(d.l > 0 && d.r < 0);                    // 흰 바닥이어도 회전 유지
  d = decide(false, true,  3000); CHECK(d.l > 0 && d.r < 0);                    // 오른쪽이 물어도 회전 유지
  d = decide(true,  false, 3100); CHECK(cornerDir == 0 && d.r > d.l);           // 왼쪽이 물면 종료 -> 완만 좌보정
  // --- 좌코너 ---
  blackSide = 0; blackSince = 0; cornerDir = 0; advanceUntil = 0;
  d = decide(true, false, 4000);  CHECK(d.r > d.l);                             // 완만 좌보정
  d = decide(true, false, 4121);  CHECK(cornerDir == -1 && d.l == BASE);        // 코너 확정 -> 전진
  d = decide(false, false, 4700); CHECK(d.l < 0 && d.r > 0);                    // 정렬 끝 -> 반시계 회전
  d = decide(false, true,  4800); CHECK(cornerDir == 0 && d.l > d.r);           // 오른쪽이 물면 종료
  blackSide = 0; blackSince = 0; cornerDir = 0; advanceUntil = 0;
  d = decide(false, false, 5000); CHECK(d.l == BASE && d.r == BASE);            // 복귀
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
  Serial.println(F("ADAPTIVE: 라인을 한 번 가로질러야 흑/백 범위가 잡힌다"));
  delay(1000);                       // 손 떼는 시간
}

void loop() {
  unsigned long now = millis();
  int vL = analogRead(IR_L), vR = analogRead(IR_R);
  bool L = track(tL, vL, now), R = track(tR, vR, now);
  int ldr = analogRead(LDR);

#if CALIBRATE
  Serial.print(F("L=")); Serial.print(vL);
  Serial.print(F(" [")); Serial.print(tL.lo); Serial.print('~'); Serial.print(tL.hi);
  Serial.print(L ? F("] BLACK\t") : F("] white\t"));
  Serial.print(F("R=")); Serial.print(vR);
  Serial.print(F(" [")); Serial.print(tR.lo); Serial.print('~'); Serial.print(tR.hi);
  Serial.print(R ? F("] BLACK\t") : F("] white\t"));
  Serial.print(F("LDR=")); Serial.println(ldr);
  halt(); delay(200); return;   // 0.2초마다
#endif

  // 1) 장애물
  static unsigned long lastPing = 0; static long dist = 999;
  if (now - lastPing > 60) { lastPing = now; dist = pingCm(); }

  // 2) 그림자
  static unsigned long darkSince = 0;
  if (ldr > LDR_DARK) { if (!darkSince) darkSince = now; }
  else darkSince = 0;
  bool shadowed = darkSince && (now - darkSince > DARK_MS);

  // 3) 피니시 라인 (T자). 흰 바닥을 한 번이라도 본 뒤에야 판정을 켠다.
  static bool armed = false, finished = false;
  if (!(L && R)) armed = true;
  if (armed && !finished && isFinish(L, R, now)) {
    finished = true;
    tone(BUZZ, 2600, 600);           // 완주 신호
  }

  Drive d = mk(0, 0);
  const __FlashStringHelper *state;
  if (finished)             state = F("FINISH");   // RESET 눌러야 재출발
  else if (dist < STOP_CM)  state = F("STOP-장애물");
  else if (shadowed)        state = F("STOP-그림자");
  else if (!armed)          state = F("WAIT-바닥에내려놓으세요");
  else { d = decide(L, R, now); state = F("RUN"); }
  drive(d);

  if (!finished && (dist < STOP_CM || shadowed)) {   // 치울 때까지 경고음
    static unsigned long lastBeep = 0;
    if (now - lastBeep > 800) { lastBeep = now; tone(BUZZ, 2000, 120); }
    blackSince = now;                // 재출발 시 코너 오판 방지
  }

  // 4) 상태 출력. mid는 지금 학습된 경계값 — 고정 IR_TH 대신 이걸 보고 튜닝한다
  static unsigned long lastLog = 0;
  if (LOG_MS && now - lastLog > LOG_MS) {
    lastLog = now;
    Serial.print(F("L=")); Serial.print(vL);
    Serial.print('/'); Serial.print(tL.lo + (tL.hi - tL.lo) / 2);
    Serial.print(L ? F("(B)\t") : F("(w)\t"));
    Serial.print(F("R=")); Serial.print(vR);
    Serial.print('/'); Serial.print(tR.lo + (tR.hi - tR.lo) / 2);
    Serial.print(R ? F("(B)\t") : F("(w)\t"));
    Serial.print(F("LDR=")); Serial.print(ldr);
    Serial.print(ldr > LDR_DARK ? F("(dark)\t") : F("\t"));
    Serial.print(F("dist=")); Serial.print(dist); Serial.print(F("cm\t"));
    Serial.print(F("PWM ")); Serial.print(d.l); Serial.print('/'); Serial.print(d.r);
    Serial.print('\t'); Serial.println(state);
  }
}
