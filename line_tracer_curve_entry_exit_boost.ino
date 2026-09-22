#include <Arduino.h>

// ============================================================

//  2-Sensor Straddle Line Tracer

//  Arduino UNO + L298N

//

//  - 검은 테이프는 두 IR 센서 사이로 지나감

//  - 완만한 곡선: 안쪽 바퀴를 역회전시켜 제자리 회전에 가깝게 보정

//  - 직각 코너: 한쪽 바퀴 정지 + 반대쪽 바퀴 회전

//  - 직각 회전 시작 순간에만 PIVOT_BOOST 적용

//  - 출발 순간 START_BOOST 적용

// ============================================================

// -------------------- 핀 --------------------

const uint8_t IR_L = A3;

const uint8_t IR_R = A1;

const uint8_t LDR  = A2;

const uint8_t TRIG = 4;

const uint8_t ECHO = 2;

const uint8_t BUZZ = 3;

const uint8_t ENA = 5, IN1 = 7, IN2 = 8;    // 오른쪽 모터

const uint8_t ENB = 6, IN3 = 9, IN4 = 10;   // 왼쪽 모터

// -------------------- 센서 캘리브레이션 --------------------

#define CALIBRATE 0

const int IR_TH_L = 100;

const int IR_TH_R = 50;

const bool BLACK_HIGH = true;

// 모터 방향이 반대면 true/false 변경

const bool FLIP_R = true;
  
const bool FLIP_L = true;

// 좌우 직진 보정

const int TRIM_L = 100;

const int TRIM_R = 99;

// -------------------- 주행 세팅 --------------------

const int BASE = 94;            // 87 x1.1

// 곡선 보정. 안쪽 바퀴를 얼마나 죽이느냐로 선회 강도를 정한다.

//   CURVE_IN > 0  : 안쪽도 전진 (예전 BASE-DIFF 방식, 가장 완만)

//   CURVE_IN = 0  : 안쪽 정지 (한쪽 바퀴 축 피벗)

//   CURVE_IN < 0  : 안쪽 역회전 (제자리 회전에 가까움)  <- 현재

const int CURVE_IN  = -100;   // 안쪽 바퀴 (-85 x1.1)

const int CURVE_OUT = 120;   // 바깥 바퀴 (110 x1.1)

// 곡선 진입 순간 토크 부스트
const int CURVE_BOOST_IN  = -130;
const int CURVE_BOOST_OUT = 160;
const unsigned long CURVE_BOOST_MS = 100;

// 곡선 탈출 직후 직진 토크 부스트
const int CURVE_EXIT_BOOST_PWM = 135;
const unsigned long CURVE_EXIT_BOOST_MS = 130;

// 주행 PWM은 전부 같은 1.1배. MIN_PWM은 정지마찰 실측 하한이라 그대로 둔다.

// 너무 낮은 PWM에서는 모터가 정지마찰을 못 이길 수 있음

const int MIN_PWM = 80;

// motor()가 0이 아닌 명령을 MIN_PWM까지 끌어올리므로, 그보다 작은 보정값은

// 적어둔 대로 나가지 않는다. 0(정지)이거나 MIN_PWM 이상이어야 의미가 있다.

static_assert(CURVE_IN == 0 || CURVE_IN <= -MIN_PWM || CURVE_IN >= MIN_PWM,

              "CURVE_IN is swallowed by MIN_PWM");

static_assert(CURVE_OUT >= MIN_PWM && CURVE_OUT <= 255, "CURVE_OUT out of range");

// 정지 -> 출발 순간 부스트

const int START_BOOST_PWM = 202;   // 180 x1.1

const unsigned long START_BOOST_MS = 140;

// 직각 회전 기본 PWM

const int PIVOT = 153;             // 135 x1.1

// 직각 회전이 처음 시작될 때만 토크 부스트

const int PIVOT_BOOST = 193;       // 175 x1.1

const unsigned long PIVOT_BOOST_MS = 100;

// 한쪽 센서가 이 시간 이상 계속 검정을 보면 직각 코너로 판정

const unsigned long CORNER_MS = 250;

// 코너 판정 후 실제 회전을 시작하기 전에 앞으로 가는 시간

// 센서가 바퀴축보다 약 9 cm 앞에 있는 현재 구조용 테스트값

const unsigned long ADVANCE_MS = 250;

// 회전 종료 후 반대로 살짝 되감는 시간

const unsigned long BACKOFF_MS = 120;

// -------------------- 기타 센서 --------------------

const int LDR_DARK = 850;

const unsigned long DARK_MS = 400;

const int STOP_CM = 10;

const unsigned long LOG_MS = 100;

// ============================================================

// 조향 상태

// ============================================================

struct Drive {

  int l;

  int r;

};

static Drive mk(int l, int r) {

  Drive d;

  d.l = l;

  d.r = r;

  return d;

}

int8_t blackSide = 0;       // -1: 왼쪽 검정, +1: 오른쪽 검정

unsigned long blackSince = 0;

// 곡선 탈출 후 짧은 직진 부스트 종료 시각
unsigned long curveExitBoostUntil = 0;

// 코너 상태

int8_t cornerDir = 0;       // -1: 좌회전, +1: 우회전

unsigned long advanceUntil = 0;

unsigned long backUntil = 0;

// 직각 회전 전용 부스트 상태

bool pivotStarted = false;

unsigned long pivotBoostUntil = 0;

// ============================================================

// 조향 결정

// ============================================================

Drive decide(bool L, bool R, unsigned long now) {

  // ----------------------------------------------------------

  // 이미 직각 코너 처리 중

  // ----------------------------------------------------------

  if (cornerDir) {

    // 1) 센서가 코너를 먼저 봤으므로 바퀴축을 코너 쪽으로 조금 전진

    if (now < advanceUntil) {

      return mk(BASE, BASE);

    }

    // 2) 반대편 센서가 새 라인을 잡을 때까지 회전

    bool opposite = (cornerDir < 0) ? R : L;

    if (!backUntil && !opposite) {

      // 실제 회전이 시작되는 순간 딱 한 번 부스트

      if (!pivotStarted) {

        pivotStarted = true;

        pivotBoostUntil = now + PIVOT_BOOST_MS;

      }

      int turnPWM = (now < pivotBoostUntil) ? PIVOT_BOOST : PIVOT;

      // 볼캐스터 마찰을 줄이기 위해 한쪽 바퀴를 정지시키고 회전

      // 좌회전: 왼쪽 정지 / 오른쪽 전진

      // 우회전: 왼쪽 전진 / 오른쪽 정지

      return (cornerDir < 0)

               ? mk(0, turnPWM)

               : mk(turnPWM, 0);

    }

    // 3) 반대편 센서가 라인을 잡으면 살짝 되감아서 중앙 쪽으로 복귀

    bool detector = (cornerDir < 0) ? L : R;

    if (!backUntil) {

      backUntil = now + BACKOFF_MS;

      // 다음 코너를 위해 회전 부스트 상태 초기화

      pivotStarted = false;

      pivotBoostUntil = 0;

    }

    if (now < backUntil && !detector) {

      // 회전과 같은 방식(한쪽 바퀴 정지)으로 반대 방향. 회전은 호를 그리는데

      // 되감기만 제자리 스핀이면 궤적이 대칭이 아니라 중앙으로 안 돌아온다.

      return (cornerDir < 0)

               ? mk(PIVOT, 0)

               : mk(0, PIVOT);

    }

    // 코너 처리 종료

    cornerDir = 0;

    backUntil = 0;

    blackSide = 0;

    pivotStarted = false;

    pivotBoostUntil = 0;

  }

  // ----------------------------------------------------------

  // 일반 라인 추종

  // ----------------------------------------------------------

  int8_t side = 0;

  if (L && !R) {

    side = -1;

  }

  else if (R && !L) {

    side = +1;

  }

  else if (L && R) {

    // 교차선/코너 진입 시 직전 방향 유지

    side = blackSide;

  }
  // 곡선(한쪽 검정) -> 양쪽 흰색으로 복귀하는 순간
  // 짧은 직진 토크 부스트 예약
  if (side == 0 && blackSide != 0) {
    curveExitBoostUntil = now + CURVE_EXIT_BOOST_MS;
  }

  if (side != blackSide) {
    blackSide = side;
    blackSince = now;
  }

  // 양쪽 모두 흰색이면 직진
  if (side == 0) {
    if (now < curveExitBoostUntil) {
      return mk(CURVE_EXIT_BOOST_PWM, CURVE_EXIT_BOOST_PWM);
    }
    return mk(BASE, BASE);
  }

  // 같은 쪽이 오래 검정이면 직각 코너로 승격

  if ((now - blackSince) > CORNER_MS) {

    cornerDir = side;

    advanceUntil = now + ADVANCE_MS;

    pivotStarted = false;

    pivotBoostUntil = 0;

    return mk(BASE, BASE);

  }

  // 곡선 보정: 진입 순간에는 짧게 토크 부스트
  unsigned long curveElapsed = now - blackSince;

  if (curveElapsed < CURVE_BOOST_MS) {
    if (side < 0) {
      return mk(CURVE_BOOST_IN, CURVE_BOOST_OUT);
    }
    else {
      return mk(CURVE_BOOST_OUT, CURVE_BOOST_IN);
    }
  }

  // 부스트 이후 기존 곡선 출력
  if (side < 0) {
    return mk(CURVE_IN, CURVE_OUT);   // 왼쪽 검정 -> 반시계
  }
  else {
    return mk(CURVE_OUT, CURVE_IN);   // 오른쪽 검정 -> 시계
  }

}

// ============================================================

// 하드웨어

// ============================================================

bool isBlack(uint8_t pin, int th) {

  int v = analogRead(pin);

  return BLACK_HIGH ? (v > th) : (v < th);

}

void motor(uint8_t en, uint8_t a, uint8_t b, int v, bool flip) {

  if (flip) {

    v = -v;

  }

  int p = constrain(abs(v), 0, 255);

  // 모터가 움직이라는 명령을 받았는데 PWM이 너무 낮으면 최소 PWM 적용

  if (p > 0 && p < MIN_PWM) {

    p = MIN_PWM;

  }

  digitalWrite(a, v >= 0);

  digitalWrite(b, v < 0);

  analogWrite(en, p);

}

// 정지 상태에서 출발하는 순간에만 부스트.

// 주행 중 전진<->후진 방향이 바뀐다고 START_BOOST를 다시 걸지는 않는다.

// 직각회전 토크는 decide()의 PIVOT_BOOST에서 별도로 처리한다.

int applyStartBoost(

  int v,

  int8_t &lastSign,

  unsigned long &boostUntil,

  unsigned long now

) {

  int8_t sign = 0;

  if (v > 0) sign = 1;

  else if (v < 0) sign = -1;

  if (sign == 0) {

    lastSign = 0;

    boostUntil = 0;

    return 0;

  }

  if (lastSign == 0) {

    boostUntil = now + START_BOOST_MS;

  }

  lastSign = sign;

  if (now < boostUntil && abs(v) < START_BOOST_PWM) {

    return sign * START_BOOST_PWM;

  }

  return v;

}

void drive(Drive d) {

  static int8_t lastSignL = 0;

  static int8_t lastSignR = 0;

  static unsigned long boostUntilL = 0;

  static unsigned long boostUntilR = 0;

  unsigned long now = millis();

  int leftCmd = applyStartBoost(

    d.l, lastSignL, boostUntilL, now

  );

  int rightCmd = applyStartBoost(

    d.r, lastSignR, boostUntilR, now

  );

  motor(

    ENB, IN3, IN4,

    leftCmd * TRIM_L / 100,

    FLIP_L

  );

  motor(

    ENA, IN1, IN2,

    rightCmd * TRIM_R / 100,

    FLIP_R

  );

}

void halt() {

  drive(mk(0, 0));

}

// ============================================================

// 초음파

// ============================================================

long pingCm() {

  digitalWrite(TRIG, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG, LOW);

  long us = pulseIn(ECHO, HIGH, 25000UL);

  return us ? us / 58 : 999;

}

// ============================================================

// Setup

// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(IR_L, INPUT);

  pinMode(IR_R, INPUT);

  pinMode(LDR, INPUT_PULLUP);

  pinMode(TRIG, OUTPUT);

  pinMode(ECHO, INPUT);

  pinMode(BUZZ, OUTPUT);

  const uint8_t outs[] = {

    ENA, IN1, IN2,

    ENB, IN3, IN4

  };

  for (uint8_t i = 0; i < sizeof(outs); i++) {

    pinMode(outs[i], OUTPUT);

  }

  halt();

  // 차를 바닥에 놓을 시간

  delay(1000);

}

// ============================================================

// Main Loop

// ============================================================

void loop() {

#if CALIBRATE

  Serial.print(F("L="));

  Serial.print(analogRead(IR_L));

  Serial.print(

    isBlack(IR_L, IR_TH_L)

      ? F("(BLACK)\t")

      : F("(white)\t")

  );

  Serial.print(F("R="));

  Serial.print(analogRead(IR_R));

  Serial.print(

    isBlack(IR_R, IR_TH_R)

      ? F("(BLACK)\t")

      : F("(white)\t")

  );

  Serial.print(F("LDR="));

  Serial.print(analogRead(LDR));

  Serial.println(

    analogRead(LDR) > LDR_DARK

      ? F("(dark)")

      : F("")

  );

  halt();

  delay(200);

  return;

#endif

  unsigned long now = millis();

  bool L = isBlack(IR_L, IR_TH_L);

  bool R = isBlack(IR_R, IR_TH_R);

  int ldr = analogRead(LDR);

  // ----------------------------------------------------------

  // 1) 장애물

  // ----------------------------------------------------------

  static unsigned long lastPing = 0;

  static long dist = 999;

  if (now - lastPing > 60) {

    lastPing = now;

    dist = pingCm();

  }

  // ----------------------------------------------------------

  // 2) 그림자

  // ----------------------------------------------------------

  static unsigned long darkSince = 0;

  if (ldr > LDR_DARK) {

    if (!darkSince) {

      darkSince = now;

    }

  }

  else {

    darkSince = 0;

  }

  bool shadowed =

    darkSince &&

    (now - darkSince > DARK_MS);

  // ----------------------------------------------------------

  // 3) 출발 게이트

  // ----------------------------------------------------------

  static bool armed = false;

  // 양쪽 센서가 모두 검정이 아닌 상태를 한 번이라도 보면 출발 허용

  if (!(L && R)) {

    armed = true;

  }

  Drive d = mk(0, 0);

  const __FlashStringHelper *state;

  if (dist < STOP_CM) {

    state = F("STOP-장애물");

  }

  else if (shadowed) {

    state = F("STOP-그림자");

  }

  else if (!armed) {

    state = F("WAIT-바닥에내려놓으세요");

  }

  else {

    d = decide(L, R, now);

    state = F("RUN");

  }

  drive(d);

  // 장애물/그림자 감지 시 경고음

  if (dist < STOP_CM || shadowed) {

    static unsigned long lastBeep = 0;

    if (now - lastBeep > 800) {

      lastBeep = now;

      tone(BUZZ, 2000, 120);

    }

    // 정지 후 다시 출발할 때 코너 오판 방지

    blackSince = now;

  }

  // ----------------------------------------------------------

  // 4) 상태 출력

  // ----------------------------------------------------------

  static unsigned long lastLog = 0;

  if (LOG_MS && now - lastLog > LOG_MS) {

    lastLog = now;

    Serial.print(F("L="));

    Serial.print(analogRead(IR_L));

    Serial.print(L ? F("(B)\t") : F("(w)\t"));

    Serial.print(F("R="));

    Serial.print(analogRead(IR_R));

    Serial.print(R ? F("(B)\t") : F("(w)\t"));

    Serial.print(F("LDR="));

    Serial.print(ldr);

    Serial.print(ldr > LDR_DARK ? F("(dark)\t") : F("\t"));

    Serial.print(F("dist="));

    Serial.print(dist);

    Serial.print(F("cm\t"));

    Serial.print(F("PWM "));

    Serial.print(d.l);

    Serial.print('/');

    Serial.print(d.r);

    Serial.print(F("\t"));

    Serial.println(state);

  }

}
