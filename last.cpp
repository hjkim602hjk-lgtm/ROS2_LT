#include <Arduino.h>

// ============================================================
//  2-Sensor Straddle Line Tracer
//  Arduino UNO + L298N
//
//  - 검은 테이프는 두 IR 센서 사이로 지나감
//  - 완만한 곡선: 안쪽 바퀴를 역회전시켜 제자리 회전에 가깝게 보정
//    보정이 끝나면 짧게 반대로 되감아 남은 요잉을 죽인다
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

const int IR_TH_L = 150;
const int IR_TH_R = 200;

const bool BLACK_HIGH = true;

// 모터 방향이 반대면 true/false 변경
const bool FLIP_R = true;
const bool FLIP_L = true;

// 좌우 직진 보정
const int TRIM_L = 100;
const int TRIM_R = 99;

// -------------------- 주행 세팅 --------------------
const int BASE = 104;            // 87 x1.2

// 곡선 보정. 안쪽 바퀴를 얼마나 죽이느냐로 선회 강도를 정한다.
//   CURVE_IN > 0  : 안쪽도 전진 (예전 BASE-DIFF 방식, 가장 완만)
//   CURVE_IN = 0  : 안쪽 정지 (한쪽 바퀴 축 피벗)
//   CURVE_IN < 0  : 안쪽 역회전 (제자리 회전에 가까움)  <- 현재
const int CURVE_IN  = -90;  // 안쪽 바퀴 (-85 x1.1)
const int CURVE_OUT = 135;  // 바깥 바퀴 (110 x1.1)

// 너무 낮은 PWM에서는 모터가 정지마찰을 못 이길 수 있음
const int MIN_PWM = 85;

// motor()가 0이 아닌 명령을 MIN_PWM까지 끌어올리므로, 그보다 작은 보정값은
// 적어둔 대로 나가지 않는다. 0(정지)이거나 MIN_PWM 이상이어야 의미가 있다.
static_assert(CURVE_IN == 0 || CURVE_IN <= -MIN_PWM || CURVE_IN >= MIN_PWM,
              "CURVE_IN is swallowed by MIN_PWM");
static_assert(CURVE_OUT >= MIN_PWM && CURVE_OUT <= 255, "CURVE_OUT out of range");

// 정지 -> 출발 순간 부스트
const int START_BOOST_PWM = 225;
const unsigned long START_BOOST_MS = 200;

// 직각 회전 기본 PWM
const int PIVOT = 143;             // 135 x1.1

// 직각 회전이 처음 시작될 때만 토크 부스트
const int PIVOT_BOOST = 193;       // 175 x1.1
const unsigned long PIVOT_BOOST_MS = 100;

// 255를 넘기면 motor()가 조용히 잘라낸다. 값을 일괄 배율할 때 여기서 걸린다.
// MIN_PWM은 정지마찰 실측 하한이라 배율 대상이 아니다.
static_assert(BASE <= 255 && -CURVE_IN <= 255 && CURVE_OUT <= 255 &&
              PIVOT <= 255 && PIVOT_BOOST <= 255 && START_BOOST_PWM <= 255,
              "PWM over 255 is silently clamped by motor()");

// 아래 두 값은 PWM x1.2 상태에서 실주행으로 맞춘 값이다. 지금은 x1.15이라
// 더 느리므로 같은 거리를 가는 데 시간이 더 걸린다 -> 늘려야 할 수 있음.
//   코너를 지나쳐서 늦게 돌면 -> 줄인다
//   꼭짓점 전에 미리 돌면   -> 늘린다

// 한쪽 센서가 이 시간 이상 계속 검정을 보면 직각 코너로 판정
const unsigned long CORNER_MS = 165;   // 실주행 튜닝값 (기준선 250)

// 코너 판정 후 실제 회전을 시작하기 전에 앞으로 가는 시간
// 센서가 바퀴축보다 약 9 cm 앞에 있는 현재 구조용 테스트값
const unsigned long ADVANCE_MS = 250;   // 실주행 튜닝값 (기준선 320)

// 회전 종료 후 반대로 살짝 되감는 시간
const unsigned long BACKOFF_MS = 120;

// 완만 곡선 되감기. 보정이 끝나(양쪽 흰색) 라인이 센서 사이로 돌아온 직후에도
// 차체에는 요잉이 남아 그대로 지나쳐 반대쪽으로 넘어간다. 코너 되감기와 같은
// 개념으로 짧게 반대로 돌려 그 요잉을 죽인다.
// 0으로 두면 이 동작이 꺼지고 예전처럼 곧바로 직진한다.
// 2026-09-23 실험으로 끔. 켜 두면 되감기 동안 한쪽 바퀴가 서 있다가 다음 보정에서
// 부스트 없이 재출발해야 해서 정지마찰에 걸린다. 보정이 끝날 때마다 반대 요잉을
// 넣으므로 좌우 진동도 키운다. 기준선(copy)에는 이 기능이 없었다. 다시 켜려면 94.
const int CURVE_BACK_PWM = 0;
const unsigned long CURVE_BACK_MS = 100;

static_assert(CURVE_BACK_PWM == 0 ||
              (CURVE_BACK_PWM >= MIN_PWM && CURVE_BACK_PWM <= 255),
              "CURVE_BACK_PWM is swallowed by MIN_PWM");

// -------------------- 코너 탈출 가속 --------------------
// 코너 처리가 끝나고 직진으로 돌아온 직후 짧게 밀어준다.
// 예전 copy.cpp에서는 되감기의 0인 바퀴가 START_BOOST를 오발동시켜 우연히
// 이 효과가 났었다. 다만 한쪽 바퀴만 튀어서(좌 87 / 우 178) 좌우 비대칭이었고
// 에너지 대부분이 요잉으로 갔다. 전진 성분은 평균 약 132뿐.
// 여기서는 양 바퀴 대칭으로 줘서 전부 전진에 쓴다. 0으로 두면 꺼진다.
const int EXIT_BOOST_PWM = 190;
const unsigned long EXIT_BOOST_MS = 70;

static_assert(EXIT_BOOST_PWM == 0 ||
              (EXIT_BOOST_PWM >= BASE && EXIT_BOOST_PWM <= 255),
              "EXIT_BOOST_PWM must be between BASE and 255");

// -------------------- 기타 센서 --------------------
const int LDR_DARK = 850;
const unsigned long DARK_MS = 400;

const int STOP_CM = 7;

// pulseIn이 기다리는 최대 시간. 이 동안 루프가 멈춰 IR을 못 읽는다.
// 3000으로 줄였더니 실물에서 초음파가 전혀 동작하지 않았다(2026-09-23).
// 이 모듈은 에코가 늦게 올라오거나 이전 에코가 오래 유지되는 것으로 보인다.
// 원래 잘 되던 25000으로 되돌렸다. 줄이지 말 것.
const unsigned long ECHO_TIMEOUT_US = 25000;

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

// 완만 곡선 되감기 상태
int8_t curveBack = 0;              // 0 = 아님, -1/+1 = 직전 보정 방향
unsigned long curveBackUntil = 0;

// 코너 탈출 가속 상태
// pending으로 한 단계 미루는 이유: 코너가 끝나도 곧바로 직진이 아니라 보정이나
// 곡선 되감기를 거칠 수 있다. 그때부터 시간을 세면 정작 직진할 때 남는 게 없다.
bool exitBoostPending = false;
unsigned long exitBoostUntil = 0;

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

      // 제자리 회전. 양 바퀴를 반대로 돌려 회전축을 차체 중심에 둔다.
      // ADVANCE_MS가 바퀴축을 꼭짓점에 갖다 놓았으므로 그 점을 중심으로 돌아야
      // 축이 꼭짓점에 남는다.
      // 좌회전: 왼쪽 후진 / 오른쪽 전진
      // 우회전: 왼쪽 전진 / 오른쪽 후진
      return (cornerDir < 0)
               ? mk(-turnPWM, turnPWM)
               : mk(turnPWM, -turnPWM);
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
      // 한쪽 바퀴를 정지시켜 호를 그리며 되감는다 (제자리 회전 아님).
      // 좌회전 코너였으면 회전은 반시계였으니 되감기는 시계 방향.
      //
      // 위 회전은 제자리 스핀(회전축 = 차체 중심)이고 여기는 호(회전축 = 정지한
      // 바퀴, 중심에서 윤거의 절반)라 둘의 기하가 다르다. 의도된 선택이다.
      // 맞추고 싶으면 mk(PIVOT, -PIVOT) / mk(-PIVOT, PIVOT) 으로 바꾼다.
      return (cornerDir < 0)
               ? mk(PIVOT, 0)
               : mk(0, PIVOT);
    }

    // 코너 처리 종료
    cornerDir = 0;
    backUntil = 0;
    blackSide = 0;

    if (EXIT_BOOST_PWM) {
      exitBoostPending = true;
    }

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

  if (side != blackSide) {
    // 보정 중이다가 양쪽 흰색이 된 순간 = 라인이 센서 사이로 막 돌아왔다.
    // 남은 요잉을 죽이도록 되감기를 예약한다.
    if (side == 0 && blackSide != 0 && CURVE_BACK_PWM) {
      curveBack = blackSide;
      curveBackUntil = now + CURVE_BACK_MS;
    }
    blackSide = side;
    blackSince = now;
  }

  // 양쪽 모두 흰색
  if (side == 0) {
    if (curveBack && now < curveBackUntil) {
      // 한쪽 바퀴를 정지시켜 호를 그리며 되감는다 (제자리 회전 아님).
      // 왼쪽 검정 보정 = 반시계였으니 되감기는 시계 방향.
      //
      // 참고: 보정 mk(CURVE_IN, CURVE_OUT)은 좌우가 역회전이라 회전축이 차체
      // 중심 근처(윤거의 약 0.1배)에 있고, 이 되감기는 정지한 바퀴 위(0.5배)다.
      // 궤적을 대칭으로 맞추고 싶으면 mk(CURVE_BACK_PWM, -CURVE_BACK_PWM)으로
      // 바꾸면 되지만, 그러면 되감는 동안 전진이 0이 된다.
      return (curveBack < 0)
               ? mk(CURVE_BACK_PWM, 0)
               : mk(0, CURVE_BACK_PWM);
    }
    curveBack = 0;

    // 되감기까지 끝나고 진짜 직진으로 돌아온 순간부터 센다.
    if (exitBoostPending) {
      exitBoostPending = false;
      exitBoostUntil = now + EXIT_BOOST_MS;
    }

    if (now < exitBoostUntil) {
      return mk(EXIT_BOOST_PWM, EXIT_BOOST_PWM);
    }

    return mk(BASE, BASE);
  }

  // 다시 검정을 봤다 = 되감기 취소. 보정이 우선이다.
  curveBack = 0;

  // 보정에 들어가면 탈출 가속은 끝. 보정 비율을 흐트러뜨리면 안 된다.
  exitBoostUntil = 0;

  // 같은 쪽이 오래 검정이면 직각 코너로 승격
  if ((now - blackSince) > CORNER_MS) {
    cornerDir = side;
    advanceUntil = now + ADVANCE_MS;

    pivotStarted = false;
    pivotBoostUntil = 0;

    return mk(BASE, BASE);
  }

  // 곡선 보정: 라인이 있는 쪽 바퀴를 죽이고 반대쪽을 밀어 제자리 회전에 가깝게
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
//
// halted = 두 바퀴가 모두 0. 한쪽 바퀴만 0인 것은 되감기(mk(PIVOT, 0),
// mk(CURVE_BACK_PWM, 0))의 정상 동작이지 정지가 아니다. 그걸 정지로 보면
// 되감기가 끝날 때마다 그 바퀴가 다음 명령에서 START_BOOST_PWM으로 튀어,
// 되감기가 없애려던 요잉을 반대로 다시 넣는다.
int applyStartBoost(
  int v,
  bool halted,
  int8_t &lastSign,
  unsigned long &boostUntil,
  unsigned long now
) {
  int8_t sign = 0;

  if (v > 0) sign = 1;
  else if (v < 0) sign = -1;

  if (halted) {
    lastSign = 0;
    boostUntil = 0;
    return 0;
  }

  if (sign == 0) {
    return 0;   // 회전축이 되는 바퀴. 출발 상태를 리셋하지 않는다.
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

  bool halted = (d.l == 0 && d.r == 0);

  int leftCmd = applyStartBoost(
    d.l, halted, lastSignL, boostUntilL, now
  );

  int rightCmd = applyStartBoost(
    d.r, halted, lastSignR, boostUntilR, now
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

  long us = pulseIn(ECHO, HIGH, ECHO_TIMEOUT_US);

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
    exitBoostPending = false;
    exitBoostUntil = 0;
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
