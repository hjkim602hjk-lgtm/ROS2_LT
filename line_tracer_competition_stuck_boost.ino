#include <Arduino.h>

// ============================================================

//  2-Sensor Straddle Line Tracer

//  Arduino UNO + L298N

//

//  추가 기능

//  1. 리셋 후 첫 직진 시작 시 RESET BOOST

//  2. 양쪽 흰색 3초 유지 -> 직선 코스로 판단 -> PWM 상승

//  3. 검정 감지 즉시 FAST 해제 -> 저속 곡선 제어

//  4. 곡선/직각회전 중 마찰 정지 대비 짧은 TORQUE KICK

// ============================================================

// ============================================================

// 핀

// ============================================================

const uint8_t IR_L = A3;

const uint8_t IR_R = A1;

const uint8_t LDR  = A2;

const uint8_t TRIG = 4;

const uint8_t ECHO = 2;

const uint8_t BUZZ = 3;

const uint8_t ENA = 5, IN1 = 7, IN2 = 8;    // 오른쪽 모터

const uint8_t ENB = 6, IN3 = 9, IN4 = 10;   // 왼쪽 모터

// ============================================================

// 센서 캘리브레이션

// ============================================================

#define CALIBRATE 0

const int IR_TH_L = 100;

const int IR_TH_R = 60;

const bool BLACK_HIGH = true;

// 모터 방향

const bool FLIP_R = true;

const bool FLIP_L = true;

// 좌우 직진 보정

const int TRIM_L = 100;

const int TRIM_R = 99;

// ============================================================

// 주행 PWM

// ============================================================

// 일반 직진

const int BASE_NORMAL = 100;

// ------------------------------------------------------------

// NEW : 흰색 3초 이상 -> 직선 고속 모드

// ------------------------------------------------------------

const unsigned long STRAIGHT_CONFIRM_MS = 3000;

// 직선이라고 확인됐을 때 PWM

const int BASE_FAST = 118;

// ------------------------------------------------------------

// 곡선

//

// 검정이 들어오면 FAST 모드를 바로 끄고

// 아래 PWM으로 회전.

//

// 기존보다 조금 얌전하게 설정.

// 너무 약하면 CURVE_OUT부터 올리면 됨.

// ------------------------------------------------------------

const int CURVE_IN  = -88;

const int CURVE_OUT = 108;

// 너무 낮으면 정지마찰 때문에 모터 안 움직임

const int MIN_PWM = 80;

static_assert(

  CURVE_IN == 0 ||

  CURVE_IN <= -MIN_PWM ||

  CURVE_IN >= MIN_PWM,

  "CURVE_IN is swallowed by MIN_PWM"

);

static_assert(

  CURVE_OUT >= MIN_PWM &&

  CURVE_OUT <= 255,

  "CURVE_OUT out of range"

);

// ============================================================

// 정지 -> 재출발 부스트

// ============================================================

const int START_BOOST_PWM = 130;

const unsigned long START_BOOST_MS = 120;

// ============================================================

// NEW : 리셋 후 첫 직진 시작 토크

//

// 전원을 켜거나 RESET 버튼을 누른 후

// 최초로 양쪽 바퀴가 직진 명령을 받을 때 한 번만 동작.

// ============================================================

const int RESET_BOOST_PWM = 255;

const unsigned long RESET_BOOST_MS = 500;  // 

// ============================================================

// 곡선 주행 중 토크 재부스트

//

// 엔코더가 없으므로 실제 정지를 감지할 수는 없음.

// 대신 같은 방향 검정이 일정 시간 유지되면

// 한 번 짧게 PWM을 올려 정지마찰 극복.

// ============================================================

const unsigned long CURVE_KICK_DELAY_MS = 130;

const unsigned long CURVE_KICK_MS       = 60;

const int CURVE_KICK_ADD = 12;

// ============================================================

// 직각 회전

// ============================================================

const int PIVOT = 149;

// 처음 직각 회전 시작 부스트

const int PIVOT_BOOST = 193;

const unsigned long PIVOT_BOOST_MS = 100;

// ------------------------------------------------------------

// NEW : 직각 회전 중 재부스트

//

// 회전하다 볼캐스터/노면 마찰 때문에 힘이 부족한 경우

// 주기적으로 아주 짧게 조금만 추가.

// ------------------------------------------------------------

const unsigned long PIVOT_REKICK_START_MS  = 250;

const unsigned long PIVOT_REKICK_PERIOD_MS = 300;

const unsigned long PIVOT_REKICK_MS        = 55;

const int PIVOT_REKICK_ADD = 12;

// ============================================================
// NEW : 센서 상태가 오래 안 바뀌면 순간 토크 부스트
// 한쪽만 검정(L!=R) 상태가 오래 유지될 때만 적용
// 양쪽 흰색은 기존 3초 FAST, 양쪽 검정은 제외
// ============================================================
const unsigned long STUCK_MS = 3000;
const unsigned long STUCK_BOOST_MS = 300;
const int STUCK_BOOST_PWM = 220;


// ============================================================

// 코너 판정

// ============================================================

// 한쪽 검정이 이 시간 이상 유지 -> 직각 코너

const unsigned long CORNER_MS = 250;

// 센서가 바퀴보다 앞에 있기 때문에

// 코너를 감지한 뒤 살짝 더 앞으로 감

const unsigned long ADVANCE_MS = 320;

// 회전 완료 후 반대 방향으로 살짝 되감기

const unsigned long BACKOFF_MS = 120;

// ============================================================

// 기타 센서

// ============================================================

const int LDR_DARK = 850;

const unsigned long DARK_MS = 400;

const int STOP_CM = 10;

const unsigned long LOG_MS = 100;

// ============================================================

// Drive 구조체

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

// ============================================================

// 상태 변수

// ============================================================

// -1 : 왼쪽 검정

// +1 : 오른쪽 검정

int8_t blackSide = 0;

unsigned long blackSince = 0;

// 직각 코너

int8_t cornerDir = 0;

unsigned long advanceUntil = 0;

unsigned long backUntil = 0;

// 직각 회전 부스트

bool pivotStarted = false;

unsigned long pivotBoostUntil = 0;

unsigned long pivotStartTime = 0;

// ------------------------------------------------------------

// NEW : 직선 고속모드

// ------------------------------------------------------------

bool fastStraight = false;

unsigned long whiteSince = 0;

// 센서 상태 고정 감지
uint8_t lastSensorState = 255;
unsigned long sameStateSince = 0;
unsigned long stuckBoostUntil = 0;


// ============================================================

// PWM 크기 조금 증가시키기

// ============================================================

int addPWM(int v, int add) {

  if (v > 0) {

    v += add;

    if (v > 255) {

      v = 255;

    }

  }

  else if (v < 0) {

    v -= add;

    if (v < -255) {

      v = -255;

    }

  }

  return v;

}

// ============================================================

// 조향 결정

// ============================================================

Drive decide(bool L, bool R, unsigned long now) {

  // ==========================================================

  // 이미 직각 코너 처리 중

  // ==========================================================

  if (cornerDir) {

    // --------------------------------------------------------

    // 1. 코너 감지 후 조금 더 전진

    // --------------------------------------------------------

    if (now < advanceUntil) {

      return mk(

        BASE_NORMAL,

        BASE_NORMAL

      );

    }

    // --------------------------------------------------------

    // 2. 반대쪽 센서가 새 라인을 잡을 때까지 회전

    // --------------------------------------------------------

    bool opposite =

      (cornerDir < 0)

        ? R

        : L;

    if (!backUntil && !opposite) {

      // 실제 회전 시작

      if (!pivotStarted) {

        pivotStarted = true;

        pivotStartTime = now;

        pivotBoostUntil =

          now + PIVOT_BOOST_MS;

      }

      int turnPWM = PIVOT;

      // ------------------------------------------------------

      // 최초 회전 부스트

      // ------------------------------------------------------

      if (now < pivotBoostUntil) {

        turnPWM = PIVOT_BOOST;

      }

      // ------------------------------------------------------

      // NEW

      // 회전이 길어지는 경우 짧은 재부스트

      // ------------------------------------------------------

      else {

        unsigned long pivotElapsed =

          now - pivotStartTime;

        if (pivotElapsed >

            PIVOT_REKICK_START_MS) {

          unsigned long phase =

            (

              pivotElapsed -

              PIVOT_REKICK_START_MS

            )

            %

            PIVOT_REKICK_PERIOD_MS;

          if (phase < PIVOT_REKICK_MS) {

            turnPWM =

              min(

                255,

                PIVOT +

                PIVOT_REKICK_ADD

              );

          }

        }

      }

      // 좌회전

      // 왼쪽 정지 / 오른쪽 전진

      if (cornerDir < 0) {

        return mk(

          0,

          turnPWM

        );

      }

      // 우회전

      // 왼쪽 전진 / 오른쪽 정지

      else {

        return mk(

          turnPWM,

          0

        );

      }

    }

    // --------------------------------------------------------

    // 3. 반대 센서가 라인을 잡았으면 되감기

    // --------------------------------------------------------

    bool detector =

      (cornerDir < 0)

        ? L

        : R;

    if (!backUntil) {

      backUntil =

        now + BACKOFF_MS;

      pivotStarted = false;

      pivotBoostUntil = 0;

      pivotStartTime = 0;

    }

    if (

      now < backUntil &&

      !detector

    ) {

      // 좌회전 후 반대로 살짝

      if (cornerDir < 0) {

        return mk(

          PIVOT,

          0

        );

      }

      // 우회전 후 반대로

      else {

        return mk(

          0,

          PIVOT

        );

      }

    }

    // --------------------------------------------------------

    // 코너 처리 종료

    // --------------------------------------------------------

    cornerDir = 0;

    backUntil = 0;

    blackSide = 0;

    pivotStarted = false;

    pivotBoostUntil = 0;

    pivotStartTime = 0;

  }

  // ==========================================================

  // 일반 라인 추종

  // ==========================================================

  int8_t side = 0;

  if (L && !R) {

    side = -1;

  }

  else if (R && !L) {

    side = +1;

  }

  else if (L && R) {

    // 둘 다 검정일 경우

    // 직전 방향 유지

    side = blackSide;

  }

  // 방향 바뀜

  if (side != blackSide) {

    blackSide = side;

    blackSince = now;

  }

  // ==========================================================

  // 양쪽 흰색 -> 직진

  // ==========================================================

  if (side == 0) {

    // --------------------------------------------------------

    // NEW

    // 3초 이상 흰색이면 고속

    // --------------------------------------------------------

    if (fastStraight) {

      return mk(

        BASE_FAST,

        BASE_FAST

      );

    }

    return mk(

      BASE_NORMAL,

      BASE_NORMAL

    );

  }

  // ==========================================================

  // 검정 감지 상태

  // ==========================================================

  unsigned long blackElapsed =

    now - blackSince;

  // ==========================================================

  // 같은 방향 검정 오래 유지 -> 직각 코너

  // ==========================================================

  if (blackElapsed > CORNER_MS) {

    cornerDir = side;

    advanceUntil =

      now + ADVANCE_MS;

    pivotStarted = false;

    pivotBoostUntil = 0;

    pivotStartTime = 0;

    // 코너 진입 시에는 반드시 저속

    return mk(

      BASE_NORMAL,

      BASE_NORMAL

    );

  }

  // ==========================================================

  // NEW : 일반 곡선 중 TORQUE KICK

  // ==========================================================

  int curveIn  = CURVE_IN;

  int curveOut = CURVE_OUT;

  if (

    blackElapsed >= CURVE_KICK_DELAY_MS &&

    blackElapsed <

      (

        CURVE_KICK_DELAY_MS +

        CURVE_KICK_MS

      )

  ) {

    curveIn =

      addPWM(

        curveIn,

        CURVE_KICK_ADD

      );

    curveOut =

      addPWM(

        curveOut,

        CURVE_KICK_ADD

      );

  }

  // ==========================================================

  // 곡선 제어

  // ==========================================================

  // 왼쪽 검정

  if (side < 0) {

    return mk(

      curveIn,

      curveOut

    );

  }

  // 오른쪽 검정

  else {

    return mk(

      curveOut,

      curveIn

    );

  }

}

// ============================================================

// 센서

// ============================================================

bool isBlack(uint8_t pin, int th) {

  int v = analogRead(pin);

  return BLACK_HIGH

           ? (v > th)

           : (v < th);

}

// ============================================================

// 모터

// ============================================================

void motor(

  uint8_t en,

  uint8_t a,

  uint8_t b,

  int v,

  bool flip

) {

  if (flip) {

    v = -v;

  }

  int p =

    constrain(

      abs(v),

      0,

      255

    );

  // 정지마찰 보완

  if (

    p > 0 &&

    p < MIN_PWM

  ) {

    p = MIN_PWM;

  }

  digitalWrite(

    a,

    v >= 0

  );

  digitalWrite(

    b,

    v < 0

  );

  analogWrite(

    en,

    p

  );

}

// ============================================================

// 정지 -> 출발 START BOOST

// ============================================================

int applyStartBoost(

  int v,

  int8_t &lastSign,

  unsigned long &boostUntil,

  unsigned long now

) {

  int8_t sign = 0;

  if (v > 0) {

    sign = 1;

  }

  else if (v < 0) {

    sign = -1;

  }

  // 정지

  if (sign == 0) {

    lastSign = 0;

    boostUntil = 0;

    return 0;

  }

  // 정지 상태에서 다시 움직이기 시작

  if (lastSign == 0) {

    boostUntil =

      now + START_BOOST_MS;

  }

  lastSign = sign;

  if (

    now < boostUntil &&

    abs(v) < START_BOOST_PWM

  ) {

    return

      sign *

      START_BOOST_PWM;

  }

  return v;

}

// ============================================================

// Drive

// ============================================================

void drive(Drive d) {

  static int8_t lastSignL = 0;

  static int8_t lastSignR = 0;

  static unsigned long boostUntilL = 0;

  static unsigned long boostUntilR = 0;

  // ----------------------------------------------------------

  // NEW : 리셋 후 첫 직진 부스트

  // ----------------------------------------------------------

  static bool resetBoostDone = false;

  static unsigned long resetBoostUntil = 0;

  unsigned long now = millis();

  // 양쪽 바퀴가 처음으로 전진 명령을 받는 순간

  if (

    !resetBoostDone &&

    d.l > 0 &&

    d.r > 0

  ) {

    resetBoostDone = true;

    resetBoostUntil =

      now + RESET_BOOST_MS;

  }

  // 기존 START BOOST

  int leftCmd =

    applyStartBoost(

      d.l,

      lastSignL,

      boostUntilL,

      now

    );

  int rightCmd =

    applyStartBoost(

      d.r,

      lastSignR,

      boostUntilR,

      now

    );

  // ----------------------------------------------------------

  // NEW

  // Reset 이후 첫 직진에 조금 더 강한 토크

  // ----------------------------------------------------------

  if (now < resetBoostUntil) {

    if (

      leftCmd > 0 &&

      leftCmd < RESET_BOOST_PWM

    ) {

      leftCmd =

        RESET_BOOST_PWM;

    }

    if (

      rightCmd > 0 &&

      rightCmd < RESET_BOOST_PWM

    ) {

      rightCmd =

        RESET_BOOST_PWM;

    }

  }

  motor(

    ENB,

    IN3,

    IN4,

    leftCmd *

    TRIM_L /

    100,

    FLIP_L

  );

  motor(

    ENA,

    IN1,

    IN2,

    rightCmd *

    TRIM_R /

    100,

    FLIP_R

  );

}

// ============================================================

// 정지

// ============================================================

void halt() {

  drive(

    mk(0, 0)

  );

}

// ============================================================

// 초음파

// ============================================================

long pingCm() {

  digitalWrite(

    TRIG,

    LOW

  );

  delayMicroseconds(2);

  digitalWrite(

    TRIG,

    HIGH

  );

  delayMicroseconds(10);

  digitalWrite(

    TRIG,

    LOW

  );

  long us =

    pulseIn(

      ECHO,

      HIGH,

      25000UL

    );

  return us

           ? us / 58

           : 999;

}

// ============================================================

// Setup

// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(

    IR_L,

    INPUT

  );

  pinMode(

    IR_R,

    INPUT

  );

  pinMode(

    LDR,

    INPUT_PULLUP

  );

  pinMode(

    TRIG,

    OUTPUT

  );

  pinMode(

    ECHO,

    INPUT

  );

  pinMode(

    BUZZ,

    OUTPUT

  );

  const uint8_t outs[] = {

    ENA,

    IN1,

    IN2,

    ENB,

    IN3,

    IN4

  };

  for (

    uint8_t i = 0;

    i < sizeof(outs);

    i++

  ) {

    pinMode(

      outs[i],

      OUTPUT

    );

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

  Serial.print(

    analogRead(IR_L)

  );

  Serial.print(

    isBlack(

      IR_L,

      IR_TH_L

    )

      ? F("(BLACK)\t")

      : F("(white)\t")

  );

  Serial.print(F("R="));

  Serial.print(

    analogRead(IR_R)

  );

  Serial.print(

    isBlack(

      IR_R,

      IR_TH_R

    )

      ? F("(BLACK)\t")

      : F("(white)\t")

  );

  Serial.print(F("LDR="));

  Serial.print(

    analogRead(LDR)

  );

  Serial.println(

    analogRead(LDR) >

    LDR_DARK

      ? F("(dark)")

      : F("")

  );

  halt();

  delay(200);

  return;

#endif

  unsigned long now =

    millis();

  bool L =

    isBlack(

      IR_L,

      IR_TH_L

    );

  bool R =

    isBlack(

      IR_R,

      IR_TH_R

    );

  int ldr =

    analogRead(LDR);

  // ==========================================================
  // NEW : 센서 상태가 오래 안 바뀌는지 확인
  // ==========================================================
  uint8_t sensorState = (L ? 2 : 0) | (R ? 1 : 0);

  if (lastSensorState == 255) {
    lastSensorState = sensorState;
    sameStateSince = now;
  }
  else if (sensorState != lastSensorState) {
    lastSensorState = sensorState;
    sameStateSince = now;
  }
  else {
    if (
      (L != R) &&
      (now - sameStateSince >= STUCK_MS) &&
      (now >= stuckBoostUntil)
    ) {
      stuckBoostUntil = now + STUCK_BOOST_MS;
      sameStateSince = now;
    }
  }


  // ==========================================================

  // 1. 장애물

  // ==========================================================

  static unsigned long lastPing = 0;

  static long dist = 999;

  if (

    now - lastPing >

    60

  ) {

    lastPing = now;

    dist = pingCm();

  }

  // ==========================================================

  // 2. 그림자

  // ==========================================================

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

    (

      now - darkSince >

      DARK_MS

    );

  // ==========================================================

  // 3. 출발 게이트

  // ==========================================================

  static bool armed = false;

  if (!(L && R)) {

    armed = true;

  }

  // ==========================================================

  // NEW

  // 4. 직선 코스 판단

  // ==========================================================

  bool canDrive =

    armed &&

    dist >= STOP_CM &&

    !shadowed;

  // ----------------------------------------------------------

  // 양쪽 모두 흰색이고

  // 정상적으로 주행 가능한 상태이며

  // 직각 코너 처리 중이 아닐 때만 시간 측정

  // ----------------------------------------------------------

  if (

    canDrive &&

    !cornerDir &&

    !L &&

    !R

  ) {

    if (!whiteSince) {

      whiteSince = now;

    }

    if (

      now - whiteSince >=

      STRAIGHT_CONFIRM_MS

    ) {

      fastStraight = true;

    }

  }

  // ----------------------------------------------------------

  // 검정을 하나라도 감지하는 순간

  // 즉시 FAST 해제

  // ----------------------------------------------------------

  else {

    whiteSince = 0;

    if (

      L ||

      R ||

      cornerDir ||

      !canDrive

    ) {

      fastStraight = false;

    }

  }

  // ==========================================================

  // 5. 실제 주행

  // ==========================================================

  Drive d =

    mk(0, 0);

  const __FlashStringHelper *state;

  if (

    dist < STOP_CM

  ) {

    state =

      F("STOP-장애물");

  }

  else if (

    shadowed

  ) {

    state =

      F("STOP-그림자");

  }

  else if (

    !armed

  ) {

    state =

      F("WAIT-바닥에내려놓으세요");

  }

  else {

    d =

      decide(

        L,

        R,

        now

      );

    if (fastStraight) {

      state =

        F("RUN-FAST-STRAIGHT");

    }

    else {

      state =

        F("RUN");

    }

  }


  // ==========================================================
  // NEW : 센서 상태 고정 시 순간 토크 부스트
  // ==========================================================
  if (now < stuckBoostUntil) {
    if (d.l > 0) d.l = max(d.l, STUCK_BOOST_PWM);
    else if (d.l < 0) d.l = min(d.l, -STUCK_BOOST_PWM);

    if (d.r > 0) d.r = max(d.r, STUCK_BOOST_PWM);
    else if (d.r < 0) d.r = min(d.r, -STUCK_BOOST_PWM);
  }

  drive(d);

  // ==========================================================

  // 장애물 / 그림자 경고

  // ==========================================================

  if (

    dist < STOP_CM ||

    shadowed

  ) {

    static unsigned long lastBeep = 0;

    if (

      now - lastBeep >

      800

    ) {

      lastBeep = now;

      tone(

        BUZZ,

        2000,

        120

      );

    }

    // 다시 시작할 때

    // 코너 오판 방지

    blackSince = now;

  }

  // ==========================================================

  // 상태 출력

  // ==========================================================

  static unsigned long lastLog = 0;

  if (

    LOG_MS &&

    now - lastLog >

    LOG_MS

  ) {

    lastLog = now;

    Serial.print(F("L="));

    Serial.print(

      analogRead(IR_L)

    );

    Serial.print(

      L

        ? F("(B)\t")

        : F("(w)\t")

    );

    Serial.print(F("R="));

    Serial.print(

      analogRead(IR_R)

    );

    Serial.print(

      R

        ? F("(B)\t")

        : F("(w)\t")

    );

    Serial.print(F("LDR="));

    Serial.print(ldr);

    Serial.print(

      ldr > LDR_DARK

        ? F("(dark)\t")

        : F("\t")

    );

    Serial.print(F("dist="));

    Serial.print(dist);

    Serial.print(F("cm\t"));

    Serial.print(F("PWM "));

    Serial.print(d.l);

    Serial.print('/');

    Serial.print(d.r);

    Serial.print(F("\tFAST="));

    Serial.print(

      fastStraight

        ? F("1")

        : F("0")

    );

    Serial.print(F("	STUCK="));
    Serial.print(now < stuckBoostUntil ? F("1") : F("0"));

    Serial.print(F("\tWHITE="));

    if (whiteSince) {

      Serial.print(

        now - whiteSince

      );

    }

    else {

      Serial.print(0);

    }

    Serial.print(F("ms\t"));

    Serial.println(state);

  }

}
