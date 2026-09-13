#include <Arduino.h>

// SeoulTech 2026 line tracer: UNO + L298N + HC-SR04 + CdS + 2 TCRT5000.
// Read README before wiring. Default: analog IR, white floor / dark line.
// Calibration is RAM-only: repeat w, b, c after power-on/reset.
namespace Config {
const uint8_t EN_L=6, IN_L1=9, IN_L2=10, EN_R=5, IN_R1=7, IN_R2=8;
const uint8_t TRIG=4, ECHO=2, IR_L=A0, IR_R=A1, CDS=A2, START=12;
const bool DIGITAL_IR=false; // true only for DO-only modules
const uint8_t DIGITAL_LINE_LEVEL=LOW; // verify your module
const bool REVERSE_L=false, REVERSE_R=false;
const int BASE_PWM=105, MAX_PWM=165, SEARCH_PWM=75;
const int TRIM_L=0, TRIM_R=0; // measured straight-line correction
const float KP=65.0f, KD=0.30f; // error normalized to [-1,1], derivative / second
const float STOP_CM=18.0f, RELEASE_CM=24.0f;
const uint32_t SONAR_PERIOD_MS=60, SONAR_TIMEOUT_US=6000;
// No echo also happens when a board is removed. Cannot distinguish disconnection.
const uint8_t NO_ECHO_RELEASE_COUNT=5;
const uint32_t DARK_HOLD_MS=80, LIGHT_RELEASE_MS=250;
const uint32_t LOST_LIMIT_MS=450, START_DELAY_MS=1500;
const float DARK_RATIO=0.65f, LIGHT_RATIO=0.80f;
}

class Motors {
  void wheel(uint8_t en, uint8_t a, uint8_t b, int speed, bool reverse) {
    if(reverse) speed=-speed;
    speed=constrain(speed,-255,255);
    analogWrite(en,0); // disable bridge before direction update
    digitalWrite(a,speed>0?HIGH:LOW);
    digitalWrite(b,speed<0?HIGH:LOW);
    analogWrite(en,abs(speed));
  }
public:
  void begin() {
    const uint8_t pins[]={Config::EN_L,Config::IN_L1,Config::IN_L2,
                          Config::EN_R,Config::IN_R1,Config::IN_R2};
    for(uint8_t p:pins) { pinMode(p,OUTPUT); digitalWrite(p,LOW); }
  }
  void drive(int l,int r) {
    wheel(Config::EN_L,Config::IN_L1,Config::IN_L2,l,Config::REVERSE_L);
    wheel(Config::EN_R,Config::IN_R1,Config::IN_R2,r,Config::REVERSE_R);
  }
  void stop() { drive(0,0); } // enable LOW = coast, not active brake
} motors;

int whiteValue[2]={0,0}, lineValue[2]={0,0};
bool haveWhite=false, haveLine=false, haveLight=false;
int lightReference=0;
float lightFiltered=0, distanceCm=-1, leftLine=0, rightLine=0;
bool armed=false, fault=false, obstacle=true, dark=false;
bool darkCandidate=false, lightCandidate=false;
uint32_t darkSince=0, lightSince=0, startAt=0, lastSonar=0;
uint32_t lastControl=0, lastLog=0, lastSeen=0;
uint8_t clearCount=0, noEchoCount=0;
float previousError=0, filteredDerivative=0;
int lastTurn=0;

int averageAnalog(uint8_t pin) {
  long sum=0;
  for(uint8_t i=0;i<32;i++) sum+=analogRead(pin);
  return sum/32;
}

bool ready() {
  bool ir=Config::DIGITAL_IR || (haveWhite && haveLine &&
    abs(lineValue[0]-whiteValue[0])>=100 &&
    abs(lineValue[1]-whiteValue[1])>=100);
  return ir && haveLight && lightReference>=50;
}

float readLine(uint8_t pin,uint8_t index) {
  if(Config::DIGITAL_IR)
    return digitalRead(pin)==Config::DIGITAL_LINE_LEVEL?1.0f:0.0f;
  int span=lineValue[index]-whiteValue[index];
  if(abs(span)<100) return 0;
  return constrain(float(analogRead(pin)-whiteValue[index])/span,0.0f,1.0f);
}

void armRobot() {
  if(!ready()) { Serial.println(F("CALIBRATE FIRST: w, b, c")); return; }
  fault=false; armed=true; startAt=millis(); lastSeen=startAt;
  previousError=0; filteredDerivative=0; lastTurn=0;
  motors.stop(); Serial.println(F("ARMED: starts after 1.5s if clear"));
}

void commands() {
  if(!Serial.available()) return;
  char c=Serial.read();
  if(c=='x') { armed=false; motors.stop(); }
  else if(c=='g') armRobot();
  else if(!armed && (c=='w' || c=='b')) {
    int *target=c=='w'?whiteValue:lineValue;
    target[0]=averageAnalog(Config::IR_L); target[1]=averageAnalog(Config::IR_R);
    if(c=='w') haveWhite=true; else haveLine=true;
    Serial.print(c); Serial.print(':'); Serial.print(target[0]);
    Serial.print(','); Serial.println(target[1]);
  } else if(!armed && c=='c') {
    lightReference=averageAnalog(Config::CDS); lightFiltered=lightReference;
    haveLight=true; dark=false; darkCandidate=false; lightCandidate=false;
    Serial.print(F("LIGHT=")); Serial.println(lightReference);
  }
}

void startButton(uint32_t now) {
  static bool lastRaw=HIGH, stable=HIGH;
  static uint32_t changed=0;
  bool raw=digitalRead(Config::START);
  if(raw!=lastRaw) { lastRaw=raw; changed=now; }
  if(now-changed>=40 && raw!=stable) {
    stable=raw;
    if(stable==LOW) {
      if(armed) { armed=false; motors.stop(); }
      else armRobot();
    }
  }
}

void updateSonar(uint32_t now) {
  if(now-lastSonar<Config::SONAR_PERIOD_MS) return;
  lastSonar=now;
  digitalWrite(Config::TRIG,LOW); delayMicroseconds(2);
  digitalWrite(Config::TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(Config::TRIG,LOW);
  // Bounded blocking (6ms max), unlike default 1s pulseIn timeout.
  unsigned long duration=pulseIn(Config::ECHO,HIGH,Config::SONAR_TIMEOUT_US);
  distanceCm=duration?duration*0.0343f/2.0f:-1.0f;
  if(duration && distanceCm<2.0f) { // too close or invalid short pulse: hold
    obstacle=true; clearCount=0; noEchoCount=0;
  } else if(duration && distanceCm<=Config::STOP_CM) {
    obstacle=true; clearCount=0; noEchoCount=0;
  } else if(duration) {
    noEchoCount=0;
    if(distanceCm>=Config::RELEASE_CM) {
      if(clearCount<3) ++clearCount;
      if(clearCount>=3) obstacle=false;
    } else clearCount=0; // hysteresis: keep previous state
  } else {
    clearCount=0;
    if(noEchoCount<Config::NO_ECHO_RELEASE_COUNT) ++noEchoCount;
    if(noEchoCount>=Config::NO_ECHO_RELEASE_COUNT) obstacle=false;
  }
}

void updateLight(uint32_t now) {
  lightFiltered+=0.25f*(analogRead(Config::CDS)-lightFiltered);
  if(!haveLight) return;
  if(lightFiltered<lightReference*Config::DARK_RATIO) {
    lightCandidate=false;
    if(!darkCandidate) { darkCandidate=true; darkSince=now; }
    if(now-darkSince>=Config::DARK_HOLD_MS) dark=true;
  } else if(lightFiltered>lightReference*Config::LIGHT_RATIO) {
    darkCandidate=false;
    if(!lightCandidate) { lightCandidate=true; lightSince=now; }
    if(now-lightSince>=Config::LIGHT_RELEASE_MS) dark=false;
  } else { darkCandidate=false; lightCandidate=false; }
}

void followLine(uint32_t now,float dt) {
  leftLine=readLine(Config::IR_L,0); rightLine=readLine(Config::IR_R,1);
  // Geometry: at least one sensor sees line when centered. Both off = lost.
  if(leftLine<0.25f && rightLine<0.25f) {
    if(now-lastSeen>=Config::LOST_LIMIT_MS) {
      fault=true; motors.stop(); Serial.println(F("LINE LOST: x, align, g/button"));
      return;
    }
    // Slow arc toward last seen line. No blind high-speed forward motion.
    if(lastTurn<0) motors.drive(0,Config::SEARCH_PWM);
    else if(lastTurn>0) motors.drive(Config::SEARCH_PWM,0);
    else motors.stop();
    previousError=0; filteredDerivative=0;
    return;
  }
  lastSeen=now;
  float error=rightLine-leftLine; // line right => left wheel faster
  if(error>0.15f) lastTurn=1;
  else if(error< -0.15f) lastTurn=-1;
  float derivative=(error-previousError)/dt;
  filteredDerivative=0.7f*filteredDerivative+0.3f*derivative;
  previousError=error;
  float correction=Config::KP*error+Config::KD*filteredDerivative;
  correction=constrain(correction,-90.0f,90.0f);
  int base=Config::BASE_PWM-int(abs(error)*25.0f);
  int l=constrain(int(base+correction)+Config::TRIM_L,0,Config::MAX_PWM);
  int r=constrain(int(base-correction)+Config::TRIM_R,0,Config::MAX_PWM);
  motors.drive(l,r);
}

void setup() {
  motors.begin(); pinMode(Config::TRIG,OUTPUT); digitalWrite(Config::TRIG,LOW);
  pinMode(Config::ECHO,INPUT); pinMode(Config::START,INPUT_PULLUP);
  pinMode(Config::IR_L,INPUT); pinMode(Config::IR_R,INPUT);
  Serial.begin(115200);
  lightFiltered=analogRead(Config::CDS);
  Serial.println(F("w=both on floor, b=both on line, c=uncovered light, g=start, x=stop"));
}

void loop() {
  commands(); startButton(millis()); updateSonar(millis());
  uint32_t now=millis();
  if(now-lastControl>=10) {
    float dt=(now-lastControl)/1000.0f; lastControl=now;
    updateLight(now);
    if(!armed || fault || !ready() || now-startAt<Config::START_DELAY_MS || obstacle || dark) {
      motors.stop(); lastSeen=now; previousError=0; filteredDerivative=0;
    } else followLine(now,dt);
  }
  if(now-lastLog>=250 && Serial.availableForWrite()>=60) {
    lastLog=now;
    Serial.print(F("L=")); Serial.print(analogRead(Config::IR_L));
    Serial.print(F(" R=")); Serial.print(analogRead(Config::IR_R));
    Serial.print(F(" CdS=")); Serial.print(int(lightFiltered));
    Serial.print(F(" cm=")); Serial.print(distanceCm,0);
    Serial.print(F(" obs/dark/fault=")); Serial.print(obstacle);
    Serial.print('/'); Serial.print(dark); Serial.print('/'); Serial.println(fault);
  }
}
