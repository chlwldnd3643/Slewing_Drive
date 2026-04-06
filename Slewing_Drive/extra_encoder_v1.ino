#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

// =========================================================
// 1. 오염(Pollution) 파라미터 설정
// =========================================================
bool e1_on = false; int e1_off = 150;  // E1: 상수 오프셋 (Bias) - 기계적 틀어짐 모사
bool e2_on = false; float e2_r = 0.2f; // E2: 무작위 노이즈 (Noise) - 전기적/기계적 진동 모사

// =========================================================
// 2. 유틸리티 함수
// =========================================================
void printStatus() {
  Serial.println(F("\n-------------------------------------------"));
  Serial.print(F("[STATUS]  E1(Offset): ")); Serial.print(e1_on ? F("ON ") : F("OFF"));
  Serial.print(F(" | Value: ")); Serial.println(e1_off);
  Serial.print(F("[STATUS]  E2(Noise) : ")); Serial.print(e2_on ? F("ON ") : F("OFF"));
  Serial.print(F(" | Ratio: ")); Serial.println(e2_r, 2);
  Serial.println(F("-------------------------------------------"));
}

void printHelp() {
  Serial.println(F("\n======= [ POLLUTION CONTROL MENU ] ======="));
  Serial.println(F("  h       : 도움말 표시 (Help)"));
  Serial.println(F("  e1      : 상수 오프셋(E1) ON/OFF 토글"));
  Serial.println(F("  i1 <n>  : 오프셋 값 설정 (예: i1 200)"));
  Serial.println(F("  e2      : 무작위 노이즈(E2) ON/OFF 토글"));
  Serial.println(F("  i2 <n>  : 노이즈 비율 설정 (예: i2 0.5)"));
  Serial.println(F("=========================================="));
}

// =========================================================
// 3. 메인 초기화 및 루프
// =========================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  encoder.begin(4);
  randomSeed(analogRead(A0));

  Serial.println(F("\n\n##########################################"));
  Serial.println(F("#   SIGNAL POLLUTER UNIT INITIALIZED     #"));
  Serial.println(F("##########################################"));
  
  printHelp();
  printStatus();
  
  Serial.println(F(">> DATA STREAM STARTING..."));
}

void loop() {
  // --- A. 파이썬 및 터미널 명령어 처리 ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); 
    cmd.trim();
    bool changed = false;

    if (cmd == "h") { printHelp(); printStatus(); }
    else if (cmd == "e1") { e1_on = !e1_on; changed = true; }
    else if (cmd == "e2") { e2_on = !e2_on; changed = true; }
    else if (cmd.startsWith("i1 ")) { e1_off = cmd.substring(3).toInt(); changed = true; }
    else if (cmd.startsWith("i2 ")) { e2_r = cmd.substring(3).toFloat(); changed = true; }
    // 참고: 파이썬에서 보내는 "en", "abs x" 등의 모터 구동 명령은 이 노드에서는 무시됩니다.

    if (changed) {
      Serial.println(F("\n[OK] Parameter Updated!"));
      printStatus(); 
    }
  }

  // --- B. 데이터 취득 및 오염 로직 적용 ---
  int16_t norm = map(encoder.rawAngle(), 0, 4095, -1000, 1000);
  int32_t poll = norm;

  if (e1_on) poll += e1_off;
  if (e2_on) {
    float r = ((float)random(-1000, 1001) / 1000.0f) * e2_r;
    poll = (int32_t)(poll * (1.0f + r));
  }
  int16_t poll_final = (int16_t)constrain(poll, -1000, 1000);

  // --- C. 파이썬 파서 규격에 맞춘 이중 데이터 송신 ---
  // 포맷: "P,내부엔코더값(순수),외부엔코더값(오염)"
  Serial.print(F("P,"));
  Serial.print(norm);        
  Serial.print(F(","));
  Serial.println(poll_final); 

  delay(20); // 50Hz 전송 주기 (파이썬 수신 부하 조절)
}