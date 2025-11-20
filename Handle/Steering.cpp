// === 핸들 아두이노: AS5600 + 핸들 값 송신 (90% 마진 포함) ===
#include <Wire.h>
#include <AS5600.h>
#include <SoftwareSerial.h>

AS5600 magneticEncoder;

// 소프트웨어 시리얼: 핸들 → 슬루잉 보드
const uint8_t LINK_TX_PIN = 8;   // 이 핀을 슬루잉 보드의 RX로 연결
const uint8_t LINK_RX_PIN = 7;   // RX는 안 써도 됨
SoftwareSerial linkSerial(LINK_RX_PIN, LINK_TX_PIN); // (RX, TX)

// AS5600 / 핸들 범위 설정
const int   TICKS_PER_REV         = 4096;     // AS5600 12bit
const int   WRAP_THRESHOLD        = 2048;     // 랩 감지 기준 (반바퀴)
const long  MAX_TURNS_EACH_SIDE   = 2;        // 중앙 기준 ±2바퀴까지만 유효
const long  MAX_TICKS             = TICKS_PER_REV * MAX_TURNS_EACH_SIDE;

// 최대 범위 중 제어에 실제 사용하는 비율 (예: 90% 지점에서 이미 full scale)
const float SAFE_RATIO            = 0.9f;
const long  SAFE_TICKS            = (long)(MAX_TICKS * SAFE_RATIO);

// 상태 변수
long     turnCount   = 0;       // 몇 바퀴 돌았는지
uint16_t prevRaw     = 0;       // 직전 rawAngle
long     zeroAbsTick = 0;       // 전원 ON 시점 절대 tick (중앙 기준)

void init_encoder_center()
{
  magneticEncoder.begin(4);
  magneticEncoder.setDirection(AS5600_CLOCK_WISE);

  int connected = magneticEncoder.isConnected();
  Serial.print("AS5600 Connect: ");
  Serial.println(connected);

  // 부팅 직후 약간 평균 내서 초기값 안정화
  uint32_t sum = 0;
  const int N = 10;
  for(int i = 0; i < N; i++){
    sum += magneticEncoder.rawAngle();
    delay(5);
  }
  uint16_t raw0 = (uint16_t)(sum / N);

  turnCount   = 0;
  prevRaw     = raw0;
  zeroAbsTick = (long)raw0;   // 이 tick을 중앙(0) 기준으로 사용

  Serial.print("Init raw0 = ");
  Serial.println(raw0);
  Serial.println("Power-on steering position set as CENTER (0).");
}

// 핸들 위치를 –1.0 ~ +1.0으로 리턴 (±90% 지점에서 이미 풀스케일)
float get_handle_normalized()
{
  uint16_t raw = magneticEncoder.rawAngle();
  int diff = (int)raw - (int)prevRaw;

  // 랩 보정 (0↔4095 점프)
  if(diff > WRAP_THRESHOLD){
    // 예: 50 → 4000 점프 (실제로는 반대 방향 조금 회전)
    turnCount--;
  } else if(diff < -WRAP_THRESHOLD){
    // 예: 4000 → 50 점프
    turnCount++;
  }

  prevRaw = raw;

  long absTick = turnCount * (long)TICKS_PER_REV + raw;
  long relTick = absTick - zeroAbsTick;  // 전원 ON 시점 기준

  // 절대적으로 ±MAX_TICKS 밖으로 못 나가게 1차 제한
  if(relTick >  MAX_TICKS) relTick =  MAX_TICKS;
  if(relTick < -MAX_TICKS) relTick = -MAX_TICKS;

  // 실제 제어에 쓰는 건 ±SAFE_TICKS(=MAX의 90%)까지만
  if(relTick >  SAFE_TICKS) relTick =  SAFE_TICKS;
  if(relTick < -SAFE_TICKS) relTick = -SAFE_TICKS;

  // -SAFE_TICKS ~ +SAFE_TICKS → -1.0 ~ +1.0
  float norm = (float)relTick / (float)SAFE_TICKS;
  return norm;
}

void setup()
{
  Serial.begin(115200);
  while(!Serial){}

  Wire.begin();
  linkSerial.begin(57600);  // 보드 간 링크 속도

  init_encoder_center();

  Serial.println("Handle Arduino READY. Sending steering command via SoftwareSerial.");
}

void loop()
{
  float norm = get_handle_normalized();   // -1.0 ~ +1.0
  // –1000 ~ +1000 범위 정수로 변환
  int16_t cmd = (int16_t)(norm * 1000.0f);
  if(cmd >  1000) cmd =  1000;
  if(cmd < -1000) cmd = -1000;

  // 프로토콜: "S<값>\n"  (예: S0\n, S-523\n, S1000\n)
  linkSerial.print('S');
  linkSerial.println(cmd);

  // 디버깅 출력
  static unsigned long last = 0;
  if(millis() - last > 200){
    uint16_t raw = magneticEncoder.rawAngle();
    float angle = raw * AS5600_RAW_TO_DEGREES;

    Serial.print("raw=");
    Serial.print(raw);
    Serial.print("  angle=");
    Serial.print(angle, 2);
    Serial.print("  norm=");
    Serial.print(norm, 3);
    Serial.print("  cmd=");
    Serial.println(cmd);

    last = millis();
  }

  delay(20); // 약 50 Hz 전송
}
