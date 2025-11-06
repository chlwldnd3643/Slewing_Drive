#include <Wire.h>
#include "Adafruit_AS5600.h"

Adafruit_AS5600 as5600;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Leonardo/Micro용

  if (!as5600.begin()) {
    Serial.println("AS5600을 찾을 수 없습니다!");
    while (1) delay(10);
  }

  Serial.println("AS5600 시작됨!");
}

void loop() {
  // Raw angle 값 (0 ~ 4095, 12비트)
  uint16_t rawAngle = as5600.readAngle();

  // 도(degree) 단위로 변환 (0 ~ 360도)
  float angleDeg = rawAngle * (360.0 / 4096.0);

  Serial.print("Raw: ");
  Serial.print(rawAngle);
  Serial.print(" | Angle: ");
  Serial.print(angleDeg, 2); // 소수점 2자리까지
  Serial.println(" deg");

  delay(100); // 출력 간격 조절
}
