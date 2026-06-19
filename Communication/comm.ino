#define MODEM_RX     27  
#define MODEM_TX     26  
#define MODEM_PWRKEY  4  
#define MODEM_RESET  5
 
HardwareSerial lteSerial(2);
const String TARGET_PHONE = "01094963882";
 
void setup() {
  Serial.begin(115200);
  delay(1000);
 
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(500);
 
  Serial.println("\n=============================================");
  Serial.println("  [LILYGO T-A7670E R2] 병목 제로 최종 관통 시동 ");
  Serial.println("=============================================");
 
  Serial.println("[SYSTEM] PWRKEY (4번 핀) LOW 트리거 주입...");
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(2000);
  digitalWrite(MODEM_PWRKEY, HIGH);
 
  Serial.println("[SYSTEM] 기지국 세션 동기화 대기 (8초)...");
  delay(8000);
 
  lteSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(500);
 
  // 버퍼 청소
  while(lteSerial.available()) { lteSerial.read(); }
 
  Serial.println("[SYSTEM] 모뎀 응답 확인 (AT)...");
  lteSerial.println("AT");
  delay(500);
  printClearLog();
 
  Serial.println("[SYSTEM] KT LTE APN 세션 주입...");
  lteSerial.println("AT+CGDCONT=1,\"IP\",\"lte.ktfwing.com\"");
  delay(500);
  printClearLog();
 
  // 프리징 차단을 위해 마진 대기 2초 추가
  delay(2000);
 
  Serial.println("[SYSTEM] 웰컴 문자 발송 시작...");
  if (sendSMS(TARGET_PHONE, "LILYGO T-A7670E Final Lock Success!")) {
    Serial.println("\n[SUCCESS] 기지국 관통 성공! 형님 폰 확인하십시오!");
  } else {
    Serial.println("\n[FAIL] 발송 실패. 유심 세션이나 락 상태 확인 요망.");
  }
}
 
void loop() {
  while (lteSerial.available()) { Serial.write(lteSerial.read()); }
}
 
bool sendSMS(String phone, String msg) {
  lteSerial.println("AT+CMGF=1"); delay(200);
  lteSerial.println("AT+CSCS=\"GSM\""); delay(200);
 
  // 번호 인가
  lteSerial.print("AT+CMGS=\""); lteSerial.print(phone); lteSerial.println("\"");
  delay(500);
 
  // 메시지 본문 주입
  lteSerial.print(msg);
  delay(200);
 
  // Ctrl+Z 기폭제 발사
  lteSerial.write(0x1A);
  Serial.println("[SYSTEM] 패킷 기폭제(Ctrl+Z) 투하 완료. 응답 포획 중...");
 
  // lteSerial.readString()을 버리고 자바식 뼈대 매칭으로 변경 (프리징 원천 차단)
  String response = "";
  long timeout = millis() + 10000;
 
  while (millis() < timeout) {
    while (lteSerial.available()) {
      char c = lteSerial.read();
      response += c;
      Serial.write(c); // 모뎀 대답 실시간 중계
    }
    // 영수증 번호(+CMGS) 확인 시 즉시 리턴하여 병목 폭파
    if (response.indexOf("+CMGS:") != -1 && response.indexOf("OK") != -1) {
      return true;
    }
  }
  return false;
}
 
void printClearLog() {
  while(lteSerial.available()) {
    Serial.write(lteSerial.read());
  }
  Serial.println();
}
 
잠시만 빌리겠습니다
 
제 회사 컴터 계정을 지워서요
 
