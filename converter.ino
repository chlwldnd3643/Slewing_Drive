#include <SPI.h>
#include "mcp2515_can.h"
#include <avr/wdt.h>


const int SPI_CS_PIN = 9;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

// sendOne
byte start_value1 = 0x1; 
byte mode_value1 = 0x2;  // 제어 모드

byte one_value1 = 0x02;  // 출력 전압 설정: 상위 바이트 (51.3V → 513 → 0x0201)
byte two_value1 = 0x01;  // 출력 전압 설정: 하위 바이트

byte three_value1 = 0x06; // 입력 전류 설정
byte four_value1  = 0x40;

byte five_value1 = 0x01;
byte six_value1  = 0x00;
byte seven_value1 = 0x00;

// sendTwo
byte zero_value2 = 0x00;

byte one_value2 = 0x01;  // 입력 전압 설정: 상위 바이트 (25.8V → 258 → 0x0102)
byte two_value2 = 0x90;  // 입력 전압 설정: 하위 바이트

byte three_value2 = 0x06; // 출력 전류 설정 (예: -10A → 1400 → 0x0578), 10A -> 1600 -> 0x0640
byte four_value2  = 0x40;

byte five_value2 = 0x00;
byte six_value2  = 0x00;
byte seven_value2 = 0x00;

// receive
float outV = 0.0;
float outA = 0.0;
float inV = 0.0;
float inA = 0.0;

byte temp = 0;
byte disability = 0;
byte condition = 0;
byte error_code = 0;

byte life = 10;
byte lifeCheck = 0;

// else
bool systemStarted = false;
bool contin = false;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 80;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;  // 1초마다 한 번 출력

byte condition_0 = 0;

void setup() {
  Serial.begin(115200);

  waitForStart();

  while (CAN_OK != CAN.begin(CAN_250KBPS)) {
    Serial.println("CAN BUS init failed, retrying...");

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
 
      if (command == "stop") {
        stop_control();
      }
    }
    delay(100);
  }

  Serial.println("CAN BUS init OK");

  CAN.init_Mask(0, 1, 0x00000000);  // RXM0, 확장 프레임용
  CAN.init_Mask(1, 1, 0x00000000);  // RXM1, 확장 프레임용

  // 6개의 필터를 모두 0x00000000 → 모든 ID 통과
  for (byte i = 0; i < 6; i++) {
      CAN.init_Filt(i, 1, 0x00000000);  // 모든 필터를 확장 프레임용으로 설정
  }

  // byte data11[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // CAN.sendMsgBuf(0x18FF1A01, 1, 8, data11);

  // delay(2000);

  byte data11[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(0x18FF1A01, 1, 8, data11);

  delay(150);

  byte data12[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  CAN.sendMsgBuf(0x18FF1A01, 1, 8, data12);

  delay(8000);


  byte data100[8] = {0x21, 0x02, 0x01, 0x06, 0x40, 0x01, 0x00, 0x00};
  CAN.sendMsgBuf(0x18FF1A01, 1, 8, data100);

  delay(100);

  // byte data300[8] = {0x01, 0x02, 0x01, 0x3A, 0xFC, 0x01, 0x01, 0x00};
  // CAN.sendMsgBuf(0x18FF1A01, 1, 8, data300);

  // delay(50);


  unsigned long startTime = millis();
  while (!contin && (millis() - startTime < 1000)) {
    checkStartupState();
    delay(20); 
  }

  if (!contin) {
    stop_control();
  }

}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
 
    if (command == "stop") {
      stop_control();
    }
  }

  if (millis() - lastSendTime > sendInterval) {
    sendOne();
    delay(10);
    sendTwo();

    lastSendTime = millis();
  }

  receive();

  delay(90);

}


void sendOne() {
  byte zero_value1 = (mode_value1 << 4) | (start_value1 & 0x0F);
  byte data1[8] = {zero_value1, one_value1, two_value1, three_value1, four_value1, five_value1, six_value1, seven_value1};

  CAN.sendMsgBuf(0x18FF1A01, 1, 8, data1);
}

void sendTwo() {
  byte data2[8] = {zero_value2, one_value2, two_value2, three_value2, four_value2, five_value2, six_value2, seven_value2};

  CAN.sendMsgBuf(0x18FF1A02, 1, 8, data2);

}


void receive() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned char len = 0;
    unsigned char buf[8];
    CAN.readMsgBuf(&len, buf);        // 데이터 읽기
    unsigned long canId = CAN.getCanId(); // ID 읽기

    // ID에 따라 처리
    switch (canId) {
        case 0x18FF1587:
          outV = (buf[0] << 8 | buf[1]) * 0.1;
          outA = (buf[2] << 8 | buf[3]) * 0.1 - 1500;
          inV  = (buf[4] << 8 | buf[5]) * 0.1;
          inA  = (buf[6] << 8 | buf[7])  * 0.1 - 1500;

          if (millis() - lastPrintTime > printInterval) {
            Serial.print("출력전압: "); Serial.print(outV);
            Serial.print("V, 출력전류: "); Serial.print(outA);
            Serial.print("A, 입력전압: "); Serial.print(inV);
            Serial.print("V, 입력전류: "); Serial.println(inA);
            lastPrintTime = millis();
          }
          break;
        case 0x18FF2587:
          temp = buf[0];
          condition = buf[1] & 0x0F;
          disability = (buf[1] & 0xF0) >> 4;
          error_code = buf[2];
          life = buf[7];

          if (condition != 0x01) {
            Serial.print("DCDC condtion: ");
            Serial.println(condition);
          }
          if ((temp-40) > 65) {
            Serial.print("temperature warning!");
            stop_control();
          }
          if (disability != 0x00) {
	          Serial.print("Disability: ");
	          Serial.println(disability, HEX);
          }
          if (error_code != 0x00) {
            Serial.print("Error Code: ");
            Serial.println(error_code, HEX);
          }
          if (life != lifeCheck) {
            lifeCheck = life;
          } else {
            Serial.println("life warning!");
          }
            
          break;
        default:
          // 그 외 처리
          break;
    }
  }
}


void checkStartupState() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned char len = 0;
    unsigned char buf[8];
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();

    if (canId == 0x18FF2587) {
      condition_0 = (buf[1] & 0x0F);
      if (condition_0 == 0x1) {
        Serial.println("실행 상태 감지됨.");
        contin = true;
      }
      else {
        Serial.println("실행 상태 감지되지 않음.");
        Serial.print("데이터: ");
        for (int i = 0; i < len; i++) {
          Serial.print(buf[i], HEX);
          Serial.print(" ");
        }
      }
    }
    else {
      Serial.print("수신 ID: 0x"); Serial.println(canId, HEX);
      Serial.print("데이터: ");
      for (int i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();  // 한 줄 띄우기
    }
  }
}


void waitForStart() {
  Serial.println("If you want to start, please press 'S'!!");	
  while (!systemStarted) {
    if (Serial.available()) {
      char command = Serial.read();
      if (command == 'S' || command == 's') {
        systemStarted = true;
        Serial.println("시스템 시작");
      }
    }
    delay(100);
  }
}


void stop_control() {
  byte data3[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
  CAN.sendMsgBuf(0x18FF1A01, 1, 8, data3);
  delay(50);
  byte data4[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(0x18FF1A02, 1, 8, data4);

  delay(200);

  byte data5[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.sendMsgBuf(0x18FF1A01, 1, 8, data5);
  delay(200);

  Serial.println("stopping the system..");
  delay(500);

  wdt_enable(WDTO_250MS);
  while (true) { }
}


