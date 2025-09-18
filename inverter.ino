#include <SPI.h>
#include "mcp2515_can.h"
#include <avr/wdt.h>


const int SPI_CS_PIN = 53;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

// CAN ID deficition_inverter
const long RxPDO1 = 0x201;
const long RxPDO2 = 0x301;
const long TxPDO1 = 0x181;
const long TxPDO2 = 0x281;


// variables
bool systemStarted = false;

int targetRPM = 0;
int actualSpeed = 0;
int batteryCurrent = 0;
byte rollover = 0;
byte faultCode = 0;
unsigned int faultSubcode = 0;


int motorCurrent = 0;
byte driveStatus = 0;
byte speedLimit = 0;
int actualTorque = 0;
byte torqueLimit = 0;

byte receivedRollover1 = 0;
byte receivedRollover2 = 0;
byte Rollover1 = 0;
byte Rollover2 = 0;

bool operational = false;

unsigned long sendtime = 0;
unsigned long receivetime1 = 0;
unsigned long receivetime2 = 0;

bool sendtimeoutActive = false;
bool receivetimeoutActive1 = false;
bool receivetimeoutActive2 = false;

const unsigned long TIMEOUT_RxPDO1 = 200;
const unsigned long TIMEOUT_TxPDO1 = 1000;
const unsigned long TIMEOUT_TxPDO2 = 200;

// digital outputs
const int INVERTER_PIN = 22;
const int SS1_PIN = 23;
const int STO_PIN = 24;


void setup() {
  Serial.begin(115200);

  waitForStart();

  pinMode(INVERTER_PIN, OUTPUT);
  pinMode(SS1_PIN, OUTPUT);
  pinMode(STO_PIN, OUTPUT);

  while (CAN_OK != CAN.begin(CAN_250KBPS)) {
    Serial.println("CAN Init Fail");
    delay(100);
  }

  Serial.println("CAN Init OK");

  delay(2000);  // 2초 대기

  digitalWrite(INVERTER_PIN, HIGH);  // 인버터 On

  delay(1000);  //1초 대기

  enableSafety();  // SS1, STO close

  delay(1000);  // 1초 대기

  // Heartbeat 상태 확인
  if (checkPreOperationalHeartbeat()) {
    Serial.println("Pre-Operational 상태 감지: Operational로 전환 중...");

    byte nmtData[2] = {0x01, 0x01}; // NMT: Start (Operational), Node 1
    CAN.sendMsgBuf(0x000, 0, 2, nmtData);
    operational = checkOperationalHeartbeat();
  } else {
    Serial.println("Pre-Operational 상태 확인 실패. 다시 시도하려면 'r' 입력");
    waitForRestartCommand();
  }

  if (operational) {
    Serial.println("Operational 상태 확인됨");
    sendInitialDriveCommand();    // 초기 '0' 명령어 전달
    sendTimeoutTimer();   // RxPDO1 timer 시작
    receiveTimeoutTimer();   // TxPDO2 timer 시작
  } else {
    Serial.println("Operational 상태 확인 실패. 다시 시도하려면 'r' 입력");
    waitForRestartCommand();
  }

}

void loop() {
  // CAN 메세지 새로운 송신
  if (Serial.available()) {
    targetRPM = Serial.parseInt();
    Serial.print("Target RPM Set to: ");
    Serial.println(targetRPM);

    sendDriveCommand();
  }
  // CAN 메세지 지속 송신
  if (millis() > sendtime+TIMEOUT_RxPDO1) {
    sendCANMessages();
    sendTimeoutTimer();
  }

  // CAN 메시지 수신
  receiveCANMessages();

}



// 시작 대기 함수
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


// 안전 기능 open 함수 (STO, SS1)
void disableSafety() {
  digitalWrite(STO_PIN, LOW); // STO open
  digitalWrite(SS1_PIN, LOW); // SS1 open
  Serial.println("STO 및 SS1 open");
  systemStarted = false;
  Serial.println("DisableSafety 작동. 다시 시도하려면 'r' 입력");
  waitForRestartCommand();
}

// 안전 기능 close 함수 (STO, SS1)
void enableSafety() {
  digitalWrite(STO_PIN, HIGH);  // STO close
  digitalWrite(SS1_PIN, HIGH);  // SS1 close
  Serial.println("STO 및 SS1 close");
}


// 초기 drive command (속도 0) 전송 함수
void sendInitialDriveCommand() {
  byte data[8];
  data[0] = 0x00; // Speed = 0 (LSB)
  data[1] = 0x00; // Speed = 0 (MSB)
  data[2] = 0x00; // 추가 신호 없음
  data[3] = 0x00;
  data[4] = 0x00; // Rollover Counter = 0
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;

  if (CAN.sendMsgBuf(RxPDO1, 0, 8, data) == CAN_OK) {
    Serial.println("CAN 메세지 전송 완료");
  } else {
    Serial.println("CAN 메세지 전송 실패");
  }
}


// heartbeat 메세지 확인 함수 (preoperational 상태 확인)
bool checkPreOperationalHeartbeat() {
  unsigned long startTime = millis();

  while (millis() - startTime < 5000) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned char len_heart = 0;
      unsigned char buf_heart[8];
      CAN.readMsgBuf(&len_heart, buf_heart);        // 데이터 읽기
      unsigned long canId_heart = CAN.getCanId(); // ID 읽기

      
      if (canID_heart == 0x701 && len_heart == 1) {
        if  (buf_heart[0] == 0x7F) {
          Serial.println("Heartbeat: pre-operational 상태 확인");
          return true;
        } else {
          Serial.print("Heartbeat 상태 확인 실패 값: ");
          Serial.print(buf_heart[0], HEX);
          return false;
        }
      }
    }
    delay(50);
  }
  Serial.println("Heartbeat 상태 확인 실패 (시간초과)");
  return false;
}


// heartbeat 메세지 확인 함수 (operational 상태 확인)
bool checkOperationalHeartbeat() {
  unsigned long startTime = millis();

  Serial.println("Heartbeat 메세지 대기 중");

  while (millis() - startTime < 5000) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned char len_heart = 0;
      unsigned char buf_heart[8];
      CAN.readMsgBuf(&len_heart, buf_heart);        // 데이터 읽기
      unsigned long canId_heart = CAN.getCanId(); // ID 읽기
      
      if (canId_heart == 0x701 && len_heart == 1) {
        if  (buf_heart[0] == 0x05) {
          Serial.println("Heartbeat: operational 상태 확인");
          return true;
        } else {
          Serial.print("Heartbeat 상태 확인 실패 값: ");
          Serial.print(buf_heart[0], HEX);
          return false;
        }
      }
    }
    delay(50);
  }
  Serial.println("Heartbeat 상태 확인 실패 (시간초과)");
  return false;
}


// Serial 입력을 통해 다시 시작 대기
void waitForRestartCommand() {
  while (true) {
    if (Serial.available()) {
      char input = Serial.read();
      if (input == 'r' || input == 'R') {
        Serial.println("다시 시작 중...");
        systemReset();
      }
    }
    delay(100);
  }
}

// 시스템 리셋 함수
void systemReset() {
  Serial.println("시스템 리셋 중...");
  wdt_enable(WDTO_250MS);
  while (true) { }
}


// RxPDO1 명령 전달 함수
void sendDriveCommand() {
  byte data[8];
  data[0] = targetRPM & 0xFF;
  data[1] = (targetRPM >> 8) & 0xFF;
  data[2] = 0x00; 
  data[3] = 0x00;
  data[4] = rollover++;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;

  if (CAN.sendMsgBuf(RxPDO1, 0, 8, data) == CAN_OK) {
    Serial.println("CAN 메시지 전송 완료");
  } else {
    Serial.println("CAN 메시지 전송 실패");
  }
}


// RxPDO1 명령 지속 전달 함수
void sendCANMessages() {
  byte data[8];
  data[0] = targetRPM & 0xFF;
  data[1] = (targetRPM >> 8) & 0xFF;
  data[2] = 0x00; 
  data[3] = 0x00;
  data[4] = rollover++;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;

  if (CAN.sendMsgBuf(RxPDO1, 0, 8, data) == CAN_OK) {
    // 아무것도 하지 않음
  } else {
    Serial.println("CAN 메시지 전송 실패");
  }
}


// 메세지 수신 함수
void receiveCANMessages() {
  unsigned long recvID;
  byte len = 0;
  byte recvData[8];

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&recvID, &len, recvData);

    // Heartbeat 메세지 처리
    if (recvID == 0x701 && len == 1) {
      if (recvData[0] == 0x05) {
        Serial.println("Heartbeat: operational 상태 유지");
      } else if (recvData[0] == 0x7F) {
        Serial.println("Heartbeat: pre-operational 상태 감지 - 시스템 초기화");
        disableSafety();
      } else {
        Serial.print("Heartbeat: 예외 발생 -");
        Serial.println(recvData[0], HEX);
        disableSafety();
      }
    }

    // TxPDO1 메시지 처리
    else if (recvID == TxPDO1 && len == 8) {
      actualSpeed = (recvData[1] << 8) | recvData[0];
      batteryCurrent = (recvData[3] << 8) | recvData[2];
      receivedRollover1 = recvData[4];
      faultCode = recvData[5];
      faultSubcode = (recvData[7] << 8) | recvData[6];

      Serial.println("-- CAN 메시지 수신 (TxPDO1) --");
      Serial.print("Actual Speed (RPM): "); Serial.println(actualSpeed);
      Serial.print("Battery Current (A): "); Serial.println(batteryCurrent * 0.1);
      Serial.print("Rollover Counter (Rx): "); Serial.println(receivedRollover1);
      Serial.print("Fault Code: "); Serial.println(faultCode);
      Serial.print("Fault Subcode: "); Serial.println(faultSubcode);
      Serial.println("---------------------------\n");

      receiveCheckTimeout1();
      receiveTimeoutTimer1();

      if (receivedRollover1 != Rollover1) {
        Serial.println("TxPDO1 롤오버 카운터 불일치");
        disableSafety();
      } else {
        Rollover1 = receivedRollover1+1;
      }
    }

    // TxPDO2 메시지 처리
    else if (recvID == TxPDO2 && len == 8) {
      motorCurrent = (recvData[1] << 8) | recvData[0];
      driveStatus = recvData[2];
      speedLimit = recvData[3];
      actualTorque = (recvData[5] << 8) | recvData[4];
      torqueLimit = recvData[6];
      receivedRollover2 = recvData[7];

      Serial.println("-- CAN 메시지 수신 (TxPDO2) --");
      Serial.print("Motor Current (A): "); Serial.println(motorCurrent * 0.1);
      Serial.print("Drive Status: "); Serial.println(driveStatus);
      Serial.print("Speed Limit: "); Serial.println(speedLimit);
      Serial.print("Actual Torque (%): "); Serial.println(actualTorque * 100 / 4096.0);
      Serial.print("Torque Limit: "); Serial.println(torqueLimit);
      Serial.print("Rollover Counter (Rx): "); Serial.println(receivedRollover2);
      Serial.println("---------------------------\n");

      receiveCheckTimeout2();
      receiveTimeoutTimer2();

      if (receivedRollover2 != Rollover2) {
        Serial.println("TxPDO2 롤오버 카운터 불일치");
        disableSafety();
      } else {
        Rollover2 = receivedRollover2+1;
      }
    }
  }
}

// RxPDO1 타임아웃 타이머 시작 함수
void sendTimeoutTimer() {
  sendtime = millis();
}


// TxPDO1 타임아웃 타이머 시작 함수
void receiveTimeoutTimer1() {
  receivetime1 = millis();
  receivetimeoutActive1 = true;
}

// TxPDO1 타임아웃 체크 함수
void receiveCheckTimeout1() {
  if (receivetimeoutActive1 && (millis()-receivetime1) > TIMEOUT_TxPDO1) {
    Serial.println("TxPDO1_Timeout: No Drive Status Received. 안전 기능 비활성화");
    disableSafety();
  }
  receivetimeoutActive1 = false;
}



// TxPDO2 타임아웃 타이머 시작 함수
void receiveTimeoutTimer2() {
  receivetime2 = millis();
  receivetimeoutActive2 = true;
}

// TxPDO2 타임아웃 체크 함수
void receiveCheckTimeout2() {
  if (receivetimeoutActive2 && (millis()-receivetime2) > TIMEOUT_TxPDO2) {
    Serial.println("TxPDO2_Timeout: No Drive Status Received. 안전 기능 비활성화");
    disableSafety();
  }
  receivetimeoutActive2 = false;
}


