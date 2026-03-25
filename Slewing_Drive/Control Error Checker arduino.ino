#include <SPI.h>
#include <Wire.h>
#include "mcp2515_can.h"

mcp2515_can CAN(9);
const int AS5600_ADDR = 0x36;

long totalEncoderPos = 0;
int16_t lastRaw = 0;
long revCount = 0;
long currentCmd = 0;
String serialLine = "";

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); 
    if (CAN.begin(CAN_250KBPS, MCP_16MHz) == 0) CAN.setMode(0x00);
    
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E); Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);
    if(Wire.available() >= 2) lastRaw = (Wire.read() << 8) | Wire.read();
}

void loop() {
    // 1. CAN 지령 수신
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            serialLine.trim();
            if (serialLine.startsWith("abs")) {
                currentCmd = serialLine.substring(3).toInt();
                uint8_t d[8] = { 0x23, 0x7A, 0x60, 0x00, (uint8_t)(currentCmd & 0xFF), (uint8_t)((currentCmd >> 8) & 0xFF), (uint8_t)((currentCmd >> 16) & 0xFF), (uint8_t)((currentCmd >> 24) & 0xFF) };
                CAN.sendMsgBuf(0x601, 0, 8, d);
            } else {
                uint16_t cw = serialLine.toInt();
                uint8_t d[8] = { 0x2B, 0x40, 0x60, 0x00, (uint8_t)(cw & 0xFF), (uint8_t)((cw >> 8) & 0xFF), 0, 0 };
                CAN.sendMsgBuf(0x601, 0, 8, d);
            }
            serialLine = "";
        } else if (c != '\r') serialLine += c;
    }

    // 2. 엔코더 Unwrapping (40도 반전 버그 픽스)
    Wire.beginTransmission(AS5600_ADDR); Wire.write(0x0E); Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() >= 2) {
        int16_t raw = (Wire.read() << 8) | Wire.read();
        int16_t delta = raw - lastRaw;
        
        // 4096의 절반인 2048을 넘는 급격한 변화는 회전으로 간주
        if (delta < -2048) revCount++; 
        else if (delta > 2048) revCount--;
        
        lastRaw = raw;
        totalEncoderPos = (revCount * 4096) + raw;
    }

    // 3. 파이썬 피드백 전송 (20Hz)
    static unsigned long lastT = 0;
    if (millis() - lastT > 50) {
        Serial.print(currentCmd); Serial.print(","); Serial.println(totalEncoderPos);
        lastT = millis();
    }
}
