#include <SPI.h>
#include "max6675.h"
#include <avr/wdt.h>

#define RELAY_ON  LOW
#define RELAY_OFF HIGH


// === Pin Definitions ===
const int coolant_pump             = 22;  // 펌프
const int coolant_pump_sea         = 23;  // sea쪽 펌프
const int coolant_heater           = 24;  // 히터
// const int coolant_cbv_heater_power = ;  // heater쪽 밸브 파워
// const int coolant_cbv_heater       = ;  // heater쪽 밸브
const int coolant_cbv_sea_power    = 29;  // sea쪽 밸브 파워
const int coolant_cbv_sea          = 28;  // sea쪽 밸브

const int oxygen_compressor        = 27;  // 산소 컴프레서
// const int oxygen_drain_valve       = ;  // 산소 드레인 벨브

const int hydrogen_in_valve        = 26;  // 수소 입력 벨브
const int hydrogen_out_valve       = 25;  // 수소 드레인 벨브

bool systemStarted = false;

void setup() {
  Serial.begin(115200);
  waitForStart();
 
  pinMode(coolant_pump, OUTPUT);
  pinMode(coolant_pump_sea, OUTPUT);
  pinMode(coolant_heater, OUTPUT);
  pinMode(coolant_cbv_sea_power, OUTPUT);
  pinMode(coolant_cbv_sea, OUTPUT);
  pinMode(oxygen_compressor, OUTPUT);
  pinMode(hydrogen_in_valve, OUTPUT);
  pinMode(hydrogen_out_valve, OUTPUT);

  initialize_mode();

  Serial.println("");

  Serial.println("If you want to start pump, please press 'p1'!");
  Serial.println("If you want to stop pump, please press 'p0'!");
  Serial.println("If you want to start pump_sea, please press 'ps1'!");
  Serial.println("If you want to stop pump_sea, please press 'ps0'!");

  Serial.println("");

  Serial.println("If you want to start heater, please press 'h1'!");
  Serial.println("If you want to stop heater, please press 'h0'!");

  Serial.println("");

  Serial.println("If you want to start cbv_heater_power, please press 'chp1'!");
  Serial.println("If you want to stop cbv_heater_power, please press 'chp0'!");
  Serial.println("If you want to start cbv_heater, please press 'ch1'!");
  Serial.println("If you want to stop cbv_heater, please press 'ch0'!");

  Serial.println("");

  Serial.println("If you want to start cbv_sea_power, please press 'csp1'!");
  Serial.println("If you want to stop cbv_sea_power, please press 'csp0'!");
  Serial.println("If you want to start cbv_sea, please press 'cs1'!");
  Serial.println("If you want to stop cbv_sea, please press 'cs0'!"); 

  Serial.println("");

  Serial.println("If you want to start oxygen_compressor, please press 'c1'!");
  Serial.println("If you want to stop oxygen_compressor, please press 'c0'!");
  Serial.println("If you want to start oxygen_drain_valve, please press 'dv1'!");
  Serial.println("If you want to stop oxygen_drain_valve, please press 'dv0'!");

  Serial.println("");

  Serial.println("If you want to start hydrogen_in_valve, please press 'iv1'!");
  Serial.println("If you want to stop hydrogen_in_valve, please press 'iv0'!");
  Serial.println("If you want to start hydrogen_out_valve, please press 'ov1'!");
  Serial.println("If you want to stop hydrogen_out_valve, please press 'ov0'!");

  Serial.println("");

  Serial.println("If you want to stop, please press 'T'!");
  Serial.println("");

}

void loop() {
  // 정지
  if (Serial.available()) {
    String command_t = Serial.readStringUntil('\n');
    command_t.trim();

    // Serial.print("Received command: ");
    // Serial.println((int)command_t);  // 아스키 코드 출력

    if (command_t == "stop") {
      stop_control();
    }
    else if (command_t == "p1") {
      digitalWrite(coolant_pump, RELAY_ON);
    }
    else if (command_t == "p0") {
      digitalWrite(coolant_pump, RELAY_OFF);
    }
    else if (command_t == "ps1") {
      digitalWrite(coolant_pump_sea, RELAY_ON);
    }
    else if (command_t == "ps0") {
      digitalWrite(coolant_pump_sea, RELAY_OFF);
    }     

    else if (command_t == "h1") { 
      digitalWrite(coolant_heater, RELAY_ON);
    }
    else if (command_t == "h0") {
      digitalWrite(coolant_heater, RELAY_OFF);
    }
    else if (command_t == "csp1") {
      digitalWrite(coolant_cbv_sea_power, RELAY_ON);
    }
    else if (command_t == "csp0") {
      digitalWrite(coolant_cbv_sea_power, RELAY_OFF);
    }
    else if (command_t == "cs1") {
      digitalWrite(coolant_cbv_sea , RELAY_ON);
    }
    else if (command_t == "cs0") {
      digitalWrite(coolant_cbv_sea , RELAY_OFF);
    }

    else if (command_t == "c1") {
      digitalWrite(oxygen_compressor, RELAY_ON);
    }
    else if (command_t == "c0") {
      digitalWrite(oxygen_compressor, RELAY_OFF);
    }

    else if (command_t == "iv1") {
      digitalWrite(hydrogen_in_valve, RELAY_ON);
    }
    else if (command_t == "iv0") {
      digitalWrite(hydrogen_in_valve, RELAY_OFF);
    }
    else if (command_t == "ov1") {
      digitalWrite(hydrogen_out_valve, RELAY_ON);
    }
    else if (command_t == "ov0") {
      digitalWrite(hydrogen_out_valve, RELAY_OFF);
    }

    else {
      Serial.println("Unknown command.");
    }
  }
}

void waitForStart() {
  Serial.println("If you want to start, please press 'S'!");
  while (!systemStarted) {
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "S") {
        systemStarted = true;
        Serial.println("Start!");
      } else {
        Serial.println("you didn't press 'S'..");
      }
    }
    delay(200);
  }
}

// === Initialization ===
void initialize_mode() {
  digitalWrite(coolant_pump, RELAY_OFF);
  digitalWrite(coolant_pump_sea, RELAY_OFF);
  digitalWrite(coolant_heater, RELAY_OFF);
  digitalWrite(coolant_cbv_sea_power, RELAY_OFF);
  digitalWrite(coolant_cbv_sea, RELAY_OFF);
  digitalWrite(oxygen_compressor, RELAY_OFF);
  digitalWrite(hydrogen_in_valve, RELAY_OFF);
  digitalWrite(hydrogen_out_valve, RELAY_OFF);
}


// === Stop ===
void stop_control() {
  Serial.println("stopping the system..");
  delay(500);
  systemReset();
}

void systemReset() {
  Serial.println("system reset");
  wdt_enable(WDTO_250MS);
  while (true) { }
}