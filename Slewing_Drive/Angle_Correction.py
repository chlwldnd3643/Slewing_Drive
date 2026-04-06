import time
import threading
import queue
import random
from typing import Optional, Tuple

import numpy as np
import serial

# =========================================================
# 1. SERIAL PORT & KINEMATIC SETTINGS
# =========================================================
COM_HANDLE = "COM11"  # 핸들 Arduino (AS5600 -> Sxxx 전송)
COM_SLEW = "COM3"  # 슬루잉 드라이브 제어 Arduino (Feedback 수신 포함)
BAUD_HANDLE = 57600
BAUD_SLEW = 115200
READ_TIMEOUT = 0.05

# 기계적 기어비 기반 상수
INC_PER_DEG = (182.0 * 70.0 * 150.0) / 62.0
ENC_RES = 4096.0  # AS5600 12-bit 분해능
ECC_PERIOD = INC_PER_DEG * (360.0 / 62.0)
MESH_PERIOD = ECC_PERIOD / 150.0

MAX_HANDLE_DEG = 90.0  # 핸들 가동 범위
SEND_INTERVAL_SEC = 0.03
MOVE_TO_PREDICTED = True  # True: 예측값 기반 이동 / False: 보정명령 직접 이동


# =========================================================
# 2. INTERNAL ERROR INJECTOR (5대 기계 결함 시뮬레이터)
# =========================================================
class InternalErrorInjector:
    def __init__(self):
        self.e1_on = self.e3_on = self.e4_on = self.e5_on = self.e6_on = False
        self._a_err, self._b_err = 1.0, 0.0
        self._ecc_amp = self._mesh_amp = self._backlash = 0.0
        self._e6_cw_ratio = self._e6_ccw_ratio = 0.0
        self._accumulated_e6_err = 0.0
        self.prev_cmd = 0.0
        self.current_dir = 1

    def toggle_e1(self):
        self.e1_on = not self.e1_on
        if self.e1_on:
            self._a_err = 1.0 + random.choice([-1.0, 1.0]) * random.uniform(0.15, 0.25)
            self._b_err = 0.0
        else:
            self._a_err = 1.0
        return self.e1_on

    def toggle_e3(self):
        self.e3_on = not self.e3_on
        if self.e3_on: self._ecc_amp = INC_PER_DEG * random.uniform(0.1, 0.3)
        return self.e3_on

    def toggle_e4(self):
        self.e4_on = not self.e4_on
        if self.e4_on: self._mesh_amp = INC_PER_DEG * random.uniform(0.02, 0.05)
        return self.e4_on

    def toggle_e5(self):
        self.e5_on = not self.e5_on
        if self.e5_on: self._backlash = INC_PER_DEG * random.uniform(0.2, 0.5)
        return self.e5_on

    def toggle_e6(self):
        self.e6_on = not self.e6_on
        if self.e6_on:
            self._e6_cw_ratio = random.choice([-1.0, 1.0]) * random.uniform(0.20, 0.30)
            self._e6_ccw_ratio = random.choice([-1.0, 1.0]) * random.uniform(0.20, 0.30)
            self._accumulated_e6_err = 0.0
        else:
            self._accumulated_e6_err = 0.0
        return self.e6_on

    def apply(self, cmd_val: float, update_state: bool = True) -> float:
        delta = cmd_val - self.prev_cmd
        direction = 1 if delta >= 0 else -1
        err = 0.0
        if self.e1_on: err += (self._a_err * cmd_val + self._b_err) - cmd_val
        if self.e3_on: err += self._ecc_amp * np.sin(2.0 * np.pi * cmd_val / ECC_PERIOD)
        if self.e4_on: err += self._mesh_amp * np.sin(2.0 * np.pi * cmd_val / MESH_PERIOD)
        if self.e5_on: err += (self._backlash / 2.0) if direction >= 0 else -(self._backlash / 2.0)

        e6_step_err = 0.0
        if self.e6_on:
            e6_step_err = delta * (self._e6_cw_ratio if delta >= 0 else self._e6_ccw_ratio)
            if update_state: self._accumulated_e6_err += e6_step_err
            err += (self._accumulated_e6_err if update_state else (self._accumulated_e6_err + e6_step_err))

        if update_state:
            self.prev_cmd = cmd_val
            self.current_dir = direction
        return cmd_val + err


# =========================================================
# 3. CALIBRATION FILTER (데이터 학습 및 보정 엔진)
# =========================================================
class CalibrationFilter:
    def __init__(self):
        self.r1_on = self.r2_on = self.r3_on = self.r4_on = self.r5_on = self.r6_on = self.r7_on = False
        self.k_est, self.m_est = 0.0, 1.0
        self.ecc_A_est = self.ecc_B_est = self.mesh_A_est = self.mesh_B_est = self.backlash_est = 0.0
        self.e6_cw_ratio_est = self.e6_ccw_ratio_est = 0.0
        self.prev_target = self.prev_r7_cmd = 0.0

    def any_comp_on(self) -> bool:
        return any([self.r1_on, self.r2_on, self.r3_on, self.r4_on, self.r5_on, self.r6_on, self.r7_on])

    def system_identification(self, targets_cw, actuals_cw, targets_ccw, actuals_ccw):
        N = len(targets_cw)
        actuals_ccw_aligned = actuals_ccw[::-1]
        self.backlash_est = np.mean(actuals_cw - actuals_ccw_aligned)
        actuals_mid = (actuals_cw + actuals_ccw_aligned) / 2.0
        coeffs = np.polyfit(targets_cw, actuals_mid, 1)
        self.m_est, self.k_est = float(coeffs[0]), float(coeffs[1])
        res = actuals_mid - (self.m_est * targets_cw + self.k_est)
        self.ecc_A_est = np.sum(res * np.sin(2.0 * np.pi * targets_cw / ECC_PERIOD)) * (2.0 / N)
        self.ecc_B_est = np.sum(res * np.cos(2.0 * np.pi * targets_cw / ECC_PERIOD)) * (2.0 / N)
        res_e4 = res - (self.ecc_A_est * np.sin(2.0 * np.pi * targets_cw / ECC_PERIOD) + self.ecc_B_est * np.cos(
            2.0 * np.pi * targets_cw / ECC_PERIOD))
        self.mesh_A_est = np.sum(res_e4 * np.sin(2.0 * np.pi * targets_cw / MESH_PERIOD)) * (2.0 / N)
        self.mesh_B_est = np.sum(res_e4 * np.cos(2.0 * np.pi * targets_cw / MESH_PERIOD)) * (2.0 / N)

    def pre_compensate(self, target: float) -> float:
        delta_target = target - self.prev_target
        direction = 1 if delta_target >= 0 else -1
        delta_cmd = delta_target

        if self.r7_on:
            delta_cmd = delta_target / (1.0 + (self.e6_cw_ratio_est if delta_target >= 0 else self.e6_ccw_ratio_est))
            base_cmd = self.prev_r7_cmd + delta_cmd
        elif self.r6_on:
            delta_cmd = delta_target - (
                        delta_target * (self.e6_cw_ratio_est if delta_target >= 0 else self.e6_ccw_ratio_est))
            base_cmd = self.prev_r7_cmd + delta_cmd
        else:
            base_cmd = target

        cmd = base_cmd
        if self.r5_on: cmd -= (self.backlash_est / 2.0) * direction
        if self.r4_on: cmd -= (self.mesh_A_est * np.sin(2.0 * np.pi * target / MESH_PERIOD) + self.mesh_B_est * np.cos(
            2.0 * np.pi * target / MESH_PERIOD))
        if self.r3_on: cmd -= (self.ecc_A_est * np.sin(2.0 * np.pi * target / ECC_PERIOD) + self.ecc_B_est * np.cos(
            2.0 * np.pi * target / ECC_PERIOD))
        if self.r1_on: cmd -= self.k_est
        if self.r2_on: cmd /= self.m_est

        self.prev_target, self.prev_r7_cmd = target, base_cmd
        return cmd


# =========================================================
# 4. SERIAL CLIENTS (HANDLE & SLEW)
# =========================================================
class HandleSerialReader:
    def __init__(self, port, baud):
        self.ser = None
        self.port, self.baud = port, baud
        self._latest_v = 0
        self._stop_evt = threading.Event()

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            threading.Thread(target=self._rx_loop, daemon=True).start()
            return True
        except:
            return False

    def _rx_loop(self):
        while not self._stop_evt.is_set():
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("S"):
                try:
                    self._latest_v = int(line[1:])
                except:
                    pass

    def get_latest_inc(self):
        deg = (max(-1000, min(1000, self._latest_v)) / 1000.0) * MAX_HANDLE_DEG
        return deg * INC_PER_DEG, deg


class SlewSerialClient:
    def __init__(self, port, baud):
        self.ser = None
        self.port, self.baud = port, baud
        self.last_real_enc_inc = 0.0
        self._stop_evt = threading.Event()

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            threading.Thread(target=self._rx_loop, daemon=True).start()
            return True
        except:
            return False

    def _rx_loop(self):
        while not self._stop_evt.is_set():
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if "," in line:
                try:
                    _, act_inc = map(float, line.split(','))
                    self.last_real_enc_inc = act_inc
                except:
                    pass

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd.strip() + "\n").encode())
            self.ser.flush()


# =========================================================
# 5. MAIN CONTROLLER SYSTEM
# =========================================================
class SteeringBridgeSystem:
    def __init__(self):
        self.handle = HandleSerialReader(COM_HANDLE, BAUD_HANDLE)
        self.slew = SlewSerialClient(COM_SLEW, BAUD_SLEW)
        self.inj = InternalErrorInjector()
        self.cal = CalibrationFilter()
        self.running = False
        self.last_send_time = 0.0
        self.last_sent_abs = None

    def open(self):
        h_ok = self.handle.open()
        s_ok = self.slew.open()
        return h_ok and s_ok

    def start_rt(self):
        if not self.running:
            self.running = True
            threading.Thread(target=self._control_loop, daemon=True).start()

    def stop_rt(self):
        self.running = False

    def _control_loop(self):
        print(
            f"\n{'Target(Deg)':>10} | {'RawPred':>10} | {'CompPred':>10} | {'RealEnc(Deg)':>12} | {'RealErr':>8} | {'Mode'}")
        print("-" * 90)
        while self.running:
            target_inc, target_deg = self.handle.get_latest_inc()

            # 예측/보정 계산
            raw_pred_inc = self.inj.apply(target_inc, update_state=False)
            comp_cmd_inc = self.cal.pre_compensate(target_inc)
            comp_pred_inc = self.inj.apply(comp_cmd_inc, update_state=False)

            # 전송값 결정
            if MOVE_TO_PREDICTED:
                send_inc = int(round(comp_pred_inc if self.cal.any_comp_on() else raw_pred_inc))
                mode_str = "PRED"
            else:
                send_inc = int(round(comp_cmd_inc if self.cal.any_comp_on() else target_inc))
                mode_str = "CMD"

            # 실시간 전송 (간격 및 변화량 필터)
            now = time.perf_counter()
            if self.last_sent_abs is None or abs(send_inc - self.last_sent_abs) > 5 or (
                    now - self.last_send_time) > 0.2:
                if (now - self.last_send_time) >= SEND_INTERVAL_SEC:
                    self.slew.send(f"abs {send_inc}")
                    self.last_sent_abs = send_inc
                    self.last_send_time = now
                    _ = self.inj.apply(float(send_inc), update_state=True)

            # 실측 데이터 비교
            real_deg = (self.slew.last_real_enc_inc / ENC_RES) * 360.0
            real_err = target_deg - real_deg

            print(
                f"{target_deg:10.2f} | {raw_pred_inc / INC_PER_DEG:10.2f} | {comp_pred_inc / INC_PER_DEG:10.2f} | {real_deg:12.2f} | {real_err:8.3f} | {mode_str:>6}",
                end='\r')
            time.sleep(0.02)

    def calibrate(self):
        print("\n[System ID] 오차 모델 기반 학습 시작...")
        t = np.linspace(-MAX_HANDLE_DEG * INC_PER_DEG, MAX_HANDLE_DEG * INC_PER_DEG, 5000)
        a_cw = np.array([self.inj.apply(x, False) for x in t])
        a_ccw = np.array([self.inj.apply(x, False) for x in t[::-1]])
        self.cal.system_identification(t, a_cw, t[::-1], a_ccw)
        print("[완료] 학습 결과: a={:.4f}, b={:.1f}, Backlash={:.1f}".format(self.cal.m_est, self.cal.k_est,
                                                                       self.cal.backlash_est))


# =========================================================
# 6. CLI INTERFACE
# =========================================================
def main():
    sysm = SteeringBridgeSystem()
    if not sysm.open():
        print("포트 연결 실패! 설정을 확인하세요.");
        return

    print("\n======= 통합 정밀 제어 시스템 (Full Version) =======")
    print(" en : 기동 | rt : 실시간 추종 ON/OFF | cal : 학습")
    print(" e1~e6 : 오차 주입 | r1~r7 : 보정 활성화 | q : 종료")

    try:
        while True:
            cmd = input("\n>> ").strip().lower()
            if cmd == 'q':
                break
            elif cmd == 'en':
                sysm.slew.send("en"); print("Motor Enabled")
            elif cmd == 'rt':
                sysm.start_rt() if not sysm.running else sysm.stop_rt(); print(f"RT: {sysm.running}")
            elif cmd == 'cal':
                sysm.calibrate()

            # 오차/보정 토글 (E1~E6, R1~R7)
            elif cmd[0] == 'e' and cmd[1:].isdigit():
                idx = int(cmd[1:])
                if idx == 1:
                    sysm.inj.toggle_e1()
                elif idx == 3:
                    sysm.inj.toggle_e3()
                elif idx == 4:
                    sysm.inj.toggle_e4()
                elif idx == 5:
                    sysm.inj.toggle_e5()
                elif idx == 6:
                    sysm.inj.toggle_e6()
                print(f"Toggle {cmd.upper()}")
            elif cmd[0] == 'r' and cmd[1:].isdigit():
                attr = f"r{cmd[1:]}_on"
                if hasattr(sysm.cal, attr):
                    setattr(sysm.cal, attr, not getattr(sysm.cal, attr))
                    print(f"Toggle {cmd.upper()}")
            else:
                sysm.slew.send(cmd)
    finally:
        sysm.stop_rt()
        if sysm.slew.ser: sysm.slew.ser.close()
        if sysm.handle.ser: sysm.handle.ser.close()
        print("시스템 종료.")


if __name__ == "__main__":
    main()