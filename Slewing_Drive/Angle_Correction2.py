import time
import threading
import numpy as np
import serial
import random

# =========================================================
# 1. 하드웨어 설정
# =========================================================
COM_SLEW = "COM3"
BAUD_SLEW = 115200

INC_PER_DEG = (182.0 * 70.0 * 150.0) / 62.0
ENC_RES = 4096.0
ENC_DIR = -1.0


# =========================================================
# 2. 오차 주입 및 보정 필터 (원본 로직 유지)
# =========================================================
class InternalErrorInjector:
    def __init__(self):
        self.e1_on = self.e3_on = self.e5_on = False
        self._a_err = 1.2;
        self._ecc_amp = 500.0;
        self._backlash = 300.0
        self.prev_cmd = 0.0

    def toggle(self, t):
        setattr(self, f"{t}_on", not getattr(self, f"{t}_on")); return getattr(self, f"{t}_on")

    def apply(self, v, update=True):
        if self.e1_on: v = v * 1.2
        if update: self.prev_cmd = v
        return v


class CalibrationFilter:
    def __init__(self):
        self.r1_on = self.r5_on = False
        self.k_est, self.m_est = 0.0, 1.0

    def any_comp_on(self): return self.r1_on or self.r5_on

    def pre_compensate(self, t):
        if self.r1_on: t = (t - self.k_est) / self.m_est
        return t


# =========================================================
# 3. 메인 컨트롤러 클래스
# =========================================================
class SlewingController:
    def __init__(self, port):
        try:
            self.ser = serial.Serial(port, BAUD_SLEW, timeout=0.05)
        except Exception as e:
            print(f"[ERROR] 포트 연결 실패: {e}");
            exit()

        self.enc_correction = 35.0 / 360.0  # 초기 기준 계수
        self.last_pulse = 0.0
        self.offset_pulse = 0.0
        self.input_unit = "deg"
        self.running = True
        self.inj = InternalErrorInjector()
        self.cal = CalibrationFilter()

        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _rx_loop(self):
        while self.running:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "," in line:
                    try:
                        _, r = map(float, line.split(',')); self.last_pulse = r
                    except:
                        pass

    def get_actual_deg(self):
        """계수 적용 실측 각도 계산 (무한 누적)"""
        rel_pulse = (self.last_pulse - self.offset_pulse) * ENC_DIR
        return (rel_pulse / ENC_RES) * 360.0 * self.enc_correction

    def wait_for_motion_complete(self, timeout=15.0):
        """[핵심] 모터가 완전히 멈출 때까지 스마트 대기"""
        print(" -> 이동 중", end="", flush=True)
        time.sleep(0.5)  # 초기 가속 시간 부여

        start_time = time.time()
        prev_pulse = self.last_pulse

        while (time.time() - start_time) < timeout:
            time.sleep(0.2)  # 0.2초 단위 샘플링
            current_pulse = self.last_pulse

            # 펄스 변화량이 5 미만이면 정지(Settling) 완료로 간주
            if abs(current_pulse - prev_pulse) < 5:
                time.sleep(0.3)  # 기계적 잔진동 흡수 대기
                print(" [도달 완료]")
                return True

            prev_pulse = current_pulse
            print(".", end="", flush=True)

        print(" [타임아웃: 모터 정지 확인 불가]")
        return False

    def safe_zero_set(self):
        """2초 간격 이중 Zero-Set"""
        print("[ZERO] 영점 고정 중...")
        self.offset_pulse = self.last_pulse
        time.sleep(2.0)
        self.offset_pulse = self.last_pulse
        print(f"[ZERO] 완료 (기준 펄스: {self.offset_pulse})")

    def scan_and_calibrate(self, interval=5.0, limit=60.0):
        """스캔 및 실시간 최적 계수 산출"""
        print(f"\n[SCAN & CAL] {interval}° 단위 데이터 수집 시작 (0~{limit}°)")
        self.ser.write(f"abs 0\n".encode())
        self.wait_for_motion_complete()
        self.safe_zero_set()

        tp_list, raw_list = [], []

        print("\n[1단계] 구간별 순수 엔코더 데이터 수집 중...")
        for tp in np.arange(interval, limit + interval, interval):
            self.ser.write(f"abs {int(tp * INC_PER_DEG)}\n".encode())

            # 멍청한 time.sleep 대신 스마트 대기 로직 적용
            self.wait_for_motion_complete()

            rel_pulse = (self.last_pulse - self.offset_pulse) * ENC_DIR
            raw_deg = (rel_pulse / ENC_RES) * 360.0

            tp_list.append(tp)
            raw_list.append(raw_deg)
            print(f"    * Target: {tp:5.1f}° 캡처 완료")

        ratios = [t / r for t, r in zip(tp_list, raw_list) if r != 0]
        if not ratios: return

        self.enc_correction = np.mean(ratios)
        print(f"\n[2단계] 최적 보정 계수(K) 산출 및 적용: {self.enc_correction:.6f}")

        print(f"\n{'Target':>10} | {'Actual(보정후)':>14} | {'Error':>10} | {'Rate':>8}")
        print("-" * 55)
        for tp, raw in zip(tp_list, raw_list):
            actual = raw * self.enc_correction
            error = tp - actual
            rate = (abs(error) / tp * 100) if tp != 0 else 0
            print(f"{tp:10.1f}° | {actual:14.3f}° | {error:10.3f}° | {rate:7.2f}%")

        print("-" * 55)
        print("[완료] 0도로 복귀합니다.")
        self.ser.write(f"abs 0\n".encode())


# =========================================================
# 4. CLI 인터페이스
# =========================================================
def main():
    ctrl = SlewingController(COM_SLEW)
    print("\n" + "=" * 55)
    print(" [정밀 제어 터미널 - 동적 도달 감지 시스템]")
    print("  en    : 드라이버 기동")
    print("  z     : 이중 영점 설정 (0° 세팅)")
    print("  scan  : 스캔 + 최적 계수 자동 보정")
    print("  abs <v>: 절대 각도 이동")
    print("=" * 55)

    try:
        while True:
            cmd = input(f"\n(deg)> ").strip().lower()
            if cmd == 'q':
                break
            elif cmd == 'en':
                for c in ["134", "6", "7", "15", "4159"]:
                    ctrl.ser.write(f"{c}\n".encode());
                    time.sleep(0.15)
                print("[OK] Online.")
            elif cmd == 'z':
                ctrl.safe_zero_set()
            elif cmd == 'scan':
                ctrl.scan_and_calibrate(interval=5.0, limit=90.0)  # 90도까지 스캔 확장
            elif cmd.startswith("abs "):
                val = float(cmd.split()[1])
                target_inc = val * INC_PER_DEG
                comp_inc = ctrl.cal.pre_compensate(target_inc)

                ctrl.ser.write(f"abs {int(round(comp_inc))}\n".encode())
                _ = ctrl.inj.apply(comp_inc, True)

                # [핵심] 이동이 끝날 때까지 동적으로 기다림
                ctrl.wait_for_motion_complete()

                actual = ctrl.get_actual_deg()
                error = val - actual
                print(f" -> Target: {val:.1f}° | Actual: {actual:.3f}° | Error: {error:.3f}°")
            elif (cmd[0] in ['e', 'r']) and len(cmd) > 1:
                if cmd[0] == 'e':
                    print(f"Toggle {cmd}: {ctrl.inj.toggle(cmd)}")
                else:
                    attr = f"{cmd}_on"
                    setattr(ctrl.cal, attr, not getattr(ctrl.cal, attr))
                    print(f"Toggle {cmd.upper()}: {getattr(ctrl.cal, attr)}")
            else:
                ctrl.ser.write(f"{cmd}\n".encode())
    finally:
        ctrl.running = False;
        ctrl.ser.close()


if __name__ == "__main__":
    main()