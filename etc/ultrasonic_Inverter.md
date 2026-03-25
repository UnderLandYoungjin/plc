##초음파 센서로 인버터 안전정지

https://www.youtube.com/watch?v=CIfZujDv2W8


```python
"""
LS G100 인버터 + 초음파 센서 연동 제어
──────────────────────────────────────
· Arduino : 초음파 거리값 시리얼 전송
· Python  : 거리값 수신 → 인버터 Modbus RTU 제어
· 동작    : 정방향 20Hz 기동, 거리 ≤15cm → 정지
            거리 >15cm 를 2초 이상 유지 → 재기동
"""

import serial
import struct
import time

# ── 포트 설정 ──
ARDUINO_PORT = "COM3"       # 아두이노 시리얼 포트
INVERTER_PORT = "COM11"     # RS-485 인버터 포트
BAUDRATE = 9600
SLAVE_ID = 1

# ── 인버터 레지스터 / 명령 ──
REG_FREQ = 0x0004
REG_CMD  = 0x0005
CMD_STOP = 0x0001
CMD_FWD  = 0x0002

# ── 제어 파라미터 ──
TARGET_FREQ = 10.0          # 운전 주파수 (Hz)
DIST_THRESHOLD = 25.0       # 정지 기준 거리 (cm)
RESTART_HOLD = 2.0          # 재기동 조건 유지 시간 (초)


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)


def write_register(ser, reg_addr, value):
    msg = struct.pack('>BBHH', SLAVE_ID, 0x06, reg_addr, value)
    msg += crc16(msg)
    ser.write(msg)
    resp = ser.read(8)
    ok = resp == msg if resp else False
    print(f"  {'✓' if ok else '✗'} REG=0x{reg_addr:04X} VAL={value}")
    return ok


def set_freq(ser, hz):
    return write_register(ser, REG_FREQ, int(hz * 100))


def run_fwd(ser):
    print("[▶ 정방향 RUN]")
    return write_register(ser, REG_CMD, CMD_FWD)


def stop(ser):
    print("[■ STOP]")
    return write_register(ser, REG_CMD, CMD_STOP)


def main():
    # 시리얼 포트 열기
    ard = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=1)
    inv = serial.Serial(
        port=INVERTER_PORT, baudrate=BAUDRATE,
        bytesize=8, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=1
    )
    inv.reset_input_buffer()
    ard.reset_input_buffer()
    time.sleep(2)  # 아두이노 리셋 대기

    print(f"아두이노: {ARDUINO_PORT} / 인버터: {INVERTER_PORT}")
    print(f"운전 주파수: {TARGET_FREQ}Hz / 정지 거리: ≤{DIST_THRESHOLD}cm\n")

    # 초기 기동: 주파수 설정 → 정방향 운전
    set_freq(inv, TARGET_FREQ)
    time.sleep(0.1)
    run_fwd(inv)

    motor_running = True
    clear_since = None  # 거리 >15cm 시작 시각

    try:
        while True:
            line = ard.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            try:
                dist = float(line)
            except ValueError:
                continue

            # 상태 표시
            status = "RUN" if motor_running else "STOP"
            print(f"거리: {dist:6.1f}cm  | 모터: {status}")

            # ── 정지 조건: 거리 ≤ 15cm & 모터 가동 중 ──
            if dist <= DIST_THRESHOLD and motor_running:
                stop(inv)
                motor_running = False
                clear_since = None

            # ── 재기동 조건: 거리 > 15cm 를 2초 이상 유지 ──
            elif dist > DIST_THRESHOLD and not motor_running:
                if clear_since is None:
                    clear_since = time.time()
                elif time.time() - clear_since >= RESTART_HOLD:
                    set_freq(inv, TARGET_FREQ)
                    time.sleep(0.1)
                    run_fwd(inv)
                    motor_running = True
                    clear_since = None

            # 거리 ≤ 15cm 이면 타이머 리셋 (정지 상태에서 다시 가까워진 경우)
            elif dist <= DIST_THRESHOLD and not motor_running:
                clear_since = None

    except KeyboardInterrupt:
        print("\n중단 — STOP 전송")
        stop(inv)
    finally:
        ard.close()
        inv.close()
        print("포트 닫힘.")


if __name__ == "__main__":
    main()
```


