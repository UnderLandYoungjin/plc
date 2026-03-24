import serial
import struct

# ===== 시리얼 설정 =====
PORT     = "COM5"
BAUDRATE = 9600
SLAVE_ID = 1

# ===== LS G100 확정 레지스터 맵 =====
REG_FREQ = 0x0004  # 주파수 설정 (0.01Hz 단위 / 3000=30Hz, 6000=60Hz)
REG_CMD  = 0x0005  # 운전 명령 레지스터

# 운전 명령 비트값 (매뉴얼 확인)
CMD_STOP = 0x0001  # bit0 = 정지
CMD_FWD  = 0x0002  # bit1 = 정방향 RUN
CMD_REV  = 0x0004  # bit2 = 역방향 RUN

# ===== Modbus RTU CRC16 =====
def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)

# ===== Modbus FC06 단일 레지스터 쓰기 =====
def write_register(ser, reg_addr, value, label):
    msg = struct.pack('>BBHH', SLAVE_ID, 0x06, reg_addr, value)
    msg += crc16(msg)
    ser.write(msg)
    print(f"  TX: {' '.join(f'{b:02X}' for b in msg)}")
    resp = ser.read(8)
    if resp:
        ok = "✓" if resp == msg else "✗ 불일치"
        print(f"  RX: {' '.join(f'{b:02X}' for b in resp)}  {ok}")
        return resp == msg
    else:
        print("  응답 없음")
        return False

# ===== 제어 함수 =====
def set_freq(ser, hz: float):
    """주파수 설정 (Hz → 0.01Hz 단위 변환)"""
    print(f"\n[주파수 설정] {hz}Hz")
    return write_register(ser, REG_FREQ, int(hz * 100), "FREQ")

def run_fwd(ser):
    """정방향 운전"""
    print("\n[정방향 RUN]")
    return write_register(ser, REG_CMD, CMD_FWD, "FWD")

def run_rev(ser):
    """역방향 운전"""
    print("\n[역방향 RUN]")
    return write_register(ser, REG_CMD, CMD_REV, "REV")

def stop(ser):
    """정지"""
    print("\n[STOP]")
    return write_register(ser, REG_CMD, CMD_STOP, "STOP")

# ===== 메뉴 =====
def print_menu():
    print("\n" + "=" * 35)
    print("  LS G100 인버터 제어")
    print("=" * 35)
    print("  1. 정방향 RUN")
    print("  2. 역방향 RUN")
    print("  3. STOP")
    print("  4. 주파수 설정")
    print("  0. 종료")
    print("=" * 35)

# ===== 메인 =====
def main():
    ser = serial.Serial(PORT, BAUDRATE,
                        bytesize=8,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1)
    ser.reset_input_buffer()
    print(f"[연결 성공] {PORT} / {BAUDRATE}bps / Slave ID={SLAVE_ID}")

    try:
        while True:
            print_menu()
            cmd = input("  선택 → ").strip()

            if cmd == "1":
                run_fwd(ser)
            elif cmd == "2":
                run_rev(ser)
            elif cmd == "3":
                stop(ser)
            elif cmd == "4":
                try:
                    hz = float(input("  주파수 입력 (Hz, 예: 30.0): ").strip())
                    if 0.0 <= hz <= 60.0:
                        set_freq(ser, hz)
                    else:
                        print("  ⚠ 0 ~ 60Hz 범위로 입력하세요.")
                except ValueError:
                    print("  ⚠ 숫자를 입력하세요.")
            elif cmd == "0":
                print("\n종료 전 STOP 전송...")
                stop(ser)
                break
            else:
                print("  ⚠ 0~4 중 선택하세요.")

    except KeyboardInterrupt:
        print("\n중단 — STOP 전송")
        stop(ser)
    finally:
        ser.close()
        print("포트 닫힘.")

if __name__ == "__main__":
    main()
