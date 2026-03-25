"""
LS G100 인버터 Modbus RTU 제어 프로그램
─────────────────────────────────────────
· 통신 방식 : Modbus RTU (RS-485)
· 사용 라이브러리 : pyserial, struct
· 기능 : 정방향/역방향 운전, 정지, 주파수 설정
"""

import serial   # RS-485 시리얼 통신용 라이브러리
import struct   # 바이트 데이터 패킹/언패킹 (Modbus 프레임 생성에 사용)

# ──────────────────────────────────────────
# 1. 시리얼 포트 및 통신 설정
# ──────────────────────────────────────────
PORT     = "COM10"      # PC에 연결된 RS-485 컨버터 포트 번호
BAUDRATE = 9600         # 인버터와 동일하게 맞춰야 함 (인버터 파라미터에서 설정)
SLAVE_ID = 1            # Modbus 슬레이브 주소 (인버터 기본값 = 1)

# ──────────────────────────────────────────
# 2. LS G100 인버터 레지스터 맵
#    - Modbus에서 레지스터 = 인버터 내부 메모리 주소
#    - 이 주소에 값을 쓰면 인버터가 해당 동작 수행
# ──────────────────────────────────────────
REG_FREQ = 0x0004       # 주파수 설정 레지스터 (0.01Hz 단위, 예: 3000 → 30.00Hz)
REG_CMD  = 0x0005       # 운전 명령 레지스터 (정방향/역방향/정지 제어)

# 운전 명령 비트값 — 레지스터에 이 값을 쓰면 해당 동작 실행
CMD_STOP = 0x0001       # bit0 = 정지
CMD_FWD  = 0x0002       # bit1 = 정방향 운전 (모터 CW 회전)
CMD_REV  = 0x0004       # bit2 = 역방향 운전 (모터 CCW 회전)


# ──────────────────────────────────────────
# 3. Modbus RTU CRC-16 오류 검출 함수
#    - 모든 Modbus RTU 프레임 끝에 2바이트 CRC가 붙어야 함
#    - CRC가 없거나 틀리면 인버터가 프레임을 무시함
#    - 다항식: 0xA001 (Modbus 표준)
# ──────────────────────────────────────────
def crc16(data: bytes) -> bytes:
    crc = 0xFFFF                    # CRC 초기값
    for byte in data:
        crc ^= byte                 # 각 바이트와 XOR
        for _ in range(8):          # 8비트 순회
            if crc & 1:             # 최하위 비트가 1이면
                crc = (crc >> 1) ^ 0xA001   # 우측 시프트 후 다항식 XOR
            else:
                crc = crc >> 1      # 최하위 비트가 0이면 우측 시프트만
    return struct.pack('<H', crc)   # 리틀엔디안 2바이트로 반환


# ──────────────────────────────────────────
# 4. Modbus FC06 — 단일 레지스터 쓰기 함수
#    - Function Code 06 = 레지스터 1개에 값 쓰기
#    - 프레임 구조: [슬레이브ID][FC][레지스터주소][값][CRC]
#      · 총 8바이트 (데이터 6 + CRC 2)
#    - 정상 응답: 인버터가 동일한 프레임을 그대로 되돌려 보냄 (Echo)
# ──────────────────────────────────────────
def write_register(ser, reg_addr, value):
    # '>BBHH' = 빅엔디안, 1바이트(ID) + 1바이트(FC) + 2바이트(주소) + 2바이트(값)
    msg = struct.pack('>BBHH', SLAVE_ID, 0x06, reg_addr, value)
    msg += crc16(msg)               # CRC 2바이트 추가 → 총 8바이트
    ser.write(msg)                   # 시리얼로 전송
    print(f"  TX: {' '.join(f'{b:02X}' for b in msg)}")

    resp = ser.read(8)              # 응답 8바이트 수신 대기
    if resp:
        ok = "✓ 정상" if resp == msg else "✗ 불일치"
        print(f"  RX: {' '.join(f'{b:02X}' for b in resp)}  {ok}")
        return resp == msg
    else:
        print("  응답 없음 (타임아웃)")
        return False


# ──────────────────────────────────────────
# 5. 인버터 제어 함수
# ──────────────────────────────────────────
def set_freq(ser, hz: float):
    """주파수 설정 — Hz 값을 0.01Hz 단위 정수로 변환하여 전송"""
    print(f"\n[주파수 설정] {hz}Hz → 레지스터 값: {int(hz * 100)}")
    return write_register(ser, REG_FREQ, int(hz * 100))

def run_fwd(ser):
    """정방향(CW) 운전 시작"""
    print("\n[정방향 RUN]")
    return write_register(ser, REG_CMD, CMD_FWD)

def run_rev(ser):
    """역방향(CCW) 운전 시작"""
    print("\n[역방향 RUN]")
    return write_register(ser, REG_CMD, CMD_REV)

def stop(ser):
    """모터 정지"""
    print("\n[STOP]")
    return write_register(ser, REG_CMD, CMD_STOP)


# ──────────────────────────────────────────
# 6. 콘솔 메뉴 UI
# ──────────────────────────────────────────
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


# ──────────────────────────────────────────
# 7. 메인 실행부
# ──────────────────────────────────────────
def main():
    # 시리얼 포트 열기 (Modbus RTU 표준: 8N1)
    ser = serial.Serial(
        port     = PORT,
        baudrate = BAUDRATE,
        bytesize = 8,                      # 데이터 비트 8
        parity   = serial.PARITY_NONE,     # 패리티 없음
        stopbits = serial.STOPBITS_ONE,    # 정지 비트 1
        timeout  = 1                        # 응답 대기 1초
    )
    ser.reset_input_buffer()    # 수신 버퍼 비우기 (잔여 데이터 제거)
    print(f"[연결 성공] {PORT} / {BAUDRATE}bps / Slave ID={SLAVE_ID}")

    try:
        while True:
            print_menu()
            cmd = input("  선택 → ").strip()

            if   cmd == "1":  run_fwd(ser)
            elif cmd == "2":  run_rev(ser)
            elif cmd == "3":  stop(ser)
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
        # Ctrl+C 강제 종료 시에도 안전하게 모터 정지
        print("\n중단 — STOP 전송")
        stop(ser)
    finally:
        ser.close()
        print("포트 닫힘.")


if __name__ == "__main__":
    main()
