
import sys
import subprocess
import threading
import can
import time


# --- [CONFIGURATIONS] ---
# 버스 환경 설정 전역 변수
CAN_INTERFACE = 'socketcan'
CAN_CHANNEL = 'vcan0'  # 실제 장치는 'can0', 테스트는 'vcan0'
CAN_BITRATE = 500000

# 메시지 설정
REQ_ID = 0x7DF    # 송신 ID (예: OBD-II Functional Request)
RES_ID = 0x7E8    # 수신 대기할 특정 ID (예: ECU Response)

# --- [MODULES] ---

def setup_can_bus():
    """
    1. CAN 버스 환경 설정 함수
    전역 변수를 참조하여 버스 객체를 생성하고 반환합니다.
    """
    try:
        bus = can.interface.Bus(
            interface=CAN_INTERFACE,
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE
        )
        print(f"[INFO] Bus initialized on {CAN_CHANNEL} ({CAN_INTERFACE})")
        return bus
    except Exception as e:
        print(f"[ERROR] Failed to setup bus: {e}")
        return None

def send_can_message(bus, msg_id, payload_str):
    """
    2. CAN 메시지 송신 함수
    payload_str: "02 10 02 00 00 00 00 00" 형태의 문자열
    """
    try:
        # 공백 제거 후 16진수 바이트 리스트로 변환
        data_bytes = [int(byte, 16) for byte in payload_str.split()]
        
        msg = can.Message(
            arbitration_id=msg_id,
            data=data_bytes,
            is_extended_id=False
        )
        
        bus.send(msg)
        #print(f"[TX] ID: {hex(msg_id)} | Data: {payload_str}")
    except ValueError as e:
        print(f"[ERROR] Invalid payload format: {e}")
    except can.CanError as e:
        print(f"[ERROR] Message not sent: {e}")

def receive_response_return_byte(bus, target_id, timeout_ms):
    """
    ISO-TP 지원 및 기존 인덱스 호환성 유지 버전 (Bytes 반환)
    """
    timeout_sec = timeout_ms / 1000.0
    start_time = time.time()
    full_data = bytearray()
    expected_len = 0

    while (time.time() - start_time) < timeout_sec:
        remaining_time = timeout_sec - (time.time() - start_time)
        msg = bus.recv(timeout=max(0, remaining_time))
        
        if msg is not None and msg.arbitration_id == target_id:
            pci = msg.data[0] >> 4

            # 1. Single Frame: 기존처럼 8바이트 전체 반환 (인덱스 유지)
            if pci == 0x00:
                return bytes(msg.data)

            # 2. First Frame
            elif pci == 0x01:
                expected_len = ((msg.data[0] & 0x0F) << 8) | msg.data[1]
                full_data.extend(msg.data[2:])
                bus.send(can.Message(arbitration_id=REQ_ID, data=[0x30, 0x00, 0x00, 0, 0, 0, 0, 0], is_extended_id=False))
                continue

            # 3. Consecutive Frame
            elif pci == 0x02:
                full_data.extend(msg.data[1:])
                if len(full_data) >= expected_len:
                    # 인덱스 [3:] 규칙을 위해 앞에 3바이트 더미(0x00) 추가
                    return bytes([0x00, 0x00, 0x00]) + bytes(full_data[:expected_len])
                    
    return None

def receive_response_return_hex(bus, target_id, timeout_ms):
    timeout_sec = timeout_ms / 1000.0
    start_time = time.time()
    
    full_data = bytearray()
    expected_len = 0

    while (time.time() - start_time) < timeout_sec:
        remaining_time = timeout_sec - (time.time() - start_time)
        msg = bus.recv(timeout=max(0, remaining_time))
        
        if msg is not None and msg.arbitration_id == target_id:
            pci = msg.data[0] >> 4

            # 1. Single Frame (기존 인덱스 체계를 유지하기 위해 전체 8바이트 반환)
            if pci == 0x00:
                return ' '.join([f"{b:02X}" for b in msg.data])

            # 2. First Frame (긴 데이터의 시작)
            elif pci == 0x01:
                expected_len = ((msg.data[0] & 0x0F) << 8) | msg.data[1]
                # FF 응답도 전체 형식을 맞추기 위해 앞에 00 등을 채우거나 
                # 유저님의 체크 로직을 고려하여 데이터만 취합합니다.
                full_data.extend(msg.data[2:])
                
                fc_msg = can.Message(arbitration_id=REQ_ID, data=[0x30, 0x00, 0x00, 0, 0, 0, 0, 0], is_extended_id=False)
                bus.send(fc_msg)
                continue

            # 3. Consecutive Frame
            elif pci == 0x02:
                full_data.extend(msg.data[1:])
                if len(full_data) >= expected_len:
                    # 긴 응답도 기존 체크 로직(인덱스 3번부터 데이터)을 위해 앞에 더미 바이트 추가
                    # "00 00 00 63 ..." 처럼 구성하여 3번 인덱스가 63(긍정응답)이 되게 함
                    final_hex = '00 00 00 ' + ' '.join([f"{b:02X}" for b in full_data[:expected_len]])
                    return final_hex

    return None

def receive_and_print_response(bus, target_id, timeout_ms):
    """
    ISO-TP 지원 및 출력/반환 호환성 유지 버전
    """
    timeout_sec = timeout_ms / 1000.0
    start_time = time.time()
    full_data = bytearray()
    expected_len = 0
    
    while (time.time() - start_time) < timeout_sec:
        remaining_time = timeout_sec - (time.time() - start_time)
        msg = bus.recv(timeout=max(0, remaining_time))
        
        if msg is not None and msg.arbitration_id == target_id:
            pci = msg.data[0] >> 4

            if pci == 0x00:  # Single Frame
                length = msg.data[0] & 0x0F
                actual_data = msg.data[1:1+length]
                print(f"[RX] ID: {hex(target_id)} | Data: {' '.join([f'{b:02X}' for b in actual_data])}")
                # 반환은 기존 인덱스 체크를 위해 8바이트 전체 반환
                return bytes(msg.data)

            elif pci == 0x01:  # First Frame
                expected_len = ((msg.data[0] & 0x0F) << 8) | msg.data[1]
                full_data.extend(msg.data[2:])
                bus.send(can.Message(arbitration_id=REQ_ID, data=[0x30, 0x00, 0x00, 0, 0, 0, 0, 0], is_extended_id=False))
                continue

            elif pci == 0x02:  # Consecutive Frame
                full_data.extend(msg.data[1:])
                if len(full_data) >= expected_len:
                    final_payload = bytes(full_data[:expected_len])
                    print(f"[RX-LONG] ID: {hex(target_id)} | Total: {expected_len} bytes")
                    print(f"[DATA] {' '.join([f'{b:02X}' for b in final_payload])}")
                    # 반환은 인덱스 [3:] 규칙 보존을 위해 더미 포함
                    return bytes([0x00, 0x00, 0x00]) + final_payload
                    
    return None

def bytes_to_ascii(byte_data):
    """
    바이트 데이터를 아스키 문자열로 변환하여 리턴합니다.
    """
    try:
        # 'ascii' 코덱을 사용하여 디코딩
        return byte_data.decode('ascii')
    except UnicodeDecodeError:
        return "오류: 아스키 범위(0~127)를 벗어나는 데이터가 포함되어 있습니다."

# --- 활용 예시 ---
# received_data = receive_and_print_response(can_bus, 0x7E8, 500)
# if received_data:
#     print_as_ascii(received_data)

def invert_data(input):
    segment = input[3:5]
    inverted_list = [(~b & 0xFF) for b in segment]
    result_hex = ' '.join([f'{b:02X}' for b in inverted_list])
    return result_hex

class SessionKeeper:
    def __init__(self, bus):
        self.bus = bus
        self.stop_event = threading.Event()
        self.thread = None

    def _run(self):
        """내부적으로 2초마다 Tester Present 전송 (일반적으로 T3server 타임아웃 고려)"""
        print("[Thread] Tester Present 시작")
        while not self.stop_event.is_set():
            # 01 3E 80 (Suppress Response 사용 추천)
            msg = can.Message(arbitration_id=REQ_ID, data=[0x02, 0x3E, 0x80, 0, 0, 0, 0, 0], is_extended_id=False)
            try:
                self.bus.send(msg)
            except Exception as e:
                print(f"[Thread Error] {e}")
            
            # stop_event가 set될 때까지 2초 대기 (중간에 종료 신호 오면 즉시 깨어남)
            self.stop_event.wait(2.0) 
        print("[Thread] Tester Present 중단")

    def start(self):
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True  # 메인 스레드 종료 시 함께 종료
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join()


def hex_string_to_bytes(hex_str):
    """
    "00 62 01 02" 형태의 문자열을 b'\x00\x62\x01\x02' 바이트 타입으로 변환
    """
    try:
        # 공백 제거 후 16진수 바이트로 변환
        clean_hex = hex_str.replace(" ", "")
        return bytes.fromhex(clean_hex)
    except ValueError as e:
        print(f"[ERROR] Invalid hex string: {e}")
        return None
    
def bytes_to_hex_string(byte_data):
    """
    b'\x00\x62\x01\x02' 바이트 타입을 "00 62 01 02" 형태의 문자열로 변환
    """
    if not byte_data:
        return ""
    # 각 바이트를 2자리 대문자 16진수로 변환하고 공백으로 연결
    return " ".join([f"{b:02X}" for b in byte_data])



if __name__ == "__main__":
    bus = setup_can_bus()
    if not bus: sys.exit()

    print("[*] Scan Start: Searching for positive Routine ID")
    
    for i in range(0x0000, 0x10000):
        did_hi = (i >> 8) & 0xFF
        did_lo = i & 0xFF
        
        # 16진수 포맷팅 (반드시 2자리 유지)
        payload = f"04 31 01 {did_hi:02X} {did_lo:02X} 00 00 00"
        send_can_message(bus, REQ_ID, payload)
        
        # 응답 수신 (스캔 속도를 위해 타임아웃 0.05초 설정)
        response_bytes = receive_response_return_byte(bus, RES_ID, 100)
        
        if response_bytes:
            # response_bytes[1] 혹은 response_bytes[3] (ISO-TP 더미 포함 시) 확인
            # 작성하신 receive_response_return_byte 규칙에 따라 인덱스 설정
            # Single Frame은 전체 8바이트 반환, Long은 앞에 3바이트 더미 추가됨
            
            # 긍정 응답 0x62 찾기
            is_positive = False
            data_start_idx = 0
            
            if response_bytes[0] == 0x00 and response_bytes[3] == 0x71: # Long Message (더미 포함)
                is_positive = True
                data_start_idx = 6 # 00 00 00 62 [DID] [DATA...]
            elif response_bytes[1] == 0x71: # Single Frame
                is_positive = True
                data_start_idx = 4 # [PCI] 62 [DID] [DATA...]

            if is_positive:
                actual_data = response_bytes[data_start_idx:]
                ascii_val = "".join([chr(b) if 32 <= b <= 126 else "." for b in actual_data])
                
                print(f"\n[!] Found Valid Routine ID: 0x{did_hi:02X}{did_lo:02X}")
                print(f"Routuin 0x{did_hi:02X}{did_lo:02X} is started")
                user_decision_01 = input("Do you want to request this Routine 0x{did_hi:02X}{did_lo:02X}?, Press \"N\" to exit the program, otherwise press any key")

                if user_decision_01 == "N":
                    bus.shutdown() # 안전하게 종료
                    exit()
                else:
                    print("Requesting Routine...")
                    send_can_message(bus, REQ_ID, f"04 31 03 {did_hi:02X} {did_lo:02X} 00 00 00")

                    routine_exe_ret_byte = receive_and_print_response(bus, RES_ID, 1000)
                    ret_ascii = bytes_to_ascii(routine_exe_ret_byte)
                    print(ret_ascii)
                    

                    user_decision_02 = input("to quit press E, to run next press any key")
                    if user_decision_02 == "E":
                        bus.shutdown() # 안전하게 종료
                        exit()
                    else:
                        continue

                

                # if "CTF" in ascii_val or "flag" in ascii_val:
                #     print(">>> FLAG FOUND! <<<")
                #     break # 플래그 찾으면 중단