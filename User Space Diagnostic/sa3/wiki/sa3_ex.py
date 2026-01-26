import isotp
import struct
import time

# p = 인트형을 바이트형 으로 변환, u = 바이트형을 인트형으로 변환
p8, u8 = lambda x: struct.pack('>B', x), lambda x: struct.unpack('>B', x)[0]
p16, u16 = lambda x: struct.pack('>H', x), lambda x: struct.unpack('>H', x)[0]
p32, u32 = lambda x: struct.pack('>I', x), lambda x: struct.unpack('>I', x)[0]
p64, u64 = lambda x: struct.pack('>Q', x), lambda x: struct.unpack('>Q', x)[0]

#REQ, RES ID 정의
rxid = 0x7e8
txid = 0x7df

# 소켓(버스) 연결
s = isotp.socket()
s.bind("vcan0", isotp.Address(rxid=rxid, txid=txid))

# SA 3 seed 요청 & seed 받기
q1 = bytes([0x27, 0x03])
s.send(q1)
a1 = s.recv()
if a1 and a1[0] == 0x67:
	seed = a1[2:]
	print("got seed")
	print(seed.hex(' ').upper)
else:
	print("seed not recved")
	exit()

# seed 기반 key 계산

# key = [0,0,0,0]
# key[1] = ((seed[1] + (seed[2] ^ seed[1]) ^ 0xed) + seed[2] * -0x10) & 0xFF
# key[0] = ((seed[0] + (seed[3] ^ seed[0]) ^ 0xfe) + seed[3] * -0x10) & 0xFF
# key[2] = ((seed[2] + (seed[1] ^ seed[3]) ^ 0xfa) + seed[1] * -0x10) & 0xFF
# key[3] = ((seed[3] + (seed[0] ^ seed[2]) ^ 0xce) - ((seed[0]) << 4) & 0xFF) & 0xFF
# seed 기반 key 계산
key = [0, 0, 0, 0]
# local_24[1] -> key[1]
key[1] = ((seed[1] + (seed[2] ^ seed[1]) ^ 0xed) + (seed[2] * -0x10)) & 0xFF
key[0] = ((seed[0] + (seed[3] ^ seed[0]) ^ 0xfe) + (seed[3] * -0x10)) & 0xFF
key[2] = ((seed[2] + (seed[1] ^ seed[3]) ^ 0xfa) + (seed[1] * -0x10)) & 0xFF
# 뺄셈 연산까지 모두 마친 후 마지막에 & 0xFF를 적용합니다.
key[3] = ((seed[3] + (seed[0] ^ seed[2]) ^ 0xce) - (seed[0] << 4)) & 0xFF

print("got key successfully")
print("calculated key is = ")
key_str = ["","","",""]
key_byte = b''
for i in range(4):
	tmp_byte = key[i].to_bytes(1, byteorder='big')
	key_byte += tmp_byte
	key_str[i] = tmp_byte.hex().upper()
for str in key_str:
	print(str, end=' ')

# SA 3 send key
print("\n")
print("sent key")
q2 = bytes([0x27, 0x04]) + key_byte
s.send(q2)


a2 = s.recv() # a2 = key send 에 대한 ecu 응답

if a2[0] == 0x67:
	print("SA 3 Unlocked !!")
	data_byte = a2[2:]
	print(data_byte.hex(' ').upper())
else:
	print("Invalid key")


