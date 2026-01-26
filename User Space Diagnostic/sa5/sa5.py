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
txid = 0x7e0



# 소켓 연결
s = isotp.socket()
s.bind("vcan0", isotp.Address(rxid=rxid, txid=txid))


#########################################################################################

