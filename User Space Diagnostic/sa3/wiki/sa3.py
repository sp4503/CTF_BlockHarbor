# dump.py

import isotp
import struct
import time

p8, u8 = lambda x: struct.pack('>B', x), lambda x: struct.unpack('>B', x)[0]
p16, u16 = lambda x: struct.pack('>H', x), lambda x: struct.unpack('>H', x)[0]
p32, u32 = lambda x: struct.pack('>I', x), lambda x: struct.unpack('>I', x)[0]
p64, u64 = lambda x: struct.pack('>Q', x), lambda x: struct.unpack('>Q', x)[0]

rxid = 0x7e8
txid = 0x7e0

s = isotp.socket()
s.bind("vcan0", isotp.Address(rxid=rxid, txid=txid))

cmd = b'\x23\x24\x00'
size = b'\x01\x00'

mybin = b''

# 0x40 ~ 0x4f 40ff00
for i in range(0x40, 0x50):
	for j in range(0x100):
		# 23 24 / 00 40 00 00 / 01 00
		pl = cmd+p8(i)+p8(j)+b'\x00'+size
		print(f'processing... {pl.hex()}', end='\r')
		s.send(pl)
		result = s.recv()
		if result == b'\x7f\x23\x31' : break
		mybin += result
		time.sleep(0.05)
		

with open('./firmware.bin', 'wb') as f:
	f.write(mybin)
	f.close()
	