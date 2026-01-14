# upload.py

import json
import requests

with open('./firmware.bin', 'rb') as f:
	res = requests.post('https://file.io/', files={ 'file': ( "firmware.bin", f ) })
	key = json.loads(res.text)['key']
	print(f'{key = }')