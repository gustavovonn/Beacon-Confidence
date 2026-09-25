#!/usr/bin/env python3
import time, requests, json

get_url ='https://iot.lisha.ufsc.br/api/get.php'

epoch = int(time.time() * 1000000)
query = {
        'series' : {
            'version' : '1.2',
            'unit'    : 0xC4963924,
            'r'       : 99999999,
            't0'      : 1672531200000000,
            'tf'      : 1672531230000000,
            'dev'   : 0,
            'signature': 7777
        },
        'credentials' : {
        	'domain' : 'tutorial',
                'username' : 'tutorial',
                'password' : 'tuto20182'
        }
    }
session = requests.Session()
session.headers = {'Content-type' : 'application/json'}

response = session.post(get_url, json.dumps(query))

print(response.text)

# print("Get [", str(response.status_code), "] (", len(query), ") ", query, sep='')
# if response.status_code == 200:
# 	print(json.dumps(response.json(), indent=4, sort_keys=False))