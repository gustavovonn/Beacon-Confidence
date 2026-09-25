#!/usr/bin/env python3

import os, sys, time, datetime, argparse, csv, json, struct, ssl, requests
from struct import unpack, unpack_from, pack_into, pack
from base64 import b64encode
from http.client import HTTPSConnection
import math
from requests.packages.urllib3.exceptions import InsecureRequestWarning

requests.packages.urllib3.disable_warnings(InsecureRequestWarning)


from os import listdir
from os.path import isfile, join, isdir

location_unit = 0xC4B24924 # longitude (dev=0), latitude (dev=1), altitude (dev=2)
speed_unit = 0xC4963924 # (dev=0)
heading_unit =0xC4B24924 # (dev=3)
yaw_rate_unit = 0xC4B23924 # (dev=0)
accel_unit = 0xC4962924 # (dev=0)

digital_unit = 0x4924924
time_unit = 0x84925924

lon_dev=0
lat_dev=1
alt_dev=2
speed_dev=0
heading_dev=3
yaw_rate_dev=0
accel_dev=0

# starttime    = 1672531200000000
endtime=900000000

HOST = "iot.lisha.ufsc.br"

create_url ='https://'+HOST+'/api/create.php'
mv_put_url ='https://'+HOST+'/api/mv_put.php'
get_url = 'https://'+HOST+'/api/get.php'
put_url ='https://'+HOST+'/api/put.php'
finish_url = 'https://'+HOST+'/api/finish.php'

CREDS = { "domain": "tutorial",  "username": "tutorial", "password": "tuto20182" }
payload={}

def create_series(unit, dev, sig):
	global CREDS
	global starttime
	global endtime

	create_series = { "series": {
		"version": "1.2",
		"unit": unit,
		"t0": starttime,
		"t1": starttime+endtime,
		"workflow" : 0,
		"dev" : dev, # Unlike version 1.1, dev must be defined in series.  If there is another device with same unit, another series must be created.
		"signature": sig
		}, "credentials" : CREDS }

	return create_series

def get_create_series(name, data):
    global starttime
    global endtime

    res=[]
    res.append(create_series(location_unit, lon_dev, name))
    res.append(create_series(location_unit, lat_dev, name))
    res.append(create_series(location_unit, alt_dev, name))

    res.append(create_series(speed_unit, speed_dev, name))

    res.append(create_series(heading_unit, heading_dev, name))

    res.append(create_series(yaw_rate_unit, yaw_rate_dev, name))

    res.append(create_series(accel_unit, accel_dev, name))

    return res

def create_smartdata(unit, _x, _y, _z, _sig, _dev, _ts, _val, _conf):
	global CREDS
	global starttime
	sd={
		'version' : '1.2',
		'unit'    : unit,
		'timestamp' : int(starttime+_ts),
		'value'   : float(_val),
		'error'   : 0,
		'confidence' : 0,
		'x'       : float(_x),
		'y'       : float(_y),
		'z'       : float(_z),
		'signature' : _sig,
		'dev'     : _dev
		}

	return sd


def get_smartdata(vehid, contents):
    global starttime_gap
    time_str="timestamp"
    # print(contents)
    currtime = float(contents.get("origin").get(time_str))-starttime_gap
    # print("currtime")
    # print(currtime)
    res = []
    # print(contents)
    # print(contents.get("speed").get("value"))


    res.append(create_smartdata(location_unit, contents.get("loc")[0].get("value"), contents.get("loc")[1].get("value"), contents.get("loc")[2].get("value"), vehid, lon_dev, currtime, contents.get("loc")[0].get("value"), 100.0))
    res.append(create_smartdata(location_unit, contents.get("loc")[0].get("value"), contents.get("loc")[1].get("value"), contents.get("loc")[2].get("value"), vehid, lon_dev, currtime, contents.get("loc")[1].get("value"), 100.0))
    res.append(create_smartdata(location_unit, contents.get("loc")[0].get("value"), contents.get("loc")[1].get("value"), contents.get("loc")[2].get("value"), vehid, lon_dev, currtime, contents.get("loc")[2].get("value"), 100.0))

    res.append(create_smartdata(location_unit, contents.get("loc")[0].get("value"), contents.get("loc")[1].get("value"), contents.get("loc")[2].get("value"), vehid, speed_dev, currtime, contents.get("speed").get("value"), 100.0))

    res.append(create_smartdata(location_unit, contents.get("loc")[0].get("value"), contents.get("loc")[1].get("value"), contents.get("loc")[2].get("value"), vehid, heading_dev, currtime, contents.get("heading").get("value"), 100.0))

    res.append(create_smartdata(location_unit, contents.get("loc")[0].get("value"), contents.get("loc")[1].get("value"), contents.get("loc")[2].get("value"), vehid, yaw_rate_dev, currtime, contents.get("yaw_rate").get("value"), 100.0))

    res.append(create_smartdata(location_unit, contents.get("loc")[0].get("value"), contents.get("loc")[1].get("value"), contents.get("loc")[2].get("value"), vehid, accel_dev, currtime, contents.get("acceleration").get("value"), 100.0))

    # print(res)
    return res


def create_iot_series(session, vehid, contents):
	creation_series=(get_create_series(vehid, contents))
	for x in creation_series:
		# print(x)
		response = session.post(create_url, json.dumps(x), verify=False)
		resp = response.status_code
		# print(resp)
		if resp != 204:
			print("Creation failed, aborting test...")
			return False
		# else:
		# 	print("Create with finish V1.2 - OK");

def send_iot_smartdata(session, vehid, contents):
	creation_series=(get_smartdata(vehid, contents.get("Motion_Vector_Global")))
	# return
	for x in creation_series:
		# print(x)
		response = session.post(put_url, json.dumps(x), verify=False)
		resp = response.status_code
		# print(resp)
		if resp != 204:
			print("Sending failed, aborting test...")
			return False
		# else:
		# 	print("Create with finish V1.2 - OK");

def read_file(path, name, vehid):
    global starttime_gap
    vehcreated = False
    session = requests.Session()

    # Create the series
    session.headers = {'Content-type' : 'application/json'}
    time_str="timestamp"

    print(vehid)
    with open(path+name) as f:
        count=0
        sd = { "smartdata": [], "credentials" : CREDS }
        for contents in f.readlines():
            # contents=contents[contents.index(":")+1:]
            # print(contents)
            contents=json.loads(contents)
            if(starttime_gap==-1):
                # print(contents.get('Motion_Vector_Global').get("origin").get(time_str))
                starttime_gap= int(contents.get('Motion_Vector_Global').get("origin").get(time_str))
            if(not vehcreated):
                vehcreated=True
                create_iot_series(session, vehid, contents)
            for x in get_smartdata(vehid, contents.get("Motion_Vector_Global")):
                sd["smartdata"].append(x)
            
            count=len(sd["smartdata"])
            if(count>=100):
                count=0
                resp=0
                while(resp!=204):
                    response = session.post(put_url, json.dumps(sd), verify=False)
                    resp = response.status_code
                    if resp != 204:
                        print("Muliple insertion V1.2 failed: ", resp, " - trying again...")
                        if resp==400:
                            print(response)
                            print(sd)
                            raise Exception("error")
                sd = { "smartdata": [], "credentials" : CREDS }

    resp=0
    while(resp!=204):
        response = session.post(put_url, json.dumps(sd), verify=False)
        resp = response.status_code
        if resp != 204:
            print("Muliple insertion V1.2 failed: ", resp, " - trying again...")
            if resp==400:
                print(sd)
                raise Exception("error")

starttime=1672531200000000
starttime_gap=-1
read_file("results/results_0/","monitor_car-vehicle_0.out", "7777")