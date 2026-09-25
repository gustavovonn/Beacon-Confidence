import os
import errno
import time, requests, json

get_url ='https://iot.lisha.ufsc.br/api/get.php'
put_url ='https://iot.lisha.ufsc.br/api/put.php'
create_url = 'https://iot.lisha.ufsc.br/api/create.php'


session = requests.Session()
session.headers = {'Content-type' : 'application/json'}

sdchannel = "/tmp/sdchannel"
sdcontrol = "/tmp/sdcontrol"

startts=90*1000000
endts=0
starttime=0

smartdatacount=10

sdlist=[]

import requests, json, struct, base64
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)
session = requests.Session()

def read_input():
    outputfifo = open(sdchannel, "r")
    data =""
    while(len(data)==0):
        data += outputfifo.readline()
    outputfifo.close()
    return data.strip("\n")

def write_command(wcmd):
    inputfifo = open(sdcontrol, "w")
    inputfifo.write(wcmd)
    inputfifo.close()

def handleseries(data):
    global session
    # print(data)
    seriesjson = json.loads(data)
    seriesjson["series"]["t1"]=endts-seriesjson["series"]["t1"]+starttime
    seriesjson["series"]["t0"]=starttime
    print(seriesjson)
    session = requests.Session()

    # Create the series
    session.headers = {'Content-type' : 'application/json'}
    response = session.post(create_url, json.dumps(seriesjson), verify=False)
    resp = response.status_code
    if resp != 204:
        print(resp, "Creation failed, aborting test...")
        return False
    else:
        print("Create with finish V1.2 - OK")
        return True

# 1718962040,
# 1729724453,

# 1728362040,
def handlesd(data):
    global session
    global sdlist
    # print(data)
    seriessd = json.loads(data)
    seriessd["smartdata"][0]["timestamp"]=seriessd["smartdata"][0]["timestamp"]-startts+starttime
    print(seriessd)

    sdlist.append(seriessd)

    if(len(sdlist)>=smartdatacount):

        finalsd=sdlist.pop()
        while(len(sdlist)>0):
            tempsd=sdlist.pop()
            finalsd["smartdata"].append(tempsd["smartdata"][0])

        # print(finalsd)

        session = requests.Session()

        # send the SD
        session.headers = {'Content-type' : 'application/json'}
        response = session.post(put_url, json.dumps(seriessd), verify=False)
        resp = response.status_code
        if resp != 204:
            print(resp, "Send failed, aborting test...")
            return False
        else:
            print("Send with finish V1.2 - OK")
            return True

while(True):
    data = read_input()
    if("Connect" in data and starttime==0):
        starttime = int(time.time())
        print(starttime)
        data = data[0:-1]
        endts=int(float(data.split(" ")[1])*1000000)
        print(endts)
        
    if("series" in data):
        handleseries(data)
    elif("smartdata" in data):
        handlesd(data)

    write_command("ok\n")

# response = session.post(get_url, json.dumps(query))

# if response.status_code == 200:
#     print("SmartData Query ok")
#     smartdata = json.loads(response.text)
# else:
#     print("Error in query")
