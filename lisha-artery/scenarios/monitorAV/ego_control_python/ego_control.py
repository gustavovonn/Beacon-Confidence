import os
import errno
import time, requests, json

from enum import Enum

get_url ='https://iot.lisha.ufsc.br/api/get.php'

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

controlinput = "/tmp/controlinput"
controloutput = "/tmp/controloutput"


def read_input():
    outputfifo = open(controloutput, "r")
    data =""
    while(len(data)==0):
        data = outputfifo.read()
    outputfifo.close()
    return data

def write_command(wcmd):
    inputfifo = open(controlinput, "w")
    inputfifo.write(wcmd)
    inputfifo.close()

class commands(Enum):
    set_pos=1
    set_target=2
    set_report_mv=3
    set_route=4
    set_route_speed=5

config ={
    # "set_pos": { "pos": [375.0, 286.0]},
    # "set_target": { "pos": [0.0, 30.0]},
    # "set_report_mv": 0,
    "set_route": 0
}

print("Waiting for ego controller:")

data = read_input()

time.sleep(1)

write_command("Done")

read_mv=False

for configkey in config.keys():
    data = read_input()

    write_command(str(commands[configkey].value))

    if(configkey == commands.set_pos.name):
        print("Setting position")
        data = read_input()
        write_command(str(config[configkey]["pos"][0]))
        data = read_input()
        write_command(str(config[configkey]["pos"][1]))
    if(configkey == commands.set_target.name):
        print("Setting target")
        data = read_input()
        write_command(str(config[configkey]["pos"][0]))
        data = read_input()
        write_command(str(config[configkey]["pos"][1]))
    if(configkey == commands.set_report_mv.name):
        print("Setting report mv")
        read_mv=True
    if(configkey == commands.set_route.name or configkey == commands.set_route_speed.name):
        print("Setting route")
        response = session.post(get_url, json.dumps(query))

        if response.status_code == 200:
            print("SmartData Query ok")
            smartdata = json.loads(response.text)
        else:
            print("Error in query")
        data = read_input()
        write_command(str(len(smartdata.get("series"))))
        for line in smartdata.get("series"):
            data = read_input()
            write_command(json.dumps(line))


data = read_input()

write_command(str(0))


if(read_mv):
    while(True):
        data = read_input()
        if(data == "mv"):
            write_command("command")
            data = read_input()
            print(data)
        elif(data == "exitexitexit"):
            break


# data =""
# while(len(data)==0):
#     data = outputfifo.read()
# print("received data")
# print(data)
# data =""

# inputfifo.write("Done1\n")
# data =""
# while(len(data)==0):
#     data = outputfifo.read()
# print("received data")
# print(data)
# inputfifo.write("Done2\n")


# inputfifo.close()
# outputfifo.close()
# while True:
#     print("Opening FIFO...")
#     with open(FIFO) as fifo:
#         print("FIFO opened")
#         while True:
#             data = fifo.read()
#             if len(data) == 0:
#                 print("Writer closed")
#                 break
#             print('Read: "{0}"'.format(data))