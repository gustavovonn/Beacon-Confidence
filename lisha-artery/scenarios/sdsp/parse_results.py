with open("results/log.txt", "r") as f:
    log_devs = f.read().split("METRICS DEVICE")[1:] # skip first portion of the log once it is trash from execution

METRICS = ["\nECDH Latency: \n", 
           "\nECDH Request Latency (simTime):\n", 
           "\nPTP Latency (chrono):\n",
           "\nPTP Request Latency (simTime):\n",
           "\nOTP Latency (chrono):\n",
           "\nAuth Request Latency (simTime):\n",
           "\nGroup Key Processing (chrono):\n",
           "\nGroup Key Distribution (simTime):\n",
           "\nSec Msg Encrypt (chrono):\n",
           "\nSec Msg Decrypt (chrono):\n",
           "\nSec Msg Net Latency (simTime):\n", None]

def get_metric_value(metric: str, next_metric : str, text : str) -> float:
    try:
        if (next_metric is None):
            metric_value = text.split(metric)[1].split("\n")[0].split(",")[1]
        else:
            metric_value = text.split(metric)[1].split(next_metric)[0].split("\n")[0].split(",")[1]
    except:
        return 0.0
    return float(metric_value.replace("\t", "")) 

devices = {}
devices["0.0"] = {}
devices["1.0"] = {}
devices["2.0"] = {}
for entry in log_devs:
    text = entry.split("Device Finished;")[0]
    ecdh = text.split("\nECDH Latency: \n")
    dev = ecdh[0].replace(" ", "")
    #if (dev == "0.0"): 
        #print(dev)
    for i, metric in enumerate(METRICS):
        #if (dev == "0.0" and metric == "\nECDH Latency: \n"):
        #    print(text.split(metric)[1].split(METRICS[i+1])[0])
        if (i+1 == len(METRICS)):
            break
        val = get_metric_value(metric, METRICS[i+1], text)
        try:
            devices[dev][metric.replace(":", "").replace(" ","").replace("\n", "")].append(val)
        except:
            devices[dev][metric.replace(":", "").replace(" ","").replace("\n", "")] = [val]

print(devices["0.0"])
print("-----------")
print(devices["1.0"])
print("-----------")
print(devices["2.0"])


import pandas as pd

df_0 = pd.DataFrame(devices["0.0"]) # this one has columns with 0 values, we need to ignore those in avg calculation
df_1 = pd.DataFrame(devices["1.0"]) 
df_2 = pd.DataFrame(devices["2.0"]) # skip this one

df_combined = pd.concat([df_0, df_1], ignore_index=True)

print(df_combined)