import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

def camreceptions(data):
    camrecgendeltatime=data[data.name == 'reception:vector(camGenerationDeltaTime)']
    stationid=data[data.name == 'reception:vector(camStationId)']
    camrecgendeltatime=camrecgendeltatime[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"gendeltatime"})
    stationid=stationid[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"stationid"})

    camrecgendeltatime.module.unique(), stationid.module.unique(), 
    camrecgendeltatime=camrecgendeltatime.merge(stationid, left_on="module", right_on="module", how="left").drop(columns=["vectime_y"]).rename(columns={"vectime_x":"vectime"})


    vectime = pd.DataFrame(camrecgendeltatime.vectime.to_list(), index=camrecgendeltatime.module).stack() 
    vectime = vectime.reset_index(["module"]) 
    vectime.columns = ["module", "vectime"] 
    vectime['id'] = np.arange(0, len(vectime))

    lgendeltatime = pd.DataFrame(camrecgendeltatime.gendeltatime.to_list(), index=camrecgendeltatime.module).stack() 
    lgendeltatime = lgendeltatime.reset_index(["module"]) 
    lgendeltatime.columns = ["module", "gendeltatime"] 
    lgendeltatime['id'] = np.arange(0, len(lgendeltatime))

    stationid = pd.DataFrame(camrecgendeltatime.stationid.to_list(), index=camrecgendeltatime.module).stack() 
    stationid = stationid.reset_index(["module"]) 
    stationid.columns = ["module", "stationid"] 
    stationid['id'] = np.arange(0, len(stationid))

    return vectime.merge(lgendeltatime, left_on=["id","module"], right_on=["id","module"], how="left").merge(stationid, left_on=["id","module"], right_on=["id","module"], how="left")

def camtransmissions(data):
    camtragendeltatime=data[data.name == 'transmission:vector(camGenerationDeltaTime)']
    stationid=data[data.name == 'transmission:vector(camStationId)']
    camtragendeltatime=camtragendeltatime[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"gendeltatime"})
    stationid=stationid[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"stationid"})

    camtragendeltatime.module.unique(), stationid.module.unique(), 
    camtragendeltatime=camtragendeltatime.merge(stationid, left_on="module", right_on="module", how="left").drop(columns=["vectime_y"]).rename(columns={"vectime_x":"vectime"})

    vectime = pd.DataFrame(camtragendeltatime.vectime.to_list(), index=camtragendeltatime.module).stack() 
    vectime = vectime.reset_index(["module"]) 
    vectime.columns = ["module", "vectime"] 
    vectime['id'] = np.arange(0, len(vectime))
    vectime

    gendeltatime = pd.DataFrame(camtragendeltatime.gendeltatime.to_list(), index=camtragendeltatime.module).stack() 
    gendeltatime = gendeltatime.reset_index(["module"]) 
    gendeltatime.columns = ["module", "gendeltatime"] 
    gendeltatime['id'] = np.arange(0, len(gendeltatime))
    gendeltatime

    stationid = pd.DataFrame(camtragendeltatime.stationid.to_list(), index=camtragendeltatime.module).stack() 
    stationid = stationid.reset_index(["module"]) 
    stationid.columns = ["module", "stationid"] 
    stationid['id'] = np.arange(0, len(stationid))
    stationid

    return vectime.merge(gendeltatime, left_on=["id","module"], right_on=["id","module"], how="left").merge(stationid, left_on=["id","module"], right_on=["id","module"], how="left")
    
def camlatencies(datarx, datatx):
    latency=datatx.merge(datarx, left_on=["stationid","gendeltatime"], right_on=["stationid","gendeltatime"], how="left")
    latency["latency"]=latency["vectime_y"]-latency["vectime_x"]
    latency["simtime"]=latency["vectime_x"].transform(lambda x: np.floor(x))
    return latency.drop(columns=["module_y", "id_x", "id_y"])

def summarizecamlatencies(data):
    latency=data

    latency["min"]=latency.groupby(["simtime"])["latency"].transform(lambda x: x.min())
    latency["max"]=latency.groupby(["simtime"])["latency"].transform(lambda x: x.max())
    latency["mean"]=latency.groupby(["simtime"])["latency"].transform(lambda x: x.mean())
    latency=latency.drop(columns=["vectime_x", "vectime_y"])

    summary_latency=latency.drop_duplicates(subset=["min", "max", "mean"])
    summary_latency["sortkey"]=summary_latency["module_x"].apply(lambda x: int(x[x.find("[")+1:x.find("]")]) )
    summary_latency=summary_latency.sort_values(by=["sortkey"])

    latency["sortkey"]=latency["module_x"].apply(lambda x: int(x[x.find("[")+1:x.find("]")]) )
    latency=latency.sort_values(by=["sortkey"])
    return summary_latency

def plotcamlatency(data):
    node0latency=data.sort_values(by=["simtime"])

    val = []
    fig, ax = plt.subplots()
    curr=0
    for names in node0latency["module_x"].unique():
        print(len(node0latency[node0latency.module_x==names]))
        for i in range(len(node0latency[node0latency.module_x==names])):
            min=node0latency[node0latency.module_x==names]["min"].to_list()[i]*1000
            max=node0latency[node0latency.module_x==names]["max"].to_list()[i]*1000
            mean=node0latency[node0latency.module_x==names]["mean"].to_list()[i]*1000
        
            # print([node0latency[node0latency.module_x==names]["simtime"].to_list()[i], min, mean, max])

            ax.plot([curr,curr,curr], [min, mean, max], marker="_", linestyle="dotted", color="black")
            curr+=1
        
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("CAM latency over time")

    # plt.show()
    plt.savefig('camlatency.png', dpi=600)



### CPM methods

def cpmreceptions(data):
    cpmrecgendeltatime=vectors[vectors.name == 'reception:vector(cpmGenerationDeltaTime)']
    stationid=vectors[vectors.name == 'reception:vector(cpmStationId)']
    cpmrecgendeltatime=cpmrecgendeltatime[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"gendeltatime"})
    stationid=stationid[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"stationid"})

    cpmrecgendeltatime.module.unique(), stationid.module.unique(), 
    cpmrecgendeltatime=cpmrecgendeltatime.merge(stationid, left_on="module", right_on="module", how="left").drop(columns=["vectime_y"]).rename(columns={"vectime_x":"vectime"})
    cpmrecgendeltatime

    vectime = pd.DataFrame(cpmrecgendeltatime.vectime.to_list(), index=cpmrecgendeltatime.module).stack() 
    vectime = vectime.reset_index(["module"]) 
    vectime.columns = ["module", "vectime"] 
    vectime['id'] = np.arange(0, len(vectime))
    vectime

    lgendeltatime = pd.DataFrame(cpmrecgendeltatime.gendeltatime.to_list(), index=cpmrecgendeltatime.module).stack() 
    lgendeltatime = lgendeltatime.reset_index(["module"]) 
    lgendeltatime.columns = ["module", "gendeltatime"] 
    lgendeltatime['id'] = np.arange(0, len(lgendeltatime))
    lgendeltatime

    stationid = pd.DataFrame(cpmrecgendeltatime.stationid.to_list(), index=cpmrecgendeltatime.module).stack() 
    stationid = stationid.reset_index(["module"]) 
    stationid.columns = ["module", "stationid"] 
    stationid['id'] = np.arange(0, len(stationid))
    stationid

    return vectime.merge(lgendeltatime, left_on=["id","module"], right_on=["id","module"], how="left").merge(stationid, left_on=["id","module"], right_on=["id","module"], how="left")

def cpmtransmissions(data):
    cpmtragendeltatime=vectors[vectors.name == 'transmission:vector(cpmGenerationDeltaTime)']
    stationid=vectors[vectors.name == 'transmission:vector(cpmStationId)']
    cpmtragendeltatime=cpmtragendeltatime[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"gendeltatime"})
    stationid=stationid[["module","vectime", "vecvalue"]].rename(columns={"vecvalue":"stationid"})

    cpmtragendeltatime.module.unique(), stationid.module.unique(), 
    cpmtragendeltatime=cpmtragendeltatime.merge(stationid, left_on="module", right_on="module", how="left").drop(columns=["vectime_y"]).rename(columns={"vectime_x":"vectime"})

    vectime = pd.DataFrame(cpmtragendeltatime.vectime.to_list(), index=cpmtragendeltatime.module).stack() 
    vectime = vectime.reset_index(["module"]) 
    vectime.columns = ["module", "vectime"] 
    vectime['id'] = np.arange(0, len(vectime))
    vectime

    gendeltatime = pd.DataFrame(cpmtragendeltatime.gendeltatime.to_list(), index=cpmtragendeltatime.module).stack() 
    gendeltatime = gendeltatime.reset_index(["module"]) 
    gendeltatime.columns = ["module", "gendeltatime"] 
    gendeltatime['id'] = np.arange(0, len(gendeltatime))
    gendeltatime

    stationid = pd.DataFrame(cpmtragendeltatime.stationid.to_list(), index=cpmtragendeltatime.module).stack() 
    stationid = stationid.reset_index(["module"]) 
    stationid.columns = ["module", "stationid"] 
    stationid['id'] = np.arange(0, len(stationid))
    stationid

    return vectime.merge(gendeltatime, left_on=["id","module"], right_on=["id","module"], how="left").merge(stationid, left_on=["id","module"], right_on=["id","module"], how="left")
    

def cpmlatencies(datarx, datatx):
    latency=datatx.merge(datarx, left_on=["stationid","gendeltatime"], right_on=["stationid","gendeltatime"], how="left")
    latency["latency"]=latency["vectime_y"]-latency["vectime_x"]
    latency["simtime"]=latency["vectime_x"].transform(lambda x: np.floor(x))
    return latency.drop(columns=["module_y", "id_x", "id_y"])

def summarizecpmlatencies(data):
    latency=data

    latency["min"]=latency.groupby(["simtime"])["latency"].transform(lambda x: x.min())
    latency["max"]=latency.groupby(["simtime"])["latency"].transform(lambda x: x.max())
    latency["mean"]=latency.groupby(["simtime"])["latency"].transform(lambda x: x.mean())
    latency=latency.drop(columns=["vectime_x", "vectime_y"])

    summary_latency=latency.drop_duplicates(subset=["min", "max", "mean"])
    summary_latency["sortkey"]=summary_latency["module_x"].apply(lambda x: int(x[x.find("[")+1:x.find("]")]) )
    summary_latency=summary_latency.sort_values(by=["sortkey"])

    latency["sortkey"]=latency["module_x"].apply(lambda x: int(x[x.find("[")+1:x.find("]")]) )
    latency=latency.sort_values(by=["sortkey"])
    return summary_latency

def plotcpmlatency(data):
    node0latency=data.sort_values(by=["simtime"])

    val = []
    fig, ax = plt.subplots()
    curr=0
    for names in node0latency["module_x"].unique():
        print(len(node0latency[node0latency.module_x==names]))
        for i in range(len(node0latency[node0latency.module_x==names])):
            min=node0latency[node0latency.module_x==names]["min"].to_list()[i]*1000
            max=node0latency[node0latency.module_x==names]["max"].to_list()[i]*1000
            mean=node0latency[node0latency.module_x==names]["mean"].to_list()[i]*1000
        
            # print([node0latency[node0latency.module_x==names]["simtime"].to_list()[i], min, mean, max])

            ax.plot([curr,curr,curr], [min, mean, max], marker="_", linestyle="dotted", color="black")
            curr+=1
        
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Latency (ms)")
    ax.set_title("CPM latency over time")

    # plt.show()
    plt.savefig('cpmlatency.png', dpi=600)

cwd = os.getcwd()
# print(cwd)
# scavetool x *.sca *.vec -o routing.csv
filess=[]
for root, dirs, files in os.walk(cwd):
    for file in files:
        if file.endswith(".sca"):
            filename=file[file.find("_")+1:file.find("-")]
            cmdcall="scavetool x "+root+"/*.sca "+root+"/*.vec -o "+root+"/"+filename+".csv"
            filepath=root+"/"+filename+".csv"
            filess.append(filepath)
            if(not os.path.isfile(filepath)):
                print(filepath+" not found\n\nGenerating")
                os.system(cmdcall)
            else:
                print("Found "+filepath)


filess.sort()

for file in filess:
    print("Loading "+file)
    routing = pd.read_csv(file, converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})
    vectors = routing[routing.type=='vector']

    cam_rx = camreceptions(vectors)

    print(cam_rx.head())

    cam_tx = camtransmissions(vectors)

    print(cam_tx.head())

    camlat=camlatencies(cam_rx, cam_tx)

    print(camlat.head())

    camlatsum=summarizecamlatencies(camlat)

    print(camlatsum.head())

    plotcamlatency(camlatsum)

    routing = pd.read_csv(file, converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})
    vectors = routing[routing.type=='vector']


    cpm_rx = cpmreceptions(vectors)

    cpm_tx = cpmtransmissions(vectors)

    cpmlat=camlatencies(cpm_rx, cpm_tx)

    cpmlatsum=summarizecpmlatencies(cpmlat)

    plotcpmlatency(cpmlatsum)

    # print(latsum)

    break