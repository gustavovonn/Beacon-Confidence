import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import json
import re
from geo_coordinates import WGS84Transformer
from math import pi
import plotly
import plotly.express as px

def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

def load_log_file(full_path: str) :
    # Obtain list[dict] from the log files
    parsed_data = []
    with open(full_path, "r") as input_file:
        for line in input_file:
            line = hacks_to_make_it_work(line=line)
            try:
                # print(line)
                # We have to load json objects line by line
                json_dict = json.loads(line)
                if isinstance(json_dict, list):
                    raise LineIsListError
                parsed_data.append(json_dict)
            except json.JSONDecodeError as e:
                print(f"Error parsing JSON: {e}")

    return parsed_data

class LineIsListError(json.JSONDecodeError):
    def __init__(self, message="We expect this line to be a json object, not a list"):
        self.message = message
        super().__init__(self.message)


def hacks_to_make_it_work(line: str) -> str:
    line = add_missing_double_quotes(line)
    line = remove_additional_double_quotes(line)
    line=line.replace("\",\"", ",\"")
    line=line.replace(",\"c", "\",\"c")
    line=line.replace("=", ":")
    line = add_missing_double_quotes(line)
    line=line.replace(",\"c", "\",\"c")
    line=line.replace("\"\"", "\"")
    return line


def remove_additional_double_quotes(line: str) -> str:
    pattern_to_detect_1 = '"uncertainty"=100,"length"=1'
    pattern_to_replace_1 = "uncertainty=100,length=1"
    line = line.replace(pattern_to_detect_1, pattern_to_replace_1)

    pattern_to_detect_2 = '"uncertainty"=0,"length"=1'
    pattern_to_replace_2 = "uncertainty=0,length=1"
    line = line.replace(pattern_to_detect_2, pattern_to_replace_2)
    return line


def add_missing_double_quotes(line: str) -> str:
    pattern = r'(?<!")(\b\w+\b)(?=:)'  # matches keys that are not enclosed in double quotes
    formatted_json = re.sub(pattern, r'"\1"', line)  # add double quotes to keys that did not have them
    return formatted_json





cwd = os.getcwd()
# print(cwd)

GTfiles=[]
mainfile=0
for root, dirs, files in os.walk(cwd):
    for file in files:
        if file.endswith(".out"):
            if(file == "localperception-vehicle_0.out"):
                mainfile=root+"/"+file
            elif(file.startswith("monitor_car") or file.startswith("localperceptionGT") ):
                filepath=root+"/"+file
                # print(filepath)
                GTfiles.append(filepath)
            

GTfiles.sort()

# print(type(data[0]))

def get_x(row):
    transformer = WGS84Transformer(0,0)
    microradiants_to_degrees = 1e-7 * 180 / pi  # Note: should be 1e-6, we will figure out conversion error.
    longitude_deg = row["lat"] * microradiants_to_degrees
    latitude_deg = row["lon"] * microradiants_to_degrees
    easting, northing = transformer.transform_wgs84_to_local_metric(longitude=longitude_deg, latitude=latitude_deg)
    return easting;

def get_y(row):
    transformer = WGS84Transformer(0,0)
    microradiants_to_degrees = 1e-7 * 180 / pi  # Note: should be 1e-6, we will figure out conversion error.
    longitude_deg = row["lat"] * microradiants_to_degrees
    latitude_deg = row["lon"] * microradiants_to_degrees
    easting, northing = transformer.transform_wgs84_to_local_metric(longitude=longitude_deg, latitude=latitude_deg)
    return northing;

def get_df(filepath):
    data =load_log_file(filepath)
    if("monitor_car" in filepath):
        df = pd.json_normalize(data).drop(columns=["State.parking", "State.steering.uncertainty", "State.steering.value", "State.orientation.uncertainty", "State.orientation.value", "State.unit", "State.class", "State.uncertainty", "State.length", "State.speed.uncertainty", "State.heading.uncertainty", "State.yaw_rate.uncertainty", "State.acceleration.uncertainty", "State.dim"])
        df=df.rename(columns={"State.origin.id":"stationid", "State.origin.idlocal":"idlocal", "State.origin.timestamp":"timestamp", "State.speed.value":"speed", "State.heading.value":"heading", "State.acceleration.value":"acceleration", "State.yaw_rate.value":"yawrate"})
        df=pd.concat([df.drop(['State.loc'], axis=1), df['State.loc'].apply(pd.Series)], axis=1).rename(columns={0:"lat", 1:"lon"}).drop(columns=[2])
        df=pd.concat([df.drop(['lat'], axis=1), df['lat'].apply(pd.Series)], axis=1).rename(columns={"value":"lat"}).drop(columns=["uncertainty"])
        df=pd.concat([df.drop(['lon'], axis=1), df['lon'].apply(pd.Series)], axis=1).rename(columns={"value":"lon"}).drop(columns=["uncertainty"])
        df['posx'] = df.apply(get_x, axis=1)
        df['posy'] = df.apply(get_y, axis=1)

    else:
        df = pd.json_normalize(data).drop(columns=[ "Motion_Vector_Global.unit", "Motion_Vector_Global.class", "Motion_Vector_Global.uncertainty", "Motion_Vector_Global.length", "Motion_Vector_Global.speed.uncertainty", "Motion_Vector_Global.heading.uncertainty", "Motion_Vector_Global.yaw_rate.uncertainty", "Motion_Vector_Global.acceleration.uncertainty", "Motion_Vector_Global.dim"])
        df=df.rename(columns={"Motion_Vector_Global.origin.idlocal":"idlocal", "Motion_Vector_Global.origin.id":"stationid", "Motion_Vector_Global.origin.timestamp":"timestamp", "Motion_Vector_Global.speed.value":"speed", "Motion_Vector_Global.heading.value":"heading", "Motion_Vector_Global.acceleration.value":"acceleration", "Motion_Vector_Global.yaw_rate.value":"yawrate"})
        df=pd.concat([df.drop(['Motion_Vector_Global.loc'], axis=1), df['Motion_Vector_Global.loc'].apply(pd.Series)], axis=1).rename(columns={0:"lat", 1:"lon"}).drop(columns=[2])
        df=pd.concat([df.drop(['lat'], axis=1), df['lat'].apply(pd.Series)], axis=1).rename(columns={"value":"lat"}).drop(columns=["uncertainty"])
        df=pd.concat([df.drop(['lon'], axis=1), df['lon'].apply(pd.Series)], axis=1).rename(columns={"value":"lon"}).drop(columns=["uncertainty"])
        df['posx'] = df.apply(get_x, axis=1)
        df['posy'] = df.apply(get_y, axis=1)
    return df

gtdf=pd.DataFrame()

# data0 = get_df(GTfiles[0])
# print(data0)

# data1 = get_df(GTfiles[1])
# print(data1)

# data2 = get_df(GTfiles[2])
# print(data2)

for files in GTfiles:
    # print(files)
    data = get_df(files)
    gtdf=pd.concat([gtdf, data])
    # break

gtdf=gtdf.drop_duplicates()
    

# gtdf[gtdf.stationid==447]#.drop_duplicates(subset=["timestamp", "max", "mean"])
perc = get_df(mainfile)
# perc[perc.stationid==2409]



# print(gtdf)



# gtdf[gtdf.stationid==2409]
gtdf=gtdf.drop_duplicates(subset=["timestamp", "stationid"])

def error_spd(row):
    return row["speed"]-row["speed_gt"];

def error_head(row):
    return row["heading"]-row["heading_gt"];

def error_accel(row):
    return row["acceleration"]-row["acceleration_gt"];

def error_lat(row):
    return row["lat"]-row["lat_gt"];

def error_lon(row):
    return row["lon"]-row["lon_gt"];

def error_posx(row):
    return row["posx"]-row["posx_gt"];

def error_posy(row):
    return row["posy"]-row["posy_gt"];

test3=perc.merge(gtdf, on=["stationid", "timestamp"], suffixes=["", "_gt"])
test3["speed_delta"]=test3.apply(error_spd, axis=1)
test3["heading_delta"]=test3.apply(error_head, axis=1)
test3["acceleration_delta"]=test3.apply(error_accel, axis=1)
test3["lat_delta"]=test3.apply(error_lat, axis=1)
test3["lon_delta"]=test3.apply(error_lon, axis=1)
test3["posx_delta"]=test3.apply(error_posx, axis=1)
test3["posy_delta"]=test3.apply(error_posy, axis=1)


df=test3
df = df.drop(df[df['lon'] < -40].index)
df = df.drop(df[df['lon'] > 640].index)
df = df[df.timestamp>100000000]
# df = df.drop(df[df['lon'] == -1508.662605].index)
stats=df[["stationid", "timestamp", "speed_delta", "heading_delta", "acceleration_delta", "posx_delta", "posy_delta"]].groupby("timestamp").describe()
# da=test3.drop(columns=[""])
df["timestamp"]=df["timestamp"]/1000000
df["timestamp"]=df["timestamp"]-df["timestamp"].values[0]



test=df
test["timestamp"]=(test["timestamp"]).transform(lambda x: np.floor(x))


deltares=test.abs().describe()[["posx_delta", "posy_delta", "speed_delta", "heading_delta"]]

deltares=test.abs().groupby(["timestamp"]).describe()[["posx_delta", "posy_delta", "speed_delta", "heading_delta"]]

names=["posx_delta", "posy_delta", "speed_delta", "heading_delta"]

fig, ax = plt.subplots()
for x in names:
    # print(deltares[x])
    ax.scatter(deltares[x].index.values.tolist(), deltares[x]["std"], label=x+" std", s=4)


    

# for x in deltares["stationid"].unique():
#     print(x)
#     vals=df[df.stationid==x]
#     # print(vals)
#     time=np.array(vals["timestamp"].values.tolist())/1000000

#     time=time-time[0]
#     ax.scatter(time, vals["speed_gt"], color="red", s=2, label="Ground Truth")
#     ax.scatter(time, vals["speed"], color="black", s=2, label="Predicted")
#     ax.set_xlabel("Time (s)")
#     ax.set_ylabel("Speed (m/s)")
#     ax.set_title("Tracked object speed")
ax.legend()

deltares=test.abs().describe()[["posx_delta", "posy_delta", "speed_delta", "heading_delta"]]

t = pd.DataFrame(columns=['posx_delta', 'posy_delta', 'speed_delta', 'heading_delta'])
t.loc['RMSE'] = [(df["posx_delta"]**2).mean()**.5, (df["posy_delta"]**2).mean()**.5, (df["speed_delta"]**2).mean()**.5, (df["heading_delta"]**2).mean()**.5]
deltares=deltares.append(t)
print(deltares)

# plt.show()
plt.savefig("aerrorplot.png", dpi=600)