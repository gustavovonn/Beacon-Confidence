Usefull Links for dependencies:
- Install Omnetpp (use version 5.6) https://omnetpp.org/download/old ; Guide: https://doc.omnetpp.org/omnetpp/InstallGuide.pdf
- Install SUMO from source (version 1.11.0) https://sumo.dlr.de/releases/1.11.0/ ; Guide: https://sumo.dlr.de/docs/Installing/Linux_Build.html
- Install Artery: http://artery.v2x-research.eu/install/

SmartData Model: https://lisha.ufsc.br/SDAV+-+SmartData+Model

How to set up this repository inside artery:
- clone this repository in another folder and "git checkout artery_sim" this branch
- rename ".git" folder at artery code root dir to ".git_artery"
- copy&paste ".git" from sdav to artery code root dir
- now git will use .git from sdav (which ignores artery folders)
- do a git status to test if it works
- do a "git pull"

Dependencies:
- nlohmann json parser (https://github.com/nlohmann/json/tree/develop)
-- Download branch and extract nlohmann from "single_include/json.hpp" file to the monitorAV folder

What to do to configure a new Service:
- Create service code following CamelCase (file name must match service class name)
- Add Define_Module(<"ServiceName">) at .cc file
- Add .ned file according to service (e.g., extends OtherService or like OtherService, see CAMonitor.ned and MonitorService.ned)
- add .cc file at "add_artery_feature(monitor MonitorService.cc CAMonitor.cc <"ServiceName">.cc)"
- Compile and run "cmake --build /home/hoffmann/Documents/artery/artery/build --target run_monitorAV -j <cpu_amount>"

What we have:
- A simulation based on rsu_grid where vehicles drive on a 5x5 artificial city grid. Vehicles follow fixed routes until the end of simulation unless collisions occurr. The omnetpp.ini file defines the overall network simulation and assigns the grid_scenario.sumocfg file as the physical scenario configuration.
grid_scenario.sumocfg defines the sumo simulation.
- grid_scenario_traffic_light.net.xml defines the roads and the traffic light positions in the network.
- grid_scenario_v200.rou.xml defines the vehicles and vehicle routes on the road.
    - Vehicles spawn at "depart" time.
        - Two types of vehicle are defined here: ego and car
        - The ego vehicle is the only one running the monitor service at the moment.
        - grid_scenario.poly.xml defines the buildings that block line of sight and create radio interference
    - vehicles.xml defines the types of vehicle that spawn.
        - PlainVehicle (Unconnected) and Car (Connected).
- MonitorService will periodically collect data from "ego" AV and store them in a file at results
- CAMonitor will collect all ETSI-CAM messages received by "ego" during simulation and store them in a file at results
- CaService is a Default service from Artery that handles ETSI-CAM communication, we only import it as a base service that every vehicle will use 

Compile and run:
- at artery build folder run cmake ..
- at artery root folder, run cmake --build /home/hoffmann/Documents/artery/artery/build --target run_monitorAV -j 10

TODO:
 1. Time Stamps: SIM_TIME or SECONDS SINCE EPOCH;
 2. Collisions in SUMO remove car from simulation: check if collision happened with EGO and finish simulation or re-spawn ego;
 3. Integrate with SmartData Framework (Replace files with SmartData Handlers)
 4. RSS Coordinates;
 5. Create a service to control of EGO vehicle; (Actuator SmartData);
 6. Configure MonitorService frequency;
 7. Collect collisions statistics;
 8. Plots or something to show in papers
```
 --------------

 --------------
  car1
 --------------
        car2
 --------------


 ------------------
 
 ------------------
           
 -------------car1-
        car2
 ------------------
 ```
