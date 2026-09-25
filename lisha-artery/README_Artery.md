## Usefull Links for dependencies
- Install Omnetpp (use version 5.6) https://omnetpp.org/download/old ; Guide: https://doc.omnetpp.org/omnetpp/InstallGuide.pdf
- Install SUMO from source (version 1.11.0) https://sumo.dlr.de/releases/1.11.0/ ; Guide: https://sumo.dlr.de/docs/Installing/Linux_Build.html
- Install Artery: http://artery.v2x-research.eu/install/

![Artery V2X Simulation Framework](https://raw.githubusercontent.com/riebl/artery/master/logo.png)

Artery enables V2X simulations based on ETSI ITS-G5 protocols like GeoNetworking and BTP.
Single vehicles can be equipped with multiple ITS-G5 services through Artery's middleware, which also provides common Facilities for these services.

Artery started as an extension of the [Veins framework](http://veins.car2x.org) but can be used independently nowadays.
Please refer to its [documentation](http://veins.car2x.org/documentation) for details about Veins.

## Artery Documentation

The authors have started to extend the available documentation about Artery and created a website for this purpose.
Please visit [artery.v2x-research.eu](http://artery.v2x-research.eu).
The [install instructions](http://artery.v2x-research.eu/install/) previously found in this README have also been moved to this website.
We welcome your contribution to this documentation effort just as to Artery itself.
If you want to build the website yourself see also [our mkdocs guide](http://artery.v2x-research.eu/mkdocs).

## Installation Artery@LISHA

- Verify that all dependencies are installed:
    - Install SUMO v1_11_0: https://github.com/eclipse-sumo/sumo/releases/tag/v1_11_0
        - Install instructions: https://sumo.dlr.de/docs/Installing/Linux_Build.html
    - Install OmNET++ v5.6: https://github.com/omnetpp/omnetpp/releases/tag/omnetpp-5.6
        - Install instructions contained in omnetpp-5.6/doc/InstallGuide.pdf
    - Install Eigen 3.4: 
        - wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
        - unzip eigen-3.4.0.zip
        - sudo mv eigen-3.4.0/Eigen/ /usr/include
        - rm -rf eigen-3.4.0/
        - rm eigen-3.4.0.zip
- Clone this repository with git clone https://gitlab.lisha.ufsc.br/iot/artery.git
- Clone Artery in a temporary directory with git clone --recurse-submodule https://github.com/riebl/artery.git 
- Copy the extern folder from the temporary to the artery folder without overwriting
- Delete the temporary directory

## Configuring simulations at monitorAV

- OmNET++ simulations are configured through a omnetpp.ini file. Artery selects the correct configuration through the CMakeLists.txt contained inside the simulation directory.
- In the current CMakeLists you will find how the simulation is currently configured.
- add_opp_run(monitorAV CONFIG omnetpp.ini NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR}) is the command that generates the build target run_monitorAV with the omnetpp.ini configuration while setting the NED_FOLDERS variable to the current directory.
- The add_opp_run command takes as input the CONFIG, DEPENDENCY, WORKING_DIRECTORY and NED_FOLDERS.
- After we call SET(RUN_FLAGS "-uCmdenv"), all target configured with add_opp_run will run in the CMD environment. No GUI will be shown.
