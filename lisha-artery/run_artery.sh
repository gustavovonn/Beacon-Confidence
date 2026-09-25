#!/bin/bash
OPP_RUNALL=/home/manu/omnetpp-5.6-src-linux/omnetpp-5.6/bin/opp_runall
OPP_RUN=/home/manu/omnetpp-5.6-src-linux/omnetpp-5.6/bin/opp_run_release
NED_FOLDERS="/home/manu/SDSPV2X/artery/src/artery:/home/manu/SDSPV2X/artery/src/traci:/home/manu/SDSPV2X/artery/extern/veins/examples/veins:/home/manu/SDSPV2X/artery/extern/veins/src/veins:/home/manu/SDSPV2X/artery/extern/inet/src:/home/manu/SDSPV2X/artery/extern/inet/examples:/home/manu/SDSPV2X/artery/extern/inet/tutorials:/home/manu/SDSPV2X/artery/extern/inet/showcases"
LIBRARIES="-l/home/manu/SDSPV2X/artery/src/artery/libartery_core.so -l/home/manu/SDSPV2X/artery/src/traci/libtraci.so -l/home/manu/SDSPV2X/artery/extern/libveins.so -l/home/manu/SDSPV2X/artery/extern/libINET.so -l/home/manu/SDSPV2X/artery/src/artery/storyboard/libartery_storyboard.so -l/home/manu/SDSPV2X/artery/src/artery/envmod/libartery_envmod.so"

RUNALL=false
for arg do
    shift
    [[ "$arg" == -j* ]] && RUNALL=true && J=$arg && continue
    [[ "$arg" == -b* ]] && RUNALL=true && B=$arg && continue
    # run opp_runall with default values for -j* and -b* options by just specifying '--all'
    [[ "$arg" == "--all" ]] && RUNALL=true && continue
    set -- "$@" "$arg"
done

if [ "$RUNALL" = true ] ; then
    $OPP_RUNALL $J $B $OPP_RUN -n $NED_FOLDERS $LIBRARIES $@
else
    $OPP_RUN -n $NED_FOLDERS $LIBRARIES $@
fi
