# SDSPV2X – README (Simulation Execution, ASN.1 Generation and Packet Interception)

This project extends Artery + Vanetza with a secure communication framework for V2X
---

# 1. Running the SDSP Simulation

Before running the simulation, make sure the following components are installed:

* OMNeT++
* INET Framework
* Veins
* Artery
* Vanetza

First of all, the main simulation target of this project is `run_sdsp`.

Step-by-step exactly as used in the project environment:

```bash
cd ~/SDSPV2X
cd artery/
cd build/
ls
```

You should see files such as:

* Makefile
* run_sdsp
* scenarios
* src
* extern

Now compile and execute the SDSP simulation:

```bash
cmake --build . --target run_sdsp -j12
```

This command:

* Compiles Artery
* Compiles Vanetza
* Compiles the security modifications
* Executes the SDSP scenario configured in the project

The SDSP protocol files (e.g., PTP, ECDH, key exchange, and secure services) are located at:

```
artery/scenarios/sdsp/
```

---

# 2. CAM Message Modification (ASN.1)

The CAM message is NOT modified directly in the C++ source code.
It is defined by the official ASN.1 file used by Vanetza:

```
artery/extern/vanetza/asn1/EN302637-2v141-CAM.asn
```

This file defines:

* The CAM structure (ETSI ITS standard)
* ASN.1 fields
* PER encoding used in V2X communications

During compilation, Vanetza automatically generates:

* `.c` files
* `.h` files
* Encoding/decoding functions
* ASN.1 CAM structures used in the simulation

In other words:

> Any modification to the CAM structure must be done FIRST in the ASN.1 file.

---

# 3. How to Recompile ASN.1 After Modifying the CAM

Whenever you modify:

```
artery/extern/vanetza/asn1/EN302637-2v141-CAM.asn
```

you must regenerate the ASN.1 code before recompiling the simulation.

Otherwise, the simulation will continue using the old message structure.

## Official Vanetza Procedure

From the root of the Artery project:

```bash
mkdir build.asn1
cd build.asn1
cmake -DVANETZA_ASN1_WITH_ASN1C=ON -DVANETZA_ASN1_WITH_ISO=ON ..
make generate_asn1c
cmake --build .
```

This process:

* Processes the CAM.asn file
* Regenerates ASN.1 source files (.c and .h)
* Updates the code used internally by Vanetza

After that, recompile the simulation normally:

```bash
cd ~/SDSPV2X/artery/build
cmake --build . --target run_sdsp -j12
```


Never manually edit the `.c` and `.h` files generated from ASN.1.

---

# 4. Packet Identification (CAM)

The project implements V2X message interception based on the BTP port on which the packet arrives.

The message type is mainly identified by the destination port:

```
destination_port.host() == 2001
```

This port is defined in:

```
artery/scenarios/sdsp/services.xml
```

Example:

```xml
<service type="artery.application.CaService" name="CA">
    <listener port="2001" channel="180" aid="36"/>
</service>
```

---

# 5. Transmission Interception (TX) – Router.cc

Modified file:

```
artery/src/artery/networking/Router.cc
```

Main function:

```cpp
void Router::request(const vanetza::btp::DataRequestB& request,
                     std::unique_ptr<vanetza::DownPacket> packet)
```

### Implemented Features:

* Message type identification via BTP port
* Extraction of the Application layer payload
* Conversion to CAM object (ASN.1)
* Logging by stationID

Key snippet:

```cpp
if (request.destination_port.host() == 2001)
```

This means:

* The message type is identified using the BTP port
* Port 2001 → treated as a secure CAM message of the framework

Technical flow:

1. Intercepts BTP packet
2. Accesses Application layer
3. Converts to CaObject
4. Decodes ASN.1 CAM
5. Extracts `stationID` and `messageID`
6. Writes logs to file

Generated log files:

```
<stationID>cam_log.txt
```

---

# 6. Reception Interception (RX) – TransportDispatcher.cc

Modified file:

```
artery/src/artery/application/TransportDispatcher.cc
```

Main function:

```cpp
void TransportDispatcher::indicate(...)
```

### Implemented Functionalities

#### 6.1 Message Type Identification by Port

```cpp
if (btp_ind.destination_port.host() == 2001)
```

Here, the system identifies that:

* The received packet is a secure CAM message
* Based on BTP_B port (2001)

#### 6.2 Application Layer Payload Extraction

```cpp
vanetza::ByteBuffer payload =
    boost::apply_visitor(LayerVisitor(vanetza::OsiLayer::Application), *packet);
```

#### 6.3 ASN.1 CAM Decoding

```cpp
vanetza::asn1::Cam cam;
cam.decode(payload.data(), payload.size());
```

After decoding, the code accesses:

* stationID
* messageID
* CAM header

#### 6.4 Logging of Received Messages

Generated log files:

```
<stationID>_rx_log.txt
```

Containing:

* Simulation time (simTime)
* CAM MessageID
