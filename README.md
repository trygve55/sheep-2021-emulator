# sheep-2021-emulator
Python scripts to emulate different parts of the sheepRTT system. Only tested on linux.

## Install
```bash
git clone https://github.com/trygve55/sheep-2021-emulator
git submodule update --init --recursive
python3 -m venv .
source bin/activate
pip install lxml future pyserial
cd mavlink
python -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=pymavlink/dialects/v20/sheeprtt_ardupilotmega.py message_definitions/v1.0/sheeprtt_ardupilotmega.xml
python -m pymavlink.tools.mavgen --lang=Python --wire-protocol=1.0 --output=pymavlink/dialects/v10/sheeprtt_ardupilotmega.py message_definitions/v1.0/sheeprtt_ardupilotmega.xml
cd pymavlink
python setup.py install
```

## Running
Connecting to drone via usb serial to emulate the sheepRTT nRF52833.
```bash
python main.py /dev/ttyUSB0 57600
```
Connecting gcs-addon to Mission Planner MAVLINK mirror. In Mission Planner press ctrl+f then click on "Mavlink", select "TCP HOST - 14550", enable write access and Connect.
```bash
python gcs-addon.py tcp:<ipadress>:14550
```
Start drone emulator listening in TCP port 14530.
```bash
python drone.py tcpin:localhost:14530
```
Connecting to drone emulator via TCP to emulate the sheepRTT nRF52833.
```bash
python main.py tcp:localhost:14530
```